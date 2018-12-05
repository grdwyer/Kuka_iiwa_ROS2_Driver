//
// Created by george on 21/09/17.
//

#include "iiwa_fri_ros/IiwaHWInterface.h"

IiwaHWInterface::IiwaHWInterface(const ros::NodeHandle &nh, std::shared_ptr<IiwaState> state):
        nh_(nh), fri_state_handle_(state){

    int rate;
    nh_.param<int>("rate", rate, 500);
    rate_ = new ros::Rate(rate);

}

IiwaHWInterface::~IiwaHWInterface() {
    delete rate_;
}

bool IiwaHWInterface::start() {
    if ( ros::param::get("joints", joint_names_) ) {
        if ( joint_names_.size() != 7 ) {
            for(auto const& value: joint_names_){
                ROS_INFO_STREAM(value << ", ");
            };
            ROS_ERROR("iiwa's have 7 joints which is not given");
        }
    } else {
        ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
        throw std::runtime_error("No joint name specification");
    }

    if (!(urdf_model_.initParam("robot_description"))) {
        ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
        throw std::runtime_error("No URDF model available");
    }

    urdf::JointConstSharedPtr current_joint;
    // For each joint
    for(int i = 0; i < 7; i++){
        ROS_INFO_STREAM("Handling joint: " << joint_names_[i]);

        // get current joint configuration
        current_joint = urdf_model_.getJoint(joint_names_[i]);
        if(!current_joint.get()) {
            ROS_ERROR_STREAM("The specified joint "<< joint_names_[i] << " can't be found in the URDF model. "
                    "Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
            throw std::runtime_error("Wrong joint name specification");
        }

        // joint state handle

        hardware_interface::JointStateHandle state_handle(joint_names_[i],
                                                          &(current_position_[i]),
                                                          &(current_velocity_[i]),
                                                          &(current_torque_[i]));

        state_interface_.registerHandle(state_handle);

        // position command handle
        hardware_interface::JointHandle position_joint_handle = hardware_interface::JointHandle(
                state_interface_.getHandle(joint_names_[i]), &command_position_[i]);

        position_interface_.registerHandle(position_joint_handle);

        registerJointLimits(joint_names_[i], position_joint_handle, &urdf_model_);

    }
    /**
     * Add new interfaces here
     */

    // Register each of the interfaces
    this->registerInterface(&state_interface_);
    this->registerInterface(&position_interface_);

    return true;
}


void IiwaHWInterface::registerJointLimits(const std::string &joint_name,
                                          const hardware_interface::JointHandle &joint_handle,
                                          const urdf::Model *urdf_model) {
    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != nullptr) {
        const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);

        if (urdf_joint != nullptr) {
            // Get limits from the URDF file.
            if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                has_limits = true;

            if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                has_soft_limits = true;
        }
    }

    if (!has_limits)
        return;

    if (limits.has_position_limits) {
        ROS_DEBUG_STREAM("Joint has position limits");
    }

    if (limits.has_velocity_limits)
        ROS_DEBUG_STREAM("Joint has velocity limits");

    if (limits.has_effort_limits)
        ROS_DEBUG_STREAM("Joint has effort limits");

    if (has_soft_limits) {
        const joint_limits_interface::PositionJointSoftLimitsHandle position_soft_limits_handle(joint_handle, limits, soft_limits);
        position_joint_limits_interface_.registerHandle(position_soft_limits_handle);
    }
    else {
        ROS_WARN_STREAM("Hardcoded to use saturation limits");
        const joint_limits_interface::PositionJointSaturationHandle position_saturation_handle(joint_handle, limits);
        position_joint_saturation_interface_.registerHandle(position_saturation_handle);
    }
    ROS_INFO_STREAM("Registering joint limits\n\tLower: " << limits.min_position << "\n\tUpper: " << limits.max_position <<
    "\n\tEffort: " << limits.max_effort << std::endl);
}

void IiwaHWInterface::read(ros::Duration duration) {
    // copy the over the previous position
    previous_position_ = current_position_;
    // Get the current state from the state handle
    fri_state_handle_->getCurrentState(current_position_, current_torque_);
    for (int i = 0; i < 7; i++){
        current_velocity_[i] = (current_position_[i] - previous_position_[i])/(double)duration.nsec/(double)10e-9;
    }
    ROS_DEBUG_STREAM_THROTTLE(1, "Duration: " << duration.sec << "\nJoint 1\nPosition: " << current_position_[0] << "\nVelocity: " << current_velocity_[0]);

}

void IiwaHWInterface::write(ros::Duration duration) {
    ROS_DEBUG_STREAM_THROTTLE(1, "Command position (pre-enforcement): "
            << "\nLast commanded position: "
            << angles::to_degrees(command_position_[0]) << ", "
            << angles::to_degrees(command_position_[1]) << ", "
            << angles::to_degrees(command_position_[2]) << ", "
            << angles::to_degrees(command_position_[3]) << ", "
            << angles::to_degrees(command_position_[4]) << ", "
            << angles::to_degrees(command_position_[5]) << ", "
            << angles::to_degrees(command_position_[6]) << std::endl);
    //position_joint_limits_interface_.enforceLimits(duration);
    position_joint_saturation_interface_.enforceLimits(duration);
    // Get latest commands from the interface and pass to the state handle
    fri_state_handle_->setCommandedPosition(command_position_);
}
