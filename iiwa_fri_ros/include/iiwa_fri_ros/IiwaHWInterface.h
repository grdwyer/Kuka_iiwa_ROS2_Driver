//
// Created by george on 21/09/17.
//

#ifndef IIWA_FRI_ROS_IIWAHWINTERFACE_H
#define IIWA_FRI_ROS_IIWAHWINTERFACE_H

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <control_toolbox/filters.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <std_msgs/Duration.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <array>
#include <mutex>
#include <sstream>
#include <memory>

class IiwaState{
public:
    IiwaState(){
        for(int i = 0; i < 7; i++) {
            current_position_[i] = 0.0;
            current_torque_[i] = 0.0;;
            command_position_[i] = 0.0;
            command_wrench_[i] = 0.0;
            command_torque_[i] = 0.0;
        }
    };
    void getCurrentState(std::array<double, 7> &pos, std::array<double, 7> &torque){
        std::lock_guard<std::mutex> guard(current_mutex_);
        pos = current_position_;
        torque = current_torque_;
    };

    void getCommandedState(std::array<double, 7> &pos, std::array<double, 7> &wrench, std::array<double, 7> &torque){
        std::lock_guard<std::mutex> guard(command_mutex_);
        pos = command_position_;
        wrench = command_wrench_;
        torque = command_torque_;
    };

    void setCurrentState(std::array<double, 7> &pos, std::array<double, 7> &torque){
        std::lock_guard<std::mutex> guard(current_mutex_);
        current_position_ = pos;
        current_torque_ = torque;
    };

    void setCommandedState(std::array<double, 7> &pos, std::array<double, 7> &wrench, std::array<double, 7> &torque){
        std::lock_guard<std::mutex> guard(command_mutex_);
        command_position_ = pos;
        command_wrench_ = wrench;
        command_torque_ = torque;
    };

// private:
    std::array<double, 7> current_position_, current_torque_;
    std::array<double, 7> command_position_, command_wrench_, command_torque_;
    std::mutex current_mutex_, command_mutex_;
};

class IiwaHWInterface : public hardware_interface::RobotHW{
public:
    /*
     * Constructor
     */
    IiwaHWInterface(const ros::NodeHandle &nh, std::shared_ptr<IiwaState> state);

    /*
     * Destructor
     */

    ~IiwaHWInterface();

    /*
     * Hardware Interface Implementations
     */

    /**
     * \brief Initialises fri, ros interfaces and state containers
     */
    bool start();

    /**
    * \brief Registers the limits of the joint specified by joint_name and joint_handle.
    *
    * The limits are retrieved from the urdf_model.
    * Returns the joint's type, lower position limit, upper position limit, and effort limit.
    */
    void registerJointLimits(const std::string& joint_name,
                             const hardware_interface::JointHandle& joint_handle,
                             const urdf::Model *const urdf_model,
                             double *const lower_limit, double *const upper_limit,
                             double *const effort_limit);

    /**
     * \brief Retrieves the current state from the shared memory (iiwa state) and passes it to the joint state publisher
     */
    void read(ros::Duration duration);


    /**
     * \brief Retrieves the current commanded state from the interfaces and passes it to the shared memory
     */
    void write(ros::Duration duration);

private:
    ros::NodeHandle nh_;
    ros::Rate* rate_;

    std::shared_ptr<IiwaState> fri_state_handle_;

    std::array<double, 7> current_position_, current_velocity_, current_torque_;
    std::array<double, 7> command_position_, command_velocity_, command_torque_;
    std::vector<std::string> joint_names_;

    std::vector<std::string> interface_type_;
    std::string selected_interface_;

    // Interfaces
    // TODO: add more interfcaes (velocity and effort)
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::PositionJointInterface position_interface_;

    urdf::Model urdf_model_;
    // Interfaces for limits
    // TODO: need to look into this more to check how it is implemented
    joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
    joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;


};


#endif //IIWA_FRI_ROS_IIWAHWINTERFACE_H
