//
// Created by george on 21/09/17.
//

#ifndef IIWA_FRI_ROS_IIWAHWINTERFACE_H
#define IIWA_FRI_ROS_IIWAHWINTERFACE_H

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/robot_hardware_interface.hpp>
#include <hardware_interface/joint_handle.hpp>

#include <builtin_interfaces/msg/duration.h>
#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.h>
#include <angles/angles.h>

#include <vector>
#include <array>
#include <mutex>
#include <sstream>
#include <memory>

class IiwaState{
public:
    IiwaState(){
            current_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            current_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            current_ext_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            command_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            command_wrench_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            command_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

    void setCommandedPosition(std::array<double, 7> &pos){
        std::lock_guard<std::mutex> guard(command_mutex_);
        command_position_ = pos;
    }

// private:
    std::array<double, 7> current_position_, current_torque_, current_ext_torque_;
    std::array<double, 7> command_position_, command_wrench_, command_torque_;
    std::mutex current_mutex_, command_mutex_;
};

class IiwaHWInterface : public hardware_interface::RobotHardwareInterface{
public:
    /*
     * Constructor
     */
    IiwaHWInterface(std::shared_ptr<IiwaState> state);

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
                             const urdf::Model *const urdf_model);

    /**
     * \brief Retrieves the current state from the shared memory (iiwa state) and passes it to the joint state publisher
     * Additionally sends the external torque as a normal joint_state message but to a different ros_topic
     */
    hardware_interface::return_type read();


    /**
     * \brief Retrieves the current commanded state from the interfaces and passes it to the shared memory
     */
    hardware_interface::return_type write();

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<sensor_msgs::msg::JointState> external_torque_publisher_;
    sensor_msgs::msg::JointState external_torque_state_;

    std::shared_ptr<IiwaState> fri_state_handle_;

    std::array<double, 7> current_position_, previous_position_, current_velocity_, current_torque_;
    std::array<double, 7> command_position_;
    std::vector<std::string> joint_names_;

//    std::vector<std::string> interface_type_;
//    std::string selected_interface_;

    // Interfaces
    // TODO: add more interfcaes (velocity and effort)
    hardware_interface::StateInterface state_interface_;
    hardware_interface::CommandInterface position_interface_;

    urdf::Model urdf_model_;
    // Interfaces for limits
//    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
//    joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;

};


#endif //IIWA_FRI_ROS_IIWAHWINTERFACE_H
