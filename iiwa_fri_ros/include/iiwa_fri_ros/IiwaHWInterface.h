//
// Created by george on 21/09/17.
//

#ifndef IIWA_FRI_ROS_IIWAHWINTERFACE_H
#define IIWA_FRI_ROS_IIWAHWINTERFACE_H


#include <builtin_interfaces/msg/duration.h>
#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.h>

// System
#include <memory>
#include <vector>

// ros2_control hardware_interface
#include "hardware_interface/actuator.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "iiwa_fri_ros/iiwa_state.h"
#include <iiwa_fri_ros/iiwa_fri_driver.h>
// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/clock.hpp>


class IiwaHWInterface : public hardware_interface::SystemInterface{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IiwaHWInterface)

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo& system_info) final;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    hardware_interface::status get_status() const final
    {
        return status_;
    }

    std::string get_name() const final
    {
        return info_.name;
    }

    hardware_interface::return_type start() final;
    hardware_interface::return_type stop() final;
    hardware_interface::return_type read() final;
    hardware_interface::return_type write() final;

private:
    hardware_interface::HardwareInfo info_;
    hardware_interface::status status_;
    std::shared_ptr<IiwaState> fri_state_handle_;
    std::unique_ptr<IiwaFriDriver> iiwa_driver_;

    rclcpp::Clock clock_;

    uint32_t runtime_state_;
    bool controllers_initialized_;
    bool position_interface_in_use_;

    std::array<double, 7> current_position_, previous_position_, current_velocity_, current_torque_, current_ext_torque_;
    std::array<double, 7> command_position_;
    std::vector<std::string> joint_names_;

};

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(IiwaHWInterface, hardware_interface::SystemInterface)

#endif //IIWA_FRI_ROS_IIWAHWINTERFACE_H
