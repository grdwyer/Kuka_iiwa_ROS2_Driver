//
// Created by george on 21/09/17.
//

#include "iiwa_fri_ros/IiwaHWInterface.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>


hardware_interface::return_type IiwaHWInterface::configure(const hardware_interface::HardwareInfo& system_info)
{
    info_ = system_info;

    // initialize
    current_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    previous_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    current_ext_torque_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    command_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    controllers_initialized_ = false;

    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
                         "Joint '%s' has %d command interfaces found. 2 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
                         "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
                         "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"), "Joint '%s' has %d state interface. 3 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
                         "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
                         "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
                         "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }
    }

    status_ = hardware_interface::status::CONFIGURED;

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> IiwaHWInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &current_position_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &current_velocity_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &current_torque_[i]));
    }


    for (auto& sensor : info_.sensors)
    {
        for (uint j = 0; j < sensor.state_interfaces.size(); ++j)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
                                                                             &current_ext_torque_[j]));
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IiwaHWInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_position_[i]));

        // TODO: replace with an effort interface and possibly wrench in the future
//        command_interfaces.emplace_back(hardware_interface::CommandInterface(
//                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_velocity_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type IiwaHWInterface::start()
{
    RCLCPP_INFO(rclcpp::get_logger("IiwaHWInterface"), "Starting ...please wait...");

    position_interface_in_use_ = false;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // The robot's IP address.
    std::string robot_ip = info_.hardware_parameters["robot_ip"];
    int robot_port = std::stoi(info_.hardware_parameters["robot_port"]);

    RCLCPP_INFO(rclcpp::get_logger("IiwaHWInterface"), "Initializing driver...");

    status_ = hardware_interface::status::STARTED;

    RCLCPP_INFO(rclcpp::get_logger("IiwaHWInterface"), "System successfully started!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IiwaHWInterface::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("IiwaHWInterface"), "Stopping ...please wait...");

    position_interface_in_use_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Nedd to implement an actual stop for the iiwaFRIInterface
    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(rclcpp::get_logger("IiwaHWInterface"), "System successfully stopped!");

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type IiwaHWInterface::read()
{
    previous_position_ = current_position_;
    // Get the current state from the state handle
    fri_state_handle_->getCurrentState(current_position_, current_torque_);
    for (int i = 0; i < 7; i++){
//        current_velocity_[i] = (current_position_[i] - previous_position_[i])/(double)duration.nsec/(double)10e-9;
        current_velocity_[i] = (current_position_[i] - previous_position_[i])/(double)1.0/500.0;
    }

    return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type IiwaHWInterface::write()
{
    fri_state_handle_->setCommandedPosition(command_position_);
    return hardware_interface::return_type::OK;
}