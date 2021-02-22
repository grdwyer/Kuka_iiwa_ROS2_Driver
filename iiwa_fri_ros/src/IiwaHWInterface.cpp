//
// Created by george on 21/09/17.
//

#include "iiwa_fri_ros/IiwaHWInterface.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>


hardware_interface::return_type IiwaHWInterface::configure(const hardware_interface::HardwareInfo& system_info)
{
    clock_ = rclcpp::Clock(RCL_STEADY_TIME);
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
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
                         "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
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

//        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
//        {
//            RCLCPP_FATAL(rclcpp::get_logger("IiwaHWInterface"),
//                         "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
//                         joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
//            return hardware_interface::return_type::ERROR;
//        }

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
    RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"), "Setting up state interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"), "Setting joint: " << info_.joints[i].name << " at index " << i);
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
    RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"), "Setting up command interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"), "Setting joint: " << info_.joints[i].name << " at index " << i);
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

    RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"), "Initializing driver using: \n\tIP: " << robot_ip << "\n\tPort: " << robot_port);
    iiwa_driver_ = std::make_unique<FakeIiwaFriDriver>(robot_ip, robot_port);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"), "instantiated driver");
    status_ = iiwa_driver_->initialise_connection()? hardware_interface::status::STARTED : hardware_interface::status::UNKNOWN;

    if (status_ == hardware_interface::status::STARTED) {
        RCLCPP_INFO(rclcpp::get_logger("IiwaHWInterface"), "System successfully started!");
        return hardware_interface::return_type::OK;
    } else{
        RCLCPP_ERROR(rclcpp::get_logger("IiwaHWInterface"), "System not initialised, unable to create a connection!");
        return hardware_interface::return_type::ERROR;
    }
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
    RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger("IiwaHWInterface"), clock_, 1e3, "Reading from driver");
    iiwa_driver_->read_joint_position(current_position_);
    iiwa_driver_->read_joint_velocity(current_velocity_);
    iiwa_driver_->read_joint_torque(current_torque_);
    iiwa_driver_->read_external_joint_torque(current_ext_torque_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IiwaHWInterface::write()
{
    RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger("IiwaHWInterface"), clock_, 1e3, "Writing to driver");
    iiwa_driver_->write_joint_position(command_position_);
//    iiwa_driver_->write_joint_torque(command_torque_);
//    iiwa_driver_->write_joint_torque(command_wrench_);
    return hardware_interface::return_type::OK;
}