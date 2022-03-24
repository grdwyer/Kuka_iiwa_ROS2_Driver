//
// Created by george on 21/09/17.
//

#include "iiwa_fri_ros/iiwa_fri_interface.h"

#include <utility>

IiwaFRIInterface::IiwaFRIInterface(std::shared_ptr<IiwaState> state): iiwa_state_(std::move(state)){
    startup_ = 0;
    init_ = false;
}

IiwaFRIInterface::~IiwaFRIInterface() {

}


void IiwaFRIInterface::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);


    //TODO: Set as publisher to inform network of state change
    switch (newState){
        case KUKA::FRI::MONITORING_WAIT:
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"),"Entering monitoring wait state\nLast commanded position: "
                                << angles::to_degrees(iiwa_state_->command_position_[0]) << ", "
                                << angles::to_degrees(iiwa_state_->command_position_[1]) << ", "
                                << angles::to_degrees(iiwa_state_->command_position_[2]) << ", "
                                << angles::to_degrees(iiwa_state_->command_position_[3]) << ", "
                                << angles::to_degrees(iiwa_state_->command_position_[4]) << ", "
                                << angles::to_degrees(iiwa_state_->command_position_[5]) << ", "
                                << angles::to_degrees(iiwa_state_->command_position_[6]) <<
                                "\nLast known position: "
                                << angles::to_degrees(iiwa_state_->current_position_[0]) << ", "
                                << angles::to_degrees(iiwa_state_->current_position_[1]) << ", "
                                << angles::to_degrees(iiwa_state_->current_position_[2]) << ", "
                                << angles::to_degrees(iiwa_state_->current_position_[3]) << ", "
                                << angles::to_degrees(iiwa_state_->current_position_[4]) << ", "
                                << angles::to_degrees(iiwa_state_->current_position_[5]) << ", "
                                << angles::to_degrees(iiwa_state_->current_position_[6]) << std::endl);
            std::copy(iiwa_state_->current_position_.begin(), iiwa_state_->current_position_.end(), iiwa_state_->command_position_.begin());
            break;
        }

        case KUKA::FRI::MONITORING_READY:
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"),"Entering monitoring ready state\nLast commanded position: "
                                    << angles::to_degrees(iiwa_state_->command_position_[0]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[1]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[2]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[3]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[4]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[5]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[6]) <<
                                    "\nLast known position: "
                                    << angles::to_degrees(iiwa_state_->current_position_[0]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[1]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[2]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[3]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[4]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[5]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[6]) << std::endl);
            std::copy(iiwa_state_->current_position_.begin(), iiwa_state_->current_position_.end(), iiwa_state_->command_position_.begin());
            break;
        }

        case KUKA::FRI::COMMANDING_WAIT:
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"),"Entering commanding wait state\nLast commanded position: "
                                    << angles::to_degrees(iiwa_state_->command_position_[0]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[1]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[2]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[3]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[4]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[5]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[6]) <<
                                    "\nLast known position: "
                                    << angles::to_degrees(iiwa_state_->current_position_[0]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[1]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[2]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[3]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[4]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[5]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[6]) << std::endl);
            std::copy(iiwa_state_->current_position_.begin(), iiwa_state_->current_position_.end(), iiwa_state_->command_position_.begin());
            break;
        }

        case KUKA::FRI::COMMANDING_ACTIVE:
        {
            std::copy(iiwa_state_->current_position_.begin(), iiwa_state_->current_position_.end(), iiwa_state_->command_position_.begin());
            RCLCPP_INFO_STREAM(rclcpp::get_logger("IiwaHWInterface"),"Entering commanding active state\nLast commanded position: "
                                    << angles::to_degrees(iiwa_state_->command_position_[0]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[1]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[2]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[3]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[4]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[5]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[6]) <<
                                    "\nLast known position: "
                                    << angles::to_degrees(iiwa_state_->current_position_[0]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[1]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[2]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[3]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[4]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[5]) << ", "
                                    << angles::to_degrees(iiwa_state_->current_position_[6]) << std::endl);
            break;
        }
        default: {
            break;
        }
    }
}

void IiwaFRIInterface::command() {
    // Update current state
    update_state();

    if(init_) {
        auto mode = robotState().getClientCommandMode();

        if (mode == KUKA::FRI::EClientCommandMode::POSITION) {
            // Take current commanded values
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("IiwaHWInterface"),"Commanded Position: "
                                    << angles::to_degrees(iiwa_state_->command_position_[0]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[1]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[2]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[3]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[4]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[5]) << ", "
                                    << angles::to_degrees(iiwa_state_->command_position_[6]) << std::endl);

            robotCommand().setJointPosition(iiwa_state_->command_position_.data());
//            KUKA::FRI::LBRClient::waitForCommand();
        } else if (mode == KUKA::FRI::EClientCommandMode::TORQUE) {
            // Take current commanded values
            KUKA::FRI::LBRClient::command();
            robotCommand().setTorque(iiwa_state_->command_torque_.data());
        } else if (mode == KUKA::FRI::EClientCommandMode::WRENCH) {
            // Take current commanded values
            KUKA::FRI::LBRClient::command();
            robotCommand().setWrench(iiwa_state_->command_wrench_.data());
        }
    }
    else{
        RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("IiwaHWInterface"), "Values not inited  \n"); //TODO: Change this to throttle

        waitForCommand();

        if(startup_ > 100){
            init_ = true;
        }

        startup_++;
    }
}

void IiwaFRIInterface::monitor() {
    update_state();
    //Copy current position as comanded postiion
//    auto current_pos = robotState().getMeasuredJointPosition();
//
//    for (int i = 0; i < 7; i++) {
//        iiwa_state_->command_position_[i] = current_pos[i];
//    }
    std::copy(iiwa_state_->current_position_.begin(), iiwa_state_->current_position_.end(), iiwa_state_->command_position_.begin());
    KUKA::FRI::LBRClient::monitor();

}

void IiwaFRIInterface::update_state() {
    auto current_pos = robotState().getMeasuredJointPosition();
    auto current_torque = robotState().getMeasuredTorque();
    auto current_ext_torque = robotState().getExternalTorque();

    for (int i = 0; i < 7; i++) {
        iiwa_state_->current_position_[i] = current_pos[i];
//        ROS_DEBUG_STREAM_THROTTLE(1, "Current Position: "
//                                << angles::to_degrees(current_pos[0]) << ", "
//                                << angles::to_degrees(current_pos[1]) << ", "
//                                << angles::to_degrees(current_pos[2]) << ", "
//                                << angles::to_degrees(current_pos[3]) << ", "
//                                << angles::to_degrees(current_pos[4]) << ", "
//                                << angles::to_degrees(current_pos[5]) << ", "
//                                << angles::to_degrees(current_pos[6]) << std::endl);
        iiwa_state_->current_torque_[i] = current_torque[i];
        iiwa_state_->current_ext_torque_[i] = current_ext_torque[i];
    }
}

void IiwaFRIInterface::waitForCommand() {
    update_state();
//    auto current_pos = robotState().getMeasuredJointPosition();

    std::copy(iiwa_state_->current_position_.begin(), iiwa_state_->current_position_.end(), iiwa_state_->command_position_.begin());
//    for (int i = 0; i < 7; i++) {
//        iiwa_state_->command_position_[i] = current_pos[i];
//    }

    KUKA::FRI::LBRClient::waitForCommand();
}