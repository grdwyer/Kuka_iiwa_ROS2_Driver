//
// Created by george on 21/09/17.
//

#include "iiwa_fri_ros/IiwaFRIInterface.h"

IiwaFRIInterface::IiwaFRIInterface(std::shared_ptr<IiwaState> state): iiwa_state_(state){
    startup_ = 0;
    init_ = false;
};

IiwaFRIInterface::~IiwaFRIInterface() {

}


void IiwaFRIInterface::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);


    //TODO: Set as publisher to inform network of state change
    switch (newState){
        case KUKA::FRI::MONITORING_WAIT:
        {
            update_state();
            auto current_pos = robotState().getMeasuredJointPosition();

            for (int i = 0; i < 7; i++) {
                iiwa_state_->command_position_[i] = current_pos[i];
            }
            ROS_INFO_STREAM("Entering monitoring wait state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_[6] << std::endl);
            break;
        }

        case KUKA::FRI::MONITORING_READY:
        {
            update_state();
            auto current_pos = robotState().getMeasuredJointPosition();

            for (int i = 0; i < 7; i++) {
                iiwa_state_->command_position_[i] = current_pos[i];
            }
            ROS_INFO_STREAM("Entering monitoring ready state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_.data()[6] << std::endl);
            break;
        }

        case KUKA::FRI::COMMANDING_WAIT:
        {
            update_state();
            auto current_pos = robotState().getMeasuredJointPosition();

            for (int i = 0; i < 7; i++) {
                iiwa_state_->command_position_[i] = current_pos[i];
            }
            ROS_INFO_STREAM("Entering commanding wait state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_[6] << std::endl);
            break;
        }

        case KUKA::FRI::COMMANDING_ACTIVE:
        {
            update_state();
            auto current_pos = robotState().getMeasuredJointPosition();

            for (int i = 0; i < 7; i++) {
                iiwa_state_->command_position_[i] = current_pos[i];
            }

            ROS_INFO_STREAM("Entering commanding active state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_[6] << std::endl);

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
//            ROS_DEBUG_STREAM("Commanded Position: ");
//            for (int i = 0; i < 6; i++){
//                ROS_DEBUG_STREAM(iiwa_state_->command_position_[i] << ", ");
//            }
//            ROS_DEBUG_STREAM(iiwa_state_->command_position_[6] << std::endl);
            robotCommand().setJointPosition(iiwa_state_->command_position_.data());
        } else if (mode == KUKA::FRI::EClientCommandMode::TORQUE) {
            // Take current commanded values
            KUKA::FRI::LBRClient::command();
            robotCommand().setTorque(iiwa_state_->command_torque_.data());
        } else if (mode == KUKA::FRI::EClientCommandMode::WRENCH) {
            // Take current commanded values
            KUKA::FRI::LBRClient::command();
            robotCommand().setJointPosition(iiwa_state_->command_wrench_.data());
        }
    }
    else{
        ROS_DEBUG_STREAM("Values not inited  \n");

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
    auto current_pos = robotState().getMeasuredJointPosition();

    for (int i = 0; i < 7; i++) {
        iiwa_state_->command_position_[i] = current_pos[i];
    }
};

void IiwaFRIInterface::update_state() {
    auto current_pos = robotState().getMeasuredJointPosition();
    auto current_torque = robotState().getMeasuredTorque();

    for (int i = 0; i < 7; i++) {
        iiwa_state_->current_position_[i] = current_pos[i];
        ROS_DEBUG_STREAM("CurrentPos J" << i  << ": " << current_pos[i] << "\n");
        iiwa_state_->current_torque_[i] = current_torque[i];
    }
}

void IiwaFRIInterface::waitForCommand() {
    update_state();
    auto current_pos = robotState().getMeasuredJointPosition();

    for (int i = 0; i < 7; i++) {
        iiwa_state_->command_position_[i] = current_pos[i];
    }

    KUKA::FRI::LBRClient::monitor();
}