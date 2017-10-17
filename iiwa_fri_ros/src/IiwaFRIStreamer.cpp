//
// Created by localadmin on 10/10/17.
//

#include "iiwa_fri_ros/IiwaFRIStreamer.h"

IiwaFRIStreamer::IiwaFRIStreamer(std::shared_ptr<IiwaState> state): iiwa_state_(state){

};

IiwaFRIStreamer::~IiwaFRIStreamer() {

}


void IiwaFRIStreamer::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);

    //TODO: Set as publisher to inform network of state change
    switch (newState){
        case KUKA::FRI::MONITORING_WAIT:
        {
            ROS_INFO_STREAM("Entering monitoring wait state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_.data()[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_.data()[6] << std::endl);
            break;
        }

        case KUKA::FRI::MONITORING_READY:
        {
            ROS_INFO_STREAM("Entering monitoring ready state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_.data()[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_.data()[6] << std::endl);
            break;
        }

        case KUKA::FRI::COMMANDING_WAIT:
        {
            ROS_INFO_STREAM("Entering commanding wait state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_.data()[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_.data()[6] << std::endl);
            break;
        }

        case KUKA::FRI::COMMANDING_ACTIVE:
        {
            ROS_INFO_STREAM("Entering commanding active state \nCurrent Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->current_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->current_position_.data()[6] << std::endl);
            ROS_INFO_STREAM("Commanded Position: ");
            for (int i = 0; i < 6; i++){
                ROS_INFO_STREAM(iiwa_state_->command_position_.data()[i] << ", ");
            }
            ROS_INFO_STREAM(iiwa_state_->command_position_.data()[6] << std::endl);
            break;
        }
        default: {
            break;
        }
    }
}

void IiwaFRIStreamer::command() {
    // Update current state
    update_state();
    KUKA::FRI::LBRClient::command();
}

void IiwaFRIStreamer::monitor() {
    update_state();
    //Copy current position as comanded postiion
    std::memcpy(iiwa_state_->command_position_.data(), robotState().getMeasuredJointPosition(), 7 * sizeof(double));

};


void IiwaFRIStreamer::waitForCommand() {
    update_state();
    KUKA::FRI::LBRClient::waitForCommand();
}

void IiwaFRIStreamer::update_state() {
    std::memcpy(iiwa_state_->current_position_.data(), robotState().getMeasuredJointPosition(), 7 * sizeof(double));
    std::memcpy(iiwa_state_->current_torque_.data(), robotState().getMeasuredTorque(), 7 * sizeof(double));

}