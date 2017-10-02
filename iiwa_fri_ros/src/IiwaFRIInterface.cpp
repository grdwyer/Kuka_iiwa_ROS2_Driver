//
// Created by george on 21/09/17.
//

#include "iiwa_fri_ros/IiwaFRIInterface.h"

IiwaFRIInterface::IiwaFRIInterface(std::shared_ptr<IiwaState> state): iiwa_state_(state){

};

IiwaFRIInterface::~IiwaFRIInterface() {

}


void IiwaFRIInterface::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);

    //TODO: Set as publisher to inform network of state change
    switch (newState){
        case KUKA::FRI::MONITORING_WAIT:
        {

            break;
        }

        case KUKA::FRI::MONITORING_READY:
        {

            break;
        }

        case KUKA::FRI::COMMANDING_WAIT:
        {

            break;
        }

        case KUKA::FRI::COMMANDING_ACTIVE:
        {

            break;
        }
        default: {
            break;
        }
    }
}

void IiwaFRIInterface::command() {
    // Update current state
    monitor();

    auto mode = robotState().getClientCommandMode();

    if(mode == KUKA::FRI::EClientCommandMode::POSITION){
        // Take current commanded values
        //TODO: add in check to compare it to current position and ensure the difference is not too large
        robotCommand().setJointPosition(iiwa_state_->command_position_.data());
    }
    else if(mode == KUKA::FRI::EClientCommandMode::TORQUE){
        // Take current commanded values
        KUKA::FRI::LBRClient::command();
        robotCommand().setTorque(iiwa_state_->command_torque_.data());
    }
    else if(mode == KUKA::FRI::EClientCommandMode::WRENCH){
        // Take current commanded values
        KUKA::FRI::LBRClient::command();
        robotCommand().setJointPosition(iiwa_state_->command_wrench_.data());
    }
}

void IiwaFRIInterface::monitor(){

    std::memcpy(iiwa_state_->current_position_.data(), robotState().getMeasuredJointPosition(), 7);
    std::memcpy(iiwa_state_->current_torque_.data(), robotState().getMeasuredTorque(), 7);

}

void IiwaFRIInterface::waitForCommand() {
    monitor();
    KUKA::FRI::LBRClient::waitForCommand();
}