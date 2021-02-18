//
// Created by george on 2/18/21.
//

#ifndef IIWA_FRI_ROS_IIWA_STATE_H
#define IIWA_FRI_ROS_IIWA_STATE_H

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

#endif //IIWA_FRI_ROS_IIWA_STATE_H
