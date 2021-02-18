//
// Created by george on 2/18/21.
//

#include <iiwa_fri_ros/iiwa_fri_driver.h>

IiwaFriDriver::IiwaFriDriver(const std::string &robot_ip, int robot_port) {
    state_ = std::make_shared<IiwaState>();
    client_ = std::make_unique<IiwaFRIInterface>(state_);

    app_ = std::make_unique<KUKA::FRI::ClientApplication>(connection_, *client_);

    robot_hostname_ = robot_ip;
    robot_port_ = robot_port;
}

bool IiwaFriDriver::initialise_connection() {
    if(app_->connect(robot_port_, robot_hostname_.c_str())){
        // start run thread
        run_thread_ = std::thread(&IiwaFriDriver::run, this);

//        //For setting the thread policy taken from
//        int priority, policy
//        sch_params.sched_priority = priority;
//        if(pthread_setschedparam(th.native_handle(), policy, &sch_params)) {
//            std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        return true;
    }


}

void IiwaFriDriver::read_joint_position(const std::array<double, 7> &joint_positions) {
    std::copy(state_->current_position_.begin(), state_->current_position_.end(), std::inserter(joint_positions, joint_positions.begin()));
}

void IiwaFriDriver::read_joint_velocity(const std::array<double, 7> &joint_velocity) {
    std::copy(state_->current_position_.begin(), state_->current_position_.end(), std::inserter(joint_velocity, joint_velocity.begin()));
}

void IiwaFriDriver::read_joint_torque(const std::array<double, 7> &joint_torque) {
    std::copy(state_->current_torque_.begin(), state_->current_torque_.end(), std::inserter(joint_torque, joint_torque.begin()));
}

void IiwaFriDriver::read_external_joint_torque(const std::array<double, 7> &joint_torque) {
    std::copy(state_->current_ext_torque_.begin(), state_->current_ext_torque_.end(), std::inserter(joint_torque, joint_torque.begin()));
}

void IiwaFriDriver::write_joint_position(const std::array<double, 7> &joint_positions) {
    std::copy(joint_positions.begin(), joint_positions.end(), std::inserter(state_->command_position_, state_->command_position_.begin()));
}

void IiwaFriDriver::write_joint_torque(const std::array<double, 7> &joint_torque) {
    std::copy(joint_torque.begin(), joint_torque.end(), std::inserter(state_->command_torque_, state_->command_torque_.begin()));
}

void IiwaFriDriver::write_end_effector_wrench(const std::array<double, 6> &wrench) {
    std::copy(wrench.begin(), wrench.end(), std::inserter(state_->command_wrench_, state_->command_wrench_.begin()));
}

void IiwaFriDriver::run() {
    bool success = true;
    while (success)
    {
        success = app_->step();
    }
    app_->disconnect();

}
