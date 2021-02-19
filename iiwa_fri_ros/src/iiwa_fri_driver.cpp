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
    } else{
        return false;
    }

}

void IiwaFriDriver::read_joint_position(std::array<double, 7> &joint_positions) {
    joint_positions = state_->current_position_;
}

void IiwaFriDriver::read_joint_velocity(std::array<double, 7> &joint_velocity) {
    joint_velocity = state_->current_velocity_;
}

void IiwaFriDriver::read_joint_torque(std::array<double, 7> &joint_torque) {
    joint_torque = state_->current_torque_;
}

void IiwaFriDriver::read_external_joint_torque(std::array<double, 7> &joint_torque) {
    joint_torque = state_->current_ext_torque_;
}

void IiwaFriDriver::write_joint_position(std::array<double, 7> &joint_positions) {
   state_->command_position_ = joint_positions;
}

void IiwaFriDriver::write_joint_torque(std::array<double, 7> &joint_torque) {
    state_->command_torque_ = joint_torque;
}

void IiwaFriDriver::write_end_effector_wrench(std::array<double, 6> &wrench) {
    state_->command_wrench_ = wrench;
}

void IiwaFriDriver::run() {
    rclcpp::Rate rate(500.0); // 500 hz maybe param this
    bool success = true;
    while (success)
    {
        success = app_->step();
        rate.sleep();
    }
    app_->disconnect();

}



bool FakeIiwaFriDriver::initialise_connection() {
    active_ = true;
    run_thread_ = std::thread(&FakeIiwaFriDriver::run, this);

//        //For setting the thread policy taken from

    return true;
}

void FakeIiwaFriDriver::run() {
    rclcpp::Rate rate(500.0); // 500 hz maybe param this
    while (active_)
    {
        state_->current_position_ = state_->command_position_;
        state_->current_torque_ = state_->command_torque_;

        rate.sleep();
    }
}
