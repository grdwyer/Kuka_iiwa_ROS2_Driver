//
// Created by george on 2/18/21.
//

#ifndef IIWA_FRI_ROS_IIWA_FRI_DRIVER_H
#define IIWA_FRI_ROS_IIWA_FRI_DRIVER_H

#include <iiwa_fri_ros/IiwaFRIInterface.h>
#include <fri_client_sdk/friUdpConnection.h>
#include <fri_client_sdk/friClientApplication.h>
#include <thread>

class IiwaFriDriver{
public:

    IiwaFriDriver(const std::string& robot_ip, int robot_port);

    bool initialise_connection();

    void read_joint_position(const std::array<double, 7> &joint_positions);
    void read_joint_velocity(const std::array<double, 7> &joint_velocity);
    void read_joint_torque(const std::array<double, 7> &joint_torque);
    void read_external_joint_torque(const std::array<double, 7> &joint_torque);

    void write_joint_position(const std::array<double, 7> &joint_positions);
    void write_joint_torque(const std::array<double, 7> &joint_torque);
    void write_end_effector_wrench(const std::array<double, 6> &wrench);

private:
    void run();

    KUKA::FRI::UdpConnection connection_;
    std::unique_ptr<KUKA::FRI::ClientApplication> app_;
    std::unique_ptr<IiwaFRIInterface> client_;
    std::shared_ptr<IiwaState> state_;

    std::string robot_hostname_;
    int robot_port_;

    std::thread run_thread_;


};


#endif //IIWA_FRI_ROS_IIWA_FRI_DRIVER_H
