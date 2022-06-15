//
// Created by george on 2/18/21.
//

#ifndef IIWA_FRI_ROS_IIWA_FRI_DRIVER_H
#define IIWA_FRI_ROS_IIWA_FRI_DRIVER_H

#include <iiwa_fri_ros/iiwa_fri_interface.h>
#include <fri_client_sdk/friUdpConnection.h>
#include <fri_client_sdk/friClientApplication.h>
#include <thread>
#include <iterator>
#include <rclcpp/rate.hpp>

class IiwaFriDriver{
public:

    /**
     * @brief creates the shared state, fri interface and fri application but does not start a connection with the robot controller
     * @param robot_ip ip address of the robot controller
     * @param robot_port port used for fri communication
     */
    IiwaFriDriver(const std::string& robot_ip, int robot_port);

    /**
     * @brief starts the connection with the robot controller and starts the @ref run function
     * @return true if the connection is made with the robot controller
     */
    bool initialise_connection();

    /**
     * @brief gets the latest joint position from the iiwa state
     * @param joint_positions reference to array that will hold the joint positions
     */
    void read_joint_position(std::array<double, 7> &joint_positions);

    /**
     * @brief gets the latest joint velocity from the iiwa state
     * @param joint_velocity reference to array that will hold the joint velocities
     */
    void read_joint_velocity(std::array<double, 7> &joint_velocity);

    /**
     * @brief gets the latest joint torque from the iiwa state
     * @param joint_torque reference to array that will hold the joint torques
     */
    void read_joint_torque(std::array<double, 7> &joint_torque);

    /**
     * @brief gets the latest external joint torque from the iiwa state
     * The external joint torque is only the torques being applied externally and
     * has compensated for the weights of the links and payload (if known by the robot controller)
     * @param joint_torque reference to array that will hold the joint torques
     */
    void read_external_joint_torque(std::array<double, 7> &joint_torque);

    /**
     * @brief sets the joint positions to be sent to the robot controller on the next write command
     * @param joint_positions array of desired joint positions to send to the robot controller
     */
    void write_joint_position(std::array<double, 7> &joint_positions);

    /**
     * @brief sets the joint torque to be sent to the robot controller on the next write command
     * @param joint_torque array of desired joint torques to send to the robot controller
     */
    void write_joint_torque(std::array<double, 7> &joint_torque);

    /**
     * @brief sets the desired wrench to be sent to the robot controller on the next write command
     * @param wrench desired wrench to send to the robot controller
     */
    void write_end_effector_wrench(std::array<double, 6> &wrench);

protected:
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
