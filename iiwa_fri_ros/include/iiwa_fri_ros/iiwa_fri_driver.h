//
// Created by george on 2/18/21.
//

#ifndef IIWA_FRI_ROS_IIWA_FRI_DRIVER_H
#define IIWA_FRI_ROS_IIWA_FRI_DRIVER_H

#include <iiwa_fri_ros/IiwaFRIInterface.h>
#include <fri_client_sdk/friUdpConnection.h>
#include <fri_client_sdk/friClientApplication.h>

class IiwaFriDriver{

    IiwaFriDriver(const std::string& robot_ip, int robot_port);

};


#endif //IIWA_FRI_ROS_IIWA_FRI_DRIVER_H
