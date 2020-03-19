FROM ros:melodic-perception-stretch

RUN apt update && apt install -y python-catkin-tools \
ros-melodic-moveit-* \
ros-melodic-controller-manager \
ros-melodic-joint-state-controller \
ros-melodic-effort-controllers \
ros-melodic-position-controllers \
ros-melodic-velocity-controllers \
ros-melodic-gazebo-* \
ros-melodic-joint-trajectory-controller \
ros-melodic-combined-robot-hw \
ros-melodic-rqt-joint-trajectory-controller \
ros-melodic-web-video-server \
ros-melodic-trac-ik-kinematics-plugin

RUN mkdir -p catkin_ws/src
