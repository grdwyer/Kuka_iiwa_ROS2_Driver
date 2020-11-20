FROM ros:noetic-robot

RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-pip \
    ros-noetic-rqt \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers

RUN pip3 install catkin_tools

RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws/src
RUN git clone https://github.com/grdwyer/iiwa_fri_description.git 
COPY . /catkin_ws/src/iiwa_fri_stack

WORKDIR /catkin_ws/
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    catkin_make"]