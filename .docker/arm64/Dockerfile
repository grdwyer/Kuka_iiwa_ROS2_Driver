FROM arm64v8/ros:noetic-robot

RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers

RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws/src
RUN git clone https://github.com/grdwyer/iiwa_fri_description.git 
COPY . /catkin_ws/src/iiwa_fri_stack

WORKDIR /catkin_ws/
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    catkin_make"]