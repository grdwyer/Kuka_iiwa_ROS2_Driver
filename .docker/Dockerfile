ARG ROSDISTRO=foxy
FROM ros:$ROSDISTRO
ARG ROSDISTRO

RUN apt-get update && apt-get install -y \
    software-properties-common \
    ssh \
    git \
    libboost-dev \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    ros-foxy-xacro \
    wget \
    ros-$ROSDISTRO-rqt \
    ros-$ROSDISTRO-rviz2 \
    ros-$ROSDISTRO-angles

RUN mkdir -p /dev_ws/src

WORKDIR /dev_ws/src
COPY . /dev_ws/src/iiwa_fri_stack

WORKDIR /dev_ws/
RUN ["/bin/bash", "-c", "source /opt/ros/$ROSDISTRO/setup.bash &&\
    vcs import src < ./src/iiwa_fri_stack/iiwa_fri_ros/ros2_control.rosinstall &&\
    vcs pull src || true &&\
    colcon build --symlink-install --continue-on-error --cmake-args -Wno-dev || true"]