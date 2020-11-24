cmake_minimum_required(VERSION 3.5)
project(iiwa_fri_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fri_client_sdk REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
        src/IiwaHWInterface.cpp
#        include/iiwa_fri_ros/IiwaHWInterface.h
        src/IiwaFRIInterface.cpp
#        include/iiwa_fri_ros/IiwaFRIInterface.h
        )

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        hardware_interface
        fri_client_sdk
        joint_limits_interface
        geometry_msgs
        )

############################
## iiwa_streamer
############################
#add_executable(iiwa_streamer
#        src/IiwaStreamer.cpp)
#
### Add cmake target dependencies of the executable
### same as for the library above
#add_dependencies(iiwa_streamer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
### Specify libraries to link a library or executable target against
#target_link_libraries(iiwa_streamer
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        )

############################
## iiwa_controller
############################
add_executable(iiwa_controller
        src/IiwaController.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(iiwa_controller
        ${PROJECT_NAME}
        )

ament_package()