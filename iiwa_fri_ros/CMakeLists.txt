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
    add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)


find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fri_client_sdk REQUIRED)
find_package(controller_manager REQUIRED)

include_directories(
        include
        #${controller_manager_INCLUDE_DIRS}
)

add_library(IiwaFriDriver
        STATIC
        src/iiwa_fri_driver.cpp
        src/IiwaFRIInterface.cpp
        )

ament_target_dependencies(IiwaFriDriver
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        fri_client_sdk
        rclcpp
        )

add_library(${PROJECT_NAME}
        SHARED
        src/IiwaHWInterface.cpp
        )
target_link_libraries(${PROJECT_NAME}
        IiwaFriDriver
        )

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        hardware_interface
        pluginlib
        rclcpp
        std_srvs
        fri_client_sdk

        )

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


# Installation
install(
        TARGETS ${PROJECT_NAME}
        DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
        IiwaFriDriver
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()