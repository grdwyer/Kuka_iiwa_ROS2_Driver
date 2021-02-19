import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    ld = LaunchDescription()

    use_ros2_control = True


    # Get URDF via xacro
    robot_description_path = os.path.join(get_package_share_directory('iiwa_fri_ros'), 'config', 'load_iiwa.xacro')

    robot_description_config = xacro.process_file(robot_description_path,
                                                  mappings={'use_ros2_control': str(use_ros2_control).lower(),
                                                            'robot_ip': '192.170.10.2',
                                                            'robot_port': '30200'}
                                                  )

    robot_description = {'robot_description': robot_description_config.toxml()}


    iiwa_controller = os.path.join(
        get_package_share_directory('iiwa_fri_ros'),
        'config',
        'iiwa_fri_control.yaml'
    )

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'iiwa_link_0'])

    # Publishes tf's for the robot
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # # RViz
    # rviz_config_file = get_package_share_directory(
    #     'ur_ros2_control_demos') + "/config/rviz/config.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     # arguments=['-d', rviz_config_file],
                     parameters=[robot_description]
                     )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, iiwa_controller],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    return LaunchDescription([ros2_control_node, rviz_node, robot_state_pub_node, static_tf])
