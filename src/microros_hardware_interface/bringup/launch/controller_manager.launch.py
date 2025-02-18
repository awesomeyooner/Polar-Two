import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    hardware_package = "microros_hardware_interface"

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Toggle to use sim time'
        )

    # Check if we're told to use sim time
    controller_params_file = os.path.join(get_package_share_directory(hardware_package),'bringup', 'config','controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        parameters=[controller_params_file, {'-use_sim_time': use_sim_time}]
    )
    

    # Launch!
    return LaunchDescription([
        use_sim_time_arg,
        controller_manager
    ])
