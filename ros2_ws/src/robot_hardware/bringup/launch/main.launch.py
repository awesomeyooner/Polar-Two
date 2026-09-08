import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("robot_hardware"), "launch", 
        "controller_manager.launch.py"))
    )
    
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("robot_hardware"), "launch", 
        "robot_state_publisher.launch.py"))
    )

    joint_state_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("robot_hardware"), "launch", 
        "joint_state_broadcaster.launch.py"))
    )

    external_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("robot_hardware"), "launch", 
        "external_controllers.launch.py"))
    )
    
    nodes = [
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster,
        external_controllers
    ]

    return LaunchDescription(nodes)