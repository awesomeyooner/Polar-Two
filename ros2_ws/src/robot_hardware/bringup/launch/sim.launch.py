import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    package = "robot_hardware"
    
    # Declare arguments
    declared_arguments = []

    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", "controller_manager.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
        }.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", "robot_state_publisher.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
        }.items()
    )

    joint_state_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", "joint_state_broadcaster.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
        }.items()
    )

    external_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", "external_controllers.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
        }.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": "default.world",
        }.items()
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), "launch", "rviz2.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
        }.items()
    )
   
    nodes = [
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster,
        external_controllers,

        gazebo,
        rviz2
    ]

    return LaunchDescription(declared_arguments + nodes)
