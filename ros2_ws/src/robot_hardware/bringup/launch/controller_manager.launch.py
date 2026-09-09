import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    # CHANGE ME
    package = "robot_hardware"

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Whether or not to use sim time. Defaults to False",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    controller_params_file = PathJoinSubstitution(
        [
            FindPackageShare(package),
            "config",
            "robot_hardware_controllers.yaml",
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        parameters=[controller_params_file, {'-use_sim_time': use_sim_time}]
    )

    lifecycle_tracker = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager,
            on_exit=[
                EmitEvent(event=Shutdown(reason="controller_manager exited"))
            ],
        )
    )

    nodes = [
        controller_manager,
        lifecycle_tracker
    ]

    # Launch!
    return LaunchDescription(declared_arguments + nodes)