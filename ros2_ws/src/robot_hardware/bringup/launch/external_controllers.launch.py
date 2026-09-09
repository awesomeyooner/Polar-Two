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

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether or not to use sim time. Defaults to false",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    

    external_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "differential_drive_controller",
            "--controller-manager", 
            "/controller_manager"
            ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
   
    nodes = [
        external_controllers_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)