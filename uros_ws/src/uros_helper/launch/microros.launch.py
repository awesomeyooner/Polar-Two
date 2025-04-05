from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
import launch
import launch_ros

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_node',
            arguments=['serial', '--dev', '/dev/ttyUSB0'],
            output='screen',
        ),
  ])