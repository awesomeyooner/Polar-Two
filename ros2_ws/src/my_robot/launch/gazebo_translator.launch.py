from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch
import launch_ros

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(get_package_share_directory('my_robot'),'config','gazebo_params.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_robot',
            executable='gazebo_translator',
            name='gazebo_translator',
            parameters=[params]
            )

            #('/cmd_vel', '/diffbot_base_controller/cmd_vel')
  ])