from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch
import launch_ros

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('my_robot'),'config','joystick.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joystick_driver'),

        launch_ros.actions.Node(
            package='my_robot',
            executable='joystick_teleop',
            name='joystick_teleop',
            parameters=[joy_params]
            )

            #('/cmd_vel', '/diffbot_base_controller/cmd_vel')
  ])