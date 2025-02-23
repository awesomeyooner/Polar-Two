# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value="False",
        description='use sim time'
        )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_file_arg = DeclareLaunchArgument(
        'rviz_file',
        default_value="view.rviz",
        description='which .rviz file to use'
        )

    rviz_file = LaunchConfiguration('rviz_file')
    
    hardware_package = "microros_hardware_interface"

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            '-d', PathJoinSubstitution([
                get_package_share_directory(hardware_package), 'description', 'rviz', rviz_file
            ])
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_file_arg,
        rviz
    ])
