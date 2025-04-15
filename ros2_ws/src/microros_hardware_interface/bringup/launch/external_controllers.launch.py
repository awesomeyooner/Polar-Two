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

    hardware_package = "microros_hardware_interface"
    
    external_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            # "effort_controllers",
            # "-c", "/controller_manager",
            # "-t", "effort_controllers/JointGroupEffortController", 
            # "velocity_controller",
            "differential_drive_controller",
            # "position_controller",
            # "effort_controller",
            "--controller-manager", 
            "/controller_manager"
            ],
    )

    nodes = [
        external_controllers
    ]

    return LaunchDescription(nodes)
