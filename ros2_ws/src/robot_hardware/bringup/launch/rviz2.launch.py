# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Declare arguments
    declared_arguments = []

    # Which package to get the config from
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="robot_hardware",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )

    # The name of the config file
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value="default.rviz",
            description="The RViz .rviz config file to load. Defaults to `default.rviz`",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether or not to use sim time. Defaults to false",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    rviz_config_filename = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_config_filename]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nodes = [
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
