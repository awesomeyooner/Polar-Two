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

    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", 
        "microros.launch.py"))
    )

    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", 
        "controller_manager.launch.py"))
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", 
        "robot_state_publisher.launch.py")),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    joint_state_broadcaster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", 
        "joint_state_broadcaster.launch.py"))
    )

    external_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", 
        "external_controllers.launch.py")),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", 
        "slam_toolbox.launch.py")),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='boxed_world.world',
        description='World to load'
        )
    

    world = PathJoinSubstitution([
        get_package_share_directory(hardware_package),
        'bringup',
        'worlds',
        LaunchConfiguration('world')
    ])

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_robot',
                                   '-z', '0.1'],
                        output='screen')
    
    bridge_params = os.path.join(get_package_share_directory(hardware_package), 'bringup', 'config','gz_bridge.yaml')

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(hardware_package), 'bringup', 'launch', 'rviz2.launch.py')]),
                    launch_arguments={'use_sim_time': 'True', 'rviz_file': 'view.rviz'}.items()
             )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image_raw", 
            "/oak/left/image_raw", 
            "/oak/right/image_raw", 
            "/oak/rgb/image", 
            "/oak/rgb/depth_image"
        ]
    )

    return LaunchDescription([
        # microros_launch,
        controller_manager_launch,
        joint_state_broadcaster_launch,
        robot_state_publisher_launch,
        external_controllers,

        world_arg,
        gazebo,
        spawn_entity,

        ros_gz_bridge,
        rviz,
        ros_gz_image_bridge,

        slam_toolbox_launch
    ])
