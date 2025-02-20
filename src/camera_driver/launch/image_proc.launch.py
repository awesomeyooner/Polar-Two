from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch
import launch_ros
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

# https://docs.ros.org/en/rolling/p/image_proc/doc/tutorials.html

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            namespace='camera',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ],
        )
    ]

    container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
    )

    return launch.LaunchDescription([container])