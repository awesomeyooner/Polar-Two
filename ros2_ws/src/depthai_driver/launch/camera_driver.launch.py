import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    config_file = os.path.join(get_package_share_directory('depthai_driver'),'config','config.yaml')

    camera_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')]),
                    launch_arguments={'params_file': config_file}.items()
             )


    return LaunchDescription([
        camera_launch
    ])