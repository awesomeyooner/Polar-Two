import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():

    params = os.path.join(get_package_share_directory('my_robot'),'config','effort_translator.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_robot',
            executable='effort_translator',
            name='effort_translator',
            parameters=[params]
            )
  ])