import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    #ros2 launch depthai_ros_driver rgbd_pcl.launch.py

    depth_launch = IncludeLaunchDescription(
        (os.path.join(FindPackageShare("depthai_ros_driver").find("depthai_ros_driver"), "launch", "rgbd_pcl.launch.py"))      
    )

    return launch.LaunchDescription([
        depth_launch
  ])