import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "package",
            default_value="robot_hardware",
            description="The package the URDF file is in",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether or not to use sim time",
        )
    )

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    xacro_file = os.path.join(get_package_share_directory('robot_hardware'), 'urdf', 'core.urdf.xacro')
  
    # Add the sim_mode argument to the xacro file
    robot_description_contents = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_contents, 'use_sim_time': use_sim_time}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params]
    )

    nodes = [
        robot_state_publisher_node
    ]
    
    # Launch!
    return LaunchDescription(declared_arguments + nodes)