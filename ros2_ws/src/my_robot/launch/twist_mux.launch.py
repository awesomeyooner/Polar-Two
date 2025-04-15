import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():

    package_name = "my_robot"

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    
    joystick_driver = Node(
        package="joy",
        executable="joy_node",
        name="joystick_driver"
    )

    joystick_teleop = Node(
        package="my_robot",
        executable="joystick_teleop",
        name="joystick_teleop"
    )
    
    return launch.LaunchDescription([
        twist_mux,
        joystick_driver,
        joystick_teleop
            #('/cmd_vel', '/diffbot_base_controller/cmd_vel')
  ])