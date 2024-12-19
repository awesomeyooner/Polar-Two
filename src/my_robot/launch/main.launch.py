import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    hardware_package = "carbot_hardware"

    carbot_hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", "carlikebot.launch.py"))
    )

    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", "controller_manager.launch.py"))
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", "robot_state_publisher.launch.py")),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_bicycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            # "effort_controllers",
            # "-c", "/controller_manager",
            # "-t", "effort_controllers/JointGroupEffortController", 
            "velocity_controller",
            "position_controller",
            "--controller-manager", 
            "/controller_manager"
            ],
    )

    # =============================================================================================================

    robot_package= "my_robot"

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot_package), "launch", "joystick.launch.py"))
    )

    mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot_package), "launch", "twist_mux.launch.py"))
    )

    translator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(robot_package), "launch", "effort_translator.launch.py"))
    )

    # ============================================================================================================================================

    basic_camera_package = "camera_handler"

    camera_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(basic_camera_package), "launch", "camera_driver.launch.py"))
    )

    camera_compressor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(basic_camera_package), "launch", "camera_compressor.launch.py"))
    )

    # ================================================================================================================================

    lane_detection_package = "lane_detector"

    detection_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(lane_detection_package), "launch", "detection_model.launch.py"))
    )
    
    # ===============================================================================================================================

    foxglove = IncludeLaunchDescription(
        (os.path.join(FindPackageShare("foxglove_bridge").find("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"))      
    )
    

    return LaunchDescription([
        #diffbot_hardware_interface_launch,
        #carbot_hardware_interface_launch,
        controller_manager_launch,
        robot_state_publisher_launch,
        joint_state_broadcaster_spawner,
        robot_bicycle_controller_spawner,

        #joystick_launch,
        translator_launch,
        #camera_driver_launch,
        #camera_compressor_launch,
        #detection_model_launch,

        #foxglove

        # component2_launch,
        # Add any additional launch files or actions here
    ])