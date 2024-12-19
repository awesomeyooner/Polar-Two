import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    hardware_package = "carbot_hardware"

    carbot_hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", "carlikebot.launch.py"))
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(hardware_package), "bringup", "launch", "robot_state_publisher.launch.py")),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ===================================================================================

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # ======================================================================================

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

    # ===================================================================================

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
            "position_controller",
            "velocity_controller",
            "--controller-manager", 
            "/controller_manager"
            ],
    )

    # =====================================================================================================

    vision_package = "lane_detector"

    camera_compressor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(vision_package), "launch", "camera_compressor.launch.py"))
    )

    detection_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(vision_package), "launch", "detection_model.launch.py"))
    )

    # ===================================================================================================


    gazebo_params_file = os.path.join(get_package_share_directory('my_robot'),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot'],
                        output='screen')

    rviz_params = os.path.join(get_package_share_directory('my_robot'), 'config', 'simtime.yaml')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[rviz_params]
    )

    return LaunchDescription([
        #diffbot_hardware_interface_launch,
        robot_state_publisher_launch,
        #joint_state_publisher_node,
        joystick_launch,
        translator_launch,

        gazebo,
        spawn_entity,
        robot_bicycle_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz_node,

        #camera_compressor_launch,
        #detection_model_launch
        # component2_launch,
        # Add any additional launch files or actions here
    ])