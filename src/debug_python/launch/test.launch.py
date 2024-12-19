import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([

        # launch_ros.actions.Node(
        #     package='lane_detector',
        #     executable='camera_driver',
        #     name='camera_driver',
        #     ),

        launch_ros.actions.Node(
            package='debug_python',
            executable='test',
            name='test'
        )

            #('/cmd_vel', '/diffbot_base_controller/cmd_vel')
  ])