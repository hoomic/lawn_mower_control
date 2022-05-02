from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='lawn_mower_control',
            executable='test_stereo_control.py',
            output='screen',
            arguments=["0.5"]),
    ])