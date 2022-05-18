from launch import LaunchDescription

import launch.actions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
          package='lawn_mower_control',
          executable='path_planner',
          name="path_planner",
          output='screen'),
        Node(
          package='lawn_mower_control',
          executable='zone_mapper',
          name="zone_mapper",
          output='screen'),
    ])