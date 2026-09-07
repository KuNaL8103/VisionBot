#!/usr/bin/env python3
"""
Perception Launch File - Placeholder

Launches the perception node with camera driver.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('visual_teleop')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the ROS2 parameters file to use'
        ),

        Node(
            package='visual_teleop',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            # TODO: Add camera driver node (v4l2_camera or similar)
            # TODO: Add image transport if needed
        ),
    ])