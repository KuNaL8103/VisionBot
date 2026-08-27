#!/usr/bin/env python3
"""
Full System Launch File - Placeholder

Launches perception + controller + simulated TurtleBot3 together.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visual_teleop_share = FindPackageShare('visual_teleop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([visual_teleop_share, 'config', 'params.yaml']),
            description='Path to the ROS2 parameters file to use'
        ),

        # Launch perception node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([visual_teleop_share, 'launch', 'perception.launch.py'])
            ]),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
            }.items()
        ),

        # Launch controller node
        Node(
            package='visual_teleop',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # TODO: Launch simulated TurtleBot3 (include sim_turtlebot.launch.py)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([visual_teleop_share, 'launch', 'sim_turtlebot.launch.py'])
        #     ]),
        # ),
    ])


from launch_ros.actions import Node