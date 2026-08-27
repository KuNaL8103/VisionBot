#!/usr/bin/env python3
"""
Simulated TurtleBot3 Launch File - Placeholder

Launches Gazebo with TurtleBot3 simulation.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # TODO: Get turtlebot3_gazebo package share directory
    # turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='waffle',
            description='TurtleBot3 model type [burger, waffle, waffle_pi]'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Gazebo world file'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial x position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial y position'
        ),

        # TODO: Include turtlebot3_gazebo launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py'])
        #     ]),
        #     launch_arguments={
        #         'model': LaunchConfiguration('model'),
        #         'world': LaunchConfiguration('world'),
        #         'x_pose': LaunchConfiguration('x_pose'),
        #         'y_pose': LaunchConfiguration('y_pose'),
        #     }.items()
        # ),
    ])