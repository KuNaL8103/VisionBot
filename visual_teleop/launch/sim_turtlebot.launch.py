#!/usr/bin/env python3
"""
Simulated TurtleBot3 Launch File

Launches Gazebo with TurtleBot3 Burger model in the empty world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # Set TURTLEBOT3_MODEL to burger (required by spawn_turtlebot3.launch.py)
    set_turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    return LaunchDescription([
        set_turtlebot3_model,
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Include the empty_world launch from turtlebot3_gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(turtlebot3_gazebo_share, 'launch', 'empty_world.launch.py')
            ]),
            launch_arguments={
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ),
    ])