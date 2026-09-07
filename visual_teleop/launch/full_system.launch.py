#!/usr/bin/env python3
"""
Full System Launch File

Launches the complete visual teleoperation pipeline:
1. TurtleBot3 Gazebo simulation (empty world, Burger model)
2. Perception node (webcam -> YOLO -> ByteTrack -> /target/pose)
3. Controller node (/target/pose -> /cmd_vel TwistStamped)

With parameters loaded from config/params.yaml. Adds a startup delay
before controller_node so the Gazebo simulation is fully up first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visual_teleop_share = get_package_share_directory('visual_teleop')
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # Set TURTLEBOT3_MODEL to burger (required by spawn_turtlebot3.launch.py)
    set_turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Parameters file
    params_file = os.path.join(visual_teleop_share, 'config', 'params.yaml')

    # Declare launch arguments
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to the ROS2 parameters file to use'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of TurtleBot3'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of TurtleBot3'
    )

    # 1. Launch TurtleBot3 Gazebo simulation (empty world, Burger model)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_share, 'launch', 'empty_world.launch.py')
        ]),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # 2. Launch perception node (with short delay to let Gazebo start first)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(visual_teleop_share, 'launch', 'perception.launch.py')
        ]),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    # 3. Launch controller node with delay (so sim and perception are up first)
    # Delay of 8 seconds: enough for Gazebo to fully start, bridge to connect, and perception to warm up
    controller_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='visual_teleop',
                executable='controller_node',
                name='controller_node',
                output='screen',
                parameters=[LaunchConfiguration('params_file')],
                remappings=[
                    ('/cmd_vel', '/cmd_vel'),  # explicit for clarity
                ],
            )
        ]
    )

    return LaunchDescription([
        set_turtlebot3_model,
        declare_params_file,
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,

        # Start simulation first
        sim_launch,

        # Start perception node (short delay built into perception node via warmup_frames)
        perception_launch,

        # Start controller after sim + perception are ready
        controller_node,
    ])