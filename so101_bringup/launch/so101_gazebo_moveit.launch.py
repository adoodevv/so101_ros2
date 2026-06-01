#!/usr/bin/env python3
"""
Launch Gazebo simulation and MoveIt move_group together.

Starts Gazebo with controllers first, then move_group + MoveIt RViz once
controllers are expected to be active.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = FindPackageShare('so101_gazebo')
    pkg_moveit = FindPackageShare('so101_moveit_config')

    use_camera = LaunchConfiguration('use_camera')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    spawn_delay = LaunchConfiguration('spawn_delay')
    controller_load_delay = LaunchConfiguration('controller_load_delay')
    moveit_delay = LaunchConfiguration('moveit_delay')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo.find('so101_gazebo'), 'launch', 'so101.gazebo.launch.py')
        ),
        launch_arguments={
            'load_controllers': 'true',
            'use_camera': use_camera,
            'use_rviz': 'false',
            'use_robot_state_pub': 'true',
            'use_sim_time': use_sim_time,
            'world_file': world_file,
            'spawn_delay': spawn_delay,
            'controller_load_delay': controller_load_delay,
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit.find('so101_moveit_config'), 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_camera': use_camera,
            'use_gazebo': 'true',
            'use_sim_time': use_sim_time,
            'use_rviz': 'true',
        }.items(),
    )

    delayed_moveit = TimerAction(
        period=moveit_delay,
        actions=[moveit_launch],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='pick_and_place.world'),
        DeclareLaunchArgument('spawn_delay', default_value='2.0'),
        DeclareLaunchArgument('controller_load_delay', default_value='5.0'),
        DeclareLaunchArgument(
            'moveit_delay',
            default_value='10.0',
            description='Seconds after launch before starting move_group (after controllers load)'),
        gazebo_launch,
        delayed_moveit,
    ])
