#!/usr/bin/env python3
"""
Teleoperate the SO-101 follower with the SO-101 leader arm.

so101_real_moveit.launch.py with teleop:=true: the follower runs on
so101_hardware with streaming position controllers, and so101_leader reads
the leader and commands the follower. On start the follower moves slowly to
the leader's pose (sync_velocity), then follows it.

    ros2 launch so101_bringup so101_teleop.launch.py
    ros2 launch so101_bringup so101_teleop.launch.py leader_port:=/dev/ttyACM1
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = {
    'port': ('/dev/ttyACM0', 'Serial port of the follower arm'),
    'follower_id': ('my_follower', 'LeRobot id of the follower calibration'),
    'calibration_file': ('', 'Follower calibration JSON; empty derives it from follower_id'),
    'leader_port': ('/dev/ttyACM1', 'Serial port of the leader arm'),
    'leader_id': ('my_leader', 'LeRobot id of the leader calibration'),
    'leader_calibration_file': ('', 'Leader calibration JSON; empty derives it from leader_id'),
    'use_rviz': ('true', 'Show the follower in RViz'),
}


def generate_launch_description():
    """Include the real-hardware launch with teleop:=true."""
    real = os.path.join(
        FindPackageShare('so101_bringup').find('so101_bringup'),
        'launch', 'so101_real_moveit.launch.py')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, (default, description) in ARGUMENTS.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in ARGUMENTS},
                'teleop': 'true',
            }.items()),
    ])
