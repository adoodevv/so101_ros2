#!/usr/bin/env python3
"""
Launch the SO-101 MoveIt stack on fake (mock) hardware, without Gazebo.

Starts robot_state_publisher, ros2_control_node backed by
mock_components/GenericSystem, the joint_state_broadcaster, arm_controller and
gripper_controller spawners, and move_group. RViz is optional and off by
default, so the stack runs headless (CI, system tests, containers).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the fake-hardware MoveIt launch description."""
    pkg_description = FindPackageShare('so101_description')
    pkg_moveit = FindPackageShare('so101_moveit_config')

    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')

    ros2_controllers_path = PathJoinSubstitution(
        [pkg_moveit, 'config', robot_name, 'ros2_controllers.yaml'])

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description.find('so101_description'),
                         'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'use_gazebo': 'false',
            'use_fake_hardware': 'true',
            'use_camera': 'false',
            'use_sim_time': 'false',
            'use_rviz': 'false',
            'use_jsp': 'false',
            'jsp_gui': 'false',
        }.items(),
    )

    # Reads the URDF (including the mock_components <ros2_control> block) from
    # the /robot_description topic published by robot_state_publisher.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_path],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'arm_controller',
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit.find('so101_moveit_config'),
                         'launch', 'move_group.launch.py')),
        launch_arguments={
            'robot_name': robot_name,
            'use_gazebo': 'false',
            'use_fake_hardware': 'true',
            'use_camera': 'false',
            'use_sim_time': 'false',
            'use_rviz': use_rviz,
        }.items(),
    )

    # Each include runs in its own GroupAction scope: launch arguments set by
    # one include (robot_state_publisher's use_rviz:=false and rviz_config_file)
    # would otherwise leak into the next and override what it is passed.
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='so101',
            description='Robot config directory under so101_moveit_config/config'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', choices=['true', 'false'],
            description='Start RViz with the MoveIt MotionPlanning plugin'),
        GroupAction([robot_state_publisher_launch]),
        ros2_control_node,
        controller_spawner,
        GroupAction([move_group_launch]),
    ])
