#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the robot.

This script creates a launch description that starts the necessary controllers
for operating the robotic arm and gripper in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Arm Controller: Controls the robot arm movements via /follow_joint_trajectory
    3. Gripper Action Controller: Controls gripper actions via /gripper_action

Launch Sequence:
    1. Joint State Broadcaster
    2. Arm Controller (starts after Joint State Broadcaster)
    3. Gripper Action Controller (starts after Arm Controller)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers."""
    controller_load_delay = LaunchConfiguration('controller_load_delay')

    declare_controller_load_delay_cmd = DeclareLaunchArgument(
        name='controller_load_delay',
        default_value='5.0',
        description='Seconds to wait before loading ros2_control controllers')

    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen')

    start_gripper_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen')

    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')

    delayed_start = TimerAction(
        period=controller_load_delay,
        actions=[start_joint_state_broadcaster_cmd]
    )

    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_arm_controller_cmd]))

    load_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_arm_controller_cmd,
            on_exit=[start_gripper_controller_cmd]))

    return LaunchDescription([
        declare_controller_load_delay_cmd,
        delayed_start,
        load_joint_state_broadcaster_cmd,
        load_arm_controller_cmd,
    ])
