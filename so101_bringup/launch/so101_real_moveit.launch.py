#!/usr/bin/env python3
"""
Launch the SO-101 follower arm (real hardware) with ros2_control and MoveIt.

Starts robot_state_publisher, ros2_control_node driving the arm through
so101_hardware/SO101SystemHardware, spawners for joint_state_broadcaster,
arm_controller and gripper_controller, and move_group, with RViz by default.

The arm is calibrated with LeRobot. By default the calibration is read from
$HF_LEROBOT_HOME/calibration/robots/so_follower/<follower_id>.json
(HF_LEROBOT_HOME defaults to ~/.cache/huggingface/lerobot), the same file
`lerobot-calibrate --robot.type=so101_follower --robot.id=<follower_id>` writes.

When the hardware activates it holds the arm's current pose; on shutdown
torque is disabled and the arm goes limp, so support it if it is raised.

read_only:=true never enables torque: only joint_state_broadcaster and RViz
run, and you move the arm by hand to check that the model follows (joint
directions, zero pose, gripper open/closed) before the first powered run.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def default_calibration_file(follower_id):
    """Return LeRobot's calibration path for an so_follower with this id."""
    lerobot_home = os.environ.get(
        'HF_LEROBOT_HOME', os.path.expanduser('~/.cache/huggingface/lerobot'))
    return os.path.join(
        lerobot_home, 'calibration', 'robots', 'so_follower', f'{follower_id}.json')


def launch_setup(context):
    """Resolve the calibration file, then build the actions."""
    robot_name = LaunchConfiguration('robot_name').perform(context)
    port = LaunchConfiguration('port').perform(context)
    follower_id = LaunchConfiguration('follower_id').perform(context)
    calibration_file = os.path.expanduser(
        LaunchConfiguration('calibration_file').perform(context)
        or default_calibration_file(follower_id))
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    read_only = LaunchConfiguration('read_only').perform(context)

    if not os.path.isfile(calibration_file):
        raise RuntimeError(
            f"Calibration file '{calibration_file}' not found. Calibrate the follower with "
            f'`lerobot-calibrate --robot.type=so101_follower --robot.port={port} '
            f'--robot.id={follower_id}` or pass calibration_file:=<path>.')
    if not os.path.exists(port):
        raise RuntimeError(
            f"Serial port '{port}' does not exist. Is the follower arm plugged in and "
            'powered? List ports with `ls /dev/ttyACM*`.')

    pkg_description = FindPackageShare('so101_description').find('so101_description')
    pkg_moveit = FindPackageShare('so101_moveit_config').find('so101_moveit_config')
    hardware_args = {
        'robot_name': robot_name,
        'use_gazebo': 'false',
        'use_fake_hardware': 'false',
        'use_real_hardware': 'true',
        'port': port,
        'calibration_file': calibration_file,
        'use_camera': 'false',
        'use_sim_time': 'false',
    }

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={
            **hardware_args,
            'read_only': read_only,
            # In read-only mode show the plain model; otherwise MoveIt's RViz is used.
            'use_rviz': 'true' if read_only == 'true' else 'false',
            'use_jsp': 'false',
            'jsp_gui': 'false',
        }.items(),
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_moveit, 'config', robot_name, 'ros2_controllers.yaml'),
            # so101_hardware clamps commands to the URDF limits and the calibrated
            # range itself. The controller manager's limiter would instead stop
            # every controller when a joint reads past a URDF limit, which the
            # real arm does in its folded rest pose.
            {'enforce_command_limits': False},
        ],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen',
    )

    controllers = ['joint_state_broadcaster']
    if read_only != 'true':
        controllers += ['arm_controller', 'gripper_controller']
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            *controllers,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')),
        launch_arguments={**hardware_args, 'use_rviz': use_rviz}.items(),
    )

    # Without the hardware nothing else is useful; stop instead of leaving
    # move_group and RViz running against a dead controller manager.
    shutdown_without_hardware = RegisterEventHandler(OnProcessExit(
        target_action=ros2_control_node,
        on_exit=[EmitEvent(event=Shutdown(
            reason='ros2_control_node exited; see its errors above'))],
    ))
    actions = [robot_state_publisher, ros2_control_node, controller_spawner,
               shutdown_without_hardware]
    if read_only != 'true':
        actions.append(move_group)
    return actions


def generate_launch_description():
    """Generate the real-hardware MoveIt launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyACM0',
            description='Serial port of the follower arm. Ports are numbered in plug-in order; '
                        'a /dev/serial/by-id/... path is stable.'),
        DeclareLaunchArgument(
            'follower_id', default_value='my_follower',
            description='LeRobot id the follower was calibrated with'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='LeRobot calibration JSON; empty derives it from follower_id'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz with the MotionPlanning plugin'),
        DeclareLaunchArgument(
            'read_only', default_value='false', choices=['true', 'false'],
            description='Never enable torque; show the arm in RViz while you move it by hand'),
        DeclareLaunchArgument(
            'robot_name', default_value='so101',
            description='Robot config directory under so101_moveit_config/config'),
        OpaqueFunction(function=launch_setup),
    ])
