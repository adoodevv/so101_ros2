#!/usr/bin/env python3
"""
Launch MoveIt 2 for the SO-101 arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name_moveit_config = 'so101_moveit_config'
    package_name_description = 'so101_description'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')
    use_gazebo = LaunchConfiguration('use_gazebo')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_package = LaunchConfiguration('rviz_config_package')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='so101',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_use_camera_cmd = DeclareLaunchArgument(
        name='use_camera',
        default_value='true',
        description='Include the D435 camera in the robot description')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Include Gazebo ros2_control blocks in the robot description')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='move_group.rviz',
        description='RViz configuration file')

    declare_rviz_config_package_cmd = DeclareLaunchArgument(
        name='rviz_config_package',
        default_value=package_name_moveit_config,
        description='Package containing the RViz configuration file')

    def configure_setup(context):
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        use_camera_str = LaunchConfiguration('use_camera').perform(context)
        use_gazebo_str = LaunchConfiguration('use_gazebo').perform(context)

        pkg_share_moveit_config = get_package_share_directory(package_name_moveit_config)
        pkg_share_description = get_package_share_directory(package_name_description)

        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)
        urdf_path = os.path.join(
            pkg_share_description, 'urdf', 'robots', f'{robot_name_str}.urdf.xacro')

        initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
        joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(
            config_path, 'pilz_cartesian_limits.yaml')

        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .robot_description(
                file_path=urdf_path,
                mappings={
                    'robot_name': robot_name_str,
                    'use_gazebo': use_gazebo_str,
                    'use_camera': use_camera_str,
                },
            )
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=['ompl', 'pilz_industrial_motion_planner', 'stomp'],
                default_planning_pipeline='ompl',
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        move_group_capabilities = {
            'capabilities': 'move_group/ExecuteTaskSolutionCapability'
        }

        start_move_group_node_cmd = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                {'start_state': {'content': initial_positions_file_path}},
                move_group_capabilities,
            ],
        )

        start_rviz_node_cmd = Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                [FindPackageShare(rviz_config_package), '/rviz/', rviz_config_file],
            ],
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': use_sim_time},
            ],
        )

        exit_event_handler = RegisterEventHandler(
            condition=IfCondition(use_rviz),
            event_handler=OnProcessExit(
                target_action=start_rviz_node_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        )

        return [start_move_group_node_cmd, start_rviz_node_cmd, exit_event_handler]

    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_config_package_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_camera_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld
