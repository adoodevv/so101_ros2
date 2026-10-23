#!/usr/bin/env python3
"""
Launch RViz visualization for the SO-101 arm.

This launch file sets up robot state publisher, joint state publisher, and RViz2.
It processes the URDF/XACRO model and generates ros2_control configuration when
simulation support is enabled.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def process_ros2_controllers_config(context):
    """
    Render a prefixed ros2_control controller config for Gazebo, if needed.

    With an empty prefix gz_ros2_control uses the installed
    so101_moveit_config/config/<robot_name>/ros2_controllers.yaml. With a
    prefix, ros2_controllers_template.yaml is rendered to a temporary file
    and passed to xacro as `controllers_file`; nothing is written into the
    source or install trees.
    """
    prefix = LaunchConfiguration('prefix').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context)

    if use_gazebo != 'true' or not prefix:
        return []

    template_path = os.path.join(
        get_package_share_directory('so101_moveit_config'),
        'config', robot_name, 'ros2_controllers_template.yaml')
    with open(template_path, 'r', encoding='utf-8') as file:
        processed_content = file.read().replace('${prefix}', prefix)

    with tempfile.NamedTemporaryFile(
            'w', encoding='utf-8', prefix=f'{prefix}{robot_name}_ros2_controllers_',
            suffix='.yaml', delete=False) as file:
        file.write(processed_content)

    return [SetLaunchConfiguration('controllers_file', file.name)]


ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='so101',
                          description='Name of the robot'),
    DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('use_camera', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use the RGBD Gazebo plugin for point cloud'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation'),
    DeclareLaunchArgument('use_fake_hardware', default_value='false',
                          choices=['true', 'false'],
                          description='Use mock_components/GenericSystem for ros2_control '
                                      '(mutually exclusive with use_gazebo)'),
]


def generate_launch_description():
    """Generate the launch description for SO-101 arm visualization."""
    urdf_package = 'so101_description'
    urdf_filename = 'so101.urdf.xacro'
    rviz_config_filename = 'robot_arm_description.rviz'

    pkg_share = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share, 'urdf', 'robots', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, 'rviz', rviz_config_filename])

    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_jsp = LaunchConfiguration('use_jsp')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_use_jsp_cmd = DeclareLaunchArgument(
        name='use_jsp',
        default_value='false',
        choices=['true', 'false'],
        description='Enable the joint state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'robot_name:=', LaunchConfiguration('robot_name'), ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo'), ' ',
        'use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'), ' ',
        'controllers_file:=', LaunchConfiguration('controllers_file', default=''), ' ',
        'use_camera:=', LaunchConfiguration('use_camera')
    ]), value_type=str)

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp))

    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(OpaqueFunction(function=process_ros2_controllers_config))

    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_jsp_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
