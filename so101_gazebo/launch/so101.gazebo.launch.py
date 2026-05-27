#!/usr/bin/env python3
"""
Launch Gazebo simulation with the SO-101 arm.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name_gazebo = 'so101_gazebo'
    package_name_description = 'so101_description'
    package_name_moveit = 'so101_moveit_config'

    default_robot_name = 'so101'
    default_world_file = 'pick_and_place.world'
    gazebo_worlds_path = 'worlds'

    ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'

    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_description = FindPackageShare(
        package=package_name_description).find(package_name_description)
    pkg_share_moveit = FindPackageShare(package=package_name_moveit).find(package_name_moveit)

    default_ros_gz_bridge_config_file_path = os.path.join(
        pkg_share_gazebo, ros_gz_bridge_config_file_path)

    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')

    world_path = PathJoinSubstitution([
        pkg_share_gazebo,
        gazebo_worlds_path,
        world_file
    ])

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    spawn_delay = LaunchConfiguration('spawn_delay')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='The name for the robot')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Flag to enable loading of ROS 2 controllers')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_camera_cmd = DeclareLaunchArgument(
        name='use_camera',
        default_value='false',
        description='Flag to enable the RGBD camera for Gazebo point cloud simulation')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Flag to enable RViz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, pick_and_place.world)')

    declare_x_cmd = DeclareLaunchArgument(
        name='x', default_value='0.0',
        description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
        name='y', default_value='0.0',
        description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
        name='z', default_value='0.05',
        description='z component of initial position, meters')

    declare_roll_cmd = DeclareLaunchArgument(
        name='roll', default_value='0.0',
        description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch', default_value='0.0',
        description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw', default_value='0.0',
        description='yaw angle of initial orientation, radians')

    declare_spawn_delay_cmd = DeclareLaunchArgument(
        name='spawn_delay',
        default_value='2.0',
        description='Seconds to wait before spawning the robot into Gazebo')

    declare_controller_load_delay_cmd = DeclareLaunchArgument(
        name='controller_load_delay',
        default_value='5.0',
        description='Seconds to wait before loading ros2_control controllers')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'jsp_gui': jsp_gui,
            'use_camera': use_camera,
            'use_gazebo': use_gazebo,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_robot_state_pub)
    )

    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_moveit, 'launch', 'load_ros2_controllers.launch.py')
        ]),
        launch_arguments={
            'controller_load_delay': LaunchConfiguration('controller_load_delay'),
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(load_controllers)
    )

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -v 4 ', world_path])])

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_config_file_path,
        }],
        output='screen'
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/camera_head/depth_image',
            '/camera_head/image',
        ],
        remappings=[
            ('/camera_head/depth_image', '/camera_head/depth/image_rect_raw'),
            ('/camera_head/image', '/camera_head/color/image_raw'),
        ],
        condition=IfCondition(use_camera)
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ])

    delayed_spawn_cmd = TimerAction(
        period=spawn_delay,
        actions=[start_gazebo_ros_spawner_cmd]
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_camera_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_spawn_delay_cmd)
    ld.add_action(declare_controller_load_delay_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(load_controllers_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)
    ld.add_action(delayed_spawn_cmd)

    return ld
