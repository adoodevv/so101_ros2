# so101_ros2

![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

ROS 2 stack for the [SO-101](https://github.com/TheRobotStudio/SO-ARM100) robot arm on **ROS 2 Jazzy**.

## Packages

| Package | Description |
|---------|-------------|
| [so101_ros2](so101_ros2/) | Meta-package that pulls in the full stack |
| [so101_description](so101_description/) | URDF/xacro, meshes, RViz config |
| [so101_moveit_config](so101_moveit_config/) | ros2_control controller configuration |
| [so101_gazebo](so101_gazebo/) | Gazebo Harmonic simulation and worlds |
| [so101_bringup](so101_bringup/) | Convenience launch scripts |
| [so101_system_tests](so101_system_tests/) | System test scripts (planned) |

## Status

- [x] Robot description (URDF/xacro, meshes)
- [x] RViz visualization
- [x] ros2_control integration (Gazebo)
- [x] Gazebo simulation with pick-and-place world
- [x] Arm and gripper control in simulation
- [ ] MoveIt 2 motion planning (SRDF, move_group, planning pipelines)
- [ ] Hardware interface (real robot)
- [ ] System test scripts

## Prerequisites

- Ubuntu 24.04 (Noble)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Gazebo Harmonic (`ros-jazzy-ros-gz`)
- ros2_control stack (`ros-jazzy-ros2-control`, `ros-jazzy-gz-ros2-control`)

## Build

```bash
cd ~/ros2_ws/src
git clone <repo-url> so101_ros2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Quick start

**RViz only** (no simulation):

```bash
ros2 launch so101_description robot_state_publisher.launch.py
```

**Gazebo simulation** (robot + controllers + RViz):

```bash
ros2 launch so101_gazebo so101.gazebo.launch.py
```

**Move the arm** (after simulation is running):

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'],
  points: [{positions: [0.3, -0.8, 0.6, -0.3, 0.5], time_from_start: {sec: 3, nanosec: 0}}]
}"
```

**Open / close gripper**:

```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.5]}"
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.17]}"
```

See individual package READMEs for details.

## Robot joints

| Joint | Type | Limit (rad) |
|-------|------|-------------|
| `shoulder_pan` | revolute | -1.92 … 1.92 |
| `shoulder_lift` | revolute | -1.75 … 1.75 |
| `elbow_flex` | revolute | -1.69 … 1.69 |
| `wrist_flex` | revolute | -1.66 … 1.66 |
| `wrist_roll` | revolute | -2.74 … 2.84 |
| `gripper` | revolute | -0.17 … 1.75 |

## Roadmap

description → simulation → planning → hardware control
