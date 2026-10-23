# Changelog

All notable changes to this repository are listed here. Versions apply to every
package in the repository.

## [0.2.0] - Unreleased

### Added

- **Fake hardware:** `use_fake_hardware` xacro/launch argument (default `false`)
  selecting `mock_components/GenericSystem` with `calculate_dynamics: false` and
  initial values from `so101_moveit_config/config/so101/initial_positions.yaml`.
  Combining it with `use_gazebo` aborts xacro with an error. Threaded through
  `so101.urdf.xacro`, `robot_state_publisher.launch.py` and `move_group.launch.py`.
- `so101_bringup/launch/so101_fake_moveit.launch.py`: robot_state_publisher,
  ros2_control_node, controller spawners and move_group without Gazebo;
  RViz optional (`use_rviz`, default `false`).
- **Mountable description:** `so101_robot` macro params `parent` (default
  `world`), `xyz` and `rpy`. A non-`world` parent gets `${prefix}mount_joint`
  and no `world` link. `so101.urdf.xacro` exposes them as xacro args.
- **System tests** (`so101_system_tests`, `launch_testing`, headless): controllers
  active; arm to SRDF `home` then `ready`; gripper `open` → `closed` → `open`;
  out-of-limits arm goal never drives a joint past its limit.
- `NOTICE.md` crediting TheRobotStudio/SO-ARM100 (Apache-2.0) for meshes and
  URDF data.
- `LICENSE` in `so101_teleop_bridge`.

### Changed

- `prefix` is now applied to every link, joint, transmission, actuator and
  material in the arm macro, to the D435 camera links/joints, and to the
  ros2_control system name. Previously a non-empty prefix only renamed the
  ros2_control joints and broke the robot. With the default empty prefix the
  generated URDF is unchanged.
- On fake hardware the gripper exposes only a `position` state interface; the
  unused `velocity`/`effort` interfaces remain in Gazebo (simulated effort,
  not force sensing).
- `so101_teleop_bridge` license declared as BSD-3-Clause.
- Package versions set to 0.2.0.

### Fixed

- Documentation: the gripper runs `joint_trajectory_controller/JointTrajectoryController`
  (`/gripper_controller/follow_joint_trajectory`), not `forward_command_controller`
  with `/gripper_controller/commands`. Also corrected the MoveIt config README
  (MoveIt is configured), the `controller_load_delay` default and the Gazebo
  spawn `z` default.

### Known issues

- The MoveIt configuration (SRDF and YAMLs) assumes an empty `prefix`.
- The SRDF `ready` pose is about 0.03 rad from a moving-jaw/shoulder
  self-collision; MoveIt rejects a share of OMPL paths into it after time
  parameterization and has to re-plan.
- MoveIt clamps out-of-bounds joint goals to the limit rather than rejecting
  them. Direct controller commands are not limited on fake hardware
  (`enforce_command_limits` is off).
- `move_group.launch.py` requests the `move_group/ExecuteTaskSolutionCapability`
  capability, which logs an error when MoveIt Task Constructor is not installed.
