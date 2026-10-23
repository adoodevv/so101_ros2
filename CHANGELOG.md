# Changelog

All notable changes to this repository are listed here. Versions apply to every
package in the repository.

## [Unreleased]

### Added

- `so101_hardware`: ros2_control `SystemInterface` for the SO-101 follower arm
  (Feetech STS3215 servos, protocol 0, 1 Mbps). It reads LeRobot calibration
  files and verifies them against the servos' EEPROM, refusing to start on a
  mismatch. Joint positions use LeRobot's degrees convention (zero at
  mid-range); the gripper maps linearly onto its URDF limits. On activate it
  holds the current pose before enabling torque. It applies LeRobot's servo
  settings (including gripper torque/current protection) and exposes
  velocity, load, temperature and voltage state interfaces.
- `so101_hardware_check`: read-only diagnostic (ping, calibration, live angles).
- `so101_servo_sim`: simulated STS3215 servo bus on a pseudo-terminal.
- `use_real_hardware`, `port`, `calibration_file`, `read_only` xacro/launch args.
- `so101_bringup/launch/so101_real_moveit.launch.py` (`follower_id`, `port`,
  `read_only`).
- System tests run against two backends on every `colcon test`: fake hardware,
  and the real plugin on simulated servos. They can also target the physical
  arm (`SO101_TEST_BACKEND=real`).
- On real hardware the plugin enforces the joint limits (URDF limits intersected
  with the calibrated range; arm joints held 0.03 rad inside, since a loaded
  servo settles ~10 ticks short of its goal) and the launch disables the
  controller manager's `enforce_command_limits`, which stopped every controller
  when the arm rested past a URDF limit. Activation moves a joint up to 0.1 rad
  outside its limits back to the limit, and refuses beyond that.
- `so101_hardware_check` names the calibration file that matches the servos on a
  mismatch, and flags joints outside their limits.
- `so101_servo_sim --start JOINT=TICKS` sets initial positions.

### Fixed

- `.gitignore` ignored every `src/` directory, so `so101_hardware/src` was
  never committed.

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
  out-of-limits arm goal never drives a joint past its limit; a trajectory sent
  straight to `arm_controller` past the limit is clamped.
- `NOTICE.md` crediting TheRobotStudio/SO-ARM100 (Apache-2.0) for meshes and
  URDF data.
- `LICENSE` in `so101_teleop_bridge`; the upstream Apache-2.0 text next to the
  SO-ARM100 meshes (`so101_description/meshes/so_arm101/LICENSE`).
- `controllers_file` xacro arg for the Gazebo ros2_control plugin.

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
- **Joint limits enforced for every command:** `enforce_command_limits: true`
  in `ros2_controllers.yaml`. Commands from any source (MoveIt, controller
  topics/actions, teleop) are clamped to the URDF limits, in fake hardware and
  Gazebo alike.
- **SRDF `ready` redefined** to `shoulder_lift -0.7, elbow_flex 0.6,
  wrist_flex 1.4` (tool pointing down ~0.19 m in front of the base). The old
  pose folded the gripper back onto the shoulder and only passed collision
  checking because the jaw mesh was buried inside the shoulder mesh. The new
  pose keeps >= 0.15 rad of clearance from self-collision and plans first time.
- SRDF `arm` group is a chain `base_link` -> `gripper_frame_link`, so the tool
  frame is the IK tip and the gripper end-effector has a parent group;
  `arm_with_gripper` no longer has a (non-functional) KDL solver.
- `move_group` only requests `ExecuteTaskSolutionCapability` when MoveIt Task
  Constructor is installed.
- `robot_state_publisher.launch.py` no longer writes `ros2_controllers.yaml`
  into `~/ros2_ws/src` and `~/ros2_ws/install`; prefixed Gazebo runs render the
  template to a temporary file instead.
- Package versions set to 0.2.0.

### Fixed

- Documentation: the gripper runs `joint_trajectory_controller/JointTrajectoryController`
  (`/gripper_controller/follow_joint_trajectory`), not `forward_command_controller`
  with `/gripper_controller/commands`. Also corrected the MoveIt config README
  (MoveIt is configured), the `controller_load_delay` default and the Gazebo
  spawn `z` default.
- MoveIt gripper planning in Gazebo: `closed` was an invalid goal and `open`
  could not be planned, because the Gazebo jaw collision box touches
  `wrist_link` below ~0.25 rad. The pair never touches with the real meshes
  over the full `wrist_roll` x `gripper` range and is now disabled in the SRDF.
- The arm tipped over in Gazebo: the base was not attached to the world, and at
  `home` the arm's overhang (~0.39 m) pulls the 0.147 kg base forward (~39 deg
  pitch). The `world` link and fixed `world_joint` are now also generated with
  `use_gazebo`, which pins the base at its spawn pose.
- Pre-existing lint failures (lint_cmake, flake8, pep257); `colcon test` passes
  for every package.

### Known issues

- The MoveIt configuration (SRDF and YAMLs) assumes an empty `prefix`.
- MoveIt clamps out-of-bounds joint goals to the limit rather than rejecting
  them; with limit enforcement the arm stops at the limit.
- `move_group` (MoveIt 2.12) segfaults in `rclcpp::Executor::~Executor()` as it
  exits, after all work has stopped. backward-cpp then symbolizes the stack
  trace, which can take longer than launch's 5 s SIGINT grace period on slower
  machines, so launch escalates to SIGTERM. Harmless; upstream issue.
- `ament_xmllint` downloads the `package.xml` schema from download.ros.org, so
  it can fail without network access.
