# so101_system_tests

End-to-end system tests for the SO-101 stack. They run headless (no Gazebo, no RViz)
and drive the stack only through its public ROS interfaces. The same suite runs against
several backends, selected with `SO101_TEST_BACKEND`:

| Backend | Launches | Runs on `colcon test` |
|---------|----------|-----------------------|
| `fake` | `so101_fake_moveit.launch.py` (mock_components) | yes (`test_moveit_system_fake`) |
| `servo_sim` | `so101_real_moveit.launch.py`: the real `so101_hardware` plugin talking to `so101_servo_sim` (simulated STS3215 servos) | yes (`test_moveit_system_servo_sim`) |
| `real` | `so101_real_moveit.launch.py` on the physical arm (`SO101_PORT`, `SO101_CALIBRATION_FILE`), at 30 % speed | no; run by hand, see [so101_hardware](../so101_hardware/README.md#first-run-on-your-arm) |

## Tests

`test/test_moveit_system.launch.py` (`launch_testing`), run in order against one launched stack:

| Test | Checks | Budget |
|------|--------|--------|
| `test_01_controllers_active` | `joint_state_broadcaster`, `arm_controller`, `gripper_controller` reach `active` (`/controller_manager/list_controllers`) | 90 s |
| `test_02_arm_home_then_ready` | `arm` plans and executes through MoveIt (`/move_action`) to SRDF states `home`, then `ready`; `/joint_states` settles within the position tolerance of the SRDF values | 60 s |
| `test_03_gripper_open_closed_open` | `gripper` goes `open` → `closed` → `open` through MoveIt, each within the position tolerance | 60 s |
| `test_04_out_of_limits_goal_never_exceeds_limits` | An `arm` goal 0.5 rad past the `shoulder_pan` URDF limit never drives any joint past its limit: either it is rejected with no motion, or MoveIt clamps it to the limit (MoveIt 2 `JointConstraint` behaviour) and the other joints hold | 30 s |
| `test_05_direct_controller_command_clamped_to_limits` | A trajectory sent straight to `/arm_controller/follow_joint_trajectory` with `shoulder_pan` 0.5 rad past its limit leaves every joint within its URDF limits (controller manager `enforce_command_limits` on fake hardware, `so101_hardware` itself on the servo backends) | 30 s |
| `test_06_arm_returns_to_rest` | `arm` goes to the folded SRDF `rest` pose, so the physical arm rests when the motors switch off | 60 s |

The position tolerance is 0.01 rad in simulation and 0.08 rad on the physical arm, which sags
under gravity (worst measured: 0.048 rad on `elbow_flex` at `home`).

`test/test_teleop.launch.py` (`test_teleop`) runs leader-arm teleoperation on two simulated
arms: the leader's pose on `/leader/joint_states`; the follower moving to it no faster than
the 0.5 rad/s sync speed and then matching it; and, with the leader stopped (`SIGSTOP`), the
follower holding still and re-syncing when the leader answers again.

Target values are read from the installed SRDF and joint limits from
`/robot_description`, so the tests follow the robot configuration. Every wait
inside a test is bounded by that test's budget; CTest bounds the whole run at
300 s. Each run gets its own `ROS_DOMAIN_ID` (73 fake, 74 servo_sim, 75 teleop) with
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, isolated from other ROS 2 graphs on the machine.

## Run

```bash
cd ~/ros2_ws
colcon build --packages-up-to so101_system_tests
source install/setup.bash
colcon test --packages-select so101_system_tests
colcon test-result --verbose --test-result-base build/so101_system_tests
```

Full launch and test output is written to
`build/so101_system_tests/launch_test/test_moveit_system_<backend>.txt`.

## Known behaviour

- `move_group` may re-plan up to 3 times per goal (OMPL is randomized).
- **Joint goals beyond the limits are clamped, not rejected,** by MoveIt
  (`Assuming maximum bounds instead`). Independently, commands from any source
  are clamped to the URDF limits before reaching the hardware: by the controller
  manager (`enforce_command_limits: true`) in simulation, by `so101_hardware`
  on the real arm. The controller still
  reports `SUCCEEDED` for such a trajectory, so test 5 checks positions, not
  the action result.

## Dependencies

`so101_bringup`, `so101_moveit_config`, `launch_testing_ament_cmake`, `rclpy`,
`moveit_msgs`, `control_msgs`, `controller_manager_msgs`, `sensor_msgs`,
`std_msgs`, `trajectory_msgs`, `builtin_interfaces`.
