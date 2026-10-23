# so101_hardware

ros2_control hardware interface for the **SO-101 follower arm**: six Feetech STS3215
servos on one serial bus, calibrated with [LeRobot](https://github.com/huggingface/lerobot).
With it, the same controllers, MoveIt config and system tests used in simulation drive
the real arm.

| Piece | What it is |
|-------|------------|
| `so101_hardware/SO101SystemHardware` | ros2_control `SystemInterface` plugin |
| `so101_hardware_check` | Read-only diagnostic: ping, calibration check, live joint angles |
| `so101_servo_sim` | Six simulated STS3215 servos on a pseudo-terminal, for testing without the arm |
| `config/example_follower_calibration.json` | Example LeRobot calibration (used by the tests) |

## How it maps LeRobot to ROS

LeRobot writes each servo's `homing_offset` and `[range_min, range_max]` into the servo's
EEPROM, so the position read from the servo is already calibrated. The plugin **reads and
verifies** those registers against your calibration file and refuses to start if they
differ. It never rewrites your calibration.

| Joint | Mapping |
|-------|---------|
| shoulder_pan … wrist_roll | `rad = (ticks − mid) · 2π / 4095`, with `mid = (range_min + range_max) / 2`. This is LeRobot's degrees mode in radians, matching the SO-101 "new calibration" URDF (zero at mid-range). |
| gripper | linear: `range_min` (closed, LeRobot 0 %) → −0.1745 rad, `range_max` (open, 100 %) → 1.7453 rad, the URDF gripper limits |

`drive_mode: 1` reverses a joint.

**Limits.** The plugin clamps every command to the URDF limits intersected with the
calibrated range. Arm joints are held 0.03 rad inside the URDF limits: with LeRobot's soft
gains a loaded servo settles up to ~10 ticks short of its goal, and MoveIt refuses to plan
from a start state even one tick past a limit. The gripper keeps its full travel. The real launch turns off the controller manager's
`enforce_command_limits`: its limiter stops every controller as soon as a joint *reads*
past a URDF limit, and the real arm's folded rest pose is a few degrees past
`shoulder_lift`'s.

**Lifecycle:**
- **configure:** opens the port and checks every servo (ping, model, calibration). It then applies LeRobot's connect-time settings: PID 16/0/32, acceleration 254, return delay 0, and 50 % torque/current limits on the gripper. Only registers that differ are written; on an arm already used with LeRobot, nothing is.
- **activate:** reads the pose, sets the goal to it, *then* enables torque. The arm holds where it is and does not jump. A joint resting up to 0.1 rad outside its limits is moved to the limit (with a warning; the folded `shoulder_lift` does this, about 2°). Further out, activation is refused and the launch stops: move the joint inside by hand and relaunch.
- **deactivate:** torque off, so the arm goes limp. Support it if it is raised.
- **communication error:** after `max_consecutive_read_errors` failed reads, the hardware goes to error and torque is left as it is, so the servos keep holding their last goal.

State interfaces per joint: `position`, `velocity` (rad/s), `load` (fraction of max torque,
−1..1; not a calibrated effort), `temperature` (°C), `voltage` (V). `position` and
`velocity` are on `/joint_states`; all of them are on `/dynamic_joint_states`.

## First run on your arm

The follower must already be calibrated in LeRobot, e.g. as `my_follower`:
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json`.

**Ports.** `/dev/ttyACM*` numbers follow plug-in order: plug the follower first and it is
`/dev/ttyACM0`, the leader then `/dev/ttyACM1`. To make this independent of plug order,
use the stable path under `/dev/serial/by-id/` (`ls -l /dev/serial/by-id/`) as `port:=`.

**1. Check the servos (read-only, nothing moves).** Power the arm and run:
```bash
ros2 run so101_hardware so101_hardware_check --port /dev/ttyACM0 \
  --calibration ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json
```
You want six `ok` / `matches` rows, `ok` (or "slightly outside") in the `limits` column,
and "All servos present and calibrated". Add `--watch` to keep printing angles while you
move joints by hand.

- **`MISMATCH`**: the servos hold a different calibration than the file. LeRobot writes
  the calibration of whichever id it last connected with into the servos, so this usually
  means the arm was last used under another id. The tool searches the file's directory
  and prints the file that matches; use it (`follower_id:=<that name>` in the launch),
  or connect once with LeRobot under the id you want so it writes that file to the servos.
- **`OUTSIDE`**: the joint is too far past its limits for the plugin to enable torque
  (for example a gripper opened past its calibrated range). Move it in by hand.

**2. Check directions in RViz (torque off).**
```bash
ros2 launch so101_bringup so101_real_moveit.launch.py read_only:=true
```
The arm stays limp. Move each joint by hand, and the RViz model must follow **in the same
direction**. With every joint near the middle of its travel, the model should look
like `home` (all zeros). Open and close the gripper by hand: the model's jaw must open and close with it.
If a joint moves the wrong way, stop and report it; do not continue to step 3.

**3. First powered run.** Clear the workspace, keep a hand near the power switch, and
start with the arm resting in a safe pose.
```bash
ros2 launch so101_bringup so101_real_moveit.launch.py
```
The arm should stiffen in place without moving. In RViz (MotionPlanning panel) set
**Velocity/Accel. Scaling to 0.1**, pick group `gripper`, goal `open`, and click **Plan & Execute**.
Then group `arm`, goal `home`, then `ready`.

**4. System tests on the real arm** (moves through home/ready, the gripper, and
shoulder_pan to its limit at 30 % speed):
```bash
cd ~/ros2_ws
SO101_TEST_BACKEND=real SO101_PORT=/dev/ttyACM0 \
SO101_CALIBRATION_FILE=$HOME/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json \
  launch_test src/so101_ros2/so101_system_tests/test/test_moveit_system.launch.py
```

## Without the arm

```bash
ros2 run so101_hardware so101_servo_sim \
  --calibration $(ros2 pkg prefix so101_hardware)/share/so101_hardware/config/example_follower_calibration.json \
  --link /tmp/so101_sim_port
ros2 launch so101_bringup so101_real_moveit.launch.py port:=/tmp/so101_sim_port \
  calibration_file:=$(ros2 pkg prefix so101_hardware)/share/so101_hardware/config/example_follower_calibration.json
```
`--missing 4` makes servo 4 silent, for testing the error handling. The
`so101_system_tests` suite runs against this simulator on every `colcon test`.

## Parameters

Set in `so101_description/urdf/control/so101_ros2_control.urdf.xacro` (`use_real_hardware:=true`).

| Hardware parameter | Default | |
|--------------------|---------|--|
| `port` | — | serial device (required) |
| `calibration_file` | — | LeRobot calibration JSON (required) |
| `baud_rate` | `1000000` | |
| `reply_timeout_ms` | `10` | per-servo reply timeout |
| `max_consecutive_read_errors` | `5` | then `read()` returns ERROR |
| `configure_servos` | `true` | apply LeRobot's servo settings on configure |
| `disable_torque_on_deactivate` | `true` | |
| `read_only` | `false` | never enable torque |
| `p_coefficient`, `i_coefficient`, `d_coefficient` | `16`, `0`, `32` | servo position PID (LeRobot defaults) |

Joint parameters: `motor` (calibration key), `mapping` (`centered` or `linear`),
`max_torque_limit`, `protection_current`, `overload_torque`.

## Tests

`colcon test --packages-select so101_hardware` runs gtest suites for the protocol (packet
bytes checked against the Feetech spec), the bus (fake transport: timeouts, garbage,
wrong id, bad checksum, sync read/write) and the calibration mapping (with real
`my_follower` values).
