# so101_teleop_bridge

Bridge between a physical SO-101 **leader arm** (via [LeRobot](https://github.com/huggingface/lerobot)) and ROS 2. Reads joint positions from the leader over USB and publishes them on `/joint_states`, so you can drive the robot model in RViz (or any node that consumes joint states).

## What it does

The `leader_bridge` node:

1. Connects to the leader arm on a serial port (Feetech servos, same stack as LeRobot)
2. Polls present joint positions at a fixed rate (default 30 Hz)
3. Converts body joints from degrees to radians
4. Publishes `sensor_msgs/JointState` on `/joint_states` with joint names matching the SO-101 URDF

```
Physical leader arm  →  leader_bridge  →  /joint_states  →  robot_state_publisher  →  RViz
```

This package does **not** run Gazebo, MoveIt, or a follower arm. It only exposes leader joint positions to ROS 2.

## Prerequisites

- ROS 2 Jazzy workspace built with `so101_description` (for RViz)
- [LeRobot](https://github.com/huggingface/lerobot) installed in a Python environment (e.g. conda env `lerobot`) with the SO-101 leader teleoperator support
- Leader arm plugged in via USB (typically `/dev/ttyACM0` or `/dev/ttyACM1`)
- Leader calibration file for your `leader_id` (created with LeRobot’s SO leader tools)

**Important:** Activate the `lerobot` conda env before running the bridge. The node uses LeRobot’s Python API, which is not available in the system ROS Python. The installed launcher script calls `python3` from your active environment.

## Build

From your workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select so101_teleop_bridge
source install/setup.bash
```

## Usage

### 1. RViz (terminal 1)

Start the robot model and RViz. Disable the joint slider GUI so it does not publish competing `/joint_states` messages:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_description robot_state_publisher.launch.py jsp_gui:=false
```

### 2. Leader bridge (terminal 2)

```bash
conda activate lerobot
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run so101_teleop_bridge leader_bridge --ros-args \
  -p leader_port:=/dev/ttyACM0 \
  -p leader_id:=adoodevv_leader
```

Move the physical leader arm; the RViz model should follow.

On first connect, LeRobot may prompt for calibration. Press **Enter** to use the existing calibration file for your `leader_id`, or type **`c`** and press Enter to run a new calibration.

### Find the serial port

```bash
ls /dev/ttyACM*
```

Use the device that appears when the leader controller is connected.

## Parameters

| Parameter     | Default           | Description                                      |
|---------------|-------------------|--------------------------------------------------|
| `leader_port` | `/dev/ttyACM1`    | USB serial port for the leader arm               |
| `leader_id`   | `adoodevv_leader` | LeRobot calibration ID (must match your cal file)|
| `rate_hz`     | `30.0`            | Joint state publish rate                         |

Example with a custom rate:

```bash
ros2 run so101_teleop_bridge leader_bridge --ros-args \
  -p leader_port:=/dev/ttyACM0 \
  -p leader_id:=my_leader \
  -p rate_hz:=50.0
```

## Published topic

| Topic           | Type                    | Description              |
|-----------------|-------------------------|--------------------------|
| `/joint_states` | `sensor_msgs/JointState`| Leader joint positions   |

Joint names: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`.

## Troubleshooting

**`ModuleNotFoundError: No module named 'lerobot'`**  
Activate the `lerobot` conda env (or whichever env has LeRobot installed) before `ros2 run`.

**`ConnectionError: There is no status packet!`**  
Usually a transient USB/serial glitch or a loose cable. Check the USB connection and port. If it happens repeatedly, try another USB port or cable, and ensure nothing else has the serial port open.

**RViz model does not move**  
Confirm `leader_bridge` is running and publishing:

```bash
ros2 topic echo /joint_states
```

Make sure `jsp_gui:=false` when launching RViz so the joint slider GUI is not also publishing `/joint_states`.

**Gripper looks wrong in RViz**  
Body joints are converted from degrees to radians. The gripper is reported by LeRobot on a 0–100 scale; the bridge currently applies the same conversion as the arm joints, so gripper motion in RViz may not match the physical gripper exactly.

## Dependencies

- ROS 2: `rclpy`, `sensor_msgs`
- Python: [LeRobot](https://github.com/huggingface/lerobot) (`lerobot.teleoperators.so_leader`)
