# so101_gazebo

Gazebo Harmonic simulation for the SO-101 arm using `ros_gz_sim` and `gz_ros2_control`.

## Contents

```
so101_gazebo/
├── config/
│   └── ros_gz_bridge.yaml          # Clock bridge (camera bridges optional)
├── launch/
│   └── so101.gazebo.launch.py      # Full simulation launch
└── worlds/
    ├── empty.world
    └── pick_and_place.world        # Fuel-hosted Sun, ground plane, cardboard box
```

## Run

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_gazebo so101.gazebo.launch.py
```

This starts:

1. `robot_state_publisher` with Gazebo-enabled URDF
2. Gazebo with the pick-and-place world
3. ros2_control controller loading (after delay)
4. Robot spawn into the world
5. ROS–Gazebo clock bridge
6. RViz (optional)

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world_file` | `pick_and_place.world` | World file in `worlds/` |
| `load_controllers` | `true` | Load ros2_control controllers |
| `use_rviz` | `true` | Start RViz |
| `use_sim_time` | `true` | Use Gazebo clock |
| `use_camera` | `false` | Enable camera image bridges |
| `spawn_delay` | `2.0` | Seconds before spawning the robot |
| `controller_load_delay` | `5.0` | Seconds before loading controllers |
| `x`, `y`, `z` | `0.0`, `0.0`, `0.05` | Robot spawn position |
| `roll`, `pitch`, `yaw` | `0.0` | Robot spawn orientation |

### Examples

Empty world, no RViz:

```bash
ros2 launch so101_gazebo so101.gazebo.launch.py world_file:=empty.world use_rviz:=false
```

Slower startup on a heavy machine:

```bash
ros2 launch so101_gazebo so101.gazebo.launch.py spawn_delay:=3.0 controller_load_delay:=8.0
```

## Worlds

**pick_and_place.world** — Uses [Gazebo Fuel](https://app.gazebosim.org) URIs for the Sun, ground plane, and cardboard box. Network access is required on first load; models are cached locally afterward.

**empty.world** — Minimal world with a local ground plane.

## Control after launch

Wait until controllers are loaded (`ros2 control list_controllers` shows all three active), then see [so101_moveit_config/README.md](../so101_moveit_config/README.md) for arm and gripper commands.

## Dependencies

`ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `ros2_control`, `ros2_controllers`, `so101_description`, `so101_moveit_config`
