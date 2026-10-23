# so101_bringup

Launch files and convenience scripts for running the SO-101 stack.

## Contents

```
so101_bringup/
├── launch/
│   ├── so101_gazebo_moveit.launch.py   # Gazebo + controllers + move_group + MoveIt RViz
│   └── so101_fake_moveit.launch.py     # Fake hardware + controllers + move_group, no Gazebo
└── scripts/
    ├── so101_gazebo_launch.sh          # Gazebo + controllers + RViz
    └── so101_gazebo_and_moveit.sh      # Gazebo, then MoveIt, as separate processes
```

## Fake hardware + MoveIt (no Gazebo)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup so101_fake_moveit.launch.py            # headless
ros2 launch so101_bringup so101_fake_moveit.launch.py use_rviz:=true
```

Starts `robot_state_publisher` (URDF with `use_fake_hardware:=true`),
`ros2_control_node` with `so101_moveit_config/config/so101/ros2_controllers.yaml`
backed by `mock_components/GenericSystem`, spawners for `joint_state_broadcaster`,
`arm_controller` and `gripper_controller`, and `move_group`. Uses wall-clock time.

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `false` | Start RViz with the MotionPlanning plugin |
| `robot_name` | `so101` | Config directory under `so101_moveit_config/config` |

`so101_system_tests` runs its end-to-end tests against this launch file.

## Gazebo + MoveIt

```bash
ros2 launch so101_bringup so101_gazebo_moveit.launch.py
```

## Scripts

After building and sourcing the workspace:

```bash
source ~/ros2_ws/install/setup.bash
bash $(ros2 pkg prefix so101_bringup)/share/so101_bringup/scripts/so101_gazebo_launch.sh
```

Or from the source tree during development:

```bash
bash ~/ros2_ws/src/so101_ros2/so101_bringup/scripts/so101_gazebo_launch.sh
```

The script launches Gazebo with the pick-and-place world, ros2_control controllers, and RViz. Press `Ctrl+C` to stop; the script runs a cleanup trap that kills related ROS and Gazebo processes.

Equivalent direct launch:

```bash
ros2 launch so101_gazebo so101.gazebo.launch.py \
    load_controllers:=true \
    world_file:=pick_and_place.world \
    use_rviz:=true \
    use_sim_time:=true
```

## Planned

- Real hardware bringup launch files
