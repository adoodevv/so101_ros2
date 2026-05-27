# so101_bringup

Convenience scripts for launching the SO-101 stack.

## Contents

```
so101_bringup/
└── scripts/
    └── so101_gazebo_launch.sh    # Gazebo + controllers + RViz
```

## Run

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

- Hardware bringup launch files
- MoveIt demo launch
- Real robot controller loading
