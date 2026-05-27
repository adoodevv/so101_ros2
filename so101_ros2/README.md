# so101_ros2

Meta-package for the SO-101 ROS 2 stack. Installing or building this package pulls in all sub-packages via `exec_depend` entries.

## Included packages

- `so101_description`
- `so101_moveit_config`
- `so101_gazebo`
- `so101_bringup`
- `so101_system_tests`

## Build

Build the entire workspace from the repository root:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

There is nothing to run directly from this package. See the [root README](../README.md) for launch instructions.
