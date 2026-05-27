# so101_system_tests

System-level test scripts for the SO-101 stack.

## Status

This package is a placeholder. Automated arm and gripper loop tests are planned but not yet implemented.

## Planned contents

- Arm trajectory loop demo node
- Gripper open/close cycle test
- Integration tests run against a live Gazebo simulation

## Run

Nothing to run yet. Once scripts are added, they will be installed to:

```
share/so101_system_tests/scripts/
```

Example usage (future):

```bash
source ~/ros2_ws/install/setup.bash
# With Gazebo already running:
ros2 run so101_system_tests arm_gripper_loop.py
```

## Dependencies

Will depend on `so101_gazebo`, `so101_moveit_config`, and `control_msgs` once implemented.
