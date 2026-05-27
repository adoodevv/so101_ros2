# so101_description

URDF/xacro, meshes, and RViz configuration for the SO-101 arm.

## Contents

```
so101_description/
├── launch/
│   └── robot_state_publisher.launch.py   # RViz + robot_state_publisher
├── meshes/so_arm101/                     # STL mesh files
├── rviz/
│   └── robot_arm_description.rviz
└── urdf/
    ├── robot.urdf                        # Plain URDF (Onshape export)
    ├── so101_robot.urdf.xacro            # Robot body (links, joints, meshes)
    ├── robots/so101.urdf.xacro           # Top-level xacro (includes ros2_control)
    └── control/
        ├── so101_ros2_control.urdf.xacro
        └── gazebo_sim_ros2_control.urdf.xacro
```

## Links

`world`, `base_link`, `shoulder_link`, `upper_arm_link`, `lower_arm_link`, `wrist_link`, `gripper_link`, `gripper_frame_link`, `moving_jaw_so101_v1_link`

## Joints

| Joint | Parent → Child |
|-------|----------------|
| `shoulder_pan` | `base_link` → `shoulder_link` |
| `shoulder_lift` | `shoulder_link` → `upper_arm_link` |
| `elbow_flex` | `upper_arm_link` → `lower_arm_link` |
| `wrist_flex` | `lower_arm_link` → `wrist_link` |
| `wrist_roll` | `wrist_link` → `gripper_link` |
| `gripper` | `gripper_link` → `moving_jaw_so101_v1_link` |

## Run

**RViz visualization** with interactive joint GUI:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_description robot_state_publisher.launch.py
```

Launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Start RViz |
| `jsp_gui` | `true` | Joint state publisher GUI sliders |
| `use_jsp` | `false` | Non-GUI joint state publisher |
| `use_sim_time` | `false` | Use simulation clock |
| `use_gazebo` | `false` | Embed ros2_control + Gazebo plugin in URDF |
| `robot_name` | `so101` | Robot name for controller config path |

When launched from Gazebo, `use_gazebo:=true` is set automatically so the URDF includes the `gz_ros2_control` plugin and box collision geometry for the gripper.

## Dependencies

`robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xacro`
