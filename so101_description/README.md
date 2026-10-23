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

## Mounting and name prefixes

`so101_robot.urdf.xacro` defines the `so101_robot` macro:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_gazebo` | — | Gazebo collision geometry; with `parent=world`, no `world` link is created |
| `prefix` | `""` | Prepended to every link, joint, transmission, actuator and material name |
| `parent` | `world` | Link that `${prefix}base_link` is attached to |
| `xyz` / `rpy` | `0 0 0` | Pose of `${prefix}base_link` in `parent` |

- `parent == world` and Gazebo off: creates the `world` link plus `${prefix}world_joint` (the default, identical to earlier releases).
- `parent == world` and Gazebo on: no joint; `base_link` is pinned to the Gazebo world.
- Any other `parent`: creates `${prefix}mount_joint` from `parent` to `${prefix}base_link`; the caller provides the parent link and no `world` link is created.

To mount the arm on another robot, include the macro from your own xacro:

```xml
<xacro:include filename="$(find so101_description)/urdf/so101_robot.urdf.xacro"/>
<xacro:so101_robot use_gazebo="false" prefix="left_" parent="mount_link" xyz="0 0.2 0" rpy="0 0 0"/>
```

The top-level `robots/so101.urdf.xacro` exposes the same values as xacro args (`prefix`, `parent`, `xyz`, `rpy`). When `parent` is not `world` it also declares an empty `parent` link as the tree root, so the file still produces a standalone URDF:

```bash
xacro $(ros2 pkg prefix so101_description)/share/so101_description/urdf/robots/so101.urdf.xacro \
  prefix:=left_ parent:=mount_link use_fake_hardware:=true > /tmp/so101_left.urdf
check_urdf /tmp/so101_left.urdf
```

> **MoveIt and controllers assume an empty prefix.** `so101_moveit_config` (SRDF, `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`, `ros2_controllers.yaml`, `initial_positions.yaml`) uses unprefixed joint and link names. A prefixed description is valid URDF and works with `robot_state_publisher`, but MoveIt and the controller configs must be adapted by the integrator.

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
