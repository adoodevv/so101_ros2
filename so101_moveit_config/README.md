# so101_moveit_config

ros2_control controller configuration for the SO-101 arm. Despite the package name, full MoveIt 2 planning (SRDF, `move_group`, OMPL pipelines) is not yet configured — this package currently provides the controller YAML and spawner launch file used by Gazebo simulation.

## Contents

```
so101_moveit_config/
├── config/so101/
│   ├── ros2_controllers_template.yaml   # Template with ${prefix} placeholders
│   └── ros2_controllers.yaml            # Generated at launch time
└── launch/
    └── load_ros2_controllers.launch.py  # Sequential controller spawner
```

## Controllers

| Controller | Type | Interface |
|------------|------|-----------|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes `/joint_states` |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | `/arm_controller/joint_trajectory` |
| `gripper_controller` | `forward_command_controller/ForwardCommandController` | `/gripper_controller/commands` |

### Arm joints

`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`

### Gripper joint

`gripper` — valid range **-0.17** (closed) to **1.75** (open) radians

## Run

This package is not launched on its own. It is included automatically by the Gazebo launch file. To load controllers manually (e.g. after custom spawn):

```bash
ros2 launch so101_moveit_config load_ros2_controllers.launch.py controller_load_delay:=5.0
```

Launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `controller_load_delay` | `5.0` | Seconds before loading the first controller |

## Control examples

**Arm trajectory** (after controllers are active):

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'],
  points: [{positions: [0.3, -0.8, 0.6, -0.3, 0.5], time_from_start: {sec: 3, nanosec: 0}}]
}"
```

**Gripper open / close**:

```bash
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.5]}"
ros2 topic pub --once /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [-0.17]}"
```

**Arm action interface**:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]
  }
}"
```

## Planned

- SRDF with planning groups
- `move_group` launch files
- MoveIt controller configuration (`moveit_controllers.yaml`)
- Kinematics and joint limit overrides
