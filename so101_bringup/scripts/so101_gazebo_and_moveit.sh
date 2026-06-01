#!/bin/bash
# Launch Gazebo simulation with MoveIt 2 for the SO-101 arm.
# Same pattern as mycobot_ros2: Gazebo and MoveIt run as separate processes.

cleanup() {
    echo "Cleaning up..."
    sleep 2.0
    pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|moveit"
}

trap 'cleanup' SIGINT SIGTERM

if [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
    source "${HOME}/ros2_ws/install/setup.bash"
fi

echo "Launching Gazebo simulation..."
ros2 launch so101_gazebo so101.gazebo.launch.py \
    load_controllers:=true \
    world_file:=pick_and_place.world \
    use_camera:=true \
    use_rviz:=false \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    spawn_delay:=2.0 \
    controller_load_delay:=5.0 \
    x:=0.0 \
    y:=0.0 \
    z:=0.0 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

sleep 15

echo "Launching MoveIt + RViz..."
ros2 launch so101_moveit_config move_group.launch.py \
    use_camera:=true \
    use_gazebo:=true \
    use_sim_time:=true \
    use_rviz:=true &

echo "Adjusting Gazebo camera..."
gz service -s /gui/move_to/pose \
    --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "pose: {position: {x: 1.36, y: -0.58, z: 0.95} orientation: {x: -0.26, y: 0.1, z: 0.89, w: 0.35}}"

wait
