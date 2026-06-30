import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

ALL_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

LIMITS = {
    'shoulder_pan':  (-1.92, 1.92),
    'shoulder_lift': (-1.75, 1.75),
    'elbow_flex':    (-1.69, 1.69),
    'wrist_flex':    (-1.66, 1.66),
    'wrist_roll':    (-2.74, 2.84),
    'gripper':       (-0.17, 1.75),
}


def clamp(name, val):
    lo, hi = LIMITS[name]
    return max(lo, min(hi, val))


class LeaderBridge(Node):
    def __init__(self):
        super().__init__('leader_bridge')

        self.declare_parameter('leader_port', '/dev/ttyACM1')
        self.declare_parameter('leader_id', 'adoodevv_leader')
        self.declare_parameter('rate_hz', 30.0)

        port = self.get_parameter('leader_port').value
        leader_id = self.get_parameter('leader_id').value
        rate = self.get_parameter('rate_hz').value

        cfg = SO101LeaderConfig(port=port, id=leader_id, use_degrees=True)
        self.leader = SO101Leader(cfg)
        self.leader.connect()
        self.get_logger().info(f'Connected to leader on {port}')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        period = 1.0 / rate
        self.timer = self.create_timer(period, self.tick)

    def tick(self):
        action = self.leader.get_action()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [
            clamp(j, math.radians(action.get(f'{j}.pos', 0.0)))
            for j in ALL_JOINTS
        ]
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.leader.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = LeaderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
