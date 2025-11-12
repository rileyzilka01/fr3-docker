#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class GoToZeroPublisher(Node):
    def __init__(self):
        super().__init__('go_to_zero_pub')
        self.pub = self.create_publisher(JointState, '/NS1/franka_robot_state_broadcaster/desired_joint_states', 10)
        self.send_zero()

    def send_zero(self):
        msg = JointState()
        # 7 Franka joints
        msg.name = [f'joint{i+1}' for i in range(7)]
        msg.position = [0.0] * 7
        self.get_logger().info('Publishing zero joint positions...')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToZeroPublisher()
    rclpy.spin_once(node, timeout_sec=0.1)  # Publish once
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
