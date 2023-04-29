#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TwistToPositionControllerNode(Node):
    def __init__(self):
        super().__init__('twist_to_position_controller')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

        self.publisher = self.create_publisher(Float64MultiArray, 'forward_position_controller/commands', 10)

    def twist_callback(self, msg):
        angular_z = msg.angular.z
        self.publish_position(angular_z)

    def publish_position(self, angular_z):
        position_msg = Float64MultiArray()
        position_msg.data = [angular_z]
        self.publisher.publish(position_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_to_position_controller_node = TwistToPositionControllerNode()
    rclpy.spin(twist_to_position_controller_node)
    twist_to_position_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
