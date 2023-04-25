import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class TwistToJointController(Node):

    def __init__(self):
        super().__init__('twist_to_joint_controller')

        # Configure joint mappings
        # Replace with your robot's specific joint names and mapping functions
        self.joint_names = ['joint1', 'joint2']

        # Subscribe to cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create a joint state publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

    def cmd_vel_callback(self, msg):
        # Calculate joint velocities based on the received Twist message
        # Replace with your robot's specific kinematic mappings
        joint_velocities = [msg.linear.x, msg.angular.z]

        # Publish joint velocities as JointState messages
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.velocity = joint_velocities
        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    twist_to_joint_controller = TwistToJointController()
    rclpy.spin(twist_to_joint_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
