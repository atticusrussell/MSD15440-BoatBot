# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Denis Štogl, Lovro Ivanov
# Adapted to velocity controller by: Atticus Russell

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class PublisherForwardVelocity(Node):
    def __init__(self):
        super().__init__("publisher_forward_velocity_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "forward_velocity_controller")
        # self.declare_parameter("publish_topic", "/velocity_commands")
        self.declare_parameter("wait_sec_between_publish", 5)
        self.declare_parameter("goal_names", ["vel1", "vel2"])

        # Read parameters
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        # publish_topic = self.get_parameter("publish_topic").value
        controller_name = self.get_parameter("controller_name").value

        # Read all velocitys from parameters
        self.goals = []
        for name in goal_names:
            self.declare_parameter(name)
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = [float(value) for value in goal]
            self.goals.append(float_goal)

        publish_topic = "/" + controller_name + "/" + "commands"

        self.get_logger().info(
            f'Publishing {len(goal_names)} goals on topic "{publish_topic}"\
              every {wait_sec_between_publish} s'
        )

        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.goals[self.i]
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.publisher_.publish(msg)
        self.i += 1
        self.i %= len(self.goals)


def main(args=None):
    rclpy.init(args=args)

    publisher_forward_velocity = PublisherForwardVelocity()

    rclpy.spin(publisher_forward_velocity)
    publisher_forward_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
