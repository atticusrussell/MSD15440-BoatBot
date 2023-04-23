# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    velocity_goals = PathJoinSubstitution(
        [FindPackageShare("craft_bringup"), "config", "test_goal_publishers_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="my_r2c_test_nodes",
                executable="publisher_forward_velocity_controller",
                name="publisher_forward_velocity_controller",
                parameters=[velocity_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )
