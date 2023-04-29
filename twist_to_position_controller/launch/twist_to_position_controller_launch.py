from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_to_position_controller',
            executable='twist_to_position_node.py',
            name='twist_to_position_controller',
            output='screen'
        ),
    ])
