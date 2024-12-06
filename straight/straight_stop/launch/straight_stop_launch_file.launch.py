from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='straight_stop',
            executable='straightandstop',
            output='screen'),
    ])
