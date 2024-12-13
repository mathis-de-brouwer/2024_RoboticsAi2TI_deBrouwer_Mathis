from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handtrack_this_pkg',
            executable='handtrack_this',
            output='screen'),
        Node(
            package='handtrack_this_pkg',
            executable='gesture_control',
            output='screen'),
    ])
