from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_mediapipe_mathisdebrouwer_pkg',
            executable='hand_detector',
            name='hand_detector'
        ),
        Node(
            package='lidar_mediapipe_mathisdebrouwer_pkg',
            executable='obstacle_avoidance',
            name='obstacle_avoidance'
        ),
        Node(
            package='lidar_mediapipe_mathisdebrouwer_pkg',
            executable='robot_controller',
            name='robot_controller'
        ),
    ]) 