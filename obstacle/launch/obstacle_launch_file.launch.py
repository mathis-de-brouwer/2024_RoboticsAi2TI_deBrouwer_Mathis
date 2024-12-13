from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_avoidance',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            parameters=[{
                'forward_speed': 0.2,
                'turning_speed': 0.5,
                'obstacle_threshold': 0.35,
                'lidar_max_range': 3.5,
                'front_angle_range': 15,
                'side_angle_range': 90,
                'min_turn_angle': 30,
                'max_turn_angle': 60,
                'turn_time_multiplier': 5,
                'side_clearance_threshold': 0.1,
            }],
            output='screen'
        )
    ])
