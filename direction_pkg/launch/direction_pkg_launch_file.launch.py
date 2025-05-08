from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='direction_pkg',
            executable='direction',
            name='direction_publisher',
            parameters=[{
                'model_path': os.path.join(os.environ['HOME'], 'models/path_segmentation.pth'),
                'min_safe_distance': 1.0,
                'max_steering_angle': 0.8,
                'turn_around_threshold': 0.1,
                'path_class_id': 1
            }],
            output='screen'
        )
    ])