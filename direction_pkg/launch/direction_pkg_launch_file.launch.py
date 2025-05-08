from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='direction_pkg',
            executable='direction_node',
            name='direction_publisher',
            output='screen',
            parameters=[
                {'model_path': 'path_to_your_model.pth'},
                {'min_safe_distance': 0.5},
                {'max_steering_angle': 0.8},
            ]
        )
    ])