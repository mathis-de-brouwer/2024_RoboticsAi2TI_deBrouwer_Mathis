from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_mediapipe_mathisdebrouwer_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'opencv-python', 'mediapipe'],
    zip_safe=True,
    maintainer='ththis',
    maintainer_email='mathis.debrouwer@gmail.com',
    description='lidar_mediapipe_mathisdebrouwer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_detector = lidar_mediapipe_mathisdebrouwer_pkg.lidar_mediapipe_mathisdebrouwer:main_hand_detector',
            'obstacle_avoidance = lidar_mediapipe_mathisdebrouwer_pkg.obstacle_avoidance:main',
            'robot_controller = lidar_mediapipe_mathisdebrouwer_pkg.robot_controller:main',
        ],
    },
)
