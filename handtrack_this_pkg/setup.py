from setuptools import find_packages, setup

package_name = 'handtrack_this_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),('share/handtrack_this_pkg/launch', ['launch/handtrack_this_pkg_launch_file.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'opencv-python', 'mediapipe'],
    zip_safe=True,
    maintainer='ththis',
    maintainer_email='mathis.debrouwer@gmail.com',
    description='handtrackiingggg',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handtrack_this = handtrack_this_pkg.handtrack_this:main',
            'gesture_control = handtrack_this_pkg.gesture_control:main',
        ],
    },
)
