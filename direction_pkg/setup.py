from setuptools import setup
import os
from glob import glob

package_name = 'direction_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ththis',
    maintainer_email='145472006+mathis-de-brouwer@users.noreply.github.com',
    description='ROS2 package for obstacle avoidance using deep learning',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'direction_node = direction_pkg.direction:main',
        ],
    },
)