from setuptools import find_packages, setup

package_name = 'straight_stop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='ththis',
    maintainer_email='mathis.de.brouwer@student.ehb.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straightandstop = straight_stop.straightandstop:main',
        ],
    },
)
