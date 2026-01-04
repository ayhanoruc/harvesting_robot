import os
from glob import glob
from setuptools import setup

package_name = 'robot_arm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='4-DOF robotic arm simulation for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'write_pos = robot_arm.write_pos:main',
            'landmark_publisher = robot_arm.landmark_publisher:main',
        ],
    },
)
