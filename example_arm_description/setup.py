import os
from glob import glob
from setuptools import setup

package_name = 'example_arm_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install marker file for ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install URDF/xacro files
        (os.path.join('share', package_name, 'description'),
            glob('description/*.xacro') + glob('description/*.urdf')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install rviz config files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayhan',
    maintainer_email='ayhan@todo.com',
    description='Example robotic arm URDF for learning ROS2 visualization',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tcp_monitor = example_arm_description.tcp_monitor:main',
        ],
    },
)
