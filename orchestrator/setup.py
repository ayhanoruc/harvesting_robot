from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include YOLO model
        ('share/' + package_name + '/models', glob('models/*')),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayhan',
    maintainer_email='ayhan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'orchestrator_node = orchestrator.main:main',
            'explorer = orchestrator.explorer:main',
            'depth_processor = orchestrator.depth_processor:main',
            'camera_focus = orchestrator.camera_focus:main',
            'mock_yolo_detector = orchestrator.mock_yolo_detector:main',
            'spatial_detection_pipeline = orchestrator.spatial_detection_pipeline:main',
            'real_yolo_detector = orchestrator.real_yolo_detector:main',
            'gripper_controller = orchestrator.gripper_controller:main',
            'harvest_executor = orchestrator.harvest_executor:main',
            'wasd_teleop = orchestrator.wasd_teleop:main',
            'simple_cluster_harvester = orchestrator.simple_cluster_harvester:main',
            'harvest_orchestrator = orchestrator.harvest_orchestrator:main',
            'sim_helpers = orchestrator.sim_helpers:main',
            'arm_teleop = orchestrator.arm_teleop:main',
            'cv_boll_detector = orchestrator.cv_boll_detector:main',
            'cluster_scanner = orchestrator.cluster_scanner:main',
        ],
    },
)
