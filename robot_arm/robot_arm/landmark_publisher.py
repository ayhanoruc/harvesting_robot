#!/usr/bin/env python3
"""
Landmark Publisher Node

Reads environment_config.yaml and:
1. Publishes static TF frames for all landmarks (clusters, reservoir, explore points)
2. Publishes CollisionObjects to /planning_scene for MoveIt

Usage:
    ros2 run robot_arm landmark_publisher --ros-args -p config_file:=/path/to/environment_config.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

import yaml
import os

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class LandmarkPublisher(Node):
    """Publishes TF frames and collision objects from environment config."""

    def __init__(self):
        super().__init__('landmark_publisher')

        # Parameter for config file path
        self.declare_parameter('config_file', '')

        config_file = self.get_parameter('config_file').value
        if not config_file:
            # Default path
            from ament_index_python.packages import get_package_share_directory
            pkg_path = get_package_share_directory('robot_arm')
            config_file = os.path.join(pkg_path, 'config', 'environment_config.yaml')

        self.get_logger().info(f"Loading config from: {config_file}")

        # Load config
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return

        # Static TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Planning scene publisher (latched/transient_local for MoveIt)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', qos)

        # Publish TF frames
        self.publish_landmark_transforms()

        # Publish collision objects (after small delay to ensure MoveIt is ready)
        self._collision_timer = self.create_timer(2.0, self.publish_collision_objects_once)

        self.get_logger().info("Landmark publisher initialized")

    def publish_landmark_transforms(self):
        """Publish static TF frames for all landmarks."""
        transforms = []
        frame_id = self.config['environment']['frame_id']

        # Landmarks (reservoir, explore points)
        landmarks = self.config.get('landmarks', {})
        for name, data in landmarks.items():
            t = self.create_transform(frame_id, name, data['position'])
            transforms.append(t)
            self.get_logger().info(f"TF: {frame_id} -> {name} at {data['position']}")

        # Clusters
        clusters = self.config.get('clusters', {})
        for name, data in clusters.items():
            t = self.create_transform(frame_id, name, data['position'])
            transforms.append(t)
            self.get_logger().info(f"TF: {frame_id} -> {name} at {data['position']}")

        # Broadcast all transforms
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
            self.get_logger().info(f"Published {len(transforms)} static TF frames")

    def create_transform(self, parent_frame: str, child_frame: str, position: list) -> TransformStamped:
        """Create a TransformStamped message."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])

        # Identity rotation (landmarks are just points)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        return t

    def publish_collision_objects_once(self):
        """Publish collision objects to MoveIt planning scene (one-shot)."""
        collision_objects = self.config.get('collision_objects', {})

        if not collision_objects:
            self.get_logger().info("No collision objects defined in config")
            return

        scene = PlanningScene()
        scene.is_diff = True

        for name, data in collision_objects.items():
            obj = CollisionObject()
            obj.header.frame_id = data.get('frame_id', 'world')
            obj.header.stamp = self.get_clock().now().to_msg()
            obj.id = name
            obj.operation = CollisionObject.ADD

            # Create primitive shape
            primitive = SolidPrimitive()
            obj_type = data.get('type', 'box')

            if obj_type == 'box':
                primitive.type = SolidPrimitive.BOX
                dims = data.get('dimensions', [0.1, 0.1, 0.1])
                primitive.dimensions = [float(dims[0]), float(dims[1]), float(dims[2])]
            elif obj_type == 'cylinder':
                primitive.type = SolidPrimitive.CYLINDER
                primitive.dimensions = [
                    float(data.get('height', 0.1)),
                    float(data.get('radius', 0.05))
                ]
            elif obj_type == 'sphere':
                primitive.type = SolidPrimitive.SPHERE
                primitive.dimensions = [float(data.get('radius', 0.05))]

            obj.primitives.append(primitive)

            # Set pose
            pose = Pose()
            pos = data.get('position', [0.0, 0.0, 0.0])
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.w = 1.0
            obj.primitive_poses.append(pose)

            scene.world.collision_objects.append(obj)
            self.get_logger().info(f"Collision object: {name} ({obj_type}) at {pos}")

        # Publish scene
        self.scene_pub.publish(scene)
        self.get_logger().info(f"Published {len(scene.world.collision_objects)} collision objects to /planning_scene")

        # Cancel timer (only publish once)
        self.destroy_timer(self._collision_timer)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
