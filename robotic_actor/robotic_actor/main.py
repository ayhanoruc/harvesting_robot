"""
Handle movement and actuation requests, report progress.

MOCK FEATURES:

MoveToCluster Service
    Provides /move_to_cluster service → prints mock travel to coordinates.

Action Server for BollHarvest
    /harvest_boll action with feedback: "moving", "gripping", "transferring", "done".

Position Feedback Publisher
    Publishes /robot_position topic (random coordinates).

End-Effector Simulator
    Subscribes to /grip_command topic and logs “gripper activated”.

Reservoir Monitor
    Publishes /reservoir_level topic (fake percentage 0–100%).

"""


import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger  # used for move_to_cluster
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import random
import time
import math


class RoboticActorNode(Node):
    def __init__(self):
        super().__init__('robotic_actor_node')
        self.get_logger().info('Robotic Actor node initialized.')

        # Service: respond to "move to cluster" commands
        self.move_service = self.create_service(Trigger, 'move_to_cluster', self.move_callback)

        # Publisher: reservoir fill level
        self.reservoir_pub = self.create_publisher(Float32, 'reservoir_level', 10)
        self.timer = self.create_timer(5.0, self.publish_reservoir_status)

        # Publisher: joint states for four-bar linkage
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz update

        # Simulated internal state
        self.position = (0.0, 0.0, 0.0)
        self.reservoir_level = 0.0

        # Four-bar linkage joint states
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]

    def move_callback(self, request, response):
        """Simulate movement and gripping routine."""
        x, y, z = [random.uniform(0.0, 1.0) for _ in range(3)]
        self.position = (x, y, z)

        self.get_logger().info(f"Moving to target coordinates {self.position}")
        time.sleep(1.5)  # simulate motion delay

        success = random.choice([True, True, False])  # mostly succeed
        response.success = success
        response.message = "Move completed" if success else "Move failed"
        self.get_logger().info(f"Move result: {response.message}")

        if success:
            self.reservoir_level = min(100.0, self.reservoir_level + random.uniform(5.0, 15.0))

        return response

    def publish_reservoir_status(self):
        msg = Float32()
        msg.data = self.reservoir_level
        self.reservoir_pub.publish(msg)
        self.get_logger().info(f"Reservoir level: {msg.data:.1f}%")

    def publish_joint_states(self):
        """Publish random joint states for the four-bar linkage."""
        # Update joint1 (crank) with random angle within limits [-1.57, 1.57]
        self.joint_positions[0] = random.uniform(-1.57, 1.57)

        # For a real four-bar linkage, joints 2-4 would be kinematically constrained
        # For now, set them to random values for visualization
        self.joint_positions[1] = random.uniform(-1.57, 1.57)
        self.joint_positions[2] = random.uniform(-3.14, 3.14)
        self.joint_positions[3] = random.uniform(-3.14, 3.14)

        # Create and publish JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions

        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = RoboticActorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()