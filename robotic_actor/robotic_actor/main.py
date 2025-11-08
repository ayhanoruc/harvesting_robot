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
import random
import time


class RoboticActorNode(Node):
    def __init__(self):
        super().__init__('robotic_actor_node')
        self.get_logger().info('Robotic Actor node initialized.')

        # Service: respond to "move to cluster" commands
        self.move_service = self.create_service(Trigger, 'move_to_cluster', self.move_callback)

        # Publisher: reservoir fill level
        self.reservoir_pub = self.create_publisher(Float32, 'reservoir_level', 10)
        self.timer = self.create_timer(5.0, self.publish_reservoir_status)

        # Simulated internal state
        self.position = (0.0, 0.0, 0.0)
        self.reservoir_level = 0.0

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