"""
Simulate ML inference and object detection.

MOCK FEATURES:

Fake Detection Stream
    Publishes /detected_clusters (std_msgs/String or custom Cluster msg) every 2s.
    Example data: "Cluster_12 at (0.2, 0.5, 0.1)".

Ripeness Classifier Service
    Exposes /classify_ripeness (std_srvs/srv/SetBool or custom) → returns random score.

Bounding Box Publisher
    Publishes /boll_positions topic with mock 3D coordinates for each boll.

Frame Feedback
    Subscribes to /system_status; pauses detection when the orchestrator says "PRESSING".
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
import random


class VisionMLNode(Node):
    def __init__(self):
        super().__init__('vision_ml_node')
        self.get_logger().info('Vision ML node initialized.')

        # Publisher: fake detection messages
        self.detect_pub = self.create_publisher(String, 'detected_clusters', 10)
        self.timer = self.create_timer(3.0, self.publish_fake_detection)

        # Service: ripeness classification
        self.srv = self.create_service(SetBool, 'classify_ripeness', self.classify_ripeness_cb)

        # Subscriber: listen to orchestrator status
        self.create_subscription(String, 'system_status', self.status_callback, 10)
        self.paused = False

    def publish_fake_detection(self):
        if self.paused:
            return
        cluster_id = random.randint(1, 50)
        msg = String()
        msg.data = f"Detected Cluster_{cluster_id} at (x={random.random():.2f}, y={random.random():.2f})"
        self.detect_pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def classify_ripeness_cb(self, request, response):
        """Mock service that returns a random ripeness boolean."""
        is_ripe = random.choice([True, False])
        response.success = is_ripe
        response.message = f"Ripeness={'RIPE' if is_ripe else 'UNRIPE'}"
        self.get_logger().info(f"Ripeness query -> {response.message}")
        return response

    def status_callback(self, msg: String):
        """Pause publishing while system is pressing."""
        if msg.data == "PRESSING":
            self.paused = True
        elif msg.data == "IDLE":
            self.paused = False


def main(args=None):
    rclpy.init(args=args)
    node = VisionMLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()