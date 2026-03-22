"""
Aggregate data and monitor system activity.

Mock Features:

Subscribe to Everything
    Topics: /cluster_command, /system_status, /reservoir_level, /detected_clusters.

Write Logs to CSV
    Save messages with timestamps to ~/harvesting_logs/log.csv.

Event Count Summary
    Every 10s, print total detections, total clusters harvested.

Error Detector
    If no messages received on /system_status for >10s, print a warning.

"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import csv, os, time


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.get_logger().info('Logger node initialized.')

        # Subscriptions
        self.create_subscription(String, 'system_status', self.status_cb, 10)
        self.create_subscription(String, 'cluster_command', self.cluster_cb, 10)
        self.create_subscription(String, 'detected_clusters', self.detection_cb, 10)
        self.create_subscription(Float32, 'reservoir_level', self.reservoir_cb, 10)

        # Metrics
        self.status = "IDLE"
        self.total_clusters = 0
        self.total_detections = 0
        self.last_reservoir = 0.0

        # CSV setup
        os.makedirs(os.path.expanduser("~/harvesting_logs"), exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.expanduser(f"~/harvesting_logs/log_{timestamp}.csv")
        with open(self.log_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "topic", "message"])

        # Summary timer
        self.create_timer(10.0, self.print_summary)

    # ---------- Topic callbacks ----------

    def status_cb(self, msg: String):
        self.status = msg.data
        self.write_log("system_status", msg.data)

    def cluster_cb(self, msg: String):
        if "NEW_CLUSTER_BEGIN" in msg.data:
            self.total_clusters += 1
        self.write_log("cluster_command", msg.data)

    def detection_cb(self, msg: String):
        self.total_detections += 1
        self.write_log("detected_clusters", msg.data)

    def reservoir_cb(self, msg: Float32):
        self.last_reservoir = msg.data
        self.write_log("reservoir_level", f"{msg.data:.1f}")

    # ---------- Helpers ----------

    def write_log(self, topic: str, message: str):
        ts = time.strftime("%H:%M:%S")
        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow([ts, topic, message])

    def print_summary(self):
        self.get_logger().info(
            f"Summary → clusters:{self.total_clusters} "
            f"detections:{self.total_detections} "
            f"reservoir:{self.last_reservoir:.1f}% status:{self.status}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()