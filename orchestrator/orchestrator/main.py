"""
Manage the sequence of operations and talk to other nodes through topics, services, and actions

Mock Features:

Cluster Manager
    Publishes /cluster_command (std_msgs/String) messages like "NEW_CLUSTER_BEGIN" and "CLUSTER_DONE".

Harvest Sequencer
    Calls a service /robotic_actor/move_to_cluster with mock coordinates.

Ripeness Request
    Sends a service request /vision_ml/classify_ripeness → gets a mock float score (0–1).

Task Status Broadcast
    Publishes /system_status topic with states: IDLE, HARVESTING, PRESSING.

Cluster Counter
    Maintains an internal count of processed clusters and publishes /harvest_summary every 30 seconds.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from example_interfaces.srv import Trigger  # we'll use Trigger for move_to_cluster

import random
import time


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        self.get_logger().info('Orchestrator node initialized.')

        # Publishers
        self.status_pub = self.create_publisher(String, 'system_status', 10)
        self.cluster_pub = self.create_publisher(String, 'cluster_command', 10)

        # Service clients
        self.ripeness_client = self.create_client(SetBool, 'classify_ripeness')
        self.move_client = self.create_client(Trigger, 'move_to_cluster')

        # Timer
        self.timer = self.create_timer(5.0, self.orchestrate_once)

        self.cluster_id = 0
        self.phase = 'IDLE'

    def orchestrate_once(self):
        """Mock orchestration routine."""
        self.cluster_id += 1
        cluster_name = f"Cluster_{self.cluster_id}"
        self.get_logger().info(f"Processing {cluster_name}")

        # Announce cluster begin
        self.publish_status("HARVESTING")
        self.cluster_pub.publish(String(data=f"NEW_CLUSTER_BEGIN {cluster_name}"))

        # Step 1: Ask vision_ml for ripeness
        ripe = self.ask_ripeness()
        if ripe:
            # Step 2: Move to cluster (ask robotic_actor)
            self.move_to_cluster()
        else:
            self.get_logger().info(f"{cluster_name} not ripe. Skipping.")

        # Step 3: Announce cluster done
        self.cluster_pub.publish(String(data=f"CLUSTER_DONE {cluster_name}"))
        self.publish_status("PRESSING")
        time.sleep(2)
        self.publish_status("IDLE")

    def publish_status(self, state: str):
        self.status_pub.publish(String(data=state))
        self.get_logger().info(f"System state -> {state}")

    def ask_ripeness(self) -> bool:
        """Send async ripeness request; handle reply in callback."""
        if not self.ripeness_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Ripeness service unavailable.")
            return False

        req = SetBool.Request()
        req.data = True
        future = self.ripeness_client.call_async(req)
        future.add_done_callback(self.ripeness_response_cb)
        self.get_logger().info("Ripeness request sent.")
        return True  # don't block; keep spinning

    def ripeness_response_cb(self, future):
        try:
            resp = future.result()
            if resp is None:
                self.get_logger().warn("Ripeness service returned no result.")
                return
            self.get_logger().info(
                f"Ripeness response received: {resp.success}, message: {resp.message}"
            )
            if resp.success:
                self.move_to_cluster()
            else:
                self.get_logger().info("Cluster not ripe. Skipping.")
        except Exception as e:
            self.get_logger().warn(f"Ripeness service failed: {e}")


    def move_to_cluster(self):
        """Send async move request and handle response in callback."""
        if not self.move_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Move service unavailable.")
            return

        req = Trigger.Request()
        future = self.move_client.call_async(req)
        future.add_done_callback(self.move_response_cb)
        self.get_logger().info("Move request sent.")
        
    def move_response_cb(self, future):
        try:
            resp = future.result()
            if resp is None:
                self.get_logger().warn("Move service returned no result.")
                return
            self.get_logger().info(
                f"Move actor response received: {resp.success}, message: {resp.message}"
            )
        except Exception as e:
            self.get_logger().warn(f"Move service failed: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()