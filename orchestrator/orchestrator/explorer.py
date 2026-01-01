#!/usr/bin/env python3
"""
Explorer Node - Structured Mini-Sweep for Cluster Detection

Orchestrates the robot arm to perform a systematic scan of the cotton field:
1. Move to explore_start position
2. For each cluster: perform a 3-view mini-sweep (center, left, right)
3. Move to explore_end position

The mini-sweep at each cluster provides multiple viewing angles for robust detection.

Topics Published:
    /explorer/viewpoint_reached (std_msgs/String) - Signals when arm reaches a viewpoint
    /explorer/scan_status (std_msgs/String) - Current scan status (IDLE, SCANNING, COMPLETE)

Services:
    /explorer/start_scan (std_srvs/Trigger) - Triggers the full scan sequence

Parameters:
    config_file - Path to environment_config.yaml
    view_distance - Distance from cluster for viewpoints (default: 0.35m)
    lateral_offset - Y offset for left/right views (default: 0.12m)
    pause_at_viewpoint - Seconds to pause at each viewpoint (default: 1.5s)

Usage:
    ros2 run orchestrator explorer
    ros2 service call /explorer/start_scan std_srvs/srv/Trigger "{}"
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import yaml
import os
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum
import threading


class ScanState(Enum):
    IDLE = "IDLE"
    SCANNING = "SCANNING"
    MOVING = "MOVING"
    CAPTURING = "CAPTURING"
    COMPLETE = "COMPLETE"
    ERROR = "ERROR"


@dataclass
class Viewpoint:
    """Represents a single viewpoint for scanning."""
    name: str
    position: List[float]
    cluster_name: Optional[str] = None
    view_type: str = "center"


class ExplorerNode(Node):
    """Orchestrates structured mini-sweep exploration for cluster detection."""

    def __init__(self):
        super().__init__('explorer')

        # Callback groups for concurrent operations
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('view_distance', 0.35)
        self.declare_parameter('lateral_offset', 0.12)
        self.declare_parameter('pause_at_viewpoint', 1.5)

        # Load config and generate viewpoints
        self.config = self._load_config()
        self.viewpoints = self._generate_viewpoints()

        # State
        self.state = ScanState.IDLE
        self.current_viewpoint_idx = 0
        self.scan_in_progress = False
        self._scan_lock = threading.Lock()

        # Publishers
        self.viewpoint_pub = self.create_publisher(String, '/explorer/viewpoint_reached', 10)
        self.status_pub = self.create_publisher(String, '/explorer/scan_status', 10)

        # Service clients for arm_commander
        self.go_to_pose_client = self.create_client(
            SetBool, '/go_to_pose',
            callback_group=self.client_cb_group
        )
        self.param_client = self.create_client(
            SetParameters, '/arm_commander/set_parameters',
            callback_group=self.client_cb_group
        )

        # Service to trigger scan
        self.create_service(
            Trigger,
            '/explorer/start_scan',
            self.start_scan_callback,
            callback_group=self.service_cb_group
        )

        # Timer for scan execution (initially disabled)
        self._scan_timer = None

        # Print scan plan
        self._print_scan_plan()
        self._publish_status(ScanState.IDLE)
        self.get_logger().info("Explorer node ready!")

    def _load_config(self) -> dict:
        """Load environment configuration."""
        config_file = self.get_parameter('config_file').value

        if not config_file:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_path = get_package_share_directory('robot_arm')
                config_file = os.path.join(pkg_path, 'config', 'environment_config.yaml')
            except Exception:
                self.get_logger().error("Could not find config file!")
                return {}

        if not os.path.exists(config_file):
            self.get_logger().error(f"Config file not found: {config_file}")
            return {}

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f"Loaded config from: {config_file}")
            return config
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return {}

    def _generate_viewpoints(self) -> List[Viewpoint]:
        """Generate the full sequence of viewpoints for scanning."""
        viewpoints = []

        view_distance = self.get_parameter('view_distance').value
        lateral_offset = self.get_parameter('lateral_offset').value

        landmarks = self.config.get('landmarks', {})
        clusters = self.config.get('clusters', {})

        # 1. Start with explore_start
        if 'explore_start' in landmarks:
            pos = landmarks['explore_start']['position']
            viewpoints.append(Viewpoint(
                name="explore_start",
                position=pos,
                view_type="explore"
            ))

        # 2. For each cluster, generate 3 viewpoints (center, left, right)
        # Sort by cluster name for consistent order: cluster_1, cluster_2, cluster_3
        for cluster_name in sorted(clusters.keys()):
            cluster_data = clusters[cluster_name]
            cx, cy, cz = cluster_data['position']

            # Viewpoint base: pulled back toward robot, slightly higher
            vx = cx - view_distance
            vz = cz + 0.08

            # Center view
            viewpoints.append(Viewpoint(
                name=f"{cluster_name}_center",
                position=[vx, cy, vz],
                cluster_name=cluster_name,
                view_type="center"
            ))

            # Left view (+Y offset)
            viewpoints.append(Viewpoint(
                name=f"{cluster_name}_left",
                position=[vx, cy + lateral_offset, vz],
                cluster_name=cluster_name,
                view_type="left"
            ))

            # Right view (-Y offset)
            viewpoints.append(Viewpoint(
                name=f"{cluster_name}_right",
                position=[vx, cy - lateral_offset, vz],
                cluster_name=cluster_name,
                view_type="right"
            ))

        # 3. End with explore_end
        if 'explore_end' in landmarks:
            pos = landmarks['explore_end']['position']
            viewpoints.append(Viewpoint(
                name="explore_end",
                position=pos,
                view_type="explore"
            ))

        return viewpoints

    def _print_scan_plan(self):
        """Print the planned scan sequence."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("EXPLORER - Structured Mini-Sweep Scan Plan")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Total viewpoints: {len(self.viewpoints)}")
        self.get_logger().info("-" * 60)

        current_cluster = None
        for i, vp in enumerate(self.viewpoints):
            if vp.cluster_name and vp.cluster_name != current_cluster:
                current_cluster = vp.cluster_name
                self.get_logger().info(f"  [{current_cluster}]")

            pos_str = f"({vp.position[0]:.2f}, {vp.position[1]:.2f}, {vp.position[2]:.2f})"
            prefix = "    " if vp.cluster_name else "  "
            self.get_logger().info(f"{prefix}{i+1:2d}. {vp.name:25s} {pos_str}")

        self.get_logger().info("=" * 60)
        self.get_logger().info("To start: ros2 service call /explorer/start_scan std_srvs/srv/Trigger \"{}\"")
        self.get_logger().info("=" * 60)

    def _publish_status(self, state: ScanState):
        """Publish current scan status."""
        self.state = state
        msg = String()
        msg.data = state.value
        self.status_pub.publish(msg)

    def _publish_viewpoint_reached(self, viewpoint: Viewpoint):
        """Publish that a viewpoint has been reached."""
        msg = String()
        msg.data = f"{viewpoint.name}|{viewpoint.view_type}|{viewpoint.cluster_name or 'none'}"
        self.viewpoint_pub.publish(msg)

    def _move_to_position_sync(self, x: float, y: float, z: float) -> bool:
        """Synchronously move arm to position. Returns success status."""
        # Wait for services
        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Parameter service not available")
            return False

        if not self.go_to_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("go_to_pose service not available")
            return False

        # Set target parameters
        params = [
            Parameter(name='target_x', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(x))),
            Parameter(name='target_y', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(y))),
            Parameter(name='target_z', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(z))),
        ]

        param_req = SetParameters.Request()
        param_req.parameters = params

        try:
            param_future = self.param_client.call_async(param_req)
            rclpy.spin_until_future_complete(self, param_future, timeout_sec=5.0)

            if param_future.result() is None:
                self.get_logger().error("Parameter setting timed out")
                return False

            if not all(r.successful for r in param_future.result().results):
                self.get_logger().error("Failed to set parameters")
                return False

        except Exception as e:
            self.get_logger().error(f"Parameter setting error: {e}")
            return False

        # Call go_to_pose
        pose_req = SetBool.Request()
        pose_req.data = True

        try:
            pose_future = self.go_to_pose_client.call_async(pose_req)
            rclpy.spin_until_future_complete(self, pose_future, timeout_sec=60.0)

            if pose_future.result() is None:
                self.get_logger().error("Move timed out")
                return False

            result = pose_future.result()
            if result.success:
                return True
            else:
                self.get_logger().warn(f"Move failed: {result.message}")
                return False

        except Exception as e:
            self.get_logger().error(f"Move error: {e}")
            return False

    def _execute_scan_thread(self):
        """Execute scan in a separate thread to not block callbacks."""
        pause_duration = self.get_parameter('pause_at_viewpoint').value

        self.get_logger().info("=" * 60)
        self.get_logger().info("STARTING SCAN SEQUENCE")
        self.get_logger().info("=" * 60)

        for i, viewpoint in enumerate(self.viewpoints):
            with self._scan_lock:
                if not self.scan_in_progress:
                    self.get_logger().info("Scan cancelled")
                    self._publish_status(ScanState.IDLE)
                    return

            self.current_viewpoint_idx = i
            progress = f"[{i+1}/{len(self.viewpoints)}]"

            self.get_logger().info(f"{progress} Moving to: {viewpoint.name}")
            self._publish_status(ScanState.MOVING)

            # Move to viewpoint
            x, y, z = viewpoint.position
            success = self._move_to_position_sync(x, y, z)

            if success:
                self._publish_status(ScanState.CAPTURING)
                self._publish_viewpoint_reached(viewpoint)
                self.get_logger().info(f"{progress} REACHED: {viewpoint.name} - capturing for {pause_duration}s")

                # Pause for capture
                import time
                time.sleep(pause_duration)
            else:
                self.get_logger().warn(f"{progress} FAILED to reach {viewpoint.name}, continuing...")

        # Scan complete
        with self._scan_lock:
            self.scan_in_progress = False

        self._publish_status(ScanState.COMPLETE)
        self.get_logger().info("=" * 60)
        self.get_logger().info("SCAN COMPLETE!")
        self.get_logger().info(f"Visited {len(self.viewpoints)} viewpoints")
        self.get_logger().info("=" * 60)

    def start_scan_callback(self, request, response):
        """Service callback to start the scan sequence."""
        with self._scan_lock:
            if self.scan_in_progress:
                response.success = False
                response.message = "Scan already in progress"
                return response

            if not self.viewpoints:
                response.success = False
                response.message = "No viewpoints configured"
                return response

            self.scan_in_progress = True

        # Start scan in a separate thread
        scan_thread = threading.Thread(target=self._execute_scan_thread, daemon=True)
        scan_thread.start()

        response.success = True
        response.message = f"Scan started - {len(self.viewpoints)} viewpoints"
        return response


def main(args=None):
    rclpy.init(args=args)

    node = ExplorerNode()

    # Use multi-threaded executor
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.scan_in_progress = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
