#!/usr/bin/env python3
"""
Explorer Node - Circular Arc Sweep for Robust Cluster Detection

Orchestrates the robot arm to perform a systematic scan of the cotton field:
1. Move to explore_start position
2. For each cluster: perform a circular arc sweep (multiple viewing angles)
3. Move to explore_end position

The arc sweep provides comprehensive coverage around each cluster for robust detection.

Topics Published:
    /explorer/viewpoint_reached (std_msgs/String) - Signals when arm reaches a viewpoint
    /explorer/scan_status (std_msgs/String) - Current scan status

Services:
    /explorer/start_scan (std_srvs/Trigger) - Triggers the full scan sequence

Parameters:
    config_file - Path to environment_config.yaml
    view_distance - Distance from cluster to viewpoint (default: 0.35m)
    arc_angle_deg - Total arc angle in degrees (default: 90°, i.e., ±45° from center)
    views_per_cluster - Number of viewpoints per cluster (default: 5)
    height_variation - Add height variation to arc (default: 0.05m)
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
import math
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
    position: List[float]  # [x, y, z]
    cluster_name: Optional[str] = None
    view_type: str = "center"  # "center", "arc_N", "explore"
    angle_deg: float = 0.0  # Angle from center (for arc viewpoints)


class ExplorerNode(Node):
    """Orchestrates circular arc sweep exploration for cluster detection."""

    def __init__(self):
        super().__init__('explorer')

        # Callback groups
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('view_distance', 0.35)      # Distance from cluster
        self.declare_parameter('arc_angle_deg', 90.0)      # Total arc span (±45° from center)
        self.declare_parameter('views_per_cluster', 5)     # Number of views per cluster
        self.declare_parameter('height_variation', 0.06)   # Z variation across arc
        self.declare_parameter('pause_at_viewpoint', 1.5)  # Pause duration

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

    def _generate_arc_viewpoints(self, cluster_name: str, cx: float, cy: float, cz: float) -> List[Viewpoint]:
        """
        Generate viewpoints on a circular arc around a cluster.

        The arc is in the XY plane, centered on the cluster, with the robot at origin.
        Viewpoints are distributed evenly across the arc.

        Arc geometry:
        - Center of arc: cluster position
        - Radius: view_distance
        - Arc spans from -arc_angle/2 to +arc_angle/2 around the direct line from robot to cluster
        """
        viewpoints = []

        view_distance = self.get_parameter('view_distance').value
        arc_angle_deg = self.get_parameter('arc_angle_deg').value
        num_views = self.get_parameter('views_per_cluster').value
        height_var = self.get_parameter('height_variation').value

        # Calculate the angle from robot (origin) to cluster
        # This is the "center" direction of the arc
        base_angle = math.atan2(cy, cx)  # Angle in XY plane from origin to cluster

        # Arc angles: distribute evenly from -arc_angle/2 to +arc_angle/2
        half_arc = math.radians(arc_angle_deg / 2)

        if num_views == 1:
            angles = [0.0]
        else:
            angles = [
                -half_arc + (i * 2 * half_arc / (num_views - 1))
                for i in range(num_views)
            ]

        for i, arc_offset in enumerate(angles):
            # Calculate viewpoint position on the arc
            # The viewpoint is at distance `view_distance` from the cluster
            # The angle is base_angle + arc_offset (but we want to VIEW the cluster, so we're on the opposite side)

            # Direction from cluster to viewpoint (opposite of viewing direction)
            view_angle = base_angle + math.pi + arc_offset  # +pi because viewpoint is on robot side

            vx = cx + view_distance * math.cos(view_angle)
            vy = cy + view_distance * math.sin(view_angle)

            # Height variation: parabolic profile (higher at edges, lower at center)
            # This helps see the cluster from different vertical angles
            normalized_pos = arc_offset / half_arc if half_arc != 0 else 0  # -1 to 1
            height_offset = height_var * (normalized_pos ** 2)  # Parabolic: 0 at center, max at edges
            vz = cz + 0.08 + height_offset  # Base offset + variation

            # Determine view type name
            angle_deg = math.degrees(arc_offset)
            if abs(angle_deg) < 5:
                view_type = "center"
            elif angle_deg > 0:
                view_type = f"left_{abs(angle_deg):.0f}deg"
            else:
                view_type = f"right_{abs(angle_deg):.0f}deg"

            viewpoints.append(Viewpoint(
                name=f"{cluster_name}_{view_type}",
                position=[vx, vy, vz],
                cluster_name=cluster_name,
                view_type=view_type,
                angle_deg=angle_deg
            ))

        return viewpoints

    def _generate_viewpoints(self) -> List[Viewpoint]:
        """Generate the full sequence of viewpoints for scanning."""
        viewpoints = []

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

        # 2. For each cluster, generate arc viewpoints
        for cluster_name in sorted(clusters.keys()):
            cluster_data = clusters[cluster_name]
            cx, cy, cz = cluster_data['position']

            arc_viewpoints = self._generate_arc_viewpoints(cluster_name, cx, cy, cz)
            viewpoints.extend(arc_viewpoints)

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
        """Print the planned scan sequence with visual arc diagram."""
        arc_angle = self.get_parameter('arc_angle_deg').value
        num_views = self.get_parameter('views_per_cluster').value
        view_dist = self.get_parameter('view_distance').value

        self.get_logger().info("=" * 65)
        self.get_logger().info("EXPLORER - Circular Arc Sweep Scan Plan")
        self.get_logger().info("=" * 65)
        self.get_logger().info(f"Arc configuration:")
        self.get_logger().info(f"  - View distance:    {view_dist:.2f}m")
        self.get_logger().info(f"  - Arc angle:        {arc_angle:.0f}° (±{arc_angle/2:.0f}° from center)")
        self.get_logger().info(f"  - Views per cluster: {num_views}")
        self.get_logger().info(f"  - Total viewpoints:  {len(self.viewpoints)}")
        self.get_logger().info("-" * 65)

        # Visual representation of arc pattern
        self.get_logger().info("Arc pattern (top view):")
        self.get_logger().info("                    [CLUSTER]")
        self.get_logger().info("                        *")
        self.get_logger().info("                      / | \\")
        self.get_logger().info("                     /  |  \\")
        self.get_logger().info("                    v1 v2 v3 v4 v5  <- viewpoints")
        self.get_logger().info("                        |")
        self.get_logger().info("                     [ROBOT]")
        self.get_logger().info("-" * 65)

        # List all viewpoints
        current_cluster = None
        for i, vp in enumerate(self.viewpoints):
            if vp.cluster_name and vp.cluster_name != current_cluster:
                current_cluster = vp.cluster_name
                self.get_logger().info(f"  [{current_cluster}]")

            pos_str = f"({vp.position[0]:.2f}, {vp.position[1]:.2f}, {vp.position[2]:.2f})"
            prefix = "    " if vp.cluster_name else "  "

            angle_str = ""
            if vp.angle_deg != 0:
                angle_str = f" [{vp.angle_deg:+.0f}°]"

            self.get_logger().info(f"{prefix}{i+1:2d}. {vp.name:30s} {pos_str}{angle_str}")

        self.get_logger().info("=" * 65)
        self.get_logger().info("To start: ros2 service call /explorer/start_scan std_srvs/srv/Trigger \"{}\"")
        self.get_logger().info("=" * 65)

    def _publish_status(self, state: ScanState):
        """Publish current scan status."""
        self.state = state
        msg = String()
        msg.data = state.value
        self.status_pub.publish(msg)

    def _publish_viewpoint_reached(self, viewpoint: Viewpoint):
        """Publish that a viewpoint has been reached."""
        msg = String()
        # Format: name|view_type|cluster_name|angle_deg
        msg.data = f"{viewpoint.name}|{viewpoint.view_type}|{viewpoint.cluster_name or 'none'}|{viewpoint.angle_deg:.1f}"
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
        """Execute scan in a separate thread."""
        pause_duration = self.get_parameter('pause_at_viewpoint').value

        self.get_logger().info("=" * 65)
        self.get_logger().info("STARTING CIRCULAR ARC SCAN SEQUENCE")
        self.get_logger().info("=" * 65)

        successful_views = 0
        failed_views = 0

        for i, viewpoint in enumerate(self.viewpoints):
            with self._scan_lock:
                if not self.scan_in_progress:
                    self.get_logger().info("Scan cancelled")
                    self._publish_status(ScanState.IDLE)
                    return

            self.current_viewpoint_idx = i
            progress = f"[{i+1}/{len(self.viewpoints)}]"

            # Log with angle info for arc viewpoints
            angle_info = ""
            if viewpoint.angle_deg != 0:
                angle_info = f" (angle: {viewpoint.angle_deg:+.0f}°)"

            self.get_logger().info(f"{progress} Moving to: {viewpoint.name}{angle_info}")
            self._publish_status(ScanState.MOVING)

            # Move to viewpoint
            x, y, z = viewpoint.position
            success = self._move_to_position_sync(x, y, z)

            if success:
                successful_views += 1
                self._publish_status(ScanState.CAPTURING)
                self._publish_viewpoint_reached(viewpoint)
                self.get_logger().info(f"{progress} REACHED: {viewpoint.name} - capturing for {pause_duration}s")

                # Pause for capture
                import time
                time.sleep(pause_duration)
            else:
                failed_views += 1
                self.get_logger().warn(f"{progress} FAILED to reach {viewpoint.name}, continuing...")

        # Scan complete
        with self._scan_lock:
            self.scan_in_progress = False

        self._publish_status(ScanState.COMPLETE)
        self.get_logger().info("=" * 65)
        self.get_logger().info("SCAN COMPLETE!")
        self.get_logger().info(f"  Successful: {successful_views}/{len(self.viewpoints)}")
        self.get_logger().info(f"  Failed:     {failed_views}/{len(self.viewpoints)}")
        self.get_logger().info("=" * 65)

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
        response.message = f"Scan started - {len(self.viewpoints)} viewpoints (arc sweep)"
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
