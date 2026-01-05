#!/usr/bin/env python3
"""
YOLO Spatial Detection Pipeline

Coordinates the full detection pipeline:
1. YOLO detection -> bounding boxes
2. Camera focus iterations -> center target in view
3. Depth processing -> 3D world coordinates
4. Cluster tracking -> merge detections, select best
5. Ground truth validation -> compare with known positions

Services:
    /detection/run_at_position (std_srvs/Trigger)
        - Run detection pipeline at current position
    /detection/get_results (harvester_interfaces/srv/GetDetectedClusters)
        - Get all detected clusters
    /detection/validate (std_srvs/Trigger)
        - Validate detections against ground truth
    /detection/clear (std_srvs/Trigger)
        - Clear all detections

Topics Published:
    /detection/status (std_msgs/String) - Pipeline status updates
    /detection/clusters (harvester_interfaces/msg/DetectedCluster[]) - Detected clusters
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from harvester_interfaces.msg import BoundingBox, DetectedCluster
from harvester_interfaces.srv import YoloDetect, PixelTo3D, FocusFromPixel

import yaml
import os
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional
import time

from ament_index_python.packages import get_package_share_directory


@dataclass
class Detection:
    """Single detection from one scan position."""
    cluster_label: str
    position_3d: np.ndarray  # [x, y, z] in world frame
    confidence: float
    bbox_area: int
    scan_position: str  # Name of scan position where detected
    pixel_center: tuple  # (u, v) pixel coordinates


@dataclass
class TrackedCluster:
    """Tracked cluster with multiple detections."""
    cluster_id: str
    detections: List[Detection] = field(default_factory=list)

    @property
    def best_detection(self) -> Optional[Detection]:
        """Return detection with highest bbox area."""
        if not self.detections:
            return None
        return max(self.detections, key=lambda d: d.bbox_area)

    @property
    def position(self) -> Optional[np.ndarray]:
        """Return position from best detection."""
        best = self.best_detection
        return best.position_3d if best else None

    @property
    def num_detections(self) -> int:
        return len(self.detections)


class SpatialDetectionPipeline(Node):
    """Coordinates YOLO detection, focus, and 3D localization."""

    def __init__(self):
        super().__init__('spatial_detection_pipeline')
        self.get_logger().info('Spatial Detection Pipeline initializing...')

        # Parameters
        self.declare_parameter('focus_iterations', 2)
        self.declare_parameter('validation_tolerance', 0.05)  # 5cm
        self.declare_parameter('z_offset_correction', 0.03)   # 3cm offset for mesh origin
        self.declare_parameter('config_file', '')
        self.declare_parameter('merge_radius', 0.0)  # 0 = auto-calculate from cluster distances
        self.declare_parameter('merge_radius_factor', 0.25)  # 25% of min cluster distance

        self.focus_iters = self.get_parameter('focus_iterations').value
        self.tolerance = self.get_parameter('validation_tolerance').value
        self.z_offset = self.get_parameter('z_offset_correction').value
        merge_radius_param = self.get_parameter('merge_radius').value
        self.merge_radius_factor = self.get_parameter('merge_radius_factor').value

        # Load ground truth
        self.ground_truth = self._load_ground_truth()

        # Calculate merge_radius from cluster distances if not specified
        if merge_radius_param <= 0:
            self.merge_radius = self._calculate_merge_radius()
        else:
            self.merge_radius = merge_radius_param

        # Tracked clusters
        self.tracked_clusters: Dict[str, TrackedCluster] = {}
        self.current_scan_position = "unknown"

        # Callback group for service calls
        self.callback_group = ReentrantCallbackGroup()

        # Service clients
        self.yolo_client = self.create_client(
            YoloDetect, '/yolo/detect',
            callback_group=self.callback_group
        )
        self.depth_client = self.create_client(
            PixelTo3D, '/depth_processor/pixel_to_3d',
            callback_group=self.callback_group
        )
        self.focus_client = self.create_client(
            FocusFromPixel, '/camera_focus/center_on_pixel',
            callback_group=self.callback_group
        )

        # Publishers
        self.status_pub = self.create_publisher(String, '/detection/status', 10)

        # Subscriber for current scan position name
        self.position_sub = self.create_subscription(
            String,
            '/detection/current_position',
            self._position_callback,
            10
        )

        # Services
        self.create_service(
            Trigger,
            '/detection/run_at_position',
            self.run_at_position_callback,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger,
            '/detection/validate',
            self.validate_callback,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger,
            '/detection/clear',
            self.clear_callback,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger,
            '/detection/print_results',
            self.print_results_callback,
            callback_group=self.callback_group
        )
        self.create_service(
            Trigger,
            '/detection/wait_ready',
            self.wait_ready_callback,
            callback_group=self.callback_group
        )

        # Wait for service clients
        self._wait_for_services()

        self.get_logger().info('Spatial Detection Pipeline ready.')
        self.get_logger().info(f'  Focus iterations: {self.focus_iters}')
        self.get_logger().info(f'  Validation tolerance: {self.tolerance}m')
        self.get_logger().info(f'  Z offset correction: {self.z_offset}m')
        self.get_logger().info(f'  Merge radius (X,Y): {self.merge_radius:.3f}m')
        self.get_logger().info(f'  Clustering: world-space X,Y with complete-linkage')

    def _wait_for_services(self):
        """Wait for required services to be available."""
        services = [
            (self.yolo_client, '/yolo/detect'),
            (self.depth_client, '/depth_processor/pixel_to_3d'),
        ]
        # Note: focus_client is optional - we can skip focus iterations

        for client, name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'Service {name} not available')
            else:
                self.get_logger().info(f'  Connected to {name}')

    def _load_ground_truth(self) -> Dict[str, np.ndarray]:
        """Load ground truth cluster positions."""
        config_file = self.get_parameter('config_file').value

        if not config_file:
            # Try to find via ROS2 package share directory
            try:
                pkg_share = get_package_share_directory('robot_arm')
                config_file = os.path.join(pkg_share, 'config', 'environment_config.yaml')
            except Exception:
                # Fallback to relative path (for development)
                config_file = os.path.join(
                    os.path.dirname(__file__),
                    '..', '..', '..', 'robot_arm', 'config', 'environment_config.yaml'
                )
                config_file = os.path.normpath(config_file)

        ground_truth = {}

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            if 'clusters' in config:
                for name, data in config['clusters'].items():
                    pos = data.get('position', [0, 0, 0])
                    ground_truth[name] = np.array(pos)
                    self.get_logger().info(
                        f'Ground truth {name}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]'
                    )
        except Exception as e:
            self.get_logger().warn(f'Failed to load ground truth: {e}')
            # Fallback
            ground_truth = {
                'cluster_1': np.array([0.75, 0.45, 0.46]),
                'cluster_2': np.array([0.85, 0.0, 0.52]),
                'cluster_3': np.array([0.75, -0.45, 0.42]),
            }

        return ground_truth

    def _calculate_merge_radius(self) -> float:
        """
        Calculate merge_radius as a fraction of minimum distance between clusters.

        Uses X,Y coordinates only (ignores Z) since clusters are at different heights
        but we care about horizontal separation.
        """
        if len(self.ground_truth) < 2:
            default = 0.12
            self.get_logger().warn(f'Not enough clusters for distance calc, using default: {default}m')
            return default

        # Calculate all pairwise X,Y distances
        positions = list(self.ground_truth.values())
        names = list(self.ground_truth.keys())
        min_dist = float('inf')
        min_pair = ('', '')

        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                # X,Y distance only (ignore Z)
                dx = positions[i][0] - positions[j][0]
                dy = positions[i][1] - positions[j][1]
                dist = np.sqrt(dx*dx + dy*dy)

                if dist < min_dist:
                    min_dist = dist
                    min_pair = (names[i], names[j])

        merge_radius = min_dist * self.merge_radius_factor

        self.get_logger().info(
            f'Cluster distance calculation:'
        )
        self.get_logger().info(
            f'  Min X,Y distance: {min_dist:.3f}m ({min_pair[0]} <-> {min_pair[1]})'
        )
        self.get_logger().info(
            f'  Merge radius: {merge_radius:.3f}m ({self.merge_radius_factor*100:.0f}% of min)'
        )

        return merge_radius

    def _publish_status(self, status: str):
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'[Pipeline] {status}')

    def _position_callback(self, msg: String):
        """Update current scan position name from explorer."""
        self.current_scan_position = msg.data
        self.get_logger().debug(f'Scan position updated: {self.current_scan_position}')

    def set_scan_position(self, position_name: str):
        """Set current scan position name (called by explorer)."""
        self.current_scan_position = position_name

    def run_at_position_callback(self, request, response):
        """Run the full detection pipeline at current position."""
        self._publish_status(f'Running detection at {self.current_scan_position}')

        # Step 1: YOLO detection
        detections = self._call_yolo_detect()
        if not detections:
            response.success = True
            response.message = 'No detections at this position'
            return response

        self._publish_status(f'Found {len(detections)} detections, processing...')

        # Step 2: Process each detection
        for bbox in detections:
            self._process_detection(bbox)

        response.success = True
        response.message = f'Processed {len(detections)} detections'
        return response

    def _call_yolo_detect(self) -> List[BoundingBox]:
        """Call YOLO detection service."""
        if not self.yolo_client.service_is_ready():
            self.get_logger().warn('YOLO service not ready')
            return []

        request = YoloDetect.Request()
        future = self.yolo_client.call_async(request)

        # Wait for result
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('YOLO detection failed')
            return []

        result = future.result()
        if not result.success:
            self.get_logger().warn(f'YOLO: {result.message}')
            return []

        return list(result.detections)

    def _process_detection(self, bbox: BoundingBox):
        """Process a single detection through focus + depth pipeline."""
        label = bbox.label
        center_u = (bbox.u_min + bbox.u_max) // 2
        center_v = (bbox.v_min + bbox.v_max) // 2

        self.get_logger().info(
            f'Processing {label}: center=({center_u}, {center_v}), area={bbox.area}'
        )

        # Focus iterations (optional)
        final_u, final_v = center_u, center_v
        final_bbox = bbox

        for i in range(self.focus_iters):
            self._publish_status(f'Focus iteration {i+1}/{self.focus_iters} for {label}')

            # Call focus service (if available)
            if self.focus_client.service_is_ready():
                focus_success = self._call_focus(final_u, final_v)
                if focus_success:
                    # Wait a bit for arm to settle
                    time.sleep(0.5)

                    # Re-detect to get updated bbox
                    new_detections = self._call_yolo_detect()
                    for new_bbox in new_detections:
                        if new_bbox.label == label:
                            final_u = (new_bbox.u_min + new_bbox.u_max) // 2
                            final_v = (new_bbox.v_min + new_bbox.v_max) // 2
                            final_bbox = new_bbox
                            self.get_logger().info(
                                f'  After focus {i+1}: center=({final_u}, {final_v}), area={final_bbox.area}'
                            )
                            break
            else:
                self.get_logger().debug('Focus service not available, skipping')

        # Get 3D position
        position_3d = self._call_pixel_to_3d(final_u, final_v)
        if position_3d is None:
            self.get_logger().warn(f'Failed to get 3D position for {label}')
            return

        # Apply Z offset correction (ground truth is mesh origin, detection is center)
        position_3d[2] -= self.z_offset

        # Create detection record
        detection = Detection(
            cluster_label=label,
            position_3d=position_3d,
            confidence=final_bbox.confidence,
            bbox_area=final_bbox.area,
            scan_position=self.current_scan_position,
            pixel_center=(final_u, final_v)
        )

        # Add to tracking
        self._add_detection(detection)

    def _call_focus(self, u: int, v: int) -> bool:
        """Call camera focus service."""
        request = FocusFromPixel.Request()
        request.u = u
        request.v = v
        request.view_distance = 0.0  # Use default

        future = self.focus_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            return False

        return future.result().success

    def _call_pixel_to_3d(self, u: int, v: int, max_retries: int = 3) -> Optional[np.ndarray]:
        """Call depth processor to get 3D position with retry logic."""
        for attempt in range(max_retries):
            if not self.depth_client.service_is_ready():
                self.get_logger().warn(f'Depth processor not ready (attempt {attempt+1}/{max_retries})')
                time.sleep(0.5)
                continue

            request = PixelTo3D.Request()
            request.u = u
            request.v = v

            future = self.depth_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is None:
                self.get_logger().warn(f'Depth call returned None (attempt {attempt+1}/{max_retries})')
                time.sleep(0.5)
                continue

            result = future.result()
            if not result.success:
                self.get_logger().warn(f'Pixel to 3D failed: {result.message} (attempt {attempt+1}/{max_retries})')
                time.sleep(0.5)
                continue

            # Success!
            return np.array([
                result.position.x,
                result.position.y,
                result.position.z
            ])

        self.get_logger().error(f'Pixel to 3D failed after {max_retries} attempts')
        return None

    def _add_detection(self, detection: Detection):
        """
        Add detection to tracked clusters using world-space X,Y clustering.

        Uses complete-linkage: a detection joins a cluster only if it's within
        merge_radius (in X,Y plane) of ALL existing detections in that cluster.
        """
        position = detection.position_3d

        def xy_distance(pos1, pos2):
            """Calculate X,Y distance (ignore Z)."""
            dx = pos1[0] - pos2[0]
            dy = pos1[1] - pos2[1]
            return np.sqrt(dx*dx + dy*dy)

        def is_close_to_all(new_pos, cluster):
            """Check if new_pos is within merge_radius of ALL detections in cluster."""
            for det in cluster.detections:
                if xy_distance(new_pos, det.position_3d) > self.merge_radius:
                    return False
            return True

        # Find existing cluster where this detection fits (complete-linkage)
        matched_cluster_id = None
        for cluster_id, cluster in self.tracked_clusters.items():
            if is_close_to_all(position, cluster):
                matched_cluster_id = cluster_id
                break

        if matched_cluster_id:
            # Add to existing cluster
            self.tracked_clusters[matched_cluster_id].detections.append(detection)
            cluster_id = matched_cluster_id
        else:
            # Create new cluster with unique ID
            cluster_id = f'detected_cluster_{len(self.tracked_clusters)}'
            cluster = TrackedCluster(cluster_id=cluster_id)
            cluster.detections.append(detection)
            self.tracked_clusters[cluster_id] = cluster

        self.get_logger().info(
            f'Added detection to {cluster_id}: '
            f'pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}], '
            f'area={detection.bbox_area}, total={self.tracked_clusters[cluster_id].num_detections}'
        )

    def validate_callback(self, request, response):
        """
        Validate detected clusters against ground truth using nearest-neighbor matching.

        Since detected clusters have dynamic IDs (detected_cluster_0, etc.),
        we match each ground truth to its nearest detected cluster.
        """
        self._publish_status('Validating detections against ground truth...')

        # Build list of detected cluster positions
        detected = {}
        for cluster_id, cluster in self.tracked_clusters.items():
            if cluster.position is not None:
                detected[cluster_id] = cluster.position

        results = []
        total_error = 0.0
        valid_count = 0
        matched_detected = set()  # Track which detected clusters are already matched

        # Match each ground truth to nearest detected cluster
        for gt_name, gt_pos in self.ground_truth.items():
            best_match = None
            best_dist = float('inf')

            for det_id, det_pos in detected.items():
                if det_id in matched_detected:
                    continue  # Already matched to another ground truth

                # Calculate X,Y distance (consistent with clustering)
                dx = det_pos[0] - gt_pos[0]
                dy = det_pos[1] - gt_pos[1]
                dist_xy = np.sqrt(dx*dx + dy*dy)

                # Also calculate full 3D distance for reporting
                dist_3d = np.linalg.norm(det_pos - gt_pos)

                if dist_xy < best_dist:
                    best_dist = dist_xy
                    best_match = det_id
                    best_dist_3d = dist_3d

            if best_match is not None:
                matched_detected.add(best_match)
                det_pos = detected[best_match]
                status = "PASS" if best_dist_3d <= self.tolerance else "FAIL"
                total_error += best_dist_3d

                results.append(
                    f'{gt_name} -> {best_match}: error={best_dist_3d:.3f}m (xy={best_dist:.3f}m) [{status}]\n'
                    f'    detected=[{det_pos[0]:.3f}, {det_pos[1]:.3f}, {det_pos[2]:.3f}]\n'
                    f'    gt=[{gt_pos[0]:.3f}, {gt_pos[1]:.3f}, {gt_pos[2]:.3f}]'
                )

                if best_dist_3d <= self.tolerance:
                    valid_count += 1
            else:
                results.append(f'{gt_name}: NOT DETECTED (no unmatched clusters)')

        # Report unmatched detected clusters
        unmatched = set(detected.keys()) - matched_detected
        if unmatched:
            results.append(f'\nUnmatched detected clusters: {list(unmatched)}')

        # Print results
        self.get_logger().info('=' * 60)
        self.get_logger().info('VALIDATION RESULTS (Nearest-Neighbor Matching)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Detected clusters: {len(detected)}')
        self.get_logger().info(f'Ground truth clusters: {len(self.ground_truth)}')
        self.get_logger().info(f'Merge radius used: {self.merge_radius:.3f}m')
        self.get_logger().info('-' * 60)

        for r in results:
            for line in r.split('\n'):
                self.get_logger().info(line)

        if valid_count > 0:
            avg_error = total_error / valid_count
            self.get_logger().info('-' * 60)
            self.get_logger().info(f'Average error: {avg_error:.3f}m')

        self.get_logger().info(f'Valid: {valid_count}/{len(self.ground_truth)} (tolerance: {self.tolerance}m)')
        self.get_logger().info('=' * 60)

        response.success = True
        response.message = f'Validated {len(self.ground_truth)} ground truth, {valid_count} matched'
        return response

    def print_results_callback(self, request, response):
        """Print all tracked clusters."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('DETECTED CLUSTERS')
        self.get_logger().info('=' * 60)

        for label, cluster in self.tracked_clusters.items():
            best = cluster.best_detection
            if best:
                self.get_logger().info(
                    f'{label}: position=[{best.position_3d[0]:.3f}, {best.position_3d[1]:.3f}, {best.position_3d[2]:.3f}], '
                    f'confidence={best.confidence:.2f}, bbox_area={best.bbox_area}, '
                    f'detections={cluster.num_detections}'
                )
            else:
                self.get_logger().info(f'{label}: No detections')

        response.success = True
        response.message = f'{len(self.tracked_clusters)} clusters tracked'
        return response

    def clear_callback(self, request, response):
        """Clear all tracked detections."""
        self.tracked_clusters.clear()
        response.success = True
        response.message = 'Cleared all detections'
        self._publish_status('Cleared all detections')
        return response

    def wait_ready_callback(self, request, response):
        """
        Wait until all required services are ready with data.

        Checks:
        - YOLO detection service is ready
        - Depth processor service is ready
        - Both can successfully respond (have camera data)
        """
        self._publish_status('Checking pipeline readiness...')

        timeout = 10.0  # Total timeout
        check_interval = 0.5
        elapsed = 0.0

        # Check YOLO service readiness
        self.get_logger().info('Waiting for YOLO service...')
        while not self.yolo_client.service_is_ready() and elapsed < timeout:
            time.sleep(check_interval)
            elapsed += check_interval

        if not self.yolo_client.service_is_ready():
            response.success = False
            response.message = 'YOLO service not ready (timeout)'
            return response

        # Check depth processor readiness
        self.get_logger().info('Waiting for depth processor service...')
        while not self.depth_client.service_is_ready() and elapsed < timeout:
            time.sleep(check_interval)
            elapsed += check_interval

        if not self.depth_client.service_is_ready():
            response.success = False
            response.message = 'Depth processor service not ready (timeout)'
            return response

        # Test YOLO detection to ensure camera data is available
        self.get_logger().info('Testing YOLO detection...')
        yolo_request = YoloDetect.Request()
        future = self.yolo_client.call_async(yolo_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            response.success = False
            response.message = 'YOLO detection test failed (no response)'
            return response

        yolo_result = future.result()
        if not yolo_result.success:
            # Check if it's just "no detections" vs actual failure
            if 'Camera info not received' in yolo_result.message or 'timeout' in yolo_result.message.lower():
                response.success = False
                response.message = f'YOLO not ready: {yolo_result.message}'
                return response

        # Test depth processor with center pixel
        self.get_logger().info('Testing depth processor...')
        depth_request = PixelTo3D.Request()
        depth_request.u = 320  # Center of 640px image
        depth_request.v = 240  # Center of 480px image

        future = self.depth_client.call_async(depth_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            response.success = False
            response.message = 'Depth processor test failed (no response)'
            return response

        depth_result = future.result()
        if not depth_result.success:
            # Check if it's just invalid depth at center vs actual failure
            if 'Camera info not received' in depth_result.message or 'No depth image' in depth_result.message:
                response.success = False
                response.message = f'Depth processor not ready: {depth_result.message}'
                return response

        # All checks passed
        response.success = True
        response.message = 'Pipeline ready'
        self._publish_status('Pipeline ready for detection')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpatialDetectionPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
