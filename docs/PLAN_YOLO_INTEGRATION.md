# YOLO Integration Plan

**Date**: 2026-01-05
**Status**: Ready for Implementation

---

## Overview

Integrate real YOLO cotton detection into the robocot system, replacing the mock detector.

**Current**: `mock_yolo_detector.py` projects known 3D positions to fake 2D boxes
**Target**: `real_yolo_detector.py` runs actual YOLO inference on camera feed

---

## Phase 1: Environment Setup

### 1.1 Install uv in WSL

```bash
# In WSL2
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
uv --version
```

### 1.2 Create Virtual Environment for YOLO

```bash
cd /mnt/c/Users/ayhan/harvesting_ws/src/docs/RESEARCH/Cotton-Tracking-YOLO
uv venv .venv
source .venv/bin/activate

# Install dependencies
uv pip install ultralytics opencv-python numpy
```

### 1.3 Verify YOLO Works

```bash
source .venv/bin/activate
python3 -c "from ultralytics import YOLO; m = YOLO('best.pt'); print('Model loaded successfully')"
```

---

## Phase 2: Static Image Testing

### 2.1 Save Gazebo Camera Frames

**Step 1**: Start Gazebo simulation (in WSL terminal 1):
```bash
cd /mnt/c/Users/ayhan/harvesting_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch robot_arm bot.launch.py
```

**Step 2**: Save camera frames (in WSL terminal 2):
```bash
cd /mnt/c/Users/ayhan/harvesting_ws
source install/setup.bash

# Create output directory
mkdir -p src/docs/RESEARCH/Cotton-Tracking-YOLO/gazebo_frames

# Save single frame
ros2 run image_tools cam2image --ros-args -r image:=/camera/color/image_raw

# OR use this Python snippet:
python3 << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')
        self.bridge = CvBridge()
        self.count = 0
        self.max_frames = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.callback, 10
        )
        self.get_logger().info(f'Saving {self.max_frames} frames...')

    def callback(self, msg):
        if self.count >= self.max_frames:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        path = f'src/docs/RESEARCH/Cotton-Tracking-YOLO/gazebo_frames/frame_{self.count:03d}.png'
        cv2.imwrite(path, img)
        self.get_logger().info(f'Saved: {path}')
        self.count += 1
        if self.count >= self.max_frames:
            self.get_logger().info('Done!')
            raise SystemExit

rclpy.init()
node = FrameSaver()
try:
    rclpy.spin(node)
except SystemExit:
    pass
node.destroy_node()
rclpy.shutdown()
EOF
```

**Tip**: Move arm to different positions before saving to get varied viewpoints:
```bash
ros2 service call /explorer/panoramic_scan std_srvs/srv/Trigger "{}"
# Save frames at different scan positions
```

### 2.2 Run YOLO on Static Images

Create test script:

**File**: `src/docs/RESEARCH/Cotton-Tracking-YOLO/test_yolo_static.py`

```python
#!/usr/bin/env python3
"""
Test YOLO detection on static Gazebo frames.
Saves annotated images to gazebo_frames/output/
"""

from ultralytics import YOLO
import cv2
import os
from pathlib import Path

# Configuration
MODEL_PATH = 'best.pt'
INPUT_DIR = 'gazebo_frames'
OUTPUT_DIR = 'gazebo_frames/output'
CONFIDENCE = 0.7  # Configurable threshold

def main():
    # Setup
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    model = YOLO(MODEL_PATH)

    # Find input images
    input_path = Path(INPUT_DIR)
    images = list(input_path.glob('*.png')) + list(input_path.glob('*.jpg'))

    if not images:
        print(f"No images found in {INPUT_DIR}")
        return

    print(f"Found {len(images)} images")
    print(f"Using confidence threshold: {CONFIDENCE}")
    print("-" * 50)

    for img_path in sorted(images):
        print(f"\nProcessing: {img_path.name}")

        # Load image
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  Failed to load image")
            continue

        # Run detection
        results = model.predict(
            source=img,
            conf=CONFIDENCE,
            verbose=False
        )

        # Get detections
        boxes = results[0].boxes
        print(f"  Detections: {len(boxes)}")

        for i, box in enumerate(boxes):
            cls_id = int(box.cls[0])
            cls_name = model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
            area = (x2 - x1) * (y2 - y1)
            print(f"    [{i}] {cls_name}: conf={conf:.2f}, box=({x1},{y1})-({x2},{y2}), area={area}")

        # Draw annotations
        annotated = results[0].plot()

        # Save output
        output_path = Path(OUTPUT_DIR) / f"detected_{img_path.name}"
        cv2.imwrite(str(output_path), annotated)
        print(f"  Saved: {output_path}")

    print("\n" + "=" * 50)
    print(f"Done! Check {OUTPUT_DIR}/ for results")

if __name__ == '__main__':
    main()
```

**Run it**:
```bash
cd /mnt/c/Users/ayhan/harvesting_ws/src/docs/RESEARCH/Cotton-Tracking-YOLO
source .venv/bin/activate
python3 test_yolo_static.py
```

---

## Phase 3: Real YOLO ROS2 Node

### 3.1 Copy Model to Package

```bash
mkdir -p /mnt/c/Users/ayhan/harvesting_ws/src/orchestrator/models
cp /mnt/c/Users/ayhan/harvesting_ws/src/docs/RESEARCH/Cotton-Tracking-YOLO/best.pt \
   /mnt/c/Users/ayhan/harvesting_ws/src/orchestrator/models/
```

### 3.2 Create Real YOLO Detector Node

**File**: `src/orchestrator/orchestrator/real_yolo_detector.py`

```python
#!/usr/bin/env python3
"""
Real YOLO Detector Node

Runs actual YOLO inference on camera images.
Replaces mock_yolo_detector.py for real detection.

Service:
    /yolo/detect (harvester_interfaces/srv/YoloDetect)
        - Returns bounding boxes from YOLO model

Parameters:
    - model_path: Path to YOLO model (.pt file)
    - confidence: Detection confidence threshold (default: 0.7)
    - camera_topic: Camera image topic (default: /camera/color/image_raw)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from harvester_interfaces.msg import BoundingBox
from harvester_interfaces.srv import YoloDetect

from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import os


class RealYoloDetector(Node):
    """Real YOLO detection using ultralytics."""

    def __init__(self):
        super().__init__('real_yolo_detector')
        self.get_logger().info('Real YOLO Detector initializing...')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence', 0.7)
        self.declare_parameter('camera_topic', '/camera/color/image_raw')

        model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        camera_topic = self.get_parameter('camera_topic').value

        # Find model
        if not model_path:
            # Default: look in package models/ folder
            model_path = os.path.join(
                os.path.dirname(__file__), '..', 'models', 'best.pt'
            )
            model_path = os.path.normpath(model_path)

        if not os.path.exists(model_path):
            self.get_logger().error(f'Model not found: {model_path}')
            raise FileNotFoundError(f'YOLO model not found: {model_path}')

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Model loaded. Classes: {self.model.names}')

        # CV Bridge
        self.bridge = CvBridge()

        # Latest frame storage
        self.latest_frame = None
        self.frame_timestamp = None

        # QoS for camera
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            sensor_qos
        )

        # Detection service
        self.detect_srv = self.create_service(
            YoloDetect,
            '/yolo/detect',
            self.detect_callback
        )

        self.get_logger().info('Real YOLO Detector ready.')
        self.get_logger().info(f'  Model: {model_path}')
        self.get_logger().info(f'  Confidence: {self.confidence}')
        self.get_logger().info(f'  Camera: {camera_topic}')
        self.get_logger().info(f'  Service: /yolo/detect')

    def image_callback(self, msg: Image):
        """Store latest camera frame."""
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frame_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def detect_callback(self, request, response):
        """Run YOLO detection on latest frame."""
        response.detections = []
        response.success = False

        # Check for frame
        if self.latest_frame is None:
            response.message = 'No camera frame received'
            self.get_logger().warn(response.message)
            return response

        # Run YOLO inference
        try:
            results = self.model.predict(
                source=self.latest_frame,
                conf=self.confidence,
                verbose=False
            )
        except Exception as e:
            response.message = f'YOLO inference failed: {e}'
            self.get_logger().error(response.message)
            return response

        # Process detections
        boxes = results[0].boxes

        for box in boxes:
            bbox = BoundingBox()

            # Bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
            bbox.u_min = x1
            bbox.v_min = y1
            bbox.u_max = x2
            bbox.v_max = y2

            # Confidence
            bbox.confidence = float(box.conf[0])

            # Class label
            cls_id = int(box.cls[0])
            bbox.label = self.model.names[cls_id]  # 'cotton_boll' or 'cotton_boll-cluster'

            # Area
            bbox.area = (x2 - x1) * (y2 - y1)

            response.detections.append(bbox)

        response.success = True
        response.message = f'Detected {len(response.detections)} objects'

        if response.detections:
            labels = [d.label for d in response.detections]
            self.get_logger().info(f'YOLO detect: {len(labels)} detections - {labels}')
        else:
            self.get_logger().info('YOLO detect: No detections')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RealYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.3 Update Package Files

**Update `setup.py`** - Add entry point:
```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'real_yolo_detector = orchestrator.real_yolo_detector:main',
    ],
},
```

**Update `package.xml`** - Add cv_bridge dependency:
```xml
<depend>cv_bridge</depend>
```

### 3.4 Install Python Dependencies in ROS Environment

```bash
# In the ROS2 Python environment (not the uv venv)
pip3 install ultralytics
```

---

## Phase 4: Pipeline Integration

### 4.1 Label Handling Strategy

**Problem**:
- Mock detector returns: `cluster_1`, `cluster_2`, `cluster_3`
- Real YOLO returns: `cotton_boll-cluster` (same label for all)

**Solution**: Modify `SpatialDetectionPipeline` to use **spatial clustering**:

1. YOLO returns `cotton_boll-cluster` for each detection
2. After depth processing, we have 3D positions
3. Use spatial proximity to merge nearby detections
4. Assign unique IDs: `detected_cluster_0`, `detected_cluster_1`, etc.
5. During validation, match to ground truth by **nearest neighbor**

**Key insight**: We do identification AFTER panoramic scan completes (line 420-465 in spatial_detection_pipeline.py), so we can rename clusters by their spatial positions.

### 4.2 Update SpatialDetectionPipeline

Changes needed in `spatial_detection_pipeline.py`:

```python
# In _add_detection(), change from label-based to spatial-based tracking:

def _add_detection(self, detection: Detection):
    """Add detection using spatial clustering instead of label matching."""
    position = detection.position_3d

    # Find existing cluster within merge_radius
    matched_cluster = None
    for cluster_id, cluster in self.tracked_clusters.items():
        if cluster.position is not None:
            distance = np.linalg.norm(position - cluster.position)
            if distance < self.merge_radius:
                matched_cluster = cluster_id
                break

    if matched_cluster:
        # Add to existing cluster
        self.tracked_clusters[matched_cluster].detections.append(detection)
    else:
        # Create new cluster with unique ID
        new_id = f"detected_cluster_{len(self.tracked_clusters)}"
        cluster = TrackedCluster(cluster_id=new_id)
        cluster.detections.append(detection)
        self.tracked_clusters[new_id] = cluster
```

### 4.3 Update Validation to Use Nearest-Neighbor Matching

```python
# In validate_callback(), match by proximity instead of name:

def validate_callback(self, request, response):
    """Validate using nearest-neighbor matching to ground truth."""

    # Build list of detected positions
    detected = {}
    for cluster_id, cluster in self.tracked_clusters.items():
        if cluster.position is not None:
            detected[cluster_id] = cluster.position

    # Match each ground truth to nearest detected cluster
    matched = {}
    for gt_name, gt_pos in self.ground_truth.items():
        best_match = None
        best_dist = float('inf')

        for det_id, det_pos in detected.items():
            if det_id in matched.values():
                continue  # Already matched
            dist = np.linalg.norm(det_pos - gt_pos)
            if dist < best_dist:
                best_dist = dist
                best_match = det_id

        if best_match and best_dist <= self.tolerance:
            matched[gt_name] = best_match
            # ... report PASS
        else:
            # ... report FAIL or NOT DETECTED
```

---

## Phase 5: Visualization (Future)

For later implementation - publish annotated images:

**File**: `src/orchestrator/orchestrator/yolo_visualizer.py`

```python
# Subscribe to /camera/color/image_raw
# Call /yolo/detect service
# Draw bounding boxes on image
# Publish to /yolo/annotated_image (sensor_msgs/Image)
# View with: ros2 run rqt_image_view rqt_image_view /yolo/annotated_image
```

---

## File Summary

| File | Action | Description |
|------|--------|-------------|
| `Cotton-Tracking-YOLO/test_yolo_static.py` | CREATE | Static image testing script |
| `orchestrator/models/best.pt` | COPY | YOLO model |
| `orchestrator/orchestrator/real_yolo_detector.py` | CREATE | Real YOLO ROS node |
| `orchestrator/setup.py` | MODIFY | Add entry point |
| `orchestrator/package.xml` | MODIFY | Add cv_bridge |
| `orchestrator/orchestrator/spatial_detection_pipeline.py` | MODIFY | Spatial clustering |

---

## Commands Reference

### Phase 1 - Setup
```bash
# Install uv in WSL
curl -LsSf https://astral.sh/uv/install.sh | sh

# Create venv and install deps
cd /mnt/c/Users/ayhan/harvesting_ws/src/docs/RESEARCH/Cotton-Tracking-YOLO
uv venv .venv
source .venv/bin/activate
uv pip install ultralytics opencv-python numpy
```

### Phase 2 - Static Test
```bash
# Terminal 1: Run Gazebo
ros2 launch robot_arm bot.launch.py

# Terminal 2: Save frames (move arm to different positions first)
# ... save frame script ...

# Terminal 3: Run YOLO
cd /mnt/c/Users/ayhan/harvesting_ws/src/docs/RESEARCH/Cotton-Tracking-YOLO
source .venv/bin/activate
python3 test_yolo_static.py
# Check gazebo_frames/output/ for results
```

### Phase 3 - ROS Integration
```bash
# Build
colcon build --packages-select orchestrator

# Run (replaces mock_yolo_detector)
ros2 run orchestrator real_yolo_detector

# Test
ros2 service call /yolo/detect harvester_interfaces/srv/YoloDetect "{}"
```

---

## Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| `confidence` | 0.7 | Configurable, may need tuning for Gazebo |
| `merge_radius` | 0.10m | Spatial clustering radius |
| `validation_tolerance` | 0.05m | 5cm error tolerance |

---

## Next Steps After YOLO Works

1. Run panoramic scan with real YOLO
2. Verify 3 clusters detected at correct 3D positions
3. Create CLUSTER_VIEW positions for each detected cluster
4. Implement boll detection within CLUSTER_VIEW
5. Full harvesting cycle demo
