#!/bin/bash
# =============================================================
# RoboCot Environment Verification Script
# Run on both Mac (RoboStack) and Monster (WSL/Ubuntu) to compare
# =============================================================

echo "============================================"
echo "  RoboCot Environment Verification"
echo "  Date: $(date)"
echo "  Host: $(hostname)"
echo "  OS:   $(uname -s -m)"
echo "============================================"
echo ""

# --- 1. System Info ---
echo "=== 1. SYSTEM ==="
echo "Python:  $(python --version 2>&1)"
echo "Arch:    $(uname -m)"
if [ -f /etc/os-release ]; then
    echo "Distro:  $(grep PRETTY_NAME /etc/os-release | cut -d= -f2)"
elif [ "$(uname)" = "Darwin" ]; then
    echo "Distro:  macOS $(sw_vers -productVersion)"
fi
echo ""

# --- 2. ROS2 ---
echo "=== 2. ROS2 ==="
if command -v ros2 &>/dev/null; then
    echo "ros2 CLI:     OK ($(which ros2))"
    echo "ROS_DISTRO:   ${ROS_DISTRO:-NOT SET}"
    echo "RMW:          ${RMW_IMPLEMENTATION:-default}"
    ros2 pkg list 2>/dev/null | grep -cE "ros-" | xargs -I{} echo "Total pkgs:   {} ros packages"
else
    echo "ros2 CLI:     NOT FOUND"
fi
echo ""

# --- 3. Custom Packages ---
echo "=== 3. CUSTOM PACKAGES ==="
for pkg in robot_arm robot_arm_moveit_config orchestrator harvester_interfaces; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        echo "  $pkg: OK"
    else
        echo "  $pkg: MISSING"
    fi
done
echo ""

# --- 4. Gazebo ---
echo "=== 4. GAZEBO ==="
if command -v ign &>/dev/null; then
    echo "ign CLI:      $(ign gazebo --version 2>&1 | head -1)"
elif command -v gz &>/dev/null; then
    echo "gz CLI:       $(gz sim --version 2>&1 | head -1)"
else
    echo "Gazebo:       NOT FOUND"
fi
echo ""

# --- 5. MoveIt2 ---
echo "=== 5. MOVEIT2 ==="
python -c "import moveit_msgs.msg; print('  moveit_msgs:     OK')" 2>/dev/null || echo "  moveit_msgs:     MISSING"
for pkg in moveit_ros_planning_interface moveit_ros_move_group moveit_kinematics moveit_planners_ompl; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        echo "  $pkg: OK"
    else
        echo "  $pkg: MISSING"
    fi
done
echo ""

# --- 6. ros2_control ---
echo "=== 6. ROS2_CONTROL ==="
for pkg in controller_manager joint_state_broadcaster joint_trajectory_controller gz_ros2_control; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        echo "  $pkg: OK"
    else
        echo "  $pkg: MISSING"
    fi
done
echo ""

# --- 7. Vision/ML ---
echo "=== 7. VISION & ML ==="
python -c "from ultralytics import YOLO; print('  YOLO (ultralytics): OK')" 2>/dev/null || echo "  YOLO: MISSING"
python -c "import cv2; print('  OpenCV:             OK (v' + cv2.__version__ + ')')" 2>/dev/null || echo "  OpenCV: MISSING"
python -c "from cv_bridge import CvBridge; print('  cv_bridge:          OK')" 2>/dev/null || echo "  cv_bridge: MISSING"
python -c "import torch; dev='MPS' if torch.backends.mps.is_available() else ('CUDA' if torch.cuda.is_available() else 'CPU'); print(f'  PyTorch:            OK (v{torch.__version__}, {dev})')" 2>/dev/null || echo "  PyTorch: MISSING"
python -c "import numpy; print('  NumPy:              OK (v' + numpy.__version__ + ')')" 2>/dev/null || echo "  NumPy: MISSING"
echo ""

# --- 8. ROS-Gazebo Bridge ---
echo "=== 8. ROS-GZ BRIDGE ==="
for pkg in ros_gz_bridge ros_gz_image ros_gz_sim; do
    if ros2 pkg list 2>/dev/null | grep -q "^${pkg}$"; then
        echo "  $pkg: OK"
    else
        echo "  $pkg: MISSING"
    fi
done
echo ""

# --- 9. Custom Interfaces ---
echo "=== 9. CUSTOM INTERFACES ==="
for iface in BoundingBox DetectedCluster; do
    ros2 interface show harvester_interfaces/msg/$iface &>/dev/null && echo "  msg/$iface: OK" || echo "  msg/$iface: MISSING"
done
for iface in YoloDetect PixelTo3D FocusFromPixel FocusFromPosition; do
    ros2 interface show harvester_interfaces/srv/$iface &>/dev/null && echo "  srv/$iface: OK" || echo "  srv/$iface: MISSING"
done
echo ""

# --- 10. URDF Parse Test ---
echo "=== 10. URDF PARSE ==="
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
CONTROLLERS_YAML="${WS_DIR}/install/robot_arm/share/robot_arm/yaml/controllers.yaml"
URDF_FILE="${WS_DIR}/robot_arm/urdf/m1013_robocot.urdf.xacro"
if [ -f "$URDF_FILE" ] && [ -f "$CONTROLLERS_YAML" ]; then
    LINES=$(ros2 run xacro xacro "$URDF_FILE" controllers_yaml:="$CONTROLLERS_YAML" 2>/dev/null | wc -l)
    JOINTS=$(ros2 run xacro xacro "$URDF_FILE" controllers_yaml:="$CONTROLLERS_YAML" 2>/dev/null | grep -c '<joint.*type="revolute"\|type="prismatic"')
    echo "  URDF lines:  $LINES"
    echo "  Active joints: $JOINTS (expected: 8 = 6 arm + 2 gripper)"
else
    echo "  URDF or controllers.yaml not found (run colcon build first)"
fi
echo ""

# --- 11. Build Tools ---
echo "=== 11. BUILD TOOLS ==="
echo "  colcon:   $(colcon version-check 2>&1 | head -1 || echo 'NOT FOUND')"
echo "  cmake:    $(cmake --version 2>&1 | head -1)"
echo "  make:     $(make --version 2>&1 | head -1)"
echo ""

# --- 12. SSH Readiness ---
echo "=== 12. SSH (Lab Access) ==="
echo "  ssh:      $(ssh -V 2>&1)"
if command -v ping &>/dev/null; then
    echo "  ping:     available"
else
    echo "  ping:     NOT FOUND"
fi
echo ""

echo "============================================"
echo "  Verification Complete"
echo "============================================"
