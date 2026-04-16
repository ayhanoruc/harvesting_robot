#!/bin/bash
# =============================================================
# RoboCot Xavier Deployment Script
# =============================================================
# Run this ON the Xavier after SSH'ing in.
# It handles: system check, repo clone, Docker build, and test.
#
# PREREQUISITE: Code already transferred to Xavier via scp:
#   scp -r harvesting_robot robocob@192.168.3.4:~/harvesting_ws/src
#
# Usage (on Xavier):
#   ssh robocob@192.168.3.4  (password: 1)
#   cd ~/harvesting_ws
#   bash src/scripts/xavier_deploy.sh all
#
# Or step by step:
#   bash src/scripts/xavier_deploy.sh check    # System + permissions check
#   bash src/scripts/xavier_deploy.sh verify   # Check source code exists
#   bash src/scripts/xavier_deploy.sh build    # Build Docker image
#   bash src/scripts/xavier_deploy.sh run      # Run container
#   bash src/scripts/xavier_deploy.sh test     # Run tests inside container
# =============================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

REPO_URL="https://github.com/ayhanoruc/harvesting_robot.git"
WORKSPACE="$HOME/harvesting_ws"
IMAGE_NAME="robocot"
CONTAINER_NAME="robocot"

# -----------------------------------------------------------
# STEP 0: System Check
# -----------------------------------------------------------
check_system() {
    echo -e "${CYAN}============================================${NC}"
    echo -e "${CYAN}  RoboCot Xavier System Check${NC}"
    echo -e "${CYAN}  $(date)${NC}"
    echo -e "${CYAN}============================================${NC}"
    echo ""

    # --- sudo / docker permissions ---
    echo -e "${YELLOW}--- 0. Permissions (sudo & docker group) ---${NC}"
    if sudo -n true 2>/dev/null; then
        echo -e "  sudo: ${GREEN}OK (passwordless)${NC}"
    elif echo "1" | sudo -S true 2>/dev/null; then
        echo -e "  sudo: ${GREEN}OK (password '1' works)${NC}"
    else
        echo -e "  sudo: ${RED}FAIL — cannot run sudo! Docker won't work.${NC}"
        echo -e "  ${YELLOW}Try: su - root, or ask lab admin to add robocob to sudoers${NC}"
    fi

    if groups 2>/dev/null | grep -qw docker; then
        echo -e "  docker group: ${GREEN}YES (can run docker without sudo)${NC}"
    else
        echo -e "  docker group: ${YELLOW}NO — need sudo for every docker command${NC}"
    fi

    # Quick docker access test
    if sudo docker ps > /dev/null 2>&1; then
        echo -e "  sudo docker: ${GREEN}OK${NC}"
    elif docker ps > /dev/null 2>&1; then
        echo -e "  docker (no sudo): ${GREEN}OK${NC}"
    else
        echo -e "  docker access: ${RED}FAIL — neither sudo docker nor docker works!${NC}"
        echo -e "  ${YELLOW}Possible fixes:${NC}"
        echo -e "    1. sudo usermod -aG docker robocob && newgrp docker"
        echo -e "    2. Ask lab admin for docker/sudo permissions"
        echo -e "    3. Try: sudo systemctl start docker"
    fi
    echo ""

    # --- JetPack / L4T version ---
    echo -e "${YELLOW}--- 1. JetPack / L4T Version ---${NC}"
    if [ -f /etc/nv_tegra_release ]; then
        L4T_LINE=$(head -1 /etc/nv_tegra_release)
        echo -e "  ${GREEN}$L4T_LINE${NC}"
        # Extract R35 (revision 4.1) -> r35.4.1
        L4T_MAJOR=$(echo "$L4T_LINE" | sed -nE 's/.*R([0-9]+).*/\1/p')
        L4T_MINOR=$(echo "$L4T_LINE" | sed -nE 's/.*REVISION: ([0-9]+\.[0-9]+).*/\1/p')
        L4T_TAG="r${L4T_MAJOR}.${L4T_MINOR}"
        echo -e "  L4T tag for Docker: ${GREEN}${L4T_TAG}${NC}"
        echo ""
        echo -e "  ${YELLOW}IMPORTANT: Use this tag when building Docker image:${NC}"
        echo -e "  ${CYAN}sudo docker build --build-arg L4T_TAG=${L4T_TAG} -t robocot src/${NC}"
    else
        echo -e "  ${RED}ERROR: /etc/nv_tegra_release not found. Is this a Jetson?${NC}"
        L4T_TAG="r35.4.1"
    fi
    echo ""

    # --- Ubuntu version ---
    echo -e "${YELLOW}--- 2. Ubuntu Version ---${NC}"
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        echo -e "  ${GREEN}$PRETTY_NAME${NC}"
    fi
    echo ""

    # --- Disk space ---
    echo -e "${YELLOW}--- 3. Disk Space ---${NC}"
    df -h / | tail -1 | awk '{printf "  Total: %s, Used: %s, Free: \033[0;32m%s\033[0m (%s)\n", $2, $3, $4, $5}'
    FREE_GB=$(df -BG / | tail -1 | awk '{print $4}' | tr -d 'G')
    if [ "$FREE_GB" -lt 15 ]; then
        echo -e "  ${RED}WARNING: Less than 15GB free! Docker image needs ~10-15GB${NC}"
    else
        echo -e "  ${GREEN}OK: Enough space for Docker build${NC}"
    fi
    echo ""

    # --- Docker ---
    echo -e "${YELLOW}--- 4. Docker ---${NC}"
    if command -v docker &>/dev/null; then
        echo -e "  Docker: ${GREEN}$(docker --version)${NC}"
    else
        echo -e "  ${RED}Docker NOT installed!${NC}"
        echo "  Install: curl -fsSL https://get.docker.com | sh"
        return 1
    fi

    # NVIDIA runtime
    if sudo docker info 2>/dev/null | grep -qi "nvidia"; then
        echo -e "  NVIDIA runtime: ${GREEN}Available${NC}"
    else
        echo -e "  ${RED}NVIDIA runtime NOT found!${NC}"
    fi
    echo ""

    # --- Network: Doosan ---
    echo -e "${YELLOW}--- 5. Network: Doosan Controller ---${NC}"
    if ping -c 1 -W 2 192.168.3.5 &>/dev/null; then
        echo -e "  Ping 192.168.3.5: ${GREEN}OK${NC}"
    else
        echo -e "  Ping 192.168.3.5: ${RED}FAIL (robot controller off or not connected)${NC}"
    fi

    if nc -zv -w 2 192.168.3.5 12345 2>&1 | grep -q "succeeded\|open"; then
        echo -e "  Port 12345: ${GREEN}OPEN (remote mode active)${NC}"
    else
        echo -e "  Port 12345: ${YELLOW}CLOSED (enable remote mode on teach pendant)${NC}"
    fi
    echo ""

    # --- USB devices ---
    echo -e "${YELLOW}--- 6. USB Devices ---${NC}"
    echo "  Connected USB devices:"
    lsusb 2>/dev/null | while IFS= read -r line; do
        echo "    $line"
    done
    echo ""
    echo "  Serial ports:"
    ls /dev/ttyUSB* /dev/ttyACM* /dev/robotiq_gripper 2>/dev/null | while IFS= read -r line; do
        echo -e "    ${GREEN}$line${NC}"
    done
    if ! ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1 &>/dev/null; then
        echo -e "    ${YELLOW}No serial ports found (Hand-E RS485 converter not connected?)${NC}"
    fi
    echo ""

    # --- Camera ---
    echo -e "${YELLOW}--- 7. Camera ---${NC}"
    ls /dev/video* 2>/dev/null | while IFS= read -r line; do
        echo -e "    ${GREEN}$line${NC}"
    done
    if ! ls /dev/video* 2>/dev/null | head -1 &>/dev/null; then
        echo -e "    ${YELLOW}No video devices found${NC}"
    fi
    if lsusb 2>/dev/null | grep -qi "intel\|realsense"; then
        echo -e "    ${GREEN}RealSense detected in lsusb${NC}"
    fi
    echo ""

    # --- Internet ---
    echo -e "${YELLOW}--- 8. Internet Access ---${NC}"
    if ping -c 1 -W 3 8.8.8.8 &>/dev/null; then
        echo -e "  Internet: ${GREEN}OK${NC}"
    else
        echo -e "  Internet: ${RED}NO (Docker pull will fail, need local image)${NC}"
    fi
    echo ""

    # --- Existing ROBOCOB processes ---
    echo -e "${YELLOW}--- 9. Running Processes ---${NC}"
    if systemctl is-active --quiet robocob 2>/dev/null; then
        echo -e "  ROBOCOB service: ${YELLOW}RUNNING (this is fine, won't conflict)${NC}"
    else
        echo -e "  ROBOCOB service: not running"
    fi
    if pgrep -f "roscore\|rosmaster" &>/dev/null; then
        echo -e "  ROS1 master: ${YELLOW}RUNNING${NC}"
    fi
    if sudo docker ps --format '{{.Names}}' 2>/dev/null | head -5; then
        echo "  Running containers shown above"
    fi
    echo ""

    echo -e "${CYAN}============================================${NC}"
    echo -e "${CYAN}  System Check Complete${NC}"
    echo -e "${CYAN}  L4T_TAG = ${L4T_TAG}${NC}"
    echo -e "${CYAN}============================================${NC}"
}

# -----------------------------------------------------------
# STEP 1: Verify Source Code Exists
# -----------------------------------------------------------
verify_src() {
    echo -e "${CYAN}--- Verifying source code ---${NC}"
    if [ -f "$WORKSPACE/src/Dockerfile" ] && [ -d "$WORKSPACE/src/orchestrator" ]; then
        echo -e "  ${GREEN}Source code found at $WORKSPACE/src/${NC}"
        echo "  Packages:"
        for pkg in harvester_interfaces robot_arm robot_arm_moveit_config orchestrator; do
            if [ -d "$WORKSPACE/src/$pkg" ]; then
                echo -e "    ${GREEN}$pkg${NC}"
            else
                echo -e "    ${RED}$pkg MISSING${NC}"
            fi
        done
        echo -e "  Dockerfile: ${GREEN}OK${NC}"
    else
        echo -e "  ${RED}Source code NOT found at $WORKSPACE/src/${NC}"
        echo ""
        echo -e "  ${YELLOW}Transfer the code from your machine first:${NC}"
        echo -e "  ${CYAN}  scp -r harvesting_robot robocob@192.168.3.4:~/harvesting_ws/src${NC}"
        echo ""
        return 1
    fi
}

# -----------------------------------------------------------
# STEP 2: Build Docker Image
# -----------------------------------------------------------
build_image() {
    echo -e "${CYAN}--- Building Docker image ---${NC}"
    cd "$WORKSPACE"

    # Detect L4T tag
    if [ -f /etc/nv_tegra_release ]; then
        L4T_MAJOR=$(head -1 /etc/nv_tegra_release | sed -nE 's/.*R([0-9]+).*/\1/p')
        L4T_MINOR=$(head -1 /etc/nv_tegra_release | sed -nE 's/.*REVISION: ([0-9]+\.[0-9]+).*/\1/p')
        L4T_TAG="r${L4T_MAJOR}.${L4T_MINOR}"
    else
        L4T_TAG="r35.4.1"
    fi

    echo -e "  Using L4T_TAG=${GREEN}${L4T_TAG}${NC}"
    echo -e "  ${YELLOW}This will take 15-30 minutes...${NC}"
    echo ""

    sudo docker build \
        --build-arg L4T_TAG="$L4T_TAG" \
        -t "$IMAGE_NAME" \
        -f src/Dockerfile \
        src/

    echo -e "  ${GREEN}Docker image built: ${IMAGE_NAME}${NC}"
}

# -----------------------------------------------------------
# STEP 3: Run Container
# -----------------------------------------------------------
run_container() {
    echo -e "${CYAN}--- Starting container ---${NC}"

    # Stop existing container if running
    sudo docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

    sudo docker run -it \
        --runtime nvidia \
        --network host \
        --privileged \
        -v /dev:/dev \
        -v "$WORKSPACE":/ros2_ws_host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=$DISPLAY \
        -e ROS_DOMAIN_ID=0 \
        --name "$CONTAINER_NAME" \
        "$IMAGE_NAME" \
        bash
}

# -----------------------------------------------------------
# STEP 4: Run Tests (inside container)
# -----------------------------------------------------------
run_tests() {
    echo -e "${CYAN}--- Running tests inside container ---${NC}"

    sudo docker exec -it "$CONTAINER_NAME" bash -c '
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash 2>/dev/null

        echo "============================================"
        echo "  RoboCot Container Tests"
        echo "============================================"
        echo ""

        # Test 1: ROS2
        echo "--- Test 1: ROS2 CLI ---"
        ros2 --help > /dev/null 2>&1 && echo "  ros2 CLI: OK" || echo "  ros2 CLI: FAIL"
        echo "  ROS_DISTRO: $ROS_DISTRO"
        echo ""

        # Test 2: Our packages
        echo "--- Test 2: Our ROS2 packages ---"
        for pkg in harvester_interfaces robot_arm robot_arm_moveit_config orchestrator; do
            ros2 pkg list 2>/dev/null | grep -q "^${pkg}$" && \
                echo "  $pkg: OK" || echo "  $pkg: MISSING"
        done
        echo ""

        # Test 3: MoveIt
        echo "--- Test 3: MoveIt2 ---"
        ros2 pkg list 2>/dev/null | grep -q "moveit_ros_move_group" && \
            echo "  MoveIt2: OK" || echo "  MoveIt2: MISSING"
        echo ""

        # Test 4: YOLO model
        echo "--- Test 4: YOLO model ---"
        YOLO_PATH=$(ros2 pkg prefix orchestrator 2>/dev/null)/share/orchestrator/models/best.pt
        if [ -f "$YOLO_PATH" ]; then
            echo "  YOLO model: OK ($YOLO_PATH)"
            python3 -c "from ultralytics import YOLO; m=YOLO(\"$YOLO_PATH\"); print(\"  YOLO load: OK, classes:\", m.names)" 2>/dev/null || \
                echo "  YOLO load: FAIL (ultralytics issue)"
        else
            echo "  YOLO model: NOT FOUND at $YOLO_PATH"
        fi
        echo ""

        # Test 5: GPU
        echo "--- Test 5: GPU Access ---"
        python3 -c "
import torch
if torch.cuda.is_available():
    print(f\"  CUDA: OK ({torch.cuda.get_device_name(0)})\")
    print(f\"  Memory: {torch.cuda.get_device_properties(0).total_mem / 1e9:.1f} GB\")
else:
    print(\"  CUDA: NOT AVAILABLE\")
" 2>/dev/null || echo "  PyTorch/CUDA: import failed"
        echo ""

        # Test 6: Network
        echo "--- Test 6: Doosan Network ---"
        ping -c 1 -W 2 192.168.3.5 > /dev/null 2>&1 && \
            echo "  Ping 192.168.3.5: OK" || echo "  Ping 192.168.3.5: FAIL"
        nc -zv -w 2 192.168.3.5 12345 2>&1 | grep -q "succeeded\|open" && \
            echo "  Port 12345: OPEN" || echo "  Port 12345: CLOSED"
        echo ""

        # Test 7: USB / Serial
        echo "--- Test 7: USB Devices ---"
        ls /dev/ttyUSB* /dev/ttyACM* /dev/robotiq_gripper 2>/dev/null || echo "  No serial ports found"
        ls /dev/video* 2>/dev/null || echo "  No video devices found"
        echo ""

        # Test 8: Custom interfaces
        echo "--- Test 8: Custom Interfaces ---"
        ros2 interface show harvester_interfaces/srv/YoloDetect > /dev/null 2>&1 && \
            echo "  YoloDetect srv: OK" || echo "  YoloDetect srv: MISSING"
        ros2 interface show harvester_interfaces/srv/HarvestBoll > /dev/null 2>&1 && \
            echo "  HarvestBoll srv: OK" || echo "  HarvestBoll srv: MISSING"
        echo ""

        echo "============================================"
        echo "  Tests Complete"
        echo "============================================"
    '
}

# -----------------------------------------------------------
# Main
# -----------------------------------------------------------
case "${1:-all}" in
    check)
        check_system
        ;;
    verify)
        verify_src
        ;;
    build)
        verify_src && build_image
        ;;
    run)
        run_container
        ;;
    test)
        run_tests
        ;;
    all)
        check_system
        echo ""
        verify_src || exit 1
        echo ""
        echo -e "${YELLOW}Continue with build + run? (y/n)${NC}"
        read -r answer
        if [ "$answer" = "y" ]; then
            build_image
            run_container
        fi
        ;;
    *)
        echo "Usage: $0 {check|verify|build|run|test|all}"
        exit 1
        ;;
esac
