# =============================================================
# RoboCot Docker Image for Jetson AGX Xavier (JetPack 5.x)
# =============================================================
#
# This Dockerfile builds our ROS2 Humble stack on Xavier's aarch64.
# Base image: dusty-nv's pre-built ROS2 Humble for L4T.
#
# IMPORTANT: The base image tag MUST match your JetPack version.
# Check with: cat /etc/nv_tegra_release
#   JetPack 5.1.0 -> r35.2.1
#   JetPack 5.1.1 -> r35.3.1
#   JetPack 5.1.2 -> r35.4.1
#   JetPack 5.0.2 -> r35.1.0
#
# Default: r35.4.1 (most common). Override with:
#   docker build --build-arg L4T_TAG=r35.3.1 -t robocot .
# =============================================================

ARG L4T_TAG=r35.4.1
FROM dustynv/ros:humble-desktop-l4t-${L4T_TAG}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---- System dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 packages
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-ros-visualization \
    ros-humble-rviz2 \
    # Build tools
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    nano \
    iputils-ping \
    net-tools \
    netcat-openbsd \
    # Serial/USB for Hand-E
    python3-serial \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ---- Python dependencies (YOLO + vision) ----
# NOTE: dustynv base image ships PyTorch pre-built for Jetson CUDA.
# ultralytics pulls torch as a dependency — --no-deps avoids overriding
# the Jetson-optimized torch with a CPU-only version from PyPI.
# We install ultralytics deps manually instead.
RUN pip3 install --no-cache-dir \
    ultralytics --no-deps && \
    pip3 install --no-cache-dir \
    opencv-python-headless \
    numpy \
    PyYAML \
    pymodbus \
    matplotlib \
    pandas \
    tqdm \
    scipy \
    seaborn \
    Pillow \
    psutil \
    py-cpuinfo \
    setuptools==58.2.0

# ---- Workspace setup ----
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# ---- Copy our source code ----
# Copy only ROS2 packages (not docs/research/remotion/build/install)
COPY harvester_interfaces/ /ros2_ws/src/harvester_interfaces/
COPY robot_arm/ /ros2_ws/src/robot_arm/
COPY robot_arm_moveit_config/ /ros2_ws/src/robot_arm_moveit_config/
COPY orchestrator/ /ros2_ws/src/orchestrator/
COPY scripts/ /ros2_ws/src/scripts/

# ---- Clone Doosan ROS2 driver ----
RUN cd /ros2_ws/src && \
    git clone https://github.com/doosan-robotics/doosan-robot2.git 2>/dev/null || true

# ---- Build workspace ----
RUN source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install \
        --packages-select harvester_interfaces && \
    source install/setup.bash && \
    colcon build --symlink-install \
        --packages-select robot_arm robot_arm_moveit_config orchestrator 2>&1 | tail -20 && \
    echo "=== Our packages built ==="

# NOTE: doosan-robot2 may fail to build due to missing deps on aarch64.
# That's OK for initial testing. Build it separately if needed:
#   colcon build --packages-select dsr_bringup2 dsr_control2

# ---- Shell setup ----
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash 2>/dev/null' >> /root/.bashrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc && \
    echo '' >> /root/.bashrc && \
    echo '# RoboCot aliases' >> /root/.bashrc && \
    echo 'alias cb="cd /ros2_ws && colcon build --symlink-install"' >> /root/.bashrc && \
    echo 'alias src="source /ros2_ws/install/setup.bash"' >> /root/.bashrc && \
    echo 'alias ping-doosan="ping -c 3 192.168.3.5"' >> /root/.bashrc && \
    echo 'alias nc-doosan="nc -zv 192.168.3.5 12345"' >> /root/.bashrc

# ---- Entrypoint ----
COPY scripts/docker_entrypoint.sh /ros2_ws/docker_entrypoint.sh
RUN chmod +x /ros2_ws/docker_entrypoint.sh 2>/dev/null || true

CMD ["/bin/bash"]
