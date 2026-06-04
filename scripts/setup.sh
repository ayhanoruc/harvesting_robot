#!/usr/bin/env bash
# RoboCot — bring-up installer for a fresh Ubuntu 22.04 (native or WSL2) host.
#
# What this script does (in order):
#   1. Verify the host is Ubuntu 22.04 (Jammy) — anything else is rejected.
#   2. Install ROS 2 Humble (desktop, MoveIt 2, ros2_control, ros_gz, ompl).
#   3. Install Gazebo Ignition Fortress 6.x (pulled as a ros-humble-ros-gz dep).
#   4. Install uv (Python package manager).
#   5. uv pip-install the Python deps in src/pyproject.toml into the system
#      Python (the same one ROS uses).
#   6. colcon build the four workspace packages.
#
# Each step is idempotent — re-running skips work that's already done.
# After it finishes, run scripts/verify_env.sh to confirm the stack.
#
# Usage:
#   bash src/scripts/setup.sh          # full install
#   bash src/scripts/setup.sh check    # OS + dependency probe only, no installs
#   bash src/scripts/setup.sh build    # just (re)build the workspace

set -euo pipefail

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; NC='\033[0m'
log()  { echo -e "${CYAN}==>${NC} $*"; }
ok()   { echo -e "  ${GREEN}OK${NC} $*"; }
warn() { echo -e "  ${YELLOW}WARN${NC} $*"; }
err()  { echo -e "  ${RED}FAIL${NC} $*"; exit 1; }

WS_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"   # .../harvesting_ws
SRC_DIR="$WS_ROOT/src"

# ─────────────────────────── 1. OS check ────────────────────────────────────
check_os() {
    log "OS check"
    [ -f /etc/os-release ] || err "/etc/os-release missing — not a standard Linux."
    . /etc/os-release
    [ "${ID:-}" = "ubuntu" ]      || err "Distro is $ID, need ubuntu."
    [ "${VERSION_ID:-}" = "22.04" ] || err "Ubuntu $VERSION_ID detected, need 22.04."
    ok "Ubuntu 22.04 ($PRETTY_NAME)"
    if grep -qi microsoft /proc/version; then ok "Running in WSL2"; fi
    [ "$(uname -m)" = "x86_64" ] || warn "Arch $(uname -m) — Dockerfile targets aarch64 for Jetson, x86_64 for dev."
}

# ──────────────────────── 2. ROS 2 Humble apt ───────────────────────────────
install_ros() {
    log "ROS 2 Humble"
    if dpkg -s ros-humble-desktop >/dev/null 2>&1; then
        ok "ros-humble-desktop already installed ($(dpkg -s ros-humble-desktop | awk '/^Version/ {print $2}'))"
    else
        sudo apt-get update
        sudo apt-get install -y software-properties-common curl gnupg lsb-release
        sudo add-apt-repository universe -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
             -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
             | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
        sudo apt-get update
        sudo apt-get install -y ros-humble-desktop
        ok "ros-humble-desktop installed"
    fi

    # Packages the pipeline actually needs on top of -desktop.
    APT_PKGS=(
        ros-humble-moveit
        ros-humble-ros2-control
        ros-humble-ros2-controllers
        ros-humble-controller-manager
        ros-humble-joint-state-broadcaster
        ros-humble-joint-trajectory-controller
        ros-humble-xacro
        ros-humble-cv-bridge
        ros-humble-image-geometry
        ros-humble-tf2-ros
        ros-humble-tf2-geometry-msgs
        ros-humble-moveit-planners-ompl
        ros-humble-ompl
        # ros_gz bundle (bridge + sim + image + interfaces) pulls in Fortress (Gazebo Sim 6.x)
        ros-humble-ros-gz
        ros-humble-ros-gz-bridge
        ros-humble-ros-gz-image
        ros-humble-ros-gz-sim
        ros-humble-gz-ros2-control
        python3-colcon-common-extensions
        python3-rosdep
        python3-pip
    )
    MISSING=()
    for p in "${APT_PKGS[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p")
    done
    if [ ${#MISSING[@]} -eq 0 ]; then
        ok "All required apt packages present (${#APT_PKGS[@]} packages)"
    else
        log "Installing ${#MISSING[@]} missing apt packages"
        sudo apt-get install -y "${MISSING[@]}"
        ok "Apt installs done"
    fi
}

# ─────────────────────────── 3. uv ──────────────────────────────────────────
install_uv() {
    log "uv (Python package manager)"
    if command -v uv >/dev/null 2>&1; then
        ok "uv $(uv --version | awk '{print $2}')"
        return
    fi
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
    ok "uv installed to $HOME/.local/bin"
}

# ─────────────────────────── 4. Python deps ─────────────────────────────────
install_python_deps() {
    log "Python deps (from src/pyproject.toml into system Python 3.10)"
    [ -f "$SRC_DIR/pyproject.toml" ] || err "$SRC_DIR/pyproject.toml not found"
    # --break-system-packages: Ubuntu 22.04 still permits system-pip but newer
    # uv versions echo PEP-668 — pass the flag explicitly.
    uv pip install -r "$SRC_DIR/pyproject.toml" \
        --python /usr/bin/python3 \
        --break-system-packages
    ok "Python deps installed (run scripts/verify_env.sh to confirm import works)"
    warn "torch installed above is CPU-only. For CUDA on a dev GPU:"
    warn "  uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128 \\"
    warn "      --python /usr/bin/python3 --break-system-packages --upgrade"
}

# ─────────────────────────── 5. colcon build ────────────────────────────────
colcon_build() {
    log "colcon build"
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    cd "$WS_ROOT"
    colcon build \
        --packages-select harvester_interfaces robot_arm robot_arm_moveit_config orchestrator \
        --symlink-install
    ok "Workspace built. Source it with:"
    ok "  source $WS_ROOT/install/setup.bash"
}

# ─────────────────────────── 5b. shell hooks ────────────────────────────────
install_shell_hooks() {
    log "Shell hooks (~/.bashrc)"
    local marker='# >>> robocot setup.sh >>>'
    if grep -qF "$marker" "$HOME/.bashrc" 2>/dev/null; then
        ok "Hooks already in ~/.bashrc"
        return
    fi
    cat >> "$HOME/.bashrc" <<EOF

$marker
source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
eval "\$(register-python-argcomplete3 colcon)" 2>/dev/null
# WSL2-only: software GL fallback for RViz / Gazebo when Mesa misbehaves.
grep -qi microsoft /proc/version 2>/dev/null && export LIBGL_ALWAYS_SOFTWARE=1
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
# <<< robocot setup.sh <<<
EOF
    ok "Appended to ~/.bashrc — open a new shell or 'source ~/.bashrc'"
}

# ─────────────────────────── 6. Probe ───────────────────────────────────────
probe() {
    log "Probe: current versions"
    [ -f /etc/os-release ] && (. /etc/os-release && echo "  $PRETTY_NAME") || true
    grep -qi microsoft /proc/version && echo "  WSL2 kernel: $(uname -r)" || echo "  Native Linux: $(uname -r)"
    echo "  CPU: $(lscpu | awk -F: '/Model name/ {print $2; exit}' | sed 's/^ *//')"
    echo "  RAM: $(awk '/MemTotal/ {printf "%.1f GiB", $2/1024/1024}' /proc/meminfo)"
    command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader | sed 's/^/  GPU: /' || echo "  GPU: (no nvidia-smi)"
    if [ -x /opt/ros/humble/bin/ros2 ]; then echo "  ros2:      /opt/ros/humble/bin/ros2 (ROS_DISTRO=${ROS_DISTRO:-not sourced})"
    else echo "  ros2:      not installed"; fi
    dpkg -s ros-humble-moveit >/dev/null 2>&1 && echo "  moveit:    $(dpkg -s ros-humble-moveit  | awk '/^Version/ {print $2}')" || true
    dpkg -s ros-humble-ros-gz >/dev/null 2>&1 && echo "  ros_gz:    $(dpkg -s ros-humble-ros-gz | awk '/^Version/ {print $2}')" || true
    command -v ign >/dev/null && echo "  gz sim:    $(ign gazebo --version 2>&1 | head -1)" || true
    command -v python3 >/dev/null && echo "  python:    $(python3 --version | awk '{print $2}')" || true
    command -v uv >/dev/null && echo "  uv:        $(uv --version | awk '{print $2}')" || echo "  uv: not installed"
}

# ─────────────────────────── main ───────────────────────────────────────────
case "${1:-all}" in
    check)
        check_os; probe
        ;;
    build)
        colcon_build
        ;;
    all)
        check_os
        install_ros
        install_uv
        install_python_deps
        colcon_build
        install_shell_hooks
        log "All done. Verify with:"
        echo "    bash $SRC_DIR/scripts/verify_env.sh"
        ;;
    *)
        echo "Usage: $0 {all|check|build}"
        exit 1
        ;;
esac
