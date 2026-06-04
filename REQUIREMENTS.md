# Requirements & Bring-up

Two host profiles are supported:

| Profile | Use | OS / arch |
|---|---|---|
| **Dev** (this repo's primary target) | Local development + Gazebo Fortress sim | Ubuntu 22.04 on x86_64 (native or WSL2) |
| **Deploy** (future field hardware) | Onboard compute next to the Doosan controller | Ubuntu 20.04 / JetPack 5.x on a Jetson AGX Xavier (aarch64) — runs the whole stack in a container |

The dev profile is what you bring up to run the demo and develop against. The deploy profile is documented in [INTEGRATION.md](docs/INTEGRATION.md#further-notes) and built via the [Dockerfile](Dockerfile); it's out of scope for this document beyond the version pins.

---

## Hardware

### Dev (verified)
The pipeline was developed and tested on:

| Component | Value |
|---|---|
| CPU | Intel Core i5-11400H (6C/12T, 2.7 GHz) |
| RAM | 16 GiB |
| GPU | NVIDIA GeForce GTX 1650 (4 GiB VRAM), driver 560.94 |
| Disk free | ≥20 GiB (workspace + Gazebo cache) |
| OS | Windows 11 + WSL2 (Ubuntu 22.04.5 LTS, kernel 6.6.x) |

WSL2-specific: GUI works through WSLg (no XServer required); CUDA passthrough works for YOLO inference. Gazebo Fortress runs in software-render mode under WSL2 (~5% real-time on this CPU/GPU class) — that is the speed bottleneck for the demo.

### Minimum sensible
4-core CPU, 8 GiB RAM, 20 GiB disk. Gazebo Fortress soft-renders the world even without a GPU; YOLO inference falls back to CPU at ~10x slower. A discrete NVIDIA GPU (≥4 GiB) lets `real_yolo_detector` keep up with the camera frame rate.

### Deploy target (Jetson AGX Xavier)
JetPack 5.x (L4T r35.x), 16 GiB unified memory, 512 CUDA cores. See [`Dockerfile`](Dockerfile) for the matching image tags and [`scripts/xavier_deploy.sh`](scripts/xavier_deploy.sh) for the field bring-up.

---

## Software stack (versions used)

ROS, MoveIt, Gazebo, and ros2_control are pinned to whatever the `apt install ros-humble-*` set resolves on Ubuntu 22.04 today — there's no pip-style version constraint. The numbers below are what `dpkg -s` reports on the verified dev machine; treat them as "tested with", not "required exactly".

| Layer | Component | Version |
|---|---|---|
| OS | Ubuntu | **22.04** (Jammy Jellyfish) |
| OS | WSL2 kernel | 6.6.87.2-microsoft-standard-WSL2 (WSL 2.5.10.0, WSLg 1.0.66) |
| Middleware | ROS 2 Humble (`ros-humble-desktop`) | 0.10.0 |
| Simulator | Gazebo Sim (Ignition Fortress) | 6.16.0 |
| ROS↔Gazebo | `ros-humble-ros-gz` (bridge, image, sim, interfaces) | 0.244.20 |
| Hardware iface | `ros-humble-gz-ros2-control` | 0.7.17 |
| Hardware iface | `ros-humble-ros2-control` | 2.52.2 |
| Hardware iface | `ros-humble-joint-trajectory-controller` | 2.50.2 |
| Hardware iface | `ros-humble-joint-state-broadcaster` | 2.50.2 |
| Motion planning | `ros-humble-moveit` | 2.5.9 |
| Motion planning | `ros-humble-moveit-planners-ompl`, `ros-humble-ompl` | 1.7.0 |
| TF / sensors | `ros-humble-tf2*`, `ros-humble-cv-bridge`, `ros-humble-image-geometry` | 0.25.16 / 3.2.1 / 3.2.1 |
| Build | `python3-colcon-common-extensions`, `colcon-core` | 0.20.1 |
| Python | system Python 3 | **3.10.12** |
| Python mgr | uv | 0.9.21 |
| Vision | ultralytics (YOLO11) | 8.3.248 |
| Vision | torch / torchvision | 2.9.1+cu128 / 0.24.1 |
| Vision | opencv-python | 4.9.0.80 |
| UI | PyQt5 | 5.15.6 |
| Hardware driver | pymodbus (deploy only — real Hand-E) | ≥3 |

Full apt list installed by the bring-up script is in [`scripts/setup.sh`](scripts/setup.sh) under `install_ros()`. Full pip list is in [`pyproject.toml`](pyproject.toml).

---

## Bring-up

### One-shot installer (recommended)

On a fresh Ubuntu 22.04 host (or a fresh WSL2 distro), from the repo root:

```bash
bash src/scripts/setup.sh        # full: OS check → ROS apt → uv → pip deps → colcon → shell hooks
bash src/scripts/setup.sh check  # OS + dependency probe only, no installs
bash src/scripts/setup.sh build  # (re)build the workspace only
```

What the full path does, in order:

1. Reject anything that isn't Ubuntu 22.04.
2. `apt install` ROS 2 Humble desktop plus the exact set of `moveit`, `ros2_control`, `joint-*-controller`, `ros-gz-*`, `gz-ros2-control`, `tf2_*`, `cv-bridge`, `image-geometry`, `ompl`, and `colcon` packages the pipeline imports.
3. Install [uv](https://docs.astral.sh/uv/) via Astral's install script.
4. `uv pip install -r src/pyproject.toml --python /usr/bin/python3` — installs `ultralytics`, `torch`, `opencv-python`, `PyYAML`, `Pillow`, `matplotlib`, `scipy`, `PyQt5`, `pymodbus`, `numpy<2` into the system Python that ROS uses.
5. `colcon build --symlink-install` for the four workspace packages.
6. Append to `~/.bashrc`: ROS source, workspace overlay source, colcon autocomplete, `LIBGL_ALWAYS_SOFTWARE=1` (WSL2 only), `ROS_DOMAIN_ID=0`.

CUDA torch isn't pulled by default (PyPI torch is CPU-only). After the script finishes, if you have an NVIDIA GPU:

```bash
uv pip install torch torchvision \
    --index-url https://download.pytorch.org/whl/cu128 \
    --python /usr/bin/python3 --break-system-packages --upgrade
```

### Manual install (reference)

If you want to walk through it yourself — WSL2 + Ubuntu install, the ROS apt key dance, colcon autocomplete, workspace layout, RViz GL flag — the original commentary lives in [`docs/ROS2_starter.md`](docs/ROS2_starter.md). That document is the human prose; `scripts/setup.sh` is its codified, idempotent form.

### Verify

```bash
bash src/scripts/verify_env.sh
```

Reports per category: ROS CLI, custom packages, Gazebo, MoveIt, ros2_control, ultralytics + cv_bridge + torch, ros_gz bridges, custom interfaces, URDF parse, build tools.

### Run

After verify is clean (one-time, or after every new shell):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run orchestrator control_panel
```

The control panel auto-launches the sim. See [REPO_STRUCTURE.md](REPO_STRUCTURE.md#running-the-active-demo) for the four-terminal equivalent.

---

## Python deps (uv-managed)

[`pyproject.toml`](pyproject.toml) is the source of truth for the non-apt Python dependencies. It lists what the orchestrator's Python nodes import that ROS doesn't supply via `apt install ros-humble-*`. There is no managed `.venv` — `uv pip install -r pyproject.toml --python /usr/bin/python3` installs into the same system Python ROS uses.

To add a new dep:

1. Add it under `[project].dependencies` in `pyproject.toml`.
2. `uv pip install -r src/pyproject.toml --python /usr/bin/python3 --break-system-packages`.

The `analysis` extra (`pandas`, `seaborn`, `tqdm`) is for log-parsing / figure scripts under `docs/figures/` and `docs/parse_harvest_log.py`; install with `uv pip install -r src/pyproject.toml --extra analysis ...`.

---

## Runtime gotchas worth knowing

- **WSL2 + RViz / Gazebo render**: software GL is the only reliable path. `LIBGL_ALWAYS_SOFTWARE=1` is exported by the bashrc hook on WSL2 hosts. Without it, RViz or Gazebo can segfault on startup with `MESA-LOADER` errors.
- **Shell sourcing order**: ROS overlay (`/opt/ros/humble/setup.bash`) **before** the workspace overlay (`install/setup.bash`). `setup.sh` writes them in that order.
- **`ROS_DOMAIN_ID=0`**: hard-coded by the bashrc hook so multiple distros / containers on the same machine don't collide. Change it if you need isolation.
- **MoveIt planning_scene staleness**: the DiffDrive plugin doesn't publish wheel `/joint_states`, so `planning_scene_monitor` logs `The complete state of the robot is not yet known` indefinitely. This is harmless for the active pipeline — `arm_commander` uses direct `JointTrajectory` publishes for the moves where staleness would matter (see [arm_commander.py](robot_arm_moveit_config/robot_arm_moveit_config/arm_commander.py)).
- **YOLO output dir**: `real_yolo_detector` and `cluster_scanner` write annotated PNGs to `/mnt/c/Users/ayhan/harvesting_ws/yolo_output` by default. Override via the `output_dir` ROS param or the `YOLO_OUTPUT_DIR` env var.
- **numpy 1.x pin**: `ros-humble-cv-bridge` and `rclpy` on Humble were built against the numpy 1.x ABI. Upgrading to numpy 2.x breaks imports — keep the `numpy<2` pin in `pyproject.toml`.

---

## Adding a new robot or world

URDF authoring is out of scope for the installer but documented in [`docs/ROS2_starter.md`](docs/ROS2_starter.md) (SolidWorks `sw_urdf_exporter`, Onshape browser exporter). The repo-side integration steps are in [REPO_STRUCTURE.md](REPO_STRUCTURE.md#plug-and-play-extension-points).
