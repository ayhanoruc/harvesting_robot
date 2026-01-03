# Gazebo Classic to Gazebo Sim Migration Guide

## Overview

**Date**: January 2026
**Current Setup**: Gazebo Classic 11.10.2 + ROS2 Humble
**Target**: Gazebo Sim (Fortress or Harmonic)
**Reason**: Gazebo Classic reached EOL January 2025, plus native GLB/GLTF support needed for cotton cluster models

---

## Background

### Naming History
- **Gazebo Classic**: Numbered releases (9, 11) - OLD, EOL
- **Ignition Gazebo**: Intermediate naming (Citadel, Fortress, etc.)
- **Gazebo Sim**: Current naming (just "Gazebo" with letter releases)

### Key Architecture Change
> "The new Gazebo shifts from a monolithic architecture to a collection of loosely coupled libraries"

- `gazebo_ros_pkgs` → `ros_gz` (bridge-based communication)
- Plugins run inside Gazebo, but ROS communication goes through bridges

---

## Version Compatibility Matrix

| Gazebo Version | ROS2 Humble Support | Install Method | GLB/GLTF | EOL |
|----------------|---------------------|----------------|----------|-----|
| Fortress | Official | `apt install` | Yes | Sep 2026 |
| Harmonic | Unofficial | Source build | Better | Sep 2028 |

**Recommendation**: Start with **Fortress** (simpler), upgrade to Harmonic later if needed.

---

## Current RoboCot Setup Analysis

### Files to Migrate

| File | Location | Changes Needed |
|------|----------|----------------|
| `package.xml` | `src/robot_arm/` | Replace dependencies |
| `bot.launch.py` | `src/robot_arm/launch/` | New launch structure |
| `cotton_field.world` | `src/robot_arm/worlds/` | World plugins + Fuel models |
| `mybot.urdf.xacro` | `src/robot_arm/urdf/` | Camera + ros2_control plugins |
| `bridge.yaml` | `src/robot_arm/config/` | **NEW**: Topic bridge config |
| `CMakeLists.txt` | `src/robot_arm/` | Install bridge.yaml |

### Current Dependencies (package.xml)
```xml
<!-- OLD - Gazebo Classic -->
<depend>gazebo_ros</depend>
<depend>gazebo_ros2_control</depend>
```

### Current Launch (bot.launch.py)
```python
# OLD - Gazebo Classic
pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
    ),
    launch_arguments={'world': world_file}.items()
)

spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description', '-entity', 'robot_arm']
)
```

### Current World (cotton_field.world)
```xml
<!-- OLD - Uses model:// URI -->
<include>
  <uri>model://sun</uri>
</include>
```

### Current URDF Plugins (mybot.urdf.xacro)
```xml
<!-- OLD - Camera plugin -->
<plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
  <ros>
    <namespace>/camera</namespace>
    <remapping>image_raw:=color/image_raw</remapping>
  </ros>
  <frame_name>camera_optical_frame</frame_name>
</plugin>

<!-- OLD - ros2_control hardware -->
<plugin>gazebo_ros2_control/GazeboSystem</plugin>

<!-- OLD - Control plugin -->
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
```

---

## Migration Tasks

### Task 1: Install Gazebo Fortress

```bash
# In WSL Ubuntu 22.04
sudo apt-get update
sudo apt-get install ros-humble-ros-gz ros-humble-gz-ros2-control

# Verify - Fortress uses "ign" command, not "gz"
ign gazebo --version
# Should show: Ignition Gazebo, version 6.x.x

# Test GUI
ign gazebo shapes.sdf
```

**IMPORTANT**: Fortress (Ignition) uses `ign gazebo` command, NOT `gz sim`.
The `ros_gz_sim` package handles this internally.

**Note**: Gazebo Classic 11 may still be installed. They can coexist but you should
use one or the other in your launch files, not both.

---

### Task 2: Update package.xml

```xml
<!-- NEW - Gazebo Sim -->
<depend>ros_gz_bridge</depend>
<depend>ros_gz_image</depend>
<depend>ros_gz_sim</depend>
<depend>gz_ros2_control</depend>

<!-- REMOVE -->
<!-- <depend>gazebo_ros</depend> -->
<!-- <depend>gazebo_ros2_control</depend> -->

<!-- REMOVE from export (no longer needed) -->
<!-- <gazebo_ros gazebo_model_path="${prefix}/.."/> -->
```

---

### Task 3: Update bot.launch.py

```python
# NEW - Gazebo Sim launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('robot_arm')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_path, 'worlds', 'cotton_field.world')
    controllers_yaml = os.path.join(pkg_path, 'yaml', 'controllers.yaml')
    bridge_params = os.path.join(pkg_path, 'config', 'bridge.yaml')

    # Process xacro
    xacro_file = os.path.join(pkg_path, 'urdf', 'mybot.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_file,
        mappings={'controllers_yaml': controllers_yaml}
    ).toxml()

    # Clean up URDF (same as before)
    import re
    robot_description_content = re.sub(r'<\?xml.*?\?>', '', robot_description_content)
    robot_description_content = re.sub(r'<!--.*?-->', '', robot_description_content, flags=re.DOTALL)
    robot_description_content = re.sub(r'\s+', ' ', robot_description_content).strip()

    robot_description = {'robot_description': robot_description_content}
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set GZ_SIM_RESOURCE_PATH for model discovery
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_path, 'models')
    )

    # NEW: Gazebo Sim launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # NEW: Spawn with ros_gz_sim/create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Controller spawners (same as before)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    # NEW: ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # NEW: Image bridge (more efficient for camera)
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/color/image_raw'],
        output='screen'
    )

    # Event handlers for sequencing
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Landmark publisher
    config_file = os.path.join(pkg_path, 'config', 'environment_config.yaml')
    landmark_publisher = Node(
        package='robot_arm',
        executable='landmark_publisher.py',
        parameters=[{'use_sim_time': use_sim_time}, {'config_file': config_file}]
    )

    delay_landmark_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[landmark_publisher],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        set_env_vars,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        delay_landmark_publisher,
    ])
```

---

### Task 4: Update cotton_field.world

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="cotton_field">

    <!-- Required world plugins for Ignition Fortress -->
    <!-- NOTE: Fortress uses "ignition-gazebo-*" naming, Harmonic uses "gz-sim-*" -->
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu">
    </plugin>

    <!-- Sun (inline light instead of Fuel for offline use) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane (inline, same as before) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.4 0.3 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Plants and other models remain the same -->
    <!-- ... (copy plant_1, plant_2, plant_3, reservoir, markers from original) ... -->

  </world>
</sdf>
```

---

### Task 5: Update mybot.urdf.xacro

#### Camera Sensor (replace plugin-based with generic sensor)

```xml
<!-- OLD -->
<gazebo reference="camera_link">
  <sensor type="depth" name="wrist_rgbd_camera">
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <!-- ROS-specific config -->
    </plugin>
  </sensor>
</gazebo>

<!-- NEW - Generic sensor, ROS communication via bridge -->
<!-- NOTE: Fortress uses "ignition_frame_id", Harmonic uses "gz_frame_id" -->
<gazebo reference="camera_link">
  <sensor type="rgbd_camera" name="wrist_rgbd_camera">
    <always_on>true</always_on>
    <update_rate>30.0</update_rate>
    <topic>camera</topic>
    <ignition_frame_id>camera_optical_frame</ignition_frame_id>
    <camera name="wrist_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3.0</far>
      </clip>
      <depth_camera>
        <clip>
          <near>0.05</near>
          <far>3.0</far>
        </clip>
      </depth_camera>
    </camera>
  </sensor>
</gazebo>
```

#### ros2_control Hardware Interface

```xml
<!-- OLD -->
<ros2_control name="robot_arm_control" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <!-- joints... -->
</ros2_control>

<!-- NEW - Fortress uses "ign_ros2_control", Harmonic uses "gz_ros2_control" -->
<ros2_control name="robot_arm_control" type="system">
  <hardware>
    <plugin>ign_ros2_control/IgnitionSystem</plugin>
  </hardware>
  <!-- joints remain the same -->
</ros2_control>
```

#### Control Plugin

```xml
<!-- OLD -->
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(arg controllers_yaml)</parameters>
  </plugin>
</gazebo>

<!-- NEW - Fortress naming -->
<gazebo>
  <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">
    <parameters>$(arg controllers_yaml)</parameters>
  </plugin>
</gazebo>
```

#### Material References

```xml
<!-- OLD -->
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>
</gazebo>

<!-- NEW - Use PBR materials or basic colors -->
<gazebo reference="base_link">
  <visual>
    <material>
      <ambient>0.5 0.5 0.5 1</ambient>
      <diffuse>0.7 0.7 0.7 1</diffuse>
    </material>
  </visual>
</gazebo>
```

---

### Task 6: Create gz_bridge.yaml

Create `src/robot_arm/config/gz_bridge.yaml`:

```yaml
# ROS-Gazebo Bridge Configuration for Ignition Fortress
# NOTE: Fortress uses "ignition.msgs.*", Harmonic uses "gz.msgs.*"

# Clock (required for use_sim_time)
- ros_topic_name: "clock"
  gz_topic_name: "clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "ignition.msgs.Clock"
  direction: GZ_TO_ROS

# Camera info (color)
- ros_topic_name: "camera/color/camera_info"
  gz_topic_name: "camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "ignition.msgs.CameraInfo"
  direction: GZ_TO_ROS

# Camera info (depth) - same as color
- ros_topic_name: "camera/depth/camera_info"
  gz_topic_name: "camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "ignition.msgs.CameraInfo"
  direction: GZ_TO_ROS

# NOTE: Image topics handled by ros_gz_image bridge for efficiency
# Joint states handled by ign_ros2_control plugin directly
```

---

### Task 7: Update CMakeLists.txt

Add bridge.yaml to install:

```cmake
install(DIRECTORY
  launch
  urdf
  worlds
  yaml
  config  # Add this if not already present
  DESTINATION share/${PROJECT_NAME}
)
```

---

## GLB/GLTF Model Support

With Gazebo Sim, you can directly use GLB models:

```xml
<model name="cotton_cluster">
  <static>true</static>
  <pose>0.75 0 0.5 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://cotton_models/Object0.glb</uri>
          <scale>0.01 0.01 0.01</scale>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
```

Or using Fuel URI:
```xml
<uri>https://fuel.gazebosim.org/1.0/username/models/cotton_cluster</uri>
```

---

## Verification Checklist

After migration, verify:

- [ ] `gz sim --version` shows Fortress (6.x) or Harmonic (8.x)
- [ ] World loads without plugin errors
- [ ] Robot spawns correctly
- [ ] `ros2 topic list` shows `/clock`, `/joint_states`, `/camera/*`
- [ ] Controllers load (`ros2 control list_controllers`)
- [ ] Camera publishes images (`ros2 topic hz /camera/color/image_raw`)
- [ ] Depth works (`ros2 topic echo /camera/depth/image_raw --once`)
- [ ] TF tree is complete (`ros2 run tf2_tools view_frames`)

---

## Troubleshooting

### Common Issues

1. **"Model not found"**
   - Set `GZ_SIM_RESOURCE_PATH` in launch file
   - Use full Fuel URIs instead of `model://`

2. **"Plugin not found"**
   - Check plugin filename (e.g., `gz-sim-physics-system` not `libgazebo_ros...`)
   - Verify `ros-humble-gz-ros2-control` is installed

3. **"No /clock topic"**
   - Add clock to bridge.yaml
   - Ensure bridge node is running

4. **Camera not publishing**
   - Add `Sensors` plugin to world with `<render_engine>ogre2</render_engine>`
   - Check bridge is configured for camera topics

5. **Controller manager not starting**
   - Verify `gz_ros2_control-system` plugin in URDF
   - Check `GzSimSystem` hardware interface

---

## References

- [Official Migration Guide](https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/)
- [ros_gz GitHub](https://github.com/gazebosim/ros_gz)
- [gz_ros2_control Docs](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)
- [Gazebo Sim Systems List](https://gazebosim.org/api/sim/8/namespacegz_1_1sim_1_1systems.html)
