#!/usr/bin/env python3
"""Copy cotton DAE into car model structure and update model.sdf"""

import shutil
import os

BASE = r"C:\Users\ayhan\harvesting_ws\src\robot_arm\models\cotton_cluster"

# Source files
src_dae = os.path.join(BASE, "meshes", "object_0", "object_0.dae")
src_png = os.path.join(BASE, "meshes", "object_0", "image0.png")

# Destination (car meshes folder)
dst_dir = os.path.join(BASE, "car", "car", "meshes")
dst_dae = os.path.join(dst_dir, "cotton.dae")
dst_png = os.path.join(dst_dir, "image0.png")

print("Copying files...")
shutil.copy2(src_dae, dst_dae)
print(f"  {src_dae} -> {dst_dae}")
shutil.copy2(src_png, dst_png)
print(f"  {src_png} -> {dst_png}")

# Update model.sdf to use cotton.dae instead of car.dae
model_sdf_path = os.path.join(BASE, "car", "car", "model.sdf")

new_sdf = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="car">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <scale>0.5 0.5 0.5</scale>
            <uri>model://car/meshes/cotton.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
'''

with open(model_sdf_path, 'w') as f:
    f.write(new_sdf)
print(f"Updated: {model_sdf_path}")

print("\nDone! Now run:")
print("export IGN_GAZEBO_RESOURCE_PATH=/mnt/c/Users/ayhan/harvesting_ws/src/robot_arm/models/cotton_cluster/car:$IGN_GAZEBO_RESOURCE_PATH")
print("ign gazebo /mnt/c/Users/ayhan/harvesting_ws/src/robot_arm/worlds/test_car.world")
