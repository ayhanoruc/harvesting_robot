#!/usr/bin/env python3
"""
Script to restructure cotton_cluster model for Gazebo Sim compatibility.
Creates proper model.config, model.sdf, and organizes mesh files.
"""

import os
import shutil

# Paths
BASE_DIR = r"C:\Users\ayhan\harvesting_ws\src\robot_arm\models\cotton_cluster"
MESHES_DIR = os.path.join(BASE_DIR, "meshes")

# Source directories (where DAE files currently are)
SRC_OBJ0 = os.path.join(BASE_DIR, "ImageToStl.com_object_0")
SRC_OBJ1 = os.path.join(BASE_DIR, "ImageToStl.com_object_1")

# Target directories
DST_OBJ0 = os.path.join(MESHES_DIR, "object_0")
DST_OBJ1 = os.path.join(MESHES_DIR, "object_1")

def create_directories():
    """Create the meshes directory structure."""
    os.makedirs(DST_OBJ0, exist_ok=True)
    os.makedirs(DST_OBJ1, exist_ok=True)
    print(f"Created: {MESHES_DIR}")
    print(f"Created: {DST_OBJ0}")
    print(f"Created: {DST_OBJ1}")

def copy_mesh_files():
    """Copy DAE and texture files to new locations."""
    # Object 0
    for filename in ["object_0.dae", "image0.png"]:
        src = os.path.join(SRC_OBJ0, filename)
        dst = os.path.join(DST_OBJ0, filename)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"Copied: {src} -> {dst}")
        else:
            print(f"WARNING: Source not found: {src}")

    # Object 1
    for filename in ["object_1.dae", "image0.png"]:
        src = os.path.join(SRC_OBJ1, filename)
        dst = os.path.join(DST_OBJ1, filename)
        if os.path.exists(src):
            shutil.copy2(src, dst)
            print(f"Copied: {src} -> {dst}")
        else:
            print(f"WARNING: Source not found: {src}")

def create_model_config():
    """Create model.config file."""
    content = '''<?xml version="1.0"?>
<model>
  <name>cotton_cluster</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>RoboCot Project</name>
  </author>
  <description>
    Cotton cluster meshes for harvesting simulation.
    Contains two mesh variants (object_0 and object_1).
  </description>
</model>
'''
    filepath = os.path.join(BASE_DIR, "model.config")
    with open(filepath, 'w') as f:
        f.write(content)
    print(f"Created: {filepath}")

def create_model_sdf():
    """Create model.sdf file - a simple static model with one mesh variant."""
    content = '''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="cotton_cluster">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>meshes/object_0/object_0.dae</uri>
            <scale>0.5 0.5 0.5</scale>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
'''
    filepath = os.path.join(BASE_DIR, "model.sdf")
    with open(filepath, 'w') as f:
        f.write(content)
    print(f"Created: {filepath}")

def main():
    print("=" * 60)
    print("Setting up cotton_cluster model for Gazebo Sim")
    print("=" * 60)

    # Check if source directories exist
    if not os.path.exists(SRC_OBJ0):
        print(f"ERROR: Source directory not found: {SRC_OBJ0}")
        return
    if not os.path.exists(SRC_OBJ1):
        print(f"ERROR: Source directory not found: {SRC_OBJ1}")
        return

    print("\n1. Creating directories...")
    create_directories()

    print("\n2. Copying mesh files...")
    copy_mesh_files()

    print("\n3. Creating model.config...")
    create_model_config()

    print("\n4. Creating model.sdf...")
    create_model_sdf()

    print("\n" + "=" * 60)
    print("Done! New structure:")
    print("=" * 60)

    # List the new structure
    for root, dirs, files in os.walk(BASE_DIR):
        level = root.replace(BASE_DIR, '').count(os.sep)
        indent = '  ' * level
        print(f"{indent}{os.path.basename(root)}/")
        sub_indent = '  ' * (level + 1)
        for file in files:
            print(f"{sub_indent}{file}")

if __name__ == "__main__":
    main()
