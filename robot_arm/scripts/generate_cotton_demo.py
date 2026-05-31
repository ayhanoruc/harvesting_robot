#!/usr/bin/env python3
"""
Generate `cotton_demo.world` + `cotton_demo_bolls.yaml` from Deniz's bundle
USING THE BUNDLE'S NATIVE BOLL POSITIONS.

Why this version
----------------
Earlier attempt: I extracted ONE cluster's 4 socket positions and reused them
for every instance, then scaled by plant_scale. That produced bolls that
floated AROUND clusters but didn't FIT the actual sepal cups — because each
cluster in the bundle has its own unique socket positions (3-5 per cluster,
41 total across 12 clusters), and reusing one cluster's offsets doesn't
match the geometry of the others.

Correct approach (this script):
  1. Read bundle's config/cotton_targets.yaml — 41 boll positions calibrated
     to fit inside each cluster's sepal cup in the baked monolithic mesh.
  2. Scale EVERYTHING uniformly (plant mesh + boll positions + boll mesh):
     bolls stay locked to their cluster sockets at any scale.
  3. Each boll is grouped to its nearest cluster anchor (one of 12) for
     `tree_id` so simple_cluster_harvester can pick per-cluster.

Outputs
  - worlds/cotton_demo.world           (cotton_orchard_clean + 41 fitted bolls)
  - config/cotton_demo_bolls.yaml      (harvester inventory, items[] schema)

Notes
  - Bundle anchors (scale 1): X ∈ [15.4, 29.7], Y ∈ [7.6, 9.2].
    At SCALE=4 → X ∈ [61.6, 118.8], Y ∈ [30.3, 37.0].
  - Aisle Y center at scale 4: ≈ 33.6.
  - User wanted plants at 'our tree scale' (~2 m) → SCALE 4 makes plant
    height ~2 m (matches our orchard tree heights).
  - Cluster spacing in X (12 m at scale 4) is fixed by the bundle's baked
    mesh; we can't tighten without splitting the DAE per cluster. Use a
    smaller SCALE (--scale 2 → 1 m plants, 6 m X-spacing) for a tighter demo.
"""

from __future__ import annotations

import argparse
import math
import os
import yaml

SCRIPT_DIR        = os.path.dirname(os.path.abspath(__file__))
PKG               = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
BUNDLE_TARGETS    = os.path.join(PKG, 'config', 'cotton_targets_bundle.yaml')
BOLLS_YML         = os.path.join(PKG, 'config', 'cotton_demo_bolls.yaml')
WORLD_OUT         = os.path.join(PKG, 'worlds', 'cotton_demo.world')

# 12 branch_main anchor positions at scale 1 (from inspect_cluster_layout.py)
ANCHORS_SCALE1 = {
    'cluster_A_01': (15.40, 7.66),
    'cluster_A_02': (23.50, 7.60),
    'cluster_A_03': (16.20, 9.20),
    'cluster_A_04': (24.30, 9.12),
    'cluster_B_01': (18.10, 7.58),
    'cluster_B_02': (26.20, 7.72),
    'cluster_B_03': (18.90, 9.10),
    'cluster_B_04': (27.00, 9.26),
    'cluster_C_01': (20.80, 7.70),
    'cluster_C_02': (28.90, 7.62),
    'cluster_C_03': (21.60, 9.24),
    'cluster_C_04': (29.70, 9.14),
}


def nearest_cluster(x_s1: float, y_s1: float) -> str:
    """Assign a boll (at scale-1 position) to its nearest of 12 cluster anchors."""
    best_id, best_d2 = None, float('inf')
    for cid, (ax, ay) in ANCHORS_SCALE1.items():
        d2 = (x_s1 - ax) ** 2 + (y_s1 - ay) ** 2
        if d2 < best_d2:
            best_d2, best_id = d2, cid
    return best_id


def render_world(items, scale: float, ground_size: int) -> str:
    parts = [f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="cotton_demo">

    <physics name="fast_physics" type="dart">
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu"/>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.70 0.85 1.00 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.90 0.90 0.85 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 -0.3 -0.85</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{ground_size * 2} {ground_size * 2}</size>
            </plane>
          </geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <!-- No visual: the bundle's cotton_orchard_clean mesh has its own
             Ground node (baked terrain texture). Z-fighting would result if
             we added a second flat-color visual here. -->
      </link>
    </model>

    <!-- ===== Cotton orchard (no-trees variant, scale baked into model.sdf
             — currently 4x; if you change SCALE here, update model.sdf too) ===== -->
    <include>
      <uri>model://cotton_orchard_clean</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- ===== Pickable bolls (static visual-only, fitted to each cluster's
             socket cup via bundle's calibrated cotton_targets.yaml) ===== -->
''']
    # Fluff sphere size: bundle collision_radius is ~0.018-0.022 at scale 1
    # (≈4 cm diameter). At our scale (default 4), that's ~8 cm. Fill the cup
    # cavity with a slightly-smaller sphere so we don't see the dark interior
    # (the cotton_pick DAE is just an open sepal cup, no fluff modeled).
    fluff_r = 0.018 * scale
    for it in items:
        mesh_uri = f'model://{it["model"]}/meshes/{it["model"]}.dae'
        parts.append(
            f'    <model name="{it["id"]}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{it["x"]:.4f} {it["y"]:.4f} {it["z"]:.4f} 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <visual name="cup">\n'
            f'          <geometry>\n'
            f'            <mesh>\n'
            f'              <uri>{mesh_uri}</uri>\n'
            f'              <scale>{scale} {scale} {scale}</scale>\n'
            f'            </mesh>\n'
            f'          </geometry>\n'
            f'        </visual>\n'
            f'        <visual name="fluff">\n'
            f'          <geometry>\n'
            f'            <sphere><radius>{fluff_r:.4f}</radius></sphere>\n'
            f'          </geometry>\n'
            f'          <material>\n'
            f'            <ambient>0.92 0.92 0.88 1</ambient>\n'
            f'            <diffuse>0.98 0.98 0.95 1</diffuse>\n'
            f'            <specular>0.10 0.10 0.10 1</specular>\n'
            f'          </material>\n'
            f'        </visual>\n'
            f'      </link>\n'
            f'    </model>\n'
        )
    parts.append('\n  </world>\n</sdf>\n')
    return ''.join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scale', type=float, default=4.0,
                    help='Uniform scale (default 4 → plants ~2m, matches our trees). '
                         'Bundle native is 1 (plants ~50cm). Plant-mesh scale lives '
                         'in models/cotton_orchard_clean/model.sdf; keep them in sync.')
    args = ap.parse_args()
    scale = float(args.scale)

    with open(BUNDLE_TARGETS, 'r', encoding='utf-8') as f:
        bundle = yaml.safe_load(f)
    targets = bundle['cotton_targets']
    print(f'Loaded {len(targets)} bolls from bundle cotton_targets.yaml')

    items = []
    counts = {cid: 0 for cid in ANCHORS_SCALE1}
    for t in targets:
        name = t['name']                          # e.g. 'cotton_pick_A_01'
        x_s1, y_s1, z_s1 = t['pose_xyz']
        cluster_id = nearest_cluster(x_s1, y_s1)
        counts[cluster_id] += 1
        items.append({
            'id':       f'boll_{name}',          # Gazebo model name (set_pose target)
            'tree_id':  cluster_id,              # harvester groups per cluster
            'type':     'ripe',                  # bundle bolls are 'pickable'
            'x':        round(x_s1 * scale, 4),
            'y':        round(y_s1 * scale, 4),
            'z':        round(z_s1 * scale, 4),
            'radius':   round(t.get('collision_radius_m', 0.02) * scale, 5),
            'rgba':     [0.95, 0.95, 0.90, 1.0],
            'model':    t['model'],              # cotton_pick_X
            'mesh_scale': scale,
            'source':   name,
        })

    # Field bounds (for ground sizing)
    xs = [it['x'] for it in items]
    ys = [it['y'] for it in items]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # Husky spawn: in front of FIRST cluster (A_01), on the +Y aisle side, within arm reach.
    #
    # Convention (matches husky_orchard_demo.launch.py):
    #   - Husky yaw=0 → front along +X = row line direction
    #   - arm rotates joint1=-π/2 → looks to Husky's right (= -Y in world)
    #   - so Husky should be on +Y side of the cluster's bolls, facing +X
    #
    # Row 1 (A_01,B_01,C_01,A_02,B_02,C_02) has bolls on the +Y side of anchors
    # (socket Y-offset is positive in the bundle's frame). So Husky at
    # anchor_Y + ~1.0 m is ≈0.65 m from the closest boll → comfortable reach.
    a01_ax, a01_ay = ANCHORS_SCALE1['cluster_A_01']
    a01_bolls_y = [it['y'] for it in items if it['tree_id'] == 'cluster_A_01']
    closest_boll_y = min(a01_bolls_y) if a01_bolls_y else a01_ay * scale
    spawn_x  = round(a01_ax * scale, 4)             # aligned with A_01 in X
    spawn_y  = round(closest_boll_y + 0.65, 4)      # 0.65 m on +Y from nearest boll
    spawn_yaw = 0.0                                 # face +X = drive along row line
    print(f'Field bounds at scale {scale}: '
          f'X in [{min_x:.2f}, {max_x:.2f}], Y in [{min_y:.2f}, {max_y:.2f}]')
    print(f'First cluster A_01 anchor: ({a01_ax*scale:.2f}, {a01_ay*scale:.2f})')
    print(f'Suggested Husky spawn: ({spawn_x}, {spawn_y}, 0)  yaw=0  '
          f'(in front of A_01, {round(spawn_y - closest_boll_y, 2)}m from nearest boll)')

    ground_size = max(60, int(max(max_x, max_y) * 1.4))

    with open(WORLD_OUT, 'w', encoding='utf-8') as f:
        f.write(render_world(items, scale, ground_size))
    print(f'Wrote {WORLD_OUT}')

    out_doc = {
        'bolls': {
            'generation': {
                'plan':            'bundle_native_positions_scaled_uniformly',
                'scale':           scale,
                'bundle_source':   'cotton_orchard_pickable_gazebo/config/cotton_targets.yaml',
                'cluster_count':   len(ANCHORS_SCALE1),
                'bolls_per_cluster': counts,
                'field_bounds': {
                    'x_min': round(min_x, 4), 'x_max': round(max_x, 4),
                    'y_min': round(min_y, 4), 'y_max': round(max_y, 4),
                },
                'spawn_hint': {
                    'x':   spawn_x,
                    'y':   spawn_y,
                    'yaw': spawn_yaw,
                    'note': 'In front of cluster_A_01 on +Y aisle side; '
                            'yaw=0 follows row line along +X.',
                },
            },
            'count': {'total': len(items)},
        },
        'items': items,
    }
    with open(BOLLS_YML, 'w', encoding='utf-8') as f:
        yaml.dump(out_doc, f, sort_keys=False, default_flow_style=False)
    print(f'Wrote {BOLLS_YML}  ({len(items)} bolls, {len(ANCHORS_SCALE1)} clusters)')
    print('Per-cluster boll counts:')
    for cid, n in counts.items():
        print(f'  {cid}: {n} bolls')


if __name__ == '__main__':
    main()
