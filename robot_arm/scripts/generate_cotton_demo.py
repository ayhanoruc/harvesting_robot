#!/usr/bin/env python3
"""
Generate `cotton_demo.world` + `cotton_demo_bolls.yaml` + `cotton_demo_clusters.yaml`
using cotton_cluster_template instancing in a COMPACT 2-row grid.

Why this version (vs bundle-monolithic)
---------------------------------------
The bundle's cotton_orchard_static mesh bakes 12 clusters at fixed XY positions
with ~10.8 m in-row spacing when scaled 4x. The user wants WITHIN-row clusters
closer together (not across-row). To do that the mesh can't be monolithic —
each cluster needs its own pose.

Approach: re-use the already-extracted single-cluster template
(`cotton_cluster_template.dae`, anchor at origin, 4 sockets) and instance it
12 times in a grid we control. Each instance:
  - plant visual: cotton_cluster_template.dae @ PLANT_SCALE
  - 4 pickable bolls at template socket offsets (cotton_cluster_sockets.yaml)
    rotated by per-cluster yaw, scaled by PLANT_SCALE, with a white fluff
    sphere inside the cup (so we don't see the dark cavity from the side)

Layout (compact, 12 clusters):
  Row 1 (Y=ROW1_Y, yaw=0,  bolls face +Y): A_01,B_01,C_01,A_02,B_02,C_02
  Row 2 (Y=ROW2_Y, yaw=π,  bolls face -Y): A_03,B_03,C_03,A_04,B_04,C_04
  In-row X spacing: COL_SPACING_M (default 3.0 m, compact vs bundle's 10.8)
  Row 2 X-offset:   COL_SPACING_M / 2   (offset half-period like real orchards)
  Husky aisle:      Y = AISLE_Y, between Row 1 and Row 2

Outputs
  - worlds/cotton_demo.world          (template-instanced field, no monolithic
                                       cluster mesh, with ground)
  - config/cotton_demo_bolls.yaml     (simple_cluster_harvester inventory,
                                       items[] schema — same as orchard_bolls)
  - config/cotton_demo_clusters.yaml  (row_navigator inventory, trees[] schema
                                       — same as orchard_tree_positions.yaml)
"""

from __future__ import annotations

import argparse
import math
import os
import yaml

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PKG         = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
SOCKETS_YML = os.path.join(PKG, 'config', 'cotton_cluster_sockets.yaml')
BOLLS_YML   = os.path.join(PKG, 'config', 'cotton_demo_bolls.yaml')
CLUSTERS_YML = os.path.join(PKG, 'config', 'cotton_demo_clusters.yaml')
WORLD_OUT   = os.path.join(PKG, 'worlds', 'cotton_demo.world')
PICKS_DIR   = os.path.join(PKG, 'models', 'cotton_picks')

CLUSTER_MESH_URI = 'model://cotton_orchard_static/meshes/cotton_cluster_template.dae'

# Bundle's anchor naming preserved (12 clusters: A_01..A_04, B_01..B_04, C_01..C_04).
# A,B,C are X-columns; _01/_02 = Row 1 (low Y), _03/_04 = Row 2 (high Y).
# Bundle X-order within Row 1: A_01, B_01, C_01, A_02, B_02, C_02 → keep same order
# so existing route configs (cluster_A_01 → cluster_B_01 → ...) still make sense.
ROW1_CLUSTERS = ['cluster_A_01', 'cluster_B_01', 'cluster_C_01',
                 'cluster_A_02', 'cluster_B_02', 'cluster_C_02']
ROW2_CLUSTERS = ['cluster_A_03', 'cluster_B_03', 'cluster_C_03',
                 'cluster_A_04', 'cluster_B_04', 'cluster_C_04']


def list_pick_models():
    if not os.path.isdir(PICKS_DIR):
        raise RuntimeError(f'No cotton_picks dir: {PICKS_DIR}')
    names = sorted(
        d for d in os.listdir(PICKS_DIR)
        if d.startswith('cotton_pick_') and os.path.isdir(os.path.join(PICKS_DIR, d))
    )
    if not names:
        raise RuntimeError(f'No cotton_pick_* models in {PICKS_DIR}')
    return names


def load_socket_offsets():
    with open(SOCKETS_YML, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)['sockets']


def render_world_xml(clusters, items, scale, ground_size):
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

    <!-- Physics-only ground plane (collision for Husky to drive on). The
         visible terrain texture comes from cotton_field_ground below — no
         visual here so we don't Z-fight with the textured mesh. -->
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
      </link>
    </model>

    <!-- ===== Textured field terrain (Ground node extracted from bundle's
             cotton_orchard_static_no_trees.dae — plant nodes stripped).
             Covers ~X[0,50] × Y[0,20] at scale 1, which contains our
             compact cluster field with margin. Visual-only — physics
             handled by ground_plane above. ===== -->
    <include>
      <uri>model://cotton_field_ground</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <!-- ===== Cotton clusters: cotton_cluster_template.dae instanced per cluster
             at custom (X, Y, 0, 0, 0, yaw). Each anchor is a single cluster
             (4 branches + 4 pedicels + 4 sockets, branches+pedicels only —
             bolls are placed separately below). ===== -->
''']

    for c in clusters:
        parts.append(
            f'    <model name="{c["id"]}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{c["x"]:.4f} {c["y"]:.4f} 0 0 0 {c["yaw"]:.6f}</pose>\n'
            f'      <link name="link">\n'
            f'        <visual name="visual">\n'
            f'          <geometry>\n'
            f'            <mesh>\n'
            f'              <uri>{CLUSTER_MESH_URI}</uri>\n'
            f'              <scale>{scale} {scale} {scale}</scale>\n'
            f'            </mesh>\n'
            f'          </geometry>\n'
            f'        </visual>\n'
            f'      </link>\n'
            f'    </model>\n'
        )

    parts.append('\n    <!-- ===== Pickable bolls (cotton_pick_*.dae cup + white fluff sphere\n'
                 '             for opaque interior) at each cluster\'s socket offsets ===== -->\n')

    fluff_r = round(0.018 * scale, 4)
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
            f'            <sphere><radius>{fluff_r}</radius></sphere>\n'
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
    ap.add_argument('--col-spacing', type=float, default=3.0,
                    help='In-row X spacing between cluster centers (m). Default 3.0 '
                         '(vs bundle native ~10.8m at scale 4). Plant XY radius at '
                         'PLANT_SCALE=2 is ~0.3 m so 3 m gives ~2.4 m clearance.')
    ap.add_argument('--row-spacing', type=float, default=6.0,
                    help='Between-row Y distance between Row 1 and Row 2 (m). '
                         'Default 6.0 — kept wide (user requested compactness '
                         'WITHIN rows, not across).')
    ap.add_argument('--plant-scale', type=float, default=4.0,
                    help='Mesh scale for cotton_cluster_template (default 4 → '
                         '~1.6m visible plant). The extracted template is just '
                         'branches+pedicels+sockets (no leaves/foliage), so it '
                         'reads visually smaller than the bundle full mesh at '
                         'same scale. Use 4 to match the visual prominence the '
                         'bundle had at scale 4. Boll socket offsets and fluff '
                         'sphere radius all scale uniformly, locking bolls in '
                         'the sepal cups at any scale.')
    ap.add_argument('--row1-y', type=float, default=0.0,
                    help='World Y of Row 1 anchors (default 0).')
    ap.add_argument('--row1-x0', type=float, default=0.0,
                    help='World X of Row 1 first cluster (cluster_A_01) (default 0).')
    ap.add_argument('--aisle-offset', type=float, default=1.0,
                    help='Husky aisle Y offset from Row 1 (m). Default 1.0 — Husky '
                         'sits 1m on +Y of Row 1 anchors, bolls at Y≈0.34 → arm-to-'
                         'boll dy≈0.66 m (matches old orchard_bolls reach budget).')
    args = ap.parse_args()

    scale       = float(args.plant_scale)
    col_sp      = float(args.col_spacing)
    row_sp      = float(args.row_spacing)
    row1_y      = float(args.row1_y)
    row1_x0     = float(args.row1_x0)
    aisle_off   = float(args.aisle_offset)
    row2_y      = row1_y + row_sp
    row2_x0     = row1_x0 + col_sp / 2.0  # offset half-period (real-orchard pattern)

    socket_offsets = load_socket_offsets()
    pick_models    = list_pick_models()
    print(f'Sockets per cluster: {len(socket_offsets)}')
    print(f'cotton_pick_* models available: {len(pick_models)}')

    # Build cluster anchors + bolls
    clusters = []
    items    = []
    pick_idx = 0
    for r, names in enumerate([ROW1_CLUSTERS, ROW2_CLUSTERS]):
        # Row 1: yaw=0 → bolls on +Y side (toward aisle)
        # Row 2: yaw=π → bolls on -Y side (toward aisle)
        yaw  = 0.0 if r == 0 else math.pi
        y    = row1_y if r == 0 else row2_y
        x0   = row1_x0 if r == 0 else row2_x0
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        for c_idx, cname in enumerate(names):
            cx = x0 + c_idx * col_sp
            cy = y
            clusters.append({'id': cname, 'x': cx, 'y': cy, 'yaw': yaw,
                             'row': r, 'col': c_idx})
            # Per-cluster 4 bolls at template socket offsets
            for s in socket_offsets:
                sx = s['x'] * scale
                sy = s['y'] * scale
                sz = s['z'] * scale
                wx = cx + cos_y * sx - sin_y * sy
                wy = cy + sin_y * sx + cos_y * sy
                wz = sz
                socket_tag = s['name'].split('_', 1)[1]   # 'A_01' from 'socket_A_01'
                items.append({
                    'id':         f'boll_{cname}_{socket_tag}',
                    'tree_id':    cname,
                    'type':       'ripe',
                    'x':          round(wx, 4),
                    'y':          round(wy, 4),
                    'z':          round(wz, 4),
                    'radius':     round(0.018 * scale, 5),
                    'rgba':       [0.95, 0.95, 0.90, 1.0],
                    'model':      pick_models[pick_idx % len(pick_models)],
                    'socket':     s['name'],
                    'cluster_row': r,
                })
                pick_idx += 1

    # Field bounds (for ground sizing) — pad for camera/depth visibility
    xs = [c['x'] for c in clusters]
    ys = [c['y'] for c in clusters]
    margin = 6.0
    ground_size = int(max(max(xs) - min(xs), max(ys) - min(ys)) + margin * 2 + 10)

    husky_spawn = {
        'x':   round(row1_x0, 4),                        # X of cluster_A_01
        'y':   round(row1_y + aisle_off, 4),             # +Y aisle from Row 1
        'z':   0.0,
        'yaw': 0.0,                                       # face +X = row line
        'comment': 'in front of cluster_A_01, +Y aisle side, drives along row +X',
    }

    with open(WORLD_OUT, 'w', encoding='utf-8') as f:
        f.write(render_world_xml(clusters, items, scale, ground_size))
    print(f'Wrote {WORLD_OUT}')
    print(f'  Clusters: {len(clusters)}  (Row1 Y={row1_y}, Row2 Y={row2_y})')
    print(f'  Bolls:    {len(items)}     ({len(socket_offsets)} per cluster)')
    print(f'  X-spacing: {col_sp}m in row (vs bundle native ~10.8m at scale 4)')

    # cotton_demo_bolls.yaml — harvester (simple_cluster_harvester) inventory
    bolls_doc = {
        'bolls': {
            'generation': {
                'plan':            'cotton_cluster_template_instanced_compact',
                'plant_scale':     scale,
                'col_spacing_m':   col_sp,
                'row_spacing_m':   row_sp,
                'row1_y':          row1_y,
                'row1_x0':         row1_x0,
                'aisle_offset_m':  aisle_off,
                'sockets_per_cluster': len(socket_offsets),
            },
            'count': {'total': len(items)},
        },
        'spawn_hint': husky_spawn,
        'items': items,
    }
    with open(BOLLS_YML, 'w', encoding='utf-8') as f:
        yaml.dump(bolls_doc, f, sort_keys=False, default_flow_style=False)
    print(f'Wrote {BOLLS_YML}  ({len(items)} bolls)')

    # cotton_demo_clusters.yaml — row_navigator trees inventory (same schema
    # as robot_arm/config/orchard_tree_positions.yaml, drop-in via param
    # tree_positions_yaml:=cotton_demo_clusters.yaml)
    clusters_doc = {
        'metadata': {
            'source':      'cotton_cluster_template instances (compact)',
            'plant_scale': scale,
            'col_spacing_m': col_sp,
            'row_spacing_m': row_sp,
            'aisle_offset_m': aisle_off,
        },
        'spawn_hint': husky_spawn,
        'sample_route_row1': ROW1_CLUSTERS,
        'sample_route_row2': ROW2_CLUSTERS,
        'trees': [
            {
                'id':            c['id'],
                'row':           c['row'],
                'col':           c['col'],
                'x':             round(c['x'], 4),
                'y':             round(c['y'], 4),
                'yaw':           round(c['yaw'], 6),
                'canopy_z_min':  0.4 * scale,      # template plant base
                'canopy_z_max':  0.4 * scale,      # template plant top (sockets at z≤0.38 at scale1)
            }
            for c in clusters
        ],
    }
    with open(CLUSTERS_YML, 'w', encoding='utf-8') as f:
        yaml.dump(clusters_doc, f, sort_keys=False, default_flow_style=False)
    print(f'Wrote {CLUSTERS_YML}  ({len(clusters)} clusters)')

    print('')
    print('Husky spawn for cotton_demo.world:')
    print(f'  spawn_x:={husky_spawn["x"]} spawn_y:={husky_spawn["y"]} '
          f'spawn_yaw:={husky_spawn["yaw"]}')
    print('Row navigator example:')
    print(f'  ros2 run orchestrator row_navigator --ros-args \\')
    print(f'    -p tree_positions_yaml:={CLUSTERS_YML} \\')
    print(f'    -p scout_y:={husky_spawn["y"]} \\')
    print(f'    -p scout_yaw:=0.0 \\')
    print(f'    -p route:="{ROW1_CLUSTERS}"')


if __name__ == '__main__':
    main()
