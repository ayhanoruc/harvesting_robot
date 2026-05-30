#!/usr/bin/env python3
"""
Generate `orchard_pickable.world` — uses the cotton_orchard_pickable bundle's
own plant geometry AND boll positions together.

The bundle ships two coupled pieces:
  - `cotton_orchard_static`: ONE baked DAE mesh of all plant stems, sockets,
    husks, and green/brown distractors. The bolls' sockets are sculpted into
    this mesh — pickable boll positions only "make sense" against this mesh.
  - `cotton_pick_*` (41): the loose white cotton boll meshes that go in the
    sockets. Positions come from `config/cotton_targets.yaml`.

So we use BOTH:
  - Include cotton_orchard_static in the world (one static <include>; that
    works fine in Gazebo Fortress — only dynamic <include>s have the GUI
    deserialize bug).
  - Inline the 41 dynamic pickable bolls at the positions from
    cotton_targets.yaml. Inline (not <include>) to avoid the GUI bug.

We keep our own physics/scene/sun from `orchard.world`, drop our trunks/leaves
links, and skip our `orchard_tree_positions.yaml` entirely for this world —
the bundle has its own layout (12 clusters in a 2×6 grid around x∈[15,30],
y∈[7.7, 9.0]).

Inputs:
  - robot_arm/worlds/orchard.world             (physics + scene base)
  - robot_arm/config/cotton_targets.yaml       (41 boll positions)
  - robot_arm/models/cotton_orchard_static/    (plant mesh)
  - robot_arm/models/cotton_picks/cotton_pick_*  (boll meshes)

Outputs:
  - robot_arm/worlds/orchard_pickable.world
  - robot_arm/config/orchard_pickable_bolls.yaml   (inventory mirror of cotton_targets)

Usage:
  cd src/robot_arm/scripts
  python3 generate_pickable_orchard.py
"""

from __future__ import annotations

import argparse
import os
import re
import yaml

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
PKG           = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
TARGETS_YAML  = os.path.join(PKG, 'config', 'cotton_targets.yaml')

# Two output world variants — same boll inventory, different plant mesh:
TARGET_WORLD_TREES    = os.path.join(PKG, 'worlds', 'orchard_pickable.world')
TARGET_WORLD_NO_TREES = os.path.join(PKG, 'worlds', 'orchard_pickable_no_trees.world')
BOLL_YAML             = os.path.join(PKG, 'config', 'orchard_pickable_bolls.yaml')


WORLD_HEADER_TEMPLATE = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="orchard">

    <!-- Physics: 5x speed (matches our previous orchard.world). -->
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
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.85 1.0 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 -0.3 -0.85</direction>
    </light>

    <!-- Infinite ground plane: Husky physics (cotton mesh provides visual ground). -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>500 500</size></plane>
          </geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
'''

WORLD_FOOTER = '''
  </world>
</sdf>
'''


def render_boll_model(boll_id: str, model_name: str, link_name: str,
                      x: float, y: float, z: float,
                      mesh_scale: float) -> str:
    """Inline a pickable cotton boll — STATIC, VISUAL-ONLY.

    Matches the proven pattern from generate_bolls.py (sphere variant):
      <static>true</static> + <link><visual/></link>
      no <collision>, no <inertial>, no <gravity> tag

    Why static:
      - Pickup is done by mock teleport (Gazebo set_pose), not physics grasp,
        so we don't need dynamics. Static models still respond to set_pose.
      - Static models don't trigger Gazebo Fortress's
        "Unable to deserialize sdf::Model" GUI warnings (only dynamics do).
      - Without <collision>, arm passes through bolls until teleport fires —
        same passthrough as our original sphere setup.

    `mesh_scale` is applied uniformly so the visible boll grows with the
    enclosing plant (cotton_orchard_static is scaled in its own model.sdf).
    """
    mesh_uri = f'model://{model_name}/meshes/{model_name}.dae'
    return (
        f'    <model name="{boll_id}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n'
        f'      <link name="{link_name}">\n'
        f'        <visual name="visual">\n'
        f'          <geometry>\n'
        f'            <mesh>\n'
        f'              <uri>{mesh_uri}</uri>\n'
        f'              <scale>{mesh_scale} {mesh_scale} {mesh_scale}</scale>\n'
        f'            </mesh>\n'
        f'          </geometry>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


def render_world(plant_model_uri: str, boll_chunks: list) -> str:
    """Compose a SELF-CONTAINED world: physics + sky + sun + ground_plane
    + cotton orchard mesh + inlined bolls.

    Critically: we do NOT include any geometry from our own orchard.world
    (terrain DAE, trunks, leaves, orchard_static). That mesh covers x∈[4,41]
    y∈[3,35] and conflicts visually with the cotton bundle's own ground +
    plant mesh — combining them produced the "two separate fields" bug.
    """
    body = (
        '\n    <!-- ===== Cotton orchard plant mesh ===== -->\n'
        '    <include>\n'
        '      <name>cotton_orchard_plants</name>\n'
       f'      <uri>{plant_model_uri}</uri>\n'
        '      <pose>0 0 0 0 0 0</pose>\n'
        '    </include>\n'
        '\n    <!-- ===== Pickable bolls (inlined from cotton_targets.yaml) ===== -->\n'
        + ''.join(boll_chunks)
    )
    return WORLD_HEADER_TEMPLATE + body + WORLD_FOOTER


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--scale', type=float, default=4.0,
                    help='Uniform scale for plants + bolls (default 4.0). '
                         'Must match the <scale> in cotton_orchard_static/model.sdf '
                         'AND cotton_orchard_clean/model.sdf so plants and bolls '
                         'stay aligned.')
    ap.add_argument('--no-trees', action='store_true',
                    help='Use cotton_orchard_clean (Tree_lp_* nodes stripped) '
                         'and write to orchard_pickable_no_trees.world. Default '
                         'keeps the Tree_lp_* distractors and writes to '
                         'orchard_pickable.world.')
    args = ap.parse_args()
    scale = float(args.scale)
    if args.no_trees:
        plant_model_uri = 'model://cotton_orchard_clean'
        target_world    = TARGET_WORLD_NO_TREES
        variant_label   = 'NO-TREES'
    else:
        plant_model_uri = 'model://cotton_orchard_static'
        target_world    = TARGET_WORLD_TREES
        variant_label   = 'WITH-TREES'

    with open(TARGETS_YAML, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    targets = data.get('cotton_targets', [])
    if not targets:
        raise RuntimeError(f'No targets found in {TARGETS_YAML}')
    print(f'Loaded {len(targets)} boll targets from {TARGETS_YAML}')
    print(f'Scale factor: {scale}x (plant + boll positions + boll mesh)')

    # Render inline boll models — positions AND meshes scaled by `scale`
    boll_chunks = []
    inventory   = []
    for t in targets:
        boll_id    = str(t['name'])
        model_name = str(t['model'])
        link_name  = str(t['link'])
        x0, y0, z0 = (float(v) for v in t['pose_xyz'])
        # Position scaled uniformly to stay aligned with the scaled plant mesh
        x, y, z = x0 * scale, y0 * scale, z0 * scale
        boll_chunks.append(
            render_boll_model(boll_id, model_name, link_name, x, y, z, scale))
        inventory.append({
            'id':    boll_id,
            'model': model_name,
            'link':  link_name,
            'x':     round(x, 4),
            'y':     round(y, 4),
            'z':     round(z, 4),
            'scale': scale,
        })

    # Compose world from scratch — NO reference to our orchard.world.
    out = render_world(plant_model_uri, boll_chunks)
    with open(target_world, 'w', encoding='utf-8') as f:
        f.write(out)
    print(f'Wrote {target_world}  [{variant_label}]')
    print(f'  {len(targets)} bolls inlined + plant mesh: {plant_model_uri}')
    print('  Self-contained world (only cotton bundle + ground_plane).')

    # Inventory mirror (handy for orchestrator/cluster_scanner targeting)
    with open(BOLL_YAML, 'w', encoding='utf-8') as f:
        yaml.dump(
            {'source': 'cotton_targets.yaml (cotton_orchard_pickable bundle)',
             'count': len(inventory),
             'items': inventory},
            f, sort_keys=False, default_flow_style=False)
    print(f'Wrote {BOLL_YAML}')

    # Cluster overview — group by approximate XY
    clusters = {}
    for it in inventory:
        key = (round(it['x'] / 1.5) * 1.5, round(it['y'] / 1.5) * 1.5)
        clusters.setdefault(key, []).append(it)
    print(f'\nApprox cluster centers (~{len(clusters)} clusters):')
    for (cx, cy), members in sorted(clusters.items()):
        xs = [m['x'] for m in members]
        ys = [m['y'] for m in members]
        print(f'  center~({cx:.1f}, {cy:.1f}): {len(members)} bolls  '
              f'X[{min(xs):.2f}..{max(xs):.2f}]  Y[{min(ys):.2f}..{max(ys):.2f}]')


if __name__ == '__main__':
    main()
