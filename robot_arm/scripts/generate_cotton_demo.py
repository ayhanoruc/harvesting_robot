#!/usr/bin/env python3
"""
Generate `cotton_demo.world` + `cotton_demo_bolls.yaml` + `cotton_demo_clusters.yaml`
using Deniz's `branch_variant_*` + `cotton_pick_*` bundle (real-physics solids,
no hollow cups), instanced into a dense cotton-field layout.

Why this version (vs cotton_cluster_template)
---------------------------------------------
The previous generator instanced a single hollow cup template (sepal husk only)
with a separately-rendered fluff sphere. The cup had no collision, so depth
rays sometimes flew through the cup-fluff gap and projected bolls 0.4-0.6m
past the real position. Deniz's `cotton_pick_*` models are SOLID meshes with
proper sphere collision baked into model.sdf — ray hits the boll directly.

Each branch_variant_X model already contains baked-in green/brown UNRIPE
bolls as static visuals; the white MATURE bolls are the separate pickable
cotton_pick_* models. So "some bolls unripe" comes for free.

Layout (preserves the 12 anchor positions of the old cotton_demo)
-----------------------------------------------------------------
Row 1 (Y=row1_y=0,    facing-toward-aisle yaw): 6 target clusters
  cluster_A_01 .. cluster_C_02   at X = 0, 3, 6, 9, 12, 15
Row 2 (Y=row1_y+row_spacing=6,  facing-toward-aisle yaw): 6 target clusters
  cluster_A_03 .. cluster_C_04   at X = 1.5, 4.5, 7.5, 10.5, 13.5, 16.5

Husky aisle:  Y = aisle_y (default 0.85, was 1.0). Tighter than old config
              because branch_variant bolls sit closer to the plant stem than
              the cotton_cluster_template sockets did — arm reach budget
              matches old setup at this aisle.

Variant assignment to targets is fixed (not random) for reproducibility.

Fillers (denser cotton-field aesthetic, no pickable bolls)
----------------------------------------------------------
We add ~40 extra branch_variant_* instances in:
  - between target clusters in each row (mid-X slots)
  - one "front" row at Y = -2  (in front of Row 1, away from Husky)
  - two "middle" rows at Y = 2.5, 4.5  (between Row 1 and Row 2)
  - one "back" row at Y = 8     (behind Row 2)
Fillers carefully avoid the Husky aisle envelope [aisle_y ± 0.4 m].

Outputs
  - worlds/cotton_demo.world          (branch_variant + cotton_pick instances
                                       with proper collisions, plus fillers)
  - config/cotton_demo_bolls.yaml     (simple_cluster_harvester inventory,
                                       items[] schema — id field is also the
                                       Gazebo model name for teleport)
  - config/cotton_demo_clusters.yaml  (row_navigator inventory, trees[] schema)
"""

from __future__ import annotations

import argparse
import math
import os
import yaml

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PKG         = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
TARGETS_YML = os.path.join(PKG, 'config', 'branch_variant_targets.yaml')
BOLLS_YML   = os.path.join(PKG, 'config', 'cotton_demo_bolls.yaml')
CLUSTERS_YML = os.path.join(PKG, 'config', 'cotton_demo_clusters.yaml')
WORLD_OUT   = os.path.join(PKG, 'worlds', 'cotton_demo.world')

# Bundle's 6 branch variants. The boll-side native Y direction (read from
# branch_variant_targets.yaml) is used to pick the right yaw so bolls always
# face the Husky aisle.
VARIANT_NATIVE_BOLL_Y_SIGN = {
    'branch_variant_A_mature_white_01':  +1,   # bolls at native +Y
    'branch_variant_B_mixed_green_01':   +1,
    'branch_variant_C_mixed_brown_01':   -1,
    'branch_variant_D_sparse_green_01':  +1,   # mostly +Y
    'branch_variant_E_tall_mature_01':   -1,   # tall, dense, -Y native
    'branch_variant_F_dry_brown_01':     -1,
}

# Stem cylinder approx from each variant's model.sdf <collision>.
# Used for filler stem collisions (not load-bearing, just so Husky bumper
# bounces off if it veers into a plant).
VARIANT_STEM = {
    # name : (radius_m, length_m, z_center_m)  — at scale 1
    'branch_variant_A_mature_white_01': (0.018, 0.58, 0.29),
    'branch_variant_B_mixed_green_01':  (0.018, 0.62, 0.31),
    'branch_variant_C_mixed_brown_01':  (0.018, 0.54, 0.27),
    'branch_variant_D_sparse_green_01': (0.018, 0.48, 0.24),
    'branch_variant_E_tall_mature_01':  (0.018, 0.78, 0.39),
    'branch_variant_F_dry_brown_01':    (0.018, 0.50, 0.25),
}

# Target anchor positions (kept identical to the previous cotton_demo so the
# row_navigator route configs don't need to change).
ROW1_CLUSTERS = ['cluster_A_01', 'cluster_B_01', 'cluster_C_01',
                 'cluster_A_02', 'cluster_B_02', 'cluster_C_02']
ROW2_CLUSTERS = ['cluster_A_03', 'cluster_B_03', 'cluster_C_03',
                 'cluster_A_04', 'cluster_B_04', 'cluster_C_04']

# Variant assignment per anchor — fixed for visual variety + reach budget.
# Row 1 wants bolls facing +Y (toward aisle); the yaw is picked from
# VARIANT_NATIVE_BOLL_Y_SIGN so we don't have to hand-flip variants.
# Mix mature_white (A) + tall_mature (E) for prominent ripe targets,
# B/C for visual variety.
TARGET_VARIANTS = {
    'cluster_A_01': 'branch_variant_A_mature_white_01',  # 6 ripe
    'cluster_B_01': 'branch_variant_E_tall_mature_01',   # 7 ripe (tall)
    'cluster_C_01': 'branch_variant_B_mixed_green_01',   # 4 ripe
    'cluster_A_02': 'branch_variant_A_mature_white_01',  # 6 ripe
    'cluster_B_02': 'branch_variant_E_tall_mature_01',   # 7 ripe
    'cluster_C_02': 'branch_variant_C_mixed_brown_01',   # 4 ripe
    'cluster_A_03': 'branch_variant_A_mature_white_01',  # 6 ripe
    'cluster_B_03': 'branch_variant_E_tall_mature_01',   # 7 ripe
    'cluster_C_03': 'branch_variant_B_mixed_green_01',   # 4 ripe
    'cluster_A_04': 'branch_variant_A_mature_white_01',  # 6 ripe
    'cluster_B_04': 'branch_variant_E_tall_mature_01',   # 7 ripe
    'cluster_C_04': 'branch_variant_C_mixed_brown_01',   # 4 ripe
}
# Total: 6×A(6) + 6×E(7) but mixed = 6+7+4+6+7+4 +6+7+4+6+7+4 = 68 ripe pickables.


def load_variant_targets():
    """Read bundle's branch_variant_targets.yaml — relative cotton poses
    per variant + collision sphere radius."""
    with open(TARGETS_YML, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    by_variant: dict[str, list[dict]] = {}
    for t in data['targets']:
        by_variant.setdefault(t['variant'], []).append({
            'cotton': t['cotton'],
            'xyz':    t['pose_xyz_m'],
            'r':      float(t['collision_radius_m']),
        })
    return by_variant


def yaw_for_target_row(row_idx: int, variant: str) -> float:
    """Pick yaw so the variant's bolls face the Husky aisle.

    Row 1 aisle on +Y side  → bolls must end up at world +Y from anchor
    Row 2 aisle on -Y side  → bolls must end up at world -Y from anchor
                              (Husky drives the same aisle from the OTHER row)
    """
    native_sign = VARIANT_NATIVE_BOLL_Y_SIGN[variant]
    want_sign   = +1 if row_idx == 0 else -1
    return 0.0 if native_sign == want_sign else math.pi


def render_pick_block(model_name: str, mesh_variant: str,
                      wx: float, wy: float, wz: float, radius: float,
                      scale: float = 1.0) -> str:
    """One pickable boll: just the solid cotton_pick_*.dae visual mesh.

    Deniz's bundle ships closed/solid boll meshes (not the old hollow sepal
    cup), so the Gazebo depth camera — which renders VISUAL geometry, not
    collision shapes — gets a direct hit on the boll surface. No collision
    block needed: we don't do physical grasping (carry is teleport-based),
    and MoveIt's planning_scene doesn't include these models. `static=true`
    keeps physics out entirely — no gravity, no contact bounce. set_pose
    teleport works on static models (proven with the legacy template
    generator) so harvest grip/carry/drop still work.

    `radius` is kept in the YAML for matcher tuning but unused in the SDF.
    """
    del radius   # see docstring — unused
    mesh_uri = f'model://{mesh_variant}/meshes/{mesh_variant}.dae'
    # Explicit material overrides Gazebo/OGRE2's COLLADA quirk: when the
    # DAE Lambert effect omits the <transparent> tag (Blender exporter
    # never writes it), OGRE2 applies the COLLADA-spec default A_ONE
    # mode and the mesh renders nearly transparent. Setting <material>
    # on the SDF visual forces Gazebo to use these RGBA values directly,
    # bypassing the broken DAE effect parse. Off-white cream color (0.95,
    # 0.93, 0.88) gives the mature-white-cotton look from the bundle's
    # own diffuse (0.91/0.88/0.76) but a touch brighter for visibility.
    return (
        f'    <model name="{model_name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{wx:.4f} {wy:.4f} {wz:.4f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <geometry>\n'
        f'            <mesh>\n'
        f'              <uri>{mesh_uri}</uri>\n'
        f'              <scale>{scale} {scale} {scale}</scale>\n'
        f'            </mesh>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.95 0.93 0.88 1</ambient>\n'
        f'            <diffuse>0.95 0.93 0.88 1</diffuse>\n'
        f'            <specular>0.10 0.10 0.10 1</specular>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


def render_branch_block(model_name: str, variant: str,
                        cx: float, cy: float, yaw: float,
                        scale: float = 1.0) -> str:
    """Static branch+pedicel+husk+unripe-boll visual + stem collision.
    Stem collision dims are scaled to keep the cylinder matching the
    visual mesh (radius × scale, length × scale, z_center × scale)."""
    mesh_uri = f'model://{variant}/meshes/{variant}.dae'
    sr, sl, sz = VARIANT_STEM[variant]
    sr_s = sr * scale
    sl_s = sl * scale
    sz_s = sz * scale
    return (
        f'    <model name="{model_name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{cx:.4f} {cy:.4f} 0 0 0 {yaw:.6f}</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <geometry>\n'
        f'            <mesh>\n'
        f'              <uri>{mesh_uri}</uri>\n'
        f'              <scale>{scale} {scale} {scale}</scale>\n'
        f'            </mesh>\n'
        f'          </geometry>\n'
        f'        </visual>\n'
        f'        <collision name="stem">\n'
        f'          <pose>0 0 {sz_s:.4f} 0 0 0</pose>\n'
        f'          <geometry><cylinder><radius>{sr_s:.4f}</radius>'
        f'<length>{sl_s:.4f}</length></cylinder></geometry>\n'
        f'        </collision>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


def build_filler_positions(aisle_y: float, row1_y: float, row2_y: float,
                           row1_x0: float, col_sp: float, row1_n: int,
                           aisle_clear: float = 0.40):
    """Return list of (x, y, variant_idx_seed) for filler plants.

    Avoids any Y within [aisle_y - aisle_clear, aisle_y + aisle_clear] to
    keep the Husky drive lane clear. We don't actively avoid X-wise — the
    Husky body is short enough (1m) that plants 0.5m to the side are fine.
    """
    row_span_x_min = row1_x0 - col_sp * 0.8
    row_span_x_max = row1_x0 + (row1_n - 1) * col_sp + col_sp * 0.8

    candidate_rows = [
        # (y, x_step, x_offset)
        (row1_y - 2.0, 2.0, 0.0),       # front row (in front of Row 1, away from Husky)
        (row1_y + 0.0, col_sp, col_sp / 2.0),  # mid-X fillers between Row 1 targets
        (row2_y + 0.0, col_sp, 0.0),    # mid-X fillers between Row 2 targets (Row 2 offset already 1.5)
        ((row1_y + row2_y) / 2.0 - 1.25, 1.8, 0.5),   # middle row near Row 1 side
        ((row1_y + row2_y) / 2.0 + 0.25, 1.8, -0.4),  # middle row near Row 2 side
        (row2_y + 2.0, 2.0, 1.0),       # back row (behind Row 2)
    ]

    fillers = []
    seed = 0
    for (y, x_step, x_off) in candidate_rows:
        if abs(y - aisle_y) < aisle_clear:
            continue  # skip aisle envelope
        x = row_span_x_min + x_off
        while x <= row_span_x_max:
            fillers.append((x, y, seed))
            seed += 1
            x += x_step
    return fillers


FILLER_VARIANT_CYCLE = [
    'branch_variant_D_sparse_green_01',
    'branch_variant_F_dry_brown_01',
    'branch_variant_B_mixed_green_01',
    'branch_variant_C_mixed_brown_01',
    'branch_variant_D_sparse_green_01',
    'branch_variant_F_dry_brown_01',
    'branch_variant_A_mature_white_01',  # occasional mature in BG
    'branch_variant_E_tall_mature_01',
]


def render_world_xml(targets, picks, fillers, ground_size):
    # cotton_field_ground model.sdf scales the mesh by GROUND_SCALE (2.0).
    # Native mesh covers X[0,50] Y[0,20] → at scale 2 = X[0,100] Y[0,40],
    # midpoint at (50, 20). To center under the cluster field, shift
    # so (50, 20) lands at the cluster-field midpoint.
    GROUND_SCALE = 2.0
    cxs = [t['x'] for t in targets]
    cys = [t['y'] for t in targets]
    cluster_cx = (min(cxs) + max(cxs)) / 2.0
    cluster_cy = (min(cys) + max(cys)) / 2.0
    ground_offset_x = cluster_cx - 25.0 * GROUND_SCALE
    ground_offset_y = cluster_cy - 10.0 * GROUND_SCALE

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
      <ambient>0.48 0.48 0.48 1</ambient>
      <background>0.70 0.76 0.82 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.90 0.86 0.78 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.35 0.25 -0.90</direction>
    </light>

    <!-- Physics-only ground plane (Husky drives on this; visual texture
         comes from cotton_field_ground below). -->
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

    <!-- Textured field terrain (Ground node from bundle's
         cotton_orchard_static_no_trees.dae). Visual-only.
         Centered under the cluster field so the textured patch
         actually sits beneath the clusters instead of stretching
         off to one side. cotton_field_ground/model.sdf currently
         scales by 2 → 100x40m, native midpoint (50,20). To put that
         midpoint at the cluster-field center we offset by
         (cluster_cx - 50, cluster_cy - 20). -->
    <include>
      <uri>model://cotton_field_ground</uri>
      <pose>{ground_offset_x:.2f} {ground_offset_y:.2f} 0 0 0 0</pose>
    </include>

    <!-- ===== TARGET clusters: branch_variant + its cotton_pick_* models ===== -->
''']

    scale = render_world_xml.plant_scale   # set by main() before call

    for t in targets:
        parts.append(render_branch_block(
            f'{t["id"]}__branch', t['variant'],
            t['x'], t['y'], t['yaw'], scale=scale))

    parts.append('\n    <!-- ===== Pickable cotton bolls (cotton_pick_*.dae visuals) ===== -->\n')
    for p in picks:
        parts.append(render_pick_block(
            p['id'], p['model'], p['x'], p['y'], p['z'], p['radius'], scale=scale))

    if fillers:
        parts.append('\n    <!-- ===== FILLER background plants (no pickable bolls) ===== -->\n')
        for fi, f in enumerate(fillers):
            parts.append(render_branch_block(
                f'filler_{fi:03d}', f['variant'],
                f['x'], f['y'], f['yaw'], scale=scale))

    if render_world_xml.tree_band_xml:
        parts.append('\n    <!-- ===== Trees (one per cluster, outside row) ===== -->\n')
        parts.append(render_world_xml.tree_band_xml)

    parts.append('\n  </world>\n</sdf>\n')
    return ''.join(parts)


def render_single_tree(name: str, x: float, y: float,
                       yaw: float, scale: float) -> str:
    """One cropped tree from single_tree model (trunk + leaves DAEs).

    Extracted by extract_single_tree.py from the monolithic
    cpr_orchard_gazebo meshes — base at origin, real-world Z height.
    Visual-only — no collision (trees sit outside Husky's drive path).
    """
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.6f}</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="trunk">\n'
        f'          <geometry>\n'
        f'            <mesh>\n'
        f'              <uri>model://single_tree/meshes/single_tree_trunk.dae</uri>\n'
        f'              <scale>{scale} {scale} {scale}</scale>\n'
        f'            </mesh>\n'
        f'          </geometry>\n'
        f'        </visual>\n'
        f'        <visual name="leaves">\n'
        f'          <geometry>\n'
        f'            <mesh>\n'
        f'              <uri>model://single_tree/meshes/single_tree_leaves.dae</uri>\n'
        f'              <scale>{scale} {scale} {scale}</scale>\n'
        f'            </mesh>\n'
        f'          </geometry>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--col-spacing', type=float, default=3.0,
                    help='In-row X spacing between TARGET clusters (m). Default 3.0 '
                         '(preserves old cotton_demo anchor positions).')
    ap.add_argument('--row-spacing', type=float, default=6.0,
                    help='Between-row Y distance between Row 1 and Row 2 (m). '
                         'Default 6.0 (preserves old anchor positions).')
    ap.add_argument('--row1-y', type=float, default=0.0,
                    help='World Y of Row 1 anchors.')
    ap.add_argument('--row1-x0', type=float, default=0.0,
                    help='World X of Row 1 first cluster (cluster_A_01).')
    ap.add_argument('--aisle-offset', type=float, default=1.0,
                    help='Husky aisle Y offset from Row 1 (m). Default 1.0 '
                         '(matches old cotton_demo). With plant_scale=4, '
                         'variant boll Y offset × 4 reaches ~0.40m for E/B '
                         '(post-yaw flip), so dY ≈ 1.0-0.40 = 0.60m — close '
                         'to old template arm reach budget.')
    ap.add_argument('--plant-scale', type=float, default=3.0,
                    help='Uniform mesh scale for branch_variant and '
                         'cotton_pick models (default 3.0 → 4.0 was too '
                         'big visually with Deniz natural-scale meshes). '
                         'Affects: mesh scale tag, boll pose offsets '
                         '(CSV × scale), stem collision dims.')
    ap.add_argument('--fillers', action='store_true',
                    help='Add background filler plants for cotton-field '
                         'density (~57 extra branch_variants). OFF by '
                         'default — user wanted only the 12 targets.')
    ap.add_argument('--no-trees', action='store_true',
                    help='Skip the background tree bands (cpr orchard '
                         'trunks+leaves). On by default — gives the field '
                         'a forest backdrop behind each cluster row.')
    ap.add_argument('--tree-scale', type=float, default=2.0,
                    help='Scale for the orchard tree meshes (default 2.0 '
                         '→ ~2x the cotton plants at plant_scale=3, so '
                         'trees tower over the cotton field).')
    args = ap.parse_args()

    col_sp    = float(args.col_spacing)
    row_sp    = float(args.row_spacing)
    row1_y    = float(args.row1_y)
    row1_x0   = float(args.row1_x0)
    aisle_off = float(args.aisle_offset)
    scale     = float(args.plant_scale)
    row2_y    = row1_y + row_sp
    row2_x0   = row1_x0 + col_sp / 2.0   # half-period offset (orchard pattern)
    aisle_y   = row1_y + aisle_off

    # render_world_xml() reads these attributes to scale meshes/stems/picks
    # and to inject one tree per cluster (mirrored to the outside of each row).
    render_world_xml.plant_scale = scale
    tree_scale = float(args.tree_scale)
    if args.no_trees:
        render_world_xml.tree_band_xml = ''
    else:
        # One tree per cluster, placed `back_off` meters on the OUTSIDE
        # side of the cluster row (away from the Husky aisle). Trees scale
        # `tree_scale` (default 2.0 → ~2x the cotton clusters at plant
        # scale 3 → ~3.5m total height vs ~1.7m for cotton plants).
        back_off = 2.5  # offset from cluster row to tree row
        # Row 1 (Y=row1_y): aisle is at +Y → trees go on -Y side
        # Row 2 (Y=row2_y): aisle is at -Y from row2 → trees go on +Y side
        tree_xml_parts: list[str] = []
        for r, names in enumerate([ROW1_CLUSTERS, ROW2_CLUSTERS]):
            row_y = row1_y if r == 0 else row2_y
            x0    = row1_x0 if r == 0 else row2_x0
            ty    = row_y - back_off if r == 0 else row_y + back_off
            for c_idx, cname in enumerate(names):
                tx = x0 + c_idx * col_sp
                # Pseudo-random yaw per cluster index so trees don't all
                # face the same direction (deterministic — no Random use).
                yaw = ((c_idx * 53 + r * 17) % 360) * math.pi / 180.0
                tree_xml_parts.append(render_single_tree(
                    f'tree_{cname}', tx, ty, yaw, tree_scale))
        render_world_xml.tree_band_xml = ''.join(tree_xml_parts)

    by_variant = load_variant_targets()
    print(f'Loaded variant targets: {len(by_variant)} variants')
    for v, picks in by_variant.items():
        print(f'  {v}: {len(picks)} mature bolls')

    targets, picks = [], []
    for r, names in enumerate([ROW1_CLUSTERS, ROW2_CLUSTERS]):
        y  = row1_y if r == 0 else row2_y
        x0 = row1_x0 if r == 0 else row2_x0
        for c_idx, cname in enumerate(names):
            variant = TARGET_VARIANTS[cname]
            yaw     = yaw_for_target_row(r, variant)
            cx      = x0 + c_idx * col_sp
            cy      = y
            targets.append({'id': cname, 'variant': variant,
                            'x': cx, 'y': cy, 'yaw': yaw,
                            'row': r, 'col': c_idx})

            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            for pp in by_variant[variant]:
                # Scale the CSV relative pose to match the visual mesh scale
                # (mesh scale tag stretches verts uniformly, so the boll
                # socket position on the visible branch lives at scale*rel).
                rx, ry, rz = [v * scale for v in pp['xyz']]
                # Rotate relative offset by yaw, then translate to anchor.
                wx = cx + cos_y * rx - sin_y * ry
                wy = cy + sin_y * rx + cos_y * ry
                wz = rz
                pick_id = f'{cname}__{pp["cotton"]}'   # unique per instance
                picks.append({
                    'id':         pick_id,
                    'tree_id':    cname,
                    'type':       'ripe',
                    'x':          round(wx, 4),
                    'y':          round(wy, 4),
                    'z':          round(wz, 4),
                    'radius':     round(pp['r'], 5),
                    'rgba':       [0.95, 0.95, 0.90, 1.0],
                    'model':      pp['cotton'],     # shared mesh source name
                    'variant':    variant,
                    'cluster_row': r,
                })

    # Fillers — OFF by default (user wants only the 12 targets, no background)
    fillers = []
    if args.fillers:
        for (fx, fy, seed) in build_filler_positions(
                aisle_y, row1_y, row2_y, row1_x0, col_sp,
                row1_n=len(ROW1_CLUSTERS)):
            # skip if too close to a target stem (keep visuals from clipping)
            too_close = any(
                (fx - t['x'])**2 + (fy - t['y'])**2 < 0.35**2 for t in targets)
            if too_close:
                continue
            variant = FILLER_VARIANT_CYCLE[seed % len(FILLER_VARIANT_CYCLE)]
            # Pseudo-random yaw from seed (deterministic; no Date/Random use).
            yaw = ((seed * 37) % 360) * math.pi / 180.0
            fillers.append({'x': round(fx, 4), 'y': round(fy, 4),
                            'yaw': round(yaw, 6), 'variant': variant,
                            'seed': seed})

    # Ground sizing
    xs = [t['x'] for t in targets] + [f['x'] for f in fillers]
    ys = [t['y'] for t in targets] + [f['y'] for f in fillers]
    margin = 6.0
    ground_size = int(max(max(xs) - min(xs), max(ys) - min(ys)) + margin * 2 + 10)

    husky_spawn = {
        'x':   round(row1_x0, 4),                           # X of cluster_A_01
        'y':   round(aisle_y, 4),                           # aisle, +Y of Row 1
        'z':   0.0,
        'yaw': 0.0,                                          # face +X = row line
        'comment': f'aisle Y={aisle_y:.2f} (Row1 +0.85), Husky drives along row +X',
    }

    with open(WORLD_OUT, 'w', encoding='utf-8') as f:
        f.write(render_world_xml(targets, picks, fillers, ground_size))
    print(f'Wrote {WORLD_OUT}')
    print(f'  Targets:   {len(targets)}  (Row1 Y={row1_y}, Row2 Y={row2_y})')
    print(f'  Pickables: {len(picks)}    (sum over variant boll counts)')
    print(f'  Fillers:   {len(fillers)}  (background branch_variants)')

    # cotton_demo_bolls.yaml
    bolls_doc = {
        'bolls': {
            'generation': {
                'plan':              'branch_variant_instanced_with_fillers',
                'col_spacing_m':     col_sp,
                'row_spacing_m':     row_sp,
                'row1_y':            row1_y,
                'row1_x0':           row1_x0,
                'aisle_offset_m':    aisle_off,
                'fillers_enabled':   bool(args.fillers),
                'plant_scale':       scale,
                'target_variants':   TARGET_VARIANTS,
            },
            'count': {'total': len(picks), 'targets': len(targets),
                      'fillers': len(fillers)},
        },
        'spawn_hint': husky_spawn,
        'items': picks,
    }
    with open(BOLLS_YML, 'w', encoding='utf-8') as f:
        yaml.dump(bolls_doc, f, sort_keys=False, default_flow_style=False)
    print(f'Wrote {BOLLS_YML}  ({len(picks)} bolls)')

    # cotton_demo_clusters.yaml — row_navigator inventory
    clusters_doc = {
        'metadata': {
            'source':           'branch_variant_* + cotton_pick_* instances',
            'col_spacing_m':    col_sp,
            'row_spacing_m':    row_sp,
            'aisle_offset_m':   aisle_off,
            'target_variants':  TARGET_VARIANTS,
        },
        'spawn_hint': husky_spawn,
        'sample_route_row1': ROW1_CLUSTERS,
        'sample_route_row2': ROW2_CLUSTERS,
        'trees': [
            {
                'id':            t['id'],
                'row':           t['row'],
                'col':           t['col'],
                'x':             round(t['x'], 4),
                'y':             round(t['y'], 4),
                'yaw':           round(t['yaw'], 6),
                'variant':       t['variant'],
                'canopy_z_min':  0.30,
                'canopy_z_max':  0.85,   # E variant goes up to z≈0.83
            }
            for t in targets
        ],
    }
    with open(CLUSTERS_YML, 'w', encoding='utf-8') as f:
        yaml.dump(clusters_doc, f, sort_keys=False, default_flow_style=False)
    print(f'Wrote {CLUSTERS_YML}  ({len(targets)} clusters)')

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
