#!/usr/bin/env python3
"""
Generate cotton boll markers for the orchard simulation (Phase 2 Plan B1).

Goals
-----
- Spawn **only static visuals**: spheres with **no collision / no inertia**.
  Grip + transport are handled by orchestrator teleport (mock Gazebo coupling).
- Place bolls on the **aisle-facing outer shell** of each tree canopy so a robot
  in the alley (wrist camera) tends to see them (not buried in interior volume).

Inputs
------
  robot_arm/config/orchard_tree_positions.yaml

Outputs
-------
  robot_arm/config/orchard_bolls.yaml  — inventory for harvest_executor matching
  robot_arm/worlds/orchard_bolls.world — orchard.world + inserted boll models

Usage (from ws):
  cd src/robot_arm/scripts && python3 generate_bolls.py --num-trees 30 --seed 42
"""

from __future__ import annotations

import argparse
import os
import random
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROBOT_ARM = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
TREE_YAML = os.path.join(PKG_ROBOT_ARM, 'config', 'orchard_tree_positions.yaml')
BOLL_YAML = os.path.join(PKG_ROBOT_ARM, 'config', 'orchard_bolls.yaml')
SOURCE_WORLD = os.path.join(PKG_ROBOT_ARM, 'worlds', 'orchard.world')
TARGET_WORLD = os.path.join(PKG_ROBOT_ARM, 'worlds', 'orchard_bolls.world')

RIPE_RADIUS = 0.035
UNRIPE_RADIUS = 0.025
RIPE_RGBA = (0.95, 0.95, 0.90, 1.0)
UNRIPE_RGBA_RANGE = (
    (0.45, 0.55, 0.25, 1.0),
    (0.55, 0.40, 0.20, 1.0),
)
RIPE_PROBABILITY = 0.70

# Shell placement (world frame; orchard rows spaced along +Y).
OUT_DEPTH_MIN_M = 0.22  # radial-ish offset along aisle normal from trunk
OUT_DEPTH_MAX_M = 0.42
JITTER_X_M = 0.14
Z_BIAS_POWER = 1.65  # >1 biases samples toward canopy_z_max (upper foliage)


def _aisle_normal_y(tree_row_idx: int) -> float:
    """±1 along Y alternating by row → two-sided aisle exposure."""
    return 1.0 if (tree_row_idx % 2 == 0) else -1.0


def _biased_z(canopy_z_min: float, canopy_z_max: float, rng: random.Random) -> float:
    span = canopy_z_max - canopy_z_min
    if span <= 1e-6:
        return canopy_z_min
    u = rng.random()
    t = u ** Z_BIAS_POWER
    return canopy_z_min + t * span


def _sample_outer_shell(
    tx: float,
    ty: float,
    canopy_z_min: float,
    canopy_z_max: float,
    row_idx: int,
    rng: random.Random,
) -> tuple[float, float, float]:
    ny = _aisle_normal_y(row_idx)
    depth = rng.uniform(OUT_DEPTH_MIN_M, OUT_DEPTH_MAX_M)
    jx = rng.uniform(-JITTER_X_M, JITTER_X_M)
    x = tx + jx
    y = ty + ny * depth
    z = _biased_z(canopy_z_min, canopy_z_max, rng)
    return (x, y, z)


def _iter_trees(path: str) -> tuple[dict, list]:
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    trees = list(data['trees'])
    orchard_meta = dict(data['orchard']) if isinstance(data.get('orchard'), dict) else {}
    return orchard_meta, trees


def generate_items(
    trees: list,
    bolls_range: tuple[int, int],
    ripe_prob: float,
    seed: int,
) -> list:
    items: list = []
    for tree in trees:
        tid = str(tree['id'])
        rng = random.Random((seed ^ hash(tid)) & 0xFFFFFFFF)
        n = rng.randint(bolls_range[0], bolls_range[1])
        row_idx = int(tree.get('row', 0))
        for bi in range(n):
            cx, cy, cz = _sample_outer_shell(
                float(tree['x']),
                float(tree['y']),
                float(tree['canopy_z_min']),
                float(tree['canopy_z_max']),
                row_idx,
                rng,
            )
            is_ripe = rng.random() < ripe_prob
            if is_ripe:
                b_type = 'ripe'
                radius = RIPE_RADIUS
                rgba = RIPE_RGBA
            else:
                b_type = 'unripe'
                radius = UNRIPE_RADIUS
                rgba = rng.choice(UNRIPE_RGBA_RANGE)
            sid = tid.replace('/', '_').replace('.', '_')
            items.append({
                'id': f'boll_{sid}_{bi}',
                'tree_id': tid,
                'type': b_type,
                'x': round(cx, 4),
                'y': round(cy, 4),
                'z': round(cz, 4),
                'radius': radius,
                'rgba': list(rgba),
            })
    return items


def sdf_static_visual_sphere(it: dict) -> str:
    r, g, b, a = it['rgba']
    rad = it['radius']
    name = it['id']
    x, y, z = it['x'], it['y'], it['z']
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x} {y} {z} 0 0 0</pose>\n'
        f'      <link name="shell">\n'
        f'        <visual name="visual">\n'
        f'          <geometry><sphere><radius>{rad}</radius></sphere></geometry>\n'
        f'          <material>\n'
        f'            <ambient>{r} {g} {b} {a}</ambient>\n'
        f'            <diffuse>{r} {g} {b} {a}</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


def insert_before_world_close(base_sdf_text: str, extra: str) -> str:
    """Insert boll SDF models just before the </world> closing tag.

    IMPORTANT: must preserve everything AFTER </world> (notably </sdf>)
    or the resulting file will be malformed XML.
    """
    closing = '</world>'
    marker = '\n    <!-- ===== Bolls (auto-generated by generate_bolls.py) ===== -->\n'
    idx = base_sdf_text.rfind(closing)
    if idx < 0:
        raise ValueError('Malformed orchard.world — missing closing </world>')
    head = base_sdf_text[:idx]
    tail = base_sdf_text[idx:]  # keeps </world> + </sdf> + any trailing content
    return head + marker + extra + '  ' + tail


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--bolls-min', type=int, default=5)
    ap.add_argument('--bolls-max', type=int, default=7)
    ap.add_argument('--ripe-prob', type=float, default=RIPE_PROBABILITY)
    ap.add_argument(
        '--num-trees',
        type=int,
        default=None,
        help='Subset first N trees (recommended on weak CPUs; default: all)',
    )
    args = ap.parse_args()

    _, trees = _iter_trees(TREE_YAML)
    if args.num_trees is not None:
        trees = trees[: args.num_trees]

    items = generate_items(trees, (args.bolls_min, args.bolls_max), args.ripe_prob, args.seed)

    ripe_n = sum(1 for x in items if x['type'] == 'ripe')
    out_doc = {
        'bolls': {
            'generation': {
                'plan': 'B1_static_visual_mock_teleport',
                'shell': {
                    'out_depth_min_m': OUT_DEPTH_MIN_M,
                    'out_depth_max_m': OUT_DEPTH_MAX_M,
                    'jitter_x_m': JITTER_X_M,
                    'z_bias_power': Z_BIAS_POWER,
                    'aisle_normal': 'alternate_sign_by_row (+Y/-Y)',
                },
                'seed': args.seed,
                'bolls_per_tree_min': args.bolls_min,
                'bolls_per_tree_max': args.bolls_max,
                'ripe_probability': args.ripe_prob,
                'tree_subset': len(trees),
                'model_static_visual_only': True,
            },
            'count': {
                'total': len(items),
                'ripe': ripe_n,
                'unripe': len(items) - ripe_n,
            },
        },
        'items': items,
    }

    with open(BOLL_YAML, 'w', encoding='utf-8') as f:
        yaml.dump(out_doc, f, sort_keys=False, default_flow_style=False)

    with open(SOURCE_WORLD, 'r', encoding='utf-8') as f:
        base_txt = f.read()
    sdf_extra = ''.join(sdf_static_visual_sphere(it) for it in items)
    merged = insert_before_world_close(base_txt, sdf_extra)

    with open(TARGET_WORLD, 'w', encoding='utf-8') as f:
        f.write(merged)

    print(f'Wrote {BOLL_YAML} ({len(items)} bolls, {len(trees)} trees)')
    print(f'Wrote {TARGET_WORLD}')
    print(
        '  Placement: aisle-facing +/-Y canopy shell, upper-canopy biased Z, jitter X; '
        'static visuals only (collision-free).')


if __name__ == '__main__':
    main()
