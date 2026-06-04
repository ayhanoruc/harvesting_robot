#!/usr/bin/env python3
"""
Render an engineering-style sketch of the Husky+M1013 + cotton field
setup to PNG and SVG. Uses real meters with equal aspect so dimensions
are visually trustworthy.

Output:
  src/docs/sketches/system_topdown.{png,svg}
  src/docs/sketches/system_side.{png,svg}
  src/docs/sketches/system_summary.{png,svg}   ← 2×2 combined panel

Source dimensions: pulled from URDF and generate_cotton_demo.py defaults.
If those change, just re-run this script.
"""

from __future__ import annotations

import math
import os
from typing import Optional

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# -------- All dimensions in meters (mirror SYSTEM_GEOMETRY.md) --------
HUSKY_LX, HUSKY_LY, HUSKY_LZ = 0.99, 0.67, 0.30
WHEEL_R, WHEEL_W = 0.165, 0.10
WHEEL_BASE, WHEEL_TRACK = 0.51, 0.555
BODY_Z_MIN = 0.13
DECK_Z = BODY_Z_MIN + HUSKY_LZ                  # 0.43

ARM_MOUNT_X = 0.20
ARM_MOUNT_LX = ARM_MOUNT_LY = 0.20
ARM_MOUNT_LZ = 0.05
ARM_BASE_Z = DECK_Z + ARM_MOUNT_LZ              # 0.48
ARM_REACH = 0.85                                 # M1013 nominal max reach

RESERVOIR_X = -0.30
RESERVOIR_LX, RESERVOIR_LY, RESERVOIR_LZ = 0.40, 0.40, 0.20
RESERVOIR_CENTER_Z = DECK_Z + RESERVOIR_LZ / 2  # 0.53

# Cluster grid (world frame)
ROW1_Y = 0.0
ROW2_Y = 6.0
COL_SP = 3.0
AISLE_Y = 1.0                                    # scout_y / spawn_y

ROW1_CLUSTERS = [
    ('cluster_A_01', 0.0,  'B_mixed_green',  'unripe_g'),
    ('cluster_B_01', 3.0,  'D_sparse_green', 'unripe_g'),
    ('cluster_C_01', 6.0,  'C_mixed_brown',  'dry_b'),
    ('cluster_A_02', 9.0,  'F_dry_brown',    'dry_b'),
    ('cluster_B_02', 12.0, 'A_mature_white', 'ripe'),
    ('cluster_C_02', 15.0, 'E_tall_mature',  'ripe'),
]
ROW2_CLUSTERS = [(name, x + COL_SP / 2.0, var, kind)
                 for (name, x, var, kind) in ROW1_CLUSTERS]
# Renaming row 2 ids
ROW2_CLUSTERS = [
    ('cluster_A_03', 1.5,  'B_mixed_green',  'unripe_g'),
    ('cluster_B_03', 4.5,  'D_sparse_green', 'unripe_g'),
    ('cluster_C_03', 7.5,  'C_mixed_brown',  'dry_b'),
    ('cluster_A_04', 10.5, 'F_dry_brown',    'dry_b'),
    ('cluster_B_04', 13.5, 'A_mature_white', 'ripe'),
    ('cluster_C_04', 16.5, 'E_tall_mature',  'ripe'),
]

# Variant heights after normalization (max boll Z = 1.51 m for all)
VARIANT_MAX_Z_AFTER_NORM = 1.51

# Per-variant: (native_max_z, effective_scale, boll_count)
VARIANTS = {
    'A_mature_white':  (0.614, 3.0 * 0.821, 6),
    'B_mixed_green':   (0.690, 3.0 * 0.730, 4),
    'C_mixed_brown':   (0.587, 3.0 * 0.859, 4),
    'D_sparse_green':  (0.504, 3.0 * 1.000, 3),
    'E_tall_mature':   (0.825, 3.0 * 0.611, 7),
    'F_dry_brown':     (0.543, 3.0 * 0.928, 4),
}

CLUSTER_COLOR = {
    'unripe_g': '#7cb342',   # immature green bolls visible
    'dry_b':    '#a1887f',   # dry brown bolls visible
    'ripe':     '#fafafa',   # all mature white
}
HUSKY_COLOR    = '#f2a900'
WHEEL_COLOR    = '#1a1a1a'
ARM_COLOR      = '#4d4d4d'
RESERVOIR_COLOR = '#3f51b5'
REACH_COLOR    = '#e57373'

OUT_DIR = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'docs', 'sketches'))


# ─── Drawing helpers ────────────────────────────────────────────

def draw_husky_topdown(ax, x0: float, y0: float, yaw: float = 0.0):
    """Draw Husky body + wheels + arm mount + reservoir as a top-down."""
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    def world(local_x, local_y):
        return (x0 + cos_y * local_x - sin_y * local_y,
                y0 + sin_y * local_x + cos_y * local_y)

    # Body
    body = mpatches.Rectangle(
        world(-HUSKY_LX / 2, -HUSKY_LY / 2),
        HUSKY_LX, HUSKY_LY,
        angle=math.degrees(yaw),
        facecolor=HUSKY_COLOR, edgecolor='black', linewidth=1.2, alpha=0.7)
    ax.add_patch(body)

    # Wheels (4)
    for sx in (-WHEEL_BASE / 2, +WHEEL_BASE / 2):
        for sy in (-WHEEL_TRACK / 2, +WHEEL_TRACK / 2):
            wx, wy = world(sx - WHEEL_R, sy - WHEEL_W / 2)
            w = mpatches.Rectangle(
                (wx, wy), 2 * WHEEL_R, WHEEL_W,
                angle=math.degrees(yaw),
                facecolor=WHEEL_COLOR, edgecolor='black', linewidth=0.6)
            ax.add_patch(w)

    # Arm mount footprint
    am = mpatches.Rectangle(
        world(ARM_MOUNT_X - ARM_MOUNT_LX / 2, -ARM_MOUNT_LY / 2),
        ARM_MOUNT_LX, ARM_MOUNT_LY,
        angle=math.degrees(yaw),
        facecolor=ARM_COLOR, edgecolor='black', linewidth=0.8, alpha=0.9)
    ax.add_patch(am)

    # Reservoir footprint
    res = mpatches.Rectangle(
        world(RESERVOIR_X - RESERVOIR_LX / 2, -RESERVOIR_LY / 2),
        RESERVOIR_LX, RESERVOIR_LY,
        angle=math.degrees(yaw),
        facecolor=RESERVOIR_COLOR, edgecolor='black', linewidth=0.8, alpha=0.65)
    ax.add_patch(res)

    # Heading arrow
    hx, hy = world(HUSKY_LX / 2 + 0.15, 0)
    ax.annotate('', xy=(hx, hy), xytext=(x0, y0),
                arrowprops=dict(arrowstyle='->', color='black', lw=1.0))

    # Center marker (wheel-contact center = base_link XY)
    ax.plot([x0], [y0], 'k+', markersize=10, markeredgewidth=1.5)


def draw_clusters_topdown(ax):
    for cid, cx, variant, kind in ROW1_CLUSTERS + ROW2_CLUSTERS:
        color = CLUSTER_COLOR[kind]
        cy = ROW1_Y if cid.endswith(('_01', '_02')) else ROW2_Y
        cy = ROW1_Y if cid in [r[0] for r in ROW1_CLUSTERS] else ROW2_Y
        circ = mpatches.Circle(
            (cx, cy), 0.25,
            facecolor=color, edgecolor='black', linewidth=0.8, alpha=0.85)
        ax.add_patch(circ)
        # Label below circle
        ax.text(cx, cy - 0.55,
                cid.replace('cluster_', '') + f'\n{variant[:1]}',
                ha='center', va='top', fontsize=6.5, family='monospace')


def draw_husky_sideview(ax, x0: float, y0: float = 0.0):
    """Draw Husky from the side (looking along −Y). x0 = Husky world X."""
    # Body (z = body_z_min to body_z_max)
    body = mpatches.Rectangle(
        (x0 - HUSKY_LX / 2, BODY_Z_MIN), HUSKY_LX, HUSKY_LZ,
        facecolor=HUSKY_COLOR, edgecolor='black', linewidth=1.0, alpha=0.7)
    ax.add_patch(body)

    # Wheels (just two visible from the side)
    for sx in (-WHEEL_BASE / 2, +WHEEL_BASE / 2):
        w = mpatches.Circle(
            (x0 + sx, WHEEL_R), WHEEL_R,
            facecolor=WHEEL_COLOR, edgecolor='black', linewidth=0.6, alpha=0.9)
        ax.add_patch(w)

    # Deck line
    ax.axhline(DECK_Z, x0 - HUSKY_LX / 2, x0 + HUSKY_LX / 2,
               color='black', lw=0.5, linestyle=':')

    # Arm mount
    am = mpatches.Rectangle(
        (x0 + ARM_MOUNT_X - ARM_MOUNT_LX / 2, DECK_Z),
        ARM_MOUNT_LX, ARM_MOUNT_LZ,
        facecolor=ARM_COLOR, edgecolor='black', linewidth=0.6)
    ax.add_patch(am)

    # Arm: stylized stem + extended into canopy (showing reach envelope)
    arm_base_x = x0 + ARM_MOUNT_X
    arm_base_z = ARM_BASE_Z
    # vertical "tower" segment
    ax.plot([arm_base_x, arm_base_x], [arm_base_z, arm_base_z + 0.4],
            color=ARM_COLOR, lw=4)
    # angled "forearm" toward canopy
    ext_x = arm_base_x - 0.55
    ext_z = arm_base_z + 0.85
    ax.plot([arm_base_x, ext_x], [arm_base_z + 0.4, ext_z],
            color=ARM_COLOR, lw=4)
    # gripper
    ax.plot([ext_x], [ext_z], 'o', color='black', markersize=6)
    ax.text(ext_x + 0.05, ext_z, 'TCP', fontsize=7, va='center')

    # Reservoir (side rectangle)
    res = mpatches.Rectangle(
        (x0 + RESERVOIR_X - RESERVOIR_LX / 2, DECK_Z),
        RESERVOIR_LX, RESERVOIR_LZ,
        facecolor=RESERVOIR_COLOR, edgecolor='black',
        linewidth=0.8, alpha=0.65)
    ax.add_patch(res)


def draw_plant_sideview(ax, x_anchor: float, max_boll_z: float,
                        boll_count: int, kind: str):
    """Stylized plant: stem + bolls clustered on top."""
    color = CLUSTER_COLOR[kind]
    # Stem
    ax.plot([x_anchor, x_anchor], [0, max_boll_z * 0.95],
            color='#5d4037', lw=2)
    # Bolls (scatter along upper canopy)
    for i in range(boll_count):
        bz = max_boll_z - 0.05 - 0.18 * (i / max(1, boll_count - 1))
        bx = x_anchor + 0.08 * (-1 if i % 2 else 1)
        b = mpatches.Circle((bx, bz), 0.07,
                            facecolor=color, edgecolor='black', linewidth=0.6)
        ax.add_patch(b)


# ─── Sketches ──────────────────────────────────────────────────

def sketch_topdown(ax):
    """Top-down view of the cotton field with Husky in the aisle."""
    # Clusters
    draw_clusters_topdown(ax)

    # Aisle line (Husky drive path)
    ax.plot([-1, 17], [AISLE_Y, AISLE_Y],
            color='#999', lw=0.7, linestyle='--')
    ax.text(17.2, AISLE_Y, 'Husky aisle\n(Y=1.0)', fontsize=7,
            va='center', color='#666')

    # Husky at spawn
    draw_husky_topdown(ax, 0, AISLE_Y, yaw=0.0)

    # Row labels
    ax.text(-1.0, ROW1_Y, 'Row 1', fontsize=9, va='center',
            ha='right', color='black')
    ax.text(-1.0, ROW2_Y, 'Row 2', fontsize=9, va='center',
            ha='right', color='black')

    # Dimension annotations
    ax.annotate('', xy=(3, -0.9), xytext=(0, -0.9),
                arrowprops=dict(arrowstyle='<->', color='black'))
    ax.text(1.5, -1.1, '3.0 m', ha='center', fontsize=7)

    ax.annotate('', xy=(0.0, 6.0), xytext=(0.0, 0.0),
                arrowprops=dict(arrowstyle='<->', color='black'))
    ax.text(-0.15, 3.0, '6.0 m', rotation=90, va='center', fontsize=7)

    # Legend
    legend_handles = [
        mpatches.Patch(color=HUSKY_COLOR, label='Husky body'),
        mpatches.Patch(color=ARM_COLOR, label='Arm mount'),
        mpatches.Patch(color=RESERVOIR_COLOR, label='Reservoir', alpha=0.65),
        mpatches.Patch(color=CLUSTER_COLOR['unripe_g'],
                       label='Cluster (B/D = green unripe)'),
        mpatches.Patch(color=CLUSTER_COLOR['dry_b'],
                       label='Cluster (C/F = brown dry)'),
        mpatches.Patch(color=CLUSTER_COLOR['ripe'],
                       label='Cluster (A/E = full mature)'),
    ]
    ax.legend(handles=legend_handles, loc='upper left',
              fontsize=7, framealpha=0.95)

    ax.set_xlim(-2, 19)
    ax.set_ylim(-2, 8)
    ax.set_aspect('equal')
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_xlabel('X (m, world)')
    ax.set_ylabel('Y (m, world)')
    ax.set_title('Top-down: cotton field + Husky aisle (X right, Y left)')


def sketch_side(ax):
    """Side view at cluster_B_01 (variant D_sparse_green) showing
    Husky + arm reaching into the canopy."""
    cluster_x = 3.0  # cluster_B_01

    # Husky sits 1 m to +Y of the cluster (we're looking along −Y so
    # in this XZ view the Husky appears AT cluster_x in X).
    draw_husky_sideview(ax, cluster_x)

    # Plant
    draw_plant_sideview(
        ax, cluster_x, VARIANT_MAX_Z_AFTER_NORM, boll_count=3,
        kind='unripe_g')

    # Arm reach envelope (arc from arm base)
    arm_base_x = cluster_x + ARM_MOUNT_X
    th = [math.pi / 2 - 1.0 + 0.05 * i for i in range(40)]
    rx = [arm_base_x + ARM_REACH * math.cos(t) for t in th]
    rz = [ARM_BASE_Z + ARM_REACH * math.sin(t) for t in th]
    ax.plot(rx, rz, color=REACH_COLOR, lw=0.8, linestyle='--')
    ax.text(arm_base_x + 0.3, ARM_BASE_Z + 0.95,
            f'arm reach ≈ {ARM_REACH:.2f} m',
            color=REACH_COLOR, fontsize=7)

    # Reach max Z line
    ax.axhline(ARM_BASE_Z + ARM_REACH, color=REACH_COLOR,
               lw=0.5, linestyle=':')
    ax.text(cluster_x - 1.1, ARM_BASE_Z + ARM_REACH + 0.03,
            f'reach max z={ARM_BASE_Z + ARM_REACH:.2f} m',
            color=REACH_COLOR, fontsize=6)

    # Max boll z line
    ax.axhline(VARIANT_MAX_Z_AFTER_NORM, color='#2e7d32',
               lw=0.5, linestyle=':')
    ax.text(cluster_x - 1.1, VARIANT_MAX_Z_AFTER_NORM + 0.03,
            f'top boll z={VARIANT_MAX_Z_AFTER_NORM:.2f} m',
            color='#2e7d32', fontsize=6)

    # Ground
    ax.axhline(0, color='black', lw=0.8)

    # Z annotations
    for z, label in [(DECK_Z, 'deck'),
                     (ARM_BASE_Z, 'arm base'),
                     (RESERVOIR_CENTER_Z, 'reservoir top')]:
        ax.text(cluster_x - 1.5, z, f'{label} z={z:.2f}',
                fontsize=6, va='center', color='#444')

    ax.set_xlim(cluster_x - 1.8, cluster_x + 1.0)
    ax.set_ylim(-0.1, 2.0)
    ax.set_aspect('equal')
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Side view (along −Y): Husky + arm + plant @ cluster_B_01')


def sketch_variant_heights(ax):
    """Bar chart: native vs effective max boll Z per variant."""
    labels = list(VARIANTS.keys())
    native_max = [VARIANTS[k][0] for k in labels]
    eff_scale  = [VARIANTS[k][1] for k in labels]
    eff_max    = [n * s / 3.0 * 3.0 for n, s in zip(native_max, eff_scale)]
    # Wait: effective max = native_max * eff_scale (since scale 3 already in eff_scale)
    eff_max    = [n * s for n, s in zip(native_max, eff_scale)]

    x = list(range(len(labels)))
    ax.bar([i - 0.18 for i in x], [n * 3 for n in native_max],
           width=0.35, label='At raw scale=3 (no norm)',
           color='#bdbdbd', edgecolor='black', linewidth=0.6)
    ax.bar([i + 0.18 for i in x], eff_max,
           width=0.35, label='With per-variant norm (target D)',
           color='#43a047', edgecolor='black', linewidth=0.6)

    ax.axhline(ARM_BASE_Z + ARM_REACH, color=REACH_COLOR,
               lw=1.0, linestyle='--',
               label=f'arm reach max z = {ARM_BASE_Z + ARM_REACH:.2f} m')
    ax.axhline(VARIANT_MAX_Z_AFTER_NORM, color='#2e7d32',
               lw=0.6, linestyle=':',
               label=f'normalized top z = {VARIANT_MAX_Z_AFTER_NORM} m')

    ax.set_xticks(x)
    ax.set_xticklabels([l.replace('_', '\n', 1) for l in labels],
                       fontsize=7, rotation=0)
    ax.set_ylabel('Max boll Z (m, world)')
    ax.set_title('Cluster heights: raw vs normalized (M1013 reach budget)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, axis='y', linewidth=0.3, alpha=0.5)


def sketch_scan_sweep(ax):
    """Visualize the pan × tilt scan grid as polar-like cone."""
    pans = [-12, 0, 12]
    tilts = [-32, -24, -16, -8, 0, 8]

    # Origin at "arm" symbolically
    for p in pans:
        for t in tilts:
            # Convert pan/tilt to a 2D fan in the YZ plane (we view from arm)
            # x ~ pan offset, y ~ tilt direction
            ax.plot([0, math.sin(math.radians(p))],
                    [0, -math.sin(math.radians(t))],
                    color='#0288d1', lw=0.4, alpha=0.7)
            ax.plot(math.sin(math.radians(p)), -math.sin(math.radians(t)),
                    'o', color='#0277bd', markersize=3)

    # Mark canopy band
    canopy_top = -math.sin(math.radians(-32))
    canopy_mid = 0
    ax.axhline(canopy_top, color='#2e7d32', lw=0.5, linestyle=':')
    ax.text(0.3, canopy_top + 0.01,
            'tilt −32° (top of canopy)', color='#2e7d32', fontsize=7)
    ax.axhline(0, color='gray', lw=0.4)
    ax.text(0.3, 0.01, 'level (z = camera)', color='gray', fontsize=7)

    ax.set_xlim(-0.3, 0.4)
    ax.set_ylim(-0.2, 0.7)
    ax.set_aspect('equal')
    ax.set_xlabel('sin(pan)')
    ax.set_ylabel('sin(tilt) [up positive]')
    ax.set_title(f'Scan grid: {len(pans)} pan × {len(tilts)} tilt = '
                 f'{len(pans) * len(tilts)} poses')
    ax.grid(True, linewidth=0.3, alpha=0.5)


# ─── Main render ───────────────────────────────────────────────

def render_combined() -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))

    sketch_topdown(axes[0, 0])
    sketch_side(axes[0, 1])
    sketch_variant_heights(axes[1, 0])
    sketch_scan_sweep(axes[1, 1])

    fig.suptitle('Cotton Harvesting Simulator — System Geometry',
                 fontsize=12, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    os.makedirs(OUT_DIR, exist_ok=True)
    for ext in ('png', 'svg'):
        out = os.path.join(OUT_DIR, f'system_summary.{ext}')
        fig.savefig(out, dpi=180 if ext == 'png' else None,
                    bbox_inches='tight')
        print(f'  Wrote {out}')
    plt.close(fig)


def render_solo(name: str, sketch_fn, w: float, h: float) -> None:
    fig, ax = plt.subplots(figsize=(w, h))
    sketch_fn(ax)
    fig.tight_layout()
    os.makedirs(OUT_DIR, exist_ok=True)
    for ext in ('png', 'svg'):
        out = os.path.join(OUT_DIR, f'{name}.{ext}')
        fig.savefig(out, dpi=180 if ext == 'png' else None,
                    bbox_inches='tight')
        print(f'  Wrote {out}')
    plt.close(fig)


def main() -> None:
    render_combined()
    render_solo('system_topdown', sketch_topdown, 12, 5)
    render_solo('system_side', sketch_side, 7, 6)
    render_solo('variant_heights', sketch_variant_heights, 9, 5)
    render_solo('scan_sweep', sketch_scan_sweep, 6, 6)


if __name__ == '__main__':
    main()
