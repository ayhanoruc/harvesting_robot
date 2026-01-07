#!/usr/bin/env python3
"""
Generate FOV Overlap Diagram (Figure 11) for RoboCot Report

This script generates a top-down view showing:
- Robot base position at origin
- 7 camera FOV cones at different pan angles
- Overlapping regions between FOV cones
- Cotton cluster positions

Parameters match the actual simulation settings from:
- explorer.py: pan_hip_angles = [-0.78, -0.52, -0.26, 0.0, 0.26, 0.52, 0.78] rad
- environment_config.yaml: cluster positions, scan_distance
- Camera specs: 90° horizontal FOV
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Wedge, Circle, FancyArrowPatch
from matplotlib.collections import PatchCollection
import matplotlib.colors as mcolors

# =============================================================================
# SIMULATION PARAMETERS (from explorer.py and environment_config.yaml)
# =============================================================================

# Pan hip angles from explorer.py (line 100)
PAN_HIP_ANGLES_RAD = [-0.78, -0.52, -0.26, 0.0, 0.26, 0.52, 0.78]
PAN_HIP_ANGLES_DEG = [np.degrees(a) for a in PAN_HIP_ANGLES_RAD]

# Camera FOV from Table 22 in report
HORIZONTAL_FOV_DEG = 90.0

# Scan configuration from environment_config.yaml (lines 63-64)
SCAN_DISTANCE = 0.4  # X distance from base during scan (camera position)
SCAN_HEIGHT = 0.55   # Z height during scanning (not used in top-down view)

# Camera viewing range for visualization (depth range from Table 22)
# Using a shorter range for cleaner visualization
FOV_DISPLAY_RANGE = 0.8  # meters - how far to draw the FOV wedges

# Cluster positions from environment_config.yaml (lines 44-57)
CLUSTERS = {
    'cluster_1': {'position': [0.875, 0.475, 0.46], 'label': 'Left'},
    'cluster_2': {'position': [0.975, 0.0, 0.52], 'label': 'Center'},
    'cluster_3': {'position': [0.875, -0.475, 0.42], 'label': 'Right'},
}

# Robot base position
ROBOT_BASE = [0.0, 0.0]

# =============================================================================
# FIGURE GENERATION
# =============================================================================

def create_fov_overlap_diagram(save_path='figure_11_fov_overlap.svg'):
    """Generate the FOV overlap diagram."""

    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    # Colors for FOV wedges (using colormap for gradation)
    cmap = plt.cm.Blues
    fov_colors = [cmap(0.3 + 0.08 * i) for i in range(len(PAN_HIP_ANGLES_RAD))]

    # Alternative: distinct colors for each position
    # fov_colors = ['#cce5ff', '#99ccff', '#66b3ff', '#3399ff', '#0080ff', '#0066cc', '#004c99']

    # ==========================================================================
    # Draw FOV wedges
    # ==========================================================================

    # The camera position during panoramic scan:
    # - Hip rotation rotates the entire arm around Z-axis
    # - Camera is at the end of the arm, roughly scan_distance from base
    # - Camera looks forward (in the direction the arm is pointing)

    fov_patches = []

    for i, hip_angle_rad in enumerate(PAN_HIP_ANGLES_RAD):
        hip_angle_deg = np.degrees(hip_angle_rad)

        # Camera position (rotates with hip)
        # During scan, camera is roughly at scan_distance from base
        cam_x = SCAN_DISTANCE * np.cos(hip_angle_rad)
        cam_y = SCAN_DISTANCE * np.sin(hip_angle_rad)

        # Camera looks forward (in direction of arm)
        # The viewing direction is perpendicular to the Y-axis of the camera
        # For our setup, camera looks in +X direction rotated by hip angle
        view_direction_deg = hip_angle_deg  # 0° means looking in +X

        # FOV wedge angles
        # Wedge is drawn from theta1 to theta2 (counterclockwise)
        half_fov = HORIZONTAL_FOV_DEG / 2
        theta1 = view_direction_deg - half_fov
        theta2 = view_direction_deg + half_fov

        # Create wedge patch
        wedge = Wedge(
            center=(cam_x, cam_y),
            r=FOV_DISPLAY_RANGE,
            theta1=theta1,
            theta2=theta2,
            alpha=0.25,
            facecolor=fov_colors[i],
            edgecolor='steelblue',
            linewidth=0.5,
            linestyle='-'
        )
        ax.add_patch(wedge)
        fov_patches.append(wedge)

        # Draw camera position marker
        ax.plot(cam_x, cam_y, 'o', color='navy', markersize=6, zorder=10)

        # Add angle label near camera position
        label_offset = 0.05
        label_x = cam_x - label_offset * np.sin(hip_angle_rad)
        label_y = cam_y + label_offset * np.cos(hip_angle_rad)
        ax.annotate(
            f'{hip_angle_deg:+.0f}°',
            (label_x, label_y),
            fontsize=8,
            ha='center',
            va='center',
            color='darkblue',
            fontweight='bold'
        )

    # ==========================================================================
    # Draw cluster positions
    # ==========================================================================

    cluster_colors = {'cluster_1': '#2ca02c', 'cluster_2': '#d62728', 'cluster_3': '#ff7f0e'}

    for name, data in CLUSTERS.items():
        x, y, z = data['position']
        label = data['label']

        # Draw cluster marker (larger circle with fill)
        circle = Circle(
            (x, y),
            radius=0.04,
            facecolor=cluster_colors[name],
            edgecolor='black',
            linewidth=2,
            zorder=15
        )
        ax.add_patch(circle)

        # Add label
        ax.annotate(
            f'{name}\n({label})',
            (x, y + 0.08),
            fontsize=9,
            ha='center',
            va='bottom',
            fontweight='bold',
            color='black'
        )

        # Add coordinates
        ax.annotate(
            f'({x:.2f}, {y:.2f})',
            (x, y - 0.08),
            fontsize=7,
            ha='center',
            va='top',
            color='gray'
        )

    # ==========================================================================
    # Draw robot base
    # ==========================================================================

    # Robot base as a square
    base_size = 0.08
    robot_rect = mpatches.FancyBboxPatch(
        (-base_size/2, -base_size/2),
        base_size, base_size,
        boxstyle="round,pad=0.01",
        facecolor='gray',
        edgecolor='black',
        linewidth=2,
        zorder=20
    )
    ax.add_patch(robot_rect)
    ax.annotate('Robot\nBase', (0, -0.12), fontsize=9, ha='center', va='top', fontweight='bold')

    # ==========================================================================
    # Draw arc showing camera positions
    # ==========================================================================

    # Draw arc connecting camera positions
    arc_angles = np.linspace(PAN_HIP_ANGLES_RAD[0], PAN_HIP_ANGLES_RAD[-1], 50)
    arc_x = SCAN_DISTANCE * np.cos(arc_angles)
    arc_y = SCAN_DISTANCE * np.sin(arc_angles)
    ax.plot(arc_x, arc_y, '--', color='navy', linewidth=1.5, alpha=0.5, label='Camera arc')

    # ==========================================================================
    # Add overlap region highlighting
    # ==========================================================================

    # Calculate and annotate overlap
    overlap_angle = 2 * (HORIZONTAL_FOV_DEG/2 - 15)  # Adjacent positions are 15° apart

    # Add text box with FOV analysis
    textstr = (
        f'FOV Analysis\n'
        f'─────────────\n'
        f'Horizontal FOV: {HORIZONTAL_FOV_DEG:.0f}°\n'
        f'Pan increment: 15°\n'
        f'Adjacent overlap: {overlap_angle:.0f}°\n'
        f'Total coverage: ±{abs(PAN_HIP_ANGLES_DEG[0]) + HORIZONTAL_FOV_DEG/2:.0f}°'
    )
    props = dict(boxstyle='round,pad=0.5', facecolor='lightyellow', edgecolor='gray', alpha=0.9)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', fontfamily='monospace', bbox=props)

    # ==========================================================================
    # Add scale and legend
    # ==========================================================================

    # Grid
    ax.set_axisbelow(True)
    ax.grid(True, linestyle='--', alpha=0.3)

    # Axis settings
    ax.set_xlim(-0.3, 1.3)
    ax.set_ylim(-0.8, 0.8)
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)', fontsize=11)
    ax.set_ylabel('Y (meters)', fontsize=11)
    ax.set_title('Figure 11: FOV Overlap Diagram (Top-Down View)\n7 Pan Positions × 90° Horizontal FOV',
                 fontsize=12, fontweight='bold')

    # Legend
    legend_elements = [
        mpatches.Patch(facecolor=fov_colors[3], edgecolor='steelblue', alpha=0.4, label='Camera FOV (90°)'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='navy', markersize=8, label='Camera positions'),
        plt.Line2D([0], [0], linestyle='--', color='navy', alpha=0.5, label='Camera arc path'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10,
                   markeredgecolor='black', markeredgewidth=2, label='Cotton clusters'),
    ]
    ax.legend(handles=legend_elements, loc='lower right', fontsize=9)

    # Add scale bar
    scale_bar_x = 1.0
    scale_bar_y = -0.7
    scale_length = 0.2
    ax.plot([scale_bar_x, scale_bar_x + scale_length], [scale_bar_y, scale_bar_y],
            'k-', linewidth=3)
    ax.annotate(f'{scale_length*100:.0f} cm',
                (scale_bar_x + scale_length/2, scale_bar_y - 0.03),
                ha='center', va='top', fontsize=8)

    # ==========================================================================
    # Save figure
    # ==========================================================================

    plt.tight_layout()

    # Save as SVG (vector) and PNG (raster)
    svg_path = save_path
    png_path = save_path.replace('.svg', '.png')

    plt.savefig(svg_path, format='svg', dpi=150, bbox_inches='tight')
    plt.savefig(png_path, format='png', dpi=300, bbox_inches='tight')

    print(f"Saved: {svg_path}")
    print(f"Saved: {png_path}")

    plt.show()

    return fig, ax


def create_simplified_fov_diagram(save_path='figure_11_fov_overlap_simple.svg'):
    """
    Generate a cleaner, more schematic version of the FOV diagram.
    Better suited for academic reports.
    """

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    # Use only 3 representative positions (left, center, right) for clarity
    # But show all 7 camera positions

    # Draw all FOV wedges with very light fill
    for i, hip_angle_rad in enumerate(PAN_HIP_ANGLES_RAD):
        hip_angle_deg = np.degrees(hip_angle_rad)

        cam_x = SCAN_DISTANCE * np.cos(hip_angle_rad)
        cam_y = SCAN_DISTANCE * np.sin(hip_angle_rad)

        half_fov = HORIZONTAL_FOV_DEG / 2
        theta1 = hip_angle_deg - half_fov
        theta2 = hip_angle_deg + half_fov

        # All wedges in light blue
        wedge = Wedge(
            center=(cam_x, cam_y),
            r=FOV_DISPLAY_RANGE,
            theta1=theta1,
            theta2=theta2,
            alpha=0.15,
            facecolor='steelblue',
            edgecolor='steelblue',
            linewidth=1,
            linestyle='-'
        )
        ax.add_patch(wedge)

        # Camera position
        ax.plot(cam_x, cam_y, 's', color='navy', markersize=5, zorder=10)

    # Highlight the overlap regions by drawing darker where multiple FOVs intersect
    # This is done by drawing the central FOV with slightly higher alpha
    center_idx = len(PAN_HIP_ANGLES_RAD) // 2
    for i in [center_idx - 1, center_idx, center_idx + 1]:
        hip_angle_rad = PAN_HIP_ANGLES_RAD[i]
        hip_angle_deg = np.degrees(hip_angle_rad)

        cam_x = SCAN_DISTANCE * np.cos(hip_angle_rad)
        cam_y = SCAN_DISTANCE * np.sin(hip_angle_rad)

        half_fov = HORIZONTAL_FOV_DEG / 2
        theta1 = hip_angle_deg - half_fov
        theta2 = hip_angle_deg + half_fov

        wedge = Wedge(
            center=(cam_x, cam_y),
            r=FOV_DISPLAY_RANGE,
            theta1=theta1,
            theta2=theta2,
            alpha=0.1,
            facecolor='steelblue',
            edgecolor='none'
        )
        ax.add_patch(wedge)

    # Draw clusters
    cluster_markers = {'cluster_1': '^', 'cluster_2': 'o', 'cluster_3': 'v'}

    for name, data in CLUSTERS.items():
        x, y, z = data['position']
        label = data['label']

        ax.plot(x, y, cluster_markers[name], color='red', markersize=15,
                markeredgecolor='darkred', markeredgewidth=2, zorder=20)
        ax.annotate(f'{label}\n({x:.2f}, {y:.2f})', (x + 0.08, y),
                    fontsize=9, va='center', ha='left')

    # Robot base
    ax.plot(0, 0, 's', color='black', markersize=12, zorder=20)
    ax.annotate('Robot', (0, -0.1), fontsize=10, ha='center', va='top', fontweight='bold')

    # Pan angle labels
    for i, (hip_rad, hip_deg) in enumerate(zip(PAN_HIP_ANGLES_RAD, PAN_HIP_ANGLES_DEG)):
        cam_x = SCAN_DISTANCE * np.cos(hip_rad)
        cam_y = SCAN_DISTANCE * np.sin(hip_rad)

        # Position label
        ax.annotate(f'P{i+1}\n{hip_deg:+.0f}°',
                    (cam_x, cam_y + 0.06),
                    fontsize=7, ha='center', va='bottom', color='darkblue')

    # Axis settings
    ax.set_xlim(-0.2, 1.2)
    ax.set_ylim(-0.7, 0.7)
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)', fontsize=11)
    ax.set_ylabel('Y (meters)', fontsize=11)
    ax.set_title('Figure 11: Camera FOV Coverage During Panoramic Scan',
                 fontsize=12, fontweight='bold')

    ax.grid(True, linestyle=':', alpha=0.4)

    # Annotation
    ax.annotate(
        f'7 positions (±45° @ 15° steps)\n90° FOV → 60° overlap',
        (0.02, 0.98), xycoords='axes fraction',
        fontsize=9, va='top', ha='left',
        bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray', alpha=0.8)
    )

    plt.tight_layout()

    svg_path = save_path
    png_path = save_path.replace('.svg', '.png')

    plt.savefig(svg_path, format='svg', dpi=150, bbox_inches='tight')
    plt.savefig(png_path, format='png', dpi=300, bbox_inches='tight')

    print(f"Saved: {svg_path}")
    print(f"Saved: {png_path}")

    plt.show()

    return fig, ax


if __name__ == '__main__':
    print("Generating FOV Overlap Diagrams...")
    print("=" * 50)
    print(f"Parameters from simulation:")
    print(f"  Pan angles: {[f'{d:+.0f}°' for d in PAN_HIP_ANGLES_DEG]}")
    print(f"  Horizontal FOV: {HORIZONTAL_FOV_DEG}°")
    print(f"  Scan distance: {SCAN_DISTANCE}m")
    print(f"  Clusters: {list(CLUSTERS.keys())}")
    print("=" * 50)

    # Generate detailed version
    print("\n[1/2] Generating detailed FOV diagram...")
    create_fov_overlap_diagram('figure_11_fov_overlap.svg')

    # Generate simplified version
    print("\n[2/2] Generating simplified FOV diagram...")
    create_simplified_fov_diagram('figure_11_fov_overlap_simple.svg')

    print("\nDone!")
