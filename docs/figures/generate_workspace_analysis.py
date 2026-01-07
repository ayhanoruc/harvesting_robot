#!/usr/bin/env python3
"""
Generate Workspace Reachability Analysis (Figure 5) for RoboCot Report

Creates two views:
(a) Top view (XY plane) with cluster positions marked
(b) Side view (XZ plane) showing height range

Uses Braccio arm parameters from URDF:
- braccio_arm.xacro.urdf

Environment settings from:
- environment_config.yaml
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Wedge, FancyBboxPatch
from matplotlib.collections import PatchCollection
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D

# =============================================================================
# BRACCIO ARM PARAMETERS (from braccio_arm.xacro.urdf)
# =============================================================================

# Link lengths (meters) - from URDF joint origins
L_BASE = 0.072      # base_link to shoulder (Z offset in shoulder_joint origin)
L_SHOULDER = 0.125  # shoulder to elbow (Z offset in elbow_joint origin)
L_ELBOW = 0.125     # elbow to wrist_pitch (Z offset in wrist_pitch_joint origin)
L_WRIST_PITCH = 0.06   # wrist_pitch to wrist_roll
L_WRIST_ROLL = 0.03    # wrist_roll to gripper base
L_GRIPPER = 0.08       # gripper reach (approximate)

# Total arm length (fully extended)
TOTAL_ARM_LENGTH = L_BASE + L_SHOULDER + L_ELBOW + L_WRIST_PITCH + L_WRIST_ROLL + L_GRIPPER
# = 0.072 + 0.125 + 0.125 + 0.06 + 0.03 + 0.08 = 0.492m

# For workspace calculation, use effective reach (shoulder to end effector)
EFFECTIVE_REACH = L_SHOULDER + L_ELBOW + L_WRIST_PITCH + L_WRIST_ROLL + L_GRIPPER
# = 0.125 + 0.125 + 0.06 + 0.03 + 0.08 = 0.42m

# Joint limits (radians) - from URDF
JOINT_LIMITS = {
    'base': (0.05, 5.0),           # ~3° to ~286° (Z-axis rotation)
    'shoulder': (1.6, 4.0),        # ~92° to ~229° (X-axis, with -2.8 rad offset)
    'elbow': (1.0, 4.6),           # ~57° to ~264° (X-axis, with -2.8 rad offset)
    'wrist_pitch': (0.77, 4.8),    # ~44° to ~275° (X-axis, with -2.8 rad offset)
}

# Robot spawn position (from environment_config.yaml)
ROBOT_SPAWN_Z = 0.1  # Robot base is 0.1m above ground

# =============================================================================
# ENVIRONMENT PARAMETERS
# =============================================================================

# ADJUSTED cluster positions to fit within Braccio arm workspace
# - Base joint range: 3° to 286° (mostly upper-left quadrant)
# - Effective reach: ~0.42m, so clusters at ~0.25-0.32m from base
# - Placed within the workspace wedge (positive Y region)
# - Heights: 0.25-0.35m (within comfortable arm height range)

CLUSTERS = {
    'cluster_1': {'position': [0.15, 0.28, 0.30], 'label': 'Left', 'color': '#2ca02c'},
    'cluster_2': {'position': [0.0, 0.32, 0.35], 'label': 'Center', 'color': '#d62728'},
    'cluster_3': {'position': [-0.15, 0.28, 0.25], 'label': 'Right', 'color': '#ff7f0e'},
}

# Reservoir position - to the side
RESERVOIR = {
    'position': [-0.25, 0.0, 0.15],
    'dimensions': [0.12, 0.12, 0.12]
}

# Workspace bounds
WORKSPACE_BOUNDS = {
    'x': (-0.3, 1.2),
    'y': (-0.8, 0.8),
    'z': (0.0, 1.0)
}

# =============================================================================
# FORWARD KINEMATICS (Simplified for workspace visualization)
# =============================================================================

def forward_kinematics_simplified(theta_base, theta_shoulder, theta_elbow, theta_wrist=0):
    """
    Simplified forward kinematics for Braccio arm.

    The Braccio has a vertical base rotation and pitch joints for the rest.
    For workspace visualization, we treat it as a planar arm that rotates about Z.

    Args:
        theta_base: Base rotation about Z (rad)
        theta_shoulder: Shoulder angle (rad) - measured from vertical
        theta_elbow: Elbow angle (rad) - relative to shoulder
        theta_wrist: Wrist angle (rad) - relative to elbow

    Returns:
        (x, y, z) end effector position
    """
    # The arm angles in the URDF have offsets. For simplicity, we'll
    # interpret the angles as:
    # - theta_shoulder: angle from vertical (0 = pointing up)
    # - theta_elbow: angle relative to previous link

    # Convert from URDF convention to geometric angles
    # The -2.8 rad offset in URDF means when joint=0, the link points roughly horizontal-backward
    # We need to interpret the joint range [1.6, 4.0] for shoulder

    # For visualization, let's use a simpler model:
    # phi_shoulder = angle from horizontal (0 = horizontal forward)
    # phi_elbow = angle relative to shoulder link

    # Map joint angles to geometric angles (approximate)
    # shoulder_joint=2.8 (middle of range ~2.8) -> arm horizontal
    # We'll map [1.6, 4.0] to roughly [-70°, +70°] from horizontal

    phi_shoulder = (theta_shoulder - 2.8)  # Centered around horizontal
    phi_elbow = (theta_elbow - 2.8)        # Relative bend
    phi_wrist = (theta_wrist - 2.8) if theta_wrist != 0 else 0

    # Cumulative angle for each link
    angle1 = phi_shoulder
    angle2 = angle1 + phi_elbow
    angle3 = angle2 + phi_wrist

    # Position in the vertical plane (before base rotation)
    # Starting from base height
    r = 0  # horizontal distance
    z = L_BASE + ROBOT_SPAWN_Z  # start at base height above ground

    # Shoulder link
    r += L_SHOULDER * np.cos(angle1)
    z += L_SHOULDER * np.sin(angle1)

    # Elbow link
    r += L_ELBOW * np.cos(angle2)
    z += L_ELBOW * np.sin(angle2)

    # Wrist (simplified - treat as extension)
    wrist_length = L_WRIST_PITCH + L_WRIST_ROLL + L_GRIPPER
    r += wrist_length * np.cos(angle3)
    z += wrist_length * np.sin(angle3)

    # Apply base rotation to get XY
    x = r * np.cos(theta_base)
    y = r * np.sin(theta_base)

    return x, y, z


def compute_workspace_points(n_samples=50000):
    """
    Compute workspace by sampling joint configurations.

    Returns array of (x, y, z) reachable points.
    """
    points = []

    # Sample joint angles
    np.random.seed(42)  # For reproducibility

    for _ in range(n_samples):
        # Random joint angles within limits
        theta_base = np.random.uniform(*JOINT_LIMITS['base'])
        theta_shoulder = np.random.uniform(*JOINT_LIMITS['shoulder'])
        theta_elbow = np.random.uniform(*JOINT_LIMITS['elbow'])
        theta_wrist = np.random.uniform(*JOINT_LIMITS['wrist_pitch'])

        x, y, z = forward_kinematics_simplified(
            theta_base, theta_shoulder, theta_elbow, theta_wrist
        )

        # Only keep valid points (positive z, reasonable range)
        if z > 0 and -0.5 < x < 0.6 and -0.5 < y < 0.5:
            points.append([x, y, z])

    return np.array(points)


def compute_workspace_boundary(n_angles=100):
    """
    Compute the outer boundary of the workspace in the XY plane.

    Returns arrays of (r, theta) for plotting polar boundary.
    """
    # For each base angle, find max and min reach
    base_angles = np.linspace(JOINT_LIMITS['base'][0], JOINT_LIMITS['base'][1], n_angles)

    max_reach = []
    min_reach = []

    for theta_base in base_angles:
        # Sample shoulder and elbow to find reach extremes
        reaches = []
        for theta_s in np.linspace(*JOINT_LIMITS['shoulder'], 20):
            for theta_e in np.linspace(*JOINT_LIMITS['elbow'], 20):
                x, y, z = forward_kinematics_simplified(theta_base, theta_s, theta_e)
                r = np.sqrt(x**2 + y**2)
                if z > 0.1:  # Only count if above ground
                    reaches.append(r)

        if reaches:
            max_reach.append(max(reaches))
            min_reach.append(min(reaches))
        else:
            max_reach.append(0)
            min_reach.append(0)

    return base_angles, np.array(max_reach), np.array(min_reach)


# =============================================================================
# FIGURE GENERATION
# =============================================================================

def create_workspace_figure(save_path='figure_05_workspace_analysis.svg'):
    """Generate the workspace analysis figure with top and side views."""

    # Compute workspace points
    print("Computing workspace points...")
    points = compute_workspace_points(n_samples=80000)
    print(f"  Generated {len(points)} valid points")

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # ==========================================================================
    # (a) Top View (XY Plane)
    # ==========================================================================

    ax1.set_title('(a) Top View - Horizontal Reach', fontsize=12, fontweight='bold')

    # Plot workspace points (use alpha for density visualization)
    ax1.scatter(points[:, 0], points[:, 1], c=points[:, 2], cmap='viridis',
                alpha=0.1, s=1, rasterized=True)

    # Add colorbar for height
    # sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=0, vmax=0.8))
    # sm.set_array([])
    # cbar = plt.colorbar(sm, ax=ax1, label='Height Z (m)', shrink=0.8)

    # Draw approximate workspace boundary (circular arc for base rotation)
    theta_range = np.linspace(JOINT_LIMITS['base'][0], JOINT_LIMITS['base'][1], 100)

    # Max reach arc (approximate)
    r_max = EFFECTIVE_REACH * 0.95  # 95% of theoretical max
    r_min = 0.08  # Minimum reach (close to base)

    x_outer = r_max * np.cos(theta_range)
    y_outer = r_max * np.sin(theta_range)
    ax1.plot(x_outer, y_outer, 'b-', linewidth=2, label=f'Max reach ({r_max:.2f}m)')

    x_inner = r_min * np.cos(theta_range)
    y_inner = r_min * np.sin(theta_range)
    ax1.plot(x_inner, y_inner, 'b--', linewidth=1, alpha=0.5, label=f'Min reach ({r_min:.2f}m)')

    # Connect the arcs at the ends
    ax1.plot([x_outer[0], x_inner[0]], [y_outer[0], y_inner[0]], 'b-', linewidth=1, alpha=0.5)
    ax1.plot([x_outer[-1], x_inner[-1]], [y_outer[-1], y_inner[-1]], 'b-', linewidth=1, alpha=0.5)

    # Fill workspace region
    from matplotlib.patches import Wedge
    workspace_wedge = Wedge(
        center=(0, 0),
        r=r_max,
        theta1=np.degrees(JOINT_LIMITS['base'][0]),
        theta2=np.degrees(JOINT_LIMITS['base'][1]),
        width=r_max - r_min,
        alpha=0.15,
        facecolor='blue',
        edgecolor='none'
    )
    ax1.add_patch(workspace_wedge)

    # Draw robot base
    ax1.plot(0, 0, 's', color='black', markersize=12, zorder=20, label='Robot base')

    # Draw clusters
    for name, data in CLUSTERS.items():
        x, y, z = data['position']

        # Check if in workspace
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        in_reach = r_min < r < r_max
        in_angle = JOINT_LIMITS['base'][0] < theta < JOINT_LIMITS['base'][1]
        reachable = in_reach and in_angle

        marker = 'o' if reachable else 'x'
        edge_color = 'green' if reachable else 'red'

        ax1.plot(x, y, marker, color=data['color'], markersize=15,
                markeredgecolor=edge_color, markeredgewidth=3, zorder=15)
        ax1.annotate(f"{data['label']}\n({x:.2f}, {y:.2f})",
                    (x + 0.05, y + 0.05), fontsize=8, va='bottom')

        # Distance annotation
        ax1.annotate(f'r={r:.2f}m', (x, y - 0.08), fontsize=7, ha='center', color='gray')

    # Draw reservoir
    res_x, res_y, res_z = RESERVOIR['position']
    res_w, res_h, _ = RESERVOIR['dimensions']
    reservoir_rect = Rectangle(
        (res_x - res_w/2, res_y - res_h/2), res_w, res_h,
        facecolor='lightblue', edgecolor='blue', linewidth=2, alpha=0.5
    )
    ax1.add_patch(reservoir_rect)
    ax1.annotate('Reservoir', (res_x, res_y), ha='center', va='center', fontsize=8)

    ax1.set_xlabel('X (meters)', fontsize=10)
    ax1.set_ylabel('Y (meters)', fontsize=10)
    ax1.set_xlim(-0.3, 0.6)
    ax1.set_ylim(-0.5, 0.5)
    ax1.set_aspect('equal')
    ax1.grid(True, linestyle='--', alpha=0.3)
    ax1.legend(loc='upper left', fontsize=8)

    # ==========================================================================
    # (b) Side View (XZ Plane, Y=0 slice)
    # ==========================================================================

    ax2.set_title('(b) Side View - Height Range', fontsize=12, fontweight='bold')

    # Filter points near Y=0 for side view
    y_tolerance = 0.1
    side_mask = np.abs(points[:, 1]) < y_tolerance
    side_points = points[side_mask]

    # Also compute workspace boundary for Y=0 plane
    # Sample with base_angle = 0 (looking along +X)
    side_boundary = []
    for theta_s in np.linspace(*JOINT_LIMITS['shoulder'], 50):
        for theta_e in np.linspace(*JOINT_LIMITS['elbow'], 50):
            for theta_w in np.linspace(*JOINT_LIMITS['wrist_pitch'], 10):
                x, y, z = forward_kinematics_simplified(0, theta_s, theta_e, theta_w)
                if abs(y) < 0.05:  # Near Y=0
                    side_boundary.append([x, z])

    side_boundary = np.array(side_boundary) if side_boundary else np.array([[0, 0]])

    # Plot workspace region
    if len(side_boundary) > 0:
        ax2.scatter(side_boundary[:, 0], side_boundary[:, 1],
                   c='steelblue', alpha=0.3, s=2, rasterized=True, label='Reachable')

    # Draw ground line
    ax2.axhline(y=0, color='brown', linewidth=2, linestyle='-', label='Ground')
    ax2.fill_between([-0.5, 1.5], -0.1, 0, color='brown', alpha=0.2)

    # Draw robot base
    base_height = ROBOT_SPAWN_Z + L_BASE
    ax2.add_patch(Rectangle((-0.05, 0), 0.1, base_height,
                            facecolor='gray', edgecolor='black', linewidth=2))
    ax2.annotate('Base', (0, base_height/2), ha='center', va='center', fontsize=8, color='white')

    # Draw clusters (projected to Y=0)
    for name, data in CLUSTERS.items():
        x, y, z = data['position']

        # Check height reachability (simplified)
        reachable = 0.1 < z < 0.7  # Approximate height range

        marker = 'o' if reachable else 'x'
        edge_color = 'green' if reachable else 'red'

        ax2.plot(x, z, marker, color=data['color'], markersize=12,
                markeredgecolor=edge_color, markeredgewidth=2, zorder=15)
        ax2.annotate(f"{data['label']}\nz={z:.2f}m",
                    (x + 0.03, z), fontsize=8, va='center')

    # Height range indicators
    z_min = 0.1
    z_max = 0.65
    ax2.axhline(y=z_min, color='blue', linewidth=1, linestyle=':', alpha=0.7)
    ax2.axhline(y=z_max, color='blue', linewidth=1, linestyle=':', alpha=0.7)
    ax2.annotate(f'Z min ≈ {z_min:.2f}m', (0.48, z_min), fontsize=8, va='center')
    ax2.annotate(f'Z max ≈ {z_max:.2f}m', (0.48, z_max), fontsize=8, va='center')

    ax2.set_xlabel('X (meters)', fontsize=10)
    ax2.set_ylabel('Z (meters)', fontsize=10)
    ax2.set_xlim(-0.25, 0.55)
    ax2.set_ylim(-0.05, 0.7)
    ax2.set_aspect('equal')
    ax2.grid(True, linestyle='--', alpha=0.3)

    # ==========================================================================
    # Add info box
    # ==========================================================================

    info_text = (
        f'Braccio Arm Specifications\n'
        f'────────────────────────\n'
        f'Total length: {TOTAL_ARM_LENGTH*100:.0f} cm\n'
        f'Effective reach: {EFFECTIVE_REACH*100:.0f} cm\n'
        f'Base rotation: {np.degrees(JOINT_LIMITS["base"][0]):.0f}° - {np.degrees(JOINT_LIMITS["base"][1]):.0f}°\n'
        f'Height range: ~{z_min*100:.0f} - {z_max*100:.0f} cm'
    )

    props = dict(boxstyle='round,pad=0.5', facecolor='lightyellow', edgecolor='gray', alpha=0.9)
    fig.text(0.02, 0.02, info_text, fontsize=8, fontfamily='monospace',
             verticalalignment='bottom', bbox=props)

    # ==========================================================================
    # Save
    # ==========================================================================

    plt.suptitle('Figure 5: Workspace Reachability Analysis - Braccio 6-DOF Arm',
                 fontsize=13, fontweight='bold', y=1.02)

    plt.tight_layout()

    svg_path = save_path
    png_path = save_path.replace('.svg', '.png')

    plt.savefig(svg_path, format='svg', dpi=150, bbox_inches='tight')
    plt.savefig(png_path, format='png', dpi=300, bbox_inches='tight')

    print(f"Saved: {svg_path}")
    print(f"Saved: {png_path}")

    plt.show()

    return fig


def create_3d_workspace_figure(save_path='figure_05_workspace_3d.png'):
    """Generate a 3D visualization of the workspace."""

    print("Computing workspace points for 3D view...")
    points = compute_workspace_points(n_samples=50000)

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot workspace points with color based on height
    scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                        c=points[:, 2], cmap='viridis', alpha=0.1, s=1, rasterized=True)

    # Plot clusters
    for name, data in CLUSTERS.items():
        x, y, z = data['position']
        ax.scatter([x], [y], [z], c=data['color'], s=200, marker='o',
                  edgecolors='black', linewidths=2, label=data['label'], zorder=10)
        ax.text(x, y, z + 0.05, f"{data['label']}", fontsize=9, ha='center')

    # Plot robot base
    ax.scatter([0], [0], [ROBOT_SPAWN_Z], c='black', s=300, marker='s', label='Robot base')

    # Draw reservoir
    res_x, res_y, res_z = RESERVOIR['position']
    ax.scatter([res_x], [res_y], [res_z], c='blue', s=150, marker='^', label='Reservoir')

    # Ground plane
    xx, yy = np.meshgrid(np.linspace(-0.3, 0.5, 10), np.linspace(-0.4, 0.4, 10))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.2, color='brown')

    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('Braccio Arm Workspace - 3D View', fontsize=12, fontweight='bold')

    # Set viewing angle
    ax.view_init(elev=25, azim=45)

    # Colorbar
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.5, label='Height Z (m)')

    ax.legend(loc='upper left')

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {save_path}")

    plt.show()

    return fig


def analyze_cluster_reachability():
    """Analyze whether each cluster is reachable."""

    print("\n" + "=" * 60)
    print("CLUSTER REACHABILITY ANALYSIS")
    print("=" * 60)

    r_max = EFFECTIVE_REACH * 0.95
    r_min = 0.08
    z_min, z_max = 0.1, 0.65

    print(f"\nWorkspace bounds:")
    print(f"  Horizontal reach: {r_min:.2f}m - {r_max:.2f}m")
    print(f"  Base rotation: {np.degrees(JOINT_LIMITS['base'][0]):.0f}° - {np.degrees(JOINT_LIMITS['base'][1]):.0f}°")
    print(f"  Height range: {z_min:.2f}m - {z_max:.2f}m")

    print(f"\nCluster analysis:")
    print("-" * 60)

    for name, data in CLUSTERS.items():
        x, y, z = data['position']
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)
        theta_deg = np.degrees(theta)

        # Check reachability
        in_radial = r_min < r < r_max
        in_angle = JOINT_LIMITS['base'][0] < theta < JOINT_LIMITS['base'][1]
        in_height = z_min < z < z_max

        reachable = in_radial and in_angle and in_height

        print(f"\n{name} ({data['label']}):")
        print(f"  Position: ({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"  Distance from base: {r:.3f}m (limit: {r_min:.2f}-{r_max:.2f}m) {'✓' if in_radial else '✗'}")
        print(f"  Angle: {theta_deg:.1f}° (limit: {np.degrees(JOINT_LIMITS['base'][0]):.0f}°-{np.degrees(JOINT_LIMITS['base'][1]):.0f}°) {'✓' if in_angle else '✗'}")
        print(f"  Height: {z:.3f}m (limit: {z_min:.2f}-{z_max:.2f}m) {'✓' if in_height else '✗'}")
        print(f"  REACHABLE: {'YES ✓' if reachable else 'NO ✗ (may need base repositioning)'}")

    print("\n" + "=" * 60)


if __name__ == '__main__':
    print("=" * 60)
    print("Generating Workspace Reachability Analysis")
    print("=" * 60)
    print(f"\nBraccio Arm Parameters:")
    print(f"  Link lengths: base={L_BASE}m, shoulder={L_SHOULDER}m, elbow={L_ELBOW}m")
    print(f"  Wrist: pitch={L_WRIST_PITCH}m, roll={L_WRIST_ROLL}m, gripper={L_GRIPPER}m")
    print(f"  Total length: {TOTAL_ARM_LENGTH:.3f}m")
    print(f"  Effective reach: {EFFECTIVE_REACH:.3f}m")

    # Analyze reachability first
    analyze_cluster_reachability()

    # Generate 2D figure (main figure for report)
    print("\n[1/2] Generating 2D workspace analysis...")
    create_workspace_figure('figure_05_workspace_analysis.svg')

    # Generate 3D figure (supplementary)
    print("\n[2/2] Generating 3D workspace visualization...")
    create_3d_workspace_figure('figure_05_workspace_3d.png')

    print("\nDone!")
