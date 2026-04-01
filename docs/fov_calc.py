#!/usr/bin/env python3
"""
Simple 2D FOV coverage: camera at home, clusters in front.
Top-down view (X-Y plane). How many j1 pan steps to cover all clusters?
"""
import numpy as np

# Camera at home — roughly at base, looking forward (+X)
cam_x, cam_y = 0.0, 0.0
HFOV = 90  # degrees

# Clusters (X, Y in world)
clusters = {
    'cluster_1': (0.875, 0.475),
    'cluster_2': (0.975, 0.0),
    'cluster_3': (0.875, -0.475),
}

print("="*60)
print("2D FOV COVERAGE (top-down view)")
print(f"Camera at ({cam_x}, {cam_y}), HFOV={HFOV}°")
print("="*60)

# Angular position of each cluster from camera
print("\nCluster angles from camera (0° = forward/+X):")
for name, (cx, cy) in clusters.items():
    angle = np.degrees(np.arctan2(cy - cam_y, cx - cam_x))
    dist = np.sqrt((cx - cam_x)**2 + (cy - cam_y)**2)
    print(f"  {name}: angle={angle:+.1f}°, dist={dist:.2f}m")

angles = [np.degrees(np.arctan2(cy, cx)) for cx, cy in clusters.values()]
min_a, max_a = min(angles), max(angles)
span = max_a - min_a
print(f"\nTotal angular span: {min_a:.1f}° to {max_a:.1f}° = {span:.1f}°")
print(f"Camera HFOV: {HFOV}°")

# Check: does one frame cover all?
half_fov = HFOV / 2
center = (min_a + max_a) / 2
print(f"\nOptimal single-frame center: {center:.1f}° (j1 ~ {np.radians(center):.3f} rad)")
if span <= HFOV:
    print(f"  -> {span:.1f}° span < {HFOV}° FOV — ALL CLUSTERS IN ONE FRAME!")
    margin = (HFOV - span) / 2
    print(f"  -> Margin: {margin:.1f}° on each side")
else:
    print(f"  -> {span:.1f}° span > {HFOV}° FOV — need multiple frames")

# Simulate j1 sweep — which clusters visible at each angle?
print("\n" + "-"*60)
print("j1 sweep simulation:")
print("-"*60)
for j1_deg in np.arange(-60, 61, 10):
    cam_dir = j1_deg  # camera points at this angle
    fov_min = cam_dir - half_fov
    fov_max = cam_dir + half_fov
    visible = []
    for name, (cx, cy) in clusters.items():
        a = np.degrees(np.arctan2(cy, cx))
        if fov_min <= a <= fov_max:
            visible.append(name)
    marker = " <<<" if len(visible) == 3 else ""
    print(f"  j1={j1_deg:+4.0f}° (FOV: {fov_min:+.0f}° to {fov_max:+.0f}°): "
          f"{visible}{marker}")

# Now vertical (side view, X-Z plane)
print("\n" + "="*60)
print("2D FOV COVERAGE (side view, vertical)")
print("="*60)
VFOV = 74  # degrees

cam_z = 0.75  # approximate camera height at HOME

print(f"Camera height: ~{cam_z}m")
print(f"VFOV: {VFOV}°")
print("\nCluster vertical angles:")
for name, (cx, cy) in clusters.items():
    cz = {'cluster_1': 0.50, 'cluster_2': 0.56, 'cluster_3': 0.46}[name]
    horiz_dist = cx  # roughly
    vert_angle = np.degrees(np.arctan2(cz - cam_z, horiz_dist))
    print(f"  {name}: z={cz}m, vert_angle={vert_angle:+.1f}° from horizontal")

# Summary
print("\n" + "="*60)
print("SUMMARY")
print("="*60)
if span <= HFOV:
    print(f"Horizontal: 1 position enough (span={span:.0f}° < FOV={HFOV}°)")
    print(f"  Optimal j1 = {center:.1f}° = {np.radians(center):.3f} rad")
    print(f"  But for robustness, use 2-3 positions with overlap:")
    for n, j1 in enumerate(np.linspace(min_a, max_a, 3)):
        print(f"    pos_{n+1}: j1={j1:+.1f}° ({np.radians(j1):.3f} rad)")
    print(f"\nVertical: clusters are ~0.2-0.3m below camera")
    print(f"  With {VFOV}° VFOV, tilt needed = ~15° downward")
    print(f"  1-2 tilt positions should suffice")
    print(f"\nRECOMMENDED GRID: 3 pan × 1 tilt = 3 positions (minimum)")
    print(f"  Or: 3 pan × 2 tilt = 6 positions (robust)")
