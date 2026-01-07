#!/usr/bin/env python3
"""
Generate Figure 12: Visual Servoing Convergence
Combined figure showing error reduction and pixel trajectory
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Data from logs
iterations = [0, 1, 2]
labels = ['Initial', 'Iter 1', 'Iter 2']

# Pixel positions (u, v)
pixels = [
    (64, 351),   # Initial - cluster at edge
    (331, 336),  # After focus iteration 1
    (313, 316),  # After focus iteration 2
]

# Image center
center = (320, 240)

# Calculate errors
errors_u = [p[0] - center[0] for p in pixels]
errors_v = [p[1] - center[1] for p in pixels]
total_errors = [np.sqrt(eu**2 + ev**2) for eu, ev in zip(errors_u, errors_v)]

# Create figure with 2 subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle('Figure 12: Visual Servoing Convergence', fontsize=14, fontweight='bold')

# === Left panel: Error convergence chart ===
ax1.set_title('(a) Pixel Error Convergence')

# Plot total error
ax1.plot(iterations, total_errors, 'b-o', linewidth=2, markersize=10, label='Total Error')
ax1.plot(iterations, [abs(e) for e in errors_u], 'r--s', linewidth=1.5, markersize=8, label='|Error u| (horizontal)')
ax1.plot(iterations, [abs(e) for e in errors_v], 'g--^', linewidth=1.5, markersize=8, label='|Error v| (vertical)')

# Add threshold line
ax1.axhline(y=20, color='gray', linestyle=':', linewidth=1.5, label='Target threshold (20px)')

# Annotate points
for i, (te, eu, ev) in enumerate(zip(total_errors, errors_u, errors_v)):
    ax1.annotate(f'{te:.0f}px', (i, te), textcoords="offset points",
                 xytext=(10, 10), fontsize=9, color='blue')

ax1.set_xlabel('Focus Iteration', fontsize=11)
ax1.set_ylabel('Pixel Error (px)', fontsize=11)
ax1.set_xticks(iterations)
ax1.set_xticklabels(labels)
ax1.set_ylim(0, 320)
ax1.legend(loc='upper right', fontsize=9)
ax1.grid(True, alpha=0.3)

# === Right panel: 2D trajectory on image plane ===
ax2.set_title('(b) Pixel Trajectory in Image Plane')

# Draw image boundary
img_width, img_height = 640, 480
rect = patches.Rectangle((0, 0), img_width, img_height,
                          linewidth=2, edgecolor='black', facecolor='lightgray', alpha=0.3)
ax2.add_patch(rect)

# Draw center crosshair
ax2.axhline(y=center[1], color='gray', linestyle='--', linewidth=1, alpha=0.5)
ax2.axvline(x=center[0], color='gray', linestyle='--', linewidth=1, alpha=0.5)
ax2.plot(center[0], center[1], 'k+', markersize=20, markeredgewidth=2, label='Image center')

# Draw trajectory
colors = ['red', 'orange', 'green']
for i, (px, py) in enumerate(pixels):
    # Draw point
    ax2.plot(px, py, 'o', color=colors[i], markersize=12, markeredgecolor='black', markeredgewidth=1.5)
    # Label
    offset = (15, -20) if i == 0 else (10, 10)
    ax2.annotate(f'{labels[i]}\n({px}, {py})', (px, py),
                 textcoords="offset points", xytext=offset, fontsize=9,
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

# Draw arrows between points (trajectory)
for i in range(len(pixels) - 1):
    dx = pixels[i+1][0] - pixels[i][0]
    dy = pixels[i+1][1] - pixels[i][1]
    ax2.annotate('', xy=pixels[i+1], xytext=pixels[i],
                 arrowprops=dict(arrowstyle='->', color='blue', lw=2))

# Draw error vectors from each point to center
for i, (px, py) in enumerate(pixels):
    if i == len(pixels) - 1:  # Only for final position
        ax2.plot([px, center[0]], [py, center[1]], 'r:', linewidth=1.5, alpha=0.7)
        mid_x, mid_y = (px + center[0]) / 2, (py + center[1]) / 2
        ax2.annotate(f'error={total_errors[i]:.0f}px', (mid_x, mid_y),
                     fontsize=8, color='red',
                     bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7))

# Set axis properties
ax2.set_xlim(-20, img_width + 20)
ax2.set_ylim(img_height + 20, -20)  # Invert y-axis (image coordinates)
ax2.set_xlabel('u (pixels)', fontsize=11)
ax2.set_ylabel('v (pixels)', fontsize=11)
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)

# Add legend
ax2.plot([], [], 'o', color='red', markersize=8, label='Initial position')
ax2.plot([], [], 'o', color='orange', markersize=8, label='After iter 1')
ax2.plot([], [], 'o', color='green', markersize=8, label='After iter 2 (final)')
ax2.plot([], [], '->', color='blue', markersize=8, label='Focus adjustment')
ax2.legend(loc='lower right', fontsize=8)

plt.tight_layout()

# Save figure
output_path = 'figure_12_visual_servoing_convergence.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight', facecolor='white')
print(f'Saved: {output_path}')

# Also save as SVG for report
svg_path = 'figure_12_visual_servoing_convergence.svg'
plt.savefig(svg_path, format='svg', bbox_inches='tight', facecolor='white')
print(f'Saved: {svg_path}')

plt.show()
