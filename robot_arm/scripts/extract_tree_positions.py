"""
Extract tree positions from cpr_orchard_gazebo's orchard_trunks.dae
and write them to robot_arm/config/orchard_tree_positions.yaml.

Strategy:
  1. Parse the COLLADA mesh and extract all vertex positions.
  2. Apply the scene-graph transform (rotation+scale+translation from <node><matrix>).
  3. Filter low-Z vertices (Z < z_max_trunk) -> trunk-base points.
  4. Voxelize XY plane, find busy voxels (high vertex density).
  5. 8-neighbor connected-component label busy voxels -> each component = one tree.
  6. Take centroid of each component -> tree XY position.
  7. Cross-reference with leaves.dae Z range -> canopy_z_min/max per tree.

Output: robot_arm/config/orchard_tree_positions.yaml

Usage:
  python extract_tree_positions.py
"""

import os
import xml.etree.ElementTree as ET
import numpy as np
import yaml

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}

REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', '..'))
TRUNKS_DAE = os.path.join(REPO_ROOT, 'docs', 'RESEARCH', 'cpr_gazebo',
                          'cpr_orchard_gazebo', 'meshes', 'orchard_trunks.dae')
LEAVES_DAE = os.path.join(REPO_ROOT, 'docs', 'RESEARCH', 'cpr_gazebo',
                          'cpr_orchard_gazebo', 'meshes', 'orchard_leaves.dae')
OUT_YAML = os.path.join(REPO_ROOT, 'robot_arm', 'config',
                        'orchard_tree_positions.yaml')

VOXEL_SIZE = 0.4   # meters
Z_MAX_TRUNK = 1.0  # consider Z < this as trunk-base for clustering


def parse_position_source(mesh_elem):
    """For a single <mesh>, return Nx3 array of vertex positions."""
    verts = mesh_elem.find('c:vertices', NS)
    if verts is None:
        return None
    pos_id = None
    for inp in verts.findall('c:input', NS):
        if inp.get('semantic') == 'POSITION':
            pos_id = inp.get('source', '').lstrip('#')
            break
    if pos_id is None:
        return None
    src = None
    for s in mesh_elem.findall('c:source', NS):
        if s.get('id') == pos_id:
            src = s
            break
    if src is None:
        return None
    accessor = src.find('.//c:accessor', NS)
    farr = src.find('c:float_array', NS)
    if farr is None:
        return None
    stride = int(accessor.get('stride', '3')) if accessor is not None else 3
    if stride != 3:
        return None
    vals = np.fromstring(farr.text, sep=' ', dtype=np.float32)
    if vals.size % 3 != 0:
        return None
    return vals.reshape(-1, 3)


def get_node_matrix(dae_root):
    """Return the 4x4 scene-graph matrix for the (single) node referencing geometry."""
    nodes = dae_root.findall('.//c:visual_scene//c:node', NS)
    for node in nodes:
        if node.find('c:instance_geometry', NS) is None:
            continue
        m = node.find('c:matrix', NS)
        if m is not None:
            return np.array([float(v) for v in m.text.split()],
                            dtype=np.float32).reshape(4, 4)
        t = node.find('c:translate', NS)
        if t is not None:
            mat = np.eye(4, dtype=np.float32)
            mat[:3, 3] = [float(v) for v in t.text.split()]
            return mat
    return np.eye(4, dtype=np.float32)


def load_world_pts(dae_path):
    """Return Nx3 vertex positions in world frame for the given .dae."""
    tree = ET.parse(dae_path)
    root = tree.getroot()
    meshes = root.findall('.//c:geometry/c:mesh', NS)
    matrix = get_node_matrix(root)
    all_local = []
    for mesh in meshes:
        pts = parse_position_source(mesh)
        if pts is not None and pts.size:
            all_local.append(pts)
    if not all_local:
        return np.zeros((0, 3))
    local = np.vstack(all_local)
    homo = np.hstack([local, np.ones((local.shape[0], 1), dtype=np.float32)])
    world = (matrix @ homo.T).T[:, :3]
    return world


def cluster_trees(pts, voxel=VOXEL_SIZE, z_max=Z_MAX_TRUNK):
    """Connected-component cluster on busy voxels of low-Z subset."""
    low = pts[pts[:, 2] < z_max]
    if low.shape[0] == 0:
        return np.zeros((0, 2))

    xy = low[:, :2]
    ix = np.floor(xy[:, 0] / voxel).astype(int)
    iy = np.floor(xy[:, 1] / voxel).astype(int)
    keys = ix * 100000 + iy  # encode (ix, iy) as single int
    unique, counts = np.unique(keys, return_counts=True)

    threshold = max(30, int(np.percentile(counts, 70)))
    busy = set(unique[counts >= threshold].tolist())

    visited = set()
    components = []
    for seed in busy:
        if seed in visited:
            continue
        stack = [seed]
        comp = []
        while stack:
            cur = stack.pop()
            if cur in visited or cur not in busy:
                continue
            visited.add(cur)
            comp.append(cur)
            ax, ay = cur // 100000, cur % 100000
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = (ax + dx) * 100000 + (ay + dy)
                    if nb in busy and nb not in visited:
                        stack.append(nb)
        if comp:
            components.append(comp)

    centers = []
    for comp in components:
        xs = [(k // 100000) * voxel + voxel / 2 for k in comp]
        ys = [(k % 100000) * voxel + voxel / 2 for k in comp]
        centers.append((float(np.mean(xs)), float(np.mean(ys))))
    return np.array(centers)


def estimate_canopy_z_per_tree(tree_xys, leaf_pts, search_radius=0.8):
    """For each tree XY, find min/max Z of leaf vertices within search_radius."""
    canopy = []
    leaf_xy = leaf_pts[:, :2]
    leaf_z = leaf_pts[:, 2]
    # Use simple euclidean per-tree (219 trees × ~64k vertices = manageable)
    for tx, ty in tree_xys:
        d2 = (leaf_xy[:, 0] - tx) ** 2 + (leaf_xy[:, 1] - ty) ** 2
        nearby = leaf_z[d2 < search_radius ** 2]
        if nearby.size:
            canopy.append((float(nearby.min()), float(nearby.max())))
        else:
            canopy.append((1.5, 3.0))  # fallback default
    return canopy


def main():
    print(f"Loading trunks: {TRUNKS_DAE}")
    trunk_pts = load_world_pts(TRUNKS_DAE)
    print(f"  Trunk vertices: {len(trunk_pts):,}")
    print(f"  Trunk bbox X[{trunk_pts[:,0].min():.1f},{trunk_pts[:,0].max():.1f}] "
          f"Y[{trunk_pts[:,1].min():.1f},{trunk_pts[:,1].max():.1f}] "
          f"Z[{trunk_pts[:,2].min():.1f},{trunk_pts[:,2].max():.1f}]")

    print(f"Loading leaves: {LEAVES_DAE}")
    leaf_pts = load_world_pts(LEAVES_DAE)
    print(f"  Leaf vertices: {len(leaf_pts):,}")
    print(f"  Leaf bbox X[{leaf_pts[:,0].min():.1f},{leaf_pts[:,0].max():.1f}] "
          f"Y[{leaf_pts[:,1].min():.1f},{leaf_pts[:,1].max():.1f}] "
          f"Z[{leaf_pts[:,2].min():.1f},{leaf_pts[:,2].max():.1f}]")

    print()
    print("Clustering trunk points to estimate tree positions...")
    tree_xys = cluster_trees(trunk_pts)
    print(f"  Estimated trees: {len(tree_xys)}")

    print()
    print("Estimating canopy Z range per tree from nearby leaf vertices...")
    canopy = estimate_canopy_z_per_tree(tree_xys, leaf_pts)

    # Sort: by row (Y bin) first, then X within row, for readable YAML
    rows = []
    y_sorted_indices = np.argsort(tree_xys[:, 1])
    sorted_tree_xys = tree_xys[y_sorted_indices]
    sorted_canopy = [canopy[i] for i in y_sorted_indices]

    # Group into rows by Y gaps > 1.5m
    row_groups = []
    current_row = [(sorted_tree_xys[0], sorted_canopy[0])]
    for i in range(1, len(sorted_tree_xys)):
        prev_y = sorted_tree_xys[i - 1][1]
        cur_y = sorted_tree_xys[i][1]
        if cur_y - prev_y > 1.5:
            row_groups.append(current_row)
            current_row = []
        current_row.append((sorted_tree_xys[i], sorted_canopy[i]))
    if current_row:
        row_groups.append(current_row)

    print(f"  Detected rows: {len(row_groups)}")

    # Build YAML structure
    trees_yaml = []
    tree_id = 0
    for row_idx, row in enumerate(row_groups):
        # Sort within row by X
        row_sorted = sorted(row, key=lambda r: r[0][0])
        for col_idx, ((x, y), (cz_min, cz_max)) in enumerate(row_sorted):
            trees_yaml.append({
                'id': f'tree_{tree_id:03d}',
                'row': row_idx,
                'col': col_idx,
                'x': round(float(x), 3),
                'y': round(float(y), 3),
                'canopy_z_min': round(cz_min, 2),
                'canopy_z_max': round(cz_max, 2),
            })
            tree_id += 1

    out = {
        'orchard': {
            'description': 'Tree positions extracted from cpr_orchard_gazebo orchard_trunks.dae',
            'source_mesh': 'orchard_trunks.dae',
            'extraction_method': 'voxel-cluster on low-Z trunk vertices, 8-neighbor connected components',
            'voxel_size_m': VOXEL_SIZE,
            'tree_count': len(trees_yaml),
            'row_count': len(row_groups),
            'bbox': {
                'x_min': round(float(tree_xys[:, 0].min()), 2),
                'x_max': round(float(tree_xys[:, 0].max()), 2),
                'y_min': round(float(tree_xys[:, 1].min()), 2),
                'y_max': round(float(tree_xys[:, 1].max()), 2),
            },
        },
        'trees': trees_yaml,
    }

    os.makedirs(os.path.dirname(OUT_YAML), exist_ok=True)
    with open(OUT_YAML, 'w') as f:
        yaml.dump(out, f, sort_keys=False, default_flow_style=False)
    print(f"\nWrote {OUT_YAML}")
    print(f"  Trees: {len(trees_yaml)}, Rows: {len(row_groups)}")
    print(f"  Bbox X: [{out['orchard']['bbox']['x_min']}, {out['orchard']['bbox']['x_max']}]m")
    print(f"  Bbox Y: [{out['orchard']['bbox']['y_min']}, {out['orchard']['bbox']['y_max']}]m")

    # Also print first few entries as preview
    print("\nFirst 5 trees:")
    for t in trees_yaml[:5]:
        print(f"  {t}")


if __name__ == '__main__':
    main()
