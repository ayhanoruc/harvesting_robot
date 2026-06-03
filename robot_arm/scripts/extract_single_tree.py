#!/usr/bin/env python3
"""
Extract ONE tree (trunk + leaves) from the monolithic cpr_orchard_gazebo
meshes (orchard_trunks.dae, orchard_leaves.dae), centered at origin.

Why: the orchard meshes bake 236 trees into a single geometry each. To
place "one tree per cluster" in cotton_demo.world, we need a single-tree
asset. This script crops the monolithic mesh to a square XY region
around a chosen tree (tree_005 by default, ~middle of the orchard) and
translates the kept geometry so the tree base sits at world origin.

Output:
  robot_arm/models/single_tree/meshes/single_tree_trunk.dae
  robot_arm/models/single_tree/meshes/single_tree_leaves.dae
  robot_arm/models/single_tree/model.sdf
  robot_arm/models/single_tree/model.config

Crop strategy: keep every triangle whose 3 vertices ALL lie inside
[tree_cx ± half, tree_cy ± half] after applying the source DAE's scene-
node <matrix> transform. We then re-index vertices to compact storage
and rewrite the float arrays + index <p> lists in place.

Usage:
  python extract_single_tree.py [--tree-id tree_005] [--half 0.6]
"""

from __future__ import annotations

import argparse
import os
import re
import xml.etree.ElementTree as ET
import yaml

NS_COLLADA = 'http://www.collada.org/2005/11/COLLADASchema'
NS = {'c': NS_COLLADA}
ET.register_namespace('', NS_COLLADA)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT   = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
TRUNKS_IN  = os.path.join(PKG_ROOT, 'meshes', 'orchard', 'orchard_trunks.dae')
LEAVES_IN  = os.path.join(PKG_ROOT, 'meshes', 'orchard', 'orchard_leaves.dae')
TREES_YML  = os.path.join(PKG_ROOT, 'config', 'orchard_tree_positions.yaml')
OUT_DIR    = os.path.join(PKG_ROOT, 'models', 'single_tree')


def parse_matrix(text: str):
    """Parse a 16-float row-major COLLADA <matrix> into a 4x4 list."""
    vals = [float(x) for x in text.split()]
    return [vals[0:4], vals[4:8], vals[8:12], vals[12:16]]


def apply_matrix(m, x, y, z):
    """Apply 4x4 row-major matrix to a column vector (x,y,z,1)."""
    nx = m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3]
    ny = m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3]
    nz = m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3]
    return nx, ny, nz


def floats_from_array(elem) -> list[float]:
    return [float(x) for x in (elem.text or '').split()]


def ints_from_p(elem) -> list[int]:
    return [int(x) for x in (elem.text or '').split()]


def crop_one_dae(in_path: str, out_path: str,
                 cx: float, cy: float, half: float,
                 tree_z_offset: float = 0.0):
    """Crop the monolithic DAE to a single tree, write to out_path.

    cx, cy : tree center in WORLD coordinates (post-scene-matrix).
    half   : crop half-width (square XY filter).
    tree_z_offset : after cropping, translate Z so the tree base sits at
                    world Z=0 (subtract this from each kept vertex's z).
    """
    tree = ET.parse(in_path)
    root = tree.getroot()

    # Find the scene-node matrix that transforms mesh-local → world.
    node_matrix = None
    for node in root.iter(f'{{{NS_COLLADA}}}node'):
        mat = node.find('c:matrix', NS)
        if mat is not None and mat.get('sid') == 'transform':
            node_matrix = parse_matrix(mat.text)
            break
    if node_matrix is None:
        raise RuntimeError(f'No <node><matrix> in {in_path}')

    # COLLADA layout: <library_geometries>/<geometry>/<mesh>:
    #   <source id="..."><float_array .../> ... </source>   (positions, normals, uvs)
    #   <vertices id="..."><input semantic="POSITION" source="#..."/></vertices>
    #   <triangles count="N" material="..."><input .../> ... <p>idx idx ...</p></triangles>
    mesh = root.find('.//c:library_geometries/c:geometry/c:mesh', NS)
    if mesh is None:
        raise RuntimeError(f'No <mesh> in {in_path}')

    # Resolve POSITION source.
    verts_elem = mesh.find('c:vertices', NS)
    pos_src_id = None
    for inp in verts_elem.findall('c:input', NS):
        if inp.get('semantic') == 'POSITION':
            pos_src_id = inp.get('source', '').lstrip('#')
            break
    pos_source = next(
        s for s in mesh.findall('c:source', NS) if s.get('id') == pos_src_id)
    pos_array_elem = pos_source.find('c:float_array', NS)
    raw_positions = floats_from_array(pos_array_elem)
    # Positions are flat [x0,y0,z0, x1,y1,z1, ...].
    n_verts = len(raw_positions) // 3

    # Apply scene transform once to all positions; this puts each vertex
    # into world coordinates we can crop against (cx,cy).
    world_pos = []
    for i in range(n_verts):
        x, y, z = raw_positions[3*i:3*i+3]
        wx, wy, wz = apply_matrix(node_matrix, x, y, z)
        world_pos.append((wx, wy, wz))

    def inside(idx: int) -> bool:
        wx, wy, _ = world_pos[idx]
        return (cx - half <= wx <= cx + half) and (cy - half <= wy <= cy + half)

    # Find ALL <triangles>/<polylist>/<polygons> primitives and filter
    # their <p> index lists. Each primitive has inputs with offsets, so
    # one "vertex" in <p> takes stride = max(offset)+1 ints.
    kept_old_verts: set[int] = set()
    kept_prim_indices: list[tuple[ET.Element, list[int]]] = []

    for tag in ('triangles', 'polylist', 'polygons'):
        for prim in mesh.findall(f'c:{tag}', NS):
            inputs = prim.findall('c:input', NS)
            if not inputs:
                continue
            max_off = max(int(i.get('offset', '0')) for i in inputs)
            stride  = max_off + 1
            vert_off = None
            for i in inputs:
                if i.get('semantic') == 'VERTEX':
                    vert_off = int(i.get('offset', '0'))
                    break
            if vert_off is None:
                continue
            for p_elem in prim.findall('c:p', NS):
                idx = ints_from_p(p_elem)
                # triangles: every 3 *vertices* (= 3 * stride ints) is a tri
                n_tri = len(idx) // (3 * stride)
                kept_chunks: list[int] = []
                kept_tris = 0
                for t in range(n_tri):
                    base = t * 3 * stride
                    v0 = idx[base + 0*stride + vert_off]
                    v1 = idx[base + 1*stride + vert_off]
                    v2 = idx[base + 2*stride + vert_off]
                    if inside(v0) and inside(v1) and inside(v2):
                        kept_chunks.extend(idx[base : base + 3*stride])
                        kept_old_verts.update((v0, v1, v2))
                        kept_tris += 1
                kept_prim_indices.append((prim, kept_chunks))
                # Update count on primitive
                if tag == 'triangles':
                    prim.set('count', str(kept_tris))

    if not kept_old_verts:
        raise RuntimeError(
            f'Crop kept ZERO vertices for ({cx:.2f},{cy:.2f}) ±{half:.2f}m in {in_path}. '
            f'Try larger --half or different tree position.')

    # Re-index: old_idx → new_idx. Sort for stable layout.
    old_sorted = sorted(kept_old_verts)
    remap_v = {old: new for new, old in enumerate(old_sorted)}

    # Rebuild the POSITION float_array with kept verts only, translated
    # so the tree base sits at world origin. We also drop the scene-node
    # matrix (replace with identity) so the cropped DAE is "world clean".
    new_positions: list[str] = []
    for old in old_sorted:
        wx, wy, wz = world_pos[old]
        new_positions.append(f'{wx - cx:.6f}')
        new_positions.append(f'{wy - cy:.6f}')
        new_positions.append(f'{wz - tree_z_offset:.6f}')
    pos_array_elem.text = ' '.join(new_positions)
    pos_array_elem.set('count', str(len(new_positions)))
    accessor = pos_source.find('c:technique_common/c:accessor', NS)
    if accessor is not None:
        accessor.set('count', str(len(new_positions) // 3))

    # NORMAL and TEXCOORD sources are indexed independently via per-input
    # offsets in <p>. We keep them intact (they still match the original
    # index space, since we kept the original indices for non-VERTEX inputs).
    # That keeps the DAE small and correct as long as the original indices
    # remain valid — which they do, because we never drop normal/texcoord
    # entries, only filter which (vertex, normal, texcoord) triples are
    # emitted. Re-indexing only the VERTEX semantic suffices.

    # Replace <p> contents with the remapped (only VERTEX changes) sequences.
    for prim, kept_chunks in kept_prim_indices:
        inputs = prim.findall('c:input', NS)
        max_off = max(int(i.get('offset', '0')) for i in inputs)
        stride  = max_off + 1
        vert_off = next(int(i.get('offset', '0'))
                        for i in inputs if i.get('semantic') == 'VERTEX')
        # Re-index VERTEX entries in kept_chunks.
        n_groups = len(kept_chunks) // stride
        for g in range(n_groups):
            base = g * stride
            kept_chunks[base + vert_off] = remap_v[kept_chunks[base + vert_off]]
        # Some primitives have multiple <p> blocks; we shoved everything
        # into one. Drop all <p> children, write one new <p>.
        for p in list(prim.findall('c:p', NS)):
            prim.remove(p)
        new_p = ET.SubElement(prim, f'{{{NS_COLLADA}}}p')
        new_p.text = ' '.join(str(i) for i in kept_chunks)

    # Replace the scene-node <matrix> with identity so we can spawn the
    # cropped mesh directly without re-applying the world transform.
    for node in root.iter(f'{{{NS_COLLADA}}}node'):
        mat = node.find('c:matrix', NS)
        if mat is not None and mat.get('sid') == 'transform':
            mat.text = '1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1'

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    tree.write(out_path, xml_declaration=True, encoding='utf-8')
    print(f'  Wrote {out_path}  (verts: {len(old_sorted)}, '
          f'tris: {sum(c.get("count") for p, c in [] if False)})')


def write_model_sdf(out_dir: str, model_name: str = 'single_tree'):
    sdf = (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.7">\n'
        f'  <model name="{model_name}">\n'
        '    <static>true</static>\n'
        '    <link name="link">\n'
        '      <visual name="trunk">\n'
        '        <geometry><mesh>\n'
        '          <uri>model://single_tree/meshes/single_tree_trunk.dae</uri>\n'
        '        </mesh></geometry>\n'
        '      </visual>\n'
        '      <visual name="leaves">\n'
        '        <geometry><mesh>\n'
        '          <uri>model://single_tree/meshes/single_tree_leaves.dae</uri>\n'
        '        </mesh></geometry>\n'
        '      </visual>\n'
        '    </link>\n'
        '  </model>\n'
        '</sdf>\n'
    )
    p = os.path.join(out_dir, 'model.sdf')
    with open(p, 'w', encoding='utf-8') as f:
        f.write(sdf)
    print(f'  Wrote {p}')


def write_model_config(out_dir: str, model_name: str = 'single_tree'):
    cfg = (
        '<?xml version="1.0"?>\n'
        '<model>\n'
        f'  <name>{model_name}</name>\n'
        '  <version>1.0</version>\n'
        '  <sdf version="1.7">model.sdf</sdf>\n'
        '  <description>One tree extracted from cpr_orchard_gazebo.</description>\n'
        '</model>\n'
    )
    p = os.path.join(out_dir, 'model.config')
    with open(p, 'w', encoding='utf-8') as f:
        f.write(cfg)
    print(f'  Wrote {p}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tree-id', default='tree_005',
                    help='Which tree from orchard_tree_positions.yaml to crop '
                         '(default tree_005 — mid-orchard row 0).')
    ap.add_argument('--half', type=float, default=0.7,
                    help='Crop half-width in meters (square XY filter). '
                         'Default 0.7 → 1.4m on each side. Trees are ~0.6m '
                         'radius so this is generous.')
    args = ap.parse_args()

    with open(TREES_YML, 'r', encoding='utf-8') as f:
        trees_doc = yaml.safe_load(f)
    # YAML schema: `trees:` is top-level sibling of `orchard:` metadata.
    target = next(t for t in trees_doc['trees'] if t['id'] == args.tree_id)
    cx, cy = float(target['x']), float(target['y'])
    print(f'Extracting {args.tree_id} at world ({cx:.2f}, {cy:.2f}), half={args.half}m')

    # Z offset: trunk base sits at the orchard ground plane. The scene
    # matrix translation z is ~-0.087 — close to zero — and trunk base
    # vertices sit at Z≈0 after transform. We pass 0 so the cropped tree
    # sits with its base at Z=0 in the output mesh.
    crop_one_dae(TRUNKS_IN,
                 os.path.join(OUT_DIR, 'meshes', 'single_tree_trunk.dae'),
                 cx, cy, args.half, tree_z_offset=0.0)
    crop_one_dae(LEAVES_IN,
                 os.path.join(OUT_DIR, 'meshes', 'single_tree_leaves.dae'),
                 cx, cy, args.half, tree_z_offset=0.0)

    write_model_sdf(OUT_DIR)
    write_model_config(OUT_DIR)
    print('Done.')


if __name__ == '__main__':
    main()
