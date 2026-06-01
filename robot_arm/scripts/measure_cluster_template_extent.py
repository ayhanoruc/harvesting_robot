#!/usr/bin/env python3
"""Compute the actual XYZ bounding box of cotton_cluster_template.dae as
rendered — walks every <node> kept in <visual_scene>, multiplies its
4x4 matrix into each referenced geometry's vertex positions, and finds
min/max across all of them.

The point: node translations alone (e.g. socket_A_04 at z=0.40) tell us
where a piece STARTS, not how big it is. The geometry vertices may
extend above/below the node origin. Without this, scale estimates are
guesses.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

NS    = 'http://www.collada.org/2005/11/COLLADASchema'
NSPRE = f'{{{NS}}}'

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
DAE = os.path.join(PKG, 'models', 'cotton_orchard_static', 'meshes',
                   'cotton_cluster_template.dae')


def _parse_matrix(node):
    m = node.find(f'{NSPRE}matrix')
    if m is None or not m.text:
        return None
    vals = [float(v) for v in m.text.split()]
    if len(vals) != 16:
        return None
    return vals


def _apply_4x4(m, x, y, z):
    """Transform (x,y,z) by row-major 4x4 matrix m."""
    nx = m[0]*x + m[1]*y + m[2]*z + m[3]
    ny = m[4]*x + m[5]*y + m[6]*z + m[7]
    nz = m[8]*x + m[9]*y + m[10]*z + m[11]
    return nx, ny, nz


def _collect_geom_verts(root):
    """Return {geom_id: [(x,y,z), ...]} for every <geometry> in <library_geometries>."""
    out = {}
    for g in root.iter(f'{NSPRE}geometry'):
        gid = g.get('id', '')
        verts = []
        for src in g.iter(f'{NSPRE}source'):
            fa = src.find(f'{NSPRE}float_array')
            if fa is None or not fa.text:
                continue
            # take only the positions source (heuristic: id contains 'positions')
            if 'position' not in src.get('id', '').lower():
                continue
            nums = [float(v) for v in fa.text.split()]
            for i in range(0, len(nums) - 2, 3):
                verts.append((nums[i], nums[i+1], nums[i+2]))
            break
        out[gid] = verts
    return out


def _node_instance_geom_ids(node):
    out = []
    for ig in node.iter(f'{NSPRE}instance_geometry'):
        url = ig.get('url', '')
        if url.startswith('#'):
            out.append(url[1:])
    return out


def main():
    if not os.path.isfile(DAE):
        raise FileNotFoundError(DAE)
    print(f'Parsing {DAE}')
    tree = ET.parse(DAE)
    root = tree.getroot()
    geom_verts = _collect_geom_verts(root)
    print(f'Geometries with positions: {len(geom_verts)}')

    bbox = None
    for vs in root.iter(f'{NSPRE}visual_scene'):
        for node in vs.iter(f'{NSPRE}node'):
            name = node.get('name', '')
            m = _parse_matrix(node)
            if m is None:
                continue
            gids = _node_instance_geom_ids(node)
            if not gids:
                continue
            for gid in gids:
                vs_list = geom_verts.get(gid, [])
                if not vs_list:
                    continue
                for vx, vy, vz in vs_list:
                    wx, wy, wz = _apply_4x4(m, vx, vy, vz)
                    if bbox is None:
                        bbox = [wx, wx, wy, wy, wz, wz]
                    else:
                        bbox[0] = min(bbox[0], wx); bbox[1] = max(bbox[1], wx)
                        bbox[2] = min(bbox[2], wy); bbox[3] = max(bbox[3], wy)
                        bbox[4] = min(bbox[4], wz); bbox[5] = max(bbox[5], wz)
                print(f'  node {name:25s} geom {gid:35s} verts={len(vs_list)}')

    if bbox is None:
        print('No vertices found — check DAE structure')
        return
    print('\n=== Template bbox at scale 1 (the DAE\'s native coords) ===')
    print(f'  X: [{bbox[0]:+.4f}, {bbox[1]:+.4f}]  span = {bbox[1] - bbox[0]:.4f} m')
    print(f'  Y: [{bbox[2]:+.4f}, {bbox[3]:+.4f}]  span = {bbox[3] - bbox[2]:.4f} m')
    print(f'  Z: [{bbox[4]:+.4f}, {bbox[5]:+.4f}]  span = {bbox[5] - bbox[4]:.4f} m')
    print('\n=== Visible plant top (Z) at common scales ===')
    for s in [1.0, 2.0, 2.5, 3.0, 4.0]:
        print(f'  scale {s:.1f} -> plant top at z = {bbox[5] * s:.2f} m, '
              f'plant total height = {(bbox[5] - bbox[4]) * s:.2f} m')


if __name__ == '__main__':
    main()
