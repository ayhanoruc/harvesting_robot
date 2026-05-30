#!/usr/bin/env python3
"""
Strip distractor nodes (Tree_lp_*) from cotton_orchard_static DAE.

The cotton bundle's orchard mesh contains:
  - branch_*, pedicel_*, socket_* → cotton plant geometry (KEEP)
  - brown_*, green_*              → off-color "distractor" cotton bolls (KEEP — these
                                     test the detector's color discrimination)
  - Tree_lp_*                     → large low-poly TREE decorations (REMOVE — these
                                     dwarf the cotton plants in our scale and aren't
                                     wanted in the "clean" test world)
  - Ground                        → terrain (KEEP)

Reads:
  robot_arm/models/cotton_orchard_static/meshes/cotton_orchard_static_without_pickable_cottons.dae

Writes:
  robot_arm/models/cotton_orchard_static/meshes/cotton_orchard_static_no_trees.dae

The unreferenced tree geometry stays in <library_geometries> but is harmless —
nothing in the scene graph instances it anymore so nothing renders.
"""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET

NS_COLLADA = 'http://www.collada.org/2005/11/COLLADASchema'
NSPRE = f'{{{NS_COLLADA}}}'

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
MESH_DIR = os.path.join(PKG, 'models', 'cotton_orchard_static', 'meshes')
SRC = os.path.join(MESH_DIR, 'cotton_orchard_static_without_pickable_cottons.dae')
DST = os.path.join(MESH_DIR, 'cotton_orchard_static_no_trees.dae')

# Patterns of node names to REMOVE (anything else is kept)
REMOVE_PATTERNS = [
    re.compile(r'^Tree_lp_'),
]


def should_remove(name: str) -> bool:
    if not name:
        return False
    for p in REMOVE_PATTERNS:
        if p.match(name):
            return True
    return False


def _strip_recursive(parent: ET.Element, kept: list, removed: list) -> None:
    """Walk <node> descendants, remove ones whose name matches REMOVE_PATTERNS."""
    to_remove = []
    for child in list(parent):
        if child.tag != f'{NSPRE}node':
            # Recurse into non-node containers too
            _strip_recursive(child, kept, removed)
            continue
        name = child.get('name', '')
        if should_remove(name):
            removed.append(name)
            to_remove.append(child)
        else:
            kept.append(name or '(unnamed)')
            _strip_recursive(child, kept, removed)
    for ch in to_remove:
        parent.remove(ch)


def main():
    if not os.path.isfile(SRC):
        raise FileNotFoundError(f'Source DAE missing: {SRC}')

    # Preserve the COLLADA default namespace so output stays compatible
    ET.register_namespace('', NS_COLLADA)

    print(f'Parsing {SRC} ({os.path.getsize(SRC) / 1e6:.1f} MB) ...')
    tree = ET.parse(SRC)
    root = tree.getroot()

    kept: list = []
    removed: list = []
    _strip_recursive(root, kept, removed)

    print(f'Kept nodes: {len(kept)}  |  Removed nodes: {len(removed)}')
    if removed:
        print(f'  removed examples: {removed[:5]}')

    tree.write(DST, xml_declaration=True, encoding='utf-8')
    print(f'Wrote {DST} ({os.path.getsize(DST) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
