#!/usr/bin/env python3
"""
Extract ONLY the Ground node (and its referenced geometry) from
cotton_orchard_static_no_trees.dae so the textured terrain can be reused
in our compact cotton_demo world WITHOUT pulling in the 12 baked clusters.

Why: cotton_demo.world uses template-instanced clusters at compact custom
positions (X-spacing 3 m). Re-including cotton_orchard_clean would also
drop the bundle's 12 clusters at their original positions (~60 m strip),
visible off in the distance — confusing for demos. Extracting just the
Ground gives us the dirt+grass texture under our compact field with no
extra plants.

Approach (mirrors extract_cluster_template.py): drop every node in
<visual_scene> direct children EXCEPT the Ground node. Library geometries
aren't pruned (unused geometries stay but don't render) — keeps the script
trivial, file stays a few MB which Gazebo loads once.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

NS = 'http://www.collada.org/2005/11/COLLADASchema'
NSPRE = f'{{{NS}}}'
ET.register_namespace('', NS)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(SCRIPT_DIR, '..'))
SRC_DAE = os.path.join(PKG, 'models', 'cotton_orchard_static', 'meshes',
                       'cotton_orchard_static_no_trees.dae')
DST_DIR = os.path.join(PKG, 'models', 'cotton_field_ground', 'meshes')
DST_DAE = os.path.join(DST_DIR, 'cotton_field_ground.dae')


def main():
    if not os.path.isfile(SRC_DAE):
        raise FileNotFoundError(SRC_DAE)
    print(f'Parsing {SRC_DAE} ({os.path.getsize(SRC_DAE) / 1e6:.1f} MB)')
    tree = ET.parse(SRC_DAE)
    root = tree.getroot()

    kept = []
    removed = 0
    for vs in root.iter(f'{NSPRE}visual_scene'):
        for node in list(vs):
            if node.tag != f'{NSPRE}node':
                continue
            name = node.get('name', '')
            if name == 'Ground' or name.startswith('Ground'):
                kept.append(name)
            else:
                vs.remove(node)
                removed += 1

    print(f'Kept {len(kept)} ground node(s): {kept}')
    print(f'Removed {removed} other nodes (branches, pedicels, sockets, ...).')

    os.makedirs(DST_DIR, exist_ok=True)
    tree.write(DST_DAE, xml_declaration=True, encoding='utf-8')
    print(f'Wrote {DST_DAE} ({os.path.getsize(DST_DAE) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
