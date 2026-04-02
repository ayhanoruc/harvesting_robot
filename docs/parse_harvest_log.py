#!/usr/bin/env python3
"""
Parse harvest pipeline log and extract critical info into a clean summary.

Usage:
    # Record:
    ros2 launch orchestrator harvest_pipeline.launch.py 2>&1 | tee yolo_output/harvest.log

    # Parse:
    python3 parse_harvest_log.py yolo_output/harvest.log
    # or on Windows:
    python parse_harvest_log.py yolo_output/harvest.log
"""

import sys
import re
from collections import defaultdict

TAGS = [
    'STATE', 'PROGRESS', 'ARM', 'PICK', 'GRIPPER', 'IK',
    'SCAN STATUS', 'STEP-1', 'STEP-2', 'TCP',
    'VALIDATE', 'RETRY',
]

# Patterns for key events
PATTERNS = {
    'state_change': re.compile(r'\[STATE\] (\w+) -> (\w+)'),
    'progress': re.compile(r'\[PROGRESS\] (.+)'),
    'pick_start': re.compile(r'PICK #(\d+) START: boll=\(([^)]+)\)'),
    'pick_result': re.compile(r'PICK #(\d+) (SUCCESS|FAILED)'),
    'step': re.compile(r'\[(\d)/8\] (\w+):.+?in (\d+\.\d+)s'),
    'arm_result': re.compile(r'\[ARM\] .+result: (OK|FAIL) - (.+)'),
    'gripper': re.compile(r'\[GRIPPER\] (\w+): (OK|FAIL)'),
    'ik_solution': re.compile(r'\[IK\] Best solution \(cost=(\d+\.\d+)'),
    'yolo_detect': re.compile(r'/yolo/detect(?:_clusters)?: (\d+)'),
    'cluster_detected': re.compile(r'Scan: detected (detected_cluster_\d+) at \(([^)]+)\)'),
    'config_cluster': re.compile(r'Config cluster (\w+): \[([^\]]+)\]'),
    'harvest_complete': re.compile(r'HARVEST COMPLETE: (\d+)/(\d+) bolls, (\d+)s'),
    'saved_image': re.compile(r'Saved: (.+\.png)'),
    'depth_fail': re.compile(r'pixel_to_3d.+?: (.+)'),
    'tcp_pos': re.compile(r'\[TCP\] .+pos=\(([^)]+)\)'),
}


def parse_log(filepath):
    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        lines = f.readlines()

    summary = {
        'states': [],
        'progress': [],
        'picks': [],
        'images_saved': [],
        'clusters_detected': [],
        'clusters_config': [],
        'errors': [],
        'warnings': [],
    }
    current_pick = None

    for line in lines:
        line = line.strip()

        # State changes
        m = PATTERNS['state_change'].search(line)
        if m:
            summary['states'].append(f'{m.group(1)} -> {m.group(2)}')

        # Progress messages
        m = PATTERNS['progress'].search(line)
        if m:
            summary['progress'].append(m.group(1))

        # Pick tracking
        m = PATTERNS['pick_start'].search(line)
        if m:
            current_pick = {'id': m.group(1), 'boll': m.group(2), 'steps': []}

        m = PATTERNS['step'].search(line)
        if m and current_pick:
            current_pick['steps'].append(f'[{m.group(1)}/8] {m.group(2)}: {m.group(3)}s')

        m = PATTERNS['pick_result'].search(line)
        if m:
            if current_pick:
                current_pick['result'] = m.group(2)
                summary['picks'].append(current_pick)
                current_pick = None

        # Images
        m = PATTERNS['saved_image'].search(line)
        if m:
            summary['images_saved'].append(m.group(1))

        # Clusters
        m = PATTERNS['cluster_detected'].search(line)
        if m:
            summary['clusters_detected'].append(f'{m.group(1)}: ({m.group(2)})')

        m = PATTERNS['config_cluster'].search(line)
        if m:
            summary['clusters_config'].append(f'{m.group(1)}: [{m.group(2)}]')

        # Harvest complete
        m = PATTERNS['harvest_complete'].search(line)
        if m:
            summary['harvest_result'] = f'{m.group(1)}/{m.group(2)} bolls in {m.group(3)}s'

        # Errors and warnings
        if '[ERROR]' in line or 'FAIL' in line:
            # Keep it short
            node_match = re.search(r'\[(\w+)\].*?(FAIL|ERROR|error|failed)(.*)', line, re.IGNORECASE)
            if node_match:
                summary['errors'].append(line[-200:])  # last 200 chars

        if '[WARN]' in line:
            summary['warnings'].append(line[-200:])

    return summary


def print_summary(summary):
    print('=' * 70)
    print('HARVEST LOG SUMMARY')
    print('=' * 70)

    print(f'\n--- State transitions ({len(summary["states"])}) ---')
    for s in summary['states']:
        print(f'  {s}')

    print(f'\n--- Clusters detected by vision ({len(summary["clusters_detected"])}) ---')
    for c in summary['clusters_detected']:
        print(f'  {c}')

    print(f'\n--- Clusters used (config) ({len(summary["clusters_config"])}) ---')
    for c in summary['clusters_config']:
        print(f'  {c}')

    print(f'\n--- Progress log ({len(summary["progress"])}) ---')
    for p in summary['progress']:
        print(f'  {p}')

    print(f'\n--- Pick cycles ({len(summary["picks"])}) ---')
    for pick in summary['picks']:
        result = pick.get('result', '?')
        icon = 'OK' if result == 'SUCCESS' else 'FAIL'
        print(f'  Pick #{pick["id"]} [{icon}] boll=({pick["boll"]})')
        for step in pick['steps']:
            print(f'    {step}')

    if 'harvest_result' in summary:
        print(f'\n--- Final result ---')
        print(f'  {summary["harvest_result"]}')

    print(f'\n--- Images saved ({len(summary["images_saved"])}) ---')
    for img in summary['images_saved']:
        print(f'  {img}')

    if summary['errors']:
        print(f'\n--- Errors ({len(summary["errors"])}) ---')
        for e in summary['errors'][:20]:  # limit
            print(f'  {e}')

    if summary['warnings']:
        print(f'\n--- Warnings ({len(summary["warnings"])}) ---')
        for w in summary['warnings'][:20]:
            print(f'  {w}')

    print('\n' + '=' * 70)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python parse_harvest_log.py <logfile>')
        print('')
        print('Record with:')
        print('  ros2 launch orchestrator harvest_pipeline.launch.py 2>&1 | tee yolo_output/harvest.log')
        sys.exit(1)

    summary = parse_log(sys.argv[1])
    print_summary(summary)
