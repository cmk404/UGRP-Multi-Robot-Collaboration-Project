"""TOP-RGB item labels and view for mixed zone goals (goal v2; zone team A2).

Robot input only: the fixed TOP JPEGs, the authored TOP calibration and the
static cargo catalogue (``harness.zone_cargo_perception``, profile
``top_cargo_v1``; colour boxes through its ``top_zone_v2`` box path, minus box
blobs lying on detected cargo). No simulator state. Colour-only goals keep
``harness.zone_perception`` unchanged.

Labels are fixed names from the first TOP images: per kind (colours, then
catalogue kinds), west to east then south to north, like the box labels
(``red-1``, ``long_beam-1``). A cargo label also carries its handles (grip
points and approach poses from the RGB pose estimate + the static grasp
roles). Symmetric roles can swap names between views; the teacher binds a
claimed role to the physical handle nearest the RGB handle position, never by
role name alone.
"""
from __future__ import annotations

import copy
import math

from harness.zone_cargo_perception import DEDUPE_M, PROFILE as CARGO_PROFILE, detect_all_cargo, grasp_handles

SOURCE = ('TOP RGB colour+shape (top_cargo_v1; boxes top_zone_v2) and authored TOP calibration; '
          'static cargo catalogue; not simulator state')
PICKUP_MARGIN_M = .05


def _kind(row):
    return row['colour'] if row['kind'] == 'box' else row['kind']


def detect_items(tops, static_map):
    """[{'kind', 'floor_xy_m', 'yaw_rad', 'yaw_symmetry_deg', 'confidence', 'camera'}] from all TOPs."""
    view = detect_all_cargo(tops, static_map, include_boxes=True)
    return [{'kind': _kind(r), 'floor_xy_m': list(r['floor_xy_m']), 'yaw_rad': r['yaw_rad'],
             'yaw_symmetry_deg': r['yaw_symmetry_deg'], 'confidence': r['confidence'], 'camera': r['camera']}
            for r in view['items']]


def _inside(xy, region, margin=0.):
    (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
    return abs(xy[0]-cx) <= hx+margin and abs(xy[1]-cy) <= hy+margin


def _track_radius(kind):
    return DEDUPE_M.get(kind, DEDUPE_M['box'])


def label_items(detections, static_map):
    """Initial labels of every item in the pickup area (boxes and cargo)."""
    pickup = static_map['regions']['pickup']
    rows = [d for d in detections if _inside(d['floor_xy_m'], pickup, PICKUP_MARGIN_M)]
    kinds = list(static_map['box_kinds']) + sorted({d['kind'] for d in rows} - set(static_map['box_kinds']))
    labels = {}
    for kind in kinds:
        same = sorted((d for d in rows if d['kind'] == kind),
                      key=lambda d: (round(d['floor_xy_m'][0], 1), d['floor_xy_m'][1]))
        for i, d in enumerate(same, 1):
            value = {'kind': kind, 'floor_xy_m': list(d['floor_xy_m'])}
            if kind not in static_map['box_kinds']:
                value['yaw_rad'] = d['yaw_rad']
                handles = grasp_handles({'kind': kind, 'floor_xy_m': d['floor_xy_m'], 'yaw_rad': d['yaw_rad']})
                value['handles'] = {h['role']: {'grip_xy_m': h['grip_xyz_m'][:2],
                                                'approach_base_xyyaw': h['approach_base_xyyaw']}
                                    for h in handles['handles']}
            labels[f'{kind}-{i}'] = value
    return labels


def view_from_detections(detections, static_map, labels):
    remaining = {}
    for label, item in labels.items():
        if any(d['kind'] == item['kind'] and math.dist(d['floor_xy_m'], item['floor_xy_m']) <= _track_radius(item['kind'])
               for d in detections):
            remaining[label] = item
    zones = {}
    for rid, region in static_map['regions'].items():
        if not rid.startswith('zone_'):
            continue
        counts = {}
        for d in detections:
            if _inside(d['floor_xy_m'], region):
                counts[d['kind']] = counts.get(d['kind'], 0) + 1
        zones[rid.split('_', 1)[1]] = counts
    return {'source': SOURCE, 'profile': CARGO_PROFILE, 'pickup_items_still_visible': sorted(remaining),
            'zone_counts_seen': zones, 'detections': len(detections)}


def observe_items(tops, static_map, labels):
    """Current RGB view: labelled items still at their pickup spot, and items of each kind per zone."""
    return view_from_detections(detect_items(tops, static_map), static_map, labels)


def goal_met(goal, view):
    return all(view['zone_counts_seen'].get(zone, {}) == kinds for zone, kinds in goal.items())


def public_labels(labels):
    """What a robot's model reads about the labels (RGB estimates only)."""
    out = {}
    for name, v in labels.items():
        row = {'kind': v['kind'], 'rgb_floor_xy_m': copy.deepcopy(v['floor_xy_m'])}
        if 'handles' in v:
            row['rgb_yaw_rad'] = v.get('yaw_rad')
            row['handles'] = {role: {'rgb_grip_xy_m': h['grip_xy_m']} for role, h in v['handles'].items()}
        out[name] = row
    return out


__all__ = ['SOURCE', 'detect_items', 'label_items', 'observe_items', 'view_from_detections', 'goal_met',
           'public_labels']
