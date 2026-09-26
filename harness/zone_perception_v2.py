"""TOP-RGB item labels and view for mixed zone goals (goal v2; zone team A2).

Robot input only: the fixed TOP JPEGs, the authored TOP calibration and the
static cargo catalogue. Cargo profile ``top_cargo_v2`` by default
(``harness.zone_cargo_perception_v2``; ``top_cargo_v1`` stays selectable and
byte-frozen for the first A2 cohort); colour boxes through its ``top_zone_v2``
box path, minus box blobs lying on detected cargo. No simulator state.
Colour-only goals keep ``harness.zone_perception`` unchanged.

Low-confidence beams (v2 only): v2 sometimes reports a duplicate beam fragment,
always with confidence < 0.5. Such a beam never becomes a new item outright:
next to a confident beam it is a fragment of that beam; elsewhere it counts
only when another TOP view (a cross-view merge) or a later capture at the same
place confirms it. Unconfirmed beams are recorded in the view, never labelled
and never counted in a zone.

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

from harness import zone_cargo_perception as _cargo_v1
from harness import zone_cargo_perception_v2 as _cargo_v2
from harness.zone_cargo_perception import DEDUPE_M, grasp_handles

PROFILES = {'top_cargo_v1': _cargo_v1, 'top_cargo_v2': _cargo_v2}
DEFAULT_PROFILE = 'top_cargo_v2'
# Beams below this confidence need confirmation (v2 only; v1 keeps its cohort behaviour).
BEAM_CONFIRM_BELOW = .5
CONFIRM_PROFILES = ('top_cargo_v2',)
BEAM_FRAGMENT_M = .35          # half a beam: a weak beam this close to a confident one is its fragment
BEAM_SAME_PLACE_M = .05
PICKUP_MARGIN_M = .05


def source_text(profile):
    return (f'TOP RGB colour+shape ({profile}; boxes top_zone_v2) and authored TOP calibration; '
            'static cargo catalogue; not simulator state')


SOURCE = source_text(DEFAULT_PROFILE)


def _kind(row):
    return row['colour'] if row['kind'] == 'box' else row['kind']


def detect_items(tops, static_map, profile=DEFAULT_PROFILE):
    """[{'kind', 'floor_xy_m', 'yaw_rad', 'yaw_symmetry_deg', 'confidence', 'camera', 'views'}] from all TOPs."""
    view = PROFILES[profile].detect_all_cargo(tops, static_map, include_boxes=True)
    return [{'kind': _kind(r), 'floor_xy_m': list(r['floor_xy_m']), 'yaw_rad': r['yaw_rad'],
             'yaw_symmetry_deg': r['yaw_symmetry_deg'], 'confidence': r['confidence'], 'camera': r['camera'],
             'views': len(str(r['camera']).split('+'))}
            for r in view['items']]


def _weak_beam(d, profile):
    return (profile in CONFIRM_PROFILES and d['kind'] == 'long_beam'
            and float(d.get('confidence') or 0.) < BEAM_CONFIRM_BELOW)


def confirmed(detections, profile=DEFAULT_PROFILE, *, later=()):
    """(items, unconfirmed): weak beams are kept only when confirmed (see module doc).

    ``later``: detections of a later capture (time confirmation at the same place)."""
    strong = [d for d in detections if not _weak_beam(d, profile)]
    beams = [d for d in strong if d['kind'] == 'long_beam']
    items, unconfirmed = list(strong), []
    for d in detections:
        if not _weak_beam(d, profile):
            continue
        if any(math.dist(d['floor_xy_m'], b['floor_xy_m']) < BEAM_FRAGMENT_M for b in beams):
            unconfirmed.append(dict(d, reason='fragment_of_confident_beam'))
        elif d.get('views', 1) >= 2:
            items.append(dict(d, confirmed_by='another_view'))
        elif any(e['kind'] == 'long_beam' and math.dist(e['floor_xy_m'], d['floor_xy_m']) < BEAM_SAME_PLACE_M
                 for e in later):
            items.append(dict(d, confirmed_by='later_capture'))
        else:
            unconfirmed.append(dict(d, reason='weak_beam_unconfirmed'))
    return items, unconfirmed


def _inside(xy, region, margin=0.):
    (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
    return abs(xy[0]-cx) <= hx+margin and abs(xy[1]-cy) <= hy+margin


def _track_radius(kind):
    return DEDUPE_M.get(kind, DEDUPE_M['box'])


def label_items(detections, static_map, profile=DEFAULT_PROFILE, *, later=()):
    """Initial labels of every item in the pickup area (boxes and cargo); weak beams need confirmation."""
    pickup = static_map['regions']['pickup']
    items, _ = confirmed(detections, profile, later=later)
    rows = [d for d in items if _inside(d['floor_xy_m'], pickup, PICKUP_MARGIN_M)]
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


def view_from_detections(detections, static_map, labels, profile=DEFAULT_PROFILE):
    """Labelled items still seen at their pickup spot (any detection of the kind confirms an existing
    label), and confirmed items of each kind per zone (weak unconfirmed beams are never counted)."""
    counted, unconfirmed = confirmed(detections, profile)
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
        for d in counted:
            if _inside(d['floor_xy_m'], region):
                counts[d['kind']] = counts.get(d['kind'], 0) + 1
        zones[rid.split('_', 1)[1]] = counts
    return {'source': source_text(profile), 'profile': profile, 'pickup_items_still_visible': sorted(remaining),
            'zone_counts_seen': zones, 'detections': len(detections),
            'unconfirmed_beams': [{'floor_xy_m': d['floor_xy_m'], 'confidence': d['confidence'],
                                   'camera': d['camera'], 'reason': d['reason']} for d in unconfirmed]}


def observe_items(tops, static_map, labels, profile=DEFAULT_PROFILE):
    """Current RGB view: labelled items still at their pickup spot, and items of each kind per zone."""
    return view_from_detections(detect_items(tops, static_map, profile), static_map, labels, profile)


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


__all__ = ['SOURCE', 'PROFILES', 'DEFAULT_PROFILE', 'BEAM_CONFIRM_BELOW', 'source_text', 'confirmed', 'detect_items', 'label_items', 'observe_items', 'view_from_detections', 'goal_met',
           'public_labels']
