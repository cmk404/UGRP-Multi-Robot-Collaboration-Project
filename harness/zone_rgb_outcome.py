"""RGB-only job outcome check for the zone benchmark (candidate replacement for audit leak L4).

The zone runner tells robots whether a job finished from the TEACHER's
ground-truth checks (lift height, drop height, blocked path; see
``experiments/2026-09-25-zone-comm-boundary-audit`` L4). This module decides the
outcome of one robot's (or one team's) job from allowed robot inputs only:

* fixed TOP RGB JPEGs with the authored TOP calibration: the frame the item
  labels were made from ("reference", normally the first TOP frame), the frame
  at the job's assignment ("before") and the current frame,
* optionally the robot's own RGB JPEG with its own *issued* arm pulses,
* the static map (zones), the job's own target area (zone slot or landing
  area) and the static cargo catalogue (item footprints),
* the job spec the robot(s) issued: item label, kind, RGB-estimated source
  position (``box_labels[*].floor_xy_m``),
* the carriers' own issued command log (port commands with SIM times).

It never reads simulator poses, contacts, measured joints, teacher phases or
outcomes, or injection flags. Issued commands never count as success: they
only say what to look for, give the camera pose for own RGB, and act as a
*necessary* gate (a carried item is not "delivered" while its carriers'
grippers are still commanded closed).

Two layers
    ``observe``      one momentary observation from the current frames.
    ``JobTracker``   fed on a fixed SIM cadence from the job's assignment (not
                     from the executor's end), returns ``decide``'s decision:
                     ``status`` is ``confirmed`` or ``unconfirmed``. Only a
                     confirmed decision may become a receipt or a wake-up.

Outcomes
    ``delivered``        an item of the job's kind is newly seen inside the
                         job's target area (cargo: its full footprint and yaw),
                         stable over two ticks, and the source is *proven*
                         visibly empty.
    ``still_at_source``  the item is still seen at (or near) its source.
    ``seen_elsewhere``   the source is proven visibly empty and the kind is
                         newly seen somewhere that is not the target.
    ``not_seen``         none of the above can be shown (occluded or lost).

Source proof: the item was detected at the source in the reference frame in
some TOP (so that camera had a line of sight to the spot), the floor ring
around it looked like floor then, the ring is unchanged now, and the centre
now looks like that bare floor. "Unchanged" alone is not "unoccluded": a robot
that stood over the source already at the reference or assignment and never
moved leaves the ring unchanged, but fails the first two conditions.

``confidence`` is a fixed heuristic score per rule (not a calibrated
probability). Thresholds were chosen on dev splits only
(``experiments/2026-09-25-zone-rgb-outcome``).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np

from harness import zone_color_boxes as _zcb
from harness import zone_perception as _zp
from harness import zone_team_footprint as _ztf

try:   # claude/zone-cargo-perception (PR #168); imported, never copied
    from harness import zone_cargo_perception as _zcp
except ImportError:   # pragma: no cover - depends on the checkout
    _zcp = None

SCHEMA = 'ugrp.zone_rgb_outcome.v2'
OUTCOMES = ('delivered', 'still_at_source', 'seen_elsewhere', 'not_seen')
UNCONFIRMED = 'unconfirmed'
BOX_KINDS = _zcb.KINDS
CARGO_KINDS = ('can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')
PROVENANCE = ('TOP RGB reference/before/current + authored TOP calibration + static map/target area/catalogue'
              ' + own job spec + own issued commands (+ own RGB); no simulator state, contact or teacher result')

# --- thresholds -------------------------------------------------------------
TOP_PROFILE = _zcb.TOP_PROFILE_ZONE
SOURCE_TRACK_M = _zp.TRACK_RADIUS_M
SOURCE_NEAR_M = {'box': .12, 'can': .12, 'tile': .12, 'heavy_crate': .16, 'long_beam': .22, 'tri_frame': .22}
NEW_SIGHTING_M = .06
TARGET_TOL_M = .06
RING_INNER_M, RING_OUTER_M = .045, .11
CENTRE_M = .03
PIXEL_DIFF = 28            # max-channel |a - b| counted as changed
RING_CHANGED_MAX = .30     # ring pixels changed since the reference
FLOOR_DIST = 30            # max-channel distance to the ring median counted as "floor-like"
RING_FLOORLIKE_MIN = .70   # reference ring: share of floor-like pixels
CENTRE_BARE_MIN = .70      # current centre: share of pixels like the current ring floor
IMAGE_MARGIN_PX = 3
RING_MIN_INSIDE = .5
OWN_REACH_M = .50
# Cargo landing: detected yaw within this of the landing yaw (mod symmetry).
CARGO_YAW_TOL_DEG = 15.
# Decision policy
CADENCE_S = 1.
STABLE_TICKS = 2
STABLE_M = .03
DEADLINE_S = 180.          # teacher-free: SIM seconds after assignment
RELEASES_AT_SOURCE = 3     # command-evidenced releases with the item still at the source
RELEASE_SETTLE_S = 1.
GRIPPER_SERVO = 1
GRIPPER_OPEN_MIN = 1750
COMMIT_CONFIDENCE = .6

CONFIDENCE = {
    'delivered_source_proven': .95,
    'delivered_source_unproven': .50,
    'delivered_zone_count': .60,
    'still_at_source_tracked': .95,
    'still_at_source_near': .75,
    'still_at_source_conflict': .50,
    'still_at_source_own_rgb': .60,
    'seen_elsewhere': .80,
    'seen_elsewhere_own_rgb': .55,
    'not_seen': .0,
}


# --------------------------------------------------------------------------- geometry

def floor_to_pixel(x, y, camera, shape, *, height=.016):
    """Inverse of ``zone_perception.pixel_to_floor`` for the authored downward TOP."""
    if camera['quaternion_wxyz'] != [1, 0, 0, 0]:
        raise ValueError('unsupported TOP orientation')
    h, w = shape[:2]
    cx, cy, cz = camera['position_m']
    scale = h / (2*(cz-height)*math.tan(math.radians(camera['fov_y_deg'])/2))
    return (x-cx)*scale + (w-1)/2, -(y-cy)*scale + (h-1)/2, scale


def inside_rect(xy, center, half, margin=0.):
    return abs(xy[0]-center[0]) <= half[0]+margin and abs(xy[1]-center[1]) <= half[1]+margin


def _near_kind(kind):
    return SOURCE_NEAR_M['box' if kind in BOX_KINDS else kind]


def _yaw_diff_deg(a, b, symmetry_deg):
    d = math.degrees(a - b)
    if symmetry_deg:
        d = (d + symmetry_deg/2) % symmetry_deg - symmetry_deg/2
    else:
        d = (d + 180) % 360 - 180
    return abs(d)


# --------------------------------------------------------------------------- detection

def _decode(jpeg):
    if isinstance(jpeg, np.ndarray):
        return jpeg
    return _zp._decode(jpeg)


def sightings(tops: Mapping[str, bytes], static_map: Mapping[str, Any], kinds: Sequence[str], *,
              profile: str = TOP_PROFILE, per_camera: bool = False) -> list[dict[str, Any]]:
    """Item sightings of the given kinds from all TOPs (floor xy), one row per item
    (or per camera view with ``per_camera``). Colour boxes: ``zone_color_boxes``;
    catalogue kinds: ``zone_cargo_perception`` (rows keep ``yaw_rad``, ``clipped``)."""
    box_kinds = [k for k in kinds if k in BOX_KINDS]
    cargo_kinds = [k for k in kinds if k not in BOX_KINDS]
    rows = []
    if box_kinds:
        raw = []
        for camera in static_map['top_cameras']:
            for r in _zcb.detect_top(tops[camera['name']], camera, box_kinds, profile=profile):
                raw.append((math.hypot(r['pixel'][0]-.5, r['pixel'][1]-.5), dict(r)))
        raw.sort(key=lambda t: t[0])
        if per_camera:
            rows += [r for _, r in raw]
        else:
            for _, r in raw:   # same merge rule as zone_perception.detect_all
                if not any(k['kind'] == r['kind'] and math.dist(k['floor_xy_m'], r['floor_xy_m']) < .05 for k in rows):
                    rows.append(r)
    if cargo_kinds:
        if _zcp is None:
            raise RuntimeError('catalogue kinds need harness.zone_cargo_perception (claude/zone-cargo-perception)')
        if per_camera:
            for camera in static_map['top_cameras']:
                for r in _zcp.detect_cargo_top(tops[camera['name']], camera, cargo_kinds, include_boxes=False):
                    rows.append({'kind': r['kind'], 'floor_xy_m': list(r['floor_xy_m']), 'camera': camera['name'],
                                 'yaw_rad': r.get('yaw_rad'), 'clipped': r.get('clipped', False)})
        else:
            found = _zcp.detect_all_cargo(tops, static_map, cargo_kinds, include_boxes=False)
            rows += [{'kind': r['kind'], 'floor_xy_m': list(r['floor_xy_m']), 'camera': r.get('camera'),
                      'yaw_rad': r.get('yaw_rad'), 'clipped': r.get('clipped', False),
                      'confidence': r.get('confidence')} for r in found['items'] if r['kind'] in cargo_kinds]
    return rows


def _ring_geometry(u, v, scale, shape):
    h, w = shape[:2]
    r_out = RING_OUTER_M*scale
    x0, x1 = int(u-r_out)-1, int(u+r_out)+2
    y0, y1 = int(v-r_out)-1, int(v+r_out)+2
    yy, xx = np.mgrid[y0:y1, x0:x1]
    d = np.hypot(xx-u, yy-v)
    inside = (xx >= IMAGE_MARGIN_PX) & (xx <= w-1-IMAGE_MARGIN_PX) & (yy >= IMAGE_MARGIN_PX) & (yy <= h-1-IMAGE_MARGIN_PX)
    full = (d >= RING_INNER_M*scale) & (d <= r_out)
    centre = (d <= CENTRE_M*scale) & inside
    cx0, cy0, cx1, cy1 = max(x0, 0), max(y0, 0), min(x1, w), min(y1, h)
    crop = (slice(cy0, cy1), slice(cx0, cx1))
    local = (slice(cy0-y0, cy1-y0), slice(cx0-x0, cx1-x0))
    return full, full & inside, centre, crop, local


def _floorlike(pixels, reference):
    if not len(pixels):
        return 0.
    return float((np.abs(pixels.astype(np.int16) - reference.astype(np.int16)).max(axis=1) <= FLOOR_DIST).mean())


def point_state(reference_tops, current_tops, static_map, xy, *, reference_rows=None, decoded=None):
    """Per-camera state of the floor spot ``xy``: is the ring unchanged since the
    reference, was the reference ring floor-like, does the centre now look like
    bare floor, and did that camera see an item at ``xy`` in the reference.

    ``proven_empty`` needs all four in one camera. ``unchanged`` alone (any
    camera) is reported separately and is never enough to prove anything."""
    decoded = {} if decoded is None else decoded
    seen_in = {r['camera'] for r in (reference_rows or [])
               if math.dist(r['floor_xy_m'], xy) <= SOURCE_TRACK_M}
    views = []
    for camera in static_map['top_cameras']:
        name = camera['name']
        for key, tops in (('r', reference_tops), ('c', current_tops)):
            if (key, name) not in decoded:
                decoded[(key, name)] = _decode(tops[name])
        ref, cur = decoded[('r', name)], decoded[('c', name)]
        u, v, scale = floor_to_pixel(xy[0], xy[1], camera, cur.shape)
        full, ring, centre, crop, local = _ring_geometry(u, v, scale, cur.shape)
        if ring.sum() < RING_MIN_INSIDE*full.sum() or not ring.any():
            continue
        ring_l, centre_l = ring[local], centre[local]
        ref_c, cur_c = ref[crop], cur[crop]
        diff = np.abs(cur_c.astype(np.int16) - ref_c.astype(np.int16)).max(axis=2)
        changed = float((diff[ring_l] > PIXEL_DIFF).mean())
        ref_ring, cur_ring = ref_c[ring_l], cur_c[ring_l]
        ref_floor = _floorlike(ref_ring, np.median(ref_ring, axis=0))
        centre_bare = _floorlike(cur_c[centre_l], np.median(cur_ring, axis=0)) if centre_l.any() else 0.
        unchanged = changed <= RING_CHANGED_MAX
        proven = bool(unchanged and ref_floor >= RING_FLOORLIKE_MIN and centre_bare >= CENTRE_BARE_MIN
                      and name in seen_in)
        views.append({'camera': name, 'pixel': [round(u, 1), round(v, 1)], 'ring_changed': round(changed, 3),
                      'reference_ring_floorlike': round(ref_floor, 3), 'centre_bare': round(centre_bare, 3),
                      'item_seen_here_in_reference': name in seen_in, 'unchanged': unchanged, 'proven_empty': proven})
    return {'proven_empty': any(v['proven_empty'] for v in views),
            'unchanged': any(v['unchanged'] for v in views), 'views': views}


def ring_unchanged(before_tops, current_tops, static_map, xy, *, decoded=None):
    """Weak cue only (target flags): some TOP sees an unchanged floor ring around ``xy``."""
    return point_state(before_tops, current_tops, static_map, xy, decoded=decoded)['unchanged']


def own_near(own_rgb, own_arm_pulses, kind):
    """Own RGB cue from the robot's own issued arm pulses: an item of ``kind``
    within reach (near floor fit within OWN_REACH_M, or clipped at the border)."""
    if own_rgb is None or own_arm_pulses is None or kind not in BOX_KINDS:
        return None
    if not all(s in own_arm_pulses for s in (3, 4, 5, 6)):
        return None
    res = _zcb.detect_own(own_rgb, own_arm_pulses, [kind], profile=_zcb.OWN_PROFILE_ZONE)
    near = [d for d in res['detections'] if d['range_class'] == 'near'
            and math.hypot(*d['estimated_box_center_base_m'][:2]) <= OWN_REACH_M]
    return {'within_reach': bool(near) or kind in res['clipped_kinds'],
            'near_detections': [[round(v, 3) for v in d['estimated_box_center_base_m'][:2]] for d in near],
            'clipped_at_border': kind in res['clipped_kinds']}


# --------------------------------------------------------------------------- commands (own, issued)

def issued_arm(commands, t):
    """Last issued pulse per servo at SIM time ``t`` from a port command log."""
    pulses = {}
    for c in commands or ():
        if c['sim_time_s'] > t + 1e-9:
            break
        if c['kind'] == 'arm':
            pulses[int(c['servo_id'])] = int(c['pulse'])
        elif c['kind'] == 'look':
            pulses[6] = int(c['pan_pulse'])
        elif c['kind'] == 'initial':
            pulses.update({int(k): int(v) for k, v in c['pulses'].items()})
    return pulses


def gripper_open(commands, t):
    """True/False from the last issued gripper pulse, None when never issued."""
    p = issued_arm(commands, t).get(GRIPPER_SERVO)
    return None if p is None else p >= GRIPPER_OPEN_MIN


def releases(commands, t0, t1):
    """SIM times in (t0, t1] at which the issued gripper went from closed to open."""
    out, prev = [], None
    for c in commands or ():
        if c['sim_time_s'] > t1 + 1e-9:
            break
        p = None
        if c['kind'] == 'arm' and int(c['servo_id']) == GRIPPER_SERVO:
            p = int(c['pulse'])
        elif c['kind'] == 'initial' and str(GRIPPER_SERVO) in {str(k) for k in c['pulses']}:
            p = int({str(k): v for k, v in c['pulses'].items()}[str(GRIPPER_SERVO)])
        if p is None:
            continue
        is_open = p >= GRIPPER_OPEN_MIN
        if prev is False and is_open and c['sim_time_s'] > t0:
            out.append(c['sim_time_s'])
        prev = is_open
    return out


# --------------------------------------------------------------------------- jobs

def job_spec(*, robot_ids, item, kind, source_xy_m, zone, target=None):
    """What the robot(s) issued. ``target``: {'center_m', 'half_extents_m'} of the
    job's own slot, or a landing area (``landing_target``, carries yaw), or None."""
    robot_ids = [robot_ids] if isinstance(robot_ids, str) else list(robot_ids)
    tgt = None
    if target is not None:
        tgt = {'center_m': [float(v) for v in target['center_m']],
               'half_extents_m': [float(v) for v in target['half_extents_m']]}
        if target.get('item_yaw_rad') is not None:
            tgt['item_yaw_rad'] = float(target['item_yaw_rad'])
        tgt['includes_tolerance'] = bool(target.get('includes_tolerance', False))
    if kind not in BOX_KINDS and tgt is None:
        raise ValueError('catalogue items need a landing area target')
    return {'robot_ids': robot_ids, 'item': item, 'kind': kind, 'source_xy_m': [float(v) for v in source_xy_m],
            'zone': zone, 'target': tgt}


def slot_target(static_map, slot_id):
    for slots in static_map['zone_slots'].values():
        for s in slots:
            if s['slot_id'] == slot_id:
                return {'center_m': s['center_m'], 'half_extents_m': s['half_extents_m']}
    raise KeyError(slot_id)


def landing_target(area):
    """A ``zone_goal_v2.landing_layout`` area (its half extents already include the tolerance)."""
    return {'center_m': area['landing_center_m'], 'half_extents_m': area['landing_half_extents_m'],
            'item_yaw_rad': area['item_pose'][2], 'includes_tolerance': True}


def _in_target(row, job):
    """Boxes: centre in the area (+ tolerance). Cargo: unclipped, full footprint
    inside the landing area, yaw within tolerance of the landing yaw."""
    tgt = job['target']
    if tgt is None:
        return False
    tol = 0. if tgt.get('includes_tolerance') else TARGET_TOL_M
    if job['kind'] in BOX_KINDS:
        return inside_rect(row['floor_xy_m'], tgt['center_m'], tgt['half_extents_m'], tol)
    if row.get('clipped') or row.get('yaw_rad') is None and job['kind'] != 'can':
        return False
    yaw = row.get('yaw_rad') or 0.
    polys = [_ztf.transform(p, (row['floor_xy_m'][0], row['floor_xy_m'][1], yaw)) for p in _ztf.item_polygons(job['kind'])]
    if not _ztf.polygons_inside_rect(polys, tgt['center_m'], [h+tol for h in tgt['half_extents_m']]):
        return False
    sym = _zcp.YAW_SYMMETRY_DEG.get(job['kind']) if _zcp is not None else None
    if job['kind'] != 'can' and tgt.get('item_yaw_rad') is not None:
        return _yaw_diff_deg(yaw, tgt['item_yaw_rad'], sym) <= CARGO_YAW_TOL_DEG
    return True


def observe(job, reference_tops, before_tops, current_tops, static_map, *, own_rgb=None, own_arm_pulses=None,
            profile=TOP_PROFILE, reference_rows=None, before_rows=None, names=None):
    """One momentary observation (not a decision). ``reference_tops``: the frame
    the item labels came from; ``before_tops``: at assignment."""
    kind, src = job['kind'], job['source_xy_m']
    target = job['target']
    zone_region = static_map['regions']['zone_'+job['zone']]
    if reference_rows is None:
        reference_rows = [r for r in sightings(reference_tops, static_map, [kind], profile=profile, per_camera=True)
                          if r['kind'] == kind]
    before = before_rows if before_rows is not None else sightings(before_tops, static_map, [kind], profile=profile)
    after = sightings(current_tops, static_map, [kind], profile=profile)
    before = [r for r in before if r['kind'] == kind]
    after = [r for r in after if r['kind'] == kind]
    near_m = _near_kind(kind)

    def at_source(rows, radius):
        return [r for r in rows if math.dist(r['floor_xy_m'], src) <= radius]

    def in_zone(rows):
        return [r for r in rows if inside_rect(r['floor_xy_m'], zone_region['center_m'], zone_region['half_extents_m'])]

    def is_new(r):
        return all(math.dist(r['floor_xy_m'], b['floor_xy_m']) > NEW_SIGHTING_M for b in before)

    src_tracked, src_near = at_source(after, SOURCE_TRACK_M), at_source(after, near_m)
    others_near_before = max(0, len(at_source(before, near_m)) - 1)
    tgt_before = [r for r in before if _in_target(r, job)]
    tgt_after = [r for r in after if _in_target(r, job)]
    tgt_new = [r for r in tgt_after if is_new(r)] if len(tgt_after) > len(tgt_before) else []
    zone_gain = len(in_zone(after)) - len(in_zone(before))
    elsewhere = [r for r in after if is_new(r) and r not in tgt_after and math.dist(r['floor_xy_m'], src) > near_m]
    src_state = point_state(reference_tops, current_tops, static_map, src, reference_rows=reference_rows)
    tgt_unchanged = (ring_unchanged(before_tops, current_tops, static_map, target['center_m'])
                     if target is not None else None)
    in_own_zone = [r for r in elsewhere if r in in_zone(elsewhere)]
    flags = []
    if in_own_zone and not tgt_unchanged:
        elsewhere = [r for r in elsewhere if r not in in_own_zone]
        flags.append('new_same_kind_in_zone_while_target_changed')
    elif in_own_zone:
        flags.append('new_same_kind_in_zone_outside_target')
    own = own_near(own_rgb, own_arm_pulses, kind)
    proven = src_state['proven_empty'] and not src_near
    if others_near_before:
        flags.append('another_same_kind_item_near_source_before')
    if target is not None and tgt_before:
        flags.append('target_area_held_same_kind_before')
    if src_near and not src_tracked:
        flags.append('source_item_moved_but_near')
    if not src_state['proven_empty']:
        flags.append('source_emptiness_unproven')

    if src_near:
        if tgt_new:
            outcome, rule = 'still_at_source', 'still_at_source_conflict'
        elif src_tracked and not others_near_before:
            outcome, rule = 'still_at_source', 'still_at_source_tracked'
        else:
            outcome, rule = 'still_at_source', 'still_at_source_near'
    elif target is not None and tgt_new:
        outcome, rule = 'delivered', ('delivered_source_proven' if proven else 'delivered_source_unproven')
    elif target is None and zone_gain > 0 and proven:
        outcome, rule = 'delivered', 'delivered_zone_count'
    elif proven and elsewhere:
        outcome, rule = 'seen_elsewhere', 'seen_elsewhere'
    elif not src_state['unchanged'] and own and own['within_reach']:
        outcome, rule = 'still_at_source', 'still_at_source_own_rgb'
    elif proven and own and own['within_reach']:
        outcome, rule = 'seen_elsewhere', 'seen_elsewhere_own_rgb'
    else:
        outcome, rule = 'not_seen', 'not_seen'
        if target is not None and not tgt_unchanged:
            flags.append('target_changed')

    sighting = tgt_new[0] if tgt_new else None
    return {
        'schema': SCHEMA, 'outcome': outcome, 'confidence': CONFIDENCE[rule], 'rule': rule, 'flags': flags,
        'target_sighting': None if sighting is None else {'floor_xy_m': sighting['floor_xy_m'],
                                                          'yaw_rad': sighting.get('yaw_rad')},
        'evidence': {
            'images': sorted(set((names or {}).values())), 'top_profile': profile,
            'source': {'tracked': [r['floor_xy_m'] for r in src_tracked], 'near': [r['floor_xy_m'] for r in src_near],
                       'state': src_state},
            'target': None if target is None else {'before': [r['floor_xy_m'] for r in tgt_before],
                                                   'after': [r['floor_xy_m'] for r in tgt_after],
                                                   'new': [r['floor_xy_m'] for r in tgt_new],
                                                   'ring_unchanged_since_assignment': tgt_unchanged},
            'zone_count_change': zone_gain, 'new_elsewhere': [r['floor_xy_m'] for r in elsewhere],
            'own_rgb': own,
        },
        'provenance': PROVENANCE,
    }


# --------------------------------------------------------------------------- decisions

def decide(job, history, *, assigned_at, now, commands=None, released_at_source=0, deadline_s=DEADLINE_S):
    """Decision from the observation history (oldest first). Returns
    ``{'status': 'confirmed'|'unconfirmed', 'outcome': <outcome>|'unconfirmed', ...}``.

    * delivered: the last STABLE_TICKS observations are ``delivered_source_proven``
      (or ``delivered_zone_count``), CADENCE-consecutive, with the target sighting
      within STABLE_M; and no carrier's issued gripper is commanded closed.
      Catalogue items also need every carrier's gripper known and open.
    * still_at_source early: RELEASES_AT_SOURCE command-evidenced releases each
      followed by the item still tracked at the source, and now still there.
    * at the teacher-free deadline: the last three observations agree on a
      non-delivered outcome with confidence >= COMMIT_CONFIDENCE, else not_seen.
    * otherwise unconfirmed.
    """
    base = {'status': 'unconfirmed', 'outcome': UNCONFIRMED, 'confidence': 0., 'rule': 'observing',
            'decided_at': None, 'evidence_images': [], 'flags': []}
    if not history:
        return base
    last = history[-1]
    tail = history[-STABLE_TICKS:]
    ok_rules = ('delivered_source_proven', 'delivered_zone_count')
    stable = (len(tail) == STABLE_TICKS and all(o['rule'] in ok_rules for o in tail)
              and all(abs((b['t'] - a['t']) - CADENCE_S) < 1e-6 for a, b in zip(tail, tail[1:])))
    if stable and tail[0]['rule'] == 'delivered_source_proven':
        xy = [o['target_sighting']['floor_xy_m'] for o in tail]
        stable = all(math.dist(xy[0], p) <= STABLE_M for p in xy)
    if stable:
        grip = {rid: [gripper_open((commands or {}).get(rid), o['t']) for o in tail] for rid in job['robot_ids']}
        closed = any(g is False for gs in grip.values() for g in gs)
        unknown = any(g is None for gs in grip.values() for g in gs)
        if closed:
            base['flags'] = ['target_item_but_carrier_gripper_closed']
        elif unknown and job['kind'] not in BOX_KINDS:
            base['flags'] = ['cargo_lifted_state_unknown']
        else:
            conf = min(o['confidence'] for o in tail)
            return {'status': 'confirmed', 'outcome': 'delivered', 'confidence': conf, 'rule': tail[-1]['rule'],
                    'decided_at': last['t'], 'evidence_images': sorted({i for o in tail for i in o['images']}),
                    'flags': ['gripper_state_not_logged'] if unknown else []}
    if released_at_source >= RELEASES_AT_SOURCE and last['rule'] == 'still_at_source_tracked':
        return {'status': 'confirmed', 'outcome': 'still_at_source', 'confidence': .95,
                'rule': 'released_at_source', 'decided_at': last['t'], 'evidence_images': last['images'],
                'flags': [f'{released_at_source}_releases_with_item_still_at_source']}
    if now - assigned_at >= deadline_s:
        tail3 = history[-3:]
        outs = {o['outcome'] for o in tail3}
        if (len(outs) == 1 and last['outcome'] not in ('delivered', 'not_seen')
                and all(o['confidence'] >= COMMIT_CONFIDENCE for o in tail3)):
            return {'status': 'confirmed', 'outcome': last['outcome'], 'confidence': min(o['confidence'] for o in tail3),
                    'rule': 'deadline_' + last['rule'], 'decided_at': last['t'],
                    'evidence_images': sorted({i for o in tail3 for i in o['images']}), 'flags': []}
        return {'status': 'confirmed', 'outcome': 'not_seen', 'confidence': 0., 'rule': 'deadline_not_seen',
                'decided_at': last['t'], 'evidence_images': last['images'], 'flags': sorted(outs)}
    base['latest_observation'] = {k: last[k] for k in ('t', 'outcome', 'confidence', 'rule')}
    return base


class JobTracker:
    """Feed on a fixed SIM cadence from the job's assignment (``CADENCE_S``)."""

    def __init__(self, job, reference_tops, before_tops, static_map, *, assigned_at, profile=TOP_PROFILE,
                 deadline_s=DEADLINE_S, keep_evidence=False):
        self.job, self.static, self.profile = job, static_map, profile
        self.reference, self.before = reference_tops, before_tops
        self.assigned_at, self.deadline_s = float(assigned_at), float(deadline_s)
        kind = job['kind']
        self.reference_rows = [r for r in sightings(reference_tops, static_map, [kind], profile=profile, per_camera=True)
                               if r['kind'] == kind]
        self.before_rows = sightings(before_tops, static_map, [kind], profile=profile)
        self.history, self.observations = [], []
        self.release_checks = {}   # release time -> item still tracked at the source after it (or None)
        self.decision = decide(job, [], assigned_at=self.assigned_at, now=self.assigned_at)
        self.keep_evidence = keep_evidence

    def update(self, t, current_tops, *, commands=None, own_rgb=None, names=None):
        if self.decision['status'] == 'confirmed':
            return self.decision
        own_pulses = None
        if commands is not None and own_rgb is not None:
            own_pulses = issued_arm(commands.get(self.job['robot_ids'][0]), t) or None
        obs = observe(self.job, self.reference, self.before, current_tops, self.static, own_rgb=own_rgb,
                      own_arm_pulses=own_pulses, profile=self.profile, reference_rows=self.reference_rows,
                      before_rows=self.before_rows, names=names)
        entry = {'t': round(float(t), 3), 'outcome': obs['outcome'], 'confidence': obs['confidence'],
                 'rule': obs['rule'], 'flags': obs['flags'], 'target_sighting': obs['target_sighting'],
                 'images': obs['evidence']['images']}
        self.history.append(entry)
        if self.keep_evidence:
            self.observations.append(obs)
        if commands is not None:
            for rid in self.job['robot_ids']:
                for r in releases(commands.get(rid), self.assigned_at, t - RELEASE_SETTLE_S):
                    if (rid, r) not in self.release_checks:
                        self.release_checks[(rid, r)] = obs['rule'] == 'still_at_source_tracked'
        at_source = sum(1 for v in self.release_checks.values() if v)
        self.decision = decide(self.job, self.history, assigned_at=self.assigned_at, now=t, commands=commands,
                               released_at_source=at_source, deadline_s=self.deadline_s)
        return self.decision


# --------------------------------------------------------------------------- runner-facing helpers

RECEIPT_TEXT = {
    'delivered': 'RGB check: item seen in the target area, source seen empty',
    'still_at_source': 'RGB check: item still seen at its source',
    'seen_elsewhere': 'RGB check: item seen away from source and target',
    'not_seen': 'RGB check: item not seen (occluded or lost) by the deadline',
    UNCONFIRMED: 'RGB check: unconfirmed, still observing',
}


def receipt(decision):
    """Receipt text for ``own_jobs[*].status``; an unconfirmed decision never
    yields an outcome text."""
    confirmed = decision['status'] == 'confirmed'
    outcome = decision['outcome'] if confirmed else UNCONFIRMED
    return {'status': RECEIPT_TEXT[outcome], 'rgb_outcome': outcome, 'confirmed': confirmed,
            'confidence': decision['confidence'] if confirmed else 0.,
            'evidence_images': decision['evidence_images'] if confirmed else []}


__all__ = ['SCHEMA', 'OUTCOMES', 'UNCONFIRMED', 'TOP_PROFILE', 'CONFIDENCE', 'RECEIPT_TEXT', 'COMMIT_CONFIDENCE',
           'CADENCE_S', 'DEADLINE_S', 'floor_to_pixel', 'sightings', 'point_state', 'ring_unchanged', 'own_near',
           'issued_arm', 'gripper_open', 'releases', 'job_spec', 'slot_target', 'landing_target', 'observe',
           'decide', 'JobTracker', 'receipt']
