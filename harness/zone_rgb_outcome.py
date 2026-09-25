"""RGB-only job outcome check for the zone benchmark (candidate replacement for audit leak L4).

The zone runner currently tells robots whether a job finished from the TEACHER's
ground-truth checks (lift height, drop height, blocked path; see
``experiments/2026-09-25-zone-comm-boundary-audit`` L4). This module decides the
outcome of one robot's (or one team's) job from allowed robot inputs only:

* the fixed TOP RGB JPEGs before the job (at assignment) and after it, with the
  authored TOP calibration in the static map,
* optionally the robot's own RGB JPEG with its own *commanded* arm pulses,
* the static map (zones) and the job's own target area (zone slot or landing
  area, part of the robot's own issued job),
* the job spec the robot itself issued: item label, kind, and the item's
  RGB-estimated source position (``box_labels[*].floor_xy_m``).

It never reads simulator poses, contacts, measured joints, teacher phases or
injection flags. Issued commands are used only to say *what to look for*
(kind, source, target); they are never evidence of success.

Outcomes
    ``delivered``        an item of the job's kind is newly seen inside the job's
                         target area (or, without a target area, the zone's
                         count of that kind rose) and the item is not seen at
                         the source.
    ``still_at_source``  the item is still seen at (or near) its source; the
                         grasp failed or never happened.
    ``seen_elsewhere``   the source is visibly empty and the kind is newly seen
                         somewhere that is not the target (dropped in transit,
                         wrong place, or carried).
    ``not_seen``         none of the above can be shown (occluded or lost).

``confidence`` is a fixed heuristic score per rule (not a calibrated
probability). ``evidence`` names the images and the detections behind the
decision. Thresholds were chosen on the dev split only
(``experiments/2026-09-25-zone-rgb-outcome/split.json``).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np

from harness import zone_color_boxes as _zcb
from harness import zone_perception as _zp

try:   # merged later from claude/zone-cargo-perception (PR #168); imported, never copied
    from harness import zone_cargo_perception as _zcp
except ImportError:   # pragma: no cover - depends on the branch
    _zcp = None

SCHEMA = 'ugrp.zone_rgb_outcome.v1'
OUTCOMES = ('delivered', 'still_at_source', 'seen_elsewhere', 'not_seen')
BOX_KINDS = _zcb.KINDS
CARGO_KINDS = ('can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')
PROVENANCE = ('TOP RGB before/after + authored TOP calibration + static map/target area + own job spec'
              ' (+ own RGB with own commanded arm pulses); no simulator state, contact or teacher result')

# --- thresholds (dev split only) -------------------------------------------
TOP_PROFILE = _zcb.TOP_PROFILE_ZONE
# Same item: a detection within this distance of the labelled source.
SOURCE_TRACK_M = _zp.TRACK_RADIUS_M
# Pushed by a failed grasp: still counts as "at source" out to this distance.
SOURCE_NEAR_M = {'box': .12, 'can': .12, 'tile': .12, 'heavy_crate': .16, 'long_beam': .22, 'tri_frame': .22}
# A new sighting must be this far from every before-sighting of the same kind.
NEW_SIGHTING_M = .06
# Target area tolerance (placement error around a slot / landing area).
TARGET_TOL_M = .06
# Occlusion check: ring of floor around a point, before vs after.
RING_INNER_M, RING_OUTER_M = .045, .11
PIXEL_DIFF = 28          # max-channel |after - before| counted as changed
RING_CHANGED_MAX = .30   # more changed ring pixels than this: something covers the point
IMAGE_MARGIN_PX = 3
RING_MIN_INSIDE = .5
# Own RGB: a same-kind item this close to the robot base (near fit) or clipped
# at the image border counts as "within reach".
OWN_REACH_M = .50

CONFIDENCE = {
    'delivered_source_empty': .95,
    # Colour cannot tell two boxes of one kind apart: with the source hidden, a
    # same-kind box a peer put in the own area looks like a delivery (dev
    # synthetic case). Below COMMIT_CONFIDENCE: re-check until the source shows.
    'delivered_source_occluded': .50,
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


# --------------------------------------------------------------------------- detection

def sightings(tops: Mapping[str, bytes], static_map: Mapping[str, Any], kinds: Sequence[str], *,
              profile: str = TOP_PROFILE) -> list[dict[str, Any]]:
    """Item sightings of the given kinds from all TOPs (floor xy), one row per item.

    Colour boxes use ``zone_color_boxes.detect_top`` (profile as given; the
    baseline profile is exactly ``zone_perception.detect_boxes``). Catalogue
    kinds use ``zone_cargo_perception.detect_all_cargo`` when that module is
    available; otherwise asking for them raises.
    """
    box_kinds = [k for k in kinds if k in BOX_KINDS]
    cargo_kinds = [k for k in kinds if k not in BOX_KINDS]
    rows = []
    if box_kinds:
        if profile == _zcb.TOP_PROFILE_BASELINE:
            rows += [dict(r) for r in _zp.detect_all(tops, {**static_map, 'box_kinds': box_kinds})]
        else:
            raw = []
            for camera in static_map['top_cameras']:
                for r in _zcb.detect_top(tops[camera['name']], camera, box_kinds, profile=profile):
                    raw.append((math.hypot(r['pixel'][0]-.5, r['pixel'][1]-.5), r))
            raw.sort(key=lambda t: t[0])
            for _, r in raw:   # same merge rule as zone_perception.detect_all
                if not any(k['kind'] == r['kind'] and math.dist(k['floor_xy_m'], r['floor_xy_m']) < .05 for k in rows):
                    rows.append(dict(r))
    if cargo_kinds:
        if _zcp is None:
            raise RuntimeError('catalogue kinds need harness.zone_cargo_perception (claude/zone-cargo-perception)')
        found = _zcp.detect_all_cargo(tops, static_map, [k for k in cargo_kinds if k in _zcp.CARGO_KINDS],
                                      include_boxes=False)
        rows += [{'kind': r['kind'], 'floor_xy_m': list(r['floor_xy_m']), 'camera': r.get('camera'),
                  'confidence': r.get('confidence')} for r in found['items'] if r['kind'] in cargo_kinds]
    return rows


def _decode(jpeg):
    if isinstance(jpeg, np.ndarray):
        return jpeg
    return _zp._decode(jpeg)


def point_visibility(before_tops, after_tops, static_map, xy, *, decoded=None):
    """Is the floor around ``xy`` visible in some TOP now? Compares a floor ring
    around the point before vs after: a robot, arm or shadow over the point
    changes the ring; an item removed from the centre does not."""
    decoded = {} if decoded is None else decoded
    views = []
    for camera in static_map['top_cameras']:
        name = camera['name']
        for key, tops in (('b', before_tops), ('a', after_tops)):
            if (key, name) not in decoded:
                decoded[(key, name)] = _decode(tops[name])
        before, after = decoded[('b', name)], decoded[('a', name)]
        u, v, scale = floor_to_pixel(xy[0], xy[1], camera, after.shape)
        h, w = after.shape[:2]
        r_out = RING_OUTER_M*scale
        x0, x1 = int(u-r_out)-1, int(u+r_out)+2
        y0, y1 = int(v-r_out)-1, int(v+r_out)+2
        yy, xx = np.mgrid[y0:y1, x0:x1]
        d = np.hypot(xx-u, yy-v)
        full = (d >= RING_INNER_M*scale) & (d <= r_out)
        # Near an image edge only part of the ring is in view (the TOP overlap
        # strips): use the part inside, if it is at least RING_MIN_INSIDE of it.
        inside = ((xx >= IMAGE_MARGIN_PX) & (xx <= w-1-IMAGE_MARGIN_PX)
                  & (yy >= IMAGE_MARGIN_PX) & (yy <= h-1-IMAGE_MARGIN_PX))
        ring = full & inside
        if ring.sum() < RING_MIN_INSIDE*full.sum():
            continue
        cx0, cy0 = max(x0, 0), max(y0, 0)
        cx1, cy1 = min(x1, w), min(y1, h)
        ring = ring[cy0-y0:cy1-y0, cx0-x0:cx1-x0]
        diff = np.abs(after[cy0:cy1, cx0:cx1].astype(np.int16)
                      - before[cy0:cy1, cx0:cx1].astype(np.int16)).max(axis=2)
        changed = float((diff[ring] > PIXEL_DIFF).mean())
        views.append({'camera': name, 'pixel': [round(u, 1), round(v, 1)], 'ring_changed': round(changed, 3),
                      'clear': changed <= RING_CHANGED_MAX})
    return {'visible': any(v['clear'] for v in views), 'views': views}


def own_near(own_rgb, own_servo_pose, kind):
    """Own RGB cue: an item of ``kind`` within reach of the robot (near floor fit
    within OWN_REACH_M of the base, or the colour clipped at the image border)."""
    if own_rgb is None or own_servo_pose is None or kind not in BOX_KINDS:
        return None
    res = _zcb.detect_own(own_rgb, own_servo_pose, [kind], profile=_zcb.OWN_PROFILE_ZONE)
    near = [d for d in res['detections'] if d['range_class'] == 'near'
            and math.hypot(*d['estimated_box_center_base_m'][:2]) <= OWN_REACH_M]
    return {'within_reach': bool(near) or kind in res['clipped_kinds'],
            'near_detections': [[round(v, 3) for v in d['estimated_box_center_base_m'][:2]] for d in near],
            'clipped_at_border': kind in res['clipped_kinds']}


# --------------------------------------------------------------------------- jobs

def job_spec(*, robot_ids, item, kind, source_xy_m, zone, target=None):
    """What the robot(s) issued. ``target``: {'center_m', 'half_extents_m'} of the
    job's own slot or landing area, or None (zone only)."""
    robot_ids = [robot_ids] if isinstance(robot_ids, str) else list(robot_ids)
    return {'robot_ids': robot_ids, 'item': item, 'kind': kind, 'source_xy_m': [float(v) for v in source_xy_m],
            'zone': zone, 'target': None if target is None else
            {'center_m': [float(v) for v in target['center_m']],
             'half_extents_m': [float(v) for v in target['half_extents_m']]}}


def slot_target(static_map, slot_id):
    for slots in static_map['zone_slots'].values():
        for s in slots:
            if s['slot_id'] == slot_id:
                return {'center_m': s['center_m'], 'half_extents_m': s['half_extents_m']}
    raise KeyError(slot_id)


def landing_target(area):
    """A ``zone_goal_v2.landing_layout`` area (already includes its placement tolerance)."""
    return {'center_m': area['landing_center_m'], 'half_extents_m': area['landing_half_extents_m']}


def job_outcome(job, before_tops, after_tops, static_map, *, own_rgb=None, own_servo_pose=None,
                before_names=None, after_names=None, own_name=None, profile=TOP_PROFILE,
                before_rows=None):
    """Decide one job's outcome from RGB. Returns a JSON-able dict."""
    kind, src = job['kind'], job['source_xy_m']
    target = job['target']
    zone_region = static_map['regions']['zone_'+job['zone']]
    before = before_rows if before_rows is not None else sightings(before_tops, static_map, [kind], profile=profile)
    after = sightings(after_tops, static_map, [kind], profile=profile)
    before = [r for r in before if r['kind'] == kind]
    after = [r for r in after if r['kind'] == kind]
    near_m = _near_kind(kind)

    def at_source(rows, radius):
        return [r for r in rows if math.dist(r['floor_xy_m'], src) <= radius]

    def in_target(rows):
        if target is None:
            return []
        return [r for r in rows if inside_rect(r['floor_xy_m'], target['center_m'], target['half_extents_m'],
                                              TARGET_TOL_M)]

    def in_zone(rows):
        return [r for r in rows if inside_rect(r['floor_xy_m'], zone_region['center_m'], zone_region['half_extents_m'])]

    def is_new(r):
        return all(math.dist(r['floor_xy_m'], b['floor_xy_m']) > NEW_SIGHTING_M for b in before)

    decoded = {}
    src_tracked, src_near = at_source(after, SOURCE_TRACK_M), at_source(after, near_m)
    others_near_before = max(0, len(at_source(before, near_m)) - 1)
    tgt_before, tgt_after = in_target(before), in_target(after)
    tgt_new = [r for r in tgt_after if is_new(r)] if len(tgt_after) > len(tgt_before) else []
    zone_gain = len(in_zone(after)) - len(in_zone(before))
    elsewhere = [r for r in after if is_new(r) and r not in tgt_after
                 and math.dist(r['floor_xy_m'], src) > near_m]
    src_vis = point_visibility(before_tops, after_tops, static_map, src, decoded=decoded)
    tgt_vis = (point_visibility(before_tops, after_tops, static_map, target['center_m'], decoded=decoded)
               if target is not None else None)
    in_own_zone = [r for r in elsewhere if r in in_zone(elsewhere)]
    if in_own_zone and (tgt_vis is None or not tgt_vis['visible']):
        # A new same-kind item in the job's zone while the own area is hidden
        # may be this item or a peer's delivery: not evidence either way.
        elsewhere = [r for r in elsewhere if r not in in_own_zone]
        flags_zone = ['new_same_kind_in_zone_while_target_hidden']
    else:
        flags_zone = ['new_same_kind_in_zone_outside_target'] if in_own_zone else []
    own = own_near(own_rgb, own_servo_pose, kind)
    source_empty_seen = src_vis['visible'] and not src_near

    flags = list(flags_zone)
    if others_near_before:
        flags.append('another_same_kind_item_near_source_before')
    if target is not None and tgt_before:
        flags.append('target_area_held_same_kind_before')
    if src_near and not src_tracked:
        flags.append('source_item_moved_but_near')

    if src_near:
        if tgt_new:
            outcome, rule = 'still_at_source', 'still_at_source_conflict'
            flags.append('new_item_in_target_but_source_still_seen')
        elif src_tracked and not others_near_before:
            outcome, rule = 'still_at_source', 'still_at_source_tracked'
        else:
            outcome, rule = 'still_at_source', 'still_at_source_near'
    elif target is not None and tgt_new:
        outcome, rule = 'delivered', ('delivered_source_empty' if src_vis['visible'] else 'delivered_source_occluded')
    elif target is None and zone_gain > 0 and source_empty_seen:
        outcome, rule = 'delivered', 'delivered_zone_count'
    elif source_empty_seen and elsewhere:
        outcome, rule = 'seen_elsewhere', 'seen_elsewhere'
    elif not src_vis['visible'] and own and own['within_reach']:
        # TOP cannot see the source (typically the robot's own body over it);
        # the robot's own camera sees the kind right in front of it.
        outcome, rule = 'still_at_source', 'still_at_source_own_rgb'
    elif source_empty_seen and own and own['within_reach']:
        # Source visibly empty, nothing new on TOP, but the kind is right in
        # front of the robot: dropped where TOP cannot see (under/beside it).
        outcome, rule = 'seen_elsewhere', 'seen_elsewhere_own_rgb'
    else:
        outcome, rule = 'not_seen', 'not_seen'
        flags.append('source_occluded' if not src_vis['visible'] else 'source_empty_item_not_found')
        if tgt_vis is not None and not tgt_vis['visible']:
            flags.append('target_occluded')

    images = sorted(set((before_names or {}).values()) | set((after_names or {}).values())
                    | ({own_name} if own_name and own is not None else set()))
    return {
        'schema': SCHEMA, 'outcome': outcome, 'confidence': CONFIDENCE[rule], 'rule': rule, 'flags': flags,
        'job': {k: job[k] for k in ('robot_ids', 'item', 'kind', 'zone', 'source_xy_m', 'target')},
        'evidence': {
            'images': images, 'top_profile': profile,
            'source': {'tracked': [r['floor_xy_m'] for r in src_tracked], 'near': [r['floor_xy_m'] for r in src_near],
                       'visibility': src_vis},
            'target': None if target is None else {'before': [r['floor_xy_m'] for r in tgt_before],
                                                   'after': [r['floor_xy_m'] for r in tgt_after],
                                                   'new': [r['floor_xy_m'] for r in tgt_new], 'visibility': tgt_vis},
            'zone_count_change': zone_gain,
            'new_elsewhere': [r['floor_xy_m'] for r in elsewhere],
            'own_rgb': own,
        },
        'provenance': PROVENANCE,
    }


# --------------------------------------------------------------------------- runner-facing helpers

RECEIPT_TEXT = {
    'delivered': 'RGB check: item seen in the target area, source empty',
    'still_at_source': 'RGB check: item still seen at its source',
    'seen_elsewhere': 'RGB check: item seen away from source and target',
    'not_seen': 'RGB check: item not seen (occluded or lost)',
}
COMMIT_CONFIDENCE = .6


def receipt(result):
    """Text that would replace the teacher receipt in ``own_jobs[*].status``."""
    return {'status': RECEIPT_TEXT[result['outcome']], 'rgb_outcome': result['outcome'],
            'confidence': result['confidence'], 'evidence_images': result['evidence']['images']}


def commit(results, *, min_confidence=COMMIT_CONFIDENCE):
    """Re-check policy: the first result (in time order) whose outcome is not
    ``not_seen`` and whose confidence reaches ``min_confidence``; otherwise the
    last result (``not_seen`` or a low-confidence outcome)."""
    for i, r in enumerate(results):
        if r['outcome'] != 'not_seen' and r['confidence'] >= min_confidence:
            return i, r
    return len(results)-1, results[-1]


__all__ = ['SCHEMA', 'OUTCOMES', 'TOP_PROFILE', 'CONFIDENCE', 'RECEIPT_TEXT', 'COMMIT_CONFIDENCE',
           'floor_to_pixel', 'sightings', 'point_visibility', 'own_near', 'job_spec', 'slot_target',
           'landing_target', 'job_outcome', 'receipt', 'commit']
