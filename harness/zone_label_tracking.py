"""Label tracking for items moved off their pickup spot (zone teacher fix B5, 2026-09-26).

Robot-input side of the PR #169 v2 pipeline: TOP RGB detections and the
static map only, never simulator state. Used only by the perception profile
``top_cargo_v2_track`` (``harness.zone_perception_v2``); ``top_cargo_v2`` and
``top_cargo_v1`` are unchanged.

B5 root cause: ``view_from_detections`` keeps a label only while a detection of
its kind lies within the dedupe radius of the label's FIRST position, so an
item set down elsewhere (``hold_lower`` mid-route, a drop) vanishes from the
claimable list for good. Fix: a missing label follows a detection of its kind
that lies outside every goal zone and away from every other label of that
kind, once the same detection was seen at the same place (``STILL_M``) in two
consecutive observations; the label's RGB position, yaw and handles are then
re-estimated from that detection. Limitations: consecutive observations can be
close in SIM time (two claim attempts of one round), so a team item paused
mid-carry can be re-labelled; the teacher then ends such a claim with
``item_taken``.
"""
from __future__ import annotations

import math

STILL_M = .03
PENDING_KEY = '_moved_candidate_xy'   # internal; public_labels never exports it


def _in_zone(xy, static_map):
    for rid, region in static_map['regions'].items():
        if not rid.startswith('zone_'):
            continue
        (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
        if abs(xy[0] - cx) <= hx and abs(xy[1] - cy) <= hy:
            return True
    return False


def track_moved_labels(labels, detections, static_map, remaining, *, radius, handles, box_kinds):
    """Mutates ``labels`` (and ``remaining``) for missing labels that moved; returns {label: record}.

    radius(kind) -> dedupe radius; handles(kind, xy, yaw) -> handle dict for cargo labels.
    """
    moved, used = {}, set()
    for label in sorted(labels):
        item = labels[label]
        if label in remaining:
            item.pop(PENDING_KEY, None)
            continue
        kind = item['kind']
        others = [o for o in labels if o != label and labels[o]['kind'] == kind]
        cands = [i for i, d in enumerate(detections)
                 if i not in used and d['kind'] == kind and not _in_zone(d['floor_xy_m'], static_map)
                 and not any(math.dist(d['floor_xy_m'], labels[o]['floor_xy_m']) <= radius(kind) for o in others)]
        if not cands:
            item.pop(PENDING_KEY, None)
            continue
        i = min(cands, key=lambda k: math.dist(detections[k]['floor_xy_m'], item['floor_xy_m']))
        d = detections[i]
        used.add(i)
        prev = item.get(PENDING_KEY)
        if prev is None or math.dist(prev, d['floor_xy_m']) > STILL_M:
            item[PENDING_KEY] = list(d['floor_xy_m'])
            continue
        old = list(item['floor_xy_m'])
        item.pop(PENDING_KEY, None)
        item['floor_xy_m'] = list(d['floor_xy_m'])
        if kind not in box_kinds:
            item['yaw_rad'] = d['yaw_rad']
            item['handles'] = handles(kind, d['floor_xy_m'], d['yaw_rad'])
        remaining[label] = item
        moved[label] = {'kind': kind, 'from_rgb_xy_m': [round(v, 3) for v in old],
                        'to_rgb_xy_m': [round(v, 3) for v in d['floor_xy_m']]}
    return moved


__all__ = ['STILL_M', 'PENDING_KEY', 'track_moved_labels']
