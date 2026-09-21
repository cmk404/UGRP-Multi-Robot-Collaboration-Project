"""Conservative fixed-TOP RGB identity, without world/segmentation inputs.

IDs are grounded ONCE by an explicit visual naming convention: within a kind,
left column first, bottom to top within each column. Subsequent frames never
re-sort identities. Ambiguous associations latch a stop; an unobserved crossing
cannot be resolved by silently choosing the nearest object.
"""
from __future__ import annotations

import copy
import hashlib
import itertools
import math

import cv2
import numpy as np

from harness.multi_object_plan import validate_mission


def detections(jpeg):
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('valid TOP RGB JPEG required')
    h, w = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    found = []
    for kind, low, high in [('box', (80, 70, 35), (100, 255, 255)),
                            ('beam', (3, 105, 45), (35, 255, 255))]:
        mask = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        for i in range(1, count):
            x, y, bw, bh, area = map(int, stats[i])
            if x <= 1 or y <= 1 or x+bw >= w-1 or y+bh >= h-1:
                continue
            if kind == 'box':
                if not (40 <= area <= 1800 and .3 <= bw/bh <= 3):
                    continue
            else:
                points = np.column_stack(np.where(labels == i)[::-1]).astype(np.float32)
                (_, _), sides, _ = cv2.minAreaRect(points)
                length, width = max(sides), min(sides)
                if not (width >= 3 and length/width >= 3.5 and 65 <= length <= 180 and width <= 25):
                    continue
            found.append({'kind': kind, 'center': (centers[i]/[w,h]).tolist(),
                          'bounds': [x/w, y/h, bw/w, bh/h], 'area_px': area})
    return found


def visual_order(rows):
    """Authored naming rule, not an initial world-position lookup."""
    columns = []
    for row in sorted(rows, key=lambda r:r['center'][0]):
        if not columns or row['center'][0]-columns[-1][0]['center'][0] > .025:
            columns.append([])
        columns[-1].append(row)
    return [r for col in columns for r in sorted(col, key=lambda r:-r['center'][1])]


class CargoTracker:
    def __init__(self, mission, *, max_gap_s=.8, max_speed_image_s=.10, slack=.015):
        self.objects = validate_mission(mission)['objects']
        self.max_gap, self.speed, self.slack = max_gap_s, max_speed_image_s, slack
        if not all(math.isfinite(x) and x > 0 for x in (max_gap_s,max_speed_image_s,slack)):
            raise ValueError('positive finite tracking limits required')
        self.tracks, self.last_time, self.clock = {}, None, -1.
        self.last_frame = -1
        self.latched = None
        self.latest = None

    def observe(self, jpeg, *, frame_id, now_s):
        if (type(frame_id) is not int or frame_id <= self.last_frame or
                type(now_s) not in (int,float) or not math.isfinite(now_s) or now_s < self.clock):
            raise ValueError('fresh frame and monotonic timestamp required')
        self.last_frame, self.clock = frame_id, now_s
        result = {'frame_id': frame_id, 'observed_at_s': now_s,
                  'top_sha256': hashlib.sha256(jpeg).hexdigest(), 'valid': False,
                  'reason': self.latched, 'tracks': {}}
        rows = detections(jpeg)
        if self.latched:
            self.latest = result
            return copy.deepcopy(result)
        if self.last_time is not None and now_s-self.last_time > self.max_gap:
            self.latched = result['reason'] = 'identity_gap_requires_new_mission_binding'
        updates = {}
        for kind in ('beam','box'):
            ids = [o['object_id'] for o in self.objects if o['kind'] == kind]
            candidates = [r for r in rows if r['kind'] == kind]
            if len(candidates) != len(ids):
                result['reason'] = result['reason'] or 'visible_inventory_mismatch'
                continue
            if not self.tracks:
                ordered = visual_order(candidates)
            else:
                radius = self.slack+self.speed*(now_s-self.last_time)
                feasible = []
                for perm in itertools.permutations(candidates):
                    if all(math.dist(self.tracks[oid]['center'], r['center']) <= radius and
                           .4 <= r['area_px']/self.tracks[oid]['area_px'] <= 2.5 for oid,r in zip(ids,perm)):
                        feasible.append(perm)
                        if len(feasible)>1: break
                if len(feasible) != 1:
                    self.latched = result['reason'] = 'ambiguous_identity' if feasible else 'identity_motion_unresolved'
                    continue
                ordered = feasible[0]
            updates.update(dict(zip(ids,ordered)))
        if not result['reason']:
            self.tracks, self.last_time = copy.deepcopy(updates), now_s
            result.update(valid=True, reason='rgb_identity_continuous', tracks=updates)
        self.latest = copy.deepcopy(result)
        return copy.deepcopy(result)
