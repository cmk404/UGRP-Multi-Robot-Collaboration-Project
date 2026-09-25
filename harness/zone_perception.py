"""TOP-RGB box detection for the zone benchmark (controller-side perception).

Everything here is computed from the fixed TOP JPEGs and the authored
camera calibration. No simulator pose is read. Box labels are fixed from the
first TOP frames by a stated convention: per colour, west to east, then south
to north ("red-1" is the west-most red box). Later frames re-identify pickup
boxes only by staying within a small radius of their labelled floor position.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

# HSV ranges (OpenCV H in 0..179). Red wraps around 0.
HSV_RANGES = {
    'cyan': [((80, 110, 60), (100, 255, 255))],
    'red': [((0, 150, 70), (6, 255, 255)), ((172, 150, 70), (179, 255, 255))],
    'green': [((45, 120, 50), (75, 255, 255))],
    'yellow': [((22, 150, 120), (32, 255, 255))],
}
# A box top is about 3.4 x 4.0 cm; at the TOP scale (~277 px/m) that is ~9x11 px.
MIN_AREA_PX, MAX_AREA_PX = 30, 260
MIN_FILL = .55
BOX_TOP_Z_M = .032
TRACK_RADIUS_M = .06


def _decode(jpeg):
    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('invalid TOP JPEG')
    return frame


def pixel_to_floor(u, v, camera, shape, *, height=BOX_TOP_Z_M):
    """Authored downward camera: image x -> world +x, image y -> world -y."""
    if camera['quaternion_wxyz'] != [1, 0, 0, 0]:
        raise ValueError('unsupported TOP orientation')
    h, w = shape[:2]
    cx, cy, cz = camera['position_m']
    scale = h / (2*(cz-height)*math.tan(math.radians(camera['fov_y_deg'])/2))
    return cx + (u-(w-1)/2)/scale, cy - (v-(h-1)/2)/scale


def detect_boxes(jpeg, camera, kinds):
    """Compact colour blobs of box size; returns floor positions from RGB only."""
    frame = _decode(jpeg)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    found = []
    for kind in kinds:
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for low, high in HSV_RANGES[kind]:
            mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
        for i in range(1, count):
            x, y, w, h, area = (int(v) for v in stats[i])
            if not MIN_AREA_PX <= area <= MAX_AREA_PX or area < MIN_FILL*w*h or max(w, h) > 2.2*min(w, h):
                continue
            u, v = (float(c) for c in centers[i])
            fx, fy = pixel_to_floor(u, v, camera, frame.shape)
            found.append({'kind': kind, 'pixel': [round(u/frame.shape[1], 4), round(v/frame.shape[0], 4)],
                          'floor_xy_m': [round(fx, 3), round(fy, 3)], 'area_px': area,
                          'camera': camera['name']})
    return found


def detect_all(tops, static_map):
    """All TOPs; a box seen twice in an overlap keeps the view nearer its centre."""
    kinds = static_map['box_kinds']
    rows = []
    for camera in static_map['top_cameras']:
        for row in detect_boxes(tops[camera['name']], camera, kinds):
            # Same-row views share the image y, so this orders the retired zone_open's
            # east-west overlap exactly as the former |x - .5| key did.
            row['_centre_offset'] = math.hypot(row['pixel'][0]-.5, row['pixel'][1]-.5)
            rows.append(row)
    rows.sort(key=lambda r: r['_centre_offset'])
    kept = []
    for row in rows:
        if any(k['kind'] == row['kind'] and math.dist(k['floor_xy_m'], row['floor_xy_m']) < .05 for k in kept):
            continue
        kept.append(row)
    for row in kept:
        row.pop('_centre_offset')
    return kept


def _inside(xy, region, margin=0.):
    (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
    return abs(xy[0]-cx) <= hx+margin and abs(xy[1]-cy) <= hy+margin


def label_pickup(detections, static_map):
    """Initial labels: per colour, west to east then south to north."""
    pickup = static_map['regions']['pickup']
    boxes = [d for d in detections if _inside(d['floor_xy_m'], pickup, .05)]
    labels = {}
    for kind in static_map['box_kinds']:
        same = sorted((d for d in boxes if d['kind'] == kind),
                      key=lambda d: (round(d['floor_xy_m'][0], 1), d['floor_xy_m'][1]))
        for i, d in enumerate(same, 1):
            labels[f'{kind}-{i}'] = {'kind': kind, 'floor_xy_m': d['floor_xy_m']}
    return labels


def observe(tops, static_map, labels):
    """Current RGB view: which labelled boxes are still at their pickup spot,
    and how many boxes of each colour are inside each zone."""
    detections = detect_all(tops, static_map)
    remaining = {}
    for label, item in labels.items():
        if any(d['kind'] == item['kind'] and math.dist(d['floor_xy_m'], item['floor_xy_m']) <= TRACK_RADIUS_M
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
    return {'source': 'TOP RGB colour blobs and authored camera calibration; not simulator state',
            'pickup_boxes_still_visible': sorted(remaining), 'zone_counts_seen': zones,
            'detections': len(detections)}
