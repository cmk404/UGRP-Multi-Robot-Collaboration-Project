"""Kind-aware TOP-RGB cargo detection for the zone benchmark (profile ``top_cargo_v1``).

Robot input only: the fixed TOP RGB JPEGs and the authored TOP calibration
(``static_map['top_cameras']``), plus the *static* cargo catalogue
(``sim.zone_cargo``: shapes, sizes, grasp roles), which is allowed static
information. No simulator pose, segmentation or contact is read here.

The profile is opt-in. ``harness.zone_perception`` and
``harness.zone_color_boxes`` (profiles ``zone_perception_v1``,
``top_zone_v2``, ``own_*``) are not modified and keep their outputs byte for
byte; boxes in this profile come from ``zone_color_boxes.detect_top`` with the
``top_zone_v2`` profile, minus box blobs that lie on a detected cargo item.

Kinds and cues (thresholds chosen on the dev split only,
``experiments/2026-09-25-zone-cargo-perception``):

* ``can``         violet, small compact disc; no yaw (round).
* ``tile``        magenta, small 60x40 mm rectangle; yaw mod 180 deg.
* ``long_beam``   lime, thin 15:1 bar; collinear fragments are merged (a robot
                  arm can split it); yaw mod 180 deg; centre from the known
                  length when one end is clipped by the image border.
* ``heavy_crate`` pink 140x100 mm body; yaw mod 180 deg from the body's long
                  axis, checked against the two dark end lugs.
* ``tri_frame``   cream open triangle; three side lines are fitted and
                  intersected; yaw mod 120 deg; dark vertex lugs checked.

``confidence`` is a heuristic gate score in [0, 1] (not a calibrated
probability). Positions are floor coordinates of the object origin
(``sim.zone_cargo``: on the floor under the centroid).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np

from harness import zone_color_boxes as _zcb
from harness import zone_perception as _zp

PROFILE = 'top_cargo_v1'
SCHEMA = 'ugrp.zone_cargo_perception.v1'
CARGO_KINDS = ('can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')
BOX_KINDS = _zcb.KINDS
PROVENANCE = 'TOP RGB colour+shape and authored TOP calibration; static cargo catalogue; not simulator state'

# OpenCV HSV (H 0..179), dev split only. Measured on dev renders: zone C paint
# H 141-143 (S 136-170), zone C slots H 142 (S ~80), can H 129-140, tile
# H 160-167 (S >= 148), crate H 166-171 (S 71-157) or washed out, beam top H 34-51,
# yellow box H 22-33 (S >= 140), cream frame H 19-28 (S 46-123).
CARGO_HSV = {
    # Second bands: washed-out north TOP views shift can to H 140-148 and tile
    # to H ~150 at V >= 210; zone C slot squares stay at S 75-94, V 184-201.
    'can': (((122, 110, 60), (139, 255, 255)), ((136, 100, 210), (148, 255, 255))),
    'tile': (((153, 130, 60), (168, 255, 255)), ((149, 105, 205), (158, 255, 255))),
    # The north TOPs see a washed-out crate (H 150-168, S 25-110, V ~255).
    'heavy_crate': (((145, 20, 110), (176, 135, 255)),),
    # Second band: under the washed-out north TOPs the lime bar renders yellow
    # (H 30-37); only the elongated-shape gates separate it from yellow boxes.
    # H >= 29 keeps the cream frame (H <= 28) out; a beam on a frame is dropped.
    'long_beam': (((34, 90, 50), (54, 255, 255)), ((29, 70, 120), (33, 255, 255))),
    # Under the north TOPs the cream frame renders almost white (S 2-45, V >= 225);
    # no floor or paint pixel falls in that band on dev (robots and a washed-out
    # crate do, and are rejected by the triangle-outline gates).
    'tri_frame': (((15, 35, 120), (30, 130, 255)), ((0, 0, 225), (60, 45, 255)), ((0, 0, 235), (179, 40, 255))),
}
COLOUR = {'can': 'violet', 'tile': 'magenta', 'long_beam': 'lime', 'heavy_crate': 'pink', 'tri_frame': 'cream'}
YAW_SYMMETRY_DEG = {'can': None, 'tile': 180., 'long_beam': 180., 'heavy_crate': 180., 'tri_frame': 120., 'box': None}
# Height of the surface the TOP mostly sees (object frame, metres).
TOP_Z_M = {'can': .050, 'tile': .012, 'long_beam': .032, 'heavy_crate': .060, 'tri_frame': .024}

# Geometry mirrored from sim.zone_cargo catalogue v1 (tests check it against
# the catalogue so a catalogue change flags this detector for re-validation).
CAN_RADIUS_M = .019
TILE_DIMS_M = (.060, .040)
BEAM_LENGTH_M = .60
BEAM_WIDTH_M = .040
BEAM_BAND_INNER_M = .252          # lime between the two dark bands spans +-0.252 m
CRATE_BODY_M = (.140, .100)
CRATE_LUG_CENTRE_M = .090
CRATE_LUG_DIMS_M = (.060, .040)
FRAME_VERTEX_RADIUS_M = .20
FRAME_BAR_WIDTH_M = .030
FRAME_LUG_RADIUS_M = .22

# Gates (dev split).
SMALL_AREA_FRAC = {'can': (.35, 3.2), 'tile': (.35, 2.6)}     # of the ideal top-face area
SMALL_MAX_ASPECT = {'can': 1.8, 'tile': 2.8}
SMALL_MIN_FILL = .50
# Washed-out band shared by can and tile (dev): zone C slots are S <= 94 and
# V <= 201, a washed-out crate is S <= 65 (p90); kind then comes from shape.
BRIGHT_VM_HSV = (((136, 70, 210), (158, 255, 255)),)
BRIGHT_TILE_MIN_ASPECT = 1.3
BRIGHT_TILE_MIN_FILL = .86
CRATE_AREA_FRAC = (.30, 1.7)
CRATE_MIN_FILL = .60
CRATE_MAX_ASPECT = 2.4
# Fully washed-out crate (S ~2, V 255): a white body is accepted only with dark
# lugs at both ends of its long axis and a crate-like aspect (dev: no floor or
# paint pixel is this white; robots and the frame are).
WHITE_HSV = (((0, 0, 235), (179, 40, 255)),)
CRATE_WHITE_MIN_LUG = .5
CRATE_WHITE_ASPECT = (1.15, 2.0)
BEAM_MIN_FRAGMENT_PX = 12
BEAM_SEED_MIN_LEN_M = .10
BEAM_MIN_LEN_M = .12
BEAM_LINE_TOL_PX = 5.
BEAM_MAX_GAP_M = .30
BEAM_FULL_LEN_M = .44              # a span this long is both ends (caps may be lost)
BEAM_END_OFFSET_M = .276           # visible end -> centre when one end is clipped
FRAME_MIN_AREA_PX = 250
FRAME_SIDE_M = (.24, .52)
FRAME_FILL_OF_TRIANGLE = (.12, .80)
DARK_V_MAX = 55
DARK_REL = .72
BORDER_PX = 3
DEDUPE_M = {'box': .05, 'can': .05, 'tile': .05, 'heavy_crate': .10, 'long_beam': .12, 'tri_frame': .15}


# ------------------------------------------------------------------ helpers

def _scale(camera, shape, z):
    """Pixels per metre on a horizontal plane at height z."""
    h = shape[0]
    return h/(2*(camera['position_m'][2]-z)*math.tan(math.radians(camera['fov_y_deg'])/2))


def _floor(u, v, camera, shape, z):
    x, y = _zp.pixel_to_floor(u, v, camera, shape, height=z)
    return [round(float(x), 4), round(float(y), 4)]


def _img_angle_to_yaw(dx, dy):
    """Image direction (du, dv) -> world yaw (image y points to world -y)."""
    return math.atan2(-dy, dx)


def _wrap_sym(yaw, sym_deg):
    if yaw is None or not sym_deg:
        return None if yaw is None else math.atan2(math.sin(yaw), math.cos(yaw))
    period = math.radians(sym_deg)
    return (yaw + period/2) % period - period/2


def _mask(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask |= cv2.inRange(hsv, np.asarray(low, np.uint8), np.asarray(high, np.uint8))
    return mask


def _touches_border(points, shape, pad=BORDER_PX):
    h, w = shape[:2]
    return bool(np.any(points[:, 0] <= pad) or np.any(points[:, 1] <= pad) or
                np.any(points[:, 0] >= w-1-pad) or np.any(points[:, 1] >= h-1-pad))


def _dark_threshold(v_channel, component, centre, radius_px):
    """Dark = V <= max(DARK_V_MAX, DARK_REL * local background V).

    Under the washed-out north TOPs the floor is V ~140-175 and the black lugs
    render V ~60-80, so an absolute threshold alone misses them."""
    h, w = v_channel.shape
    r = int(max(4, radius_px))
    x0, x1 = max(0, int(centre[0])-r), min(w, int(centre[0])+r+1)
    y0, y1 = max(0, int(centre[1])-r), min(h, int(centre[1])+r+1)
    patch = v_channel[y0:y1, x0:x1]
    keep = component[y0:y1, x0:x1] == 0
    bg = float(np.median(patch[keep])) if keep.any() else 0.
    return max(DARK_V_MAX, DARK_REL*bg), bg


def _dark_fraction(v_channel, centre, axis, half_len, half_w, threshold=DARK_V_MAX):
    """Fraction of dark pixels in an oriented window (image coordinates)."""
    h, w = v_channel.shape
    ax = np.asarray(axis, float)
    nx = np.array([-ax[1], ax[0]])
    samples = []
    for a in np.linspace(-half_len, half_len, 7):
        for b in np.linspace(-half_w, half_w, 5):
            p = np.asarray(centre) + a*ax + b*nx
            x, y = int(round(p[0])), int(round(p[1]))
            if 0 <= x < w and 0 <= y < h:
                samples.append(v_channel[y, x] <= threshold)
    return float(np.mean(samples)) if samples else None


def _row(kind, camera, shape, u, v, yaw, confidence, footprint_px, clipped, evidence):
    z = TOP_Z_M[kind]
    return {'kind': kind, 'colour': COLOUR[kind], 'floor_xy_m': _floor(u, v, camera, shape, z),
            'yaw_rad': None if yaw is None else round(float(_wrap_sym(yaw, YAW_SYMMETRY_DEG[kind])), 4),
            'yaw_symmetry_deg': YAW_SYMMETRY_DEG[kind], 'confidence': round(float(confidence), 3),
            'camera': camera['name'], 'pixel': [round(u/shape[1], 4), round(v/shape[0], 4)],
            'clipped': bool(clipped), 'footprint_px': [[int(round(a)), int(round(b))] for a, b in footprint_px],
            'evidence': evidence}


# ------------------------------------------------------------------ per kind

def _small_ideal(kind, camera, shape):
    s = _scale(camera, shape, TOP_Z_M[kind])
    return (math.pi*CAN_RADIUS_M**2 if kind == 'can' else TILE_DIMS_M[0]*TILE_DIMS_M[1])*s*s


def _small_from_mask(kind, mask, camera, shape, source):
    """Small blobs of one mask. ``kind=None`` classifies each blob by shape
    (round -> can, rectangular -> tile); used for the washed-out band where
    the violet and magenta hues meet."""
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    out = []
    for i in range(1, count):
        area = int(stats[i][4])
        if area < 20:
            continue
        pts = np.column_stack(np.nonzero(labels == i)[::-1]).astype(np.float32)
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
        rw, rh = rw+1., rh+1.
        aspect = max(rw, rh)/min(rw, rh)
        fill = area/(rw*rh)
        k = kind
        if k is None:
            k = 'tile' if (aspect >= BRIGHT_TILE_MIN_ASPECT or fill >= BRIGHT_TILE_MIN_FILL) else 'can'
        ideal = _small_ideal(k, camera, shape)
        lo, hi = SMALL_AREA_FRAC[k]
        if not lo*ideal <= area <= hi*ideal:
            continue
        if aspect > SMALL_MAX_ASPECT[k] or fill < SMALL_MIN_FILL:
            continue
        clipped = _touches_border(pts, shape)
        yaw = None
        if k == 'tile':
            a = math.radians(ang if rw >= rh else ang+90)
            yaw = _img_angle_to_yaw(math.cos(a), math.sin(a))
        u, v = (float(c) for c in centers[i])
        conf = min(1., area/ideal) * (.6 if clipped else 1.)
        if k == 'tile' and aspect < 1.15:
            conf *= .7          # nearly square blob: yaw is unreliable
        if kind is None:
            conf *= .7          # kind from shape only
        box = cv2.boxPoints(((cx, cy), (rw, rh), ang))
        out.append(_row(k, camera, shape, u, v, yaw, conf, box, clipped,
                        {'area_px': area, 'ideal_area_px': round(ideal, 1), 'aspect': round(aspect, 2),
                         'rect_fill': round(fill, 2), 'kind_source': source}))
    return out


def _small(kind, hsv, camera, shape):
    mask = cv2.morphologyEx(_mask(hsv, CARGO_HSV[kind]), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return _small_from_mask(kind, mask, camera, shape, 'colour')


def _small_bright(hsv, camera, shape, found):
    """Washed-out violet/magenta blobs that neither colour band claimed."""
    mask = cv2.morphologyEx(_mask(hsv, BRIGHT_VM_HSV), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    out = []
    for row in _small_from_mask(None, mask, camera, shape, 'washed_out_shape'):
        u, v = row['pixel'][0]*shape[1], row['pixel'][1]*shape[0]
        if any(math.hypot(u-f['pixel'][0]*shape[1], v-f['pixel'][1]*shape[0]) < 8 for f in found):
            continue
        out.append(row)
    return out


def _crate(hsv, camera, shape):
    s = _scale(camera, shape, TOP_Z_M['heavy_crate'])
    ideal = CRATE_BODY_M[0]*CRATE_BODY_M[1]*s*s
    pink = _mask(hsv, CARGO_HSV['heavy_crate'])
    mask = cv2.morphologyEx(pink | _mask(hsv, WHITE_HSV), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    v_channel = hsv[..., 2]
    out = []
    for i in range(1, count):
        area = int(stats[i][4])
        if not CRATE_AREA_FRAC[0]*ideal <= area <= CRATE_AREA_FRAC[1]*ideal:
            continue
        pink_fraction = float((pink[labels == i] > 0).mean())
        pts = np.column_stack(np.nonzero(labels == i)[::-1]).astype(np.float32)
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(pts)
        rw, rh = rw+1., rh+1.
        aspect = max(rw, rh)/min(rw, rh)
        fill = area/(rw*rh)
        if aspect > CRATE_MAX_ASPECT or fill < CRATE_MIN_FILL:
            continue
        clipped = _touches_border(pts, shape)
        a_long = math.radians(ang if rw >= rh else ang+90)
        axes = [(math.cos(a_long), math.sin(a_long)), (-math.sin(a_long), math.cos(a_long))]
        # Lug evidence along each rect axis: dark windows just beyond the body.
        dark_t, bg_v = _dark_threshold(v_channel, labels == i, (cx, cy), .16*s)
        lug = []
        for ax in axes:
            fr = []
            for sign in (-1, 1):
                c = (cx+sign*ax[0]*CRATE_LUG_CENTRE_M*s*1.05, cy+sign*ax[1]*CRATE_LUG_CENTRE_M*s*1.05)
                fr.append(_dark_fraction(v_channel, c, ax, .018*s, .012*s, dark_t))
            lug.append(None if None in fr else min(fr))
        long_idx = 0
        axis_source = 'body_long_axis'
        if aspect < 1.2 and lug[0] is not None and lug[1] is not None and lug[1] > lug[0]+.25:
            long_idx, axis_source = 1, 'lugs'      # near-square (occluded) body: trust the lugs
        ax = axes[long_idx]
        yaw = _img_angle_to_yaw(*ax)
        lug_score = lug[long_idx]
        washed_out = pink_fraction < .5
        if washed_out and (lug_score is None or lug_score < CRATE_WHITE_MIN_LUG or
                           not CRATE_WHITE_ASPECT[0] <= aspect <= CRATE_WHITE_ASPECT[1]):
            continue
        conf = min(1., area/ideal)
        if washed_out:
            conf *= .8
        if lug_score is not None:
            conf *= .6 + .4*min(1., lug_score/.5)
        if clipped:
            conf *= .6
        u, v = (float(c) for c in centers[i])
        box = cv2.boxPoints(((cx, cy), (rw, rh), ang))
        out.append(_row('heavy_crate', camera, shape, u, v, yaw, conf, box, clipped,
                        {'area_px': area, 'ideal_body_area_px': round(ideal, 1), 'aspect': round(aspect, 2),
                         'rect_fill': round(fill, 2), 'lug_dark_fraction': lug_score,
                         'axis_source': axis_source, 'pink_fraction': round(pink_fraction, 2),
                         'background_v': round(bg_v, 1)}))
    return out


def _beam(hsv, camera, shape):
    s = _scale(camera, shape, TOP_Z_M['long_beam'])
    mask = cv2.morphologyEx(_mask(hsv, CARGO_HSV['long_beam']), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    frags = []
    for i in range(1, count):
        if int(stats[i][4]) < BEAM_MIN_FRAGMENT_PX:
            continue
        pts = np.column_stack(np.nonzero(labels == i)[::-1]).astype(np.float32)
        (_, _), (rw, rh), _ = cv2.minAreaRect(pts)
        frags.append({'pts': pts, 'area': int(stats[i][4]), 'long': max(rw, rh)+1., 'short': min(rw, rh)+1.})
    frags.sort(key=lambda f: -f['area'])
    used = set()
    out = []
    for si, seed in enumerate(frags):
        if si in used or seed['long'] < BEAM_SEED_MIN_LEN_M*s or seed['long'] < 3*seed['short']:
            continue
        group = [si]
        pts = seed['pts']
        for _ in range(2):        # grow, refit, grow
            vx, vy, x0, y0 = (float(v) for v in cv2.fitLine(pts, cv2.DIST_HUBER, 0, .01, .01).ravel())
            d = np.array([vx, vy])
            n = np.array([-vy, vx])
            t = (pts - [x0, y0]) @ d
            tmin, tmax = float(t.min()), float(t.max())
            for fi, f in enumerate(frags):
                if fi in used or fi in group:
                    continue
                rel = f['pts'] - [x0, y0]
                if np.median(np.abs(rel @ n)) > BEAM_LINE_TOL_PX or np.max(np.abs(rel @ n)) > 2*BEAM_LINE_TOL_PX:
                    continue
                ft = rel @ d
                gap = max(float(ft.min())-tmax, tmin-float(ft.max()), 0.)
                new_len = max(tmax, float(ft.max())) - min(tmin, float(ft.min()))
                if gap <= BEAM_MAX_GAP_M*s and new_len <= (BEAM_LENGTH_M+.06)*s:
                    group.append(fi)
                    tmin, tmax = min(tmin, float(ft.min())), max(tmax, float(ft.max()))
            pts = np.concatenate([frags[g]['pts'] for g in group])
        vx, vy, x0, y0 = (float(v) for v in cv2.fitLine(pts, cv2.DIST_HUBER, 0, .01, .01).ravel())
        d = np.array([vx, vy])
        n = np.array([-vy, vx])
        t = (pts - [x0, y0]) @ d
        width = float(np.percentile(np.abs((pts - [x0, y0]) @ n), 95))*2+1
        tmin, tmax = float(t.min()), float(t.max())
        length_m = (tmax-tmin)/s
        if length_m < BEAM_MIN_LEN_M or width > 2.2*BEAM_WIDTH_M*s:
            continue
        used.update(group)
        p0 = np.array([x0, y0]) + tmin*d
        p1 = np.array([x0, y0]) + tmax*d
        h, w = shape[:2]

        def end_clipped(p):
            return bool(p[0] <= BORDER_PX+2 or p[1] <= BORDER_PX+2 or p[0] >= w-3-BORDER_PX or p[1] >= h-3-BORDER_PX)
        c0, c1 = end_clipped(p0), end_clipped(p1)
        if length_m >= BEAM_FULL_LEN_M:
            centre, mode, conf = (p0+p1)/2, 'both_ends', 1.
        elif c0 != c1:
            anchor, sign = (p1, -1) if c0 else (p0, 1)
            centre, mode, conf = anchor + sign*d*BEAM_END_OFFSET_M*s, 'one_end_known_length', .6
        else:
            centre, mode, conf = (p0+p1)/2, 'partial_midpoint', .3
        conf *= min(1., .5 + length_m/BEAM_FULL_LEN_M/2)
        yaw = _img_angle_to_yaw(vx, vy)
        z = TOP_Z_M['long_beam']
        foot = [p0 + n*BEAM_WIDTH_M*s, p1 + n*BEAM_WIDTH_M*s, p1 - n*BEAM_WIDTH_M*s, p0 - n*BEAM_WIDTH_M*s]
        row = _row('long_beam', camera, shape, float(centre[0]), float(centre[1]), yaw, conf, foot, c0 or c1,
                   {'visible_length_m': round(length_m, 3), 'fragments': len(group), 'width_px': round(width, 1),
                    'centre_mode': mode})
        # ``pixel`` is the observation (visible segment middle); ``floor_xy_m``
        # is the estimated beam centre, which lies off the visible part when an
        # end is clipped.
        mid = (p0+p1)/2
        row['pixel'] = [round(float(mid[0])/shape[1], 4), round(float(mid[1])/shape[0], 4)]
        row['segment_floor_m'] = [_floor(*p0, camera, shape, z), _floor(*p1, camera, shape, z)]
        row['segment_end_clipped'] = [c0, c1]
        out.append(row)
    return out


def _intersect(l1, l2):
    (p, d), (q, e) = l1, l2
    a = np.array([[d[0], -e[0]], [d[1], -e[1]]])
    if abs(np.linalg.det(a)) < 1e-6:
        return None
    t = np.linalg.solve(a, np.asarray(q)-np.asarray(p))
    return np.asarray(p) + t[0]*np.asarray(d)


def _frame(hsv, camera, shape):
    s = _scale(camera, shape, TOP_Z_M['tri_frame'])
    mask = cv2.morphologyEx(_mask(hsv, CARGO_HSV['tri_frame']), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    v_channel = hsv[..., 2]
    out = []
    comps = [i for i in range(1, count) if int(stats[i][4]) >= FRAME_MIN_AREA_PX]
    for i in comps:
        pts = np.column_stack(np.nonzero(labels == i)[::-1]).astype(np.float32)
        hull = cv2.convexHull(pts)
        _, tri = cv2.minEnclosingTriangle(hull)
        tri = tri.reshape(3, 2).astype(float)
        sides = [np.linalg.norm(tri[k]-tri[(k+1) % 3])/s for k in range(3)]
        tri_area = abs(float(cv2.contourArea(tri.astype(np.float32))))
        fill = len(pts)/max(tri_area, 1.)
        hull_ratio = float(cv2.contourArea(hull))/max(tri_area, 1.)
        if not (FRAME_SIDE_M[0] <= min(sides) and max(sides) <= FRAME_SIDE_M[1]):
            continue
        if not FRAME_FILL_OF_TRIANGLE[0] <= fill <= FRAME_FILL_OF_TRIANGLE[1] or hull_ratio < .6:
            continue
        # Refine: fit a line to the pixels nearest each enclosing-triangle side.
        lines, support = [], []
        for k in range(3):
            a, b = tri[k], tri[(k+1) % 3]
            d = (b-a)/np.linalg.norm(b-a)
            nrm = np.array([-d[1], d[0]])
            dist = np.abs((pts-a) @ nrm)
            others = [np.abs((pts-tri[j]) @ np.array([-(tri[(j+1) % 3]-tri[j])[1], (tri[(j+1) % 3]-tri[j])[0]])
                             / np.linalg.norm(tri[(j+1) % 3]-tri[j])) for j in range(3) if j != k]
            near = (dist <= FRAME_BAR_WIDTH_M*s*1.5) & (dist <= np.minimum(*others))
            support.append(int(near.sum()))
            if near.sum() >= 10:
                vx, vy, x0, y0 = (float(v) for v in cv2.fitLine(pts[near], cv2.DIST_HUBER, 0, .01, .01).ravel())
                lines.append(((x0, y0), (vx, vy)))
            else:
                lines.append(None)
        verts, mode = None, 'enclosing_triangle'
        if all(l is not None for l in lines):
            cand = [_intersect(lines[(k-1) % 3], lines[k]) for k in range(3)]
            if all(c is not None for c in cand):
                cand = np.array(cand)
                side = [np.linalg.norm(cand[k]-cand[(k+1) % 3])/s for k in range(3)]
                if FRAME_SIDE_M[0]*.8 <= min(side) and max(side) <= FRAME_SIDE_M[1]:
                    verts, mode = cand, 'fitted_side_lines'
        if verts is None:
            verts = tri
        centre = verts.mean(axis=0)
        yaws = []
        for vtx in verts:
            yaws.append(_img_angle_to_yaw(*(vtx-centre)))
        # Average the three vertex directions modulo 120 deg.
        z3 = sum(complex(math.cos(3*y), math.sin(3*y)) for y in yaws)
        yaw = math.atan2(z3.imag, z3.real)/3
        side_m = float(np.mean([np.linalg.norm(verts[k]-verts[(k+1) % 3])/s for k in range(3)]))
        dark_t, _ = _dark_threshold(v_channel, labels == i, centre, .32*s)
        lug = []
        for vtx in verts:
            dvec = (vtx-centre)/max(np.linalg.norm(vtx-centre), 1e-6)
            c = centre + dvec*FRAME_LUG_RADIUS_M*s*(np.linalg.norm(vtx-centre)/(FRAME_VERTEX_RADIUS_M*s))
            lug.append(_dark_fraction(v_channel, c, dvec, .02*s, .012*s, dark_t))
        lug_seen = [x for x in lug if x is not None]
        lug_score = float(np.median(lug_seen)) if lug_seen else None
        clipped = _touches_border(pts, shape)
        ideal_side = FRAME_VERTEX_RADIUS_M*math.sqrt(3)
        conf = max(0., 1. - abs(side_m-ideal_side)/ideal_side*2)
        conf *= (1. if mode == 'fitted_side_lines' else .7)
        if lug_score is not None:
            conf *= .7 + .3*min(1., lug_score/.4)
        if clipped:
            conf *= .6
        out.append(_row('tri_frame', camera, shape, float(centre[0]), float(centre[1]), yaw, conf, verts, clipped,
                        {'area_px': int(len(pts)), 'side_m': round(side_m, 3), 'fill_of_triangle': round(fill, 2),
                         'vertex_mode': mode, 'side_support_px': support, 'lug_dark_fraction': lug_score}))
    return out


# A box counts as lying on a tri_frame only near the frame's bars (and vertex
# lugs), not anywhere inside the open triangle (zone team A2, 2026-09-25: the
# held-out test hid a box resting inside a frame).
FRAME_BOX_SUPPRESS_M = FRAME_BAR_WIDTH_M/2 + .030


def _on_cargo(row, u, v, camera, shape):
    poly = np.asarray(row['footprint_px'], np.float32).reshape(-1, 1, 2)
    dist = cv2.pointPolygonTest(poly, (u, v), True)
    if row['kind'] == 'tri_frame':
        return abs(dist) <= FRAME_BOX_SUPPRESS_M*_scale(camera, shape, TOP_Z_M['tri_frame'])
    return dist >= -4


def _box_rows(jpeg, camera, shape, cargo_rows):
    rows = []
    for d in _zcb.detect_top(jpeg, camera, BOX_KINDS, profile=_zcb.TOP_PROFILE_ZONE):
        u, v = d['pixel'][0]*shape[1], d['pixel'][1]*shape[0]
        on_cargo = any(_on_cargo(r, u, v, camera, shape) for r in cargo_rows)
        rows.append({'kind': 'box', 'colour': d['kind'], 'floor_xy_m': d['floor_xy_m'], 'yaw_rad': None,
                     'yaw_symmetry_deg': None, 'confidence': 1.0, 'camera': camera['name'], 'pixel': d['pixel'],
                     'clipped': False, 'footprint_px': [], 'evidence': {'area_px': d['area_px'],
                                                                         'source_profile': _zcb.TOP_PROFILE_ZONE},
                     '_suppressed': on_cargo})
    return rows


def detect_cargo_top(jpeg: bytes, camera: Mapping[str, Any], kinds: Sequence[str] = CARGO_KINDS,
                     *, include_boxes: bool = True) -> list[dict[str, Any]]:
    """One TOP image -> per-item rows (cargo kinds, plus boxes via ``top_zone_v2``)."""
    frame = _zp._decode(jpeg) if not isinstance(jpeg, np.ndarray) else jpeg
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rows = []
    for kind in kinds:
        if kind not in CARGO_KINDS:
            raise ValueError(f'unknown cargo kind: {kind}')
        if kind in ('can', 'tile'):
            rows += _small(kind, hsv, camera, frame.shape)
            if kind == 'tile' or 'tile' not in kinds:
                rows += [r for r in _small_bright(hsv, camera, frame.shape,
                                                  [r for r in rows if r['kind'] in ('can', 'tile')])
                         if r['kind'] in kinds]
        elif kind == 'heavy_crate':
            rows += _crate(hsv, camera, frame.shape)
        elif kind == 'long_beam':
            rows += _beam(hsv, camera, frame.shape)
        else:
            rows += _frame(hsv, camera, frame.shape)
    frames = [np.asarray(r['footprint_px'], np.float32).reshape(-1, 1, 2) for r in rows if r['kind'] == 'tri_frame']
    if frames:
        def on_frame(r):
            mid = np.mean(np.asarray(r['footprint_px'], float), axis=0)
            return any(cv2.pointPolygonTest(f, (float(mid[0]), float(mid[1])), True) >= -12 for f in frames) and \
                r['evidence']['visible_length_m'] < .40
        rows = [r for r in rows if not (r['kind'] == 'long_beam' and on_frame(r))]
    if include_boxes:
        if isinstance(jpeg, np.ndarray):
            ok, enc = cv2.imencode('.jpg', jpeg)
            jpeg = enc.tobytes()
        boxes = _box_rows(jpeg, camera, frame.shape, rows)
        rows += [b for b in boxes if not b.pop('_suppressed')]
    return rows


# ------------------------------------------------------------------ team merge

def _merge_beams(beams):
    """World-space union of collinear beam pieces seen by different TOPs."""
    groups = []
    for b in sorted(beams, key=lambda r: -r['evidence']['visible_length_m']):
        p0, p1 = (np.asarray(p) for p in b['segment_floor_m'])
        for g in groups:
            q0, q1 = g['axis']
            d = (q1-q0)/max(np.linalg.norm(q1-q0), 1e-6)
            n = np.array([-d[1], d[0]])
            if abs(float((p0-q0) @ n)) > .05 or abs(float((p1-q0) @ n)) > .05:
                continue
            t = sorted([float((p0-q0) @ d), float((p1-q0) @ d)])
            ts = [float((e-q0) @ d) for e in (q0, q1)]
            if t[0] > max(ts)+.10 or t[1] < min(ts)-.10:
                continue
            g['members'].append(b)
            break
        else:
            groups.append({'axis': (p0, p1), 'members': [b]})
    out = []
    for g in groups:
        if len(g['members']) == 1:
            out.append(g['members'][0])
            continue
        q0, q1 = g['axis']
        d = (q1-q0)/np.linalg.norm(q1-q0)
        ends = []
        for m in g['members']:
            for e, clipped in zip(m['segment_floor_m'], m['segment_end_clipped']):
                ends.append((float((np.asarray(e)-q0) @ d), clipped))
        lo = min(t for t, _ in ends)
        hi = max(t for t, _ in ends)
        lo_clip = all(c for t, c in ends if t <= lo+.02)
        hi_clip = all(c for t, c in ends if t >= hi-.02)
        length = hi-lo
        if length >= BEAM_FULL_LEN_M or (not lo_clip and not hi_clip):
            mid, mode, conf = (lo+hi)/2, 'merged_both_ends' if length >= BEAM_FULL_LEN_M else 'merged_partial_midpoint', 1. if length >= BEAM_FULL_LEN_M else .3
        elif lo_clip != hi_clip:
            mid = (hi-BEAM_END_OFFSET_M) if lo_clip else (lo+BEAM_END_OFFSET_M)
            mode, conf = 'merged_one_end_known_length', .6
        else:
            mid, mode, conf = (lo+hi)/2, 'merged_partial_midpoint', .3
        centre = q0 + d*mid
        best = max(g['members'], key=lambda r: r['confidence'])
        merged = dict(best)
        merged['floor_xy_m'] = [round(float(centre[0]), 4), round(float(centre[1]), 4)]
        merged['yaw_rad'] = round(float(_wrap_sym(math.atan2(d[1], d[0]), 180.)), 4)
        merged['confidence'] = round(conf*min(1., .5+length/BEAM_FULL_LEN_M/2), 3)
        merged['camera'] = '+'.join(sorted({m['camera'] for m in g['members']}))
        merged['clipped'] = bool(lo_clip or hi_clip)
        merged['evidence'] = {**best['evidence'], 'centre_mode': mode, 'visible_length_m': round(length, 3),
                              'merged_views': len(g['members'])}
        merged['segment_floor_m'] = [[round(float(v), 4) for v in q0+d*lo], [round(float(v), 4) for v in q0+d*hi]]
        merged['segment_end_clipped'] = [lo_clip, hi_clip]
        out.append(merged)
    return out


def detect_all_cargo(tops: Mapping[str, bytes], static_map: Mapping[str, Any], kinds: Sequence[str] = CARGO_KINDS,
                     *, include_boxes: bool = True) -> dict[str, Any]:
    """All TOPs -> one item list. Overlap duplicates keep the unclipped, more
    confident, more central view; beam pieces are united along their line."""
    rows = []
    for camera in static_map['top_cameras']:
        for row in detect_cargo_top(tops[camera['name']], camera, kinds, include_boxes=include_boxes):
            row['_off'] = math.hypot(row['pixel'][0]-.5, row['pixel'][1]-.5)
            rows.append(row)
    beams = [r for r in rows if r['kind'] == 'long_beam']
    rest = [r for r in rows if r['kind'] != 'long_beam']
    rest.sort(key=lambda r: (r['clipped'], -r['confidence'], r['_off']))
    kept = []
    for row in rest:
        if any(k['kind'] == row['kind'] and k.get('colour') == row.get('colour') and
               math.dist(k['floor_xy_m'], row['floor_xy_m']) < DEDUPE_M[row['kind']] for k in kept):
            continue
        kept.append(row)
    kept += _merge_beams(beams)
    for row in kept:
        row.pop('_off', None)
        row.pop('footprint_px', None)
    kept.sort(key=lambda r: (r['kind'], r.get('colour') or '', r['floor_xy_m'][0], r['floor_xy_m'][1]))
    return {'schema': SCHEMA, 'profile': PROFILE, 'source': PROVENANCE, 'items': kept}


# ------------------------------------------------------------------ static grasp derivation

def grasp_handles(item: Mapping[str, Any]) -> dict[str, Any]:
    """Grip points and approach base poses for a detected item.

    Derived from the RGB pose estimate (``floor_xy_m``, ``yaw_rad``) and the
    static catalogue's grasp roles (``sim.zone_cargo``). No simulator state.
    Because the estimated yaw is only known modulo the kind's symmetry, role
    names are interchangeable within that symmetry: the *set* of grip points
    is well defined; which robot takes which point is a team decision.
    """
    from sim.zone_cargo import CATALOGUE, GRASP_RADIUS_M
    kind = item['kind']
    if kind not in CATALOGUE:
        return {'kind': kind, 'handles': [], 'note': 'boxes use the map approach convention'}
    spec = CATALOGUE[kind]
    x0, y0 = item['floor_xy_m']
    yaw0 = item['yaw_rad']
    free_heading = yaw0 is None
    if free_heading:
        yaw0 = 0.
    c, s = math.cos(yaw0), math.sin(yaw0)
    handles = []
    for g in spec.grasps:
        gx, gy, gz = g.grip_xyz
        bx, by, byaw = g.approach_base()
        handles.append({'role': g.role, 'grip_xyz_m': [round(x0+c*gx-s*gy, 4), round(y0+s*gx+c*gy, 4), gz],
                        'approach_base_xyyaw': None if free_heading else
                        [round(x0+c*bx-s*by, 4), round(y0+s*bx+c*by, 4),
                         round(math.atan2(math.sin(yaw0+byaw), math.cos(yaw0+byaw)), 4)],
                        'grasp_radius_m': GRASP_RADIUS_M})
    return {'kind': kind, 'required_carriers': spec.required_carriers, 'formations': [list(f) for f in spec.formations],
            'yaw_symmetry_deg': YAW_SYMMETRY_DEG[kind], 'handles': handles,
            'role_names_interchangeable_under_symmetry': YAW_SYMMETRY_DEG[kind] is not None and len(spec.grasps) > 1,
            'approach_heading_free': free_heading,
            'source': 'RGB pose estimate + static catalogue grasp roles (sim.zone_cargo); not simulator state'}


__all__ = ['PROFILE', 'SCHEMA', 'CARGO_KINDS', 'CARGO_HSV', 'detect_cargo_top', 'detect_all_cargo', 'grasp_handles']
