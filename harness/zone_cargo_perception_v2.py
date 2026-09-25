"""TOP-RGB cargo detection, profile ``top_cargo_v2`` (opt-in).

Same inputs, colour/shape cues and output schema as ``top_cargo_v1``
(``harness.zone_cargo_perception``, which is not modified and keeps its
outputs byte for byte). v2 changes only what the adversarial review of PR #168
found (2026-09-25):

1. Beams (high). v1 could report two separate parallel beams a few
   centimetres apart as one: the 3x3 close fuses them into one component
   whose width still passes v1's 2.2x gate, and ``_merge_beams`` united
   pieces 5 cm apart laterally without checking the camera or the length.
   v2 splits a component wider than 1.6x the bar (a single bar measures
   <= 1.45x on dev renders) along its normal, never merges or de-duplicates
   two detections from the same camera, and unites pieces from different
   TOPs only when they are collinear (<= 2 cm, <= 5 deg), overlap along the
   axis (the TOP views overlap), respect each other's visible (unclipped)
   ends and the union is no longer than the static beam length.
2. Boxes inside a frame (medium). v1 suppressed a box blob anywhere inside
   the frame's triangle. v2 suppresses a box only on the frame's bars and
   vertex lugs; the open interior is floor.

Inputs stay RGB + authored TOP calibration + static catalogue only.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import cv2
import numpy as np

from harness import zone_cargo_perception as v1

PROFILE = 'top_cargo_v2'
SCHEMA = v1.SCHEMA
CARGO_KINDS = v1.CARGO_KINDS
PROVENANCE = v1.PROVENANCE

# Beam (per camera): a single bar is <= 1.45x its nominal width on dev renders.
BEAM_SPLIT_WIDTH_RATIO = 1.6
BEAM_MAX_WIDTH_RATIO = 1.6
BEAM_SPLIT_MIN_SHARE = .2
# Beam (across TOPs).
MERGE_LATERAL_M = .02
MERGE_ANGLE_DEG = 5.
MERGE_MIN_OVERLAP_M = -.03          # the pieces must overlap (TOP views overlap ~0.3 m)
MERGE_END_SLACK_M = .03
MERGE_MAX_LENGTH_M = v1.BEAM_LENGTH_M + .03
# Box suppression on a frame: bars' centre-line +- (half bar + margin), lugs.
FRAME_BOX_MARGIN_M = .010
FRAME_LUG_SUPPRESS_M = .045


# ------------------------------------------------------------------ beams (per camera)

def _line(pts):
    vx, vy, x0, y0 = (float(v) for v in cv2.fitLine(pts, cv2.DIST_HUBER, 0, .01, .01).ravel())
    d = np.array([vx, vy])
    return d, np.array([-vy, vx]), np.array([x0, y0])


def _width(pts, n, p0):
    return float(np.percentile(np.abs((pts - p0) @ n), 95))*2+1


def _split_wide(pts, s):
    """Split a fused group of parallel bars along its normal (Otsu on offsets)."""
    out, todo = [], [pts]
    while todo:
        p = todo.pop()
        d, n, p0 = _line(p)
        if _width(p, n, p0) <= BEAM_SPLIT_WIDTH_RATIO*v1.BEAM_WIDTH_M*s or len(p) < 40:
            out.append(p)
            continue
        off = (p - p0) @ n
        lo, hi = float(off.min()), float(off.max())
        hist = np.round((off - lo)/(hi - lo + 1e-9)*255).astype(np.uint8).reshape(-1, 1)
        t, _ = cv2.threshold(hist, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        cut = lo + (hi - lo)*float(t)/255
        a, b = p[off <= cut], p[off > cut]
        if min(len(a), len(b)) < BEAM_SPLIT_MIN_SHARE*len(p):
            out.append(p)
            continue
        todo += [a, b]
    return out


def _beam(hsv, camera, shape):
    """v1's fragment grouping; then fused parallel bars are split and every
    resulting bar must be no wider than 1.6x the nominal width."""
    s = v1._scale(camera, shape, v1.TOP_Z_M['long_beam'])
    mask = cv2.morphologyEx(v1._mask(hsv, v1.CARGO_HSV['long_beam']), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    frags = []
    for i in range(1, count):
        if int(stats[i][4]) < v1.BEAM_MIN_FRAGMENT_PX:
            continue
        pts = np.column_stack(np.nonzero(labels == i)[::-1]).astype(np.float32)
        for part in _split_wide(pts, s):
            (_, _), (rw, rh), _ = cv2.minAreaRect(part)
            frags.append({'pts': part, 'area': len(part), 'long': max(rw, rh)+1., 'short': min(rw, rh)+1.,
                          'split': len(part) != len(pts)})
    frags.sort(key=lambda f: -f['area'])
    used = set()
    out = []
    for si, seed in enumerate(frags):
        if si in used or seed['long'] < v1.BEAM_SEED_MIN_LEN_M*s or seed['long'] < 3*seed['short']:
            continue
        group = [si]
        pts = seed['pts']
        for _ in range(2):
            d, n, p0 = _line(pts)
            t = (pts - p0) @ d
            tmin, tmax = float(t.min()), float(t.max())
            for fi, f in enumerate(frags):
                if fi in used or fi in group:
                    continue
                rel = f['pts'] - p0
                if np.median(np.abs(rel @ n)) > v1.BEAM_LINE_TOL_PX or np.max(np.abs(rel @ n)) > 2*v1.BEAM_LINE_TOL_PX:
                    continue
                ft = rel @ d
                gap = max(float(ft.min())-tmax, tmin-float(ft.max()), 0.)
                new_len = max(tmax, float(ft.max())) - min(tmin, float(ft.min()))
                if gap <= v1.BEAM_MAX_GAP_M*s and new_len <= (v1.BEAM_LENGTH_M+.06)*s:
                    group.append(fi)
                    tmin, tmax = min(tmin, float(ft.min())), max(tmax, float(ft.max()))
            pts = np.concatenate([frags[g]['pts'] for g in group])
        d, n, p0 = _line(pts)
        t = (pts - p0) @ d
        width = _width(pts, n, p0)
        tmin, tmax = float(t.min()), float(t.max())
        length_m = (tmax-tmin)/s
        if length_m < v1.BEAM_MIN_LEN_M or width > BEAM_MAX_WIDTH_RATIO*v1.BEAM_WIDTH_M*s:
            continue
        used.update(group)
        e0, e1 = p0 + tmin*d, p0 + tmax*d
        h, w = shape[:2]

        def end_clipped(p):
            return bool(p[0] <= v1.BORDER_PX+2 or p[1] <= v1.BORDER_PX+2 or
                        p[0] >= w-3-v1.BORDER_PX or p[1] >= h-3-v1.BORDER_PX)
        c0, c1 = end_clipped(e0), end_clipped(e1)
        if length_m >= v1.BEAM_FULL_LEN_M:
            centre, mode, conf = (e0+e1)/2, 'both_ends', 1.
        elif c0 != c1:
            anchor, sign = (e1, -1) if c0 else (e0, 1)
            centre, mode, conf = anchor + sign*d*v1.BEAM_END_OFFSET_M*s, 'one_end_known_length', .6
        else:
            centre, mode, conf = (e0+e1)/2, 'partial_midpoint', .3
        conf *= min(1., .5 + length_m/v1.BEAM_FULL_LEN_M/2)
        yaw = v1._img_angle_to_yaw(d[0], d[1])
        z = v1.TOP_Z_M['long_beam']
        foot = [e0 + n*v1.BEAM_WIDTH_M*s, e1 + n*v1.BEAM_WIDTH_M*s, e1 - n*v1.BEAM_WIDTH_M*s, e0 - n*v1.BEAM_WIDTH_M*s]
        row = v1._row('long_beam', camera, shape, float(centre[0]), float(centre[1]), yaw, conf, foot, c0 or c1,
                      {'visible_length_m': round(length_m, 3), 'fragments': len(group), 'width_px': round(width, 1),
                       'centre_mode': mode, 'split_from_wider_blob': any(frags[g]['split'] for g in group)})
        mid = (e0+e1)/2
        row['pixel'] = [round(float(mid[0])/shape[1], 4), round(float(mid[1])/shape[0], 4)]
        row['segment_floor_m'] = [v1._floor(*e0, camera, shape, z), v1._floor(*e1, camera, shape, z)]
        row['segment_end_clipped'] = [c0, c1]
        out.append(row)
    return out


# ------------------------------------------------------------------ box suppression

def _frame_material(row, u, v, camera, shape):
    """True when (u, v) lies on a frame bar or vertex lug (not the open interior)."""
    s = v1._scale(camera, shape, v1.TOP_Z_M['tri_frame'])
    verts = np.asarray(row['footprint_px'], float)
    p = np.array([u, v], float)
    band = (v1.FRAME_BAR_WIDTH_M/2 + FRAME_BOX_MARGIN_M)*s
    for k in range(3):
        a, b = verts[k], verts[(k+1) % 3]
        ab = b - a
        t = float(np.clip((p-a) @ ab / max(ab @ ab, 1e-9), 0., 1.))
        if np.linalg.norm(p - (a + t*ab)) <= band:
            return True
    centre = verts.mean(axis=0)
    for vtx in verts:
        lug = centre + (vtx-centre)*(v1.FRAME_LUG_RADIUS_M/v1.FRAME_VERTEX_RADIUS_M)
        if np.linalg.norm(p - lug) <= FRAME_LUG_SUPPRESS_M*s:
            return True
    return False


def _on_cargo(row, u, v, camera, shape):
    if row['kind'] == 'tri_frame':
        return _frame_material(row, u, v, camera, shape)
    poly = np.asarray(row['footprint_px'], np.float32).reshape(-1, 1, 2)
    return cv2.pointPolygonTest(poly, (float(u), float(v)), True) >= -4


def _box_rows(jpeg, camera, shape, cargo_rows):
    rows = []
    for d in v1._zcb.detect_top(jpeg, camera, v1.BOX_KINDS, profile=v1._zcb.TOP_PROFILE_ZONE):
        u, v = d['pixel'][0]*shape[1], d['pixel'][1]*shape[0]
        if any(_on_cargo(r, u, v, camera, shape) for r in cargo_rows):
            continue
        rows.append({'kind': 'box', 'colour': d['kind'], 'floor_xy_m': d['floor_xy_m'], 'yaw_rad': None,
                     'yaw_symmetry_deg': None, 'confidence': 1.0, 'camera': camera['name'], 'pixel': d['pixel'],
                     'clipped': False, 'footprint_px': [],
                     'evidence': {'area_px': d['area_px'], 'source_profile': v1._zcb.TOP_PROFILE_ZONE}})
    return rows


# ------------------------------------------------------------------ one TOP

def detect_cargo_top(jpeg: bytes, camera: Mapping[str, Any], kinds: Sequence[str] = CARGO_KINDS,
                     *, include_boxes: bool = True) -> list[dict[str, Any]]:
    frame = v1._zp._decode(jpeg) if not isinstance(jpeg, np.ndarray) else jpeg
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rows = []
    for kind in kinds:
        if kind not in CARGO_KINDS:
            raise ValueError(f'unknown cargo kind: {kind}')
        if kind in ('can', 'tile'):
            rows += v1._small(kind, hsv, camera, frame.shape)
            if kind == 'tile' or 'tile' not in kinds:
                rows += [r for r in v1._small_bright(hsv, camera, frame.shape,
                                                     [r for r in rows if r['kind'] in ('can', 'tile')])
                         if r['kind'] in kinds]
        elif kind == 'heavy_crate':
            rows += v1._crate(hsv, camera, frame.shape)
        elif kind == 'long_beam':
            rows += _beam(hsv, camera, frame.shape)
        else:
            rows += v1._frame(hsv, camera, frame.shape)
    frames = [np.asarray(r['footprint_px'], np.float32).reshape(-1, 1, 2) for r in rows if r['kind'] == 'tri_frame']
    if frames:
        def on_frame(r):
            mid = np.mean(np.asarray(r['footprint_px'], float), axis=0)
            return any(cv2.pointPolygonTest(f, (float(mid[0]), float(mid[1])), True) >= -12 for f in frames) and \
                r['evidence']['visible_length_m'] < .40
        rows = [r for r in rows if not (r['kind'] == 'long_beam' and on_frame(r))]
    if include_boxes:
        if isinstance(jpeg, np.ndarray):
            _, enc = cv2.imencode('.jpg', jpeg)
            jpeg = enc.tobytes()
        rows += _box_rows(jpeg, camera, frame.shape, rows)
    return rows


# ------------------------------------------------------------------ team merge

def _axis(seg):
    p0, p1 = (np.asarray(p, float) for p in seg)
    d = (p1-p0)/max(np.linalg.norm(p1-p0), 1e-9)
    return p0, d


def _compatible(members, cand):
    """May ``cand`` (another TOP) be the same physical beam as ``members``?"""
    if any(m['camera'] == cand['camera'] for m in members):
        return False
    ref = max(members, key=lambda r: r['evidence']['visible_length_m'])
    q0, d = _axis(ref['segment_floor_m'])
    n = np.array([-d[1], d[0]])
    c0, cd = _axis(cand['segment_floor_m'])
    angle = math.degrees(math.acos(min(1., abs(float(d @ cd)))))
    if angle > MERGE_ANGLE_DEG:
        return False
    ends = [np.asarray(e, float) for e in cand['segment_floor_m']]
    if any(abs(float((e-q0) @ n)) > MERGE_LATERAL_M for e in ends):
        return False
    cand_t = sorted(float((e-q0) @ d) for e in ends)
    lo = min(float((np.asarray(e)-q0) @ d) for m in members for e in m['segment_floor_m'])
    hi = max(float((np.asarray(e)-q0) @ d) for m in members for e in m['segment_floor_m'])
    overlap = min(hi, cand_t[1]) - max(lo, cand_t[0])
    if overlap < MERGE_MIN_OVERLAP_M:
        return False
    if max(hi, cand_t[1]) - min(lo, cand_t[0]) > MERGE_MAX_LENGTH_M:
        return False
    # A visible (unclipped) end of either piece is a true beam end: the other
    # piece may not run past it.
    for m in members + [cand]:
        others = [x for x in members + [cand] if x is not m]
        ot = [float((np.asarray(e)-q0) @ d) for x in others for e in x['segment_floor_m']]
        mt = [(float((np.asarray(e)-q0) @ d), c) for e, c in zip(m['segment_floor_m'], m['segment_end_clipped'])]
        (ta, ca), (tb, cb) = sorted(mt)
        if not ca and min(ot) < ta - MERGE_END_SLACK_M:
            return False
        if not cb and max(ot) > tb + MERGE_END_SLACK_M:
            return False
    return True


def _merge_beams(beams):
    clusters = []
    for b in sorted(beams, key=lambda r: -r['evidence']['visible_length_m']):
        for c in clusters:
            if _compatible(c, b):
                c.append(b)
                break
        else:
            clusters.append([b])
    out = []
    for members in clusters:
        if len(members) == 1:
            out.append(members[0])
            continue
        ref = max(members, key=lambda r: r['evidence']['visible_length_m'])
        q0, d = _axis(ref['segment_floor_m'])
        ends = [(float((np.asarray(e)-q0) @ d), c) for m in members
                for e, c in zip(m['segment_floor_m'], m['segment_end_clipped'])]
        lo = min(t for t, _ in ends)
        hi = max(t for t, _ in ends)
        lo_clip = all(c for t, c in ends if t <= lo+.02)
        hi_clip = all(c for t, c in ends if t >= hi-.02)
        length = hi-lo
        if length >= v1.BEAM_FULL_LEN_M:
            mid, mode, conf = (lo+hi)/2, 'merged_both_ends', 1.
        elif lo_clip != hi_clip:
            mid = (hi-v1.BEAM_END_OFFSET_M) if lo_clip else (lo+v1.BEAM_END_OFFSET_M)
            mode, conf = 'merged_one_end_known_length', .6
        else:
            mid, mode, conf = (lo+hi)/2, 'merged_partial_midpoint', .3
        centre = q0 + d*mid
        best = max(members, key=lambda r: r['confidence'])
        merged = dict(best)
        merged['floor_xy_m'] = [round(float(centre[0]), 4), round(float(centre[1]), 4)]
        merged['yaw_rad'] = round(float(v1._wrap_sym(math.atan2(d[1], d[0]), 180.)), 4)
        merged['confidence'] = round(conf*min(1., .5+length/v1.BEAM_FULL_LEN_M/2), 3)
        merged['camera'] = '+'.join(sorted({m['camera'] for m in members}))
        merged['clipped'] = bool(lo_clip or hi_clip)
        merged['evidence'] = {**best['evidence'], 'centre_mode': mode, 'visible_length_m': round(length, 3),
                              'merged_views': len(members)}
        merged['segment_floor_m'] = [[round(float(v), 4) for v in q0+d*lo], [round(float(v), 4) for v in q0+d*hi]]
        merged['segment_end_clipped'] = [lo_clip, hi_clip]
        out.append(merged)
    return out


def detect_all_cargo(tops: Mapping[str, bytes], static_map: Mapping[str, Any], kinds: Sequence[str] = CARGO_KINDS,
                     *, include_boxes: bool = True) -> dict[str, Any]:
    """All TOPs -> one item list. Only detections from *different* TOPs are
    ever treated as the same item."""
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
        if any(k['kind'] == row['kind'] and k.get('colour') == row.get('colour') and k['camera'] != row['camera'] and
               math.dist(k['floor_xy_m'], row['floor_xy_m']) < v1.DEDUPE_M[row['kind']] for k in kept):
            continue
        kept.append(row)
    kept += _merge_beams(beams)
    for row in kept:
        row.pop('_off', None)
        row.pop('footprint_px', None)
    kept.sort(key=lambda r: (r['kind'], r.get('colour') or '', r['floor_xy_m'][0], r['floor_xy_m'][1]))
    return {'schema': SCHEMA, 'profile': PROFILE, 'source': PROVENANCE, 'items': kept}


grasp_handles = v1.grasp_handles

__all__ = ['PROFILE', 'SCHEMA', 'CARGO_KINDS', 'detect_cargo_top', 'detect_all_cargo', 'grasp_handles']
