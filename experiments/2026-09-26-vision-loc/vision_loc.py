"""Tag-free own-camera localization from a learned floor/wall segmentation (no torch here).

Pipeline for every own wrist frame (environment v3 walls, 0.40 m, zero AprilTags):

1. raw fisheye JPEG -> pinhole image (``markerless_probe.undistort``, PR #210);
2. per-pixel classes floor / wall / self / object / background from a small
   segmentation network (``seg_model.py``, torchvision LR-ASPP MobileNetV3);
3. per image column (strip), an INTERVAL observation of the row where the first
   wall's bottom edge (floor/wall boundary) lies: rows labelled wall lie above it,
   rows labelled floor below it. With nothing in between it is a sharp edge
   (sub-pixel crossing of the class probabilities); with the own arm, the carried
   box, a peer robot or a box in between it is an interval (occlusion); with no
   wall in the column it is an upper bound (free floor up to the highest floor
   pixel); with wall down to the image bottom it is a lower bound. The same is done
   for the wall's top edge (wall below, background above) when it is in view.
   Door openings, door jambs (divider ends) and wall corners appear as jumps and
   kinks of these rows across columns; free-floor columns are negative evidence;
4. likelihood of every particle: rows ray cast from the static map
   (``markerless_probe.MapGeometry.expected_rows``, PR #210) through each column's
   floor and 0.40 m traces, compared with the intervals (Gaussian-smoothed
   interval probability, robust outlier floor, effective column count);
5. the M1 particle filter (``owncam_localizer.py`` at the M1 commit, loaded
   byte-for-byte by ``markerless_probe.load_m1_localizer``): same motion model,
   own load state, motion profiles. Measurements are used only on settled frames
   (no own arm/pan command in the last ``settle_s``): the issued PWM leads the
   servos, and during a pan sweep the camera is up to 6 deg behind the command.

Robot inputs only: own frames, own issued commands, own motion-profile switches,
the static tag-free map and fixed calibrations (camera K/D, M1 motion model,
the extrinsic sag table fitted offline on the TRAIN split). ``eval_only/`` is read
only by the calibration, oracle-diagnostic and scoring code in
``run_vision_loc.py``; nothing in this module opens it.
"""
from __future__ import annotations

import importlib.util
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
_PROBE = ROOT/'experiments'/'2026-09-26-markerless-probe'/'markerless_probe.py'


def _load_probe():
    """PR #210's ``markerless_probe`` (camera model, column traces, map ray cast, M1 PF loader)."""
    import sys
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    if 'markerless_probe' in sys.modules:
        return sys.modules['markerless_probe']
    spec = importlib.util.spec_from_file_location('markerless_probe', _PROBE)
    module = importlib.util.module_from_spec(spec)
    sys.modules['markerless_probe'] = module
    spec.loader.exec_module(module)
    return module


mp = _load_probe()
SCHEMA = 'ugrp.vision_loc.v1'
WIDTH, HEIGHT = mp.WIDTH, mp.HEIGHT
CLASSES = ('floor', 'wall', 'self', 'object', 'background')
FLOOR, WALL, SELF, OBJECT, BACKGROUND = range(5)
IGNORE = 255
WALL_HEIGHT_M = .40                  # wall profile walls_v3 (static map wall_profile)
# observation kinds per column
NONE, EDGE, INTERVAL = 0, 1, 2
NEG_INF, POS_INF = -1e4, 1e4

DEFAULT_OBS = {
    'columns': 96,             # evenly spaced image columns
    'strip_half_px': 2,        # class probabilities averaged over 2*2+1 image columns
    'min_run_px': 3,           # a wall / floor / background run must be this long
    'use_top_edge': True,
    'consistency_px': None,    # drop sharp edges inconsistent with neighbour columns (None: keep all)
}
DEFAULT_MEASUREMENT = {
    'sigma_px': 2.5,           # row noise of the expected edge (extrinsic + label)
    'outlier_prob': .1,
    'effective_columns': 12.,  # columns in one frame share the extrinsic error
    'min_columns': 6,          # fewer informative columns: no update
    'settle_s': .4,            # measurement only this long after the last own arm/pan command
    'top_weight': 1.,
    # 'all': temper by every observed column; 'discriminative': only columns whose probability differs
    # across the particle cloud (> discriminative_min) count, so columns every particle explains
    # (a near wall behind the carried box) do not dilute the few informative ones (door jambs)
    'scale_by': 'all',
    'discriminative_min': .1,
}


def valid_mask() -> np.ndarray:
    """Pinhole pixels that exist in the raw fisheye frame (bool, H x W).

    The simulator renders pinhole and remaps to fisheye; undistorting the raw
    frame back leaves pixels outside the raw frame black. They are ignored in
    training and in the column observations.
    """
    white = np.full((HEIGHT, WIDTH), 255, np.uint8)
    back = cv2.remap(white, mp._UNDISTORT[0], mp._UNDISTORT[1], cv2.INTER_NEAREST, borderValue=0)
    return cv2.erode((back > 0).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0


VALID = valid_mask()


def column_positions(n: int, half: int) -> np.ndarray:
    return mp.column_positions(n, half)


# ----------------------------------------------------------------------------- extrinsic correction
@dataclass
class ColumnModelDZ(mp.ColumnModel):
    """PR #210 ``ColumnModel`` with a camera height correction ``dz`` and a base yaw offset ``dyaw``.

    ``dyaw`` rotates the whole FK camera pose about the base z axis: with the box
    held, a pan sweep turns the chassis against the pan on its wheels (reversibly;
    ``pan_base_yaw`` in the calibration), so the camera sees the map from the
    nominal base yaw plus this offset.
    """
    dz: float = 0.
    dyaw: float = 0.

    def __post_init__(self):
        orig = mp.camera_in_base
        dz = float(self.dz)
        c, s = math.cos(float(self.dyaw)), math.sin(float(self.dyaw))
        rz = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

        def shifted(pose):
            o, r = orig(pose)
            return rz @ o + np.array([0., 0., dz]), rz @ r
        mp.camera_in_base = shifted
        try:
            super().__post_init__()
        finally:
            mp.camera_in_base = orig


_CM_CACHE: dict = {}


def column_model(servo: Mapping, bias: float, dz: float, columns: np.ndarray, dyaw: float = 0.) -> ColumnModelDZ:
    key = (tuple(sorted((int(k), int(v)) for k, v in servo.items())), round(float(bias), 6), round(float(dz), 5),
           round(float(dyaw), 6), tuple(int(c) for c in columns))
    if key not in _CM_CACHE:
        if len(_CM_CACHE) > 1024:
            _CM_CACHE.clear()
        _CM_CACHE[key] = ColumnModelDZ(key[0], float(bias), np.asarray(columns), dz=float(dz), dyaw=float(dyaw))
    return _CM_CACHE[key]


def pan_yaw(table: Mapping, loaded: bool, servo: Mapping) -> float:
    """Reversible chassis yaw offset (rad) caused by the own pan pulse (calibration ``pan_base_yaw``)."""
    k = (table or {}).get('loaded' if loaded else 'unloaded', 0.)
    return float(k)*(float(servo.get(6, 1500)) - 1500.)


def sag(table: Mapping, loaded: bool, servo: Mapping) -> tuple[float, float]:
    """(elevation bias rad, camera height correction m) for the commanded pose and own load state."""
    t = table['loaded' if loaded else 'unloaded']
    s3 = float(servo[3])
    return (float(np.interp(s3, np.asarray(t['s3'], float), np.asarray(t['bias'], float))),
            float(np.interp(s3, np.asarray(t['s3'], float), np.asarray(t['dz'], float))))


def elevation_and_dz(r_true_b: np.ndarray, p_true_b: np.ndarray, servo: Mapping) -> tuple[float, float, float]:
    """(bias, dz, azimuth error) of a true camera pose (base frame) against the commanded-PWM FK.

    ``ColumnModel`` uses ``R_fk @ bias_rotation(b).T`` as the corrected optical
    frame, so ``b`` is the rotation of the true optical frame about the FK optical
    x axis. Used offline by the calibration step (GT camera poses of TRAIN frames).
    """
    o, r_fk = mp.camera_in_base(dict(servo))
    m = r_fk.T @ r_true_b
    b = math.atan2(m[2, 1], m[1, 1])
    az = math.atan2(r_true_b[1, 2], r_true_b[0, 2]) - math.atan2(r_fk[1, 2], r_fk[0, 2])
    return b, float(p_true_b[2] - o[2]), (az + math.pi) % (2*math.pi) - math.pi


# ----------------------------------------------------------------------------- observations
@dataclass
class ColumnObs:
    """Per-column interval observations of the first wall's bottom (and top) edge rows.

    ``b_lo``/``b_hi``: the bottom-edge row lies in [b_lo, b_hi] (rows grow downward;
    NEG_INF / POS_INF for open ends); ``b_kind`` EDGE when the interval is a sharp
    crossing (then ``b_lo == b_hi`` is the sub-pixel row), INTERVAL, or NONE.
    ``t_*`` likewise for the top edge (wall below, background above).
    """
    columns: np.ndarray
    b_kind: np.ndarray
    b_lo: np.ndarray
    b_hi: np.ndarray
    t_kind: np.ndarray
    t_lo: np.ndarray
    t_hi: np.ndarray

    @property
    def informative(self) -> np.ndarray:
        return (self.b_kind != NONE) | (self.t_kind != NONE)

    def as_dict(self) -> dict:
        r = lambda a: [None if not np.isfinite(x) else round(float(x), 2) for x in a]
        return {'b_kind': self.b_kind.tolist(), 'b_lo': r(self.b_lo), 'b_hi': r(self.b_hi),
                't_kind': self.t_kind.tolist(), 't_lo': r(self.t_lo), 't_hi': r(self.t_hi)}

    @classmethod
    def from_dict(cls, d: Mapping, columns: np.ndarray) -> 'ColumnObs':
        f = lambda k: np.asarray([np.nan if v is None else v for v in d[k]], float)
        return cls(np.asarray(columns), np.asarray(d['b_kind'], int), f('b_lo'), f('b_hi'),
                   np.asarray(d['t_kind'], int), f('t_lo'), f('t_hi'))


def strip_probs(probs: np.ndarray, columns: np.ndarray, half: int) -> np.ndarray:
    """(H, C, K) class probabilities averaged over each column strip; invalid rows NaN.

    ``probs``: (H, W, K) float probabilities (or one-hot of a label map).
    """
    out = np.empty((probs.shape[0], len(columns), probs.shape[2]), np.float32)
    for j, u in enumerate(columns):
        lo, hi = max(0, u - half), min(WIDTH, u + half + 1)
        out[:, j] = probs[:, lo:hi].mean(1)
        bad = ~VALID[:, lo:hi].all(1)
        out[bad, j] = np.nan
    return out


def one_hot(label: np.ndarray) -> np.ndarray:
    """(H, W, 5) one-hot of a label map; IGNORE -> all zeros (later NaN via VALID)."""
    out = np.zeros(label.shape + (len(CLASSES),), np.float32)
    for c in range(len(CLASSES)):
        out[..., c] = label == c
    return out


def _runs(lab: np.ndarray):
    """[(class, start_row, end_row_inclusive)] of a column label sequence (top -> bottom)."""
    out = []
    start = 0
    for v in range(1, len(lab) + 1):
        if v == len(lab) or lab[v] != lab[start]:
            out.append((int(lab[start]), start, v - 1))
            start = v
    return out


def _crossing(p: np.ndarray, v_above: int, a: int, b: int) -> float:
    """Sub-pixel row between ``v_above`` (class a) and ``v_above + 1`` (class b)."""
    d0 = p[v_above, a] - p[v_above, b]
    d1 = p[v_above + 1, a] - p[v_above + 1, b]
    if d0 > 0 > d1 and d0 - d1 > 1e-6:
        return float(v_above + d0/(d0 - d1))
    return v_above + .5


def column_observations(probs: np.ndarray, columns: np.ndarray, params: Mapping | None = None) -> ColumnObs:
    """Interval observations of every column from class probabilities (H, W, K) (see ``ColumnObs``)."""
    p_ = {**DEFAULT_OBS, **(params or {})}
    sp = strip_probs(probs, columns, int(p_['strip_half_px']))
    r_min = int(p_['min_run_px'])
    n = len(columns)
    b_kind, t_kind = np.zeros(n, int), np.zeros(n, int)
    b_lo, b_hi, t_lo, t_hi = (np.full(n, np.nan) for _ in range(4))
    for j in range(n):
        p = sp[:, j]
        ok = np.isfinite(p[:, 0])
        rows = np.flatnonzero(ok)
        if rows.size < 2*r_min:
            continue
        top, bot = int(rows[0]), int(rows[-1])
        lab = np.argmax(np.nan_to_num(p[top:bot + 1]), 1)
        runs = [(c, s + top, e + top) for c, s, e in _runs(lab) if e - s + 1 >= r_min]
        if not runs:
            continue
        # merge adjacent runs of one class separated by dropped short runs
        merged = [list(runs[0])]
        for c, s, e in runs[1:]:
            if c == merged[-1][0]:
                merged[-1][2] = e
            else:
                merged.append([c, s, e])
        walls = [r for r in merged if r[0] == WALL]
        floors = [r for r in merged if r[0] == FLOOR]
        if walls:
            w = walls[-1]                               # lowest wall run
            below = [r for r in merged if r[1] > w[2]]
            if below and below[0][0] == FLOOR and below[0][1] - w[2] <= 3:
                # wall directly above floor: a sharp edge (short dropped runs in between: midpoint)
                v = _crossing(p, w[2], WALL, FLOOR) if below[0][1] == w[2] + 1 else (w[2] + below[0][1])/2
                b_kind[j], b_lo[j], b_hi[j] = EDGE, v, v
            else:
                # own arm / carried box / peer / box in between, or wall down to the image bottom:
                # the bottom edge is hidden somewhere between the wall run and the next floor run
                floor_below = [r for r in below if r[0] == FLOOR]
                hi = floor_below[0][1] - .5 if floor_below else POS_INF
                b_kind[j], b_lo[j], b_hi[j] = INTERVAL, w[2] + .5, hi
            if p_['use_top_edge']:
                above = [r for r in merged if r[2] < w[1]]
                if above and above[-1][0] == BACKGROUND:
                    if above[-1][2] == w[1] - 1:
                        v = _crossing(p, above[-1][2], BACKGROUND, WALL)
                        t_kind[j], t_lo[j], t_hi[j] = EDGE, v, v
                elif not above and w[1] <= top + 1:
                    t_kind[j], t_lo[j], t_hi[j] = INTERVAL, NEG_INF, w[1] - .5    # top edge above the view
        elif floors:
            f = floors[0]                               # highest floor run
            above = [r for r in merged if r[2] < f[1]]
            if not above or above[-1][0] == BACKGROUND:
                b_kind[j], b_lo[j], b_hi[j] = INTERVAL, NEG_INF, f[1] - .5       # free floor to the top
    if p_.get('consistency_px') is not None:
        tol = float(p_['consistency_px'])
        for kind, lo, hi in ((b_kind, b_lo, b_hi), (t_kind, t_lo, t_hi)):
            drop = _inconsistent_edges(kind, lo, hi, tol)
            kind[drop] = NONE
            lo[drop] = hi[drop] = np.nan
    return ColumnObs(np.asarray(columns), b_kind, b_lo, b_hi, t_kind, t_lo, t_hi)


def _inconsistent_edges(kind: np.ndarray, lo: np.ndarray, hi: np.ndarray, tol: float) -> np.ndarray:
    """Sharp edges that are not bottom (top) edges of one wall face within the column.

    The camera is pitched, so an image column is not a vertical plane: next to a
    vertical wall edge (a door jamb, a wall end) its upper rays can hit the wall
    while its lower rays pass the edge and hit the floor behind it. That wall/floor
    transition is the vertical edge, not the wall's bottom edge the floor-trace
    model predicts. Such columns sit at a discontinuity: a sharp edge is dropped
    when it is not explained by its neighbours -- both neighbours sharp edges and
    the edge deviates by more than ``tol`` from their mean (straight or slanted
    bottom edges pass), or a neighbour interval does not contain it (+- tol).
    Neighbour columns keep the jump itself.
    """
    n = len(kind)
    drop = np.zeros(n, bool)
    for j in np.flatnonzero(kind == EDGE):
        nb = [i for i in (j - 1, j + 1) if 0 <= i < n and kind[i] != NONE]
        if not nb:
            continue
        edges = [i for i in nb if kind[i] == EDGE]
        if len(edges) == 2:
            drop[j] = abs(lo[j] - .5*(lo[edges[0]] + lo[edges[1]])) > tol
        elif len(edges) == 1 and len(nb) == 1:
            drop[j] = False                          # one sharp neighbour only: a straight edge may be steep
        for i in nb:
            if kind[i] == INTERVAL and not (lo[i] - tol <= lo[j] <= hi[i] + tol):
                drop[j] = True
    return drop


# ----------------------------------------------------------------------------- likelihood
def _erf(x: np.ndarray) -> np.ndarray:
    # Abramowitz-Stegun 7.1.26 (|error| < 1.5e-7), vectorised
    s = np.sign(x)
    x = np.abs(x)
    t = 1./(1. + .3275911*x)
    y = 1. - (((((1.061405429*t - 1.453152027)*t) + 1.421413741)*t - .284496736)*t + .254829592)*t*np.exp(-x*x)
    return s*y


def interval_prob(expected: np.ndarray, kind: np.ndarray, lo: np.ndarray, hi: np.ndarray, sigma: float) -> np.ndarray:
    """P(observation | expected row) per (particle, column), max 1; NaN where kind == NONE.

    EDGE: exp(-0.5 d^2/sigma^2) (unnormalised Gaussian). INTERVAL: probability that
    the expected row plus N(0, sigma^2) noise falls in [lo, hi].
    """
    e = np.where(np.isfinite(expected), expected, POS_INF)
    out = np.full(np.broadcast_shapes(e.shape, lo.shape), np.nan)
    edge = kind == EDGE
    iv = kind == INTERVAL
    if edge.any():
        d = (lo[None, edge] - e[:, edge])/sigma
        out[:, edge] = np.exp(-.5*d*d)
    if iv.any():
        a = (lo[None, iv] - e[:, iv])/sigma
        b = (hi[None, iv] - e[:, iv])/sigma
        pa = .5*(1 + _erf(np.clip(a, -30, 30)/math.sqrt(2)))
        pb = .5*(1 + _erf(np.clip(b, -30, 30)/math.sqrt(2)))
        out[:, iv] = np.maximum(pb - pa, 0.)
    return out


def column_loglik(vb_exp: np.ndarray, vt_exp: np.ndarray, obs: ColumnObs, params: Mapping) -> np.ndarray:
    """Robust per-particle log-likelihood of one frame's column observations."""
    m = {**DEFAULT_MEASUREMENT, **params}
    eps = float(m['outlier_prob'])
    n_cols = int(obs.informative.sum())
    if n_cols < int(m['min_columns']):
        return np.zeros(vb_exp.shape[0])
    terms = []
    pb = interval_prob(vb_exp, obs.b_kind, obs.b_lo, obs.b_hi, float(m['sigma_px']))
    terms.append((pb[:, obs.b_kind != NONE], 1.))
    if params.get('use_top_edge', True) and (obs.t_kind != NONE).any():
        pt = interval_prob(vt_exp, obs.t_kind, obs.t_lo, obs.t_hi, float(m['sigma_px']))
        terms.append((pt[:, obs.t_kind != NONE], float(m['top_weight'])))
    total = np.zeros(vb_exp.shape[0])
    n_terms = 0
    for p, w in terms:
        if p.shape[1] == 0:
            continue
        total += w*np.log(eps + (1 - eps)*p).sum(1)
        if m.get('scale_by', 'all') == 'discriminative':
            n_terms += int(((p.max(0) - p.min(0)) > float(m['discriminative_min'])).sum())
        else:
            n_terms += p.shape[1]
    return total*min(1., float(m['effective_columns'])/max(n_terms, 1))


# ----------------------------------------------------------------------------- expected rows
def _rects(geometry) -> np.ndarray:
    return np.asarray(geometry.rects, float)[:, :4]


def _segment_hits(ox, oy, ex, ey, rect) -> np.ndarray:
    """Segment (o -> e) meets the interior of an axis-aligned rectangle (cx, cy, hx, hy) (slab test)."""
    cx, cy, hx, hy = rect
    dx, dy = ex - ox, ey - oy
    dx = np.where(np.abs(dx) < 1e-9, 1e-9, dx)
    dy = np.where(np.abs(dy) < 1e-9, 1e-9, dy)
    tx1, tx2 = (cx - hx - ox)/dx, (cx + hx - ox)/dx
    ty1, ty2 = (cy - hy - oy)/dy, (cy + hy - oy)/dy
    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    return (tmax > np.maximum(tmin, 0.) + 1e-9) & (tmin < 1.)


def first_blocked(rects: np.ndarray, ox, oy, qx, qy, dx, dy, t_min: float | np.ndarray = 0.,
                  corners: set | None = None) -> np.ndarray:
    """Smallest t >= t_min at which the segment from o to q + t*d meets a footprint (inf if never).

    Walls at least as tall as the camera block a ray to a floor point F (or to a
    point at the walls' top height) exactly when the horizontal segment from the
    camera to F crosses a wall footprint. Moving F outward along a column's trace,
    the first contact is either F entering a footprint or the segment sweeping
    over a footprint corner (convex polygons), so both event sets are tested.
    Shapes broadcast over (particles, columns). ``corners``: the (rect index, corner
    index) pairs to test (None: all); ``expected_rows`` passes only corners inside
    the particle cloud's view fan (an exact pruning, see ``_fan_corners``).
    """
    # float32: ~2x faster on (particles x columns) arrays; 5 m coordinates keep ~1 um resolution
    ox, oy, qx, qy, dx, dy = np.broadcast_arrays(*(np.asarray(a, np.float32) for a in (ox, oy, qx, qy, dx, dy)))
    t_min = np.broadcast_to(np.asarray(t_min, np.float32), ox.shape)
    best = np.full(ox.shape, np.inf, np.float32)
    sx, sy = qx + t_min*dx, qy + t_min*dy
    # already blocked at t_min: only footprints within reach of the first segment can do that
    reach = float(np.sqrt(np.max((sx - ox)**2 + (sy - oy)**2))) if ox.size else 0.
    blocked0 = np.zeros(ox.shape, bool)
    if reach > 1e-6:
        for r in rects:
            cx, cy, hx, hy = r
            gap = np.hypot(np.maximum(np.abs(ox - cx) - hx, 0.), np.maximum(np.abs(oy - cy) - hy, 0.))
            if float(gap.min()) <= reach:
                blocked0 |= _segment_hits(ox, oy, sx, sy, r)
    ddx = np.where(np.abs(dx) < 1e-9, 1e-9, dx)
    ddy = np.where(np.abs(dy) < 1e-9, 1e-9, dy)
    rx, ry = qx - ox, qy - oy
    for ri, r in enumerate(rects):
        cx, cy, hx, hy = r
        # (1) the trace point enters the footprint (slab test on the line q + t d, t >= t_min)
        tx1, tx2 = (cx - hx - qx)/ddx, (cx + hx - qx)/ddx
        ty1, ty2 = (cy - hy - qy)/ddy, (cy + hy - qy)/ddy
        tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
        tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
        enter = np.maximum(tmin, t_min)
        best = np.where((tmax > enter) & (enter < best), enter, best)
        # (2) the segment sweeps over a corner c: o + mu (c - o) = q + t d with mu >= 1
        for ki, (kx, ky) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            if corners is not None and (ri, ki) not in corners:
                continue
            ax, ay = cx + kx*hx - ox, cy + ky*hy - oy
            det = -ax*dy + ay*dx
            ok = np.abs(det) > 1e-9
            det = np.where(ok, det, 1.)
            mu = (-rx*dy + ry*dx)/det
            t = (ax*ry - ay*rx)/det
            idx = np.flatnonzero(ok & (mu >= 1.) & (t >= t_min) & (t < best))
            if idx.size == 0:
                continue
            te = t.flat[idx] + 1e-4
            hit = _segment_hits(ox.flat[idx], oy.flat[idx], qx.flat[idx] + te*dx.flat[idx],
                                qy.flat[idx] + te*dy.flat[idx], r)
            best.flat[idx[hit]] = t.flat[idx[hit]]
    return np.where(blocked0, -np.inf, best)


def _fan_corners(rects: np.ndarray, ox, oy, dirs_x, dirs_y, margin_rad: float = math.radians(3.)) -> set:
    """Footprint corners that can lie inside some particle's view fan.

    A corner event needs the corner on a line of sight to a trace point, i.e.
    inside the horizontal fan spanned by the sight directions of all columns
    (``dirs``, shape (P, C, M)). A corner is kept when, for some particle, its
    bearing lies within [min, max] of that particle's sight bearings (+ margin).
    """
    keep = set()
    ang = np.arctan2(dirs_y, dirs_x)                      # (P, C*M)
    ref = ang[:, :1]
    rel = (ang - ref + np.pi) % (2*np.pi) - np.pi
    lo, hi = rel.min(1) - margin_rad, rel.max(1) + margin_rad
    for ri, (cx, cy, hx, hy) in enumerate(rects):
        for ki, (kx, ky) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            b = np.arctan2(cy + ky*hy - oy[:, 0], cx + kx*hx - ox[:, 0])
            r = (b - ref[:, 0] + np.pi) % (2*np.pi) - np.pi
            if np.any((r >= lo) & (r <= hi)):
                keep.add((ri, ki))
    return keep


def expected_rows(geometry, poses: np.ndarray, cm: 'ColumnModelDZ', wall_height_m: float = WALL_HEIGHT_M):
    """Expected (bottom, top) edge rows (P, C) of the walls in each column, for walls >= camera height.

    Reuses PR #210's column traces (``ColumnModel.q0/d/trace_at/rows/rows_at``) and
    footprints (``MapGeometry.rects``). PR #210 took the first footprint ALONG the
    floor trace (cast from 0.6 m behind the lowest visible floor point). That is
    exact for a vertical column plane only: the camera is pitched, so next to a
    vertical wall edge (door jamb, wall end) the upper rays of a column hit the
    wall while its trace passes the edge. Here the bottom edge is the first floor
    trace point whose line of sight from the camera is blocked by a footprint
    (``first_blocked``), the top edge likewise on the trace at the wall-top height.
    A wall behind the camera never counts. Bottom already blocked at the lowest
    visible row -> POS_INF (the wall hides the image bottom); bottom never
    blocked -> NEG_INF (open floor up to the horizon); top never blocked or behind
    the image plane -> NEG_INF (the top edge is above the view).
    """
    poses = np.asarray(poses, float).reshape(-1, 3)
    if np.any(geometry.rects[:, 4] < cm.origin[2] - 1e-6):
        raise ValueError('expected_rows assumes walls at least as tall as the camera')
    rects = _rects(geometry).astype(np.float32)
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    dx = c*cm.d[None, :, 0] - s*cm.d[None, :, 1]
    dy = s*cm.d[None, :, 0] + c*cm.d[None, :, 1]
    o = cm.origin[:2]
    ox = poses[:, :1] + c*o[0] - s*o[1]
    oy = poses[:, 1:2] + s*o[0] + c*o[1]

    def world(q):
        return poses[:, :1] + c*q[None, :, 0] - s*q[None, :, 1], poses[:, 1:2] + s*q[None, :, 0] + c*q[None, :, 1]
    qx, qy = world(cm.q0)
    q0h, _ = cm.trace_at(wall_height_m)
    hx, hy = world(q0h)
    s_cam = np.sum((o[None, :] - q0h)*cm.d, 1)[None, :]   # top-trace point level with the camera
    # sight directions: to the nearest bottom-trace point, to the top-trace point level with the
    # camera and along the traces (far points); the fan of each particle spans all of them
    sight = [(qx - ox, qy - oy), (hx + s_cam*dx - ox, hy + s_cam*dy - oy), (dx, dy)]
    fan = _fan_corners(rects, ox, oy, np.concatenate([a for a, _ in sight], 1),
                       np.concatenate([b for _, b in sight], 1))
    t = first_blocked(rects, ox, oy, qx, qy, dx, dy, 0., fan).astype(float)
    with np.errstate(invalid='ignore'):
        vb = np.where(np.isfinite(t), cm.rows(np.where(np.isfinite(t), t, 0.)), np.nan)
    vb = np.where(t == -np.inf, POS_INF, vb)          # blocked at the lowest visible floor row
    vb = np.where(np.isnan(vb), NEG_INF, vb)          # never blocked (open view): no wall bottom in the column
    st = first_blocked(rects, ox, oy, hx, hy, dx, dy, s_cam, fan).astype(float)
    ok = np.isfinite(st)
    with np.errstate(invalid='ignore'):
        vt = np.where(ok, cm.rows_at(np.where(ok, st, 0.), wall_height_m), np.nan)
    return vb, np.nan_to_num(vt, nan=NEG_INF)


# ----------------------------------------------------------------------------- particle filter
def make_vision_pf(m1_module, static_map: Mapping, params: Mapping, measurement: Mapping, obs_params: Mapping,
                   sag_table: Mapping, seed: int, pan_table: Mapping | None = None):
    """Subclass of the M1 ``OwnCamLocalizer`` whose measurement is the segmentation column scan."""
    base = m1_module.OwnCamLocalizer

    class VisionScanLocalizer(base):
        def __init__(self):
            super().__init__(static_map, params, seed=seed)
            self.geometry = mp.MapGeometry(static_map, include_posts=False)
            self.measurement = {**DEFAULT_MEASUREMENT, **measurement}
            self.obs_params = {**DEFAULT_OBS, **obs_params}
            self.columns = column_positions(int(self.obs_params['columns']), int(self.obs_params['strip_half_px']))
            self.sag_table = sag_table
            self.pan_table = pan_table or {}
            self.last_scan_t = None
            self.own_servo_cmd_t = -1e9            # own arm / pan command time (settle gate)
            self.stats.update(scan_updates=0, scan_columns=0, unsettled_skips=0)

        def command(self, row: Mapping) -> None:
            super().command(row)
            if row['kind'] in ('initial_servo_command', 'arm', 'look'):
                self.own_servo_cmd_t = float(row['t'])

        def init_gaussian(self, mean: Sequence[float], std: Sequence[float]):
            self.px = np.asarray(mean, float)[None, :] + self.rng.normal(size=(self.n, 3))*np.asarray(std, float)
            self.px[:, 2] = m1_module.wrap(self.px[:, 2])
            sd = self.params['motion']['scale_std']
            self.scale = 1. + self.rng.normal(size=(self.n, 3))*sd
            self.logw = self._map_logprior(self.px)
            self.initialized = True

        def column_model_for(self, pose: Mapping, columns: np.ndarray | None = None):
            b, dz = sag(self.sag_table, self.load.loaded, pose)
            return column_model(pose, b, dz, self.columns if columns is None else columns,
                                pan_yaw(self.pan_table, self.load.loaded, pose))

        def estimate(self) -> dict:
            """The M1 estimate of the nominal base, plus the current pan-induced chassis yaw offset."""
            est = super().estimate()
            if est.get('initialized') and self.servo:
                off = pan_yaw(self.pan_table, self.load.loaded, self.servo)
                est['yaw'] = float(m1_module.wrap(est['yaw'] + off))
                est['pan_yaw_offset'] = off
            return est

        def expected(self, px: np.ndarray, pose: Mapping):
            return expected_rows(self.geometry, px, self.column_model_for(pose))

        def settled(self, t: float) -> bool:
            return t - self.own_servo_cmd_t >= float(self.measurement['settle_s']) - 1e-9

        def update_obs(self, t: float, obs: ColumnObs | None, pose: Mapping) -> dict:
            self.predict_to(t)
            used = False
            if self.initialized and obs is not None:
                if not self.settled(t):
                    self.stats['unsettled_skips'] += 1
                elif int(obs.informative.sum()) >= int(self.measurement['min_columns']):
                    vb, vt = self.expected(self.px, pose)
                    self.logw = self.logw + column_loglik(vb, vt, obs, {**self.measurement,
                                                                         'use_top_edge': self.obs_params['use_top_edge']})
                    self.stats['scan_updates'] += 1
                    self.stats['scan_columns'] += int(obs.informative.sum())
                    self.last_scan_t = t
                    used = True
            if self.initialized:
                self._normalize_and_resample()
            est = self.estimate()
            est['since_scan_s'] = None if self.last_scan_t is None else round(t - self.last_scan_t, 3)
            est['measured'] = used
            return est

    return VisionScanLocalizer()


# ----------------------------------------------------------------------------- episode inputs (student view)
def read_jsonl(path: Path) -> list[dict]:
    with open(path) as fh:
        return [json.loads(line) for line in fh if line.strip()]


def student_inputs(ep_dir: Path) -> dict:
    """Robot-side inputs of one rendered episode (never ``eval_only/`` or ``teacher/``)."""
    ep_dir = Path(ep_dir)
    return {'commands': read_jsonl(ep_dir/'inputs'/'commands.jsonl'),
            'frames': read_jsonl(ep_dir/'inputs'/'frames.jsonl'),
            'profile_events': [(float(r['t']), None if r.get('profile') in (None, 'default') else r['profile'])
                               for r in read_jsonl(ep_dir/'inputs'/'motion_profile.jsonl')]}


def replay(ep_dir: Path, sinks: Sequence, *, on_frame=None, frame_filter=None) -> int:
    """Own commands, motion-profile switches and frames to each sink in time order.

    Same ordering as PR #210 ``markerless_probe.replay`` (events stamped at a
    frame's time are applied after that frame). ``sinks`` have ``command(row)``,
    ``set_motion_profile(t, name)`` and ``frame(t, bgr, row)``.
    """
    data = student_inputs(ep_dir)
    cmds, frames, prof = data['commands'], data['frames'], data['profile_events']
    ci = pi = 0
    for k, row in enumerate(frames):
        t = float(row['t'])
        while True:
            tc = float(cmds[ci]['t']) if ci < len(cmds) else math.inf
            tp = prof[pi][0] if pi < len(prof) else math.inf
            if min(tc, tp) >= t - 1e-9:
                break
            if tp < tc:
                for s in sinks:
                    s.set_motion_profile(tp, prof[pi][1])
                pi += 1
            else:
                for s in sinks:
                    s.command(cmds[ci])
                ci += 1
        if frame_filter is not None and not frame_filter(k, row):
            continue
        bgr = cv2.imread(str(Path(ep_dir)/row['file']), cv2.IMREAD_COLOR)
        for s in sinks:
            s.frame(t, bgr, row)
        if on_frame is not None:
            on_frame(k, row)
    return len(frames)
