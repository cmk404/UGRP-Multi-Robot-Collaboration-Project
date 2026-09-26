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
}
DEFAULT_MEASUREMENT = {
    'sigma_px': 2.5,           # row noise of the expected edge (extrinsic + label)
    'outlier_prob': .1,
    'effective_columns': 12.,  # columns in one frame share the extrinsic error
    'min_columns': 6,          # fewer informative columns: no update
    'settle_s': .4,            # measurement only this long after the last own arm/pan command
    'top_weight': 1.,
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
    """PR #210 ``ColumnModel`` with a fixed camera height correction ``dz`` (base frame)."""
    dz: float = 0.

    def __post_init__(self):
        orig = mp.camera_in_base
        dz = float(self.dz)

        def shifted(pose):
            o, r = orig(pose)
            return o + np.array([0., 0., dz]), r
        mp.camera_in_base = shifted
        try:
            super().__post_init__()
        finally:
            mp.camera_in_base = orig


_CM_CACHE: dict = {}


def column_model(servo: Mapping, bias: float, dz: float, columns: np.ndarray) -> ColumnModelDZ:
    key = (tuple(sorted((int(k), int(v)) for k, v in servo.items())), round(float(bias), 6), round(float(dz), 5),
           tuple(int(c) for c in columns))
    if key not in _CM_CACHE:
        if len(_CM_CACHE) > 1024:
            _CM_CACHE.clear()
        _CM_CACHE[key] = ColumnModelDZ(key[0], float(bias), np.asarray(columns), dz=float(dz))
    return _CM_CACHE[key]


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
    return ColumnObs(np.asarray(columns), b_kind, b_lo, b_hi, t_kind, t_lo, t_hi)


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
    total = np.zeros(vb_exp.shape[0])
    pb = interval_prob(vb_exp, obs.b_kind, obs.b_lo, obs.b_hi, float(m['sigma_px']))
    use = obs.b_kind != NONE
    total += np.log(eps + (1 - eps)*pb[:, use]).sum(1)
    n_terms = int(use.sum())
    if params.get('use_top_edge', True) and (obs.t_kind != NONE).any():
        pt = interval_prob(vt_exp, obs.t_kind, obs.t_lo, obs.t_hi, float(m['sigma_px']))
        ut = obs.t_kind != NONE
        total += float(m['top_weight'])*np.log(eps + (1 - eps)*pt[:, ut]).sum(1)
        n_terms += int(ut.sum())
    return total*min(1., float(m['effective_columns'])/max(n_terms, 1))


# ----------------------------------------------------------------------------- expected rows
def expected_rows(geometry, poses: np.ndarray, cm: 'ColumnModelDZ', wall_height_m: float = WALL_HEIGHT_M):
    """Expected (bottom, top) edge rows (P, C) of the first map footprint in front of the camera.

    Adapted from PR #210 ``MapGeometry.expected_rows`` (same slab ray cast
    ``MapGeometry.raycast`` and column traces). Change: every trace is cast from
    the point level with the camera (its projection on the trace) instead of a
    fixed 0.6 m behind the lowest visible floor point, so a wall behind the robot
    is never taken as the first footprint, and for walls taller than the camera
    the top trace starts at the camera (PR #210 cast it from ~0.7 m ahead, which
    misses near walls). No hit / behind the image plane: bottom -> POS_INF (the
    footprint hides the image bottom), top -> NEG_INF (above the view).
    """
    poses = np.asarray(poses, float).reshape(-1, 3)
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    dx = c*cm.d[None, :, 0] - s*cm.d[None, :, 1]
    dy = s*cm.d[None, :, 0] + c*cm.d[None, :, 1]
    o = cm.origin[:2]

    def cast(q0):
        back = np.sum((q0 - o[None, :])*cm.d, 1)            # distance from the camera level point to q0
        start = q0 - back[:, None]*cm.d
        ox = poses[:, :1] + c*start[None, :, 0] - s*start[None, :, 1]
        oy = poses[:, 1:2] + s*start[None, :, 0] + c*start[None, :, 1]
        t, h = geometry.raycast(ox, oy, dx, dy)
        return t - back[None, :], h
    t, _ = cast(cm.q0)
    fin = np.isfinite(t)
    with np.errstate(invalid='ignore'):
        vb = np.where(fin, cm.rows(np.where(fin, t, 0.)), np.nan)
    q0h, _ = cm.trace_at(wall_height_m)
    st, sh = cast(q0h)
    ok = np.isfinite(st) & (np.abs(sh - wall_height_m) < 1e-6)
    with np.errstate(invalid='ignore'):
        vt = np.where(ok, cm.rows_at(np.where(ok, st, 0.), wall_height_m), np.nan)
    return np.nan_to_num(vb, nan=POS_INF), np.nan_to_num(vt, nan=NEG_INF)


# ----------------------------------------------------------------------------- particle filter
def make_vision_pf(m1_module, static_map: Mapping, params: Mapping, measurement: Mapping, obs_params: Mapping,
                   sag_table: Mapping, seed: int):
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

        def column_model_for(self, pose: Mapping):
            b, dz = sag(self.sag_table, self.load.loaded, pose)
            return column_model(pose, b, dz, self.columns)

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
