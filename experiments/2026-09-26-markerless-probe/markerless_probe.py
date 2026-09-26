"""Tag-free own-camera localization probe on recorded M1 wrist frames (offline).

Question (user, 2026-09-26): do robots need AprilTags on door posts and zone
entrances, or can the static map geometry itself localize them?

Pipeline for every own wrist frame:

1. raw fisheye JPEG -> undistorted pinhole image (fixed measured K and D);
2. per-column floor/wall-boundary detector. A map wall is a vertical face of
   known height (0.10 m). For a candidate bottom edge at row ``vb`` the ground
   plane gives its range, and the range gives the row ``vt`` where the wall's
   top edge must appear. The candidate is accepted when the band between the
   two rows is uniform and differs from the floor below it (and, when the top
   is in view, from what is above it). The lowest accepted band in a column is
   the floor/wall boundary;
3. pseudo range scan: the accepted ``vb`` back-projected to the floor with the
   commanded-PWM extrinsics and a fixed elevation bias;
4. beam likelihood of the observed rows against rows ray cast from the static
   map (wall and door-post footprints) for every particle, inside the M1
   particle filter (same motion model, calibration and motion-profile
   switches as the online tag PF of the M1 runs).

Robot inputs only: the robot's own frames, its own issued commands, the static
map's walls/door posts/bounds (``landmarks.tags`` is emptied before use) and
fixed calibrations fitted offline on the dev split. The recorded frames still
show the physical tags. Nothing here detects or decodes them; columns through
a tag plate fail the band-uniformity test and are dropped. Ground truth
(``eval_only/``) is read only by the ``calibrate`` and ``score`` steps of
``run_probe.py``, never by this module.

The M1 localizer is loaded byte-for-byte from the M1 branch commit (PR #201),
verified against the runtime hash recorded in the M1 manifests.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import subprocess
import types
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from harness.wall_tags import camera_in_base
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = 'ugrp.markerless_probe.v1'

# M1 branch (PR #201, claude/zone-m1-owncam) commit whose localizer ran the M1 test cohort.
M1_SHA = '22c84842b507264de496b54b1e8c9217701b9b5c'
M1_LOCALIZER = 'harness/owncam_localizer.py'
M1_LOCALIZER_SHA256 = '0304d7c491dfe6ae68cea6550f7a13e3c99e8e1d3f8e8b6c4b7b1d06893b1d63'
M1_MAP = 'maps/zones/zone_wide_door_tags_v2.json'
M1_CALIBRATION = 'experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json'
M1_CALIBRATION_SHA256 = '126cadaa9265ed2ae2b034f675e67c227193a2e1678f6b0c82dd53b186e6fc72'
# Files the M1 localizer imports from this tree; they must equal the M1 runtime files.
SHARED_RUNTIME_SHA256 = {
    'harness/wall_tags.py': 'eb0327012047cc413b7bb930fb7884cdfe4c18239058d954b1cce56593e0336e',
}

WIDTH, HEIGHT = 640, 480
K = scaled_camera_matrix(WIDTH, HEIGHT)
K_INV = np.linalg.inv(K)
FX, FY, CX, CY = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
_UNDISTORT = cv2.fisheye.initUndistortRectifyMap(K, np.asarray(CAMERA_FISHEYE_D, float).reshape(4, 1),
                                                 np.eye(3), K, (WIDTH, HEIGHT), cv2.CV_32FC1)
T_BACK_M = .6          # ray casts start this far behind the bottom-row floor point

DEFAULT_DETECTOR = {
    'columns': 48,             # evenly spaced image columns
    'strip_half_px': 3,        # each column averages 2*3+1 image columns
    'wall_height_m': .10,      # static map: every wall is 0.10 m high
    'min_range_m': .12,        # floor distance from the camera nadir
    'max_range_m': 6.,
    'min_band_px': 4.,         # shorter implied bands are not tested
    'edge_margin_px': 1.5,     # band rows this close to an edge are excluded
    'window_px': 3,            # floor window below / window above the band
    'chroma_weight': 1.,       # (B-R) contrast weight next to luminance
    'min_contrast_below': 6.,  # 8-bit levels
    'min_contrast_above': 4.,
    'max_band_std': 3.,
    'min_open_band_px': 12.,   # band that runs out of the image top
    'self_margin_px': 3,       # stay this far above the carried-object mask
    'candidates': 3,           # non-overlapping bands kept per column, lowest first
    'top_tol_px': 1.,          # implied top row tolerance: px + fraction of the band height
    'top_tol_frac': .15,
}

DEFAULT_MEASUREMENT = {
    'sigma_px': 3.,            # row noise of observed vs expected edges
    'outlier_prob': .2,
    'outlier_margin': 3.,      # robust floor below log(outlier_prob)
    'use_top_edge': True,
    'effective_columns': 8.,   # columns in one frame share the extrinsic error
    'min_columns': 4,          # fewer detections: no update
    'use_paint': False,        # painted floor-marking edges as alternative explanations
    # Fixed elevation bias (camera sag below the commanded-PWM FK), dev fit per own
    # load state and arm pose: {'s3': [...], 'bias': [...]} interpolated over the
    # commanded shoulder pulse (servo 3), or a single number.
    'bias_rad': {'unloaded': 0., 'loaded': 0.},
}


def elevation_bias(table, servo: Mapping) -> float:
    """Bias for the commanded arm pose from a per-pose table (or a constant)."""
    if isinstance(table, Mapping):
        return float(np.interp(float(servo[3]), np.asarray(table['s3'], float), np.asarray(table['bias'], float)))
    return float(table)


# ----------------------------------------------------------------------------- provenance
def git_blob(sha: str, path: str) -> bytes:
    return subprocess.run(['git', 'show', f'{sha}:{path}'], cwd=ROOT, capture_output=True, check=True).stdout


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def load_m1_localizer():
    """The M1 ``owncam_localizer`` module, loaded from the M1 commit and hash-checked."""
    code = git_blob(M1_SHA, M1_LOCALIZER)
    if sha256(code) != M1_LOCALIZER_SHA256:
        raise RuntimeError('M1 localizer blob hash differs from the M1 runtime hash')
    for rel, want in SHARED_RUNTIME_SHA256.items():
        if sha256((ROOT/rel).read_bytes()) != want:
            raise RuntimeError(f'{rel} differs from the file the M1 runs imported')
    module = types.ModuleType('owncam_localizer_m1')
    module.__file__ = f'{M1_SHA[:8]}:{M1_LOCALIZER}'
    exec(compile(code, module.__file__, 'exec'), module.__dict__)
    return module


def load_m1_map() -> tuple[dict, dict]:
    """(markerless map view, provenance). Tags are removed; walls, posts and bounds stay."""
    blob = git_blob(M1_SHA, M1_MAP)
    full = json.loads(blob)
    view = copy.deepcopy(full)
    view['landmarks'] = {'tags': [], 'door_posts': full['landmarks'].get('door_posts', []),
                         'note': 'markerless probe view: tag list removed'}
    return view, {'source': f'{M1_SHA}:{M1_MAP}', 'file_sha256': sha256(blob), 'map_id': full['map_id'],
                  'base_map': full.get('base_map'), 'tags_removed': len(full['landmarks']['tags'])}


def load_m1_calibration() -> tuple[dict, dict]:
    blob = git_blob(M1_SHA, M1_CALIBRATION)
    if sha256(blob) != M1_CALIBRATION_SHA256:
        raise RuntimeError('M1 calibration hash differs from the M1 manifests')
    return json.loads(blob), {'source': f'{M1_SHA}:{M1_CALIBRATION}', 'file_sha256': sha256(blob)}


# ----------------------------------------------------------------------------- camera
def undistort(bgr: np.ndarray) -> np.ndarray:
    """Raw fisheye frame -> pinhole image with the same K (the sim renders pinhole then distorts)."""
    return cv2.remap(bgr, _UNDISTORT[0], _UNDISTORT[1], cv2.INTER_LINEAR)


def bias_rotation(bias_rad: float) -> np.ndarray:
    """Rotation in the optical y-z plane: observed elevation = predicted elevation + bias."""
    c, s = math.cos(bias_rad), math.sin(bias_rad)
    return np.array([[1., 0., 0.], [0., c, s], [0., -s, c]])


def column_positions(n: int, half: int = 3) -> np.ndarray:
    return np.linspace(8 + half, WIDTH - 9 - half, int(n)).round().astype(int)


@dataclass
class ColumnModel:
    """Per-column floor trace of one commanded arm pose (base frame, robot at the origin).

    Column ``u`` sees floor points ``q0 + t*d`` (t >= 0 moves away from the
    robot); ``alpha + t*beta`` are their corrected camera coordinates, so the
    image row is ``CY + FY*(alpha_y + t*beta_y)/(alpha_z + t*beta_z)``. The
    column's rays cut every horizontal plane ``z = h`` in a line parallel to
    the floor trace (``trace_at``); a wall's top edge seen in column ``u`` lies
    on that line, not vertically above the bottom-edge point (the column plane
    is tilted by the camera pitch).
    """
    servo: tuple
    bias_rad: float
    columns: np.ndarray
    origin: np.ndarray = field(init=False)
    q0: np.ndarray = field(init=False)
    d: np.ndarray = field(init=False)
    alpha: np.ndarray = field(init=False)
    beta: np.ndarray = field(init=False)
    gamma: np.ndarray = field(init=False)

    def __post_init__(self):
        o, r_bc = camera_in_base(dict(self.servo))
        rot = r_bc @ bias_rotation(self.bias_rad).T        # corrected optical -> base
        self.origin = o
        self._rot = rot
        u = self.columns.astype(float)
        a = rot @ (K_INV @ np.stack([u, np.zeros_like(u), np.ones_like(u)]))   # ray(v) = a + v*b
        b = rot @ (K_INV @ np.array([0., 1., 0.]))
        v_bottom = HEIGHT - 1.
        v_h = -a[2]/b[2]                                     # horizon row of each column
        v_far = np.minimum(v_bottom - 1., v_h + 25.)
        q = []
        for v in (np.full_like(u, v_bottom), v_far):
            ray = a + v[None, :]*b[:, None]
            if np.any(ray[2] >= 0):
                raise ValueError('bottom image row does not see the floor')
            q.append(o[:, None] + ray*(-o[2]/ray[2]))
        q0, q1 = q
        d = q1 - q0
        d /= np.linalg.norm(d[:2], axis=0, keepdims=True)
        self.q0 = q0[:2].T
        self.d = d[:2].T
        self.alpha = (rot.T @ (q0 - o[:, None])).T
        self.beta = (rot.T @ np.vstack([d[:2], np.zeros(len(u))])).T
        self.gamma = rot.T @ np.array([0., 0., 1.])
        self._traces = {}

    def rows(self, t, h=0.):
        """Image rows of the point ``h`` above floor-trace point ``t`` (approximate for h > 0).

        For ``h > 0`` that point is not exactly in the column (see the class
        docstring); the detector uses it only for the band-height test, with a
        tolerance. Exact top-edge rows come from ``trace_at``/``rows_at``.
        """
        cy = self.alpha[:, 1] + t*self.beta[:, 1] + h*self.gamma[1]
        cz = self.alpha[:, 2] + t*self.beta[:, 2] + h*self.gamma[2]
        with np.errstate(divide='ignore', invalid='ignore'):
            return np.where(cz > 1e-6, CY + FY*cy/cz, np.nan)

    def trace_at(self, h: float):
        """(q0_h (C, 2), alpha_h (C, 3)): the column's trace in the plane ``z = h``.

        The start point is where a reference ray of the column reaches height
        ``h``: the bottom-row ray below the camera, a ray 100 rows above the
        column's horizon above it (the row may lie outside the image; the line
        and its rows are still exact). The direction is the floor trace's ``d``.
        """
        key = round(float(h), 6)
        if key not in self._traces:
            o, rot = self.origin, self._rot
            u = self.columns.astype(float)
            if key < o[2] - 1e-6:
                v_ref = np.full_like(u, HEIGHT - 1.)
            elif key > o[2] + 1e-6:
                a = rot @ (K_INV @ np.stack([u, np.zeros_like(u), np.ones_like(u)]))
                b = rot @ (K_INV @ np.array([0., 1., 0.]))
                v_ref = -a[2]/b[2] - 100.
            else:
                raise ValueError('trace at the camera height is the horizon')
            ray = rot @ (K_INV @ np.stack([u, v_ref, np.ones_like(u)]))
            q = o[:, None] + ray*((key - o[2])/ray[2])
            self._traces[key] = (q[:2].T, (rot.T @ (q - o[:, None])).T)
        return self._traces[key]

    def rows_at(self, s, h: float):
        """Image rows of points ``s`` along the height-``h`` trace (exact, in the column)."""
        _, a = self.trace_at(h)
        cy = a[:, 1] + s*self.beta[:, 1]
        cz = a[:, 2] + s*self.beta[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            return np.where(cz > 1e-6, CY + FY*cy/cz, np.nan)

    def t_of_row(self, v):
        """Inverse of ``rows(t, 0)``: trace distance of a floor pixel in each column."""
        k = (np.asarray(v, float) - CY)/FY
        with np.errstate(divide='ignore', invalid='ignore'):
            return (self.alpha[:, 1] - k*self.alpha[:, 2])/(k*self.beta[:, 2] - self.beta[:, 1])

    def floor_point(self, t):
        return self.q0 + np.asarray(t, float)[..., None]*self.d

    def range_bearing(self, t):
        """Floor distance and bearing of trace points from the camera nadir (base frame)."""
        p = self.floor_point(t) - self.origin[:2]
        return np.hypot(p[..., 0], p[..., 1]), np.arctan2(p[..., 1], p[..., 0])


_COLUMN_CACHE: dict = {}


def column_model(servo: Mapping, bias_rad: float, columns: np.ndarray) -> ColumnModel:
    key = (tuple(sorted((int(k), int(v)) for k, v in servo.items())), round(float(bias_rad), 7),
           tuple(int(c) for c in columns))
    if key not in _COLUMN_CACHE:
        if len(_COLUMN_CACHE) > 512:
            _COLUMN_CACHE.clear()
        _COLUMN_CACHE[key] = ColumnModel(key[0], float(bias_rad), np.asarray(columns))
    return _COLUMN_CACHE[key]


# ----------------------------------------------------------------------------- static map
class MapGeometry:
    """Axis-aligned wall and door-post footprints of the static map, with heights."""

    def __init__(self, static_map: Mapping, include_posts: bool = True):
        rects = [(o['center_m'][0], o['center_m'][1], o['half_extents_m'][0], o['half_extents_m'][1],
                  o['height_m']) for o in static_map['obstacles'] if o.get('kind') == 'wall']
        if any(o.get('yaw_rad') for o in static_map['obstacles'] if o.get('kind') == 'wall'):
            raise ValueError('rotated walls are not supported by this probe')
        if include_posts:
            rects += [(p['center_m'][0], p['center_m'][1], p['half_extents_m'][0], p['half_extents_m'][1],
                       p['height_m']) for p in static_map.get('landmarks', {}).get('door_posts', [])]
        self.rects = np.asarray(rects, float).reshape(-1, 5)
        # painted floor markings of the static map (regions: pickup area, zones A/B/C;
        # zone slots): their edges are floor-floor lines at known places
        paint = [(r['center_m'][0], r['center_m'][1], r['half_extents_m'][0], r['half_extents_m'][1])
                 for r in static_map.get('regions', {}).values()]
        paint += [(s_['center_m'][0], s_['center_m'][1], s_['half_extents_m'][0], s_['half_extents_m'][1])
                  for slots in static_map.get('zone_slots', {}).values() for s_ in slots]
        self.paint = np.asarray(paint, float).reshape(-1, 4)

    def raycast(self, ox, oy, dx, dy, with_exit: bool = False):
        """First footprint hit along rays (slab test): (t_entry, height[, t_exit]); t=inf if none."""
        ox, oy, dx, dy = np.broadcast_arrays(*(np.asarray(a, float) for a in (ox, oy, dx, dy)))
        dx = np.where(np.abs(dx) < 1e-12, 1e-12, dx)
        dy = np.where(np.abs(dy) < 1e-12, 1e-12, dy)
        best = np.full(ox.shape, np.inf)
        exit_ = np.full(ox.shape, np.inf)
        height = np.zeros(ox.shape)
        for cx, cy, hx, hy, h in self.rects:
            tx1, tx2 = (cx - hx - ox)/dx, (cx + hx - ox)/dx
            ty1, ty2 = (cy - hy - oy)/dy, (cy + hy - oy)/dy
            tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            hit = tmax >= np.maximum(tmin, 0.)
            t = np.where(tmin > 0., tmin, 0.)
            closer = hit & (t < best)
            best = np.where(closer, t, best)
            exit_ = np.where(closer, tmax, exit_)
            height = np.where(closer, h, height)
        return (best, height, exit_) if with_exit else (best, height)

    def paint_rows(self, poses: np.ndarray, cm: ColumnModel, t_wall: np.ndarray) -> np.ndarray:
        """Rows (P, C, 2*M) where each column's floor trace crosses a painted marking edge.

        Only crossings in view (t >= 0) and in front of the first footprint
        (``t_wall`` from ``expected_rows``) count; NaN otherwise.
        """
        poses = np.asarray(poses, float).reshape(-1, 3)
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        start = cm.q0 - T_BACK_M*cm.d
        ox = poses[:, :1] + c*start[None, :, 0] - s*start[None, :, 1]
        oy = poses[:, 1:2] + s*start[None, :, 0] + c*start[None, :, 1]
        dx = c*cm.d[None, :, 0] - s*cm.d[None, :, 1]
        dy = s*cm.d[None, :, 0] + c*cm.d[None, :, 1]
        dx = np.where(np.abs(dx) < 1e-12, 1e-12, dx)
        dy = np.where(np.abs(dy) < 1e-12, 1e-12, dy)
        out = []
        for cx, cy, hx, hy in self.paint:
            tx1, tx2 = (cx - hx - ox)/dx, (cx + hx - ox)/dx
            ty1, ty2 = (cy - hy - oy)/dy, (cy + hy - oy)/dy
            tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            hit = tmax >= tmin
            for tt in (tmin, tmax):
                t = tt - T_BACK_M
                ok = hit & (t >= 0.) & (t < np.where(np.isfinite(t_wall), t_wall, np.inf))
                out.append(np.where(ok, cm.rows(np.where(ok, t, 0.)), np.nan))
        return np.stack(out, -1) if out else np.full(t_wall.shape + (0,), np.nan)

    def expected_rows(self, poses: np.ndarray, cm: ColumnModel, wall_height_m: float = .10):
        """Expected rows (P, C) of the first footprint along every column's traces.

        Returns ``(vb, vt, vt_far, t, h)``. Bottom: first footprint along the
        floor trace. Top: first footprint along the trace at the wall height;
        ``vt`` is its near edge (face -> top face) and ``vt_far`` the far edge of
        the top face (top face -> beyond), both exact in the column; NaN when
        that footprint is taller than the walls (a door post has no top edge
        there). NaN bottom row: nothing hit. A bottom row below the image means
        the footprint starts before the lowest visible floor point.
        """
        poses = np.asarray(poses, float).reshape(-1, 3)
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        dx = c*cm.d[None, :, 0] - s*cm.d[None, :, 1]
        dy = s*cm.d[None, :, 0] + c*cm.d[None, :, 1]

        def cast(start):
            ox = poses[:, :1] + c*start[None, :, 0] - s*start[None, :, 1]
            oy = poses[:, 1:2] + s*start[None, :, 0] + c*start[None, :, 1]
            t, h, te = self.raycast(ox, oy, dx, dy, with_exit=True)
            return t - T_BACK_M, h, te - T_BACK_M
        t, h, _ = cast(cm.q0 - T_BACK_M*cm.d)
        finite = np.isfinite(t)
        vb = np.where(finite, cm.rows(np.where(finite, t, 0.)), np.nan)
        q0h, _ = cm.trace_at(wall_height_m)
        st, sh, se = cast(q0h - T_BACK_M*cm.d)
        top_ok = np.isfinite(st) & (np.abs(sh - wall_height_m) < 1e-6)
        vt = np.where(top_ok, cm.rows_at(np.where(top_ok, st, 0.), wall_height_m), np.nan)
        vtf = np.where(top_ok & np.isfinite(se), cm.rows_at(np.where(top_ok & np.isfinite(se), se, 0.),
                                                             wall_height_m), np.nan)
        return vb, vt, vtf, t, h


# ----------------------------------------------------------------------------- detector
@dataclass
class Scan:
    """One frame's floor/wall-boundary pseudo scan.

    ``vb``/``vt`` are (columns, K) rows of up to K accepted, non-overlapping
    wall-like bands per column, lowest (nearest) first; NaN pads. Candidate 0
    is the pseudo range scan; later candidates exist because a painted floor
    edge below a wall can form a band of the right proportions (the
    likelihood takes the best-matching candidate).
    """
    columns: np.ndarray
    vb: np.ndarray
    vt: np.ndarray
    top_visible: np.ndarray
    contrast: np.ndarray
    band_std: np.ndarray
    range_m: np.ndarray
    bearing_rad: np.ndarray
    self_top: np.ndarray

    @property
    def detected(self) -> np.ndarray:
        """Columns with at least one accepted band."""
        return np.isfinite(self.vb[:, 0])

    def as_dict(self) -> dict:
        r = lambda a, n=2: [[None if not np.isfinite(x) else round(float(x), n) for x in row] for row in a]
        return {'columns': self.columns.tolist(), 'vb': r(self.vb), 'vt': r(self.vt),
                'range_m': r(self.range_m, 3), 'n_detected': int(self.detected.sum()),
                'n_candidates': int(np.isfinite(self.vb).sum())}


def carried_mask_top(und_bgr: np.ndarray, columns: np.ndarray, half: int, min_frac: float = .6,
                     gap_rows: int = 3) -> np.ndarray:
    """Top row of the carried cyan box region that reaches the image bottom, per column (HEIGHT if none).

    Used only while the robot's own commands say it is loaded (``LoadState``);
    the carried box is part of the robot's own view, not of the map. Cyan is a
    ratio test (G and B well above R), so the box in shadow still counts.
    """
    img = und_bgr.astype(np.float32)
    b, g, r = img[..., 0], img[..., 1], img[..., 2]
    cyan = (g > 1.6*r + 8.) & (b > 1.6*r + 8.) & (g > 40.)
    out = np.full(len(columns), HEIGHT, int)
    start = HEIGHT - 6                      # the last rows can carry remap border effects
    for j, u in enumerate(columns):
        col = cyan[:, max(0, u - half):u + half + 1].mean(1) >= min_frac
        if not col[start - 10:start].mean() > .8:
            continue
        v, miss = start, 0
        while v > 0 and miss < gap_rows:
            v -= 1
            miss = 0 if col[v] else miss + 1
        out[j] = v + miss
    return out


def detect_boundaries(und_bgr: np.ndarray, cm: ColumnModel, params: Mapping | None = None,
                      self_top: np.ndarray | None = None) -> Scan:
    """Wall-like bands per column, lowest first (see the module docstring)."""
    p = {**DEFAULT_DETECTOR, **(params or {})}
    cols = cm.columns
    n_c = len(cols)
    n_k = int(p['candidates'])
    half = int(p['strip_half_px'])
    img = und_bgr.astype(np.float32)
    lum = .114*img[..., 0] + .587*img[..., 1] + .299*img[..., 2]
    chroma = (img[..., 0] - img[..., 2])*float(p['chroma_weight'])
    # column strips: (HEIGHT, C)
    csum = lambda a: np.concatenate([np.zeros((a.shape[0], 1), np.float64), np.cumsum(a, 1, dtype=np.float64)], 1)
    lo, hi = np.clip(cols - half, 0, WIDTH), np.clip(cols + half + 1, 0, WIDTH)
    strips = []
    for a in (lum, chroma):
        cs = csum(a)
        strips.append((cs[:, hi] - cs[:, lo])/(hi - lo)[None, :])
    L, Cr = strips
    # row prefix sums for window means / variances
    pre = lambda a: np.vstack([np.zeros((1, n_c)), np.cumsum(a, 0)])
    SL, SL2, SC, SC2 = pre(L), pre(L*L), pre(Cr), pre(Cr*Cr)

    def window(r0, r1):
        """Mean and variance of rows [r0, r1) per candidate (arrays shaped (R, C))."""
        r0 = np.clip(r0, 0, HEIGHT)
        r1 = np.clip(r1, 0, HEIGHT)
        n = np.maximum(r1 - r0, 1)
        ci = np.broadcast_to(np.arange(n_c), r0.shape)
        mL = (SL[r1, ci] - SL[r0, ci])/n
        mC = (SC[r1, ci] - SC[r0, ci])/n
        vL = np.maximum((SL2[r1, ci] - SL2[r0, ci])/n - mL*mL, 0.)
        vC = np.maximum((SC2[r1, ci] - SC2[r0, ci])/n - mC*mC, 0.)
        return mL, mC, vL + vC, r1 - r0

    if self_top is None:
        self_top = np.full(n_c, HEIGHT, int)
    vb = np.arange(HEIGHT - 2, 2, -1, dtype=float)[:, None]*np.ones((1, n_c))    # bottom -> top
    t = cm.t_of_row(vb)
    rng, _ = cm.range_bearing(np.where(np.isfinite(t), t, 0.))
    vt = cm.rows(np.where(np.isfinite(t), t, 0.), float(p['wall_height_m']))
    ok = (np.isfinite(t) & (rng >= p['min_range_m']) & (rng <= p['max_range_m']) & np.isfinite(vt)
          & (vb <= (self_top - int(p['self_margin_px']) - int(p['window_px']) - 1)[None, :])
          & (vb - vt >= p['min_band_px']))
    m = float(p['edge_margin_px'])
    w = int(p['window_px'])
    vb_i = np.round(vb).astype(int)
    # The implied top row is approximate for oblique walls (``ColumnModel.rows``):
    # keep the band and the window above it clear of +-tol around it.
    tol = np.where(ok, float(p['top_tol_px']) + float(p['top_tol_frac'])*(vb - vt), 0.)
    band_top = np.floor(np.where(ok, np.maximum(vt + tol, 0.), 0.) + m).astype(int)
    band_bot = (vb_i - int(math.ceil(m)) + 1)
    band = window(band_top, band_bot)
    below = window(vb_i + 1, vb_i + 1 + w)
    vt_i = np.floor(np.where(ok, vt - tol, 0.)).astype(int)
    top_visible = vt_i - 1 - w >= 0
    above = window(vt_i - w - 1, vt_i - 1)
    contrast = lambda x, y: np.hypot(x[0] - y[0], x[1] - y[1])
    c_below = contrast(band, below)
    c_above = np.where(top_visible, contrast(band, above), 0.)
    std = np.sqrt(band[2])
    band_rows = band[3]
    accept = ok & (band_rows >= 2) & (c_below >= p['min_contrast_below']) & (std <= p['max_band_std']) & (
        np.where(top_visible, c_above >= p['min_contrast_above'], band_rows >= p['min_open_band_px']))

    def step(j, v):
        v = int(np.clip(v, 1, HEIGHT - 2))
        return math.hypot(L[v + 1, j] - L[v - 1, j], Cr[v + 1, j] - Cr[v - 1, j])

    def refine(j, v0, half_win):
        """Row of the strongest luminance/chroma step near ``v0`` (parabolic sub-pixel)."""
        vs = np.arange(int(round(v0)) - half_win, int(round(v0)) + half_win + 1)
        vs = vs[(vs >= 1) & (vs <= HEIGHT - 2)]
        if vs.size < 3:
            return float(v0)
        g = np.array([step(j, v) for v in vs])
        k = int(np.argmax(g))
        off = 0.
        if 0 < k < len(g) - 1:
            den = g[k - 1] - 2*g[k] + g[k + 1]
            off = .5*(g[k - 1] - g[k + 1])/den if abs(den) > 1e-9 else 0.
        return float(vs[k] + np.clip(off, -.5, .5))

    out = {k: np.full((n_c, n_k), np.nan) for k in ('vb', 'vt', 'c', 's', 'r', 'b')}
    out_top = np.zeros((n_c, n_k), bool)
    for j in range(n_c):
        rows_ok = np.flatnonzero(accept[:, j])
        k = 0
        ceiling = np.inf                 # next band must lie above the previous band's top
        for i in rows_ok:
            if k >= n_k:
                break
            if vb[i, j] > ceiling:
                continue
            v_b = refine(j, vb_i[i, j], 2)
            out['vb'][j, k] = v_b
            out['c'][j, k] = c_below[i, j]
            out['s'][j, k] = std[i, j]
            if top_visible[i, j]:
                # measured top edge: the band height carries range information that
                # does not depend on the camera pitch
                span = int(math.ceil(tol[i, j])) + 2
                out['vt'][j, k] = refine(j, vt[i, j], span)
                out_top[j, k] = True
            ceiling = max(float(vt[i, j]), 0.) + 2.
            k += 1
    det = np.isfinite(out['vb'])
    t_obs = cm.t_of_row(np.where(det, out['vb'], CY + 1.).T).T          # (C, K)
    r_obs, b_obs = cm.range_bearing(np.where(np.isfinite(t_obs), t_obs, 0.).T)
    return Scan(cols.copy(), out['vb'], np.where(det & out_top, out['vt'], np.nan), out_top & det,
                out['c'], out['s'], np.where(det, r_obs.T, np.nan), np.where(det, b_obs.T, np.nan),
                np.asarray(self_top, int))


# ----------------------------------------------------------------------------- likelihood
def edge_errors(vb_exp: np.ndarray, vt_exp: np.ndarray, scan: Scan, sigma_px: float = 1.,
                use_top_edge: bool = True, occlusion_margin_px: float = 3.,
                vtf_exp: np.ndarray | None = None, vpaint_exp: np.ndarray | None = None) -> np.ndarray:
    """Best normalized squared row error per (particle, detected column); inf = no match.

    Each observed band bottom is compared with the expected wall bottom edge
    (and its measured top with the expected top) when that bottom edge is in
    view. When the expected bottom is hidden (by the carried box or below the
    image) but the expected top edge is in view, the observed band bottom is
    compared with the expected top edge instead: above a carried box the
    lowest visible structural edge is the wall top. A top edge may be the near
    (face -> top face) or far (top face -> beyond) edge of the wall's top face
    (``vtf_exp``); the closer one counts. ``vpaint_exp`` (P, C, M): rows of
    painted floor-marking edges in view; an observed band bottom may also be
    such an edge (a marking under a uniform floor band mimics a wall band), so
    it is an alternative explanation without a top-edge term.
    """
    det = scan.detected
    ob, ot = scan.vb[det], scan.vt[det]                                   # (D, K)
    limit = np.minimum(scan.self_top[det].astype(float) - occlusion_margin_px, HEIGHT - 1.)   # (D,)
    vbe, vte = vb_exp[:, det], vt_exp[:, det]                             # (P, D)
    vtfe = vte if vtf_exp is None else vtf_exp[:, det]

    def top_sq(obs):                                                      # (P, D, K)
        best = np.full(np.broadcast_shapes(obs[None].shape, vte[..., None].shape), np.inf)
        for e in (vte, vtfe):
            ok = np.isfinite(e)[..., None] & np.isfinite(obs)[None]
            d = np.where(ok, ((obs[None] - np.where(np.isfinite(e), e, 0.)[..., None])/sigma_px)**2, np.inf)
            best = np.minimum(best, d)
        return best
    with np.errstate(invalid='ignore'):
        bottom_in = np.isfinite(vbe) & (vbe >= 0.) & (vbe <= limit[None, :])
        top_in = np.isfinite(vte) & (vte >= 0.) & (vte <= limit[None, :])
        sq_b = ((ob[None] - vbe[..., None])/sigma_px)**2
        if use_top_edge:
            tt = top_sq(ot)
            sq_b = sq_b + np.where(np.isfinite(tt), tt, 0.)
        sq_b = np.where(bottom_in[..., None], sq_b, np.inf)
        sq_t = np.where((~bottom_in & top_in)[..., None], top_sq(ob), np.inf)
        sq = np.minimum(sq_b, sq_t)
        if vpaint_exp is not None and vpaint_exp.shape[-1]:
            vp = vpaint_exp[:, det, :]                                        # (P, D, M)
            vp = np.where(np.isfinite(vp) & (vp <= limit[None, :, None]), vp, np.nan)
            dp = np.abs(ob[None, :, :, None] - vp[:, :, None, :])             # (P, D, K, M)
            dp = np.where(np.isfinite(dp), dp, np.inf).min(-1)
            sq = np.minimum(sq, (dp/sigma_px)**2)
    sq = np.where(np.isfinite(sq), sq, np.inf)                            # NaN pads -> no match
    return sq.min(2)


def boundary_loglik(vb_exp: np.ndarray, vt_exp: np.ndarray, scan: Scan, params: Mapping,
                    vtf_exp: np.ndarray | None = None, vpaint_exp: np.ndarray | None = None) -> np.ndarray:
    """Robust per-particle log-likelihood of one scan.

    ``vb_exp``/``vt_exp``: (particles, columns) expected rows. Per column the best
    observed candidate is used (max-mixture, ``edge_errors``) against a flat
    outlier floor; the frame total is scaled to ``effective_columns`` because
    columns share the extrinsic (pitch) error.
    """
    mp = {**DEFAULT_MEASUREMENT, **params}
    det = scan.detected
    if det.sum() < int(mp['min_columns']):
        return np.zeros(vb_exp.shape[0])
    best = edge_errors(vb_exp, vt_exp, scan, float(mp['sigma_px']), bool(mp.get('use_top_edge', True)),
                       vtf_exp=vtf_exp, vpaint_exp=vpaint_exp if mp.get('use_paint', False) else None)
    ll = np.logaddexp(math.log(1 - mp['outlier_prob']) - .5*best,
                      math.log(mp['outlier_prob']) - float(mp['outlier_margin']))
    n = int(det.sum())
    return ll.sum(1)*min(1., float(mp['effective_columns'])/n)


def make_boundary_pf(m1_module, static_map: Mapping, params: Mapping, measurement: Mapping,
                     detector: Mapping, seed: int):
    """Subclass of the M1 ``OwnCamLocalizer`` whose measurement is the boundary scan."""
    base = m1_module.OwnCamLocalizer

    class BoundaryScanLocalizer(base):
        def __init__(self):
            super().__init__(static_map, params, seed=seed)
            self.geometry = MapGeometry(static_map)
            self.measurement = {**DEFAULT_MEASUREMENT, **measurement}
            self.detector = {**DEFAULT_DETECTOR, **detector}
            self.columns = column_positions(self.detector['columns'], int(self.detector['strip_half_px']))
            self.last_scan_t = None
            self.stats.update(scan_updates=0, scan_columns=0)

        def init_gaussian(self, mean: Sequence[float], std: Sequence[float]):
            self.px = np.asarray(mean, float)[None, :] + self.rng.normal(size=(self.n, 3))*np.asarray(std, float)
            self.px[:, 2] = m1_module.wrap(self.px[:, 2])
            self._init_common()

        def init_uniform(self, n: int | None = None):
            """Global localization prior: uniform over the map's free space and all yaws."""
            n = int(n or self.n)
            x0, x1, y0, y1 = self.bounds
            px = np.empty((0, 3))
            while len(px) < n:
                cand = np.column_stack([self.rng.uniform(x0, x1, 4*n), self.rng.uniform(y0, y1, 4*n),
                                        self.rng.uniform(-np.pi, np.pi, 4*n)])
                px = np.vstack([px, cand[self._map_logprior(cand) == 0.]])
            self.n = n
            self.px = px[:n]
            self._init_common()

        def _init_common(self):
            sd = self.params['motion']['scale_std']
            self.scale = 1. + self.rng.normal(size=(self.n, 3))*sd
            self.logw = self._map_logprior(self.px)
            self.initialized = True

        def resize(self, n: int):
            """Resample to ``n`` particles (low variance), e.g. after global convergence."""
            w = np.exp(self.logw - self.logw.max())
            w /= w.sum()
            positions = (np.arange(n) + self.rng.uniform())/n
            idx = np.minimum(np.searchsorted(np.cumsum(w), positions), self.n - 1)
            self.px, self.scale = self.px[idx].copy(), self.scale[idx].copy()
            self.n = int(n)
            self.logw = np.zeros(self.n)

        def bias(self, pose: Mapping) -> float:
            b = self.measurement['bias_rad']
            return elevation_bias(b['loaded'] if self.load.loaded else b['unloaded'], pose)

        def scan(self, und_bgr: np.ndarray, pose: Mapping) -> Scan:
            cm = column_model(pose, self.bias(pose), self.columns)
            self_top = carried_mask_top(und_bgr, self.columns, int(self.detector['strip_half_px'])) \
                if self.load.loaded else None
            return detect_boundaries(und_bgr, cm, self.detector, self_top)

        def update_scan(self, t: float, scan: Scan, pose: Mapping) -> dict:
            self.predict_to(t)
            if self.initialized and scan.detected.sum() >= int(self.measurement['min_columns']):
                cm = column_model(pose, self.bias(pose), self.columns)
                vb, vt, vtf, tw, _ = self.geometry.expected_rows(self.px, cm)
                vp = self.geometry.paint_rows(self.px, cm, tw) if self.measurement.get('use_paint') else None
                self.logw = self.logw + boundary_loglik(vb, vt, scan, self.measurement, vtf, vp)
                self.stats['scan_updates'] += 1
                self.stats['scan_columns'] += int(scan.detected.sum())
                self.last_scan_t = t
            if self.initialized:
                self._normalize_and_resample()
            est = self.estimate()
            est['since_scan_s'] = None if self.last_scan_t is None else round(t - self.last_scan_t, 3)
            return est

    return BoundaryScanLocalizer()


# ----------------------------------------------------------------------------- episode replay
def read_jsonl(path: Path) -> list[dict]:
    with open(path) as fh:
        return [json.loads(line) for line in fh if line.strip()]


def episode_inputs(ep_dir: Path) -> dict:
    """Robot-side inputs of one recorded episode (never ``eval_only/``)."""
    ep_dir = Path(ep_dir)
    manifest = json.loads((ep_dir/'manifest.json').read_text())
    events = read_jsonl(ep_dir/'controller_events.jsonl')
    return {'manifest': manifest, 'commands': read_jsonl(ep_dir/'inputs'/'commands.jsonl'),
            'frames': read_jsonl(ep_dir/'inputs'/'frames.jsonl'),
            'profile_events': [(float(e['t']), None if e.get('profile') in (None, 'default') else e['profile'])
                               for e in events if e.get('event') == 'motion_profile']}


def replay(ep_dir: Path, filters: Mapping[str, object], *, decode_every: int = 1, on_frame=None):
    """Feed own commands, motion-profile switches and frames to each filter in time order.

    ``filters`` maps a name to an object with ``command``, ``set_motion_profile`` and
    ``frame(t, bgr_raw, frame_row)``. Events stamped at a frame's time are applied
    after that frame: the M1 runner captures decision frames before the
    controller acts at the same instant (this ordering reproduces every frame's
    recorded ``commanded_servo``). Returns the number of frames processed.
    """
    data = episode_inputs(ep_dir)
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
                for f in filters.values():
                    f.set_motion_profile(tp, prof[pi][1])
                pi += 1
            else:
                for f in filters.values():
                    f.command(cmds[ci])
                ci += 1
        if k % decode_every:
            continue
        bgr = cv2.imread(str(Path(ep_dir)/row['file']), cv2.IMREAD_COLOR)
        for f in filters.values():
            f.frame(t, bgr, row)
        if on_frame is not None:
            on_frame(k, row)
    return len(frames)
