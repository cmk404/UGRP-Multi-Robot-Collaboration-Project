"""Own-camera observation memory for one robot: "look once, remember".

Inputs (and nothing else): this robot's own ``robot_cam`` frames (the tag
detections the pose source already computed, own-RGB box detections from
``harness.zone_color_boxes.detect_own``), its own ``PoseReport`` (the
own-camera particle filter), its own issued servo pulses, the static tagged
map and the fixed calibrations. It never imports the simulator, never reads
TOP, ``nav_cam`` or another robot, and is never shared between robots.

What it remembers (design and reuse survey:
docs/design/2026-09-26-owncam-memory-reuse.md)

* Tag/pose fixes: a per-frame fix log, and per 0.5 m cell x posture x load x
  pan how often the wall tags the map says are in view were detected (the
  look planner's detection prior).
* Box tracks: one static 2-D Kalman filter per box
  (``harness.owncam_memory_kf``: filterpy predict/update, PythonRobotics
  EKF-SLAM association). First/last seen, near/far hits, misses since the
  last hit, state tentative / confirmed / absent / claimed / held / placed.
  Staleness = covariance growth (process noise) + age.
* Free / blocked floor: a log-odds occupancy grid (Elfes 1989, clamped as in
  OctoMap) over the static map. Settled own SEARCH_POSE / LOOK_P20 frames
  mark the near floor footprint free and own box detections blocked; far
  footprint coverage is counted separately; log-odds decay toward unknown.

What it decides

* Expected view: which map tags the camera should see from the current
  estimate (fisheye projection of the tag corners, facing angle, 3-D occlusion
  by walls and door posts, the held-box image band).
* Look planning: which LOOK_P20 pans to use, chosen greedily by Fisher
  information of the tag measurements per second of arm motion (active
  localization in the sense of Burgard, Fox and Thrun, IJCAI 1997), weighted
  by the remembered detection rate; a full look when the map predicts too
  little.
* Search coverage: how much of a search view's floor footprint is still
  unknown, so viewpoints and pans that would only re-observe known floor are
  skipped.
"""
from __future__ import annotations

import copy
import math
from collections.abc import Callable, Mapping, Sequence

import cv2
import numpy as np

from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SEARCH_POSE, SETTLE_S
from harness.owncam_memory_kf import SOURCES as KF_SOURCES
from harness.owncam_memory_kf import associate, kf_predict, kf_update, observation_to_map
from harness.wall_tags import camera_in_base, tag_world_frame
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

SCHEMA = 'ugrp.owncam_memory.v1'
IMAGE_W, IMAGE_H = 640, 480
# The simulated wrist frame is a pinhole render (same K) remapped to the raw fisheye
# geometry (sim.masterpi_camera_profile.raw_fisheye_remap): a raw pixel has content iff its
# pinhole (ideal) pixel lies inside the 640x480 render. Visibility is tested on both.
# With a box held, the box covers the image below this row in both loaded postures
# (M1 test s102/s105 own frames: loaded tag detections max row p99 149 px).
LOADED_ROW_LIMIT_PX = 150.
POSTURE_TOL_PWM = 3
POSTURES = {'look': LOOK_P20, 'search': SEARCH_POSE, 'carry': CARRY_POSTURE}
SETTLED_S = .3                       # frames this long after an own arm/pan command count as settled
# Planning visibility (look planner and detection statistics). The margin may be slightly
# negative: tags touching the render edge are still detected (M1 test own frames).
PLAN_MIN_SIDE_PX, PLAN_MAX_RANGE_M, PLAN_MAX_OBLIQUE_DEG, PLAN_MARGIN_PX = 10., 3.5, 70., -2.
PLAN_LOADED_ROW_LIMIT_PX = 172.      # loaded detections reach row 169 at the frame sides
# Detection probability of a predicted tag by its projected side and obliquity, from the
# M1 test s102/s105 own frames (predicted vs detected, settled LOOK_P20 frames).
DETECT_P_BY_SIDE = ((12., .03), (14., .40), (17., .72), (22., .97), (1e9, .95))
DETECT_P_BY_OBLIQUE = ((30., 1.), (50., .85), (60., .50), (1e9, .15))
# Strict expectation (the "expected view is missing" trigger): near, large, well inside the frame.
EXPECT_MIN_SIDE_PX, EXPECT_MAX_RANGE_M, EXPECT_MAX_OBLIQUE_DEG, EXPECT_MARGIN_PX = 20., 1.8, 55., 18.
EXPECT_LOADED_ROW_LIMIT_PX = 132.
MISSING_FRAMES = 3
NO_TAG_S = 3.                        # unloaded: no tag for this long while the map predicts some in view
# Boxes (own-RGB detections; box centre 0.016 m above the floor).
BOX_CENTRE_Z_M = .016
NEAR_SIGMA_M = (.012, .02)           # detection noise = a + b*range (near floor fit)
FAR_SIGMA_M = (.10, .20)             # far_coarse (M1 test s101: 0.72 m error at ~2.9 m)
TRACK_Q_M2_S = 1e-5                  # static boxes: slow covariance growth (staleness)
TRACK_INIT_FLOOR_M = .01
CONFIRM_NEAR_HITS = 2
ABSENT_MISSES = 3
ABSENT_RANGE_M = 1.0
ABSENT_MARGIN_PX = 30.
STALE_AGE_S = 60.
STALE_SIGMA_M = .05
KEEPOUT_BASE_HALF_M = .03            # = m1_owncam_delivery.SEEN_BOX_HALF_M
KEEPOUT_MAX_EXTRA_M = .05
# Floor grid.
GRID_M = .05
FREE_RANGE_M = 1.1
FAR_RANGE_M = (1.1, 1.7)
L_FREE, L_OCC, L_OCC_FAR = -.4, .85, .4
L_MIN, L_MAX, L_KNOWN = -2., 3.5, .6
GRID_TAU_S = 120.
DETECTION_CLEAR_M = .08
FAR_SEEN_KNOWN = 2
FOOTPRINT_MARGIN_PX = 10.
# Look planner.
LOOK_CANDIDATE_PANS = (1500, 1230, 970, 1770, 2030)
FRAMES_PER_DWELL = 3
MAX_SHORT_PANS = 3
MIN_GAIN_NATS = .5
PAN_RATE_PWM_S = 600.                # 60 PWM per 0.1 s control tick
STAT_CELL_M = .5
CONFIG = {k: v for k, v in dict(globals()).items() if k.isupper() and isinstance(v, (int, float, tuple, str))}


def posture_name(servo: Mapping[int, int]) -> str:
    for name, pose in POSTURES.items():
        if all(abs(int(servo.get(s, -9999)) - int(pose[s])) <= POSTURE_TOL_PWM for s in (3, 4, 5)):
            return name
    return 'other'


def tag_detect_prob(side_px: float, oblique_deg: float) -> float:
    p = next(v for lim, v in DETECT_P_BY_SIDE if side_px < lim)
    return p*next(v for lim, v in DETECT_P_BY_OBLIQUE if oblique_deg < lim)


def _wrap(a):
    return (np.asarray(a) + np.pi) % (2*np.pi) - np.pi


def _logdet(m: np.ndarray) -> float:
    sign, value = np.linalg.slogdet(m)
    return float(value) if sign > 0 else -math.inf


class ViewModel:
    """Map geometry seen through this robot's wrist fisheye (static map + fixed calibration)."""

    def __init__(self, static_map: Mapping, params: Mapping, width: int = IMAGE_W, height: int = IMAGE_H):
        self.K = scaled_camera_matrix(width, height)
        self.D = np.asarray(CAMERA_FISHEYE_D, float).reshape(4, 1)
        self.f = float(self.K[0, 0])
        tags = static_map['landmarks']['tags']
        self.tag_ids = np.array([int(t['id']) for t in tags])
        self.tag_centre = np.array([t['center_m'] for t in tags], float)
        corners, normals = [], []
        for t in tags:
            c_w, rot = tag_world_frame(t)
            h = float(t['size_m'])/2
            corners.append(c_w + np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]) @ rot.T)
            normals.append(rot[:, 2])
        self.tag_corners = np.array(corners, float)          # (T, 4, 3)
        self.tag_normal = np.array(normals, float)           # (T, 3), out of the wall
        occ = [(o['center_m'], o['half_extents_m'], float(o.get('height_m', .10)))
               for o in static_map.get('obstacles', []) if o.get('kind') == 'wall']
        occ += [(p['center_m'], p['half_extents_m'], float(p['height_m']))
                for p in static_map['landmarks'].get('door_posts', [])]
        self.occluders = [(np.asarray(c, float)[:2], np.asarray(h, float)[:2], z) for c, h, z in occ]
        mp = dict(params['measurement'])
        self.meas = {False: mp, True: {**mp, **(params.get('measurement_loaded') or {})}}
        x0, x1, y0, y1 = static_map['bounds_m']
        self.bounds = (float(x0), float(x1), float(y0), float(y1))
        xs = np.arange(x0 + GRID_M/2, x1, GRID_M)
        ys = np.arange(y0 + GRID_M/2, y1, GRID_M)
        self.grid_x, self.grid_y = np.meshgrid(xs, ys)          # (ny, nx)
        self.grid_shape = self.grid_x.shape
        self.cells = np.stack([self.grid_x.ravel(), self.grid_y.ravel()], axis=1)
        pickup = static_map.get('regions', {}).get('pickup')
        self.pickup = None if pickup is None else (np.asarray(pickup['center_m'], float),
                                                   np.asarray(pickup['half_extents_m'], float))
        self.pickup_mask = (np.zeros(len(self.cells), bool) if self.pickup is None else
                            np.all(np.abs(self.cells - self.pickup[0]) <= self.pickup[1], axis=1))

    # ------------------------------------------------------------ transforms
    def _correct(self, p_c: np.ndarray, loaded: bool) -> np.ndarray:
        """The particle filter's own camera-model corrections (fixed calibration)."""
        mp = self.meas[bool(loaded)]
        corr = mp.get('camera_correction')
        if corr:
            om = np.asarray(corr['omega_rad'], float)
            p_c = p_c - np.asarray(corr['delta_m'], float) + np.cross(om, p_c)
        bias = float(mp.get('elevation_bias_rad', 0.) or 0.)
        if bias:
            b = -bias                                  # observed elevation = predicted + bias
            cb, sb = math.cos(b), math.sin(b)
            y, z = p_c[..., 1].copy(), p_c[..., 2].copy()
            p_c = p_c.copy()
            p_c[..., 1], p_c[..., 2] = y*cb - z*sb, y*sb + z*cb
        return p_c

    @staticmethod
    def _to_base(points_w: np.ndarray, poses: np.ndarray) -> np.ndarray:
        """World points (M, 3) into the base frames of N planar poses -> (N, M, 3)."""
        poses = np.asarray(poses, float).reshape(-1, 3)
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        dx = points_w[None, :, 0] - poses[:, None, 0]
        dy = points_w[None, :, 1] - poses[:, None, 1]
        dz = np.broadcast_to(points_w[None, :, 2], dx.shape)
        return np.stack((c*dx + s*dy, -s*dx + c*dy, dz), axis=-1)

    def to_camera(self, points_w: np.ndarray, poses, servo: Mapping[int, int], loaded: bool) -> np.ndarray:
        o_bc, r_bc = camera_in_base(servo)
        p_b = self._to_base(np.asarray(points_w, float).reshape(-1, 3), poses)
        return self._correct((p_b - o_bc) @ r_bc, loaded)

    def camera_world(self, pose: Sequence[float], servo: Mapping[int, int]) -> np.ndarray:
        o_bc, _ = camera_in_base(servo)
        x, y, yaw = (float(v) for v in pose)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([x + c*o_bc[0] - s*o_bc[1], y + s*o_bc[0] + c*o_bc[1], o_bc[2]])

    def project(self, p_c: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(raw fisheye px, ideal pinhole px, front/in-domain mask) for camera points (..., 3)."""
        z = p_c[..., 2]
        ok = z > .05
        norm = p_c[..., :2]/np.where(ok, z, 1.)[..., None]
        ok &= np.hypot(norm[..., 0], norm[..., 1]) < 8.
        norm = np.where(ok[..., None], norm, 0.)
        flat = norm.reshape(-1, 1, 2).astype(np.float64)
        raw = cv2.fisheye.distortPoints(flat, self.K, self.D).reshape(p_c.shape[:-1] + (2,))
        ideal = np.stack((self.K[0, 0]*norm[..., 0] + self.K[0, 2], self.K[1, 1]*norm[..., 1] + self.K[1, 2]), axis=-1)
        return raw, ideal, ok

    def in_view(self, raw: np.ndarray, ideal: np.ndarray, loaded: bool, margin, row_margin=None,
                row_limit: float | None = None) -> np.ndarray:
        """Pixel content mask (per point): inside the render (ideal) and the raw frame, above the held box."""
        mx = np.asarray(margin, float)
        my = mx if row_margin is None else np.asarray(row_margin, float)
        ok = ((ideal[..., 0] >= mx) & (ideal[..., 0] <= IMAGE_W - 1 - mx)
              & (ideal[..., 1] >= my) & (ideal[..., 1] <= IMAGE_H - 1 - my)
              & (raw[..., 0] >= mx) & (raw[..., 0] <= IMAGE_W - 1 - mx)
              & (raw[..., 1] >= my) & (raw[..., 1] <= IMAGE_H - 1 - my))
        if loaded:
            ok &= raw[..., 1] <= (LOADED_ROW_LIMIT_PX if row_limit is None else row_limit) - my
        return ok

    def occluded(self, p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
        """3-D sight lines p0 (3,) -> p1 (N, 3) blocked by a wall or door post (height-aware)."""
        p1 = np.asarray(p1, float).reshape(-1, 3)
        blocked = np.zeros(len(p1), bool)
        d = p1[:, :2] - p0[:2]
        for centre, half, height in self.occluders:
            t0, t1 = np.zeros(len(p1)), np.ones(len(p1))
            for a in (0, 1):
                lo, hi = centre[a] - half[a], centre[a] + half[a]
                da = d[:, a]
                with np.errstate(divide='ignore', invalid='ignore'):
                    ta, tb = (lo - p0[a])/da, (hi - p0[a])/da
                par = np.abs(da) < 1e-12
                inside = lo <= p0[a] <= hi
                tmin = np.where(par, -np.inf if inside else np.inf, np.minimum(ta, tb))
                tmax = np.where(par, np.inf if inside else -np.inf, np.maximum(ta, tb))
                t0, t1 = np.maximum(t0, tmin), np.minimum(t1, tmax)
            hit = t0 < t1
            if not np.any(hit):
                continue
            z0 = p0[2] + (p1[:, 2] - p0[2])*t0
            z1 = p0[2] + (p1[:, 2] - p0[2])*t1
            blocked |= hit & (np.minimum(z0, z1) < height - 1e-3)
        return blocked

    # ------------------------------------------------------------ tags
    def visible_tags(self, pose: Sequence[float], servo: Mapping[int, int], loaded: bool, *, strict: bool = False,
                     sigma: tuple[float, float] | None = None) -> list[dict]:
        """Map tags this camera should see from ``pose`` (x, y, yaw) with the issued ``servo`` pulses."""
        pose = np.asarray(pose, float)
        corners_c = self.to_camera(self.tag_corners.reshape(-1, 3), pose, servo, loaded)[0].reshape(-1, 4, 3)
        centre_c = corners_c.mean(axis=1)
        px, ideal, ok = self.project(corners_c)
        rng = np.linalg.norm(centre_c, axis=1)
        n_b = self._to_base(self.tag_normal, np.array([[0., 0., pose[2]]]))[0]
        _, r_bc = camera_in_base(servo)
        n_c = n_b @ r_bc
        cos_oblique = -np.sum(n_c*centre_c, axis=1)/np.maximum(rng, 1e-9)
        edges = np.linalg.norm(px - np.roll(px, 1, axis=1), axis=2)
        side = edges.mean(axis=1)
        if strict:
            sx, syaw = sigma if sigma is not None else (0., 0.)
            mx = EXPECT_MARGIN_PX + self.f*(2*syaw + 2*sx/np.maximum(rng, .3))
            my = np.full(len(rng), EXPECT_MARGIN_PX)
            row_limit = EXPECT_LOADED_ROW_LIMIT_PX
            min_side, max_range, max_obl = EXPECT_MIN_SIDE_PX, EXPECT_MAX_RANGE_M, EXPECT_MAX_OBLIQUE_DEG
        else:
            mx = my = np.full(len(rng), PLAN_MARGIN_PX)
            row_limit = PLAN_LOADED_ROW_LIMIT_PX
            min_side, max_range, max_obl = PLAN_MIN_SIDE_PX, PLAN_MAX_RANGE_M, PLAN_MAX_OBLIQUE_DEG
        inside = np.all(ok, axis=1) & np.all(self.in_view(px, ideal, loaded, mx[:, None], my[:, None], row_limit), axis=1)
        keep = inside & (side >= min_side) & (rng <= max_range) & (cos_oblique >= math.cos(math.radians(max_obl)))
        idx = np.flatnonzero(keep)
        if len(idx):
            cam = self.camera_world(pose, servo)
            towards = cam[None, :] - self.tag_centre[idx]
            towards /= np.maximum(np.linalg.norm(towards, axis=1, keepdims=True), 1e-9)
            blocked = self.occluded(cam, self.tag_centre[idx] + .01*towards)
            idx = idx[~blocked]
        rows = []
        for i in idx:
            obl = math.degrees(math.acos(min(1., float(cos_oblique[i]))))
            rows.append({'id': int(self.tag_ids[i]), 'index': int(i), 'px': px[i].mean(axis=0).round(1).tolist(),
                         'side_px': round(float(side[i]), 1), 'range_m': round(float(rng[i]), 3),
                         'oblique_deg': round(obl, 1), 'p_detect': round(tag_detect_prob(float(side[i]), obl), 3)})
        return rows

    def _tag_measurements(self, poses: np.ndarray, index: np.ndarray, servo: Mapping[int, int], loaded: bool):
        p_c = self.to_camera(self.tag_centre[index], poses, servo, loaded)
        return np.stack((np.arctan2(p_c[..., 0], p_c[..., 2]), np.arctan2(p_c[..., 1], p_c[..., 2]),
                         np.log(np.maximum(np.linalg.norm(p_c, axis=-1), 1e-6))), axis=-1)

    def fisher(self, pose: Sequence[float], servo: Mapping[int, int], loaded: bool, visible: Sequence[Mapping],
               frames: int = FRAMES_PER_DWELL) -> np.ndarray:
        """Pose information (3x3) of ``frames`` frames of the visible tags, the PF measurement model.

        Azimuth, elevation and log range with the particle filter's own noise
        floors; ``tag_temper`` as in the filter (joint log-likelihood / n**temper).
        """
        if not visible:
            return np.zeros((3, 3))
        index = np.array([v['index'] for v in visible])
        eps = np.array([1e-3, 1e-3, 1e-3])
        base = np.asarray(pose, float)
        poses = np.vstack([base, base + np.diag(eps)])
        m = self._tag_measurements(poses, index, servo, loaded)               # (4, M, 3)
        diff = m[1:] - m[:1]
        diff[..., 0] = _wrap(diff[..., 0])
        J = diff/eps[:, None, None]                                           # (3 state, M, 3 meas)
        mp = self.meas[bool(loaded)]
        s = np.array([max(mp['azimuth_std_rad'], mp.get('azimuth_floor_rad', 0.)),
                      max(mp['elevation_std_rad'], mp.get('elevation_floor_rad', 0.)),
                      max(mp['range_log_std'], mp.get('range_floor', 0.))])
        W = np.diag(1./s**2)
        info = np.zeros((3, 3))
        for i, v in enumerate(visible):
            H = J[:, i, :].T                                                  # (3 meas, 3 state)
            info += float(v.get('p_detect', 1.))*(H.T @ W @ H)
        n = max(1., sum(float(v.get('p_detect', 1.)) for v in visible))   # expected detections per frame
        return frames*info/(n**float(mp.get('tag_temper', 0.)))

    # ------------------------------------------------------------ floor
    def floor_footprint(self, pose: Sequence[float], servo: Mapping[int, int], loaded: bool, *,
                        max_range: float, min_range: float = 0., z: float = 0., mask: np.ndarray | None = None,
                        margin: float = FOOTPRINT_MARGIN_PX) -> tuple[np.ndarray, np.ndarray]:
        """Grid cells whose floor point the camera sees: (flat cell indices, ranges from the camera)."""
        cam = self.camera_world(pose, servo)
        r = np.hypot(self.cells[:, 0] - cam[0], self.cells[:, 1] - cam[1])
        sel = (r <= max_range) & (r > min_range)
        if mask is not None:
            sel &= mask
        idx = np.flatnonzero(sel)
        if not len(idx):
            return idx, r[idx]
        pts = np.column_stack((self.cells[idx], np.full(len(idx), z)))
        p_c = self.to_camera(pts, np.asarray(pose, float), servo, loaded)[0]
        px, ideal, ok = self.project(p_c)
        ok &= self.in_view(px, ideal, loaded, margin)
        idx, pts = idx[ok], pts[ok]
        if len(idx):
            keep = ~self.occluded(cam, pts)
            idx = idx[keep]
        return idx, r[idx]

    def point_in_view(self, pose, servo, loaded, point_w, margin: float = ABSENT_MARGIN_PX) -> bool:
        point = np.asarray(point_w, float).reshape(1, 3)
        p_c = self.to_camera(point, np.asarray(pose, float), servo, loaded)[0]
        px, ideal, ok = self.project(p_c)
        if not (ok[0] and self.in_view(px, ideal, loaded, margin)[0]):
            return False
        return not bool(self.occluded(self.camera_world(pose, servo), point)[0])


class BoxTrack:
    def __init__(self, track_id: str, kind: str, z: np.ndarray, R: np.ndarray, now: float, frame_id: int,
                 range_class: str):
        self.track_id, self.kind = track_id, kind
        self.x = np.asarray(z, float).copy()
        self.P = np.asarray(R, float) + (TRACK_INIT_FLOOR_M**2)*np.eye(2)
        self.first_seen_t = self.last_seen_t = self.t = float(now)
        self.near_hits = int(range_class == 'near')
        self.far_hits = int(range_class != 'near')
        self.misses = 0
        self.last_absent_t: float | None = None
        self.state = 'tentative'
        self.confirmed_t: float | None = None
        self.frames = [int(frame_id)]
        self._maybe_confirm(now)

    def _maybe_confirm(self, now):
        if self.state == 'tentative' and self.near_hits >= CONFIRM_NEAR_HITS:
            self.state, self.confirmed_t = 'confirmed', float(now)
            return True
        return False

    def predict(self, now: float) -> None:
        dt = max(0., float(now) - self.t)
        if dt:
            self.x, self.P = kf_predict(self.x, self.P, TRACK_Q_M2_S*dt*np.eye(2))
            self.t = float(now)

    def sigma_m(self) -> float:
        return float(math.sqrt(max(np.linalg.eigvalsh(self.P).max(), 0.)))

    def record(self, now: float) -> dict:
        return {'track_id': self.track_id, 'kind': self.kind, 'xy': [round(float(v), 4) for v in self.x],
                'sigma_m': round(self.sigma_m(), 4), 'state': self.state, 'near_hits': self.near_hits,
                'far_hits': self.far_hits, 'misses': self.misses, 'first_seen_t': round(self.first_seen_t, 3),
                'last_seen_t': round(self.last_seen_t, 3), 'age_s': round(float(now) - self.last_seen_t, 3),
                'confirmed_t': None if self.confirmed_t is None else round(self.confirmed_t, 3),
                'last_absent_t': None if self.last_absent_t is None else round(self.last_absent_t, 3),
                'frames': self.frames[-8:]}


class OwnCamMemory:
    """Per-robot memory; feed every own frame with ``observe_frame``."""

    def __init__(self, static_map: Mapping, params: Mapping, *, robot_id: str,
                 on_event: Callable[[str, dict], None] | None = None, detect=None):
        self.robot_id = robot_id
        self.view = ViewModel(static_map, params)
        self.on_event = on_event
        self._detect = detect
        self.t = None
        self.tracks: list[BoxTrack] = []
        self._next_track = 0
        self.log_odds = np.zeros(len(self.view.cells))
        self.far_seen = np.zeros(len(self.view.cells), np.int16)
        self.pan_stats: dict[tuple, list[int]] = {}
        self.fixes: list[dict] = []
        self.last_fix: dict | None = None
        self.missing_run = 0
        self.last_detect_t: float | None = None
        self.last_plan_visible_t: float | None = None
        self.claimed: str | None = None
        self.counts = {'frames': 0, 'settled_frames': 0, 'box_frames': 0, 'box_detections': 0,
                       'free_updates': 0, 'occupied_updates': 0, 'view_missing_events': 0,
                       'ambiguous_detections': 0, 'looks_planned_short': 0, 'looks_full': 0}
        self.events: list[dict] = []

    # ------------------------------------------------------------ bookkeeping
    def event(self, now: float, kind: str, **detail) -> None:
        row = {'t': round(float(now), 3), 'memory_event': kind, **detail}
        self.events.append(row)
        if self.on_event is not None:
            self.on_event(kind, row)

    def _decay_to(self, now: float) -> None:
        if self.t is not None and now > self.t:
            self.log_odds *= math.exp(-(now - self.t)/GRID_TAU_S)
        self.t = float(now) if self.t is None else max(self.t, float(now))
        for tr in self.tracks:
            tr.predict(now)

    def _stat_key(self, pose, posture, loaded, pan):
        return (int(math.floor(pose[0]/STAT_CELL_M)), int(math.floor(pose[1]/STAT_CELL_M)), posture, bool(loaded),
                int(pan))

    # ------------------------------------------------------------ input
    def observe_frame(self, now: float, *, frame_id: int, image, servo: Mapping[int, int],
                      tag_detections: Sequence[Mapping], report, arm_settled_s: float, loaded: bool) -> dict:
        """One own frame: fix log, expected view, box tracks and floor (unloaded, settled)."""
        self._decay_to(now)
        self.counts['frames'] += 1
        servo = {int(k): int(v) for k, v in servo.items()}
        posture = posture_name(servo)
        detected = {int(d['id']) for d in tag_detections}
        if detected:
            self.last_detect_t = float(now)
        out = {'posture': posture, 'detected_tags': sorted(detected), 'settled': arm_settled_s >= SETTLED_S}
        if not report.initialized:
            return out
        pose = (report.x_m, report.y_m, report.yaw_rad)
        cov = np.asarray(report.cov, float).reshape(3, 3)
        if detected:
            self.last_fix = {'t': round(float(now), 3), 'frame_id': int(frame_id), 'tags': sorted(detected),
                             'posture': posture, 'pan': servo.get(6), 'loaded': bool(loaded),
                             'xyyaw': [round(float(v), 4) for v in pose], 'std_xy_m': round(report.std_xy_m, 4),
                             'std_yaw_rad': round(report.std_yaw_rad, 5)}
            self.fixes.append(self.last_fix)
        if not out['settled'] or posture == 'other':
            return out
        self.counts['settled_frames'] += 1
        plan_vis = self.view.visible_tags(pose, servo, loaded)
        if plan_vis:
            self.last_plan_visible_t = float(now)
            key = self._stat_key(pose, posture, loaded, servo.get(6, 1500))
            st = self.pan_stats.setdefault(key, [0, 0])
            st[0] += 1
            st[1] += int(bool(detected & {v['id'] for v in plan_vis}))
        # The missing-view check runs in the driving postures only (during a look the
        # estimate is being corrected; the check is reset when a look starts and ends).
        strict = ([] if posture == 'look' else
                  self.view.visible_tags(pose, servo, loaded, strict=True, sigma=(report.std_xy_m, report.std_yaw_rad)))
        out['expected_tags'] = [v['id'] for v in strict]
        if strict:
            if detected & set(out['expected_tags']):
                self.missing_run = 0
            else:
                self.missing_run += 1
                if self.missing_run == MISSING_FRAMES:
                    self.counts['view_missing_events'] += 1
                    self.event(now, 'view_missing', expected=out['expected_tags'], detected=sorted(detected),
                               frame_id=int(frame_id), posture=posture, pan=servo.get(6))
        if not loaded and posture in ('search', 'look'):
            out['boxes'] = self._observe_boxes(now, frame_id, image, pose, cov, servo)
        return out

    def _detections(self, image, servo):
        if self._detect is not None:
            return self._detect(image, servo)
        from harness.zone_color_boxes import OWN_PROFILE_ZONE, detect_own
        return detect_own(image, servo, profile=OWN_PROFILE_ZONE)['detections']

    def _observe_boxes(self, now, frame_id, image, pose, cov, servo) -> list[dict]:
        dets = self._detections(image, servo)
        self.counts['box_frames'] += 1
        self.counts['box_detections'] += len(dets)
        meas, rows = [], []
        for d in dets:
            bx, by = d['estimated_box_center_base_m'][:2]
            rng = math.hypot(bx, by)
            a, b = NEAR_SIGMA_M if d['range_class'] == 'near' else FAR_SIGMA_M
            z, R = observation_to_map(pose, cov, (bx, by), a + b*rng)
            meas.append((z, R))
            rows.append({'kind': d['kind'], 'range_class': d['range_class'], 'map_xy': [round(float(v), 4) for v in z],
                         'base_xy': [round(float(bx), 4), round(float(by), 4)], 'range_m': round(rng, 3),
                         'sigma_m': round(float(math.sqrt(max(np.linalg.eigvalsh(R).max(), 0.))), 4)})
        updated: set[str] = set()
        for kind in sorted({r['kind'] for r in rows}):
            idx = [i for i, r in enumerate(rows) if r['kind'] == kind]
            cands = [t for t in self.tracks if t.kind == kind and t.state not in ('held', 'placed')]
            decisions = associate([(t.x, t.P) for t in cands], [meas[i] for i in idx])
            for dec in decisions:
                i = idx[dec['measurement']]
                z, R = meas[i]
                rows[i]['decision'] = dec['decision']
                if dec['decision'] == 'update':
                    tr = cands[dec['track']]
                    tr.x, tr.P, _ = kf_update(tr.x, tr.P, z, R)
                    # Frames share the pose error, so repeated updates are not independent:
                    # keep the track covariance above a floor instead of shrinking to zero.
                    w, v = np.linalg.eigh(tr.P)
                    tr.P = (v*np.maximum(w, TRACK_INIT_FLOOR_M**2)) @ v.T
                    tr.last_seen_t = float(now)
                    tr.misses = 0
                    tr.frames.append(int(frame_id))
                    if rows[i]['range_class'] == 'near':
                        tr.near_hits += 1
                    else:
                        tr.far_hits += 1
                    if tr.state == 'absent':
                        tr.state = 'confirmed' if tr.near_hits >= CONFIRM_NEAR_HITS else 'tentative'
                        self.event(now, 'track_reappeared', track=tr.record(now))
                    if tr._maybe_confirm(now):
                        self.event(now, 'track_confirmed', track=tr.record(now))
                    rows[i]['track'] = tr.track_id
                    updated.add(tr.track_id)
                elif dec['decision'] == 'new':
                    tr = BoxTrack(f'{self.robot_id}-box-{self._next_track:03d}', kind, z, R, now, frame_id,
                                  rows[i]['range_class'])
                    self._next_track += 1
                    self.tracks.append(tr)
                    rows[i]['track'] = tr.track_id
                    updated.add(tr.track_id)
                    self.event(now, 'track_new', track=tr.record(now), detection=rows[i])
                else:
                    self.counts['ambiguous_detections'] += 1
        # absence evidence: a remembered box that should be in this near view was not detected
        for tr in self.tracks:
            if tr.track_id in updated or tr.state not in ('tentative', 'confirmed', 'claimed'):
                continue
            cam = self.view.camera_world(pose, servo)
            if math.hypot(tr.x[0] - cam[0], tr.x[1] - cam[1]) > ABSENT_RANGE_M:
                continue
            if self.view.point_in_view(pose, servo, False, (tr.x[0], tr.x[1], BOX_CENTRE_Z_M)):
                tr.misses += 1
                tr.last_absent_t = float(now)
                if tr.misses >= ABSENT_MISSES and tr.state != 'claimed':
                    tr.state = 'absent'
                    self.event(now, 'track_absent', track=tr.record(now))
        self._observe_floor(now, pose, servo, rows)
        return rows

    def _observe_floor(self, now, pose, servo, rows) -> None:
        near_idx, _ = self.view.floor_footprint(pose, servo, False, max_range=FREE_RANGE_M)
        far_idx, _ = self.view.floor_footprint(pose, servo, False, min_range=FAR_RANGE_M[0], max_range=FAR_RANGE_M[1])
        occ_near = np.zeros(len(self.view.cells), bool)
        clear = np.zeros(len(self.view.cells), bool)
        for r in rows:
            d = np.hypot(self.view.cells[:, 0] - r['map_xy'][0], self.view.cells[:, 1] - r['map_xy'][1])
            clear |= d <= max(DETECTION_CLEAR_M, 2*r['sigma_m'])
            if r['range_class'] == 'near':
                occ_near |= d <= KEEPOUT_BASE_HALF_M + GRID_M/2
            else:
                self.log_odds[d <= GRID_M] += L_OCC_FAR
        free = near_idx[~clear[near_idx]]
        self.log_odds[free] += L_FREE
        self.log_odds[occ_near] += L_OCC
        np.clip(self.log_odds, L_MIN, L_MAX, out=self.log_odds)
        far_free = far_idx[~clear[far_idx]]
        self.far_seen[far_free] = np.minimum(self.far_seen[far_free] + 1, 1000)
        self.counts['free_updates'] += int(len(free))
        self.counts['occupied_updates'] += int(occ_near.sum())

    def observe_blocked(self, now: float, centre_xy: Sequence[float], half_xy: Sequence[float], source: str) -> None:
        """External own-camera blockage judgement (e.g. PR #193 ``judge_route_blockage`` 'yes')."""
        self._decay_to(now)
        m = np.all(np.abs(self.view.cells - np.asarray(centre_xy, float)) <= np.asarray(half_xy, float), axis=1)
        self.log_odds[m] = np.clip(self.log_odds[m] + L_OCC, L_MIN, L_MAX)
        self.event(now, 'blocked_observed', centre=list(centre_xy), half=list(half_xy), source=source)

    # ------------------------------------------------------------ expected view
    def view_missing(self) -> bool:
        return self.missing_run >= MISSING_FRAMES

    def no_tag_while_expected(self, now: float) -> bool:
        """Unloaded: no tag for NO_TAG_S although the map predicted some in view meanwhile."""
        last = self.last_detect_t
        return (last is not None and now - last > NO_TAG_S and self.last_plan_visible_t is not None
                and self.last_plan_visible_t > last)

    def reset_view_checks(self) -> None:
        self.missing_run = 0

    # ------------------------------------------------------------ look planning
    def _pan_detect_prob(self, pose, posture, loaded, pan, visible) -> float:
        """P(at least one tag detected at this pan): the map model, corrected by what this robot
        remembers from its own frames in this cell / posture / load / pan (Beta, prior weight 2)."""
        model = 1. - float(np.prod([1. - v['p_detect'] for v in visible]))
        trials, hits = self.pan_stats.get(self._stat_key(pose, posture, loaded, pan), (0, 0))
        return (hits + 2*model)/(trials + 2)

    def plan_look(self, estimate: Mapping, *, loaded: bool, now: float, reason: str, start_pan: int = 1500,
                  candidate_pans: Sequence[int] = LOOK_CANDIDATE_PANS, max_pans: int = MAX_SHORT_PANS) -> dict:
        """Greedy pan selection for a LOOK_P20 look (empty ``pans`` = do a full look)."""
        if not estimate.get('initialized'):
            return {'pans': [], 'mode': 'full', 'why': 'not_initialized'}
        pose = np.array([estimate['x'], estimate['y'], estimate['yaw']], float)
        P0 = np.asarray(estimate['cov'], float).reshape(3, 3) + np.diag([1e-8, 1e-8, 1e-10])
        info0 = np.linalg.inv(P0)
        per_pan = {}
        for pan in candidate_pans:
            servo = {**LOOK_P20, 6: int(pan)}
            vis = self.view.visible_tags(pose, servo, loaded)
            if not vis:
                per_pan[pan] = {'tags': [], 'p_detect': 0., 'info': np.zeros((3, 3))}
                continue
            p = self._pan_detect_prob(pose, 'look', loaded, pan, vis)
            model = 1. - float(np.prod([1. - v['p_detect'] for v in vis]))
            # the per-tag model already weights the information; rescale it to the remembered rate
            per_pan[pan] = {'tags': [v['id'] for v in vis], 'p_detect': round(p, 3),
                            'info': (p/max(model, 1e-6))*self.view.fisher(pose, servo, loaded, vis)}
        chosen, info, last = [], info0.copy(), int(start_pan)
        while len(chosen) < max_pans:
            best = None
            for pan, row in per_pan.items():
                if pan in chosen or not row['tags']:
                    continue
                gain = _logdet(info + row['info']) - _logdet(info)
                cost = SETTLE_S + abs(int(pan) - last)/PAN_RATE_PWM_S
                if gain >= MIN_GAIN_NATS and (best is None or gain/cost > best[0]):
                    best = (gain/cost, pan, gain)
            if best is None:
                break
            chosen.append(best[1])
            info = info + per_pan[best[1]]['info']
            last = int(best[1])
        summary = {str(p): {'tags': r['tags'], 'p_detect': r['p_detect']} for p, r in per_pan.items()}
        if not chosen:
            self.counts['looks_full'] += 1
            plan = {'pans': [], 'mode': 'full', 'why': 'map_predicts_too_little', 'per_pan': summary}
        else:
            # Greedy order (best gain per second first, travel already in the cost), so an early
            # stop after the first dwell keeps the most informative pan.
            order = list(chosen)
            post = np.linalg.inv(info)
            self.counts['looks_planned_short'] += 1
            plan = {'pans': order, 'mode': 'short', 'why': 'fisher_greedy', 'per_pan': summary,
                    'predicted_std_xy_m': round(float(math.sqrt(max(post[0, 0] + post[1, 1], 0.))), 5),
                    'predicted_std_yaw_rad': round(float(math.sqrt(max(post[2, 2], 0.))), 6),
                    'prior_std_xy_m': round(float(math.sqrt(max(P0[0, 0] + P0[1, 1], 0.))), 5),
                    'prior_std_yaw_rad': round(float(math.sqrt(max(P0[2, 2], 0.))), 6)}
        self.event(now, 'look_plan', reason=reason, loaded=bool(loaded), **{k: v for k, v in plan.items()
                                                                           if k != 'per_pan'})
        return plan

    @staticmethod
    def _order_pans(pans: Sequence[int], start: int) -> list[int]:
        """Visit order with the least pan travel from ``start`` and back to it."""
        up = sorted(p for p in pans if p >= start)
        down = sorted((p for p in pans if p < start), reverse=True)

        def travel(seq):
            cur, total = start, 0
            for p in seq:
                total += abs(p - cur)
                cur = p
            return total + abs(cur - start)
        a, b = up + down, down + up
        return list(a if travel(a) <= travel(b) else b)

    # ------------------------------------------------------------ search coverage
    def view_coverage(self, pose: Sequence[float], servo: Mapping[int, int]) -> dict:
        """Unknown share of the near (free-space) and far footprints inside the pickup region."""
        mask = self.view.pickup_mask
        near, _ = self.view.floor_footprint(pose, servo, False, max_range=FREE_RANGE_M, mask=mask)
        far, _ = self.view.floor_footprint(pose, servo, False, min_range=FAR_RANGE_M[0], max_range=FAR_RANGE_M[1],
                                           mask=mask)
        near_unknown = int(np.sum(np.abs(self.log_odds[near]) < L_KNOWN))
        far_unknown = int(np.sum(self.far_seen[far] < FAR_SEEN_KNOWN))
        return {'near_cells': int(len(near)), 'near_unknown': near_unknown, 'far_cells': int(len(far)),
                'far_unknown': far_unknown,
                'near_unknown_frac': round(near_unknown/len(near), 3) if len(near) else 0.,
                'far_unknown_frac': round(far_unknown/len(far), 3) if len(far) else 0.}

    def plan_search_pans(self, pose: Sequence[float], pans: Sequence[int], *, min_frac: float = .2,
                         min_cells: int = 10) -> dict:
        rows, keep = {}, []
        for pan in pans:
            cov = self.view_coverage(pose, {**SEARCH_POSE, 6: int(pan)})
            rows[str(pan)] = cov
            useful = ((cov['near_cells'] >= min_cells and cov['near_unknown_frac'] >= min_frac) or
                      (cov['far_cells'] >= min_cells and cov['far_unknown_frac'] >= min_frac))
            if useful and int(pan) not in keep:
                keep.append(int(pan))
        return {'pans': keep, 'coverage': rows}

    # ------------------------------------------------------------ boxes
    def _in_region(self, xy, margin: float) -> bool:
        if self.view.pickup is None:
            return True
        c, h = self.view.pickup
        return bool(np.all(np.abs(np.asarray(xy, float) - c) <= h + margin))

    def track(self, track_id: str | None) -> BoxTrack | None:
        return next((t for t in self.tracks if t.track_id == track_id), None)

    def fresh(self, tr: BoxTrack, now: float) -> bool:
        tr.predict(now)
        return now - tr.last_seen_t <= STALE_AGE_S and tr.sigma_m() <= STALE_SIGMA_M

    def best_target(self, kind: str, now: float, near_xy: Sequence[float] | None = None) -> BoxTrack | None:
        """Nearest confirmed, fresh track of ``kind`` in the pickup region (own RGB only)."""
        cands = [t for t in self.tracks if t.kind == kind and t.state in ('confirmed', 'claimed')
                 and self.fresh(t, now) and self._in_region(t.x, .10)]
        if not cands:
            return None
        ref = np.asarray(near_xy, float) if near_xy is not None else None
        return min(cands, key=lambda t: (0. if ref is None else float(np.linalg.norm(t.x - ref)), t.sigma_m()))

    def best_far(self, kind: str, now: float, exclude: Sequence[str] = ()) -> BoxTrack | None:
        cands = [t for t in self.tracks if t.kind == kind and t.state == 'tentative' and t.track_id not in exclude
                 and self._in_region(t.x, .50)]
        return min(cands, key=lambda t: t.sigma_m()) if cands else None

    def claim(self, track_id: str, now: float) -> None:
        tr = self.track(track_id)
        if tr is not None:
            self.claimed = track_id
            tr.state = 'claimed'
            self.event(now, 'track_claimed', track=tr.record(now))

    def mark_held(self, now: float) -> None:
        tr = self.track(self.claimed)
        if tr is not None and tr.state == 'claimed':
            tr.state = 'held'
            self.event(now, 'track_held', track=tr.record(now))

    def mark_released(self, now: float) -> None:
        tr = self.track(self.claimed)
        if tr is not None and tr.state == 'held':
            tr.state = 'placed'
            self.event(now, 'track_placed', track=tr.record(now))
            self.claimed = None

    def reverify(self, track_id: str | None, now: float) -> dict:
        """Before a grasp: the remembered target must still be confirmed, fresh and not absent."""
        tr = self.track(track_id)
        if tr is None:
            return {'status': 'missing'}
        status = ('absent' if tr.state == 'absent' or tr.misses >= ABSENT_MISSES else
                  'fresh' if tr.state in ('confirmed', 'claimed') and self.fresh(tr, now) else 'stale')
        row = {'status': status, 'track': tr.record(now)}
        self.event(now, 'reverify', **row)
        return row

    def keepouts(self, exclude: Sequence[str] = ()) -> list[dict]:
        """Planner keep-outs: confirmed box tracks, inflated by their uncertainty."""
        out = []
        for tr in self.tracks:
            if tr.track_id in exclude or tr.state not in ('confirmed', 'claimed'):
                continue
            half = KEEPOUT_BASE_HALF_M + min(2*tr.sigma_m(), KEEPOUT_MAX_EXTRA_M)
            out.append({'id': tr.track_id, 'center_m': [float(tr.x[0]), float(tr.x[1])],
                        'half_extents_m': [half, half],
                        'source': f'own RGB memory track ({tr.kind}, {tr.near_hits} near hits, sigma '
                                  f'{tr.sigma_m():.3f} m)'})
        return out

    def slot_state(self, now: float, centre_xy: Sequence[float], half_xy: Sequence[float],
                   exclude: Sequence[str] = ()) -> dict:
        """Remembered occupancy of a destination slot: occupied / free / unknown."""
        self._decay_to(now)
        c, h = np.asarray(centre_xy, float), np.asarray(half_xy, float)
        occupants = [tr.record(now) for tr in self.tracks if tr.track_id not in exclude
                     and tr.state in ('confirmed',) and np.all(np.abs(tr.x - c) <= h + .02)]
        cells = np.all(np.abs(self.view.cells - c) <= h, axis=1)
        lo = self.log_odds[cells]
        state = ('occupied' if occupants else 'free' if len(lo) and np.all(lo <= -L_KNOWN) else 'unknown')
        row = {'state': state, 'occupants': occupants, 'cells': int(cells.sum()),
               'known_free_cells': int(np.sum(lo <= -L_KNOWN))}
        self.event(now, 'slot_check', slot_centre=list(map(float, c)), **row)
        return row

    # ------------------------------------------------------------ output
    def snapshot(self, now: float) -> dict:
        free = int(np.sum(self.log_odds <= -L_KNOWN))
        occ = int(np.sum(self.log_odds >= L_KNOWN))
        pm = self.view.pickup_mask
        return {'schema': SCHEMA, 'robot_id': self.robot_id, 't': round(float(now), 3),
                'tracks': [t.record(now) for t in self.tracks],
                'last_fix': copy.deepcopy(self.last_fix), 'fix_frames': len(self.fixes),
                'grid': {'cell_m': GRID_M, 'known_free_cells': free, 'known_occupied_cells': occ,
                         'pickup_cells': int(pm.sum()),
                         'pickup_known_near': int(np.sum(np.abs(self.log_odds[pm]) >= L_KNOWN)),
                         'pickup_far_seen': int(np.sum(self.far_seen[pm] >= FAR_SEEN_KNOWN))},
                'pan_stats': [{'cell': [k[0], k[1]], 'posture': k[2], 'loaded': k[3], 'pan': k[4],
                               'trials': v[0], 'hits': v[1]} for k, v in sorted(self.pan_stats.items())],
                'counts': dict(self.counts), 'config': dict(CONFIG), 'kf_sources': KF_SOURCES}

    def grid_record(self) -> dict:
        """Final grid for offline audit (flat cell order = ViewModel.cells)."""
        nz = np.flatnonzero(np.abs(self.log_odds) >= .05)
        return {'cell_m': GRID_M, 'bounds_m': list(self.view.bounds), 'shape': list(self.view.grid_shape),
                'cells': nz.tolist(), 'log_odds': np.round(self.log_odds[nz], 3).tolist(),
                'far_seen_cells': np.flatnonzero(self.far_seen > 0).tolist()}
