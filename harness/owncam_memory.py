"""Own-camera observation memory for one robot: "look once, remember" (v2, landmark-agnostic).

Inputs (and nothing else): this robot's own ``robot_cam`` frames, its own ``PoseReport``
(the own-camera particle filter), its own issued servo pulses, the static map, the fixed
calibrations and ONE landmark provider (``harness.owncam_landmarks``). It never imports
the simulator, never reads TOP, ``nav_cam`` or another robot, and is never shared.

v2 (2026-09-26, prereg amendment A1-A3; v1 = commit ad78ef2, run in dev-a1 only)

* A1 landmark-agnostic (user: tags are being removed from the environment). Nothing
  here reads the tag list. The memory stores generic landmark observations (landmark id
  and type from the static-map catalogue, bearing / range, own pose estimate and sigma at
  that time, frame id, provider) and plans "where to look" from the catalogue geometry
  (door posts, wall corners, door gaps, wall faces). The provider says how well it can
  observe each catalogue landmark from a view. Today's runs use the INTERIM tag provider
  (``harness.owncam_landmark_tags``), labelled "interim, tag provider".
* A2 own-RGB box positions are re-projected with the camera elevation bias the PF
  calibration already estimated (``correct_box_detection``; dev-a1 showed a 0.10 m range
  bias at 1.1 m that this removes, ``experiments/2026-09-26-zone-owncam-memory/
  box_bias_check.json``), and a track is never more certain than the own pose was at its
  best observation (correlated pose error).
* A3 stale pose fix: a stationary look-posture fix decays with travel and time; the
  driver may skip an arrival check only with a fresh one (dev-a1: driving-only fixes let
  the pose drift 0.16 m before the grasp with sigma 0.03 m).

What it remembers

* Landmark observations and fixes: the full observation log, the last fix, the last
  stationary look fix, and per 0.5 m cell x posture x load x pan how often the provider
  observed the landmarks the map places in view (the look planner's detection prior).
* Box tracks: one static 2-D Kalman filter per box (``harness.owncam_memory_kf``:
  filterpy predict/update, PythonRobotics EKF-SLAM association); first/last seen,
  near/far hits, misses, state tentative / confirmed / absent / claimed / held / placed;
  staleness = covariance growth + age.
* Free / blocked floor: a log-odds occupancy grid (Elfes 1989, clamped as in OctoMap)
  over the static map; decays toward unknown.
"""
from __future__ import annotations

import copy
import math
from collections.abc import Callable, Mapping, Sequence

import cv2
import numpy as np

from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SEARCH_POSE, SETTLE_S
from harness.owncam_landmarks import GeometricLandmarkProvider, LandmarkCatalogue, LandmarkObservation
from harness.owncam_memory_kf import SOURCES as KF_SOURCES
from harness.owncam_memory_kf import associate, kf_predict, kf_update, observation_to_map
from harness.visual_arm import camera_extrinsics
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

SCHEMA = 'ugrp.owncam_memory.v2'
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
# Landmark planning rows (floor contact points of the catalogue landmarks).
PLAN_MARGIN_PX, PLAN_MAX_RANGE_M, PLAN_LOADED_ROW_LIMIT_PX = 4., 3.5, 168.
PLAN_MIN_SUPPORT = .05
# Strict expectation (the "expected view is missing" trigger).
EXPECT_MARGIN_PX, EXPECT_MAX_RANGE_M, EXPECT_LOADED_ROW_LIMIT_PX = 18., 1.8, 132.
EXPECT_MIN_SUPPORT = .9
MISSING_FRAMES = 3
NO_LANDMARK_S = 3.                   # unloaded: nothing observed this long although the map predicted some
# A3: a stationary look-posture fix is fresh for this much own travel / SIM time.
LOOK_FIX_STALE_M = .5
LOOK_FIX_STALE_S = 60.
# Boxes (own-RGB detections; box centre 0.016 m above the floor).
BOX_CENTRE_Z_M = .016
NEAR_SIGMA_M = (.012, .02)           # detection noise = a + b*range (after the A2 correction)
FAR_SIGMA_M = (.10, .20)             # far_coarse
TRACK_Q_M2_S = 1e-5                  # static boxes: slow covariance growth (staleness)
TRACK_INIT_FLOOR_M = .01
TRACK_FLOOR_M = .02                  # A2: floor of a track's sigma (frames share the pose error)
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
OBS_LOG_IN_SNAPSHOT = 200
CONFIG = {k: v for k, v in dict(globals()).items() if k.isupper() and isinstance(v, (int, float, tuple, str))}


def camera_in_base(servo: Mapping[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """(origin (3,), R_bc (3x3, columns = optical axes in base)) from the issued PWM (commanded-pose FK)."""
    origin, axes = camera_extrinsics(servo)
    return np.asarray(origin, float), np.asarray(axes, float).T


def posture_name(servo: Mapping[int, int]) -> str:
    for name, pose in POSTURES.items():
        if all(abs(int(servo.get(s, -9999)) - int(pose[s])) <= POSTURE_TOL_PWM for s in (3, 4, 5)):
            return name
    return 'other'


def _wrap(a):
    return (np.asarray(a) + np.pi) % (2*np.pi) - np.pi


def _logdet(m: np.ndarray) -> float:
    sign, value = np.linalg.slogdet(m)
    return float(value) if sign > 0 else -math.inf


def _camera_correction(params: Mapping, loaded: bool) -> dict:
    mp = dict(params['measurement'])
    if loaded:
        mp.update(params.get('measurement_loaded') or {})
    return mp


def correct_box_detection(base_xy: Sequence[float], servo: Mapping[int, int], params: Mapping, *,
                          loaded: bool = False, z: float = BOX_CENTRE_Z_M) -> tuple[float, float]:
    """A2: re-project an own-RGB box position with the PF's calibrated camera model.

    ``harness.zone_color_boxes.detect_own`` inverts the pixel ray with the nominal
    commanded-PWM camera. The own-camera PF calibration (fixed; fitted for the tag
    measurements) says the real camera sees a nominal camera point ``p`` at
    ``Rb (p - delta + omega x p)`` (``camera_correction``, then the elevation bias ``b``).
    Here the observed ray is mapped back through that model and intersected with the
    box-centre plane ``z``. Nothing is fitted.
    """
    mp = _camera_correction(params, loaded)
    o, r_bc = camera_in_base(servo)
    p_c = (np.array([float(base_xy[0]), float(base_xy[1]), z]) - o) @ r_bc      # observed ray (nominal frame)
    bias = float(mp.get('elevation_bias_rad', 0.) or 0.)
    cb, sb = math.cos(bias), math.sin(bias)
    w = np.array([p_c[0], p_c[1]*cb - p_c[2]*sb, p_c[1]*sb + p_c[2]*cb])        # Rb^T v
    delta, A = np.zeros(3), np.eye(3)
    corr = mp.get('camera_correction')
    if corr:
        om = np.asarray(corr['omega_rad'], float)
        delta = np.asarray(corr['delta_m'], float)
        A = np.linalg.inv(np.eye(3) + np.array([[0., -om[2], om[1]], [om[2], 0., -om[0]], [-om[1], om[0], 0.]]))
    a, b = r_bc @ (A @ w), r_bc @ (A @ delta)
    if abs(a[2]) < 1e-9:
        return float(base_xy[0]), float(base_xy[1])
    lam = (z - o[2] - b[2])/a[2]
    q = o + lam*a + b
    return float(q[0]), float(q[1])


class ViewModel:
    """Static-map geometry seen through this robot's wrist fisheye (static map + fixed calibration)."""

    def __init__(self, static_map: Mapping, params: Mapping, width: int = IMAGE_W, height: int = IMAGE_H):
        self.K = scaled_camera_matrix(width, height)
        self.D = np.asarray(CAMERA_FISHEYE_D, float).reshape(4, 1)
        self.f = float(self.K[0, 0])
        occ = [(o['center_m'], o['half_extents_m'], float(o.get('height_m', .10)))
               for o in static_map.get('obstacles', []) if o.get('kind') == 'wall']
        occ += [(p['center_m'], p['half_extents_m'], float(p['height_m']))
                for p in (static_map.get('landmarks') or {}).get('door_posts', []) or []]
        self.occluders = [(np.asarray(c, float)[:2], np.asarray(h, float)[:2], z) for c, h, z in occ]
        self.meas = {False: _camera_correction(params, False), True: _camera_correction(params, True)}
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

    # ------------------------------------------------------------ static-map landmarks
    def visible_landmarks(self, catalogue: LandmarkCatalogue, pose: Sequence[float], servo: Mapping[int, int],
                          loaded: bool, *, strict: bool = False, sigma: tuple[float, float] | None = None) -> list[dict]:
        """Catalogue planning points this camera sees from ``pose`` with the issued pulses.

        A landmark is a vertical extent (floor contact to its wall / post height): it is in view
        if its floor point, mid-height or top point is (inside the frame, above the held box,
        in range, seen from a free side, not behind a wall). The lowest visible point is the
        planning measurement point.
        """
        plan = catalogue.plan_points()
        n = len(plan['ids'])
        if not n:
            return []
        pose = np.asarray(pose, float)
        levels = [0., .5, 1.]
        pts = np.concatenate([np.column_stack((plan['xy'], f*plan['height'])) for f in levels])
        p_c = self.to_camera(pts, pose, servo, loaded)[0]
        px, ideal, ok = self.project(p_c)
        rng = np.linalg.norm(p_c, axis=1)
        if strict:
            sx, syaw = sigma if sigma is not None else (0., 0.)
            mx = EXPECT_MARGIN_PX + self.f*(2*syaw + 2*sx/np.maximum(rng, .3))
            my, row_limit, max_range = np.full(len(rng), EXPECT_MARGIN_PX), EXPECT_LOADED_ROW_LIMIT_PX, EXPECT_MAX_RANGE_M
        else:
            mx = my = np.full(len(rng), PLAN_MARGIN_PX)
            row_limit, max_range = PLAN_LOADED_ROW_LIMIT_PX, PLAN_MAX_RANGE_M
        keep = (ok & self.in_view(px, ideal, loaded, mx, my, row_limit) & (rng <= max_range)).reshape(len(levels), n)
        cand = np.flatnonzero(keep.any(axis=0))
        if not len(cand):
            return []
        cam = self.camera_world(pose, servo)
        cand = cand[catalogue.seen_from(cand, cam[:2])]
        rows = []
        for i in cand:
            towards = cam[:2] - plan['xy'][i]
            towards /= max(float(np.linalg.norm(towards)), 1e-9)
            for li, f in enumerate(levels):
                if not keep[li, i]:
                    continue
                probe = np.array([[*(plan['xy'][i] + .01*towards), f*plan['height'][i]]])
                if self.occluded(cam, probe)[0]:
                    continue
                k = li*n + i
                rows.append({'id': plan['ids'][i], 'type': plan['types'][i], 'plan_index': int(i),
                             'xy': [round(float(v), 4) for v in plan['xy'][i]], 'z': round(float(f*plan['height'][i]), 4),
                             'px': [round(float(v), 1) for v in px[k]], 'range_m': round(float(rng[k]), 3),
                             'face_dir': plan['face_dir'][i].tolist()})
                break
        return rows

    def _bearings(self, poses: np.ndarray, rows: Sequence[Mapping], servo, loaded) -> np.ndarray:
        pts = np.array([[r['xy'][0], r['xy'][1], r.get('z', 0.)] for r in rows], float)
        p_c = self.to_camera(pts, poses, servo, loaded)
        return np.stack((np.arctan2(p_c[..., 0], p_c[..., 2]), np.arctan2(p_c[..., 1], p_c[..., 2]),
                         np.log(np.maximum(np.linalg.norm(p_c, axis=-1), 1e-6))), axis=-1)

    def landmark_fisher(self, pose: Sequence[float], servo: Mapping[int, int], loaded: bool, rows: Sequence[Mapping],
                        weights: np.ndarray, noise: Mapping[str, float], *, identified_face_points: bool,
                        frames: int = FRAMES_PER_DWELL) -> np.ndarray:
        """Pose information (3x3) of ``frames`` frames of the provider's expected landmark observations.

        Azimuth and elevation (and log range when the provider measures it) of each row's
        floor point, weighted by the provider's detection probability; tempered like the
        own-camera PF (joint log-likelihood / n**temper). Wall-face rows of a provider whose
        face features are not identified carry no information along the face.
        """
        if not rows:
            return np.zeros((3, 3))
        eps = np.array([1e-3, 1e-3, 1e-3])
        base = np.asarray(pose, float)
        m = self._bearings(np.vstack([base, base + np.diag(eps)]), rows, servo, loaded)   # (4, M, 3)
        diff = m[1:] - m[:1]
        diff[..., 0] = _wrap(diff[..., 0])
        J = diff/eps[:, None, None]                                           # (3 state, M, 3 meas)
        stds = [noise['azimuth_std_rad'], noise['elevation_std_rad']]
        use = [0, 1]
        if noise.get('range_log_std'):
            stds.append(noise['range_log_std'])
            use.append(2)
        W = np.diag(1./np.asarray(stds)**2)
        info = np.zeros((3, 3))
        for i, r in enumerate(rows):
            H = J[:, i, use].T                                                # (meas, 3 state)
            I_r = float(weights[i])*(H.T @ W @ H)
            if r['type'] == 'wall_face' and not identified_face_points:
                u = np.array([r['face_dir'][0], r['face_dir'][1], 0.])
                P = np.eye(3) - np.outer(u, u)
                I_r = P @ I_r @ P
            info += I_r
        n = max(1., float(np.sum(weights)))
        return frames*info/(n**float(noise.get('temper', 0.)))

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
                 range_class: str, pose_var: float):
        self.track_id, self.kind = track_id, kind
        self.x = np.asarray(z, float).copy()
        self.P = np.asarray(R, float) + (TRACK_INIT_FLOOR_M**2)*np.eye(2)
        self.floor_var = max(TRACK_FLOOR_M**2, float(pose_var))
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
                'floor_sigma_m': round(math.sqrt(self.floor_var), 4), 'frames': self.frames[-8:]}


class OwnCamMemory:
    """Per-robot memory; feed every own frame with ``observe_frame``."""

    def __init__(self, static_map: Mapping, params: Mapping, *, robot_id: str, provider=None,
                 on_event: Callable[[str, dict], None] | None = None, detect=None):
        self.robot_id = robot_id
        self.params = params
        self.view = ViewModel(static_map, params)
        self.catalogue = LandmarkCatalogue(static_map)
        self.provider = provider if provider is not None else GeometricLandmarkProvider()
        self.provider.bind(self.view, self.catalogue)
        self.on_event = on_event
        self._detect = detect
        self.t = None
        self.tracks: list[BoxTrack] = []
        self._next_track = 0
        self.log_odds = np.zeros(len(self.view.cells))
        self.far_seen = np.zeros(len(self.view.cells), np.int16)
        self.pan_stats: dict[tuple, list[int]] = {}
        self.observations: list[dict] = []
        self.obs_by_type: dict[str, int] = {}
        self.fix_frames = 0
        self.last_fix: dict | None = None
        self.last_look_fix: dict | None = None
        self.missing_run = 0
        self.last_detect_t: float | None = None
        self.last_plan_visible_t: float | None = None
        self.claimed: str | None = None
        self.counts = {'frames': 0, 'settled_frames': 0, 'box_frames': 0, 'box_detections': 0,
                       'free_updates': 0, 'occupied_updates': 0, 'view_missing_events': 0,
                       'ambiguous_detections': 0, 'looks_planned_short': 0, 'looks_full': 0,
                       'landmark_observations': 0, 'observations_uninitialized': 0}
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

    def _support(self, rows, pose, servo, loaded, **kw) -> dict:
        return self.provider.support(rows, pose, servo, loaded, **kw)

    # ------------------------------------------------------------ input
    def observe_frame(self, now: float, *, frame_id: int, image, servo: Mapping[int, int], report,
                      arm_settled_s: float, loaded: bool, provider_inputs: Mapping | None = None) -> dict:
        """One own frame: landmark observations, expected view, box tracks and floor (unloaded, settled)."""
        self._decay_to(now)
        self.counts['frames'] += 1
        servo = {int(k): int(v) for k, v in servo.items()}
        posture = posture_name(servo)
        settled = arm_settled_s >= SETTLED_S
        rows = self.provider.observe(image=image, servo=servo, loaded=bool(loaded), **dict(provider_inputs or {}))
        if rows:
            self.last_detect_t = float(now)
        out = {'posture': posture, 'settled': settled, 'observed': sorted({r['landmark_id'] for r in rows})}
        if not report.initialized:
            self.counts['observations_uninitialized'] += len(rows)
            return out
        pose = (report.x_m, report.y_m, report.yaw_rad)
        cov = np.asarray(report.cov, float).reshape(3, 3)
        obs = [LandmarkObservation(t=float(now), frame_id=int(frame_id), provider=self.provider.name,
                                   interim=bool(self.provider.interim), pose_xyyaw=tuple(float(v) for v in pose),
                                   std_xy_m=float(report.std_xy_m), std_yaw_rad=float(report.std_yaw_rad),
                                   posture=posture, settled=settled, **{**r, 'supports': tuple(r.get('supports', ()))})
               for r in rows]
        if obs:
            self.fix_frames += 1
            self.counts['landmark_observations'] += len(obs)
            for o in obs:
                self.observations.append(o.as_dict())
                self.obs_by_type[o.landmark_type] = self.obs_by_type.get(o.landmark_type, 0) + 1
            self.last_fix = {'t': round(float(now), 3), 'frame_id': int(frame_id), 'posture': posture,
                             'landmarks': sorted({o.landmark_id for o in obs}), 'provider': self.provider.name,
                             'interim': bool(self.provider.interim), 'xyyaw': [round(float(v), 4) for v in pose],
                             'std_xy_m': round(report.std_xy_m, 4), 'std_yaw_rad': round(report.std_yaw_rad, 5)}
            if settled and posture == 'look':
                self.last_look_fix = {**self.last_fix, 'xy': [float(pose[0]), float(pose[1])]}
        if not settled or posture == 'other':
            return out
        self.counts['settled_frames'] += 1
        observed = set()
        for o in obs:
            observed.update(o.supports or (o.landmark_id,))
        plan = self.view.visible_landmarks(self.catalogue, pose, servo, loaded)
        sup = self._support(plan, pose, servo, loaded)
        supported = {r['id'] for r, w in zip(plan, sup['w']) if w > PLAN_MIN_SUPPORT}
        if supported:
            self.last_plan_visible_t = float(now)
            key = self._stat_key(pose, posture, loaded, servo.get(6, 1500))
            st = self.pan_stats.setdefault(key, [0, 0])
            st[0] += 1
            st[1] += int(bool(observed & supported))
        # The missing-view check runs in the driving postures only (during a look the
        # estimate is being corrected; the check is reset when a look starts and ends).
        expected = set()
        if posture != 'look':
            sigma = (report.std_xy_m, report.std_yaw_rad)
            strict = self.view.visible_landmarks(self.catalogue, pose, servo, loaded, strict=True, sigma=sigma)
            ssup = self._support(strict, pose, servo, loaded, strict=True, sigma=sigma)
            expected = {r['id'] for r, w in zip(strict, ssup['w']) if w >= EXPECT_MIN_SUPPORT}
        out['expected_landmarks'] = sorted(expected)
        if expected:
            if observed & expected:
                self.missing_run = 0
            else:
                self.missing_run += 1
                if self.missing_run == MISSING_FRAMES:
                    self.counts['view_missing_events'] += 1
                    self.event(now, 'view_missing', expected=sorted(expected), observed=sorted(observed),
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
            rx, ry = d['estimated_box_center_base_m'][:2]
            bx, by = correct_box_detection((rx, ry), servo, self.params, loaded=False)
            rng = math.hypot(bx, by)
            a, b = NEAR_SIGMA_M if d['range_class'] == 'near' else FAR_SIGMA_M
            sig = a + b*rng
            z, R = observation_to_map(pose, cov, (bx, by), sig)
            pose_var = max(float(np.linalg.eigvalsh(R - sig**2*np.eye(2)).min()), 0.)
            meas.append((z, R, pose_var))
            rows.append({'kind': d['kind'], 'range_class': d['range_class'], 'map_xy': [round(float(v), 4) for v in z],
                         'base_xy': [round(float(bx), 4), round(float(by), 4)],
                         'raw_base_xy': [round(float(rx), 4), round(float(ry), 4)], 'range_m': round(rng, 3),
                         'sigma_m': round(float(math.sqrt(max(np.linalg.eigvalsh(R).max(), 0.))), 4)})
        updated: set[str] = set()
        for kind in sorted({r['kind'] for r in rows}):
            idx = [i for i, r in enumerate(rows) if r['kind'] == kind]
            cands = [t for t in self.tracks if t.kind == kind and t.state not in ('held', 'placed')]
            decisions = associate([(t.x, t.P) for t in cands], [meas[i][:2] for i in idx])
            for dec in decisions:
                i = idx[dec['measurement']]
                z, R, pose_var = meas[i]
                rows[i]['decision'] = dec['decision']
                if dec['decision'] == 'update':
                    tr = cands[dec['track']]
                    tr.x, tr.P, _ = kf_update(tr.x, tr.P, z, R)
                    # Frames share the pose error, so repeated updates are not independent (A2):
                    # a track is never more certain than the pose at its best observation.
                    tr.floor_var = min(tr.floor_var, max(TRACK_FLOOR_M**2, pose_var))
                    w, v = np.linalg.eigh(tr.P)
                    tr.P = (v*np.maximum(w, tr.floor_var)) @ v.T
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
                                  rows[i]['range_class'], pose_var)
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

    # ------------------------------------------------------------ expected view / fix staleness
    def view_missing(self) -> bool:
        return self.missing_run >= MISSING_FRAMES

    def nothing_observed_while_expected(self, now: float) -> bool:
        """Unloaded: no landmark observed for NO_LANDMARK_S although the map predicted some in view."""
        last = self.last_detect_t
        return (last is not None and now - last > NO_LANDMARK_S and self.last_plan_visible_t is not None
                and self.last_plan_visible_t > last)

    def reset_view_checks(self) -> None:
        self.missing_run = 0

    def look_fix_fresh(self, now: float, xy: Sequence[float]) -> bool:
        """A3: a stationary look-posture fix within LOOK_FIX_STALE_M of own travel and LOOK_FIX_STALE_S."""
        f = self.last_look_fix
        return (f is not None and now - f['t'] <= LOOK_FIX_STALE_S
                and math.hypot(float(xy[0]) - f['xy'][0], float(xy[1]) - f['xy'][1]) <= LOOK_FIX_STALE_M)

    def look_fix_since(self, t: float) -> bool:
        return self.last_look_fix is not None and self.last_look_fix['t'] >= float(t) - 1e-9

    # ------------------------------------------------------------ look planning
    def _pan_detect_prob(self, pose, posture, loaded, pan, model: float) -> float:
        """P(observing at this pan): the map + provider model, corrected by what this robot
        remembers from its own frames in this cell / posture / load / pan (Beta, prior weight 2)."""
        trials, hits = self.pan_stats.get(self._stat_key(pose, posture, loaded, pan), (0, 0))
        return (hits + 2*model)/(trials + 2)

    def pan_view(self, pose, pan: int, loaded: bool) -> dict:
        servo = {**LOOK_P20, 6: int(pan)}
        rows = self.view.visible_landmarks(self.catalogue, pose, servo, loaded)
        sup = self._support(rows, pose, servo, loaded)
        keep = [i for i, w in enumerate(sup['w']) if w > PLAN_MIN_SUPPORT]
        return {'servo': servo, 'rows': [rows[i] for i in keep], 'w': np.asarray(sup['w'])[keep] if keep else np.zeros(0),
                'p_any': sup['p_any'], 'features': sup.get('features', [])}

    def plan_look(self, estimate: Mapping, *, loaded: bool, now: float, reason: str, start_pan: int = 1500,
                  candidate_pans: Sequence[int] = LOOK_CANDIDATE_PANS, max_pans: int = MAX_SHORT_PANS) -> dict:
        """Greedy pan selection for a LOOK_P20 look (empty ``pans`` = do a full look)."""
        if not estimate.get('initialized'):
            return {'pans': [], 'mode': 'full', 'why': 'not_initialized'}
        pose = np.array([estimate['x'], estimate['y'], estimate['yaw']], float)
        P0 = np.asarray(estimate['cov'], float).reshape(3, 3) + np.diag([1e-8, 1e-8, 1e-10])
        info0 = np.linalg.inv(P0)
        noise = self.provider.noise(loaded)
        per_pan = {}
        for pan in candidate_pans:
            v = self.pan_view(pose, pan, loaded)
            if not v['rows']:
                per_pan[pan] = {'landmarks': [], 'p_detect': 0., 'info': np.zeros((3, 3))}
                continue
            p = self._pan_detect_prob(pose, 'look', loaded, pan, v['p_any'])
            info = self.view.landmark_fisher(pose, v['servo'], loaded, v['rows'], v['w'], noise,
                                             identified_face_points=bool(self.provider.face_points_identified))
            per_pan[pan] = {'landmarks': sorted({r['id'] for r in v['rows']}), 'p_detect': round(p, 3),
                            'info': (p/max(v['p_any'], 1e-6))*info}
        chosen, info, last = [], info0.copy(), int(start_pan)
        while len(chosen) < max_pans:
            best = None
            for pan, row in per_pan.items():
                if pan in chosen or not row['landmarks']:
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
        summary = {str(p): {'landmarks': r['landmarks'], 'p_detect': r['p_detect']} for p, r in per_pan.items()}
        if not chosen:
            self.counts['looks_full'] += 1
            plan = {'pans': [], 'mode': 'full', 'why': 'map_predicts_too_little', 'per_pan': summary}
        else:
            # Greedy order (best gain per second first, travel already in the cost), so an early
            # stop after the first dwell keeps the most informative pan.
            post = np.linalg.inv(info)
            self.counts['looks_planned_short'] += 1
            plan = {'pans': list(chosen), 'mode': 'short', 'why': 'fisher_greedy', 'per_pan': summary,
                    'predicted_std_xy_m': round(float(math.sqrt(max(post[0, 0] + post[1, 1], 0.))), 5),
                    'predicted_std_yaw_rad': round(float(math.sqrt(max(post[2, 2], 0.))), 6),
                    'prior_std_xy_m': round(float(math.sqrt(max(P0[0, 0] + P0[1, 1], 0.))), 5),
                    'prior_std_yaw_rad': round(float(math.sqrt(max(P0[2, 2], 0.))), 6)}
        self.event(now, 'look_plan', reason=reason, loaded=bool(loaded), provider=self.provider.name,
                   interim=bool(self.provider.interim), **{k: v for k, v in plan.items() if k != 'per_pan'},
                   landmarks={p: r['landmarks'][:6] for p, r in summary.items()})
        return plan

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
                'catalogue': self.catalogue.describe(), 'provider': self.provider.describe(),
                'tracks': [t.record(now) for t in self.tracks],
                'last_fix': copy.deepcopy(self.last_fix), 'last_look_fix': copy.deepcopy(self.last_look_fix),
                'fix_frames': self.fix_frames, 'observations_by_type': dict(self.obs_by_type),
                'observations_tail': self.observations[-OBS_LOG_IN_SNAPSHOT:],
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
