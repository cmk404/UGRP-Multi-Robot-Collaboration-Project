"""``wrist_zone_skill_v6``: v5 + head-on face yaw, a public carry re-anchor hook, static keep-outs.

v1-v5 stay byte-identical as recorded results of
experiments/2026-09-25-zone-owncam-skill. v6 is a new profile requested by the
M1 integration (PR #201, dev-a2 s91/s92 ``GRASP_FACE_UNOBSERVABLE_AFTER_RELOOKS``):

(a) Head-on face yaw from the top-face far edge (``EdgeYawFaceAligner``).
    v5 voted over N7's floor-cuboid yaw fits. When one face is seen head-on
    (axis-aligned pickup-grid boxes) that fit is unstable: in recorded v4/v5
    approach frames with |relative yaw| <= 3 deg, 24.7 % of fits were more than
    7 deg off (p90 21 deg); the M1 agent measured 35-53 % on s91/s92. v6
    measures the yaw from the far edge of the box's top face instead: in the
    current own robot_cam frame, the topmost cyan pixel of each column of the
    target's component is undistorted with the fisheye model, cast through the
    issued-PWM camera pose and intersected with the top-face plane (z = box
    height); a RANSAC line through those floor-frame points gives the edge
    direction and hence the yaw (mod 90). A far top edge is a straight line in
    the floor frame for any yaw, so 0 deg needs no special case. Recorded-frame
    replay (same frames): median 0.30 deg, p90 1.16 deg, 0 % > 7 deg head-on;
    rotated 0.51 / 2.18 deg, 0.2 % > 7 deg. The vote (window 6, >= 3 fits and
    >= 60 % within ``FACE_INLIER_DEG``) and the face selection are v5's; only
    the per-frame evidence changes. N7's cuboid yaw is recorded as evidence and
    never voted. There is still no map fallback.

(b) ``WristOnlyBoxSkillV6.reanchor_after_probe(home_before, left, right,
    home_after)`` (and the delivery wrapper of the same name): a public
    replacement for writing ``_attachment_image`` / ``_carry_previous_image``
    after an own pan probe. Every frame passes N7's strict observation check;
    the pan deltas come from each frame's own issued PWM (not from the caller);
    the arm servos must be unchanged; and N7's post-lift probe evidence
    (``attachment_home_reference='previous_endpoint'``) is required: home->left,
    home->right, left->right (opposite endpoints) and right->home all attached.
    Only then do the carry anchor and the previous carry frame become
    ``home_after``.

(c) Static keep-outs for the pre-grasp approach point. The delivery accepts
    fixed discs from the static layout (e.g. idle peers' parking / spawn spots;
    never live peer poses). ``approach_point()`` keeps v5's point (bay west edge
    - 0.25 m, bay centre y, facing east) when it and the straight approach
    towards the bay clear every disc by the robot radius + margin, otherwise it
    tries the next candidate (shorter standoff, lateral offsets, then the south
    and north sides facing the bay) and ends with ``BAY_APPROACH_BLOCKED`` if
    none fits. It is compatible with the M1
    amendment A2 (bay half 0.15 m in x): with no keep-out in the way the point
    is exactly v5's. A guard stops the run with ``STATIC_KEEPOUT_GUARD`` if a
    chassis command would move the base further into a keep-out disc (own pose
    estimate; pure turns are allowed).
"""
from __future__ import annotations

import base64
import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

from harness import m1_contract
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v5 as v5
from harness.owncam_view import project_base_points, valid_pixel_mask
from harness.visual_arm import camera_extrinsics
from harness.visual_box_skill import VisualBoxSkill
from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix

PROFILE = 'wrist_zone_skill_v6'
_PERIOD = math.pi / 2

# ---- (a) top-face far-edge yaw ----
BOX_HEIGHT_M = .032
CYAN_HSV_LO, CYAN_HSV_HI = (75, 70, 40), (105, 255, 255)
EDGE_MIN_COMPONENT_PX = 150
EDGE_VALID_ERODE_PX = 11
EDGE_ABOVE_ROWS = (2, 6)            # the rows just above an edge pixel must be visible, non-cyan background
EDGE_VIGNETTE_MAX_V = 25            # darker than this is the lens vignette (a clipped box), not floor
EDGE_MAX_CLIPPED_FRACTION = .25     # more clipped columns than this: the far edge is not in view, refuse the frame
EDGE_RANSAC_ITERS = 200
EDGE_RANSAC_TOL_M = .0015
EDGE_MIN_INLIERS = 12
EDGE_MIN_SPAN_M = .015
EDGE_MAX_RESIDUAL_M = .0015
FACE_WINDOW = 6
FACE_MIN_INLIERS = 3
FACE_MIN_INLIER_FRACTION = .6
FACE_INLIER_DEG = 5.

# ---- (b) re-anchor probe ----
REANCHOR_MIN_PAN_DELTA_PWM = 40
REANCHOR_MAX_PAN_DELTA_PWM = 120
REANCHOR_HOME_TOL_PWM = 5
REANCHOR_ARM_SERVOS = ('1', '3', '4', '5')

# ---- (c) static keep-outs ----
KEEPOUT_SOURCES = ('static_layout_idle_spawn', 'static_map_parking')
ROBOT_RADIUS_M = .17
KEEPOUT_MARGIN_M = .05
KEEPOUT_GUARD_MARGIN_M = .02
APPROACH_GRASP_STANDOFF_M = .20     # the straight approach is checked up to this far west of the bay centre
APPROACH_OFFSETS = ((.25, 0.), (.20, 0.), (.15, 0.), (.25, .10), (.25, -.10), (.20, .10), (.20, -.10),
                    (.25, .20), (.25, -.20), (.15, .10), (.15, -.10), (.15, .20), (.15, -.20))
# (side, heading): the robot faces the bay. West is v5's side; south / north only when every west candidate is blocked.
APPROACH_SIDES = (('west', 0.), ('south', math.pi / 2), ('north', -math.pi / 2))

_VALID_MASK: np.ndarray | None = None


def _mod90_dist(a: float, b: float) -> float:
    d = (a - b) % _PERIOD
    return min(d, _PERIOD - d)


def _valid_mask() -> np.ndarray:
    global _VALID_MASK
    if _VALID_MASK is None:
        kernel = np.ones((EDGE_VALID_ERODE_PX, EDGE_VALID_ERODE_PX), np.uint8)
        _VALID_MASK = cv2.erode(valid_pixel_mask(1).astype(np.uint8), kernel).astype(bool)
    return _VALID_MASK


def decode_jpeg_b64(image: str) -> np.ndarray | None:
    try:
        data = base64.b64decode(image, validate=True)
    except Exception:                                     # noqa: BLE001 - any decode failure is "no frame"
        return None
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    return frame


def top_edge_yaw(frame: np.ndarray, pose: Mapping[int | str, int | float],
                 target_xy: Sequence[float] | None = None) -> dict[str, Any]:
    """Yaw (mod 90, robot base frame) of the far top edge of the target cyan box in one own frame."""
    if frame is None or frame.ndim != 3 or frame.shape[:2] != (480, 640):
        return {'ok': False, 'reason': 'NO_FRAME'}
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(CYAN_HSV_LO), np.array(CYAN_HSV_HI))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    comps = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= EDGE_MIN_COMPONENT_PX]
    if not comps:
        return {'ok': False, 'reason': 'NO_CYAN_COMPONENT'}
    comp = max(comps, key=lambda i: stats[i, cv2.CC_STAT_AREA])
    selection = 'largest'
    if target_xy is not None:
        px = project_base_points(pose, np.array([[float(target_xy[0]), float(target_xy[1]), BOX_HEIGHT_M / 2]]))[0]
        if np.all(np.isfinite(px)):
            comp = min(comps, key=lambda i: float(np.hypot(*(centroids[i] - px))))
            selection = 'nearest_projected_own_target'
    x0, _y0, w, _h = (int(v) for v in stats[comp, :4])
    inset = max(2, int(.1 * w))
    sel = labels == comp
    valid = _valid_mask()
    pixels = []
    columns = clipped = 0
    for col in range(x0 + inset, x0 + w - inset):
        rows = np.flatnonzero(sel[:, col])
        if not len(rows):
            continue
        columns += 1
        row = int(rows.min())
        lo, hi = row - EDGE_ABOVE_ROWS[1], row - EDGE_ABOVE_ROWS[0]
        if lo < 0 or not valid[row, col] or (hsv[lo:hi + 1, col, 2] <= EDGE_VIGNETTE_MAX_V).any():
            clipped += 1                                   # cut by the image / lens vignette: not the far edge
            continue
        if mask[lo:hi + 1, col].any():
            continue
        pixels.append((col, row))
    if columns and clipped > EDGE_MAX_CLIPPED_FRACTION * columns:
        return {'ok': False, 'reason': 'TOP_EDGE_CLIPPED', 'columns': columns, 'clipped': clipped}
    if len(pixels) < EDGE_MIN_INLIERS:
        return {'ok': False, 'reason': 'FEW_EDGE_PIXELS', 'pixels': len(pixels)}
    k = scaled_camera_matrix(640, 480)
    d = np.asarray(CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
    und = cv2.fisheye.undistortPoints(np.asarray(pixels, np.float64).reshape(1, -1, 2), k, d).reshape(-1, 2)
    origin, axes = camera_extrinsics(pose)
    origin = np.asarray(origin, np.float64)
    dirs = np.column_stack((und, np.ones(len(und)))) @ np.asarray(axes, np.float64)
    down = dirs[:, 2] < -1e-6
    if int(down.sum()) < EDGE_MIN_INLIERS:
        return {'ok': False, 'reason': 'FEW_EDGE_RAYS'}
    s = (BOX_HEIGHT_M - origin[2]) / dirs[down, 2]
    pts = origin[:2] + s[:, None] * dirs[down, :2]
    rng = np.random.default_rng(0)                         # deterministic
    best = None
    for _ in range(EDGE_RANSAC_ITERS):
        i, j = rng.choice(len(pts), 2, replace=False)
        vec = pts[j] - pts[i]
        length = float(np.linalg.norm(vec))
        if length < .005:
            continue
        normal = np.array([-vec[1], vec[0]]) / length
        inl = np.abs((pts - pts[i]) @ normal) <= EDGE_RANSAC_TOL_M
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or int(best.sum()) < EDGE_MIN_INLIERS:
        return {'ok': False, 'reason': 'NO_EDGE_LINE'}
    q = pts[best]
    mean = q.mean(0)
    _vals, vecs = np.linalg.eigh(np.cov((q - mean).T))
    u = vecs[:, -1]
    along = (q - mean) @ u
    span = float(along.max() - along.min())
    residual = float(np.std((q - mean) @ np.array([-u[1], u[0]])))
    info = {'inliers': int(best.sum()), 'points': int(len(pts)), 'span_m': round(span, 4),
            'residual_m': round(residual, 5), 'component': selection}
    if span < EDGE_MIN_SPAN_M:
        return {'ok': False, 'reason': 'EDGE_TOO_SHORT', **info}
    if residual > EDGE_MAX_RESIDUAL_M:
        return {'ok': False, 'reason': 'EDGE_NOT_STRAIGHT', **info}
    return {'ok': True, 'yaw_mod90_rad': float(math.atan2(u[1], u[0]) % _PERIOD), **info}


class EdgeYawFaceAligner:
    """Inlier vote over own-RGB top-edge yaw fits (v5's vote and face selection; no map fallback)."""

    used_fallback = False                 # read by v1's grasp event; always False here

    def __init__(self):
        self._fits: deque[float] = deque(maxlen=FACE_WINDOW)
        self._previous_normal: tuple[float, float] | None = None
        self.last_ready: dict[str, Any] | None = None
        self.observations = 0
        self.edge_accepted = 0
        self.frame: Mapping[str, Any] | None = None        # set by the box skill before each decide

    def reset_window(self) -> None:
        self._fits.clear()

    def observe(self, box: Mapping[str, Any], target_xy: Sequence[float]) -> dict[str, Any]:
        self.observations += 1
        base = {'ready': False, 'normal_xy': None, 'reason': '', 'evidence': {}}
        try:
            tx, ty = (float(v) for v in target_xy)
        except (TypeError, ValueError):
            return {**base, 'reason': 'INVALID_TARGET_XY'}
        if not (math.isfinite(tx) and math.isfinite(ty)) or math.hypot(tx, ty) <= 1e-9:
            return {**base, 'reason': 'INVALID_TARGET_XY'}
        frame = self.frame
        edge = {'ok': False, 'reason': 'NO_CURRENT_OWN_FRAME'}
        if isinstance(frame, Mapping):
            pose = (frame.get('actuator_state') or {}).get('servo_pulses')
            image = decode_jpeg_b64(frame.get('image')) if isinstance(frame.get('image'), str) else None
            if isinstance(pose, Mapping) and image is not None:
                edge = top_edge_yaw(image, pose, (tx, ty))
        ok = bool(edge.get('ok'))
        if ok:
            self.edge_accepted += 1
            self._fits.append(float(edge['yaw_mod90_rad']) % _PERIOD)
        cuboid = (box or {}).get('estimated_yaw_mod_pi_rad') if isinstance(box, Mapping) else None
        fits = list(self._fits)
        tol = math.radians(FACE_INLIER_DEG)
        best = max(fits, key=lambda f: sum(_mod90_dist(f, g) <= tol for g in fits), default=None)
        inliers = [g for g in fits if best is not None and _mod90_dist(best, g) <= tol]
        evidence = {'estimator': 'own_rgb_top_edge_yaw', 'window': [round(math.degrees(f), 2) for f in fits],
                    'inliers': len(inliers), 'min_inliers': FACE_MIN_INLIERS, 'min_fraction': FACE_MIN_INLIER_FRACTION,
                    'inlier_tol_deg': FACE_INLIER_DEG, 'frame_accepted': ok,
                    'edge': {k: (round(math.degrees(v), 2) if k == 'yaw_mod90_rad' else v) for k, v in edge.items()},
                    'n7_cuboid_yaw_mod90_deg_not_voted': (None if not isinstance(cuboid, (int, float)) or isinstance(cuboid, bool)
                                                          or not math.isfinite(float(cuboid))
                                                          else round(math.degrees(float(cuboid) % _PERIOD), 2))}
        if len(inliers) < FACE_MIN_INLIERS or len(inliers) < FACE_MIN_INLIER_FRACTION * len(fits):
            return {**base, 'reason': 'WAITING_FOR_CONSISTENT_OWN_RGB_EDGE_FITS' if ok else
                    'EDGE_YAW_EVIDENCE_NOT_ACCEPTED', 'evidence': evidence}
        s = sum(math.sin(4 * g) for g in inliers)
        c = sum(math.cos(4 * g) for g in inliers)
        yaw_est = (math.atan2(s, c) / 4) % _PERIOD
        candidates = [(math.cos(yaw_est + k * _PERIOD), math.sin(yaw_est + k * _PERIOD)) for k in range(4)]
        if self._previous_normal is None:
            norm = math.hypot(tx, ty)
            toward = (-tx / norm, -ty / norm)
            selected = max(candidates, key=lambda n: n[0] * toward[0] + n[1] * toward[1])
            selection = 'outward_face_toward_chassis'
        else:
            prev = self._previous_normal
            selected = max(candidates, key=lambda n: n[0] * prev[0] + n[1] * prev[1])
            selection = 'current_own_frame_candidate_nearest_previous_normal'
        self._previous_normal = selected
        evidence.update({'yaw_mod90_deg': round(math.degrees(yaw_est), 2), 'selection': selection})
        result = {'ready': True, 'normal_xy': [selected[0], selected[1]], 'reason': 'OWN_RGB_EDGE_YAW_INLIER_VOTE',
                  'normal_source': 'own_rgb_markerless_top_edge_vote', 'evidence': evidence}
        self.last_ready = result
        return result


class WristOnlyBoxSkillV6(v5.WristOnlyBoxSkillV5):
    """v5 box skill with the top-edge face aligner and a public carry re-anchor hook."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._face_aligner = EdgeYawFaceAligner()
        self._probe_gate_kwargs = (args, kwargs)
        self.reanchors: list[dict[str, Any]] = []

    def decide(self, observation):
        # The aligner reads the current frame only when N7 calls it, i.e. after N7 validated this observation.
        self._face_aligner.frame = observation
        try:
            return super().decide(observation)
        finally:
            self._face_aligner.frame = None

    def _fresh_gate(self) -> VisualBoxSkill:
        args, kwargs = self._probe_gate_kwargs
        return VisualBoxSkill(*args, **kwargs)

    def reanchor_after_probe(self, home_before, left, right, home_after) -> dict[str, Any]:
        """Re-anchor the held box after an own pan probe (N7 post-lift probe evidence).

        Frames are full own robot_cam observations in capture order. ``home_before``
        may be the last frame this skill already validated; the other three must be
        new. Raises ``ValueError`` for an invalid frame or probe geometry and
        ``RuntimeError`` outside the carry phase; returns the verdict otherwise.
        """
        if self.phase != 'carry' or not self.held:
            raise RuntimeError('reanchor_after_probe only while carrying a held box')
        frames = (home_before, left, right, home_after)
        gate = self._fresh_gate()                         # strict schema/hash/PWM + in-probe ordering
        poses = []
        for obs in frames:
            _obs, pose = gate._validate_observation(obs)
            poses.append(pose)
        known = {(h['frame_id'], h['sha256']) for h in self._history}
        for index, obs in enumerate(frames):
            if obs['frame_id'] <= self._last_frame_id:
                if index != 0 or (obs['frame_id'], obs['sha256']) not in known:
                    raise ValueError('probe frames must be new own frames (home_before may be the last validated one)')
        p0, pl, pr, p1 = (int(p['6']) for p in poses)
        for servo in REANCHOR_ARM_SERVOS:
            if len({p[servo] for p in poses}) != 1:
                raise ValueError(f'arm servo {servo} changed during the pan probe')
        if not (REANCHOR_MIN_PAN_DELTA_PWM <= pl - p0 <= REANCHOR_MAX_PAN_DELTA_PWM
                and REANCHOR_MIN_PAN_DELTA_PWM <= p0 - pr <= REANCHOR_MAX_PAN_DELTA_PWM
                and abs(p1 - p0) <= REANCHOR_HOME_TOL_PWM):
            raise ValueError('pan probe must go left (+40..+120 PWM), right (-40..-120 PWM) and back home')
        for obs in frames:                                # advance this skill's own monotonic state
            if obs['frame_id'] > self._last_frame_id:
                self._validate_observation(obs)
        images = [obs['image'] for obs in frames]
        checks = {
            'home_to_left': self._compare_attachment(images[0], images[1], camera_pan_delta_pwm=pl - p0),
            'home_to_right': self._compare_attachment(images[0], images[2], camera_pan_delta_pwm=pr - p0),
            'left_to_right': self._compare_attachment(images[1], images[2], camera_pan_delta_pwm=pr - pl),
            'right_to_home': self._compare_attachment(images[2], images[3], camera_pan_delta_pwm=p1 - pr)}
        attached = all(bool(c.get('attached')) for c in checks.values())
        if attached:
            self._attachment_image = images[3]
            self._carry_previous_image = images[3]
            self._attachment_pan = p1
        summary = {k: {kk: c.get(kk) for kk in ('attached', 'reason', 'mask_iou', 'centroid_delta_px', 'area_ratio')
                       if kk in c} for k, c in checks.items()}
        result = {'attached': attached, 'method': 'reanchor_after_probe_v6', 'reference': 'previous_endpoint',
                  'anchor_updated': attached, 'frame_ids': [obs['frame_id'] for obs in frames],
                  'pans_pwm': [p0, pl, pr, p1], 'checks': summary}
        self.last_attachment = {'attached': attached, 'probe': result}
        self.reanchors.append(result)
        return result


@dataclass(frozen=True)
class StaticKeepout:
    """A fixed disc from the static layout (e.g. an idle peer's parking spot). Never a live pose."""
    keepout_id: str
    xy_m: tuple[float, float]
    radius_m: float
    source: str

    def __post_init__(self):
        if self.source not in KEEPOUT_SOURCES:
            raise ValueError(f'keep-out source must be one of {KEEPOUT_SOURCES}')
        if not (len(self.xy_m) == 2 and all(math.isfinite(float(v)) for v in self.xy_m)):
            raise ValueError('keep-out xy must be two finite numbers')
        if not (0. < float(self.radius_m) <= .5):
            raise ValueError('keep-out radius must be in (0, 0.5] m')

    def record(self) -> dict[str, Any]:
        return {'id': self.keepout_id, 'xy_m': [float(v) for v in self.xy_m], 'radius_m': float(self.radius_m),
                'source': self.source}


def _segment_point_distance(a, b, p) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0. if length2 <= 0 else max(0., min(1., ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


class WristZoneDeliveryV6(v5.WristZoneDeliveryV5):
    """v5 delivery + edge-yaw face, public probe re-anchor, static keep-out approach point and guard."""

    def __init__(self, order: v5.CoarseOrderSheet, *, mode: str = 'diagnostic',
                 static_keepouts: Sequence[StaticKeepout] = (), static_bounds_m: Sequence[float] | None = None,
                 **kwargs):
        keepouts = tuple(static_keepouts)
        if not all(isinstance(k, StaticKeepout) for k in keepouts):
            raise TypeError('static_keepouts must be StaticKeepout records (static layout only)')
        self.static_keepouts = keepouts
        self.static_bounds_m = None if static_bounds_m is None else tuple(float(v) for v in static_bounds_m)
        if self.static_bounds_m is not None and len(self.static_bounds_m) != 4:
            raise ValueError('static_bounds_m is (x_min, x_max, y_min, y_max)')
        super().__init__(order, mode=mode, **kwargs)
        self.approach_choice: dict[str, Any] | None = None
        self.keepout_guard_stops = 0

    def _new_box(self):
        return WristOnlyBoxSkillV6(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)

    # ---------------- (c) approach point ----------------
    def approach_point(self) -> dict[str, Any]:
        """First static candidate that clears every keep-out disc, or ``{'blocked': True}``.

        Candidates: v5's point first (west side, 0.25 m standoff, bay centre y), then shorter
        standoffs and lateral offsets on the west side, then the same on the south / north side.
        """
        (cx, cy), (hx, hy) = self.order.pickup_bay_center_m, self.order.pickup_bay_half_m
        rejected = []
        index = 0
        for side, heading in APPROACH_SIDES:
            fx, fy = math.cos(heading), math.sin(heading)            # facing the bay
            lx, ly = -fy, fx                                         # lateral axis
            half = hx if side == 'west' else hy
            end = (cx - APPROACH_GRASP_STANDOFF_M * fx, cy - APPROACH_GRASP_STANDOFF_M * fy)
            for clearance, lateral in APPROACH_OFFSETS:
                back = half + clearance
                goal = (cx - back * fx + lateral * lx, cy - back * fy + lateral * ly)
                worst = None
                if self.static_bounds_m is not None:
                    x0, x1, y0, y1 = self.static_bounds_m
                    edge = min(goal[0] - x0, x1 - goal[0], goal[1] - y0, y1 - goal[1]) - ROBOT_RADIUS_M - KEEPOUT_MARGIN_M
                    worst = ('static_bounds', edge)
                for k in self.static_keepouts:
                    need = ROBOT_RADIUS_M + float(k.radius_m) + KEEPOUT_MARGIN_M
                    gap = min(math.hypot(goal[0] - k.xy_m[0], goal[1] - k.xy_m[1]),
                              _segment_point_distance(goal, end, k.xy_m)) - need
                    if worst is None or gap < worst[1]:
                        worst = (k.keepout_id, gap)
                if worst is None or worst[1] >= 0.:
                    return {'blocked': False, 'goal_xy_m': [round(goal[0], 4), round(goal[1], 4)],
                            'heading_rad': round(heading, 6), 'side': side, 'candidate': index,
                            'clearance_m': clearance, 'lateral_offset_m': lateral, 'is_v5_point': index == 0,
                            'rejected': rejected, 'keepouts': [k.record() for k in self.static_keepouts]}
                rejected.append({'candidate': index, 'side': side, 'nearest_keepout': worst[0],
                                 'gap_m': round(worst[1], 4)})
                index += 1
        return {'blocked': True, 'rejected': rejected, 'keepouts': [k.record() for k in self.static_keepouts]}

    def _nav_pregrasp(self, obs, est):
        if self.approach_choice is None:
            self.approach_choice = self.approach_point()
            self._event('bay_approach_point_selected', est, bay=self.order.pickup_bay_id, **self.approach_choice)
        if self.approach_choice['blocked']:
            return self._finish('BAY_APPROACH_BLOCKED')
        goal = tuple(self.approach_choice['goal_xy_m'])
        action = self._navigate(est, goal, float(self.approach_choice['heading_rad']), carrying=False)
        if action is None:
            self._event('bay_approach_point_reached', est, bay=self.order.pickup_bay_id, goal=list(goal),
                        side=self.approach_choice['side'])
            self.phase = 'grasp'
            return v5._wait(.1)
        return action

    def _guard(self, action: Mapping[str, Any], est: v1.PoseEstimate) -> str | None:
        kind = action.get('kind') if isinstance(action, Mapping) else None
        if kind == 'drive':
            fwd, left = float(action.get('fwd', 0.)), 0.
        elif kind == 'mecanum':
            fwd, left = float(action.get('forward', 0.)), float(action.get('left', 0.))
        else:
            return None
        if abs(fwd) < 1e-9 and abs(left) < 1e-9:
            return None
        c, s = math.cos(est.yaw_rad), math.sin(est.yaw_rad)
        mx, my = fwd * c - left * s, fwd * s + left * c
        for k in self.static_keepouts:
            dx, dy = k.xy_m[0] - est.x_m, k.xy_m[1] - est.y_m
            if math.hypot(dx, dy) < ROBOT_RADIUS_M + float(k.radius_m) + KEEPOUT_GUARD_MARGIN_M and mx * dx + my * dy > 0:
                return k.keepout_id
        return None

    # ---------------- public ----------------
    def decide(self, observation: Mapping[str, Any], estimate: v1.PoseEstimate) -> dict[str, Any]:
        action = super().decide(observation, estimate)
        hit = self._guard(action, estimate) if self.phase != 'finished' else None
        if hit is not None:
            self.keepout_guard_stops += 1
            self._event('static_keepout_guard', estimate, keepout=hit, blocked_action=dict(action))
            return self._finish('STATIC_KEEPOUT_GUARD')
        return action

    def reanchor_after_probe(self, home_before, left, right, home_after) -> dict[str, Any]:
        """Delivery-level wrapper: the same frames also pass this delivery's own strict gate."""
        known = {(h['frame_id'], h['sha256']) for h in self._gate._history}
        for index, obs in enumerate((home_before, left, right, home_after)):
            fid = obs.get('frame_id') if isinstance(obs, Mapping) else None
            if isinstance(fid, int) and not isinstance(fid, bool) and fid <= self._gate._last_frame_id:
                if index != 0 or (fid, obs.get('sha256')) not in known:
                    raise ValueError('probe frames must be new own frames (home_before may be the last validated one)')
        result = self.box.reanchor_after_probe(home_before, left, right, home_after)
        for obs in (home_before, left, right, home_after):
            if obs['frame_id'] > self._gate._last_frame_id:
                self._gate._validate_observation(obs)
            self.cameras_seen.add(obs['camera'])
        last = home_after
        self._validated = {'frame_id': last['frame_id'], 'sha256': last['sha256'], 'camera': last['camera'],
                           'robot_id': last['robot_id']}
        self.events.append({'event': 'reanchor_after_probe', 'phase': self.phase, **result})
        return result

    def summary(self):
        return {**super().summary(), 'profile': PROFILE, 'approach_choice': self.approach_choice,
                'keepout_guard_stops': self.keepout_guard_stops, 'reanchors': list(self.box.reanchors),
                'face_estimator': 'own_rgb_top_edge_yaw'}
