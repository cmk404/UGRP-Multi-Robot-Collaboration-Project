"""``wrist_zone_skill_v5``: v4 plus the Codex own-camera review fixes (issues 1, 2, 3).

v1-v4 stay byte-identical as recorded results of
experiments/2026-09-25-zone-owncam-skill. v5 changes behaviour, so it is a new
profile:

1. M1 contract (review issue 1, BLOCKER). ``mode`` is ``'diagnostic'`` or
   ``'m1'``. In ``'m1'`` the executor (``decide``) and the judge
   (``confirm_placement``) REJECT every pose source outside the M1 allow-list
   (``harness.m1_contract``), including ``gt_stub_eval_only``, by raising
   ``ContractViolation``. Diagnostic runs may use the GT stub; their outcome is
   ``diagnostic_success`` only (see ``m1_contract.outcome_fields``).
2. Strict observations at every stage (review issue 2, HIGH). Every
   observation, in every phase including navigation and ``look_back``, first
   passes N7's ``VisualBoxSkill._validate_observation`` on a dedicated gate
   instance (schema, robot id, camera == ``robot_cam``, monotonic frame id and
   sim time, SHA-256 of the JPEG, own servo PWM). A rejected observation ends
   the run with ``OBSERVATION_REJECTED:<why>``, before any verdict.
   ``confirm_placement`` additionally re-checks that its frame is the one the
   gate validated last.
3. Coarse order sheet, orientation from own RGB (review issue 3, HIGH). The
   order names a pickup bay (centre and half extents from the order-sheet bay
   table, 0.5 m square), never the box position. The robot drives to the
   bay's static west approach point and finds the box with the wrist camera
   (N7 search/approach). The face normal comes ONLY from own-RGB cuboid yaw
   fits (``RobustFaceAligner``); there is no static-map / east-facing
   fallback. If the face stays unobservable, the delivery backs off and
   re-looks (at most ``MAX_FACE_RELOOKS``), then aborts.

   ``RobustFaceAligner`` replaces N7's "three consecutive fits within 10 deg,
   cleared by any weak frame" rule, which in the v4 cohort was ready in only
   1/10 grasps (531-540 used the map fallback 9/10 times: occasional 16-22 deg
   outlier fits at IoU 0.73-0.85 broke the consecutive run, and v1 counted
   unready frames cumulatively). v5 keeps the last 6 accepted fits (IoU >=
   0.70) since the last chassis motion and is ready when >= 3 fits and >= 60 %
   of the window lie within 7 deg (mod 90) of one fit; the estimate is their
   circular mean. Face selection is N7's (outward face toward the chassis, then
   nearest to the previous normal).

   Face squaring (dev 409, box yaw 20 deg): N7's face approach drives to a
   waypoint 0.35 m out along the normal with differential turn-drive-turn
   moves; for a 20-30 deg face that is ~0.2 m of sideways travel and v2's
   no-progress retreat fired twice before it arrived. v5 uses the mecanum
   chassis instead: once the own-RGB face estimate is ready it rotates in
   place until the box yaw (mod 90, base frame) is within 4 deg, re-estimating
   the face after every rotation (the fit window is cleared on chassis motion),
   then strafes until the box is centred (|y| <= 1 cm; a strafe keeps the yaw
   window since translation does not change the base-frame yaw) and hands over
   to N7's straight final approach. No convergence within
   ``FACE_SQUARE_MAX_TURNS`` turns / ``FACE_SQUARE_MAX_STRAFES`` strafes is
   treated like an unobservable face (re-look, then abort).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from harness import m1_contract
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v2 as v2
from harness import wrist_zone_skill_v4 as v4
from harness.visual_box_skill import VisualBoxSkill, _clip_int, _pose, _wait

PROFILE = 'wrist_zone_skill_v5'
BAY_APPROACH_CLEARANCE_M = .25     # base centre stops this far west of the bay's west edge
MAX_FACE_RELOOKS = 2
FACE_RELOOK_BACKOFF_M = .12
FACE_WINDOW = 6
FACE_MIN_INLIERS = 3
FACE_MIN_INLIER_FRACTION = .6
FACE_INLIER_DEG = 7.
FACE_MIN_IOU = .70
FACE_SQUARE_TOL_DEG = 4.
FACE_CENTRE_TOL_M = .010
FACE_SQUARE_MAX_TURNS = 12
FACE_SQUARE_MAX_STRAFES = 40
FACE_UNOBSERVABLE_REASONS = ('GRASP_BOX_FACE_ALIGNMENT_UNOBSERVABLE', 'GRASP_BOX_FACE_SQUARING_NO_CONVERGENCE')
_PERIOD = math.pi / 2


def _mod90_dist(a: float, b: float) -> float:
    return abs((a - b + _PERIOD / 2) % _PERIOD - _PERIOD / 2)


@dataclass(frozen=True)
class CoarseOrderSheet:
    """What the robot is told: kind, a coarse pickup BAY (not the box pose) and a slot."""
    kind: str
    pickup_bay_id: str
    pickup_bay_center_m: tuple[float, float]
    pickup_bay_half_m: tuple[float, float]
    slot_id: str
    slot_xy_m: tuple[float, float]

    def __post_init__(self):
        for name in ('pickup_bay_center_m', 'pickup_bay_half_m', 'slot_xy_m'):
            value = getattr(self, name)
            if len(value) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in value):
                raise ValueError(f'{name} must be two finite numbers')
        if min(self.pickup_bay_half_m) < .15:
            raise ValueError('a pickup bay is coarse (half extent >= 0.15 m), not a box position')
        if self.kind != 'cyan':
            raise ValueError('this skill delivers cyan boxes')

    def record(self) -> dict[str, Any]:
        return {'kind': self.kind, 'pickup_bay_id': self.pickup_bay_id,
                'pickup_bay_center_m': list(self.pickup_bay_center_m),
                'pickup_bay_half_m': list(self.pickup_bay_half_m), 'slot_id': self.slot_id,
                'slot_xy_m': list(self.slot_xy_m), 'contract': 'coarse_bay; box position and yaw from own RGB'}


class RobustFaceAligner:
    """Inlier vote over own-RGB cuboid yaw fits; no map fallback exists."""

    used_fallback = False                 # read by v1's grasp event; always False here

    def __init__(self):
        self._fits: deque[tuple[float, float]] = deque(maxlen=FACE_WINDOW)
        self._previous_normal: tuple[float, float] | None = None
        self.last_ready: dict[str, Any] | None = None
        self.observations = 0

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
        yaw, iou = (box or {}).get('estimated_yaw_mod_pi_rad'), (box or {}).get('floor_hypothesis_projection_iou')
        ok = (isinstance(box, Mapping) and box.get('visible') is True
              and isinstance(yaw, (int, float)) and not isinstance(yaw, bool) and math.isfinite(float(yaw))
              and isinstance(iou, (int, float)) and not isinstance(iou, bool) and float(iou) >= FACE_MIN_IOU)
        if ok:
            self._fits.append((float(yaw) % _PERIOD, float(iou)))
        fits = [f for f, _ in self._fits]
        tol = math.radians(FACE_INLIER_DEG)
        best = max(fits, key=lambda f: sum(_mod90_dist(f, g) <= tol for g in fits), default=None)
        inliers = [g for g in fits if best is not None and _mod90_dist(best, g) <= tol]
        evidence = {'window': [round(math.degrees(f), 1) for f in fits], 'inliers': len(inliers),
                    'min_inliers': FACE_MIN_INLIERS, 'min_fraction': FACE_MIN_INLIER_FRACTION,
                    'inlier_tol_deg': FACE_INLIER_DEG, 'frame_accepted': ok}
        if len(inliers) < FACE_MIN_INLIERS or len(inliers) < FACE_MIN_INLIER_FRACTION * len(fits):
            return {**base, 'reason': 'WAITING_FOR_CONSISTENT_OWN_RGB_YAW_FITS' if ok else
                    'BOX_YAW_EVIDENCE_NOT_ACCEPTED', 'evidence': evidence}
        # circular mean with period 90 deg around the voted fit
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
        result = {'ready': True, 'normal_xy': [selected[0], selected[1]], 'reason': 'OWN_RGB_FACE_INLIER_VOTE',
                  'normal_source': 'own_rgb_markerless_face_inlier_vote', 'evidence': evidence}
        self.last_ready = result
        return result


class WristOnlyBoxSkillV5(v4.WristOnlyBoxSkillV4):
    """v4 box skill whose face normal comes only from own-RGB fits (no map convention)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._face_aligner = RobustFaceAligner()
        self.map_normal_offers_ignored = 0
        self.face_square_stats = {'moves': 0, 'turns': 0, 'strafes': 0, 'last': None, 'squared': None}

    def set_map_face_normal(self, normal_base):
        # v1's delivery offers the static-map approach convention every grasp step; v5 never uses it.
        self.map_normal_offers_ignored += 1

    def _on_base_motion(self):
        super()._on_base_motion()
        self._face_aligner.reset_window()      # base-frame yaw fits are only comparable without chassis motion

    # -------- face squaring with the mecanum chassis (replaces N7's waypoint face approach) --------
    def _approach(self, box, target, pose):
        alignment = self.last_face_alignment or {}
        if (target is not None and self._face_inspection_reached and not self._face_approach
                and alignment.get('ready') and alignment.get('normal_source', '').startswith('own_rgb')):
            return self._square_to_face(alignment, target, pose)
        return super()._approach(box, target, pose)

    def _square_to_face(self, alignment, target, pose):
        self._missing = 0
        self._last_seen_pose = dict(pose)
        fx, fy = float(target[0]), float(target[1])
        bearing = math.atan2(fy, fx)
        desired_pan = _clip_int(round(1500 + math.degrees(bearing) * 2000 / 180), 500, 2500)
        if abs(int(pose['6']) - desired_pan) > v2.PAN_HYSTERESIS_PWM:
            return _pose({6: desired_pan})              # arm only: the fit window stays valid
        yaw = float(alignment['evidence']['yaw_mod90_deg'])
        err = (yaw + 45.) % 90. - 45.                   # signed box yaw relative to the chassis
        stats = self.face_square_stats
        # a ready own-RGB face estimate is progress: v2's no-progress and N7's face-wait counters restart
        self._since_best = 0
        self._face_alignment_waits = 0
        if stats['turns'] >= FACE_SQUARE_MAX_TURNS or stats['strafes'] >= FACE_SQUARE_MAX_STRAFES:
            return self._finish('BOX_FACE_SQUARING_NO_CONVERGENCE')
        if abs(err) > FACE_SQUARE_TOL_DEG:
            turn = max(-.15, min(.15, .8 * math.radians(err)))
            turn = math.copysign(max(abs(turn), .05), turn)
            stats['moves'] += 1
            stats['turns'] += 1
            stats['last'] = {'yaw_err_deg': round(err, 2), 'y_m': round(fy, 4)}
            self._on_base_motion()
            return {'kind': 'mecanum', 'forward': 0., 'left': 0., 'turn': float(turn), 'duration': .5}
        if abs(fy) > FACE_CENTRE_TOL_M:
            # dev 409-b: 0.06 x 0.3 s strafes moved ~3 mm each. A pure strafe does not change the
            # base-frame yaw, so the own-RGB yaw window is kept (only the position fusion restarts).
            left = max(-.08, min(.08, 1.5 * fy))
            left = math.copysign(max(abs(left), v1.NAV_MIN_COMMAND), left)
            stats['moves'] += 1
            stats['strafes'] += 1
            stats['last'] = {'yaw_err_deg': round(err, 2), 'y_m': round(fy, 4)}
            v2.WristOnlyBoxSkillV2._on_base_motion(self)
            return {'kind': 'mecanum', 'forward': 0., 'left': float(left), 'turn': 0.,
                    'duration': 1. if abs(fy) > .03 else .4}
        stats['squared'] = {'yaw_err_deg': round(err, 2), 'y_m': round(fy, 4), 'x_m': round(fx, 4)}
        self._face_approach = True
        return _wait(.05)


class WristZoneDeliveryV5(v4.WristZoneDeliveryV4):
    """v4 delivery + M1 contract + strict validation at every stage + coarse-bay own-RGB approach."""

    def __init__(self, order: CoarseOrderSheet, *, mode: str = 'diagnostic', **kwargs):
        if not isinstance(order, CoarseOrderSheet):
            raise TypeError('v5 takes a CoarseOrderSheet (pickup bay), never an exact pickup position')
        self.mode = m1_contract.check_mode(mode)
        super().__init__(order, **kwargs)
        self._gate = VisualBoxSkill(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)
        self._validated: dict[str, Any] | None = None
        self.rejected_observations: list[dict[str, Any]] = []
        self.cameras_seen: set[str] = set()
        self.face_relooks = 0
        self.face_evidence: list[dict[str, Any]] = []

    def _new_box(self):
        return WristOnlyBoxSkillV5(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)

    # ---------------- public ----------------
    def decide(self, observation: Mapping[str, Any], estimate: v1.PoseEstimate) -> dict[str, Any]:
        if not isinstance(estimate, v1.PoseEstimate):
            raise ValueError('estimate must be a PoseEstimate')
        if self.mode == 'm1':
            m1_contract.require_m1_pose_source(estimate.source, f'executor:{self.phase}')
        if self.phase == 'finished':
            return {'kind': 'finish', 'reason': self.reason}
        self._gate.phase = self.phase
        try:
            obs, _pose = self._gate._validate_observation(observation)
        except ValueError as exc:
            camera = observation.get('camera') if isinstance(observation, Mapping) else None
            self.rejected_observations.append({'phase': self.phase, 'reason': str(exc), 'camera': camera})
            return self._finish(f'OBSERVATION_REJECTED:{exc}')
        self._validated = {'frame_id': obs['frame_id'], 'sha256': obs['sha256'], 'camera': obs['camera'],
                           'robot_id': obs['robot_id']}
        self.cameras_seen.add(obs['camera'])
        return super().decide(observation, estimate)

    # ---------------- coarse bay approach ----------------
    def _nav_pregrasp(self, obs, est):
        (cx, cy), (hx, _hy) = self.order.pickup_bay_center_m, self.order.pickup_bay_half_m
        goal = (cx - hx - BAY_APPROACH_CLEARANCE_M, cy)
        action = self._navigate(est, goal, 0., carrying=False)
        if action is None:
            self._event('bay_approach_point_reached', est, bay=self.order.pickup_bay_id, goal=list(goal))
            self.phase = 'grasp'
            return _wait(.1)
        return action

    def _grasp(self, obs, est):
        before = len(self.events)
        action = super()._grasp(obs, est)
        for event in self.events[before:]:
            if event['event'] == 'grasp_attached':
                event['face_evidence'] = self.box._face_aligner.last_ready
                event['map_normal_offers_ignored'] = self.box.map_normal_offers_ignored
                event['face_square'] = dict(self.box.face_square_stats)
                self.face_evidence.append({'relooks': self.face_relooks, 'evidence': self.box._face_aligner.last_ready,
                                           'square': dict(self.box.face_square_stats)})
        if self.phase == 'finished' and self.reason in FACE_UNOBSERVABLE_REASONS:
            if self.face_relooks >= MAX_FACE_RELOOKS:
                self.reason = 'GRASP_FACE_UNOBSERVABLE_AFTER_RELOOKS'
                return {'kind': 'finish', 'reason': self.reason}
            self.face_relooks += 1
            trigger = self.reason
            self.phase, self.reason = 'grasp', 'RUNNING'
            self._event('face_relook', est, relook=self.face_relooks, trigger=trigger,
                        square=dict(self.box.face_square_stats))
            return self._start_backoff(est, FACE_RELOOK_BACKOFF_M, 'regrasp')
        return action

    # ---------------- judge ----------------
    def confirm_placement(self, obs, est) -> dict[str, Any]:
        if self.mode == 'm1':
            m1_contract.require_m1_pose_source(est.source, 'judge:confirm_placement')
        check = self._validated or {}
        import base64
        import hashlib
        try:
            digest = hashlib.sha256(base64.b64decode(obs['image'], validate=True)).hexdigest()
        except Exception as exc:                       # noqa: BLE001 - any decode failure is a rejection
            raise ValueError('placement frame is not a valid JPEG payload') from exc
        if not (obs.get('camera') == 'robot_cam' == check.get('camera') and obs.get('robot_id') == self.robot_id
                and obs.get('frame_id') == check.get('frame_id') and obs.get('sha256') == check.get('sha256') == digest):
            raise ValueError('placement frame is not the last strictly validated own robot_cam frame')
        result = super().confirm_placement(obs, est)
        result.update({'observation_validated': True, 'frame_id': obs['frame_id'], 'frame_sha256': digest,
                       'mode': self.mode, 'counts_as_m1_input': m1_contract.is_m1_pose_source(est.source)})
        return result

    def summary(self):
        return {**super().summary(), 'mode': self.mode, 'face_relooks': self.face_relooks,
                'face_evidence': self.face_evidence, 'rejected_observations': self.rejected_observations,
                'cameras_seen': sorted(self.cameras_seen), 'order_contract': 'coarse_bay'}
