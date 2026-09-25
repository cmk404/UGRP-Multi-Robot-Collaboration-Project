"""Wrist-camera-only zone box delivery skill (profile ``wrist_zone_skill_v1``).

Inputs per control step are exactly:
  * one serialized ``robot_cam`` observation from ``CameraRobotPort.capture()``
    (own wrist fisheye JPEG + own ISSUED servo PWM), and
  * one ``PoseEstimate`` from an injected pose source (map frame x, y, yaw).
Static knowledge: the authored zone map and the order sheet (box kind, the
known pickup cell, the destination slot). No ``nav_cam``, no TOP image, no
world/model/data handle, no contact or weld state enters this module.

The pose source is an interface. The localisation branch supplies the
own-RGB estimator; until then a runner may feed a clearly labelled stub
(``pose_source='gt_stub_eval_only'``) for skill-isolation tests, which are
never M1 successes. Every decision records the pose source label it used.

Grasp and release reuse the N7 ``VisualBoxSkill`` state machine unchanged
through a subclass (the N7 class and its defaults are not modified):
approach/visual servo/face alignment/IK/attachment probes/release ground
sweep all run on the own wrist RGB exactly as before. New here:
  * gross navigation to the pregrasp standoff and to the pre-place pose from
    the pose estimate (optional static-map planner injected by the runner),
  * a carry posture chosen from the 2026-09-25 wrist-view study,
  * an own-RGB look-back after release that locates the box on the floor and
    maps it into the slot frame with the pose estimate.
"""
from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from harness.markerless_face import MarkerlessFaceAligner
from harness.visual_box_skill import SEARCH, VisualBoxSkill, _clip_int, _pose, _wait
from harness.visual_arm import forward_grip

PROFILE = 'wrist_zone_skill_v1'
# Grasp settings of the dispatch solo path (coloured floors, v61 closure).
BOX_SKILL_OPTIONS = {'task': 'external_navigation', 'attachment_home_reference': 'previous_endpoint',
                     'attachment_min_saturation': 150, 'release_refine_ground_fit': True}
# Carry posture (issued PWM, grip closed): grip site 0.14 m ahead, 0.18 m high,
# tool pitch -30 deg (optical axis about -22.6 deg). See
# experiments/2026-09-25-zone-owncam-skill (carry_p30).
CARRY_POSTURE = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
PREGRASP_STANDOFF_M = .40         # base centre to box centre when the RGB approach starts
NAV_TOLERANCE_M = .025
NAV_YAW_TOLERANCE_RAD = .04
PLACE_TOLERANCE_M = .012
PLACE_YAW_TOLERANCE_RAD = .02
NAV_STEP_S = .3
LOOK_BACK_RETRY_STEP = 4          # 3 reverse steps of 1 s, SEARCH pose, then look again
SLOT_HALF_M = .06
NAV_MIN_COMMAND = .035            # smaller mecanum commands stall the chassis (dev 401)
# Face alignment: the zone ceiling light renders the lower box face grey, so a
# single view often gives a wrong cyan-silhouette yaw. Instead of N7's identical
# waits, nudge the wrist for fresh views; after this many unready attempts use
# the static map approach convention (boxes are grasped facing east) turned
# into the base frame with the pose estimate. The source is recorded.
FACE_NUDGES_PWM = (10, -10, 20, -20, 30, -30, 15, -15)
FACE_FALLBACK_AFTER = len(FACE_NUDGES_PWM)


@dataclass(frozen=True)
class PoseEstimate:
    """Map-frame base pose from an injected source; ``source`` is mandatory."""
    x_m: float
    y_m: float
    yaw_rad: float
    source: str

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError('pose estimate must be finite')
        if not isinstance(self.source, str) or not self.source:
            raise ValueError('pose estimate needs a source label')


@dataclass(frozen=True)
class OrderSheet:
    """What the robot is told: kind, a known pickup cell and a slot (static)."""
    box_kind: str
    pickup_xy_m: tuple[float, float]
    slot_id: str
    slot_xy_m: tuple[float, float]

    def __post_init__(self):
        if self.box_kind != 'cyan':
            raise ValueError('wrist_zone_skill_v1 grasps the cyan production box only')


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class FaceAlignerWithMapFallback:
    """MarkerlessFaceAligner first; a labelled static-map normal only after it stays unready."""

    def __init__(self):
        self.inner = MarkerlessFaceAligner()
        self.unready = 0
        self.map_normal_base = None
        self.used_fallback = False

    def observe(self, box, target_xy):
        result = self.inner.observe(box, target_xy)
        if result.get('ready'):
            result['normal_source'] = 'own_rgb_markerless_face_fits'
            return result
        self.unready += 1
        if self.unready > FACE_FALLBACK_AFTER and self.map_normal_base is not None:
            self.used_fallback = True
            return {'ready': True, 'normal_xy': list(self.map_normal_base),
                    'reason': 'STATIC_MAP_APPROACH_CONVENTION_FALLBACK',
                    'normal_source': 'static_map_approach_convention+pose_estimate',
                    'evidence': {'rgb_reason': result.get('reason'), 'rgb_unready_attempts': self.unready}}
        return result


class WristOnlyBoxSkill(VisualBoxSkill):
    """N7 box skill with explicit external transitions; N7 class and defaults unchanged."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._face_aligner = FaceAlignerWithMapFallback()
        self._nudge_origin = None
        self.face_nudges = 0

    def set_map_face_normal(self, normal_base):
        self._face_aligner.map_normal_base = tuple(float(v) for v in normal_base)

    def decide(self, observation):
        action = super().decide(observation)
        waiting_for_face = (self.phase == 'approach' and action.get('kind') == 'wait'
                            and self._face_inspection_reached and not self._face_approach
                            and self._face_alignment_waits > 0)
        if not waiting_for_face:
            if self._face_approach:
                self._nudge_origin = None
            return action
        pose = self._last_seen_pose
        if self._nudge_origin is None:
            self._nudge_origin = int(pose['3'])
        if self.face_nudges < len(FACE_NUDGES_PWM):
            target = _clip_int(self._nudge_origin + FACE_NUDGES_PWM[self.face_nudges], 500, 2200)
            self.face_nudges += 1
            return _pose({3: target})
        return action

    def reanchor_after_posture(self, observation) -> dict[str, Any]:
        """After an issued carry-posture change, re-anchor the held-box image.

        The camera rides on the gripper, so a held box keeps its pixels while
        the background changes. Require co-motion evidence against the old
        anchor before accepting the new frame as the carry anchor.
        """
        obs, _pose_pwm = self._validate_observation(observation)
        if self.phase != 'carry' or not self.held:
            raise RuntimeError('reanchor only while carrying')
        check = self._compare_attachment(self._attachment_image, obs['image'])
        if check.get('attached'):
            self._attachment_image = obs['image']
            self._carry_previous_image = obs['image']
        self.last_attachment = check
        return check

    def begin_release(self) -> None:
        if self.phase != 'carry' or not self.held:
            raise RuntimeError('release only while carrying')
        self.phase = 'release'

    def grasp_offset_base_m(self) -> tuple[float, float]:
        """Where the box sits relative to the base when released (own grasp plan)."""
        x, y, _ = forward_grip(self._required_plan(self._grasp, 'grasp'))
        return float(x), float(y)

    def lift_top_pose(self) -> dict[int, int]:
        """The last issued lift pose (hover height, grip closed)."""
        plan = self._grasp and dict(self._hover)
        if plan is None:
            raise RuntimeError('missing hover plan')
        return {**plan, 1: 1500}


class WristZoneDelivery:
    """Order-sheet delivery of one cyan box: navigate, grasp, carry, place, look back."""

    def __init__(self, order: OrderSheet, *, robot_id: str = 'r1',
                 planner: Callable[[tuple[float, float], tuple[float, float], bool], Sequence[Sequence[float]] | None] | None = None,
                 carry_posture: Mapping[int, int] | None = None):
        self.order = order
        self.robot_id = robot_id
        self.planner = planner
        self.carry_posture = dict(carry_posture or CARRY_POSTURE)
        self.box = WristOnlyBoxSkill(robot_id=robot_id, cargo_id='small_box_01', **BOX_SKILL_OPTIONS)
        self.phase = 'nav_pregrasp'
        self.reason = 'RUNNING'
        self.path: list[tuple[float, float]] | None = None
        self.path_goal = None
        self.look_back_steps = 0
        self.placement: dict[str, Any] | None = None
        self.pose_sources: set[str] = set()
        self.events: list[dict[str, Any]] = []
        self.last_nav = None

    # ---------------- public ----------------
    def decide(self, observation: Mapping[str, Any], estimate: PoseEstimate) -> dict[str, Any]:
        if not isinstance(estimate, PoseEstimate):
            raise ValueError('estimate must be a PoseEstimate')
        self.pose_sources.add(estimate.source)
        handler = getattr(self, '_' + self.phase)
        return handler(observation, estimate)

    @property
    def finished(self) -> bool:
        return self.phase == 'finished'

    # ---------------- phases ----------------
    def _nav_pregrasp(self, obs, est):
        bx, by = self.order.pickup_xy_m
        goal = (bx - PREGRASP_STANDOFF_M, by)
        action = self._navigate(est, goal, 0., carrying=False)
        if action is None:
            self._event('pregrasp_reached', est)
            self.phase = 'grasp'
            return _wait(.1)
        return action

    def _grasp(self, obs, est):
        # Static map approach convention: the grasp face's outward normal points west (-x map).
        self.box.set_map_face_normal((-math.cos(est.yaw_rad), math.sin(est.yaw_rad)))
        action = self.box.decide(obs)
        if action['kind'] == 'finish':
            return self._finish('GRASP_' + action['reason'])
        if self.box.phase == 'carry' and self.box.held:
            self._event('grasp_attached', est, face_normal_source=(
                'static_map_approach_convention+pose_estimate' if self.box._face_aligner.used_fallback
                else 'own_rgb_markerless_face_fits'), face_nudges=self.box.face_nudges)
            self.phase = 'to_carry_posture'
            return _pose(self.carry_posture)
        return action

    def _to_carry_posture(self, obs, est):
        check = self.box.reanchor_after_posture(obs)
        if not check.get('attached'):
            return self._finish('CARRY_POSTURE_ATTACHMENT_UNCONFIRMED')
        self._event('carry_posture_anchored', est)
        self.phase = 'nav_preplace'
        return _wait(.05)

    def _preplace_goal(self):
        gx, gy = self.box.grasp_offset_base_m()
        sx, sy = self.order.slot_xy_m
        return (sx - gx, sy - gy)

    def _nav_preplace(self, obs, est):
        check = self.box.decide(obs)            # own-RGB carry attachment check
        if check['kind'] == 'finish':
            return self._finish('CARRY_' + check['reason'])
        goal = self._preplace_goal()
        action = self._navigate(est, goal, 0., carrying=True, tol=PLACE_TOLERANCE_M,
                                yaw_tol=PLACE_YAW_TOLERANCE_RAD)
        if action is None:
            self._event('preplace_reached', est, goal=goal)
            self.phase = 'pre_release'
            return _pose(self.box.lift_top_pose())
        return action

    def _pre_release(self, obs, est):
        self.box.begin_release()
        self.phase = 'release'
        return self._release(obs, est)

    def _release(self, obs, est):
        action = self.box.decide(obs)
        if action['kind'] == 'finish':
            if action['reason'] != 'VISUAL_RELEASE_CONFIRMED':
                return self._finish('RELEASE_' + action['reason'])
            self._event('release_confirmed', est)
            self.phase = 'look_back'
            return _wait(.1)            # arm already at the own inspection pose, gripper open
        return action

    def _look_back(self, obs, est):
        """Look at the released box again from a fresh frame; one reverse-and-retry."""
        if self.look_back_steps in (0, LOOK_BACK_RETRY_STEP):
            placement = self.confirm_placement(obs, est)
            if placement['reason'] in ('IN_SLOT', 'OUTSIDE_SLOT') or self.look_back_steps:
                self.placement = placement
                self._event('look_back', est, placement=placement)
                return self._finish('OWN_RGB_PLACEMENT_' + placement['reason'])
            self._event('look_back_retry', est, placement=placement)
        self.look_back_steps += 1
        if self.look_back_steps < LOOK_BACK_RETRY_STEP:
            return {'kind': 'mecanum', 'forward': -.05, 'left': 0., 'turn': 0., 'duration': 1.}
        return _pose(SEARCH)

    # ---------------- own-RGB placement confirmation ----------------
    def confirm_placement(self, obs, est) -> dict[str, Any]:
        from harness.zone_color_boxes import OWN_PROFILE_ZONE, detect_own
        pose = obs['actuator_state']['servo_pulses']
        result = detect_own(obs['image'], pose, ('cyan',), profile=OWN_PROFILE_ZONE)
        near = [d for d in result['detections'] if d['range_class'] == 'near']
        if len(near) != 1:
            return {'in_slot': False, 'reason': 'NO_UNIQUE_CYAN_BOX' if not near else 'MULTIPLE_CYAN_BOXES',
                    'detections': len(result['detections']), 'pose_source': est.source}
        bx, by = near[0]['estimated_box_center_base_m'][:2]
        c, s = math.cos(est.yaw_rad), math.sin(est.yaw_rad)
        mx, my = est.x_m + c * bx - s * by, est.y_m + s * bx + c * by
        sx, sy = self.order.slot_xy_m
        inside = abs(mx - sx) <= SLOT_HALF_M and abs(my - sy) <= SLOT_HALF_M
        return {'in_slot': inside, 'reason': 'IN_SLOT' if inside else 'OUTSIDE_SLOT',
                'box_base_m': [round(bx, 4), round(by, 4)], 'box_map_m': [round(mx, 4), round(my, 4)],
                'slot_error_m': [round(mx - sx, 4), round(my - sy, 4)],
                'projection_iou': round(near[0]['floor_hypothesis_projection_iou'], 3),
                'pose_source': est.source,
                'scope': 'own wrist RGB floor fit mapped with the pose estimate; the verdict inherits its source'}

    # ---------------- navigation from the pose estimate ----------------
    def _navigate(self, est, goal, heading, *, carrying, tol=NAV_TOLERANCE_M, yaw_tol=NAV_YAW_TOLERANCE_RAD):
        dx, dy = goal[0] - est.x_m, goal[1] - est.y_m
        dist = math.hypot(dx, dy)
        eyaw = _wrap(heading - est.yaw_rad)
        if dist <= tol and abs(eyaw) <= yaw_tol:
            self.path = None
            return None
        target = goal
        if self.planner is not None and dist > .30:
            if self.path is None or self.path_goal != tuple(goal):
                path = self.planner((est.x_m, est.y_m), tuple(goal), carrying)
                if path is None:
                    return self._finish('NO_STATIC_MAP_PATH')
                self.path, self.path_goal = [tuple(p) for p in path], tuple(goal)
            while len(self.path) > 1 and math.hypot(self.path[0][0] - est.x_m, self.path[0][1] - est.y_m) < .12:
                self.path.pop(0)
            target = self.path[0] if self.path else goal
        wx, wy = target[0] - est.x_m, target[1] - est.y_m
        norm = max(math.hypot(wx, wy), 1e-9)
        # Far away: face the travel direction; close: final heading, mecanum sideways.
        want = heading if dist < .35 else math.atan2(wy, wx)
        err = _wrap(want - est.yaw_rad)
        fwd_w = math.cos(est.yaw_rad) * wx + math.sin(est.yaw_rad) * wy
        left_w = -math.sin(est.yaw_rad) * wx + math.cos(est.yaw_rad) * wy
        gain = 1.0 if dist < .35 else .15 / norm
        scale = 1. if abs(err) < .5 else .25
        vmax = .08 if carrying else .12
        self.last_nav = {'goal': [round(goal[0], 4), round(goal[1], 4)], 'target': [round(target[0], 4), round(target[1], 4)],
                         'dist_m': round(dist, 4), 'yaw_err_rad': round(eyaw, 4)}
        duration = .5 if dist > .35 else NAV_STEP_S if dist > .05 else .2
        fwd = max(-.05, min(vmax, gain * fwd_w * scale))
        left = max(-.08, min(.08, gain * left_w * scale))
        if abs(fwd_w) > tol / 2 and abs(fwd) < NAV_MIN_COMMAND:
            fwd = math.copysign(NAV_MIN_COMMAND, fwd_w) if fwd_w > 0 else -min(.05, NAV_MIN_COMMAND)
        if abs(left_w) > tol / 2 and abs(left) < NAV_MIN_COMMAND:
            left = math.copysign(NAV_MIN_COMMAND, left_w)
        turn = max(-.15, min(.15, .8 * err))
        if abs(err) > yaw_tol and abs(turn) < .05:
            turn = math.copysign(.05, err)
        return {'kind': 'mecanum', 'forward': float(fwd), 'left': float(left), 'turn': float(turn),
                'duration': duration}

    # ---------------- bookkeeping ----------------
    def _event(self, kind, est, **detail):
        self.events.append({'event': kind, 'phase': self.phase, 'pose_source': est.source,
                            'estimate': [round(est.x_m, 4), round(est.y_m, 4), round(est.yaw_rad, 4)], **detail})

    def _finish(self, reason):
        self.phase = 'finished'
        self.reason = str(reason)
        return {'kind': 'finish', 'reason': self.reason}
