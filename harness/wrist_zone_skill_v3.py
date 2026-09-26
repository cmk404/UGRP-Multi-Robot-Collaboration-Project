"""``wrist_zone_skill_v3``: v2 plus an explicit grip check and a paint-robust look-back.

Same input boundary as v1/v2 (one own ``robot_cam`` observation + one injected
``PoseEstimate`` per step; static map; order sheet). v1 and v2 stay
byte-identical as the recorded results of experiments/2026-09-25-zone-owncam-skill.

Changes, each answering a recorded v2 failure (cohort 511-520, source edd075d):

1. 519: after a re-seat, N7's carry check stopped with
   ``TOP_GEOMETRY_AMBIGUOUS_FOR_DROP`` while the box was still held. In N7
   that finish is a hand-off: with ``task='external_navigation'`` the planner
   "must explicitly select check_grip" (harness/visual_box_skill.py) instead of
   N7 running an arm intervention by itself. v2 treated it as terminal. v3
   selects the check: the chassis stops and N7's own strict left/right/home
   own-camera attachment probe (``carry_probe_*``) runs with the current frame
   as its anchor. Pass -> carry resumes (N7 marks the low-height hypothesis
   validated); fail -> the run ends with N7's drop reason. At most
   ``MAX_GRIP_CHECKS`` per delivery.
2. 518: the look-back found no cyan box on zone B paint although the box sat in
   the slot. On the blue paint the zone-lit box is a dark, low-value teal that
   ``zone_color_boxes.detect_own(own_zone_v2)`` rejects, while the N7 floor
   cuboid fit (``markerless_box.observe_ground_box``, min saturation 150,
   refined position) - the same detector whose release check had just
   validated the box - still fits it. v3 confirms placement with that fit
   first and falls back to ``detect_own``; the detector used is recorded.
   Offline on the recorded v2 look-back frames: 518 validated at (0.168,
   -0.009) m, and 511/513/516 agree with detect_own within 1 cm.

The contact profile is NOT part of this module; the runner selects and
records it explicitly (see the experiment README, slip root cause).
"""
from __future__ import annotations

import math
from typing import Any

from harness.visual_box_skill import _pose, _wait
from harness import wrist_zone_skill as v1
from harness import wrist_zone_skill_v2 as v2

PROFILE = 'wrist_zone_skill_v3'
MAX_GRIP_CHECKS = 2
GRIP_CHECK_REASON = 'TOP_GEOMETRY_AMBIGUOUS_FOR_DROP'
GROUND_FIT_MIN_SATURATION = 150       # v1 BOX_SKILL_OPTIONS attachment/release setting


class WristOnlyBoxSkillV3(v2.WristOnlyBoxSkillV2):
    def begin_check_grip(self, obs) -> dict[str, Any]:
        """Explicit check_grip after N7's external-navigation hand-off (box still held)."""
        if not (self.phase == 'finished' and self.reason == GRIP_CHECK_REASON and self.held):
            raise RuntimeError('check_grip needs the N7 grip-uncertain hand-off with a held box')
        pan = int(obs['actuator_state']['servo_pulses']['6'])
        if not 560 <= pan <= 2440:
            raise RuntimeError('ATTACHMENT_PROBE_PAN_LIMIT')
        # Same fields N7 sets for its own attachment-change probe: the fresh
        # frame is the anchor, so compliance before the stop is not re-judged.
        self._attachment_image = obs['image']
        self._carry_previous_image = obs['image']
        self._attachment_pan = pan
        self._probe_results = []
        self._probe_origin_phase = 'surface_low_height'
        self.phase, self.reason = 'carry_probe_left', 'RUNNING'
        return _pose({6: pan + 60, 1: 1500})


class WristZoneDeliveryV3(v2.WristZoneDeliveryV2):
    """v2 delivery + explicit grip check on N7's hand-off + ground-fit look-back."""

    def __init__(self, order, **kwargs):
        super().__init__(order, **kwargs)
        self.grip_checks = []

    def _new_box(self):
        return WristOnlyBoxSkillV3(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)

    def _nav_preplace(self, obs, est):
        check = self.box.decide(obs)
        if check['kind'] == 'finish':
            if check['reason'] == GRIP_CHECK_REASON and self.box.held and len(self.grip_checks) < MAX_GRIP_CHECKS:
                self.grip_checks.append({'trigger': GRIP_CHECK_REASON, 'estimate': [round(est.x_m, 4), round(est.y_m, 4)],
                                         'result': None})
                self._event('grip_check_start', est, check=len(self.grip_checks))
                self.phase = 'grip_check'
                return self.box.begin_check_grip(obs)
            if check['reason'] == 'VISUAL_GRASP_DRIFT' and self.reseats < v2.MAX_RESEATS:
                return self._start_reseat(est, 'n7_drift_stop')
            return self._finish('CARRY_' + check['reason'])
        return self._nav_preplace_after_check(est)

    def _nav_preplace_after_check(self, est):
        # v2's _nav_preplace after its box.decide() call (anchor warning -> re-seat; navigate).
        anchor = (self.box.last_attachment or {}).get('carry_anchor_metrics') or {}
        delta, iou = anchor.get('centroid_delta_px'), anchor.get('mask_iou')
        if (isinstance(delta, (int, float)) and delta > v2.RESEAT_CENTROID_PX) or \
                (isinstance(iou, (int, float)) and iou < v2.RESEAT_MASK_IOU):
            self.carry_warnings.append({'centroid_delta_px': delta, 'mask_iou': iou})
            if self.reseats < v2.MAX_RESEATS:
                return self._start_reseat(est, 'anchor_warning')
        goal = self._preplace_goal()
        action = self._navigate(est, goal, 0., carrying=True, tol=v1.PLACE_TOLERANCE_M,
                                yaw_tol=v1.PLACE_YAW_TOLERANCE_RAD)
        if action is None:
            self._event('preplace_reached', est, goal=goal)
            self.phase = 'pre_release'
            return _pose(self.box.lift_top_pose())
        return action

    def _grip_check(self, obs, est):
        action = self.box.decide(obs)
        if action['kind'] == 'finish':
            self.grip_checks[-1]['result'] = action['reason']
            self._event('grip_check_failed', est, reason=action['reason'])
            return self._finish('GRIP_CHECK_' + action['reason'])
        if self.box.phase == 'carry':
            self.grip_checks[-1]['result'] = 'ATTACHED'
            self._event('grip_check_passed', est, check=len(self.grip_checks))
            self.phase = 'nav_preplace'
        return action if action['kind'] != 'wait' else _wait(.1)

    # ---------------- own-RGB placement confirmation ----------------
    def confirm_placement(self, obs, est) -> dict[str, Any]:
        from harness.markerless_box import observe_ground_box
        pose = obs['actuator_state']['servo_pulses']
        fit = observe_ground_box(obs['image'], pose, min_saturation=GROUND_FIT_MIN_SATURATION,
                                 refine_position=True)
        if str(fit.get('reason', '')).endswith('HYPOTHESIS_VALIDATED') and fit.get('estimated_box_center_base_m'):
            bx, by = (float(v) for v in fit['estimated_box_center_base_m'][:2])
            result = self._slot_verdict(bx, by, est)
            result.update({'detector': 'n7_floor_cuboid_fit', 'fit_reason': fit['reason']})
            return result
        fallback = super().confirm_placement(obs, est)
        fallback.update({'detector': 'zone_color_boxes.detect_own', 'ground_fit_reason': fit.get('reason')})
        return fallback

    def _slot_verdict(self, bx, by, est):
        c, s = math.cos(est.yaw_rad), math.sin(est.yaw_rad)
        mx, my = est.x_m + c * bx - s * by, est.y_m + s * bx + c * by
        sx, sy = self.order.slot_xy_m
        inside = abs(mx - sx) <= v1.SLOT_HALF_M and abs(my - sy) <= v1.SLOT_HALF_M
        return {'in_slot': inside, 'reason': 'IN_SLOT' if inside else 'OUTSIDE_SLOT',
                'box_base_m': [round(bx, 4), round(by, 4)], 'box_map_m': [round(mx, 4), round(my, 4)],
                'slot_error_m': [round(mx - sx, 4), round(my - sy, 4)], 'pose_source': est.source,
                'scope': 'own wrist RGB floor fit mapped with the pose estimate; the verdict inherits its source'}

    def summary(self):
        return {**super().summary(), 'grip_checks': self.grip_checks,
                'placement_detector': (self.placement or {}).get('detector')}
