"""``wrist_zone_skill_v2``: v1 plus a lighting-robust approach and carry re-seat.

Same input boundary as v1 (one own ``robot_cam`` observation + one injected
``PoseEstimate`` per step; static map; order sheet). v1
(``harness/wrist_zone_skill.py``) is kept byte-identical as the recorded
result of experiments/2026-09-25-zone-owncam-skill (501-505).

Changes, each answering a recorded v1 failure:

1. Approach deadlock (v1 501/502/504). Under the zone ceiling light the lower
   box face renders grey, so single-frame cyan-cuboid fits jitter (lateral SD
   6-7 mm, IoU median 0.70) and N7's pan (+-20 PWM) and vertical-error gates
   alternated without ever passing together, starving forward motion.
   * Fusion: own-RGB box estimates are base-frame points, invariant to arm
     pan/tilt; they are fused as a median over the last frames since the last
     chassis motion, preferring fits with projection IoU >= 0.75.
   * One combined correction: pan and wrist tilt are corrected in a single
     pose macro, with pan hysteresis 30 PWM and near-field vertical tolerance
     25 px (N7: two separate macros, 20 PWM / 15 px).
   * Progress guarantee: after 4 correction-only frames a short forward step
     is forced when the fused bearing is small and the box is well inside the
     image.
   * No-progress timeout: no 1.5 cm radial gain for 40 approach frames ->
     request a retreat; the delivery reverses, re-looks and re-approaches
     with a fresh skill (at most 2 times).
2. Carry slip (v1 505). A slow slide in the fingers ended in N7's
   VISUAL_GRASP_DRIFT. v2 watches the own-RGB carry anchor and, at a lower
   warning level (centroid drift > 18 px or mask IoU < 0.85) or on the N7
   drift stop itself, stops and re-seats: set the box down with the N7
   release/ground-sweep check, back off by the pose estimate, re-grasp with
   the wrist approach, re-enter carry_p30 (at most 2 re-seats). The loaded
   speed cap is raised from 0.08 to 0.12 (the empty cap) to shorten carries.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from harness.visual_box_skill import SEARCH, _clip_int, _drive, _pose, _wait
from harness.approach_geometry import assess_face_standoff
from harness.visual_arm import solve_grip_ik, tool_pose
from harness import wrist_zone_skill as v1

PROFILE = 'wrist_zone_skill_v2'
FUSION_FRAMES = 5
FUSION_GOOD_IOU = .75
PAN_HYSTERESIS_PWM = 30
NEAR_VERTICAL_TOL_PX = 25
FAR_VERTICAL_TOL_PX = 35
NEAR_WRIST_STEP_PWM = 16
FORCE_FORWARD_AFTER = 4
FORCE_MAX_BEARING_RAD = .10
FORCE_MAX_VERTICAL_PX = 80
NO_PROGRESS_FRAMES = 40
FUSED_ONLY_FRAMES = 6            # transient misses bridged by the fused estimate (base not moved)
PROGRESS_M = .015
MAX_RETREATS = 2
RESEAT_CENTROID_PX = 18.   # v1 carries reached 13.6-18.7 px at their end, 505 hit N7's 25.6
RESEAT_MASK_IOU = .85
MAX_RESEATS = 2
RESEAT_BACKOFF_M = .18
RETREAT_BACKOFF_M = .15
BACKOFF_STEP_LIMIT = 25


class WristOnlyBoxSkillV2(v1.WristOnlyBoxSkill):
    """v1 box skill with a fused, progress-guaranteed approach and a re-seat entry."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._fused = deque(maxlen=FUSION_FRAMES)
        self._corrections_since_drive = 0
        self._best_radial = math.inf
        self._since_best = 0
        self.needs_retreat = False
        self.approach_stats = {'forced_forward': 0, 'combined_corrections': 0, 'drives': 0}
        self.last_fused = None
        self._fused_only = 0

    # -------- fusion helpers --------
    def _on_base_motion(self):
        self._fused.clear()
        self._fused_only = 0
        self._corrections_since_drive = 0
        self.approach_stats['drives'] += 1

    def _fuse(self, box, target):
        iou = box.get('floor_hypothesis_projection_iou')
        iou = float(iou) if isinstance(iou, (int, float)) and not isinstance(iou, bool) else 0.
        self._fused.append((float(target[0]), float(target[1]), iou))
        good = [p for p in self._fused if p[2] >= FUSION_GOOD_IOU]
        use = good or list(self._fused)
        fx = float(np.median([p[0] for p in use]))
        fy = float(np.median([p[1] for p in use]))
        self.last_fused = {'x': fx, 'y': fy, 'n': len(use), 'good': len(good)}
        return fx, fy

    def _drive_macro(self, fwd, turn, duration):
        self._on_base_motion()
        return _drive(fwd, turn, duration)

    # -------- approach (replaces N7 _approach inside this subclass only) --------
    def _approach(self, box, target, pose):
        fused_only = False
        if target is None:
            if self._fused and self._fused_only < FUSED_ONLY_FRAMES:
                # A single-frame fit failure (grey lower face) is not a lost box:
                # the base has not moved since these own-RGB estimates.
                self._fused_only += 1
                fused_only = True
                use = [p for p in self._fused if p[2] >= FUSION_GOOD_IOU] or list(self._fused)
                fx = float(np.median([p[0] for p in use]))
                fy = float(np.median([p[1] for p in use]))
            else:
                action = super()._approach(box, target, pose)      # N7 search when nothing is known
                if action.get('kind') == 'drive':
                    self._on_base_motion()
                return action
        else:
            self._fused_only = 0
            self._missing = 0
            self._last_seen_pose = dict(pose)
            fx, fy = self._fuse(box, target)
        radial = math.hypot(fx, fy)
        if radial < self._best_radial - PROGRESS_M:
            self._best_radial, self._since_best = radial, 0
        else:
            self._since_best += 1
        if self._since_best > NO_PROGRESS_FRAMES and not self._face_approach:
            self.needs_retreat = True
            return _wait(.05)
        bearing = math.atan2(fy, fx)
        desired_pan = _clip_int(round(1500 + math.degrees(bearing) * 2000 / 180), 500, 2500)
        centroid = box.get('pixel_centroid')
        if fused_only:
            vertical_error = 0.          # unknown this frame; do not chase it
        elif not isinstance(centroid, (list, tuple)) or len(centroid) != 2:
            return self._finish('INVALID_BOX_CENTROID')
        else:
            vertical_error = float(centroid[1]) - 218.7
        near = fx < .35
        pan_off = abs(int(pose['6']) - desired_pan) > PAN_HYSTERESIS_PWM
        vert_off = abs(vertical_error) > (NEAR_VERTICAL_TOL_PX if near else FAR_VERTICAL_TOL_PX)
        can_force = (abs(bearing) <= FORCE_MAX_BEARING_RAD and abs(vertical_error) <= FORCE_MAX_VERTICAL_PX
                     and radial > .20 and not self._face_approach)
        if (pan_off or vert_off) and not (can_force and self._corrections_since_drive >= FORCE_FORWARD_AFTER):
            pulses = {}
            if pan_off:
                pulses[6] = desired_pan
            if vert_off:
                limit = NEAR_WRIST_STEP_PWM if near else 65
                delta = _clip_int(round(-vertical_error * .65), -limit, limit)
                wrist = int(pose['3'])
                adjusted = _clip_int(wrist + delta, 500, 2200)
                if adjusted != wrist:
                    pulses[3] = adjusted
                else:
                    pulses[4] = _clip_int(int(pose['4']) - delta, 500, 2500)
            self._corrections_since_drive += 1
            self.approach_stats['combined_corrections'] += 1
            return _pose(pulses)
        if (pan_off or vert_off):
            self.approach_stats['forced_forward'] += 1
            return self._drive_macro(.08, 0., .3)
        if not self._face_inspection_reached:
            if abs(bearing) > .05:
                return self._drive_macro(0.0, float(np.clip(bearing * .6, -.18, .18)), .4)
            return self._drive_macro(.10, 0.0, .6)
        if not self._face_approach:
            alignment = self.last_face_alignment or {}
            if not alignment.get('ready'):
                self._face_alignment_waits += 1
                if self._face_alignment_waits > 20:
                    return self._finish('BOX_FACE_ALIGNMENT_UNOBSERVABLE')
                return _wait(.1)
            self._face_alignment_waits = 0
            normal = np.asarray([*alignment['normal_xy'], 0.0], dtype=float)
            norm = float(np.linalg.norm(normal[:2]))
            if norm <= 1e-9 or not math.isfinite(norm):
                return self._finish('INVALID_MARKER_NORMAL')
            point = np.asarray([fx, fy])
            if assess_face_standoff(point, normal[:2]).reached:
                self._face_approach = True
                return _wait(0.05)
            waypoint = point + normal[:2] / norm * 0.35
            if float(np.linalg.norm(waypoint)) < 0.055:
                self._face_approach = True
                return _wait(0.05)
            heading = math.atan2(float(waypoint[1]), float(waypoint[0]))
            if abs(heading) > 0.06:
                return self._drive_macro(0.0, float(np.clip(heading * 0.6, -0.18, 0.18)), 0.4)
            return self._drive_macro(0.12, 0.0, 0.6 if np.linalg.norm(waypoint) > 0.2 else 0.3)
        if abs(bearing) > (0.10 if fx < 0.32 else 0.05):
            return self._drive_macro(0.0, float(np.clip(bearing * 0.6, -0.18, 0.18)), 0.4)
        if radial > 0.168:
            return self._drive_macro(0.15 if fx > 0.3 else 0.08, 0.0, 1.0 if fx > 0.35 else 0.3)
        if fx < 0.145:
            return self._finish('APPROACH_OVERSHOT')
        try:
            self._inspection_pose = {int(key): value for key, value in pose.items()}
            self._grasp = solve_grip_ik(fx, fy, 0.024, -90)
            grasp_pitch = tool_pose(self._grasp).pitch_deg
            self._hover = solve_grip_ik(fx, fy, 0.095, grasp_pitch)
            down = np.linspace(0.095, 0.024, 16)[1:]
            up = np.linspace(0.024, 0.095, 16)[1:]
            self._lower_path = [solve_grip_ik(fx, fy, float(h), grasp_pitch) for h in down]
            self._release_path = [dict(item) for item in self._lower_path]
            self._lift_path = [{**solve_grip_ik(fx, fy, float(h), grasp_pitch), 1: 1500} for h in up]
        except (ValueError, RuntimeError) as exc:
            return self._finish(f'IK_UNAVAILABLE:{exc}')
        self.phase = 'lower'
        return _pose({**self._hover, 1: 2000})

    # -------- re-seat entry --------
    def begin_reseat_release(self) -> None:
        """Set the held box down with the N7 release/ground check (carry or N7 drift stop)."""
        drift_stop = self.phase == 'finished' and self.reason == 'VISUAL_GRASP_DRIFT' and self.held
        if not (drift_stop or (self.phase == 'carry' and self.held)):
            raise RuntimeError('re-seat needs a held box')
        self.phase, self.reason = 'release', 'RUNNING'


class WristZoneDeliveryV2(v1.WristZoneDelivery):
    """v1 delivery with approach retreat/re-approach and carry stop-and-reseat."""

    def __init__(self, order, **kwargs):
        super().__init__(order, **kwargs)
        self.box = self._new_box()
        self.retreats = 0
        self.reseats = 0
        self._backoff_origin = None
        self._backoff_steps = 0
        self._backoff_target = 0.
        self._after_backoff = None
        self.carry_warnings = []

    def _new_box(self):
        return WristOnlyBoxSkillV2(robot_id=self.robot_id, cargo_id='small_box_01', **v1.BOX_SKILL_OPTIONS)

    # grasp with retreat on no progress
    def _grasp(self, obs, est):
        action = super()._grasp(obs, est)
        if self.box.needs_retreat and self.phase == 'grasp':
            if self.retreats >= MAX_RETREATS:
                return self._finish('GRASP_APPROACH_NO_PROGRESS')
            self.retreats += 1
            self._event('approach_retreat', est, retreat=self.retreats, stats=dict(self.box.approach_stats))
            return self._start_backoff(est, RETREAT_BACKOFF_M, 'regrasp')
        return action

    def _nav_preplace(self, obs, est):
        check = self.box.decide(obs)
        if check['kind'] == 'finish':
            if check['reason'] == 'VISUAL_GRASP_DRIFT' and self.reseats < MAX_RESEATS:
                return self._start_reseat(est, 'n7_drift_stop')
            return self._finish('CARRY_' + check['reason'])
        anchor = (self.box.last_attachment or {}).get('carry_anchor_metrics') or {}
        delta, iou = anchor.get('centroid_delta_px'), anchor.get('mask_iou')
        if (isinstance(delta, (int, float)) and delta > RESEAT_CENTROID_PX) or \
                (isinstance(iou, (int, float)) and iou < RESEAT_MASK_IOU):
            self.carry_warnings.append({'centroid_delta_px': delta, 'mask_iou': iou})
            if self.reseats < MAX_RESEATS:
                return self._start_reseat(est, 'anchor_warning')
        goal = self._preplace_goal()
        action = self._navigate(est, goal, 0., carrying=True, tol=v1.PLACE_TOLERANCE_M,
                                yaw_tol=v1.PLACE_YAW_TOLERANCE_RAD)
        if action is None:
            self._event('preplace_reached', est, goal=goal)
            self.phase = 'pre_release'
            return _pose(self.box.lift_top_pose())
        return action

    def _start_reseat(self, est, trigger):
        self.reseats += 1
        self._event('reseat_start', est, trigger=trigger, reseat=self.reseats,
                    anchor=(self.box.last_attachment or {}).get('carry_anchor_metrics'))
        self.box.begin_reseat_release()
        self.phase = 'reseat_release'
        return _pose(self.box.lift_top_pose())

    def _reseat_release(self, obs, est):
        action = self.box.decide(obs)
        if action['kind'] == 'finish':
            if action['reason'] != 'VISUAL_RELEASE_CONFIRMED':
                return self._finish('RESEAT_RELEASE_' + action['reason'])
            self._event('reseat_set_down', est)
            return self._start_backoff(est, RESEAT_BACKOFF_M, 'regrasp')
        return action

    def _start_backoff(self, est, distance, then):
        self._backoff_origin = (est.x_m, est.y_m)
        self._backoff_steps = 0
        self._backoff_target = distance
        self._after_backoff = then
        self.phase = 'backoff'
        return _pose(SEARCH)

    def _backoff(self, obs, est):
        moved = math.hypot(est.x_m - self._backoff_origin[0], est.y_m - self._backoff_origin[1])
        if moved < self._backoff_target and self._backoff_steps < BACKOFF_STEP_LIMIT:
            self._backoff_steps += 1
            return {'kind': 'mecanum', 'forward': -.05, 'left': 0., 'turn': 0., 'duration': 1.}
        self._event('backoff_done', est, moved_m=round(moved, 3), steps=self._backoff_steps)
        self.box = self._new_box()
        self.phase = 'grasp'
        return _wait(.1)

    def _navigate(self, est, goal, heading, *, carrying, **kwargs):
        # v2: carry at the empty-travel speed cap (v1: 0.08). Grip creep grows with time under
        # load (probe: also at rest), so a shorter carry means fewer re-seats. Planner keeps
        # the loaded clearance.
        if not carrying:
            return super()._navigate(est, goal, heading, carrying=False, **kwargs)
        planner = self.planner
        self.planner = (lambda start, target, _loaded: planner(start, target, True)) if planner else None
        try:
            return super()._navigate(est, goal, heading, carrying=False, **kwargs)
        finally:
            self.planner = planner

    def summary(self):
        return {'retreats': self.retreats, 'reseats': self.reseats, 'carry_warnings': self.carry_warnings[-5:],
                'approach_stats': dict(self.box.approach_stats)}
