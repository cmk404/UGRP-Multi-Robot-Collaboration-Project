"""M1 own-camera delivery controller (sim-free): search -> grasp -> carry through the door -> place -> look back.

Inputs (and nothing else): the robot's own ``robot_cam`` observations (JPEG +
own issued PWM), its own issued commands, the static tagged map, fixed
calibrations, the static pickup-grid rows (search viewpoints only) and the order sheet: box colour, "pickup area
(west)" and the destination slot. The exact cargo position is found with own
RGB (``detect_own``) and the own pose estimate (Codex review #3).

Composition (read-only reuse):
* pose: ``harness.owncam_pose_source.OwnCamPoseSource`` (one localizer for the
  whole episode) with ``PoseReport`` limits checked before acting (review #4);
* unloaded legs (search viewpoints, pregrasp standoff) and the loaded carry leg
  up to the door exit: ``OwnCamDriverV2`` pursuit + stop-and-look policy
  sharing that localizer;
* grasp, carry checks, re-seat, release and look-back: the wrist skill
  (``harness.wrist_zone_skill_v4`` imported read-only); in the carry leg its
  navigation commands are replaced by the driver's while its own-RGB carry
  checks still run on every frame;
* M1 contract (``harness.m1_contract``): every observation is validated and
  every pose source must be the own-camera estimator.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from harness import m1_contract
from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SEARCH_POSE, SETTLE_S, WIDE_LOOK_PANS
from harness.owncam_drive_v2 import OwnCamDriverV2
from harness.owncam_pose_source import OwnCamPoseSource, PoseLimits, PoseReport, check_limits

SCHEMA = 'ugrp.m1_owncam_delivery.v1'
# Search: static viewpoints west of the pickup grid (east-facing), visited from the
# row nearest the robot's first own estimate outward. A far_coarse cyan detection brings the robot to a closer view.
SEARCH_VIEW_X_M = -.47                 # peers idle at the spawn column x = -0.85
SEARCH_PANS = (1500, 1230, 970, 1770, 2030, 1500)
CLOSER_VIEW_STANDOFF_M = .80
NEAR_MIN_DETECTIONS = 2
PREGRASP_STANDOFF_M = .40              # = wrist_zone_skill_v1.PREGRASP_STANDOFF_M
SEEN_BOX_HALF_M = .03                  # keep-out half size around boxes seen with own RGB (box half 0.02 m)
TARGET_MATCH_M = .15
DOOR_EXIT_M = .45
# Pose limits (pre-registered in experiments/2026-09-26-zone-m1-owncam/prereg.json).
LIMITS = {
    'nav_unloaded': PoseLimits(.08, .10, name='nav_unloaded'),
    'nav_loaded': PoseLimits(.07, math.radians(3.), name='nav_loaded'),
    'release': PoseLimits(.05, .035, max_since_look_s=15., name='release'),
    'look_back': PoseLimits(.035, .035, max_since_look_s=3., name='look_back'),
}
PREPLACE_LOOK_RADIUS_M = .35
MAX_GATE_LOOKS = 3
ARM_STEP_PWM = 60


class _LegDriver(OwnCamDriverV2):
    """OwnCamDriverV2 on a shared localizer: commands/frames reach the localizer once (via the pose source)."""

    def __init__(self, shared_loc, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.loc = shared_loc
        self.last_estimate = self.loc.estimate()

    def on_command(self, row):                      # servo bookkeeping only
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])


class M1OwnCamDelivery:
    def __init__(self, static_map: Mapping, params: Mapping, *, box_kind: str, slot_id: str,
                 slot_xy: Sequence[float], skill_factory, pose_estimate_cls, search_rows_y: Sequence[float],
                 robot_id: str = 'r1', seed: int = 0):
        if box_kind != 'cyan':
            raise ValueError('M1 v1 delivers the cyan box')
        self.map = static_map
        self.static_keepouts = []
        self.params = params
        self.box_kind = box_kind
        self.slot_id = slot_id
        self.slot_xy = (float(slot_xy[0]), float(slot_xy[1]))
        self.skill_factory = skill_factory
        self.pose_estimate_cls = pose_estimate_cls
        self.robot_id = robot_id
        self.seed = seed
        self.pose = OwnCamPoseSource(static_map, params, seed=seed)
        door = next(p for p in static_map['passages'] if p['kind'] == 'door')
        self.door_xy = (float(door['center_m'][0]), float(door['center_m'][1]))
        self.exit_xy = (self.door_xy[0] + DOOR_EXIT_M, self.door_xy[1])
        self.search_rows_y = sorted(float(y) for y in search_rows_y)
        self.viewpoints: list[tuple[float, float]] = []
        self.phase = 'init'
        self.view_index = 0
        self.leg: _LegDriver | None = None
        self.sweep: dict | None = None
        self.cyan: list[dict] = []
        self.seen: list[dict] = []           # every box seen (any colour), map frame, for keep-outs
        self.last_skill_frame: int | None = None
        self.target_xy: tuple[float, float] | None = None
        self.pickup_source: str | None = None
        self.skill = None
        self.servo: dict[int, int] = {}
        self.last_obs: Mapping | None = None
        self.last_frame_id: int | None = None
        self.last_look_t: float | None = None
        self.look_count = 0
        self.gate_looks = 0
        self.preplace_look_done = False
        self.reanchor_needed = False
        self.lookback_gate: dict | None = None
        self.outcome: str | None = None
        self.events: list[dict] = []
        self.pose_sources: set[str] = set()
        self.face_fallback_used = False
        self.carry_leg_done = False
        self.init_looks = 0

    # ---------------------------------------------------------------- inputs
    def on_command(self, row: Mapping) -> None:
        self.pose.on_command(row)
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])
        if self.leg is not None:
            self.leg.on_command(row)

    def on_frame(self, now: float, obs: Mapping, rgb: np.ndarray) -> PoseReport:
        """Every own frame (5 Hz, and each skill decision frame)."""
        m1_contract.validate_observation(obs, robot_id=self.robot_id, previous_frame_id=self.last_frame_id, now=now)
        self.last_frame_id = int(obs['frame_id'])
        self.last_obs = obs
        report = self.pose.on_frame(now, rgb)
        if self.sweep is not None and self.sweep.get('purpose') == 'search' and self.sweep.get('settled'):
            self._search_detect(obs, report)
        return report

    # ---------------------------------------------------------------- helpers
    def _event(self, now, kind, **detail):
        self.events.append({'t': round(now, 3), 'event': kind, 'phase': self.phase, **detail})

    def _keepouts(self):
        """Planning obstacles: boxes seen with own RGB (any colour), except the target itself."""
        out = []
        for i, (xy, kind, n) in enumerate(self._seen_clusters()):
            if self.target_xy is not None and math.hypot(xy[0] - self.target_xy[0], xy[1] - self.target_xy[1]) < TARGET_MATCH_M:
                continue
            out.append({'id': f'seen_box_{i}', 'center_m': list(xy), 'half_extents_m': [SEEN_BOX_HALF_M]*2,
                        'source': f'own RGB detect_own ({kind}, {n} frames)'})
        return out

    def _seen_clusters(self):
        clusters = []
        for d in self.seen:
            for c in clusters:
                if math.hypot(c['xy'][0] - d['map_xy'][0], c['xy'][1] - d['map_xy'][1]) < TARGET_MATCH_M:
                    c['pts'].append(d['map_xy']); c['xy'] = np.mean(c['pts'], 0).tolist(); break
            else:
                clusters.append({'xy': list(d['map_xy']), 'pts': [d['map_xy']], 'kind': d['kind']})
        return [(tuple(c['xy']), c['kind'], len(c['pts'])) for c in clusters if len(c['pts']) >= 2]

    def _start_leg(self, goal, *, loaded):
        self.leg = _LegDriver(self.pose.loc, self.map, self.params, loaded=loaded, goal_xy=goal,
                              door_xy=self.door_xy, keepouts=self._keepouts(), initial_servo=dict(self.servo),
                              seed=self.seed)
        if loaded:
            self.leg.drive_pose = dict(CARRY_POSTURE)
        self.leg_goal = tuple(goal)

    def _arm_steps(self, target: Mapping[int, int]):
        out = []
        for servo, want in sorted(target.items()):
            cur = self.servo.get(servo, want)
            if cur == want:
                continue
            nxt = cur + int(np.clip(want - cur, -ARM_STEP_PWM, ARM_STEP_PWM))
            out.append({'kind': 'look', 'pan_pulse': nxt} if servo == 6 else
                       {'kind': 'arm', 'servo_id': servo, 'pulse': nxt})
        return out

    def _start_sweep(self, now, purpose, pose, pans, restore, reason):
        self.sweep = {'purpose': purpose, 'pose': dict(pose), 'queue': list(pans), 'restore': dict(restore),
                      'stage': 'arm', 'since': now, 'settled': False, 'reason': reason}
        self.look_count += 1 if purpose == 'look' else 0
        self._event(now, 'sweep_start', purpose=purpose, reason=reason)

    def _tick_sweep(self, now):
        """Arm to the sweep pose, dwell at each pan (frames are consumed), restore. None when done."""
        s = self.sweep
        if s['stage'] == 'arm':
            steps = self._arm_steps(s['pose'])
            if steps:
                return [{'kind': 'hold'}] + steps
            s['stage'], s['since'] = 'pan', now
            s['target'] = s['queue'].pop(0)
            return [{'kind': 'hold'}]
        if s['stage'] == 'pan':
            steps = self._arm_steps({6: s['target']})
            if steps:
                s['since'], s['settled'] = now, False
                return [{'kind': 'hold'}] + steps
            if now - s['since'] < SETTLE_S:
                s['settled'] = now - s['since'] >= .2
                return [{'kind': 'hold'}]
            if s['queue']:
                s['target'] = s['queue'].pop(0)
                s['since'], s['settled'] = now, False
                return [{'kind': 'hold'}]
            s['stage'], s['since'], s['settled'] = 'restore', now, False
            return [{'kind': 'hold'}]
        steps = self._arm_steps(s['restore'])
        if steps:
            return [{'kind': 'hold'}] + steps
        if now - s['since'] < SETTLE_S:
            return [{'kind': 'hold'}]
        if s['purpose'] == 'look':
            self.last_look_t = now
        rep = self.pose.report(now)
        self._event(now, 'sweep_done', purpose=s['purpose'], report=rep.as_dict())
        self.sweep = None
        return None

    def _search_detect(self, obs, report: PoseReport):
        from harness.zone_color_boxes import OWN_PROFILE_ZONE, detect_own
        if not report.initialized:
            return
        pose = obs['actuator_state']['servo_pulses']
        result = detect_own(obs['image'], pose, profile=OWN_PROFILE_ZONE)
        c, s = math.cos(report.yaw_rad), math.sin(report.yaw_rad)
        for d in result['detections']:
            bx, by = d['estimated_box_center_base_m'][:2]
            row = {'t': report.t_est, 'kind': d['kind'], 'range_class': d['range_class'],
                   'map_xy': [report.x_m + c*bx - s*by, report.y_m + s*bx + c*by],
                   'base_xy': [bx, by], 'iou': d['floor_hypothesis_projection_iou'], 'std_xy_m': report.std_xy_m}
            if d['range_class'] == 'near':
                self.seen.append(row)
            if d['kind'] == self.box_kind:
                self.cyan.append(row)

    def _cluster(self, range_class):
        pts = [d['map_xy'] for d in self.cyan if d['range_class'] == range_class]
        if not pts:
            return None, 0
        a = np.asarray(pts)
        med = np.median(a, 0)
        near = a[np.hypot(*(a - med).T) < .15]
        return (float(near[:, 0].mean()), float(near[:, 1].mean())), len(near)

    def _estimate(self, report: PoseReport):
        est = self.pose_estimate_cls(report.x_m, report.y_m, report.yaw_rad, m1_contract.require_m1_source(report.source))
        self.pose_sources.add(est.source)
        return est

    # ---------------------------------------------------------------- decide
    def decide(self, now: float) -> dict:
        """One control decision: {'mode': 'tick', 'commands': [...]} (0.1 s) or {'mode': 'macro', 'action': ...}."""
        if self.outcome:
            return {'mode': 'done', 'outcome': self.outcome}
        if self.sweep is not None:
            cmds = self._tick_sweep(now)
            if cmds is not None:
                return {'mode': 'tick', 'commands': cmds}
            return self._after_sweep(now)
        handler = getattr(self, '_' + self.phase)
        return handler(now)

    def _after_sweep(self, now):
        if self.phase == 'search_sweep':
            near, n_near = self._cluster('near')
            far, n_far = self._cluster('far_coarse')
            self._event(now, 'search_result', near=near, n_near=n_near, far=far, n_far=n_far)
            if near is not None and n_near >= NEAR_MIN_DETECTIONS:
                self.target_xy, self.pickup_source = near, 'own_rgb_search'
                self.phase = 'approach_leg'
                self._start_leg((near[0] - PREGRASP_STANDOFF_M, near[1]), loaded=False)
            elif far is not None and not getattr(self, '_closer_done', False):
                self._closer_done = True
                self.phase = 'search_leg'
                self._start_leg((far[0] - CLOSER_VIEW_STANDOFF_M, far[1]), loaded=False)
            else:
                self.view_index += 1
                self._closer_done = False
                if self.view_index >= len(self.viewpoints):
                    self.outcome = 'SEARCH_NOT_FOUND'
                    return {'mode': 'done', 'outcome': self.outcome}
                self.phase = 'search_leg'
                self._start_leg(self.viewpoints[self.view_index], loaded=False)
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}   # next decision continues the phase

    def _init(self, now):
        """Localize first (look sweeps), then order the search viewpoints from the own estimate."""
        rep = self.pose.report(now)
        if rep.initialized and rep.std_xy_m <= LIMITS['nav_unloaded'].max_std_xy_m:
            self.viewpoints = [(SEARCH_VIEW_X_M, y) for y in
                               sorted(self.search_rows_y, key=lambda y: (abs(y - rep.y_m), y))]
            self._event(now, 'initialized', report=rep.as_dict(), viewpoints=self.viewpoints)
            self.phase = 'search_leg'
            self._start_leg(self.viewpoints[0], loaded=False)
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        if self.init_looks >= MAX_GATE_LOOKS:
            self.outcome = 'NOT_INITIALIZED'
            return {'mode': 'done', 'outcome': self.outcome}
        self.init_looks += 1
        self._start_sweep(now, 'look', LOOK_P20, WIDE_LOOK_PANS, SEARCH_POSE, 'init')
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def _drive_leg(self, now):
        cmds = self.leg.tick(now)
        if self.leg.outcome:
            return None, self.leg.outcome
        return cmds, None

    def _search_leg(self, now):
        cmds, outcome = self._drive_leg(now)
        if outcome is None:
            return {'mode': 'tick', 'commands': cmds}
        if outcome != 'arrived':
            self.outcome = 'SEARCH_LEG_' + outcome
            return {'mode': 'done', 'outcome': self.outcome}
        self.phase = 'search_sweep'
        self.cyan = [d for d in self.cyan if False]
        self._start_sweep(now, 'search', SEARCH_POSE, SEARCH_PANS, SEARCH_POSE, 'viewpoint')
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def _approach_leg(self, now):
        cmds, outcome = self._drive_leg(now)
        if outcome is None:
            return {'mode': 'tick', 'commands': cmds}
        if outcome != 'arrived':
            self.outcome = 'APPROACH_LEG_' + outcome
            return {'mode': 'done', 'outcome': self.outcome}
        from harness.wrist_zone_skill import OrderSheet
        order = OrderSheet(self.box_kind, tuple(self.target_xy), self.slot_id, self.slot_xy)
        self.skill = self.skill_factory(order)
        self.leg = None
        self.phase = 'skill'
        self._event(now, 'skill_start', pickup_xy=[round(v, 4) for v in self.target_xy],
                    pickup_source=self.pickup_source)
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def _skill(self, now):
        obs = self.last_obs
        if obs is None or now - float(obs['sim_time']) > .25:
            return {'mode': 'capture'}                     # runner captures a fresh own frame first
        report = self.pose.report(now)
        sk = self.skill
        # Re-anchor the held box after an arm move made by this controller (looks),
        # once the arm is back in its carry pose (sweep restored / leg driving again).
        leg_busy = self.leg is not None and self.leg.state in ('look_arm', 'look_pan', 'posture_back')
        if self.reanchor_needed and not leg_busy and obs['frame_id'] != self.last_skill_frame:
            self.last_skill_frame = obs['frame_id']
            check = sk.box.reanchor_after_posture(obs)
            self.reanchor_needed = False
            self._event(now, 'reanchor', attached=bool(check.get('attached')))
            if not check.get('attached'):
                self.outcome = 'CARRY_REANCHOR_UNCONFIRMED'
                return {'mode': 'done', 'outcome': self.outcome}
            return {'mode': 'capture'}
        phase = sk.phase
        in_carry_leg = phase == 'nav_preplace' and not self.carry_leg_done   # the leg driver owns looks there
        limits = None if in_carry_leg else {'nav_pregrasp': LIMITS['nav_unloaded'], 'nav_preplace': LIMITS['nav_loaded'],
                                            'pre_release': LIMITS['release']}.get(phase)
        since_look = None if self.last_look_t is None else now - self.last_look_t
        if phase == 'look_back' and sk.look_back_steps == 0 and self.lookback_gate is None:
            if self.last_look_t is None or since_look > LIMITS['look_back'].max_since_look_s:
                return self._gate_look(now, 'look_back_fresh', obs, loaded=False)
            bad = check_limits(report, now, LIMITS['look_back'], since_look_s=since_look)
            self.lookback_gate = {'t': round(now, 3), 'violations': bad, 'report': report.as_dict()}
            self._event(now, 'look_back_gate', **self.lookback_gate)
        elif limits is not None:
            bad = check_limits(report, now, limits, since_look_s=since_look)
            if phase == 'nav_preplace' and not bad and self.carry_leg_done and not self.preplace_look_done:
                goal = sk._preplace_goal()
                if math.hypot(goal[0] - report.x_m, goal[1] - report.y_m) < PREPLACE_LOOK_RADIUS_M:
                    self.preplace_look_done = True
                    return self._gate_look(now, 'preplace', obs, loaded=True)
            if bad:
                if self.gate_looks >= MAX_GATE_LOOKS:
                    self.outcome = 'POSE_UNCERTAIN'
                    self._event(now, 'pose_uncertain', gate=limits.name, violations=bad, report=report.as_dict())
                    return {'mode': 'done', 'outcome': self.outcome}
                self.gate_looks += 1
                return self._gate_look(now, f'gate:{limits.name}:{",".join(bad)}', obs,
                                       loaded=phase != 'nav_pregrasp')
            self.gate_looks = 0
        if not report.initialized and not in_carry_leg:
            return self._gate_look(now, 'not_initialized', obs, loaded=sk.box.held)
        est = self._estimate(report) if report.initialized else None
        # Carry leg: the skill's carry checks run on every frame; its navigation is
        # replaced by the driver (heading east, stop-and-look) until the door exit.
        if phase == 'nav_preplace' and not self.carry_leg_done:
            if self.leg is None:
                self._start_leg(self.exit_xy, loaded=True)
                self.leg.state, self.leg.state_since = 'drive', now
            if est is not None and self.leg.state == 'drive' and obs['frame_id'] != self.last_skill_frame:
                # Own-RGB carry check on each new frame while the arm is in the carry posture.
                self.last_skill_frame = obs['frame_id']
                action = sk.decide(obs, est)
                if sk.phase != 'nav_preplace' or action.get('kind') not in ('mecanum', 'drive'):
                    self.leg = None                          # skill takes over (re-seat, grip check, finish)
                    if action.get('kind') == 'finish':
                        self.outcome = 'SKILL_' + action['reason']
                        return {'mode': 'done', 'outcome': self.outcome}
                    return {'mode': 'macro', 'action': action}
            cmds, outcome = self._drive_leg(now)
            if outcome is None:
                if self.leg.state in ('look_arm', 'look_pan', 'posture_back'):
                    self.reanchor_needed = True          # handled once the leg is driving again
                return {'mode': 'tick', 'commands': cmds}
            self._event(now, 'carry_leg_end', outcome=outcome, looks=self.leg.looks)
            self.carry_leg_done = True
            self.leg = None
            if outcome != 'arrived':
                self.outcome = 'CARRY_LEG_' + outcome
                return {'mode': 'done', 'outcome': self.outcome}
            self.reanchor_needed = True
            return {'mode': 'capture'}
        action = sk.decide(obs, est)
        if action.get('kind') == 'finish':
            self.outcome = 'SKILL_' + action['reason']
            return {'mode': 'done', 'outcome': self.outcome}
        return {'mode': 'macro', 'action': action}

    def _gate_look(self, now, reason, obs, *, loaded):
        pose = {int(k): int(v) for k, v in obs['actuator_state']['servo_pulses'].items()}
        restore = {k: v for k, v in pose.items() if k in (1, 3, 4, 5, 6)}
        look_pose = dict(LOOK_P20)
        if loaded:
            look_pose[1] = restore.get(1, 1500)             # keep the grip as issued
        self._start_sweep(now, 'look', look_pose, WIDE_LOOK_PANS, restore, reason)
        self.reanchor_needed = bool(loaded and self.skill is not None and self.skill.box.held)
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def summary(self) -> dict:
        if self.skill is not None:
            self.face_fallback_used = any(
                e.get('event') == 'grasp_attached' and str(e.get('face_normal_source', '')).startswith('static_map')
                for e in self.skill.events)
        return {'schema': SCHEMA, 'outcome': self.outcome, 'phase': self.phase, 'looks': self.look_count,
                'target_xy': self.target_xy, 'pickup_source': self.pickup_source,
                'cyan_detections': len(self.cyan), 'pose_sources': sorted(self.pose_sources),
                'face_fallback_used': self.face_fallback_used, 'lookback_gate': self.lookback_gate,
                'carry_leg_done': self.carry_leg_done,
                'skill_summary': self.skill.summary() if self.skill is not None and hasattr(self.skill, 'summary') else None,
                'skill_placement': getattr(self.skill, 'placement', None)}
