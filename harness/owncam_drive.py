"""Own-camera closed-loop driving on a tagged zone map (student, no simulator).

The robot drives on its own pose estimate only:

* inputs: its issued commands (fed back from its own port), its wrist RGB
  frames, the static tagged map, fixed calibrations, and static layout
  keep-outs supplied by the caller (for example the fixed pickup-grid cells
  where boxes may stand; never live object positions);
* localization: ``harness.owncam_localizer.OwnCamLocalizer``;
* planning: ``harness.map_goto.plan_path`` (grid A*, fixed east-facing body
  envelope) from the ESTIMATED start/current pose;
* control: waypoint pursuit with mecanum commands, heading held east;
* stop-and-look: stop, move the wrist to ``LOOK_P20`` and pan through
  ``WIDE_LOOK_PANS`` when the estimate is uncertain, when no tag has been seen for a
  while (unloaded) or after ``LOADED_LOOK_EVERY_M`` of estimated travel (box held),
  and once at each door checkpoint (1.5 m and 0.6 m before the door).

Nothing here imports the simulator; ``tests/test_owncam_localizer.py`` checks it.
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from harness.map_goto import UNLOADED_ENVELOPE, plan_path
from harness.owncam_localizer import OwnCamLocalizer, wrap
from harness.wall_tags import TagDetector

SCHEMA = 'ugrp.owncam_drive.v1'
# Shared named postures (issued PWM). CARRY_POSTURE and LOOK_P20 are copied
# from PR #176 (harness/wrist_zone_skill.py, carry-view study) so both
# branches use one posture; tests pin the values.
CARRY_POSTURE = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
LOOK_P20 = {3: 1072, 4: 2400, 5: 1482}
LOOK_PANS = (1500, 1230, 1770, 1500)
# Localization sweep of this driver (dev amendment 2026-09-26, not shared with
# #176): LOOK_PANS plus +-48 deg (11.1 PWM/deg). With the box held, near/low
# tags are hidden behind it, and from inside the doorway only the far east wall
# (~3.4 m, a narrow bearing cone where yaw and y trade off) is in the +-24 deg
# sweep; the wide pans reach side-wall tags ~1.4-2 m away that pin y.
WIDE_LOOK_PANS = (1500, 1230, 970, 1770, 2030, 1500)
SEARCH_POSE = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
# Loaded body envelope relative to the chassis origin (east-facing): chassis
# +-0.15 m (as UNLOADED_ENVELOPE) plus the box held ~0.14 m ahead in CARRY_POSTURE.
LOADED_ENVELOPE = {'x_m': [-.15, .20], 'y_m': [-.15, .15]}
CONTROL_S = .1
ARM_STEP_PWM = 60            # issued arm interpolation per control tick (0.6 PWM/ms)
SETTLE_S = .6
DOOR_CHECKPOINTS_M = (1.5, .6)
DOOR_EXIT_M = .45
LOOK_IF_STD_XY_M = .05
LOOK_IF_STD_YAW_RAD = math.radians(3.)
LOOK_IF_NO_TAG_S = 3.
# Loaded (box held in CARRY_POSTURE) the drive view shows no tags by design, so
# 'no tag for 3 s' fired every ~3 s of driving and looks took ~70% of the time
# (dev-a3 dev-box-s33 hit the 240 s limit). Loaded, look again after this much
# ESTIMATED travel since the last look instead (dev amendment 2026-09-26).
LOADED_LOOK_EVERY_M = .35
ARRIVE_TOL_M = .03
MAX_LOOKS_WITHOUT_FIX = 3
LOST_STD_XY_M = .15
PLAN_FAIL_LIMIT = 3


class OwnCamDriver:
    """State machine: arm to drive posture -> (look) -> plan -> drive -> arrive."""

    def __init__(self, static_map: Mapping, params: Mapping, *, loaded: bool, goal_xy: Sequence[float],
                 door_xy: Sequence[float], keepouts: Sequence[Mapping] = (), initial_servo: Mapping | None = None,
                 seed: int = 0, width: int = 640, height: int = 480):
        self.map = static_map
        self.loc = OwnCamLocalizer(static_map, params, seed=seed)
        self.detector = TagDetector.for_map(static_map, width, height)
        self.loaded = bool(loaded)
        self.goal = [float(goal_xy[0]), float(goal_xy[1])]
        self.door = [float(door_xy[0]), float(door_xy[1])]
        self.keepouts = [dict(k) for k in keepouts]
        self.envelope = LOADED_ENVELOPE if loaded else UNLOADED_ENVELOPE
        self.drive_pose = dict(CARRY_POSTURE) if loaded else dict(SEARCH_POSE)
        self.servo = {int(k): int(v) for k, v in (initial_servo or {}).items()}
        self.state, self.state_since = 'posture', None
        self.arm_target: dict[int, int] = {}
        self.look_queue: list[int] = []
        self.look_reason = None
        self.looks, self.looks_without_fix = 0, 0
        self.checkpoints_done: set[float] = set()
        self.path, self.plan_at, self.plan_fails = None, -1., 0
        self.outcome = None
        self.log: list[dict] = []
        self.last_estimate = self.loc.estimate()
        self.frames_seen = 0
        self.arrival_checked = False
        self.last_look_xy = None

    # ---------------------------------------------------------- inputs
    def on_command(self, row: Mapping) -> None:
        """Every command this robot issued (own port log), in time order."""
        self.loc.command(row)
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])

    def observe(self, now: float, rgb: np.ndarray) -> dict:
        """A fresh wrist frame: detect tags and update the estimate."""
        dets = self.detector.detect(rgb)
        self.frames_seen += 1
        self.last_estimate = self.loc.update(now, dets, self.servo)
        self.last_estimate['tags'] = [d['id'] for d in dets]
        return self.last_estimate

    # ---------------------------------------------------------- control
    def _event(self, now, kind, **detail):
        self.log.append({'t': round(now, 3), 'event': kind, 'state': self.state, **detail})

    def _set(self, state, now, **detail):
        self.state, self.state_since = state, now
        self._event(now, 'state', **detail)

    def _arm_step(self):
        """Issue one interpolation step toward arm_target; [] when there."""
        out = []
        for servo, target in sorted(self.arm_target.items()):
            cur = self.servo.get(servo, target)
            if cur == target:
                continue
            nxt = cur + int(np.clip(target - cur, -ARM_STEP_PWM, ARM_STEP_PWM))
            out.append({'kind': 'look', 'pan_pulse': nxt} if servo == 6 else
                       {'kind': 'arm', 'servo_id': servo, 'pulse': nxt})
        return out

    def _needs_look(self, est, now):
        if not est.get('initialized'):
            return 'not_initialized'
        if est['std_xy_m'] > LOOK_IF_STD_XY_M or est['std_yaw_rad'] > LOOK_IF_STD_YAW_RAD:
            return 'uncertain'
        if self.loaded:
            if self.last_look_xy is None:
                self.last_look_xy = (est['x'], est['y'])
            elif math.hypot(est['x'] - self.last_look_xy[0], est['y'] - self.last_look_xy[1]) > LOADED_LOOK_EVERY_M:
                return 'travel'
        elif est['since_tag_s'] is not None and est['since_tag_s'] > LOOK_IF_NO_TAG_S:
            return 'no_tag'
        if est['x'] < self.door[0]:
            d = math.hypot(self.door[0] - est['x'], self.door[1] - est['y'])
            for cp in DOOR_CHECKPOINTS_M:
                if cp not in self.checkpoints_done and d <= cp:
                    self.checkpoints_done.add(cp)
                    return f'door_checkpoint_{cp}'
        return None

    def _start_look(self, now, reason):
        self.looks += 1
        self.look_reason = reason
        self.look_queue = list(WIDE_LOOK_PANS)
        self.arm_target = dict(LOOK_P20)
        self._set('look_arm', now, reason=reason, look=self.looks)
        return [{'kind': 'hold'}]

    def _plan(self, est, now):
        start = [est['x'], est['y']]
        result = plan_path(self.map, start, self.goal, self.envelope, obstacles=self.keepouts,
                           escape_start_m=.25)
        self.plan_at = now
        if result is None:
            self.plan_fails += 1
            self._event(now, 'plan_failed', start=[round(v, 3) for v in start], fails=self.plan_fails)
            return False
        self.plan_fails = 0
        self.path = [list(p) for p in result['waypoints_m'][1:]]
        self._event(now, 'plan', length_m=round(result['length_m'], 3), waypoints=len(self.path),
                    plan_sha256=result['plan_sha256'])
        return True

    def tick(self, now: float) -> list[dict]:
        """One control step (every CONTROL_S). Returns commands to issue."""
        if self.outcome:
            return []
        if self.state_since is None:
            self.state_since = now
            self.arm_target = dict(self.drive_pose)
        self.loc.predict_to(now)
        est = self.loc.estimate()
        if self.state == 'posture':
            steps = self._arm_step()
            if steps:
                return [{'kind': 'hold'}] + steps
            if now - self.state_since < SETTLE_S:
                return [{'kind': 'hold'}]
            self._set('drive', now)
            reason = self._needs_look(est, now)
            return self._start_look(now, reason) if reason else [{'kind': 'hold'}]
        if self.state == 'look_arm':
            steps = self._arm_step()
            if steps:
                return [{'kind': 'hold'}] + steps
            self._set('look_pan', now)
            self.arm_target = {6: self.look_queue.pop(0)}
            return [{'kind': 'hold'}]
        if self.state == 'look_pan':
            steps = self._arm_step()
            if steps:
                self.state_since = now
                return [{'kind': 'hold'}] + steps
            if now - self.state_since < SETTLE_S:
                return [{'kind': 'hold'}]      # frames keep arriving while settled
            if self.look_queue:
                self.arm_target = {6: self.look_queue.pop(0)}
                self.state_since = now
                return [{'kind': 'hold'}]
            est = self.loc.estimate()
            fixed = est.get('initialized') and est['std_xy_m'] <= LOOK_IF_STD_XY_M
            self.looks_without_fix = 0 if fixed else self.looks_without_fix + 1
            if est.get('initialized'):
                self.last_look_xy = (est['x'], est['y'])
            self._event(now, 'look_done', fixed=bool(fixed), std_xy_m=est.get('std_xy_m'),
                        initialized=est.get('initialized'))
            if not est.get('initialized') and self.looks_without_fix >= MAX_LOOKS_WITHOUT_FIX:
                return self._finish(now, 'not_initialized')
            if est.get('initialized') and est['std_xy_m'] > LOST_STD_XY_M and \
                    self.looks_without_fix >= MAX_LOOKS_WITHOUT_FIX:
                return self._finish(now, 'lost')
            if not fixed and self.looks_without_fix < MAX_LOOKS_WITHOUT_FIX:
                return self._start_look(now, 'refix')
            self.arm_target = dict(self.drive_pose)
            self.path = None
            self._set('posture_back', now)
            return [{'kind': 'hold'}]
        if self.state == 'posture_back':
            steps = self._arm_step()
            if steps:
                return [{'kind': 'hold'}] + steps
            if now - self.state_since < SETTLE_S:
                return [{'kind': 'hold'}]
            self._set('drive', now)
            return [{'kind': 'hold'}]
        if self.state == 'drive':
            reason = self._needs_look(est, now)
            if reason:
                return self._start_look(now, reason)
            x, y, yaw = est['x'], est['y'], est['yaw']
            dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
            if dist <= ARRIVE_TOL_M:
                if self.arrival_checked:
                    return self._arrive(now)
                self.arrival_checked = True     # one final stop-and-look before declaring
                return self._start_look(now, 'arrival_check')
            if self.path is None or now - self.plan_at >= 1.:
                if not self._plan(est, now):
                    if self.plan_fails >= PLAN_FAIL_LIMIT:
                        return self._finish(now, 'no_path')
                    return [{'kind': 'hold'}]
            while len(self.path) > 1 and math.hypot(self.path[0][0] - x, self.path[0][1] - y) < .10:
                self.path.pop(0)
            tx, ty = self.path[0]
            wx, wy = tx - x, ty - y
            norm = max(math.hypot(wx, wy), 1e-9)
            near_door = math.hypot(self.door[0] - x, self.door[1] - y) < .7
            speed = min(1., dist/.25) * (.6 if near_door else 1.)
            vx, vy = wx/norm*speed, wy/norm*speed
            fwd = math.cos(yaw)*vx + math.sin(yaw)*vy
            left = -math.sin(yaw)*vx + math.cos(yaw)*vy
            err = float(wrap(0. - yaw))
            scale = 1. if abs(err) < .3 else .3
            return [{'kind': 'mecanum', 'forward': float(np.clip(.12*fwd*scale, -.05, .12)),
                     'left': float(np.clip(.08*left*scale, -.08, .08)),
                     'turn': float(np.clip(.6*err, -.10, .10)), 'duration_s': .15}]
        return []

    def _arrive(self, now):
        est = self.loc.estimate()
        self._event(now, 'arrived', estimate=[round(est['x'], 4), round(est['y'], 4), round(est['yaw'], 4)],
                    std_xy_m=est['std_xy_m'])
        return self._finish(now, 'arrived')

    def _finish(self, now, outcome):
        self.outcome = outcome
        self._event(now, 'finish', outcome=outcome, looks=self.looks)
        return [{'kind': 'hold'}]
