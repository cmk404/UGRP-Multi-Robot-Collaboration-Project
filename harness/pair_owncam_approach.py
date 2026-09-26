"""M2 pair approach: own-camera localized drive to a pre-station with a final heading.

Student only (nothing here imports the simulator). Inputs are exactly those of
the M1 loop driver (``harness.owncam_drive.OwnCamDriver``, imported read-only
from PR #201 e10f88d): the robot's own issued commands, its own ``robot_cam``
frames, the static tagged map, the fixed loop-v2 calibration and static
keep-outs supplied by the caller. The goal (pre-station x, y, heading) comes
from the COARSE ORDER SHEET (beam pose rounded to a grid, see
``coarse_order_sheet``), never from live object or robot positions.

Differences from the M1 driver (all confined to this subclass):

* the goal has a final heading; while driving the heading is steered toward
  it (M1 holds east), with translation computed in the body frame from the
  estimated yaw (mecanum, holonomic);
* a symmetric planning envelope (``APPROACH_ENVELOPE``), because the body
  turns while it travels (M1: fixed east-facing envelope);
* arrival needs position AND heading within tolerance, then one stop-and-look
  arrival check (as M1);
* ``door_xy=None`` disables the door checkpoints (open floor, stage 1).

Everything else (look posture, wide pans, look triggers, lost / not-initialised
stop rules, re-planning every 1 s) is the inherited M1 v2 behaviour (unloaded).
"""
from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from harness.owncam_drive import ARRIVE_TOL_M, PLAN_FAIL_LIMIT
from harness.owncam_drive_v2 import OwnCamDriverV2
from harness.owncam_localizer import wrap

SCHEMA = 'ugrp.pair_owncam_approach.v1'
# Body turns while it travels: symmetric square that holds the chassis (+-0.15 m)
# plus the SEARCH-pose wrist overhang at any heading.
APPROACH_ENVELOPE = {'x_m': [-.20, .20], 'y_m': [-.20, .20]}
ARRIVE_TOL_YAW_RAD = .06
TURN_GAIN, TURN_MAX = .6, .10
FAR = (1e9, 1e9)
# Coarse order sheet grid: beam centre to 0.10 m, beam axis to 10 degrees.
SHEET_XY_M = .10
SHEET_YAW_RAD = math.radians(10.)


def coarse_order_sheet(beam_pose: Sequence[float]) -> dict:
    """The order sheet's beam pose: the setup pose rounded to the sheet grid.

    This is what a coarse work order would say ("the beam lies about here,
    roughly along this axis"). It is static task information fixed before the
    run; the robots never receive the setup pose itself.
    """
    x, y, yaw = (float(v) for v in beam_pose)
    q = lambda v, g: round(round(v / g) * g, 6)   # noqa: E731
    return {'beam_xyyaw': [q(x, SHEET_XY_M), q(y, SHEET_XY_M), q(yaw, SHEET_YAW_RAD)],
            'grid': {'xy_m': SHEET_XY_M, 'yaw_rad': round(SHEET_YAW_RAD, 6)},
            'source': 'coarse order sheet (setup pose rounded to the sheet grid; static, fixed before the run)'}


def prestation(station_xyyaw: Sequence[float], back_m: float) -> list[float]:
    x, y, yaw = (float(v) for v in station_xyyaw)
    return [x - back_m * math.cos(yaw), y - back_m * math.sin(yaw), yaw]


def beam_keepout(sheet_xyyaw: Sequence[float], length_m: float, width_m: float, pad_m: float) -> dict:
    """Axis-aligned keep-out holding the order-sheet beam footprint plus the sheet error pad."""
    x, y, yaw = (float(v) for v in sheet_xyyaw)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    hx = .5 * (length_m * c + width_m * s) + pad_m
    hy = .5 * (length_m * s + width_m * c) + pad_m
    return {'id': 'order_sheet_beam', 'center_m': [x, y], 'half_extents_m': [hx, hy],
            'source': 'order-sheet beam footprint + sheet grid error pad (static), not a live pose'}


class PairApproachDriver(OwnCamDriverV2):
    """Unloaded own-camera approach to ``goal_xyyaw`` (x, y, final heading)."""
    version = 'pair_approach_v1'

    def __init__(self, static_map: Mapping, params: Mapping, *, goal_xyyaw: Sequence[float],
                 door_xy: Sequence[float] | None = None, keepouts: Sequence[Mapping] = (),
                 initial_servo: Mapping | None = None, seed: int = 0, width: int = 640, height: int = 480):
        super().__init__(static_map, params, loaded=False, goal_xy=goal_xyyaw[:2],
                         door_xy=FAR if door_xy is None else door_xy, keepouts=keepouts,
                         initial_servo=initial_servo, seed=seed, width=width, height=height)
        self.goal_yaw = float(goal_xyyaw[2])
        self.envelope = APPROACH_ENVELOPE
        self.has_door = door_xy is not None

    def _needs_look(self, est, now):
        if not self.has_door:
            # Same triggers as the parent minus the door checkpoints (the far dummy
            # door is never within a checkpoint radius, but be explicit).
            self.checkpoints_done.update((1.5, .6))
        return super()._needs_look(est, now)

    def tick(self, now: float) -> list[dict]:
        if self.outcome:
            return []
        if self.state != 'drive' or self.state_since is None:
            return super().tick(now)
        self.loc.predict_to(now)
        est = self.loc.estimate()
        reason = self._needs_look(est, now)
        if reason:
            return self._start_look(now, reason)
        x, y, yaw = est['x'], est['y'], est['yaw']
        dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
        herr = float(wrap(self.goal_yaw - yaw))
        turn = float(np.clip(TURN_GAIN * herr, -TURN_MAX, TURN_MAX))
        if dist <= ARRIVE_TOL_M:
            if abs(herr) <= ARRIVE_TOL_YAW_RAD:
                if self.arrival_checked:
                    return self._arrive(now)
                self.arrival_checked = True     # one final stop-and-look before declaring
                return self._start_look(now, 'arrival_check')
            return [{'kind': 'mecanum', 'forward': 0., 'left': 0., 'turn': turn, 'duration_s': .15}]
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
        near_door = self.has_door and math.hypot(self.door[0] - x, self.door[1] - y) < .7
        speed = min(1., dist / .25) * (.6 if near_door else 1.)
        vx, vy = wx / norm * speed, wy / norm * speed
        fwd = math.cos(yaw) * vx + math.sin(yaw) * vy
        left = -math.sin(yaw) * vx + math.cos(yaw) * vy
        return [{'kind': 'mecanum', 'forward': float(np.clip(.12 * fwd, -.05, .12)),
                 'left': float(np.clip(.08 * left, -.08, .08)), 'turn': turn, 'duration_s': .15}]

    def _arrive(self, now):
        est = self.loc.estimate()
        self._event(now, 'arrived', estimate=[round(est['x'], 4), round(est['y'], 4), round(est['yaw'], 4)],
                    std_xy_m=est['std_xy_m'], goal=[round(v, 4) for v in (*self.goal, self.goal_yaw)])
        return self._finish(now, 'arrived')


# ---------------------------------------------------------------------------------------------
# v2 (after stage 3 seed 723): r2 turned ~pi WHILE driving; the loop-v2 motion model (fit with the
# heading held east) under-predicted the combined turn, the estimate left by 0.43 rad / 0.76 m and
# settled in a confident wrong mode (std ~0.06), and the inherited unloaded look rule (look when
# std > 0.05, 'lost' only above 0.15) looped 20 looks without moving. A first v2 draft (b70a284)
# rotated in place at the start; at the r2 spawn that faces the west wall 0.16 m away, so no tag
# was visible, the in-place rotation was under-predicted by 0.67 rad and tagless looks still
# counted as fixes (dev4-723). v2 therefore:
#   * translates with the heading HELD at the heading estimated when driving starts (the regime
#     the M1 driver and the loop-v2 calibration use) until within TURN_NEAR_M of the pre-station;
#   * there rotates IN PLACE to the final heading in steps of at most TURN_STEP_RAD of estimated
#     rotation with a stop-and-look after every step (open floor, tags on several walls), then
#     finishes with the v1 final approach (heading held at the goal heading from then on);
#   * counts a look that saw no tag as NOT fixed, and relocalizes from scratch (fresh localizer,
#     same init look sweep) after MAX_LOOKS_WITHOUT_FIX unfixed looks; after MAX_RELOCALIZE such
#     restarts the outcome is 'lost'.
TURN_NEAR_M = .30
TURN_FIRST_RAD = .25
TURN_STEP_RAD = .8
TURN_IN_PLACE = .10          # issued turn command while rotating in place
MAX_RELOCALIZE = 2


class PairApproachDriverV2(PairApproachDriver):
    version = 'pair_approach_v2'

    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.turn_ref = None
        self.relocalizations = 0
        self.hold_yaw = None
        self.rotated = False
        self.look_t0 = None

    def _start_look(self, now, reason):
        self.look_t0 = now
        return super()._start_look(now, reason)

    def _relocalize(self, now):
        from harness.owncam_localizer import OwnCamLocalizer
        self.relocalizations += 1
        seed = int(self.loc.rng.integers(1 << 30))
        self.loc = OwnCamLocalizer(self.map, self.loc.params, seed=seed)
        self.loc.command({'t': float(now), 'kind': 'initial_servo_command', 'pulses': dict(self.servo)})
        self.looks_without_fix = 0
        self.turn_ref = None
        self.path = None
        self._event(now, 'relocalize', count=self.relocalizations)
        return self._start_look(now, 'relocalize')

    def _look_step(self, now):
        prev, before = self.looks_without_fix, len(self.log)
        out = super().tick(now)
        for e in self.log[before:]:
            if e['event'] == 'look_done' and e.get('fixed'):
                seen = self.loc.last_tag_t is not None and self.look_t0 is not None and self.loc.last_tag_t >= self.look_t0
                if not seen:
                    self.looks_without_fix = prev + 1
                    e['fixed'] = False
                    self._event(now, 'look_no_tags', looks_without_fix=self.looks_without_fix)
        return out

    def tick(self, now: float) -> list[dict]:
        from harness.owncam_drive import MAX_LOOKS_WITHOUT_FIX
        if self.outcome:
            return []
        if self.state == 'look_pan':
            return self._look_step(now)
        if self.state != 'drive' or self.state_since is None:
            return super().tick(now)
        if self.looks_without_fix >= MAX_LOOKS_WITHOUT_FIX:
            if self.relocalizations >= MAX_RELOCALIZE:
                return self._finish(now, 'lost')
            return self._relocalize(now)
        self.loc.predict_to(now)
        est = self.loc.estimate()
        if not est.get('initialized'):
            return self._start_look(now, 'not_initialized')
        if self.hold_yaw is None:
            self.hold_yaw = float(est['yaw'])
            self._event(now, 'hold_heading', yaw=round(self.hold_yaw, 4))
        dist = math.hypot(self.goal[0] - est['x'], self.goal[1] - est['y'])
        if not self.rotated and dist > TURN_NEAR_M:
            goal_yaw, self.goal_yaw = self.goal_yaw, self.hold_yaw     # translate, heading held
            try:
                return super().tick(now)
            finally:
                self.goal_yaw = goal_yaw
        herr = float(wrap(self.goal_yaw - est['yaw']))
        if abs(herr) > TURN_FIRST_RAD:
            self.rotated = True
            if self.turn_ref is None:
                self.turn_ref = est['yaw']
            if abs(float(wrap(est['yaw'] - self.turn_ref))) >= TURN_STEP_RAD:
                self.turn_ref = None
                return self._start_look(now, 'turn_step')
            return [{'kind': 'mecanum', 'forward': 0., 'left': 0., 'turn': math.copysign(TURN_IN_PLACE, herr),
                     'duration_s': .15}]
        self.rotated = True
        if self.turn_ref is not None:            # rotation finished: look once before the final approach
            self.turn_ref = None
            return self._start_look(now, 'turn_done')
        return super().tick(now)
