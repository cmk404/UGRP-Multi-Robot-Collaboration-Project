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
