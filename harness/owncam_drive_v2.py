"""Own-camera closed-loop driver, loop v2 look policy (box held).

Same student as ``harness.owncam_drive.OwnCamDriver`` (inputs, planner, pursuit,
postures and sweep unchanged); only the stop-and-look policy while the box is
held changes. Values come from dev statistics of the first loop cohort
(experiments/2026-09-26-zone-owncam-loop-v2/README.md):

* hysteresis: a look counts as a fix at std_xy <= 0.06 m, while driving the
  'uncertain' trigger needs std_xy > 0.07 m (or yaw std > 3 deg) AND >= 0.10 m of
  estimated travel since the last look. v1 used 0.05 m for both, right at the
  loaded post-look std (dev p90 0.048 m), so looks re-triggered on the spot;
* at most one refix in a row, then drive on (the lost / not-initialized stop
  rules still count every unfixed look);
* travel-based looks every 0.5 m of estimated travel (v1: 0.35 m), with the
  stop-lag motion model (calibration_loop_v2.json).

Unloaded behaviour is exactly v1. Nothing here imports the simulator.
"""
from __future__ import annotations

import math

from harness.owncam_drive import OwnCamDriver

SCHEMA = 'ugrp.owncam_drive.v2'
LOADED_FIX_STD_XY_M = .06
LOADED_UNCERTAIN_STD_XY_M = .07
LOADED_UNCERTAIN_STD_YAW_RAD = math.radians(3.)
LOADED_UNCERTAIN_MIN_TRAVEL_M = .10
LOADED_LOOK_EVERY_M_V2 = .5
LOADED_MAX_REFIX_IN_A_ROW = 1


class OwnCamDriverV2(OwnCamDriver):
    version = 'v2'

    def _uncertain(self, est) -> bool:
        if not self.loaded:
            return super()._uncertain(est)
        high = est['std_xy_m'] > LOADED_UNCERTAIN_STD_XY_M or est['std_yaw_rad'] > LOADED_UNCERTAIN_STD_YAW_RAD
        return high and (self.last_look_xy is None or self._since_look_m(est) >= LOADED_UNCERTAIN_MIN_TRAVEL_M)

    def _travel_look_m(self) -> float:
        return LOADED_LOOK_EVERY_M_V2 if self.loaded else super()._travel_look_m()

    def _fix_std_xy_m(self) -> float:
        return LOADED_FIX_STD_XY_M if self.loaded else super()._fix_std_xy_m()

    def _should_refix(self, fixed) -> bool:
        if not self.loaded:
            return super()._should_refix(fixed)
        return not fixed and self.looks_without_fix <= LOADED_MAX_REFIX_IN_A_ROW
