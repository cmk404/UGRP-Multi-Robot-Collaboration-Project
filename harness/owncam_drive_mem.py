"""Stop-and-look policy with the own-camera observation memory (memory ON condition).

Mixin for ``harness.owncam_drive_v2.OwnCamDriverV2`` and its subclasses (the M1
leg driver). Everything else of the student is unchanged: inputs, planner,
pursuit, postures, the v2 sigma thresholds (``_uncertain``, ``_fix_std_xy_m``)
and the lost / not-initialized stop rules. Only WHEN and HOW the driver looks
changes, from ``harness.owncam_memory.OwnCamMemory``:

* ``uncertain`` (same thresholds as OFF) -> a short look: the LOOK_P20 pans the
  memory planner picks (1-3, Fisher information per second of arm motion,
  weighted by the remembered detection rate), stopped as soon as the own
  estimate reaches the short-look target; if it is still not acceptable after
  the planned pans, one full look at once;
* door checkpoints and the arrival check: skipped when the estimate already
  meets the door / fix requirement, otherwise a short look;
* ``expected_view_missing`` (tags the map places well inside the view were not
  detected in 3 settled frames; unloaded: no tag for 3 s although the map
  predicted some) -> a full look;
* no travel-based looks (loaded) and no plain no-tag looks (unloaded).

Nothing here imports the simulator.
"""
from __future__ import annotations

import math

from harness.owncam_drive import DOOR_CHECKPOINTS_M, LOOK_P20, SETTLE_S, WIDE_LOOK_PANS

SCHEMA = 'ugrp.owncam_drive_mem.v1'
# Early-stop target of a short look (own PF sigma): about the p90 a full look reached in the
# M1 test cohort (loaded dwell std_xy p90 0.041-0.048 m, post-look yaw std p90 1.36 deg).
SHORT_TARGET = {True: (.045, math.radians(1.3)), False: (.035, math.radians(1.3))}
SHORT_ACCEPT_YAW_RAD = math.radians(2.)      # after the planned pans (with the v2 fix std_xy)
DOOR_OK = (.04, math.radians(2.))            # skip a door checkpoint look when the estimate is this good
ARRIVAL_OK_YAW_RAD = math.radians(2.)
FULL_REASONS = ('not_initialized', 'expected_view_missing', 'refix')


class MemoryLookPolicy:
    """Look-policy hooks of ``OwnCamDriver`` driven by an ``OwnCamMemory``."""

    def _init_memory_policy(self, memory) -> None:
        self.memory = memory
        self.look_mode: str | None = None
        self.look_counts = {'short': 0, 'full': 0, 'skipped': 0, 'early_stop': 0, 'escalated': 0}

    def _mem_event(self, now, kind, **detail) -> None:
        self._event(now, kind, **detail)
        self.memory.event(now, 'driver_' + kind, loaded=bool(self.loaded), **detail)

    @staticmethod
    def _sig(est) -> dict:
        return {'std_xy_m': None if not est.get('initialized') else round(est['std_xy_m'], 4),
                'std_yaw_rad': None if not est.get('initialized') else round(est['std_yaw_rad'], 5)}

    def _needs_look(self, est, now):
        if not est.get('initialized'):
            return 'not_initialized'
        if self._uncertain(est):
            return 'uncertain'
        if self.memory.view_missing() or (not self.loaded and self.memory.no_tag_while_expected(now)):
            return 'expected_view_missing'
        if est['x'] < self.door[0]:
            d = math.hypot(self.door[0] - est['x'], self.door[1] - est['y'])
            for cp in DOOR_CHECKPOINTS_M:
                if cp not in self.checkpoints_done and d <= cp:
                    self.checkpoints_done.add(cp)
                    if est['std_xy_m'] > DOOR_OK[0] or est['std_yaw_rad'] > DOOR_OK[1]:
                        return f'door_checkpoint_{cp}'
                    self.look_counts['skipped'] += 1
                    self._mem_event(now, 'look_skipped', reason=f'door_checkpoint_{cp}', **self._sig(est))
        return None

    def _start_look(self, now, reason):
        self.loc.predict_to(now)
        est = self.loc.estimate()
        if reason == 'arrival_check':
            if (est.get('initialized') and est['std_xy_m'] <= self._fix_std_xy_m()
                    and est['std_yaw_rad'] <= ARRIVAL_OK_YAW_RAD and not self.memory.view_missing()):
                self.look_counts['skipped'] += 1
                self._mem_event(now, 'look_skipped', reason='arrival_check', **self._sig(est))
                return [{'kind': 'hold'}]            # state stays 'drive': the next tick declares arrival
        plan = None
        if reason not in FULL_REASONS:
            plan = self.memory.plan_look(est, loaded=self.loaded, now=now, reason=reason,
                                         start_pan=int(self.servo.get(6, 1500)))
        pans = list(plan['pans']) if plan and plan['pans'] else list(WIDE_LOOK_PANS)
        self.look_mode = 'short' if plan and plan['pans'] else 'full'
        self.look_counts[self.look_mode] += 1
        self.looks += 1
        self.look_reason = reason
        self.look_queue = pans
        self.arm_target = dict(LOOK_P20)
        self.memory.reset_view_checks()
        self._set('look_arm', now, reason=reason, look=self.looks, mode=self.look_mode, pans=pans)
        self.memory.event(now, 'driver_look', reason=reason, mode=self.look_mode, pans=pans,
                          loaded=bool(self.loaded), **self._sig(est))
        return [{'kind': 'hold'}]

    def _should_refix(self, fixed):
        if self.look_mode == 'short':
            est = self.loc.estimate()
            ok = (bool(est.get('initialized')) and est['std_xy_m'] <= self._fix_std_xy_m()
                  and est['std_yaw_rad'] <= SHORT_ACCEPT_YAW_RAD)
            self.memory.reset_view_checks()
            if not ok:
                self.look_counts['escalated'] += 1
                self._mem_event(self.loc.t, 'short_look_escalated', **self._sig(est))
            return not ok
        self.memory.reset_view_checks()
        return super()._should_refix(fixed)

    def tick(self, now):
        if self.state == 'look_pan' and self.look_mode == 'short' and self.look_queue:
            arrived = all(self.servo.get(s, t) == t for s, t in self.arm_target.items())
            if arrived and self.state_since is not None and now - self.state_since >= SETTLE_S:
                self.loc.predict_to(now)
                est = self.loc.estimate()
                txy, tyaw = SHORT_TARGET[bool(self.loaded)]
                if est.get('initialized') and est['std_xy_m'] <= txy and est['std_yaw_rad'] <= tyaw:
                    dropped = list(self.look_queue)
                    self.look_queue.clear()
                    self.look_counts['early_stop'] += 1
                    self._mem_event(now, 'short_look_early_stop', dropped_pans=dropped, **self._sig(est))
        return super().tick(now)
