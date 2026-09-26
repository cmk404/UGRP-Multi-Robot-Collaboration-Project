"""M1 own-camera delivery with the observation memory (memory ON condition).

The student is ``harness.m1_owncam_delivery.M1OwnCamDelivery`` (frozen M1
source, not modified): the same inputs, pose source, wrist skill, planner,
gates and pose limits. This subclass adds one ``harness.owncam_memory``
memory per robot, fed with the robot's own frames and commands only:

* every own frame feeds the memory (landmark observations, expected view, own-RGB
  box tracks, free / blocked floor). Landmark observations come from the INTERIM tag
  provider (``harness.owncam_landmark_tags``: the pose source's tag detections,
  re-labelled as observations of static-map landmarks; results are "interim, tag
  provider"); the memory itself is landmark-agnostic (``harness.owncam_landmarks``);
* legs use the memory look policy (``harness.owncam_drive_mem``);
* gate looks: the first look of a gate is a planned short look stopped at the
  gate's target, a repeated look of the same gate is full; the initial and the
  post-manipulation looks stay full; the pre-place look is skipped when the
  release gate already holds;
* search: detections are remembered across viewpoints and the initial look
  (the baseline forgets them per viewpoint), a confirmed target seen while
  driving ends the leg, viewpoints and pans whose floor footprint is already
  known are skipped, and skipped viewpoints are revisited with full sweeps
  before ``SEARCH_NOT_FOUND``;
* re-verification: before the grasp the remembered target must be confirmed,
  fresh and not absent (else one re-look, then back to the search); before
  the release the destination slot must not be remembered as occupied.

v2 (prereg amendment A1-A3, 2026-09-26; v1 = commit ad78ef2, dev-a1 only): landmark-
agnostic memory with the interim tag provider, the A2 box re-projection, and short looks
that stop early only after a look-posture fix in the current dwell (A3).

Nothing here imports the simulator.
"""
from __future__ import annotations

import math

from harness.m1_owncam_delivery import (CLOSER_VIEW_STANDOFF_M, LIMITS, MAX_GATE_LOOKS, SEARCH_PANS, SEARCH_VIEW_X_M,
                                        M1OwnCamDelivery, _LegDriver)
from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SEARCH_POSE, SETTLE_S, WIDE_LOOK_PANS
from harness.owncam_drive_mem import MemoryLookPolicy
from harness.owncam_landmark_tags import INTERIM_LABEL, TagLandmarkProvider
from harness.owncam_memory import OwnCamMemory
from harness.owncam_pose_source import check_limits

SCHEMA = 'ugrp.m1_owncam_memory.v2'
# Short-look early-stop targets per gate (own PF sigma xy m, yaw rad): 0.8 x the gate limit.
GATE_TARGETS = {'release': (.04, .028), 'preplace': (.04, .028), 'look_back': (.028, .028),
                'nav_loaded': (.045, math.radians(1.3)), 'nav_unloaded': (.035, math.radians(1.3))}
FULL_GATE_REASONS = ('post_manipulation', 'not_initialized')
VIEWPOINT_MIN_UNKNOWN_FRAC = .15
REVERIFY_PANS = (1500,)
MAX_ABANDON = 1
SEARCH_CANDIDATE_PANS = tuple(dict.fromkeys(SEARCH_PANS))


class _RecordingDetector:
    """The pose source's own TagDetector; keeps the last frame's detections for the memory."""

    def __init__(self, inner):
        self.inner = inner
        self.last: list[dict] = []

    def detect(self, image):
        self.last = self.inner.detect(image)
        return self.last

    def __getattr__(self, name):
        return getattr(self.inner, name)


class _LegDriverMem(MemoryLookPolicy, _LegDriver):
    """The M1 leg driver (loop v2 plant on the shared localizer) with the memory look policy."""

    def __init__(self, memory, shared_loc, *args, **kwargs):
        super().__init__(shared_loc, *args, **kwargs)
        self._init_memory_policy(memory)


class M1OwnCamDeliveryMem(M1OwnCamDelivery):
    def __init__(self, static_map, params, **kwargs):
        super().__init__(static_map, params, **kwargs)
        self.memory = OwnCamMemory(static_map, params, robot_id=self.robot_id,
                                   provider=TagLandmarkProvider(static_map, params), on_event=self._memory_event)
        self.pose.detector = _RecordingDetector(self.pose.detector)
        self.target_track_id: str | None = None
        self.closer_done_tracks: list[str] = []
        self.skipped_viewpoints: list[tuple[float, float]] = []
        self.revisit = False
        self.reverify_done = False
        self.slot_checked = False
        self.slot_record: dict | None = None
        self.gate_modes: list[dict] = []
        self.abandoned = 0
        self._held = False

    # ---------------------------------------------------------------- memory plumbing
    def _memory_event(self, kind, row):
        self.events.append({'t': row['t'], 'event': 'memory', 'phase': self.phase, **row})

    def on_frame(self, now, obs, rgb):
        report = super().on_frame(now, obs, rgb)
        servo = {int(k): int(v) for k, v in obs['actuator_state']['servo_pulses'].items()}
        loaded = report.load_state == 'loaded'
        out = self.memory.observe_frame(now, frame_id=int(obs['frame_id']), image=obs['image'], servo=servo,
                                        report=report, arm_settled_s=now - self.pose.loc.last_servo_cmd_t,
                                        loaded=loaded, provider_inputs={'tag_detections': self.pose.detector.last})
        if loaded and not self._held:
            self.memory.mark_held(now)
        elif self._held and not loaded:
            self.memory.mark_released(now)
        self._held = loaded
        for row in out.get('boxes', []):
            if row['kind'] == self.box_kind:
                self.cyan.append({'t': round(now, 3), **row})
        return report

    def _search_detect(self, obs, report):
        return None          # the memory runs the same own-RGB detection on every settled unloaded frame

    def _keepouts(self):
        return self.memory.keepouts(exclude=[self.target_track_id] if self.target_track_id else [])

    def _start_leg(self, goal, *, loaded):
        self.leg = _LegDriverMem(self.memory, self.pose.loc, self.map, self.params, loaded=loaded, goal_xy=goal,
                                 door_xy=self.door_xy, keepouts=self._keepouts(), initial_servo=dict(self.servo),
                                 seed=self.seed)
        if loaded:
            self.leg.drive_pose = dict(CARRY_POSTURE)
        self.leg_goal = tuple(goal)

    @staticmethod
    def _hold():
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    # ---------------------------------------------------------------- search
    def _init(self, now):
        rep = self.pose.report(now)
        if rep.initialized and rep.std_xy_m <= LIMITS['nav_unloaded'].max_std_xy_m:
            self.viewpoints = [(SEARCH_VIEW_X_M, y) for y in
                               sorted(self.search_rows_y, key=lambda y: (abs(y - rep.y_m), y))]
            self._event(now, 'initialized', report=rep.as_dict(), viewpoints=self.viewpoints)
            self.view_index = -1
            return self._search_decide(now)          # the initial look may already have seen the target
        return super()._init(now)

    def _target_from_memory(self, now, why):
        target = self.memory.best_target(self.box_kind, now)
        if target is None:
            return None
        return self._approach_target(now, target, why)

    def _search_leg(self, now):
        if self.leg is not None and self.leg.state == 'drive':
            decision = self._target_from_memory(now, 'seen_while_driving')
            if decision is not None:
                return decision
        cmds, outcome = self._drive_leg(now)
        if outcome is None:
            return {'mode': 'tick', 'commands': cmds}
        if outcome != 'arrived':
            self.outcome = 'SEARCH_LEG_' + outcome
            return {'mode': 'done', 'outcome': self.outcome}
        self.phase = 'search_sweep'
        rep = self.pose.report(now)
        if self.revisit or not rep.initialized:
            pans, why = list(SEARCH_PANS), 'revisit_full' if self.revisit else 'not_initialized'
            coverage = None
        else:
            plan = self.memory.plan_search_pans((rep.x_m, rep.y_m, rep.yaw_rad), SEARCH_CANDIDATE_PANS)
            pans, why, coverage = (plan['pans'] or [1500]), 'memory_coverage', plan['coverage']
        self._event(now, 'search_pans', pans=pans, why=why,
                    skipped=[p for p in SEARCH_CANDIDATE_PANS if p not in pans], coverage=coverage)
        self._start_sweep(now, 'search', SEARCH_POSE, pans, SEARCH_POSE, 'viewpoint')
        return self._hold()

    def _after_sweep(self, now):
        if self.phase == 'search_sweep':
            return self._search_decide(now)
        if self.phase == 'reverify_sweep':
            rv = self.memory.reverify(self.target_track_id, now)
            return self._start_skill(now) if rv['status'] == 'fresh' else self._abandon_target(now, rv['status'])
        return super()._after_sweep(now)

    def _viewpoint_useful(self, vp) -> dict:
        best = {'near_unknown_frac': 0., 'far_unknown_frac': 0.}
        for pan in SEARCH_CANDIDATE_PANS:
            cov = self.memory.view_coverage((vp[0], vp[1], 0.), {**SEARCH_POSE, 6: pan})
            for k in best:
                best[k] = max(best[k], cov[k])
        best['useful'] = max(best.values()) >= VIEWPOINT_MIN_UNKNOWN_FRAC
        return best

    def _search_decide(self, now):
        target = self.memory.best_target(self.box_kind, now)
        if target is not None:
            return self._approach_target(now, target, 'own_rgb_search_memory')
        far = self.memory.best_far(self.box_kind, now, exclude=self.closer_done_tracks)
        if far is not None:
            self.closer_done_tracks.append(far.track_id)
            self._event(now, 'search_closer_view', track=far.record(now))
            self.phase = 'search_leg'
            self._start_leg((float(far.x[0]) - CLOSER_VIEW_STANDOFF_M, float(far.x[1])), loaded=False)
            return self._hold()
        while True:
            self.view_index += 1
            if self.view_index >= len(self.viewpoints):
                if self.skipped_viewpoints and not self.revisit:
                    self.revisit = True
                    self._event(now, 'search_revisit', viewpoints=list(self.skipped_viewpoints))
                    self.viewpoints = list(self.viewpoints) + list(self.skipped_viewpoints)
                    self.skipped_viewpoints = []
                    self.view_index -= 1
                    continue
                self.outcome = 'SEARCH_NOT_FOUND'
                return {'mode': 'done', 'outcome': self.outcome}
            vp = self.viewpoints[self.view_index]
            if self.revisit:
                break
            use = self._viewpoint_useful(vp)
            if use['useful']:
                break
            self.skipped_viewpoints.append(vp)
            self._event(now, 'viewpoint_skipped', viewpoint=list(vp), coverage=use)
        self.phase = 'search_leg'
        self._start_leg(self.viewpoints[self.view_index], loaded=False)
        return self._hold()

    def _approach_target(self, now, track, why):
        self.leg = None
        self.target_track_id = track.track_id
        self.target_xy = (float(track.x[0]), float(track.x[1]))
        self.pickup_source = 'own_rgb_search'
        self.memory.claim(track.track_id, now)
        self._event(now, 'memory_target', why=why, track=track.record(now))
        goal = self._approach_goal(self.target_xy)
        if goal is None:
            self.outcome = 'SKILL_BAY_APPROACH_BLOCKED'
            return {'mode': 'done', 'outcome': self.outcome}
        self.phase = 'approach_leg'
        self._start_leg(goal, loaded=False)
        return self._hold()

    def _abandon_target(self, now, status):
        tr = self.memory.track(self.target_track_id)
        if self.abandoned >= MAX_ABANDON:
            # A second failed re-verification: the skill's own approach detection decides
            # (never loop between the memory and the search).
            self._event(now, 'reverify_failed_skill_decides', status=status,
                        track=None if tr is None else tr.record(now))
            return self._start_skill(now)
        self.abandoned += 1
        if tr is not None and tr.state in ('claimed', 'confirmed'):
            tr.state = 'absent'
        self.memory.claimed = None
        self._event(now, 'target_abandoned', status=status, track=None if tr is None else tr.record(now))
        self.target_track_id, self.target_xy = None, None
        self.skill, self.approach_choice, self.order_record = None, None, None
        self.reverify_done = False
        self.leg = None
        return self._search_decide(now)

    def _approach_leg(self, now):
        cmds, outcome = self._drive_leg(now)
        if outcome is None:
            return {'mode': 'tick', 'commands': cmds}
        if outcome != 'arrived':
            self.outcome = 'APPROACH_LEG_' + outcome
            return {'mode': 'done', 'outcome': self.outcome}
        rv = self.memory.reverify(self.target_track_id, now)
        if rv['status'] == 'fresh':
            return self._start_skill(now)
        if not self.reverify_done and rv['status'] in ('stale', 'absent'):
            self.reverify_done = True
            self.leg = None
            self.phase = 'reverify_sweep'
            self._start_sweep(now, 'search', SEARCH_POSE, list(REVERIFY_PANS), SEARCH_POSE, 'reverify')
            return self._hold()
        return self._abandon_target(now, rv['status'])

    def _start_skill(self, now):
        if self.skill is None:
            self.skill = self.skill_factory(self._make_order())
        self.leg = None
        self.phase = 'skill'
        self._event(now, 'skill_start', pickup_xy=[round(v, 4) for v in self.target_xy],
                    pickup_source=self.pickup_source, target_track=self.target_track_id)
        return self._hold()

    # ---------------------------------------------------------------- place
    def _slot_half(self):
        for slots in self.map.get('zone_slots', {}).values():
            for s in slots:
                if s['slot_id'] == self.slot_id:
                    return tuple(s['half_extents_m'])
        return (.06, .06)

    def _skill(self, now):
        sk = self.skill
        if sk is not None and sk.phase == 'pre_release' and not self.slot_checked:
            self.slot_checked = True
            self.slot_record = self.memory.slot_state(now, self.slot_xy, self._slot_half(),
                                                      exclude=[self.target_track_id] if self.target_track_id else [])
            if self.slot_record['state'] == 'occupied':
                self.outcome = 'SLOT_OCCUPIED_IN_MEMORY'
                return {'mode': 'done', 'outcome': self.outcome}
        return super()._skill(now)

    # ---------------------------------------------------------------- looks
    def _gate_look(self, now, reason, obs, *, loaded):
        pose = {int(k): int(v) for k, v in obs['actuator_state']['servo_pulses'].items()}
        restore = {k: v for k, v in pose.items() if k in (1, 3, 4, 5, 6)}
        look_pose = dict(LOOK_P20)
        if loaded:
            look_pose[1] = restore.get(1, 1500)             # keep the grip as issued
        gate = reason.split(':')[1] if reason.startswith('gate:') else reason
        rep = self.pose.report(now)
        est = self.pose.loc.estimate()
        since_look = None if self.last_look_t is None else now - self.last_look_t
        if reason == 'preplace' and not check_limits(rep, now, LIMITS['release'], since_look_s=since_look):
            self.gate_modes.append({'t': round(now, 3), 'reason': reason, 'mode': 'skipped'})
            self._event(now, 'look_skipped', reason=reason, report=rep.as_dict())
            return self._hold()
        repeat = reason.startswith('gate:') and self.gate_looks >= 2
        plan = None
        if reason not in FULL_GATE_REASONS and not repeat:
            plan = self.memory.plan_look(est, loaded=bool(loaded or rep.load_state == 'loaded'), now=now,
                                         reason=reason, start_pan=int(restore.get(6, 1500)))
        pans = list(plan['pans']) if plan and plan['pans'] else list(WIDE_LOOK_PANS)
        mode = 'short' if plan and plan['pans'] else 'full'
        self.gate_modes.append({'t': round(now, 3), 'reason': reason, 'mode': mode, 'pans': pans})
        self._start_sweep(now, 'look', look_pose, pans, restore, reason)
        self.sweep['mode'] = mode
        self.sweep['short_target'] = GATE_TARGETS.get(gate, GATE_TARGETS['nav_loaded' if loaded else 'nav_unloaded'])
        self.memory.reset_view_checks()
        self.reanchor_needed = bool(loaded and self.skill is not None and self.skill.box.held)
        return self._hold()

    def _tick_sweep(self, now):
        s = self.sweep
        if s.get('mode') == 'short' and s['stage'] == 'pan' and s['queue'] and not self._arm_steps({6: s['target']}) \
                and now - s['since'] >= SETTLE_S:
            rep = self.pose.report(now)
            txy, tyaw = s['short_target']
            if rep.initialized and rep.std_xy_m <= txy and rep.std_yaw_rad <= tyaw and self.memory.look_fix_since(s['since']):
                dropped = list(s['queue'])
                s['queue'].clear()
                self._event(now, 'short_look_early_stop', reason=s['reason'], dropped_pans=dropped,
                            report=rep.as_dict())
        return super()._tick_sweep(now)

    # ---------------------------------------------------------------- output
    def summary(self) -> dict:
        out = super().summary()
        now = self.memory.t if self.memory.t is not None else 0.
        out['base_schema'], out['schema'] = out['schema'], SCHEMA
        out['landmark_provider'] = {**self.memory.provider.describe(), 'result_label': INTERIM_LABEL}
        out['memory'] = self.memory.snapshot(now)
        out['memory_grid'] = self.memory.grid_record()
        out['target_track_id'] = self.target_track_id
        out['slot_check'] = self.slot_record
        out['gate_look_modes'] = self.gate_modes
        out['skipped_viewpoints'] = [list(v) for v in self.skipped_viewpoints]
        out['search_revisit'] = self.revisit
        out['max_gate_looks'] = MAX_GATE_LOOKS
        return out
