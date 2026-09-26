"""Package F: own-camera per-robot executor API for the Korean-dialogue zone study.

The study layer (LLM actors, scheduler, scripted no-LLM fixtures) talks to one
``ZoneOwnExecutor`` per robot through a small job API::

    deliver(item_ref, zone_slot)   # one order line -> a zone slot (M1 delivery chain)
    goto(target)                   # map waypoint [x, y], zone 'A', zone slot 'A2', pickup slot 'P1-2', door 'door_1'
    look_around()                  # wide own-camera look sweep (re-localise, look for blockages)
    hold(sim_s) / wait(sim_s)      # stop and hold the last safe command for sim_s SIM seconds
    abort()                        # cancel the current job and hold
    status()                       # own executor state + own-camera judgments only

and receives the executor's events (``drain_events``): ``job_started``,
``job_done`` (``own_camera_confirmed`` | ``unconfirmed``), ``job_failed``
(reason), ``blockage_seen`` and ``pose_uncertain``. Every accepted job ends with exactly
one terminal event, whatever ends it (done, failure, abort, local timeout, episode end,
controller exception); a stopped robot refuses every new job.

Inputs (and nothing else): the robot's own ``robot_cam`` observations (JPEG + own issued
PWM), its own issued commands, the static tagged map, fixed calibrations (camera, motion,
own body), static layout keep-outs and the scenario order sheet (kind, count, destination
zone, coarse pickup-bay slot; never a coordinate). No simulator import, no world handle,
no peer handle: the physics owner (``harness.zone_own_team_host.OwnCamTeamHost``) feeds
each executor its own frames and its own command log only.

Internally (read-only reuse, nothing forked):

* pose: ``harness.owncam_pose_source.OwnCamPoseSource`` - ONE localizer per robot;
* deliver: ``harness.m1_owncam_delivery.M1OwnCamDelivery`` (PR #201) with skill
  ``harness.wrist_zone_skill_v9`` (PR #181) in ``mode='m1'``;
* driving: ``harness.zone_own_guards.GuardedDriver`` (``OwnCamDriverV2`` + uncertainty
  gate, look-sweep collision guard, progress monitor) for goto and every M1 leg;
* judgments: ``harness.zone_own_perception`` (PR #193) ``judge_route_blockage`` and
  ``judge_holding_item`` on own frames taken in the agreed postures.

M1 contract (``mode='m1'``): every pose source must be the own-camera estimator, every
observation is validated (own robot_cam, own robot id, fresh, hash) and an injected pose
source that is not own-camera is refused. ``mode='diagnostic'`` never counts as M1.
"""
from __future__ import annotations

import copy
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any

import numpy as np

from harness import m1_contract, m1_owncam_contract
from harness import zone_own_guards as guards
from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SETTLE_S, WIDE_LOOK_PANS
from harness.owncam_pose_source import OwnCamPoseSource, PoseReport
from harness.zone_own_contract import (API_TO_ACTION_KIND, EVENT_TO_TRIGGER, EVENTS, PICKUP_VIEW_X_M,  # noqa: F401
                                       ExecutorContractError, action_record, finite_number, lane_viewpoints,
                                       pickup_slot_of, pickup_slots, scheduler_trigger, validate_order_sheet,
                                       zone_slot)
from harness.zone_own_deliver import _DeliverController
from harness.zone_own_status import (BLOCKAGE_CONSECUTIVE, GRIPPER_OPEN_MIN_PWM, JUDGE_PERIOD_S,  # noqa: F401
                                     STATUS_SCHEMA, UNCERTAINTY_LEVELS, OwnStatusMixin, uncertainty_level)
from harness.zone_study_contract import ROBOTS

SCHEMA = 'ugrp.zone_own_executor.v2'
EVENT_SCHEMA = 'ugrp.zone_own_executor_event.v1'
MODES = ('m1', 'diagnostic')
JOB_KINDS = ('deliver', 'goto', 'look_around', 'hold')
CONFIRMATIONS = ('own_camera_confirmed', 'unconfirmed')
TICK_S = .1
ZONE_APPROACH_M = .25           # goto('A'): stop this far west of the zone paint
SLOT_STANDOFF_M = .40           # goto('A2'): stop this far west of the slot centre
DOOR_SIDE_M = .45               # goto('door_1'): the far side of the door, this far from its centre
DEFAULT_JOB_SIM_LIMIT_S = 720.  # = scripts/run_m1_owncam.SIM_LIMIT_S
MAX_HOLD_S = 3600.
TERMINAL_STOP = 'ROBOT_STOPPED'
REASON_TOKEN_MAX = 64
UNCERTAIN_REASONS = ('POSE_UNCERTAIN', 'NOT_INITIALIZED')
UNCERTAIN_SUFFIXES = ('lost', 'not_initialized', 'pose_uncertain', 'arrival_unconfirmed')


class _Job:
    def __init__(self, job_id: str, kind: str, args: Mapping, now: float, limit_s: float):
        self.job_id, self.kind, self.args = job_id, kind, dict(args)
        self.started_at, self.deadline = float(now), float(now) + float(limit_s)
        self.phase = 'start'
        self.ctl: _DeliverController | None = None
        self.driver: guards.GuardedDriver | None = None
        self.sweep: dict | None = None
        self.hold_until: float | None = None


class ZoneOwnExecutor(OwnStatusMixin):
    """One robot's own-camera executor. Holds no simulator, no peer and no world reference."""

    def __init__(self, robot_id: str, static_map: Mapping, params: Mapping, order_sheet: Mapping, *,
                 skill_factory: Callable[..., Any], pose_estimate_cls, search_rows_y: Sequence[float],
                 mode: str = 'm1', seed: int = 0, pose_source: OwnCamPoseSource | None = None,
                 job_sim_limit_s: float = DEFAULT_JOB_SIM_LIMIT_S, judgments: bool = True,
                 static_keepouts: Sequence[Mapping] = ()):
        m1_contract.check_mode(mode)
        if robot_id not in ROBOTS:
            raise ValueError(f'robot_id must be one of {ROBOTS}')
        if not finite_number(job_sim_limit_s) or job_sim_limit_s <= 0:
            raise ValueError('job_sim_limit_s must be a positive finite number')
        self.robot_id, self.mode, self.seed = robot_id, mode, int(seed)
        # Own copies: nothing mutable is shared with another robot's executor.
        self.map = copy.deepcopy(dict(static_map))
        self.params = copy.deepcopy(dict(params))
        self.orders = validate_order_sheet(order_sheet, self.map)
        self.slots = pickup_slots(self.map)
        self.skill_factory = skill_factory
        self.pose_estimate_cls = pose_estimate_cls
        self.search_rows_y = tuple(float(y) for y in search_rows_y)
        self.job_sim_limit_s = float(job_sim_limit_s)
        self.judgments = bool(judgments)
        self.static_keepouts = []
        for k in static_keepouts:
            if not str(k.get('source', '')).startswith('static_layout') or set(k) - {'id', 'center_m', 'radius_m', 'source'}:
                raise ExecutorContractError('static keep-outs must be static layout discs (id, center_m, radius_m, source)')
            self.static_keepouts.append(copy.deepcopy(dict(k)))
        if pose_source is None:
            pose_source = OwnCamPoseSource(self.map, self.params, seed=self.seed)
        elif mode == 'm1':
            self._require_owncam(getattr(pose_source, 'source', None), 'injected pose source')
            if not isinstance(pose_source, OwnCamPoseSource):
                raise ExecutorContractError('M1 mode takes only the own-camera OwnCamPoseSource')
        if mode == 'm1':
            self._require_owncam(pose_source.source, 'pose source')
        self.pose = pose_source
        door = next(p for p in self.map['passages'] if p['kind'] == 'door')
        self.door_id = door['id']
        self.door_xy = (float(door['center_m'][0]), float(door['center_m'][1]))
        self.gate = guards.UncertaintyGate()
        self.guard = guards.SweepGuard(self.map)
        self.streak = guards.BlockageStreak(BLOCKAGE_CONSECUTIVE)
        self.servo: dict[int, int] = {}
        self.last_obs: Mapping | None = None
        self.last_frame_id: int | None = None
        self.last_report: PoseReport | None = None
        self.now = 0.
        self.job: _Job | None = None
        self.jobs_done: list[dict] = []
        self.events: list[dict] = []
        self._outbox: list[dict] = []
        self.api_log: list[dict] = []
        self.judgment_log: list[dict] = []
        self.pose_sources_seen: set[str] = set()
        self.cameras_seen: set[str] = set()
        self.stopped: dict | None = None
        self._counter = 0
        self._local_state = 'queue_empty'
        self._holding_after = {'answer': 'no', 'source': 'episode_start_gripper_never_closed'}
        self._delivered_per_zone: dict[str, int] = {}
        self._last_blockage: dict | None = None
        self._last_holding_check: dict | None = None
        self._last_judge_t = -1e9
        self._level = 'unknown'
        self._rejected_frames = 0
        self._summaries: list[dict] = []
        self._pending_hold = False

    # ---------------------------------------------------------------- contract helpers
    def _require_owncam(self, label, where):
        try:
            m1_owncam_contract.require_m1_source(label)
            m1_contract.require_m1_pose_source(label, where)
        except (m1_owncam_contract.M1ContractError, m1_contract.ContractViolation) as exc:
            raise ExecutorContractError(f'{self.robot_id}: {exc}') from exc
        return label

    def _next_id(self, prefix):
        self._counter += 1
        return f'{self.robot_id}-{prefix}-{self._counter:03d}'

    def _emit(self, now, event, **detail):
        if event not in EVENTS:
            raise ValueError(event)
        row = {'schema': EVENT_SCHEMA, 'robot_id': self.robot_id, 'event': event, 'sim_s': round(float(now), 3),
               'job_id': self.job.job_id if self.job else detail.pop('job_id', None),
               'job_kind': self.job.kind if self.job else detail.pop('job_kind', None), 'detail': detail}
        row['scheduler_trigger'] = scheduler_trigger(row)
        self.events.append(row)
        self._outbox.append(row)

    def drain_events(self) -> list[dict]:
        out, self._outbox = self._outbox, []
        return out

    # ---------------------------------------------------------------- inputs (own only)
    def on_command(self, row: Mapping) -> None:
        """One own issued command (time ordered, as logged at this robot's port)."""
        job = self.job
        if job is not None and job.ctl is not None:
            job.ctl.on_command(row)            # forwards to the shared pose source exactly once
        else:
            self.pose.on_command(row)
        if job is not None and job.driver is not None:
            job.driver.on_command(row)         # servo bookkeeping only (shared localizer)
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])

    def on_frame(self, now: float, obs: Mapping, rgb: np.ndarray) -> PoseReport:
        """One own ``robot_cam`` frame. Another robot's frame, a stale frame or another camera is refused."""
        if not isinstance(obs, Mapping) or obs.get('robot_id') != self.robot_id:
            self._rejected_frames += 1
            raise ExecutorContractError(f"{self.robot_id} refuses an observation of "
                                        f"{obs.get('robot_id') if isinstance(obs, Mapping) else type(obs).__name__!r}")
        m1_owncam_contract.validate_observation(obs, robot_id=self.robot_id, previous_frame_id=self.last_frame_id,
                                                now=now)
        self.cameras_seen.add(str(obs['camera']))
        self.last_frame_id = int(obs['frame_id'])
        self.last_obs, self.now = obs, float(now)
        job = self.job
        report = job.ctl.on_frame(now, obs, rgb) if job is not None and job.ctl is not None else \
            self.pose.on_frame(now, rgb)
        if self.mode == 'm1':
            self._require_owncam(report.source, 'frame report')
        self.pose_sources_seen.add(report.source)
        self.last_report = report
        self._update_gate(now, report)
        if self.judgments and now - self._last_judge_t >= JUDGE_PERIOD_S - 1e-9:
            if self._judge(now, obs, rgb, report):
                self._last_judge_t = now
        return report

    def _update_gate(self, now, report):
        self._level = uncertainty_level(report)
        change = self.gate.update(now, report.initialized, report.std_xy_m, report.std_yaw_rad)
        if change == 'entered':
            self._emit(now, 'pose_uncertain', level=self._level, gate_profile=self.gate.profile.name,
                       std_xy_m=round(report.std_xy_m, 4) if finite_number(report.std_xy_m) else None, ends_job=False)

    # ---------------------------------------------------------------- API
    def _ack(self, api, arguments, accepted, reason=None, job=None):
        self._local_state = 'command_rejected' if not accepted else (
            'hold_requested' if api in ('hold', 'abort') else 'command_issued')
        ack = {'robot_id': self.robot_id, 'api': api, 'action_id': self._next_id('act'), 'sim_s': round(self.now, 3),
               'arguments': arguments, 'accepted': bool(accepted), 'rejected_reason': reason,
               'job_id': job.job_id if job else None, 'local_state': self._local_state}
        self.api_log.append(ack)
        return ack

    def _start(self, api, kind, arguments, **job_args):
        if self.stopped is not None:
            return self._ack(api, arguments, False, TERMINAL_STOP)
        if self.job is not None:
            return self._ack(api, arguments, False, f'BUSY:{self.job.kind}:{self.job.job_id}')
        job = _Job(self._next_id('job'), kind, {**arguments, **job_args}, self.now, self.job_sim_limit_s)
        self.job = job
        self._emit(self.now, 'job_started', arguments=arguments)
        return self._ack(api, arguments, True, job=job)

    @staticmethod
    def _token(value) -> str:
        return value if isinstance(value, str) else repr(value)[:REASON_TOKEN_MAX]

    def deliver(self, item_ref, zone_slot_id) -> dict:
        """Deliver the order line ``item_ref`` to a zone slot ('A2') or a zone ('A': next own slot)."""
        arguments = {'order_id': self._token(item_ref), 'target_ref': self._token(zone_slot_id)}
        if self.stopped is not None:
            return self._ack('deliver', arguments, False, TERMINAL_STOP)
        order = self.orders.get(item_ref) if isinstance(item_ref, str) else None
        if order is None:
            return self._ack('deliver', arguments, False, 'UNKNOWN_ORDER')
        if order['kind'] != 'cyan':
            return self._ack('deliver', arguments, False, 'KIND_NOT_SUPPORTED_BY_M1_SKILL')
        if not isinstance(zone_slot_id, str) or not zone_slot_id:
            return self._ack('deliver', arguments, False, 'UNKNOWN_ZONE_SLOT')
        slot_id = zone_slot_id
        if zone_slot_id in self.map['zone_slots']:
            zone_slots = self.map['zone_slots'][zone_slot_id]
            slot_id = zone_slots[min(self._delivered_per_zone.get(zone_slot_id, 0), len(zone_slots) - 1)]['slot_id']
        try:
            slot = zone_slot(self.map, slot_id)
        except KeyError:
            return self._ack('deliver', arguments, False, 'UNKNOWN_ZONE_SLOT')
        if slot_id[0] != order['destination_zone']:
            return self._ack('deliver', arguments, False, 'SLOT_OUTSIDE_ORDER_DESTINATION')
        pickup = (order.get('initial_location') or {}).get('slot')
        if pickup is None:
            return self._ack('deliver', arguments, False, 'ORDER_WITHOUT_PICKUP_SLOT')
        if self.holding()['answer'] != 'no':
            return self._ack('deliver', arguments, False, 'NOT_EMPTY_HANDED')
        return self._start('deliver', 'deliver', arguments, slot_id=slot_id, slot_xy=list(slot['center_m']),
                           pickup_slot=pickup)

    def goto(self, target) -> dict:
        """Drive (own estimate + map A*) to a waypoint [x, y], a zone, a zone slot, a pickup slot or a door."""
        if isinstance(target, str):
            arguments = ({'target_zone': target} if target in self.map['zone_slots'] else
                         {'passage': target} if target == self.door_id else {'target_ref': target})
        elif (isinstance(target, Sequence) and not isinstance(target, (str, bytes)) and len(target) == 2
              and all(finite_number(v) for v in target)):
            arguments = {'waypoints': [[float(target[0]), float(target[1])]]}
        else:
            return self._ack('goto', {'target_ref': self._token(target)}, False, 'BAD_TARGET')
        if self.stopped is not None:
            return self._ack('goto', arguments, False, TERMINAL_STOP)
        goal = self._goto_goal(target)
        if goal is None:
            return self._ack('goto', arguments, False, 'UNKNOWN_TARGET')
        x0, x1, y0, y1 = self.map['bounds_m']
        if not (x0 < goal[0] < x1 and y0 < goal[1] < y1):
            return self._ack('goto', arguments, False, 'TARGET_OUTSIDE_MAP')
        return self._start('goto', 'goto', arguments, goal_xy=list(goal))

    def _goto_goal(self, target):
        if not isinstance(target, str):
            return float(target[0]), float(target[1])
        if target in self.map['zone_slots']:
            r = self.map['regions'][f'zone_{target}']
            return r['center_m'][0] - r['half_extents_m'][0] - ZONE_APPROACH_M, r['center_m'][1]
        if target in self.slots:
            return PICKUP_VIEW_X_M, self.slots[target]['center_m'][1]
        if target == self.door_id:
            rep = self.last_report
            west = rep is None or not rep.initialized or rep.x_m < self.door_xy[0]
            return self.door_xy[0] + (DOOR_SIDE_M if west else -DOOR_SIDE_M), self.door_xy[1]
        try:
            s = zone_slot(self.map, target)
        except KeyError:
            return None
        return s['center_m'][0] - SLOT_STANDOFF_M, s['center_m'][1]

    def look_around(self) -> dict:
        return self._start('look_around', 'look_around', {'observe': 'wide_look'})

    def hold(self, sim_s) -> dict:
        ok = finite_number(sim_s) and 0 <= sim_s <= MAX_HOLD_S
        arguments = {'duration_s': float(sim_s) if ok else self._token(sim_s)}
        if not ok:
            return self._ack('hold', arguments, False, 'BAD_DURATION')
        return self._start('hold', 'hold', arguments)

    wait = hold

    def abort(self, reason_code: str = 'caller_abort') -> dict:
        ok = isinstance(reason_code, str) and 0 < len(reason_code) <= REASON_TOKEN_MAX
        arguments = {'reason_code': reason_code if ok else self._token(reason_code)}
        if not ok:
            return self._ack('abort', arguments, False, 'BAD_REASON')
        if self.job is None:
            return self._ack('abort', arguments, False, 'NO_ACTIVE_JOB')
        job = self.job
        ack = self._ack('abort', arguments, True, job=job)
        self._fail(self.now, 'ABORTED:' + reason_code)
        self._local_state = 'hold_requested'
        return ack

    # ---------------------------------------------------------------- host-side cancel path
    def deadline(self) -> float | None:
        return None if self.job is None else self.job.deadline

    def expire_if_due(self, now: float) -> bool:
        """Local SIM budget, checked by the host even while a macro runs (Codex review 2, P1-5)."""
        if self.job is None or now <= self.job.deadline:
            return False
        self.now = float(now)
        self._fail(now, 'LOCAL_TIMEOUT', limit_s=self.job_sim_limit_s)
        self._local_state = 'local_timeout'
        return True

    def cancel(self, now: float, reason: str) -> bool:
        """End the active job (if any) with ``reason`` (episode end, host stop). One terminal event."""
        if self.job is None:
            return False
        self.now = float(now)
        self._fail(now, reason)
        return True

    def refuse(self, api: str, reason: str) -> dict:
        """A host-level refusal (episode ended) logged as this robot's rejected call."""
        return self._ack('hold' if api == 'wait' else api, {}, False, reason)

    def stop(self, now: float, reason: str) -> None:
        """Permanent stop (controller exception): the job fails once, every later call is refused."""
        self.cancel(now, reason)
        self.stopped = {'t': round(float(now), 3), 'reason': reason}
        self._local_state = 'command_rejected'

    # ---------------------------------------------------------------- job control
    def _record(self, now, outcome, confirmation, detail):
        job = self.job
        self.jobs_done.append({'job_id': job.job_id, 'kind': job.kind, 'outcome': outcome, 'confirmation': confirmation,
                               'started_at_sim_s': round(job.started_at, 3), 'ended_at_sim_s': round(now, 3),
                               'slot_id': job.args.get('slot_id'), **detail})

    def _finish(self, now, confirmation, outcome, **detail):
        if confirmation == 'own_camera_confirmed' and not self.gate.ok:
            confirmation, detail = 'unconfirmed', {**detail, 'confirmation_blocked_by': 'pose_uncertainty_gate'}
        self._record(now, outcome, confirmation, detail)
        self._emit(now, 'job_done', confirmation=confirmation, outcome=outcome, **detail)
        self._end_job(now)

    def _fail(self, now, reason, **detail):
        self._record(now, reason, 'failed', detail)
        if reason.startswith(UNCERTAIN_REASONS) or reason.endswith(UNCERTAIN_SUFFIXES):
            self._emit(now, 'pose_uncertain', level=self._level, gate_profile=self.gate.profile.name, ends_job=True,
                       reason=reason)
        if reason.endswith('_blocked'):
            self._emit(now, 'blockage_seen', passage_id=None, reason=reason, region=self._region(self.last_report),
                       source='own_progress_stall', stall_keepouts=detail.get('stall_keepouts', []))
        self._emit(now, 'job_failed', reason=reason, **{k: v for k, v in detail.items() if k != 'stall_keepouts'})
        self._end_job(now)

    def _end_job(self, now):
        job = self.job
        if job.ctl is not None:
            job.ctl.archive_leg()
            try:
                ctl_summary = job.ctl.summary()
            except Exception as exc:              # noqa: BLE001 - a broken controller must not hide the job end
                ctl_summary = {'summary_error': f'{type(exc).__name__}: {str(exc)[:500]}'}
            self._summaries.append({'job_id': job.job_id, 'controller': ctl_summary,
                                    'controller_events': list(job.ctl.events), 'legs': job.ctl.legs})
            sk = job.ctl.skill
            self.pose_sources_seen |= set(job.ctl.pose_sources) | set(getattr(sk, 'pose_sources', ()) or ())
            if sk is not None:
                self.cameras_seen |= set(getattr(sk, 'cameras_seen', ()) or ())
            self._holding_after = self._holding_after_deliver(job)
            if job.ctl.motion_profile is not None:
                self.pose.set_motion_profile(now, None)
        if job.driver is not None:
            self._summaries.append({'job_id': job.job_id, 'driver_log': list(job.driver.log),
                                    'guard_log': job.driver.guard_log, 'stall_keepouts': job.driver.stall_keepouts})
        if job.sweep is not None and job.sweep.get('guard'):
            self._summaries.append({'job_id': job.job_id, 'sweep_guard': job.sweep['guard']})
        self.job = None
        self._local_state = 'queue_empty'
        self._pending_hold = True

    def _holding_after_deliver(self, job):
        sk = job.ctl.skill
        if sk is None or sk.phase in ('nav_pregrasp',) and self.servo.get(1, 0) >= GRIPPER_OPEN_MIN_PWM:
            return {'answer': 'no', 'source': 'gripper_open_issued_since_last_release'}
        if any(e.get('event') == 'release_confirmed' for e in getattr(sk, 'events', ())):
            return {'answer': 'no', 'source': 'own_rgb_release_confirmed (wrist skill)'}
        if getattr(sk, 'box', None) is not None and sk.box.held:
            return {'answer': 'unknown', 'source': 'job ended with the gripper closed on an own-RGB attached box'}
        return {'answer': 'unknown', 'source': 'job ended mid-manipulation'}

    def step(self, now: float) -> dict:
        """One control decision for the physics owner.

        ``{'mode': 'tick', 'commands': [...]}`` (issue now, decide again after ``TICK_S``),
        ``{'mode': 'macro', 'action': {...}}`` (a wrist-skill macro), ``{'mode': 'capture'}``
        (capture a fresh own frame first, then call ``step`` again at the same time).
        """
        self.now = float(now)
        if self._pending_hold:
            self._pending_hold = False
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        if self.expire_if_due(now):
            return self.step(now)
        job = self.job
        if job is None:
            return {'mode': 'tick', 'commands': []}
        return getattr(self, '_step_' + job.kind)(now, job)

    def _step_hold(self, now, job):
        if job.hold_until is None:
            job.hold_until = now + float(job.args['duration_s'])
            job.phase = 'holding'
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        if now + 1e-9 >= job.hold_until:
            self._finish(now, 'unconfirmed', 'HOLD_ELAPSED', note='hold has nothing to confirm with the camera')
        return {'mode': 'tick', 'commands': []}

    def _step_deliver(self, now, job):
        if job.ctl is None:
            # A robot that idled (hold) localised only from passive frames; like M1's own start,
            # the delivery begins with a fresh wide own look (plumbing dev-s703: idle r3 p50 12 cm, std 6 cm).
            decision = self._tick_sweep(now, job)
            if decision is not None:
                return decision
            slot = self.slots[job.args['pickup_slot']]
            rect = (tuple(slot['x_range_m']), tuple(slot['y_range_m']))
            rows = [y for y in self.search_rows_y if slot['y_range_m'][0] <= y < slot['y_range_m'][1]]
            job.ctl = _DeliverController(self.map, self.params, box_kind='cyan', slot_id=job.args['slot_id'],
                                         slot_xy=job.args['slot_xy'], skill_factory=self._skill_for(job),
                                         pose_estimate_cls=self.pose_estimate_cls, search_rows_y=rows,
                                         robot_id=self.robot_id, seed=self.seed, order_kind='own_rgb_bay',
                                         shared_pose=self.pose, servo=self.servo, slot_rect=rect,
                                         all_rows_y=self.search_rows_y, gate=self.gate, guard=self.guard,
                                         static_keepouts=self.static_keepouts)
            job.ctl.last_obs, job.ctl.last_frame_id = self.last_obs, self.last_frame_id
            job.phase = 'm1_delivery'
        decision = job.ctl.decide(now)
        if decision['mode'] != 'done':
            return decision
        outcome = decision['outcome']
        placement = getattr(job.ctl.skill, 'placement', None) or {}
        detail = {'order_id': job.args['order_id'], 'slot_id': job.args['slot_id'],
                  'placement_reason': placement.get('reason'), 'slot_error_m': placement.get('slot_error_m')}
        if outcome == 'SKILL_OWN_RGB_PLACEMENT_IN_SLOT':
            zone = job.args['slot_id'][0]
            self._delivered_per_zone[zone] = self._delivered_per_zone.get(zone, 0) + 1
            gate = next((g for g in job.ctl.lookback_gates if g.get('frame_id') == placement.get('frame_id')), None)
            self._finish(now, 'own_camera_confirmed', outcome, look_back_gate_frame=None if gate is None
                         else gate['frame_id'], **detail)
        elif outcome.startswith('SKILL_OWN_RGB_PLACEMENT_') and outcome != 'SKILL_OWN_RGB_PLACEMENT_OUTSIDE_SLOT':
            self._finish(now, 'unconfirmed', outcome, **detail)
        else:
            leg = job.ctl.leg
            self._fail(now, outcome, stall_keepouts=[] if leg is None else leg.stall_keepouts, **detail)
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def _skill_for(self, job):
        factory = self.skill_factory

        def make(order):
            return factory(order, robot_id=self.robot_id)
        return make

    def _step_goto(self, now, job):
        if job.driver is None:
            loaded = self.holding()['answer'] == 'yes' or self._holding_after.get('answer') == 'unknown'
            job.driver = guards.GuardedDriver(self.pose.loc, self.map, self.params, loaded=loaded,
                                              goal_xy=job.args['goal_xy'], door_xy=self.door_xy,
                                              initial_servo=dict(self.servo), seed=self.seed, gate=self.gate,
                                              guard=self.guard)
            if loaded:
                job.driver.drive_pose = {**CARRY_POSTURE, 1: self.servo.get(1, CARRY_POSTURE[1])}
            job.phase = 'drive'
        cmds = job.driver.tick(now)
        if job.driver.outcome is None:
            return {'mode': 'tick', 'commands': cmds}
        if job.driver.outcome == 'arrived':
            self._finish(now, 'own_camera_confirmed', 'ARRIVED', looks=job.driver.looks,
                         note='arrival after a fixed own-camera look with the uncertainty gate ok')
        else:
            self._fail(now, 'GOTO_' + job.driver.outcome, looks=job.driver.looks,
                       stall_keepouts=job.driver.stall_keepouts)
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def _step_look_around(self, now, job):
        decision = self._tick_sweep(now, job)
        if decision is not None:
            return decision
        rep = self.pose.report(now)
        level = uncertainty_level(rep)
        tag_in_sweep = rep.since_tag_s is not None and rep.since_tag_s <= now - job.started_at
        if self.gate.ok and tag_in_sweep:
            self._finish(now, 'own_camera_confirmed', 'LOOKED', level=level, std_xy_m=round(rep.std_xy_m, 4))
        else:
            self._finish(now, 'unconfirmed', 'LOOKED_POSE_UNCERTAIN', level=level, gate=self.gate.state)
        return {'mode': 'tick', 'commands': []}

    def _tick_sweep(self, now, job):
        """Guarded wide own look (LOOK_P20 over the clear part of WIDE_LOOK_PANS), then restore. None when done."""
        if job.sweep is None:
            restore = {k: v for k, v in self.servo.items() if k in (1, 3, 4, 5, 6)}
            pose = dict(LOOK_P20)
            if self.servo.get(1, 0) < GRIPPER_OPEN_MIN_PWM:
                pose[1] = self.servo.get(1, 1500)        # keep the grip as issued
            loaded = self.holding()['answer'] != 'no'
            plan = self.guard.plan(self.servo, pose, WIDE_LOOK_PANS, guards.OwnPose.from_report(self.pose.report(now)),
                                   loaded=loaded, allow_backoff=True)
            job.sweep = {'pose': pose, 'queue': list(plan['pans']) or [int(self.servo.get(6, 1500))], 'restore': restore,
                         'stage': 'arm', 'since': now, 'loaded': loaded,
                         'guard': [{'t': round(now, 3), **{k: plan[k] for k in ('pans', 'dropped', 'reason', 'backoff')}}]}
            if plan['backoff'] is not None:
                cmd, duration = guards.backoff_commands(plan['backoff'])
                job.sweep.update(stage='backoff', cmd=cmd, until=now + duration)
            job.phase = 'look'
        s = job.sweep
        if s['stage'] == 'done':
            return None
        if s['stage'] == 'backoff':
            if now + 1e-9 < s['until']:
                return {'mode': 'tick', 'commands': [dict(s['cmd'])]}
            if now + 1e-9 < s['until'] + SETTLE_S:
                return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
            plan = self.guard.plan(self.servo, s['pose'], WIDE_LOOK_PANS,
                                   guards.OwnPose.from_report(self.pose.report(now)), loaded=s['loaded'])
            s['guard'].append({'t': round(now, 3), **{k: plan[k] for k in ('pans', 'dropped', 'reason')}})
            s['queue'] = list(plan['pans']) or [int(self.servo.get(6, 1500))]
            s['stage'] = 'arm'
        if s['stage'] == 'arm':
            steps = self._arm_steps(s['pose'])
            if steps:
                return {'mode': 'tick', 'commands': [{'kind': 'hold'}] + steps}
            s['stage'], s['since'], s['target'] = 'pan', now, s['queue'].pop(0)
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        if s['stage'] == 'pan':
            steps = self._arm_steps({6: s['target']})
            if steps:
                s['since'] = now
                return {'mode': 'tick', 'commands': [{'kind': 'hold'}] + steps}
            if now - s['since'] < SETTLE_S:
                return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
            if s['queue']:
                s['target'], s['since'] = s['queue'].pop(0), now
                return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
            s['stage'], s['since'] = 'restore', now
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        steps = self._arm_steps(s['restore'])
        if steps:
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}] + steps}
        if now - s['since'] < SETTLE_S:
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        s['stage'] = 'done'
        return None

    def _arm_steps(self, target):
        out = []
        for servo, want in sorted(target.items()):
            cur = self.servo.get(servo, want)
            if cur == want:
                continue
            nxt = cur + int(np.clip(want - cur, -60, 60))
            out.append({'kind': 'look', 'pan_pulse': nxt} if servo == 6 else
                       {'kind': 'arm', 'servo_id': servo, 'pulse': nxt})
        return out

    # ---------------------------------------------------------------- M1-style outcome block
    def summary(self) -> dict:
        sources = sorted(self.pose_sources_seen)
        return {'schema': SCHEMA, 'robot_id': self.robot_id, 'mode': self.mode, 'pose_sources_seen': sources,
                'cameras_seen': sorted(self.cameras_seen), 'jobs': list(self.jobs_done),
                'events': len(self.events), 'rejected_foreign_frames': self._rejected_frames,
                'stopped': self.stopped, 'gate_transitions': list(self.gate.transitions),
                'counts_as_m1_inputs': bool(self.mode == 'm1' and sources and
                                            all(m1_contract.is_m1_pose_source(s) for s in sources)
                                            and sorted(self.cameras_seen) == ['robot_cam'])}


def __getattr__(name):
    """``OwnCamTeamHost`` moved to ``harness.zone_own_team_host`` (lazy: importing this module stays sim-free)."""
    if name == 'OwnCamTeamHost':
        from harness.zone_own_team_host import OwnCamTeamHost
        return OwnCamTeamHost
    raise AttributeError(name)
