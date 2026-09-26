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
(reason), ``blockage_seen`` and ``pose_uncertain``.

Inputs (and nothing else): the robot's own ``robot_cam`` observations (JPEG +
own issued PWM), its own issued commands (fed back from its own port), the
static tagged map, fixed calibrations and the scenario order sheet (kind,
count, destination zone, coarse pickup-bay slot; never a coordinate). No
simulator import, no world handle, no peer handle: the physics owner
(``OwnCamTeamHost`` below, or package G's runner) feeds each executor its own
frames and its own command log only.

Internally (read-only reuse, nothing forked):

* pose: ``harness.owncam_pose_source.OwnCamPoseSource`` - ONE localizer per
  robot for the whole episode, shared by every job;
* deliver: ``harness.m1_owncam_delivery.M1OwnCamDelivery`` (PR #201) with skill
  ``harness.wrist_zone_skill_v9`` (PR #181) in ``mode='m1'``; the executor only
  restricts the own-RGB search to the order sheet's coarse pickup slot;
* goto: ``harness.owncam_drive_v2.OwnCamDriverV2`` (PR #178/#197 loop driver,
  map A* from the own estimate, stop-and-look) on the shared localizer;
* judgments: ``harness.zone_own_perception`` (PR #193) ``judge_route_blockage``
  and ``judge_holding_item`` on own frames taken in the agreed postures.

M1 contract (``mode='m1'``): every pose source must be the own-camera
estimator (``harness.m1_owncam_contract`` and ``harness.m1_contract`` both
check), every observation is validated (own robot_cam, own robot id, fresh,
hash) and the executor refuses an injected pose source that is not own-camera.
``mode='diagnostic'`` accepts an injected pose source for tests; its results
never count as M1.

``OwnCamTeamHost`` (end of the file) is the multi-robot physics owner: one
MuJoCo world, one ``CameraRobotPort`` and one executor per robot, sync SIM.
It imports the simulator lazily, keeps every simulator-truth record under
``eval_only`` and never hands the world, another robot's port or another
executor to an executor.
"""
from __future__ import annotations

import copy
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any

import numpy as np

from harness import m1_contract, m1_owncam_contract
from harness.m1_owncam_delivery import LIMITS as M1_LIMITS
from harness.m1_owncam_delivery import M1OwnCamDelivery
from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SETTLE_S, WIDE_LOOK_PANS
from harness.owncam_drive_v2 import OwnCamDriverV2
from harness.owncam_pose_source import OwnCamPoseSource, PoseReport

SCHEMA = 'ugrp.zone_own_executor.v1'
EVENT_SCHEMA = 'ugrp.zone_own_executor_event.v1'
STATUS_SCHEMA = 'ugrp.zone_own_executor_status.v1'
MODES = ('m1', 'diagnostic')
ROBOTS = ('r1', 'r2', 'r3')
JOB_KINDS = ('deliver', 'goto', 'look_around', 'hold')
EVENTS = ('job_started', 'job_done', 'job_failed', 'blockage_seen', 'pose_uncertain')
CONFIRMATIONS = ('own_camera_confirmed', 'unconfirmed')
ANSWERS = ('yes', 'no', 'unknown')
UNCERTAINTY_LEVELS = ('low', 'medium', 'high', 'unknown')
TICK_S = .1

# ---------------------------------------------------------------- package A/D adapter
# Package A (PR #194, harness/zone_study_contract.py, not merged into this branch):
# ``ugrp.zone_study_action.v1`` kinds and executor local states, pinned here so the
# adapter works before the merge; tests pin the values against the #194 source text.
ACTION_LOG_SCHEMA = 'ugrp.zone_study_action.v1'
A_ACTION_KINDS = ('claim_order', 'goto', 'observe', 'grasp', 'place', 'release', 'wait', 'yield_passage',
                  'abort_job', 'noop')
A_LOCAL_STATES = ('command_issued', 'queue_empty', 'hold_requested', 'local_timeout', 'command_rejected')
API_TO_ACTION_KIND = {'deliver': 'claim_order', 'goto': 'goto', 'look_around': 'observe', 'hold': 'wait',
                      'abort': 'abort_job'}
# Package D (PR #186/#194 harness/zone_event_scheduler.py) wake triggers. None = log only, no wake.
D_TRIGGERS = ('start', 'failure', 'blockage', 'timeout', 'report', 'retry', 'idle', 'timer')
EVENT_TO_TRIGGER = {'job_started': None, 'job_done': 'idle', 'job_failed': 'failure',
                    'blockage_seen': 'blockage', 'pose_uncertain': None}
# A job_failed whose reason is a local SIM budget is a timeout for D (own timer), not a view change.
TIMEOUT_REASONS = ('LOCAL_TIMEOUT',)

# ---------------------------------------------------------------- map vocabulary
# Package A/E pickup bays (harness/zone_map_schematic.pickup_bays, PR #194): regions.pickup split
# into 2 columns (P1, P2) x 3 rows (-1 south .. -3 north). Re-derived here from the static map only.
PICKUP_BAY_COLUMNS, PICKUP_BAY_ROWS = 2, 3
SLOT_SEARCH_MARGIN_M = .15      # own-RGB cyan detections kept within the ordered pickup slot + this margin
PICKUP_VIEW_X_M = -.47          # = m1_owncam_delivery.SEARCH_VIEW_X_M (west of the pickup grid)
FAR_BAY_M = 1.0                 # a bay starting this far east of the west viewpoint gets lane viewpoints
LANE_OFFSET_M = .40             # mid-lane between static pickup rows (rows are 0.80 m apart)
ZONE_APPROACH_M = .25           # goto('A'): stop this far west of the zone paint
SLOT_STANDOFF_M = .40           # goto('A2'): stop this far west of the slot centre
DOOR_SIDE_M = .45               # goto('door_1'): the far side of the door, this far from its centre
# Pose-uncertainty levels from the own PoseReport (M1 pre-registered limits reused).
LOW_STD = (M1_LIMITS['look_back'].max_std_xy_m, M1_LIMITS['look_back'].max_std_yaw_rad)
MEDIUM_STD = (M1_LIMITS['nav_unloaded'].max_std_xy_m, M1_LIMITS['nav_unloaded'].max_std_yaw_rad)
# Own-camera judgments (PR #193 agreed postures and commit rule).
POSTURE_TOLERANCE_PWM = 90
PAN_CENTRE_PWM, PAN_TOLERANCE_PWM = 1500, 40
JUDGE_PERIOD_S = 1.
COMMIT_CONFIDENCE = .65
BLOCKAGE_CONSECUTIVE = 2
HOLDING_CHECK_MAX_AGE_S = 3.
GRIPPER_OPEN_MIN_PWM = 1900
DOOR_LANE_RANGE_M = 1.2
DEFAULT_JOB_SIM_LIMIT_S = 720.  # = scripts/run_m1_owncam.SIM_LIMIT_S
PRE_DELIVER_LOOK = True         # deliver starts with one wide own look (see _step_deliver)
CARRY_PHASES = ('to_carry_posture', 'nav_preplace', 'grip_check', 'pre_release')
ORDER_KEYS = ('order_id', 'kind', 'count', 'item_ids', 'required_robots', 'destination_zone',
              'initial_location', 'identity')


class ExecutorContractError(m1_owncam_contract.M1ContractError):
    """An input the executor contract forbids (non own-camera pose, foreign observation, coordinates in orders)."""


def pickup_slots(static_map: Mapping) -> dict[str, dict]:
    """{'P1-1': {'bay_id', 'center_m', 'x_range_m', 'y_range_m'}, ...} from ``regions.pickup`` only."""
    region = static_map['regions']['pickup']
    (cx, cy), (hx, hy) = region['center_m'], region['half_extents_m']
    bay_w, slot_h = 2. * hx / PICKUP_BAY_COLUMNS, 2. * hy / PICKUP_BAY_ROWS
    out = {}
    for col in range(PICKUP_BAY_COLUMNS):
        x0 = cx - hx + bay_w * col
        for row in range(PICKUP_BAY_ROWS):
            y0 = cy - hy + slot_h * row
            sid = f'P{col + 1}-{row + 1}'
            out[sid] = {'bay_id': f'P{col + 1}', 'center_m': [round(x0 + bay_w / 2, 4), round(y0 + slot_h / 2, 4)],
                        'x_range_m': [round(x0, 4), round(x0 + bay_w, 4)],
                        'y_range_m': [round(y0, 4), round(y0 + slot_h, 4)]}
    return out


def pickup_slot_of(static_map: Mapping, xy: Sequence[float]) -> str | None:
    """Coarse slot id of a floor point (used by scenario configs to write the order sheet)."""
    for sid, s in pickup_slots(static_map).items():
        if s['x_range_m'][0] <= xy[0] < s['x_range_m'][1] and s['y_range_m'][0] <= xy[1] < s['y_range_m'][1]:
            return sid
    return None


def zone_slot(static_map: Mapping, slot_id: str) -> dict:
    for slots in static_map['zone_slots'].values():
        for s in slots:
            if s['slot_id'] == slot_id:
                return s
    raise KeyError(f'unknown zone slot {slot_id!r}')


def validate_order_sheet(order_sheet: Mapping, static_map: Mapping) -> dict[str, dict]:
    """Order lines by id; a coordinate or unknown key anywhere in a line is refused (exact-pose smuggling)."""
    orders = order_sheet.get('orders')
    if not isinstance(orders, Sequence) or isinstance(orders, str):
        raise ExecutorContractError('order_sheet.orders must be a list')
    slots = pickup_slots(static_map)
    out = {}
    for order in orders:
        extra = sorted(set(order) - set(ORDER_KEYS))
        if extra:
            raise ExecutorContractError(f'order line keys {extra} are not order-sheet vocabulary')
        where = order.get('initial_location') or {}
        if set(where) - {'pickup_bay', 'slot'}:
            raise ExecutorContractError('initial_location may only name pickup_bay / slot')
        for v in where.values():
            if not isinstance(v, str):
                raise ExecutorContractError('initial_location must be map vocabulary, never a coordinate')
        if where.get('slot') is not None and where['slot'] not in slots:
            raise ExecutorContractError(f"unknown pickup slot {where['slot']!r}")
        if order.get('destination_zone') not in static_map['zone_slots']:
            raise ExecutorContractError(f"unknown destination zone {order.get('destination_zone')!r}")
        out[str(order['order_id'])] = copy.deepcopy(dict(order))
    return out


def lane_viewpoints(slot_rect, rows_y: Sequence[float], y_est: float) -> list[tuple[float, float]]:
    """Extra search viewpoints for a far pickup slot: its west edge, on the mid-lanes between static rows."""
    (x0, _), (y0, y1) = slot_rect
    if x0 - PICKUP_VIEW_X_M <= FAR_BAY_M:
        return []
    lanes = sorted({round(r + d, 3) for r in rows_y for d in (-LANE_OFFSET_M, LANE_OFFSET_M)
                    if y0 - .1 <= r + d <= y1 + .1}, key=lambda y: (abs(y - y_est), y))
    return [(float(x0), y) for y in lanes]


def uncertainty_level(report: PoseReport) -> str:
    if not report.initialized or not math.isfinite(report.std_xy_m):
        return 'unknown'
    if report.std_xy_m <= LOW_STD[0] and report.std_yaw_rad <= LOW_STD[1]:
        return 'low'
    if report.std_xy_m <= MEDIUM_STD[0] and report.std_yaw_rad <= MEDIUM_STD[1]:
        return 'medium'
    return 'high'


def scheduler_trigger(event: Mapping) -> str | None:
    """Package D wake trigger for one executor event (None = logged, no call)."""
    if event['event'] == 'job_failed' and str(event['detail'].get('reason', '')).startswith(TIMEOUT_REASONS):
        return 'timeout'
    return EVENT_TO_TRIGGER[event['event']]


def action_record(ack: Mapping, *, run_id: str, condition: str, seed: int, request_id: str) -> dict:
    """Package A ``ugrp.zone_study_action.v1`` record of one API call (adapter; A validates it)."""
    return {'schema': ACTION_LOG_SCHEMA, 'run_id': run_id, 'condition': condition, 'seed': int(seed),
            'actor': ack['robot_id'], 'action_id': ack['action_id'], 'request_id': request_id,
            'submitted_at_sim_s': ack['sim_s'], 'kind': API_TO_ACTION_KIND[ack['api']],
            'arguments': dict(ack['arguments']), 'order_id': ack['arguments'].get('order_id'), 'role': None,
            'accepted': bool(ack['accepted']), 'rejected_reason': ack['rejected_reason'],
            'local_state': ack['local_state']}


class _DeliverController(M1OwnCamDelivery):
    """M1 delivery on the executor's shared localizer; own-RGB search limited to the ordered pickup slot."""

    def __init__(self, *args, shared_pose: OwnCamPoseSource, servo: Mapping[int, int],
                 slot_rect: tuple[tuple[float, float], tuple[float, float]], all_rows_y: Sequence[float] = (),
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.all_rows_y = tuple(float(y) for y in all_rows_y)
        self.pose = shared_pose                 # one localizer per robot for the whole episode
        self.servo = dict(servo)                # own issued servo state at job start
        self.slot_rect = slot_rect

    def _init(self, now):
        """M1 init (localise, west viewpoints), plus lane viewpoints for a far bay (P2).

        dev-s703 plumb3: from the west viewpoints (x = -0.47) a P2 box (x 1.0 / 1.6) is 1.5-2.1 m away,
        a few pixels on the horizon; neither ``near`` nor ``far_coarse`` fitted it (SEARCH_NOT_FOUND x2).
        For a slot whose bay starts more than ``FAR_BAY_M`` east of the west viewpoint the robot also
        looks from the bay's west edge, on the mid-lanes between pickup rows (static layout rows, the
        same rows M1 uses for its viewpoints), after the west viewpoints (whose near boxes become keep-outs).
        """
        decision = super()._init(now)
        if self.phase == 'search_leg' and not getattr(self, '_lanes_added', False):
            self._lanes_added = True
            extra = lane_viewpoints(self.slot_rect, self.all_rows_y, self.pose.report(now).y_m)
            if extra:
                self.viewpoints += extra
                self._event(now, 'lane_viewpoints', viewpoints=self.viewpoints)
        return decision

    def _in_slot(self, xy) -> bool:
        (x0, x1), (y0, y1) = self.slot_rect
        m = SLOT_SEARCH_MARGIN_M
        return x0 - m <= xy[0] <= x1 + m and y0 - m <= xy[1] <= y1 + m

    def _search_detect(self, obs, report):
        n = len(self.cyan)
        super()._search_detect(obs, report)
        # Order sheet: the item stands in this coarse pickup slot, so a cyan box seen elsewhere is not it.
        self.cyan[n:] = [d for d in self.cyan[n:] if self._in_slot(d['map_xy'])]


class _Job:
    def __init__(self, job_id: str, kind: str, args: Mapping, now: float, limit_s: float):
        self.job_id, self.kind, self.args = job_id, kind, dict(args)
        self.started_at, self.deadline = float(now), float(now) + float(limit_s)
        self.phase = 'start'
        self.ctl: _DeliverController | None = None
        self.driver: OwnCamDriverV2 | None = None
        self.sweep: dict | None = None
        self.hold_until: float | None = None
        self.outcome: str | None = None


class ZoneOwnExecutor:
    """One robot's own-camera executor. Holds no simulator, no peer and no world reference."""

    def __init__(self, robot_id: str, static_map: Mapping, params: Mapping, order_sheet: Mapping, *,
                 skill_factory: Callable[..., Any], pose_estimate_cls, search_rows_y: Sequence[float],
                 mode: str = 'm1', seed: int = 0, pose_source: OwnCamPoseSource | None = None,
                 job_sim_limit_s: float = DEFAULT_JOB_SIM_LIMIT_S, judgments: bool = True):
        m1_contract.check_mode(mode)
        if robot_id not in ROBOTS:
            raise ValueError(f'robot_id must be one of {ROBOTS}')
        self.robot_id, self.mode, self.seed = robot_id, mode, int(seed)
        # Own copies: nothing mutable is shared with another robot's executor.
        self.map = copy.deepcopy(dict(static_map))
        self.params = copy.deepcopy(dict(params))
        self.orders = validate_order_sheet(order_sheet, self.map)
        self.order_sheet = copy.deepcopy(dict(order_sheet))
        self.slots = pickup_slots(self.map)
        self.skill_factory = skill_factory
        self.pose_estimate_cls = pose_estimate_cls
        self.search_rows_y = tuple(float(y) for y in search_rows_y)
        self.job_sim_limit_s = float(job_sim_limit_s)
        self.judgments = bool(judgments)
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
        self.servo: dict[int, int] = {}
        self.last_obs: Mapping | None = None
        self.last_frame_id: int | None = None
        self.last_rgb: np.ndarray | None = None
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
        self._counter = 0
        self._local_state = 'queue_empty'
        self._holding_after = {'answer': 'no', 'source': 'episode_start_gripper_never_closed'}
        self._delivered_per_zone: dict[str, int] = {}
        self._blockage_streak = 0
        self._blockage_armed = True
        self._last_blockage: dict | None = None
        self._last_holding_check: dict | None = None
        self._last_judge_t = -1e9
        self._uncertain_armed = False
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
        if obs.get('robot_id') != self.robot_id:
            self._rejected_frames += 1
            raise ExecutorContractError(f"{self.robot_id} refuses an observation of {obs.get('robot_id')!r}")
        m1_owncam_contract.validate_observation(obs, robot_id=self.robot_id, previous_frame_id=self.last_frame_id,
                                                now=now)
        self.cameras_seen.add(str(obs['camera']))
        self.last_frame_id = int(obs['frame_id'])
        self.last_obs, self.last_rgb, self.now = obs, rgb, float(now)
        job = self.job
        if job is not None and job.ctl is not None:
            report = job.ctl.on_frame(now, obs, rgb)
        else:
            report = self.pose.on_frame(now, rgb)
        if self.mode == 'm1':
            self._require_owncam(report.source, 'frame report')
        self.pose_sources_seen.add(report.source)
        self.last_report = report
        self._update_level(now, report)
        if self.judgments and now - self._last_judge_t >= JUDGE_PERIOD_S - 1e-9:
            if self._judge(now, obs, rgb, report):
                self._last_judge_t = now
        return report

    # ---------------------------------------------------------------- own-camera judgments
    def _posture(self, obs):
        pose = {int(k): int(v) for k, v in obs['actuator_state']['servo_pulses'].items()}
        if abs(pose.get(6, 0) - PAN_CENTRE_PWM) > PAN_TOLERANCE_PWM:
            return None, pose
        for name, ref in (('look_p20', LOOK_P20), ('carry', CARRY_POSTURE)):
            if all(abs(pose.get(s, -9999) - v) <= POSTURE_TOLERANCE_PWM for s, v in ref.items() if s in (3, 4, 5)):
                return name, pose
        return None, pose

    def _moving(self, obs):
        return any(abs(float(v)) > 1e-9 for v in obs['actuator_state'].get('motor_commands', ()))

    def _judge(self, now, obs, rgb, report) -> bool:
        name, pose = self._posture(obs)
        if name is None or self._moving(obs) or not report.initialized:
            return False
        from harness import zone_own_perception as perception
        level = uncertainty_level(report)
        belief = {'x_m': report.x_m, 'y_m': report.y_m, 'yaw_rad': report.yaw_rad,
                  'confidence': {'low': 'high', 'medium': 'medium'}.get(level, 'low')}
        near_door = (math.hypot(self.door_xy[0] - report.x_m, self.door_xy[1] - report.y_m) < DOOR_LANE_RANGE_M
                     and abs(math.cos(report.yaw_rad)) > .8)
        passage = self.door_id if near_door else None
        block = perception.judge_route_blockage(rgb, pose, static_map=self.map, pose_belief=belief, passage_id=passage)
        row = {'t': round(now, 3), 'judgment': 'route_blockage', 'posture': name, 'frame_id': int(obs['frame_id']),
               'answer': block['answer'], 'confidence': block['confidence'], 'reason': block['reason'],
               'passage_id': passage, 'belief_confidence': belief['confidence']}
        self.judgment_log.append(row)
        self._last_blockage = row
        if block['answer'] == 'yes' and block['confidence'] >= COMMIT_CONFIDENCE:
            self._blockage_streak += 1
            if self._blockage_streak >= BLOCKAGE_CONSECUTIVE and self._blockage_armed:
                self._blockage_armed = False
                self._emit(now, 'blockage_seen', passage_id=passage, confidence=block['confidence'],
                           reason=block['reason'], frame_id=int(obs['frame_id']), region=self._region(report))
        else:
            self._blockage_streak = 0
            if block['answer'] == 'no':
                self._blockage_armed = True
        if name == 'carry':
            hold = perception.judge_holding_item(rgb, pose, expected_kind=self._held_kind())
            hrow = {'t': round(now, 3), 'judgment': 'holding_item', 'frame_id': int(obs['frame_id']),
                    'answer': hold['answer'], 'confidence': hold['confidence'], 'reason': hold['reason']}
            self.judgment_log.append(hrow)
            self._last_holding_check = hrow
        return True

    def _held_kind(self):
        job = self.job
        if job is not None and job.kind == 'deliver':
            return self.orders[job.args['order_id']]['kind']
        return 'cyan'

    def _update_level(self, now, report):
        level = uncertainty_level(report)
        if level in ('low', 'medium'):
            self._uncertain_armed = True
        elif self._uncertain_armed and level in ('high', 'unknown'):
            self._uncertain_armed = False
            self._emit(now, 'pose_uncertain', level=level, std_xy_m=None if not report.initialized else
                       round(report.std_xy_m, 4), ends_job=False)
        self._level = level

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
        if self.job is not None:
            return self._ack(api, arguments, False, f'BUSY:{self.job.kind}:{self.job.job_id}')
        job = _Job(self._next_id('job'), kind, {**arguments, **job_args}, self.now, self.job_sim_limit_s)
        self.job = job
        self._emit(self.now, 'job_started', arguments=arguments)
        return self._ack(api, arguments, True, job=job)

    def deliver(self, item_ref: str, zone_slot_id: str) -> dict:
        """Deliver the order line ``item_ref`` to a zone slot ('A2') or a zone ('A': next own slot)."""
        arguments = {'order_id': item_ref, 'target_ref': zone_slot_id}
        order = self.orders.get(item_ref)
        if order is None:
            return self._ack('deliver', arguments, False, 'UNKNOWN_ORDER')
        if order['kind'] != 'cyan':
            return self._ack('deliver', arguments, False, 'KIND_NOT_SUPPORTED_BY_M1_SKILL')
        slot_id = zone_slot_id
        if zone_slot_id in self.map['zone_slots']:
            n = self._delivered_per_zone.get(zone_slot_id, 0)
            slot_id = self.map['zone_slots'][zone_slot_id][min(n, len(self.map['zone_slots'][zone_slot_id]) - 1)]['slot_id']
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
        arguments = ({'waypoints': [[float(target[0]), float(target[1])]]} if not isinstance(target, str)
                     else {'target_zone': target} if target in self.map['zone_slots'] else
                     {'passage': target} if target == self.door_id else {'target_ref': target})
        goal = self._goto_goal(target)
        if goal is None:
            return self._ack('goto', arguments, False, 'UNKNOWN_TARGET')
        x0, x1, y0, y1 = self.map['bounds_m']
        if not (x0 < goal[0] < x1 and y0 < goal[1] < y1):
            return self._ack('goto', arguments, False, 'TARGET_OUTSIDE_MAP')
        return self._start('goto', 'goto', arguments, goal_xy=list(goal))

    def _goto_goal(self, target):
        if not isinstance(target, str):
            if len(target) != 2 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in target):
                return None
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

    def hold(self, sim_s: float) -> dict:
        arguments = {'duration_s': float(sim_s)}
        if not (isinstance(sim_s, (int, float)) and math.isfinite(sim_s) and sim_s >= 0):
            return self._ack('hold', arguments, False, 'BAD_DURATION')
        return self._start('hold', 'hold', arguments)

    wait = hold

    def abort(self, reason_code: str = 'caller_abort') -> dict:
        arguments = {'reason_code': reason_code}
        if self.job is None:
            return self._ack('abort', arguments, False, 'NO_ACTIVE_JOB')
        job = self.job
        ack = self._ack('abort', arguments, True, job=job)
        self._fail(self.now, 'ABORTED:' + reason_code)
        self._pending_hold = True
        return ack

    # ---------------------------------------------------------------- status (own only)
    def holding(self) -> dict:
        job = self.job
        sk = job.ctl.skill if job is not None and job.ctl is not None else None
        if sk is not None:
            phase = sk.phase
            box = getattr(sk, 'box', None)
            if phase in CARRY_PHASES and box is not None and box.held:
                base = {'answer': 'yes', 'source': 'own_rgb_attachment_check (wrist skill)', 'skill_phase': phase}
            elif phase in ('look_back', 'finished') and any(e.get('event') == 'release_confirmed'
                                                              for e in getattr(sk, 'events', ())):
                base = {'answer': 'no', 'source': 'own_rgb_release_confirmed (wrist skill)', 'skill_phase': phase}
            elif phase in ('nav_pregrasp',) and self.servo.get(1, 0) >= GRIPPER_OPEN_MIN_PWM:
                base = {'answer': 'no', 'source': 'gripper_open_issued_since_last_release', 'skill_phase': phase}
            else:
                base = {'answer': 'unknown', 'source': 'wrist skill mid-manipulation', 'skill_phase': phase}
        else:
            base = dict(self._holding_after)
        check = self._last_holding_check
        if check is not None and self.now - check['t'] <= HOLDING_CHECK_MAX_AGE_S:
            base['camera_check'] = {k: check[k] for k in ('answer', 'confidence', 'reason', 't')}
            if (check['answer'] in ('yes', 'no') and base['answer'] in ('yes', 'no') and check['answer'] != base['answer']
                    and check['confidence'] >= COMMIT_CONFIDENCE):
                base['answer'], base['conflict'] = 'unknown', True
        return base

    def _region(self, report):
        if report is None or not report.initialized:
            return 'unknown'
        x, y = report.x_m, report.y_m
        if math.hypot(x - self.door_xy[0], y - self.door_xy[1]) < .35:
            return self.door_id
        for name, r in self.map['regions'].items():
            (cx, cy), (hx, hy) = r['center_m'], r['half_extents_m']
            if abs(x - cx) <= hx and abs(y - cy) <= hy:
                return name
        return 'west_floor' if x < self.door_xy[0] else 'east_floor'

    def status(self) -> dict:
        """Own executor state and own-camera judgments only. No peer, no simulator field."""
        rep = self.last_report
        job = self.job
        blocked = self._last_blockage
        if blocked is None or self.now - blocked['t'] > 5.:
            blocked_ahead = {'answer': 'unknown', 'reason': 'no recent own look in an agreed posture'}
        else:
            blocked_ahead = {k: blocked[k] for k in ('answer', 'confidence', 'reason', 'passage_id', 't')}
        loc = {'level': self._level, 'initialized': bool(rep is not None and rep.initialized)}
        if rep is not None and rep.initialized:
            loc.update(std_xy_m=round(rep.std_xy_m, 4), std_yaw_rad=round(rep.std_yaw_rad, 4),
                       since_tag_s=None if rep.since_tag_s is None else round(rep.since_tag_s, 2),
                       own_estimate_xy_yaw=[round(rep.x_m, 3), round(rep.y_m, 3), round(rep.yaw_rad, 4)])
        return {'schema': STATUS_SCHEMA, 'robot_id': self.robot_id, 'mode': self.mode, 'sim_s': round(self.now, 3),
                'local_state': self._local_state,
                'job': None if job is None else {'job_id': job.job_id, 'kind': job.kind, 'phase': self._job_phase(job),
                                                 'started_at_sim_s': round(job.started_at, 3),
                                                 'arguments': {k: v for k, v in job.args.items()
                                                               if k in ('order_id', 'target_ref', 'target_zone',
                                                                        'passage', 'waypoints', 'duration_s',
                                                                        'observe', 'slot_id', 'pickup_slot')}},
                'holding': self.holding(), 'blocked_ahead': blocked_ahead, 'localization': loc,
                'region': self._region(rep), 'jobs_finished': len(self.jobs_done)}

    def belief_projection(self) -> dict:
        """Package A ``BELIEF_KEYS`` projection of the status (for the robot's own prompt)."""
        st = self.status()
        last_done = next((j for j in reversed(self.jobs_done) if j['confirmation'] == 'own_camera_confirmed'), None)
        return {'region': st['region'], 'last_visual_anchor': None if self.last_frame_id is None else
                f'own-{self.robot_id}-{self.last_frame_id:05d}',
                'last_requested_destination': None if self.job is None else self.job.args.get('slot_id') or
                self.job.args.get('target_ref') or self.job.args.get('target_zone'),
                'last_visually_confirmed_region': None if last_done is None else last_done.get('slot_id'),
                'confidence': {'low': 'high', 'medium': 'medium', 'high': 'low'}.get(st['localization']['level'], 'low'),
                'sources': ['own_rgb', 'own_commands', 'static_map'],
                'held_item_guess': st['holding']['answer'],
                'blocked_passages': [st['blocked_ahead']['passage_id']] if st['blocked_ahead']['answer'] == 'yes'
                and st['blocked_ahead'].get('passage_id') else [],
                'notes_ko': ''}

    def _job_phase(self, job):
        if job.ctl is not None:
            return f"{job.ctl.phase}:{getattr(job.ctl.skill, 'phase', '')}"
        if job.driver is not None:
            return f'drive:{job.driver.state}'
        return job.phase

    # ---------------------------------------------------------------- job control
    def _finish(self, now, confirmation, outcome, **detail):
        job = self.job
        rec = {'job_id': job.job_id, 'kind': job.kind, 'outcome': outcome, 'confirmation': confirmation,
               'started_at_sim_s': round(job.started_at, 3), 'ended_at_sim_s': round(now, 3),
               'slot_id': job.args.get('slot_id'), **detail}
        self.jobs_done.append(rec)
        self._emit(now, 'job_done', confirmation=confirmation, outcome=outcome, **detail)
        self._end_job(now)

    def _fail(self, now, reason, **detail):
        job = self.job
        rec = {'job_id': job.job_id, 'kind': job.kind, 'outcome': reason, 'confirmation': 'failed',
               'started_at_sim_s': round(job.started_at, 3), 'ended_at_sim_s': round(now, 3),
               'slot_id': job.args.get('slot_id'), **detail}
        self.jobs_done.append(rec)
        if reason.startswith(('POSE_UNCERTAIN', 'NOT_INITIALIZED')) or reason.endswith(('lost', 'not_initialized')):
            self._emit(now, 'pose_uncertain', level=self._level, ends_job=True, reason=reason)
        self._emit(now, 'job_failed', reason=reason, **detail)
        self._end_job(now)

    def _end_job(self, now):
        job = self.job
        if job.ctl is not None:
            self._summaries.append({'job_id': job.job_id, 'controller': job.ctl.summary(),
                                    'controller_events': list(job.ctl.events)})
            sk = job.ctl.skill
            self.pose_sources_seen |= set(job.ctl.pose_sources) | set(getattr(sk, 'pose_sources', ()) or ())
            if sk is not None:
                self.cameras_seen |= set(getattr(sk, 'cameras_seen', ()) or ())
            self._holding_after = self._holding_after_deliver(job)
            if job.ctl.motion_profile is not None:
                self.pose.set_motion_profile(now, None)
        if job.driver is not None:
            self._summaries.append({'job_id': job.job_id, 'driver_log': list(job.driver.log)})
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
        job = self.job
        if job is None:
            return {'mode': 'tick', 'commands': []}
        if now > job.deadline:
            self._fail(now, 'LOCAL_TIMEOUT', limit_s=self.job_sim_limit_s)
            self._local_state = 'local_timeout'
            return self.step(now)
        handler = getattr(self, '_step_' + job.kind)
        return handler(now, job)

    def _step_hold(self, now, job):
        if job.hold_until is None:
            job.hold_until = now + float(job.args['duration_s'])
            job.phase = 'holding'
            return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
        if now + 1e-9 >= job.hold_until:
            self._finish(now, 'unconfirmed', 'HOLD_ELAPSED', note='hold has nothing to confirm with the camera')
            return {'mode': 'tick', 'commands': []}
        return {'mode': 'tick', 'commands': []}

    def _step_deliver(self, now, job):
        if job.ctl is None and PRE_DELIVER_LOOK:
            # A robot that idled (hold) localised only from passive frames; like M1's own start,
            # the delivery begins with a fresh wide own look (plumbing dev-s703: idle r3 p50 12 cm, std 6 cm).
            decision = self._tick_sweep(now, job)
            if decision is not None:
                return decision
        if job.ctl is None:
            slot = self.slots[job.args['pickup_slot']]
            rect = (tuple(slot['x_range_m']), tuple(slot['y_range_m']))
            rows = [y for y in self.search_rows_y if slot['y_range_m'][0] <= y < slot['y_range_m'][1]]
            job.ctl = _DeliverController(self.map, self.params, box_kind='cyan', slot_id=job.args['slot_id'],
                                         slot_xy=job.args['slot_xy'], skill_factory=self._skill_for(job),
                                         pose_estimate_cls=self.pose_estimate_cls, search_rows_y=rows,
                                         robot_id=self.robot_id, seed=self.seed, order_kind='own_rgb_bay',
                                         shared_pose=self.pose, servo=self.servo, slot_rect=rect,
                                         all_rows_y=self.search_rows_y)
            job.ctl.last_obs, job.ctl.last_frame_id = self.last_obs, self.last_frame_id
            job.phase = 'm1_delivery'
        decision = job.ctl.decide(now)
        if decision['mode'] != 'done':
            return decision
        outcome = decision['outcome']
        placement = getattr(job.ctl.skill, 'placement', None) or {}
        detail = {'order_id': job.args['order_id'], 'slot_id': job.args['slot_id'],
                  'placement_reason': placement.get('reason'),
                  'slot_error_m': placement.get('slot_error_m')}
        if outcome == 'SKILL_OWN_RGB_PLACEMENT_IN_SLOT':
            zone = job.args['slot_id'][0]
            self._delivered_per_zone[zone] = self._delivered_per_zone.get(zone, 0) + 1
            gate = next((g for g in job.ctl.lookback_gates if g.get('frame_id') == placement.get('frame_id')), None)
            self._finish(now, 'own_camera_confirmed', outcome, look_back_gate_frame=None if gate is None
                         else gate['frame_id'], **detail)
        elif outcome.startswith('SKILL_OWN_RGB_PLACEMENT_') and outcome != 'SKILL_OWN_RGB_PLACEMENT_OUTSIDE_SLOT':
            self._finish(now, 'unconfirmed', outcome, **detail)
        else:
            self._fail(now, outcome, **detail)
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def _skill_for(self, job):
        factory = self.skill_factory

        def make(order):
            return factory(order, robot_id=self.robot_id)
        return make

    def _step_goto(self, now, job):
        if job.driver is None:
            loaded = self.holding()['answer'] == 'yes' or self._holding_after.get('answer') == 'unknown'
            job.driver = _SharedLocDriver(self.pose.loc, self.map, self.params, loaded=loaded,
                                          goal_xy=job.args['goal_xy'], door_xy=self.door_xy,
                                          initial_servo=dict(self.servo), seed=self.seed)
            if loaded:
                job.driver.drive_pose = {**CARRY_POSTURE, 1: self.servo.get(1, CARRY_POSTURE[1])}
            job.phase = 'drive'
        cmds = job.driver.tick(now)
        if job.driver.outcome is None:
            return {'mode': 'tick', 'commands': cmds}
        if job.driver.outcome == 'arrived':
            self._finish(now, 'own_camera_confirmed', 'ARRIVED', looks=job.driver.looks,
                         note='arrival declared after a final own-camera stop-and-look')
        else:
            self._fail(now, 'GOTO_' + job.driver.outcome, looks=job.driver.looks)
        return {'mode': 'tick', 'commands': [{'kind': 'hold'}]}

    def _step_look_around(self, now, job):
        decision = self._tick_sweep(now, job)
        if decision is not None:
            return decision
        rep = self.pose.report(now)
        level = uncertainty_level(rep)
        if level in ('low', 'medium'):
            self._finish(now, 'own_camera_confirmed', 'LOOKED', level=level, std_xy_m=round(rep.std_xy_m, 4))
        else:
            self._finish(now, 'unconfirmed', 'LOOKED_POSE_UNCERTAIN', level=level)
        return {'mode': 'tick', 'commands': []}

    def _tick_sweep(self, now, job):
        """Wide own look (LOOK_P20, WIDE_LOOK_PANS), then restore the issued posture. None when finished."""
        if job.sweep is None:
            restore = {k: v for k, v in self.servo.items() if k in (1, 3, 4, 5, 6)}
            pose = dict(LOOK_P20)
            if self.servo.get(1, 0) < GRIPPER_OPEN_MIN_PWM:
                pose[1] = self.servo.get(1, 1500)        # keep the grip as issued
            job.sweep = {'pose': pose, 'queue': list(WIDE_LOOK_PANS), 'restore': restore, 'stage': 'arm',
                         'since': now}
            job.phase = 'look'
        s = job.sweep
        if s['stage'] == 'done':
            return None
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
                'counts_as_m1_inputs': bool(self.mode == 'm1' and sources and
                                            all(m1_contract.is_m1_pose_source(s) for s in sources)
                                            and sorted(self.cameras_seen) == ['robot_cam'])}


class _SharedLocDriver(OwnCamDriverV2):
    """Loop driver v2 on the executor's localizer: commands/frames reach the localizer once (via the executor)."""

    def __init__(self, shared_loc, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.loc = shared_loc
        self.last_estimate = self.loc.estimate()

    def on_command(self, row):                  # servo bookkeeping only; the executor feeds the localizer
        kind = row['kind']
        if kind == 'initial_servo_command':
            self.servo = {int(k): int(v) for k, v in row['pulses'].items()}
        elif kind == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif kind == 'look':
            self.servo[6] = int(row['pan_pulse'])


# ====================================================================== multi-robot physics owner
class _RobotSlot:
    """Host-side bookkeeping of one robot (never visible to any executor)."""

    def __init__(self, rid, port, executor):
        self.rid, self.port, self.executor = rid, port, executor
        self.commands: list[dict] = []
        self.frames: list[dict] = []
        self.timeline: list[tuple[float, list]] = []
        self.next_decide = 0.
        self.next_frame = 0.
        self.capture_after = False
        self.decisions: list[dict] = []
        self.exception: dict | None = None
        self.dead = False


class OwnCamTeamHost:
    """One MuJoCo world, three robots, one own-camera executor each (sync SIM, weld OFF).

    The host owns physics and rendering. Each executor receives exactly: its own
    port's ``robot_cam`` observations and its own issued-command rows. Everything
    read from the simulator (poses, box positions, contacts, equality constraints)
    goes to ``self.eval_only`` and is written under ``eval_only/`` by the caller.
    """

    FRAME_S = .2
    GT_S = .05

    def __init__(self, spec: Mapping, student: Mapping, *, root, study_layer: Callable, frames_dir=None):
        import importlib
        import json
        from pathlib import Path

        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_arena import LAYOUTS, layout
        from sim.zone_cargo_contact import CARGO_PROFILES, apply as apply_cargo_profile, base_profile, profile_record
        from sim.zone_landmarks import TaggedZoneScene

        self.spec, self.student, self.root = dict(spec), dict(student), Path(root)
        profile = spec['contact_profile']
        self.scene = TaggedZoneScene.from_tagged(spec['map'], spec['seed'], spec['goal'], spec.get('extra_boxes'),
                                                 contact_profile=base_profile(profile))
        xml_transform = ((lambda xml: apply_cargo_profile(self.scene.transform(xml), profile))
                         if profile in CARGO_PROFILES else self.scene.transform)
        self.world = MultiMasterPiProductionV2(seed=spec['seed'], width=640, height=480, render=True,
                                               warehouse_layout=self.scene.engine_layout, warehouse_cargo_ids=None,
                                               xml_transform=xml_transform)
        self.scene.setup(self.world)
        self.contact_record = {'profile': profile, 'base_profile': base_profile(profile),
                               'cargo_profile': profile_record(profile) if profile in CARGO_PROFILES else None,
                               'noslip_iterations': int(self.world.model.opt.noslip_iterations),
                               'timestep_s': float(self.world.model.opt.timestep),
                               'user_decision': 'pending user approval (PR #181/#189)'}
        if profile == 'cargo_noslip_v1' and self.contact_record['noslip_iterations'] <= 0:
            raise RuntimeError('cargo_noslip_v1 requested but noslip_iterations is 0')
        self.static = self.scene.config['static_map']
        self.objects = self.scene.config['setup_only']['objects']
        self.spawns = self.scene.config['setup_only']['spawns']
        calibration = json.loads((self.root / student['calibration']).read_text())
        module, name = student['skill_module'], student['skill_class']
        skill_mod = importlib.import_module(module)
        skill_cls = getattr(skill_mod, name)
        arena = layout(self.static['base_map']['map_id'])
        # Static layout only (idle-spawn discs, #181 v6 contract; v9 inherits v6's class), never a live pose.
        from harness.wrist_zone_skill_v6 import StaticKeepout
        keepouts = tuple(StaticKeepout(f'spawn_row_{i}', (float(arena['spawn_x']), float(y)), .17,
                                       'static_layout_idle_spawn')
                         for i, y in enumerate(arena['spawn_rows_y']))
        self.keepout_records = [k.record() for k in keepouts]
        from harness.map_goto import UNLOADED_ENVELOPE, plan_path
        from harness.owncam_drive import LOADED_ENVELOPE
        from harness.wrist_zone_skill import PoseEstimate
        rows_y = LAYOUTS['zone_wide']['pickup_rows_y']
        self.order_sheet = spec['order_sheet']
        self.robots: dict[str, _RobotSlot] = {}
        for rid in ROBOTS:
            port = CameraRobotPort(self.world, rid, allow_reverse=True, allow_mecanum=True)
            ex_ref: dict = {}
            own_static = copy.deepcopy(self.static)          # per-robot copies: no shared mutable state

            def planner(start, goal, carrying, _ref=ex_ref, _static=own_static):
                job = _ref['ex'].job
                obstacles = job.ctl._keepouts() if job is not None and job.ctl is not None else []
                result = plan_path(_static, start, goal, LOADED_ENVELOPE if carrying else UNLOADED_ENVELOPE,
                                   obstacles=obstacles, escape_start_m=.25)
                return None if result is None else [tuple(p) for p in result['waypoints_m'][1:]]

            def factory(order, *, robot_id, _planner=planner, _keepouts=keepouts,
                        _bounds=tuple(own_static['bounds_m'])):
                return skill_cls(order, planner=_planner, robot_id=robot_id, mode='m1', static_keepouts=_keepouts,
                                 static_bounds_m=list(_bounds))

            ex = ZoneOwnExecutor(rid, own_static, calibration['params'], self.order_sheet, skill_factory=factory,
                                 pose_estimate_cls=PoseEstimate, search_rows_y=rows_y, mode=student.get('mode', 'm1'),
                                 seed=spec['seed'], job_sim_limit_s=spec.get('job_sim_limit_s', DEFAULT_JOB_SIM_LIMIT_S))
            ex_ref['ex'] = ex
            self.robots[rid] = _RobotSlot(rid, port, ex)
        self.study_layer = study_layer
        self.frames_dir = Path(frames_dir) if frames_dir else None
        self.eval_only = {'gt': [], 'frames_eval': [], 'contacts': [], 'kind_steps': {r: {} for r in ROBOTS},
                          'retention': {r: {'carry_steps': 0, 'both_finger_steps': 0, 'low_box_steps': 0,
                                            'min_box_z_m': None} for r in ROBOTS},
                          'max_eq_active': 0}
        self.api_calls: list[dict] = []
        self.event_log: list[dict] = []
        self._geoms()
        self._next_gt = 0.
        for rid, slot in self.robots.items():
            pulses = {int(k): int(v) for k, v in self.world.robot(rid).servo_command_pulses.items()}
            self._sink(rid, {'t': 0.0, 'kind': 'initial_servo_command', 'pulses': pulses})

    # ------------------------------------------------------------ eval-only geometry
    def _geoms(self):
        import mujoco
        m = self.world.model
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
        self._names = names
        self._wall = {g for g, n in enumerate(names) if n.startswith('zone_wall_')}
        self._own = {r: {g for g, n in enumerate(names) if n.startswith(r + '__')} for r in ROBOTS}
        self._fingers = {r: ({g for g, n in enumerate(names) if n == r + '__left_finger'},
                             {g for g, n in enumerate(names) if n == r + '__right_finger'}) for r in ROBOTS}
        self._box_geom = {oid: {g for g, n in enumerate(names) if n == o['body_name'] + '_geom'}
                          for oid, o in self.objects.items()}
        self._all_box = set().union(*self._box_geom.values())
        self.assigned_box = {}                  # eval-only: robot -> box id of its scripted order line

    # ------------------------------------------------------------ own-input plumbing
    def _sink(self, rid, row):
        slot = self.robots[rid]
        slot.commands.append(row)
        slot.executor.on_command(row)

    def _apply(self, rid, action, now):
        row = {'t': round(float(now), 4), **action}
        self._sink(rid, row)
        self.robots[rid].port.apply(action, now)

    def _hold(self, rid, now):
        self._sink(rid, {'t': round(float(now), 4), 'kind': 'hold'})
        self.robots[rid].port.hold(now)

    def _guard(self, rid, now, fn, *args):
        """A controller/skill exception fails THAT robot's job and parks the robot; the others go on."""
        slot = self.robots[rid]
        if slot.dead:
            return None
        try:
            return fn(rid, now, *args)
        except OSError:
            raise                                  # host I/O (disk full, render device): infrastructure, not the robot
        except Exception as exc:                  # noqa: BLE001 - recorded as a failed job, never swallowed
            import traceback
            slot.exception = {'t': round(now, 3), 'type': type(exc).__name__, 'message': str(exc)[:2000],
                              'traceback': traceback.format_exc()[-6000:]}
            slot.dead, slot.timeline, slot.capture_after = True, [], False
            ex = slot.executor
            if ex.job is not None:
                ex._fail(now, f'EXCEPTION:{type(exc).__name__}')
            self._hold(rid, now)
            return None

    def _capture(self, rid, now):
        return self._guard(rid, now, self._capture_raw)

    def _capture_raw(self, rid, now):
        import base64

        import cv2
        slot = self.robots[rid]
        obs = slot.port.capture()                         # this robot's own robot_cam only
        jpeg = base64.b64decode(obs['image'])
        rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        index = len(slot.frames)
        if self.frames_dir is not None:
            d = self.frames_dir / rid
            d.mkdir(parents=True, exist_ok=True)
            (d / f'{index:05d}.jpg').write_bytes(jpeg)
        report = slot.executor.on_frame(now, obs, rgb)
        slot.frames.append({'frame': index, 't': round(now, 4), 'frame_id': obs['frame_id'], 'sha256': obs['sha256'],
                            'robot_id': obs['robot_id'], 'camera': obs['camera'],
                            'commanded_servo': obs['actuator_state']['servo_pulses'], 'report': report.as_dict()})
        x, y, yaw = self._truth(rid)
        row = {'robot_id': rid, 'frame': index, 't': round(now, 4), 'gt': [round(x, 5), round(y, 5), round(yaw, 6)]}
        if report.initialized:
            row.update(pos_err_m=round(math.hypot(report.x_m - x, report.y_m - y), 5),
                       yaw_err_deg=round(abs(math.degrees((report.yaw_rad - yaw + math.pi) % (2 * math.pi) - math.pi)), 4),
                       std_xy_m=round(report.std_xy_m, 5))
        self.eval_only['frames_eval'].append(row)
        slot.next_frame = now + self.FRAME_S

    def _truth(self, rid):
        r = self.world.robot(rid)
        xyz, rpy = r.base_xyz(), r.base_rpy()
        return float(xyz[0]), float(xyz[1]), float(rpy[2])

    # ------------------------------------------------------------ physics
    def _physics_until(self, t_end):
        world, data = self.world, self.world.data
        while float(data.time) < t_end - 1e-9:
            now = float(data.time)
            for s in self.robots.values():
                s.port.tick(now)
            world._physics_step_for(world.controllers['r1'])
            now = float(data.time)
            kinds = {r: set() for r in ROBOTS}
            fingers = {r: {} for r in ROBOTS}
            for i in range(data.ncon):
                c = data.contact[i]
                pair = {int(c.geom1), int(c.geom2)}
                for r in ROBOTS:
                    mine = pair & self._own[r]
                    if not mine:
                        continue
                    other = next(iter(pair - mine), None)
                    kind = ('wall' if other in self._wall else
                            'peer_robot' if any(other in self._own[q] for q in ROBOTS if q != r) else
                            'box' if other in self._all_box else None)
                    if kind:
                        kinds[r].add(kind)
                    if other in self._all_box:
                        lf, rf = self._fingers[r]
                        box = next(b for b, gs in self._box_geom.items() if other in gs)
                        f = fingers[r].setdefault(box, [False, False])
                        f[0] |= bool(mine & lf)
                        f[1] |= bool(mine & rf)
            for r in ROBOTS:
                for k in kinds[r]:
                    self.eval_only['kind_steps'][r][k] = self.eval_only['kind_steps'][r].get(k, 0) + 1
                    last = self.eval_only['contacts'][-1] if self.eval_only['contacts'] else None
                    if not last or last['robot_id'] != r or last['kind'] != k or last['t'] < now - .1:
                        self.eval_only['contacts'].append({'t': round(now, 4), 'robot_id': r, 'kind': k})
                ex = self.robots[r].executor
                sk = ex.job.ctl.skill if ex.job is not None and ex.job.ctl is not None else None
                box = self.assigned_box.get(r)
                if sk is not None and sk.phase in CARRY_PHASES and box is not None:
                    ret = self.eval_only['retention'][r]
                    bz = float(data.body(self.objects[box]['body_name']).xpos[2])
                    ret['carry_steps'] += 1
                    ret['both_finger_steps'] += int(all(fingers[r].get(box, (False, False))))
                    ret['low_box_steps'] += int(bz < .04)
                    ret['min_box_z_m'] = bz if ret['min_box_z_m'] is None else min(ret['min_box_z_m'], bz)
            if len(data.eq_active):
                self.eval_only['max_eq_active'] = max(self.eval_only['max_eq_active'], int(data.eq_active.max()))
            if now + 1e-9 >= self._next_gt:
                self.eval_only['gt'].append({'t': round(now, 4), 'robots': {r: [round(v, 5) for v in self._truth(r)]
                                                                             for r in ROBOTS},
                                             'boxes': {b: [round(float(v), 4) for v in
                                                           data.body(o['body_name']).xpos]
                                                       for b, o in self.objects.items() if o['kind'] == 'cyan'}})
                self._next_gt = now + self.GT_S
            for rid, s in self.robots.items():
                if not s.dead and now + 1e-9 >= s.next_frame:
                    self._capture(rid, now)

    # ------------------------------------------------------------ macros (as scripts/run_m1_owncam.execute_macro)
    def _macro_timeline(self, rid, action, now):
        ex = self.robots[rid].executor
        kind = action['kind']
        if kind == 'drive':
            cmd = {'kind': 'drive', 'forward': action['fwd'], 'turn': action['turn'], 'duration_s': action['duration']}
            return [(now, [cmd]), (now + action['duration'] + .2, ['hold'])]
        if kind == 'mecanum':
            cmd = {'kind': 'mecanum', 'forward': action['forward'], 'left': action['left'], 'turn': action['turn'],
                   'duration_s': action['duration']}
            return [(now, [cmd]), (now + action['duration'] + .1, ['hold'])]
        if kind == 'pose':
            start = ex.last_obs['actuator_state']['servo_pulses']
            targets = action['pulses']
            delta = max(abs(p - start[str(s)]) for s, p in targets.items())
            duration = max(.25, delta / 600.)
            count = max(5, math.ceil(duration / .05))
            out = []
            for sample in range(1, count + 1):
                u = sample / count
                ease = u * u * (3 - 2 * u)
                cmds = []
                for servo, end in targets.items():
                    pulse = round(start[str(servo)] + ease * (end - start[str(servo)]))
                    cmds.append({'kind': 'look', 'pan_pulse': pulse} if int(servo) == 6 else
                                {'kind': 'arm', 'servo_id': int(servo), 'pulse': pulse})
                out.append((now + (sample - 1) * duration / count, cmds))
            sk = ex.job.ctl.skill if ex.job is not None and ex.job.ctl is not None else None
            settle = .3 if sk is not None and sk.phase == 'grasp' and sk.box.phase == 'approach' else .15
            out.append((now + duration + settle, []))
            return out
        if kind == 'wait':
            return [(now, [{'kind': 'wait'}]), (now + max(.05, action['duration']), [])]
        raise ValueError('UNKNOWN_MACRO')

    def _decide(self, rid, now):
        return self._guard(rid, now, self._decide_raw)

    def _decide_raw(self, rid, now):
        slot = self.robots[rid]
        ex = slot.executor
        for _ in range(12):
            decision = ex.step(now)
            mode = decision['mode']
            if mode == 'capture':
                self._capture(rid, now)
                continue
            if mode == 'tick':
                for cmd in decision['commands']:
                    if cmd['kind'] == 'hold':
                        self._hold(rid, now)
                    else:
                        self._apply(rid, cmd, now)
                slot.next_decide = now + TICK_S
                return
            if mode == 'macro':
                slot.decisions.append({'t': round(now, 3), 'action': decision['action']})
                slot.timeline = self._macro_timeline(rid, decision['action'], now)
                slot.capture_after = True
                return
            raise ValueError(f'unknown decision mode {mode!r}')
        slot.next_decide = now + TICK_S

    def _run_timeline(self, rid, now):
        return self._guard(rid, now, self._run_timeline_raw)

    def _run_timeline_raw(self, rid, now):
        slot = self.robots[rid]
        while slot.timeline and slot.timeline[0][0] <= now + 1e-9:
            _, cmds = slot.timeline.pop(0)
            for cmd in cmds:
                if cmd == 'hold':
                    self._hold(rid, now)
                else:
                    self._apply(rid, cmd, now)
        if not slot.timeline and slot.capture_after:
            slot.capture_after = False
            self._capture(rid, now)
            slot.next_decide = now

    def call(self, rid, api, *args):
        """The study layer's only door into an executor: the job API of THAT robot."""
        ex = self.robots[rid].executor
        ex.now = float(self.world.data.time)
        ack = getattr(ex, api)(*args)
        self.api_calls.append(ack)
        return ack

    def run(self, sim_limit_s: float, done: Callable[[], bool] | None = None) -> dict:
        data = self.world.data
        self._physics_until(.5)
        for s in self.robots.values():
            s.next_decide = float(data.time)
        self.study_layer(self, 'start', None, float(data.time))
        outcome = None
        while True:
            now = float(data.time)
            if now > sim_limit_s:
                outcome = 'SIM_LIMIT'
                break
            for rid in ROBOTS:
                slot = self.robots[rid]
                if slot.dead:
                    continue
                if slot.timeline or slot.capture_after:
                    self._run_timeline(rid, now)
                if not slot.timeline and not slot.capture_after and now + 1e-9 >= slot.next_decide:
                    self._decide(rid, now)
            for rid in ROBOTS:
                for ev in self.robots[rid].executor.drain_events():
                    self.event_log.append(ev)
                    self.study_layer(self, 'event', ev, now)
            if done is not None and done():
                outcome = 'STUDY_LAYER_DONE'
                break
            live = [s for s in self.robots.values() if not s.dead]
            if not live:
                outcome = 'ALL_ROBOTS_STOPPED'
                break
            nxt = min(s.timeline[0][0] if s.timeline else s.next_decide for s in live)
            self._physics_until(max(nxt, now + float(self.world.model.opt.timestep)))
        for rid in ROBOTS:
            self._hold(rid, float(data.time))
        self._physics_until(float(data.time) + .5)
        return {'outcome': outcome, 'sim_s': round(float(data.time), 3)}

    def close(self):
        self.world.close()
