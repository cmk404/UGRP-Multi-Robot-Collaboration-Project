"""Integrated zone study: the study core (#194) drives three own-camera executors (#206).

One trial = one condition x scenario x seed. The pieces are reused, not re-implemented:

* conditions, per-call inputs, prompts, protocol, message bus, SIM cost, scheduler
  and the no-LLM fixture actor come from ``harness.zone_study_offline.OfflineTrial``
  (packages A/C/D/E/I of PR #194). ``IntegratedTrial`` subclasses it and replaces
  only the stored-frame input source and the action sink;
* each robot's executor is ``harness.zone_own_executor.ZoneOwnExecutor`` (PR #206),
  reached through a :class:`RobotLink` that exposes that ONE robot's own frames,
  own executor state and own job API;
* the pair status channel is ``harness.team_carry_status`` (PR #200, main).

Clock. The physics owner advances in ``QUANTUM_S`` chunks (the SIM cost quantum
of ``harness.zone_sim_cost``), so every scheduler event (call start, charged
release, delivery, timer) lands exactly on a chunk boundary: a call captures the
robot's own inputs at the physics time it starts, and its action reaches the
executor at the physics time its SIM cost ends.

Information boundary (issue #223, Codex review of PR #169). Between robots there
are exactly two channels:

1. the condition's dialogue channel (package C ``Transport`` + package D
   scheduler), absent in ``no_comm``, hub-and-spoke in ``leader_ko``;
2. the pair status channel (:class:`PairStatusBus`): fixed enum, no task content,
   identical in every condition (user decision 2026-09-26).

There is no shared board, no host claim arbitration, no peer-job-end wake and no
GT-driven wake. A robot's call triggers are its own start, its own executor
events, its own timers and (only where the channel is open) messages it actually
received. Physical safety stops inside the executor stay as they are and are the
same in every condition.

Pose provider seam. The executor's PoseReport source is chosen by
``configs/zone_study_integration/pose_providers.json`` and hashed into the run
bundle. ``tags_temporary`` (own-camera wall-tag PF) is a TEMPORARY stand-in:
every record carries ``pose_provider`` and the note ``TEMPORARY_NOTE_KO``.

Nothing here imports the simulator; ``scripts/run_zone_study_integration.py`` is
the physics owner.
"""
from __future__ import annotations

import copy
import hashlib
import importlib
import json
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from harness import m1_contract, m1_owncam_contract
from harness import team_carry_status as tcs
from harness import zone_study_offline as zo
from harness import zone_study_prompts_ko as pk
from harness.zone_own_executor import API_TO_ACTION_KIND, EVENTS as EXECUTOR_EVENTS
from harness.zone_sim_cost import params as cost_params_for
from harness.zone_study_contract import (COMMAND_ARGUMENT_KEYS, MAIN_CONDITIONS, ROBOTS, ContractViolation,
                                         digest)
from harness.zone_study_inputs import (action_log_record, build_call_input, command_entry, own_rgb_ref,
                                       provenance)

ROOT = Path(__file__).resolve().parents[1]
INTEGRATION_SCHEMA = 'ugrp.zone_study_integration.v1'
EXECUTION_BUNDLE_ID = 'zone-study-integration-v1'
PROVIDER_CONFIG = ROOT / 'configs' / 'zone_study_integration' / 'pose_providers.json'
PROVIDER_SCHEMA = 'ugrp.zone_study_pose_providers.v1'
PROVIDER_KEYS = ('factory', 'version', 'source_label_prefix', 'maps', 'calibration', 'source_files',
                 'temporary', 'research_result', 'uses_landmark_tags', 'note_ko')
PROVIDER_METHODS = ('on_command', 'on_frame', 'report', 'set_motion_profile')
TEMPORARY_NOTE_KO = '임시, 표식 사용, 연구 결과 아님'
#: Physics chunk = the SIM cost quantum, so scheduler events land on chunk boundaries.
QUANTUM_S = cost_params_for().quantum_s
#: While a robot's call is in flight: an idle robot holds (the executor idles with
#: a hold), a running executor job continues. The executor API has no pause/resume
#: (#206); the thinking cost reaches physics through the delayed action release.
THINK_HOLD_POLICY = 'idle_robot_holds_busy_job_continues'
ACTION_MAP_VERSION = 'zone_study_action_map.v1'
#: ``wait`` on an idle executor = hold this long (then job_done -> idle wake).
WAIT_HOLD_S = 10.0
FIXTURE_ACTOR = 'fixture_v1'
#: Planned real-LLM settings (coordinator decision on #222). Not callable here:
#: this runner makes no model call (see ``check_actor``).
PLANNED_MODEL = {'completer': 'harness.gemini_proxy.GeminiProxyCompleter', 'model': 'gemini-3.8-flash',
                 'temperature': 0.2, 'reasoning_effort': 'none', 'enabled': False}


def file_sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_actor(actor: str) -> str:
    """Only the #194 no-LLM fixture actor may drive this runner (no live model call)."""
    if actor != FIXTURE_ACTOR:
        raise ContractViolation(f'actor {actor!r} is not enabled: this runner makes no model call; only '
                                f'{FIXTURE_ACTOR!r} (harness.zone_study_offline.FixtureActor) is allowed')
    return actor


# ---------------------------------------------------------------------------
# Pose provider seam

def load_pose_providers(path=PROVIDER_CONFIG) -> dict:
    data = json.loads(Path(path).read_text())
    if data.get('schema') != PROVIDER_SCHEMA or not isinstance(data.get('providers'), Mapping):
        raise ContractViolation(f'{path}: not a {PROVIDER_SCHEMA} file')
    return data


def pose_provider_spec(provider_id, *, map_id, path=PROVIDER_CONFIG) -> dict:
    """The registered provider ``provider_id`` for ``map_id`` (closed keys, map allow-list)."""
    providers = load_pose_providers(path)['providers']
    if not isinstance(provider_id, str) or provider_id not in providers:
        raise ContractViolation(f'unknown pose provider {provider_id!r}; registered: {sorted(providers)}')
    spec = providers[provider_id]
    if set(spec) != set(PROVIDER_KEYS):
        raise ContractViolation(f'pose provider {provider_id}: keys {sorted(spec)} != {sorted(PROVIDER_KEYS)}')
    if map_id not in spec['maps']:
        raise ContractViolation(f'pose provider {provider_id} is not registered for map {map_id!r}')
    if spec['uses_landmark_tags'] and (spec['research_result'] or not spec['temporary']
                                       or spec['note_ko'] != TEMPORARY_NOTE_KO):
        raise ContractViolation(f'pose provider {provider_id} uses landmark tags, so it must be temporary, '
                                f'not a research result, and carry the note {TEMPORARY_NOTE_KO!r}')
    return {'provider_id': provider_id, **copy.deepcopy(spec)}


def provider_record(spec, root=ROOT) -> dict:
    """What the run bundle pins about the provider: the spec plus source/calibration hashes."""
    files = {f: file_sha256(Path(root) / f) for f in spec['source_files']}
    row = {'pose_provider': spec['provider_id'], 'spec': copy.deepcopy(spec), 'source_files_sha256': files,
           'calibration_sha256': file_sha256(Path(root) / spec['calibration']),
           'label': {'pose_provider': spec['provider_id'], 'temporary': spec['temporary'],
                     'research_result': spec['research_result'], 'note_ko': spec['note_ko']}}
    row['record_sha256'] = digest(row)
    return row


def build_pose_provider(spec, static_map, params, seed):
    """Instantiate the registered provider and check the executor-facing interface."""
    module, _, name = spec['factory'].partition(':')
    provider = getattr(importlib.import_module(module), name)(static_map, params, seed=int(seed))
    check_pose_provider(provider, spec)
    return provider


def check_pose_provider(provider, spec) -> str:
    """Interface + M1 own-camera label. Raises ContractViolation; returns the source label."""
    missing = [m for m in PROVIDER_METHODS if not callable(getattr(provider, m, None))]
    loc = getattr(provider, 'loc', None)
    missing += [f'loc.{m}' for m in ('estimate', 'predict_to') if not callable(getattr(loc, m, None))]
    if missing:
        raise ContractViolation(f'pose provider {spec["provider_id"]} misses {missing}')
    label = getattr(provider, 'source', None)
    if not isinstance(label, str) or not label.startswith(spec['source_label_prefix']):
        raise ContractViolation(f'pose provider {spec["provider_id"]} source {label!r} does not start with '
                                f'{spec["source_label_prefix"]!r}')
    try:
        m1_owncam_contract.require_m1_source(label)
        m1_contract.require_m1_pose_source(label, 'pose provider')
    except (m1_owncam_contract.M1ContractError, m1_contract.ContractViolation) as exc:
        raise ContractViolation(f'pose provider {spec["provider_id"]}: {exc}') from exc
    return label


# ---------------------------------------------------------------------------
# Pair status channel (all conditions)

class PairStatusBus:
    """``harness.team_carry_status`` channels, one per order that needs 2+ robots.

    Built the same way in every condition from the order sheet alone. A message is
    exactly ``robot_id, task_id, seq, state, sent_at_s`` with ``state`` in the
    fixed enum (``StatusMessage.parse``); it never reaches a model input.
    """

    def __init__(self, order_sheet):
        tasks = sorted(o['order_id'] for o in order_sheet['orders'] if int(o['required_robots']) >= 2)
        self.channels = {t: tcs.StatusChannel(t, ROBOTS) for t in tasks}
        self._publishers: dict[tuple[str, str], tcs.StatusPublisher] = {}

    def config(self) -> dict:
        return {'profile': tcs.PROFILE, 'states': list(tcs.STATES), 'fields': sorted(tcs.FIELDS),
                'heartbeat_s': tcs.HEARTBEAT_S, 'stale_s': tcs.STALE_S,
                'max_partner_align_wait_s': tcs.MAX_PARTNER_ALIGN_WAIT_S, 'tasks': sorted(self.channels),
                'participants': list(ROBOTS)}

    def config_sha256(self) -> str:
        return digest(self.config())

    def publish(self, robot_id, task_id, state, now) -> bool:
        """Own executor state of ``robot_id`` on pair task ``task_id`` (enum only)."""
        if task_id not in self.channels or robot_id not in ROBOTS or state not in tcs.STATES:
            return False
        key = (robot_id, task_id)
        pub = self._publishers.setdefault(key, tcs.StatusPublisher(self.channels[task_id], robot_id))
        before = len(self.channels[task_id].log)
        pub.tick(state, float(now))
        return len(self.channels[task_id].log) > before

    def partner_view(self, robot_id, task_id, now):
        """Partners' latest enum state; only for a robot that is on the task itself."""
        if (robot_id, task_id) not in self._publishers:
            return None
        return self.channels[task_id].partner_view(robot_id, float(now))

    def record(self) -> dict:
        return {'config': self.config(), 'config_sha256': self.config_sha256(),
                'channels': {t: {'log': list(c.log), 'rejected': list(c.rejected)}
                             for t, c in self.channels.items()},
                'messages': sum(len(c.log) for c in self.channels.values())}


# ---------------------------------------------------------------------------
# Decision -> executor API

@dataclass(frozen=True)
class Plan:
    api: str | None
    args: tuple = ()
    rejected_reason: str | None = None


def executor_plan(action, job) -> Plan:
    """The executor call for one validated model action (at most one call).

    ``job`` is the robot's OWN running job (``{'kind', 'order_id'}``) or None.
    claim -> deliver(order_id, destination_zone); continue -> no call;
    wait -> abort the own job, or hold ``WAIT_HOLD_S`` when idle;
    release -> abort the own deliver job of that order. Anything else is refused.
    """
    kind = action.get('kind') if isinstance(action, Mapping) else None
    if kind == 'claim':
        order, zone = action.get('order_id'), action.get('destination_zone')
        if not isinstance(order, str) or not isinstance(zone, str):
            return Plan(None, rejected_reason='BAD_CLAIM')
        return Plan('deliver', (order, zone))
    if kind == 'continue':
        return Plan(None)
    if kind == 'wait':
        return Plan('abort', ('wait_requested',)) if job else Plan('hold', (WAIT_HOLD_S,))
    if kind == 'release':
        order = action.get('order_id')
        if job and job.get('kind') == 'deliver' and job.get('order_id') == order:
            return Plan('abort', ('release_requested',))
        return Plan(None, rejected_reason='NO_ACTIVE_JOB_FOR_ORDER')
    return Plan(None, rejected_reason='UNSUPPORTED_ACTION')


# ---------------------------------------------------------------------------
# One robot as the study layer sees it

@dataclass(frozen=True)
class OwnFrame:
    index: int
    t: float
    jpeg: bytes
    sha256: str


class RobotLink(Protocol):
    """ONE robot: its own frames, its own executor state and its own job API."""

    robot_id: str

    def frame_at(self, t: float) -> OwnFrame | None: ...
    def belief(self) -> dict: ...
    def job(self) -> dict | None: ...
    def call(self, api: str, *args) -> dict: ...
    def clock(self) -> float: ...


class _NoStoredFrames:
    """The integrated trial never reads the offline stored-frame library."""

    def pick(self, robot_id, index):
        raise AssertionError('IntegratedTrial reads live own robot_cam frames, never stored ones')

    def manifest(self):
        return {'frame_dir': None, 'note': 'live own robot_cam frames (RobotLink.frame_at)'}


class _LiveTransport(zo._FixtureTransport):
    """Snapshot the caller's own inputs at submit (= call start), reply offline."""

    def submit(self, call):
        self.trial.snapshot(call)
        return super().submit(call)


class IntegratedTrial(zo.OfflineTrial):
    """OfflineTrial whose inputs come from live robots and whose actions reach executors."""

    def __init__(self, scenario, *, condition, seed, links, horizon_s, code_sha='unknown', map_bundle=None,
                 cost_params=None, policy=None, actor=FIXTURE_ACTOR, pose_label=None):
        check_actor(actor)
        if condition not in MAIN_CONDITIONS:
            raise ContractViolation(f'{condition!r} is not a main condition {MAIN_CONDITIONS}; the reference '
                                    'commander R is not wired into this runner')
        if sorted(links) != sorted(ROBOTS) or any(links[r].robot_id != r for r in links):
            raise ContractViolation(f'links must be exactly one own RobotLink per robot {ROBOTS}')
        super().__init__(scenario, condition=condition, seed=seed, map_bundle=map_bundle,
                         cost_params=cost_params, policy=policy, library=_NoStoredFrames(),
                         horizon_s=horizon_s, run_id=None, code_sha=code_sha)
        self.provenance = provenance(source=self.source, code_sha=code_sha,
                                     execution_bundle_id=EXECUTION_BUNDLE_ID, model=zo.FIXTURE_MODEL,
                                     provider=None, model_settings_sha256=None,
                                     prompt_template_sha256=digest(pk.PROMPT_VERSION),
                                     cost_profile_id=self.params.version)
        self.transport = self.scheduler.transport = _LiveTransport(self)
        self.links = dict(links)
        self.pose_label = dict(pose_label or {})
        self.pair_status = PairStatusBus(self.sheet)
        self._snapshots, self._jobs = {}, {}
        self.request_images: dict[str, bytes] = {}
        self.input_log, self.dispatch_log, self.executor_events = [], [], []
        self.clock_drift_s = 0.0

    # -- clock --------------------------------------------------------------
    def begin(self, t0_s):
        self.scheduler.run(until_s=t0_s, close_at_horizon=False)
        for actor in self.actors:
            self.scheduler.trigger(actor, 'start', at=t0_s)
        self.scheduler.arm_observations(period_s=1.)
        return self.step_to(t0_s)

    def step_to(self, t_s):
        """Process every scheduler event up to the physics time ``t_s`` (a chunk boundary)."""
        return self.scheduler.run(until_s=t_s, close_at_horizon=False)

    def on_executor_event(self, event, *, at_s):
        """One OWN executor event of ``event['robot_id']``: own history state + own wake only."""
        rid = event.get('robot_id') if isinstance(event, Mapping) else None
        if rid not in self.actors or event.get('event') not in EXECUTOR_EVENTS:
            raise ContractViolation(f'not an own executor event of a robot of this trial: {event!r:.200}')
        self.executor_events.append(copy.deepcopy(dict(event)))
        if event['event'] in ('job_done', 'job_failed'):
            entry = self._jobs.pop(event.get('job_id'), None)
            if entry is not None:
                reason = str((event.get('detail') or {}).get('reason', ''))
                entry['local_state'] = 'local_timeout' if reason.startswith('LOCAL_TIMEOUT') else 'queue_empty'
        trigger = event.get('scheduler_trigger')
        if trigger is not None and at_s <= self.horizon_s + 1e-9:
            self.scheduler.trigger(rid, trigger, at=at_s)

    def quiescent(self) -> bool:
        """No further decision can happen: every budget spent, nobody thinking, every executor idle."""
        spent = all(self.scheduler.metrics[a]['calls'] >= self.policy.max_calls_per_actor for a in self.actors) \
            or self.scheduler.budget.remaining() == 0
        return spent and not self.scheduler.holding() and all(self.links[a].job() is None for a in self.actors)

    def finish(self, t_end_s) -> zo.TrialResult:
        report = self.scheduler.run(until_s=t_end_s)
        self._collect()
        return zo.TrialResult(run_id=self.run_id, condition=self.condition, scenario_id=self.scenario_id,
                              seed=self.seed, leader_id=self.leader_id, calls=self.calls, messages=self.messages,
                              actions=self.actions, requests=self.requests, trace=self.scheduler.trace(),
                              report=report.to_dict(), channel=self.channel_summary(), cost=self.cost_summary(),
                              end_reason='budget_exhausted' if any(self.scheduler.metrics[a]['budget_refused']
                                                                   for a in self.actors) else 'sim_horizon')

    # -- inputs ---------------------------------------------------------------
    def snapshot(self, call):
        """The caller's OWN inputs at the call start (physics time == scheduler time)."""
        actor, t = call.actor, float(call.started_sim_s)
        link = self.links[actor]
        self.clock_drift_s = max(self.clock_drift_s, abs(link.clock() - t))
        frame = link.frame_at(t)
        if frame is None:
            raise ContractViolation(f'{actor} has no own robot_cam frame at or before {t}')
        snap = {'frame': frame, 'belief': link.belief(),
                'history': [copy.deepcopy(e) for e in self._history[actor] if e['issued_at_sim_s'] <= t + 1e-9],
                'inbox': list(self.channel.inbox(actor, now_sim_s=t)) if self.spec.channel_open else None}
        self._snapshots[(actor, round(t, 6))] = snap

    def build_inputs(self, actor, *, sim_time_s, request_id):
        snap = self._snapshots.pop((actor, round(float(sim_time_s), 6)), None)
        if snap is None:
            raise ContractViolation(f'no own-input snapshot of {actor} at {sim_time_s}')
        frame = snap['frame']
        kw = {'own_rgb_refs': [own_rgb_ref(actor, frame.index, frame.t, frame.sha256)],
              'own_command_history': snap['history'], 'self_belief': snap['belief']}
        if self.spec.channel_open:
            kw['inbox'] = snap['inbox']
        payload = build_call_input(robot_id=actor, condition_name=self.condition, request_id=request_id,
                                   sim_time_s=sim_time_s, static_map=self.static_map, source=self.source,
                                   seed=self.seed, **kw)
        self.request_images[frame.sha256] = frame.jpeg
        self.input_log.append({'request_id': request_id, 'robot': actor, 'sim_s': sim_time_s,
                               'frame_index': frame.index, 'frame_t': frame.t, 'frame_sha256': frame.sha256,
                               'history_entries': len(snap['history']),
                               'inbox_ids': [m['message_id'] for m in snap['inbox'] or ()]})
        return pk.StudyInputs(payload=payload, wrist_jpeg=frame.jpeg, seed=self.seed, pinned=self.source.pinned)

    # -- actions ------------------------------------------------------------
    def _on_action(self, actor, action, sim_s):
        """A released action reaches THIS robot's executor at its charged SIM time."""
        call_id = self.scheduler.calls[-1].call_id
        extra = getattr(self, '_pending', {}).get(call_id)
        if extra is None or self.scheduler.calls[-1].actor != actor:
            raise AssertionError(f'released action of {actor} has no recorded call')
        link = self.links[actor]
        plan = executor_plan(action, link.job())
        kind, arguments, order_id, role = zo._action_row(action)
        ack = link.call(plan.api, *plan.args) if plan.api else None
        self.dispatch_log.append({'call_id': call_id, 'actor': actor, 'sim_s': sim_s, 'action': action,
                                  'api': plan.api, 'args': list(plan.args), 'ack': ack,
                                  'rejected_reason': plan.rejected_reason})
        accepted = ack['accepted'] if ack else plan.rejected_reason is None
        reason = ack['rejected_reason'] if ack else plan.rejected_reason
        local = ack['local_state'] if ack else ('command_rejected' if reason else
                                                'command_issued' if link.job() else 'queue_empty')
        self.actions.append(action_log_record(
            run_id=self.run_id, condition_name=self.condition, seed=self.seed, actor=actor,
            action_id=extra['action_id'], request_id=extra['request_id'], submitted_at_sim_s=sim_s, kind=kind,
            arguments=arguments, accepted=accepted, order_id=order_id, role=role, rejected_reason=reason,
            local_state=local))
        if ack or reason:
            self._remember_command(actor, call_id, sim_s, plan, ack, kind, arguments, local)
        self._arm_reask(actor, sim_s)

    def _remember_command(self, actor, call_id, sim_s, plan, ack, kind, arguments, local):
        """Own command history: what this robot issued and its own command state."""
        if ack:
            kind = API_TO_ACTION_KIND[plan.api]
            arguments = {k: v for k, v in ack['arguments'].items() if k in COMMAND_ARGUMENT_KEYS}
        entry = command_entry(f'cmd_{call_id.replace("-", "_")}', sim_s, kind, arguments, local_state=local)
        self._history[actor].append(entry)
        if ack and ack['accepted'] and ack.get('job_id'):
            self._jobs[ack['job_id']] = entry

    def _arm_reask(self, actor, sim_s):
        """Own timer: idle re-ask when the own executor is idle, busy re-ask while it runs."""
        if self.scheduler.metrics[actor]['calls'] >= self.policy.max_calls_per_actor:
            return
        busy = self.links[actor].job() is not None
        at = sim_s + (self.policy.busy_reask_s if busy else self.policy.idle_reask_s)
        if at <= self.horizon_s:
            self.scheduler.timer(actor, 'timer' if busy else 'idle', at=at)

    # -- records --------------------------------------------------------------
    def wakeups(self, actor) -> list:
        """(sim_s, trigger) of every call start of ``actor`` (for the isolation audit)."""
        return [(row['sim_s'], row['line'].split()[3]) for row in self.scheduler.events
                if row.get('kind') == 'call_start' and row.get('actor') == actor]

    def study_config(self) -> dict:
        """Everything the study layer fixes for this trial (hashed into the run bundle)."""
        policy = {k: getattr(self.policy, k) for k in self.policy.__dataclass_fields__}
        return {'schema': INTEGRATION_SCHEMA, 'execution_bundle_id': EXECUTION_BUNDLE_ID,
                'condition': self.condition, 'topology': self.spec.topology, 'encoding': self.spec.encoding,
                'leader_id': self.leader_id, 'seed': self.seed, 'actor': FIXTURE_ACTOR,
                'planned_model': dict(PLANNED_MODEL), 'prompt_version': pk.PROMPT_VERSION,
                'cost_params': {'version': self.params.version, 'digest': self.params.digest()},
                'call_policy': policy, 'quantum_s': QUANTUM_S, 'think_hold_policy': THINK_HOLD_POLICY,
                'action_map': {'version': ACTION_MAP_VERSION, 'wait_hold_s': WAIT_HOLD_S},
                'pair_status': self.pair_status.config(), 'pair_status_sha256': self.pair_status.config_sha256(),
                'order_sheet_sha256': self.source.sha256, 'pose_provider': dict(self.pose_label),
                'inter_robot_channels': (['dialogue'] if self.spec.channel_open else []) + ['pair_status']}


def condition_invariant_config(config) -> dict:
    """The part of ``study_config`` that must be identical in all four conditions."""
    return {k: v for k, v in config.items()
            if k not in ('condition', 'topology', 'encoding', 'leader_id', 'inter_robot_channels')}
