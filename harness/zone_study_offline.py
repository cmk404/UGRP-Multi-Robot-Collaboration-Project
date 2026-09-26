"""no-LLM offline end-to-end loop of the Korean-dialogue zone study.

This is the FIRST gate of the study (design section 10): stored RGB references
plus a fake clock, no physics and no model call. It wires the five packages
together and proves the wiring only::

    scenario config (E)  -> harness.zone_study_scenarios
    per-call inputs (A)  -> harness.zone_study_inputs.build_call_input
    prompt + protocol(C) -> harness.zone_study_prompts_ko / zone_study_protocol
    SIM cost + clock (D) -> harness.zone_sim_cost / zone_event_scheduler
    logs (A)             -> call / message / action records
    evaluation (I)       -> harness.zone_study_eval

What it does NOT show: language understanding, any communication effect, any
physical transport success. A fixture reply is an explicit offline stand-in, never
a fallback for a model failure.

Boundary rules this module keeps, so the gate is meaningful:

* :class:`FixtureActor.respond` receives ONE argument: the model request package C
  built. It never sees the host, the scheduler, the referee, the ground-truth
  delivery list or the scenario's hidden-event schedule. Its whole decision comes
  from the request payload, which package A validated.
* package C's ``Transport`` is the only message authority (topology, encoding,
  recipients, window caps); package D's ``EventScheduler`` is the only clock and
  cost authority. The two delivery delays are the SAME number by construction.
* the evaluation record is built AFTER the loop finished, from the logs. Nothing
  the evaluator computes is ever fed back into a robot input or a call trigger.
* wrist RGB is a REFERENCE to a stored frame plus its sha256. No perception runs
  here, so a reference is not a claim about what the robot can see.
"""
from __future__ import annotations

import copy
import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

from harness import zone_study_prompts_ko as pk
from harness import zone_study_protocol as zp
from harness.zone_event_scheduler import CallPolicy, CallReply, EventScheduler, Message
from harness.zone_sim_cost import (Attempt, call_cost, censored_call_record, contract_call_record,
                                   delivery_delay_s, params)
from harness.zone_study_contract import (COMMANDER, ROBOTS, ZONE_IDS, ContractViolation,
                                         condition as contract_condition, digest, leader_for_seed)
from harness.zone_study_eval import TRIAL_SCHEMA
from harness.zone_study_inputs import (OrderSheetSource, action_log_record, belief_skeleton,
                                       build_call_input, command_entry, message_log_record,
                                       own_rgb_ref, provenance, static_map_for_call, vocabulary)
from harness.zone_study_scenarios import bundle_for, load as load_scenario

OFFLINE_VERSION = 'ugrp.zone_study_offline.v1'
#: The single owner of the message bus: package C validates and mints the
#: canonical envelope id, the SIM scheduler decides WHEN an inbox changes
#: (2026-09-26 review finding 2).
BUS_OWNER = 'sim_scheduler'
EXECUTION_BUNDLE_ID = 'zone_study_offline_v1'
FIXTURE_MODEL = 'none-fixture-v1'
#: Stored wrist frames referenced by the payloads. Real robot frames with real
#: byte hashes; this loop runs no perception on them.
FRAME_DIR = Path(__file__).resolve().parents[1] / 'tests' / 'fixtures' / 'markerless_box' / 'blue_floor_release'
#: SIM seconds the loop runs at most, and the referee horizon of the trial record.
DEFAULT_HORIZON_S = 300.0
#: Output token counts of a fixture call. Fixed so the SIM cost is reproducible
#: and is NOT a measurement of any model's real token use. Input tokens are the
#: frozen-tokenizer count of the actual request under package C's
#: ``FIXED_PROMPT_POLICY`` (second review, finding 18): the fixed system prompt is
#: billed at one size for every main condition, the payload/inbox as counted.
FIXTURE_OUTPUT_TOKENS_BASE = 40
FIXTURE_OUTPUT_TOKENS_PER_MESSAGE = 30


def frames() -> list[tuple[str, bytes]]:
    """(name, bytes) of the stored wrist frames, sorted. Raises if none exist."""
    found = sorted(FRAME_DIR.glob('*-wrist.jpg'))
    if not found:
        raise FileNotFoundError(f'no stored wrist frames under {FRAME_DIR}')
    return [(p.name, p.read_bytes()) for p in found]


class FrameLibrary:
    """Stored wrist frames addressed deterministically by (robot, call index)."""

    def __init__(self, entries=None):
        self.entries = list(entries or frames())
        self.sha256 = {name: hashlib.sha256(data).hexdigest() for name, data in self.entries}

    def pick(self, robot_id: str, index: int) -> tuple[str, bytes, str]:
        slot = (ROBOTS.index(robot_id) * 3 + index) % len(self.entries) if robot_id in ROBOTS \
            else index % len(self.entries)
        name, data = self.entries[slot]
        return name, data, self.sha256[name]

    def manifest(self) -> dict:
        return {'frame_dir': str(FRAME_DIR.relative_to(FRAME_DIR.parents[3])),
                'frames': {name: self.sha256[name] for name, _ in self.entries}}


# ---------------------------------------------------------------------------
# The no-LLM actor

@dataclass
class FixtureActor:
    """Deterministic stand-in for one LLM actor.

    ``respond`` takes ONE argument, the model request package C built, and returns
    the raw JSON reply string a model would return. It holds no reference to the
    host, the scheduler, the referee or the hidden-event schedule, so a fixture
    "success" cannot come from privileged information.
    """

    actor: str
    condition: str
    seed: int | None = None
    calls: int = 0

    def respond(self, request) -> str:
        """One reply, decided from ``request`` alone."""
        payload = json.loads(request['messages'][1]['content'])
        self.calls += 1
        channel = payload['channel']
        if payload['robot_id'] == COMMANDER:
            return json.dumps(self._commander(payload), ensure_ascii=False)
        return json.dumps(self._robot(payload, channel), ensure_ascii=False)

    # -- robots ------------------------------------------------------------
    def _target_order(self, payload):
        """Which order this robot works on: own id and own command history only."""
        orders = payload['order_sheet']['orders']
        rank = ROBOTS.index(payload['robot_id'])
        return orders[(rank + len(payload.get('own_command_history', ()))) % len(orders)]

    def _role(self, payload, order):
        roles = (payload['order_sheet'].get('kinds') or {}).get(order['kind'], {}).get('roles') or ['any']
        return roles[ROBOTS.index(payload['robot_id']) % len(roles)]

    def _robot(self, payload, channel):
        order = self._target_order(payload)
        role = self._role(payload, order)
        history = list(payload.get('own_command_history', ()))
        claimed = {e.get('arguments', {}).get('order_id') for e in history}
        sources = ['static_map', 'order_sheet', 'own_rgb', 'own_commands']
        if payload.get('inbox'):
            sources.append('message')
        action = ({'kind': 'continue'} if order['order_id'] in claimed else
                  {'kind': 'claim', 'order_id': order['order_id'], 'role': role,
                   'destination_zone': order['destination_zone']})
        return {'request_id': payload['request_id'], 'action': action,
                'decision_sources': sources,
                'messages': self._messages(payload, channel, order, role)}

    def _messages(self, payload, channel, order, role):
        recipients = list(channel['can_send_to'])
        if not recipients or payload.get('own_command_history'):
            return []                                  # one announcement per actor, then silence
        if channel['encoding'] == 'free_ko':
            if channel['role'] == 'leader':            # hub-and-spoke: one spoke per message
                return [{'recipients': [rid], 'reply_to': None,
                         'text': f'{rid}은 {order["order_id"]}을 {order["destination_zone"]} 구역으로 '
                                 f'{role} 역할로 운반하십시오.'} for rid in recipients]
            return [{'recipients': recipients, 'reply_to': None,
                     'text': f'{order["order_id"]}을 {role} 역할로 맡겠습니다. '
                             f'{order["destination_zone"]} 구역으로 갑니다.'}]
        return [{'recipients': recipients, 'reply_to': None,
                 'message': {'act': 'propose', 'item': order['order_id'],
                             'zone': order['destination_zone'], 'role': role, 'passage': None,
                             'location_ref': order['initial_location']['pickup_bay'],
                             'state': 'unknown', 'confidence': 'low',
                             'observed_at_sim_s': payload['sim_time_s'], 'reply_to': None}}]

    # -- reference_R commander --------------------------------------------
    def _commander(self, payload):
        orders = payload['order_sheet']['orders']
        issued = {e.get('order_ref') for e in payload.get('issued_orders', ())}
        assignments = {}
        for rank, rid in enumerate(ROBOTS):
            order = orders[rank % len(orders)]
            if order['order_id'] in issued:
                assignments[rid] = None
                continue
            roles = (payload['order_sheet'].get('kinds') or {}).get(order['kind'], {}).get('roles') or ['any']
            assignments[rid] = {'order_id': order['order_id'], 'role': roles[rank % len(roles)],
                                'destination_zone': order['destination_zone']}
        return {'request_id': payload['request_id'],
                'action': {'kind': 'order', 'assignments': assignments},
                'decision_sources': ['static_map', 'order_sheet', 'own_rgb'], 'messages': []}


# ---------------------------------------------------------------------------
# One trial

@dataclass
class TrialResult:
    run_id: str
    condition: str
    scenario_id: str
    seed: int
    leader_id: str | None
    calls: list = field(default_factory=list)
    messages: list = field(default_factory=list)
    actions: list = field(default_factory=list)
    requests: list = field(default_factory=list)
    trace: tuple = ()
    report: dict = field(default_factory=dict)
    channel: dict = field(default_factory=dict)
    cost: dict = field(default_factory=dict)
    #: Why the loop stopped. No physics runs here, so no order is ever delivered
    #: and a trial is never a success: ``budget_exhausted`` when an actor's call
    #: budget refused a call, otherwise ``sim_horizon``.
    end_reason: str = 'sim_horizon'

    def trial_record(self, *, horizon_s=DEFAULT_HORIZON_S, provenance_row=None, literals=()) -> dict:
        """The package I trial record (``ugrp.zone_study_trial.v1``).

        The referee sub-log is EMPTY on purpose: this loop runs no physics, so
        there is no ground truth to compare a claim against. Filling it with the
        fixture's own claims would make the evaluation circular.
        """
        end = self.report.get('sim_s') if self.report.get('sim_s') is not None else \
            max([c['released_at_sim_s'] for c in self.calls] or [0.])
        value = {'schema': TRIAL_SCHEMA, 'trial_id': self.run_id, 'condition': self.condition,
                 'scenario': self.scenario_id, 'seed': self.seed, 'robots': list(ROBOTS),
                 't0_sim_s': 0.0, 'end_sim_s': end, 'end_reason': self.end_reason,
                 'budget': {'sim_horizon_s': horizon_s, 'http_attempts': self.cost['http_attempts']},
                 'orders': [copy.deepcopy(o) for o in self.cost['orders']],
                 'calls': [copy.deepcopy(c) for c in self.calls],
                 'messages': [copy.deepcopy(m) for m in self.messages],
                 'actions': [copy.deepcopy(a) for a in self.actions],
                 # the payload keys and the final-request digest of every call, for
                 # package I's input-boundary audit (second review, finding 1)
                 'request_archive': [{k: r[k] for k in ('request_id', 'input_keys', 'input_sha256',
                                                        'request_sha256')}
                                     for r in self.requests],
                 'literals': list(literals),
                 'idle': {rid: {'thinking': self.cost['thinking_sim_s'].get(rid, 0.)} for rid in ROBOTS},
                 'referee': {},
                 'model': {'logical_calls': len(self.calls),
                           'http_attempts': self.cost['http_attempts'],
                           'censored_calls': self.cost['censored_calls'],
                           'tokens': {'input': self.cost['input_tokens'],
                                      'output': self.cost['output_tokens'], 'image': 0, 'cached': 0},
                           'sim_cost_s': {'call': self.cost['call_sim_s'],
                                          'think': self.cost['think_sim_s'],
                                          'talk': self.cost['talk_sim_s'],
                                          'delivery': self.cost['delivery_sim_s'],
                                          'censored_elapsed': self.cost['censored_elapsed_sim_s']},
                           'wall_latency_ms': []},
                 'provenance': dict(provenance_row or {})}
        if self.leader_id:
            value['leader_id'] = self.leader_id
        return value


class _FixtureTransport:
    """Scheduler transport that runs the whole per-call pipeline offline.

    ``submit`` only records the call. ``reply`` builds the package A payload and
    the package C request, asks the fixture actor, validates the reply with C,
    relays the accepted messages through C's ``Transport`` at the call's charged
    RELEASE time, and returns the costed attempts to the scheduler.

    The release time is computed with the SAME ``call_cost`` and parameters the
    scheduler uses, so the message a robot sends enters the channel exactly when
    the scheduler charges the call.
    """

    def __init__(self, trial: 'OfflineTrial'):
        self.trial = trial
        self.submitted, self.resolved = [], []

    def submit(self, call):
        self.submitted.append(call.call_id)
        return call

    def reply(self, token):
        self.resolved.append(token.call_id)
        return self.trial.run_call(token)


class OfflineTrial:
    """One condition x scenario x seed trial of the offline smoke."""

    def __init__(self, scenario, *, condition, seed, map_bundle=None, cost_params=None, policy=None,
                 library=None, horizon_s=DEFAULT_HORIZON_S, run_id=None, code_sha='unknown'):
        self.spec = zp.spec(condition)
        self.condition = condition
        self.seed = int(seed)
        self.params = cost_params or params()
        self.library = library or FrameLibrary()
        self.horizon_s = float(horizon_s)
        self.code_sha = code_sha
        self.bundle = map_bundle or bundle_for(scenario)
        self.source = OrderSheetSource(scenario, self.bundle)
        self.scenario_id = self.source.scenario_id
        self.static_map = static_map_for_call(self.bundle)
        self.sheet = self.source.sheet()
        self.leader_id = leader_for_seed(condition, self.seed) if self.spec.rotating_leader else None
        self.actors = (COMMANDER,) if self.spec.commander_llm else ROBOTS
        self.run_id = run_id or f'{condition}-{self.scenario_id}-s{self.seed}'
        self.provenance = provenance(source=self.source, code_sha=code_sha,
                                     execution_bundle_id=EXECUTION_BUNDLE_ID, model=FIXTURE_MODEL,
                                     provider=None, model_settings_sha256=None,
                                     prompt_template_sha256=digest(pk.PROMPT_VERSION),
                                     cost_profile_id=self.params.version)
        # package C owns message VALIDATION and the canonical envelope id; the SIM
        # scheduler owns DELIVERY, so there is exactly one inbox and one clock
        # (2026-09-26 review finding 2).
        self.channel = zp.Transport(condition, seed=self.seed,
                                    vocabulary=self.source and _vocabulary(self.sheet, self.bundle),
                                    delivery_owner=BUS_OWNER,
                                    delivery_delay_sim_s=delivery_delay_s(1, self.params))
        self.channel.open_window('w1', at_sim_s=0.)
        self.fixtures = {actor: FixtureActor(actor, condition, self.seed) for actor in self.actors}
        self.transport = _FixtureTransport(self)
        self.policy = policy or CallPolicy()
        self.scheduler = EventScheduler(self.transport, cost_params=self.params,
                                        policy=self.policy, actors=self.actors,
                                        on_action=self._on_action,
                                        bus=self.channel, bus_owner=BUS_OWNER)
        self.calls, self.messages, self.actions, self.requests = [], [], [], []
        self.envelopes = {}
        self._history, self._issued = {actor: [] for actor in self.actors}, []
        self._call_index = {actor: 0 for actor in self.actors}
        self._observations = {actor: 0 for actor in self.actors}

    # -- per-call pipeline -------------------------------------------------
    def allowlist(self) -> frozenset:
        """Package A's input allowlist of this condition (the payload key set)."""
        return contract_condition(self.condition).input_allowlist

    def build_inputs(self, actor, *, sim_time_s, request_id):
        """Package A payload plus the stored frames it references (package C bundle)."""
        if actor not in self.actors:
            raise ContractViolation(f'{actor!r} is not an LLM actor of {self.condition} '
                                    f'(actors: {self.actors})')
        index = self._observations[actor]
        self._observations[actor] = index + 1
        kw, views, wrist = {}, None, None
        allow = self.spec
        if actor == COMMANDER:
            refs, views = [], {}
            for rid in ROBOTS:
                name, data, sha = self.library.pick(rid, index)
                refs.append(own_rgb_ref(rid, index + 1, sim_time_s, sha))
                views[rid] = data
            kw['team_rgb_refs'] = refs
            kw['issued_orders'] = list(self._issued)
        else:
            name, wrist, sha = self.library.pick(actor, index)
            kw['own_rgb_refs'] = [own_rgb_ref(actor, index + 1, sim_time_s, sha)]
            kw['own_command_history'] = list(self._history[actor])
            kw['self_belief'] = belief_skeleton()
        if allow.channel_open:
            kw['inbox'] = list(self.channel.inbox(actor, now_sim_s=sim_time_s))
        payload = build_call_input(robot_id=actor, condition_name=self.condition,
                                   request_id=request_id, sim_time_s=sim_time_s,
                                   static_map=self.static_map, source=self.source,
                                   seed=self.seed, **kw)
        return pk.StudyInputs(payload=payload, wrist_jpeg=wrist, robot_views=views, seed=self.seed,
                              pinned=self.source.pinned)

    def run_call(self, call) -> CallReply:
        """Inputs -> prompt -> fixture reply -> validation -> relay -> costed attempts."""
        actor = call.actor
        request_id = f'req_{call.call_id.replace("-", "_")}'
        bundled = self.build_inputs(actor, sim_time_s=call.started_sim_s, request_id=request_id)
        window = self.channel.window_context(actor, now_sim_s=call.started_sim_s) \
            if self.spec.channel_open else None
        request = pk.build_request(bundled, window=window)
        raw = self.fixtures[actor].respond(request)
        input_tokens = request['billed_tokens']['total_text_billed']
        try:
            value = zp.validate_reply(raw, request_id=request_id, condition=self.condition, actor=actor,
                                      order_ids=bundled.order_ids(), item_ids=bundled.item_ids(),
                                      roles_by_order=bundled.roles_by_order(),
                                      vocabulary=bundled.vocabulary(),
                                      passages=bundled.passages(), location_refs=bundled.location_refs(),
                                      robots=ROBOTS)
        except zp.ProtocolError as exc:
            # Second review, finding 6: a malformed reply is still a reply the
            # model GENERATED. Its tokens and every utterance it contained are
            # billed; nothing of it runs or is delivered.
            produced = generated_utterances(raw)
            attempts = (Attempt(outcome='invalid', input_tokens=input_tokens,
                                output_tokens=pk.count_tokens(raw if isinstance(raw, str) else json.dumps(raw)),
                                utterances=produced),)
            self._archive(call, bundled, request, status='invalid_json', messages_out=0,
                          unparsed_utterances=produced, error=str(exc))
            return CallReply(attempts=attempts, action=None, messages=(), unparsed_utterances=produced)
        # Every utterance the model produced is billed, accepted or not
        # (review finding 6).
        utterances = len(value['messages'])
        attempts = (Attempt(outcome='ok', input_tokens=input_tokens,
                            output_tokens=FIXTURE_OUTPUT_TOKENS_BASE
                            + FIXTURE_OUTPUT_TOKENS_PER_MESSAGE * utterances,
                            utterances=utterances),)
        cost = call_cost(attempts, self.params)
        release = round(call.started_sim_s + cost.sim_s, 6)
        receipts = zp.relay(self.channel, actor, value, at_sim_s=release) if value['messages'] else ()
        messages = []
        for index, receipt in enumerate(receipts):
            if not receipt.accepted:
                # billed, never delivered: the scheduler records the rejection
                messages.append(Message(sender=actor,
                                        recipients=tuple(value['messages'][index]['recipients']),
                                        body=None, encoding=self.spec.encoding,
                                        message_id=f'{call.call_id}-rejected-{index + 1}',
                                        rejection=receipt.rejection or 'rejected'))
                continue
            envelope = receipt.envelope
            # ONE canonical id and ONE body from package C all the way into the
            # SIM inbox and the log (second review, finding 2)
            self.envelopes[envelope.message_id] = envelope.record()
            messages.append(Message(sender=actor, recipients=tuple(envelope.recipients),
                                    body=envelope.body(), encoding=self.spec.encoding,
                                    message_id=envelope.message_id, reply_to=envelope.reply_to))
        self._record(call, bundled, value, release, request)
        return CallReply(attempts=attempts, action=value['action'], messages=tuple(messages))

    def _archive(self, call, bundled, request, *, status, messages_out, unparsed_utterances=0, error=None):
        """Keep the FINAL request of this call (second review, finding 1)."""
        row = pk.archive_request(request)
        row.update({'call_id': call.call_id, 'robot': call.actor, 'sim_s': call.started_sim_s,
                    'payload_validated': True, 'status': status,
                    'input_keys': sorted(bundled.payload),
                    'images': [image['label'] for image in request['images']],
                    'messages_out': messages_out, 'unparsed_utterances': unparsed_utterances})
        if error is not None:
            row['error'] = error
        self.requests.append(row)
        return row

    def _record(self, call, bundled, value, release, request):
        """Remember what this call WOULD do; it is logged only when it runs."""
        action_id = f'act-{call.call_id}'
        self._archive(call, bundled, request, status='ok', messages_out=len(value['messages']))
        self._pending = getattr(self, '_pending', {})
        self._pending[call.call_id] = {
            'request_id': bundled.request_id, 'input_sha256': bundled.payload_sha256,
            'action_id': action_id, 'decision_sources': list(value['decision_sources']),
            'action': value['action'], 'release': release}

    def _on_action(self, actor, action, sim_s):
        """The scheduler released an action at its charged SIM time: log it now.

        Second review: the action row and the own command history used to be
        written when the reply was FETCHED, so a call that was censored at the
        horizon (or discarded) still left an accepted action and a command the
        robot never issued.
        """
        call_id = self.scheduler.calls[-1].call_id
        extra = getattr(self, '_pending', {}).get(call_id)
        if extra is None or self.scheduler.calls[-1].actor != actor:
            raise AssertionError(f'released action of {actor} has no recorded call')
        kind, arguments, order_id, role = _action_row(action)
        self.actions.append(action_log_record(
            run_id=self.run_id, condition_name=self.condition, seed=self.seed, actor=actor,
            action_id=extra['action_id'], request_id=extra['request_id'], submitted_at_sim_s=sim_s,
            kind=kind, arguments=arguments, accepted=True, order_id=order_id, role=role))
        if actor == COMMANDER:
            for rid, job in (action.get('assignments') or {}).items():
                if job:
                    self._issued.append({'order_ref': job['order_id'], 'to': rid,
                                         'at_sim_s': sim_s, 'instruction': job['role']})
        elif kind == 'claim_order':
            self._history[actor].append(command_entry(
                f'cmd_{call_id.replace("-", "_")}', sim_s, 'goto',
                {'order_id': order_id, 'role': role, 'target_zone': arguments.get('target_zone')}))
        self._arm_idle_reask(actor, action, sim_s)

    # -- the loop ----------------------------------------------------------
    def _arm_idle_reask(self, actor, action, sim_s):
        """Arm this actor's own idle timer (``CallPolicy.idle_reask_s``).

        A LOCAL timer, the study's own call trigger: never a peer's job end, a
        teacher receipt or global progress. Stops once the actor reached its call
        budget or the timer would fire past the horizon.
        """
        del action
        if self.scheduler.metrics[actor]['calls'] >= self.policy.max_calls_per_actor:
            return
        at = sim_s + self.policy.idle_reask_s
        if at <= self.horizon_s:
            self.scheduler.timer(actor, 'idle', at=at)

    def run(self) -> TrialResult:
        for actor in self.actors:
            self.scheduler.trigger(actor, 'start', at=0.)
        self.scheduler.arm_observations(period_s=1.)
        report = self.scheduler.run(until_s=self.horizon_s)
        self._collect()
        result = TrialResult(run_id=self.run_id, condition=self.condition,
                             scenario_id=self.scenario_id, seed=self.seed, leader_id=self.leader_id,
                             calls=self.calls, messages=self.messages, actions=self.actions,
                             requests=self.requests, trace=self.scheduler.trace(),
                             report=report.to_dict(), channel=self.channel_summary(),
                             cost=self.cost_summary(),
                             end_reason='budget_exhausted' if any(
                                 self.scheduler.metrics[a]['budget_refused']
                                 for a in self.actors) else 'sim_horizon')
        return result

    def _collect(self):
        pending = {row['call_id']: {'request_id': row['request_id'], 'input_sha256': row['input_sha256']}
                   for row in self.requests}
        for call_id, row in getattr(self, '_pending', {}).items():
            pending.setdefault(call_id, {}).update(row)
        rows = [(record.started_sim_s, record.actor, record.call_id, record, None)
                for record in self.scheduler.calls]
        rows += [(row['started_sim_s'], row['actor'], row['call_id'], None, row)
                 for row in self.scheduler.censored]
        rank = {actor: i for i, actor in enumerate(self.actors)}
        index = {actor: 0 for actor in self.actors}
        executed = {row['action_id'] for row in self.actions}
        for _, actor, call_id, record, censored in sorted(rows, key=lambda r: (r[0], rank[r[1]], r[2])):
            extra = pending.get(call_id, {})
            if record is not None:
                row = contract_call_record(
                    record, run_id=self.run_id, condition_name=self.condition, seed=self.seed,
                    request_id=extra.get('request_id', call_id),
                    call_index=index[actor],
                    input_sha256=extra.get('input_sha256', '0' * 64), provenance=self.provenance,
                    action_id=extra.get('action_id') if extra.get('action_id') in executed else None,
                    message_ids=[m.message_id for m in self.scheduler.messages
                                 if m.call_id == call_id],
                    decision_sources=extra.get('decision_sources', ()))
            else:
                # still in flight at the horizon: kept as censored, never dropped
                row = censored_call_record(
                    censored, run_id=self.run_id, condition_name=self.condition, seed=self.seed,
                    request_id=extra.get('request_id', call_id), call_index=index[actor],
                    input_sha256=extra.get('input_sha256', '0' * 64), provenance=self.provenance)
            self.calls.append(row)
            index[actor] += 1
        edges = {}
        for edge in self.scheduler.messages:
            edges.setdefault(edge.message_id, []).append(edge)
        for message_id in sorted(edges):
            group = sorted(edges[message_id], key=lambda e: e.recipient)
            envelope = self.envelopes[message_id]
            deliveries = [{'recipient': e.recipient, 'delivered_at_sim_s': round(e.delivered_sim_s, 6),
                           'status': 'delivered'} for e in group]
            earliest = min(d['delivered_at_sim_s'] for d in deliveries)
            self.messages.append(message_log_record(
                run_id=self.run_id, condition_name=self.condition, seed=self.seed, envelope=envelope,
                deliveries=deliveries,
                delivery_delay_s=round(earliest - envelope['created_at_sim_s'], 6),
                status='delivered'))

    # -- summaries ---------------------------------------------------------
    def channel_summary(self) -> dict:
        """Falsifiable channel-isolation facts of this trial."""
        free_text = sum(1 for m in self.messages
                        if isinstance(m['body'], dict) and isinstance(m['body'].get('text'), str)
                        and m['body']['text'].strip())
        follower_to_follower = 0
        if self.leader_id:
            follower_to_follower = sum(1 for m in self.messages if m['sender'] != self.leader_id
                                       and any(r != self.leader_id for r in m['recipients']))
        received = {actor: (self.channel.delivered_count(actor) if actor in ROBOTS else 0)
                    for actor in self.actors}
        agrees = all(received[actor] == len(self.scheduler.inbox(actor)) for actor in self.actors)
        return {'condition': self.condition, 'topology': self.spec.topology,
                'encoding': self.spec.encoding, 'leader_id': self.leader_id,
                'sent': self.channel.sent_count(), 'accepted_messages': len(self.messages),
                'rejected': len(self.channel.rejections),
                'delivery_edges': sum(len(m['deliveries']) for m in self.messages),
                'free_text_messages': free_text,
                'follower_to_follower': follower_to_follower,
                'received': received,
                'inbox_agrees_with_scheduler': agrees}

    def cost_summary(self) -> dict:
        # Censored calls carry the SIM time that elapsed until the horizon, which
        # is NOT the charged thinking cost of a completed call, so the two are
        # reported apart (review finding 16). Their API usage is real and is part
        # of the resource totals (second review).
        done = [c for c in self.calls if c['status'] != 'censored']
        censored = [c for c in self.calls if c['status'] == 'censored']
        # Second review, finding 10: the charged call cost already CONTAINS the
        # utterance term, so ``think`` is the call cost without it and ``talk``
        # is the utterance term; think + talk == call total, never more.
        call_total = sum(c['sim_cost_s'] for c in done)
        talk = sum(c['cost_terms']['gamma_s_per_utterance'] for c in done)
        delivery = sum(m['delivery_delay_s'] for m in self.messages)
        return {'orders': [copy.deepcopy(o) for o in self.sheet['orders']],
                'http_attempts': sum(c['http_attempts'] for c in self.calls),
                'input_tokens': sum(c['input_tokens']['text'] for c in self.calls),
                'output_tokens': sum(c['output_tokens'] for c in self.calls),
                'call_sim_s': round(call_total, 6),
                'think_sim_s': round(call_total - talk, 6), 'talk_sim_s': round(talk, 6),
                'delivery_sim_s': round(delivery, 6),
                'censored_calls': len(censored),
                'censored_elapsed_sim_s': round(sum(c['sim_cost_s'] for c in censored), 6),
                'censored_http_attempts': sum(c['http_attempts'] for c in censored),
                'censored_output_tokens': sum(c['output_tokens'] for c in censored),
                'thinking_sim_s': {actor: self.scheduler.metrics[actor]['thinking_sim_s']
                                   for actor in self.actors},
                'attempt_budget': self.scheduler.budget.to_dict(),
                'over_budget_attempts': [dict(r) for r in self.scheduler.over_budget_attempts],
                'rejected_messages': len(self.scheduler.rejected_messages),
                'discarded_calls': len(self.scheduler.discarded),
                'discarded_utterances': sum(r['messages'] + r['unparsed_utterances']
                                            for r in self.scheduler.discarded),
                'censored_utterances': sum(r['messages'] + r['unparsed_utterances']
                                           for r in self.scheduler.censored),
                'undelivered_messages': len(self.scheduler.undelivered()),
                'params_version': self.params.version, 'params_digest': self.params.digest()}

    def literals(self) -> tuple:
        """Tokens that stay literal in a Korean message: order/item/kind ids, grasp
        role names, passage ids, zone letters and robot ids. Declared from the
        order sheet and the public map only, never from the simulator."""
        ids = [o['order_id'] for o in self.sheet['orders']]
        ids += [i for o in self.sheet['orders'] for i in o.get('item_ids') or ()]
        ids += [o['kind'] for o in self.sheet['orders']]
        ids += [role for row in (self.sheet.get('kinds') or {}).values()
                for role in row.get('roles') or ()]
        ids += [p['id'] for p in self.bundle['public_map'].get('passages', ())]
        ids += list(ZONE_IDS) + list(ROBOTS)
        return tuple(dict.fromkeys(ids))

    def trial_record(self, result: TrialResult) -> dict:
        return result.trial_record(horizon_s=self.horizon_s, provenance_row=self.provenance,
                                  literals=self.literals())


def generated_utterances(raw) -> int:
    """How many utterances a (possibly malformed) reply GENERATED.

    A reply that parses as JSON counts the entries of its ``messages`` list,
    valid or not. Text that is not JSON counts its ``"recipients"`` keys, one per
    attempted utterance; that is a frozen, deterministic rule, not a guess about
    intent. Used so a malformed reply keeps its utterance cost (second review,
    finding 6).
    """
    value = raw
    if isinstance(raw, str):
        try:
            value = json.loads(raw)
        except ValueError:
            return raw.count('"recipients"')
    if isinstance(value, dict) and isinstance(value.get('messages'), list):
        return len(value['messages'])
    return 0


def _vocabulary(sheet, map_bundle):
    return vocabulary(sheet, map_bundle['public_map'])


_ACTION_KIND = {'claim': 'claim_order', 'continue': 'noop', 'wait': 'wait', 'release': 'release',
                'order': 'claim_order'}


def _action_row(action):
    """Model action -> (package A action kind, arguments, order_id, role)."""
    kind = _ACTION_KIND[action['kind']]
    if action['kind'] == 'claim':
        return (kind, {'order_id': action['order_id'], 'role': action['role'],
                       'target_zone': action['destination_zone']}, action['order_id'], action['role'])
    if action['kind'] == 'release':
        return ('release', {'order_id': action['order_id']}, action['order_id'], None)
    if action['kind'] == 'order':
        jobs = {rid: job for rid, job in (action.get('assignments') or {}).items() if job}
        first = next(iter(sorted(jobs)), None)
        arguments = {'order_id': jobs[first]['order_id'], 'role': jobs[first]['role'],
                     'target_zone': jobs[first]['destination_zone']} if first else {}
        return (kind, arguments, arguments.get('order_id'), arguments.get('role'))
    return (kind, {}, None, None)


# ---------------------------------------------------------------------------
# Falsifiable checks of the gate

def request_checks(result: TrialResult) -> dict:
    """Every call's FINAL request is archived and re-hashes to its digest.

    Second review, finding 1: the log kept only the payload digest, so the
    dialogue window, the system text and the image bytes that actually went out
    could not be audited afterwards.
    """
    problems = []
    for row in result.requests:
        problems.extend(pk.verify_archived_request(row))
    calls = {c['request_id'] for c in result.calls}
    archived = {r['request_id'] for r in result.requests}
    if calls - archived:
        problems.append(f'calls without an archived request: {sorted(calls - archived)[:5]}')
    return {'ok': not problems, 'problems': problems, 'archived': len(result.requests)}


def channel_checks(trial: 'OfflineTrial', result: TrialResult) -> dict:
    """Channel isolation facts of one trial (``ok`` False = the gate failed)."""
    summary = result.channel
    spec = trial.spec
    problems = []
    if not spec.channel_open and (summary['sent'] or summary['accepted_messages']
                                 or any(summary['received'].values())):
        problems.append(f'{trial.condition} has no channel but sent/received a message')
    if spec.encoding == 'schema' and summary['free_text_messages']:
        problems.append('a schema condition carried free text')
    if spec.encoding == 'free_ko' and summary['accepted_messages'] \
            and summary['free_text_messages'] != summary['accepted_messages']:
        problems.append('a free Korean condition carried a non-text body')
    if trial.leader_id and summary['follower_to_follower']:
        problems.append('leader_ko delivered a follower-to-follower message')
    if not summary['inbox_agrees_with_scheduler']:
        problems.append('package C inboxes and the package D scheduler disagree')
    for record in result.messages:
        edges = {d['recipient'] for d in record['deliveries']}
        if edges != set(record['recipients']):
            problems.append(f'{record["message_id"]} lost a recipient between send and delivery')
        if record['sender'] in record['recipients']:
            problems.append(f'{record["message_id"]} was addressed to its sender')
    return {'ok': not problems, 'problems': problems, **summary}


def cost_checks(trial: 'OfflineTrial', result: TrialResult) -> dict:
    """Cost accounting facts of one trial."""
    problems = []
    charged, censored_elapsed = 0., 0.
    for call in result.calls:
        span = round(call['released_at_sim_s'] - call['requested_at_sim_s'], 6)
        if abs(span - call['sim_cost_s']) > 1e-9:
            problems.append(f'{call["request_id"]}: released-requested != sim_cost_s')
        if call['status'] == 'censored':
            # a call still in flight at the horizon: SIM time elapsed, action
            # never released, API usage as recorded (review finding 16, second
            # review: usage is kept, not zeroed)
            censored_elapsed += call['sim_cost_s']
            if not call['cost_terms'].get('censored'):
                problems.append(f'{call["request_id"]}: a censored call must be labelled in cost_terms')
            if call['cost_terms'].get('usage_known') and not call['input_tokens']['text']:
                problems.append(f'{call["request_id"]}: a censored call with known usage lost its tokens')
            continue
        charged += call['sim_cost_s']
    by_actor = sum(result.cost['thinking_sim_s'].values())
    if abs(round(charged, 6) - round(by_actor, 6)) > 1e-6:
        problems.append(f'charged {charged} != scheduler thinking {by_actor}')
    budget = result.cost['attempt_budget']
    used = sum(c['http_attempts'] for c in result.calls)
    if budget['used_total'] != used:
        problems.append(f'attempt budget used {budget["used_total"]} != call log {used}')
    if budget['total'] is not None and budget['used_total'] > budget['total']:
        problems.append(f'HTTP attempts {budget["used_total"]} exceeded the reserved budget '
                        f'{budget["total"]}')
    for actor, count in budget['used'].items():
        if budget['per_actor'] is not None and count > budget['per_actor']:
            problems.append(f'{actor} used {count} HTTP attempts, over its {budget["per_actor"]} cap')
    expected_delay = delivery_delay_s(1, trial.params)
    for record in result.messages:
        want = delivery_delay_s(len(record['recipients']), trial.params)
        if abs(record['delivery_delay_s'] - want) > 1e-9:
            problems.append(f'{record["message_id"]}: delivery delay {record["delivery_delay_s"]} != {want}')
    if not trial.spec.channel_open and result.cost['talk_sim_s']:
        problems.append(f'{trial.condition} paid a talk cost without a channel')
    if trial.spec.channel_open and result.messages and not result.cost['talk_sim_s']:
        problems.append(f'{trial.condition} sent messages but paid no talk cost')
    if result.cost.get('over_budget_attempts'):
        problems.append(f'HTTP attempts sent without a reservation: {result.cost["over_budget_attempts"]}')
    # review finding 6: every utterance the model produced is billed — the
    # delivered log, the rejected ones, the ones of discarded (malformed) and
    # censored replies, and accepted ones still in transit at the horizon.
    billed = sum(int(c['cost_terms'].get('utterances') or 0) for c in result.calls)
    produced = (len(result.messages) + result.cost['rejected_messages'] + result.cost['discarded_utterances']
                + result.cost['censored_utterances'] + result.cost['undelivered_messages'])
    if billed != produced:
        problems.append(f'billed {billed} utterance(s) but the replies produced {produced}')
    return {'ok': not problems, 'problems': problems, 'charged_sim_s': round(charged, 6),
            'scheduler_thinking_sim_s': round(by_actor, 6),
            'censored_calls': result.cost['censored_calls'],
            'censored_elapsed_sim_s': round(censored_elapsed, 6),
            'billed_utterances': billed, 'produced_utterances': produced,
            'attempt_budget': budget,
            'talk_sim_s': result.cost['talk_sim_s'], 'delivery_sim_s': result.cost['delivery_sim_s'],
            'delivery_delay_s': expected_delay,
            'params_version': result.cost['params_version'],
            'params_digest': result.cost['params_digest']}


#: A private section a backflow probe injects: a different hidden-event schedule
#: plus a fake ground-truth delivery list. A robot that can see either of these
#: would change its behaviour and the SIM trace would differ.
PROBE_EVAL_SECTION = {
    'hidden_events': [{'kind': 'passage_blocked', 'trigger': {'kind': 'sim_time', 'at_sim_s': 5.0},
                       'passage': 'door_narrow'}],
    'referee_deliveries': [{'item_id': 'probe-item-1', 'zone': 'A', 'sim_s': 1.0, 'correct': True}],
    'notes': 'backflow probe only; never a robot input',
}


def backflow_probe(scenario_id, condition, seed, **kw) -> dict:
    """Same trial twice: once as configured, once with the PRIVATE section changed.

    The evaluation/setup section of a scenario config (hidden events) and a fake
    referee delivery list must not reach any robot, so both runs must produce the
    SAME SIM trace, the same logs and the same channel counts. A difference means
    evaluation data flowed back into an input or a call trigger.
    """
    base = load_scenario(scenario_id)
    probe = copy.deepcopy(base)
    probe['eval'] = copy.deepcopy(PROBE_EVAL_SECTION)
    out = []
    for scenario in (base, probe):
        trial = OfflineTrial(scenario, condition=condition, seed=seed, **kw)
        result = trial.run()
        out.append({'trace_sha256': digest(list(result.trace)),
                    'calls_sha256': digest(result.calls), 'messages_sha256': digest(result.messages),
                    'actions_sha256': digest(result.actions),
                    'request_sha256': digest([r['input_sha256'] for r in result.requests]),
                    'hidden_events': len(trial.source.hidden_events()),
                    'referee': trial.trial_record(result)['referee']})
    same = {k: out[0][k] == out[1][k] for k in ('trace_sha256', 'calls_sha256', 'messages_sha256',
                                                'actions_sha256', 'request_sha256')}
    problems = [f'{k} changed with the private section' for k, ok in same.items() if not ok]
    if out[0]['referee'] or out[1]['referee']:
        problems.append('the offline trial record must carry an empty referee sub-log')
    return {'ok': not problems, 'problems': problems, 'identical': same,
            'hidden_events': [row['hidden_events'] for row in out],
            'scenario': scenario_id, 'condition': condition, 'seed': seed}


def actor_isolation() -> dict:
    """The fixture actor's whole input is its own request (signature-level check)."""
    import inspect

    signature = inspect.signature(FixtureActor.respond)
    names = tuple(signature.parameters)
    problems = []
    if names != ('self', 'request'):
        problems.append(f'FixtureActor.respond takes {names}, expected (self, request)')
    fields = tuple(f.name for f in field_names(FixtureActor))
    allowed = ('actor', 'condition', 'seed', 'calls')
    if fields != allowed:
        problems.append(f'FixtureActor holds {fields}, expected {allowed}')
    return {'ok': not problems, 'problems': problems, 'parameters': list(names),
            'attributes': list(fields)}


def field_names(cls):
    import dataclasses

    return dataclasses.fields(cls)


# ---------------------------------------------------------------------------
# The whole smoke

def run_trial(scenario_id, condition, seed, **kw) -> tuple[OfflineTrial, TrialResult]:
    """One trial from a scenario id."""
    scenario = load_scenario(scenario_id)
    trial = OfflineTrial(scenario, condition=condition, seed=seed, **kw)
    return trial, trial.run()


def run_smoke(scenario_ids, conditions, *, seeds=None, code_sha='unknown', horizon_s=DEFAULT_HORIZON_S,
              cost_params=None, library=None, probe=True) -> dict:
    """Every (scenario, condition) trial with one seed each.

    ``seeds`` maps a scenario id to the seed to use; by default the scenario's
    first declared seed is used, so the paired design stays reproducible.
    ``probe`` also re-runs each trial with a CHANGED private section and requires
    an identical SIM trace (see :func:`backflow_probe`).
    """
    library = library or FrameLibrary()
    seeds = dict(seeds or {})
    bundles, trials, records, probes = {}, [], [], []
    for scenario_id in scenario_ids:
        scenario = load_scenario(scenario_id)
        key = f'{scenario["map_id"]}:{scenario.get("landmark_detail", "full")}'
        if key not in bundles:
            bundles[key] = bundle_for(scenario)
        seed = seeds.get(scenario_id, scenario['seeds'][0])
        for condition in conditions:
            trial = OfflineTrial(scenario, condition=condition, seed=seed, map_bundle=bundles[key],
                                 cost_params=cost_params, library=library, horizon_s=horizon_s,
                                 code_sha=code_sha)
            result = trial.run()
            trials.append({'run_id': result.run_id, 'condition': condition,
                           'scenario': result.scenario_id, 'seed': seed,
                           'leader_id': result.leader_id, 'calls': len(result.calls),
                           'messages': len(result.messages), 'actions': len(result.actions),
                           'end_sim_s': result.report['sim_s'], 'end_reason': result.end_reason,
                           'channel': channel_checks(trial, result),
                           'cost': cost_checks(trial, result),
                           'requests': request_checks(result),
                           'scheduler': result.report,
                           'trace_sha256': digest(list(result.trace)),
                           'request_input_sha256': [r['input_sha256'] for r in result.requests]})
            records.append(trial.trial_record(result))
            if probe:
                probes.append(backflow_probe(scenario_id, condition, seed, map_bundle=bundles[key],
                                             cost_params=cost_params, library=library,
                                             horizon_s=horizon_s, code_sha=code_sha))
    isolation = actor_isolation()
    ok = (all(row['channel']['ok'] and row['cost']['ok'] and row['requests']['ok'] for row in trials)
          and all(row['ok'] for row in probes) and isolation['ok'])
    return {'schema': OFFLINE_VERSION, 'ok': ok, 'scenarios': list(scenario_ids),
            'conditions': list(conditions), 'trials': trials, 'trial_records': records,
            'backflow_probes': probes, 'actor_isolation': isolation,
            'frames': library.manifest(),
            'note_ko': ('프로토콜·비용·로그·평가 배선만 검증한다. 언어 이해, 통신 효과, '
                        '물리 운반 성공은 입증하지 않는다.')}
