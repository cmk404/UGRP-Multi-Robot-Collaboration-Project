"""Single SIM event queue for the zone dialogue study (package D).

The scheduler owns SIM time. Thinking and talking cost SIM seconds
(:mod:`harness.zone_sim_cost`), physics runs on through them, and the logical
order of events never depends on wall time or on the order in which HTTP
replies happen to arrive.

Contract implemented here (design: section 5 of
``docs/design/2026-09-25-zone-dialogue-study-design-codex.md``):

1. A call captures its inputs at its start time ``t``; the reply is a decision
   about ``t``, not about ``t + d``.
2. The calling actor holds its last safe command while it thinks. Other actors
   and physics keep going, so waiting is not free.
3. Neither the action nor its messages are visible before ``t + d``.
4. Messages enter an inbox at their scheduled delivery time only.
5. Concurrent calls overlap: three actors thinking at once cost roughly one
   call, not three.
6. A slow API may make the *computation* block in wall time, but never changes
   the SIM cost or the event order.
7. Same replies, seed and settings => identical SIM trace, whatever order the
   HTTP responses complete in.

How 6/7 are achieved: the reply content (and therefore the cost) is only needed
when SIM time approaches a possible completion. The loop processes every event
that is provably earlier than any in-flight completion first, using
``CostParams.min_call_s()`` as a sound lower bound, and resolves in-flight calls
in a fixed order (start time, actor, call id) rather than in arrival order.
Deliveries are ordered by ``(delivery time, origin finish time, sender, message
index, recipient)``.

Not integrated into any runner: the runner owner wires this in after PR 169.
This module holds no simulator state; ``advance`` is a caller-supplied callback
that steps physics from one SIM time to the next.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq

from harness.zone_sim_cost import (Attempt, CallCostRecord, FAILED_OUTCOMES, MessageCostRecord,
                                   TRIGGER_TO_CONTRACT, call_cost, contract_call_record,
                                   contract_message_records, delivery_delay_s, params, quantize)
from harness.zone_study_contract import CONDITIONS as CONTRACT_CONDITIONS, ROBOTS

SCHEDULER_SCHEMA = 'ugrp.zone_event_scheduler.v1'
DEFAULT_ACTORS = ROBOTS
#: Package A's message encodings (``harness.zone_study_contract``).
ENCODINGS = tuple(sorted({c.encoding for c in CONTRACT_CONDITIONS.values()} - {'none'}))

#: Call triggers. The number is the merge priority: when several triggers reach
#: one actor while it is busy they collapse into one call carrying the strongest
#: label, and the collapsed labels are recorded. Every label maps onto package
#: A's ``TRIGGERS`` enum through ``zone_sim_cost.TRIGGER_TO_CONTRACT``.
TRIGGERS = {'start': 70, 'failure': 60, 'blockage': 50, 'timeout': 40, 'report': 30,
            'retry': 25, 'idle': 20, 'timer': 10}
assert set(TRIGGERS) == set(TRIGGER_TO_CONTRACT)

#: Event kinds processed at one SIM time, in this order. Deliveries land before
#: anything else so a call starting at ``t`` sees every message delivered at
#: ``t``; a call starts last so it also sees the effects of completions at ``t``.
KIND_ORDER = {'message': 0, 'call_done': 1, 'timer': 2, 'observe': 3, 'call_start': 4}


def _round(value):
    return round(float(value), 6)


@dataclass(frozen=True)
class Message:
    """One utterance leaving one actor. Several recipients = one broadcast."""

    sender: str
    recipients: tuple
    body: object = ''
    encoding: str = 'free_ko'

    def __post_init__(self):
        if self.encoding not in ENCODINGS:
            raise ValueError(f'encoding must be one of {ENCODINGS}, got {self.encoding!r}')
        if not self.recipients:
            raise ValueError('a message needs at least one recipient')
        if self.sender in self.recipients:
            raise ValueError('a message cannot be addressed to its sender')

    @property
    def broadcast(self):
        return len(self.recipients) > 1


@dataclass(frozen=True)
class CallReply:
    """What one logical call produced: costed attempts, an action, utterances."""

    attempts: tuple = (Attempt(),)
    action: object = None
    messages: tuple = ()

    def __post_init__(self):
        if not self.attempts:
            raise ValueError('a reply needs at least one attempt')


@dataclass
class PendingCall:
    """An in-flight call: submitted to the transport, not yet costed."""

    call_id: str
    actor: str
    trigger: str
    started_sim_s: float
    token: object = None
    merged: tuple = ()
    retry_of: str = ''


@dataclass(frozen=True)
class CallPolicy:
    """Call eligibility and budget. Identical for every condition.

    Fairness is equal *eligibility and limits*, not an equal number of calls:
    an actor that talks more pays more SIM time. Start values come from the
    design document and are provisional.
    """

    min_interval_s: float = 2.
    max_outstanding_per_actor: int = 1
    idle_reask_s: float = 10.
    busy_reask_s: float = 60.
    observe_period_s: float = 1.
    trigger_on_message: bool = True
    max_retries: int = 1
    max_calls_per_actor: int = 30
    max_attempts_total: int = 90


@dataclass(frozen=True)
class RunReport:
    """Why the loop stopped and what it spent."""

    sim_s: float
    stop_reason: str
    events: int
    calls: int
    deliveries: int

    def to_dict(self):
        return {'schema': SCHEDULER_SCHEMA, 'sim_s': _round(self.sim_s), 'stop_reason': self.stop_reason,
                'events': self.events, 'calls': self.calls, 'deliveries': self.deliveries}


@dataclass(order=True)
class _Event:
    at: float
    kind_rank: int
    tie: tuple
    seq: int
    kind: str = field(compare=False)
    payload: dict = field(compare=False, default_factory=dict)


def _metrics_row():
    return {'calls': 0, 'attempts': 0, 'retries': 0, 'invalid': 0, 'errors': 0, 'timeouts': 0,
            'thinking_sim_s': 0., 'utterances': 0, 'messages_sent': 0, 'delivery_edges_out': 0,
            'broadcasts': 0, 'messages_received': 0, 'merged_triggers': 0, 'deferred': 0,
            'budget_refused': 0, 'rate_limited': 0}


class EventScheduler:
    """Fake-clock event loop: triggers -> costed calls -> deliveries -> triggers.

    ``transport`` provides the replies and is the only place wall time may be
    spent::

        token = transport.submit(pending_call)   # start the work, return a handle
        reply = transport.reply(token)           # may block; returns a CallReply

    Optional callbacks, all pure observers from the scheduler's point of view:

    ``advance(from_s, to_s)``   step physics over a SIM interval,
    ``on_hold(actor, holding, sim_s)``  hold / release the last safe command,
    ``on_action(actor, action, sim_s)`` apply a decision at its charged time,
    ``on_message(actor, message, sim_s)`` a delivery reached ``actor``'s inbox,
    ``on_observe(actor, sim_s)`` own-camera observation tick,
    ``on_timer(actor, label, sim_s)`` a timer fired.
    """

    def __init__(self, transport, *, cost_params=None, policy=None, actors=DEFAULT_ACTORS,
                 advance=None, on_hold=None, on_action=None, on_message=None, on_observe=None,
                 on_timer=None, start_s=0.):
        self.transport = transport
        self.params = cost_params or params()
        self.policy = policy or CallPolicy()
        self.actors = tuple(actors)
        self._rank = {actor: i for i, actor in enumerate(self.actors)}
        self.advance_fn, self.on_hold, self.on_action = advance, on_hold, on_action
        self.on_message, self.on_observe, self.on_timer = on_message, on_observe, on_timer
        self.clock = float(start_s)
        self._queue, self._seq = [], 0
        self._pending = {}
        self._thinking = {}
        self._deferred = {}
        self._last_start = {}
        self._retries = {}
        self._calls_started = 0
        self._attempts_total = 0
        self.calls = []
        self.messages = []
        self.transport_errors = []
        self.inboxes = {actor: [] for actor in self.actors}
        self.holds = []
        self.events = []
        self.metrics = {actor: _metrics_row() for actor in self.actors}

    # -- public API ---------------------------------------------------------

    def now(self):
        return _round(self.clock)

    def holding(self):
        """Actors currently thinking, i.e. holding their last safe command.

        A call counts as thinking from its start until its charged completion in
        SIM time, whether or not its reply has already been fetched.
        """
        busy = {c.actor for c in self._pending.values()} | set(self._thinking.values())
        return tuple(a for a in self.actors if a in busy)

    def trigger(self, actor, trigger='idle', *, at=None, retry_of=''):
        """Make ``actor`` eligible for a call (``at`` defaults to now)."""
        self._check_actor(actor)
        if trigger not in TRIGGERS:
            raise ValueError(f'unknown trigger {trigger!r}; known: {sorted(TRIGGERS)}')
        when = self.clock if at is None else float(at)
        if when < self.clock:
            raise ValueError('cannot schedule a call in the SIM past')
        self._push('call_start', when, (0., self._rank[actor], 0, 0),
                   {'actor': actor, 'trigger': trigger, 'merged': (), 'retry_of': retry_of})

    def timer(self, actor, label='timer', *, delay_s=None, at=None):
        """Arm a local timer; ``label`` becomes the trigger of the call it makes."""
        self._check_actor(actor)
        when = self.clock + float(delay_s) if at is None else float(at)
        self._push('timer', when, (0., self._rank[actor], 0, 0), {'actor': actor, 'label': label})

    def arm_observations(self, actors=None, *, period_s=None, first_at=None):
        """Start the periodic own-camera observation tick (re-arms itself)."""
        period = self.policy.observe_period_s if period_s is None else float(period_s)
        if period <= 0:
            raise ValueError('observation period must be positive')
        for actor in (actors or self.actors):
            self._check_actor(actor)
            at = self.clock + period if first_at is None else float(first_at)
            self._push('observe', at, (0., self._rank[actor], 0, 0), {'actor': actor, 'period_s': period})

    def inbox(self, actor):
        """Delivered messages of ``actor``, in delivery order."""
        self._check_actor(actor)
        return tuple(self.inboxes[actor])

    def trace(self):
        """Compact, comparable SIM trace: what happened, when, to whom.

        Two runs whose HTTP replies completed in different orders must produce
        the same tuple.
        """
        return tuple(row['line'] for row in self.events)

    def contract_log(self, *, run_id, condition_name, seed, provenance, envelopes=None,
                     request_ids=None, input_sha256=None, acts=None):
        """This run's calls and messages as package A log records.

        ``request_ids``/``input_sha256`` map ``call_id`` to the request id and the
        digest of the validated payload of that call; ``envelopes`` maps
        ``message_id`` to the package A envelope that was relayed. Every record
        is validated by ``harness.zone_study_contract.validate_log_record``.
        """
        request_ids, input_sha256 = dict(request_ids or {}), dict(input_sha256 or {})
        index = {actor: 0 for actor in self.actors}
        calls = []
        for record in self.calls:
            calls.append(contract_call_record(
                record, run_id=run_id, condition_name=condition_name, seed=seed,
                request_id=request_ids.get(record.call_id, record.call_id),
                call_index=index[record.actor],
                input_sha256=input_sha256.get(record.call_id, '0' * 64),
                provenance=provenance,
                message_ids=[m.message_id for m in self.messages if m.call_id == record.call_id]))
            index[record.actor] += 1
        messages = contract_message_records(self.messages, run_id=run_id,
                                            condition_name=condition_name, seed=seed,
                                            envelopes=dict(envelopes or {}), acts=acts) \
            if envelopes else []
        return {'schema': SCHEDULER_SCHEMA, 'calls': calls, 'messages': messages}

    def run(self, until_s=None, max_events=None):
        """Process events until the queue is quiet, ``until_s`` or ``max_events``."""
        processed, reason = 0, 'quiet'
        while True:
            if max_events is not None and processed >= max_events:
                reason = 'max_events'
                break
            next_at = self._queue[0].at if self._queue else None
            if self._pending and self._resolve_due(next_at):
                continue
            if next_at is None:
                break
            if until_s is not None and next_at > float(until_s):
                self._advance(float(until_s))
                reason = 'until'
                break
            event = heapq.heappop(self._queue)
            self._advance(event.at)
            self._dispatch(event)
            processed += 1
        return RunReport(sim_s=self.clock, stop_reason=reason, events=processed,
                         calls=len(self.calls), deliveries=len(self.messages))

    # -- queue -------------------------------------------------------------

    def _check_actor(self, actor):
        if actor not in self._rank:
            raise KeyError(f'unknown actor {actor!r}; known: {self.actors}')

    def _push(self, kind, at, tie, payload):
        self._seq += 1
        heapq.heappush(self._queue, _Event(at=_round(at), kind_rank=KIND_ORDER[kind], tie=tie,
                                           seq=self._seq, kind=kind, payload=payload))

    def _advance(self, to_s):
        target = _round(to_s)
        if target < self.clock - 1e-9:
            raise AssertionError(f'SIM time cannot go backwards: {self.clock} -> {target}')
        if target > self.clock and self.advance_fn is not None:
            self.advance_fn(self.clock, target)
        self.clock = max(self.clock, target)

    def _log(self, line, **fields):
        self.events.append({'sim_s': _round(self.clock), 'line': f'{self.clock:9.3f} {line}', **fields})

    # -- in-flight calls ---------------------------------------------------

    def _resolve_due(self, before):
        """Cost every in-flight call that could complete at or before ``before``.

        Returns True when something was resolved. Resolution order is fixed
        (start time, actor rank, call id), so a reply that arrived first does
        not get scheduled first.
        """
        floor = self.params.min_call_s()
        due = [c for c in self._pending.values()
               if before is None or _round(c.started_sim_s + floor) <= _round(before)]
        if not due:
            return False
        for call in sorted(due, key=lambda c: (c.started_sim_s, self._rank[c.actor], c.call_id)):
            del self._pending[call.call_id]
            reply = self._reply_of(call)
            cost = call_cost(reply.attempts, self.params)
            finish = _round(call.started_sim_s + cost.sim_s)
            # The actor keeps holding until ``finish``, even though the reply is
            # already in hand: SIM time, not the HTTP response, ends the wait.
            self._thinking[call.call_id] = call.actor
            self._push('call_done', finish, (0., self._rank[call.actor], 0, 0),
                       {'call': call, 'cost': cost, 'reply': reply})
        return True

    def _reply_of(self, call):
        """Fetch the reply; a transport exception is itself a costed error attempt."""
        try:
            reply = self.transport.reply(call.token)
        except Exception as exc:  # noqa: BLE001 - a failed call must still cost SIM time
            self.transport_errors.append({'call_id': call.call_id, 'actor': call.actor,
                                         'error': f'{type(exc).__name__}: {exc}'})
            return CallReply(attempts=(Attempt(outcome='error'),))
        if not isinstance(reply, CallReply):
            raise TypeError(f'transport returned {type(reply).__name__}, expected CallReply')
        return reply

    # -- dispatch ----------------------------------------------------------

    def _dispatch(self, event):
        getattr(self, f'_on_{event.kind}')(event.payload)

    def _on_call_start(self, payload):
        actor, trigger = payload['actor'], payload['trigger']
        merged = tuple(payload.get('merged', ()))
        outstanding = (sum(1 for c in self._pending.values() if c.actor == actor)
                       + sum(1 for a in self._thinking.values() if a == actor))
        if outstanding >= self.policy.max_outstanding_per_actor:
            self._defer(actor, trigger, merged, 'outstanding')
            return
        earliest = self._last_start.get(actor)
        if earliest is not None:
            earliest = quantize(earliest + self.policy.min_interval_s, self.params.quantum_s)
            if earliest > self.clock + 1e-9:
                self.metrics[actor]['rate_limited'] += 1
                self._log(f'call_deferred {actor} {trigger} until={earliest:.3f}',
                          kind='call_deferred', actor=actor)
                self._push('call_start', earliest, (0., self._rank[actor], 0, 0),
                           {'actor': actor, 'trigger': trigger, 'merged': merged,
                            'retry_of': payload.get('retry_of', '')})
                return
        if (self.metrics[actor]['calls'] >= self.policy.max_calls_per_actor
                or self._attempts_total >= self.policy.max_attempts_total):
            self.metrics[actor]['budget_refused'] += 1
            self._log(f'call_refused {actor} {trigger} budget', kind='call_refused', actor=actor)
            return
        self._calls_started += 1
        call = PendingCall(call_id=f'call-{self._calls_started:04d}-{actor}', actor=actor, trigger=trigger,
                           started_sim_s=self.clock, merged=merged, retry_of=payload.get('retry_of', ''))
        call.token = self.transport.submit(call)
        self._pending[call.call_id] = call
        self._last_start[actor] = self.clock
        self.metrics[actor]['calls'] += 1
        self.metrics[actor]['merged_triggers'] += len(merged)
        self.holds.append({'actor': actor, 'call_id': call.call_id, 'from_sim_s': self.clock, 'to_sim_s': None})
        if self.on_hold:
            self.on_hold(actor, True, self.now())
        self._log(f'call_start {actor} {trigger} {call.call_id}', kind='call_start', actor=actor,
                  call_id=call.call_id)

    def _defer(self, actor, trigger, merged, reason):
        held = self._deferred.get(actor)
        labels = set(merged) | {trigger} | set(held['merged'] if held else ())
        if held:
            labels.add(held['trigger'])
        best = max(labels, key=lambda t: (TRIGGERS[t], t))
        self._deferred[actor] = {'trigger': best, 'merged': tuple(sorted(labels - {best}))}
        self.metrics[actor]['deferred'] += 1
        self._log(f'call_merged {actor} {best} ({reason})', kind='call_merged', actor=actor)

    def _on_call_done(self, payload):
        call, cost, reply = payload['call'], payload['cost'], payload['reply']
        actor = call.actor
        self._thinking.pop(call.call_id, None)
        for hold in reversed(self.holds):
            if hold['call_id'] == call.call_id:
                hold['to_sim_s'] = self.now()
                break
        if self.on_hold:
            self.on_hold(actor, False, self.now())
        row = self.metrics[actor]
        row['thinking_sim_s'] = _round(row['thinking_sim_s'] + cost.sim_s)
        row['attempts'] += len(cost.attempts)
        self._attempts_total += len(cost.attempts)
        for attempt in cost.attempts:
            if attempt.outcome == 'invalid':
                row['invalid'] += 1
            elif attempt.outcome == 'error':
                row['errors'] += 1
            elif attempt.outcome == 'timeout':
                row['timeouts'] += 1
        row['utterances'] += cost.breakdown['utterances']
        self.calls.append(CallCostRecord(call_id=call.call_id, actor=actor, trigger=call.trigger,
                                         started_sim_s=call.started_sim_s, finished_sim_s=self.clock,
                                         cost=cost, merged_triggers=call.merged, retry_of=call.retry_of,
                                         notes={'messages': len(reply.messages)}))
        self._log(f'call_done {actor} {call.call_id} {cost.outcome} cost={cost.sim_s:.3f}',
                  kind='call_done', actor=actor, call_id=call.call_id)
        if reply.action is not None:
            self._log(f'action {actor} {reply.action}', kind='action', actor=actor)
            if self.on_action:
                self.on_action(actor, reply.action, self.now())
        self._emit(call, reply)
        if cost.outcome in FAILED_OUTCOMES:
            self._retry(call, cost)
        held = self._deferred.pop(actor, None)
        if held:
            self._push('call_start', self.clock, (0., self._rank[actor], 0, 0),
                       {'actor': actor, 'trigger': held['trigger'], 'merged': held['merged'], 'retry_of': ''})

    def _emit(self, call, reply):
        """Schedule deliveries of one reply's utterances, in a fixed order."""
        sender = call.actor
        for index, message in enumerate(reply.messages):
            if message.sender != sender:
                raise ValueError(f'{call.call_id}: message sender {message.sender!r} is not the caller {sender!r}')
            delay = delivery_delay_s(len(message.recipients), self.params)
            self.metrics[sender]['messages_sent'] += 1
            if message.broadcast:
                self.metrics[sender]['broadcasts'] += 1
            for slot, recipient in enumerate(message.recipients):
                self._check_actor(recipient)
                self.metrics[sender]['delivery_edges_out'] += 1
                self._push('message', self.clock + delay,
                           (self.clock, self._rank[sender], index, self._rank[recipient]),
                           {'message_id': f'{call.call_id}-m{index + 1}', 'call_id': call.call_id,
                            'sender': sender, 'recipient': recipient, 'encoding': message.encoding,
                            'body': message.body, 'broadcast': message.broadcast,
                            'sent_sim_s': self.clock, 'slot': slot})

    def _retry(self, call, cost):
        """A failed call may be retried as a separate, separately costed call."""
        root = call.retry_of or call.call_id
        if self._retries.get(root, 0) >= self.policy.max_retries:
            self._log(f'retry_exhausted {call.actor} {call.call_id} {cost.outcome}',
                      kind='retry_exhausted', actor=call.actor)
            return
        self._retries[root] = self._retries.get(root, 0) + 1
        self.metrics[call.actor]['retries'] += 1
        trigger = 'timeout' if cost.outcome == 'timeout' else 'retry'
        self._push('call_start', self.clock, (0., self._rank[call.actor], 0, 0),
                   {'actor': call.actor, 'trigger': trigger, 'merged': (), 'retry_of': root})

    def _on_message(self, payload):
        recipient = payload['recipient']
        record = MessageCostRecord(message_id=payload['message_id'], sender=payload['sender'],
                                   recipient=recipient, encoding=payload['encoding'],
                                   sent_sim_s=payload['sent_sim_s'], delivered_sim_s=self.clock,
                                   broadcast=payload['broadcast'], call_id=payload['call_id'])
        self.messages.append(record)
        self.inboxes[recipient].append({'message_id': record.message_id, 'sender': record.sender,
                                        'encoding': record.encoding, 'body': payload['body'],
                                        'delivered_sim_s': self.now(), 'broadcast': record.broadcast})
        self.metrics[recipient]['messages_received'] += 1
        self._log(f'deliver {payload["sender"]}->{recipient} {record.message_id}',
                  kind='message', actor=recipient, message_id=record.message_id)
        if self.on_message:
            self.on_message(recipient, self.inboxes[recipient][-1], self.now())
        if self.policy.trigger_on_message:
            self._push('call_start', self.clock, (0., self._rank[recipient], 0, 0),
                       {'actor': recipient, 'trigger': 'report', 'merged': (), 'retry_of': ''})

    def _on_timer(self, payload):
        actor, label = payload['actor'], payload['label']
        self._log(f'timer {actor} {label}', kind='timer', actor=actor)
        if self.on_timer:
            self.on_timer(actor, label, self.now())
        self._push('call_start', self.clock, (0., self._rank[actor], 0, 0),
                   {'actor': actor, 'trigger': label if label in TRIGGERS else 'timer',
                    'merged': (), 'retry_of': ''})

    def _on_observe(self, payload):
        actor = payload['actor']
        self._log(f'observe {actor}', kind='observe', actor=actor)
        if self.on_observe:
            self.on_observe(actor, self.now())
        self._push('observe', self.clock + payload['period_s'], (0., self._rank[actor], 0, 0), payload)


class ReplayTransport:
    """Scripted transport for fake-clock tests and no-LLM fixture runs.

    ``replies`` maps an actor to a list of :class:`CallReply` consumed in order,
    or is a callable ``(PendingCall) -> CallReply``; once a list runs out,
    ``default`` is returned. ``submitted`` and ``resolved`` record the submit and
    fetch orders, so a test can show the SIM trace does not follow them.
    """

    def __init__(self, replies, *, default=None):
        self.replies = replies
        self.default = default or CallReply()
        self.submitted, self.resolved = [], []
        self._queues = {}

    def submit(self, call):
        self.submitted.append(call.call_id)
        return call

    def reply(self, token):
        self.resolved.append(token.call_id)
        source = self.replies.get(token.actor) if isinstance(self.replies, dict) else self.replies
        if callable(source):
            return source(token)
        queue = self._queues.setdefault(token.actor, list(source or ()))
        return queue.pop(0) if queue else self.default
