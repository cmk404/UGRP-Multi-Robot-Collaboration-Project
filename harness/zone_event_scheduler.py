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

import collections
import heapq
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from types import MappingProxyType

from harness.zone_sim_cost import (Attempt, CallCostRecord, FAILED_OUTCOMES, MessageCostRecord,
                                   TRIGGER_TO_CONTRACT, call_cost, censored_call_record,
                                   contract_call_record, contract_message_records, delivery_delay_s,
                                   params, quantize)
from harness.zone_study_contract import (CONDITIONS as CONTRACT_CONDITIONS, ENVELOPE_KEYS,
                                         MESSAGE_ENVELOPE_SCHEMA, ROBOTS)

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


def _usage_dict(usage):
    """JSON copy of a provider usage report (None stays None)."""
    return None if usage is None else dict(usage)


@dataclass(frozen=True)
class Message:
    """One utterance leaving one actor. Several recipients = one broadcast.

    ``message_id`` is the CANONICAL id package C minted when it accepted the
    envelope; the scheduler no longer invents a second one (2026-09-26 review
    finding 2). ``rejection`` marks an utterance the channel refused: it is
    still BILLED, because the model generated it, but it is never delivered
    (review finding 6).
    """

    sender: str
    recipients: tuple
    body: object = ''
    encoding: str = 'free_ko'
    message_id: str = ''
    rejection: str | None = None
    reply_to: str | None = None

    def __post_init__(self):
        if self.encoding not in ENCODINGS:
            raise ValueError(f'encoding must be one of {ENCODINGS}, got {self.encoding!r}')
        if not self.recipients:
            raise ValueError('a message needs at least one recipient')
        if self.sender in self.recipients:
            raise ValueError('a message cannot be addressed to its sender')

    @property
    def delivered(self):
        return self.rejection is None

    @property
    def broadcast(self):
        return len(self.recipients) > 1


@dataclass(frozen=True)
class CallReply:
    """What one logical call produced: costed attempts, an action, utterances.

    2026-09-26 review finding 6: the BILLED utterance count of the last attempt
    must equal the number of utterances the reply actually produced, whether or
    not the channel accepted them. Generating an utterance costs SIM time even
    when the transport rejects it, and an utterance that is delivered can no
    longer be free.
    """

    attempts: tuple = (Attempt(),)
    action: object = None
    messages: tuple = ()
    #: Utterances the model GENERATED that never became an envelope because the
    #: reply failed to parse (second review, finding 6). They are billed like
    #: any other utterance and are never delivered. Only a failed last attempt
    #: can carry them.
    unparsed_utterances: int = 0
    #: Third review, finding 16: False when the transport could NOT read what
    #: the provider billed (a plain exception). The flag travels with the reply
    #: into the call ledger, the censored row and package A's ``cost_terms``, so
    #: its 0 tokens stay a labelled lower bound instead of a confirmed 0.
    usage_known: bool = True
    #: The provider's OWN usage report of this call (e.g. ``{'input_tokens':
    #: 812, 'output_tokens': 95, 'image_tokens': 0, 'cached_tokens': 0}``) or
    #: None when no provider answered (offline fixture) or it reported nothing.
    #: Kept APART from the standardised billed size in ``attempts`` that the
    #: SIM cost uses (third review, finding 18 follow-up).
    provider_usage: object = None

    def __post_init__(self):
        if not self.attempts:
            raise ValueError('a reply needs at least one attempt')
        if not isinstance(self.usage_known, bool):
            raise ValueError('usage_known must be a bool')
        if self.provider_usage is not None:
            if not isinstance(self.provider_usage, Mapping) or any(
                    isinstance(v, bool) or not isinstance(v, int) or v < 0
                    for v in self.provider_usage.values()):
                raise ValueError('provider_usage must map names to non-negative ints, or be None')
            object.__setattr__(self, 'provider_usage', MappingProxyType(dict(self.provider_usage)))
        if isinstance(self.unparsed_utterances, bool) or not isinstance(self.unparsed_utterances, int) \
                or self.unparsed_utterances < 0:
            raise ValueError('unparsed_utterances must be a non-negative int')
        if self.unparsed_utterances and self.attempts[-1].outcome not in FAILED_OUTCOMES:
            raise ValueError('only a failed reply can carry unparsed utterances; a parsed reply carries '
                             'its utterances as messages')
        billed = self.attempts[-1].utterances
        produced = len(self.messages) + self.unparsed_utterances
        if billed != produced:
            raise ValueError(f'the reply produced {produced} utterance(s) ({len(self.messages)} message(s), '
                             f'{self.unparsed_utterances} unparsed) but its last attempt bills {billed}: '
                             'the charged utterance count must be the count the model produced '
                             '(review finding 6)')

    @property
    def produced_utterances(self):
        return len(self.messages) + self.unparsed_utterances


class TransportFailure(Exception):
    """A transport failure that still KNOWS what the provider billed.

    Second review, finding 6: the scheduler turned every transport exception
    into an ``error`` attempt with no tokens and no utterances, so a malformed
    or failed reply whose usage the transport had already read became free. A
    transport raises this with the costed ``attempts`` (and the number of
    utterances the model generated) instead; a plain exception is still an
    ``error`` attempt, recorded with ``usage_known=False``.

    Fourth review, finding 16: a transport that read the usage of SOME attempts
    only (an internal retry whose last attempt failed without a usage report)
    raises this with ``usage_known=False``. The counts it does know stay in
    ``attempts`` as a labelled lower bound instead of being claimed complete.
    """

    def __init__(self, message, *, attempts, unparsed_utterances=0, provider_usage=None,
                 usage_known=True):
        super().__init__(message)
        if not isinstance(usage_known, bool):
            raise ValueError('usage_known must be a bool')
        self.attempts = tuple(attempts)
        self.unparsed_utterances = int(unparsed_utterances)
        self.provider_usage = provider_usage
        self.usage_known = usage_known


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
    #: ``reserve(count=1) -> bool``: a transport that retries internally MUST
    #: reserve every further HTTP attempt through this before sending it
    #: (second review, finding 15). False = the budget is out, do not send.
    reserve: object = None


@dataclass(frozen=True)
class CallPolicy:
    """Call eligibility and budget. Identical for every condition.

    Fairness is equal *eligibility and limits*, not an equal number of calls:
    an actor that talks more pays more SIM time. Start values come from the
    design document and are provisional.

    Two DIFFERENT units (2026-09-26 review finding 15): ``max_calls_per_actor``
    bounds LOGICAL calls, while ``max_http_attempts_per_actor`` and
    ``max_attempts_total`` bound HTTP attempts, which is what package E's
    ``http_attempts_per_actor`` means. Attempts are reserved before a request
    leaves, so concurrent actors and transport-internal retries cannot together
    exceed the cap.
    """

    min_interval_s: float = 2.
    max_outstanding_per_actor: int = 1
    idle_reask_s: float = 10.
    busy_reask_s: float = 60.
    observe_period_s: float = 1.
    trigger_on_message: bool = True
    max_retries: int = 1
    max_calls_per_actor: int = 30
    max_http_attempts_per_actor: int = 30
    max_attempts_total: int = 90


class AttemptBudget:
    """The single owner of the HTTP attempt budget (review finding 15).

    Every attempt is RESERVED before the request starts, so three actors that
    submit at the same SIM instant cannot each be told there is room for one more
    when only one is left. A transport that retries internally must reserve each
    of its own attempts through the same object, and ``commit`` reconciles the
    reservation with what the reply actually reports.
    """

    def __init__(self, *, per_actor=None, total=None):
        self.per_actor = per_actor
        self.total = total
        self.used = collections.Counter()      # committed attempts per actor
        self.reserved = collections.Counter()  # in-flight reservations per actor
        self.refusals = []

    def used_total(self):
        return sum(self.used.values())

    def outstanding(self):
        return sum(self.reserved.values())

    def remaining(self, actor=None):
        """Attempts still available (team budget, and the actor's own budget)."""
        left = None
        if self.total is not None:
            left = self.total - self.used_total() - self.outstanding()
        if actor is not None and self.per_actor is not None:
            own = self.per_actor - self.used[actor] - self.reserved[actor]
            left = own if left is None else min(left, own)
        return left

    def reserve(self, actor, count=1):
        """Reserve ``count`` attempts for ``actor``; False when the budget is out."""
        if count < 1:
            raise ValueError('an attempt reservation must be at least 1')
        left = self.remaining(actor)
        if left is not None and left < count:
            self.refusals.append({'actor': actor, 'requested': count, 'remaining': max(left, 0)})
            return False
        self.reserved[actor] += count
        return True

    def release(self, actor, count=1):
        self.reserved[actor] = max(0, self.reserved[actor] - count)

    def commit(self, actor, *, reserved, actual):
        """Turn ``reserved`` reservations into ``actual`` used attempts.

        Returns the number of attempts that were over the reservation and could
        NOT be covered by the remaining budget; the caller records them instead
        of silently exceeding the cap.
        """
        self.release(actor, reserved)
        # ``remaining`` after the release INCLUDES the slots this call had
        # reserved, so the over-budget count is ``actual - remaining`` (the
        # first round subtracted the reservation twice: cap 1, 3 attempts -> 1).
        left = self.remaining(actor)
        over = max(0, actual - max(left, 0)) if left is not None else 0
        self.used[actor] += actual
        return over

    def to_dict(self):
        return {'per_actor': self.per_actor, 'total': self.total, 'used': dict(self.used),
                'used_total': self.used_total(), 'outstanding': self.outstanding(),
                'refusals': list(self.refusals)}


@dataclass(frozen=True)
class RunReport:
    """Why the loop stopped and what it spent."""

    sim_s: float
    stop_reason: str
    events: int
    calls: int
    deliveries: int
    #: Calls that were still in flight at the horizon (review finding 16).
    censored_calls: int = 0

    def to_dict(self):
        return {'schema': SCHEDULER_SCHEMA, 'sim_s': _round(self.sim_s), 'stop_reason': self.stop_reason,
                'events': self.events, 'calls': self.calls, 'deliveries': self.deliveries,
                'censored_calls': self.censored_calls}


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
            'thinking_sim_s': 0., 'utterances': 0, 'messages_sent': 0, 'messages_rejected': 0,
            'delivery_edges_out': 0, 'broadcasts': 0, 'messages_received': 0, 'merged_triggers': 0,
            'deferred': 0, 'budget_refused': 0, 'rate_limited': 0}


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
                 on_timer=None, start_s=0., bus=None, bus_owner='sim_scheduler'):
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
        # The message bus this scheduler OWNS (review finding 2). When set, the
        # canonical envelope ids come from it and it is the only object whose
        # inbox this scheduler mutates.
        self.bus = bus
        self.bus_owner = bus_owner
        if bus is not None and getattr(bus, 'delivery_owner', None) != bus_owner:
            raise ValueError(f'the bus must be constructed with delivery_owner={bus_owner!r} so exactly '
                             'one component owns its inboxes')
        # Single owner of the HTTP attempt budget (review finding 15).
        self.budget = AttemptBudget(per_actor=self.policy.max_http_attempts_per_actor,
                                    total=self.policy.max_attempts_total)
        #: Call ledger, opened at START so a call still outstanding at the
        #: horizon stays visible as ``censored`` (review finding 16).
        self.ledger = {}
        self.calls = []
        self.censored = []
        self.messages = []
        self.rejected_messages = []
        self.discarded = []
        self.transport_errors = []
        self.over_budget_attempts = []
        #: Fifth review, P1: calls whose unknown-usage reply accounted for fewer
        #: attempts than they reserved; every reserved attempt was counted as sent.
        self.unreported_attempts = []
        #: Robot-facing inbox: package A's closed envelope (``ENVELOPE_KEYS``),
        #: the SAME shape as package C's ``Transport.inbox`` (second review,
        #: finding 2). Delivery times live in ``deliveries`` (evaluation).
        self.inboxes = {actor: [] for actor in self.actors}
        self.deliveries = {actor: [] for actor in self.actors}
        #: Every accepted utterance scheduled for delivery, delivered or not by
        #: the horizon, so billing can be reconciled with production.
        self.scheduled = {}
        self.holds = []
        self.events = []
        self.metrics = {actor: _metrics_row() for actor in self.actors}

    @property
    def _attempts_total(self):
        """Committed HTTP attempts. The budget object is the single owner."""
        return self.budget.used_total()

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
        """Delivered envelopes of ``actor``, in delivery order (package A shape)."""
        self._check_actor(actor)
        return tuple(dict(row) for row in self.inboxes[actor])

    def delivery_log(self, actor):
        """Evaluation-side delivery rows of ``actor``: envelope id, time, broadcast."""
        self._check_actor(actor)
        return tuple(dict(row) for row in self.deliveries[actor])

    def undelivered(self):
        """Accepted utterances whose delivery had not happened by the stop."""
        return [dict(row, message_id=mid) for mid, row in sorted(self.scheduled.items())
                if set(row['delivered']) != set(row['recipients'])]

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

        Calls that were still outstanding at the horizon are included with
        ``status='censored'`` (review finding 16), in call order per actor, so
        the ledger and the report cannot understate a condition's call count.
        """
        request_ids, input_sha256 = dict(request_ids or {}), dict(input_sha256 or {})
        rows = [(record.started_sim_s, record.actor, record.call_id, record, None)
                for record in self.calls]
        rows += [(row['started_sim_s'], row['actor'], row['call_id'], None, row)
                 for row in self.censored]
        index = {actor: 0 for actor in self.actors}
        calls = []
        for _, actor, call_id, record, censored in sorted(rows, key=lambda r: (r[0], self._rank[r[1]],
                                                                              r[2])):
            if record is not None:
                calls.append(contract_call_record(
                    record, run_id=run_id, condition_name=condition_name, seed=seed,
                    request_id=request_ids.get(call_id, call_id),
                    call_index=index[actor],
                    input_sha256=input_sha256.get(call_id, '0' * 64),
                    provenance=provenance,
                    message_ids=[m.message_id for m in self.messages if m.call_id == call_id]))
            else:
                calls.append(censored_call_record(
                    censored, run_id=run_id, condition_name=condition_name, seed=seed,
                    request_id=request_ids.get(call_id, call_id), call_index=index[actor],
                    input_sha256=input_sha256.get(call_id, '0' * 64), provenance=provenance))
            index[actor] += 1
        messages = contract_message_records(self.messages, run_id=run_id,
                                            condition_name=condition_name, seed=seed,
                                            envelopes=dict(envelopes or {}), acts=acts) \
            if envelopes else []
        return {'schema': SCHEDULER_SCHEMA, 'calls': calls, 'messages': messages,
                'censored_calls': [dict(row) for row in self.censored],
                'unreported_attempts': [dict(row) for row in self.unreported_attempts],
                'attempt_budget': self.budget.to_dict()}

    def run(self, until_s=None, max_events=None, *, close_at_horizon=True):
        """Process events until the queue is quiet, ``until_s`` or ``max_events``.

        With ``close_at_horizon`` (the default) ``until_s`` is the trial HORIZON:
        calls that started before it but would only be charged after it are NOT
        dropped, they are closed as ``censored`` with the SIM time that elapsed
        and the provider usage they really had (review finding 16, second
        review). ``close_at_horizon=False`` makes ``until_s`` a resumable pause:
        in-flight calls stay in flight and finish on the next ``run``.
        """
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
        if reason == 'until' and close_at_horizon:
            # Only the HORIZON closes a call as censored. A pause or a
            # ``max_events`` stop is an interrupted run the caller may resume, so
            # its in-flight calls stay pending.
            self._censor_outstanding(until_s)
        return RunReport(sim_s=self.clock, stop_reason=reason, events=processed,
                         calls=len(self.calls), deliveries=len(self.messages),
                         censored_calls=len(self.censored))

    def _censor_outstanding(self, until_s):
        """Close every call that is still in flight at the horizon (finding 16).

        Second review: the SIM action of such a call is never released, but the
        API call itself was made. A reply that was already fetched keeps its
        real usage, and a reply that was not fetched yet is fetched now (the
        request left at the call's start), so tokens, attempts and utterances are
        the provider's numbers instead of 0. Only a transport failure without
        usage stays ``usage_known=False``. Nothing of the reply is executed.
        """
        at = self.clock if until_s is None else max(self.clock, float(until_s))
        rows = []
        for call in self._pending.values():
            rows.append((call, 'pending', None, None))
        for event in self._queue:
            if event.kind == 'call_done':
                rows.append((event.payload['call'], 'charged_after_horizon', event.payload['cost'],
                             event.payload['reply']))
        censored_ids = set()
        for call, why, cost, reply in sorted(rows, key=lambda r: (r[0].started_sim_s, self._rank[r[0].actor],
                                                                   r[0].call_id)):
            entry = self.ledger.get(call.call_id)
            if entry is None or entry['status'] != 'outstanding':
                continue
            if reply is None:
                reply = self._reply_of(call)
                cost = call_cost(reply.attempts, self.params)
            # third review, finding 16: the flag is the REPLY's, whether it was
            # fetched before the horizon or just now; it is never reset to True.
            usage_known = reply.usage_known
            reserved = entry.get('reserved_attempts', 1)
            actual = len(cost.attempts)
            over = self.budget.commit(call.actor, reserved=reserved, actual=actual)
            if actual > reserved or over:
                self.over_budget_attempts.append({'call_id': call.call_id, 'actor': call.actor,
                                                  'reserved': reserved, 'actual': actual,
                                                  'unreserved': max(0, actual - reserved), 'over': over})
            self._thinking.pop(call.call_id, None)
            censored_ids.add(call.call_id)
            elapsed = _round(max(at - call.started_sim_s, 0.))
            entry.update({'status': 'censored', 'finished_sim_s': _round(at), 'reason': why,
                          'elapsed_sim_s': elapsed, 'attempts': actual})
            self.censored.append({'call_id': call.call_id, 'actor': call.actor, 'trigger': call.trigger,
                                  'started_sim_s': call.started_sim_s, 'horizon_sim_s': _round(at),
                                  'elapsed_sim_s': elapsed, 'reason': why,
                                  'reserved_attempts': reserved, 'retry_of': call.retry_of,
                                  # the API side is complete; the SIM release is not
                                  'usage_known': usage_known, 'http_attempts': actual,
                                  'input_tokens': cost.breakdown['input_tokens'],
                                  'output_tokens': cost.breakdown['output_tokens'],
                                  'utterances': cost.breakdown['utterances'],
                                  'outcome': cost.outcome, 'charged_sim_s': cost.sim_s,
                                  'would_release_sim_s': _round(call.started_sim_s + cost.sim_s),
                                  'messages': len(reply.messages),
                                  'unparsed_utterances': reply.unparsed_utterances,
                                  'provider_usage': _usage_dict(reply.provider_usage),
                                  'cost': cost.to_dict()})
            self._log(f'call_censored {call.actor} {call.call_id} elapsed={elapsed:.3f}',
                      kind='call_censored', actor=call.actor, call_id=call.call_id)
        self._pending.clear()
        if censored_ids:
            # a censored call can never complete later, even if the run resumes
            self._queue = [e for e in self._queue
                           if not (e.kind == 'call_done' and e.payload['call'].call_id in censored_ids)]
            heapq.heapify(self._queue)

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
        """Fetch the reply and account for every attempt the call reserved.

        Fifth review, P1: a transport that sent a RESERVED internal retry and
        then failed without a usage report was counted as the one attempt its
        exception implied, and ``commit`` refunded the other reservation, so the
        scheduler's own retry could spend it again (3 real sends, 2 in the
        ledger, no violation). A compliant transport reserves each attempt just
        before sending it, so when the reply cannot say what it sent
        (``usage_known=False``) every reserved attempt is counted as sent: the
        missing ones are costed ``error`` attempts placed BEFORE the reported
        ones (the reply's last attempt stays the call outcome) and the gap is
        recorded in ``unreported_attempts``. A reply with a KNOWN usage states
        its own attempts; an unused reservation of it is still refunded.
        """
        reply, reported = self._fetch_reply(call)
        reserved = self.ledger[call.call_id]['reserved_attempts']
        if reply.usage_known or len(reply.attempts) >= reserved:
            return reply
        missing = reserved - len(reply.attempts)
        self.unreported_attempts.append({'call_id': call.call_id, 'actor': call.actor,
                                         'reserved': reserved, 'reported': reported,
                                         'counted': reserved})
        self._log(f'attempts_unreported {call.actor} {call.call_id} reserved={reserved} '
                  f'reported={reported}', kind='attempts_unreported', actor=call.actor,
                  call_id=call.call_id)
        return replace(reply, attempts=(Attempt(outcome='error'),) * missing + reply.attempts)

    def _fetch_reply(self, call):
        """``(reply, reported attempts)``; a transport exception is a costed error attempt."""
        try:
            reply = self.transport.reply(call.token)
        except TransportFailure as exc:
            # the transport knows what the provider billed: keep it (finding 6)
            # fourth review: the transport says whether what it knows is ALL of it
            usage_known = bool(exc.usage_known) and bool(exc.attempts)
            self.transport_errors.append({'call_id': call.call_id, 'actor': call.actor,
                                         'error': f'{type(exc).__name__}: {exc}',
                                         'usage_known': usage_known,
                                         'unparsed_utterances': exc.unparsed_utterances})
            attempts = exc.attempts or (Attempt(outcome='error'),)
            if attempts[-1].outcome not in FAILED_OUTCOMES:
                attempts = attempts[:-1] + (Attempt(outcome='error',
                                                    input_tokens=attempts[-1].input_tokens,
                                                    output_tokens=attempts[-1].output_tokens,
                                                    utterances=attempts[-1].utterances),)
            return CallReply(attempts=attempts, unparsed_utterances=attempts[-1].utterances,
                             provider_usage=exc.provider_usage, usage_known=usage_known), len(exc.attempts)
        except Exception as exc:  # noqa: BLE001 - a failed call must still cost SIM time
            self.transport_errors.append({'call_id': call.call_id, 'actor': call.actor,
                                         'error': f'{type(exc).__name__}: {exc}', 'usage_known': False})
            return CallReply(attempts=(Attempt(outcome='error'),), usage_known=False), 0
        if not isinstance(reply, CallReply):
            raise TypeError(f'transport returned {type(reply).__name__}, expected CallReply')
        return reply, len(reply.attempts)

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
        if self.metrics[actor]['calls'] >= self.policy.max_calls_per_actor:
            self.metrics[actor]['budget_refused'] += 1
            self._log(f'call_refused {actor} {trigger} budget', kind='call_refused', actor=actor)
            return
        # finding 15: reserve the first HTTP attempt BEFORE the request leaves, so
        # three simultaneous actors cannot each consume the last attempt.
        if not self.budget.reserve(actor, 1):
            self.metrics[actor]['budget_refused'] += 1
            self._log(f'call_refused {actor} {trigger} http_budget', kind='call_refused', actor=actor)
            return
        self._calls_started += 1
        call = PendingCall(call_id=f'call-{self._calls_started:04d}-{actor}', actor=actor, trigger=trigger,
                           started_sim_s=self.clock, merged=merged, retry_of=payload.get('retry_of', ''))
        # finding 16: the ledger row exists from the START of the call.
        self.ledger[call.call_id] = {'call_id': call.call_id, 'actor': actor, 'trigger': trigger,
                                     'started_sim_s': self.clock, 'reserved_attempts': 1,
                                     'status': 'outstanding', 'finished_sim_s': None,
                                     'retry_of': call.retry_of, 'merged_triggers': tuple(merged)}
        call.reserve = lambda count=1, _call=call: self._reserve_more(_call, count)
        try:
            call.token = self.transport.submit(call)
        except Exception:
            self.budget.release(actor, 1)
            self.ledger[call.call_id]['status'] = 'submit_failed'
            raise
        self._pending[call.call_id] = call
        self._last_start[actor] = self.clock
        self.metrics[actor]['calls'] += 1
        self.metrics[actor]['merged_triggers'] += len(merged)
        self.holds.append({'actor': actor, 'call_id': call.call_id, 'from_sim_s': self.clock, 'to_sim_s': None})
        if self.on_hold:
            self.on_hold(actor, True, self.now())
        self._log(f'call_start {actor} {trigger} {call.call_id}', kind='call_start', actor=actor,
                  call_id=call.call_id)

    def _reserve_more(self, call, count=1):
        """Reserve further HTTP attempts of an in-flight call (finding 15).

        A transport calls this through ``PendingCall.reserve`` BEFORE each
        internal retry. The reservation goes through the single budget owner, so
        a refused retry is never sent instead of being found over budget later.
        """
        entry = self.ledger.get(call.call_id)
        if entry is None or entry['status'] != 'outstanding':
            raise ValueError(f'{call.call_id} is not outstanding; it cannot reserve attempts')
        if not self.budget.reserve(call.actor, count):
            self._log(f'retry_refused {call.actor} {call.call_id} http_budget', kind='retry_refused',
                      actor=call.actor, call_id=call.call_id)
            return False
        entry['reserved_attempts'] += count
        return True

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
        entry = self.ledger.get(call.call_id, {})
        reserved = entry.get('reserved_attempts', 1)
        actual = len(cost.attempts)
        unreserved = max(0, actual - reserved)
        over = self.budget.commit(actor, reserved=reserved, actual=actual)
        if unreserved or over:
            # finding 15: an attempt the transport sent WITHOUT reserving it is a
            # budget breach. It is recorded, and the reply that depended on it
            # executes nothing (second review: the retry used to run anyway).
            self.over_budget_attempts.append({'call_id': call.call_id, 'actor': actor,
                                              'reserved': reserved, 'actual': actual,
                                              'unreserved': unreserved, 'over': over})
            self._log(f'attempts_over_budget {actor} {call.call_id} unreserved={unreserved} over={over}',
                      kind='attempts_over_budget', actor=actor, call_id=call.call_id)
        for attempt in cost.attempts:
            if attempt.outcome == 'invalid':
                row['invalid'] += 1
            elif attempt.outcome == 'error':
                row['errors'] += 1
            elif attempt.outcome == 'timeout':
                row['timeouts'] += 1
        row['utterances'] += cost.breakdown['utterances']
        failed = cost.outcome in FAILED_OUTCOMES
        breach = bool(unreserved)
        notes = {'messages': len(reply.messages), 'unparsed_utterances': reply.unparsed_utterances,
                 'usage_known': reply.usage_known, 'provider_usage': _usage_dict(reply.provider_usage),
                 'failed': failed, 'attempts_over_budget': over, 'unreserved_attempts': unreserved}
        if failed or breach:
            # finding 5: a failed call PAYS but executes nothing. Its action and
            # its utterances are recorded as discarded, and only then is it
            # retried, so a wrong decision cannot run and get a second chance.
            notes['discarded_action'] = reply.action is not None
            notes['discarded_messages'] = len(reply.messages)
            self.discarded.append({'call_id': call.call_id, 'actor': actor, 'outcome': cost.outcome,
                                   'reason': 'budget_breach' if breach else 'failed',
                                   'action': reply.action, 'messages': len(reply.messages),
                                   'unparsed_utterances': reply.unparsed_utterances,
                                   'sim_s': self.now()})
        self.calls.append(CallCostRecord(call_id=call.call_id, actor=actor, trigger=call.trigger,
                                         started_sim_s=call.started_sim_s, finished_sim_s=self.clock,
                                         cost=cost, merged_triggers=call.merged, retry_of=call.retry_of,
                                         notes=notes))
        entry.update({'status': 'failed' if (failed or breach) else 'done', 'finished_sim_s': self.now(),
                      'attempts': len(cost.attempts)})
        self._log(f'call_done {actor} {call.call_id} {cost.outcome} cost={cost.sim_s:.3f}',
                  kind='call_done', actor=actor, call_id=call.call_id)
        if failed or breach:
            self._log(f'call_discarded {actor} {call.call_id} {cost.outcome} '
                      f'action={reply.action is not None} messages={len(reply.messages)}',
                      kind='call_discarded', actor=actor, call_id=call.call_id)
            if not breach:          # a budget breach is never rewarded with a retry
                self._retry(call, cost)
        else:
            if reply.action is not None:
                self._log(f'action {actor} {reply.action}', kind='action', actor=actor)
                if self.on_action:
                    self.on_action(actor, reply.action, self.now())
            self._emit(call, reply)
        held = self._deferred.pop(actor, None)
        if held:
            self._push('call_start', self.clock, (0., self._rank[actor], 0, 0),
                       {'actor': actor, 'trigger': held['trigger'], 'merged': held['merged'], 'retry_of': ''})

    def _emit(self, call, reply):
        """Schedule deliveries of one reply's utterances, in a fixed order.

        The canonical ``message_id`` comes from the reply (package C's accepted
        envelope) when there is one; a rejected utterance is recorded and billed
        but never delivered (review findings 2 and 6).
        """
        sender = call.actor
        for index, message in enumerate(reply.messages):
            if message.sender != sender:
                raise ValueError(f'{call.call_id}: message sender {message.sender!r} is not the caller {sender!r}')
            message_id = message.message_id or f'{call.call_id}-m{index + 1}'
            if self.bus is not None and not message.message_id:
                raise ValueError(f'{call.call_id}: a scheduler that owns a message bus needs the canonical '
                                 'message_id of the accepted envelope, not a second id space')
            self.metrics[sender]['messages_sent'] += 1
            if not message.delivered:
                self.rejected_messages.append({'message_id': message_id, 'call_id': call.call_id,
                                               'sender': sender, 'recipients': list(message.recipients),
                                               'rejection': message.rejection, 'sim_s': self.now()})
                self.metrics[sender]['messages_rejected'] += 1
                self._log(f'message_rejected {sender} {message_id} {message.rejection}',
                          kind='message_rejected', actor=sender, message_id=message_id)
                continue
            delay = delivery_delay_s(len(message.recipients), self.params)
            self.scheduled[message_id] = {'call_id': call.call_id, 'sender': sender,
                                          'recipients': list(message.recipients), 'delivered': []}
            if message.broadcast:
                self.metrics[sender]['broadcasts'] += 1
            for slot, recipient in enumerate(message.recipients):
                self._check_actor(recipient)
                self.metrics[sender]['delivery_edges_out'] += 1
                self._push('message', self.clock + delay,
                           (self.clock, self._rank[sender], index, self._rank[recipient]),
                           {'message_id': message_id, 'call_id': call.call_id,
                            'sender': sender, 'recipient': recipient, 'encoding': message.encoding,
                            'body': message.body, 'broadcast': message.broadcast,
                            'recipients': tuple(message.recipients), 'reply_to': message.reply_to,
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
        if self.bus is not None:
            # finding 2: the SIM scheduler is the ONLY component that puts a
            # message into an inbox, and it does so at the SIM time it charged.
            self.bus.commit_delivery(payload['message_id'], at_sim_s=self.now(),
                                     owner=self.bus_owner, recipients=[recipient])
            # second review: the robot-facing row IS the bus's canonical
            # envelope (recipients, created_at_sim_s, the real body), not a
            # second shape keyed by the id.
            envelope = self.bus.delivered[payload['message_id']].record()
        else:
            envelope = {'schema': MESSAGE_ENVELOPE_SCHEMA, 'message_id': payload['message_id'],
                        'sender': payload['sender'], 'recipients': list(payload['recipients']),
                        'encoding': payload['encoding'], 'created_at_sim_s': _round(payload['sent_sim_s']),
                        'reply_to': payload['reply_to'], 'body': payload['body']}
        if set(envelope) != set(ENVELOPE_KEYS):
            raise AssertionError(f'inbox envelope keys {sorted(envelope)} differ from package A')
        self.scheduled.get(payload['message_id'], {'delivered': []})['delivered'].append(recipient)
        record = MessageCostRecord(message_id=payload['message_id'], sender=payload['sender'],
                                   recipient=recipient, encoding=payload['encoding'],
                                   sent_sim_s=payload['sent_sim_s'], delivered_sim_s=self.clock,
                                   broadcast=payload['broadcast'], call_id=payload['call_id'])
        self.messages.append(record)
        self.inboxes[recipient].append(envelope)
        self.deliveries[recipient].append({'message_id': record.message_id, 'sender': record.sender,
                                           'delivered_sim_s': self.now(), 'broadcast': record.broadcast})
        self.metrics[recipient]['messages_received'] += 1
        self._log(f'deliver {payload["sender"]}->{recipient} {record.message_id}',
                  kind='message', actor=recipient, message_id=record.message_id)
        if self.on_message:
            self.on_message(recipient, dict(envelope), self.now())
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
            return self._reserved(token, source(token))
        queue = self._queues.setdefault(token.actor, list(source or ()))
        return self._reserved(token, queue.pop(0) if queue else self.default)

    @staticmethod
    def _reserved(token, reply):
        """Replay a multi-attempt reply as a COMPLIANT transport would send it.

        Every attempt after the first is reserved through ``token.reserve``
        before it is "sent" (second review, finding 15). A refused reservation
        means the retry never left, so the reply ends at the last attempt the
        budget covered, and a failed last attempt executes nothing.

        Fourth review, finding 16: the truncated reply keeps the scripted
        reply's ``usage_known`` (it was silently reset to True, so an unknown
        usage became "0 confirmed tokens"). The scripted ``provider_usage``
        described attempts that were never sent, so it cannot be attributed to
        the kept ones and is dropped (None = no provider report).
        """
        reserve = getattr(token, 'reserve', None)
        if reserve is None or len(reply.attempts) < 2:
            return reply
        for index in range(1, len(reply.attempts)):
            if not reserve(1):
                kept = reply.attempts[:index]
                if kept[-1].outcome not in FAILED_OUTCOMES:
                    raise ValueError('a scripted reply cannot retry after a successful attempt')
                return CallReply(attempts=kept, unparsed_utterances=kept[-1].utterances,
                                 usage_known=reply.usage_known, provider_usage=None)
        return reply
