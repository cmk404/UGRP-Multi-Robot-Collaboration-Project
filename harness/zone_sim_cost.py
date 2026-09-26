"""Deterministic SIM-time cost of thinking and talking (zone dialogue study, package D).

Why: in the current zone runners SIM time does not advance while an LLM reply is
awaited (``team.ask()`` blocks, then the loop calls ``zone.step(.5)``:
``scripts/run_zone_dispatch.py`` lines 274 and 331). Thinking and talking are
therefore free, which makes "does Korean dialogue change task efficiency?"
unanswerable: a condition may talk for free.

This module charges a cost that is a **pure function of the request/reply
content and a versioned parameter set**, never of wall time or API latency:

    raw_q  = scale * (alpha + a_in*N_in + beta*N_out + gamma*U)
    d_q    = quantum * ceil(raw_q / quantum)

* ``alpha`` (``call_overhead_s``) fixed per-call decision overhead,
* ``N_in``  input tokens counted by the frozen tokenizer/counting policy
  (image tokens included by the caller, see ``input_tokens`` below),
* ``beta``  (``output_token_s``) per output token, action JSON and messages,
* ``U``     number of non-empty utterances in the reply,
* ``gamma`` (``utterance_s``) speaking/transmission setup per utterance,
* ``scale`` global multiplier for the sensitivity sweep (``0`` = free talk, the
  diagnostic condition that reproduces today's behaviour),
* ``quantum`` the SIM step the caller advances physics with, so that every
  charged duration lands on the simulation grid.

A call is a sequence of *attempts*; every attempt that produced tokens pays for
them (malformed JSON and rejected messages included), transport errors and
timeouts pay pre-registered fixed costs, and a retry is charged as a further
attempt of the same call. The total is quantised once per call.

All default values are **provisional** (``CostParams.provisional``): they are
an experimental cost setting, not a measurement of this project's model
throughput. Freeze them, with the tokenizer and model settings, before the
cohort; see ``docs/zone_sim_cost.md``.

Scope: this module is pure arithmetic plus records. It performs no model calls,
touches no simulator state, and is not wired into any runner yet (runner
integration belongs to the runner owner, after PR 169).

Package A (``harness/zone_study_contract.py``) owns the LOG schema. The records
here are cost rows: :meth:`CallCostRecord.to_dict` is the auditable cost
breakdown, and :func:`contract_call_record` / :func:`contract_message_records`
re-emit the same numbers as A's ``ugrp.zone_study_call.v1`` /
``ugrp.zone_study_message_log.v1`` records, validated by A.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import hashlib
import itertools
import json
import math

from harness.zone_study_contract import CALL_STATUS
from harness.zone_study_inputs import TRIGGERS as CONTRACT_TRIGGERS, call_log_record, message_log_record

COST_SCHEMA = 'ugrp.zone_sim_cost.v1'
#: Schema of the cost rows below. These are a COST breakdown, not a log record:
#: the call/message log schema is package A's, reached through
#: :func:`contract_call_record` and :func:`contract_message_records`.
CALL_COST_SCHEMA = 'ugrp.zone_sim_cost.call_cost.v1'
MESSAGE_COST_SCHEMA = 'ugrp.zone_sim_cost.message_cost.v1'

#: Attempt outcomes. ``ok``/``invalid`` generated tokens and pay for them;
#: ``error`` (transport/proxy failure) and ``timeout`` (no reply within the
#: pre-registered limit) pay a fixed cost plus whatever tokens are reported.
OUTCOMES = ('ok', 'invalid', 'error', 'timeout')
#: Outcomes that must never be reported as a usable reply.
FAILED_OUTCOMES = ('invalid', 'error', 'timeout')
#: Attempt outcome -> package A ``CALL_STATUS`` value of the logical call.
OUTCOME_STATUS = {'ok': 'ok', 'invalid': 'invalid_json', 'error': 'http_error', 'timeout': 'timeout'}
assert set(OUTCOME_STATUS) == set(OUTCOMES)
assert set(OUTCOME_STATUS.values()) <= set(CALL_STATUS)


def _round(value):
    """Keep serialised seconds free of binary-float noise (grid is >= 1 ms)."""
    return round(float(value), 6)


@dataclass(frozen=True)
class CostParams:
    """One versioned, hashable cost setting.

    ``version`` identifies the setting in run records; ``digest()`` hashes every
    field so a sweep variant cannot be mistaken for its base. Sensitivity
    variants built by :func:`sweep` carry a derived version string.
    """

    version: str = 'zone_sim_cost.v1'
    call_overhead_s: float = 1.0
    input_token_s: float = .0002
    output_token_s: float = .02
    utterance_s: float = .3
    delivery_s: float = .1
    per_recipient_s: float = 0.
    error_s: float = .5
    timeout_s: float = 20.
    quantum_s: float = .1
    scale: float = 1.
    provisional: bool = True
    note: str = ''

    def __post_init__(self):
        if self.quantum_s <= 0:
            raise ValueError('quantum_s must be positive')
        if self.scale < 0:
            raise ValueError('scale must be >= 0')
        for name in ('call_overhead_s', 'input_token_s', 'output_token_s', 'utterance_s',
                     'delivery_s', 'per_recipient_s', 'error_s', 'timeout_s'):
            if getattr(self, name) < 0:
                raise ValueError(f'{name} must be >= 0')

    def to_dict(self):
        return {'schema': COST_SCHEMA, 'version': self.version,
                'call_overhead_s': self.call_overhead_s, 'input_token_s': self.input_token_s,
                'output_token_s': self.output_token_s, 'utterance_s': self.utterance_s,
                'delivery_s': self.delivery_s, 'per_recipient_s': self.per_recipient_s,
                'error_s': self.error_s, 'timeout_s': self.timeout_s,
                'quantum_s': self.quantum_s, 'scale': self.scale,
                'provisional': self.provisional, 'note': self.note}

    def digest(self):
        """SHA-256 of the canonical parameter JSON, for run provenance."""
        payload = json.dumps(self.to_dict(), sort_keys=True, ensure_ascii=False)
        return hashlib.sha256(payload.encode()).hexdigest()

    def variant(self, label, **changes):
        """Same setting with ``changes`` applied and a derived version string."""
        return replace(self, version=f'{self.version}+{label}', **changes)

    def min_call_s(self):
        """Smallest duration any single-attempt call can cost under this setting.

        The event scheduler uses this as a sound lower bound so it can process
        already-known events before blocking on an in-flight reply, without
        letting API arrival order reach the SIM trace.
        """
        floor = min(self.call_overhead_s, self.error_s, self.timeout_s)
        return quantize(self.scale * floor, self.quantum_s)


#: Registry of frozen settings. Add a new version instead of editing one in use.
PARAMS = {
    'zone_sim_cost.v1': CostParams(
        version='zone_sim_cost.v1',
        note='provisional development start values (2026-09-26); see docs/zone_sim_cost.md'),
    #: Diagnostic condition: talking and thinking are free, i.e. the behaviour of
    #: the current runners. Use it to show that any effect comes from the cost.
    'zone_sim_cost.v1_free': CostParams(
        version='zone_sim_cost.v1_free', scale=0.,
        note='diagnostic: zero cost, reproduces the free-thinking runners'),
}
#: Multipliers of the design's sensitivity plan; ``0`` is the free-talk diagnostic.
SWEEP_SCALES = (0., .5, 1., 2., 4.)


def params(version='zone_sim_cost.v1'):
    """Look up a frozen setting; unknown versions fail loudly."""
    try:
        return PARAMS[version]
    except KeyError:
        raise KeyError(f'unknown cost params version {version!r}; known: {sorted(PARAMS)}') from None


def quantize(seconds, quantum_s):
    """Round ``seconds`` up onto the SIM grid (``0`` stays ``0``)."""
    if quantum_s <= 0:
        raise ValueError('quantum_s must be positive')
    if seconds <= 0:
        return 0.
    # Tolerance so that an exact multiple does not jump a whole step because of
    # binary float representation (e.g. 3.7 / .1 == 36.99999999999999).
    return _round(quantum_s * math.ceil(seconds / quantum_s - 1e-9))


@dataclass(frozen=True)
class Attempt:
    """One HTTP attempt of one logical call.

    ``input_tokens`` is the frozen-tokenizer count of everything sent, including
    the image-token estimate of the map diagram and the own wrist RGB; the
    counting policy is part of the run record, not of this module.
    ``utterances`` counts non-empty messages in the reply (a broadcast to three
    recipients is one utterance).
    """

    outcome: str = 'ok'
    input_tokens: int = 0
    output_tokens: int = 0
    utterances: int = 0

    def __post_init__(self):
        if self.outcome not in OUTCOMES:
            raise ValueError(f'outcome must be one of {OUTCOMES}, got {self.outcome!r}')
        for name in ('input_tokens', 'output_tokens', 'utterances'):
            value = getattr(self, name)
            if int(value) != value or value < 0:
                raise ValueError(f'{name} must be a non-negative integer, got {value!r}')

    def to_dict(self):
        return {'outcome': self.outcome, 'input_tokens': self.input_tokens,
                'output_tokens': self.output_tokens, 'utterances': self.utterances}


@dataclass(frozen=True)
class CallCost:
    """Charged SIM seconds of one call plus its auditable breakdown."""

    sim_s: float
    raw_s: float
    params_version: str
    params_digest: str
    attempts: tuple
    breakdown: dict
    outcome: str

    def to_dict(self):
        return {'sim_s': self.sim_s, 'raw_s': self.raw_s, 'params_version': self.params_version,
                'params_digest': self.params_digest, 'outcome': self.outcome,
                'attempts': [a.to_dict() for a in self.attempts], 'breakdown': dict(self.breakdown)}

    def cost_terms(self):
        """The ``cost_terms`` object of package A's call record.

        A names ``alpha_s``, ``beta_s_per_token``, ``gamma_s_per_utterance``,
        ``output_tokens`` and ``utterances``; the rest is this module's audit
        trail (input-token price, scale, grid, per-attempt outcomes, digest).
        """
        return {'alpha_s': self.breakdown['overhead_s'],
                'beta_s_per_token': self.breakdown['output_s'],
                'gamma_s_per_utterance': self.breakdown['utterance_s'],
                'output_tokens': self.breakdown['output_tokens'],
                'utterances': self.breakdown['utterances'],
                'input_s': self.breakdown['input_s'], 'error_s': self.breakdown['error_s'],
                'timeout_s': self.breakdown['timeout_s'], 'raw_s': self.raw_s,
                'quantum_s': self.breakdown['quantum_s'], 'attempts': self.breakdown['attempts'],
                'params_version': self.params_version, 'params_digest': self.params_digest,
                'outcomes': [a.outcome for a in self.attempts]}

    @property
    def status(self):
        """Package A ``CALL_STATUS`` of the logical call (last attempt wins)."""
        return OUTCOME_STATUS[self.outcome]

    @property
    def input_tokens(self):
        """A's ``input_tokens`` object. Image/cache split is the caller's to fill."""
        return {'text': self.breakdown['input_tokens'], 'image': 0, 'cached': 0}


def attempt_cost_s(attempt, cost_params=None):
    """Unquantised SIM seconds of one attempt (see the module formula)."""
    p = cost_params or params()
    tokens = p.input_token_s * attempt.input_tokens + p.output_token_s * attempt.output_tokens
    talk = p.utterance_s * attempt.utterances
    if attempt.outcome in ('ok', 'invalid'):
        base = p.call_overhead_s
    elif attempt.outcome == 'error':
        base = p.error_s
    else:  # timeout: the pre-registered waiting cost, not the observed latency
        base = p.timeout_s
    return _round(p.scale * (base + tokens + talk))


def call_cost(attempts, cost_params=None):
    """Charge one logical call: every attempt pays, the total is quantised once.

    ``attempts`` is a non-empty sequence of :class:`Attempt`. The call outcome is
    the outcome of the last attempt, so a call that succeeded on its retry is
    ``ok`` while still paying for the failed attempt.
    """
    p = cost_params or params()
    items = tuple(attempts)
    if not items:
        raise ValueError('a call needs at least one attempt')
    raw = _round(sum(attempt_cost_s(a, p) for a in items))
    breakdown = {
        'attempts': len(items),
        'input_tokens': sum(a.input_tokens for a in items),
        'output_tokens': sum(a.output_tokens for a in items),
        'utterances': sum(a.utterances for a in items),
        'overhead_s': _round(p.scale * p.call_overhead_s * sum(a.outcome in ('ok', 'invalid') for a in items)),
        'input_s': _round(p.scale * p.input_token_s * sum(a.input_tokens for a in items)),
        'output_s': _round(p.scale * p.output_token_s * sum(a.output_tokens for a in items)),
        'utterance_s': _round(p.scale * p.utterance_s * sum(a.utterances for a in items)),
        'error_s': _round(p.scale * p.error_s * sum(a.outcome == 'error' for a in items)),
        'timeout_s': _round(p.scale * p.timeout_s * sum(a.outcome == 'timeout' for a in items)),
        'quantum_s': p.quantum_s,
    }
    return CallCost(sim_s=quantize(raw, p.quantum_s), raw_s=raw, params_version=p.version,
                    params_digest=p.digest(), attempts=items, breakdown=breakdown,
                    outcome=items[-1].outcome)


def delivery_delay_s(recipients=1, cost_params=None):
    """Quantised SIM delay from "reply charged" to "message in every inbox".

    Every recipient of one utterance receives it at the same SIM time: a
    broadcast is not serialised, which is what keeps delivery order independent
    of HTTP completion order. ``per_recipient_s`` (default ``0``) exists only so
    a sweep can price fan-out.
    """
    p = cost_params or params()
    count = max(int(recipients), 0)
    return quantize(p.scale * (p.delivery_s + p.per_recipient_s * count), p.quantum_s)


@dataclass(frozen=True)
class CallCostRecord:
    """Cost row of one logical call (see :func:`contract_call_record` for A's log).

    Flat and JSON-only so the same numbers can be re-emitted under package A's
    ``ugrp.zone_study_call.v1``. ``trigger`` is the event that made the call
    eligible (see ``harness.zone_event_scheduler``).
    """

    call_id: str
    actor: str
    trigger: str
    started_sim_s: float
    finished_sim_s: float
    cost: CallCost
    merged_triggers: tuple = ()
    retry_of: str = ''
    notes: dict = field(default_factory=dict)

    def to_dict(self):
        return {'schema': CALL_COST_SCHEMA, 'call_id': self.call_id, 'actor': self.actor,
                'trigger': self.trigger, 'merged_triggers': list(self.merged_triggers),
                'retry_of': self.retry_of,
                'started_sim_s': _round(self.started_sim_s),
                'finished_sim_s': _round(self.finished_sim_s),
                'sim_cost_s': self.cost.sim_s, 'outcome': self.cost.outcome,
                'status': self.cost.status,
                'cost': self.cost.to_dict(), 'notes': dict(self.notes)}


@dataclass(frozen=True)
class MessageCostRecord:
    """Cost row of one delivery edge (one sender -> one recipient)."""

    message_id: str
    sender: str
    recipient: str
    encoding: str
    sent_sim_s: float
    delivered_sim_s: float
    broadcast: bool = False
    call_id: str = ''

    def to_dict(self):
        return {'schema': MESSAGE_COST_SCHEMA, 'message_id': self.message_id, 'sender': self.sender,
                'recipient': self.recipient, 'encoding': self.encoding, 'broadcast': self.broadcast,
                'call_id': self.call_id, 'sent_sim_s': _round(self.sent_sim_s),
                'delivered_sim_s': _round(self.delivered_sim_s)}


# ---------------------------------------------------------------------------
# Package A log records
#
# The cost rows above are re-emitted under A's schemas, so package I reads one
# log shape and the SIM cost of a call is the same number in both places.

def contract_trigger(trigger, *, mapping=None):
    """Map a scheduler trigger label onto package A's ``TRIGGERS`` enum."""
    table = mapping if mapping is not None else TRIGGER_TO_CONTRACT
    try:
        return table[trigger]
    except KeyError:
        raise KeyError(f'no package A trigger for {trigger!r}; known: {sorted(table)}') from None


#: Scheduler trigger -> package A ``TRIGGERS``. The scheduler's labels carry a
#: merge priority A does not model, so several labels collapse onto one A value.
TRIGGER_TO_CONTRACT = {
    'start': 'start', 'failure': 'own_view_change', 'blockage': 'own_view_change',
    'timeout': 'own_timer', 'report': 'message_received', 'retry': 'execution_review',
    'idle': 'idle_review', 'timer': 'own_timer',
}
assert set(TRIGGER_TO_CONTRACT.values()) <= set(CONTRACT_TRIGGERS)


def contract_call_record(record, *, run_id, condition_name, seed, request_id, call_index,
                         input_sha256, provenance, input_tokens=None, wall_latency_s=None,
                         action_id=None, message_ids=(), decision_sources=(), status=None,
                         payload_validated=True, trigger_map=None):
    """Package A ``ugrp.zone_study_call.v1`` record of one costed call.

    ``sim_cost_s`` is the charged SIM cost, so A's own check
    (``released - requested == sim_cost_s``) proves the log and the cost model
    agree. ``wall_latency_s`` is recorded but never used for SIM order.
    """
    cost = record.cost
    return call_log_record(
        run_id=run_id, condition_name=condition_name, seed=seed, actor=record.actor,
        request_id=request_id, call_index=call_index,
        trigger=contract_trigger(record.trigger, mapping=trigger_map),
        requested_at_sim_s=_round(record.started_sim_s),
        released_at_sim_s=_round(record.started_sim_s + cost.sim_s),
        cost_terms=cost.cost_terms(), input_sha256=input_sha256,
        input_tokens=dict(input_tokens or cost.input_tokens),
        output_tokens=cost.breakdown['output_tokens'],
        status=status or cost.status, provenance=provenance,
        http_attempts=cost.breakdown['attempts'], wall_latency_s=wall_latency_s,
        action_id=action_id, message_ids=list(message_ids),
        decision_sources=list(decision_sources), payload_validated=payload_validated)


def censored_call_record(row, *, run_id, condition_name, seed, request_id, call_index, input_sha256,
                        provenance, trigger_map=None, decision_sources=()):
    """Package A call record of a call still OUTSTANDING at the horizon.

    2026-09-26 review finding 16: such a call used to vanish from
    ``self.calls``, so a condition that failed through long reasoning looked
    cheap. ``sim_cost_s`` is the SIM time that elapsed until the horizon (the
    action was never released). Second review: the API side of the call is
    finished, so the provider usage the scheduler recorded (tokens, attempts,
    utterances) is kept instead of 0; ``cost_terms['usage_known']`` is False only
    when a transport failure left it unknown, and then the counts are 0 as a
    labelled lower bound.
    """
    elapsed = _round(row['elapsed_sim_s'])
    known = bool(row.get('usage_known', False))
    cost = row.get('cost') if known else None
    breakdown = (cost or {}).get('breakdown') or {}
    terms = {'alpha_s': breakdown.get('overhead_s', 0.), 'beta_s_per_token': breakdown.get('output_s', 0.),
             'gamma_s_per_utterance': breakdown.get('utterance_s', 0.),
             'output_tokens': int(row.get('output_tokens') or 0) if known else 0,
             'utterances': int(row.get('utterances') or 0) if known else 0,
             'censored': True, 'usage_known': known, 'elapsed_sim_s': elapsed,
             'charged_sim_s': row.get('charged_sim_s') if known else None,
             'would_release_sim_s': row.get('would_release_sim_s') if known else None,
             'outcome': row.get('outcome') if known else None,
             'reason': row.get('reason', 'unfinished_at_horizon'),
             'params_version': (row.get('cost') or {}).get('params_version'),
             'params_digest': (row.get('cost') or {}).get('params_digest'),
             'note': ('SIM time elapsed until the horizon; the action was never released. Provider usage '
                      'is the recorded API usage' if known else
                      'SIM time elapsed until the horizon; the API usage is unknown (lower bound 0)')}
    return call_log_record(
        run_id=run_id, condition_name=condition_name, seed=seed, actor=row['actor'],
        request_id=request_id, call_index=call_index,
        trigger=contract_trigger(row['trigger'], mapping=trigger_map),
        requested_at_sim_s=_round(row['started_sim_s']),
        released_at_sim_s=_round(row['started_sim_s'] + elapsed),
        cost_terms=terms, input_sha256=input_sha256,
        input_tokens={'text': int(row.get('input_tokens') or 0) if known else 0, 'image': 0, 'cached': 0},
        output_tokens=terms['output_tokens'],
        status='censored', provenance=provenance,
        http_attempts=int(row.get('http_attempts') or row.get('reserved_attempts', 1)), wall_latency_s=None,
        action_id=None, message_ids=[], decision_sources=list(decision_sources),
        payload_validated=True)


def contract_message_records(records, *, run_id, condition_name, seed, envelopes, acts=None):
    """Package A ``ugrp.zone_study_message_log.v1`` records from the delivery edges.

    ``records`` are :class:`MessageCostRecord` edges (any order); ``envelopes``
    maps ``message_id`` to the package A envelope that was relayed. One A record
    per message carries every recipient's delivery time, so a broadcast is one
    row with several ``deliveries``.
    """
    grouped = {}
    for edge in records:
        grouped.setdefault(edge.message_id, []).append(edge)
    out = []
    for message_id in sorted(grouped):
        edges = sorted(grouped[message_id], key=lambda e: e.recipient)
        envelope = envelopes[message_id]
        deliveries = [{'recipient': e.recipient, 'delivered_at_sim_s': _round(e.delivered_sim_s),
                       'status': 'delivered'} for e in edges]
        earliest = min(_round(e.delivered_sim_s) for e in edges)
        out.append(message_log_record(
            run_id=run_id, condition_name=condition_name, seed=seed, envelope=envelope,
            deliveries=deliveries,
            delivery_delay_s=_round(earliest - float(envelope['created_at_sim_s'])),
            status='delivered', act=(acts or {}).get(message_id)))
    return out


# ---------------------------------------------------------------------------
# Sensitivity sweep helpers
#
# The comparison itself must be RE-RUN per setting: a different cost changes
# when robots look and act. Re-costing a stored log is an approximation only,
# and ``recost`` labels it as such.

def sweep(base=None, axis='scale', values=SWEEP_SCALES):
    """Vary one axis of ``base``, one variant per value (``scale`` by default).

    Every variant gets its own version label, including the value that equals the
    base, so a sweep point is never recorded as the unswept setting.
    """
    p = base or params()
    if axis not in p.to_dict():
        raise KeyError(f'unknown cost axis {axis!r}')
    out = []
    for value in values:
        label = f'{axis}={value:g}' if isinstance(value, (int, float)) else f'{axis}={value}'
        out.append(p.variant(label, **{axis: value}))
    return tuple(out)


def sweep_axes(base=None, axes=None):
    """One-at-a-time sweep over several axes.

    Default axes separate "frequent short utterances" from "rare long ones":
    the global ``scale``, the per-call ``call_overhead_s`` (alpha), the
    per-output-token ``output_token_s`` (beta) and the per-utterance
    ``utterance_s`` (gamma).
    """
    p = base or params()
    axes = axes or {'scale': SWEEP_SCALES,
                    'call_overhead_s': (0., .5, 1., 2., 4.),
                    'output_token_s': (0., .01, .02, .04, .08),
                    'utterance_s': (0., .3, 1., 3.)}
    out = []
    for axis, values in axes.items():
        out.extend(sweep(p, axis, values))
    return tuple(out)


def sweep_grid(base=None, axes=None):
    """Full cartesian product of ``axes`` (use sparingly; runs multiply)."""
    p = base or params()
    axes = axes or {'call_overhead_s': (.5, 1., 2.), 'output_token_s': (.01, .02, .04)}
    names = sorted(axes)
    out = []
    for combo in itertools.product(*(axes[n] for n in names)):
        changes = dict(zip(names, combo))
        label = ','.join(f'{n}={v:g}' for n, v in zip(names, combo))
        out.append(p.variant(label, **changes))
    return tuple(out)


def sensitivity(calls, variants=None):
    """Cost table of the same attempt log under several settings.

    ``calls`` is a sequence of attempt sequences (one entry per logical call).
    Returns one row per variant with the total, the per-call costs and the
    parameter digest. Marked ``approximate`` because a changed cost changes the
    trace: the real comparison re-runs the study.
    """
    rows = []
    for variant in (variants or sweep()):
        costs = [call_cost(attempts, variant) for attempts in calls]
        rows.append({'version': variant.version, 'params_digest': variant.digest(),
                     'total_sim_s': _round(sum(c.sim_s for c in costs)),
                     'calls': len(costs), 'per_call_sim_s': [c.sim_s for c in costs],
                     'approximate': True,
                     'note': 'recost of a fixed attempt log; behaviour under this setting was not re-run'})
    return rows


def recost(records, cost_params=None):
    """Re-price stored :class:`CallCostRecord` rows under another setting.

    Approximate by construction: the robots would have observed and acted at
    different times. Never report the result as a run under that setting.
    """
    p = cost_params or params()
    out = []
    for record in records:
        cost = call_cost(record.cost.attempts, p)
        out.append({'call_id': record.call_id, 'actor': record.actor,
                    'from_version': record.cost.params_version, 'to_version': p.version,
                    'was_sim_s': record.cost.sim_s, 'now_sim_s': cost.sim_s, 'approximate': True})
    return out
