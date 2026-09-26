"""Regressions of the sixth review round of PR 194: SIM accounting (Codex ``codex-194-r6`` + #229).

Each test reproduces a reported counterexample in memory and asserts the fixed
behaviour; the boundary tests next to it pin the neighbouring cases. Offline
only: no simulator, no model call, no network. The pre-fix failures and the
mutation check are recorded in
``experiments/2026-09-26-zone-study-offline-smoke/review-r6/``. The logdir and
contract v2 regressions of the same round are in
``tests/test_zone_study_review_r6_paths_contract.py``.

* P1 — ``submit()`` raising AFTER the request left refunded the reservation and
  dropped the call from the call/censor records (SIM cost 0). With an HTTP cap
  of 1 a caller that handled the exception and ran again sent 2 requests with
  0 in the ledger. Only :class:`NotSent` is a refund now. A transport that can
  tell how many requests left declares ``sent_attempts``, so a reserved retry
  that never left is no longer counted as a second send.
* Integration (#229, issue #222) — every action armed one more own re-ask
  timer, so message-triggered calls multiplied the chains and every condition
  spent its call budget. At most one re-ask timer is pending per robot now.
"""
from __future__ import annotations

import collections
import math
from pathlib import Path

import pytest

from harness import zone_event_scheduler as ds
from harness import zone_sim_cost as zc
from harness import zone_study_contract as c
from harness import zone_study_offline as off
from harness.zone_study_scenarios import load as load_scenario
from scripts import run_ci_tests

ROOT = Path(__file__).resolve().parents[1]
SEED = 601


def _provenance():
    return {'registry_sha256': c.registry_sha256(), 'order_sheet_sha256': 'a' * 64,
            'map_file_sha256': 'b' * 64, 'public_map_sha256': 'c' * 64, 'code_sha': 'deadbeef',
            'execution_bundle_id': off.EXECUTION_BUNDLE_ID, 'model': 'none-fixture-v1',
            'provider': None, 'model_settings_sha256': None, 'prompt_template_sha256': 'd' * 64,
            'cost_profile_id': 'zone_sim_cost.v1', 'input_profile_id': 'zone_study_inputs.v1'}


def _log(sched):
    return sched.contract_log(run_id='run', condition_name='peer_ko', seed=SEED, provenance=_provenance())


def _ledger_attempts(sched):
    return (sum(len(call.cost.attempts) for call in sched.calls)
            + sum(row['http_attempts'] for row in sched.censored))


# =========================================================================== #
# P1 — a submit() failure after sending is a charged call with an unknown usage

class _SubmitSendsThenRaises:
    """``submit`` sends the request (``sent`` += 1), then raises ``error()``."""

    def __init__(self, error, *, reserve_in_submit=0):
        self.error, self.reserve_in_submit = error, reserve_in_submit
        self.sent = 0
        self.replies = 0

    def submit(self, call):
        for _ in range(self.reserve_in_submit):
            if call.reserve(1):
                self.sent += 1
        self.sent += 1
        raise self.error()

    def reply(self, token):
        self.replies += 1
        raise AssertionError('reply() must not be called for a call whose submit() raised')


SUBMIT_FAILURES = {
    'plain_exception': lambda: RuntimeError('connection reset after the request was written'),
    'failure_without_attempts': lambda: ds.TransportFailure('sent, no usage', attempts=(),
                                                            usage_known=False),
    'failure_partly_known': lambda: ds.TransportFailure(
        'sent, usage of the final attempt unknown', usage_known=False,
        attempts=(zc.Attempt(outcome='invalid', input_tokens=833, output_tokens=40),)),
}


def _cap(http=1, **kw):
    return ds.CallPolicy(max_http_attempts_per_actor=http, max_calls_per_actor=5, **kw)


@pytest.mark.parametrize('failure', sorted(SUBMIT_FAILURES))
def test_r6_p1_a_submit_failure_after_sending_is_a_charged_unknown_usage_call(failure):
    transport = _SubmitSendsThenRaises(SUBMIT_FAILURES[failure])
    holds = []
    sched = ds.EventScheduler(transport, policy=_cap(), on_hold=lambda a, h, t: holds.append((a, h, t)))
    sched.trigger('r1', 'start')
    report_ = sched.run(until_s=60.0)                   # the loop goes on, nothing is raised
    assert report_.stop_reason == 'quiet' and transport.replies == 0
    call = sched.calls[0]
    assert len(sched.calls) == 1 and call.notes['usage_known'] is False and call.notes['failed'] is True
    # the attempt is counted and the cap holds: the scheduler's retry is refused
    assert transport.sent == 1 == _ledger_attempts(sched) == sched.budget.used['r1']
    assert sched.metrics['r1']['budget_refused'] == 1 and sched.budget.outstanding() == 0
    # SIM time is charged and the actor held for it
    assert call.cost.sim_s == zc.call_cost(call.cost.attempts, sched.params).sim_s > 0
    assert holds == [('r1', True, 0.0), ('r1', False, call.cost.sim_s)]
    assert sched.ledger[call.call_id]['status'] == 'failed'
    assert sched.transport_errors[0]['stage'] == 'submit'
    assert sched.transport_errors[0]['usage_known'] is False
    record = _log(sched)['calls'][0]
    c.validate_log_record(record)
    assert record['http_attempts'] == 1 and record['cost_terms']['usage_bound'] == 'lower_bound'
    if failure == 'failure_partly_known':
        assert (record['input_tokens']['text'], record['output_tokens']) == (833, 40)


def test_r6_p1_codex_counterexample_a_caller_that_runs_again_cannot_send_for_free():
    """HTTP cap 1: the caller handles the exception and runs again (review scenario)."""
    transport = _SubmitSendsThenRaises(SUBMIT_FAILURES['failure_without_attempts'])
    sched = ds.EventScheduler(transport, policy=_cap())
    sched.trigger('r1', 'start')
    for _ in range(2):
        try:
            sched.run(until_s=60.0)
            break
        except ds.TransportFailure:
            sched.trigger('r1', 'retry')
    assert transport.sent == 1 == _ledger_attempts(sched) == sched.budget.used['r1']
    assert sum(call.cost.sim_s for call in sched.calls) > 0


def test_r6_p1_a_submit_failure_at_the_horizon_is_censored_with_its_attempt():
    """The censor path settles the failed submit through the same rule."""
    transport = _SubmitSendsThenRaises(SUBMIT_FAILURES['plain_exception'])
    sched = ds.EventScheduler(transport, policy=_cap())
    sched.trigger('r1', 'start')
    sched.arm_observations(('r1',), period_s=1.0, first_at=0.4)
    sched.run(until_s=0.3)
    row = sched.censored[0]
    assert row['reason'] == 'pending' and row['usage_known'] is False and row['http_attempts'] == 1
    assert row['charged_sim_s'] == zc.call_cost([zc.Attempt(outcome='error')], sched.params).sim_s
    assert transport.sent == 1 == sched.budget.used['r1']
    record = _log(sched)['calls'][0]
    c.validate_log_record(record)
    assert record['status'] == 'censored'


def test_r6_p1_a_transport_that_reserved_a_retry_in_submit_keeps_both_when_it_raises():
    transport = _SubmitSendsThenRaises(SUBMIT_FAILURES['plain_exception'], reserve_in_submit=1)
    sched = ds.EventScheduler(transport, policy=_cap(http=2))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    assert transport.sent == 2 == _ledger_attempts(sched) == sched.budget.used['r1']
    assert sched.unreported_attempts[0]['counted'] == 2 and len(sched.calls) == 1


def test_r6_p1_the_team_cap_holds_when_two_actors_fail_in_submit():
    transport = _SubmitSendsThenRaises(SUBMIT_FAILURES['plain_exception'])
    sched = ds.EventScheduler(transport, policy=ds.CallPolicy(max_attempts_total=2, max_calls_per_actor=5))
    sched.trigger('r1', 'start')
    sched.trigger('r2', 'start')
    sched.run(until_s=60.0)
    assert transport.sent == 2 == sched.budget.used_total() == _ledger_attempts(sched)


class _NotSentInSubmit:
    def __init__(self, *, reserve_in_submit=0):
        self.reserve_in_submit, self.sent = reserve_in_submit, 0

    def submit(self, call):
        for _ in range(self.reserve_in_submit):
            call.reserve(1)
        raise ds.NotSent('payload failed validation before any request was written')

    def reply(self, token):
        raise AssertionError('never reached')


@pytest.mark.parametrize('reserve_in_submit', [0, 1])
def test_r6_p1_not_sent_is_the_only_refunded_submit_failure(reserve_in_submit):
    sched = ds.EventScheduler(_NotSentInSubmit(reserve_in_submit=reserve_in_submit), policy=_cap(http=2))
    sched.trigger('r1', 'start')
    with pytest.raises(ds.NotSent):
        sched.run(until_s=60.0)
    assert sched.budget.used['r1'] == 0 and sched.budget.outstanding() == 0
    assert sched.calls == [] and sched.censored == [] and sched.metrics['r1']['calls'] == 0
    (row,) = sched.unsent_calls
    assert row['refunded'] == 1 + reserve_in_submit and row['error'].startswith('NotSent')
    assert sched.ledger[row['call_id']]['status'] == 'not_sent'
    # the refunded budget is really available again
    sched.transport = ds.ReplayTransport({})
    sched.trigger('r1', 'retry')
    sched.run(until_s=60.0)
    assert sched.budget.used['r1'] == 1 and sched.calls[0].cost.outcome == 'ok'


def test_r6_p1_boundary_not_sent_raised_by_reply_is_not_a_refund():
    """``submit()`` returned a token, so the request was handed off."""
    class Transport:
        def submit(self, call):
            return call

        def reply(self, token):
            raise ds.NotSent('too late to prove it')

    sched = ds.EventScheduler(Transport(), policy=_cap(max_retries=0))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    assert sched.budget.used['r1'] == 1 and sched.unsent_calls == []
    assert sched.calls[0].notes['usage_known'] is False and sched.transport_errors[0]['stage'] == 'reply'


def test_r6_p1_boundary_an_interrupt_in_submit_keeps_the_reservation():
    class Transport:
        def submit(self, call):
            raise KeyboardInterrupt

        def reply(self, token):
            raise AssertionError('never reached')

    sched = ds.EventScheduler(Transport(), policy=_cap())
    sched.trigger('r1', 'start')
    with pytest.raises(KeyboardInterrupt):
        sched.run(until_s=60.0)
    (entry,) = sched.ledger.values()
    assert entry['status'] == 'interrupted' and entry['error'].startswith('KeyboardInterrupt')
    assert sched.budget.outstanding() == 1 and sched.budget.remaining('r1') == 0


# =========================================================================== #
# P1 follow-up — sent vs reserved, where the transport can tell

class _ReservesRetryButNeverSendsIt:
    """Sends the request, reserves a retry, decides NOT to send it, fails.

    ``declare`` = whether the failure states ``sent_attempts=1``; ``returns`` =
    return an unknown-usage ``CallReply`` instead of raising.
    """

    def __init__(self, *, declare, returns=False):
        self.declare, self.returns = declare, returns
        self.sent = 0

    def submit(self, call):
        self.sent += 1
        return call

    def reply(self, token):
        token.reserve(1)                           # reserved ... and then aborted locally
        sent = 1 if self.declare else None
        if self.returns:
            return ds.CallReply(attempts=(zc.Attempt(outcome='error'),), usage_known=False,
                                sent_attempts=sent)
        raise ds.TransportFailure('aborted before the retry', attempts=(), usage_known=False,
                                  sent_attempts=sent)


@pytest.mark.parametrize('returns', [False, True])
def test_r6_sent_a_declared_sent_count_refunds_a_reserved_retry_that_never_left(returns):
    transport = _ReservesRetryButNeverSendsIt(declare=True, returns=returns)
    sched = ds.EventScheduler(transport, policy=_cap(http=2))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    first, second = sched.calls           # the refund lets the scheduler's own retry run
    assert len(first.cost.attempts) == 1 and second.retry_of == first.call_id
    assert first.cost.sim_s == zc.call_cost([zc.Attempt(outcome='error')], sched.params).sim_s
    assert transport.sent == 2 == _ledger_attempts(sched) == sched.budget.used['r1']
    assert sched.unreported_attempts == []
    assert sched.ledger[first.call_id]['sent_attempts_declared'] == 1


def test_r6_sent_without_a_declaration_every_reserved_attempt_still_counts_as_sent():
    """The documented conservative rule: 1 real send is counted as 2."""
    transport = _ReservesRetryButNeverSendsIt(declare=False)
    sched = ds.EventScheduler(transport, policy=_cap(http=2))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    assert transport.sent == 1 and _ledger_attempts(sched) == 2 == sched.budget.used['r1']
    assert sched.unreported_attempts[0]['counted'] == 2 and len(sched.calls) == 1


def test_r6_sent_a_declared_count_above_the_reported_attempts_is_padded_to_it():
    class Transport:
        def submit(self, call):
            return call

        def reply(self, token):
            assert token.reserve(1)
            raise ds.TransportFailure('two left, usage of one known', usage_known=False, sent_attempts=2,
                                      attempts=(zc.Attempt(outcome='invalid', input_tokens=500),))

    sched = ds.EventScheduler(Transport(), policy=_cap(http=3, max_retries=0))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    assert [a.outcome for a in sched.calls[0].cost.attempts] == ['error', 'invalid']
    assert sched.unreported_attempts == [{'call_id': sched.calls[0].call_id, 'actor': 'r1', 'reserved': 2,
                                          'reported': 1, 'counted': 2}]


def test_r6_sent_a_declared_count_above_the_reservation_stays_a_budget_breach():
    class Transport:
        def submit(self, call):
            return call

        def reply(self, token):
            raise ds.TransportFailure('sent 3 without reserving', attempts=(), usage_known=False,
                                      sent_attempts=3)

    sched = ds.EventScheduler(Transport(), policy=_cap(http=5, max_retries=0))
    sched.trigger('r1', 'start')
    sched.run(until_s=60.0)
    assert sched.over_budget_attempts[0]['unreserved'] == 2
    assert sched.discarded[0]['reason'] == 'budget_breach'


@pytest.mark.parametrize('value', [0, -1, True, False, 1.5, '1', math.nan, math.inf, [1]])
def test_r6_sent_boundary_a_malformed_sent_count_is_refused(value):
    with pytest.raises(ValueError, match='sent_attempts'):
        ds.CallReply(attempts=(zc.Attempt(outcome='error'),), usage_known=False, sent_attempts=value)
    with pytest.raises(ValueError, match='sent_attempts'):
        ds.TransportFailure('x', attempts=(), sent_attempts=value)


def test_r6_sent_boundary_the_count_must_agree_with_the_reported_attempts():
    two = (zc.Attempt(outcome='error'),) * 2
    with pytest.raises(ValueError, match='fewer than'):
        ds.CallReply(attempts=two, usage_known=False, sent_attempts=1)
    with pytest.raises(ValueError, match='known usage'):
        ds.CallReply(attempts=(zc.Attempt(),), sent_attempts=2)
    with pytest.raises(ValueError, match='known usage'):
        ds.TransportFailure('x', attempts=two, sent_attempts=3)
    # agreeing declarations and the "cannot tell" default are accepted
    assert ds.CallReply(attempts=(zc.Attempt(),), sent_attempts=1).sent_attempts == 1
    assert ds.CallReply(attempts=(zc.Attempt(),)).sent_attempts is None
    assert ds.TransportFailure('x', attempts=(), sent_attempts=2).sent_attempts == 2
    assert ds.TransportFailure('x', attempts=two, usage_known=False, sent_attempts=5).sent_attempts == 5


# =========================================================================== #
# Integration #229 — at most one pending own re-ask timer per robot

def test_r6_reask_at_most_one_pending_timer_per_actor():
    sched = ds.EventScheduler(ds.ReplayTransport({}))
    assert sched.arm_reask('r1', 'idle', at=10.0) is True
    assert sched.arm_reask('r1', 'idle', at=12.0) is False           # pending: nothing armed
    assert sched.arm_reask('r2', 'timer', at=12.0) is True           # per actor
    timers = collections.Counter(e.payload['actor'] for e in sched._queue if e.kind == 'timer')
    assert timers == {'r1': 1, 'r2': 1}
    assert sched.reask_counts['r1'] == {'armed': 1, 'skipped': 1}
    sched.timer('r1', 'idle', at=5.0)                                 # an ordinary timer ...
    sched.run(until_s=6.0, close_at_horizon=False)
    assert sched.reask_pending('r1')                                  # ... does not clear it
    sched.run(until_s=10.5, close_at_horizon=False)
    assert not sched.reask_pending('r1') and sched.reask_pending('r2')
    assert sched.arm_reask('r1', 'idle', at=20.0) is True
    assert ds.REASK_POLICY == 'single_pending_own_timer.v1'           # the integration's identifier


@pytest.mark.parametrize('at', [None, math.nan, math.inf, -math.inf, True, '12', [12.0]])
def test_r6_reask_boundary_a_malformed_time_is_refused(at):
    sched = ds.EventScheduler(ds.ReplayTransport({}))
    with pytest.raises(ValueError, match='re-ask time'):
        sched.arm_reask('r1', 'idle', at=at)
    assert sched._queue == [] and not sched.reask_pending('r1')


def test_r6_reask_boundary_label_actor_and_past_are_checked():
    sched = ds.EventScheduler(ds.ReplayTransport({}), start_s=5.0)
    with pytest.raises(ValueError, match='re-ask label'):
        sched.arm_reask('r1', 'nap', at=10.0)
    with pytest.raises(KeyError):
        sched.arm_reask('r9', 'idle', at=10.0)
    with pytest.raises(ValueError, match='SIM past'):
        sched.arm_reask('r1', 'idle', at=4.0)
    assert sched.arm_reask('r1', 'idle', at=5.0) is True             # now is allowed, and 0 s delay
    assert sched._queue and not any(sched.reask_pending(a) for a in ('r2', 'r3'))


def _idle_timer_gaps(trial):
    fires = collections.defaultdict(list)
    for event in trial.scheduler.events:
        if event.get('kind') == 'timer':
            fires[event['actor']].append(event['sim_s'])
    return {actor: [b - a for a, b in zip(times, times[1:])] for actor, times in fires.items()}


@pytest.mark.parametrize('condition', ['no_comm', 'peer_ko', 'leader_ko', 'structured'])
def test_r6_reask_offline_trial_never_runs_parallel_reask_chains(condition):
    """The integration counterexample: message-triggered calls multiplied the chains.

    Gate P9 of the integration smoke: two re-asks of one robot are at least
    ``idle_reask_s`` apart. With one chain per action they came much closer and
    the channel conditions spent their whole budget (v4: 90/90 calls).
    """
    trial, result = off.run_trial('s1_normal_mixed', condition, SEED)
    gaps = _idle_timer_gaps(trial)
    assert gaps and all(gap >= trial.policy.idle_reask_s - 1e-9 for rows in gaps.values() for gap in rows)
    assert result.end_reason == 'sim_horizon'
    assert all(trial.scheduler.metrics[a]['budget_refused'] == 0 for a in trial.actors)
    fired = sum(1 + len(rows) for rows in gaps.values())
    armed = sum(row['armed'] for row in trial.scheduler.reask_counts.values())
    assert fired == armed - sum(trial.scheduler.reask_pending(a) for a in trial.actors)
    if condition != 'no_comm':          # messages triggered calls while a re-ask was pending
        assert sum(row['skipped'] for row in trial.scheduler.reask_counts.values()) > 0


def test_r6_reask_the_offline_bundle_id_names_the_new_rule():
    assert off.EXECUTION_BUNDLE_ID == 'zone_study_offline_v2'
    trial = off.OfflineTrial(load_scenario('s1_normal_mixed'), condition='no_comm', seed=SEED)
    assert trial.provenance['execution_bundle_id'] == 'zone_study_offline_v2'
    assert trial.provenance['registry_sha256'] == c.registry_sha256(c.CONTRACT_VERSION)



def test_r6_both_files_of_this_round_are_collected_by_the_ci_patterns():
    collected = {path.relative_to(ROOT).as_posix()
                 for pattern in run_ci_tests.TEST_PATTERNS for path in ROOT.glob(pattern)}
    assert {'tests/test_zone_study_review_r6.py', 'tests/test_zone_study_review_r6_paths_contract.py'} <= collected
