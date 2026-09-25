"""Fake-clock SIM event scheduler (package D).

What is pinned here: thinking and talking spend SIM time while physics keeps
running, concurrent calls overlap instead of adding up, failures and retries also
cost, and neither the event order nor the SIM trace depends on the order in which
API replies happen to complete.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import time

import pytest

from harness import zone_sim_cost as zc
from harness.zone_event_scheduler import (CallPolicy, CallReply, EventScheduler, KIND_ORDER, Message,
                                          ReplayTransport, TRIGGERS)

ACTORS = ('r1', 'r2', 'r3')


def _reply(out=120, utt=None, action=None, messages=(), outcome='ok', tokens_in=8000, attempts=None):
    """One scripted reply.

    2026-09-26 review finding 6: the BILLED utterance count must equal the number
    of utterances the reply carries, so ``utt`` defaults to ``len(messages)`` and
    a test that wants a mismatch has to state it explicitly.
    """
    messages = tuple(messages)
    if utt is None:
        utt = len(messages)
    return CallReply(attempts=attempts or (zc.Attempt(outcome=outcome, input_tokens=tokens_in,
                                                      output_tokens=out, utterances=utt),),
                     action=action, messages=messages)


def _scheduler(replies, *, params=None, policy=None, track=True, **kw):
    """Scheduler with a scripted transport and a recorded physics callback."""
    steps = []
    transport = ReplayTransport(replies)
    sched = EventScheduler(transport, cost_params=params, policy=policy, actors=ACTORS,
                           advance=(lambda a, b: steps.append((a, b))) if track else None, **kw)
    sched.physics_steps = steps
    return sched


# ---------------------------------------------------------------------------
# Zero and positive cost

def test_zero_cost_finishes_in_the_same_sim_instant():
    free = zc.params('zone_sim_cost.v1_free')
    sched = _scheduler({'r1': [_reply(action='go A')]}, params=free)
    sched.trigger('r1', 'start')
    report = sched.run(until_s=100)
    assert report.sim_s == 0. and sched.calls[0].cost.sim_s == 0.
    assert sched.calls[0].finished_sim_s == sched.calls[0].started_sim_s == 0.
    assert sched.physics_steps == []                      # nothing to step through


def test_positive_cost_moves_sim_time_and_physics_runs_through_the_wait():
    sched = _scheduler({'r1': [_reply(out=120, action='go A',
                                      messages=[Message(sender='r1', recipients=('r2',), body='보고')])]})
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    call = sched.calls[0]
    assert call.started_sim_s == 0. and call.finished_sim_s == pytest.approx(5.3)
    assert call.cost.sim_s == pytest.approx(5.3)          # 1.0 + 1.6 + 2.4 + 0.3
    # physics was advanced across the whole wait, monotonically and without gaps
    assert sched.physics_steps[0][0] == 0.
    assert all(b > a for a, b in sched.physics_steps)
    assert [b for a, b in sched.physics_steps[:-1]] == [a for a, b in sched.physics_steps[1:]]
    assert sched.physics_steps[-1][1] == pytest.approx(sched.now())


def test_the_calling_actor_holds_while_it_thinks_and_is_released_at_the_charged_time():
    holds = []
    sched = _scheduler({'r1': [_reply(out=100)]}, on_hold=lambda a, h, t: holds.append((a, h, t)))
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    assert holds == [('r1', True, 0.), ('r1', False, pytest.approx(4.6))]
    assert sched.holds[0]['from_sim_s'] == 0. and sched.holds[0]['to_sim_s'] == pytest.approx(4.6)
    assert sched.holding() == ()                          # nobody is thinking once it is done


def test_action_and_messages_are_invisible_before_the_cost_is_paid():
    applied, delivered = [], []
    reply = _reply(out=120, utt=1, action='go A',
                   messages=[Message(sender='r1', recipients=('r2',), body='왼쪽 통로 막힘')])
    sched = _scheduler({'r1': [reply]}, policy=CallPolicy(trigger_on_message=False),
                       on_action=lambda a, act, t: applied.append((a, act, t)),
                       on_message=lambda a, m, t: delivered.append((a, m['body'], t)))
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    assert applied == [('r1', 'go A', pytest.approx(5.3))]
    assert delivered == [('r2', '왼쪽 통로 막힘', pytest.approx(5.4))]   # + delivery_s
    assert sched.inbox('r2')[0]['delivered_sim_s'] == pytest.approx(5.4)
    assert sched.inbox('r3') == ()                        # not a recipient


# ---------------------------------------------------------------------------
# Concurrency

def test_concurrent_calls_overlap_instead_of_adding_up():
    # silent replies: the utterance term is billed against real messages now
    # (review finding 6), and this test is about overlap, not about talking.
    replies = {a: [_reply(out=120)] for a in ACTORS}
    sched = _scheduler(replies)
    for actor in ACTORS:
        sched.trigger(actor, 'start')
    sched.run(until_s=100)
    assert {c.actor for c in sched.calls} == set(ACTORS)
    assert [c.cost.sim_s for c in sched.calls] == [pytest.approx(5.0)] * 3
    assert sched.now() == pytest.approx(5.0)              # not 15.0
    assert sum(sched.metrics[a]['thinking_sim_s'] for a in ACTORS) == pytest.approx(15.0)


def test_different_reply_lengths_finish_at_different_sim_times():
    sched = _scheduler({'r1': [_reply(out=40)], 'r2': [_reply(out=120)], 'r3': [_reply(out=600)]})
    for actor in ACTORS:
        sched.trigger(actor, 'start')
        sched.run(until_s=.0)                             # start all three at t=0
    sched.run(until_s=100)
    done = {c.actor: c.finished_sim_s for c in sched.calls}
    assert done['r1'] < done['r2'] < done['r3']
    assert done == {'r1': pytest.approx(3.4), 'r2': pytest.approx(5.0), 'r3': pytest.approx(14.6)}


# ---------------------------------------------------------------------------
# Order independence: the core requirement

class ScrambledTransport:
    """Transport whose work completes in a chosen order, unrelated to SIM order.

    ``reply`` drains its own completion list until the requested handle is
    "finished", recording the wall order so a test can show it differs between
    runs that must share one SIM trace.
    """

    def __init__(self, replies, completion_order):
        self.replies, self.completion_order = replies, list(completion_order)
        self.wall_order, self._done = [], []
        self._queues = {}

    def submit(self, call):
        return call

    def reply(self, token):
        for actor in self.completion_order:
            if token.actor in self._done:
                break
            if actor not in self._done:
                self._done.append(actor)
                self.wall_order.append(actor)
        queue = self._queues.setdefault(token.actor, list(self.replies[token.actor]))
        return queue.pop(0) if queue else CallReply()


def _three_way_broadcast_replies():
    return {'r1': [_reply(out=120, utt=1, action='r1:A',
                          messages=[Message(sender='r1', recipients=('r2', 'r3'), body='r1 보고')])],
            'r2': [_reply(out=120, utt=1, action='r2:B',
                          messages=[Message(sender='r2', recipients=('r1', 'r3'), body='r2 보고')])],
            'r3': [_reply(out=120, utt=1, action='r3:C',
                          messages=[Message(sender='r3', recipients=('r1', 'r2'), body='r3 보고')])]}


def _run_with_completion_order(order):
    transport = ScrambledTransport(_three_way_broadcast_replies(), order)
    sched = EventScheduler(transport, actors=ACTORS, policy=CallPolicy(max_calls_per_actor=2))
    for actor in ACTORS:
        sched.trigger(actor, 'start')
    sched.run(until_s=60)
    return sched, transport


def test_reordered_api_completions_give_the_same_sim_trace():
    forward, t_forward = _run_with_completion_order(('r1', 'r2', 'r3'))
    reverse, t_reverse = _run_with_completion_order(('r3', 'r2', 'r1'))
    assert t_forward.wall_order != t_reverse.wall_order          # the two APIs really did differ
    assert forward.trace() == reverse.trace()
    assert forward.metrics == reverse.metrics
    assert [m.to_dict() for m in forward.messages] == [m.to_dict() for m in reverse.messages]
    assert [c.to_dict() for c in forward.calls] == [c.to_dict() for c in reverse.calls]
    assert forward.now() == reverse.now()


def test_delivery_order_at_one_sim_instant_follows_sender_rank_not_arrival():
    reverse, _ = _run_with_completion_order(('r3', 'r2', 'r1'))
    at_first_instant = [m for m in reverse.messages if m.delivered_sim_s == pytest.approx(5.4)]
    assert [(m.sender, m.recipient) for m in at_first_instant] == [
        ('r1', 'r2'), ('r1', 'r3'), ('r2', 'r1'), ('r2', 'r3'), ('r3', 'r1'), ('r3', 'r2')]


class ThreadedTransport:
    """Real futures with real, deliberately unequal wall latencies."""

    def __init__(self, replies, latency_s):
        self.replies, self.latency_s = replies, latency_s
        self.pool = ThreadPoolExecutor(max_workers=3)
        self.wall_order = []
        self._queues = {}

    def submit(self, call):
        def work():
            time.sleep(self.latency_s[call.actor])
            self.wall_order.append(call.actor)
            queue = self._queues.setdefault(call.actor, list(self.replies[call.actor]))
            return queue.pop(0) if queue else CallReply()
        return self.pool.submit(work)

    def reply(self, token):
        return token.result()


def _run_threaded(latency_s):
    transport = ThreadedTransport(_three_way_broadcast_replies(), latency_s)
    sched = EventScheduler(transport, actors=ACTORS, policy=CallPolicy(trigger_on_message=False))
    for actor in ACTORS:
        sched.trigger(actor, 'start')
    sched.run(until_s=60)
    transport.pool.shutdown()
    return sched, transport


def test_real_out_of_order_http_latency_does_not_change_the_sim_outcome():
    slow_last, t_a = _run_threaded({'r1': .002, 'r2': .04, 'r3': .08})
    slow_first, t_b = _run_threaded({'r1': .08, 'r2': .04, 'r3': .002})
    assert t_a.wall_order == ['r1', 'r2', 'r3'] and t_b.wall_order == ['r3', 'r2', 'r1']
    assert slow_last.trace() == slow_first.trace()
    assert slow_last.now() == slow_first.now() == pytest.approx(5.4)   # 5.3 thinking + 0.1 delivery
    assert [m.to_dict() for m in slow_last.messages] == [m.to_dict() for m in slow_first.messages]


def test_the_scheduler_never_reads_a_wall_clock():
    import harness.zone_event_scheduler as module
    assert not hasattr(module, 'time')


def test_repeated_identical_runs_are_byte_identical():
    first, _ = _run_with_completion_order(('r2', 'r1', 'r3'))
    second, _ = _run_with_completion_order(('r2', 'r1', 'r3'))
    assert first.trace() == second.trace()


# ---------------------------------------------------------------------------
# Broadcast

def test_broadcast_is_one_utterance_and_three_delivery_edges_at_one_sim_time():
    reply = _reply(out=120, utt=1, messages=[Message(sender='r1', recipients=('r2', 'r3'), body='보고')])
    sched = _scheduler({'r1': [reply]}, policy=CallPolicy(trigger_on_message=False))
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    assert sched.metrics['r1']['utterances'] == 1
    assert sched.metrics['r1']['messages_sent'] == 1 and sched.metrics['r1']['broadcasts'] == 1
    assert sched.metrics['r1']['delivery_edges_out'] == 2
    assert [m.delivered_sim_s for m in sched.messages] == [pytest.approx(5.4), pytest.approx(5.4)]
    assert all(m.broadcast for m in sched.messages)
    assert sched.metrics['r2']['messages_received'] == sched.metrics['r3']['messages_received'] == 1


def test_a_message_may_not_be_addressed_to_its_sender_or_to_nobody():
    with pytest.raises(ValueError):
        Message(sender='r1', recipients=('r1',))
    with pytest.raises(ValueError):
        Message(sender='r1', recipients=())
    with pytest.raises(ValueError):
        Message(sender='r1', recipients=('r2',), encoding='english')


def test_delivering_to_an_unknown_actor_fails_loudly():
    reply = _reply(messages=[Message(sender='r1', recipients=('r4',))])
    sched = _scheduler({'r1': [reply]})
    sched.trigger('r1', 'start')
    with pytest.raises(KeyError):
        sched.run(until_s=100)


# ---------------------------------------------------------------------------
# Errors, retries, timeouts

def test_a_malformed_reply_pays_and_its_retry_is_a_separate_costed_call():
    sched = _scheduler({'r1': [_reply(out=40, outcome='invalid'), _reply(out=120, action='go A')]})
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    first, retry = sched.calls
    assert first.cost.outcome == 'invalid' and first.cost.sim_s == pytest.approx(3.4)  # 1.0+1.6+0.8
    assert retry.trigger == 'retry' and retry.retry_of == first.call_id
    # the retry starts when the failed call finished (already past the 2 s minimum interval)
    assert retry.started_sim_s == pytest.approx(3.4) and retry.finished_sim_s == pytest.approx(8.4)
    assert sched.metrics['r1'] == {**sched.metrics['r1'], 'calls': 2, 'retries': 1, 'invalid': 1}
    assert sched.metrics['r1']['thinking_sim_s'] == pytest.approx(8.4)


def test_a_transport_error_costs_the_pre_registered_error_time():
    sched = _scheduler({'r1': [_reply(outcome='error', tokens_in=0, out=0), _reply(out=40)]})
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    assert sched.calls[0].cost.sim_s == pytest.approx(.5)
    assert sched.calls[0].cost.outcome == 'error' and sched.metrics['r1']['errors'] == 1
    assert sched.calls[1].started_sim_s == pytest.approx(2.0)     # min interval after the first start
    assert sched.now() > 0.


def test_a_raising_transport_is_charged_as_an_error_and_recorded():
    class Broken:
        def submit(self, call):
            return call

        def reply(self, token):
            raise RuntimeError('proxy down')

    sched = EventScheduler(Broken(), actors=ACTORS, policy=CallPolicy(max_retries=0))
    sched.trigger('r2', 'start')
    sched.run(until_s=100)
    assert sched.calls[0].cost.outcome == 'error'
    assert sched.calls[0].cost.sim_s == pytest.approx(.5)
    assert sched.transport_errors[0]['error'] == 'RuntimeError: proxy down'


def test_a_timeout_costs_the_pre_registered_limit_and_re_triggers_as_timeout():
    sched = _scheduler({'r3': [_reply(outcome='timeout', tokens_in=0, out=0), _reply(out=40)]})
    sched.trigger('r3', 'start')
    sched.run(until_s=100)
    assert sched.calls[0].cost.sim_s == pytest.approx(20.)
    assert sched.metrics['r3']['timeouts'] == 1
    assert sched.calls[1].trigger == 'timeout' and sched.calls[1].started_sim_s == pytest.approx(20.)


def test_retries_are_bounded_and_exhaustion_is_logged():
    always_bad = [_reply(out=10, outcome='invalid') for _ in range(5)]
    sched = _scheduler({'r1': always_bad}, policy=CallPolicy(max_retries=1))
    sched.trigger('r1', 'start')
    sched.run(until_s=200)
    assert len(sched.calls) == 2                            # original + one retry
    assert any('retry_exhausted' in line for line in sched.trace())
    assert sched.metrics['r1']['invalid'] == 2


# ---------------------------------------------------------------------------
# Eligibility, merging, budget

def test_the_same_actor_respects_its_minimum_call_interval():
    sched = _scheduler({'r1': [_reply(out=0, tokens_in=0), _reply(out=0, tokens_in=0)]})
    sched.trigger('r1', 'start')
    sched.run(until_s=1.)                                   # first call done at 1.0
    sched.trigger('r1', 'idle')
    sched.run(until_s=100)
    assert [c.started_sim_s for c in sched.calls] == [0., pytest.approx(2.)]
    assert sched.metrics['r1']['rate_limited'] == 1


def test_triggers_that_arrive_while_thinking_merge_into_one_call_with_the_strongest_label():
    sched = _scheduler({'r1': [_reply(out=120), _reply(out=0, tokens_in=0)]})
    sched.trigger('r1', 'start')
    sched.run(until_s=1.)                                   # r1 is thinking until 5.0
    for label in ('idle', 'report', 'failure'):
        sched.trigger('r1', label)
        sched.run(until_s=1.)
    assert sched.metrics['r1']['deferred'] == 3
    sched.run(until_s=100)
    assert len(sched.calls) == 2
    assert sched.calls[1].trigger == 'failure'              # highest merge priority
    assert set(sched.calls[1].merged_triggers) == {'idle', 'report'}
    assert sched.metrics['r1']['merged_triggers'] == 2
    assert TRIGGERS['failure'] > TRIGGERS['report'] > TRIGGERS['idle']


def test_only_one_call_per_actor_is_outstanding():
    sched = _scheduler({'r1': [_reply(out=120), _reply(out=120)]})
    sched.trigger('r1', 'start')
    sched.run(until_s=1.)
    sched.trigger('r1', 'blockage')
    sched.run(until_s=2.)
    assert sched.holding() == ('r1',) and len(sched.calls) == 0
    sched.run(until_s=100)
    assert [c.started_sim_s for c in sched.calls] == [0., pytest.approx(5.0)]


def test_the_call_budget_refuses_further_calls_instead_of_silently_dropping_them():
    sched = _scheduler({'r1': [_reply(out=0, tokens_in=0) for _ in range(4)]},
                       policy=CallPolicy(max_calls_per_actor=2, min_interval_s=0.))
    for _ in range(4):
        sched.trigger('r1', 'idle')
        sched.run(until_s=100)
    assert len(sched.calls) == 2 and sched.metrics['r1']['budget_refused'] == 2
    assert any('call_refused' in line and 'budget' in line for line in sched.trace())


def test_the_total_attempt_budget_also_stops_calls():
    sched = _scheduler({a: [_reply(out=0, tokens_in=0) for _ in range(5)] for a in ACTORS},
                       policy=CallPolicy(max_attempts_total=3, min_interval_s=0.))
    for _ in range(3):
        for actor in ACTORS:
            sched.trigger(actor, 'idle')
            sched.run(until_s=100)
    assert sum(len([c for c in sched.calls if c.actor == a]) for a in ACTORS) == 3
    assert sum(sched.metrics[a]['budget_refused'] for a in ACTORS) == 6


# ---------------------------------------------------------------------------
# Triggers, timers, observation, loop control

@pytest.mark.parametrize('trigger', ['start', 'idle', 'report', 'blockage', 'failure', 'timeout'])
def test_every_event_trigger_of_the_study_is_accepted(trigger):
    sched = _scheduler({'r2': [_reply(out=0, tokens_in=0)]})
    sched.trigger('r2', trigger)
    sched.run(until_s=10)
    assert sched.calls[0].trigger == trigger


def test_unknown_triggers_actors_and_past_times_are_rejected():
    sched = _scheduler({'r1': [_reply()]})
    with pytest.raises(ValueError):
        sched.trigger('r1', 'gossip')
    with pytest.raises(KeyError):
        sched.trigger('r9', 'idle')
    sched.trigger('r1', 'start')
    sched.run(until_s=10)
    with pytest.raises(ValueError):
        sched.trigger('r1', 'idle', at=0.)


def test_a_local_timer_fires_a_call_at_its_sim_deadline():
    fired = []
    sched = _scheduler({'r1': [_reply(out=0, tokens_in=0)]}, on_timer=lambda a, l, t: fired.append((a, l, t)))
    sched.timer('r1', 'idle', delay_s=CallPolicy().idle_reask_s)
    sched.run(until_s=100)
    assert fired == [('r1', 'idle', pytest.approx(10.))]
    assert sched.calls[0].started_sim_s == pytest.approx(10.) and sched.calls[0].trigger == 'idle'


def test_the_observation_tick_re_arms_itself_at_its_period():
    seen = []
    sched = _scheduler({}, on_observe=lambda a, t: seen.append((a, t)))
    sched.arm_observations(('r1',))
    report = sched.run(until_s=3.5)
    assert [t for _, t in seen] == [pytest.approx(1.), pytest.approx(2.), pytest.approx(3.)]
    assert report.stop_reason == 'until' and sched.now() == pytest.approx(3.5)
    with pytest.raises(ValueError):
        sched.arm_observations(('r1',), period_s=0.)


def test_run_stops_on_a_quiet_queue_on_until_and_on_max_events():
    sched = _scheduler({'r1': [_reply(out=0, tokens_in=0)]})
    assert sched.run().stop_reason == 'quiet'
    sched.trigger('r1', 'start')
    assert sched.run(max_events=1).stop_reason == 'max_events'
    assert sched.run(until_s=.5).stop_reason == 'until'
    assert sched.run(until_s=100).stop_reason == 'quiet'


def test_deliveries_are_processed_before_calls_that_start_at_the_same_sim_time():
    assert KIND_ORDER['message'] < KIND_ORDER['call_done'] < KIND_ORDER['call_start']
    reply = _reply(out=120, utt=1, messages=[Message(sender='r1', recipients=('r2',), body='보고')])
    sched = _scheduler({'r1': [reply], 'r2': [_reply(out=0, tokens_in=0)]})
    sched.trigger('r1', 'start')
    sched.run(until_s=100)
    # r2 was woken by the delivery, so its call saw the message already in its inbox
    lines = [line.split(None, 1)[1] for line in sched.trace()]
    assert lines.index('deliver r1->r2 call-0001-r1-m1') < lines.index('call_start r2 report call-0002-r2')
    assert sched.calls[1].trigger == 'report'
