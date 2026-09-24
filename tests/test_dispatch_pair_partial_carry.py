"""TOP-only work never gives the pair policy an incomplete observation."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.dispatch_pair_skill import BoundPairSkill
from scripts.research_dispatch_scene import PairPartialCapture


def _ready(value):
    future = Future()
    future.set_result(value)
    return future


@pytest.mark.parametrize('fail_full', [False, True])
def test_pair_policy_waits_for_both_own_views_and_full_error_holds(monkeypatch, fail_full):
    events = []
    clock = [0.]
    top = {'frame_id': 11, 'observed_at_s': 0., 'top_bytes': b'top',
           'shared_top_rgb': {'sha256': 'top-sha'}}
    raw = {rid: {'frame_id': 11, 'observed_at_s': 0., 'top_bytes': b'top',
                 'own_bytes': b'own', 'shared_top_rgb': {'sha256': 'top-sha'}}
           for rid in ('r1', 'r3')}
    full = Future()
    ticket = PairPartialCapture(_ready(top), full)

    class Worker:
        def __init__(self, _initialization):
            events.append('spawn')
        def __enter__(self):
            return self
        def __exit__(self, *_args):
            events.append('closed')
        def wait_ready(self, _pump):
            events.append('ready')
        def submit_top(self, frame_id, observed_at_s, raw_top):
            assert (frame_id, observed_at_s, raw_top) == (11, 0., b'top')
            events.append('top_analyzed')
            return _ready({'frame_id': 11, 'observed_at_s': 0.})
        def submit_own(self, frame_id, observed_at_s, own_by_slot):
            assert full.done() and not fail_full
            assert (frame_id, observed_at_s) == (11, 0.)
            assert own_by_slot == {'r1': b'own', 'r3': b'own'}
            events.append('own_analyzed')
            return _ready({'frame_id': 11, 'observed_at_s': 0.})

    monkeypatch.setattr('harness.dispatch_pair_perception.PairPerceptionProcess', Worker)
    monkeypatch.setattr('harness.dispatch_pair_perception.detached_beam_route', lambda _n: {})

    def step(seconds):
        clock[0] += seconds
        assert 'policy' not in events or full.done()
        if not full.done() and clock[0] >= .06:
            if fail_full:
                full.set_exception(RuntimeError('own render failed'))
            else:
                full.set_result(raw)

    def policy_step(*_args, **_kwargs):
        assert full.done()
        events.append('policy')
        return {'done': True, 'abort': False, 'mode': 'DONE', 'valid': True,
                'forwards': {'r1': 0., 'r3': 0.}, 'duration_s': .2}

    monkeypatch.setattr('scripts.dispatch_pair_skill.PairCarryPolicy',
                        lambda _task: SimpleNamespace(step=policy_step))
    holds = {rid: Mock(side_effect=lambda _at, rid=rid: events.append('hold-'+rid))
             for rid in ('r1', 'r3')}
    io = SimpleNamespace(realtime_control=True, time=lambda: clock[0], step=step,
        capture_pair_partial_async=Mock(return_value=ticket),
        pair_issue_bounded=Mock(side_effect=lambda *_a: events.append('issue')),
        ports={rid: SimpleNamespace(hold=holds[rid]) for rid in holds},
        realtime_stats={'pair_backpressure': 0, 'pair_decisions': 0,
                        'max_decision_age_s': 0.})
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.io = io
    pair.bindings = SimpleNamespace(cluttered=False, pair={'r1': 'r1', 'r3': 'r3'},
        committed={'plan_hash': '123456789abcdef'}, reserve_beam_apron=Mock())
    pair.phase, pair.transport_started, pair.realtime_open_capture = 'SETUP', False, False
    pair.process_pair_perception = True
    pair.reference = b'reference'
    pair.capture = lambda _tag: {rid: {'own_bytes': b'anchor'} for rid in ('r1', 'r3')}
    pair.carried_beam = SimpleNamespace(previous=None)
    pair.beam_continuity = SimpleNamespace(previous=None)
    pair.count, pair.calls = 0, []
    pair._adopt_carry_analysis = lambda _raw, _count, _result: (
        raw, {'forward': 0., 'left': 0.}, {'done': True},
        {'r1': {'ok': True}, 'r3': {'ok': True}}, 0., {}, .0)

    if fail_full:
        with pytest.raises(RuntimeError, match='own render failed'):
            pair.carry_realtime(SimpleNamespace(), max_steps=1)
        assert 'top_analyzed' in events
        assert 'own_analyzed' not in events and 'policy' not in events and 'issue' not in events
    else:
        pair.carry_realtime(SimpleNamespace(), max_steps=1)
        assert events.index('top_analyzed') < events.index('own_analyzed') < events.index('policy') < events.index('issue')
    assert 'closed' in events
    assert holds['r1'].call_count >= 2 and holds['r3'].call_count >= 2
