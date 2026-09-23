"""One bounded next-RGB capture while the current carry frame is analyzed."""
from concurrent.futures import Future
from threading import Event, get_ident
from types import SimpleNamespace
from unittest.mock import Mock
import time

import pytest

from scripts.dispatch_pair_skill import BoundPairSkill
from sim.snapshot_render import SnapshotBackpressure


def _frames(frame_id, observed_at_s):
    return {rid: {'frame_id': frame_id, 'observed_at_s': observed_at_s,
                  'raw_top_bytes': f'top-{frame_id}'.encode(),
                  'own_bytes': b'own'} for rid in ('r1', 'r3')}


def _completed(value):
    future = Future()
    future.set_result(value)
    return future


def _pair(monkeypatch, capture_async, *, start=0., policy=None):
    clock = [start]
    owner = get_ident()
    issued = []
    ports = {rid: SimpleNamespace(hold=Mock()) for rid in ('r1', 'r3')}

    def step(seconds):
        assert get_ident() == owner  # The visual worker never advances physics.
        clock[0] += seconds
        time.sleep(.0001)

    def issue(commands, duration, stage):
        assert get_ident() == owner
        issued.append((clock[0], commands, duration, stage))

    io = SimpleNamespace(time=lambda: clock[0], step=step, ports=ports,
        realtime_control=True, capture_async=capture_async,
        pair_issue_bounded=issue,
        realtime_stats={'pair_backpressure': 0, 'pair_decisions': 0,
                        'max_decision_age_s': 0.})
    binding = SimpleNamespace(cluttered=False,
        committed={'plan_hash': '0123456789abcdef'},
        pair={'r1': 'r1', 'r3': 'r3'}, reserve_beam_apron=Mock())
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.io, pair.bindings = io, binding
    pair.phase = 'SETUP'
    pair.transport_started = False
    pair.count = 0
    pair.calls = []
    pair.carried_beam = SimpleNamespace(previous=None)
    pair.capture = lambda _tag: _frames(0, clock[0])
    bound = []

    def bind(frames, count):
        bound.append((count, frames['r1']['frame_id']))
        return frames

    pair._bind_capture = bind
    monkeypatch.setattr('scripts.dispatch_pair_skill.own_payload',
                        lambda *_args, **_kwargs: (1., 0., 0.))
    monkeypatch.setattr('harness.dispatch_translation_skew.translation_skew',
                        lambda *_args: (0., {}))
    control = policy or {'mode': 'CRUISE', 'valid': True,
        'forwards': {'r1': .08, 'r3': .08}, 'duration_s': .2,
        'abort': False, 'done': False}
    monkeypatch.setattr('scripts.dispatch_pair_skill.PairCarryPolicy',
                        lambda _task: SimpleNamespace(
                            step=lambda *_args, **_kwargs: control))
    return pair, io, clock, issued, ports, bound


def test_next_capture_overlaps_delayed_current_compute_and_keeps_old_timestamp(monkeypatch):
    compute_started, prefetch_submitted = Event(), Event()
    issued_ref = []
    calls = []
    pending_render = Future()

    def capture_async(tag, **_kwargs):
        calls.append(tag)
        if len(calls) == 1:
            return _completed(_frames(11, 0.))
        assert compute_started.wait(2)
        assert not issued_ref  # This RGB instant precedes the first decision's command.
        prefetch_submitted.set()
        return pending_render

    pair, io, clock, issued, ports, bound = _pair(monkeypatch, capture_async)
    issued_ref = issued
    original_step = io.step

    def step(seconds):
        original_step(seconds)
        if prefetch_submitted.is_set() and not pending_render.done() and clock[0] >= .06:
            # Render/materialization finishes while the current worker runs.
            pending_render.set_result(_frames(12, 0.))

    io.step = step

    def observe(raw):
        if raw == b'top-11':
            compute_started.set()
            assert prefetch_submitted.wait(2)
        return {'forward': .08, 'left': 0.}, {'done': False}

    with pytest.raises(RuntimeError, match='decision budget exhausted'):
        pair.carry_realtime(SimpleNamespace(observe=observe), max_steps=2)
    assert calls == ['pair-1-carry', 'pair-2-carry']
    assert bound == [(1, 11), (2, 12)]
    assert len(issued) == 2
    records = [row for row in pair.calls if row['kind'] == 'carry']
    assert [row['frame_ids']['r1'] for row in records] == [11, 12]
    assert [row['observed_at_s'] for row in records] == [0., 0.]
    assert records[1]['decision_age_s'] > 0.  # Prefetch was never retimed.
    assert io.realtime_stats['pair_decisions'] == 2
    assert all(port.hold.called for port in ports.values())


def test_prefetch_backpressure_defers_to_current_owner_retry(monkeypatch):
    calls = []

    def capture_async(tag, **_kwargs):
        calls.append(tag)
        if len(calls) == 2:
            raise SnapshotBackpressure('in flight')
        frame_id = 21 if len(calls) == 1 else 22
        return _completed(_frames(frame_id, 0.))

    pair, io, _clock, issued, _ports, bound = _pair(monkeypatch, capture_async)
    navigator = SimpleNamespace(observe=lambda _raw:
        ({'forward': .08, 'left': 0.}, {'done': False}))
    with pytest.raises(RuntimeError, match='decision budget exhausted'):
        pair.carry_realtime(navigator, max_steps=2)
    assert calls == ['pair-1-carry', 'pair-2-carry', 'pair-2-carry']
    assert bound == [(1, 21), (2, 22)]
    assert len(issued) == 2
    assert io.realtime_stats['pair_backpressure'] == 1


def test_prefetched_stale_frames_hold_both_without_pair_motion(monkeypatch):
    calls = []

    def capture_async(tag, **_kwargs):
        calls.append(tag)
        return _completed(_frames(len(calls), 0.))

    pair, io, _clock, issued, ports, _bound = _pair(
        monkeypatch, capture_async, start=.7)
    navigator = SimpleNamespace(observe=lambda _raw:
        ({'forward': .08, 'left': 0.}, {'done': False}))
    with pytest.raises(RuntimeError, match='decision budget exhausted'):
        pair.carry_realtime(navigator, max_steps=2)
    assert len(calls) == 2
    assert not issued
    assert [row['kind'] for row in pair.calls].count('bounded_pair_stale_rgb') == 2
    assert all(port.hold.call_count >= 2 for port in ports.values())
    assert io.realtime_stats['pair_decisions'] == 2


def test_failed_prefetch_propagates_when_it_becomes_current(monkeypatch):
    failed = Future()
    failed.set_exception(RuntimeError('render failed'))
    calls = []

    def capture_async(tag, **_kwargs):
        calls.append(tag)
        return _completed(_frames(41, 0.)) if len(calls) == 1 else failed

    pair, io, _clock, issued, ports, bound = _pair(monkeypatch, capture_async)
    navigator = SimpleNamespace(observe=lambda _raw:
        ({'forward': .08, 'left': 0.}, {'done': False}))
    with pytest.raises(RuntimeError, match='render failed'):
        pair.carry_realtime(navigator, max_steps=2)
    assert calls == ['pair-1-carry', 'pair-2-carry']
    assert bound == [(1, 41)]
    assert len(issued) == 1
    assert io.realtime_stats['pair_decisions'] == 1
    assert all(port.hold.called for port in ports.values())


@pytest.mark.parametrize('abort', [True, False])
def test_unused_prefetch_is_cancelled_on_abort_or_compute_exception(monkeypatch, abort):
    prefetch = Future()
    submitted = []

    def capture_async(tag, **_kwargs):
        submitted.append(tag)
        return _completed(_frames(31, 0.)) if len(submitted) == 1 else prefetch

    control = {'mode': 'ABORT', 'valid': False,
        'forwards': {'r1': 0., 'r3': 0.}, 'duration_s': .2,
        'abort': True, 'done': False}
    pair, _io, _clock, issued, ports, _bound = _pair(
        monkeypatch, capture_async, policy=control)

    def observe(_raw):
        if not abort:
            raise ValueError('RGB compute failed')
        return {'forward': 0., 'left': 0.}, {'done': False}

    with pytest.raises(RuntimeError if abort else ValueError):
        pair.carry_realtime(SimpleNamespace(observe=observe), max_steps=2)
    assert submitted == ['pair-1-carry', 'pair-2-carry']
    assert prefetch.cancelled()
    assert not issued
    assert all(port.hold.called for port in ports.values())
