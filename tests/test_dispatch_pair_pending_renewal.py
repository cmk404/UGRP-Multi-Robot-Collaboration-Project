"""The physics owner may reuse a still-fresh accepted pair command, never its RGB TTL."""
from concurrent.futures import Future
from threading import get_ident
from types import SimpleNamespace
from unittest.mock import Mock
import time

import pytest

from scripts.dispatch_pair_skill import BoundPairSkill


def _frames(frame_id, observed_at_s):
    return {rid: {'frame_id': frame_id, 'observed_at_s': observed_at_s,
                  'raw_top_bytes': f'top-{frame_id}'.encode(), 'own_bytes': b'own'}
            for rid in ('r1', 'r3')}


def _ready(value):
    future = Future()
    future.set_result(value)
    return future


def _fixture(monkeypatch, captures, controls, *, end_at=.72, revoke_at=None,
             route_revoke_at=None, sync_hold_at=None, sync_epoch_at=None,
             complete_second_at=None, mutate_beam_at=None, late_after_issue=False):
    owner = get_ident()
    clock = [0.]
    granted = [True]
    route_granted = [True]
    issued = []
    history = {rid: [] for rid in ('r1', 'r3')}
    ports = {rid: SimpleNamespace(hold=Mock()) for rid in ('r1', 'r3')}
    queued = list(captures)
    remaining = list(captures)
    pair_ref = [None]
    jumped = [False]

    def step(seconds):
        assert get_ident() == owner
        if late_after_issue and issued and not jumped[0]:
            clock[0] = issued[0][0] + .27
            jumped[0] = True
        else:
            clock[0] += seconds
        if revoke_at is not None and clock[0] >= revoke_at:
            granted[0] = False
        if route_revoke_at is not None and clock[0] >= route_revoke_at:
            route_granted[0] = False
        if mutate_beam_at is not None and issued and clock[0] >= mutate_beam_at:
            pair_ref[0].carried_beam.previous = {'token': 'unaccepted'}
        if complete_second_at is not None and len(queued) >= 2:
            second = queued[1]
            if not second.done() and clock[0] >= complete_second_at:
                second.set_result(_frames(2, clock[0]))
        time.sleep(.0002)  # let the one RGB worker complete without owning physics
        if clock[0] >= end_at:
            raise RuntimeError('fake owner wall limit')

    def authorize():
        assert get_ident() == owner
        if not granted[0]:
            raise RuntimeError('plan revoked')

    def issue(commands, duration, stage):
        assert get_ident() == owner
        assert stage == 'TRANSIT' and 0 < duration <= .25
        assert set(commands) == {'r1', 'r3'}
        issued.append((clock[0], commands, duration))
        for rid in ('r1', 'r3'):
            history[rid].append({'issued_at_s': clock[0],
                                 'valid_until_s': clock[0] + duration,
                                 'action': dict(commands[rid])})

    def capture_async(_tag, **_kwargs):
        if remaining:
            return remaining.pop(0)
        return Future()

    io = SimpleNamespace(time=lambda: clock[0], step=step, ports=ports,
        realtime_control=True, capture_async=capture_async,
        pair_issue_bounded=issue, authorize=authorize, command_history=history,
        realtime_stats={'pair_backpressure': 0, 'pair_decisions': 0,
                        'max_decision_age_s': 0.})
    bindings = SimpleNamespace(cluttered=False, committed={'plan_hash': '0123456789abcdef'},
        pair={'r1': 'r1', 'r3': 'r3'}, permission=lambda _obj, _stage: route_granted[0],
        reserve_beam_apron=Mock())
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.io, pair.bindings = io, bindings
    pair.phase, pair.transport_started, pair.count, pair.calls = 'SETUP', False, 0, []
    pair.carried_beam = SimpleNamespace(previous={'token': 'accepted'})
    pair_ref[0] = pair
    pair.capture = lambda _tag: _frames(0, clock[0])
    pair._bind_capture = lambda frames, _count: frames
    monkeypatch.setattr('scripts.dispatch_pair_skill.own_payload',
                        lambda *_args, **_kwargs: (1., 0., 0.))
    monkeypatch.setattr('harness.dispatch_translation_skew.translation_skew',
                        lambda *_args: (0., {}))
    decisions = iter(controls)
    sync = SimpleNamespace(authorize=lambda now: {
        'phase': 'HOLD' if sync_hold_at is not None and now >= sync_hold_at else 'GO',
        'epoch': 1 if sync_epoch_at is not None and now >= sync_epoch_at else 0})
    monkeypatch.setattr('scripts.dispatch_pair_skill.PairCarryPolicy',
                        lambda _task: SimpleNamespace(sync=sync,
                            step=lambda *_args, **_kwargs: next(decisions)))
    navigator = SimpleNamespace(observe=lambda _raw:
        ({'forward': .08, 'left': 0.}, {'done': False, 'waypoint_index': 0}))
    return pair, navigator, clock, issued, history, ports, granted


def _control(*, forward=.08, mode='CRUISE', valid=True):
    return {'mode': mode, 'valid': valid,
            'forwards': {'r1': forward, 'r3': forward}, 'duration_s': .2,
            'abort': False, 'done': False, 'permission': {'phase': 'GO', 'epoch': 0}}


def test_slow_next_rgb_keeps_owner_physics_and_reuses_only_original_ttl(monkeypatch):
    first = _ready(_frames(1, 0.))
    pair, navigator, clock, issued, history, ports, _ = _fixture(
        monkeypatch, [first, Future()], [_control()])
    with pytest.raises(RuntimeError, match='fake owner wall limit'):
        pair.carry_realtime(navigator, max_steps=2)
    renewals = [row for row in pair.calls if row['kind'] == 'carry_pending_renewal']
    assert clock[0] >= .7  # physics owner kept advancing during slow RGB
    assert renewals
    assert all(row['source_frame_ids'] == {'r1': 1, 'r3': 1}
               and row['source_observed_at_s'] == 0.
               and row['original_rgb_deadline_s'] == pytest.approx(.6)
               and row['previous_valid_until_s'] <= row['issued_at_s'] + .02 + 1e-9
               and row['issued_at_s'] < row['valid_until_s'] <= .6 + 1e-9
               for row in renewals)
    assert len(issued) == 1 + len(renewals)
    assert all({rid: {key: action[key] for key in ('forward', 'left', 'turn')}
                for rid, action in issued[0][1].items()}
               == {rid: {key: action[key] for key in ('forward', 'left', 'turn')}
                   for rid, action in row[1].items()}
               for row in issued[1:])
    assert all(len(history[rid]) == 1 + len(renewals) and history[rid][-1]['pending_renewal']
               and history[rid][-1]['original_rgb_deadline_s'] == pytest.approx(.6)
               for rid in ('r1', 'r3'))
    assert all(port.hold.call_count >= 2 for port in ports.values())


def test_revoked_plan_or_route_stops_both_before_old_lease_expires(monkeypatch):
    first = _ready(_frames(1, 0.))
    pair, navigator, clock, issued, _history, ports, _ = _fixture(
        monkeypatch, [first, Future()], [_control()], revoke_at=.12)
    with pytest.raises(RuntimeError, match='plan revoked'):
        pair.carry_realtime(navigator, max_steps=2)
    assert .12 <= clock[0] < .25
    assert len(issued) == 1
    assert not any(row['kind'] == 'carry_pending_renewal' for row in pair.calls)
    assert all(port.hold.called for port in ports.values())


@pytest.mark.parametrize('revoke', ['route', 'sync_hold', 'sync_epoch'])
def test_revoked_resource_or_pair_authority_stops_without_renewal(monkeypatch, revoke):
    first = _ready(_frames(1, 0.))
    args = {'route': {'route_revoke_at': .12},
            'sync_hold': {'sync_hold_at': .12},
            'sync_epoch': {'sync_epoch_at': .12}}[revoke]
    pair, navigator, clock, issued, _history, ports, _ = _fixture(
        monkeypatch, [first, Future()], [_control()], **args)
    if revoke == 'route':
        with pytest.raises(RuntimeError, match='transit permission revoked'):
            pair.carry_realtime(navigator, max_steps=2)
    else:
        with pytest.raises(RuntimeError, match='fake owner wall limit'):
            pair.carry_realtime(navigator, max_steps=2)
    assert clock[0] >= .12
    assert len(issued) == 1
    assert not any(row['kind'] == 'carry_pending_renewal' for row in pair.calls)
    assert all(port.hold.called for port in ports.values())


def test_expired_original_lease_cannot_be_resumed_by_late_owner(monkeypatch):
    first = _ready(_frames(1, 0.))
    pair, navigator, _clock, issued, _history, ports, _ = _fixture(
        monkeypatch, [first, Future()], [_control()], late_after_issue=True)
    with pytest.raises(RuntimeError, match='fake owner wall limit'):
        pair.carry_realtime(navigator, max_steps=2)
    assert len(issued) == 1
    assert not any(row['kind'] == 'carry_pending_renewal' for row in pair.calls)
    assert all(port.hold.called for port in ports.values())


def test_pending_reservation_uses_last_accepted_rgb_feature(monkeypatch):
    first = _ready(_frames(1, 0.))
    pair, navigator, _clock, issued, _history, _ports, _ = _fixture(
        monkeypatch, [first, Future()], [_control()], mutate_beam_at=.1)
    with pytest.raises(RuntimeError, match='fake owner wall limit'):
        pair.carry_realtime(navigator, max_steps=2)
    assert len(issued) > 1
    assert pair.carried_beam.previous == {'token': 'unaccepted'}
    assert pair.bindings.reserve_beam_apron.call_count >= 2
    assert all(call.args == ({'token': 'accepted'},)
               for call in pair.bindings.reserve_beam_apron.call_args_list)


@pytest.mark.parametrize('control', [_control(forward=0.),
                                     _control(forward=0., mode='RECOVERY', valid=False)])
def test_zero_or_recovery_decision_never_renews(monkeypatch, control):
    first = _ready(_frames(1, 0.))
    pair, navigator, _clock, issued, _history, ports, _ = _fixture(
        monkeypatch, [first, Future()], [control])
    with pytest.raises(RuntimeError, match='fake owner wall limit'):
        pair.carry_realtime(navigator, max_steps=2)
    assert len(issued) == 1
    assert not any(row['kind'] == 'carry_pending_renewal' for row in pair.calls)
    assert all(port.hold.called for port in ports.values())


def test_fresh_second_decision_replaces_first_pending_command(monkeypatch):
    first, second, third = _ready(_frames(1, 0.)), Future(), Future()
    pair, navigator, _clock, issued, history, _ports, _ = _fixture(
        monkeypatch, [first, second, third], [_control(), _control(forward=.03)],
        end_at=1.1, complete_second_at=.36)
    with pytest.raises(RuntimeError, match='fake owner wall limit'):
        pair.carry_realtime(navigator, max_steps=3)
    carry = [row for row in pair.calls if row['kind'] == 'carry']
    renewal = [row for row in pair.calls if row['kind'] == 'carry_pending_renewal']
    assert len(carry) == 2
    assert len(issued) >= 4  # two decisions plus a pending renewal from each
    assert renewal[0]['source_frame_ids']['r1'] == 1
    assert renewal[-1]['source_frame_ids']['r1'] == 2
    second_decision_at = carry[1]['sim_time_s']
    assert all(row['issued_at_s'] < second_decision_at
               for row in renewal if row['source_frame_ids']['r1'] == 1)
    assert all(row['issued_at_s'] >= second_decision_at
               for row in renewal if row['source_frame_ids']['r1'] == 2)
    assert history['r1'][-1]['action']['forward'] == .03
    assert history['r3'][-1]['action']['forward'] == .03
