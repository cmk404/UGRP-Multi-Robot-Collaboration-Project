import copy
import json

import pytest

from harness.three_robot_plan import (ROBOTS, PAIR, TeamAgreement, fixture_plan,
    validate_plan, validate_plan_reply, build_plan_request, validate_inspection_reply,
    stage_deliveries, carry_deliveries)


def batch(agreement, turn, *, timing='during_approach'):
    pending = agreement.context()['proposal']
    return {r: dict(request_id=f'{agreement.run_id}-{r}-plan-{turn}',
        proposal_id=pending['proposal_id'] if pending else None,
        plan_hash=pending['plan_hash'] if pending else None, accept=True,
        plan=copy.deepcopy(pending['plan']) if pending else fixture_plan(timing),
        reason='test fixture', message='') for r in ROBOTS}


def test_proposal_is_not_permission_and_missing_third_ack_cannot_start():
    a = TeamAgreement('run')
    assert a.receive(batch(a, 0), 0) is None
    p = a.context()['proposal']
    assert not a.authorize(p['proposal_id'], p['plan_hash'])
    partial = batch(a, 1)
    partial['r2'] = None
    assert a.receive(partial, 1) is None
    assert a.committed is None
    assert a.receive(batch(a, 2), 2) == p
    assert a.authorize(p['proposal_id'], p['plan_hash'])


def test_rejection_rotates_proposer_without_repairing_decision():
    a = TeamAgreement('run')
    a.receive(batch(a, 0), 0)
    rejected = batch(a, 1)
    rejected['r3']['accept'] = False
    a.receive(rejected, 1)
    assert a.context()['proposer'] == 'r2'
    assert a.context()['proposal'] is None
    assert a.committed is None


@pytest.mark.parametrize('field,value', [('request_id', 'old'), ('proposal_id', 'old'),
    ('plan_hash', 'bad'), ('accept', 1)])
def test_stale_or_malformed_acks_do_not_commit(field, value):
    a = TeamAgreement('run')
    a.receive(batch(a, 0), 0)
    replies = batch(a, 1)
    replies['r2'][field] = value
    with pytest.raises(ValueError):
        a.receive(replies, 1)
    assert a.committed is None


def test_same_version_different_plan_cannot_ack_and_unseen_pairs_rejected():
    a = TeamAgreement('run')
    a.receive(batch(a, 0), 0)
    replies = batch(a, 1)
    replies['r2']['plan']['inspection']['timing'] = 'before_carry'
    with pytest.raises(ValueError, match='frozen'):
        a.receive(replies, 1)
    unsupported = fixture_plan()
    unsupported['transport']['participants'] = {'r1': 'bottom_end', 'r2': 'top_end'}
    with pytest.raises(ValueError, match='unsupported'):
        validate_plan(unsupported)


def test_revocation_invalidates_old_permission_and_requires_new_acks():
    a = TeamAgreement('run')
    a.receive(batch(a, 0), 0)
    old = a.receive(batch(a, 1), 1)
    with pytest.raises(ValueError, match='immutable'):
        a.receive(batch(a, 2), 2)
    a.invalidate('new mission constraint; executor must already be stopped')
    assert not a.authorize(old['proposal_id'], old['plan_hash'])
    a.receive(batch(a, 2, timing='before_carry'), 2)
    new = a.receive(batch(a, 3), 3)
    assert new['version'] == old['version']+1
    assert not a.authorize(old['proposal_id'], old['plan_hash'])
    assert a.authorize(new['proposal_id'], new['plan_hash'])
    assert not TeamAgreement('other-run').authorize(new['proposal_id'], new['plan_hash'])


def test_each_actor_gets_own_camera_history_and_delivered_messages_only():
    request = build_plan_request('r2', request_id='x', own_rgb=b'r2', top_rgb=b'shared',
        agreement=TeamAgreement('run').context(), own_history=[{'own': 'attempt'}],
        inbox=[{'from_robot': 'r1', 'message': 'claim'}])
    context = json.loads(request['messages'][1]['content'])
    assert set(context) == {'request_id', 'agreement', 'available_skills',
                            'received_peer_claims', 'own_issued_commands'}
    assert len(request['images']) == 2
    assert context['own_issued_commands'] == [{'own': 'attempt'}]
    assert context['received_peer_claims'][0]['message'] == 'claim'
    assert 'NOT observed robot positions' in request['messages'][0]['content']


@pytest.mark.parametrize('patch', [{'request_id': 'old'}, {'confidence': True},
    {'confidence': float('nan')}, {'status': 'PHYSICAL_SUCCESS'}, {'contacts': True}])
def test_inspection_cannot_smuggle_truth_or_stale_success(patch):
    value = dict(request_id='x', status='UNCERTAIN', confidence=.4, reason='', message='')
    value.update(patch)
    with pytest.raises(ValueError):
        validate_inspection_reply(json.dumps(value), 'x')


def test_fault_delivery_withholds_only_declared_reports_and_then_restores():
    history = []
    for skill in ('APPROACH', 'LIFT', 'LIFT', 'LIFT', 'CARRY'):
        expected = ('r1',) if skill == 'LIFT' and len(history) < 3 else PAIR
        assert stage_deliveries('ready_delay', skill, history) == expected
        history.append({'skill': skill})
    assert [i for i in range(25) if carry_deliveries('carry_report_loss', i) != PAIR] == list(range(12, 17))


def test_actual_carry_policy_holds_both_on_loss_and_requires_fresh_frames():
    from harness.pair_carry_policy import PairCarryPolicy
    policy = PairCarryPolicy()
    decisions = {r: dict(ok=True, held_estimate=True, ready=False, forward=.08) for r in PAIR}
    frames = {r: 1 for r in PAIR}
    first = policy.step(decisions, 0., frames, 0.)
    assert first['forwards'] == {'r1': .08, 'r3': .08}
    missing = policy.step(decisions, 0., {r: 2 for r in PAIR}, .2, delivered=('r1',))
    assert missing['permission']['phase'] == 'HOLD'
    assert missing['forwards'] == {'r1': 0., 'r3': 0.}
    old = policy.step(decisions, 0., frames, .4)
    assert old['forwards'] == {'r1': 0., 'r3': 0.}
    fresh = policy.step(decisions, 0., {r: 4 for r in PAIR}, .6)
    assert fresh['permission']['phase'] == 'GO'


def test_invalid_inspection_timing_and_extra_plan_fields_fail_closed():
    for edit in (lambda p: p.update(coordinates=[1, 2]),
                 lambda p: p['inspection'].update(timing='anytime')):
        p = fixture_plan()
        edit(p)
        with pytest.raises(ValueError):
            validate_plan(p)
