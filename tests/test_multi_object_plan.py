import copy
from dataclasses import replace

import pytest

from harness.multi_object_plan import MissionProtocol, validate_mission, validate_plan
from harness.three_robot_plan import ROBOTS, TeamAgreement, digest
from scripts.prepare_multi_object_pilot import run_fixture
from sim.multi_object_suite import fixture_plan, load_pilot, make_mission


def mission(name='mixed_pair'):
    return next(c['mission'] for c in load_pilot()[1] if c['id'] == name)


def agree(m, p=None, *, acknowledgers=ROBOTS):
    p = fixture_plan(m) if p is None else p
    team = TeamAgreement('test', plan_validator=lambda value: validate_plan(value, m))
    for turn in (0, 1):
        pending = team.pending
        team.receive({r: {'request_id': f'test-{r}-plan-{turn}',
            'proposal_id': pending['proposal_id'] if pending else None,
            'plan_hash': pending['plan_hash'] if pending else None, 'accept': True, 'plan': p,
            'reason': 'fixture', 'message': ''} for r in (ROBOTS if turn == 0 else acknowledgers)}, turn)
    return team


def protocol(name='mixed_pair', *, plan=None):
    m = mission(name)
    return MissionProtocol(m, agree(m, plan))


def report(c, tid, rid, now=1., **overrides):
    value = {'proposal_id': c.committed['proposal_id'], 'plan_hash': c.committed['plan_hash'],
             'object_id': c.jobs[tid]['object_id'], 'sequence': c.sequences.get(rid, -1)+1,
             'observed_at_s': now, 'now_s': now, 'own_rgb_ref': f'{rid}/own-{now}.jpg',
             'top_rgb_ref': f'top-{now}.jpg'}
    return {**value, **overrides}


def ready(c, tid, now=1.):
    for rid in c.tasks[tid]['participants']:
        c.report_ready(tid, rid, **report(c, tid, rid, now))


def finish(c, ticket, now=1.):
    for rid in c.tasks[ticket.task_id]['participants']:
        c.report_done(ticket, rid, **report(c, ticket.task_id, rid, now))


@pytest.mark.parametrize('name', [c['id'] for c in load_pilot()[1]])
def test_all_templates_complete_with_three_distinct_allocations(name):
    m = mission(name)
    plans = set()
    for rotation in range(3):
        result = run_fixture(m, rotation)
        assert result['summary']['whole_mission_claimed']
        assert result['summary']['final_object_claims'] == len(m['objects'])
        assert result['summary']['physical_success'] == 'not_evaluated'
        assert result['summary']['transport_attempts'] == 0
        plans.add(digest(result['fixture_plan']))
    assert len(plans) == 3


def test_twelve_templates_keep_object_count_separate_from_task_count():
    _, cases = load_pilot()
    assert len(cases) == 12
    assert {len(c['mission']['objects']) for c in cases} == {1, 2, 3, 4, 5, 6, 8}
    m = mission('staging_three')
    assert len(m['objects']) == 3 and len(m['tasks']) == 5


@pytest.mark.parametrize('mutation', ['extra', 'object_extra', 'task_extra', 'missing', 'duplicate',
                                     'cycle', 'unknown', 'same_slot', 'unordered_object', 'no_final'])
def test_mission_rejects_leaks_missing_work_and_invalid_dependencies(mutation):
    m = mission('staging_three')
    if mutation == 'extra': m['evaluation_only'] = {'success': True}
    if mutation == 'object_extra': m['objects'][0]['position_m'] = [0, 0]
    if mutation == 'task_extra': m['tasks'][0]['contact'] = True
    if mutation == 'missing': m['tasks'].pop()
    if mutation == 'duplicate': m['objects'].append(copy.deepcopy(m['objects'][0]))
    if mutation == 'cycle': m['tasks'][0]['after'] = ['deliver_02']
    if mutation == 'unknown': m['tasks'][0]['after'] = ['missing']
    if mutation == 'same_slot': m['tasks'][-1]['destination_id'] = m['tasks'][-2]['destination_id']
    if mutation == 'unordered_object':
        m['tasks'].append({'task_id': 'parallel_stage', 'object_id': 'object_02',
                           'destination_id': 'staging_03', 'after': []})
        m['tasks'][-3]['after'].append('parallel_stage')
    if mutation == 'no_final': m['destinations'][0]['kind'] = 'staging'
    with pytest.raises(ValueError): validate_mission(m)


@pytest.mark.parametrize('mutation', ['hash', 'missing', 'duplicate', 'robot', 'role', 'route', 'edge', 'cycle', 'extra'])
def test_plan_rejects_unbound_or_incomplete_plans(mutation):
    m = mission('staging_three')
    p = fixture_plan(m)
    if mutation == 'hash': p['mission_sha256'] = 'bad'
    if mutation == 'missing': p['tasks'].pop()
    if mutation == 'duplicate': p['tasks'].append(copy.deepcopy(p['tasks'][0]))
    if mutation == 'robot': p['tasks'][0]['participants'] = {'r9': 'solo'}
    if mutation == 'role': p['tasks'][2]['participants'] = {'r1': 'end_a', 'r2': 'end_a'}
    if mutation == 'route': p['tasks'][0]['route_id'] = 'north'
    if mutation == 'edge': p['tasks'][2]['after'] = []
    if mutation == 'cycle': p['tasks'][0]['after'] = ['deliver_02']
    if mutation == 'extra': p['tasks'][0]['live_xy'] = [0, 0]
    with pytest.raises(ValueError): validate_plan(p, m)


def test_missing_unanimous_ack_cannot_start_protocol():
    m = mission()
    team = agree(m, acknowledgers=('r1', 'r2'))
    assert team.committed is None
    with pytest.raises(ValueError, match='unanimously'): MissionProtocol(m, team)


def test_pair_and_solo_can_run_concurrently_on_disjoint_logical_routes():
    c = protocol()
    for tid in c.tasks: ready(c, tid)
    tickets = c.grant_ready(1.)
    assert len(tickets) == 2
    for ticket in tickets: finish(c, ticket)
    assert c.summary()['whole_mission_claimed']


def test_pair_waits_for_peer_and_next_pair_reuses_robots_only_after_completion():
    c = protocol('two_beams')
    tids = list(c.tasks)
    rid = next(iter(c.tasks[tids[0]]['participants']))
    c.report_ready(tids[0], rid, **report(c, tids[0], rid))
    assert c.grant_ready(1.) == []
    for tid in tids: ready(c, tid)
    ticket, = c.grant_ready(1.)
    assert ticket.task_id == tids[0]
    assert not c.report_done(ticket, rid, **report(c, ticket.task_id, rid))
    assert c.grant_ready(1.) == [] and c.locks
    other = next(r for r in c.tasks[ticket.task_id]['participants'] if r != rid)
    assert c.report_done(ticket, other, **report(c, ticket.task_id, other))
    next_ticket, = c.grant_ready(1.)
    assert next_ticket.task_id == tids[1]
    with pytest.raises(ValueError, match='old'): c.report_done(ticket, rid, **report(c, tids[0], rid))


def test_shared_gate_is_exclusive_and_plan_order_controls_priority():
    m = mission('six_boxes')
    p = fixture_plan(m)
    p['tasks'].reverse()
    c = MissionProtocol(m, agree(m, p))
    for tid in c.tasks: ready(c, tid)
    ticket, = c.grant_ready(1.)
    assert ticket.task_id == p['tasks'][0]['task_id']
    assert c.locks['route:shared_gate'] == ticket.task_id


def test_clear_box_precedence_overrides_beam_first_proposal_order():
    c = protocol('beam_heavy_four')
    for tid in c.tasks: ready(c, tid)
    ticket, = c.grant_ready(1.)
    assert c.jobs[ticket.task_id]['object_id'] == 'object_04'
    finish(c, ticket)
    assert c.grant_ready(1.)


def test_staging_claim_does_not_count_as_final_delivery():
    c = protocol('staging_three')
    for tid in c.tasks: ready(c, tid)
    tickets = c.grant_ready(1.)
    assert {t.task_id for t in tickets} == {'stage_02', 'stage_03'}
    for ticket in tickets: finish(c, ticket)
    assert c.summary()['completed_task_claims'] == 2
    assert c.summary()['final_object_claims'] == 0
    assert not c.summary()['whole_mission_claimed']
    assert c.residents == {'staging_02': 'object_02', 'staging_03': 'object_03'}


@pytest.mark.parametrize('bad', [dict(proposal_id='old'), dict(plan_hash='old'), dict(object_id='object_99'),
                              dict(sequence=True), dict(sequence=-1), dict(observed_at_s=2.),
                              dict(observed_at_s=float('nan')), dict(own_rgb_ref=''), dict(top_rgb_ref='')])
def test_bad_reports_cannot_acquire_resources(bad):
    c = protocol('single_box')
    tid = next(iter(c.tasks))
    rid = next(iter(c.tasks[tid]['participants']))
    with pytest.raises(ValueError): c.report_ready(tid, rid, **report(c, tid, rid, **bad))
    assert not c.ready and not c.locks and not c.sequences


def test_stale_readiness_and_wrong_task_cannot_be_reused():
    c = protocol('two_beams')
    tid = next(iter(c.tasks))
    ready(c, tid)
    assert c.grant_ready(4.) == []
    ready(c, tid, 4.)
    ticket, = c.grant_ready(4.)
    rid = next(iter(c.tasks[tid]['participants']))
    forged = replace(ticket, task_id='deliver_02')
    with pytest.raises(ValueError, match='old'): c.report_done(forged, rid, **report(c, tid, rid, 4.))
    with pytest.raises(ValueError, match='backwards'): c.grant_ready(3.)


def test_late_peer_requires_fresh_completion_claim_from_first_peer():
    c = protocol('single_beam')
    tid = next(iter(c.tasks))
    ready(c, tid)
    ticket, = c.grant_ready(1.)
    a, b = c.tasks[tid]['participants']
    assert not c.report_done(ticket, a, **report(c, tid, a, 1.))
    assert not c.report_done(ticket, b, **report(c, tid, b, 4.))
    assert c.locks and not c.completed
    assert c.report_done(ticket, a, **report(c, tid, a, 4.))


def test_revocation_retains_all_leases_and_blocks_old_completions():
    c = protocol()
    for tid in c.tasks: ready(c, tid)
    ticket = c.grant_ready(1.)[0]
    held = dict(c.locks)
    c.authority.invalidate('visual identity ambiguous; external recovery required')
    rid = next(iter(c.tasks[ticket.task_id]['participants']))
    with pytest.raises(ValueError, match='revoked'): c.report_done(ticket, rid, **report(c, ticket.task_id, rid))
    with pytest.raises(ValueError, match='revoked'): c.grant_ready(1.)
    assert c.locks == held and not c.completed


def test_occupied_staging_slot_is_not_released_by_task_completion():
    m = mission('staging_three')
    m['tasks'][1]['destination_id'] = 'staging_02'
    # This intentional logical deadlock must hold, never silently overwrite cargo.
    c = MissionProtocol(m, agree(m))
    for tid in c.tasks: ready(c, tid)
    ticket, = c.grant_ready(1.)
    finish(c, ticket)
    assert c.grant_ready(1.) == []
    assert c.residents == {'staging_02': 'object_02'}


@pytest.mark.parametrize('beams,boxes', [(True, 1), (-1, 2), (0, 0), (1, 8)])
def test_pilot_rejects_invalid_counts(beams, boxes):
    with pytest.raises(ValueError): make_mission({'id': 'bad', 'beams': beams, 'boxes': boxes, 'pattern': 'independent'})
