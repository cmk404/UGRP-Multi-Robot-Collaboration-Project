"""Integrated zone study runner (issue #223): study core (#194) x own-camera executor (#206).

Everything here runs without a simulator. Robots are ``FakeLink`` objects around a
REAL ``ZoneOwnExecutor`` (no physics) with stored real wrist JPEGs, so the
decision -> executor boundary is the real API. The physics owner
(``scripts/run_zone_study_integration.py``) is covered by the plumbing smoke.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import team_carry_status as tcs  # noqa: E402
from harness import zone_own_executor as zox  # noqa: E402
from harness import zone_study_contract as A  # noqa: E402
from harness import zone_study_integration as zi  # noqa: E402
from harness import zone_study_offline as zo  # noqa: E402
from harness.zone_study_inputs import OrderSheetSource  # noqa: E402
from harness.zone_study_scenarios import bundle_for, load as load_scenario  # noqa: E402

SCENARIO = json.loads((ROOT / 'configs/zone_study_integration/i1_cyan_three_slots.json').read_text())
MAP_ID = 'zone_wide_door_tags_v2'
MAP = json.loads((ROOT / 'maps/zones' / f'{MAP_ID}.json').read_text())
CALIB = json.loads((ROOT / 'experiments/2026-09-26-zone-m1-owncam/calibration_m1_dev.json').read_text())
FRAMES = [p.read_bytes() for p in sorted((ROOT / 'tests/fixtures/markerless_box/blue_floor_release')
                                         .glob('*-wrist.jpg'))]
BUNDLE = bundle_for(SCENARIO)
SHEET = OrderSheetSource(SCENARIO, BUNDLE).sheet()
ROWS_Y = (-2.45, -1.65, -.85, -.05, .75)
CONDITIONS = A.MAIN_CONDITIONS
SEED = 700


class FakeLink:
    """One robot: a real executor (no physics), stored real JPEGs, a shared test clock."""

    def __init__(self, rid, clock, *, frame_offset=0, belief=None):
        self.robot_id, self._clock = rid, clock
        self.frame_offset, self._belief = frame_offset, belief
        self.ex = zox.ZoneOwnExecutor(rid, MAP, CALIB['params'], SHEET, skill_factory=lambda o, robot_id: None,
                                      pose_estimate_cls=tuple, search_rows_y=ROWS_Y)
        self.accessed = []

    def clock(self):
        return self._clock[0]

    def frame_at(self, t):
        self.accessed.append('frame_at')
        index = int(math.floor(t * 5 + 1e-9))                   # the latest own frame at or before t
        data = FRAMES[(index + zox.ROBOTS.index(self.robot_id) + self.frame_offset) % len(FRAMES)]
        return zi.OwnFrame(index, round(index / 5, 4), data, hashlib.sha256(data).hexdigest())

    def belief(self):
        self.accessed.append('belief')
        return copy.deepcopy(self._belief) if self._belief is not None else self.ex.belief_projection()

    def job(self):
        self.accessed.append('job')
        job = self.ex.job
        return None if job is None else {'kind': job.kind, 'order_id': job.args.get('order_id'), 'job_id': job.job_id}

    def call(self, api, *args):
        self.accessed.append(f'call:{api}')
        self.ex.now = self._clock[0]
        return getattr(self.ex, api)(*args)


def links_for(clock, **per_robot):
    return {r: FakeLink(r, clock, **per_robot.get(r, {})) for r in zox.ROBOTS}


def run(condition, *, horizon=30., links=None, clock=None, events=(), actors=None, seed=SEED, scenario=SCENARIO):
    """Drive a trial on the QUANTUM_S grid; ``events`` = [(t, robot, fn(link) -> None)] own-executor actions."""
    clock = clock or [0.]
    links = links or links_for(clock)
    trial = zi.IntegratedTrial(scenario, condition=condition, seed=seed, links=links, horizon_s=horizon,
                               map_bundle=BUNDLE if scenario is SCENARIO else None)
    for rid, actor in (actors or {}).items():
        trial.fixtures[rid] = actor
    t = clock[0] = 1.3
    trial.begin(t)
    pending = sorted(events, key=lambda e: e[0])
    while t < horizon - 1e-9:
        t = clock[0] = round(t + zi.QUANTUM_S, 6)
        while pending and pending[0][0] <= t + 1e-9:
            _, rid, fn = pending.pop(0)
            fn(links[rid])
        for link in links.values():
            for ev in link.ex.drain_events():
                trial.on_executor_event(ev, at_s=t)
        trial.step_to(t)
    return trial, trial.finish(t), links


def requests_of(trial, rid):
    return [r for r in trial.requests if r['robot'] == rid]


class Scripted:
    """A test actor: ``respond(request)`` only, like the #194 fixture (own request is the whole input)."""

    def __init__(self, fn):
        self.fn, self.calls = fn, 0

    def respond(self, request):
        self.calls += 1
        payload = json.loads(request['messages'][1]['content'])
        return json.dumps(self.fn(payload), ensure_ascii=False)


def reply(payload, action=None, messages=()):
    return {'request_id': payload['request_id'], 'action': action or {'kind': 'continue'},
            'decision_sources': ['static_map', 'order_sheet', 'own_rgb', 'own_commands'], 'messages': list(messages)}


# ---------------------------------------------------------------- isolation (the #169 confound)
@pytest.mark.parametrize('condition', CONDITIONS)
def test_peer_private_state_does_not_reach_a_robot_inputs_or_wakeups(condition):
    """Change ONLY r2's private state (its frames, belief, executor failure): r1 and r3 are identical."""
    base, _, _ = run(condition)
    clock = [0.]
    links = links_for(clock, r2={'frame_offset': 3, 'belief': {**zi.zo.belief_skeleton(), 'region': 'zone_B',
                                                                'held_item_guess': 'yes', 'notes_ko': '다름'}})
    changed, _, _ = run(condition, links=links, clock=clock,
                        events=[(9.0, 'r2', lambda link: link.call('abort', 'test_private_failure'))])
    assert [r['request_sha256'] for r in requests_of(changed, 'r2')] != \
        [r['request_sha256'] for r in requests_of(base, 'r2')]                  # the perturbation is real
    for rid in ('r1', 'r3'):
        assert [r['request_sha256'] for r in requests_of(base, rid)] == \
            [r['request_sha256'] for r in requests_of(changed, rid)], rid
        assert base.wakeups(rid) == changed.wakeups(rid), rid
        assert [(d['sim_s'], d['api'], d['args']) for d in base.dispatch_log if d['actor'] == rid] == \
            [(d['sim_s'], d['api'], d['args']) for d in changed.dispatch_log if d['actor'] == rid]


def _r2_talks_if_holding(payload):
    """r2 speaks (to r1) only when ITS OWN belief says it holds something: private state -> channel."""
    msgs = []
    if payload['self_belief'].get('held_item_guess') == 'yes' and payload.get('channel', {}).get('can_send_to'):
        msgs = [{'recipients': ['r1'], 'reply_to': None, 'text': 'r2가 물건을 들고 있습니다.'}]
    return reply(payload, messages=msgs)


@pytest.mark.parametrize('condition', ('peer_ko', 'leader_ko', 'no_comm'))
def test_peer_private_state_reaches_a_robot_only_through_the_condition_channel(condition):
    holding = {**zi.zo.belief_skeleton(), 'held_item_guess': 'yes'}
    runs = []
    for belief in (None, holding):
        clock = [0.]
        links = links_for(clock, r2={'belief': belief} if belief else {})
        runs.append(run(condition, links=links, clock=clock, actors={'r2': Scripted(_r2_talks_if_holding)})[0])
    a, b = runs
    user = [[json.loads(r['user']) for r in requests_of(t, 'r1')] for t in (a, b)]
    if condition == 'no_comm':
        assert user[0] == user[1] and a.wakeups('r1') == b.wakeups('r1')
        assert not b.messages and all('inbox' not in u for u in user[1])
        return
    from_r2 = [m for m in b.messages if m['sender'] == 'r2']
    assert from_r2 and not [m for m in a.messages if m['sender'] == 'r2']
    assert all(m['recipients'] == ['r1'] for m in from_r2)
    others = lambda t: [(m['sender'], m['recipients'], m['body']) for m in t.messages if m['sender'] != 'r2']  # noqa: E731
    assert others(a) == others(b)                                          # nobody else changed
    t_d = min(d['delivered_at_sim_s'] for m in from_r2 for d in m['deliveries'])
    before = lambda t: [r['request_sha256'] for r in requests_of(t, 'r1') if r['sim_s'] < t_d]  # noqa: E731
    assert before(a) and before(a) == before(b)                            # identical until the delivery
    assert [w for w in a.wakeups('r1') if w[0] < t_d] == [w for w in b.wakeups('r1') if w[0] < t_d]
    first_new = next(w for w in b.wakeups('r1') if w[0] >= t_d)
    assert first_new == (t_d, 'report')                                     # woken by the delivered message
    later = [json.loads(r['user']) for r in requests_of(b, 'r1') if r['sim_s'] >= t_d]
    assert later and any(m['sender'] == 'r2' for m in later[0]['inbox'])


# ---------------------------------------------------------------- channel routing
def test_no_comm_has_no_inbox_no_recipients_and_no_messages():
    trial, result, _ = run('no_comm')
    for row in trial.requests:
        user = json.loads(row['user'])
        assert 'inbox' not in user and user['channel']['can_send_to'] == [] and user['channel']['encoding'] == 'none'
    assert not result.messages and result.channel['sent'] == 0 and result.cost['talk_sim_s'] == 0


def _follower_tries_everyone(payload):
    if payload['robot_id'] == 'r1' and not payload['own_command_history'] and payload['channel']['can_send_to']:
        return reply(payload, messages=[{'recipients': ['r3'], 'reply_to': None, 'text': 'r3에게 직접 말합니다.'},
                                        {'recipients': ['r2'], 'reply_to': None, 'text': '리더에게 보고합니다.'}])
    return reply(payload)


@pytest.mark.parametrize('seed, leader', [(700, 'r2'), (701, 'r3'), (702, 'r1')])
def test_leader_rotation_and_hub_and_spoke_routing(seed, leader):
    trial, result, _ = run('leader_ko', seed=seed)
    assert trial.leader_id == leader and result.leader_id == leader
    for m in result.messages:
        assert leader in (m['sender'], *m['recipients'])
        assert m['sender'] == leader or m['recipients'] == [leader]            # follower -> leader only
    assert result.channel['follower_to_follower'] == 0


def test_leader_ko_rejects_follower_to_follower_and_delivers_follower_report():
    trial, result, _ = run('leader_ko', actors={'r1': Scripted(_follower_tries_everyone)})
    assert trial.leader_id == 'r2'
    delivered = [(m['sender'], tuple(m['recipients'])) for m in result.messages]
    assert ('r1', ('r3',)) not in delivered and ('r1', ('r2',)) in delivered
    assert any(r['sender'] == 'r1' for r in trial.scheduler.rejected_messages)
    assert result.channel['follower_to_follower'] == 0


def test_structured_carries_no_free_text():
    trial, result, _ = run('structured')
    assert result.messages and result.channel['free_text_messages'] == 0
    assert all('text' not in m['body'] for m in result.messages)


# ---------------------------------------------------------------- SIM cost and decision -> executor
@pytest.mark.parametrize('condition', CONDITIONS)
def test_talk_and_think_cost_sim_time_and_actions_land_at_the_charged_time(condition):
    trial, result, _ = run(condition)
    checks = zo.cost_checks(trial, result)
    assert checks['ok'], checks['problems']
    done = {c.call_id: c for c in trial.scheduler.calls}
    assert trial.dispatch_log
    for d in trial.dispatch_log:
        call = done[d['call_id']]
        assert call.finished_sim_s > call.started_sim_s                        # thinking costs SIM time
        assert abs(d['sim_s'] - call.finished_sim_s) < 1e-9                    # released at the charged time
        if d['ack']:
            assert abs(d['ack']['sim_s'] - d['sim_s']) < 1e-9 and d['ack']['robot_id'] == d['actor']
    assert trial.clock_drift_s < 1e-9
    talk = result.cost['talk_sim_s']
    assert (talk > 0) == (condition != 'no_comm')


def test_claim_reaches_own_executor_and_rejections_are_recorded_not_arbitrated():
    """Two robots claim the SAME order: both reach their own executor; the host arbitrates nothing."""
    def claim_order_1(payload):
        if not payload['own_command_history']:
            return reply(payload, {'kind': 'claim', 'order_id': 'order-1', 'role': 'west', 'destination_zone': 'C'})
        return reply(payload)
    actors = {'r1': Scripted(claim_order_1), 'r2': Scripted(claim_order_1)}
    trial, result, links = run('peer_ko', actors=actors)
    firsts = [d for d in trial.dispatch_log if d['api'] == 'deliver' and d['args'] == ['order-1', 'C']]
    assert {d['actor'] for d in firsts} >= {'r1', 'r2'} and all(d['ack']['accepted'] for d in firsts[:2])
    assert links['r1'].ex.job.args['order_id'] == links['r2'].ex.job.args['order_id'] == 'order-1'
    rejected = [a for a in result.actions if not a['accepted']]
    assert all(a['rejected_reason'] for a in rejected)
    for rid in ('r1', 'r2'):
        history = json.loads(requests_of(trial, rid)[-1]['user'])['own_command_history']
        assert history[0]['kind'] == 'claim_order' and history[0]['arguments']['order_id'] == 'order-1'


def test_wait_aborts_the_own_running_job_and_release_needs_the_matching_order():
    def script(payload):
        n = len(payload['own_command_history'])
        if n == 0:
            return reply(payload, {'kind': 'claim', 'order_id': 'order-1', 'role': 'west', 'destination_zone': 'C'})
        if n == 1:
            return reply(payload, {'kind': 'release', 'order_id': 'order-2'})
        if n == 2:
            return reply(payload, {'kind': 'wait'})
        return reply(payload)
    trial, result, links = run('no_comm', actors={'r1': Scripted(script)}, horizon=150.)   # busy re-ask 60 s
    rows = [(d['api'], d['rejected_reason'], (d['ack'] or {}).get('accepted')) for d in trial.dispatch_log
            if d['actor'] == 'r1']
    assert rows[:3] == [('deliver', None, True), (None, 'NO_ACTIVE_JOB_FOR_ORDER', None), ('abort', None, True)]
    ex = links['r1'].ex
    assert ex.job is None and ex.jobs_done[-1]['outcome'] == 'ABORTED:wait_requested'
    assert ex.step(links['r1'].clock()) == {'mode': 'tick', 'commands': [{'kind': 'hold'}]}   # an actual hold


@pytest.mark.parametrize('action, job, expected', [
    ({'kind': 'claim', 'order_id': 'order-1', 'role': 'west', 'destination_zone': 'C'}, None,
     zi.Plan('deliver', ('order-1', 'C'))),
    ({'kind': 'continue'}, None, zi.Plan(None)),
    ({'kind': 'wait'}, None, zi.Plan('hold', (zi.WAIT_HOLD_S,))),
    ({'kind': 'wait'}, {'kind': 'deliver', 'order_id': 'order-1'}, zi.Plan('abort', ('wait_requested',))),
    ({'kind': 'release', 'order_id': 'order-1'}, {'kind': 'deliver', 'order_id': 'order-1'},
     zi.Plan('abort', ('release_requested',))),
    ({'kind': 'release', 'order_id': 'order-1'}, None, zi.Plan(None, rejected_reason='NO_ACTIVE_JOB_FOR_ORDER')),
    ({'kind': 'release', 'order_id': None}, {'kind': 'goto', 'order_id': None},
     zi.Plan(None, rejected_reason='NO_ACTIVE_JOB_FOR_ORDER')),
    ({'kind': 'claim', 'order_id': 0, 'destination_zone': 'C'}, None, zi.Plan(None, rejected_reason='BAD_CLAIM')),
    ({'kind': 'claim', 'order_id': 'order-1', 'destination_zone': None}, None,
     zi.Plan(None, rejected_reason='BAD_CLAIM')),
    ({'kind': 'order', 'assignments': {}}, None, zi.Plan(None, rejected_reason='UNSUPPORTED_ACTION')),
    ({'kind': None}, None, zi.Plan(None, rejected_reason='UNSUPPORTED_ACTION')),
    ({}, None, zi.Plan(None, rejected_reason='UNSUPPORTED_ACTION')),
    (None, None, zi.Plan(None, rejected_reason='UNSUPPORTED_ACTION')),
    ([], None, zi.Plan(None, rejected_reason='UNSUPPORTED_ACTION')),
    ('claim', None, zi.Plan(None, rejected_reason='UNSUPPORTED_ACTION')),
    (float('nan'), None, zi.Plan(None, rejected_reason='UNSUPPORTED_ACTION')),
])
def test_executor_plan_maps_actions_and_refuses_bad_input_without_raising(action, job, expected):
    assert zi.executor_plan(action, job) == expected


@pytest.mark.parametrize('event', [None, {}, [], 'job_done', {'robot_id': 'r9', 'event': 'job_done'},
                                   {'robot_id': 'r1'}, {'robot_id': 'r1', 'event': 'peer_done'},
                                   {'robot_id': None, 'event': 'job_done'}])
def test_malformed_or_foreign_executor_events_are_refused(event):
    clock = [0.]
    trial = zi.IntegratedTrial(SCENARIO, condition='no_comm', seed=SEED, links=links_for(clock), horizon_s=10.,
                               map_bundle=BUNDLE)
    with pytest.raises(A.ContractViolation):
        trial.on_executor_event(event, at_s=1.)


def test_an_executor_event_wakes_only_its_own_robot():
    trial, _, _ = run('no_comm', events=[(9.0, 'r2', lambda link: link.call('abort', 'probe'))])
    assert ('%.1f' % 9.0, 'failure') in [('%.1f' % t, trig) for t, trig in trial.wakeups('r2')]
    assert all(trig != 'failure' for _, trig in trial.wakeups('r1') + trial.wakeups('r3'))


# ---------------------------------------------------------------- inputs: own RGB only, no GT
@pytest.mark.parametrize('condition', CONDITIONS)
def test_inputs_are_own_wrist_rgb_map_sheet_own_history_and_delivered_messages_only(condition):
    clock = [0.]
    links = links_for(clock)
    trial, result, _ = run(condition, links=links, clock=clock)
    assert zo.request_checks(result)['ok']
    allow = A.condition(condition).input_allowlist
    own = {rid: {hashlib.sha256(f).hexdigest() for f in FRAMES} for rid in zox.ROBOTS}
    for row in trial.requests:
        user = json.loads(row['user'])
        assert set(user) - {zi.pk.WINDOW_KEY} <= allow and A.forbidden_key_hits(user) == []
        assert not any(s in row['user'] for s in A.FORBIDDEN_VALUE_SUBSTRINGS)
        assert [i['label'] for i in row['image_refs']] == ['CURRENT OWN WRIST RGB']
        assert all(i['sha256'] in own[row['robot']] for i in row['image_refs'])
        assert all(r['ref'].startswith(f'own-{row["robot"]}-') for r in user['own_rgb_refs'])
        assert user['order_sheet'] == SHEET and user['static_map']['map_id'] == MAP_ID
    for d in trial.input_log:
        assert d['frame_t'] <= d['sim_s'] + 1e-9                               # captured at or before the call


def test_a_call_needs_an_own_frame():
    class Blind(FakeLink):
        def frame_at(self, t):
            return None
    clock = [0.]
    links = {r: (Blind if r == 'r1' else FakeLink)(r, clock) for r in zox.ROBOTS}
    trial = zi.IntegratedTrial(SCENARIO, condition='no_comm', seed=SEED, links=links, horizon_s=10.,
                               map_bundle=BUNDLE)
    clock[0] = 1.3
    with pytest.raises(A.ContractViolation, match='no own robot_cam frame'):
        trial.begin(1.3)


def test_only_the_fixture_actor_and_main_conditions_are_accepted():
    clock = [0.]
    with pytest.raises(A.ContractViolation):
        zi.check_actor('gemini-3.8-flash')
    for bad in ('reference_R', 'dynamic', '', None, 0):
        with pytest.raises((A.ContractViolation, KeyError, TypeError)):
            zi.IntegratedTrial(SCENARIO, condition=bad, seed=SEED, links=links_for(clock), horizon_s=10.,
                               map_bundle=BUNDLE)
    with pytest.raises(A.ContractViolation):
        zi.IntegratedTrial(SCENARIO, condition='no_comm', seed=SEED, links=links_for(clock), horizon_s=10.,
                           map_bundle=BUNDLE, actor='gemini')
    two = dict(list(links_for(clock).items())[:2])
    with pytest.raises(A.ContractViolation):
        zi.IntegratedTrial(SCENARIO, condition='no_comm', seed=SEED, links=two, horizon_s=10., map_bundle=BUNDLE)
    swapped = links_for(clock)
    swapped['r1'], swapped['r2'] = swapped['r2'], swapped['r1']
    with pytest.raises(A.ContractViolation):
        zi.IntegratedTrial(SCENARIO, condition='no_comm', seed=SEED, links=swapped, horizon_s=10., map_bundle=BUNDLE)


def test_study_layer_touches_only_the_calling_robot():
    clock = [1.3]
    links = links_for(clock)
    trial = zi.IntegratedTrial(SCENARIO, condition='peer_ko', seed=SEED, links=links, horizon_s=30.,
                               map_bundle=BUNDLE)
    trial.begin(1.3)
    assert links['r1'].accessed and all(a in ('frame_at', 'belief') for a in links['r1'].accessed)
    for link in links.values():
        link.accessed.clear()
    trial.scheduler.trigger('r1', 'idle', at=1.3)
    trial.snapshot(type('C', (), {'actor': 'r1', 'started_sim_s': 1.3})())
    assert links['r1'].accessed and not links['r2'].accessed and not links['r3'].accessed


# ---------------------------------------------------------------- pair status channel (all conditions)
def _stub_links():
    return {r: type('Stub', (), {'robot_id': r})() for r in zox.ROBOTS}


def test_pair_status_channel_is_present_and_identical_in_all_four_conditions():
    for scenario in (SCENARIO, load_scenario('s1_normal_mixed')):
        configs = {}
        for condition in CONDITIONS:
            trial = zi.IntegratedTrial(scenario, condition=condition, seed=SEED, links=_stub_links(), horizon_s=10.)
            configs[condition] = trial.pair_status.config_sha256()
            cfg = trial.study_config()
            assert 'pair_status' in cfg['inter_robot_channels']
            assert cfg['inter_robot_channels'] == (['pair_status'] if condition == 'no_comm'
                                                   else ['dialogue', 'pair_status'])
        assert len(set(configs.values())) == 1, configs
    s1 = zi.IntegratedTrial(load_scenario('s1_normal_mixed'), condition='no_comm', seed=SEED, links=_stub_links(),
                            horizon_s=10.)
    assert sorted(s1.pair_status.channels) == ['order-5']                     # the 2-robot long_beam order


def test_pair_status_carries_only_the_fixed_enum_and_only_to_participants():
    bus = zi.PairStatusBus({'orders': [{'order_id': 'order-5', 'required_robots': 2}]})
    assert bus.publish('r1', 'order-5', 'aligning', 1.0)
    for bad in ('r1가 빔 동쪽 끝에 있음', 'done', '', None, 0, float('nan')):
        assert not bus.publish('r1', 'order-5', bad, 1.1)
    assert not bus.publish('r9', 'order-5', 'ready', 1.2) and not bus.publish('r1', 'order-9', 'ready', 1.2)
    channel = bus.channels['order-5']
    assert not channel.publish({'robot_id': 'r2', 'task_id': 'order-5', 'seq': 1, 'state': 'ready',
                                'sent_at_s': 1.0, 'text': '좌표 1.2, 0.4'}, 1.0)
    assert not channel.publish({'robot_id': 'r2', 'task_id': 'order-5', 'seq': 1, 'state': 'ready',
                                'sent_at_s': float('inf')}, 1.0)
    assert bus.partner_view('r3', 'order-5', 1.5) is None                     # not on the task: sees nothing
    assert bus.publish('r2', 'order-5', 'ready', 1.3)
    view = bus.partner_view('r1', 'order-5', 1.5)
    assert view['r2']['state'] == 'ready' and set(view['r2']) == {'state', 'age_s', 'alive'}
    assert all(set(row) == set(tcs.FIELDS) | {'received_at_s'} for row in channel.log)


def test_study_config_is_condition_invariant_apart_from_the_channel():
    configs = {c: zi.IntegratedTrial(SCENARIO, condition=c, seed=SEED, links=_stub_links(), horizon_s=10.)
               .study_config() for c in CONDITIONS}
    invariant = {c: zi.condition_invariant_config(v) for c, v in configs.items()}
    assert all(v == invariant['no_comm'] for v in invariant.values())
    assert configs['leader_ko']['leader_id'] == 'r2' and all(configs[c]['leader_id'] is None
                                                             for c in CONDITIONS if c != 'leader_ko')
    assert configs['no_comm']['actor'] == zi.FIXTURE_ACTOR and configs['no_comm']['planned_model']['enabled'] is False


# ---------------------------------------------------------------- pose provider seam
def test_tags_temporary_provider_is_labelled_temporary_and_hashed():
    spec = zi.pose_provider_spec('tags_temporary', map_id=MAP_ID)
    assert spec['temporary'] and not spec['research_result'] and spec['note_ko'] == zi.TEMPORARY_NOTE_KO
    rec = zi.provider_record(spec)
    assert rec['label'] == {'pose_provider': 'tags_temporary', 'temporary': True, 'research_result': False,
                            'note_ko': '임시, 표식 사용, 연구 결과 아님'}
    assert len(rec['record_sha256']) == 64 and set(rec['source_files_sha256']) == set(spec['source_files'])
    provider = zi.build_pose_provider(spec, MAP, CALIB['params'], 700)
    assert provider.source.startswith('owncam_pf_v2:')


@pytest.mark.parametrize('provider_id, map_id', [('vision_zero_tag', MAP_ID), ('', MAP_ID), (None, MAP_ID),
                                                 (0, MAP_ID), ('tags_temporary', 'zone_wide_door'),
                                                 ('tags_temporary', None)])
def test_unknown_provider_or_unregistered_map_is_refused(provider_id, map_id):
    with pytest.raises(A.ContractViolation):
        zi.pose_provider_spec(provider_id, map_id=map_id)


def _write_registry(tmp_path, **override):
    data = json.loads(zi.PROVIDER_CONFIG.read_text())
    entry = {**data['providers']['tags_temporary'], **override}
    data['providers'] = {'vision_stub': entry}
    path = tmp_path / 'providers.json'
    path.write_text(json.dumps(data))
    return path


def test_a_new_provider_drops_in_by_config_without_runner_change(tmp_path, monkeypatch):
    (tmp_path / 'kiro_vision_stub.py').write_text(textwrap.dedent('''
        from harness.owncam_pose_source import OwnCamPoseSource

        class VisionStub(OwnCamPoseSource):
            def __post_init__(self):
                super().__post_init__()
                self.source = 'owncam_pf_vision_stub:00000000'
    '''))
    monkeypatch.syspath_prepend(str(tmp_path))
    path = _write_registry(tmp_path, factory='kiro_vision_stub:VisionStub', version='vision_stub_v0',
                           source_label_prefix='owncam_pf_vision_stub:', uses_landmark_tags=False,
                           temporary=False, research_result=True, note_ko='표식 0개 시험용 대역')
    spec = zi.pose_provider_spec('vision_stub', map_id=MAP_ID, path=path)
    provider = zi.build_pose_provider(spec, MAP, CALIB['params'], 700)
    assert provider.source == 'owncam_pf_vision_stub:00000000'
    ex = zox.ZoneOwnExecutor('r1', MAP, CALIB['params'], SHEET, skill_factory=lambda o, robot_id: None,
                             pose_estimate_cls=tuple, search_rows_y=ROWS_Y)
    ex._require_owncam(provider.source, 'pose provider')                      # the executor's own M1 check


@pytest.mark.parametrize('override, match', [
    ({'source_label_prefix': 'gt_pose:'}, 'does not start'),
    ({'uses_landmark_tags': True, 'temporary': False}, 'must be temporary'),
    ({'uses_landmark_tags': True, 'note_ko': '연구 결과'}, 'must be temporary'),
])
def test_provider_registry_refuses_mislabelled_entries(tmp_path, override, match):
    path = _write_registry(tmp_path, **override)
    with pytest.raises(A.ContractViolation, match=match):
        spec = zi.pose_provider_spec('vision_stub', map_id=MAP_ID, path=path)
        zi.build_pose_provider(spec, MAP, CALIB['params'], 700)


def test_provider_without_the_executor_interface_or_with_a_gt_label_is_refused():
    spec = zi.pose_provider_spec('tags_temporary', map_id=MAP_ID)
    good = zi.build_pose_provider(spec, MAP, CALIB['params'], 1)

    class NoLoc:
        source = good.source
        on_command = on_frame = report = set_motion_profile = staticmethod(lambda *a: None)
    with pytest.raises(A.ContractViolation, match='loc'):
        zi.check_pose_provider(NoLoc(), spec)
    good.source = 'gt_pose_from_simulator'
    with pytest.raises(A.ContractViolation):
        zi.check_pose_provider(good, spec)
    good.source = None
    with pytest.raises(A.ContractViolation):
        zi.check_pose_provider(good, spec)


def test_the_study_modules_import_no_simulator():
    program = ('import sys; import harness.zone_study_integration, harness.owncam_pose_source; '
               'assert not ({"mujoco", "sim.multi_masterpi_production"} & sys.modules.keys())')
    subprocess.run([sys.executable, '-c', program], cwd=ROOT, check=True)


# ---------------------------------------------------------------- contract v2 (tags_v2 map placement)
def _payload():
    clock = [1.3]
    trial = zi.IntegratedTrial(SCENARIO, condition='no_comm', seed=SEED, links=links_for(clock), horizon_s=10.,
                               map_bundle=BUNDLE)
    trial.snapshot(type('C', (), {'actor': 'r1', 'started_sim_s': 1.3})())
    return trial.build_inputs('r1', sim_time_s=1.3, request_id='req_t').payload_dict()


def test_contract_v2_accepts_the_tags_v2_landmark_placement():
    payload = _payload()
    placement = payload['static_map']['public_map']['landmarks']['placement']
    assert {'near_door_spacing_m', 'near_door_radius_m', 'door_posts'} <= set(placement)
    assert A.CONTRACT_VERSION == 'ugrp.zone_study_contract.v2'
    A.validate_robot_payload(payload, seed=SEED)


@pytest.mark.parametrize('mutate', [
    lambda p: p['door_posts'].update(live_pose_m=[1., 2.]),
    lambda p: p['door_posts'].update(width_m='wide'),
    lambda p: p['door_posts'].update(tag_center_heights_m=None),
    lambda p: p['door_posts'].update(tag_center_heights_m='0.15'),
    lambda p: p['door_posts'].update(tag_center_heights_m=[0.15, {'x': 1}]),
    lambda p: p.update(door_posts=[0.05]),
    lambda p: p.update(near_door_radius_m='1 m'),
    lambda p: p.update(near_door_radius_m=None),
])
def test_contract_v2_keeps_the_placement_closed_and_typed(mutate):
    payload = _payload()
    mutate(payload['static_map']['public_map']['landmarks']['placement'])
    assert A.payload_violations(payload)


# ---------------------------------------------------------------- abort drops host macros (#221 P1)
def test_host_link_abort_drops_scheduled_macros_and_holds_now():
    from scripts.run_zone_study_integration import HostRobotLink

    class Port:
        def capture(self, camera='robot_cam'):
            raise AssertionError('not captured in this test')

    ex = zox.ZoneOwnExecutor('r1', MAP, CALIB['params'], SHEET, skill_factory=lambda o, robot_id: None,
                             pose_estimate_cls=tuple, search_rows_y=ROWS_Y)
    slot = zox._RobotSlot('r1', Port(), ex)
    holds = []
    host = type('Host', (), {})()
    host.robots, host.api_calls = {'r1': slot}, []
    host.world = type('W', (), {'data': type('D', (), {'time': 12.3})()})()
    host._hold = lambda rid, now: holds.append((rid, now))
    link = HostRobotLink(host, 'r1')
    assert link.call('hold', 5.)['accepted']
    slot.timeline = [(12.4, [{'kind': 'arm', 'servo_id': 3, 'pulse': 900}]), (12.5, ['hold'])]
    slot.capture_after, slot.next_decide = True, 99.
    ack = link.call('abort', 'wait_requested')
    assert ack['accepted'] and slot.timeline == [] and not slot.capture_after and slot.next_decide == 12.3
    assert holds == [('r1', 12.3)] and link.aborts[-1]['dropped_macro_commands'] == 2
    assert ex.step(12.3) == {'mode': 'tick', 'commands': [{'kind': 'hold'}]}
    assert not link.call('abort', 'again')['accepted'] and holds == [('r1', 12.3)]   # nothing to abort: no hold


def test_referee_counts_a_box_only_after_it_rests_in_a_zone():
    from scripts.run_zone_study_integration import SETTLE_S, referee_from_gt
    zone_a = MAP['regions']['zone_A']['center_m']
    rows = [{'t': round(i * .05, 3), 'boxes': {'b0': [zone_a[0], zone_a[1], .016 if i > 10 else .09],
                                                'b1': [0., 0., .016]}} for i in range(int((SETTLE_S + 1) / .05))]
    ref = referee_from_gt(rows, MAP, 3.0)
    assert ref['deliveries'] == [{'item_id': 'b0', 'kind': 'cyan', 'zone': 'A', 'sim_s': .55}]
    assert referee_from_gt([], MAP, 0.)['deliveries'] == []
    short = [r for r in rows if r['t'] < .55 + SETTLE_S - .1]
    assert referee_from_gt(short, MAP, 2.)['deliveries'] == []


# ---------------------------------------------------------------- registration
def test_runner_is_registered_and_collected_by_ci():
    rows = json.loads((ROOT / 'configs/simulation_workflows.json').read_text())['workflows']
    row = next(r for r in rows if r['id'] == 'zone-study-integration-run')
    assert row['entry'] == 'scripts/run_zone_study_integration.py' and (ROOT / row['docs']).is_file()
    from scripts import run_ci_tests
    assert 'tests/test_zone_study_integration.py' in run_ci_tests.TEST_PATTERNS


def test_quantum_is_the_sim_cost_quantum():
    assert zi.QUANTUM_S == zi.cost_params_for().quantum_s == .1 and math.isclose(zi.WAIT_HOLD_S, 10.)
