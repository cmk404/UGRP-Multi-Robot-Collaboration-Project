"""Dynamic (talk-when-needed) coordination: self-claims, conflicts and recovery."""
import json
from types import SimpleNamespace

import pytest

from harness import dynamic_coordination as dyn
from harness.dispatch_plan import validate_dispatch_plan
from harness.three_robot_plan import TeamAgreement, digest

BEAM_UP = {'object': 'beam', 'beam_end': 'upper', 'route': 'north', 'dock': 'dock_a'}
BEAM_LOW = {'object': 'beam', 'beam_end': 'lower', 'route': 'north', 'dock': 'dock_a'}
BOX = {'object': 'box', 'beam_end': None, 'route': 'south', 'dock': 'dock_a'}
CONSISTENT = {'r1': BEAM_LOW, 'r2': BOX, 'r3': BEAM_UP}


def _reply(request_id, claim, **extra):
    return {'request_id': request_id, 'claim': claim, 'reason': 'seen in TOP', 'message': 'hi', **extra}


def test_claim_reply_validation():
    assert dyn.validate_claim_reply(json.dumps(_reply('q', BOX)), 'q')['claim'] == BOX
    for bad in (_reply('other', BOX), _reply('q', {**BOX, 'beam_end': 'upper'}),
                _reply('q', {**BEAM_UP, 'beam_end': None}), _reply('q', {**BOX, 'route': 'east'}),
                _reply('q', BOX, plan=None), _reply('q', {**BOX, 'extra': 1})):
        with pytest.raises(ValueError):
            dyn.validate_claim_reply(json.dumps(bad), 'q')


def test_consistent_claims_form_a_plan_without_the_host_choosing():
    merged = dyn.merge_claims(CONSISTENT)
    assert merged['consistent'] and not merged['conflicts']
    beam, box = merged['plan']['tasks']
    assert beam['participants'] == ['r3', 'r1']  # upper end first, as each robot claimed
    assert beam['route'] == 'north' and box['participants'] == ['r2'] and box['route'] == 'south'
    assert merged['plan']['dock'] == 'dock_a' and beam['after'] == box['after'] == []
    assert validate_dispatch_plan(merged['plan']) == merged['plan']


@pytest.mark.parametrize('claims,kind', [
    ({'r1': BEAM_LOW, 'r2': BEAM_UP, 'r3': BEAM_UP}, 'object_count'),
    ({'r1': BEAM_UP, 'r2': BOX, 'r3': BEAM_UP}, 'same_beam_end'),
    ({'r1': {**BEAM_LOW, 'route': 'south'}, 'r2': BOX, 'r3': BEAM_UP}, 'beam_route'),
    ({'r1': BEAM_LOW, 'r2': {**BOX, 'dock': 'dock_b'}, 'r3': BEAM_UP}, 'dock'),
    ({'r1': BEAM_LOW, 'r2': None, 'r3': BEAM_UP}, 'missing_claim'),
])
def test_conflicting_claims_are_reported_not_repaired(claims, kind):
    merged = dyn.merge_claims(claims)
    assert not merged['consistent'] and merged['plan'] is None
    assert kind in {c['kind'] for c in merged['conflicts']}


def test_required_dock_is_a_conflict_when_claims_disagree_with_the_mission():
    merged = dyn.merge_claims(CONSISTENT, required_dock='dock_b')
    assert {c['kind'] for c in merged['conflicts']} == {'required_dock'}


def test_agreement_commits_consistent_claims_once():
    agreement = TeamAgreement('run', plan_validator=validate_dispatch_plan)
    plan = dyn.merge_claims(CONSISTENT)['plan']
    committed = agreement.commit_claims(plan, CONSISTENT, 0)
    assert committed['plan_hash'] == digest(plan) and committed['proposer'] == 'consistent_self_claims'
    assert agreement.authorize(committed['proposal_id'], committed['plan_hash'])
    assert agreement.events[-1]['event'] == 'COMMITTED_FROM_CLAIMS'
    with pytest.raises(ValueError):
        agreement.commit_claims(plan, CONSISTENT, 1)
    fresh = TeamAgreement('run2', plan_validator=validate_dispatch_plan)
    with pytest.raises(ValueError, match='own claim'):
        fresh.commit_claims(plan, {**CONSISTENT, 'r2': None}, 0)


def test_claim_request_carries_only_allowed_inputs():
    request = dyn.build_claim_request('r2', task={'goal': 'x'}, request_id='q', own_rgb=b'own',
                                      top_rgb=b'top', own_history=[{'a': 1}] * 20)
    context = json.loads(request['messages'][1]['content'])
    assert set(context) == {'robot_id', 'request_id', 'mission', 'own_issued_commands'}
    assert len(context['own_issued_commands']) == 16 and len(request['images']) == 2


def _team(tmp_path):
    from scripts.three_robot_runtime import ThreeRobotRuntime
    return ThreeRobotRuntime(tmp_path / 'team', run_id='dyn', mode='fixture',
                             agreement=TeamAgreement('dyn', plan_validator=validate_dispatch_plan))


FRAMES = {r: {'own_bytes': b'own', 'top_bytes': b'top', 'own_rgb': {'path': r}, 'shared_top_rgb': {'path': 't'},
              'frame_id': 1} for r in ('r1', 'r2', 'r3')}
HISTORY = {r: [] for r in ('r1', 'r2', 'r3')}


def test_start_goes_without_talking_when_claims_fit(tmp_path, monkeypatch):
    team = _team(tmp_path)
    monkeypatch.setattr('harness.dispatch_feasibility.inspect_routes', lambda *a, **k: {'feasible': True})
    monkeypatch.setattr('harness.dispatch_feasibility.negotiate_executable',
                        lambda *a, **k: pytest.fail('no negotiation expected'))
    try:
        log = dyn.start(team, FRAMES, HISTORY, {}, {}, 0., claim_fixture=CONSISTENT)
        assert log['path'] == 'claims' and team.agreement.committed['plan']['tasks'][1]['participants'] == ['r2']
        assert [c['model'] for c in team.calls] == ['scripted-fixture-not-llm'] * 3
        assert team.execution_events[-1]['event'] == 'STARTED_WITHOUT_NEGOTIATION'
    finally:
        team.close(0.)


def test_start_talks_on_conflict_and_on_capability_rejection(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr('harness.dispatch_feasibility.negotiate_executable',
                        lambda team, frames, history, task, *a, **k: seen.append(dict(task)) or {'feasible': True})
    monkeypatch.setattr('harness.dispatch_feasibility.inspect_routes', lambda *a, **k: {'feasible': False})
    team = _team(tmp_path)
    try:
        task = {}
        log = dyn.start(team, FRAMES, HISTORY, task, {}, 0., claim_fixture={**CONSISTENT, 'r2': BEAM_UP})
        assert log['path'] == 'conflict_then_talk' and 'claim_conflict' in seen[-1]
        assert team.agreement.committed is None
    finally:
        team.close(0.)
    team = _team(tmp_path / 'b')
    try:
        log = dyn.start(team, FRAMES, HISTORY, {}, {}, 0., claim_fixture=CONSISTENT)
        assert log['path'] == 'claims_rejected_then_talk' and 'execution_feedback' in seen[-1]
        assert team.agreement.committed is None and team.agreement.last_turn == 0
    finally:
        team.close(0.)


def test_consult_needs_unanimity_otherwise_aborts(tmp_path):
    team = _team(tmp_path)
    event = {'event_id': 'approach-0', 'failure': {'reason': 'x'}}
    try:
        decision, rounds = dyn.consult(team, ['r1', 'r3'], event, FRAMES, HISTORY, {}, 1., turn=0,
                                       fixture_decisions={'r1': 'retry', 'r3': 'retry'})
        assert decision == 'retry' and len(rounds) == 1
        decision, rounds = dyn.consult(team, ['r1', 'r3'], {**event, 'event_id': 'approach-1'}, FRAMES,
                                       HISTORY, {}, 1., turn=1, fixture_decisions={'r1': 'retry'})
        assert decision == 'abort' and len(rounds) == 2 and rounds[-1]['decision'] is None
    finally:
        team.close(0.)


def test_recovery_reply_validation_and_recoverable_reasons():
    ok = {'request_id': 'q', 'event_id': 'e', 'decision': 'retry', 'reason': 'r', 'message': 'm'}
    assert dyn.validate_recovery_reply(json.dumps(ok), 'q', 'e')['decision'] == 'retry'
    for bad in ({**ok, 'decision': 'resume'}, {**ok, 'event_id': 'old'}, {**ok, 'extra': 1}):
        with pytest.raises(ValueError):
            dyn.validate_recovery_reply(json.dumps(bad), 'q', 'e')
    assert dyn.recoverable(RuntimeError('coarse RGB approach budget exhausted'))
    assert not dyn.recoverable(RuntimeError('pair perception result/capture provenance mismatch'))


class _Pair:
    def __init__(self, failures):
        self.failures, self.backoffs, self.holds = list(failures), 0, 0

    def approach(self):
        if self.failures:
            raise RuntimeError(self.failures.pop(0))
        return {'approach_ok': True}

    def _hold_pair(self):
        self.holds += 1

    def back_off(self):
        self.backoffs += 1


def _scene():
    return SimpleNamespace(step=lambda s: None, capture=lambda label: FRAMES, time=lambda: 3.,
                           bindings=SimpleNamespace(pair={'r1': 'r3', 'r3': 'r1'}),
                           command_history=HISTORY)


def _run_approach(monkeypatch, pair, decisions, **args):
    from scripts import run_dispatch_skills as runner
    events, consulted = [], []
    def consult(team, participants, event, *a, **k):
        consulted.append(event)
        return decisions.pop(0), [{'round': 0}]
    monkeypatch.setattr(dyn, 'consult', consult)
    team = SimpleNamespace(event=lambda *a, **k: events.append(a))
    result = {'coordination': {'mode': 'dynamic', 'recoveries': [], 'consulted': consulted}}
    namespace = SimpleNamespace(coordination='dynamic', approach_retries=2, **args)
    return runner._approach(pair, _scene(), team, {}, namespace, result), result, events


def test_dynamic_approach_retries_after_team_agrees(monkeypatch):
    pair = _Pair(['coarse RGB approach budget exhausted'])
    report, result, _ = _run_approach(monkeypatch, pair, ['retry'])
    assert report['approach_ok'] and pair.backoffs == 1 and pair.holds == 1
    row = result['coordination']['recoveries'][0]
    assert row['decision'] == 'retry' and row['participants'] == ['r3', 'r1'] and row['retries_left'] == 2


def test_dynamic_approach_aborts_on_team_decision_or_budget(monkeypatch):
    with pytest.raises(RuntimeError, match='team decided to abort'):
        _run_approach(monkeypatch, _Pair(['fine RGB alignment outside saved skill support']), ['abort'])
    pair = _Pair(['coarse RGB approach budget exhausted'] * 3)
    with pytest.raises(RuntimeError, match='budget exhausted$'):
        _run_approach(monkeypatch, pair, ['retry', 'retry'])
    assert pair.backoffs == 2
    with pytest.raises(RuntimeError, match='provenance'):
        _run_approach(monkeypatch, _Pair(['pair perception result/capture provenance mismatch']), [])


def test_diagnostic_injection_fails_the_first_completed_approach_only(monkeypatch):
    pair = _Pair([])
    report, result, _ = _run_approach(monkeypatch, pair, ['retry'], diagnostic_fail_approach_once=True)
    assert report['approach_ok'] and pair.backoffs == 1
    assert result['coordination']['recoveries'][0]['diagnostic_injection'] is True
    # The robots see a real controller stop reason; the injection is output-only.
    seen = result['coordination']['consulted'][0]
    assert 'diagnostic_injection' not in seen
    assert seen['failure']['reason'] == 'coarse RGB approach budget exhausted'


def test_plan_first_approach_stays_fail_closed():
    from scripts import run_dispatch_skills as runner
    with pytest.raises(RuntimeError, match='budget'):
        runner._approach(_Pair(['coarse RGB approach budget exhausted']), _scene(), None, {},
                         SimpleNamespace(coordination='plan_first'), {})


@pytest.mark.parametrize('extra', [['--plan-replay', 'p.json'], ['--realtime-control'], ['--executor', 'raw']])
def test_cli_rejects_dynamic_with_replay_realtime_or_raw(tmp_path, extra):
    from scripts.run_dispatch_e2e import main
    with pytest.raises(SystemExit):
        main(['--output', str(tmp_path / 'x'), '--coordination', 'dynamic',
              '--grasp-model-dir', str(tmp_path), '--stage-model-dir', str(tmp_path), *extra])


def test_cli_forwards_dynamic_options(tmp_path, monkeypatch):
    from scripts import run_dispatch_e2e
    seen = []
    monkeypatch.setattr('scripts.run_dispatch_skills.run', lambda args: seen.append(args) or 0)
    base = ['--grasp-model-dir', str(tmp_path), '--stage-model-dir', str(tmp_path)]
    assert run_dispatch_e2e.main(['--output', str(tmp_path / 'a'), '--coordination', 'dynamic',
                                  '--approach-retries', '1', '--diagnostic-fail-approach-once', *base]) == 0
    assert seen[-1].coordination == 'dynamic' and seen[-1].approach_retries == 1
    assert seen[-1].diagnostic_fail_approach_once
    with pytest.raises(SystemExit):
        run_dispatch_e2e.main(['--output', str(tmp_path / 'b'), '--diagnostic-fail-approach-once', *base])


def _backoff_pair(top, centers):
    """BoundPairSkill with only the back-off dependencies stubbed."""
    import numpy as np
    from pathlib import Path
    from harness.dispatch_skill_binding import PairCoarsePixels
    from scripts.dispatch_pair_skill import BoundPairSkill
    root = Path(__file__).parent/'fixtures'
    tracker = PairCoarsePixels.__new__(PairCoarsePixels)
    tracker.reference = (root/'camera_goal_transport'/'reference-top.jpg').read_bytes()
    tracker.centers = {slot: np.array(c, float) for slot, c in centers.items()}
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.io, pair.coarse, pair.calls, pair.driven = SimpleNamespace(), tracker, [], []
    pair.drive = lambda forwards, duration_s=.2: pair.driven.append(forwards)
    pair.capture = lambda tag: {'r1': {'raw_top_bytes': (root/'dynamic_recovery'/top).read_bytes()}}
    return pair, tracker


# Start-of-run own-probe centres vs. the last RGB-tracked coarse centres of D3.
IDENTITY_CENTERS = {'r1': [109.1, 358.5], 'r3': [108.1, 144.2]}
TRACKED_CENTERS = {'r1': [222.4, 356.9], 'r3': [218.6, 166.7]}


def test_backoff_keeps_rgb_tracked_crops_and_recentres_them():
    stale, _ = _backoff_pair('after-long-backoff-top.jpg', IDENTITY_CENTERS)
    raw = stale.capture('x')['r1']['raw_top_bytes']
    assert all(stale.coarse.decide(raw, s)['mask']['local_wheel_pixels'] == 0 for s in ('r1', 'r3'))
    pair, tracker = _backoff_pair('after-long-backoff-top.jpg', TRACKED_CENTERS)
    pair.back_off()
    # stop dwell, then four chunks of 5 reverse slices, each followed by a stop dwell
    assert pair.coarse is tracker and len(pair.driven) == 1 + 4*(5+1)
    assert all(pair.coarse.decide(raw, s)['ok'] is True for s in ('r1', 'r3'))
    rows = [c for c in pair.calls if c['kind'] == 'recovery_recenter']
    assert [r['slices'] for r in rows] == [0, 5, 10, 15, 20] and rows[-1]['lost_slots'] == []
    assert rows[0]['crop_centers_before_px'] == TRACKED_CENTERS


def test_backoff_chunks_stay_inside_the_crop_that_a_full_backoff_leaves():
    # E2 (v51): one 20-slice reverse moved both carriers ~64 px west of their
    # tracked crops; r1 fell below the 80 wheel-pixel floor. Crops that lag by
    # one or two 5-slice chunks (~16 px each) still re-centre on both carriers.
    true = {'r1': [151, 371], 'r3': [154, 181]}
    for lag in (16, 32):
        _, tracker = _backoff_pair('e2-after-fine-failure-backoff-top.jpg',
                                   {s: [x+lag, y] for s, (x, y) in true.items()})
        pair, _ = _backoff_pair('e2-after-fine-failure-backoff-top.jpg', {})
        pair.coarse = tracker
        pair._recenter_coarse(5, .05)
        row = pair.calls[-1]
        assert row['lost_slots'] == []
        assert all(abs(row['crop_centers_after_px'][s][0]-true[s][0]) <= 3 for s in true)
    pair, _ = _backoff_pair('e2-after-fine-failure-backoff-top.jpg', {'r1': [218.3, 370.2], 'r3': [215.7, 180.7]})
    pair._recenter_coarse(20, .05)
    assert pair.calls[-1]['lost_slots'] == ['r1']


def test_backoff_stays_synchronous_and_bounded():
    pair, _ = _backoff_pair('after-short-backoff-top.jpg', TRACKED_CENTERS)
    for slices, speed in ((21, .05), (10, .06), (0, .05)):
        with pytest.raises(ValueError):
            pair.back_off(slices, speed)
    pair.io.realtime_control = True
    with pytest.raises(RuntimeError, match='synchronous'):
        pair.back_off()


# --- general team decisions: events, delivery, job drops, grasp and box recovery ---

def test_events_carry_their_own_options_and_fail_closed_choice():
    event = dyn.make_event('job_failed', 'job_failed-3', participants=['r1', 'r2', 'r3'],
                           failure='box stopped: TARGET_NOT_VISIBLE', failed_job='box')
    assert event['options'] == ['continue_others', 'stop_all'] and event['fail_closed'] == 'stop_all'
    assert set(event['option_meanings']) == set(event['options']) and event['failed_job'] == 'box'
    ok = {'request_id': 'q', 'event_id': 'e', 'decision': 'continue_others', 'reason': 'r', 'message': 'm'}
    assert dyn.validate_recovery_reply(json.dumps(ok), 'q', 'e', event['options'])['decision'] == 'continue_others'
    with pytest.raises(ValueError):
        dyn.validate_recovery_reply(json.dumps({**ok, 'decision': 'retry'}), 'q', 'e', event['options'])
    with pytest.raises(ValueError):
        dyn.make_event('box_failure', 'x', participants=['r9'], failure='f')
    assert dyn.recoverable(RuntimeError('existing pair carry guard stopped: ABORT'), dyn.RECOVERABLE_GRASP_FAILURES)
    assert not dyn.recoverable(RuntimeError('visual placement confirmation failed'), dyn.RECOVERABLE_GRASP_FAILURES)


def test_consult_messages_reach_participants_only_and_disagreement_fails_closed():
    calls = []
    def ask(robots, build, validate, fixture, **kw):
        calls.append(kw)
        return {'r1': {'decision': 'continue_others'}, 'r2': {'decision': 'stop_all'}, 'r3': None}
    team = SimpleNamespace(ask=ask, inbox={r: [] for r in ('r1', 'r2', 'r3')})
    event = dyn.make_event('job_failed', 'j', participants=['r1', 'r2', 'r3'], failure='f')
    decision, rounds = dyn.consult(team, event['participants'], event, FRAMES, HISTORY, {}, 1., turn=0)
    assert decision == 'stop_all' and len(rounds) == 2
    assert all(kw['recipients'] == ['r1', 'r2', 'r3'] for kw in calls)


def test_runtime_delivers_consultation_messages_only_to_recipients(tmp_path):
    from scripts.three_robot_runtime import ThreeRobotRuntime
    team = ThreeRobotRuntime(tmp_path, run_id='t', mode='fixture')
    try:
        reply = lambda rid, rq: {'request_id': rq, 'event_id': 'e', 'decision': 'retry',
                                 'reason': 'r', 'message': f'from {rid}'}
        team.ask(['r1', 'r3'], lambda rid, rq: {'request_id': rq, 'messages': [], 'images': []},
                 lambda raw, rq: json.loads(raw), reply, phase='recovery-e-r0', turn=0, sim_time=1.,
                 recipients=['r1', 'r3'])
        assert [m['from_robot'] for m in team.inbox['r1']] == ['r3']
        assert [m['from_robot'] for m in team.inbox['r3']] == ['r1'] and team.inbox['r2'] == []
    finally:
        team.close(1.)


def _bindings():
    from harness.dispatch_skill_binding import SkillBindings
    from sim.research_dispatch_arena import authored_map
    from tests.test_dispatch_skill_binding import committed
    return SkillBindings(committed(), authored_map('open'), route_overlap=True)


def test_dropped_job_releases_resources_and_unblocks_peers_without_counting_as_delivered():
    b = _bindings()
    assert b.permission('beam', 'TRANSIT') and b.locks
    assert not b.permission('box', 'UNLOAD')  # box unloads after the beam
    b.drop('beam')
    assert b.settled('beam') and b.tasks['beam']['id'] not in b.finished and not b.locks
    assert b.permission('box', 'UNLOAD') and b.resource_events[-2]['event'] == 'drop'
    b.finish('box')
    with pytest.raises(ValueError):
        b.drop('box')


class _Team:
    def __init__(self):
        self.events = []
    def event(self, *a, **k):
        self.events.append((a, k))


def _decisions(monkeypatch, decisions):
    consulted = []
    def consult(team, participants, event, *a, **k):
        consulted.append(event)
        return decisions.pop(0), [{'round': 0}]
    monkeypatch.setattr(dyn, 'consult', consult)
    return consulted


def _job_scene(settled=()):
    dropped = []
    bindings = SimpleNamespace(pair={'r1': 'r3', 'r3': 'r1'}, solo='r2',
                               settled=lambda job: job in settled or job in dropped,
                               drop=dropped.append)
    return SimpleNamespace(step=lambda s: None, capture=lambda label: FRAMES, time=lambda: 9.,
                           bindings=bindings, command_history=HISTORY, dropped=dropped)


def test_job_failure_asks_all_three_and_drops_only_on_continue(monkeypatch):
    from scripts import run_dispatch_skills as runner
    consulted = _decisions(monkeypatch, ['continue_others', 'stop_all'])
    scene, result, downs = _job_scene(), {'coordination': {'recoveries': []}}, []
    runner._job_failed(scene, _Team(), {}, result, 'beam', RuntimeError('budget'), set_down=lambda: downs.append(1))
    assert consulted[0]['participants'] == ['r1', 'r2', 'r3'] and consulted[0]['failed_job'] == 'beam'
    assert consulted[0]['other_job_state'] == 'in_progress'
    assert scene.dropped == ['beam'] and downs == [1] and result['coordination']['dropped_jobs'][0]['job'] == 'beam'
    with pytest.raises(runner.TeamStop, match='no agreed job left'):
        runner._job_failed(scene, _Team(), {}, result, 'box', RuntimeError('x'), set_down=lambda: None)
    scene2, result2 = _job_scene(), {'coordination': {'recoveries': []}}
    with pytest.raises(runner.TeamStop, match='stop all'):
        runner._job_failed(scene2, _Team(), {}, result2, 'box', RuntimeError('x'), set_down=lambda: None)
    assert scene2.dropped == []


def test_box_failure_retries_with_the_box_robot_then_escalates_to_the_team(monkeypatch):
    from scripts import run_dispatch_skills as runner
    consulted = _decisions(monkeypatch, ['retry', 'abort', 'continue_others'])
    scene, result = _job_scene(), {'coordination': {'recoveries': []}}
    scene.solo_events = []
    args = SimpleNamespace(box_retries=2)
    event = {'reason': 'APPROACH_OVERSHOT', 'phase': 'approach', 'diagnostic_injection': True}
    assert runner._box_event(scene, _Team(), {}, args, result, event) == 'retry'
    assert consulted[0]['participants'] == ['r2'] and consulted[0]['kind'] == 'box_failure'
    assert 'diagnostic_injection' not in consulted[0]
    assert result['coordination']['recoveries'][0]['diagnostic_injection'] is True
    assert runner._box_event(scene, _Team(), {}, args, result, event) == 'dropped'
    assert consulted[2]['kind'] == 'job_failed' and scene.dropped == ['box']
    # A stop after the load is carried is not retried; the team decides at once.
    consulted = _decisions(monkeypatch, ['stop_all'])
    carrying = _job_scene();carrying.solo_events = []
    with pytest.raises(runner.TeamStop):
        runner._box_event(carrying, _Team(), {}, args, {'coordination': {'recoveries': []}},
                          {'reason': 'VISUAL_LOAD_DROPPED', 'phase': 'carry'})
    assert consulted[0]['kind'] == 'job_failed'


class _GraspPair(_Pair):
    def __init__(self, grasp_failures):
        super().__init__([])
        self.grasp_failures, self.regrasps, self.log = list(grasp_failures), 0, []
        self.grasp_report = {}
    def finish_grasp(self, predict, models):
        self.log.append('grasp')
        if self.grasp_failures:
            raise RuntimeError(self.grasp_failures.pop(0))
        return ['calls']
    def reset_for_regrasp(self):
        self.regrasps += 1;self.log.append('reset')
    def place(self):
        self.log.append('place')
    def verify_placement(self):
        self.log.append('verify')


def _beam_scene():
    finished = []
    bindings = SimpleNamespace(pair={'r1': 'r3', 'r3': 'r1'}, solo='r2', route_overlap=False,
                               transit_started=set(), permission=lambda obj, stage: True,
                               finish=finished.append)
    return SimpleNamespace(step=lambda s: None, capture=lambda label: FRAMES, time=lambda: 5.,
                           bindings=bindings, command_history=HISTORY, finished=finished)


def test_grasp_failure_regrasps_after_team_agreement(monkeypatch):
    from scripts import run_dispatch_skills as runner
    consulted = _decisions(monkeypatch, ['regrasp'])
    monkeypatch.setattr(runner, '_carry', lambda *a: None)
    pair, scene = _GraspPair(['preclose RGB outside learned grasp support']), _beam_scene()
    result = {'coordination': {'recoveries': []}}
    args = SimpleNamespace(coordination='dynamic', approach_retries=2)
    runner._beam_job(pair, scene, _Team(), {}, args, result, {})
    assert consulted[0]['kind'] == 'pair_grasp_failure' and consulted[0]['options'] == ['regrasp', 'abort']
    assert pair.log == ['grasp', 'reset', 'grasp', 'place', 'verify'] and pair.backoffs == 1
    assert scene.finished == ['beam']


def test_grasp_injection_and_no_regrasp_after_loaded_transit(monkeypatch):
    from scripts import run_dispatch_skills as runner
    consulted = _decisions(monkeypatch, ['regrasp'])
    monkeypatch.setattr(runner, '_carry', lambda *a: None)
    pair, scene = _GraspPair([]), _beam_scene()
    result = {'coordination': {'recoveries': []}}
    args = SimpleNamespace(coordination='dynamic', approach_retries=2, diagnostic_fail_grasp_once=True)
    runner._beam_job(pair, scene, _Team(), {}, args, result, {})
    assert consulted[0]['failure']['reason'] == 'existing pair carry guard stopped: ABORT'
    assert result['coordination']['recoveries'][0]['diagnostic_injection'] is True and pair.regrasps == 1
    def carry_fails(*a):
        scene.bindings.transit_started.add('beam')
        raise RuntimeError('existing pair carry guard stopped: ABORT')
    monkeypatch.setattr(runner, '_carry', carry_fails)
    with pytest.raises(RuntimeError, match='carry guard'):
        runner._beam_job(_GraspPair([]), scene, _Team(), {}, SimpleNamespace(coordination='dynamic',
                         approach_retries=2), {'coordination': {'recoveries': []}}, {})


def test_pair_knows_it_holds_cargo_only_from_issued_close_and_open():
    from scripts.dispatch_pair_skill import BoundPairSkill
    pair = BoundPairSkill.__new__(BoundPairSkill)
    pair.trace = [{'stage': 'grasp_rgb_recovery'}]
    assert not pair.holds_cargo()
    pair.trace.append({'stage': 'grasp_close'})
    assert pair.holds_cargo()
    pair.trace.append({'stage': 'place_open'})
    assert not pair.holds_cargo()
    pair._grasp_trace_start = len(pair.trace)
    assert not pair.holds_cargo()


def test_scene_pauses_box_and_applies_the_team_decision():
    from scripts.run_dispatch_skills import SkillScene
    scene = SkillScene.__new__(SkillScene)
    scene.solo_events, scene.solo_dropped, scene._solo_recovery = [], False, None
    scene.last_frames = {'kept': True}
    decisions = ['retry', 'dropped']
    scene.team_event_handler = lambda event: decisions.pop(0)
    scene._solo_event = {'reason': 'TARGET_NOT_VISIBLE', 'phase': 'approach', 'sim_time_s': 3.}
    scene._handle_solo_event()
    assert scene._solo_event is None and scene._solo_recovery[0]['kind'] == 'pose'
    assert [a['forward'] for a in scene._solo_recovery[1:]] == [-.05] * 10
    assert scene.last_frames == {'kept': True} and scene.solo_events[0]['decision'] == 'retry'
    scene._solo_event = {'reason': 'VISUAL_LOAD_DROPPED', 'phase': 'carry', 'sim_time_s': 4.}
    scene._handle_solo_event()
    assert scene.solo_dropped


def test_replay_overlay_shows_recent_peer_messages(tmp_path):
    from scripts.dispatch_replay import dialogue, dialogue_at
    (tmp_path / 'team').mkdir()
    rows = [{'kind': 'peer_message', 'sender': 'r1', 'phase': 'claim', 'sim_time_s': 5., 'text': 'beam upper'},
            {'kind': 'decision_explanation', 'sender': 'r1', 'phase': 'claim', 'sim_time_s': 5., 'text': 'hidden'},
            {'kind': 'peer_message', 'sender': 'r3', 'phase': 'recovery', 'sim_time_s': 48., 'text': 'retry ok'}]
    (tmp_path / 'team' / 'conversation.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    said = dialogue(tmp_path)
    assert len(said) == 2 and dialogue_at(said, 1.) == ''
    assert 'beam upper' in dialogue_at(said, 6.) and 'hidden' not in dialogue_at(said, 6.)
    assert dialogue_at(said, 50.).endswith('retry ok') and 'beam upper' not in dialogue_at(said, 50.)
    assert dialogue(tmp_path / 'missing') == []


def test_cli_validates_new_dynamic_options(tmp_path, monkeypatch):
    from scripts import run_dispatch_e2e
    seen = []
    monkeypatch.setattr('scripts.run_dispatch_skills.run', lambda args: seen.append(args) or 0)
    base = ['--grasp-model-dir', str(tmp_path), '--stage-model-dir', str(tmp_path)]
    assert run_dispatch_e2e.main(['--output', str(tmp_path / 'a'), '--coordination', 'dynamic', '--box-retries', '0',
                                  '--diagnostic-fail-grasp-once', '--diagnostic-fail-box-once', *base]) == 0
    assert seen[-1].box_retries == 0 and seen[-1].diagnostic_fail_grasp_once and seen[-1].diagnostic_fail_box_once
    for bad in (['--coordination', 'dynamic', '--box-retries', '4'], ['--diagnostic-fail-box-once'],
                ['--diagnostic-fail-grasp-once']):
        with pytest.raises(SystemExit):
            run_dispatch_e2e.main(['--output', str(tmp_path / 'b'), *bad, *base])
