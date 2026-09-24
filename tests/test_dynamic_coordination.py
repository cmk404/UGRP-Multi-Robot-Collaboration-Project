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
    assert dyn.recoverable(RuntimeError('diagnostic injected approach failure after a completed approach'))
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
    events = []
    monkeypatch.setattr(dyn, 'consult', lambda *a, **k: (decisions.pop(0), [{'round': 0}]))
    team = SimpleNamespace(event=lambda *a, **k: events.append(a))
    result = {'coordination': {'mode': 'dynamic', 'recoveries': []}}
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
