"""no-LLM offline end-to-end loop of the zone dialogue study (A + C + D + E + I).

Offline only: no model call, no physics, no simulator import. These tests check
the WIRING of the five packages and the three gate properties the design asks
for: channel isolation, cost accounting, and no evaluation backflow into a robot
input. They prove nothing about language understanding or any communication
effect.
"""
from __future__ import annotations

import json

import pytest

from harness import zone_study_eval as ev
from harness import zone_study_offline as off
from harness import zone_study_prompts_ko as pk
from harness import zone_study_protocol as zp
from harness.zone_sim_cost import params
from harness.zone_study_contract import (CALL_LOG_SCHEMA, COMMANDER, MAIN_CONDITIONS,
                                         MESSAGE_LOG_SCHEMA, PAYLOAD_SCHEMA, ROBOTS,
                                         ContractViolation, forbidden_key_hits, leader_for_seed,
                                         payload_violations, validate_log_record)
from harness.zone_study_scenarios import load as load_scenario, scenario_ids

CONDITIONS = tuple(MAIN_CONDITIONS) + ('reference_R',)
# One scenario for the per-condition detail tests; the cohort test walks all six.
SCENARIO = 's1_normal_mixed'
SEED = 601
_CACHE: dict = {}


@pytest.fixture(scope='module')
def library():
    return off.FrameLibrary()


def trial_of(condition, scenario_id=SCENARIO, seed=SEED, *, library=None, horizon_s=60.):
    key = (condition, scenario_id, seed, horizon_s)
    if key not in _CACHE:
        scenario = load_scenario(scenario_id)
        trial = off.OfflineTrial(scenario, condition=condition, seed=seed, horizon_s=horizon_s,
                                 library=library)
        _CACHE[key] = (trial, trial.run())
    return _CACHE[key]


# ---------------------------------------------------------------------------
# 1. the loop runs for every condition and produces package A log records

@pytest.mark.parametrize('condition', CONDITIONS)
def test_every_condition_runs_and_logs_package_a_records(condition, library):
    trial, result = trial_of(condition, library=library)
    assert result.calls, f'{condition} made no call'
    for record in result.calls:
        assert record['schema'] == CALL_LOG_SCHEMA
        validate_log_record(record)
        assert record['condition'] == condition and record['seed'] == SEED
        assert record['payload_validated'] is True and record['status'] == 'ok'
        assert record['provenance']['model'] == off.FIXTURE_MODEL
    for record in result.messages:
        assert record['schema'] == MESSAGE_LOG_SCHEMA
        validate_log_record(record)
    for record in result.actions:
        validate_log_record(record)
        assert forbidden_key_hits(record['arguments']) == []
    assert off.channel_checks(trial, result)['ok']
    assert off.cost_checks(trial, result)['ok']


@pytest.mark.parametrize('condition', CONDITIONS)
def test_the_trial_record_is_readable_by_package_i(condition, library):
    trial, result = trial_of(condition, library=library)
    record = trial.trial_record(result)
    assert record['schema'] == ev.TRIAL_SCHEMA
    parsed = ev.parse_trial(record)
    # requests/utterances are DERIVED from the A rows, never authored
    assert len(parsed['requests']) == len(record['calls'])
    assert len(parsed['utterances']) == len(record['messages'])
    metrics = ev.trial_metrics(parsed)
    assert metrics['efficiency']['condition'] == condition
    assert metrics['dialogue']['utterances'] == len(record['messages'])
    assert metrics['boundary']['unvalidated_payloads'] == []
    assert metrics['dialogue']['channel_violations'] == 0
    # the physics-free loop never delivers, so no trial may be scored a success
    assert metrics['efficiency']['success'] is False and metrics['efficiency']['censored'] is True
    # and the record must refuse a hand-written view of the same numbers
    with pytest.raises(ev.TrialError):
        ev.parse_trial({**record, 'requests': []})


# ---------------------------------------------------------------------------
# 2. channel isolation

def test_no_comm_sends_and_receives_nothing(library):
    trial, result = trial_of('no_comm', library=library)
    assert result.messages == [] and result.channel['sent'] == 0
    assert all(count == 0 for count in result.channel['received'].values())
    assert all(record['message_ids'] == [] for record in result.calls)
    assert result.cost['talk_sim_s'] == 0.0 and result.cost['delivery_sim_s'] == 0.0
    # package A gives this condition no inbox key at all
    assert 'inbox' not in trial.allowlist()
    payload = trial.build_inputs('r1', sim_time_s=trial.scheduler.now(),
                                request_id='req_probe').payload_dict()
    assert 'inbox' not in payload and payload['channel']['can_receive_from'] == []


def test_leader_ko_is_hub_and_spoke_with_a_rotating_leader(library):
    seeds = {sid: load_scenario(sid)['seeds'][0] for sid in scenario_ids()}
    leaders = {leader_for_seed('leader_ko', seed) for seed in seeds.values()}
    assert leaders == set(ROBOTS), f'the cohort seeds never make {set(ROBOTS) - leaders} leader'
    trial, result = trial_of('leader_ko', library=library)
    leader = trial.leader_id
    assert leader == leader_for_seed('leader_ko', SEED)
    assert result.messages, 'leader_ko sent nothing'
    for record in result.messages:
        if record['sender'] == leader:
            assert record['recipients'] != [leader]
        else:
            assert record['recipients'] == [leader], 'a follower reached another follower'
    assert result.channel['follower_to_follower'] == 0
    # a follower-to-follower attempt is rejected and recorded, not repaired
    followers = [r for r in ROBOTS if r != leader]
    receipt = trial.channel.send(followers[0], recipients=[followers[1]], text='직접 전달합니다.',
                                at_sim_s=trial.scheduler.now() + 1.)
    assert receipt.accepted is False and receipt.rejection == 'no_follower_to_follower'


def test_structured_carries_no_free_text(library):
    trial, result = trial_of('structured', library=library)
    assert result.messages, 'structured sent nothing'
    for record in result.messages:
        assert record['encoding'] == 'schema'
        assert set(record['body']) <= set(zp.STRUCT_FIELDS)
        assert not any(key in record['body'] for key in zp.FREE_TEXT_KEYS)
        assert record['chars'] == 0 and record['korean_ok'] is None
    assert result.channel['free_text_messages'] == 0
    for smuggle in ({'text': '직접 말합니다.'}, {'reason': '이유'}, {'note': '메모'}):
        receipt = trial.channel.send('r1', recipients=['r2'],
                                     structured={**_struct(trial), **smuggle},
                                     at_sim_s=trial.scheduler.now() + 1.)
        assert receipt.accepted is False


def test_peer_ko_carries_korean_free_text(library):
    trial, result = trial_of('peer_ko', library=library)
    assert result.messages, 'peer_ko sent nothing'
    for record in result.messages:
        assert record['encoding'] == 'free_ko' and record['korean_ok'] is True
        assert set(record['body']) == {'text'} and record['body']['text'].strip()
    dialogue = ev.trial_metrics(ev.parse_trial(trial.trial_record(result)))['dialogue']
    assert dialogue['korean_share'] == 1.0 and dialogue['code_switch_messages'] == 0
    assert dialogue['id_issue_messages'] == 0


def test_reference_R_robots_carry_no_llm_and_no_message_channel(library):
    trial, result = trial_of('reference_R', library=library)
    assert trial.actors == (COMMANDER,)
    assert result.messages == [] and result.channel['sent'] == 0
    assert all(record['actor'] == COMMANDER for record in result.calls)
    # the commander orders through an action, never a message
    kinds = {record['kind'] for record in result.actions}
    assert kinds == {'claim_order'}
    with pytest.raises(ContractViolation):                 # a robot gets no payload here
        trial.build_inputs('r1', sim_time_s=0., request_id='req_probe')


def _struct(trial):
    order = trial.sheet['orders'][0]
    return {'act': 'inform', 'item': order['order_id'], 'zone': order['destination_zone'],
            'role': None, 'passage': None, 'location_ref': None, 'state': 'unknown',
            'confidence': 'low', 'observed_at_sim_s': 0., 'reply_to': None}


# ---------------------------------------------------------------------------
# 3. the input boundary of every single call

@pytest.mark.parametrize('condition', CONDITIONS)
def test_every_request_payload_stays_inside_the_contract(condition, library):
    trial, result = trial_of(condition, library=library)
    actor = COMMANDER if condition == 'reference_R' else 'r1'
    bundled = trial.build_inputs(actor, sim_time_s=trial.scheduler.now(), request_id='req_probe')
    payload = bundled.payload_dict()
    assert payload['schema'] == PAYLOAD_SCHEMA
    # review finding 4: the deeply frozen view refuses mutation
    with pytest.raises(TypeError):
        bundled.payload['teacher_receipt'] = {'done': True}
    assert payload_violations(payload, seed=SEED) == []
    request = pk.build_request(bundled, window=trial.channel.window_context(
        actor, now_sim_s=trial.scheduler.now()) if trial.spec.channel_open else None)
    body = json.loads(request['messages'][1]['content'])
    allowed = set(trial.allowlist()) | {pk.WINDOW_KEY}
    assert set(body) <= allowed, sorted(set(body) - allowed)
    text = request['messages'][0]['content'] + request['messages'][1]['content']
    for banned in ('top_rgb', 'top_camera', 'cctv', 'nav_cam', 'teacher_receipt', 'ground_truth',
                   'grasp_success', 'qpos', 'hidden_event', 'referee', 'zone_counts'):
        assert banned not in text, banned
    assert not any('TOP' in image['label'] for image in request['images'])
    # every stored frame reference names a real file hash
    refs = payload.get('own_rgb_refs') or payload.get('team_rgb_refs') or ()
    assert refs and all(ref['sha256'] in set(trial.library.sha256.values()) for ref in refs)


def test_the_stored_frames_are_real_files_with_recorded_hashes(library):
    manifest = library.manifest()
    assert manifest['frames'] and len(manifest['frames']) == len(library.entries)
    for name, data in library.entries:
        assert data[:3] == b'\xff\xd8\xff', f'{name} is not a JPEG'
        assert manifest['frames'][name] == library.sha256[name]


# ---------------------------------------------------------------------------
# 4. cost accounting on the fake clock

@pytest.mark.parametrize('condition', CONDITIONS)
def test_thinking_and_talking_cost_sim_time(condition, library):
    trial, result = trial_of(condition, library=library)
    checks = off.cost_checks(trial, result)
    assert checks['ok'], checks['problems']
    assert checks['charged_sim_s'] > 0
    assert checks['params_version'] == params().version
    assert checks['params_digest'] == params().digest()
    for record in result.calls:
        assert record['sim_cost_s'] > 0                    # a call is never free
        assert record['wall_latency_s'] is None            # no wall time in this loop
        assert record['cost_terms']['params_digest'] == params().digest()
    if trial.spec.channel_open and result.messages:
        assert checks['talk_sim_s'] > 0 and checks['delivery_sim_s'] > 0
    else:
        assert checks['talk_sim_s'] == 0 and checks['delivery_sim_s'] == 0


def test_a_zero_cost_setting_makes_talking_free(library):
    """The diagnostic sweep point: scale 0 reproduces the free-thinking runners."""
    from harness.zone_sim_cost import params as cost_params

    free = cost_params('zone_sim_cost.v1_free')
    scenario = load_scenario(SCENARIO)
    trial = off.OfflineTrial(scenario, condition='peer_ko', seed=SEED, horizon_s=20.,
                             cost_params=free, library=library)
    result = trial.run()
    assert result.calls and all(record['sim_cost_s'] == 0. for record in result.calls)
    assert off.cost_checks(trial, result)['charged_sim_s'] == 0.
    paid_trial, paid = trial_of('peer_ko', library=library)
    assert off.cost_checks(paid_trial, paid)['charged_sim_s'] > 0


def test_a_message_is_only_visible_after_its_delivery_time(library):
    trial, result = trial_of('peer_ko', library=library)
    assert result.messages
    record = result.messages[0]
    created = record['created_at_sim_s']
    delivered = min(d['delivered_at_sim_s'] for d in record['deliveries'])
    assert delivered > created
    recipient = record['recipients'][0]
    before = trial.channel.inbox(recipient, now_sim_s=created)
    after = trial.channel.inbox(recipient, now_sim_s=delivered)
    assert record['message_id'] not in [m['message_id'] for m in before]
    assert record['message_id'] in [m['message_id'] for m in after]


# ---------------------------------------------------------------------------
# 5. the fixture actor sees only its own request

def test_the_fixture_actor_takes_only_its_own_request():
    report = off.actor_isolation()
    assert report['ok'], report['problems']
    assert report['parameters'] == ['self', 'request']
    assert report['attributes'] == ['actor', 'condition', 'seed', 'calls']


@pytest.mark.parametrize('condition', CONDITIONS)
def test_changing_the_private_section_cannot_change_the_sim_trace(condition, library):
    """Hidden events and a fake referee list must not reach any robot."""
    probe = off.backflow_probe(SCENARIO, condition, SEED, horizon_s=60., library=library)
    assert probe['ok'], probe['problems']
    assert all(probe['identical'].values())
    assert probe['hidden_events'][0] != probe['hidden_events'][1] \
        or probe['hidden_events'] == [0, 1]                # the probe really changed the section


def test_the_evaluation_never_reaches_a_robot_input(library):
    """Running the evaluator must not change the next trial's inputs or trace."""
    trial, result = trial_of('peer_ko', library=library)
    first = trial.trial_record(result)
    metrics = ev.trial_metrics(ev.parse_trial(first))
    assert metrics['efficiency']['par_makespan_sim_s'] > 0
    assert first['referee'] == {}, 'the offline record carries no ground truth to leak'
    scenario = load_scenario(SCENARIO)
    again = off.OfflineTrial(scenario, condition='peer_ko', seed=SEED, horizon_s=60.,
                             library=library)
    repeat = again.run()
    assert list(repeat.trace) == list(result.trace)
    assert [r['input_sha256'] for r in repeat.requests] == [r['input_sha256'] for r in result.requests]
    # and the record the evaluator read is unchanged
    assert trial.trial_record(result) == first


# ---------------------------------------------------------------------------
# 6. the whole cohort: 6 scenarios x (4 main + R) x 1 seed

def test_the_cohort_runs_and_every_gate_check_passes(library):
    bundle = off.run_smoke(scenario_ids(), CONDITIONS, horizon_s=40., library=library, probe=False)
    assert len(bundle['trials']) == len(scenario_ids()) * len(CONDITIONS) == 30
    assert bundle['ok'], [row for row in bundle['trials']
                          if not (row['channel']['ok'] and row['cost']['ok'])]
    assert bundle['actor_isolation']['ok']
    by_condition = {}
    for row in bundle['trials']:
        by_condition.setdefault(row['condition'], []).append(row)
    assert set(by_condition) == set(CONDITIONS)
    for condition, rows in by_condition.items():
        assert len(rows) == len(scenario_ids())
        assert len({row['seed'] for row in rows}) == len(scenario_ids())    # one seed per scenario
        if condition == 'no_comm':
            assert all(row['channel']['sent'] == 0 for row in rows)
        if condition == 'leader_ko':
            assert {row['leader_id'] for row in rows} == set(ROBOTS)        # rotation covered
            assert all(row['channel']['follower_to_follower'] == 0 for row in rows)
        if condition == 'structured':
            assert all(row['channel']['free_text_messages'] == 0 for row in rows)
    # package I reads the whole cohort and keeps failures in the denominators
    trials = [ev.parse_trial(record) for record in bundle['trial_records']]
    summary = ev.summarise(trials)
    assert summary['trials'] == 30 and set(summary['conditions']) == set(CONDITIONS)
    for condition, row in summary['conditions'].items():
        assert row['trials'] == len(scenario_ids()) and row['successes'] == 0
        assert row['censored_trials'] == row['trials']
        assert row['is_reference'] is (condition == 'reference_R')
    assert summary['scenarios'] == sorted(scenario_ids())
