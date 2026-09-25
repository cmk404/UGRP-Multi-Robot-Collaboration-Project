"""Study contract: condition registry, channel isolation and the robot-facing boundary.

Offline only: no simulator, no model call, no file write.
"""
import copy

import pytest

from harness import zone_study_contract as c

SEEDS = (11, 12, 13, 14, 15, 16)
FRAME_SHA = 'b' * 64


def _static_map():
    projection = {'schema': 'ugrp.zone_public_map.v1', 'map_id': 'zone_x', 'version': 1,
                  'bounds_m': [-1., 5., -3., 1.], 'walls': [{'id': 'wall_north', 'center_m': [2., 1.],
                                                             'half_extents_m': [3., .025], 'height_m': .1,
                                                             'perimeter': True}],
                  'passages': [{'id': 'door_narrow', 'kind': 'door', 'width_m': .5, 'lanes': 1}],
                  'regions': {'A': {'center_m': [1., 0.]}, 'pickup': {'center_m': [0., 0.]}},
                  'zone_slots': {'A': [{'slot_id': 'A1'}]},
                  'pickup_bays': [{'bay_id': 'P1', 'slots': [{'slot_id': 'P1-2'}]}]}
    return {'map_id': 'zone_x', 'map_file_sha256': 'f' * 64, 'public_map': projection,
            'public_map_sha256': c.digest(projection),
            'schematic_ref': {'ref': 'map-zone_x-schematic', 'kind': 'map_schematic',
                              'png_sha256': 'a' * 64, 'width_px': 700, 'height_px': 400, 'px_per_m': 100}}


def _sheet():
    return {'schema': c.ORDER_SHEET_SCHEMA, 'scenario_id': 'S1', 'map_id': 'zone_x',
            'orders': [{'order_id': 'order-1', 'kind': 'long_beam', 'count': 1,
                        'item_ids': ['long_beam-1'], 'required_robots': 2, 'destination_zone': 'A',
                        'initial_location': {'pickup_bay': 'P1', 'slot': 'P1-2'},
                        'identity': 'specific_item'}],
            'kinds': {'long_beam': {'required_robots': 2, 'roles': ['end_neg', 'end_pos']}},
            'team_size': 3, 'note_ko': '설정 시점의 계획 정보다'}


def _payload(condition='peer_ko', robot='r2', seed=11, **extra):
    payload = {'schema': c.PAYLOAD_SCHEMA, 'request_id': 'req-1', 'robot_id': robot,
               'condition': condition, 'sim_time_s': 12.5, 'static_map': _static_map(),
               'order_sheet': _sheet(),
               'own_rgb_refs': [{'ref': f'own-{robot}-0042', 'kind': 'own_wrist_rgb',
                                 'captured_at_sim_s': 12.5, 'sha256': FRAME_SHA}],
               'own_command_history': [{'command_id': 'cmd-1', 'issued_at_sim_s': 3.,
                                        'kind': 'goto', 'arguments': {'target_ref': 'P1'},
                                        'local_state': 'command_issued'}],
               'self_belief': {'region': 'unknown', 'confidence': 'low', 'sources': []},
               'channel': c.channel_section(condition, robot, seed)}
    if 'inbox' in c.condition(condition).input_allowlist:
        payload['inbox'] = []
    if c.condition(condition).leader_rotation:
        payload['leader_id'] = c.leader_for_seed(condition, seed)
        payload['role'] = c.role_of(condition, robot, seed)
    payload.update(extra)
    return payload


# ---------------------------------------------------------------------------
# Condition registry

def test_registry_declares_the_five_study_conditions():
    assert sorted(c.CONDITIONS) == ['leader_ko', 'no_comm', 'peer_ko', 'reference_R', 'structured']
    assert sorted(c.MAIN_CONDITIONS) == ['leader_ko', 'no_comm', 'peer_ko', 'structured']
    assert c.CONDITIONS['reference_R'].is_main is False
    assert c.CONDITIONS['reference_R'].robot_llm is False
    assert all(c.CONDITIONS[n].robot_llm for n in c.MAIN_CONDITIONS)
    # Only the channel differs between the main conditions: same allowlist apart from the inbox keys.
    common = {'schema', 'request_id', 'robot_id', 'condition', 'sim_time_s', 'static_map', 'order_sheet',
              'own_rgb_refs', 'own_command_history', 'self_belief', 'channel'}
    for name in c.MAIN_CONDITIONS:
        assert common <= c.CONDITIONS[name].input_allowlist
        assert c.CONDITIONS[name].input_allowlist - common <= {'inbox', 'leader_id', 'role'}


def test_no_comm_has_no_channel_at_all():
    assert c.allowed_edges('no_comm') == frozenset()
    assert c.channel_section('no_comm', 'r1')['can_send_to'] == []
    assert c.channel_section('no_comm', 'r1')['can_receive_from'] == []
    with pytest.raises(c.ContractViolation):
        c.check_message('no_comm', 'r1', ['r2'], {'text': '같이 가자'})


def test_peer_conditions_are_a_full_mesh():
    for name in ('peer_ko', 'structured'):
        assert c.allowed_edges(name) == frozenset((a, b) for a in c.ROBOTS for b in c.ROBOTS if a != b)
        assert len(c.allowed_edges(name)) == 6
        assert c.role_of(name, 'r1') == 'peer'


def test_leader_rotates_over_every_robot_and_stays_hub_and_spoke():
    leaders = [c.leader_for_seed('leader_ko', s) for s in SEEDS]
    assert set(leaders) == set(c.ROBOTS)
    assert leaders == ['r3', 'r1', 'r2', 'r3', 'r1', 'r2']
    for seed in SEEDS:
        leader = c.leader_for_seed('leader_ko', seed)
        followers = [r for r in c.ROBOTS if r != leader]
        edges = c.allowed_edges('leader_ko', seed)
        assert len(edges) == 4
        for follower in followers:
            assert (leader, follower) in edges and (follower, leader) in edges
            assert c.role_of('leader_ko', follower, seed) == 'follower'
        assert (followers[0], followers[1]) not in edges
        assert (followers[1], followers[0]) not in edges
        assert c.role_of('leader_ko', leader, seed) == 'leader'
        # a follower may report and object to the leader, never to the other follower
        c.check_message('leader_ko', followers[0], [leader], {'text': 'door_narrow가 막혀서 못 갑니다'}, seed=seed)
        with pytest.raises(c.ContractViolation, match='hub-and-spoke'):
            c.check_message('leader_ko', followers[0], [followers[1]], {'text': '네가 먼저 가'}, seed=seed)


def test_leader_condition_needs_a_seed():
    with pytest.raises(c.ContractViolation):
        c.allowed_edges('leader_ko')
    with pytest.raises(c.ContractViolation):
        c.leader_for_seed('peer_ko', 11)


def test_reference_R_is_a_commander_downlink_to_no_llm_robots():
    edges = c.allowed_edges('reference_R')
    assert edges == frozenset((c.COMMANDER, r) for r in c.ROBOTS)
    with pytest.raises(c.ContractViolation):
        c.check_message('reference_R', 'r1', [c.COMMANDER], {'act': 'inform'})
    assert c.role_of('reference_R', 'r1') == 'executor'
    problems = c.payload_violations(_payload('reference_R', 'r1'))
    assert any('carry no LLM' in p for p in problems)


def test_condition_manifest_and_registry_hash_are_stable():
    manifest = c.condition_manifest('leader_ko', 13)
    assert manifest['leader_id'] == 'r2' and manifest['edges'] and manifest['is_main'] is True
    assert manifest['contract_version'] == c.CONTRACT_VERSION
    assert c.registry_sha256() == c.registry_sha256() and len(c.registry_sha256()) == 64


# ---------------------------------------------------------------------------
# Encodings

def test_free_korean_is_required_in_the_free_text_conditions():
    c.check_message('peer_ko', 'r1', ['r2', 'r3'], {'text': 'order-1은 제가 A로 가져갑니다'})
    with pytest.raises(c.ContractViolation, match='no Korean'):
        c.check_message('peer_ko', 'r1', ['r2'], {'text': 'I take order-1 to zone A'})
    with pytest.raises(c.ContractViolation):
        c.check_message('peer_ko', 'r1', ['r2'], {'text': '가자', 'act': 'inform'})
    report = c.free_text_report('r2, door_narrow는 blocked 입니다', literals={'door_narrow'})
    assert report['ok'] and report['latin_words'] == []
    assert c.free_text_report('please help')['ok'] is False
    assert c.free_text_report('r1, order-1 is blocked 입니다',
                              literals={'order-1'})['latin_words'] == ['is']


def test_structured_condition_takes_schema_messages_only():
    vocabulary = c.Vocabulary(items=frozenset({'order-1'}), roles=frozenset({'end_neg'}),
                              passages=frozenset({'door_narrow'}), location_refs=frozenset({'P1-2'}))
    body = {'act': 'propose', 'item': 'order-1', 'zone': 'A', 'role': 'end_neg',
            'passage': 'door_narrow', 'location_ref': 'P1-2', 'state': 'present',
            'confidence': 'medium', 'observed_at_sim_s': 10.5, 'reply_to': None}
    c.check_message('structured', 'r1', ['r2'], body, vocabulary=vocabulary)
    with pytest.raises(c.ContractViolation):
        c.check_message('structured', 'r1', ['r2'], {'text': '자유 문장'}, vocabulary=vocabulary)
    for smuggled in ('reason', 'note', 'other', 'text'):
        with pytest.raises(c.ContractViolation, match='free-text field'):
            c.check_message('structured', 'r1', ['r2'], {**body, smuggled: '우회'}, vocabulary=vocabulary)
    with pytest.raises(c.ContractViolation, match='vocabulary'):
        c.check_message('structured', 'r1', ['r2'], {**body, 'item': 'order-9'}, vocabulary=vocabulary)
    for bad in ({'act': 'shout'}, {'act': 'inform', 'state': 'arrived'},
                {'act': 'inform', 'confidence': 0.9}, {'act': 'inform', 'observed_at_sim_s': -1}):
        assert c.structured_violations(bad)


def test_message_envelope_keeps_the_meaning_in_the_body():
    envelope = c.message_envelope('peer_ko', 'm-1', 'r1', ['r2'], {'text': 'order-1 제가 갑니다'},
                                  created_at_sim_s=4.5)
    assert envelope['schema'] == c.MESSAGE_ENVELOPE_SCHEMA
    assert set(envelope) == {'schema', 'message_id', 'sender', 'recipients', 'encoding',
                             'created_at_sim_s', 'reply_to', 'body'}
    assert envelope['encoding'] == 'free_ko' and envelope['body'] == {'text': 'order-1 제가 갑니다'}
    with pytest.raises(c.ContractViolation, match='cannot send to itself'):
        c.message_envelope('peer_ko', 'm-2', 'r1', ['r1'], {'text': '혼잣말'}, created_at_sim_s=1.)


# ---------------------------------------------------------------------------
# Robot-facing boundary

def test_boundary_manifest_declares_the_public_private_schema():
    manifest = c.boundary_manifest()
    assert manifest['contract_version'] == c.CONTRACT_VERSION
    assert set(manifest['input_allowlist']) == set(c.CONDITIONS)
    for name, spec in c.CONDITIONS.items():
        assert manifest['input_allowlist'][name] == sorted(spec.input_allowlist)
    assert set(manifest['robot_facing']) == {'static_map', 'order_sheet', 'own_rgb_refs',
                                            'own_command_history', 'self_belief', 'inbox', 'channel'}
    assert set(manifest['evaluation_only']) == {'cameras', 'ground_truth', 'judgements', 'peer_state',
                                                'schedule', 'metrics'}
    assert all(manifest['evaluation_only'].values()) and len(manifest['forbidden_keys']) > 50
    assert manifest['own_command_states'] == list(c.LOCAL_STATES)
    # nothing a robot may see is also declared evaluation-only
    assert not set(manifest['robot_facing']) & set(manifest['forbidden_keys'])


@pytest.mark.parametrize('section, patch', [
    ('static_map', {'survey_xy_m': {'r1': [1.2, -2.3]}}),          # renamed ground-truth poses
    ('static_map', {'overhead_still': {'ref': 'overhead-0001'}}),  # renamed TOP frame
    ('static_map', {'teammate_views': [{'ref': 'own-r1-0003'}]}),  # peer cameras
    ('static_map', {'board': {'claimed_by': {'order-1': 'r1'}}}),  # host board
    ('static_map', {'upcoming': [{'at_sim_s': 60}]}),              # hidden schedule
    ('static_map', {'kpi': {'model_calls': 7}}),                   # metrics
    ('static_map', {'source_view': 'TOP-WEST'}),                   # TOP name with a hyphen
    ('order_sheet', {'status': 'fulfilled'}),                      # completion judgement
    ('order_sheet', {'coach_ack': {'order-1': 'grasped_ok'}}),     # renamed teacher receipt
])
def test_validator_rejects_renamed_evaluation_data_in_every_section(section, patch):
    payload = _payload()
    payload[section].update(patch)
    with pytest.raises(c.ContractViolation, match='outside the contract'):
        c.validate_robot_payload(payload, seed=11)


def test_validator_rejects_renamed_evaluation_data_in_list_entries():
    joints = _payload()
    joints['own_command_history'][0]['servo_deg'] = [1500, 1500, 1500]
    with pytest.raises(c.ContractViolation, match='outside the contract'):
        c.validate_robot_payload(joints, seed=11)
    contact = _payload()
    contact['own_command_history'][0]['arguments']['jaw_contact_n'] = 3.4
    with pytest.raises(c.ContractViolation, match='outside the contract'):
        c.validate_robot_payload(contact, seed=11)
    overlay = _payload()
    overlay['own_rgb_refs'][0]['overlay_from_overview'] = {'item_xy_px': [120, 88]}
    with pytest.raises(c.ContractViolation, match='outside the contract'):
        c.validate_robot_payload(overlay, seed=11)


def test_validator_requires_a_hashed_verifiable_map_and_frames():
    fake = _payload()
    fake['static_map'] = {'map_id': 'zone_x', 'public_map': {'map_id': 'zone_x', 'bounds_m': [],
                                                             'walls': [], 'regions': {}}}
    with pytest.raises(c.ContractViolation, match='public_map_sha256'):
        c.validate_robot_payload(fake, seed=11)
    unhashed = _payload()
    del unhashed['own_rgb_refs'][0]['sha256']
    with pytest.raises(c.ContractViolation, match='sha256'):
        c.validate_robot_payload(unhashed, seed=11)


def test_validator_rejects_non_json_payloads():
    payload = _payload()
    payload['self_belief']['sources'] = iter(['own-r2-0042'])
    with pytest.raises(c.ContractViolation, match='JSON'):
        c.validate_robot_payload(payload, seed=11)


def test_validator_rejects_information_from_the_future():
    frame = _payload()
    frame['own_rgb_refs'][0]['captured_at_sim_s'] = 9999.
    with pytest.raises(c.ContractViolation, match='after the call time'):
        c.validate_robot_payload(frame, seed=11)
    command = _payload()
    command['own_command_history'][0]['issued_at_sim_s'] = 9999.
    with pytest.raises(c.ContractViolation, match='after the call time'):
        c.validate_robot_payload(command, seed=11)
    message = _payload()
    message['inbox'] = [c.message_envelope('peer_ko', 'm-1', 'r1', ['r2'], {'text': '지금 갑니다'},
                                           created_at_sim_s=9999.)]
    with pytest.raises(c.ContractViolation, match='after the call time'):
        c.validate_robot_payload(message, seed=11)


def test_validator_needs_the_seed_and_never_raises_from_payload_violations():
    stale = _payload('leader_ko', 'r2', 11)
    stale['leader_id'] = 'r1'
    problems = c.payload_violations(stale)                      # no seed: reported, not raised
    assert any('needs the seed' in p for p in problems)
    assert c.payload_violations(_payload('leader_ko', 'r2', 11), seed=None)


def test_validator_checks_every_recipient_of_an_inbox_message():
    payload = _payload('peer_ko', 'r2', 11)
    payload['inbox'] = [{'schema': c.MESSAGE_ENVELOPE_SCHEMA, 'message_id': 'm-1', 'sender': 'r1',
                         'recipients': ['r2', 'commander', 'ghost_robot'], 'encoding': 'free_ko',
                         'created_at_sim_s': 4.5, 'reply_to': None, 'body': {'text': '가겠습니다'}}]
    problems = c.payload_violations(payload, seed=11)
    assert any('outside this condition' in p for p in problems)
    assert any('not allowed' in p for p in problems)


def test_structured_ids_are_checked_against_the_payloads_own_vocabulary():
    payload = _payload('structured', 'r2', 11)
    vocabulary = c.vocabulary_from_payload(payload)
    assert {'order-1', 'long_beam', 'long_beam-1'} <= vocabulary.items
    assert {'P1', 'P1-2', 'A1', 'A', 'pickup'} <= vocabulary.location_refs
    payload['inbox'] = [{'schema': c.MESSAGE_ENVELOPE_SCHEMA, 'message_id': 'm-1', 'sender': 'r1',
                         'recipients': ['r2'], 'encoding': 'schema', 'created_at_sim_s': 4.5,
                         'reply_to': None, 'body': {'act': 'inform', 'location_ref': 'x1.23y-4.56'}}]
    with pytest.raises(c.ContractViolation, match='vocabulary'):
        c.validate_robot_payload(payload, seed=11)


def test_validator_rejects_a_tampered_channel():
    payload = _payload('no_comm', 'r1', 11)
    payload['channel'] = {**payload['channel'], 'can_receive_from': ['r2', 'r3']}
    with pytest.raises(c.ContractViolation, match='channel does not match'):
        c.validate_robot_payload(payload, seed=11)


def test_validator_rejects_non_token_ids():
    payload = _payload()
    payload['request_id'] = '../../etc/passwd'
    with pytest.raises(c.ContractViolation, match='request_id'):
        c.validate_robot_payload(payload, seed=11)


def test_a_clean_payload_passes_for_every_main_condition():
    for name in c.MAIN_CONDITIONS:
        assert c.validate_robot_payload(_payload(name), seed=11)


@pytest.mark.parametrize('patch', [
    {'top_frame': 'data:image/jpeg;base64,AAA'},
    {'top_cameras': [{'name': 'cctv_top'}]},
    {'poses': {'r1': [1., 2.]}},
    {'teacher_receipt': {'order-1': 'finished'}},
    {'zone_counts': {'A': 1}},
    {'hidden_events': [{'at_sim_s': 60}]},
    {'completion': {'order-1': True}},
    {'peer_status': {'r3': 'busy'}},
    {'eval': {'makespan': 12.}},
])
def test_validator_rejects_evaluation_only_top_level_keys(patch):
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(_payload(**patch), seed=11)


def test_validator_rejects_evaluation_only_keys_nested_anywhere():
    deep = _payload()
    deep['own_command_history'][0]['arguments']['teacher_receipt'] = True
    assert c.forbidden_key_hits(deep)
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(deep, seed=11)
    nested = _payload()
    nested['order_sheet']['orders'][0]['object_pose'] = [1., 2., 0.]
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(nested, seed=11)
    inside_map = _payload()
    inside_map['static_map']['public_map']['top_views'] = ['TOP_SW']
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(inside_map, seed=11)


def test_validator_rejects_foreign_and_simulator_cameras():
    other = _payload()
    other['own_rgb_refs'] = [{'ref': 'own-r3-0007', 'kind': 'own_wrist_rgb', 'captured_at_sim_s': 1.}]
    with pytest.raises(c.ContractViolation, match='another robot'):
        c.validate_robot_payload(other, seed=11)
    nav = _payload()
    nav['own_rgb_refs'] = [{'ref': 'nav_cam-r2-0007', 'kind': 'own_wrist_rgb', 'captured_at_sim_s': 1.}]
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(nav, seed=11)
    team = _payload()
    team['team_rgb_refs'] = [{'ref': 'own-r1-0001', 'kind': 'own_wrist_rgb', 'captured_at_sim_s': 1.}]
    with pytest.raises(c.ContractViolation, match='allowlist'):
        c.validate_robot_payload(team, seed=11)


def test_validator_rejects_non_ascii_keys_and_unknown_keys():
    korean_key = _payload()
    korean_key['self_belief']['확신'] = 'low'
    assert c.non_ascii_keys(korean_key)
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(korean_key, seed=11)
    unknown = _payload(nav_hint={'xy_m': [1., 2.]})
    with pytest.raises(c.ContractViolation, match='allowlist'):
        c.validate_robot_payload(unknown, seed=11)


def test_validator_enforces_the_channel_of_each_condition():
    silent = _payload('no_comm')
    silent['inbox'] = [{'message_id': 'm-1', 'sender': 'r1', 'recipients': ['r2'],
                        'encoding': 'free_ko', 'body': {'text': '가자'}}]
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(silent, seed=11)
    leader_seed = 11                      # leader r3, so r1 and r2 are followers
    crossed = _payload('leader_ko', 'r2', leader_seed)
    crossed['inbox'] = [c.message_envelope('leader_ko', 'm-1', 'r3', ['r2'], {'text': 'order-1 맡아 주세요'},
                                           created_at_sim_s=2., seed=leader_seed)]
    assert c.validate_robot_payload(crossed, seed=leader_seed)
    peer_to_peer = copy.deepcopy(crossed)
    peer_to_peer['inbox'][0]['sender'] = 'r1'
    with pytest.raises(c.ContractViolation, match='hub-and-spoke'):
        c.validate_robot_payload(peer_to_peer, seed=leader_seed)
    not_addressed = copy.deepcopy(crossed)
    not_addressed['inbox'][0]['recipients'] = ['r1']
    with pytest.raises(c.ContractViolation, match='not addressed'):
        c.validate_robot_payload(not_addressed, seed=leader_seed)
    wrong_encoding = _payload('structured')
    wrong_encoding['inbox'] = [{'message_id': 'm-1', 'sender': 'r1', 'recipients': ['r2'],
                                'encoding': 'free_ko', 'body': {'text': '자유 문장'}}]
    with pytest.raises(c.ContractViolation):
        c.validate_robot_payload(wrong_encoding, seed=11)


def test_validator_checks_the_leader_rotation_and_the_map_hash():
    stale = _payload('leader_ko', 'r2', 11)
    stale['leader_id'] = 'r1'
    with pytest.raises(c.ContractViolation, match='rotation'):
        c.validate_robot_payload(stale, seed=11)
    tampered = _payload()
    tampered['static_map']['public_map']['regions']['A']['center_m'] = [9., 9.]
    with pytest.raises(c.ContractViolation, match='public_map_sha256'):
        c.validate_robot_payload(tampered, seed=11)


def test_validator_rejects_host_judgements_in_the_command_history():
    payload = _payload()
    payload['own_command_history'][0]['local_state'] = 'grasp_confirmed'
    with pytest.raises(c.ContractViolation, match='self state'):
        c.validate_robot_payload(payload, seed=11)


# ---------------------------------------------------------------------------
# Log schema

def _provenance():
    return {'registry_sha256': c.registry_sha256(), 'order_sheet_sha256': 'c' * 64,
            'map_file_sha256': 'd' * 64, 'public_map_sha256': 'e' * 64, 'code_sha': '5288933',
            'execution_bundle_id': 'zone-study-A-1', 'model': 'fixture-model', 'provider': 'fixture',
            'model_settings_sha256': None, 'prompt_template_sha256': None, 'cost_profile_id': 'dev-v1',
            'input_profile_id': 'zone_study_inputs.v1'}


def _call_record(**patch):
    record = {'schema': c.CALL_LOG_SCHEMA, 'run_id': 'run-1', 'condition': 'peer_ko', 'seed': 11,
              'actor': 'r2', 'role': 'peer', 'request_id': 'req-1', 'call_index': 0, 'trigger': 'start',
              'requested_at_sim_s': 10., 'released_at_sim_s': 13.7, 'sim_cost_s': 3.7,
              'cost_terms': {'alpha_s': 1., 'beta_s_per_token': .02, 'gamma_s_per_utterance': .3,
                             'output_tokens': 120, 'utterances': 1},
              'input_sha256': 'a' * 64, 'input_tokens': {'text': 5200, 'image': 2, 'cached': 0},
              'output_tokens': 120, 'wall_latency_s': 2.1, 'http_attempts': 1, 'status': 'ok',
              'action_id': 'act-1', 'message_ids': ['m-1'],
              'decision_sources': ['own-r2-0042', 'cmd-1'], 'payload_validated': True,
              'provenance': _provenance()}
    record.update(patch)
    return record


def test_call_records_follow_the_declared_schema():
    assert c.validate_log_record(_call_record())
    assert c.call_record_violations(_call_record(sim_cost_s=1.))
    assert c.call_record_violations(_call_record(released_at_sim_s=9.))
    assert c.call_record_violations(_call_record(status='fine'))
    assert c.call_record_violations(_call_record(payload_validated=False))
    assert c.call_record_violations(_call_record(condition='dynamic'))
    assert c.call_record_violations(_call_record(provenance={'model': 'x'}))
    assert c.call_record_violations(_call_record(provenance={**_provenance(), 'gpu': 'a100'}))
    # a call blocked by the validator is still logged, with the status that says so
    assert c.validate_log_record(_call_record(payload_validated=False, status='input_rejected'))
    record = _call_record()
    del record['trigger']
    assert any('missing field' in p for p in c.call_record_violations(record))
    assert c.call_record_violations({**_call_record(), 'top_frame': 'x'})


def _message_record(**patch):
    body = {'text': 'order-1은 제가 A로 갑니다'}
    record = {'schema': c.MESSAGE_LOG_SCHEMA, 'run_id': 'run-1', 'condition': 'peer_ko', 'seed': 11,
              'message_id': 'm-1', 'sender': 'r1', 'recipients': ['r2', 'r3'], 'encoding': 'free_ko',
              'reply_to': None, 'created_at_sim_s': 4.5, 'delivered_at_sim_s': 4.6,
              'deliveries': [{'recipient': 'r2', 'delivered_at_sim_s': 4.6, 'status': 'delivered'},
                             {'recipient': 'r3', 'delivered_at_sim_s': 4.8, 'status': 'delivered'}],
              'delivery_delay_s': .1, 'status': 'delivered', 'rejected_reason': None, 'body': body,
              'body_sha256': c.digest(body), 'act': 'inform', 'chars': len(body['text']),
              'korean_ok': True}
    record.update(patch)
    return record


def test_message_records_follow_the_declared_schema():
    assert c.validate_log_record(_message_record())
    assert c.message_record_violations(_message_record(body_sha256='b' * 64))
    assert c.message_record_violations(_message_record(encoding='schema'))
    assert c.message_record_violations(_message_record(delivered_at_sim_s=4.0))
    assert c.message_record_violations(_message_record(delivered_at_sim_s=None))
    assert c.message_record_violations(_message_record(status='sent'))
    # per-recipient delivery must stay consistent with the envelope and the delay
    assert c.message_record_violations(_message_record(delivered_at_sim_s=4.8))
    assert c.message_record_violations(_message_record(delivery_delay_s=5.))
    assert c.message_record_violations(_message_record(
        deliveries=[{'recipient': 'r2', 'delivered_at_sim_s': 4.6, 'status': 'delivered'}]))
    assert c.message_record_violations(_message_record(
        deliveries=[{'recipient': 'r9', 'delivered_at_sim_s': 4.6, 'status': 'delivered'},
                    {'recipient': 'r3', 'delivered_at_sim_s': 4.8, 'status': 'delivered'}]))
    assert c.message_record_violations(_message_record(
        deliveries=[{'recipient': 'r2', 'delivered_at_sim_s': 4.0, 'status': 'delivered'},
                    {'recipient': 'r3', 'delivered_at_sim_s': 4.8, 'status': 'delivered'}]))


def test_action_records_follow_the_declared_schema():
    record = {'schema': c.ACTION_LOG_SCHEMA, 'run_id': 'run-1', 'condition': 'peer_ko', 'seed': 11,
              'actor': 'r2', 'action_id': 'act-1', 'request_id': 'req-1', 'submitted_at_sim_s': 13.7,
              'kind': 'goto', 'arguments': {'target_ref': 'P1-2'}, 'order_id': 'order-1',
              'role': 'end_neg', 'accepted': True, 'rejected_reason': None,
              'local_state': 'command_issued'}
    assert c.validate_log_record(record)
    assert c.action_record_violations({**record, 'kind': 'teleport'})
    assert c.action_record_violations({**record, 'local_state': 'placed'})
    assert c.action_record_violations({**record, 'arguments': {'object_pose': [1., 2., 0.]}})
    with pytest.raises(c.ContractViolation):
        c.validate_log_record({'schema': 'ugrp.something.v1'})


def test_log_schemas_are_documented_for_packages_d_and_i():
    assert set(c.LOG_SCHEMAS) == {c.CALL_LOG_SCHEMA, c.MESSAGE_LOG_SCHEMA, c.ACTION_LOG_SCHEMA}
    for fields in c.LOG_SCHEMAS.values():
        assert all(isinstance(k, str) and k.isascii() and isinstance(v, str) for k, v in fields.items())
    assert 'sim_cost_s' in c.LOG_SCHEMAS[c.CALL_LOG_SCHEMA]
    assert 'delivered_at_sim_s' in c.LOG_SCHEMAS[c.MESSAGE_LOG_SCHEMA]
