"""Order sheet, static map projection/schematic and the per-call input builder.

Offline only: the inputs are a function of the scenario config and the map FILE,
so these tests never create a simulator, open MuJoCo or call a model.
"""
import copy
import json
import subprocess
import sys
from pathlib import Path

import pytest

from harness import zone_map_schematic as ms
from harness import zone_study_contract as c
from harness import zone_study_inputs as si

ROOT = Path(__file__).resolve().parents[1]
MAP_ID = 'zone_wide_two_doors_tags_v1'
FRAME_SHA = 'b' * 64
PLAIN_MAP_ID = 'zone_wide_two_doors'


def scenario():
    return {'schema': si.SCENARIO_SCHEMA, 'scenario_id': 'S1_normal_mixed', 'map_id': MAP_ID,
            'seeds': [11, 12, 13],
            'orders': [
                {'order_id': 'order-1', 'kind': 'long_beam', 'count': 1, 'item_ids': ['long_beam-1'],
                 'required_robots': 2, 'destination_zone': 'A', 'identity': 'specific_item',
                 'initial_location': {'pickup_bay': 'P1', 'slot': 'P1-2'}},
                {'order_id': 'order-2', 'kind': 'red', 'count': 2, 'destination_zone': 'B',
                 'initial_location': {'pickup_bay': 'P2', 'slot': 'P2-3'}},
                {'order_id': 'order-3', 'kind': 'tri_frame', 'count': 1, 'required_robots': 3,
                 'destination_zone': 'C', 'initial_location': {'pickup_bay': 'P2'}}],
            'eval': {'hidden_events': [{'at_sim_s': 60., 'kind': 'item_moved', 'item': 'long_beam-1',
                                        'to': {'pickup_bay': 'P1', 'slot': 'P1-1'}}],
                     'notes': 'evaluation-only schedule'}}


@pytest.fixture(scope='module')
def bundle():
    return ms.map_bundle(MAP_ID)


@pytest.fixture()
def source(bundle):
    return si.OrderSheetSource(scenario(), bundle)


# ---------------------------------------------------------------------------
# Static map projection and schematic

def test_map_bundle_pins_the_map_file_the_projection_and_the_schematic(bundle):
    assert bundle['schema'] == ms.MAP_BUNDLE_SCHEMA and bundle['map_id'] == MAP_ID
    assert bundle['map_file'] == f'maps/zones/{MAP_ID}.json'
    raw = (ROOT / 'maps' / 'zones' / f'{MAP_ID}.json').read_bytes()
    assert bundle['map_file_sha256'] == ms.digest_bytes(raw)
    assert bundle['public_map_sha256'] == ms.digest(bundle['public_map'])
    assert bundle['base_map']['static_map_sha256'] == json.loads(raw)['base_map']['static_map_sha256']
    assert bundle['has_landmarks'] is True
    assert bundle['schematic']['png_sha256'] and bundle['schematic']['pixels_sha256']
    again = ms.map_bundle(MAP_ID)
    assert again['public_map_sha256'] == bundle['public_map_sha256']
    assert again['schematic']['png_sha256'] == bundle['schematic']['png_sha256']


def test_public_map_carries_static_structure_and_no_evaluation_data(bundle):
    public = bundle['public_map']
    assert 'top_cameras' not in public and not c.forbidden_key_hits(public)
    assert not c.non_ascii_keys(public)
    assert sorted(public['regions']) == ['A', 'B', 'C', 'pickup']
    assert [w['id'] for w in public['walls'] if not w['perimeter']] == ['wall_divider_1', 'wall_divider_2']
    assert [p['id'] for p in public['passages']] == ['door_narrow', 'door_wide']
    assert [p['width_m'] for p in public['passages']] == [.5, 1.]
    assert [s['slot_id'] for s in public['zone_slots']['A']] == ['A1', 'A2', 'A3']
    assert [b['bay_id'] for b in public['pickup_bays']] == ['P1', 'P2']
    assert [s['slot_id'] for b in public['pickup_bays'] for s in b['slots']] == \
        ['P1-1', 'P1-2', 'P1-3', 'P2-1', 'P2-2', 'P2-3']
    assert public['landmarks']['tag_count'] == len(public['landmarks']['tags']) > 0
    assert public['landmarks']['family'] == 'tag36h11'


def test_pickup_bays_are_derived_from_the_pickup_region_only(bundle):
    data, _ = ms.load_map(MAP_ID)
    region = data['regions']['pickup']
    bays = ms.pickup_bays(data)
    assert len(bays) == ms.PICKUP_BAY_COLUMNS and len(bays[0]['slots']) == ms.PICKUP_BAY_ROWS
    xs = [b['center_m'][0] for b in bays]
    assert xs == sorted(xs)                                     # west to east
    ys = [s['center_m'][1] for s in bays[0]['slots']]
    assert ys == sorted(ys)                                     # south to north
    assert bays[0]['center_m'][0] - bays[0]['half_extents_m'][0] == \
        pytest.approx(region['center_m'][0] - region['half_extents_m'][0])
    assert ms.pickup_bays(data) == bays


def test_landmark_detail_shrinks_the_projection_without_changing_the_map(bundle):
    summary = ms.map_bundle(MAP_ID, landmark_detail='summary', schematic=False)
    none = ms.map_bundle(MAP_ID, landmark_detail='none', schematic=False)
    assert summary['map_file_sha256'] == bundle['map_file_sha256'] == none['map_file_sha256']
    sizes = [len(json.dumps(m['public_map'])) for m in (none, summary, bundle)]
    assert sizes[0] < sizes[1] < sizes[2]
    assert 'landmarks' not in none['public_map']
    assert 'tags' not in summary['public_map']['landmarks']
    assert summary['public_map']['landmarks']['tag_ids']


def test_schematic_is_deterministic_and_map_specific(tmp_path):
    data, _ = ms.load_map(MAP_ID)
    first, meta = ms.render_schematic(data, path=tmp_path / 'plan.png')
    second, meta2 = ms.render_schematic(json.loads((ROOT / 'maps' / 'zones' / f'{MAP_ID}.json').read_text()))
    assert first[:8] == b'\x89PNG\r\n\x1a\n'
    assert first == second and meta['png_sha256'] == meta2['png_sha256']
    assert (tmp_path / 'plan.png').read_bytes() == first
    assert meta['renderer']['library'] == 'PIL' and meta['px_per_m'] > 0
    other, _ = ms.render_schematic(ms.load_map(PLAIN_MAP_ID)[0])
    assert ms.digest_bytes(other) != meta['png_sha256']
    plain = ms.map_bundle(PLAIN_MAP_ID, schematic=False)
    assert plain['has_landmarks'] is False and plain['base_map'] is None


def test_static_map_section_is_verifiable_against_the_map_file(bundle):
    section = si.static_map_for_call(bundle)
    assert ms.verify_static_map_section(section)['ok'] is True
    tampered = copy.deepcopy(section)
    tampered['public_map']['passages'][0]['width_m'] = 9.
    report = ms.verify_static_map_section(tampered)
    assert report['ok'] is False and report['public_map_matches'] is False
    assert section['schematic_ref']['ref'] == f'map-{MAP_ID}-schematic'


# ---------------------------------------------------------------------------
# Order sheet

def test_order_sheet_is_built_without_any_simulator_object(bundle):
    code = ('import json, sys;'
            'from harness import zone_map_schematic as ms, zone_study_inputs as si;'
            f'b = ms.map_bundle({MAP_ID!r});'
            'scen = json.loads(sys.argv[1]);'
            'src = si.OrderSheetSource(scen, b);'
            'print(json.dumps({"sha": src.sha256, "png": b["schematic"]["png_sha256"],'
            ' "mujoco": "mujoco" in sys.modules,'
            ' "sim": sorted(m for m in sys.modules if m == "sim" or m.startswith("sim."))}))')
    out = subprocess.run([sys.executable, '-c', code, json.dumps(scenario())], cwd=ROOT,
                         capture_output=True, text=True, check=True)
    result = json.loads(out.stdout.strip().splitlines()[-1])
    assert result['mujoco'] is False and result['sim'] == []
    assert result['sha'] == si.OrderSheetSource(scenario(), bundle).sha256
    assert result['png'] == bundle['schematic']['png_sha256']      # same hash in another process


def test_a_scenario_config_can_come_from_a_json_file(tmp_path, bundle):
    path = tmp_path / 'scenario.json'
    path.write_text(json.dumps(scenario(), ensure_ascii=False))
    loaded = si.load_scenario(path)
    assert si.order_sheet(loaded, bundle) == si.order_sheet(scenario(), bundle)
    assert si.hidden_events(loaded)[0]['kind'] == 'item_moved'


def test_order_sheet_does_not_change_during_a_run(source, bundle):
    sheet = source.sheet()
    assert sheet['schema'] == c.ORDER_SHEET_SCHEMA and len(sheet['orders']) == 3
    sheet['orders'][0]['destination_zone'] = 'C'
    sheet['orders'].append({'order_id': 'order-9'})
    assert source.sheet()['orders'][0]['destination_zone'] == 'A'
    assert len(source.sheet()['orders']) == 3
    assert ms.digest(source.sheet()) == source.sha256
    source.assert_unchanged()
    # the same config after the arena changed (moved item, new hidden event) yields the same sheet
    moved = scenario()
    moved['eval']['hidden_events'].append({'at_sim_s': 90., 'kind': 'item_dropped', 'item': 'long_beam-1'})
    assert si.OrderSheetSource(moved, bundle).sha256 == source.sha256
    tampered = si.OrderSheetSource(scenario(), bundle)
    tampered._sheet['orders'][0]['destination_zone'] = 'C'
    with pytest.raises(c.ContractViolation):
        tampered.assert_unchanged()


def test_order_sheet_holds_only_setup_information(source):
    sheet = source.sheet()
    assert not c.forbidden_key_hits(sheet) and not c.non_ascii_keys(sheet)
    text = json.dumps(sheet, ensure_ascii=False)
    assert 'hidden' not in text and 'item_moved' not in text and 'P1-1' not in text
    order = sheet['orders'][0]
    assert order['initial_location'] == {'pickup_bay': 'P1', 'slot': 'P1-2'}
    assert order['required_robots'] == 2 and order['identity'] == 'specific_item'
    assert sheet['orders'][2]['initial_location'] == {'pickup_bay': 'P2', 'slot': None}
    assert sheet['note_ko'].startswith('주문서와 initial_location')
    assert source.hidden_events()[0]['kind'] == 'item_moved'
    assert si.eval_section(scenario())['notes'] == 'evaluation-only schedule'


@pytest.mark.parametrize('patch, match', [
    ({'kind': 'sofa'}, 'unknown item kind'),
    ({'required_robots': 1}, 'needs 2 robots'),
    ({'destination_zone': 'D'}, 'destination_zone'),
    ({'destination_zone': 'A', 'initial_location': {'pickup_bay': 'P9'}}, 'not in the map'),
    ({'initial_location': {'pickup_bay': 'P1', 'slot': 'P2-1'}}, 'not inside bay'),
    ({'initial_location': {'pickup_bay': 'P1', 'slot': 'P1-9'}}, 'not in the map'),
    ({'count': 3}, 'differs from len'),
    ({'item_ids': None, 'identity': 'specific_item'}, 'needs item_ids'),
    ({'order_id': 'order-2'}, 'duplicate order_id'),
    ({'teacher_receipt': True}, 'evaluation-only'),
    ({'top_frame': 'x'}, 'evaluation-only'),
    ({'unknown_key': 1}, 'unknown order key'),
])
def test_order_validation_rejects_broken_orders(bundle, patch, match):
    config = scenario()
    config['orders'][0].update(patch)
    with pytest.raises(c.ContractViolation, match=match):
        si.order_sheet(config, bundle)


@pytest.mark.parametrize('patch, match', [
    ({'schema': 'ugrp.zone_scenario.v0'}, 'scenario schema'),
    ({'seeds': []}, 'seeds'),
    ({'seeds': [11, 11]}, 'unique'),
    ({'orders': []}, 'orders'),
    ({'map_id': 'zone_open'}, 'differs from the bundle'),
    ({'hidden_events': [{'at_sim_s': 1.}]}, 'unknown scenario key'),
])
def test_scenario_validation_rejects_broken_configs(bundle, patch, match):
    config = scenario()
    config.update(patch)
    with pytest.raises(c.ContractViolation, match=match):
        si.order_sheet(config, bundle)


def test_required_robot_table_matches_the_repository_goal_definition():
    from harness import zone_goal_v2

    assert set(si.ITEM_KINDS) <= set(zone_goal_v2.ALL_KINDS)
    for kind, need in si.REQUIRED_ROBOTS.items():
        assert zone_goal_v2.required_carriers(kind) == need
    assert set(si.ROLE_NAMES) >= {r for kind in si.ITEM_KINDS for r in zone_goal_v2.formation(kind)}


def test_vocabulary_comes_from_the_sheet_and_the_public_map(source, bundle):
    vocabulary = si.vocabulary(source.sheet(), bundle['public_map'])
    assert {'order-1', 'long_beam-1', 'long_beam', 'red'} <= vocabulary.items
    assert vocabulary.passages == frozenset({'door_narrow', 'door_wide'})
    assert {'P1', 'P1-2', 'A1', 'pickup', 'A'} <= vocabulary.location_refs
    assert 'cctv_top' not in vocabulary.location_refs
    assert vocabulary.zones == frozenset(c.ZONE_IDS)


# ---------------------------------------------------------------------------
# Per-call input

def _inbox(condition_name, seed, recipient='r2'):
    if condition_name == 'no_comm':
        return recipient, None
    if condition_name == 'structured':
        body, sender = {'act': 'inform', 'zone': 'A', 'state': 'blocked', 'passage': 'door_narrow'}, 'r3'
    elif condition_name == 'leader_ko':
        body, sender = {'text': 'order-1을 맡아 주세요'}, c.leader_for_seed(condition_name, seed)
        if sender == recipient:
            recipient = next(r for r in c.ROBOTS if r != sender)
    else:
        body, sender = {'text': 'order-2는 제가 갑니다'}, 'r3'
    return recipient, [c.message_envelope(condition_name, 'm-1', sender, [recipient], body,
                                          created_at_sim_s=4.5, seed=seed)]


def _call(source, bundle, condition_name, seed=11, **extra):
    robot, inbox = _inbox(condition_name, seed)
    kwargs = {'robot_id': robot, 'condition_name': condition_name, 'request_id': 'req-1',
              'sim_time_s': 12.5, 'static_map': si.static_map_for_call(bundle), 'source': source,
              'seed': seed, 'own_rgb_refs': [si.own_rgb_ref(robot, 41, 11.5, FRAME_SHA), si.own_rgb_ref(robot, 42, 12.5, FRAME_SHA)],
              'own_command_history': [si.command_entry('cmd-1', 3., 'goto', {'target_ref': 'P1'})],
              'inbox': inbox if 'inbox' in c.condition(condition_name).input_allowlist else None}
    kwargs.update(extra)
    return si.build_call_input(**kwargs)


@pytest.mark.parametrize('condition_name', c.MAIN_CONDITIONS)
def test_every_main_condition_builds_a_payload_that_passes_the_validator(source, bundle, condition_name):
    payload = _call(source, bundle, condition_name)
    assert c.validate_robot_payload(payload, seed=11)
    assert payload['channel']['condition'] == condition_name
    assert payload['order_sheet'] == source.sheet()
    assert payload['static_map']['public_map_sha256'] == bundle['public_map_sha256']
    assert set(payload) <= c.condition(condition_name).input_allowlist
    assert ('inbox' in payload) == (condition_name != 'no_comm')


def test_only_the_channel_differs_between_the_conditions(source, bundle):
    payloads = {name: _call(source, bundle, name) for name in c.MAIN_CONDITIONS}
    shared = ('static_map', 'order_sheet', 'own_rgb_refs', 'own_command_history', 'self_belief')
    for name, payload in payloads.items():
        for key in shared:
            assert payload[key] == payloads['no_comm'][key], f'{name}.{key} differs from no_comm'
    assert payloads['no_comm']['channel']['can_receive_from'] == []
    assert payloads['peer_ko']['channel']['can_receive_from'] == ['r1', 'r3']
    assert payloads['structured']['channel']['encoding'] == 'schema'
    assert payloads['leader_ko']['leader_id'] == 'r3' and payloads['leader_ko']['role'] == 'follower'


def test_the_payload_is_invariant_to_evaluation_only_changes(source, bundle):
    first = _call(source, bundle, 'peer_ko')
    moved = scenario()
    moved['eval']['hidden_events'] = [{'at_sim_s': 5., 'kind': 'item_dropped', 'item': 'long_beam-1'}]
    moved['eval']['notes'] = 'a different hidden schedule'
    second = _call(si.OrderSheetSource(moved, bundle), bundle, 'peer_ko')
    assert si.payload_sha256(first) == si.payload_sha256(second)


def test_the_input_profile_trims_history_the_same_way_in_every_condition(source, bundle):
    robot = 'r2'
    refs = [si.own_rgb_ref(robot, i, i * .5, FRAME_SHA) for i in range(10)]
    history = [si.command_entry(f'cmd-{i}', i * .1, 'goto', {'target_ref': 'P1'}) for i in range(30)]
    for name in c.MAIN_CONDITIONS:
        payload = _call(source, bundle, name, robot_id=robot, own_rgb_refs=refs,
                        own_command_history=history)
        assert len(payload['own_rgb_refs']) == si.INPUT_PROFILE['own_rgb_frames']
        assert payload['own_rgb_refs'][-1]['ref'] == 'own-r2-0009'
        assert len(payload['own_command_history']) == si.INPUT_PROFILE['command_history_entries']
        assert payload['own_command_history'][-1]['command_id'] == 'cmd-29'


def test_the_builder_refuses_inputs_the_condition_does_not_allow(source, bundle):
    with pytest.raises(c.ContractViolation, match='no messages'):
        si.build_call_input(robot_id='r1', condition_name='no_comm', request_id='req-1', sim_time_s=1.,
                            static_map=si.static_map_for_call(bundle), source=source,
                            inbox=[{'message_id': 'm-1'}])
    with pytest.raises(c.ContractViolation, match='seed'):
        si.build_call_input(robot_id='r1', condition_name='leader_ko', request_id='req-1', sim_time_s=1.,
                            static_map=si.static_map_for_call(bundle), source=source)
    with pytest.raises(c.ContractViolation):
        si.command_entry('cmd-1', 1., 'goto', {'teacher_receipt': True})
    with pytest.raises(c.ContractViolation):
        si.own_rgb_ref('r9', 1, 1., FRAME_SHA)
    with pytest.raises(c.ContractViolation):
        si.build_call_input(robot_id='r1', condition_name='no_comm', request_id='req-1', sim_time_s=1.,
                            static_map={'map_id': 'x'}, source=source)


def test_reference_R_builds_a_commander_payload_only(source, bundle):
    payload = si.build_call_input(robot_id=si.COMMANDER, condition_name='reference_R', request_id='req-1',
                                  sim_time_s=3., static_map=si.static_map_for_call(bundle),
                                  source=source,
                                  team_rgb_refs=[si.own_rgb_ref(r, 1, 3., FRAME_SHA) for r in c.ROBOTS],
                                  issued_orders=[{'order_ref': 'order-1', 'to': 'r1'}])
    assert c.validate_robot_payload(payload)
    assert len(payload['team_rgb_refs']) == 3 and 'own_rgb_refs' not in payload
    with pytest.raises(c.ContractViolation):
        si.build_call_input(robot_id='r1', condition_name='reference_R', request_id='req-2', sim_time_s=3.,
                            static_map=si.static_map_for_call(bundle), source=source)


def test_run_provenance_links_the_contract_the_map_and_the_inputs(source, bundle):
    manifest = si.call_input_bundle(map_id=MAP_ID, source=source, condition_name='leader_ko', seed=12,
                                    code_sha='5288933', execution_bundle_id='zone-study-A-1',
                                    model='fixture-model', provider='fixture', cost_profile_id='dev-v1')
    assert manifest['contract']['leader_id'] == 'r1' and manifest['contract']['condition'] == 'leader_ko'
    assert manifest['inputs']['order_sheet_sha256'] == source.sha256
    assert manifest['inputs']['map']['map_file_sha256'] == bundle['map_file_sha256']
    assert manifest['inputs']['hidden_event_count'] == 1
    assert manifest['inputs']['input_profile'] == si.INPUT_PROFILE
    assert manifest['inputs']['leader_rotation'] == {11: 'r3', 12: 'r1', 13: 'r2'}
    assert len(manifest['registry_sha256']) == 64
    assert manifest['provenance']['code_sha'] == '5288933'
    assert manifest['provenance']['order_sheet_sha256'] == source.sha256
    assert manifest['provenance']['map_file_sha256'] == bundle['map_file_sha256']
    assert manifest['provenance']['model'] == 'fixture-model'


def test_seeds_that_would_freeze_the_leader_are_rejected(bundle):
    config = scenario()
    config['seeds'] = [11, 14, 17, 20]
    with pytest.raises(c.ContractViolation, match='every residue'):
        si.order_sheet(config, bundle)
    assert si.leader_rotation([11, 14, 17]) == {11: 'r3', 14: 'r3', 17: 'r3'}
    single = scenario()
    single['seeds'] = [11]
    assert si.order_sheet(single, bundle)                      # a one-seed pilot is allowed


# ---------------------------------------------------------------------------
# Log records

def test_log_record_builders_produce_records_that_pass_the_schema(source, bundle):
    payload = _call(source, bundle, 'peer_ko')
    call = si.call_log_record(run_id='run-1', condition_name='peer_ko', seed=11, actor='r2',
                             request_id='req-1', call_index=0, trigger='message_received',
                             requested_at_sim_s=12.5, released_at_sim_s=16.2,
                             cost_terms={'alpha_s': 1., 'beta_s_per_token': .02,
                                         'gamma_s_per_utterance': .3, 'output_tokens': 120,
                                         'utterances': 1},
                             input_sha256=si.payload_sha256(payload),
                             input_tokens={'text': 5200, 'image': 2, 'cached': 0}, output_tokens=120,
                             status='ok', action_id='act-1', message_ids=['m-2'],
                             decision_sources=['own-r2-0042', 'm-1'],
                             provenance=si.provenance(source=source, code_sha='5288933',
                                                      execution_bundle_id='zone-study-A-1',
                                                      model='fixture-model'))
    assert call['sim_cost_s'] == pytest.approx(3.7) and call['role'] == 'peer'
    envelope = c.message_envelope('peer_ko', 'm-2', 'r2', ['r1'], {'text': 'order-1 제가 갑니다'},
                                  created_at_sim_s=16.2)
    message = si.message_log_record(run_id='run-1', condition_name='peer_ko', seed=11, envelope=envelope,
                                   deliveries=[{'recipient': 'r1', 'delivered_at_sim_s': 16.3,
                                                'status': 'delivered'}],
                                   delivery_delay_s=.1, status='delivered', act='propose')
    assert message['delivered_at_sim_s'] == pytest.approx(16.3)
    assert message['korean_ok'] is True and message['chars'] > 0 and message['act'] == 'propose'
    english = c.message_envelope('peer_ko', 'm-3', 'r2', ['r1'], {'text': 'I take order-1 이제'},
                                 created_at_sim_s=17.)
    slip = si.message_log_record(run_id='run-1', condition_name='peer_ko', seed=11, envelope=english,
                                 deliveries=[{'recipient': 'r1', 'delivered_at_sim_s': 17.1,
                                              'status': 'delivered'}],
                                 delivery_delay_s=.1, status='delivered')
    assert slip['korean_ok'] is True                    # has Korean, language slips are metrics
    structured = c.message_envelope('structured', 'm-4', 'r1', ['r2'], {'act': 'yield', 'zone': 'A'},
                                    created_at_sim_s=20.)
    record = si.message_log_record(run_id='run-1', condition_name='structured', seed=11,
                                   envelope=structured,
                                   deliveries=[{'recipient': 'r2', 'delivered_at_sim_s': None,
                                                'status': 'dropped_budget'}],
                                   delivery_delay_s=.1, status='rejected', rejected_reason='budget')
    assert record['korean_ok'] is None and record['act'] == 'yield' and record['chars'] == 0
    action = si.action_log_record(run_id='run-1', condition_name='peer_ko', seed=11, actor='r2',
                                 action_id='act-1', request_id='req-1', submitted_at_sim_s=16.2,
                                 kind='goto', arguments={'target_ref': 'P1-2'}, accepted=True,
                                 order_id='order-1', role='end_neg')
    assert c.validate_log_record(action)
    with pytest.raises(c.ContractViolation):
        si.call_log_record(run_id='run-1', condition_name='peer_ko', seed=11, actor='r2', request_id='req-1',
                           call_index=0, trigger='start', requested_at_sim_s=1., released_at_sim_s=2.,
                           cost_terms={}, input_sha256='a' * 64, input_tokens={}, output_tokens=1,
                           status='ok', payload_validated=False,
                           provenance=si.provenance(source=source, code_sha='5288933',
                                                    execution_bundle_id='zone-study-A-1',
                                                    model='fixture-model'))
    with pytest.raises(c.ContractViolation):
        si.action_log_record(run_id='run-1', condition_name='peer_ko', seed=11, actor='r2',
                            action_id='act-2', request_id='req-1', submitted_at_sim_s=1., kind='goto',
                            arguments={'object_pose': [1., 2.]}, accepted=True)


# ---------------------------------------------------------------------------
# 2026-09-26 Codex review regressions

def test_the_robot_only_sees_an_opaque_scenario_reference(source):
    """Review finding 17: the descriptive config id (``s5_moved_dropped_item``)
    told the robot the hidden event KIND before it could observe anything."""
    sheet = source.sheet()
    assert sheet['scenario_id'] == c.scenario_ref('S1_normal_mixed') == source.scenario_ref
    assert c.SCENARIO_REF.match(sheet['scenario_id'])
    assert 'normal_mixed' not in json.dumps(sheet, ensure_ascii=False)
    # the mapping back stays evaluation-side
    assert source.manifest()['scenario_id'] == 'S1_normal_mixed'
    assert source.manifest()['scenario_ref'] == sheet['scenario_id']
    assert source.scenario_id == 'S1_normal_mixed'


def test_a_public_scenario_note_is_refused(source):
    """Review finding 17: a public ``notes`` string named the hidden event kind,
    its target and, in s6, the solution."""
    leaky = scenario()
    leaky['notes'] = 'cyan_1은 30초에 P1-3으로 옮겨진다'
    with pytest.raises(c.ContractViolation, match='design_notes_ko'):
        si.validate_scenario(leaky)


def test_pinned_digests_bind_a_payload_to_the_frozen_sources(source, bundle):
    """Review finding 3: the payload is compared with the FROZEN order sheet and
    map bundle, not only with the provider's own recomputed hashes."""
    payload = si.build_call_input(
        robot_id='r2', condition_name='peer_ko', request_id='req_1', sim_time_s=5.0,
        static_map=si.static_map_for_call(bundle), source=source, seed=11,
        own_rgb_refs=[si.own_rgb_ref('r2', 1, 5.0, FRAME_SHA)], own_command_history=[],
        self_belief=si.belief_skeleton(), inbox=[])
    assert c.payload_violations(payload, seed=11, pinned=source.pinned) == []
    tampered = copy.deepcopy(payload)
    tampered['order_sheet']['orders'][0]['destination_zone'] = 'C'
    assert c.payload_violations(tampered, seed=11) == []
    assert c.payload_violations(tampered, seed=11, pinned=source.pinned)
