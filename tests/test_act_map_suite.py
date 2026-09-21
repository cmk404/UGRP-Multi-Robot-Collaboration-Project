import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from sim.act_map_suite import (load_suite, generate_case, manifest, student_task, scene_config,
                               validate_splits, layout_digest, geometry_check, load_prepared_map,
                               require_frozen_protocol)
from sim.act_map_suite import camera_coverage
from sim.research_dispatch_arena import build_scene_xml, digest, FIXED_TOP
from harness.pair_navigation import footprint_clear, swept_clear


def test_regeneration_and_split_identity_are_independent_of_physics_seed():
    spec, cases = load_suite()
    assert len(cases) == 22
    assert cases == load_suite()[1]
    assert manifest(spec, cases) == manifest(*load_suite())
    assert len({layout_digest(c['map']) for c in cases}) == 22
    a, b = scene_config(cases[0], physics_seed=11), scene_config(cases[0], physics_seed=29)
    assert a['static_map'] == b['static_map']
    assert a['seed'] != b['seed']
    for case in cases:
        assert case['map']['top_camera'] == FIXED_TOP
        if case['split'] == 'regression':
            original = Path(__file__).resolve().parents[1]/case['parameters']['source']
            assert json.loads(original.read_text()) == case['map']


def test_student_projection_does_not_include_setup_holdout_or_answers():
    _, cases = load_suite()
    case = cases[0]
    expected = student_task(case['map'])
    case['evaluation_only']['live_coordinates'] = [1, 2]
    case['evaluation_only']['teacher_passage'] = 'success'
    case['split'] = 'test_b'
    assert student_task(case['map']) == expected
    task = student_task(case['map'])
    task['static_map']['goal']['center_m'][0] = 999
    assert student_task(case['map']) == expected
    raw = json.dumps(expected)
    for forbidden in ('map_seed', 'test_b', 'nominal_start_m_rad', 'teacher_passage', 'spawns', 'event_schedule', 'live_coordinates'):
        assert forbidden not in raw
    polluted = copy.deepcopy(case['map']); polluted['live_coordinates'] = [1, 2]
    with pytest.raises(ValueError):
        student_task(polluted)
    with pytest.raises(ValueError, match='event scheduling'):
        scene_config(case, event='delayed_message')


@pytest.mark.parametrize('mutation', ['seed', 'layout', 'topology', 'duplicate', 'missing_split'])
def test_split_audit_rejects_leakage_and_relabelled_layouts(mutation):
    _, cases = load_suite()
    dev = next(c for c in cases if c['split'] == 'dev')
    if mutation == 'seed':
        dev['map_seed'] = cases[0]['map_seed']
    if mutation == 'layout':
        dev['map'] = copy.deepcopy(cases[0]['map'])
        dev['map']['map_id'] = 'renamed'
        dev['map']['goal']['center_m'][0] += .01
    if mutation == 'topology':
        next(c for c in cases if c['split'] == 'test_b')['topology_id'] = cases[0]['topology_id']
    if mutation == 'duplicate':
        dev['id'] = cases[0]['id']
    if mutation == 'missing_split':
        cases = [c for c in cases if c['split'] != 'test_a']
    with pytest.raises(ValueError):
        validate_splits(cases)


def test_generated_map_rejects_unsafe_identifiers_and_invalid_seeds():
    spec, _ = load_suite()
    for change in ({'id': '../escape'}, {'map_seed': True}, {'map_seed': -1}, {'goal_yaw_deg': float('nan')}):
        with pytest.raises(ValueError):
            generate_case({**spec['cases'][0], **change})


def test_exported_loader_verifies_map_and_task_hashes(tmp_path):
    spec, cases = load_suite()
    folder = tmp_path/cases[0]['id']; folder.mkdir()
    (tmp_path/'manifest.json').write_text(json.dumps(manifest(spec, cases)))
    (folder/'map.json').write_text(json.dumps(cases[0]['map']))
    (folder/'student-task.json').write_text(json.dumps(student_task(cases[0]['map'])))
    assert load_prepared_map(folder)[0] == cases[0]['map']
    task = student_task(cases[0]['map']); task['secret_state'] = [1, 2]
    (folder/'student-task.json').write_text(json.dumps(task))
    with pytest.raises(ValueError, match='hash mismatch'):
        load_prepared_map(folder)
    (folder/'student-task.json').write_text(json.dumps(student_task(cases[0]['map'])))
    changed = copy.deepcopy(cases[0]['map']); changed['goal']['center_m'][0] += .01
    (folder/'map.json').write_text(json.dumps(changed))
    with pytest.raises(ValueError, match='hash mismatch'):
        load_prepared_map(folder)


def test_full_load_controls_differ_from_a_point_robot_and_endpoint_only_rotation():
    _, cases = load_suite()
    narrow = next(c for c in cases if c['family'] == 'too_narrow')
    check = geometry_check(narrow)
    assert check['classification'] == 'geometric_impossible_control'
    assert check['start_clear'] and check['goal_clear'] and not check['route_found']
    door = narrow['parameters']['doors'][0]
    # The doorway is wider than an unloaded chassis; it still excludes the load.
    assert .32 < door['width_m'] < check['minimum_loaded_width_m']
    data = copy.deepcopy(cases[0]['map'])
    data['obstacles'] = [{'id':'rotation_corner','center_m':[.93,-2.38],
                          'half_extents_m':[.03,.03],'height_m':.16}]
    a, b = [.55,-2,0], [.55,-2,math.pi/2]
    assert footprint_clear(a, data) and footprint_clear(b, data)
    assert not swept_clear(a, b, data)


def test_dispatch_adapter_matches_shapes_and_preserves_cameras_appearance_weld_off():
    source = '<mujoco><worldbody><geom name="floor"/><camera name="cctv_top"/>'
    source += '<body name="team_beam"><geom name="beam" size=".01 .25 .01"/></body>'
    source += ''.join(f'<body name="{r}__robot"><camera name="{r}__robot_cam" pos="1 2 3"/></body>' for r in ('r1','r2','r3'))
    source += '</worldbody><equality><weld name="test" active="true"/></equality></mujoco>'
    _, cases = load_suite()
    robot_hashes = set(); beam_hashes = set()
    for case in cases:
        config = scene_config(case)
        xml, record = build_scene_xml(source, config)
        tree = ET.fromstring(xml)
        assert tree.find('.//weld').get('active') == 'false'
        assert record['static_map_sha256'] == digest(config['static_map'])
        assert config['source_pair_map_sha256'] == digest(case['map'])
        for box in case['map']['obstacles']:
            geom = tree.find(f'.//geom[@name="dispatch_{box["id"]}"]')
            assert list(map(float, geom.get('pos').split())) == [*box['center_m'], box['height_m']/2]
            assert list(map(float, geom.get('size').split())) == [*box['half_extents_m'], box['height_m']/2]
            assert geom.get('contype') == '1'
        assert tree.find('.//geom[@name="dispatch_local_goal_beam"]').get('contype') == '0'
        robot_hashes.add(digest(record['robot_xml_sha256'])); beam_hashes.add(record['beam_xml_sha256'])
    assert len(robot_hashes) == len(beam_hashes) == 1


def test_new_route_cannot_silently_enter_old_act_or_dispatch_contract():
    from harness.pair_carry_act_contract import context
    from harness.dispatch_plan import fixture_plan, validate_dispatch_plan
    with pytest.raises(ValueError, match='route'):
        context([1, -2], 'double_door', 'r1', [0, 0, 0])
    plan = fixture_plan(); plan['tasks'][0]['route'] = 'double_door'
    with pytest.raises(ValueError, match='route'):
        validate_dispatch_plan(plan)


def test_new_instances_fit_fixed_top_and_legacy_clipping_is_explicit():
    _, cases = load_suite()
    for case in cases:
        visible = all(r['inside_top_frustum'] for r in camera_coverage(scene_config(case)))
        assert visible == (case['split'] != 'regression')
    # An otherwise valid map cannot hide an elevated wall outside the frustum.
    config = scene_config(cases[0])
    config['static_map']['obstacles'].append({'id':'high_wall','center_m':[-1,-2],
                                             'half_extents_m':[.01,.1],'height_m':.5})
    assert not camera_coverage(config)[-1]['inside_top_frustum']


def final_protocol():
    return {'status': 'frozen_final', 'map_manifest_sha256': 'a'*64, 'source_sha': 'b'*40,
            'policy_hashes': {'candidate': 'c'*64}, 'model_training_seeds': [18,19],
            'physics_initialization_seeds': [11,29], 'max_sim_seconds': 600,
            'max_actions': 3000, 'max_api_cost_usd': 0., 'success_tolerance_drop': 0.,
            'required_time_improvement': .1, 'required_cost_improvement': 0.,
            'collision_rule': 'fixture rule', 'drop_rule': 'fixture rule', 'sample_period_s': .01}


@pytest.mark.parametrize('change', [{}, {'status':'pilot_design'}, {'max_actions':0},
                                  {'max_api_cost_usd':-1}, {'sample_period_s':float('nan')},
                                  {'policy_hashes':{}}, {'source_sha':'uncommitted'},
                                  {'model_training_seeds':[18,18]}, {'drop_rule':''}])
def test_final_evaluation_gate_rejects_unfrozen_or_invalid_protocol(change):
    if not change:
        with pytest.raises(ValueError):
            require_frozen_protocol({'status':'frozen_final'})
    else:
        with pytest.raises(ValueError):
            require_frozen_protocol({**final_protocol(), **change})
    assert require_frozen_protocol(final_protocol()) == final_protocol()
