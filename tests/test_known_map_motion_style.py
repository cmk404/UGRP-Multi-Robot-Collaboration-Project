import json
from pathlib import Path

import cv2

import pytest

from harness.heading_map_navigation import HeadingMapNavigator
from harness.known_map_navigation import KnownMapNavigator
from scripts.run_known_map_navigation import navigator_class
from scripts.run_known_map_cohort import cases, decision_budget
from scripts.audit_known_map_navigation import audit_run, _inspect_actor_source
from test_known_map_audit import make_run, _rehash


def test_motion_styles_and_actor_boundary():
    assert navigator_class('heading') is HeadingMapNavigator
    assert navigator_class('holonomic') is KnownMapNavigator
    with pytest.raises(ValueError, match='motion_style'):
        navigator_class('guessed')
    assert _inspect_actor_source('heading')['forbidden_runtime_imports'] == []


def test_comparison_pairs_exact_same_private_start_and_map():
    jobs = cases('heading-comparison')
    assert len(jobs) == 12
    for first, second in zip(jobs[::2], jobs[1::2]):
        assert first['case'] == second['case']
        assert first['map'] == second['map']
        assert first['condition'] == second['condition'] == 'map'
        assert (first['motion_style'], second['motion_style']) == ('heading', 'holonomic')
    assert decision_budget('heading-comparison') == 400
    assert decision_budget('heldout') == decision_budget('development') == 240
    assert decision_budget('heading-comparison', 100) == 100
    with pytest.raises(ValueError, match='max_steps'):
        decision_budget('heading-comparison', 401)


def test_old_record_defaults_to_original_controller(tmp_path):
    make_run(tmp_path)
    assert audit_run(tmp_path)['motion_style'] == 'holonomic'


def test_heading_record_uses_matching_replay_controller(tmp_path):
    make_run(tmp_path, 'heading')
    assert audit_run(tmp_path)['motion_style'] == 'heading'


def test_audit_rejects_changed_style_even_with_new_manifest(tmp_path):
    make_run(tmp_path)
    p = tmp_path/'run.json'
    data = json.loads(p.read_text()); data['motion_style'] = 'heading'
    p.write_text(json.dumps(data)); _rehash(tmp_path)
    with pytest.raises(ValueError, match='motion style mismatch'):
        audit_run(tmp_path)


def test_real_rotation_frames_with_spinning_roller_appearance():
    from harness.heading_map_navigation import _estimate_patch_rotation_deg
    from sim.authored_navigation_map import load_map
    root = Path(__file__).resolve().parents[1]
    actor = HeadingMapNavigator(load_map(root/'maps/navigation/slalom.json'), 'r1')
    patches = []
    for i in (7, 8, 9):
        image = cv2.imread(str(root/f'tests/fixtures/known_map/heading-turn-{i:04d}.jpg'))
        position, info = actor._localize(image)
        assert info['ok']
        actor._position = position
        patches.append(actor._patch(image, position))
    for before, after in zip(patches, patches[1:]):
        angle, observation = _estimate_patch_rotation_deg(before, after, 1)
        assert observation['ok']
        assert 3 <= angle <= 10
