"""자기 카메라 폐루프 파생 뷰 생성기와 스냅샷 검증기 회귀 검사.

TensorBoard 변환까지 확인하는 검사는 선택 의존성이 있을 때만 실행한다.
"""
import hashlib
import json
from pathlib import Path

import pytest

from scripts.build_owncam_loop_views import Builder, rename_snapshot, short_name
from scripts.verify_owncam_loop_snapshot import expected_scalars, text_payload, verify


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path: Path, value) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=1) + '\n')
    return path


def raw_run(root: Path, attempt: str, episode: str, *, split, condition, seed, passed, outcome,
            declared=True, gt=0.02, p90=0.03, wall_contacts=0, teacher=None):
    directory = root / attempt / episode
    gates = {
        'R1_exit_waypoint': {'pass': gt <= 0.10 and declared, 'gt_distance_m': gt,
                             'declared_arrival': declared},
        'R2_no_wall_contact': {'pass': wall_contacts == 0, 'wall_contacts': wall_contacts},
        'R3_estimate_near_door': {'pass': p90 < 0.06, 'p90_pos_m': p90, 'p90_yaw_deg': 1.25,
                                  'frames': 240, 'uninitialized': 0},
    }
    if condition == 'box':
        gates['R4_box_held'] = {'pass': True, 'min_box_z_m': 0.085, 'end_box_z_m': 0.16}
    result = {'schema': 'ugrp.owncam_loop_run.v1', 'episode': episode, 'split': split,
              'condition': condition, 'robot_id': 'r3', 'outcome': outcome, 'episode_pass': passed,
              'gates': gates, 'student_sim_s': 85.1, 'teacher_sim_s': teacher, 'looks': 5,
              'look_reasons': ['uncertain', 'travel', 'refix', 'no_tag', 'arrival_check'],
              'student_commands': 1151, 'student_frames': 426, 'student_tag_visibility': 0.878,
              'contacts_student': {'wall': wall_contacts, 'peer_robot': 0, 'other_box': 0},
              'final_gt': [2.64, 0.06, 0.036], 'goal_m': [2.6500000000000004, 0.05]}
    manifest = {'schema': 'ugrp.owncam_loop_run.v1',
                'spec': {'episode_id': episode, 'split': split, 'condition': condition,
                         'map': 'zone_wide_door_tags_v2', 'seed': seed, 'spawn_y': -0.85,
                         'contact_profile': 'local_contact_fine', 'box_kind': 'cyan'},
                'code': {'sha': 'f' * 40, 'dirty': False},
                'static_map_sha256': 'a' * 64, 'landmarks_sha256': 'b' * 64,
                'base_static_map_sha256': 'c' * 64, 'scene_xml_sha256': 'd' * 64,
                'calibration_sha256': 'e' * 64, 'keepouts_sha256': '0' * 64,
                'weld': 'off', 'contact_profile': 'local_contact_fine', 'timestep_s': 0.00025,
                'frame_period_s': 0.2, 'control_period_s': 0.1, 'sync_sim': True,
                'env': {'python': '3.12.13', 'platform': 'macOS-27.2-arm64-arm-64bit',
                        'mujoco': '3.12.0', 'opencv': '5.0.0', 'numpy': '2.5.2',
                        'threads': {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1',
                                    'VECLIB_MAXIMUM_THREADS': '1', 'MKL_NUM_THREADS': '1'}},
                'load_average': {'start': [1.0, 1.1, 1.2], 'end': [1.3, 1.4, 1.5]}, 'wall_s': 183.4}
    write(directory / 'result.json', result)
    write(directory / 'manifest.json', manifest)
    return result, manifest


@pytest.fixture
def cohort(tmp_path):
    """dev 시도 1회 + 사전 등록 test 1회의 최소 코호트."""
    raw = tmp_path / 'raw'
    records = tmp_path / 'records'
    specs = [('dev-a1', 'dev-box-s31', 'dev', 'box', 31, False, 'arrived', 0.054, 0.071, 20.0),
             ('test', 'test-nobox-s41', 'test', 'nobox', 41, True, 'arrived', 0.022, 0.048, None)]
    rows, index = [], {}
    for attempt, episode, split, condition, seed, passed, outcome, gt, p90, teacher in specs:
        result, manifest = raw_run(raw, attempt, episode, split=split, condition=condition, seed=seed,
                                   passed=passed, outcome=outcome, gt=gt, p90=p90, teacher=teacher)
        directory = raw / attempt / episode
        index[f'{attempt}/{episode}'] = {
            'files': 2, 'bytes': 0,
            'result_sha256': sha256(directory / 'result.json'),
            'manifest_sha256': sha256(directory / 'manifest.json')}
        rows.append({'attempt': attempt, 'episode': episode, 'split': split, 'condition': condition,
                     'code_sha': manifest['code']['sha'], 'code_dirty': False, 'outcome': outcome,
                     'pass': passed,
                     'R1_gt_distance_m': result['gates']['R1_exit_waypoint']['gt_distance_m'],
                     'R1_declared': True, 'R2_wall_contacts': 0,
                     'R3_p90_pos_m': result['gates']['R3_estimate_near_door']['p90_pos_m'],
                     'R3_p90_yaw_deg': 1.25, 'R3_frames': 240, 'R3_uninitialized': 0,
                     'student_sim_s': 85.1, 'teacher_sim_s': teacher, 'looks': 5,
                     'look_reasons': result['look_reasons'], 'student_commands': 1151,
                     'student_frames': 426, 'tag_visibility': 0.878,
                     'contacts_student': result['contacts_student'],
                     'load_average': manifest['load_average'], 'wall_s': 183.4,
                     'calibration_sha256': 'e' * 64, 'static_map_sha256': 'a' * 64,
                     'scene_xml_sha256': 'd' * 64, 'weld': 'off'})
    write(records / 'results.json', {'schema': 'ugrp.owncam_loop_results.v1',
                                     'attempts': {'dev-a1': 'dev attempt 1', 'test': 'test split'},
                                     'summary_k_of_n': {'test/nobox': {'n': 1, 'pass': 1}},
                                     'runs': rows})
    write(records / 'raw_index.json', {'root': str(raw), 'storage': 'local only', 'runs': index})
    write(records / 'prereg.json', {
        'schema': 'ugrp.owncam_loop_prereg.v1', 'question': 'door crossing on own camera only',
        'gates': {'R1_exit_waypoint': 'GT distance to W <= 0.10 m',
                  'R2_no_wall_contact': 'no wall contact', 'R3_estimate_near_door': 'p90 < 0.06 m',
                  'R4_box_held': 'box z > 0.045 m', 'episode_pass': 'R1 and R2 and R3 (and R4)'},
        'teacher_part': 'box only: GT teacher grasps and lifts; reported separately',
        'episodes': [{'episode_id': episode, 'split': split, 'condition': condition, 'seed': seed}
                     for _, episode, split, condition, seed, *_ in specs]})
    write(records / 'prereg_amendments.json', {
        'schema': 'ugrp.prereg_amendments.v1', 'prereg_sha256': sha256(records / 'prereg.json'),
        'amendments': [{'date': '2026-09-26', 'commit': 'a5559e7', 'field': 'calibration',
                        'why': 'dev R3 failure', 'was': 'v1', 'now': 'loop'}]})
    write(records / 'frozen_source.json', {'schema': 'ugrp.frozen_source.v1',
                                           'code_commit': 'f' * 40})
    return records, raw


def build(cohort, tmp_path, name='views'):
    records, raw = cohort
    output = tmp_path / name
    output.mkdir()
    summary = Builder(records, raw, output).build()
    return output, summary


def test_views_carry_recorded_numbers_cohorts_and_original_hashes(cohort, tmp_path):
    output, summary = build(cohort, tmp_path)
    records, raw = cohort
    assert summary['views'] == 2
    assert summary['cohorts'] == ['dev-amended', 'test-preregistered']
    dev = json.loads((output / 'dev-a1-box-s31' / 'result.json').read_text())
    test = json.loads((output / 'test-nobox-s41' / 'result.json').read_text())
    raw_dev = json.loads((raw / 'dev-a1' / 'dev-box-s31' / 'result.json').read_text())

    assert dev['success'] is False and test['success'] is True
    assert dev['sim_s'] == raw_dev['student_sim_s'] and dev['commands'] == raw_dev['student_commands']
    assert dev['model_calls'] == 0
    assert dev['wall_s'] == 183.4
    assert dev['protocol_complete'] is True
    assert dev['evaluation']['gates'] == raw_dev['gates']
    assert dev['evaluation']['teacher_sim_s'] == 20.0
    assert dev['evaluation']['robot_robot_contact_samples'] == 0
    # 교사 구간은 학생 SIM 시간에 합산하지 않는다.
    assert dev['sim_s'] != raw_dev['student_sim_s'] + dev['evaluation']['teacher_sim_s']
    assert dev['case'] == 'dev|dev-a1|box|dev-amended'
    assert test['case'] == 'test|nobox|test-preregistered'
    assert 'pose_source=own_wrist_fisheye_tags_v2+particle_filter' in dev['policy']
    assert 'dev-amended' in dev['condition'] and 'test-preregistered' in test['condition']
    assert dev['offline_source']['sha256'] == sha256(raw / 'dev-a1' / 'dev-box-s31' / 'result.json')
    assert dev['offline_scalars']['offline/r3_p90_pos_m'] == raw_dev['gates']['R3_estimate_near_door']['p90_pos_m']
    assert 'offline/r4_min_box_z_m' in dev['offline_scalars']
    assert 'offline/r4_min_box_z_m' not in test['offline_scalars']
    assert dev['derived_view_only'] is True

    index = json.loads((output / 'index.json').read_text())
    assert [row['run'] for row in index['rows']] == ['dev-a1-box-s31', 'test-nobox-s41']
    assert index['records']['files']['results.json'] == sha256(records / 'results.json')


def test_short_names_separate_dev_attempts_from_test():
    assert short_name('dev-a4', 'box', 31) == 'dev-a4-box-s31'
    assert short_name('test', 'nobox', 41) == 'test-nobox-s41'


def test_changed_original_is_refused(cohort, tmp_path):
    records, raw = cohort
    target = raw / 'test' / 'test-nobox-s41' / 'result.json'
    payload = json.loads(target.read_text())
    payload['student_commands'] = 9999
    target.write_text(json.dumps(payload))
    with pytest.raises(SystemExit, match='SHA-256'):
        build(cohort, tmp_path, 'changed')


def test_record_table_disagreeing_with_the_original_is_refused(cohort, tmp_path):
    records, raw = cohort
    results = json.loads((records / 'results.json').read_text())
    results['runs'][1]['looks'] = 99
    write(records / 'results.json', results)
    with pytest.raises(SystemExit, match='looks'):
        build(cohort, tmp_path, 'disagreeing')


def test_missing_original_is_refused(cohort, tmp_path):
    records, raw = cohort
    (raw / 'test' / 'test-nobox-s41' / 'result.json').unlink()
    with pytest.raises(SystemExit, match='original missing'):
        build(cohort, tmp_path, 'missing')


def test_rename_keeps_exporter_name_and_adds_cohort(cohort, tmp_path):
    output, _ = build(cohort, tmp_path)
    index = output / 'index.json'
    snapshot = tmp_path / 'snapshot'
    exported = []
    for row in json.loads(index.read_text())['rows']:
        original = 'derived__' + row['run'] + '__abcd1234'
        (snapshot / original).mkdir(parents=True)
        exported.append({'name': original, 'source': row['source'], 'counts': {'scalars': 7}})
    write(snapshot / 'collection.json', {'exported': exported, 'failed': []})

    assert rename_snapshot(snapshot, index) == {'renamed': 2, 'failed': 0}
    collection = json.loads((snapshot / 'collection.json').read_text())
    assert sorted(entry['name'] for entry in collection['exported']) == ['dev-a1-box-s31', 'test-nobox-s41']
    assert all(entry['original_name'].startswith('derived__') for entry in collection['exported'])
    assert all(entry['cohort'] and entry['condition'] for entry in collection['exported'])
    assert (snapshot / 'dev-a1-box-s31').is_dir() and not (snapshot / exported[0]['name']).exists()


def test_rename_refuses_runs_outside_the_index(cohort, tmp_path):
    output, _ = build(cohort, tmp_path)
    snapshot = tmp_path / 'snapshot-unknown'
    (snapshot / 'derived__other__abcd1234').mkdir(parents=True)
    write(snapshot / 'collection.json', {'exported': [
        {'name': 'derived__other__abcd1234', 'source': str(tmp_path / 'elsewhere'), 'counts': {}}],
        'failed': []})
    with pytest.raises(SystemExit, match='not in the derived-view index'):
        rename_snapshot(snapshot, output / 'index.json')


def test_expected_scalars_read_only_recorded_fields():
    raw = {'student_sim_s': 85.1, 'student_commands': 1151, 'episode_pass': True,
           'contacts_student': {'wall': 0, 'peer_robot': 0, 'other_box': 0},
           'gates': {'R1_exit_waypoint': {'declared_arrival': True}}}
    values = expected_scalars(raw, {'wall_s': 183.4})
    assert values['result/sim_s'] == 85.1 and values['result/commands'] == 1151.0
    assert values['evaluation/reported_success'] == 1.0 and values['result/model_calls'] == 0.0
    assert values['claims/protocol_complete'] == 1.0


def test_text_payload_unwraps_escaped_card():
    assert text_payload('<pre>{&quot;a&quot;: 1}</pre>') == {'a': 1}


def test_snapshot_roundtrip_matches_the_originals(cohort, tmp_path):
    pytest.importorskip('tensorboard')
    from scripts.tensorboard_tools.export import convert

    output, _ = build(cohort, tmp_path)
    records, raw = cohort
    snapshot = tmp_path / 'snapshot-export'
    snapshot.mkdir()
    exported = []
    for row in json.loads((output / 'index.json').read_text())['rows']:
        name = 'derived__' + row['run'] + '__abcd1234'
        manifest = convert(Path(row['source']), snapshot / name, max_images=0)
        assert manifest['complete'] is True and not manifest['warnings']
        exported.append({'name': name, 'source': row['source'], 'counts': manifest['counts']})
    write(snapshot / 'collection.json', {'exported': exported, 'failed': []})
    rename_snapshot(snapshot, output / 'index.json')

    report = verify(snapshot, output / 'index.json', raw, None)
    assert report['mismatches'] == []
    assert report['totals'] == {'runs': 2, 'scalars_compared': 14, 'text_field_checks': 18,
                                'mismatches': 0}
    dev = next(run for run in report['runs'] if run['run'] == 'dev-a1-box-s31')
    assert dev['scalars']['evaluation/reported_success'] == 0.0
    assert dev['scalars']['result/sim_s'] == pytest.approx(85.1)
    assert dev['hparams']['case'] == 'dev|dev-a1|box|dev-amended'
    assert dev['export_complete'] is True


def test_verification_reports_a_tampered_event_value(cohort, tmp_path):
    pytest.importorskip('tensorboard')
    from scripts.tensorboard_tools.export import convert

    output, _ = build(cohort, tmp_path)
    records, raw = cohort
    snapshot = tmp_path / 'snapshot-tampered'
    snapshot.mkdir()
    exported = []
    for row in json.loads((output / 'index.json').read_text())['rows']:
        name = row['run']
        convert(Path(row['source']), snapshot / name, max_images=0)
        exported.append({'name': name, 'source': row['source'], 'counts': {}})
    write(snapshot / 'collection.json', {'exported': exported, 'failed': []})
    # 원본 기록이 바뀌면 이벤트 값과 어긋나고, 그 사실이 보고돼야 한다.
    target = raw / 'test' / 'test-nobox-s41' / 'result.json'
    payload = json.loads(target.read_text())
    payload['student_commands'] = 1
    target.write_text(json.dumps(payload))

    report = verify(snapshot, output / 'index.json', raw, None)
    problems = {mismatch.get('problem') for mismatch in report['mismatches']}
    assert 'original result.json changed since build' in problems
    assert 'value differs from original' in problems
