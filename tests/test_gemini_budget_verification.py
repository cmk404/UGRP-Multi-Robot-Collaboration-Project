"""Synthetic artifact-validation unit tests; NOT simulator/provider evidence."""
import hashlib
import json
from pathlib import Path

import pytest

from scripts import verify_gemini_budget_run as verifier
from scripts.run_gemini_budget_pilot import pilot_command


def write_json(root, name, obj):
    (root / name).write_text(json.dumps(obj))


def write_rows(root, name, rows):
    (root / name).write_text(''.join(json.dumps(row) + '\n' for row in rows))


@pytest.fixture
def artifact(tmp_path, monkeypatch):
    # These artificial rows exist only under pytest's temporary directory.
    # Video decoding is mocked: a passing check is explicitly NOT visual review.
    monkeypatch.setattr(verifier, '_video_ok', lambda path: True)
    config = dict(verifier.EXPECTED)
    write_json(tmp_path, 'run-config.json', config)
    source = {'OFFLINE_UNIT_TEST_FIXTURE.py': 'a' * 64}
    write_json(tmp_path, 'source-manifest.json', source)
    tokens = 1000
    result = {'seed': 41, 'active_robots': ['r1'], 'communication': 'none',
              'model': 'gemini-3.8-flash', 'success': True, 'error': None,
              'source_hash': hashlib.sha256(json.dumps(source, sort_keys=True).encode()).hexdigest(),
              'budgets': {'sim_seconds': 300, 'max_calls_per_robot': 30, 'max_input_tokens_per_robot': 60000},
              'llm_calls': {'r1': 5}, 'input_usage': {'r1': {
                  'reported_prompt_tokens': tokens * 5, 'calls_without_usage': 0, 'unsettled_reservations': 0}},
              'outcomes': {'r1': {'success': True, 'reason': 'VISUAL_RELEASE_CONFIRMED',
                  'gates': {k: True for k in ('lift', 'inside_destination', 'stable', 'no_attachment_constraint', 'visual_release')}}},
              'decision_elapsed_sim_s': 10.0, 'passive_settle_s': 1.0, 'episode_stop_sim_time': 10.0}
    write_json(tmp_path, 'result.json', result)
    rows = []
    commands = []
    for i, kind in enumerate(('approach', 'pick', 'drive', 'release', 'finish')):
        call_id = f'r1-call-{i}'
        images = []; frames = []
        for camera, label in (('wrist', 'CURRENT_WRIST'), ('nav', 'CURRENT_NAV')):
            filename = f'{i}-{camera}.jpg'
            raw = f'OFFLINE_TEST_BYTES_{i}_{camera}'.encode()
            (tmp_path / filename).write_bytes(raw)
            digest = hashlib.sha256(raw).hexdigest()
            images.append({'camera': camera, 'path': filename, 'sha256': digest})
            frames.append({'label': label, 'sha256': digest, 'transmitted_sha256': digest})
        feedback = ({'last_macro': {'decision_id': commands[-1]['decision_id'],
                                   **commands[-1]['execution']}} if commands else {})
        rows.append({'event': 'llm_request', 'robot_id': 'r1', 'call_id': call_id, 'images': images,
                     'time': i * 2, 'execution_feedback': feedback})
        rows.append({'event': 'llm_result', 'robot_id': 'r1', 'call_id': call_id,
                     'decision': {'action': {'kind': kind}}, 'disposition': 'accepted',
                     'audit': {'response_model': 'gemini-3.8-flash', 'raw_text': 'OFFLINE_FIXTURE',
                               'usage': {'prompt_tokens': tokens}, 'input_frames': frames,
                               'model_context': {'execution_feedback': feedback}}})
        commands.append({'event': 'macro_finished', 'time': i * 2 + 1, 'robot_id': 'r1',
                         'decision_id': call_id + '-decision',
                         'execution': {'macro': {'kind': kind}, 'status': 'completed',
                                       'elapsed_drive_control_s': 1 if kind == 'drive' else 0,
                                       'motion_confirmed': False}})
    write_rows(tmp_path, 'llm-decisions.jsonl', rows)
    write_rows(tmp_path, 'commands.jsonl', commands)
    return tmp_path, result, rows


def test_consistent_synthetic_artifacts_still_require_real_visual_review(artifact):
    root, _, _ = artifact
    report = verifier.verify_run(root)
    assert report['automated_checks_passed'], report
    assert report['visual_review_completed'] is False
    assert report['status'] == 'AUTOMATED_CHECKS_PASS_VISUAL_REVIEW_REQUIRED'


def test_missing_evidence_cannot_be_called_success(tmp_path):
    assert not verifier.verify_run(tmp_path)['automated_checks_passed']


@pytest.mark.parametrize('corruption', [None, 'missing', 'mixed'])
def test_explicit_reasoning_setting_matches_every_submitted_call(artifact, corruption):
    root, result, rows = artifact
    result['reasoning_effort'] = 'medium'
    for row in rows:
        if row['event'] == 'llm_request':
            row['requested_reasoning_effort'] = 'medium'
    if corruption == 'missing':
        rows[0].pop('requested_reasoning_effort')
    elif corruption == 'mixed':
        rows[0]['requested_reasoning_effort'] = 'none'
    write_json(root, 'result.json', result)
    write_rows(root, 'llm-decisions.jsonl', rows)
    report = verifier.verify_run(root)
    assert report['checks']['requested_reasoning_effort_consistent'] is (corruption is None)


@pytest.mark.parametrize('corruption', [None, 'foreign', 'current', 'unknown_label', 'metadata'])
@pytest.mark.parametrize('label', ['RECENT_BLOCKED_NAV', 'LAST_TRANSLATION_NAV'])
def test_historical_image_provenance(artifact, corruption, label):
    root, _, rows = artifact
    first_nav = rows[0]['images'][1]['sha256']
    current_nav = rows[2]['images'][1]['sha256']
    digest = {'foreign': 'f' * 64, 'current': current_nav}.get(corruption, first_nav)
    frame = {'camera': 'nav_cam', 'label': label,
             'sha256': digest, 'transmitted_sha256': digest, 'frame_id': 1, 'sim_time': 0}
    if corruption == 'unknown_label':
        frame['label'] = 'UNTRUSTED_REFERENCE'
    audit = rows[3]['audit']
    audit['input_frames'].append(frame)
    audit['model_context']['navigation_reference'] = {
        'sha256': digest, 'frame_id': 99 if corruption == 'metadata' else 1, 'sim_time': 0}
    write_rows(root, 'llm-decisions.jsonl', rows)
    report = verifier.verify_run(root)
    assert report['checks']['historical_rgb_from_prior_own_nav'] is (corruption is None)


@pytest.mark.parametrize('target', ['request', 'model_context'])
def test_logged_execution_must_reach_actual_model_context(artifact, target):
    root, _, rows = artifact
    for row in rows:
        if target == 'request' and row['event'] == 'llm_request':
            row['execution_feedback'] = {}
        elif target == 'model_context' and row['event'] == 'llm_result':
            row['audit']['model_context']['execution_feedback'] = {}
    write_rows(root, 'llm-decisions.jsonl', rows)
    assert not verifier.verify_run(root)['checks']['execution_results_reach_model_context']


def test_original_61898_token_failure_is_not_made_success_by_summary_boolean(artifact):
    root, result, rows = artifact
    for row in rows:
        if row['event'] == 'llm_result': row['audit']['usage']['prompt_tokens'] = 12379
    rows[-1]['audit']['usage']['prompt_tokens'] = 12382
    result['input_usage']['r1']['reported_prompt_tokens'] = 61898
    write_json(root, 'result.json', result); write_rows(root, 'llm-decisions.jsonl', rows)
    assert not verifier.verify_run(root)['checks']['known_reconciled_input_within_60000']


def test_larger_budget_is_rejected_even_when_usage_would_fit_original(artifact):
    root, result, _ = artifact
    result['budgets']['max_input_tokens_per_robot'] = 120000
    write_json(root, 'result.json', result)
    assert not verifier.verify_run(root)['checks']['reported_fixed_budgets']


def test_explicit_120000_limit_verifies_matching_config_report_and_usage(artifact):
    root, result, rows = artifact
    config = dict(verifier.EXPECTED)
    config['max_input_tokens'] = 120000
    result['budgets']['max_input_tokens_per_robot'] = 120000
    per_call = (13000, 13000, 13000, 13000, 13000)
    for row, tokens in zip((r for r in rows if r['event'] == 'llm_result'), per_call):
        row['audit']['usage']['prompt_tokens'] = tokens
    result['input_usage']['r1']['reported_prompt_tokens'] = sum(per_call)
    write_json(root, 'run-config.json', config)
    write_json(root, 'result.json', result)
    write_rows(root, 'llm-decisions.jsonl', rows)
    report = verifier.verify_run(root, max_input_tokens=120000)
    assert report['automated_checks_passed'], report
    assert report['checks']['known_reconciled_input_within_120000']
    assert report['details']['max_input_tokens_verified'] == 120000


def test_default_60000_still_rejects_matching_120000_run(artifact):
    root, result, _ = artifact
    config = dict(verifier.EXPECTED)
    config['max_input_tokens'] = 120000
    result['budgets']['max_input_tokens_per_robot'] = 120000
    write_json(root, 'run-config.json', config)
    write_json(root, 'result.json', result)
    report = verifier.verify_run(root)
    assert not report['automated_checks_passed']
    assert not report['checks']['fixed_pilot_config']
    assert not report['checks']['reported_fixed_budgets']
    assert 'known_reconciled_input_within_60000' in report['checks']


def test_default_seed_41_rejects_seed_42_run(artifact):
    root, result, _ = artifact
    config = dict(verifier.EXPECTED)
    config['seed'] = 42
    result['seed'] = 42
    write_json(root, 'run-config.json', config)
    write_json(root, 'result.json', result)
    report = verifier.verify_run(root)
    assert not report['automated_checks_passed']
    assert not report['checks']['fixed_pilot_config']
    assert not report['checks']['single_robot_seed_and_mode']


def test_explicit_seed_42_verifies_matching_config_and_result(artifact):
    root, result, _ = artifact
    config = dict(verifier.EXPECTED)
    config['seed'] = 42
    result['seed'] = 42
    write_json(root, 'run-config.json', config)
    write_json(root, 'result.json', result)
    report = verifier.verify_run(root, expected_seed=42)
    assert report['automated_checks_passed'], report
    assert report['details']['expected_seed_verified'] == 42


@pytest.mark.parametrize('config_seed,result_seed', [(42, 41), (41, 42)])
def test_explicit_seed_requires_config_and_result_to_match(artifact, config_seed, result_seed):
    root, result, _ = artifact
    config = dict(verifier.EXPECTED)
    config['seed'] = config_seed
    result['seed'] = result_seed
    write_json(root, 'run-config.json', config)
    write_json(root, 'result.json', result)
    report = verifier.verify_run(root, expected_seed=42)
    assert not report['automated_checks_passed']
    assert not (report['checks']['fixed_pilot_config']
                and report['checks']['single_robot_seed_and_mode'])


@pytest.mark.parametrize('invalid', [None, True, 1.5, '42'])
def test_verify_run_requires_integer_expected_seed(artifact, invalid):
    root, _, _ = artifact
    with pytest.raises(ValueError, match='expected_seed must be an integer'):
        verifier.verify_run(root, expected_seed=invalid)


def test_usage_over_explicit_limit_fails(artifact):
    root, result, rows = artifact
    config = dict(verifier.EXPECTED)
    config['max_input_tokens'] = 120000
    result['budgets']['max_input_tokens_per_robot'] = 120000
    per_call = (24000, 24000, 24000, 24000, 24001)
    for row, tokens in zip((r for r in rows if r['event'] == 'llm_result'), per_call):
        row['audit']['usage']['prompt_tokens'] = tokens
    result['input_usage']['r1']['reported_prompt_tokens'] = sum(per_call)
    write_json(root, 'run-config.json', config)
    write_json(root, 'result.json', result)
    write_rows(root, 'llm-decisions.jsonl', rows)
    report = verifier.verify_run(root, max_input_tokens=120000)
    assert not report['checks']['known_reconciled_input_within_120000']


@pytest.mark.parametrize('invalid', [None, True, 0, -1, 1.5, '120000'])
def test_verify_run_requires_positive_integer_limit(artifact, invalid):
    root, _, _ = artifact
    with pytest.raises(ValueError, match='positive integer'):
        verifier.verify_run(root, max_input_tokens=invalid)


def test_missing_last_response_is_not_silently_dropped(artifact):
    root, _, rows = artifact
    write_rows(root, 'llm-decisions.jsonl', rows[:-1])
    assert not verifier.verify_run(root)['checks']['no_missing_or_duplicate_call_accounting']


def test_tampered_input_hash_invalidates_camera_evidence(artifact):
    root, _, _ = artifact
    (root / '0-wrist.jpg').write_bytes(b'different bytes')
    assert not verifier.verify_run(root)['checks']['original_own_rgb_hashes']


def test_finish_without_actual_model_release_is_not_completion(artifact):
    root, _, rows = artifact
    for row in rows:
        if row.get('decision', {}).get('action', {}).get('kind') == 'release':
            row['decision']['action']['kind'] = 'wait'
    write_rows(root, 'llm-decisions.jsonl', rows)
    assert not verifier.verify_run(root)['checks']['model_selected_full_chain']


def test_drive_before_pick_does_not_count_as_carry_transport(artifact):
    root, _, rows = artifact
    accepted = [row for row in rows if row.get('event') == 'llm_result']
    accepted[0]['decision']['action']['kind'] = 'drive'
    accepted[2]['decision']['action']['kind'] = 'approach'
    write_rows(root, 'llm-decisions.jsonl', rows)
    assert not verifier.verify_run(root)['checks']['model_selected_full_chain']


def test_failed_physical_gate_overrides_success_flag(artifact):
    root, result, _ = artifact
    result['outcomes']['r1']['gates']['inside_destination'] = False
    write_json(root, 'result.json', result)
    assert not verifier.verify_run(root)['checks']['task_and_visual_release_gates']


def test_commands_after_deadline_invalidate_attempt(artifact):
    root, _, _ = artifact
    write_rows(root, 'commands.jsonl', [{'event': 'raw_action', 'time': 11, 'raw_action': {'kind': 'drive'}}])
    assert not verifier.verify_run(root)['checks']['no_motion_command_after_stop']


def test_explicit_test_double_model_is_rejected(artifact):
    root, _, rows = artifact
    for row in rows:
        if 'audit' in row: row['audit']['response_model'] = 'OFFLINE_TEST_DOUBLE'
    write_rows(root, 'llm-decisions.jsonl', rows)
    assert not verifier.verify_run(root)['checks']['actual_model_reply_metadata']


def test_missing_usage_cannot_be_interpreted_as_free(artifact):
    root, _, rows = artifact
    rows[-1]['audit']['usage'] = None
    write_rows(root, 'llm-decisions.jsonl', rows)
    assert not verifier.verify_run(root)['checks']['known_reconciled_input_within_60000']


def test_bad_video_stays_unverified(artifact, monkeypatch):
    root, _, _ = artifact
    monkeypatch.setattr(verifier, '_video_ok', lambda path: False)
    assert not verifier.verify_run(root)['checks']['video_decodes_first_and_last_frame']


@pytest.mark.parametrize('key,value', [('decision_elapsed_sim_s', 301.), ('passive_settle_s', 2.)])
def test_active_and_passive_time_are_separate_and_bounded(artifact, key, value):
    root, result, _ = artifact
    result[key] = value
    write_json(root, 'result.json', result)
    assert not verifier.verify_run(root)['checks']['same_active_time_and_existing_passive_settle']


def test_fixed_pilot_command_has_no_budget_or_model_substitution(tmp_path):
    command = pilot_command(tmp_path / 'fresh')
    for flag, value in {'--max-calls': '30', '--max-input-tokens': '60000', '--seconds': '300',
                        '--robots': '1', '--seed': '41', '--model': 'gemini-3.8-flash',
                        '--impratio': '10', '--noslip-iterations': '3', '--communication': 'none',
                        '--max-transient-failures': '2'}.items():
        assert command[command.index(flag)+1] == value
    assert '--record' in command
