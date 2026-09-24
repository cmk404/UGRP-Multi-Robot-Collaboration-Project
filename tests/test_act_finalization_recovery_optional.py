"""Recovery guards preserve the deployed action and failed-run provenance."""
import copy
import hashlib
import json
from types import SimpleNamespace

import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('lerobot')

from scripts.finalize_carry_input_act import (EXPECTED_BUNDLE, EXPECTED_BUNDLE_SHA,
                                              EXPECTED_DATASET, EXPECTED_SEED,
                                              EXPECTED_STEPS, EXPECTED_SELECTED_STEP,
                                              TRAINING_SOURCE, state_digest,
                                              verify_checkpoint, verify_original_metadata)
from scripts.train_carry_input_act import (classify_cached_action_chunk,
                                           compare_cached_action_chunk)
from harness.carry_input_act import metadata


def chunks():
    expected = torch.zeros(1, 8, 4)
    expected[0, :, 3] = 1.0019334554672241
    return expected, expected.clone()


def test_known_unused_done_drift_passes_but_old_whole_chunk_guard_stays_failed():
    expected, actual = chunks()
    actual[0, 1, 3] += 3.8743019104003906e-5
    classified = compare_cached_action_chunk(actual, expected)
    assert classified['first_action_original_strict_guard_passed']
    assert classified['full_chunk_bounded_guard_passed']
    assert classified['all_chunk_done_decisions_same']
    assert not classified['full_chunk_original_strict_guard_passed']
    assert classified['original_full_chunk_strict_mismatch_indices_first64'] == [[0, 1, 3]]


def test_first_action_drift_above_original_guard_is_rejected():
    expected, actual = chunks()
    actual[0, 0, 0] += 2e-5
    with pytest.raises(ValueError, match='first ACT action'):
        compare_cached_action_chunk(actual, expected)


def test_tiny_done_threshold_flip_is_rejected_before_tolerance():
    expected, actual = chunks()
    expected[0, 3, 3] = .65-1e-7
    actual[0, 3, 3] = .65+1e-7
    assert classify_cached_action_chunk(actual, expected)['full_chunk_bounded_guard_passed']
    with pytest.raises(ValueError, match='done decision changed'):
        compare_cached_action_chunk(actual, expected)


def test_large_unused_chunk_drift_is_rejected():
    expected, actual = chunks()
    actual[0, 4, 1] += 3e-4
    with pytest.raises(ValueError, match='bounded 1e-4'):
        compare_cached_action_chunk(actual, expected)


def test_nonfinite_or_shape_changed_chunk_is_rejected():
    expected, actual = chunks()
    actual[0, 1, 0] = float('nan')
    with pytest.raises(ValueError, match='nonfinite'):
        compare_cached_action_chunk(actual, expected)
    with pytest.raises(ValueError, match='both be'):
        compare_cached_action_chunk(expected[:, :7], expected)


def _record():
    selected = {'step': EXPECTED_SELECTED_STEP, 'candidate_rank': [1, 1., 1.2],
                'selection_eligible': False, 'development': {'offline_termination_pass': False}}
    report = {'source_sha': TRAINING_SOURCE, 'dataset_sha256': EXPECTED_DATASET,
              'adapter': metadata(128, 4), 'seed': EXPECTED_SEED,
              'steps': EXPECTED_STEPS, 'batch_size': 32, 'upstream_sha': 'fixture',
              'environment': {'device': 'cpu'}, 'activation_checkpointing': False,
              'termination_objective': 'deployed_first_action',
              'deployed_done_objective': {'version': 1, 'weight': 1.,
                                          'runtime_score_threshold': .65,
                                          'runtime_threshold_changed': False},
              'selected': selected, 'progress': [{'step': EXPECTED_STEPS}],
              'complete': False, 'completed_steps': EXPECTED_STEPS,
              'cpu_evaluation_batch_size': 32, 'dataset_path': '',
              'wall_s': 978., 'samples': 1}
    signature = {k: report[k] for k in ('source_sha', 'dataset_sha256', 'adapter',
                                         'seed', 'steps', 'batch_size', 'upstream_sha',
                                         'environment', 'activation_checkpointing',
                                         'termination_objective')}
    signature['deployed_done_objective'] = report['deployed_done_objective']
    state = {'weight': torch.tensor([.1, .2])}
    checkpoint = {'signature': signature, 'step': EXPECTED_STEPS,
                  'selected': selected, 'progress': report['progress'],
                  'best': tuple(selected['candidate_rank']), 'best_state': state,
                  'model': {'weight': torch.tensor([.4, .5])},
                  'elapsed_s': 978.}
    return report, checkpoint


def test_selected_state_identity_is_stable_and_final_training_state_cannot_replace_it():
    report, checkpoint = _record()
    digest = verify_checkpoint(checkpoint, report)
    assert digest == state_digest(checkpoint['best_state'])
    assert digest != state_digest(checkpoint['model'])
    changed = copy.deepcopy(checkpoint)
    changed['selected']['step'] += 1
    with pytest.raises(ValueError, match='selection/progress'):
        verify_checkpoint(changed, report)
    changed = copy.deepcopy(checkpoint)
    changed['step'] = 7999
    with pytest.raises(ValueError, match='full budget'):
        verify_checkpoint(changed, report)


def test_failed_original_and_exact_hashes_are_required(tmp_path, monkeypatch):
    from scripts import finalize_carry_input_act as finalizer
    report, _ = _record()
    dataset = tmp_path/'dataset.json'
    dataset.write_text('{}')
    report['dataset_path'] = str(dataset.resolve())
    report['train_episodes'] = ['train']
    report['development_episodes'] = ['development']
    failed = tmp_path/'failed'
    artifact = failed/'artifacts'
    artifact.mkdir(parents=True)
    paths = {'manifest': failed/'manifest.json', 'report': artifact/'report.json',
             'checkpoint': artifact/'resume.pt'}
    manifest = {'workflow_id': 'act-input-training', 'status': 'process_failed',
                'exit_code': 1, 'source': {'source_sha': TRAINING_SOURCE,
                                           'source_dirty': False},
                'source_changed_during_run': False,
                'inputs_changed_during_run': False}
    console = failed/'console.log'
    console.write_text('Traceback (most recent call last):\n  verify_cache\n'
                       'Mismatched elements: 6 / 32\n3.8743019104003906e-05\n')
    manifest['logs'] = {'path': str(console.resolve()),
                        'files': [{'path': 'console.log',
                                   'sha256': hashlib.sha256(console.read_bytes()).hexdigest()}]}
    paths['manifest'].write_text(json.dumps(manifest))
    paths['report'].write_text(json.dumps(report))
    paths['checkpoint'].write_bytes(b'opaque saved state')
    freeze = tmp_path/'freeze.json'
    freeze.write_text(json.dumps({'schema': 'ugrp.action_act_refinement_freeze.v1',
                                  'refined_source_sha': TRAINING_SOURCE,
                                  'dataset_sha256': EXPECTED_DATASET,
                                  'bundle_id': EXPECTED_BUNDLE,
                                  'bundle_sha256': EXPECTED_BUNDLE_SHA}))
    monkeypatch.setattr(finalizer, 'verify_dataset',
                        lambda _: {'dataset_sha256': EXPECTED_DATASET})
    def sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    expected = {key: sha(path) for key, path in paths.items()}
    expected['source_freeze'] = sha(freeze)
    actual, original, hashes, trace = verify_original_metadata(paths, expected, dataset, freeze)
    assert not actual['complete'] and original['status'] == 'process_failed'
    assert hashes['checkpoint'] == expected['checkpoint']
    assert 'verify_cache' in trace
    wrong = dict(expected, checkpoint='0'*64)
    with pytest.raises(ValueError, match='original checkpoint SHA-256'):
        verify_original_metadata(paths, wrong, dataset, freeze)
    report['complete'] = True
    paths['report'].write_text(json.dumps(report))
    expected['report'] = sha(paths['report'])
    with pytest.raises(ValueError, match='original did not finish'):
        verify_original_metadata(paths, expected, dataset, freeze)


def test_recovery_output_cannot_overlap_original_or_source(tmp_path, monkeypatch):
    from scripts import finalize_carry_input_act as finalizer
    failed = tmp_path/'failed'
    failed.mkdir()
    monkeypatch.setattr(finalizer, 'source_identity', lambda: 'new-clean-commit')
    with pytest.raises(ValueError, match='overlaps'):
        finalizer.run(SimpleNamespace(failed_run=failed, out=failed/'new'))
    with pytest.raises(ValueError, match='outside the committed source'):
        finalizer.run(SimpleNamespace(failed_run=failed, out=finalizer.ROOT/'outputs'/'new'))


def test_memoized_worker_first_action_guard_rejects_drift_and_done_flip(monkeypatch):
    from scripts import finalize_carry_input_act as finalizer
    from harness.pair_carry_act_contract import decode
    monkeypatch.setattr(finalizer, 'actor_batch', lambda *_: {})

    class Policy:
        def __init__(self, raw):
            self.raw = torch.tensor(raw).reshape(1, 1, 4)
            self.model = SimpleNamespace(backbone=SimpleNamespace(cache_hits=1))

        def predict_action_chunk(self, _batch):
            return self.raw

    class Actor:
        def __init__(self, raw):
            self.policy = Policy(raw)

        def predict(self, _frames):
            return decode(self.policy.raw[0, 0].tolist())

    normal = Actor([0., 0., 0., .5])
    memo = Actor([0., 0., 0., .5])
    assert finalizer.verify_memoized_runtime_first(normal, memo, [])['first_action_max_abs'] == 0
    memo = Actor([2e-5, 0., 0., .5])
    with pytest.raises(ValueError, match='memoized runtime first ACT action'):
        finalizer.verify_memoized_runtime_first(normal, memo, [])
    normal = Actor([0., 0., 0., .65-1e-7])
    memo = Actor([0., 0., 0., .65+1e-7])
    with pytest.raises(ValueError, match='memoized runtime done decision'):
        finalizer.verify_memoized_runtime_first(normal, memo, [])
