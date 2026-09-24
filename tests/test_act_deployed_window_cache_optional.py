"""Deployment-shaped frozen CNN caching for the next carry ACT study."""
import io

import pytest

pytest.importorskip('lerobot')
import torch
from PIL import Image

from harness.act_training import frozen_features
from harness.carry_input_act import InputCarryAct, actor_batch, make_policy
from harness.reference_act import IMAGE_KEYS
from scripts.train_carry_input_act import (batch_at, cache_images, frames_at,
                                           history_indices, verify_all_deployed_windows,
                                           verify_cache, memo_readback_indices,
                                           decoded_prediction_close,
                                           validated_partial_overlap)


def _jpeg(rgb):
    buffer = io.BytesIO()
    Image.new('RGB', (160, 120), rgb).save(buffer, format='JPEG')
    return buffer.getvalue()


def _episodes():
    result = []
    for episode, count in enumerate((5, 4)):
        sequence = []
        for i in range(count):
            sequence.append({
                'id': f'{episode}:{i}',
                'own_jpeg': _jpeg((25 + episode * 60 + i * 7, 45 + i * 11, 95)),
                'top_jpeg': _jpeg((80, 30 + episode * 40 + i * 9, 130)),
                'context': [.5, -.4, float(episode == 0), float(episode == 1),
                            0., .01 * i, -.01 * i, 0.],
            })
        result.append({'sequences': {'r1': sequence}})
    return result


@pytest.mark.parametrize('history,native_batch', ((4, 4), (1, 1)))
def test_deployed_window_cache_matches_real_act_inference(tmp_path, monkeypatch, history, native_batch):
    torch.set_num_threads(2)
    torch.manual_seed(204)
    policy = make_policy(128, history)
    policy.eval()
    for parameter in policy.model.backbone.parameters():
        parameter.requires_grad_(False)
    episodes = _episodes()
    rows = [row for episode in episodes for sequence in episode['sequences'].values() for row in sequence]
    windows = history_indices(episodes, history)
    assert windows[5].tolist() == ([5, 5, 5, 5] if history == 4 else [5])

    calls = []
    native = policy.model.backbone.native
    original = native.forward

    def counted(frames):
        calls.append(tuple(frames.shape))
        return original(frames)

    monkeypatch.setattr(native, 'forward', counted)
    cache = cache_images(policy, rows, 128, windows=windows, mode='deployed_window')
    assert len(calls) == 2 * len(rows)
    assert {shape[0] for shape in calls} == {native_batch}
    assert cache['_feature_cache_mode'] == 'deployed_window'
    for i in (0, 1, 4, 5, len(rows) - 1):
        native_input = actor_batch(frames_at(rows, windows[i].tolist()), 128, history)
        cached_input = batch_at(cache, windows[i:i + 1])
        with torch.no_grad():
            for key in IMAGE_KEYS:
                expected = policy.model.backbone(native_input[key])['feature_map']
                torch.testing.assert_close(cached_input[key], expected, rtol=0, atol=0)
            expected_action = policy.predict_action_chunk(native_input)
            with frozen_features(policy):
                cached_action = policy.predict_action_chunk(cached_input)
            torch.testing.assert_close(cached_action, expected_action, rtol=1e-5, atol=1e-5)
    checked = verify_cache(policy, rows, windows, cache, 128, history)
    assert checked['original_full_chunk_guard_enforced']
    assert all(p['full_chunk_original_strict_guard_passed'] for p in checked['probes'])


def test_deployed_window_rejects_wrong_history_or_corrupt_features():
    torch.set_num_threads(2)
    torch.manual_seed(205)
    policy = make_policy(128, 4)
    for parameter in policy.model.backbone.parameters():
        parameter.requires_grad_(False)
    episodes = _episodes()
    rows = [row for episode in episodes for sequence in episode['sequences'].values() for row in sequence]
    windows = history_indices(episodes, 4)
    wrong = windows.clone()
    wrong[5, -1] = 4
    with pytest.raises(ValueError, match='end at their own row'):
        cache_images(policy, rows, 128, windows=wrong, mode='deployed_window')
    cache = cache_images(policy, rows, 128, windows=windows, mode='deployed_window')
    cache[IMAGE_KEYS[0]][0, 0, 0, 0, 0] += .01
    with pytest.raises(AssertionError):
        verify_cache(policy, rows, windows, cache, 128, 4)


def test_selected_all_row_guard_uses_original_full_chunk_tolerance():
    torch.set_num_threads(2)
    torch.manual_seed(206)
    policy = make_policy(128, 4)
    for parameter in policy.model.backbone.parameters():
        parameter.requires_grad_(False)
    sequences = {}
    for slot, example in zip(('r1', 'r3'), _episodes()):
        rows = example['sequences']['r1'][:4]
        for i, row in enumerate(rows):
            row['id'] = f'case:{slot}:{i}'
            row['action'] = [0.1, 0., 0., float(i == 3)]
            row['done'] = i == 3
        sequences[slot] = rows
    episodes = [{'sequences': sequences}]
    rows = [row for sequence in sequences.values() for row in sequence]
    windows = history_indices(episodes, 4)
    cache = cache_images(policy, rows, 128, windows=windows, mode='deployed_window')
    verified, predictions = verify_all_deployed_windows(
        policy, rows, windows, cache, 128, 4, 'deployed_first_action')
    assert verified['rows'] == len(rows) == len(predictions)
    assert verified['guards']['full_chunk_original_strict_guard_passed']
    assert verified['guards']['all_chunk_done_decisions_same']
    assert verified['cache_metrics']['samples'] == verified['native_metrics']['samples'] == 8
    cache[IMAGE_KEYS[0]][0] += .1
    corrupted, _ = verify_all_deployed_windows(
        policy, rows, windows, cache, 128, 4, 'deployed_first_action')
    assert not corrupted['guards']['full_chunk_original_strict_guard_passed']
    assert 'case:r1:0' in corrupted['guards']['full_chunk_original_strict_mismatch_ids_first64']


def test_memo_readback_selects_later_real_overlap_and_preserves_short_episode_bounds():
    rows = [{'own_jpeg': b'first', 'top_jpeg': b'fixed'} for _ in range(20)]
    rows[10] = {'own_jpeg': b'new-own', 'top_jpeg': b'fixed'}
    episodes = [{'sequences': {'r1': rows}}]
    windows = history_indices(episodes, 4)
    assert memo_readback_indices(rows, windows) == [0, 1, 2, 3, 9, 10, 19]
    assert memo_readback_indices(rows[:3], history_indices(
        [{'sequences': {'r1': rows[:3]}}], 4)) == [0, 1, 2]
    assert not validated_partial_overlap(4, 9, 10, windows, hits=7, misses=0)
    assert not validated_partial_overlap(4, 9, 10, windows, hits=0, misses=1)
    assert validated_partial_overlap(4, 9, 10, windows, hits=7, misses=1)


def test_memo_decoded_readback_allows_small_numeric_drift_but_never_done_flip():
    native = {'action': {'forward': .06, 'left': -.02, 'turn': .005},
              'done': False, 'stop_score': .649999}
    small = {'action': dict(native['action']), 'done': False,
             'stop_score': native['stop_score']+1e-7}
    small['action']['forward'] += 1e-7
    assert decoded_prediction_close(small, native)
    flipped = dict(small, done=True)
    assert not decoded_prediction_close(flipped, native)
    large = dict(small, action=dict(small['action'], forward=.0601))
    assert not decoded_prediction_close(large, native)


def test_actual_memoized_worker_partial_windows_match_native_first_action():
    torch.set_num_threads(2)
    torch.manual_seed(207)
    original = make_policy(128, 4)
    cached_policy = make_policy(128, 4)
    cached_policy.load_state_dict(original.state_dict())
    native = InputCarryAct(original, 128, 4)
    memoized = InputCarryAct(cached_policy, 128, 4, cache_features=True)
    episodes = _episodes()
    rows = episodes[0]['sequences']['r1']
    windows = history_indices([episodes[0]], 4)
    observed = []
    for i in range(4):
        frames = frames_at(rows, windows[i].tolist())
        batch = actor_batch(frames, 128, 4)
        hits = memoized.policy.model.backbone.cache_hits
        misses = memoized.policy.model.backbone.cache_misses
        with torch.inference_mode():
            expected = native.policy.predict_action_chunk(batch)[0, 0]
            actual = memoized.policy.predict_action_chunk(batch)[0, 0]
        observed.append((memoized.policy.model.backbone.cache_hits-hits,
                         memoized.policy.model.backbone.cache_misses-misses))
        torch.testing.assert_close(actual, expected, rtol=1e-5, atol=1e-5)
        assert bool(actual[3] >= .65) == bool(expected[3] >= .65)
        assert decoded_prediction_close(memoized.predict(frames), native.predict(frames))
    assert all(hits > 0 and misses > 0 for hits, misses in observed[1:])
