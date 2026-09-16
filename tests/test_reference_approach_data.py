import base64

import pytest

from harness.reference_approach_data import action_chunks, split_cases
from scripts.reference_act_worker import decode_request


def test_case_split_keeps_both_robots_and_all_frames_together():
    train, heldout = split_cases([f'c{i}' for i in range(6)], ['c2', 'c4'])
    assert train == ['c0', 'c1', 'c3', 'c5']
    assert heldout == ['c2', 'c4']
    for bad in ([], ['c2', 'c2'], ['missing'], [f'c{i}' for i in range(6)]):
        with pytest.raises(ValueError):
            split_cases([f'c{i}' for i in range(6)], bad)


def test_chunks_never_use_other_case_or_heldout_labels():
    trajectories = {
        'train': [{'forward': .03, 'stop': False}, {'forward': 0, 'stop': True}],
        'heldout': [{'forward': .15, 'stop': False}],
    }
    rows, target, pad = action_chunks(trajectories, ['train'], 4)
    assert len(rows) == 2
    assert target[0] == [[.2, 0.], [0., 1.], [0., 1.], [0., 1.]]
    assert pad == [[False, False, True, True], [False, True, True, True]]
    assert all(value != [1., 0.] for chunk in target for value in chunk)


def test_inference_wire_rejects_privileged_and_training_fields():
    request = {'own_rgb': base64.b64encode(b'own').decode(),
               'top_rgb': base64.b64encode(b'top').decode()}
    assert decode_request(request) == (b'own', b'top')
    for key in ('observation.state', 'qpos', 'contacts', 'success', 'action', 'label', 'map'):
        with pytest.raises(ValueError, match='only own_rgb'):
            decode_request({**request, key: [0]})
    with pytest.raises(ValueError):
        decode_request({'own_rgb': 12, 'top_rgb': ''})
    with pytest.raises(ValueError):
        decode_request({'own_rgb': '@@not-base64', 'top_rgb': ''})
