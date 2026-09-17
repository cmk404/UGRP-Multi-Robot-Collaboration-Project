"""Real upstream integration, run in requirements-reference-act environment."""
import io
import json
from pathlib import Path
import sys

import pytest

pytest.importorskip('lerobot')
import torch
from PIL import Image

from harness.reference_act import IMAGE_KEYS, RGBAct, actor_batch, decode_prediction, make_policy
from harness.reference_act_client import ActClient


def jpeg(color):
    stream = io.BytesIO()
    Image.new('RGB', (112, 84), color).save(stream, format='JPEG')
    return stream.getvalue()


def test_real_upstream_rgb_only_backward_checkpoint_and_process(tmp_path):
    torch.set_num_threads(2)
    torch.manual_seed(123)
    own, top = jpeg('red'), jpeg('blue')
    policy = make_policy()
    batch = actor_batch(own, top)
    assert set(batch) == set(IMAGE_KEYS)
    assert policy.config.robot_state_feature is None
    assert policy.config.env_state_feature is None
    assert policy.config.n_action_steps == 1
    training = {k: v.repeat(2, 1, 1, 1) for k, v in batch.items()}
    training.update(action=torch.zeros(2, 4, 2),
                    action_is_pad=torch.tensor([[False, True, True, True]] * 2))
    loss, _ = policy(training)
    assert torch.isfinite(loss)
    loss.backward()
    assert policy.model.action_head.weight.grad is not None
    actor = RGBAct(policy)
    model = tmp_path / 'act'
    actor.save(model)
    expected = actor.predict(own, top)
    assert RGBAct.load(model).predict(own, top) == expected
    client = ActClient(Path(sys.executable), model)
    try:
        assert client.predict(own, top) == expected
        client.process.stdin.write(json.dumps({'own_rgb': '', 'top_rgb': '', 'qpos': [0]}) + '\n')
        client.process.stdin.flush()
        with pytest.raises(RuntimeError, match='only own_rgb'):
            client._read()
    finally:
        client.close()
    assert client.process.poll() is not None
    config = json.loads((model / 'config.json').read_text())
    config['input_features']['observation.state'] = {'type': 'STATE', 'shape': [6]}
    (model / 'config.json').write_text(json.dumps(config))
    with pytest.raises(ValueError, match='audited RGB preset'):
        RGBAct.load(model)


def test_nonfinite_and_bounds():
    for value in ([float('nan'), 1], [1], [float('inf'), 0]):
        assert decode_prediction(value)['ok'] is False
        assert decode_prediction(value)['forward'] == 0
    assert decode_prediction([5, -2])['forward'] == .15
    assert decode_prediction([0, 1])['ready'] is True
