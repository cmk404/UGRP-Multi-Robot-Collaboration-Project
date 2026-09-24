"""Pure frozen-feature reuse must preserve actual ACT output and frame order."""
import io

import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('lerobot')
from PIL import Image

from harness.carry_input_act import FrameBackbone, InputCarryAct, make_policy


class CountBackbone(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.calls = []

    def forward(self, images):
        self.calls.append(len(images))
        return {'feature_map': images[:, :1] * 2}


def test_features_keep_temporal_order_deduplicate_and_evict():
    native = CountBackbone()
    wrapper = FrameBackbone(native, 2, 4).eval()
    wrapper.cache_capacity = 4
    frames = [torch.full((1, 3, 2, 2), float(i)) for i in range(6)]
    with torch.inference_mode():
        first = torch.cat([frames[0]] * 4, dim=-1)
        assert torch.equal(wrapper(first)['feature_map'], first[:, :1] * 2)
        assert native.calls == [1]
        wrapper(first)
        assert native.calls == [1]
        for start in (0, 1, 2):
            value = torch.cat(frames[start:start+4], dim=-1)
            assert torch.equal(wrapper(value)['feature_map'], value[:, :1] * 2)
        assert native.calls == [1, 3, 1, 1]
        assert len(wrapper._feature_cache) == 4
        wrapper(first)
        assert native.calls[-1] == 1  # frame zero was evicted
        wrapper.load_state_dict(wrapper.state_dict())
        assert not wrapper._feature_cache
    wrapper.train()
    differentiable = first.clone().requires_grad_()
    wrapper(differentiable)['feature_map'].sum().backward()
    assert differentiable.grad is not None and not wrapper._feature_cache


def _jpeg(i):
    stream = io.BytesIO()
    Image.new('RGB', (31, 27), (i * 21, 220 - i * 13, 30 + i * 17)).save(stream, 'JPEG')
    return stream.getvalue()


def test_actual_act_cached_predictions_match_uncached_after_history_change():
    old_threads = torch.get_num_threads()
    torch.set_num_threads(2)
    try:
        torch.manual_seed(1024)
        policy = make_policy(128, 4).eval()
        frames = [{'own_rgb': _jpeg(i), 'top_rgb': _jpeg(i+1),
                   'context': [.5, .2, 1., 0., 1., i * .01, 0., 0.]}
                  for i in range(5)]
        windows = [[frames[0]] * 4, frames[:4], frames[1:]]
        uncached = InputCarryAct(policy, 128, 4)
        expected = [uncached.predict(window) for window in windows]
        cached = InputCarryAct(policy, 128, 4, cache_features=True)
        actual = [cached.predict(window) for window in windows]
        for before, after in zip(expected, actual):
            assert after['action'] == pytest.approx(before['action'], abs=2e-6)
            assert after['stop_score'] == pytest.approx(before['stop_score'], abs=2e-6)
            assert after['done'] is before['done']
        assert policy.model.backbone.cache_hits > 0
        assert policy.model.backbone.cache_misses == 6
        assert len(cached._image_cache) == 6
        assert len(policy.model.backbone._feature_cache) <= 16
    finally:
        torch.set_num_threads(old_threads)
