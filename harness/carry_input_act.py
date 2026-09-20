"""Resolution/history ablation, with independent CNN encoding of every frame.

Frozen frame features are concatenated oldest-to-current along width BEFORE
ACT's 2D positional encoding. Thus time order is identifiable without adding
an upstream fork, new learned visual layers, or cross-frame CNN seam artifacts.
This is an experimental adapter, not a new default carry policy.
"""
import io
import json
from pathlib import Path

import draccus
import numpy as np
import torch
from PIL import Image
from safetensors.torch import load_file, save_file
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.modeling_act import ACTPolicy

from harness.carry_input_history import validate_frames
from harness.pair_carry_act_contract import decode
from harness.pair_carry_act import make_carry_policy, CONTEXT_KEY
from harness.reference_act import IMAGE_KEYS


def metadata(size, history):
    if size not in (128, 256) or history not in (1, 4):
        raise ValueError('unsupported resolution/history')
    return {'version': 1, 'kind': 'carry_input_ablation', 'size': size, 'history': history,
            'order': 'oldest_to_current', 'padding': 'repeat_first_robot_local_frame',
            'temporal_encoding': 'independent_frozen_CNN_then_width_concat_before_ACT_2d_positions',
            'context': 'four chronological 8-vectors; history=1 repeats current context four times for matched parameter count',
            'outputs': ['forward', 'left', 'turn', 'done']}


class FrameBackbone(torch.nn.Module):
    def __init__(self, native, size, history):
        super().__init__()
        self.native, self.size, self.history = native, size, history

    def forward(self, images):
        if images.shape[-2:] != (self.size, self.size * self.history):
            raise ValueError('invalid temporal image dimensions')
        batch = images.shape[0]
        frames = torch.cat(images.split(self.size, dim=-1), dim=0)
        features = self.native(frames)['feature_map']
        return {'feature_map': torch.cat(features.split(batch, dim=0), dim=-1)}


def make_policy(size, history, pretrained=False):
    metadata(size, history)
    old = make_carry_policy(pretrained=pretrained)
    config = old.config
    for key in IMAGE_KEYS:
        config.input_features[key] = PolicyFeature(type=FeatureType.VISUAL,
                                                  shape=(3, size, size * history))
    config.input_features[CONTEXT_KEY] = PolicyFeature(type=FeatureType.ENV, shape=(32,))
    policy = ACTPolicy(config)
    policy.model.backbone.load_state_dict(old.model.backbone.state_dict())
    policy.model.backbone = FrameBackbone(policy.model.backbone, size, history)
    return policy


def image_tensor(jpeg, size):
    with Image.open(io.BytesIO(jpeg)) as im:
        arr = np.array(im.convert('RGB').resize((size, size), Image.Resampling.BILINEAR), dtype=np.float32)
    t = torch.from_numpy(arr).permute(2, 0, 1) / 255
    return (t - torch.tensor([.485, .456, .406])[:, None, None]) / torch.tensor([.229, .224, .225])[:, None, None]


def actor_batch(frames, size, history):
    validate_frames(frames, history)
    batch = {key: torch.cat([image_tensor(f[field], size) for f in frames], dim=-1).unsqueeze(0)
             for key, field in zip(IMAGE_KEYS, ('own_rgb', 'top_rgb'))}
    contexts = frames if history == 4 else frames * 4
    batch[CONTEXT_KEY] = torch.tensor([[v for f in contexts for v in f['context']]], dtype=torch.float32)
    return batch


class InputCarryAct:
    def __init__(self, policy, size, history):
        self.policy, self.size, self.history = policy.eval(), size, history

    @torch.inference_mode()
    def predict(self, frames):
        batch = actor_batch(frames, self.size, self.history)
        return decode(self.policy.predict_action_chunk(batch)[0, 0].tolist())

    def save(self, root):
        root = Path(root)
        root.mkdir(parents=True, exist_ok=False)
        save_file({k: v.detach().cpu().contiguous() for k, v in self.policy.state_dict().items()}, str(root / 'model.safetensors'))
        (root / 'config.json').write_text(json.dumps(draccus.encode(self.policy.config), indent=2))
        (root / 'adapter.json').write_text(json.dumps(metadata(self.size, self.history), indent=2))

    @classmethod
    def load(cls, root):
        root = Path(root)
        meta = json.loads((root / 'adapter.json').read_text())
        if meta != metadata(meta['size'], meta['history']):
            raise ValueError('unsupported adapter')
        policy = make_policy(meta['size'], meta['history'])
        if json.loads((root / 'config.json').read_text()) != json.loads(json.dumps(draccus.encode(policy.config))):
            raise ValueError('checkpoint architecture mismatch')
        policy.load_state_dict(load_file(str(root / 'model.safetensors')), strict=True)
        return cls(policy, meta['size'], meta['history'])
