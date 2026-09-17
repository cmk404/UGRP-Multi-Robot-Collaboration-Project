"""Optional upstream LeRobot ACT adapter with exactly two RGB inputs.

This small research preset is not the original ACT paper configuration.
No measured state, teacher, scene, evaluation, or simulator object enters here.
"""
from __future__ import annotations

import io
import json
from pathlib import Path

import numpy as np
import draccus
from PIL import Image
import torch
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from safetensors.torch import load_file, save_file

UPSTREAM_SHA = "89236ea0f4f81a81ca566081e20dd1ff5f823cbe"
IMAGE_SIZE = 96
IMAGE_KEYS = ("observation.images.own", "observation.images.top")


def image_tensor(jpeg: bytes) -> torch.Tensor:
    with Image.open(io.BytesIO(jpeg)) as image:
        image = image.convert("RGB").resize((IMAGE_SIZE, IMAGE_SIZE), Image.Resampling.BILINEAR)
        array = np.array(image, dtype=np.float32) / 255.0
    return torch.from_numpy(array).permute(2, 0, 1)


def make_policy() -> ACTPolicy:
    config = ACTConfig(
        input_features={key: PolicyFeature(type=FeatureType.VISUAL,
                                          shape=(3, IMAGE_SIZE, IMAGE_SIZE)) for key in IMAGE_KEYS},
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(2,))},
        chunk_size=4, n_action_steps=1, pretrained_backbone_weights=None,
        dim_model=64, n_heads=4, dim_feedforward=256, n_encoder_layers=2,
        n_decoder_layers=1, latent_dim=8, n_vae_encoder_layers=1, kl_weight=1.0,
        dropout=0.0, device="cpu",
    )
    if config.robot_state_feature is not None or config.env_state_feature is not None:
        raise ValueError("ACT configuration must be RGB-only")
    return ACTPolicy(config)


def actor_batch(own_jpeg: bytes, top_jpeg: bytes) -> dict:
    return {IMAGE_KEYS[0]: image_tensor(own_jpeg).unsqueeze(0),
            IMAGE_KEYS[1]: image_tensor(top_jpeg).unsqueeze(0)}


def decode_prediction(values) -> dict:
    value = np.asarray(values, dtype=float)
    if value.shape != (2,) or not np.all(np.isfinite(value)):
        return {"ok": False, "ready": False, "forward": 0.0, "stop_score": 0.0,
                "reason": "invalid_act_output"}
    forward = float(np.clip(value[0], 0, 1) * .15)
    stop = float(np.clip(value[1], 0, 1))
    return {"ok": True, "ready": bool(stop >= .65 and forward <= .003),
            "forward": forward, "stop_score": stop, "reason": "reference_act_rgb",
            "ood_detection": "not_implemented"}


class RGBAct:
    def __init__(self, policy):
        self.policy = policy.eval()

    @torch.inference_mode()
    def predict(self, own_jpeg: bytes, top_jpeg: bytes) -> dict:
        # n_action_steps=1: every command uses the newest camera pair.
        batch = actor_batch(own_jpeg, top_jpeg)
        value = self.policy.select_action(batch)[0].cpu().numpy()
        return decode_prediction(value)

    def save(self, root: Path) -> None:
        root.mkdir(parents=True, exist_ok=False)
        save_file({k: v.detach().cpu().contiguous() for k, v in self.policy.state_dict().items()},
                  str(root / "model.safetensors"))
        (root / "config.json").write_text(json.dumps(draccus.encode(self.policy.config), indent=2))

    @classmethod
    def load(cls, root: Path):
        policy = make_policy()
        saved = json.loads((root / "config.json").read_text())
        if saved != json.loads(json.dumps(draccus.encode(policy.config))):
            raise ValueError("checkpoint config differs from audited RGB preset")
        policy.load_state_dict(load_file(str(root / "model.safetensors")), strict=True)
        policy.reset()
        return cls(policy)
