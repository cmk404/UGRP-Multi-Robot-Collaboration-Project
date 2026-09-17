"""Four-output upstream ACT preset; deployment consumes exactly two RGBs."""
import json
from pathlib import Path
import numpy as np
import torch,draccus
from safetensors.torch import load_file,save_file
from lerobot.configs.types import FeatureType,PolicyFeature
from lerobot.policies.act.modeling_act import ACTPolicy
from harness.reference_act import make_policy,actor_batch

SCALES=np.array([.15,.10,.15],dtype=float)

def make_recovery_policy(*,pretrained=False):
    old=make_policy('imagenet128',pretrained=pretrained)
    config=old.config
    config.output_features={'action':PolicyFeature(type=FeatureType.ACTION,shape=(4,))}
    policy=ACTPolicy(config)
    policy.model.backbone.load_state_dict(old.model.backbone.state_dict())
    return policy

def decode(values):
    v=np.asarray(values,dtype=float)
    if v.shape!=(4,) or not np.isfinite(v).all():raise ValueError('four finite ACT outputs required')
    commands=np.clip(v[:3]*SCALES,[-.05,-.10,-.15],[.15,.10,.15])
    stop=float(np.clip(v[3],0,1))
    return {'ok':True,'ready':bool(stop>=.65 and np.all(np.abs(commands)<=[.003,.002,.004])),
            **dict(zip(('forward','left','turn'),map(float,commands))),'stop_score':stop,'reason':'recovery_act_rgb'}

class RecoveryAct:
    def __init__(self,policy):self.policy=policy.eval()
    @torch.inference_mode()
    def predict(self,own_jpeg,top_jpeg):
        return decode(self.policy.select_action(actor_batch(own_jpeg,top_jpeg,'imagenet128'))[0].cpu().numpy())
    def save(self,root):
        root.mkdir(parents=True,exist_ok=False)
        save_file({k:v.detach().cpu().contiguous() for k,v in self.policy.state_dict().items()},str(root/'model.safetensors'))
        (root/'config.json').write_text(json.dumps(draccus.encode(self.policy.config),indent=2))
        (root/'adapter.json').write_text(json.dumps({'profile':'imagenet128','outputs':['forward','left','turn','stop'],'version':1}))
    @classmethod
    def load(cls,root):
        meta=json.loads((root/'adapter.json').read_text())
        if meta!={'profile':'imagenet128','outputs':['forward','left','turn','stop'],'version':1}:raise ValueError('wrong adapter')
        policy=make_recovery_policy()
        if json.loads((root/'config.json').read_text())!=json.loads(json.dumps(draccus.encode(policy.config))):raise ValueError('wrong RGB-only architecture')
        policy.load_state_dict(load_file(str(root/'model.safetensors')),strict=True);policy.reset()
        return cls(policy)
