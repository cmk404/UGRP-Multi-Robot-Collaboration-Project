"""Task-conditioned ACT for local carry; env vector contains NO measured state."""
import json
from pathlib import Path
import draccus
import torch
from safetensors.torch import load_file, save_file
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.act.modeling_act import ACTPolicy
from harness.reference_act import make_policy, actor_batch
from harness.pair_carry_act_contract import CONTEXT_SIZE, decode
CONTEXT_KEY='observation.environment_state'
META={'version':1,'profile':'imagenet128','context':['authored_goal_x/3','authored_goal_y/3','route_north','route_south','model_slot_r3','own_previous_forward/.15','own_previous_left/.15','own_previous_turn/.15'],'outputs':['forward','left','turn','done']}

def make_carry_policy(pretrained=False):
    old=make_policy('imagenet128',pretrained=pretrained)
    cfg=old.config
    cfg.input_features[CONTEXT_KEY]=PolicyFeature(type=FeatureType.ENV,shape=(CONTEXT_SIZE,))
    cfg.output_features={'action':PolicyFeature(type=FeatureType.ACTION,shape=(4,))}
    cfg.chunk_size=8
    policy=ACTPolicy(cfg)
    policy.model.backbone.load_state_dict(old.model.backbone.state_dict())
    return policy

class CarryAct:
    def __init__(self,policy):self.policy=policy.eval()
    @torch.inference_mode()
    def predict(self,own,top,ctx):
        if len(ctx)!=CONTEXT_SIZE:raise ValueError('context shape')
        batch=actor_batch(own,top,'imagenet128')
        batch[CONTEXT_KEY]=torch.tensor([ctx],dtype=torch.float32)
        return decode(self.policy.predict_action_chunk(batch)[0,0].tolist())
    def save(self,root):
        root=Path(root);root.mkdir(parents=True,exist_ok=False)
        save_file({k:v.detach().cpu().contiguous() for k,v in self.policy.state_dict().items()},str(root/'model.safetensors'))
        (root/'config.json').write_text(json.dumps(draccus.encode(self.policy.config),indent=2))
        (root/'adapter.json').write_text(json.dumps(META,indent=2))
    @classmethod
    def load(cls,root):
        root=Path(root)
        if json.loads((root/'adapter.json').read_text())!=META:raise ValueError('wrong carry adapter')
        policy=make_carry_policy()
        if json.loads((root/'config.json').read_text())!=json.loads(json.dumps(draccus.encode(policy.config))):raise ValueError('wrong ACT config')
        policy.load_state_dict(load_file(str(root/'model.safetensors')),strict=True)
        return cls(policy)
