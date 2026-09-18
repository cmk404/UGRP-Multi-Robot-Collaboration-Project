"""Real upstream inference/backward/save/readback, optional runtime only."""
import io
import pytest
pytest.importorskip('lerobot')
import torch
from PIL import Image
from harness.pair_carry_act import CarryAct,make_carry_policy,CONTEXT_KEY
from harness.reference_act import actor_batch

def test_carry_native(tmp_path):
    torch.set_num_threads(2);torch.manual_seed(18)
    im=io.BytesIO();Image.new('RGB',(160,120),(90,100,20)).save(im,format='JPEG');rgb=im.getvalue()
    policy=make_carry_policy();batch=actor_batch(rgb,rgb,'imagenet128');batch[CONTEXT_KEY]=torch.zeros(1,8)
    batch['action']=torch.zeros(1,8,4);batch['action_is_pad']=torch.zeros(1,8,dtype=torch.bool)
    policy.train();loss,_=policy(batch);loss.backward();assert torch.isfinite(loss)
    actor=CarryAct(policy);before=actor.predict(rgb,rgb,[0.]*8);actor.save(tmp_path/'act')
    assert CarryAct.load(tmp_path/'act').predict(rgb,rgb,[0.]*8)==before
    assert policy.config.robot_state_feature is None
