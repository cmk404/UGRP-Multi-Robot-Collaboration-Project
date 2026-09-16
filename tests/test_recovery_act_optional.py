import io,json
from pathlib import Path
import pytest
pytest.importorskip('lerobot')
import torch
from PIL import Image
from harness.recovery_act import make_recovery_policy,RecoveryAct,decode
from harness.reference_act import actor_batch
from harness.recovery_act_client import RecoveryClient
from scripts.recovery_act_worker import decode_request

def test_actual_rgb_forward_backward_and_checkpoint(tmp_path):
 torch.set_num_threads(2);policy=make_recovery_policy();policy.train()
 image=io.BytesIO();Image.new('RGB',(160,120),(70,90,110)).save(image,format='JPEG');rgb=image.getvalue()
 batch=actor_batch(rgb,rgb,'imagenet128');batch.update(action=torch.zeros(1,4,4),action_is_pad=torch.zeros(1,4,dtype=torch.bool))
 loss,_=policy(batch);loss.backward();assert torch.isfinite(loss)
 actor=RecoveryAct(policy);before=actor.predict(rgb,rgb);actor.save(tmp_path/'act');after=RecoveryAct.load(tmp_path/'act').predict(rgb,rgb);assert before==after
 assert decode([-.5,.5,-.5,0])['forward']==-.05
 assert decode([0,0,0,1])['ready']
 with pytest.raises(ValueError):decode_request({'own_rgb':'','top_rgb':'','state':[]})
 import sys
 client=RecoveryClient(Path(sys.executable),tmp_path/'act')
 try:assert client.predict(rgb,rgb)==before
 finally:client.close()

def test_recovery_config_rejects_measured_state(tmp_path):
 actor=RecoveryAct(make_recovery_policy());actor.save(tmp_path/'act')
 p=tmp_path/'act/config.json';r=json.loads(p.read_text());r['input_features']['observation.state']={'type':'STATE','shape':[2]};p.write_text(json.dumps(r))
 with pytest.raises(ValueError):RecoveryAct.load(tmp_path/'act')
