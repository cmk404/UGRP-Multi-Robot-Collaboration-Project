"""Actual upstream cache equivalence and ready-output training regression."""
import io
import pytest
pytest.importorskip('lerobot')
import torch
from PIL import Image
from harness.reference_act import make_policy, actor_batch, IMAGE_KEYS, RGBAct
from harness.act_training import frozen_features, sampling_weights, prediction_metrics


def test_cached_frozen_cnn_preserves_predictions_gradients_and_restores(tmp_path):
    torch.set_num_threads(2); torch.manual_seed(42)
    stream=io.BytesIO(); Image.new('RGB',(128,128),'red').save(stream,format='JPEG')
    policy=make_policy('imagenet128')
    raw={k:v.repeat(2,1,1,1) for k,v in actor_batch(stream.getvalue(),stream.getvalue(),'imagenet128').items()}
    with pytest.raises(ValueError,match='frozen'):
        with frozen_features(policy): pass
    for p in policy.model.backbone.parameters(): p.requires_grad_(False)
    with torch.no_grad():
        cache={k:policy.model.backbone(v)['feature_map'] for k,v in raw.items()}
        expected=policy.predict_action_chunk(raw)
        with frozen_features(policy): actual=policy.predict_action_chunk(cache)
        torch.testing.assert_close(actual,expected,rtol=0,atol=0)
    labels={'action':torch.zeros(2,4,2),'action_is_pad':torch.zeros(2,4,dtype=torch.bool)}
    policy.train();torch.manual_seed(8);a,_=policy({**raw,**labels});a.backward()
    grad=policy.model.action_head.weight.grad.clone();policy.zero_grad()
    policy.train();torch.manual_seed(8)
    with frozen_features(policy): b,_=policy({**cache,**labels});b.backward()
    torch.testing.assert_close(a,b,rtol=0,atol=0)
    torch.testing.assert_close(grad,policy.model.action_head.weight.grad,rtol=0,atol=0)
    with pytest.raises(RuntimeError):
        with frozen_features(policy): raise RuntimeError('restore')
    actor=RGBAct(policy,'imagenet128');actor.save(tmp_path/'model')
    assert RGBAct.load(tmp_path/'model').predict(stream.getvalue(),stream.getvalue())==actor.predict(stream.getvalue(),stream.getvalue())


def test_sampling_and_metrics_do_not_reward_never_stopping():
    rows=[{'forward':.04,'stop':False}]*8+[{'forward':.01,'stop':False}]*2+[{'forward':0.,'stop':True}]
    weights=sampling_weights(rows,True)
    assert weights[:8].sum()==weights[8:10].sum()==weights[10:].sum()
    bad=prediction_metrics([[.1,0.]]*len(rows),rows)
    good=prediction_metrics([[r['forward']/.15,float(r['stop'])] for r in rows],rows)
    assert good['selection_score']==0
    assert bad['selection_score']>=1
