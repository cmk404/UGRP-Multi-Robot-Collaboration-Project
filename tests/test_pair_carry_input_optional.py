import io
import sys
import pytest
pytest.importorskip('lerobot')
import torch
from PIL import Image
from harness.carry_input_act import make_policy, InputCarryAct, actor_batch
from harness.carry_input_client import InputCarryClient
from harness.reference_act import IMAGE_KEYS
from harness.act_training import frozen_features


def frames(count):
    result = []
    for i in range(count):
        buf = io.BytesIO(); Image.new('RGB', (160, 120), (30+i*35, 70, 100)).save(buf, format='JPEG')
        result.append({'own_rgb': buf.getvalue(), 'top_rgb': buf.getvalue(), 'context': [0., 0., 1., 0., 0., .01*i, 0., 0.]})
    return result


@pytest.mark.parametrize('size,history', [(128,1),(128,4),(256,1),(256,4),(512,1),(512,4)])
def test_real_training_cache_and_checkpoint(tmp_path, size, history):
    torch.set_num_threads(2); torch.manual_seed(21)
    p = make_policy(size, history)
    for parameter in p.model.backbone.parameters():
        parameter.requires_grad_(False)
    fs = frames(history); b = actor_batch(fs, size, history)
    p.eval()
    with torch.no_grad():
        expected = p.predict_action_chunk(b)
        cached = {**b, **{k: p.model.backbone(b[k])['feature_map'] for k in IMAGE_KEYS}}
        with frozen_features(p):
            actual = p.predict_action_chunk(cached)
        torch.testing.assert_close(actual, expected, rtol=0, atol=0)
    # Temporal frames occupy different spatial positions; order is not discarded.
    if history == 4:
        reverse = actor_batch(list(reversed(fs)), size, history)
        assert not torch.equal(p.model.backbone(b[IMAGE_KEYS[0]])['feature_map'], p.model.backbone(reverse[IMAGE_KEYS[0]])['feature_map'])
    b.update(action=torch.zeros(1,8,4), action_is_pad=torch.zeros(1,8,dtype=torch.bool))
    p.train(); loss, _ = p(b); loss.backward()
    assert torch.isfinite(loss) and p.model.action_head.weight.grad is not None
    actor = InputCarryAct(p,size,history); actor.save(tmp_path/'act')
    before = actor.predict(fs)
    assert InputCarryAct.load(tmp_path/'act').predict(fs) == before
    if size in (128, 512) and history == 4:
        client = InputCarryClient(sys.executable, tmp_path/'act')
        try:
            assert client.predict(fs) == before
        finally:
            client.close()
        assert client.process.poll() is not None
    assert p.config.robot_state_feature is None


def test_history_indices_reset_at_robot_and_episode_boundaries():
    from scripts.train_carry_input_act import history_indices
    entries=[{'sequences': {'r1':[{},{}], 'r3':[{},{}]}}, {'sequences': {'r1':[{},{}]}}]
    assert history_indices(entries,4).tolist() == [[0,0,0,0],[0,0,0,1],[2,2,2,2],[2,2,2,3],[4,4,4,4],[4,4,4,5]]


@pytest.mark.parametrize('size,history', [(256,4),(512,1),(512,4)])
def test_arms_have_identical_initial_weights_and_parameter_count(size, history):
    torch.manual_seed(21);a=make_policy(128,1)
    torch.manual_seed(21);b=make_policy(size,history)
    assert sum(p.numel() for p in a.parameters())==sum(p.numel() for p in b.parameters())
    assert all(torch.equal(value,b.state_dict()[key]) for key,value in a.state_dict().items())


def test_cache_verification_uses_separately_batched_frames_and_rejects_corruption():
    from scripts.train_carry_input_act import cache_images, history_indices, verify_cache
    torch.set_num_threads(2); torch.manual_seed(21)
    policy = make_policy(256, 4)
    for parameter in policy.model.backbone.parameters():
        parameter.requires_grad_(False)
    fs = frames(4) * 8
    rows = [{'id': str(i), 'own_jpeg': f['own_rgb'], 'top_jpeg': f['top_rgb'],
             'context': f['context']} for i, f in enumerate(fs)]
    windows = history_indices([{'sequences': {'r1': rows}}], 4)
    cache = cache_images(policy, rows, 256)
    checked = verify_cache(policy, rows, windows, cache, 256, 4)
    assert len(checked['probes']) == 3
    cache[IMAGE_KEYS[0]][0, 0, 0, 0] += .01
    with pytest.raises(AssertionError):
        verify_cache(policy, rows, windows, cache, 256, 4)


@pytest.mark.parametrize('device', ['cpu', 'cuda'])
def test_activation_recomputation_preserves_two_full_batch_updates_and_rng(device):
    import copy
    from harness.act_training import checkpoint_encoder
    if device == 'cuda' and not torch.cuda.is_available(): pytest.skip('CUDA unavailable')
    torch.set_num_threads(2); torch.manual_seed(21)
    initial = make_policy(128, 4).to(device)
    for parameter in initial.model.backbone.parameters():
        parameter.requires_grad_(False)
    native = {k: v.to(device) for k, v in actor_batch(frames(4), 128, 4).items()}
    initial.eval()
    with torch.no_grad():
        batch = {k: v.repeat(32, 1) if v.ndim == 2 else initial.model.backbone(v)['feature_map'].repeat(32, 1, 1, 1)
                 for k, v in native.items()}
    batch.update(action=torch.zeros(32, 8, 4, device=device), action_is_pad=torch.zeros(32, 8, dtype=torch.bool, device=device))
    results = []
    for enabled in (False, True):
        policy = copy.deepcopy(initial)
        optimizer = torch.optim.AdamW([p for p in policy.parameters() if p.requires_grad], lr=1e-4)
        torch.manual_seed(81); losses = []; gradients = []
        for _ in range(2):
            policy.train(); optimizer.zero_grad()
            with frozen_features(policy), checkpoint_encoder(policy, enabled):
                loss, _ = policy(batch); loss.backward()
            losses.append(loss.item())
            gradients.append({k: p.grad.clone() for k, p in policy.named_parameters() if p.grad is not None})
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 1); optimizer.step()
        rng = [torch.get_rng_state()]
        if device == 'cuda': rng.append(torch.cuda.get_rng_state())
        results.append((losses, gradients, policy.state_dict(), rng))
        assert all('forward' not in layer.__dict__ for layer in policy.model.encoder.layers)
    assert results[0][0] == results[1][0]
    for before, after in zip(results[0][1], results[1][1]):
        for name in before: torch.testing.assert_close(before[name], after[name], atol=0, rtol=0)
    for name in results[0][2]: torch.testing.assert_close(results[0][2][name], results[1][2][name], atol=0, rtol=0)
    assert all(torch.equal(a, b) for a, b in zip(results[0][3], results[1][3]))


def test_cpu_export_batching_preserves_predictions_and_metrics():
    from scripts.train_carry_input_act import cache_images, history_indices, evaluate
    torch.set_num_threads(2); torch.manual_seed(21)
    policy = make_policy(128, 4)
    fs = frames(4) * 8
    rows = [{'id': str(i), 'own_jpeg': f['own_rgb'], 'top_jpeg': f['top_rgb'],
             'context': f['context'], 'action': [.1, 0., 0., 0.], 'done': False} for i, f in enumerate(fs)]
    windows = history_indices([{'sequences': {'r1': rows}}], 4)
    cache = cache_images(policy, rows, 128)
    for parameter in policy.model.backbone.parameters(): parameter.requires_grad_(False)
    before, values_before = evaluate(policy, cache, windows, rows, 32)
    after, values_after = evaluate(policy, cache, windows, rows, 8)
    torch.testing.assert_close(torch.tensor(values_before), torch.tensor(values_after), atol=1e-5, rtol=1e-5)
    assert before['missed_done_rate'] == after['missed_done_rate'] and before['false_done_rate'] == after['false_done_rate']
    assert abs(before['selection_score'] - after['selection_score']) < 1e-5
