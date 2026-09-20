"""Fixed-data 2x2 ACT ablation; checkpoint selection uses development only."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import sys
import time
import tempfile
import uuid
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from harness.carry_input_act import InputCarryAct, make_policy, image_tensor, metadata, actor_batch
from harness.carry_input_history import window_indices
from harness.act_training import frozen_features, checkpoint_encoder
from harness.pair_carry_act import CONTEXT_KEY
from harness.reference_act import IMAGE_KEYS, UPSTREAM_SHA
from scripts.train_carry_act import load
from scripts.colab_carry_bundle import source_identity, verify_dataset
from harness.carry_training_checkpoint import save_checkpoint, restore_checkpoint
from scripts.patch_reference_act import BEFORE, AFTER, ORIGINAL_SHA256
import lerobot.policies.act.modeling_act as upstream_act


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def history_indices(entries, history):
    indices, offset = [], 0
    for episode in entries:
        for sequence in episode['sequences'].values():
            indices.extend([[offset + j for j in window_indices(i, history)] for i in range(len(sequence))])
            offset += len(sequence)
    return torch.tensor(indices, dtype=torch.long)


@torch.no_grad()
def cache_images(policy, rows, size):
    result = {}
    policy.eval()
    for key, field in zip(IMAGE_KEYS, ('own_jpeg', 'top_jpeg')):
        parts = []
        for i in range(0, len(rows), 32):
            t = torch.stack([image_tensor(r[field], size) for r in rows[i:i+32]])
            parts.append(policy.model.backbone.native(t.to(next(policy.parameters()).device))['feature_map'].detach().cpu())
        result[key] = torch.cat(parts)
    result[CONTEXT_KEY] = torch.tensor([r['context'] for r in rows], dtype=torch.float32)
    return result


def batch_at(cache, windows, device="cpu"):
    result = {}
    for key in IMAGE_KEYS:
        # Same per-frame frozen CNN features and order as FrameBackbone.forward.
        value = cache[key][windows]
        result[key] = torch.cat(list(value.unbind(dim=1)), dim=-1)
    context_windows = windows if windows.shape[1] == 4 else windows.repeat(1, 4)
    result[CONTEXT_KEY] = cache[CONTEXT_KEY][context_windows].flatten(1)
    return {k: v.to(device) for k, v in result.items()}


@torch.no_grad()
def evaluate(policy, cache, windows, rows, batch_size=32):
    policy.eval()
    pred = []
    with frozen_features(policy):
        for i in range(0, len(rows), batch_size):
            pred.extend(policy.predict_action_chunk(batch_at(cache, windows[i:i+batch_size], next(policy.parameters()).device))[:, 0].tolist())
    target = np.array([r['action'] for r in rows]); values = np.array(pred)
    done, ready = target[:, 3] > .5, values[:, 3] >= .65
    groups = np.array([3 if r['done'] else int(np.argmax(np.abs(r['action'][:3]))) for r in rows])
    mae = {str(g): float(np.abs(values[groups == g, :3] - target[groups == g, :3]).mean()) for g in set(groups)}
    missed = float((done & ~ready).sum()/max(1, done.sum()))
    false = float((~done & ready).sum()/max(1, (~done).sum()))
    return {'missed_done_rate': missed, 'false_done_rate': false, 'group_normalized_mae': mae,
            'selection_score': missed + false + float(np.mean(list(mae.values()))),
            'samples': len(rows), 'done_samples': int(done.sum())}, pred


def frames_at(rows, indices):
    return [{'own_rgb': rows[i]['own_jpeg'], 'top_rgb': rows[i]['top_jpeg'],
             'context': rows[i]['context']} for i in indices]


@torch.no_grad()
def verify_cache(policy, rows, windows, cache, size, history):
    """Compare separately batched CNN execution and the complete action chunk.

    Float32 CNN kernels need not be bit-identical across batch sizes. Keep a
    bounded feature tolerance AND a stricter check of actual policy outputs.
    Run again on the selected checkpoint, not just random initial weights.
    """
    policy.eval()
    result = {'feature_atol': 5e-4, 'feature_rtol': 1e-5,
              'action_atol': 1e-5, 'action_rtol': 1e-5, 'probes': []}
    for i in sorted({0, min(4, len(rows)-1), len(rows)-1}):
        native = actor_batch(frames_at(rows, windows[i].tolist()), size, history)
        device = next(policy.parameters()).device
        native = {k: v.to(device) for k, v in native.items()}
        cached = batch_at(cache, windows[i:i+1], device)
        errors = {}
        for key in IMAGE_KEYS:
            actual = policy.model.backbone(native[key])['feature_map']
            torch.testing.assert_close(actual, cached[key], rtol=result['feature_rtol'], atol=result['feature_atol'])
            errors[key] = float((actual-cached[key]).abs().max())
        expected = policy.predict_action_chunk(native)
        with frozen_features(policy):
            actual = policy.predict_action_chunk(cached)
        torch.testing.assert_close(actual, expected, rtol=result['action_rtol'], atol=result['action_atol'])
        result['probes'].append({'id': rows[i]['id'], 'feature_max_abs': errors,
                                'action_chunk_max_abs': float((actual-expected).abs().max())})
    return result



@torch.no_grad()
def verify_cpu_deployment(policy, rows, windows, size, history):
    device = next(policy.parameters()).device
    # Quantify GPU-to-CPU deployment drift before storing the CPU artifact.
    probes = [actor_batch(frames_at(rows, windows[i].tolist()), size, history) for i in (0, len(rows)-1)]
    policy.eval()
    with torch.no_grad():
        device_outputs = [policy.predict_action_chunk({k: v.to(device) for k, v in b.items()}).cpu() for b in probes]
    policy.cpu()
    errors = []
    with torch.no_grad():
        for b, expected in zip(probes, device_outputs):
            actual = policy.predict_action_chunk(b)
            torch.testing.assert_close(actual, expected, rtol=1e-4, atol=1e-4)
            errors.append(float((actual-expected).abs().max()))
    policy.to(device)
    return errors


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--size', type=int, choices=(128, 256, 512), required=True)
    p.add_argument('--history', type=int, choices=(1, 4), required=True)
    p.add_argument('--steps', type=int, default=8000)
    p.add_argument('--seed', type=int, default=20260921)
    p.add_argument('--device', choices=('cpu', 'cuda'), default='cpu')
    p.add_argument('--resume', action='store_true')
    p.add_argument('--checkpoint-encoder', action='store_true', help='Recompute encoder activations; keep the full training batch and RNG')
    p.add_argument('--cpu-evaluation-batch-size', type=int, default=32, choices=(8, 32), help='Only final CPU prediction export; training/development selection stay batch32')
    p.add_argument('--stop-after-step', type=int, help='Pause at this step without changing the full scheduler budget')
    a = p.parse_args()
    source_sha = source_identity()
    provenance = verify_dataset(a.dataset)
    if a.device == 'cuda' and not torch.cuda.is_available():
        raise ValueError('CUDA requested but unavailable; no CPU fallback')
    if a.stop_after_step is not None and not 1 <= a.stop_after_step <= a.steps:
        raise ValueError('stop-after-step outside full budget')
    if a.steps <= 0:
        raise ValueError('positive step budget required')
    direct = json.loads(importlib.metadata.distribution('lerobot').read_text('direct_url.json'))
    if direct['vcs_info']['commit_id'] != UPSTREAM_SHA:
        raise ValueError('wrong upstream ACT revision')
    upstream_source = Path(upstream_act.__file__).read_text()
    if BEFORE in upstream_source or upstream_source.count(AFTER) != 2 or hashlib.sha256(upstream_source.replace(AFTER, BEFORE).encode()).hexdigest() != ORIGINAL_SHA256:
        raise ValueError('apply scripts/patch_reference_act.py to the pinned ACT environment first')
    torch.set_num_threads(2); torch.manual_seed(a.seed); np.random.seed(a.seed)
    if a.resume and not (a.out/'resume.pt').is_file():
        raise ValueError('resume.pt is required for resume')
    if a.resume and json.loads((a.out/'report.json').read_text()).get('complete'):
        raise ValueError('run already complete')
    a.out.mkdir(parents=True, exist_ok=a.resume); started = time.monotonic()
    # Keep float32 comparison semantics; no mixed precision or TF32.
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    data = json.loads(a.dataset.read_text())
    if {e['root'] for e in data['train']} & {e['root'] for e in data['development']}:
        raise ValueError('train/development overlap')
    rows, targets, padding = load(data['train']); dev, _, _ = load(data['development'])
    windows = history_indices(data['train'], a.history); dwindows = history_indices(data['development'], a.history)
    policy = make_policy(a.size, a.history, pretrained=True).to(a.device)
    for param in policy.model.backbone.parameters():
        param.requires_grad_(False)
    report = {'complete': False, 'source_sha': source_sha,
              'adapter': metadata(a.size, a.history), **provenance, 'dataset_path': str(a.dataset.resolve()),
              'seed': a.seed, 'steps': a.steps, 'batch_size': 32, 'samples': len(rows),
              'activation_checkpointing': a.checkpoint_encoder,
              'cpu_evaluation_batch_size': a.cpu_evaluation_batch_size,
              'train_episodes': [e['root'] for e in data['train']], 'development_episodes': [e['root'] for e in data['development']],
              'selection': 'original missed_done + false_done + mean direction-group MAE, development only',
              'trainable_parameters': sum(v.numel() for v in policy.parameters() if v.requires_grad),
              'parameters': sum(v.numel() for v in policy.parameters()), 'upstream_sha': UPSTREAM_SHA,
              'environment': {'python': sys.version, 'platform': platform.platform(),
                              'device': a.device, 'cuda': torch.version.cuda,
                              'upstream_modeling_sha256': sha(Path(upstream_act.__file__)),
                              'gpu': torch.cuda.get_device_name() if a.device == 'cuda' else None,
                              **{k: importlib.metadata.version(k) for k in ('torch', 'torchvision', 'lerobot', 'numpy')}},
              'external_model_calls': 0, 'progress': []}
    if not a.resume:
        write(a.out/'report.json', report)
    cache = cache_images(policy, rows, a.size); dcache = cache_images(policy, dev, a.size)
    report['feature_cache_wall_s'] = time.monotonic()-started
    report['initial_cache_verification'] = verify_cache(policy, rows, windows, cache, a.size, a.history)
    if not a.resume:
        write(a.out/'report.json', report)
    groups = [3 if r['done'] else int(np.argmax(np.abs(r['action'][:3]))) for r in rows]
    counts = {g: groups.count(g) for g in set(groups)}
    report['groups'] = counts
    weights = torch.tensor([1/counts[g] for g in groups], dtype=torch.double)
    generator = torch.Generator().manual_seed(a.seed)
    optimizer = torch.optim.AdamW([v for v in policy.parameters() if v.requires_grad], lr=1e-4, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, a.steps, eta_min=1e-5)
    signature = {k: report[k] for k in ('source_sha', 'dataset_sha256', 'adapter', 'seed', 'steps', 'batch_size', 'upstream_sha', 'environment', 'activation_checkpointing')}
    best, state, start_step, elapsed_before = float('inf'), None, 0, 0.0
    if a.resume:
        saved = restore_checkpoint(a.out/'resume.pt', signature=signature, policy=policy, optimizer=optimizer, scheduler=scheduler, generator=generator)
        best, state, start_step = saved['best'], saved['best_state'], saved['step']
        report['progress'], report['selected'] = saved['progress'], saved['selected']
        elapsed_before = saved['elapsed_s']
        report.update(completed_steps=start_step, wall_s=elapsed_before)
    end_step = a.stop_after_step or a.steps
    if end_step < start_step:
        raise ValueError('stop step precedes saved checkpoint')
    for step in range(start_step+1, end_step+1):
        idx = torch.multinomial(weights, 32, replacement=True, generator=generator)
        batch = batch_at(cache, windows[idx], a.device); batch.update(action=targets[idx].to(a.device), action_is_pad=padding[idx].to(a.device))
        policy.train(); optimizer.zero_grad()
        with frozen_features(policy), checkpoint_encoder(policy, a.checkpoint_encoder):
            loss, _ = policy(batch)
            if not torch.isfinite(loss):
                raise ValueError('nonfinite training loss')
            # Backbone/encoder patches must remain active during recomputation.
            loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 1); optimizer.step(); scheduler.step()
        if step == 1 or step % 500 == 0 or step == a.steps:
            metrics, _ = evaluate(policy, dcache, dwindows, dev)
            event = {'step': step, 'loss': float(loss.detach()), 'development': metrics, 'elapsed_s': elapsed_before+time.monotonic()-started}
            report['progress'].append(event)
            if metrics['selection_score'] < best:
                best = metrics['selection_score']; report['selected'] = event
                state = {k: v.detach().cpu().clone() for k, v in policy.state_dict().items()}
            write(a.out/'report.json', report); print(json.dumps(event), flush=True)
        if step == 1 or step % 500 == 0 or step == end_step:
            elapsed = elapsed_before+time.monotonic()-started
            save_checkpoint(a.out/'resume.pt', signature=signature, step=step, policy=policy, optimizer=optimizer, scheduler=scheduler, generator=generator, best=best, selected=report.get('selected'), best_state=state, progress=report['progress'], elapsed_s=elapsed)
            report.update(completed_steps=step, wall_s=elapsed)
            write(a.out/'report.json', report)
    if end_step < a.steps:
        report['paused_cache_verification'] = verify_cache(policy, rows, windows, cache, a.size, a.history)
        report['deployment_max_abs'] = {
            'train': verify_cpu_deployment(policy, rows, windows, a.size, a.history),
            'development': verify_cpu_deployment(policy, dev, dwindows, a.size, a.history)}
        report['paused'] = True
        write(a.out/'report.json', report)
        return
    policy.load_state_dict(state)
    report['selected_cache_verification'] = {
        'train': verify_cache(policy, rows, windows, cache, a.size, a.history),
        'development': verify_cache(policy, dev, dwindows, dcache, a.size, a.history)}
    report['deployment_max_abs'] = {
        'train': verify_cpu_deployment(policy, rows, windows, a.size, a.history),
        'development': verify_cpu_deployment(policy, dev, dwindows, a.size, a.history)}
    policy.cpu()
    export = Path(tempfile.mkdtemp(prefix='.act-export-', dir=a.out))/'act'
    actor = InputCarryAct(policy, a.size, a.history); actor.save(export)
    restored = InputCarryAct.load(export)
    report['readback'] = []
    for rs, ws in ((rows, windows), (dev, dwindows)):
        for i in (0, len(rs)-1):
            frames = frames_at(rs, ws[i].tolist()); decision = actor.predict(frames)
            if restored.predict(frames) != decision:
                raise ValueError('checkpoint readback mismatch')
            report['readback'].append({'id': rs[i]['id'], 'history_ids': [rs[j]['id'] for j in ws[i]], 'decision': decision})
    for name, rs, cs, ws in (('train', rows, cache, windows), ('development', dev, dcache, dwindows)):
        metrics, pred = evaluate(policy, cs, ws, rs, batch_size=a.cpu_evaluation_batch_size); report[name+'_metrics'] = metrics
        write(a.out/(name+'-predictions.json'), [{'id': r['id'], 'target': r['action'], 'prediction': v} for r, v in zip(rs, pred)])
    # A crash can leave a partial/previous export. Preserve it before publishing
    # the verified candidate; repeated finalization does not destroy evidence.
    if (a.out/'act').exists():
        (a.out/'act').rename(a.out/('act-interrupted-'+uuid.uuid4().hex[:8]))
    export.rename(a.out/'act')
    export.parent.rmdir()
    report.update(complete=True, wall_s=elapsed_before+time.monotonic()-started, model_sha256=sha(a.out/'act/model.safetensors'))
    write(a.out/'report.json', report)


if __name__ == '__main__':
    main()
