"""Fixed-data 2x2 ACT ablation; checkpoint selection uses development only."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from harness.carry_input_act import InputCarryAct, make_policy, image_tensor, metadata
from harness.carry_input_history import window_indices
from harness.act_training import frozen_features
from harness.pair_carry_act import CONTEXT_KEY
from harness.reference_act import IMAGE_KEYS, UPSTREAM_SHA
from scripts.train_carry_act import load


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


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
            parts.append(policy.model.backbone.native(t)['feature_map'].detach())
        result[key] = torch.cat(parts)
    result[CONTEXT_KEY] = torch.tensor([r['context'] for r in rows], dtype=torch.float32)
    return result


def batch_at(cache, windows):
    result = {}
    for key in IMAGE_KEYS:
        # Same per-frame frozen CNN features and order as FrameBackbone.forward.
        value = cache[key][windows]
        result[key] = torch.cat(list(value.unbind(dim=1)), dim=-1)
    context_windows = windows if windows.shape[1] == 4 else windows.repeat(1, 4)
    result[CONTEXT_KEY] = cache[CONTEXT_KEY][context_windows].flatten(1)
    return result


@torch.no_grad()
def evaluate(policy, cache, windows, rows):
    policy.eval()
    pred = []
    with frozen_features(policy):
        for i in range(0, len(rows), 32):
            pred.extend(policy.predict_action_chunk(batch_at(cache, windows[i:i+32]))[:, 0].tolist())
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


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--size', type=int, choices=(128, 256), required=True)
    p.add_argument('--history', type=int, choices=(1, 4), required=True)
    p.add_argument('--steps', type=int, default=8000)
    p.add_argument('--seed', type=int, default=20260921)
    a = p.parse_args()
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT):
        raise ValueError('commit source before training')
    if a.steps <= 0:
        raise ValueError('positive step budget required')
    direct = json.loads(importlib.metadata.distribution('lerobot').read_text('direct_url.json'))
    if direct['vcs_info']['commit_id'] != UPSTREAM_SHA:
        raise ValueError('wrong upstream ACT revision')
    torch.set_num_threads(2); torch.manual_seed(a.seed); np.random.seed(a.seed)
    a.out.mkdir(parents=True, exist_ok=False); started = time.monotonic()
    data = json.loads(a.dataset.read_text())
    if {e['root'] for e in data['train']} & {e['root'] for e in data['development']}:
        raise ValueError('train/development overlap')
    rows, targets, padding = load(data['train']); dev, _, _ = load(data['development'])
    windows = history_indices(data['train'], a.history); dwindows = history_indices(data['development'], a.history)
    policy = make_policy(a.size, a.history, pretrained=True)
    for param in policy.model.backbone.parameters():
        param.requires_grad_(False)
    report = {'complete': False, 'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'adapter': metadata(a.size, a.history), 'dataset_sha256': sha(a.dataset), 'dataset_path': str(a.dataset.resolve()),
              'seed': a.seed, 'steps': a.steps, 'batch_size': 32, 'samples': len(rows),
              'train_episodes': [e['root'] for e in data['train']], 'development_episodes': [e['root'] for e in data['development']],
              'selection': 'original missed_done + false_done + mean direction-group MAE, development only',
              'trainable_parameters': sum(v.numel() for v in policy.parameters() if v.requires_grad),
              'parameters': sum(v.numel() for v in policy.parameters()), 'upstream_sha': UPSTREAM_SHA,
              'environment': {'python': sys.version, 'platform': platform.platform(),
                              **{k: importlib.metadata.version(k) for k in ('torch', 'torchvision', 'lerobot', 'numpy')}},
              'external_model_calls': 0, 'progress': []}
    write(a.out/'report.json', report)
    cache = cache_images(policy, rows, a.size); dcache = cache_images(policy, dev, a.size)
    report['feature_cache_wall_s'] = time.monotonic()-started
    # Verify deployment preprocessing matches cached training input before updates.
    from harness.carry_input_act import actor_batch
    for i in (0, min(4, len(rows)-1), len(rows)-1):
        native = actor_batch(frames_at(rows, windows[i].tolist()), a.size, a.history)
        cached = batch_at(cache, windows[i:i+1])
        with torch.no_grad():
            for key in IMAGE_KEYS:
                torch.testing.assert_close(policy.model.backbone(native[key])['feature_map'], cached[key], rtol=1e-5, atol=1e-5)
    groups = [3 if r['done'] else int(np.argmax(np.abs(r['action'][:3]))) for r in rows]
    counts = {g: groups.count(g) for g in set(groups)}
    report['groups'] = counts
    weights = torch.tensor([1/counts[g] for g in groups], dtype=torch.double)
    generator = torch.Generator().manual_seed(a.seed)
    optimizer = torch.optim.AdamW([v for v in policy.parameters() if v.requires_grad], lr=1e-4, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, a.steps, eta_min=1e-5)
    best, state = float('inf'), None
    for step in range(1, a.steps+1):
        idx = torch.multinomial(weights, 32, replacement=True, generator=generator)
        batch = batch_at(cache, windows[idx]); batch.update(action=targets[idx], action_is_pad=padding[idx])
        policy.train(); optimizer.zero_grad()
        with frozen_features(policy):
            loss, _ = policy(batch)
        if not torch.isfinite(loss):
            raise ValueError('nonfinite training loss')
        loss.backward(); torch.nn.utils.clip_grad_norm_(policy.parameters(), 1); optimizer.step(); scheduler.step()
        if step == 1 or step % 500 == 0 or step == a.steps:
            metrics, _ = evaluate(policy, dcache, dwindows, dev)
            event = {'step': step, 'loss': float(loss.detach()), 'development': metrics, 'elapsed_s': time.monotonic()-started}
            report['progress'].append(event)
            if metrics['selection_score'] < best:
                best = metrics['selection_score']; report['selected'] = event
                state = {k: v.detach().cpu().clone() for k, v in policy.state_dict().items()}
            write(a.out/'report.json', report); print(json.dumps(event), flush=True)
    policy.load_state_dict(state)
    actor = InputCarryAct(policy, a.size, a.history); actor.save(a.out/'act')
    restored = InputCarryAct.load(a.out/'act')
    report['readback'] = []
    for rs, ws in ((rows, windows), (dev, dwindows)):
        for i in (0, len(rs)-1):
            frames = frames_at(rs, ws[i].tolist()); decision = actor.predict(frames)
            if restored.predict(frames) != decision:
                raise ValueError('checkpoint readback mismatch')
            report['readback'].append({'id': rs[i]['id'], 'history_ids': [rs[j]['id'] for j in ws[i]], 'decision': decision})
    for name, rs, cs, ws in (('train', rows, cache, windows), ('development', dev, dcache, dwindows)):
        metrics, pred = evaluate(policy, cs, ws, rs); report[name+'_metrics'] = metrics
        write(a.out/(name+'-predictions.json'), [{'id': r['id'], 'target': r['action'], 'prediction': v} for r, v in zip(rs, pred)])
    report.update(complete=True, wall_s=time.monotonic()-started, model_sha256=sha(a.out/'act/model.safetensors'))
    write(a.out/'report.json', report)


if __name__ == '__main__':
    main()
