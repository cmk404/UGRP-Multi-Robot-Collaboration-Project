"""Fixed-data 2x2 ACT ablation; checkpoint selection uses development only."""
import argparse
import hashlib
import importlib.metadata
import json
import math
from functools import lru_cache
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
from harness.carry_termination_objective import checkpoint_rank, deployed_first_action_done_loss
from harness.pair_carry_act import CONTEXT_KEY
from harness.reference_act import IMAGE_KEYS, UPSTREAM_SHA
from scripts.train_carry_act import load
from scripts.carry_training_objective import sampling_groups, selection
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
def cache_images(policy, rows, size, *, windows=None, mode='batched32'):
    """Cache frozen CNN features with a declared frame-batching contract.

    deployed_window computes each sample's chronological image window in the
    same one-sample frame batch used by FrameBackbone at inference. In
    particular, history=4 means a native CNN batch of four, not a batch of
    32 unrelated frames. No target action or referee value enters this path.
    """
    if mode not in ('batched32', 'deployed_window'):
        raise ValueError('unknown feature cache mode')
    if mode == 'deployed_window':
        if windows is None or windows.ndim != 2 or windows.shape[0] != len(rows) or windows.shape[1] not in (1, 4):
            raise ValueError('deployed_window needs one history window per row')
        if not torch.equal(windows[:, -1], torch.arange(len(rows))):
            raise ValueError('deployed_window windows must end at their own row')
        if bool(((windows < 0) | (windows >= len(rows)) | (windows > windows[:, -1:])).any()):
            raise ValueError('invalid deployed_window history index')
    result = {}
    policy.eval()
    device = next(policy.parameters()).device
    for key, field in zip(IMAGE_KEYS, ('own_jpeg', 'top_jpeg')):
        cached = None
        if mode == 'deployed_window':
            # Adjacent windows overlap. Bound decoded-image reuse without
            # changing the native CNN batch shape or carrying a large RGB cache.
            @lru_cache(maxsize=16)
            def decoded(index):
                return image_tensor(rows[index][field], size)
            for i, indices in enumerate(windows.tolist()):
                frames = torch.stack([decoded(j) for j in indices]).to(device)
                features = policy.model.backbone.native(frames)['feature_map'].detach().cpu()
                if cached is None:
                    cached = torch.empty((len(rows), *features.shape), dtype=features.dtype)
                cached[i].copy_(features)
            decoded.cache_clear()
        else:
            for i in range(0, len(rows), 32):
                t = torch.stack([image_tensor(r[field], size) for r in rows[i:i+32]])
                features = policy.model.backbone.native(t.to(device))['feature_map'].detach().cpu()
                if cached is None:
                    cached = torch.empty((len(rows), *features.shape[1:]), dtype=features.dtype)
                cached[i:i+len(features)].copy_(features)
        if cached is None:
            raise ValueError('feature cache requires nonempty rows')
        result[key] = cached
    result[CONTEXT_KEY] = torch.tensor([r['context'] for r in rows], dtype=torch.float32)
    result['_feature_cache_mode'] = mode
    return result


def batch_at(cache, windows, device="cpu"):
    result = {}
    for key in IMAGE_KEYS:
        # Same per-frame frozen CNN features and order as FrameBackbone.forward.
        value = (cache[key][windows[:, -1]] if cache.get('_feature_cache_mode') == 'deployed_window'
                 else cache[key][windows])
        result[key] = torch.cat(list(value.unbind(dim=1)), dim=-1)
    context_windows = windows if windows.shape[1] == 4 else windows.repeat(1, 4)
    result[CONTEXT_KEY] = cache[CONTEXT_KEY][context_windows].flatten(1)
    return {k: v.to(device) for k, v in result.items()}


@torch.no_grad()
def evaluate(policy, cache, windows, rows, batch_size=32, objective='legacy'):
    policy.eval()
    pred = []
    with frozen_features(policy):
        for i in range(0, len(rows), batch_size):
            pred.extend(policy.predict_action_chunk(batch_at(cache, windows[i:i+batch_size], next(policy.parameters()).device))[:, 0].tolist())
    return metrics_from_first_actions(rows, pred, objective), pred


def metrics_from_first_actions(rows, pred, objective):
    target = np.array([r['action'] for r in rows]); values = np.array(pred)
    done, ready = target[:, 3] > .5, values[:, 3] >= .65
    groups = np.array([3 if r['done'] else int(np.argmax(np.abs(r['action'][:3]))) for r in rows])
    mae = {str(g): float(np.abs(values[groups == g, :3] - target[groups == g, :3]).mean()) for g in set(groups)}
    missed = float((done & ~ready).sum()/max(1, done.sum()))
    false = float((~done & ready).sum()/max(1, (~done).sum()))
    mean_mae=float(np.mean(list(mae.values())))
    selection_objective = 'episode' if objective == 'deployed_first_action' else objective
    return {'missed_done_rate': missed, 'false_done_rate': false, 'group_normalized_mae': mae,
            **selection(rows,pred,mean_mae,missed+false+mean_mae,selection_objective),
            'samples': len(rows), 'done_samples': int(done.sum())}


def frames_at(rows, indices):
    return [{'own_rgb': rows[i]['own_jpeg'], 'top_rgb': rows[i]['top_jpeg'],
             'context': rows[i]['context']} for i in indices]


def memo_readback_indices(rows, windows):
    """Include adjacent windows with at least one genuinely new RGB image."""
    if not rows or windows.shape[0] != len(rows):
        raise ValueError('memo readback requires aligned nonempty rows/windows')
    probes = {0, len(rows)-1, *range(min(4, len(rows)))}
    if windows.shape[1] == 4:
        for i in range(1, len(rows)):
            if int(windows[i, -2]) != i-1:
                continue  # Never cross an episode or model-slot boundary.
            previous = windows[i-1].tolist()
            if any(rows[i][field] not in (rows[j][field] for j in previous)
                   for field in ('own_jpeg', 'top_jpeg')):
                probes.update((i-1, i))
                break
    return sorted(probes)


def decoded_prediction_close(actual, expected):
    """Exact decision schema, original 1e-5 tolerance for decoded numbers."""
    if (set(actual) != {'action', 'done', 'stop_score'} or
            set(expected) != set(actual) or
            set(actual['action']) != {'forward', 'left', 'turn'} or
            set(expected['action']) != set(actual['action']) or
            type(actual['done']) is not bool or type(expected['done']) is not bool or
            actual['done'] != expected['done']):
        return False
    return all(math.isclose(actual['action'][axis], expected['action'][axis],
                            rel_tol=1e-5, abs_tol=1e-5)
               for axis in ('forward', 'left', 'turn')) and math.isclose(
                   actual['stop_score'], expected['stop_score'], rel_tol=1e-5, abs_tol=1e-5)


def validated_partial_overlap(history, previous_probe, index, windows, hits, misses):
    return (history == 4 and previous_probe == index-1 and
            int(windows[index, -2]) == index-1 and hits > 0 and misses > 0)


def classify_cached_action_chunk(actual, expected):
    """Classify all cache guards without hiding the old whole-chunk failure.

    Historical verification applied 1e-5 to the whole eight-action chunk. A
    frozen CNN encoded at batch 32 can differ from deployment's batch 4 in
    float32. Only chunk[0] is issued, and it retains the original 1e-5
    tolerance. Every future done decision must stay unchanged and the full
    numerical chunk remains bounded at 1e-4.
    """
    if actual.shape != expected.shape or actual.ndim != 3 or actual.shape[1:] != (8, 4):
        raise ValueError('ACT cached/native chunks must both be [batch, 8, 4]')
    if not torch.isfinite(actual).all() or not torch.isfinite(expected).all():
        raise ValueError('nonfinite cached/native ACT action chunk')
    actual_done = actual[:, :, 3] >= .65
    expected_done = expected[:, :, 3] >= .65
    first_action_close = torch.isclose(actual[:, 0], expected[:, 0], rtol=1e-5, atol=1e-5)
    original_full_close = torch.isclose(actual, expected, rtol=1e-5, atol=1e-5)
    bounded_full_close = torch.isclose(actual, expected, rtol=1e-4, atol=1e-4)
    difference = (actual - expected).abs()
    return {
        'first_action_original_strict_guard_passed': bool(torch.all(first_action_close)),
        'first_action_max_abs': float(difference[:, 0].max()),
        'full_chunk_original_strict_guard_passed': bool(torch.all(original_full_close)),
        'full_chunk_bounded_guard_passed': bool(torch.all(bounded_full_close)),
        'full_chunk_max_abs': float(difference.max()),
        'all_chunk_done_decisions_same': bool(torch.equal(actual_done, expected_done)),
        'first_action_strict_mismatch_count': int((~first_action_close).sum()),
        'full_chunk_original_strict_mismatch_count': int((~original_full_close).sum()),
        'full_chunk_bounded_mismatch_count': int((~bounded_full_close).sum()),
        'done_decision_flip_count': int((actual_done != expected_done).sum()),
        'original_full_chunk_strict_mismatch_indices_first64': torch.nonzero(
            ~original_full_close)[:64].tolist(),
    }


def compare_cached_action_chunk(actual, expected):
    classified = classify_cached_action_chunk(actual, expected)
    if not classified['all_chunk_done_decisions_same']:
        raise ValueError('cached/native ACT done decision changed in action chunk')
    if not classified['first_action_original_strict_guard_passed']:
        raise ValueError('deployed first ACT action exceeds original 1e-5 cache guard')
    if not classified['full_chunk_bounded_guard_passed']:
        raise ValueError('ACT action chunk exceeds bounded 1e-4 cache guard')
    return classified


@torch.no_grad()
def verify_cache(policy, rows, windows, cache, size, history):
    """Compare separately batched CNN execution and the complete action chunk.

    The old whole-chunk 1e-5 guard is always reported, and is required for
    deployed_window. Legacy batch-32 cache keeps its existing first-action
    1e-5/full-chunk 1e-4 and all-chunk done-decision guards.
    """
    policy.eval()
    native_window_cache = cache.get('_feature_cache_mode') == 'deployed_window'
    result = {'feature_atol': 0. if native_window_cache else 5e-4,
              'feature_rtol': 0. if native_window_cache else 1e-5,
              'first_action_atol': 1e-5, 'first_action_rtol': 1e-5,
              'original_full_chunk_atol': 1e-5, 'original_full_chunk_rtol': 1e-5,
              'bounded_full_chunk_atol': 1e-4, 'bounded_full_chunk_rtol': 1e-4,
              'done_threshold': .65,
              'original_full_chunk_guard_enforced': native_window_cache,
              'probes': []}
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
        comparison = compare_cached_action_chunk(actual, expected)
        if result['original_full_chunk_guard_enforced'] and not comparison['full_chunk_original_strict_guard_passed']:
            raise ValueError('deployed_window ACT chunk exceeds original 1e-5 cache guard')
        result['probes'].append({'id': rows[i]['id'], 'feature_max_abs': errors,
                                **comparison})
    return result


@torch.inference_mode()
def verify_all_deployed_windows(policy, rows, windows, cache, size, history, objective):
    """Audit every selected action against the actual one-sample RGB path.

    This runs only after the fixed optimization budget and checkpoint selection.
    It cannot choose another checkpoint. Both sides use one policy sample per
    forward; the cache side substitutes only precomputed frozen CNN features.
    The original 1e-5 tolerance applies to the *entire* ACT chunk.
    """
    if cache.get('_feature_cache_mode') != 'deployed_window':
        raise ValueError('all-row deployed verification requires deployed_window cache')
    policy.eval()
    device = next(policy.parameters()).device
    with frozen_features(policy):
        cached = [policy.predict_action_chunk(batch_at(cache, windows[i:i+1], device))[0].cpu()
                  for i in range(len(rows))]
    native = []
    for i in range(len(rows)):
        batch = actor_batch(frames_at(rows, windows[i].tolist()), size, history)
        native.append(policy.predict_action_chunk({k: value.to(device) for k, value in batch.items()})[0].cpu())
    cached_chunks, native_chunks = torch.stack(cached), torch.stack(native)
    guards = classify_cached_action_chunk(cached_chunks, native_chunks)
    close = torch.isclose(cached_chunks, native_chunks, rtol=1e-5, atol=1e-5)
    changed = (cached_chunks[:, :, 3] >= .65) != (native_chunks[:, :, 3] >= .65)
    guards['full_chunk_original_strict_mismatch_ids_first64'] = [
        rows[i]['id'] for i in torch.nonzero(~close.flatten(1).all(dim=1))[:64, 0].tolist()]
    guards['done_decision_flip_ids_first64'] = [
        rows[i]['id'] for i in torch.nonzero(changed.any(dim=1))[:64, 0].tolist()]
    cache_first = cached_chunks[:, 0].tolist()
    native_first = native_chunks[:, 0].tolist()
    return {
        'rows': len(rows), 'cache_mode': cache['_feature_cache_mode'],
        'cache_metrics': metrics_from_first_actions(rows, cache_first, objective),
        'native_metrics': metrics_from_first_actions(rows, native_first, objective),
        'guards': guards,
    }, [{'id': row['id'], 'target': row['action'],
         'cache_chunk': c, 'native_chunk': n}
        for row, c, n in zip(rows, cached_chunks.tolist(), native_chunks.tolist())]



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
    p.add_argument('--feature-cache-mode', choices=('batched32', 'deployed_window'), default='batched32',
                   help='deployed_window encodes each sample history in the native inference frame batch')
    p.add_argument('--steps', type=int, default=8000)
    p.add_argument('--seed', type=int, default=20260921)
    p.add_argument('--termination-objective',choices=('legacy','episode','deployed_first_action'),default='episode',
                   help='deployed_first_action: add class-balanced raw-score MSE to the zero-latent first ACT action')
    p.add_argument('--deployed-done-weight',type=float,
                   help='required positive auxiliary-loss weight only for deployed_first_action')
    p.add_argument('--device', choices=('cpu', 'cuda'), default='cpu')
    p.add_argument('--resume', action='store_true')
    p.add_argument('--checkpoint-encoder', action='store_true', help='Recompute encoder activations; keep the full training batch and RNG')
    p.add_argument('--cpu-evaluation-batch-size', type=int, default=32, choices=(1, 8, 32),
                   help='Final CPU prediction batch; deployed_window requires 1 and also selects checkpoints at batch 1')
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
    if a.feature_cache_mode == 'deployed_window' and a.cpu_evaluation_batch_size != 1:
        raise ValueError('deployed_window requires --cpu-evaluation-batch-size 1 for deployment-matched selection/export')
    deployed_done = a.termination_objective == 'deployed_first_action'
    if deployed_done:
        if a.deployed_done_weight is None or not math.isfinite(a.deployed_done_weight) or a.deployed_done_weight <= 0:
            raise ValueError('deployed_first_action requires a finite positive --deployed-done-weight')
    elif a.deployed_done_weight is not None:
        raise ValueError('--deployed-done-weight applies only to deployed_first_action')
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
              'feature_cache_mode': a.feature_cache_mode,
              'feature_cache_native_frame_batch': a.history if a.feature_cache_mode == 'deployed_window' else 32,
              'selection_evaluation_batch_size': 1 if a.feature_cache_mode == 'deployed_window' else 32,
              'activation_checkpointing': a.checkpoint_encoder,
              'cpu_evaluation_batch_size': a.cpu_evaluation_batch_size,
              'train_episodes': [e['root'] for e in data['train']], 'development_episodes': [e['root'] for e in data['development']],
              'selection': ('episode premature-hold + missed-terminal fractions + mean direction-group MAE'
                            if a.termination_objective=='episode' else 'legacy frame rates + mean direction-group MAE'),
              'termination_objective':a.termination_objective,
              'hard_negative_window_steps':8 if a.termination_objective in ('episode','deployed_first_action') else 0,
              'trainable_parameters': sum(v.numel() for v in policy.parameters() if v.requires_grad),
              'parameters': sum(v.numel() for v in policy.parameters()), 'upstream_sha': UPSTREAM_SHA,
              'environment': {'python': sys.version, 'platform': platform.platform(),
                              'device': a.device, 'cuda': torch.version.cuda,
                              'upstream_modeling_sha256': sha(Path(upstream_act.__file__)),
                              'gpu': torch.cuda.get_device_name() if a.device == 'cuda' else None,
                              **{k: importlib.metadata.version(k) for k in ('torch', 'torchvision', 'lerobot', 'numpy')}},
              'external_model_calls': 0, 'progress': []}
    if deployed_done:
        report['selection'] = ('development offline termination pass first, then episode score; '
                               'if no passing checkpoint exists, retain the lowest-failure diagnostic checkpoint')
        report['deployed_done_objective'] = {
            'version': 1, 'weight': a.deployed_done_weight,
            'loss': 'class-balanced mean squared error on raw first-action done score toward 0/1',
            'input': 'own/top frozen RGB features and authored/own-command context only',
            'latent': 'zero, same eval branch as deployment; target actions excluded from auxiliary forward',
            'base_act_loss': 'unchanged upstream chunk L1 plus configured VAE KL',
            'runtime_score_threshold': 0.65, 'runtime_threshold_changed': False,
        }
    if not a.resume:
        write(a.out/'report.json', report)
    feature_cache_started = time.monotonic()
    cache = cache_images(policy, rows, a.size, windows=windows, mode=a.feature_cache_mode)
    dcache = cache_images(policy, dev, a.size, windows=dwindows, mode=a.feature_cache_mode)
    report['feature_cache_only_wall_s'] = time.monotonic()-feature_cache_started
    report['feature_cache_wall_s'] = time.monotonic()-started
    report['initial_cache_verification'] = verify_cache(policy, rows, windows, cache, a.size, a.history)
    if not a.resume:
        write(a.out/'report.json', report)
    groups = sampling_groups(rows,'episode' if deployed_done else a.termination_objective)
    counts = {g: groups.count(g) for g in set(groups)}
    report['groups'] = counts
    weights = torch.tensor([1/counts[g] for g in groups], dtype=torch.double)
    generator = torch.Generator().manual_seed(a.seed)
    optimizer = torch.optim.AdamW([v for v in policy.parameters() if v.requires_grad], lr=1e-4, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, a.steps, eta_min=1e-5)
    signature = {k: report[k] for k in ('source_sha', 'dataset_sha256', 'adapter', 'seed', 'steps', 'batch_size',
                 'upstream_sha', 'environment', 'activation_checkpointing', 'termination_objective',
                 'feature_cache_mode', 'feature_cache_native_frame_batch', 'selection_evaluation_batch_size')}
    if deployed_done:
        signature['deployed_done_objective'] = report['deployed_done_objective']
    best, state, start_step, elapsed_before = ((float('inf'),)*3 if deployed_done else float('inf')), None, 0, 0.0
    if a.resume:
        saved = restore_checkpoint(a.out/'resume.pt', signature=signature, policy=policy, optimizer=optimizer, scheduler=scheduler, generator=generator)
        best, state, start_step = saved['best'], saved['best_state'], saved['step']
        report['progress'], report['selected'] = saved['progress'], saved['selected']
        if deployed_done and report['progress']:
            report['first_batch_deployed_done'] = report['progress'][0]['first_batch_deployed_done']
        elapsed_before = saved['elapsed_s']
        report.update(completed_steps=start_step, wall_s=elapsed_before)
    end_step = a.stop_after_step or a.steps
    if end_step < start_step:
        raise ValueError('stop step precedes saved checkpoint')
    optimizer_started = time.monotonic()
    for step in range(start_step+1, end_step+1):
        idx = torch.multinomial(weights, 32, replacement=True, generator=generator)
        batch = batch_at(cache, windows[idx], a.device); batch.update(action=targets[idx].to(a.device), action_is_pad=padding[idx].to(a.device))
        policy.train(); optimizer.zero_grad()
        with frozen_features(policy), checkpoint_encoder(policy, a.checkpoint_encoder):
            act_loss, act_components = policy(batch)
            if deployed_done:
                done_loss, done_counts, _ = deployed_first_action_done_loss(policy, batch)
                loss = act_loss + a.deployed_done_weight * done_loss
                if step == 1:
                    # Verify that the actual first optimizer batch can
                    # differentiate the deployed done output through ACT.
                    done_grad = torch.autograd.grad(
                        done_loss, policy.model.action_head.weight, retain_graph=True)[0][3].norm()
                    if not torch.isfinite(done_grad) or done_grad <= 0:
                        raise ValueError('zero-latent first-action done gradient missing')
                    report['first_batch_deployed_done'] = {
                        **done_counts, 'action_head_done_gradient_norm': float(done_grad.detach())}
            else:
                loss = act_loss
            if not torch.isfinite(loss):
                raise ValueError('nonfinite training loss')
            # Backbone/encoder patches must remain active during recomputation.
            loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 1); optimizer.step(); scheduler.step()
        if step == 1 or step % 500 == 0 or step == a.steps:
            metrics, _ = evaluate(policy, dcache, dwindows, dev,
                                  batch_size=report['selection_evaluation_batch_size'],
                                  objective=a.termination_objective)
            event = {'step': step, 'loss': float(loss.detach()), 'development': metrics, 'elapsed_s': elapsed_before+time.monotonic()-started}
            if deployed_done:
                event['loss_components'] = {
                    'act_total': float(act_loss.detach()),
                    **{key: float(value) for key, value in act_components.items()},
                    'deployed_done_balanced_raw_mse': float(done_loss.detach()),
                    'weighted_deployed_done': float((a.deployed_done_weight * done_loss).detach()),
                    'done_positive_in_batch': done_counts['positive'],
                    'done_negative_in_batch': done_counts['negative'],
                }
                event['selection_eligible'] = bool(metrics['offline_termination_pass'])
                candidate = checkpoint_rank(metrics, a.termination_objective)
                event['candidate_rank'] = list(candidate)
                if step == 1:
                    event['first_batch_deployed_done'] = report['first_batch_deployed_done']
            else:
                candidate = checkpoint_rank(metrics, a.termination_objective)
            report['progress'].append(event)
            if candidate < best:
                best = candidate; report['selected'] = event
                state = {k: v.detach().cpu().clone() for k, v in policy.state_dict().items()}
            write(a.out/'report.json', report); print(json.dumps(event), flush=True)
        if step == 1 or step % 500 == 0 or step == end_step:
            elapsed = elapsed_before+time.monotonic()-started
            save_checkpoint(a.out/'resume.pt', signature=signature, step=step, policy=policy, optimizer=optimizer, scheduler=scheduler, generator=generator, best=best, selected=report.get('selected'), best_state=state, progress=report['progress'], elapsed_s=elapsed)
            report.update(completed_steps=step, wall_s=elapsed)
            write(a.out/'report.json', report)
    report['optimizer_loop_wall_s_this_process'] = time.monotonic()-optimizer_started
    if end_step < a.steps:
        report['paused_cache_verification'] = verify_cache(policy, rows, windows, cache, a.size, a.history)
        report['deployment_max_abs'] = {
            'train': verify_cpu_deployment(policy, rows, windows, a.size, a.history),
            'development': verify_cpu_deployment(policy, dev, dwindows, a.size, a.history)}
        report['paused'] = True
        write(a.out/'report.json', report)
        return
    policy.load_state_dict(state)
    if deployed_done:
        report['selected_checkpoint_eligible'] = bool(report['selected']['selection_eligible'])
        report['physical_success_claim'] = False
    # Training has ended; release its tensors before CPU deployment/export.
    optimizer.zero_grad(set_to_none=True)
    del optimizer, scheduler, state
    if end_step > start_step:
        del batch, loss
    report['selected_cache_verification'] = {
        'train': verify_cache(policy, rows, windows, cache, a.size, a.history),
        'development': verify_cache(policy, dev, dwindows, dcache, a.size, a.history)}
    report['deployment_max_abs'] = {
        'train': verify_cpu_deployment(policy, rows, windows, a.size, a.history),
        'development': verify_cpu_deployment(policy, dev, dwindows, a.size, a.history)}
    policy.cpu()
    all_row_predictions = {}
    if a.feature_cache_mode == 'deployed_window':
        report['selected_all_row_native_verification'] = {}
        report['selected_all_row_native_verification_wall_s'] = {}
        for split, rs, ws, features in (('train', rows, windows, cache),
                                        ('development', dev, dwindows, dcache)):
            verified_started = time.monotonic()
            verified, predictions = verify_all_deployed_windows(
                policy, rs, ws, features, a.size, a.history, a.termination_objective)
            report['selected_all_row_native_verification'][split] = verified
            report['selected_all_row_native_verification_wall_s'][split] = time.monotonic()-verified_started
            all_row_predictions[split] = predictions
            write(a.out/(split+'-native-vs-cache-predictions.json'), predictions)
            write(a.out/'report.json', report)
        for split, verified in report['selected_all_row_native_verification'].items():
            guards = verified['guards']
            if not guards['full_chunk_original_strict_guard_passed']:
                raise ValueError(f'{split} all-row deployed ACT chunk exceeds original 1e-5 guard')
            if not guards['all_chunk_done_decisions_same']:
                raise ValueError(f'{split} all-row deployed ACT done decision changed')
    export = Path(tempfile.mkdtemp(prefix='.act-export-', dir=a.out))/'act'
    actor = InputCarryAct(policy, a.size, a.history); actor.save(export)
    restored = InputCarryAct.load(export)
    memoized = InputCarryAct.load(export, cache_features=True) if a.feature_cache_mode == 'deployed_window' else None
    report['readback'] = []
    if memoized is not None:
        if not all(torch.equal(value, memoized.policy.state_dict()[key])
                   for key, value in actor.policy.state_dict().items()):
            raise ValueError('memoized runtime artifact weights differ from selected policy')
        report['memoized_runtime_readback'] = []
    observed_partial_overlap = 0
    for rs, ws in ((rows, windows), (dev, dwindows)):
        # Preserve model and image-cache state across real successive windows.
        # A one-frame advance computes only missing CNN features in the
        # runtime worker, so isolated first/end probes cannot test this path.
        probe_indices = (memo_readback_indices(rs, ws) if memoized is not None
                         else sorted({0, len(rs)-1}))
        previous_probe = None
        for i in probe_indices:
            frames = frames_at(rs, ws[i].tolist()); decision = actor.predict(frames)
            if restored.predict(frames) != decision:
                raise ValueError('checkpoint readback mismatch')
            report['readback'].append({'id': rs[i]['id'], 'history_ids': [rs[j]['id'] for j in ws[i]], 'decision': decision})
            if memoized is not None:
                batch = actor_batch(frames, a.size, a.history)
                hits_before = memoized.policy.model.backbone.cache_hits
                misses_before = memoized.policy.model.backbone.cache_misses
                with torch.inference_mode():
                    native_raw = actor.policy.predict_action_chunk(batch)[0, 0]
                    cached_raw = memoized.policy.predict_action_chunk(batch)[0, 0]
                hits_delta = memoized.policy.model.backbone.cache_hits - hits_before
                misses_delta = memoized.policy.model.backbone.cache_misses - misses_before
                partial_overlap = validated_partial_overlap(
                    a.history, previous_probe, i, ws, hits_delta, misses_delta)
                observed_partial_overlap += int(partial_overlap)
                if not torch.isfinite(cached_raw).all() or not torch.all(torch.isclose(
                        native_raw, cached_raw, rtol=1e-5, atol=1e-5)):
                    raise ValueError('memoized runtime first action exceeds original 1e-5 guard')
                if bool(native_raw[3] >= .65) != bool(cached_raw[3] >= .65):
                    raise ValueError('memoized runtime done decision changed')
                memoized_decision = memoized.predict(frames)
                if (not decoded_prediction_close(memoized_decision, decision) or
                        memoized.predict(frames) != memoized_decision):
                    raise ValueError('memoized runtime readback mismatch')
                report['memoized_runtime_readback'].append({
                    'id': rs[i]['id'], 'history_ids': [rs[j]['id'] for j in ws[i]],
                    'first_action_max_abs': float((native_raw-cached_raw).abs().max()),
                    'native_done': bool(native_raw[3] >= .65),
                    'memoized_done': bool(cached_raw[3] >= .65),
                    'feature_cache_hits': memoized.policy.model.backbone.cache_hits,
                    'partial_window_feature_cache_hits': hits_delta,
                    'partial_window_feature_cache_misses': misses_delta,
                    'validated_partial_overlap': partial_overlap})
            previous_probe = i
    if memoized is not None and memoized.policy.model.backbone.cache_hits <= 0:
        raise ValueError('memoized runtime readback never used feature cache')
    if memoized is not None:
        report['memoized_runtime_validated_partial_overlap_count'] = observed_partial_overlap
        if a.history == 4 and observed_partial_overlap <= 0:
            raise ValueError('memoized runtime readback never validated a partial-overlap window')
    del restored, memoized
    for name, rs, cs, ws in (('train', rows, cache, windows), ('development', dev, dcache, dwindows)):
        if a.feature_cache_mode == 'deployed_window':
            metrics = report['selected_all_row_native_verification'][name]['cache_metrics']
            pred = [row['cache_chunk'][0] for row in all_row_predictions.pop(name)]
        else:
            metrics, pred = evaluate(policy, cs, ws, rs, batch_size=a.cpu_evaluation_batch_size,
                                     objective=a.termination_objective)
        report[name+'_metrics'] = metrics
        write(a.out/(name+'-predictions.json'), [{'id': r['id'], 'target': r['action'], 'prediction': v} for r, v in zip(rs, pred)])
    # A crash can leave a partial/previous export. Preserve it before publishing
    # the verified candidate; repeated finalization does not destroy evidence.
    if (a.out/'act').exists():
        (a.out/'act').rename(a.out/('act-interrupted-'+uuid.uuid4().hex[:8]))
    export.rename(a.out/'act')
    export.parent.rmdir()
    report.update(complete=True, wall_s=elapsed_before+time.monotonic()-started, model_sha256=sha(a.out/'act/model.safetensors'))
    if source_identity() != source_sha or sha(a.dataset) != report['dataset_sha256']:
        raise ValueError('training source or dataset changed before complete artifact')
    write(a.out/'report.json', report)


if __name__ == '__main__':
    main()
