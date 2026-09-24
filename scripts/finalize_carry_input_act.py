"""Recover only the selected artifact from a fully trained, failed ACT run.

This standalone workflow performs no optimizer update or checkpoint selection.
It never writes into the original managed run. A successful exit certifies an
exported diagnostic model and its offline readback, not a successful policy.
"""
import argparse
from functools import lru_cache
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import torch
import lerobot.policies.act.modeling_act as upstream_act

from harness.act_training import frozen_features
from harness.carry_input_act import InputCarryAct, actor_batch, image_tensor, make_policy, metadata
from harness.pair_carry_act_contract import decode
from harness.reference_act import UPSTREAM_SHA
from scripts.colab_carry_bundle import digest, source_identity, verify_dataset
from scripts.patch_reference_act import BEFORE, AFTER, ORIGINAL_SHA256
from scripts.train_carry_act import load
from scripts.train_carry_input_act import (batch_at, cache_images, classify_cached_action_chunk,
                                           frames_at, history_indices, verify_cache,
                                           verify_cpu_deployment)
from scripts.carry_training_objective import selection

TRAINING_SOURCE = '931d910998a94361342c372ec2675c475daa625f'
EXPECTED_DATASET = '9cd42d3b6859fcb1526dab7ee675ba4349cf6fd32537d1c955a696e1e3197a57'
EXPECTED_STEPS = 8000
EXPECTED_SEED = 20260924
EXPECTED_SELECTED_STEP = 3000
EXPECTED_SIZE = 128
EXPECTED_HISTORY = 4
EXPECTED_OBJECTIVE = 'deployed_first_action'
EXPECTED_WEIGHT = 1.0
EXPECTED_BUNDLE = 'rgb-standard-dispatch-v28'
EXPECTED_BUNDLE_SHA = '3414ab8c93995bf8692f71ce4205fb17dd05698c10efc1edab97ae5c81775d25'
MAX_FINALIZATION_WALL_S = 720
INFERENCE_FILES = (
    'harness/carry_input_act.py', 'harness/carry_input_history.py',
    'harness/pair_carry_act.py', 'harness/pair_carry_act_contract.py',
    'harness/reference_act.py', 'scripts/carry_input_worker.py',
)


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def require_digest(path, expected, label):
    require(path.is_file(), f'{label} missing')
    actual = digest(path)
    require(actual == expected, f'{label} SHA-256 mismatch')
    return actual


def state_digest(state):
    """Stable selected-tensor identity independent of torch.save metadata."""
    h = hashlib.sha256()
    for name in sorted(state):
        tensor = state[name].detach().cpu().contiguous()
        h.update(name.encode() + b'\0')
        h.update(str(tensor.dtype).encode() + b'\0')
        h.update(json.dumps(list(tensor.shape)).encode() + b'\0')
        h.update(tensor.numpy().tobytes())
    return h.hexdigest()


def original_paths(failed_run):
    require(failed_run.is_dir(), 'failed managed run directory missing')
    return {'manifest': failed_run/'manifest.json',
            'report': failed_run/'artifacts/report.json',
            'checkpoint': failed_run/'artifacts/resume.pt'}


def verify_original_metadata(paths, expected, dataset, freeze):
    """Check immutable public metadata before loading any model tensor."""
    hashes = {key: require_digest(paths[key], expected[key], f'original {key}') for key in paths}
    hashes['source_freeze'] = require_digest(freeze, expected['source_freeze'], 'source freeze')
    manifest = json.loads(paths['manifest'].read_text())
    report = json.loads(paths['report'].read_text())
    frozen = json.loads(freeze.read_text())
    provenance = verify_dataset(dataset)
    require(frozen['schema'] == 'ugrp.action_act_refinement_freeze.v1', 'wrong source freeze schema')
    require(frozen['refined_source_sha'] == TRAINING_SOURCE, 'wrong frozen training source')
    require(frozen['dataset_sha256'] == EXPECTED_DATASET, 'wrong frozen dataset')
    require(frozen['bundle_id'] == EXPECTED_BUNDLE
            and frozen['bundle_sha256'] == EXPECTED_BUNDLE_SHA
            and digest(ROOT/'config/rgb_execution_bundles'/f'{EXPECTED_BUNDLE}.json') == EXPECTED_BUNDLE_SHA,
            'frozen v28 execution bundle changed')
    require(provenance['dataset_sha256'] == EXPECTED_DATASET, 'wrong dataset bytes')
    require(manifest['workflow_id'] == 'act-input-training', 'original was not managed ACT input training')
    require(manifest['status'] == 'process_failed' and manifest['exit_code'] == 1,
            'original must remain a failed process')
    require(manifest.get('inputs_changed_during_run') is False,
            'original inputs changed during training')
    require(manifest['source']['source_sha'] == TRAINING_SOURCE
            and manifest['source']['source_dirty'] is False
            and manifest.get('source_changed_during_run') is False,
            'original source was not frozen')
    require(report['complete'] is False and report['completed_steps'] == EXPECTED_STEPS,
            'original did not finish full budget before finalization failure')
    require(report['source_sha'] == TRAINING_SOURCE and report['dataset_sha256'] == EXPECTED_DATASET,
            'original report source/dataset mismatch')
    require(Path(report['dataset_path']).resolve() == dataset.resolve(),
            'dataset path differs from original report')
    require(report['adapter'] == metadata(EXPECTED_SIZE, EXPECTED_HISTORY), 'original adapter mismatch')
    require((report['seed'], report['steps'], report['batch_size']) ==
            (EXPECTED_SEED, EXPECTED_STEPS, 32), 'original training schedule mismatch')
    require(report['activation_checkpointing'] is False and report['cpu_evaluation_batch_size'] == 32,
            'original execution configuration mismatch')
    require(report['termination_objective'] == EXPECTED_OBJECTIVE, 'original objective mismatch')
    objective = report['deployed_done_objective']
    require(objective['version'] == 1 and objective['weight'] == EXPECTED_WEIGHT
            and objective['runtime_score_threshold'] == .65
            and objective['runtime_threshold_changed'] is False,
            'original deployed done objective mismatch')
    require(report['selected']['step'] == EXPECTED_SELECTED_STEP
            and report['selected']['selection_eligible'] is False
            and report['selected']['development']['offline_termination_pass'] is False,
            'original selected checkpoint differs from frozen diagnostic selection')
    require(report['progress'] and report['progress'][-1]['step'] == EXPECTED_STEPS,
            'original progress does not reach full budget')
    require('model_sha256' not in report and 'readback' not in report,
            'original report unexpectedly claims exported model')
    require(not (paths['report'].parent/'act/model.safetensors').exists(),
            'original already has an exported model')
    console = paths['manifest'].parent/'console.log'
    log_files = manifest.get('logs', {}).get('files', [])
    require(len(log_files) == 1 and log_files[0]['path'] == 'console.log'
            and digest(console) == log_files[0]['sha256'],
            'original failure console log changed')
    require(Path(manifest['logs']['path']).resolve() == console.resolve(),
            'original failure console path changed')
    text = console.read_text()
    trace = text[text.rfind('Traceback (most recent call last):'):]
    require('verify_cache' in trace and 'Mismatched elements: 6 / 32' in trace
            and '3.8743019104003906e-05' in trace,
            'original was not the recorded selected-cache failure')
    hashes['console'] = log_files[0]['sha256']
    return report, manifest, hashes, trace


def verify_execution_environment(report):
    direct = json.loads(importlib.metadata.distribution('lerobot').read_text('direct_url.json'))
    require(direct['vcs_info']['commit_id'] == UPSTREAM_SHA == report['upstream_sha'],
            'pinned upstream ACT revision changed')
    upstream_source = Path(upstream_act.__file__).read_text()
    require(BEFORE not in upstream_source and upstream_source.count(AFTER) == 2
            and hashlib.sha256(upstream_source.replace(AFTER, BEFORE).encode()).hexdigest() == ORIGINAL_SHA256,
            'pinned upstream ACT patch missing')
    environment = {'python': sys.version, 'platform': platform.platform(), 'device': 'cpu',
                   'cuda': torch.version.cuda, 'upstream_modeling_sha256': digest(Path(upstream_act.__file__)),
                   'gpu': None,
                   **{k: importlib.metadata.version(k) for k in ('torch', 'torchvision', 'lerobot', 'numpy')}}
    require(report['environment'] == environment, 'training environment differs from frozen report')
    require(report['environment']['device'] == 'cpu', 'recovery requires original CPU checkpoint')


def verify_inference_source():
    """Recovery may alter finalization only; actual runtime inference stays v28."""
    require(subprocess.run(['git', 'cat-file', '-e', TRAINING_SOURCE+'^{commit}'],
                           cwd=ROOT, check=False, capture_output=True).returncode == 0,
            'frozen inference commit unavailable')
    files = {}
    for relative in INFERENCE_FILES:
        old = subprocess.check_output(['git', 'show', f'{TRAINING_SOURCE}:{relative}'], cwd=ROOT)
        current = (ROOT/relative).read_bytes()
        require(old == current, f'inference source differs from frozen v28: {relative}')
        files[relative] = hashlib.sha256(current).hexdigest()
    return files


def verify_checkpoint(checkpoint, report):
    signature = {k: report[k] for k in ('source_sha', 'dataset_sha256', 'adapter', 'seed', 'steps',
                                         'batch_size', 'upstream_sha', 'environment',
                                         'activation_checkpointing', 'termination_objective')}
    signature['deployed_done_objective'] = report['deployed_done_objective']
    require(checkpoint['signature'] == signature, 'checkpoint training signature mismatch')
    require(checkpoint['step'] == EXPECTED_STEPS, 'checkpoint did not finish full budget')
    require(checkpoint['selected'] == report['selected'] and checkpoint['progress'] == report['progress'],
            'checkpoint selection/progress differs from original report')
    require(tuple(checkpoint['best']) == tuple(report['selected']['candidate_rank']),
            'saved best rank differs from selected checkpoint')
    require(checkpoint['best_state'] and checkpoint['model'], 'checkpoint model states missing')
    require(math.isfinite(float(checkpoint['elapsed_s']))
            and abs(checkpoint['elapsed_s']-report['wall_s']) < 1,
            'checkpoint elapsed time differs from original report')
    return state_digest(checkpoint['best_state'])


def metrics_from_raw(rows, predictions):
    target = np.array([r['action'] for r in rows])
    values = np.array(predictions)
    done = target[:, 3] > .5
    ready = values[:, 3] >= .65
    groups = np.array([3 if r['done'] else int(np.argmax(np.abs(r['action'][:3]))) for r in rows])
    mae = {str(g): float(np.abs(values[groups == g, :3]-target[groups == g, :3]).mean())
           for g in set(groups)}
    missed = float((done & ~ready).sum()/max(1, done.sum()))
    false = float((~done & ready).sum()/max(1, (~done).sum()))
    mean_mae = float(np.mean(list(mae.values())))
    return {'missed_done_rate': missed, 'false_done_rate': false, 'group_normalized_mae': mae,
            **selection(rows, predictions, mean_mae, missed+false+mean_mae, 'episode'),
            'samples': len(rows), 'done_samples': int(done.sum())}


@torch.inference_mode()
def native_chunks(policy, rows, windows, size, history, check_budget):
    """Every row follows the deployed B1 policy / B4 frozen-CNN RGB path."""
    @lru_cache(maxsize=64)
    def tensorize(jpeg, image_size):
        return image_tensor(jpeg, image_size)

    policy.eval()
    chunks = []
    for i in range(len(rows)):
        if i % 32 == 0:
            check_budget()
        batch = actor_batch(frames_at(rows, windows[i].tolist()), size, history,
                            tensorize=tensorize)
        chunks.append(policy.predict_action_chunk(batch)[0].tolist())
    return chunks


@torch.inference_mode()
def cached_chunks(policy, cache, windows, check_budget):
    chunks = []
    policy.eval()
    with frozen_features(policy):
        for i in range(0, len(windows), 32):
            check_budget()
            chunks.extend(policy.predict_action_chunk(batch_at(cache, windows[i:i+32]))
                          .tolist())
    return chunks


def compare_first_predictions(rows, cached, native):
    cache = np.asarray(cached)
    actual = np.asarray(native)
    require(cache.shape == actual.shape == (len(rows), 4), 'prediction shape mismatch')
    require(np.isfinite(cache).all() and np.isfinite(actual).all(), 'nonfinite full prediction')
    flips = np.flatnonzero((cache[:, 3] >= .65) != (actual[:, 3] >= .65))
    return {'first_action_max_abs': float(np.abs(cache-actual).max()),
            'first_action_axis_max_abs': float(np.abs(cache[:, :3]-actual[:, :3]).max()),
            'done_score_max_abs': float(np.abs(cache[:, 3]-actual[:, 3]).max()),
            'done_threshold_flip_count': int(len(flips)),
            'done_threshold_flip_ids': [rows[i]['id'] for i in flips]}


def verify_memoized_runtime_first(actor, cached_actor, frames):
    """Compare the exact first action issued by normal and worker LRU paths."""
    batch = actor_batch(frames, EXPECTED_SIZE, EXPECTED_HISTORY)
    with torch.inference_mode():
        native_raw = actor.policy.predict_action_chunk(batch)[0, 0]
        cached_raw = cached_actor.policy.predict_action_chunk(batch)[0, 0]
    cached_decision = cached_actor.predict(frames)
    repeated = cached_actor.predict(frames)
    require(cached_decision == repeated == decode(cached_raw.tolist()),
            'memoized runtime decision changed across repeated input')
    require(torch.isfinite(native_raw).all() and torch.isfinite(cached_raw).all(),
            'nonfinite memoized runtime first action')
    require(bool(torch.all(torch.isclose(native_raw, cached_raw, rtol=1e-5, atol=1e-5))),
            'memoized runtime first ACT action exceeds original 1e-5 guard')
    require(bool((native_raw[3] >= .65) == (cached_raw[3] >= .65)),
            'memoized runtime done decision changed')
    return {'native_raw_first_action': native_raw.tolist(),
            'memoized_raw_first_action': cached_raw.tolist(),
            'first_action_max_abs': float((native_raw-cached_raw).abs().max()),
            'native_done': bool(native_raw[3] >= .65),
            'memoized_done': bool(cached_raw[3] >= .65),
            'memoized_decision': cached_decision,
            'feature_cache_hits': cached_actor.policy.model.backbone.cache_hits}


def run(args):
    source_sha = source_identity()
    require(source_sha != TRAINING_SOURCE, 'recovery must run from a distinct committed source')
    output = args.out.resolve()
    failed = args.failed_run.resolve()
    require(not output.is_relative_to(failed) and not failed.is_relative_to(output),
            'new output overlaps the failed original managed run')
    require(not output.is_relative_to(ROOT.resolve()),
            'new output must be outside the committed source worktree')
    torch.set_num_threads(2)
    started = time.monotonic()
    def check_budget():
        require(time.monotonic()-started < MAX_FINALIZATION_WALL_S,
                'finalization exceeded fixed 720-second wall budget')
    paths = original_paths(args.failed_run.resolve())
    expected = {'manifest': args.expected_manifest_sha256, 'report': args.expected_report_sha256,
                'checkpoint': args.expected_checkpoint_sha256,
                'source_freeze': args.expected_source_freeze_sha256}
    report, manifest, hashes, original_trace = verify_original_metadata(
        paths, expected, args.dataset, args.source_freeze)
    verify_execution_environment(report)
    inference_files = verify_inference_source()
    checkpoint = torch.load(paths['checkpoint'], map_location='cpu', weights_only=True)
    selected_state_sha = verify_checkpoint(checkpoint, report)
    data = json.loads(args.dataset.read_text())
    require(not {e['root'] for e in data['train']} & {e['root'] for e in data['development']},
            'train/development episodes overlap')
    rows, _, _ = load(data['train'])
    dev, _, _ = load(data['development'])
    require(len(rows) == report['samples'] and
            [e['root'] for e in data['train']] == report['train_episodes'] and
            [e['root'] for e in data['development']] == report['development_episodes'],
            'episode split or training rows changed')
    windows = history_indices(data['train'], EXPECTED_HISTORY)
    dwindows = history_indices(data['development'], EXPECTED_HISTORY)
    policy = make_policy(EXPECTED_SIZE, EXPECTED_HISTORY, pretrained=False).cpu().eval()
    policy.load_state_dict(checkpoint['best_state'], strict=True)
    require(state_digest(policy.state_dict()) == selected_state_sha,
            'selected policy weights differ after load')
    del checkpoint
    for parameter in policy.model.backbone.parameters():
        parameter.requires_grad_(False)

    args.out.mkdir(parents=True, exist_ok=False)
    result = {'complete': False, 'complete_scope': 'artifact_finalization_only',
              'source_sha': source_sha, 'training_source_sha': TRAINING_SOURCE,
              'inference_source_sha': TRAINING_SOURCE, 'inference_file_sha256': inference_files,
              'original_managed_run': str(args.failed_run.resolve()),
              'original_status': manifest['status'], 'original_exit_code': manifest['exit_code'],
              'original_report_complete': report['complete'],
              'original_report_sha256': hashes['report'],
              'original_checkpoint_sha256': hashes['checkpoint'],
              'original_manifest_sha256': hashes['manifest'],
              'original_console_sha256': hashes['console'],
              'original_failure_stage': 'selected_cache_verification/train/whole_chunk_original_1e-5',
              'original_failure': 'unused chunk[1..6] done coordinates exceeded old full-chunk 1e-5; '
                                  'original exit 1 and complete=false remain authoritative',
              'original_failure_trace': original_trace,
              'source_freeze_sha256': hashes['source_freeze'],
              'dataset_path': str(args.dataset.resolve()), 'dataset_sha256': report['dataset_sha256'],
              'original_training_completed_steps': report['completed_steps'],
              'original_training_wall_s': report['wall_s'],
              'optimizer_updates_this_run': 0, 'new_checkpoint_selection': False,
              'selected_step': report['selected']['step'],
              'selected_checkpoint_eligible': report['selected']['selection_eligible'],
              'selected': report['selected'],
              'selected_state_sha256': selected_state_sha,
              'original_progress_reference': str(paths['report']),
              'adapter': report['adapter'], 'environment': report['environment'],
              'termination_objective': report['termination_objective'],
              'physical_success_claim': False, 'diagnostic_model_only': True,
              'finalization_wall_cap_s': MAX_FINALIZATION_WALL_S}
    write(args.out/'report.json', result)

    cache = cache_images(policy, rows, EXPECTED_SIZE)
    dcache = cache_images(policy, dev, EXPECTED_SIZE)
    check_budget()
    result['selected_cache_verification'] = {
        'train': verify_cache(policy, rows, windows, cache, EXPECTED_SIZE, EXPECTED_HISTORY),
        'development': verify_cache(policy, dev, dwindows, dcache, EXPECTED_SIZE, EXPECTED_HISTORY)}
    result['deployment_max_abs'] = {
        'train': verify_cpu_deployment(policy, rows, windows, EXPECTED_SIZE, EXPECTED_HISTORY),
        'development': verify_cpu_deployment(policy, dev, dwindows, EXPECTED_SIZE, EXPECTED_HISTORY)}
    write(args.out/'report.json', result)

    for split, rs, cs, ws in (('train', rows, cache, windows),
                              ('development', dev, dcache, dwindows)):
        cached_chunk = cached_chunks(policy, cs, ws, check_budget)
        native_chunk = native_chunks(policy, rs, ws, EXPECTED_SIZE, EXPECTED_HISTORY,
                                     check_budget)
        cached = [chunk[0] for chunk in cached_chunk]
        native = [chunk[0] for chunk in native_chunk]
        result[split+'_cache_metrics'] = metrics_from_raw(rs, cached)
        result[split+'_native_metrics'] = metrics_from_raw(rs, native)
        result[split+'_cache_vs_native_first_action'] = compare_first_predictions(rs, cached, native)
        cache_tensor = torch.tensor(cached_chunk)
        native_tensor = torch.tensor(native_chunk)
        guards = classify_cached_action_chunk(cache_tensor, native_tensor)
        guards['first_action_strict_mismatch_ids_first64'] = [
            rs[i]['id'] for i in torch.nonzero(~torch.all(torch.isclose(
                cache_tensor[:, 0], native_tensor[:, 0], rtol=1e-5, atol=1e-5), dim=1))[:64, 0].tolist()]
        guards['done_decision_flip_ids_first64'] = [
            rs[i]['id'] for i in torch.nonzero(torch.any(
                (cache_tensor[:, :, 3] >= .65) != (native_tensor[:, :, 3] >= .65), dim=1))[:64, 0].tolist()]
        guards['full_chunk_bounded_mismatch_ids_first64'] = [
            rs[i]['id'] for i in torch.nonzero(~torch.isclose(
                cache_tensor, native_tensor, rtol=1e-4, atol=1e-4).flatten(1).all(dim=1))[:64, 0].tolist()]
        result[split+'_all_rows_cache_guard'] = guards
        write(args.out/(split+'-predictions.json'), [
            {'id': row['id'], 'target': row['action'], 'cache_chunk': c,
             'native_chunk': n} for row, c, n in zip(rs, cached_chunk, native_chunk)])
        write(args.out/'report.json', result)
        require(guards['all_chunk_done_decisions_same'],
                f'{split} all-row ACT done threshold guard failed')
        require(guards['first_action_original_strict_guard_passed'],
                f'{split} all-row ACT deployed first-action 1e-5 guard failed')
        require(guards['full_chunk_bounded_guard_passed'],
                f'{split} all-row ACT full-chunk 1e-4 guard failed')
        check_budget()

    actor = InputCarryAct(policy, EXPECTED_SIZE, EXPECTED_HISTORY)
    actor.save(args.out/'act')
    restored = InputCarryAct.load(args.out/'act')
    cached_actor = InputCarryAct.load(args.out/'act', cache_features=True)
    require(state_digest(restored.policy.state_dict()) == selected_state_sha,
            'restored artifact weights differ from original selected state')
    require(state_digest(cached_actor.policy.state_dict()) == selected_state_sha,
            'memoized runtime artifact weights differ from original selected state')
    result['readback'] = []
    result['memoized_runtime_readback'] = []
    for split, rs, ws in (('train', rows, windows), ('development', dev, dwindows)):
        for i in (0, len(rs)-1):
            frames = frames_at(rs, ws[i].tolist())
            native = actor.predict(frames)
            require(restored.predict(frames) == native, 'selected artifact readback mismatch')
            memo = verify_memoized_runtime_first(actor, cached_actor, frames)
            result['readback'].append({'split': split, 'id': rs[i]['id'],
                                       'history_ids': [rs[j]['id'] for j in ws[i]],
                                       'decision': native})
            result['memoized_runtime_readback'].append({
                'split': split, 'id': rs[i]['id'], **memo})
        check_budget()
    require(cached_actor.policy.model.backbone.cache_hits > 0,
            'memoized runtime readback never exercised feature cache hits')
    del restored, cached_actor

    result['model_sha256'] = digest(args.out/'act/model.safetensors')
    require(state_digest(policy.state_dict()) == selected_state_sha,
            'selected weights changed during finalization')
    for key, path in paths.items():
        require_digest(path, hashes[key], f'original {key} changed during finalization')
    require_digest(paths['manifest'].parent/'console.log', hashes['console'],
                   'original console changed during finalization')
    require_digest(args.source_freeze, hashes['source_freeze'], 'source freeze changed during finalization')
    require(verify_dataset(args.dataset)['dataset_sha256'] == report['dataset_sha256'],
            'dataset changed during finalization')
    require(source_identity() == source_sha, 'recovery source changed during finalization')
    check_budget()
    result['recovery_wall_s'] = time.monotonic()-started
    result['complete'] = True
    write(args.out/'report.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--failed-run', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--source-freeze', type=Path, required=True)
    for item in ('manifest', 'report', 'checkpoint', 'source-freeze'):
        parser.add_argument('--expected-'+item+'-sha256', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    for name in ('expected_manifest_sha256', 'expected_report_sha256',
                 'expected_checkpoint_sha256', 'expected_source_freeze_sha256'):
        require(len(getattr(args, name)) == 64 and all(c in '0123456789abcdef' for c in getattr(args, name)),
                'expected SHA-256 values must be lowercase 64-character hex')
    run(args)


if __name__ == '__main__':
    main()
