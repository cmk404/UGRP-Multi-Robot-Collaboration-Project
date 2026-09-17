#!/usr/bin/env python3
"""Train upstream ACT with frozen pretrained vision; select on development only."""
from __future__ import annotations
import argparse
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import numpy as np
import torch
from harness.reference_act import IMAGE_KEYS, UPSTREAM_SHA, RGBAct, image_tensor, make_policy
from harness.reference_approach_data import load_teacher, action_chunks, sha256
from harness.act_training import frozen_features, prediction_metrics, sampling_weights
from harness.camera_approach_student import fit_approach_model, predict_approach


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def cached_images(policy, rows, profile):
    result = {}
    policy.eval()
    with torch.no_grad():
        for key, field in zip(IMAGE_KEYS, ('own_jpeg', 'top_jpeg')):
            batches = []
            for begin in range(0, len(rows), 32):
                value = torch.stack([image_tensor(r[field], profile) for r in rows[begin:begin + 32]])
                batches.append(policy.model.backbone(value)['feature_map'].detach())
            result[key] = torch.cat(batches)
    return result


@torch.no_grad()
def evaluate(policy, features, rows):
    policy.eval()
    outputs = []
    with frozen_features(policy):
        for begin in range(0, len(rows), 64):
            batch = {key: value[begin:begin + 64] for key, value in features.items()}
            outputs.extend(policy.predict_action_chunk(batch)[:, 0].tolist())
    return prediction_metrics(outputs, rows), outputs


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--teacher-dirs', nargs='+', type=Path, required=True)
    p.add_argument('--protocol', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--sampling', choices=('uniform', 'balanced'), required=True)
    p.add_argument('--steps', type=int, default=10000)
    p.add_argument('--seed', type=int, default=20260916)
    p.add_argument('--eval-every', type=int, default=500)
    args = p.parse_args()
    out = args.out_dir.resolve(); out.mkdir(parents=True, exist_ok=False)
    source_sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=ROOT):
        raise ValueError('commit code before training')
    direct = json.loads(importlib.metadata.distribution('lerobot').read_text('direct_url.json'))
    if direct.get('vcs_info', {}).get('commit_id') != UPSTREAM_SHA:
        raise ValueError('wrong upstream revision')
    torch.set_num_threads(2)
    protocol = json.loads(args.protocol.read_text())
    trajectories = {r: {} for r in ('r1', 'r3')}; sources = []; references = None
    for root in args.teacher_dirs:
        data, refs, info = load_teacher(root)
        if references is None:
            references = refs
        elif refs != references:
            raise ValueError('all sources must have identical goal RGB')
        for rid in trajectories:
            if set(data[rid]) & set(trajectories[rid]):
                raise ValueError('duplicate case ids across sources')
            trajectories[rid].update(data[rid])
        sources.append({'root': str(root.resolve()), 'source_sha': info['source_sha'],
                        'successful_cases': info['successful_cases'], 'excluded_cases': info['excluded_cases'],
                        'hashes': {n: sha256(root / n) for n in ('report.json', 'actor_samples.json', 'privileged_labels.json')}})
    dev = protocol['development_cases']
    all_cases = sorted(trajectories['r1'])
    if not set(dev) <= set(all_cases):
        raise ValueError('missing development cases')
    train = sorted(set(all_cases) - set(dev))
    if set(train) & set(dev) or len(train) < 4:
        raise ValueError('invalid split')
    report = {'complete': False, 'source_sha': source_sha, 'upstream_sha': UPSTREAM_SHA,
              'protocol': protocol, 'sources': sources, 'train_cases': train, 'development_cases': dev,
              'sampling': args.sampling, 'seed': args.seed, 'steps': args.steps, 'batch_size': 32,
              'profile': 'imagenet128', 'backbone': 'ImageNet ResNet18 frozen; exact RGB feature cache',
              'learning_rate': .0001, 'checkpoint_selection': 'lowest development selection_score; no final test data',
              'environment': {'python': sys.version, 'platform': platform.platform(),
                              **{n: importlib.metadata.version(n) for n in ('torch','torchvision','lerobot','numpy')}},
              'robots': {}, 'external_model_calls': 0, 'external_model_tokens': 0}
    write(out / 'report.json', report)
    started = time.monotonic()
    for rid in trajectories:
        torch.manual_seed(args.seed); np.random.seed(args.seed)
        folder = out / rid; folder.mkdir()
        rows, target, padding = action_chunks(trajectories[rid], train, 4)
        ref_own, ref_top = references[rid]
        anchor = {'sample_id': 'goal-anchor:' + rid, 'case_id': 'goal-anchor', 'own_jpeg': ref_own,
                  'top_jpeg': ref_top, 'forward': 0., 'stop': True}
        rows.append(anchor); target.append([[0.,1.]] * 4); padding.append([False,True,True,True])
        dev_rows = [row for c in dev for row in trajectories[rid][c]]
        write(folder / 'training-inputs.json', [{k: v for k,v in r.items() if not k.endswith('_jpeg')} for r in rows])
        begin = time.monotonic()
        kernel = fit_approach_model(ref_own, ref_top, rows[:-1], domain_samples=rows[:-1])
        write(folder / 'kernel.json', kernel)
        print(json.dumps({'robot': rid, 'event': 'kernel_fit', 'seconds': time.monotonic()-begin}), flush=True)
        policy = make_policy('imagenet128', pretrained=True)
        for parameter in policy.model.backbone.parameters():
            parameter.requires_grad_(False)
        feature_started = time.monotonic()
        features = cached_images(policy, rows, 'imagenet128')
        dev_features = cached_images(policy, dev_rows, 'imagenet128')
        print(json.dumps({'robot': rid, 'event': 'feature_cache', 'seconds': time.monotonic()-feature_started,
                          'train_samples': len(rows), 'stop_samples': sum(r['stop'] for r in rows)}), flush=True)
        target = torch.tensor(target, dtype=torch.float32); padding = torch.tensor(padding, dtype=torch.bool)
        weights = sampling_weights(rows, args.sampling == 'balanced')
        generator = torch.Generator().manual_seed(args.seed)
        optimizer = torch.optim.AdamW([v for v in policy.parameters() if v.requires_grad], lr=1e-4, weight_decay=1e-4)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.steps, eta_min=1e-5)
        progress = []; best = float('inf'); best_state = None; selected = None
        train_started = time.monotonic()
        for step in range(1, args.steps + 1):
            idx = torch.multinomial(weights, 32, replacement=True, generator=generator)
            batch = {key: value[idx] for key, value in features.items()}
            batch.update(action=target[idx], action_is_pad=padding[idx])
            policy.train(); optimizer.zero_grad()
            with frozen_features(policy):
                loss, metrics = policy(batch)
                if not torch.isfinite(loss):
                    raise ValueError('nonfinite loss')
                loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.)
            optimizer.step(); scheduler.step()
            if step == 1 or step % args.eval_every == 0 or step == args.steps:
                train_metrics, _ = evaluate(policy, features, rows)
                dev_metrics, _ = evaluate(policy, dev_features, dev_rows)
                event = {'step': step, 'loss': float(loss.detach()), **metrics,
                         'train': train_metrics, 'development': dev_metrics,
                         'elapsed_s': time.monotonic()-train_started}
                progress.append(event); write(folder / 'progress.json', progress)
                print(json.dumps({'robot': rid, **event}), flush=True)
                if dev_metrics['selection_score'] < best:
                    best = dev_metrics['selection_score']; selected = event
                    best_state = {k: v.detach().cpu().clone() for k,v in policy.state_dict().items()}
        policy.load_state_dict(best_state)
        actor = RGBAct(policy, 'imagenet128'); actor.save(folder / 'act')
        restored = RGBAct.load(folder / 'act')
        # Audit complete deployed RGB preprocessing for stop/moving probes from both splits.
        probe_rows = [next(r for r in group if bool(r['stop']) == stop)
                      for group in (rows, dev_rows) for stop in (False, True)]
        readback = []
        for row in probe_rows:
            expected = actor.predict(row['own_jpeg'], row['top_jpeg'])
            actual = restored.predict(row['own_jpeg'], row['top_jpeg'])
            if actual != expected:
                raise ValueError('checkpoint prediction mismatch')
            readback.append({'sample_id': row['sample_id'], 'prediction': actual})
        for name, group, cache in [('train',rows,features), ('development',dev_rows,dev_features)]:
            stats, values = evaluate(policy, cache, group)
            write(folder / (name+'-predictions.json'), [
                {'sample_id': row['sample_id'], 'target_forward': row['forward'], 'target_stop': row['stop'],
                 'raw_prediction': value} for row,value in zip(group,values)])
        report['robots'][rid] = {'selected': selected, 'training_s': time.monotonic()-train_started,
                                'total_s': time.monotonic()-begin, 'readback': readback,
                                'parameters': sum(v.numel() for v in policy.parameters()),
                                'trainable_parameters': sum(v.numel() for v in policy.parameters() if v.requires_grad)}
        write(out / 'report.json', report)
    write(out / 'approach-skill.json', {'schema': 'ugrp.rgb_short_approach_skill.v1',
          'runtime_inputs': ['own_rgb','fixed_top_rgb'], 'models': {
              rid: {'path': rid+'/kernel.json', 'sha256': sha256(out/rid/'kernel.json')} for rid in trajectories}})
    report.update(complete=True, wall_s=time.monotonic()-started)
    report['artifacts'] = {str(p.relative_to(out)): sha256(p) for p in sorted(out.rglob('*'))
                           if p.is_file() and p.name != 'report.json'}
    write(out / 'report.json', report)


if __name__ == '__main__':
    main()
