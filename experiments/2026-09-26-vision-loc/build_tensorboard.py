"""TensorBoard snapshot of the tag-free vision localization (native event files, repo Writer).

Reuses the repo exporter's ``Writer`` (``scripts/tensorboard_tools/export.py``,
TensorBoard event protobufs) as PR #210 did:

  <logdir>/<snapshot>/vl-<seed>-<filter>/   per-frame errors vs eval-only GT (test split)
  <logdir>/<snapshot>/vl-pooled-<filter>/   pooled test summary (+ gates for the student)
  <logdir>/<snapshot>/vl-ref-<name>/        tag references (other environments, summary only)
  <logdir>/<snapshot>/vl-train-seg/         segmentation training curve
  <logdir>/<snapshot>/collection.json       runs, sources, hashes

Existing destinations are refused. ``--verify`` re-reads every run with
EventAccumulator and compares the summary scalars with results.json.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))

FILTER_LABELS = {'vision': 'vision seg (tag-free, student)', 'boundary': 'PR210 boundary (tag-free)',
                 'deadreck': 'command integration', 'oracle': 'oracle labels (diagnostic)'}
GROUPS = ('all', 'door_zone', 'door_loaded', 'door_unloaded', 'loaded', 'unloaded')
KEYS = ('pos_p50_m', 'pos_p90_m', 'pos_p99_m', 'lat_abs_p99_m', 'yaw_p90_deg')
HP_METRICS = [f'summary/{g}/{k}' for g in GROUPS for k in KEYS] + ['gate/pass']


def sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def ang(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def write_run(writer_cls, out: Path, name: str, hp: dict, summary: dict, series: dict | None, provenance: dict,
              extra: dict | None = None):
    run_dir = out/name
    run_dir.mkdir(parents=True, exist_ok=False)
    w = writer_cls(run_dir, time.time())
    for g in GROUPS:
        for k in KEYS:
            v = (summary or {}).get(g, {}).get(k)
            if v is not None:
                w.scalar(f'summary/{g}/{k}', float(v), 0)
    for tag, v in (extra or {}).items():
        w.scalar(tag, float(v), 0)
    for tag, pts in (series or {}).items():
        for step, value in pts:
            w.scalar(tag, float(value), int(step))
    w.text('provenance/source', provenance)
    w.hparams(hp, HP_METRICS)
    w.close()
    (run_dir/'manifest.json').write_text(json.dumps(
        {'schema': 'ugrp.vision_loc.tensorboard_run.v1', 'run': name, 'hparams': hp, 'counts': w.counts,
         'provenance': provenance, 'event_wall_time': 'export time; x axis = frame index (or training step)'},
        indent=1, ensure_ascii=False) + '\n')
    return w.counts


def build(args):
    from scripts.tensorboard_tools.export import Writer
    import vision_loc_cli as cli
    results = json.loads(args.results.read_text())
    out = args.logdir/args.snapshot
    if out.exists():
        raise SystemExit(f'{out} exists; snapshots are never overwritten')
    out.mkdir(parents=True)
    test = results['test']
    est_dir = Path(test['estimates_dir'])
    common = {'source_sha': args.source_sha, 'map': 'zone_wide_door_walls_v3_notags (0 tags, walls 0.40 m)',
              'environment': 'tag-free v3', 'claim_scope': 'offline replay of teacher-driven renders; not closed loop',
              'model_sha8': results['model']['sha256'][:8]}
    exported = []
    for ep, per in test['metrics']['episodes'].items():
        est = [json.loads(x) for x in (est_dir/f'{ep}.estimates.jsonl').read_text().splitlines() if x.strip()]
        ev_path = cli.RENDER_ROOT/ep/'eval_only'/'frames_eval.jsonl'
        ev = {r['frame']: r for r in (json.loads(x) for x in ev_path.read_text().splitlines() if x.strip())}
        for f, summary in per.items():
            pos, yaw, cols = [], [], []
            for rec in est:
                v = rec.get(f)
                if not v:
                    continue
                gt = ev[rec['frame']]['gt']
                x, y, th = v['xyyaw']
                pos.append((rec['frame'], math.hypot(x - gt[0], y - gt[1])))
                yaw.append((rec['frame'], abs(math.degrees(ang(th - gt[2])))))
                if v.get('n_cols') is not None:
                    cols.append((rec['frame'], v['n_cols']))
            series = {'localization/pos_err_m': pos, 'localization/yaw_err_deg': yaw}
            if cols:
                series['localization/informative_columns'] = cols
            run = f"vl-{ep.split('-s')[-1]}-{f}"
            hp = {**common, 'filter': f, 'filter_label': FILTER_LABELS.get(f, f), 'episode': ep, 'split': 'test'}
            prov = {'estimates': str(est_dir/f'{ep}.estimates.jsonl'),
                    'estimates_sha256': sha((est_dir/f'{ep}.estimates.jsonl').read_bytes()),
                    'eval_only': str(ev_path), 'eval_only_sha256': sha(ev_path.read_bytes()),
                    'results_sha256': sha(args.results.read_bytes())}
            counts = write_run(Writer, out, run, hp, summary, series, prov)
            exported.append({'name': run, 'kind': 'episode', 'episode': ep, 'filter': f, 'counts': counts})
    gate = results.get('gate', {})
    for f, summary in test['metrics']['pooled'].items():
        run = f'vl-pooled-{f}'
        hp = {**common, 'filter': f, 'filter_label': FILTER_LABELS.get(f, f), 'episode': 'test pooled', 'split': 'test'}
        extra = {'gate/pass': float(bool(gate.get('pass')))} if f == 'vision' and 'pass' in gate else None
        counts = write_run(Writer, out, run, hp, summary, None, {'results_sha256': sha(args.results.read_bytes())}, extra)
        exported.append({'name': run, 'kind': 'pooled', 'filter': f, 'counts': counts})
    for name, ref in results.get('references', {}).items():
        run = f'vl-ref-{name}'
        hp = {**common, 'filter': name, 'filter_label': ref['label'], 'episode': ref['episodes'], 'split': 'reference',
              'environment': ref['environment'], 'claim_scope': ref['scope']}
        counts = write_run(Writer, out, run, hp, ref['summary'], None, {'source': ref['source']})
        exported.append({'name': run, 'kind': 'reference', 'filter': name, 'counts': counts})
    train = results.get('model', {}).get('train_info')
    if train:
        series = {'train/loss': [(h['step'], h['loss']) for h in train['history'] if 'loss' in h],
                  'val/miou': [(h['step'], h['val']['miou']) for h in train['history'] if 'val' in h],
                  'val/pixel_acc': [(h['step'], h['val']['pixel_acc']) for h in train['history'] if 'val' in h]}
        hp = {**common, 'filter': 'seg_train', 'filter_label': 'LR-ASPP MobileNetV3 training', 'episode': 'train split',
              'split': 'train'}
        counts = write_run(Writer, out, 'vl-train-seg', hp, {}, series, {'train_info_sha256': results['model'].get(
            'train_info_sha256')})
        exported.append({'name': 'vl-train-seg', 'kind': 'train', 'filter': 'seg_train', 'counts': counts})
    (out/'collection.json').write_text(json.dumps(
        {'schema': 'ugrp.vision_loc.tensorboard_collection.v1', 'exported': exported, 'source_sha': args.source_sha,
         'results_json': str(args.results), 'results_sha256': sha(args.results.read_bytes()),
         'exported_at_s': time.time(),
         'limits': 'offline localization errors of an open-loop replay; references are other environments'},
        indent=1, ensure_ascii=False) + '\n')
    print(f'{len(exported)} runs -> {out}')


def verify(args):
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    results = json.loads(args.results.read_text())
    out = args.logdir/args.snapshot
    collection = json.loads((out/'collection.json').read_text())
    checked = mismatches = points = 0
    for item in collection['exported']:
        acc = EventAccumulator(str(out/item['name']), size_guidance={'scalars': 0, 'tensors': 0})
        acc.Reload()
        tags = acc.Tags()['scalars']
        if item['kind'] == 'episode':
            summary = results['test']['metrics']['episodes'][item['episode']][item['filter']]
        elif item['kind'] == 'pooled':
            summary = results['test']['metrics']['pooled'][item['filter']]
        elif item['kind'] == 'reference':
            summary = results['references'][item['filter']]['summary']
        else:
            summary = {}
        for g in GROUPS:
            for k in KEYS:
                want = summary.get(g, {}).get(k)
                if want is None:
                    continue
                tag = f'summary/{g}/{k}'
                got = acc.Scalars(tag)[0].value if tag in tags else None
                checked += 1
                if got is None or abs(got - want) > 1e-5*max(1., abs(want)):
                    mismatches += 1
                    print('MISMATCH', item['name'], tag, got, want)
        if 'localization/pos_err_m' in tags:
            vals = [s.value for s in acc.Scalars('localization/pos_err_m')]
            points += len(vals)
            checked += 1
            if abs(float(np.percentile(vals, 90)) - summary['all']['pos_p90_m']) > 5e-4:
                mismatches += 1
                print('SERIES MISMATCH', item['name'])
    report = {'runs': len(collection['exported']), 'checked': checked, 'mismatches': mismatches,
              'series_points': points}
    print(json.dumps(report))
    if args.report:
        Path(args.report).write_text(json.dumps(report, indent=1) + '\n')
    return 1 if mismatches else 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--results', type=Path, default=HERE/'results'/'results.json')
    ap.add_argument('--logdir', type=Path, default=Path('/Users/changmin/projects/ugrp/outputs/tensorboard'))
    ap.add_argument('--snapshot', default='0926-vision-loc')
    ap.add_argument('--source-sha', default='')
    ap.add_argument('--verify', action='store_true')
    ap.add_argument('--report')
    args = ap.parse_args(argv)
    if args.verify:
        raise SystemExit(verify(args))
    build(args)


if __name__ == '__main__':
    main()
