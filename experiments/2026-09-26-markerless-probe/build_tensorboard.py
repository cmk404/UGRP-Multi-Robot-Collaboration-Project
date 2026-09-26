"""TensorBoard snapshot of the markerless probe (native event files, repo Writer).

The standard exporter (scripts/export_tensorboard.py) converts execution
results; it has no localization-error view. This builder reuses its ``Writer``
(TensorBoard event protobufs, no TF/torch) and writes a new snapshot:

  <logdir>/<snapshot>/ml-<episode>-<filter>/   per-frame errors vs eval-only GT
  <logdir>/<snapshot>/ml-pooled-<filter>/      pooled test summary
  <logdir>/<snapshot>/collection.json          runs, sources, hashes

Existing destinations are refused. ``--verify`` re-reads every run with
EventAccumulator and compares the summary scalars with results.json.
Evaluation data only: GT appears here because this is the scoring output.
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

import run_probe as rp  # noqa: E402

FILTER_NAMES = {'boundary': 'boundary(태그 없음)', 'tag_online': 'tag PF(실행 기록)', 'tag_replay': 'tag PF(재생)',
                'deadreck': '명령 적분'}
SUMMARY_KEYS = [('pos_p50_m', 'all'), ('pos_p90_m', 'all'), ('pos_p99_m', 'all'), ('pos_max_m', 'all'),
                ('share_pos_lt_5cm', 'all'), ('yaw_p90_deg', 'all'), ('pos_p90_m', 'door_zone'),
                ('pos_p90_m', 'carry'), ('pos_p90_m', 'search_approach')]
HP_METRICS = [f'summary/{g}/{k}' for k, g in SUMMARY_KEYS]


def sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def ang(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def write_run(writer_cls, out: Path, name: str, hp: dict, summary: dict, series: dict | None, provenance: dict):
    run_dir = out/name
    run_dir.mkdir(parents=True, exist_ok=False)
    at = time.time()
    w = writer_cls(run_dir, at)
    for (k, g) in SUMMARY_KEYS:
        v = summary.get(g, {}).get(k)
        if v is not None:
            w.scalar(f'summary/{g}/{k}', float(v), 0)
    for tag, pts in (series or {}).items():
        for step, value in pts:
            w.scalar(tag, value, step)
    w.text('provenance/source', provenance)
    w.hparams(hp, HP_METRICS)
    w.close()
    manifest = {'schema': 'ugrp.markerless_probe.tensorboard_run.v1', 'run': name, 'hparams': hp,
                'counts': w.counts, 'provenance': provenance,
                'event_wall_time': 'export time, not execution time; x axis = recorded frame index'}
    (run_dir/'manifest.json').write_text(json.dumps(manifest, indent=1, ensure_ascii=False) + '\n')
    return w.counts


def build(args):
    from scripts.tensorboard_tools.export import Writer
    results = json.loads(args.results.read_text())
    out = args.logdir/args.snapshot
    if out.exists():
        raise SystemExit(f'{out} exists; snapshots are never overwritten')
    out.mkdir(parents=True)
    src_sha = args.source_sha
    cal_sha = results['runs'][next(iter(results['runs']))]['probe_calibration']['sha256']
    exported = []
    for ep in results['episodes']:
        name = ep.replace('/', '__')
        est_path = Path(results['estimates_dir'])/f'{name}.estimates.jsonl'
        ev_path = rp.DATA_ROOT/ep/'eval_only'/'frames_eval.jsonl'
        est = [json.loads(line) for line in est_path.read_text().splitlines() if line.strip()]
        ev = {r['frame']: r for r in (json.loads(line) for line in ev_path.read_text().splitlines() if line.strip())}
        seed = results['runs'][ep]['seed']
        for f in ('boundary', 'tag_online', 'tag_replay', 'deadreck'):
            if f not in results['episodes'][ep]:
                continue
            pos, yaw, cols = [], [], []
            for rec in est:
                e = ev[rec['frame']]
                gt = e['gt']
                if f == 'tag_online':
                    if e.get('pos_err_m') is None:
                        continue
                    pos.append((rec['frame'], float(e['pos_err_m'])))
                    yaw.append((rec['frame'], float(e['yaw_err_deg'])))
                    continue
                v = rec.get(f)
                if not v:
                    continue
                x, y, th = v['xyyaw']
                pos.append((rec['frame'], math.hypot(x - gt[0], y - gt[1])))
                yaw.append((rec['frame'], abs(math.degrees(ang(th - gt[2])))))
                if f == 'boundary':
                    cols.append((rec['frame'], float(v.get('n_cols', 0))))
            series = {'localization/pos_err_m': pos, 'localization/yaw_err_deg': yaw}
            if cols:
                series['localization/scan_columns'] = cols
            run = f"ml-{ep.split('-')[-1]}-{f}"
            hp = {'filter': f, 'filter_label': FILTER_NAMES[f], 'episode': ep, 'split': 'test', 'seed': seed,
                  'map_id': 'zone_wide_door_tags_v2 (tags removed for boundary)', 'source_sha': src_sha,
                  'calibration_sha8': cal_sha[:8], 'claim_scope': 'offline replay; not closed loop'}
            prov = {'estimates': str(est_path), 'estimates_sha256': sha(est_path.read_bytes()),
                    'eval_only': str(ev_path), 'eval_only_sha256': sha(ev_path.read_bytes()),
                    'results_json': str(args.results), 'results_sha256': sha(args.results.read_bytes()),
                    'note': 'tag_online = the tag PF errors recorded by the M1 run itself'}
            counts = write_run(Writer, out, run, hp, results['episodes'][ep][f], series, prov)
            exported.append({'name': run, 'episode': ep, 'filter': f, 'counts': counts})
    for f, summary in results['pooled'].items():
        run = f'ml-pooled-{f}'
        hp = {'filter': f, 'filter_label': FILTER_NAMES.get(f, f), 'episode': 'test pooled (s101-s106)', 'split': 'test',
              'seed': 'pooled', 'map_id': 'zone_wide_door_tags_v2 (tags removed for boundary)', 'source_sha': src_sha,
              'calibration_sha8': cal_sha[:8], 'claim_scope': 'offline replay; not closed loop'}
        counts = write_run(Writer, out, run, hp, summary, None,
                           {'results_json': str(args.results), 'results_sha256': sha(args.results.read_bytes())})
        exported.append({'name': run, 'episode': 'pooled', 'filter': f, 'counts': counts})
    collection = {'schema': 'ugrp.markerless_probe.tensorboard_collection.v1', 'exported': exported,
                  'source_sha': src_sha, 'results_json': str(args.results),
                  'results_sha256': sha(args.results.read_bytes()), 'exported_at_s': time.time(),
                  'limits': 'offline localization errors of an open-loop replay; no task success; '
                            'GT only for scoring'}
    (out/'collection.json').write_text(json.dumps(collection, indent=1, ensure_ascii=False) + '\n')
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
        summary = (results['pooled'][item['filter']] if item['episode'] == 'pooled'
                   else results['episodes'][item['episode']][item['filter']])
        for k, g in SUMMARY_KEYS:
            want = summary.get(g, {}).get(k)
            tag = f'summary/{g}/{k}'
            if want is None:
                continue
            got = acc.Scalars(tag)[0].value if tag in tags else None
            checked += 1
            if got is None or abs(got - want) > 1e-5*max(1., abs(want)):
                mismatches += 1
                print('MISMATCH', item['name'], tag, got, want)
        if 'localization/pos_err_m' in tags:
            vals = [s.value for s in acc.Scalars('localization/pos_err_m')]
            points += len(vals)
            p90 = float(np.percentile(vals, 90))
            want = summary['all']['pos_p90_m']
            checked += 1
            if abs(p90 - want) > 5e-4:
                mismatches += 1
                print('SERIES MISMATCH', item['name'], p90, want)
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
    ap.add_argument('--snapshot', default='0926-markerless-probe')
    ap.add_argument('--source-sha', default='')
    ap.add_argument('--verify', action='store_true')
    ap.add_argument('--report')
    args = ap.parse_args(argv)
    if args.verify:
        raise SystemExit(verify(args))
    build(args)


if __name__ == '__main__':
    main()
