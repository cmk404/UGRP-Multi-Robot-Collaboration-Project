"""Collect the vision-localization results (test metrics, gate verdict, references, model, cost) into results.json.

Inputs are the files named in ``prereg.json`` plus the scored test metrics; the
gate is evaluated exactly as pre-registered. References are other environments
(tagged maps) and are reported next to, never merged with, the tag-free test.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PR210_RESULTS = ROOT/'experiments'/'2026-09-26-markerless-probe'/'results'/'metrics_test.json'
ENV_V3_LOOP_TEST = Path('/Users/changmin/projects/ugrp/outputs/zone-env-v3-20260926/loop/test')


def sha_file(p) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def pct(a, q):
    return None if len(a) == 0 else round(float(np.percentile(a, q)), 4)


def summ(errs):
    e = np.asarray([x[0] for x in errs])
    y = np.asarray([x[1] for x in errs])
    return {'n': int(e.size), 'pos_p50_m': pct(e, 50), 'pos_p90_m': pct(e, 90), 'pos_p99_m': pct(e, 99),
            'yaw_p90_deg': pct(y, 90)}


def reference_tags_m1() -> dict:
    """PR #210 test cohort (M1 frames, zone_wide_door_tags_v2, 0.10 m walls): the M1 tag PF as recorded online."""
    m = json.loads(PR210_RESULTS.read_text())
    tag = m['pooled']['tag_online']
    return {'label': 'tag PF, M1 test frames (tags_v2, 0.10 m walls)', 'environment': 'zone_wide_door_tags_v2',
            'episodes': 'm1test-s101..s106 (6, 12,238 frames)', 'scope': 'online tag PF of the M1 closed-loop runs',
            'summary': {'all': tag['all'], 'door_zone': tag['door_zone'], 'carry': tag.get('carry', {})},
            'source': {'file': str(PR210_RESULTS.relative_to(ROOT)), 'sha256': sha_file(PR210_RESULTS)}}


def reference_tags_v3() -> dict | None:
    """Environment v3 loop test (tags_v3, 0.40 m walls): recomputed from its raw frames_eval (student frames)."""
    if not ENV_V3_LOOP_TEST.exists():
        return None
    rows, files = [], {}
    for f in sorted(ENV_V3_LOOP_TEST.glob('envtest-*/eval_only/frames_eval.jsonl')):
        files[str(f)] = sha_file(f)
        for line in f.read_text().splitlines():
            e = json.loads(line)
            if e.get('pos_err_m') is None or e.get('phase') == 'teacher':
                continue
            gt = e['gt']
            door = abs(gt[0] - 2.2) < .6 and -.45 < gt[1] < .55
            loaded = '-box-' in f.parts[-3] and float(e.get('box_z', 0.)) > .04
            rows.append((float(e['pos_err_m']), float(e['yaw_err_deg']), door, loaded))
    return {'label': 'tag PF, environment v3 loop test (tags_v3, 0.40 m walls)', 'environment': 'zone_wide_door_tags_v3',
            'episodes': 'envtest-s721..s729 (9 closed-loop drives through door_1)',
            'scope': 'closed-loop own-camera loop test of kiro/zone-map-v3 (gate FAIL there); recomputed here from raw',
            'summary': {'all': summ([r[:2] for r in rows]), 'door_zone': summ([r[:2] for r in rows if r[2]]),
                        'door_loaded': summ([r[:2] for r in rows if r[2] and r[3]])},
            'source': {'files_sha256': files, 'recorded_by': 'kiro/zone-map-v3 007949bf (loop test record)'}}


def gate(prereg: dict, metrics: dict) -> dict:
    g = prereg['gate']
    s = metrics['pooled']['vision'].get('door_loaded', {})
    eps_with = [ep for ep, per in metrics['episodes'].items() if per.get('vision', {}).get('door_loaded', {}).get('n', 0)
                >= g['min_door_loaded_frames_per_episode']]
    checks = {
        'G1_lateral_p99': {'value_m': s.get('lat_abs_p99_m'), 'max_m': g['lateral_p99_max_m'],
                           'pass': s.get('lat_abs_p99_m') is not None and s['lat_abs_p99_m'] <= g['lateral_p99_max_m']},
        'G2_position_p90': {'value_m': s.get('pos_p90_m'), 'max_m': g['position_p90_max_m'],
                            'pass': s.get('pos_p90_m') is not None and s['pos_p90_m'] <= g['position_p90_max_m']},
        'G3_yaw_p90': {'value_deg': s.get('yaw_p90_deg'), 'max_deg': g['yaw_p90_max_deg'],
                       'pass': s.get('yaw_p90_deg') is not None and s['yaw_p90_deg'] <= g['yaw_p90_max_deg']},
        'G4_coverage': {'episodes_with_door_loaded': eps_with, 'min_episodes': g['min_episodes'],
                        'pass': len(eps_with) >= g['min_episodes']}}
    valid = checks['G4_coverage']['pass']
    ok = all(checks[k]['pass'] for k in ('G1_lateral_p99', 'G2_position_p90', 'G3_yaw_p90'))
    return {'checks': checks, 'valid': valid, 'pass': bool(valid and ok),
            'verdict': 'PASS' if valid and ok else ('INSUFFICIENT_DATA' if not valid else 'FAIL')}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--prereg', type=Path, default=HERE/'prereg.json')
    ap.add_argument('--metrics', type=Path, required=True)
    ap.add_argument('--estimates-dir', required=True)
    ap.add_argument('--train-info', type=Path, required=True)
    ap.add_argument('--bench', type=Path)
    ap.add_argument('--renders', type=Path, default=Path('/Users/changmin/projects/ugrp/outputs/vision-loc-20260926/render'))
    ap.add_argument('--output', type=Path, default=HERE/'results'/'results.json')
    args = ap.parse_args(argv)
    prereg = json.loads(args.prereg.read_text())
    metrics = json.loads(args.metrics.read_text())
    train = json.loads(args.train_info.read_text())
    renders = {}
    for d in sorted(args.renders.glob('vl-*')):
        m = d/'teacher_manifest.json'
        if m.exists():
            t = json.loads(m.read_text())
            renders[d.name] = {k: t[k] for k in ('split', 'role', 'outcome', 'diagnostic_success', 'sim_s', 'frames',
                                                 'wall_s', 'load_average')}
            renders[d.name]['teacher_manifest_sha256'] = sha_file(m)
    refs = {'tag_m1_v2': reference_tags_m1()}
    v3 = reference_tags_v3()
    if v3:
        refs['tag_env_v3'] = v3
    out = {'schema': 'ugrp.vision_loc.results.v1', 'prereg_sha256': sha_file(args.prereg),
           'test': {'episodes': prereg['test_episodes'], 'estimates_dir': args.estimates_dir, 'metrics': metrics,
                    'metrics_sha256': sha_file(args.metrics)},
           'gate': gate(prereg, metrics), 'references': refs,
           'model': {'sha256': train['sha256'], 'bytes': train['bytes'], 'params': train['params'],
                     'train_info': train, 'train_info_sha256': sha_file(args.train_info)},
           'inference_cost': json.loads(args.bench.read_text()) if args.bench and args.bench.exists() else None,
           'renders': renders}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    print(json.dumps(out['gate'], indent=1))


if __name__ == '__main__':
    main()
