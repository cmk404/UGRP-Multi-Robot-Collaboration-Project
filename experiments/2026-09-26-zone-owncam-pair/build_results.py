"""Aggregate the own-camera pair beam feasibility study into results.json (+ derived TensorBoard view).

Reads only local raw outputs (primary checkout outputs/, gitignored), records paths and SHA-256, and
enforces the FULL pre-registered seed list (a missing seed is ``missing/infrastructure_failure`` and
blocks the report). Usage:
  python experiments/2026-09-26-zone-owncam-pair/build_results.py <cohort-dir> [--tensorboard <snapshot>]
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RAW = Path('/Users/changmin/projects/ugrp/outputs/zone-owncam-pair-20260926')
TB = Path('/Users/changmin/projects/ugrp/outputs/tensorboard')
ARMS = {'O': ('own_only', (611, 612, 613, 614)), 'S': ('stub_approach', (611, 612))}
DEV = ('dev/601-own-7d97bab-calib', 'dev/602-own-dd70122')
BARRIERS = ('lift', 'carry', 'lower', 'open')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarise(folder):
    r = json.loads((folder / 'result.json').read_text())
    events = [json.loads(line) for line in (folder / 'events.jsonl').read_text().splitlines() if line.strip()]
    go = {b: {e['robot']: e['sim_time_s'] for e in events if e.get('event') == 'barrier_go' and e.get('barrier') == b}
          for b in BARRIERS}
    skew = {b: (round(abs(v['r1'] - v['r2']), 3) if set(v) == {'r1', 'r2'} else None) for b, v in go.items()}
    carry = [e for e in events if e.get('event') == 'carry_view']
    ev = r['evaluation_only']
    held = ev.get('both_gripped_gt') and ev.get('lifted_clear_gt')
    fails = r['failures']
    return {'raw_dir': str(folder), 'result_sha256': sha(folder / 'result.json'),
            'events_sha256': sha(folder / 'events.jsonl'), 'seed': r['seed'], 'condition': r['condition'],
            'source_sha': r.get('source_sha'), 'dirty_source': r.get('dirty_source'),
            'final_states': r['final_states'], 'failures': fails, 'claims': r['claims'],
            'aligned': {k: bool(v.get('aligned')) for k, v in r['claims'].items()},
            'align_errors': {k: v.get('aligned', {}).get('errors') for k, v in r['claims'].items()},
            'barrier_go_skew_s': skew, 'evaluation_only': ev,
            'false_load_alarm': bool(held) and any(f and 'LOAD_CHANGED' in f for f in fails.values()),
            'min_hold_ratio': min((e['hold_ratio'] for e in carry if 'hold_ratio' in e), default=None),
            'min_held_iou': min((e['held_iou'] for e in carry), default=None),
            'stub_approach': r.get('stub_approach'), 'pose_source': r.get('pose_source'),
            'counts_as_m1': r.get('counts_as_m1'), 'commands': r.get('commands'),
            'sim_seconds': r.get('sim_seconds'), 'wall_seconds': r.get('wall_seconds'),
            'load_average_start': r.get('load_average_start'), 'load_average_end': r.get('load_average_end'),
            'carry_odometry_calibration': r.get('carry_odometry_calibration'), 'thresholds': r.get('thresholds'),
            'contact_profile': r.get('contact_profile'), 'weld': r.get('weld')}


def gates(runs):
    own = [x for x in runs if x['condition'] == 'own_only']
    f1 = sum(all(x['aligned'].values()) and bool(x['evaluation_only'].get('both_gripped_gt')) for x in own)
    f2 = all(all(v is not None and v <= .2 for v in x['barrier_go_skew_s'].values()) for x in runs
             if all(s == 'done' for s in x['final_states'].values()))
    lifted = [x for x in runs if x['evaluation_only'].get('lifted_clear_gt')]
    f3 = (all(x['evaluation_only']['max_tilt_deg_lifted'] <= 5 for x in lifted)
          and not any(x['false_load_alarm'] for x in runs))
    f4 = sum(bool(x['evaluation_only'].get('on_floor_released')) and x['evaluation_only']['final_error_m'] <= .10
             for x in own)
    f5 = (all(x['evaluation_only'].get('weld_eq_active_max') == 0 for x in runs)
          and all(x['pose_source'] == 'none' for x in own))
    return {'F1_aligned_and_both_gripped_own': f'{f1}/{len(own)}', 'F1_pass': f1 >= 3,
            'F2_barrier_skew_le_0.2s_all_completed': f2,
            'F3_tilt_le_5_and_no_false_load_alarm': f3,
            'F4_placed_within_0.10m_own': f'{f4}/{len(own)}', 'F4_pass': f4 >= 3, 'F5': f5}


def main():
    cohort = sys.argv[1]
    runs, missing = [], []
    for arm, (condition, seeds) in ARMS.items():
        for seed in seeds:
            folder = RAW / cohort / arm / str(seed)
            if (folder / 'result.json').exists():
                runs.append({'arm': arm, **summarise(folder)})
            else:
                missing.append({'arm': arm, 'seed': seed, 'status': 'missing/infrastructure_failure'})
    if missing and '--allow-incomplete' not in sys.argv:
        raise SystemExit(f'pre-registered runs missing {missing}; the cohort report is blocked')
    log = RAW / cohort / 'cohort.log'
    out = {'experiment_id': '2026-09-26-zone-owncam-pair', 'raw_root_local_only': str(RAW),
           'raw_note': 'raw outputs are local (gitignored); not a remote backup', 'cohort': cohort,
           'cohort_log': {'path': str(log), 'sha256': sha(log)} if log.exists() else None,
           'runs': runs, 'missing': missing, 'report_blocked': bool(missing), 'gates': gates(runs),
           'development_runs': [summarise(RAW / d) for d in DEV if (RAW / d / 'result.json').exists()]}
    (HERE / 'results.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    print(json.dumps(out['gates'], ensure_ascii=False))
    if '--tensorboard' in sys.argv:
        snapshot = sys.argv[sys.argv.index('--tensorboard') + 1]
        tensorboard(runs, out['development_runs'], cohort, snapshot)


def tensorboard(runs, dev, cohort, snapshot):
    view = RAW / 'tensorboard-view'
    names = {}
    for x in runs + [{'arm': 'dev', **d} for d in dev]:
        name = f"pair{x['arm']}-s{x['seed']}" if x['arm'] != 'dev' else f"pairdev-{Path(x['raw_dir']).name}"
        ev = x['evaluation_only']
        derived = {'derived_view_only': True, 'derived_from': x['raw_dir'], 'source_result_sha256': x['result_sha256'],
                   'success': False, 'success_definition': 'feasibility study: no success claim; see evaluation',
                   'mode': 'diagnostic', 'counts_as_m1': False, 'm1_success': False,
                   'diagnostic_success': bool(ev.get('success_gt')), 'pose_source': x['pose_source'],
                   'pose_sources_seen': [x['pose_source']],
                   'input_contract': {'camera': 'own robot_cam only', 'sync': 'PairCarrySync (own frame ids)',
                                      'stub_approach_gt': x['condition'] == 'stub_approach'},
                   'success_semantics': 'success = m1_success (always false here); diagnostic_success = GT placement',
                   'stop_reason': json.dumps(x['failures']), 'scope': 'own-camera pair beam feasibility, open floor',
                   'policy': 'owncam_pair_beam_v1', 'case': f"pair {x['condition']}",
                   'config': {'contact_profile': x['contact_profile'], 'condition': x['condition']},
                   'sim_s': x['sim_seconds'], 'wall_s': x['wall_seconds'],
                   'commands': sum((x['commands'] or {}).values()) if isinstance(x['commands'], dict) else x['commands'],
                   'model_calls': 0, 'evaluation': {**ev, 'barrier_go_skew_s': x['barrier_go_skew_s'],
                                                   'min_hold_ratio': x['min_hold_ratio'],
                                                   'min_held_iou': x['min_held_iou']},
                   'seed': x['seed'], 'source_sha': x['source_sha']}
        (view / name).mkdir(parents=True, exist_ok=True)
        (view / name / 'result.json').write_text(json.dumps(derived, indent=1) + '\n')
        names[name] = f"{x['condition']} ({'dev' if x['arm'] == 'dev' else 'pre-registered ' + cohort})"
    target = TB / snapshot
    if target.exists():
        raise SystemExit(f'{target} exists; snapshots are never overwritten')
    cmd = [sys.executable, str(ROOT / 'scripts/export_tensorboard.py'), '--output', str(target), '--max-images', '0']
    for name in names:
        cmd += ['--source', str(view / name)]
    subprocess.run(cmd, check=True)
    collection = json.loads((target / 'collection.json').read_text())
    for entry in collection['exported']:
        short = Path(entry['source']).name
        old = target / entry['name']
        if old.exists() and entry['name'] != short:
            old.rename(target / short)
        entry['original_name'], entry['name'] = entry['name'], short
        entry['condition'] = names[short] + '; own robot_cam only, NOT M1, success=m1_success=false'
    (target / 'collection.json').write_text(json.dumps(collection, indent=2) + '\n')
    print(json.dumps({'snapshot': str(target), 'runs': len(collection['exported']), 'failed': collection.get('failed')}))


if __name__ == '__main__':
    main()
