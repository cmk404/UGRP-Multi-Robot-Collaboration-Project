"""Derived TensorBoard view for the memory ON/OFF experiment (results.json -> derived result.json -> snapshot).

Every derived run is one episode row of ``results.json`` (built by build_results.py from the raw
folders, which are never modified). success = m1_success (the frozen M1 judge); memory_v2 runs are
labelled "interim, tag provider". Robot-side counts (looks, pan dwells, look time, commands) come from
the robot's own issued commands; localization error and false confirmations are evaluation-only
(GT) numbers, shown as offline/* scalars with results.json as their hashed source.
Snapshots are never overwritten. The exporter is the primary checkout's scripts/export_tensorboard.py.

  python experiments/2026-09-26-zone-owncam-memory/build_tensorboard.py <snapshot-name> [--attempt test-a1 ...]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PRIMARY = Path('/Users/changmin/projects/ugrp')
TB = PRIMARY/'outputs'/'tensorboard'
VIEW_ROOT = PRIMARY/'outputs'/'owncam-memory-20260926'/'tensorboard-view'
PY = PRIMARY/'.venv-sim-worker-mac'/'bin'/'python'
SHORT = {'off': 'off', 'memory_v1': 'memv1', 'memory_v2': 'memv2'}


def sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def derived(row: dict, results_path: Path, results_sha: str) -> dict:
    mem = row.get('memory') or {}
    loc = row.get('localization') or {}
    offline = {'offline/look_sweeps': row['look_sweeps'], 'offline/look_dwells': row['look_dwells'],
               'offline/look_time_s': row['look_time_s'], 'offline/search_sweeps': row['search_sweeps'],
               'offline/search_dwells': row['search_dwells'],
               'offline/eval_pos_err_p50_m': loc.get('pos_err_m_p50'), 'offline/eval_pos_err_p90_m': loc.get('pos_err_m_p90'),
               'offline/eval_yaw_err_p90_deg': loc.get('yaw_err_deg_p90'),
               'offline/eval_carry_pos_err_p90_m': loc.get('carry_pos_err_m_p90'),
               'offline/eval_gate_pos_err_m': loc.get('gate_pos_err_m'),
               'offline/eval_search_target_err_m': row.get('search_target_error_m'),
               'offline/false_success': int(bool(row['false_success']))}
    if row['condition'].startswith('memory'):
        offline.update({'offline/false_confirmed_tracks': len(mem.get('false_confirmed_tracks') or []),
                        'offline/false_free_cells_near_boxes': len(mem.get('false_free_cells_near_boxes') or []),
                        'offline/confirmed_tracks': mem.get('confirmed_tracks')})
    offline = {k: v for k, v in offline.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}
    label = row.get('result_label')
    return {'derived_view_only': True, 'derived_from': row['folder'],
            'success': bool(row['m1_success']), 'm1_success': bool(row['m1_success']),
            'false_success': bool(row['false_success']), 'stop_reason': row['outcome'],
            'policy': f"M1 own-camera, {row['condition']}" + (f' ({label})' if label else ''),
            'case': f"{row['attempt']} {row['episode']}", 'condition': row['condition'] + (f' / {label}' if label else ''),
            'seed': row['seed'], 'source_sha': row['code_sha'], 'scope': (
                'M1 single-robot own-camera delivery (frozen M1 judge). memory ON vs OFF; sync SIM; '
                'cargo_noslip_v1; weld OFF; no LLM' + ('; interim, tag provider' if label else '')),
            'clock': 'sync SIM', 'sim_s': row['sim_s'], 'wall_s': row['wall_s'], 'commands': row['commands'],
            'model_calls': 0, 'config': {'contact_profile': row['contact_profile'], 'map': row['map']},
            'evaluation': {'m1_success': row['m1_success'], 'false_success': row['false_success'],
                           'outcome': row['outcome'], 'gt_box_final_xyz': row.get('gt_box_final_xyz'),
                           'slot_xy': row.get('slot_xy'), 'contacts': row.get('contacts'),
                           'memory': {k: mem.get(k) for k in ('false_confirmed_tracks', 'false_free_cells_near_boxes',
                                                              'event_counts', 'gate_look_modes')} if mem else None},
            'offline_scalars': offline,
            'offline_scalar_scope': ('robot-side looks/pans from own issued commands; eval_* = evaluation-only GT '
                                     'localization / box error; false_* = evaluation-only false confirmations'),
            'offline_source': {'path': str(results_path), 'sha256': results_sha},
            'offline_source_pointer': f"episodes[attempt={row['attempt']}, condition={row['condition']}, episode={row['episode']}]"}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('snapshot')
    p.add_argument('--attempt', action='append', default=[])
    p.add_argument('--results', default=str(HERE/'results.json'))
    args = p.parse_args(argv)
    results_path = Path(args.results).resolve()
    results = json.loads(results_path.read_text())
    digest = sha(results_path)
    target = TB/args.snapshot
    if target.exists():
        raise SystemExit(f'{target} exists; snapshots are never overwritten')
    view = VIEW_ROOT/args.snapshot
    if view.exists():
        raise SystemExit(f'{view} exists')
    names, conditions = [], {}
    for row in results['episodes']:
        if args.attempt and row['attempt'] not in args.attempt:
            continue
        if row.get('infrastructure_failure'):
            print('skip (no result.json):', row['attempt'], row['condition'], row['episode'])
            continue
        name = f"{row['attempt']}-{SHORT[row['condition']]}-s{row['seed']}"
        folder = view/name
        folder.mkdir(parents=True)
        (folder/'result.json').write_text(json.dumps(derived(row, results_path, digest), indent=1, ensure_ascii=False) + '\n')
        names.append(name)
        conditions[name] = f"{row['condition']}{' (' + row['result_label'] + ')' if row.get('result_label') else ''}, " \
                           f"{row['split']} seed {row['seed']}, source {row['code_sha'][:8]}"
    cmd = [str(PY), str(PRIMARY/'scripts/export_tensorboard.py'), '--output', str(target), '--max-images', '0']
    for name in names:
        cmd += ['--source', str(view/name)]
    subprocess.run(cmd, check=True, cwd=PRIMARY)
    collection = json.loads((target/'collection.json').read_text())
    for entry in collection['exported']:
        short = Path(entry['source']).name
        old = target/entry['name']
        if old.exists() and entry['name'] != short:
            old.rename(target/short)
        entry['original_name'], entry['name'] = entry['name'], short
        entry['condition'] = conditions[short]
    collection['results_json'] = {'path': str(results_path), 'sha256': digest}
    (target/'collection.json').write_text(json.dumps(collection, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'snapshot': str(target), 'runs': len(collection['exported']), 'failed': collection.get('failed'),
                      'collection_sha256': sha(target/'collection.json')}))


if __name__ == '__main__':
    main()
