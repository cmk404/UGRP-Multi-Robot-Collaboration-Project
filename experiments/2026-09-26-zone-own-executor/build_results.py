"""Results table + derived TensorBoard view for the executor smoke (raw results are never modified).

Usage (from the worktree root, after both smoke episodes finished):
    python experiments/2026-09-26-zone-own-executor/build_results.py <smoke-dir> [--tensorboard <snapshot-name>]

Writes ``results.json`` and ``raw_index.json`` next to this file (gates E1-E6 evaluated as
pre-registered in prereg.json). With ``--tensorboard`` it writes one derived result per
robot-episode (``success`` = m1_success, M1 outcome block validated) and exports a NEW
snapshot under the shared ``outputs/tensorboard`` root; the M1 dev-a8 results (PR #201,
single robot) are added as the baseline when present.
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from harness import m1_contract  # noqa: E402

TB = Path('/Users/changmin/projects/ugrp/outputs/tensorboard')
M1_RAW = Path('/Users/changmin/projects/ugrp/outputs/m1-owncam-20260926')
M1_BASELINE = ('dev-a8/m1dev-s93', 'dev-a8/m1devdiag-s95')
EPISODES = ('smoke-s700', 'smoke-s701')


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def jsonl(p):
    return [json.loads(line) for line in Path(p).read_text().splitlines() if line.strip()]


def gates(smoke):
    rows, per = {}, {}
    for ep in EPISODES:
        d = smoke / ep
        res = d / 'result.json'
        per[ep] = {'result': res.exists()}
        if not res.exists():
            continue
        r = json.loads(res.read_text())
        m = json.loads((d / 'manifest.json').read_text())
        per[ep].update(r=r, m=m, events=jsonl(d / 'study' / 'events.jsonl'), api=jsonl(d / 'study' / 'api_calls.jsonl'))
    ok = [ep for ep in EPISODES if per[ep]['result']]
    rows['E1'] = {'pass': len(ok) == 2 and all(per[e]['r']['host_exception'] is None for e in ok),
                  'detail': {e: (per[e]['result'] and per[e]['r']['host_exception']) for e in EPISODES}}
    e2 = {}
    for e in ok:
        accepted = [a['job_id'] for a in per[e]['api'] if a['accepted'] and a['api'] != 'abort']
        ev = per[e]['events']
        starts = {j: sum(1 for x in ev if x['job_id'] == j and x['event'] == 'job_started') for j in accepted}
        ends = {j: sum(1 for x in ev if x['job_id'] == j and x['event'] in ('job_done', 'job_failed')) for j in accepted}
        got_deliver = {rid for a in per[e]['api'] if a['accepted'] and a['api'] == 'deliver' for rid in [a['robot_id']]}
        e2[e] = {'starts_ok': all(v == 1 for v in starts.values()), 'ends_ok': all(v == 1 for v in ends.values()),
                 'open_jobs_at_end': [j for j, v in ends.items() if v == 0],
                 'every_robot_got_deliver': got_deliver == {'r1', 'r2', 'r3'}}
    rows['E2'] = {'pass': bool(ok) and len(ok) == 2 and all(v['starts_ok'] and v['ends_ok'] and v['every_robot_got_deliver']
                                                             for v in e2.values()), 'detail': e2}
    rows['E3'] = {'pass': len(ok) == 2 and all(rr['foreign_frames_fed'] == 0 and rr['executor_summary']['cameras_seen'] ==
                                               ['robot_cam'] for e in ok for rr in per[e]['r']['robots'].values()),
                  'detail': {e: {k: (v['foreign_frames_fed'], v['executor_summary']['cameras_seen'])
                                 for k, v in per[e]['r']['robots'].items()} for e in ok}}
    # E4 as registered has two clauses. (a) own-camera pose sources only. (b) "counts_as_m1 true for every robot
    # whose deliver job reached the own-RGB search". Every deliver job reached the search, and the M1 judge sets
    # counts_as_m1 only when the search FOUND the target (pickup_from_own_rgb), so (b) read literally fails for
    # every SEARCH_NOT_FOUND robot. Both clauses are reported; the gate is the literal reading (no reinterpretation).
    sources_ok = len(ok) == 2 and all(rr['pose_sources_seen'] and all(s.startswith('owncam_pf_v2:') for s in
                                                                       rr['pose_sources_seen'])
                                      for e in ok for rr in per[e]['r']['robots'].values())
    counts_ok = len(ok) == 2 and all(rr['counts_as_m1'] for e in ok for rr in per[e]['r']['robots'].values()
                                     if rr['deliver_outcome'] is not None)
    rows['E4'] = {'pass': sources_ok and counts_ok, 'clause_a_own_camera_sources': sources_ok,
                  'clause_b_counts_as_m1_literal': counts_ok,
                  'detail': {e: {k: {'sources': v['pose_sources_seen'], 'counts_as_m1': v['counts_as_m1'],
                                     'pickup_from_own_rgb': v['m1_checks']['pickup_from_own_rgb']}
                                 for k, v in per[e]['r']['robots'].items()} for e in ok}}
    rows['E5'] = {'pass': len(ok) == 2 and all(per[e]['r']['weld_max_eq_active'] == 0 and
                                               per[e]['r']['contact_profile']['noslip_iterations'] > 0 for e in ok),
                  'detail': {e: (per[e]['r']['weld_max_eq_active'], per[e]['r']['contact_profile']['noslip_iterations'])
                             for e in ok}}
    rows['E6'] = {'pass': len(ok) == 2 and all(per[e]['r']['false_confirmations'] == 0 for e in ok),
                  'detail': {e: per[e]['r']['false_confirmations'] for e in ok}}
    return rows, per


def robot_row(ep, rid, rr, r):
    jobs = rr['jobs']
    deliver = next((j for j in jobs if j['kind'] == 'deliver'), None)
    ev = rr['evaluation_only']
    return {'episode': ep, 'robot_id': rid, 'order': rr['order']['order_id'],
            'pickup_slot': rr['order']['initial_location']['slot'], 'zone_slot': rr['zone_slot'],
            'deliver_outcome': rr['deliver_outcome'], 'confirmation': rr['deliver_confirmation'],
            'deliver_sim_s': rr['deliver_sim_s'],
            'deliver_duration_s': None if deliver is None else round(deliver['ended_at_sim_s'] - deliver['started_at_sim_s'], 2),
            'm1_success': rr['m1_success'], 'm1_failed_checks': rr['m1_failed_checks'],
            'diagnostic_success': rr['diagnostic_success'], 'false_success': rr['false_success'],
            'events': rr['events'], 'gt_assigned_box_in_slot': ev['assigned_box_in_slot'],
            'contact_steps': ev['contact_steps'], 'retention': ev['retention'],
            'pose_error_eval_only': r['evaluation_only_pose_error'].get(rid), 'frames': rr['frames'],
            'commands': rr['commands'], 'exception': (rr['exception'] or {}).get('type')}


def derived(view, name, payload):
    m1_contract.validate_outcome(payload)
    folder = view / name
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'result.json').write_text(json.dumps(payload, indent=1, default=str) + '\n')
    return name


def main():
    smoke = Path(sys.argv[1]).resolve()
    snapshot = sys.argv[sys.argv.index('--tensorboard') + 1] if '--tensorboard' in sys.argv else None
    rows, per = gates(smoke)
    robots = [robot_row(e, rid, rr, per[e]['r']) for e in EPISODES if per[e]['result']
              for rid, rr in sorted(per[e]['r']['robots'].items())]
    results = {'schema': 'ugrp.zone_own_executor_smoke_results.v1', 'smoke_dir': str(smoke),
               'source_sha': sorted({per[e]['m']['code']['sha'] for e in EPISODES if per[e]['result']}),
               'dirty': sorted({per[e]['m']['code']['dirty'] for e in EPISODES if per[e]['result']}),
               'gates': rows, 'smoke_pass': all(g['pass'] for g in rows.values()),
               'm1_style_success': f"{sum(r['m1_success'] for r in robots)}/{len(robots)}",
               'own_camera_confirmed': f"{sum(r['confirmation'] == 'own_camera_confirmed' for r in robots)}/{len(robots)}",
               'robots': robots,
               'runs': {e: {'run': per[e]['r']['run'], 'wall_s': per[e]['m']['wall_s'],
                            'load_average': per[e]['m']['load_average']} for e in EPISODES if per[e]['result']}}
    (HERE / 'results.json').write_text(json.dumps(results, indent=1, ensure_ascii=False, default=str) + '\n')
    index = {str(p.relative_to(smoke)): {'bytes': p.stat().st_size, 'sha256': sha(p)}
             for p in sorted(smoke.rglob('*')) if p.is_file() and p.suffix in ('.json', '.jsonl', '.log', '.txt')}
    (HERE / 'raw_index.json').write_text(json.dumps({'root': str(smoke), 'note': 'local only, not a remote backup; '
                                                     'frames/*.jpg and scene.xml hashed in each manifest',
                                                     'files': index}, indent=1) + '\n')
    print(json.dumps({'smoke_pass': results['smoke_pass'], 'gates': {k: v['pass'] for k, v in rows.items()},
                      'm1_style': results['m1_style_success'], 'confirmed': results['own_camera_confirmed']}))
    if not snapshot or '--no-export' in sys.argv:
        return
    view = smoke.parent / f'tensorboard-view-{smoke.name}'
    runs, conditions = [], {}
    for row in robots:
        e, rid = row['episode'], row['robot_id']
        rr = per[e]['r']['robots'][rid]
        name = f"exec-{e.split('-')[1]}-{rid}"
        payload = {'derived_view_only': True, 'derived_from': str(smoke / e), 'source_result_sha256': sha(smoke / e / 'result.json'),
                   **{k: rr[k] for k in m1_contract.REQUIRED_OUTCOME_KEYS}, 'success': rr['m1_success'],
                   'stop_reason': row['deliver_outcome'], 'policy': 'zone_own_executor v1 + M1 chain + skill v9',
                   'case': f"{e} {rid} {row['pickup_slot']}->{row['zone_slot']}",
                   'config': {'contact_profile': 'cargo_noslip_v1', 'robots': 3, 'scripted_no_llm': True,
                              'hold_before_deliver_s': per[e]['r']['scripted_jobs'][rid]['hold_s']},
                   'sim_s': row['deliver_duration_s'], 'wall_s': per[e]['m']['wall_s'], 'commands': row['commands'],
                   'model_calls': 0,
                   'evaluation': {'m1_success': rr['m1_success'], 'diagnostic_success': rr['diagnostic_success'],
                                  'counts_as_m1': rr['counts_as_m1'], 'own_camera_confirmed':
                                  row['confirmation'] == 'own_camera_confirmed',
                                  'false_confirmation': rr['evaluation_only']['false_confirmation'],
                                  'failed_checks': rr['m1_failed_checks']},
                   'seed': per[e]['r']['seed'], 'source_sha': per[e]['m']['code']['sha']}
        runs.append(derived(view, name, payload))
        conditions[name] = f'executor smoke {e} robot {rid} (3 robots in one world, scripted, no LLM)'
    for rel in M1_BASELINE:
        src = M1_RAW / rel / 'result.json'
        if not src.exists():
            continue
        r = json.loads(src.read_text())
        name = 'm1base-' + rel.replace('/', '-')
        payload = {'derived_view_only': True, 'derived_from': str(src.parent), 'source_result_sha256': sha(src),
                   **{k: r[k] for k in m1_contract.REQUIRED_OUTCOME_KEYS}, 'success': r['m1_success'],
                   'stop_reason': r['outcome'], 'policy': 'M1 runner (PR #201), single robot', 'case': rel,
                   'config': {'contact_profile': 'cargo_noslip_v1', 'robots': 1}, 'sim_s': r['sim_s'],
                   'commands': r['commands'], 'model_calls': 0,
                   'evaluation': {'m1_success': r['m1_success'], 'diagnostic_success': r['diagnostic_success'],
                                  'counts_as_m1': r['counts_as_m1']}, 'seed': r.get('seed')}
        runs.append(derived(view, name, payload))
        conditions[name] = 'baseline: PR #201 M1 dev-a8 (single robot, other agent, not pooled)'
    target = TB / snapshot
    if target.exists():
        raise SystemExit(f'{target} exists; snapshots are never overwritten')
    cmd = [sys.executable, str(ROOT / 'scripts/export_tensorboard.py'), '--output', str(target), '--max-images', '0']
    for name in runs:
        cmd += ['--source', str(view / name)]
    subprocess.run(cmd, check=True)
    collection = json.loads((target / 'collection.json').read_text())
    for entry in collection['exported']:
        short = Path(entry['source']).name
        old = target / entry['name']
        if old.exists() and entry['name'] != short:
            old.rename(target / short)
        entry['original_name'], entry['name'] = entry['name'], short
        d = json.loads((view / short / 'result.json').read_text())
        entry['condition'] = conditions[short] + f"; success=m1_success={d['m1_success']}, counts_as_m1={d['counts_as_m1']}"
    (target / 'collection.json').write_text(json.dumps(collection, indent=2) + '\n')
    print(json.dumps({'snapshot': str(target), 'runs': len(collection['exported']), 'failed': collection.get('failed')}))


if __name__ == '__main__':
    main()
