"""Observer-only analysis of the zone team A2 smokes: per-run table, blockers/failures, hashes, mp4 renders.

Reads only the run outputs (result.json, teacher-events.json, replay/). Renders
reuse the hard-routes observer renderer (experiments/2026-09-25-zone-hard-routes/analyze_runs.py).

  .venv-sim/bin/python experiments/2026-09-25-zone-team-a2/analyze.py --runs outputs/zone-team-a2-20260925 \
      --media outputs/zone-team-a2-20260925/media --render a-two-dynamic-s11 b-two-tri-dynamic-s11 c-two-graspfail-dynamic-s11
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def metrics(run_dir, row):
    r = json.loads((run_dir/'result.json').read_text())
    ex = r.get('team_executor') or {}
    jobs = ex.get('jobs', [])
    items = {}
    for iid, it in (r.get('items') or {}).items():
        ref = it.get('referee') or {}
        states = [j['state'] for j in it['jobs']]
        items[iid] = {'kind': it['kind'], 'referee_zone': ref.get('zone'), 'footprint': ref.get('footprint'),
                      'resting': ref.get('resting'), 'job_states': states,
                      'claims': len(it['claims']),
                      'claim_outcomes': sorted({c['outcome'] for c in it['claims'] if c['outcome']})}
    formation = {j['job_id']: j['formation_wait_s'] for j in jobs if len(j['participants']) > 1}
    solo_waits = [w for j in jobs if len(j['participants']) == 1 for w in j['formation_wait_s'].values()]
    pauses = [dict(p, job=j['job_id']) for j in jobs for p in j.get('pauses', [])]
    failures = [dict(f, job=j['job_id']) for j in jobs for f in j.get('failures', [])]
    slip = {j['job_id']: j.get('slip_mm') for j in jobs if j.get('state') in ('FINISHED', 'ABORTED')}
    door_waits = {}
    for e in r.get('door_waits', []):
        door_waits[e['passage']] = round(door_waits.get(e['passage'], 0.) + e['wait_s'], 2)
    gate_waits = {}
    for e in r.get('door_gate_waits', []):
        gate_waits[e['passage']] = round(gate_waits.get(e['passage'], 0.) + e['wait_s'], 2)
    gate_limits = sum(1 for e in r.get('door_gate_waits', []) if e.get('reason') == 'limit')
    claims = ex.get('claims', [])
    outcomes = {}
    for c in claims:
        outcomes[c['outcome'] or 'live'] = outcomes.get(c['outcome'] or 'live', 0) + 1
    return {**{k: row.get(k) for k in ('run', 'variant', 'seed', 'coordination', 'extra', 'wall_s',
                                       'load_1min_start', 'load_1min_end', 'returncode')},
            'phase': r['phase'], 'error': r['error'], 'goal': r['goal'],
            'referee_v2_goal_met': r['referee_v2']['goal_met'], 'referee_v2_zone_counts': r['referee_v2']['zone_counts'],
            'centre_point_counts': r['referee_v2']['centre_point_counts'], 'goal_met_rgb': r['goal_met_rgb'],
            'control_end_sim_s': r.get('control_end_sim_s'), 'max_sim_s': r.get('max_sim_s'),
            'sim_end_s': r.get('sim_end_s'), 'llm_calls': r['llm_calls'], 'eq_active_max': r['eq_active_max'],
            'neq': r.get('neq'), 'items': items, 'claim_outcomes': outcomes,
            'team_formation_wait_s': formation, 'solo_station_wait_s_max': max(solo_waits, default=None),
            'door_wait_s': door_waits, 'door_gate_wait_s': gate_waits, 'door_gate_limits': gate_limits,
            'door_standoff_ticks': r.get('door_standoffs'), 'team_pauses': pauses,
            'team_pause_near_passage_s': round(sum(p.get('s', 0.) for p in pauses if p.get('near_passage')), 2),
            'failures': failures, 'grip_lost': [g for j in jobs for g in j.get('grip_lost', [])],
            'drops': sum(j.get('drops', 0) for j in jobs), 'slip_mm': slip,
            'routes': [{k: rt.get(k) for k in ('job_id', 'ok', 'reason', 'legs', 'reference_s', 'wall_s')}
                       for rt in ex.get('routes', [])],
            'injection': ex.get('injection'), 'robot_events': ex.get('robot_events', []),
            'executor_metrics': ex.get('metrics'), 'coordination_stats': r.get('coordination_stats'),
            'sha256': {name: sha(run_dir/name) for name in ('result.json', 'teacher-events.json', 'scene.xml',
                                                            'item-labels.json', 'task.json')}
            | {'replay_manifest': sha(run_dir/'replay'/'replay.json')}}


def _renderer():
    path = ROOT/'experiments'/'2026-09-25-zone-hard-routes'/'analyze_runs.py'
    spec = importlib.util.spec_from_file_location('hard_routes_analyze', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.render_mp4


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--runs', type=Path, required=True)
    p.add_argument('--media', type=Path)
    p.add_argument('--render', nargs='*', default=[])
    p.add_argument('--out', type=Path)
    args = p.parse_args()
    rows = []
    for row_file in sorted((args.runs/'runs').glob('*.json')):
        row = json.loads(row_file.read_text())
        if (args.runs/row['run']/'result.json').is_file():
            rows.append(metrics(args.runs/row['run'], row))
    out = args.out or args.runs/'analysis.json'
    out.write_text(json.dumps(rows, indent=2, ensure_ascii=False) + '\n')
    for m in rows:
        print(json.dumps({k: m[k] for k in ('run', 'phase', 'referee_v2_goal_met', 'control_end_sim_s',
                                             'eq_active_max', 'claim_outcomes', 'drops')}))
    if args.render:
        render = _renderer()
        args.media.mkdir(parents=True, exist_ok=True)
        for run in args.render:
            target = args.media/f'{run}.mp4'
            render(args.runs/run, target)
            print('rendered', target, sha(target))


if __name__ == '__main__':
    main()
