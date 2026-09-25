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


CAUSE = {
    'teacher_path_blocked': 'no disc path to the station/landing for the drive-phase limit (240 s with walls), '
                            'or its goal occupied',
    'path_blocked': 'no disc path to the station/landing for the drive-phase limit (240 s with walls)',
    'rendezvous_timeout': 'team not complete within 60 SIM s after this robot reached its station',
    'station_blocked': 'another robot stood nearer this robot\'s station (physical L1 rule)',
    'box_taken_by_peer': 'item already being grasped or delivered when this robot got there',
    'no_team_route': 'team route search found no swept-clear route before contact (cancel_retreat)',
    'team_aborted': 'team job aborted after a failure (see team_failure)',
    'no_landing_area': 'no free landing area of this kind left in the zone',
    'landing_align_timeout': 'solo landing alignment did not converge in 20 s',
}


def blockers(run_dir, r):
    """Every place a run blocked or failed, with its recorded cause (observer-only, from the run outputs)."""
    events = json.loads((run_dir/'teacher-events.json').read_text())
    out = []
    for e in events:
        t, rid = e.get('sim_time_s'), e.get('robot_id')
        ev = e['event']
        if ev == 'claim_end' and e.get('outcome') != 'placed_by_teacher':
            out.append({'t': t, 'robot': rid, 'kind': 'claim_end', 'outcome': e.get('outcome'), 'item': e.get('item'),
                        'cause': CAUSE.get(e.get('outcome'), e.get('outcome'))})
        elif ev == 'team_failure':
            out.append({'t': t, 'robot': rid, 'kind': 'team_failure', 'job': e.get('job'), 'outcome': e.get('reason'),
                        'action': e.get('action'), 'cause': e.get('reason')})
        elif ev in ('commit_rejected', 'no_landing_area'):
            out.append({'t': t, 'robot': rid, 'kind': ev, 'item': e.get('item'), 'cause': e.get('reason', ev)})
        elif ev == 'passage_gate_end' and e.get('reason') == 'limit':
            out.append({'t': t, 'robot': rid, 'kind': 'door_gate_limit', 'passage': e.get('passage'),
                        'cause': f"held {e.get('wait_s')} s at a single-lane entry, then went on"})
        elif ev == 'passage_wait' and e.get('wait_s', 0) >= 20:
            out.append({'t': t, 'robot': rid, 'kind': 'door_wait', 'passage': e.get('passage'),
                        'cause': f"still {e.get('wait_s')} s inside the passage zone (standoff/blocked)"})
    for j in (r.get('team_executor') or {}).get('jobs', []):
        for pz in j.get('pauses', []):
            if pz.get('s', 0) >= 20:
                out.append({'t': pz.get('t0'), 'robot': None, 'kind': 'team_carry_pause', 'job': j['job_id'],
                            'cause': f"carry paused {pz.get('s')} s for {pz.get('by')} near {pz.get('near_passage')}"})
    wanted = {k for z in r['goal'].values() for k in z}
    for iid, it in (r.get('referee_v2') or {}).get('items', {}).items():
        if it['kind'] in wanted and (it.get('zone') is None or it.get('footprint') != 'inside'):
            out.append({'t': r.get('sim_end_s'), 'robot': None, 'kind': 'not_delivered_at_end', 'item': iid,
                        'cause': f"referee_v2: zone {it.get('zone')}, footprint {it.get('footprint')}"})
    if r.get('phase') != 'FINISHED' or r.get('error'):
        out.append({'t': r.get('sim_end_s'), 'robot': None, 'kind': 'run_end', 'outcome': r.get('phase'),
                    'cause': r.get('error') or r.get('phase')})
    return sorted(out, key=lambda b: (b['t'] is None, b['t'] or 0))


def ledger_checks(ex):
    """Double membership (a robot in two live jobs at once), once-per-item decrement, barrier history."""
    jobs = ex.get('jobs', [])
    overlaps = 0
    spans = {}
    for j in jobs:
        for rid in j['participants']:
            spans.setdefault(rid, []).append((j.get('commit_sim_s') or 0., j.get('end_sim_s') or 1e9, j['job_id']))
    for rid, ss in spans.items():
        ss.sort()
        overlaps += sum(1 for a, b in zip(ss, ss[1:]) if b[0] < a[1] - 1e-6)
    ledger = ex.get('ledger') or {}
    finished_items = sorted(j['job']['item_label'] for j in jobs if j.get('state') == 'FINISHED' and j.get('job'))
    delivered_n = sum(n for z in (ledger.get('delivered') or {}).values() for n in z.values())
    return {'double_membership': overlaps, 'finished_jobs': len(finished_items),
            'finished_items_unique': len(set(finished_items)), 'ledger_delivered_units': delivered_n,
            'decrement_once_per_item': delivered_n == len(set(finished_items)) == len(finished_items),
            'over_delivered': ledger.get('over_delivered'),
            'barrier_histories': sum(1 for j in jobs if (j.get('job') or {}).get('history'))}


def labels_vs_setup(run_dir, r):
    """Evaluation only: start label counts per kind against the setup (episode-setup-only.json)."""
    setup = json.loads((run_dir/'episode-setup-only.json').read_text())
    truth = {}
    for o in setup['setup_only']['objects'].values():
        truth[o['kind']] = truth.get(o['kind'], 0) + 1
    for c in setup['cargo_items']:
        truth[c['kind']] = truth.get(c['kind'], 0) + 1
    seen = {}
    for kind in (r.get('labels') or {}).values():
        seen[kind] = seen.get(kind, 0) + 1
    return {'setup': truth, 'labels': seen, 'match': truth == seen}


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
            'blockers': blockers(run_dir, r), 'perception_profile': r.get('perception_profile'),
            'ledger_checks': ledger_checks(ex),
            'labels_vs_setup': labels_vs_setup(run_dir, r),
            'unconfirmed_beams_final': len((r.get('final_rgb_view') or {}).get('unconfirmed_beams', [])),
            'contact_profile': (r.get('config') or {}).get('contact_profile_effective'),
            'condition_switches': r.get('condition_switches'),
            'robot_facing_outcome_source': (r.get('robot_facing_outcome_source') or {}).get('name'),
            'robot_facing_delivered': (r.get('robot_results') or {}).get('delivered_labels'),
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
