"""Build results.json for experiments/2026-09-26-zone-teacher-fix from the raw cohort outputs (local only).

TEACHER FEASIBILITY ONLY: every number here is a GT teacher result (demonstrations, training targets,
scenario feasibility). The teacher is never the study executor and its success is never robot success.

Reuses the observer-only helpers of PR #169's analysis (experiments/2026-09-25-zone-team-a2/analyze.py:
blockers, blocker_summary, ledger_checks, labels_vs_setup) without modifying them.

  python3 experiments/2026-09-26-zone-teacher-fix/build_results.py \\
      --raw outputs/zone-teacher-fix-20260926 --b5 <b5-offline.json> --output experiments/.../results.json
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
A2 = HERE.parent/'2026-09-25-zone-team-a2'/'analyze.py'
NEW_SEED_RUNS = [f'mix-{m}-s{s}' for s in (21, 22, 23) for m in ('dynamic', 'independent')] + \
    [f'tri-dynamic-s{s}' for s in (21, 22)]
BLOCKER_RUNS = ['d-b3-mix-dynamic-s12', 'd-b4-mix-dynamic-s12-b3off', 'g-b2-door-tri-s11', 'g-b8-two-tri-s11']


def _a2():
    spec = importlib.util.spec_from_file_location('a2_analyze', A2)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run_row(a2, d, meta):
    row = {'run': d.name, 'driver': meta}
    gate = d/'feasibility-gate.json'
    if gate.is_file():
        g = json.loads(gate.read_text())
        row['gate'] = {'verdict': g['verdict'], 'run_started': g['run_started'],
                       'items': [{k: r.get(k) for k in ('item_id', 'kind', 'zone', 'verdict', 'blocking_solo_items',
                                                        'blocking_solo_item_poses')} for r in g['items']]}
        row['files_sha256'] = {'feasibility-gate.json': sha(gate)}
    res = d/'result.json'
    if not res.is_file():
        row['status'] = 'refused_by_gate' if gate.is_file() else 'no_result'
        return row
    r = json.loads(res.read_text())
    te = r['team_executor']
    jobs = te['jobs']
    ledger = a2.ledger_checks(te)
    labels = a2.labels_vs_setup(d, r)
    blocks = a2.blockers(d, r)
    ok = (r['referee_v2']['goal_met'] and r['eq_active_max'] == 0 and sum(j.get('drops', 0) for j in jobs) == 0
          and ledger['double_membership'] == 0 and ledger['decrement_once_per_item'])
    row.update({
        'status': 'ran', 'source_sha': r['source_sha'], 'coordination': r['coordination'],
        'perception_profile': r['perception_profile'],
        'contact_profile': r['config']['contact_profile_effective'] + ' (pending user approval)',
        'phase': r['phase'], 'error': r['error'], 'referee_v2_goal_met': r['referee_v2']['goal_met'],
        'referee_v2_per_zone_exact': r['referee_v2']['per_zone_exact'],
        'teacher_feasibility_success': ok,
        'goal_met_rgb': r['goal_met_rgb'], 'control_end_sim_s': r.get('control_end_sim_s'),
        'max_sim_s': r['max_sim_s'], 'sim_end_s': r.get('sim_end_s'), 'eq_active_max': r['eq_active_max'],
        'llm_calls': r['llm_calls'], 'claim_rounds': r['coordination_stats']['claim_rounds'],
        'drops': sum(j.get('drops', 0) for j in jobs),
        'slip_max_mm': max((v for j in jobs for v in (j.get('slip_mm') or {}).values()), default=0.),
        'ledger_checks': ledger, 'labels_vs_setup': labels,
        'not_delivered_items': sorted(b['item'] for b in blocks if b['kind'] == 'not_delivered_at_end'),
        'blocker_summary': a2.blocker_summary(blocks),
        'executor_metrics': te['metrics'], 'teacher_fix': te['teacher_fix'],
        'final_rgb_view': {k: r['final_rgb_view'].get(k) for k in ('profile', 'pickup_items_still_visible',
                                                                    'zone_counts_seen', 'moved_labels')},
        'jobs': [{k: j.get(k) for k in ('job_id', 'state', 'participants', 'commit_sim_s', 'end_sim_s', 'failures',
                                         'returns', 'formation_wait_s', 'drops', 'slip_mm')}
                 | {'pause_total_s': round(sum(p.get('s', 0.) for p in j.get('pauses', [])), 2),
                    'pauses': len(j.get('pauses', []))} for j in jobs],
        'door_gate_waits': len(r['door_gate_waits']), 'door_standoffs': r['door_standoffs'],
        'load_avg': r['load_avg'], 'wall_s': r['wall_s'],
        'files_sha256': {**row.get('files_sha256', {}), 'result.json': sha(res),
                         'teacher-events.json': sha(d/'teacher-events.json')},
    })
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--raw', type=Path, required=True)
    p.add_argument('--b5', type=Path, required=True)
    p.add_argument('--n1', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    a2 = _a2()
    meta = {f.stem: json.loads(f.read_text()) for f in sorted((args.raw/'runs').glob('*.json'))}
    rows = [run_row(a2, args.raw/name, meta[name]) for name in meta]
    by = {r['run']: r for r in rows}
    cohort = [by[n] for n in NEW_SEED_RUNS if n in by]
    out = {'experiment': '2026-09-26-zone-teacher-fix',
           'claim_scope': ('GT TEACHER feasibility only (demonstrations, training targets, scenario feasibility); '
                           'never the study executor; teacher success is never robot, RGB-skill or student success'),
           'robot_facing_outcome_source': 'teacher_receipt_L4 (unresolved)',
           'contact_profile': 'cargo_noslip_v1 (pending user approval)',
           'weld': 'off', 'physics_clock': 'synchronous SIM', 'llm_calls_total': sum(r.get('llm_calls') or 0
                                                                                    for r in rows),
           'raw_root_local_only': str(args.raw.resolve()),
           'new_seed_cohort': {'pre_registered': NEW_SEED_RUNS, 'finished': [r['run'] for r in cohort],
                               'teacher_feasibility_success': sum(bool(r.get('teacher_feasibility_success'))
                                                                  for r in cohort),
                               'n': len(cohort)},
           'blocker_reruns': BLOCKER_RUNS, 'runs': rows,
           'b5_offline': json.loads(args.b5.read_text()) | {'file_sha256': sha(args.b5)}}
    if args.n1:
        out['n1_offline'] = json.loads(args.n1.read_text()) | {'file_sha256': sha(args.n1)}
    args.output.write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ('run', 'status', 'phase', 'referee_v2_goal_met',
                                                'teacher_feasibility_success', 'control_end_sim_s')}))
    print(json.dumps(out['new_seed_cohort']))


if __name__ == '__main__':
    main()
