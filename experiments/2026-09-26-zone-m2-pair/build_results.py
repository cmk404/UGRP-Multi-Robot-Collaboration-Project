"""Aggregate M2 pair runs into results-<cohort>.json (+ derived native TensorBoard snapshot).

Reads only local raw outputs (primary checkout outputs/, gitignored), records paths and SHA-256 and
enforces the FULL pre-registered run list of the cohort (a missing run blocks the report).
Usage:
  python experiments/2026-09-26-zone-m2-pair/build_results.py stage1 <cohort-dir> [--tensorboard <snapshot>]
  python experiments/2026-09-26-zone-m2-pair/build_results.py stage3 <cohort-dir> [--tensorboard <snapshot>]
  python experiments/2026-09-26-zone-m2-pair/build_results.py stage2 <cohort-dir> [--tensorboard <snapshot>]
  python experiments/2026-09-26-zone-m2-pair/build_results.py stage2c <cohort-dir> --raw <root> [--tensorboard <snapshot>]
  python experiments/2026-09-26-zone-m2-pair/build_results.py stage2b-completion <cohort-dir> --raw <root>
Dev runs (``dev*`` folders under the raw root) are added to the TensorBoard view labelled dev.
``--raw`` (Kiro, door v3): raw root other than Claude's outputs/zone-m2-pair-20260926 (never written by Kiro).
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RAW = Path('/Users/changmin/projects/ugrp/outputs/zone-m2-pair-20260926')
TB = Path('/Users/changmin/projects/ugrp/outputs/tensorboard')
STAGES = {'stage1': tuple(range(711, 719)), 'stage3': tuple(range(721, 725)), 'stage2': tuple(range(811, 817)), 'stage2b': tuple(range(821, 827)),
          'stage2c': tuple(range(831, 837)), 'stage2b-completion': (826,)}
ARMS = ('on', 'off')
# stage 2c experimenter open-at-lift arm (pre-registered): seed -> (arm, folder suffix)
STAGE2C_INTERVENTION = {837: 'on-openlift', 838: 'on-openlift'}
if '--raw' in sys.argv:
    RAW = Path(sys.argv[sys.argv.index('--raw') + 1])


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarise(folder):
    r = json.loads((folder / 'result.json').read_text())
    ev = r['evaluation_only']
    ap = ev['approach']
    arr = {k: v['at_arrival'] for k, v in ap.items()}
    return {'raw_dir': str(folder), 'result_sha256': sha(folder / 'result.json'),
            'events_sha256': sha(folder / 'events.jsonl'), 'commands_sha256': sha(folder / 'commands.jsonl'),
            'seed': r['seed'], 'stage': r['stage'], 'arm': 'on' if r['status_channel']['enabled'] else 'off',
            'on_failure': r['on_failure'], 'source_sha': r['source_sha'], 'dirty_source': r['dirty_source'],
            'gt_at_runtime': r['gt_at_runtime'], 'development_seed': r['development_seed'],
            'final_states': r['final_states'], 'failures': r['failures'], 'success_gt': ev['success_gt'],
            'final_error_m': ev['final_error_m'], 'lifted_clear_gt': ev['lifted_clear_gt'],
            'on_floor_released': ev['on_floor_released'], 'max_tilt_deg_lifted': ev['max_tilt_deg_lifted'],
            'order_sheet_error': ev['order_sheet_error'], 'approach': ap,
            'arrived_both': all(v['outcome'] == 'arrived' for v in ap.values()),
            'max_arrival_err_to_true_prestation_m': max((a['pos_err_to_true_prestation_m'] for a in arr.values() if a), default=None),
            'max_arrival_err_to_own_goal_m': max((a['pos_err_to_goal_m'] for a in arr.values() if a), default=None),
            'max_arrival_yaw_err_rad': max((a['yaw_err_to_true_rad'] for a in arr.values() if a), default=None),
            'max_arrival_localizer_err_m': max((a['localizer_pos_err_m'] for a in arr.values() if a and a['localizer_pos_err_m'] is not None), default=None),
            'robot_robot_contact_samples': ev['robot_robot_contact_samples'],
            'robot_beam_contact_samples_in_approach': ev['robot_beam_contact_samples_in_approach'],
            'robot_wall_contact_samples': ev['robot_wall_contact_samples'],
            'beam_wall_contact_samples': ev.get('beam_wall_contact_samples'),
            'beam_disp_before_grasp_m': ev['beam_disp_before_grasp_m'], 'weld_eq_active_max': ev['weld_eq_active_max'],
            'drop_injection': ev.get('drop_injection'), 'failure_propagation': ev.get('failure_propagation'),
            'door': ev.get('door'), 'claims': r['claims'],
            'sim_seconds': r['sim_seconds'], 'wall_seconds': r['wall_seconds'],
            'commands': sum(r['commands'].values()), 'frames': sum(r['frames'].values()),
            'looks': sum(v['looks'] for v in ap.values()),
            'status_messages': r['status_channel']['messages'],
            'load_average_start': r['load_average_start'], 'load_average_end': r['load_average_end'],
            'contact_profile': r['contact_profile'], 'weld': r['weld'], 'profile': r['profile'],
            'door_version': r.get('door_version'), 'lift_check': r.get('lift_check'),
            'open_at_lift_injection': ev.get('open_at_lift_injection'), 'unplanned_drops': ev.get('unplanned_drops'),
            'makespan_sim_s': ev.get('makespan_sim_s'), 'lift_views': lift_views(folder)}


def lift_views(folder):
    """Every lift_view with the evaluation-only held label (beam z > 0.03 m, both own fingers > 1 N)."""
    ev = [json.loads(line) for line in (folder / 'events.jsonl').read_text().splitlines()]
    truth = [json.loads(line) for line in (folder / 'evaluation-only.jsonl').read_text().splitlines()]
    out = []
    for e in (e for e in ev if e['event'] == 'lift_view'):
        gt = min(truth, key=lambda r: abs(r['t'] - e['sim_time_s']))
        out.append({'robot': e['robot'], 't': e['sim_time_s'], 'seg': e.get('seg'), 'held_iou': e['held_iou'],
                    'held_gt': bool(gt['beam_xyz'][2] > .03 and min(gt['finger_n'][e['robot']]) > 1.)})
    return out


def gates_stage1(runs):
    out = {}
    for arm in ARMS:
        rs = [x for x in runs if x['arm'] == arm]
        s1 = sum(x['success_gt'] for x in rs)
        arrived = sum(x['arrived_both'] for x in rs)
        worst_pos = max(x['max_arrival_err_to_true_prestation_m'] for x in rs)
        worst_yaw = max(x['max_arrival_yaw_err_rad'] for x in rs)
        over = [x['seed'] for x in rs if x['max_arrival_err_to_true_prestation_m'] > .10 or x['max_arrival_yaw_err_rad'] > .10]
        s3 = all(x['robot_robot_contact_samples'] == 0 and x['robot_beam_contact_samples_in_approach'] == 0
                 and x['beam_disp_before_grasp_m'] <= .02 and x['weld_eq_active_max'] == 0 for x in rs)
        s4 = all(x['gt_at_runtime'] is False and x['dirty_source'] is False for x in rs)
        out[arm] = {
            'M2-S1_success': {'value': f'{s1}/{len(rs)}', 'pass': s1 >= 6},
            'M2-S2_approach': {'arrived_both': f'{arrived}/{len(rs)}', 'worst_err_to_true_prestation_m': worst_pos,
                               'worst_yaw_err_rad': worst_yaw, 'seeds_over_limit': over,
                               'worst_err_to_own_goal_m': max(x['max_arrival_err_to_own_goal_m'] for x in rs),
                               'pass': arrived >= 7 and not over},
            'M2-S3_safety': {'pass': s3},
            'M2-S4_inputs': {'pass': s4, 'note': 'plus tests/test_pair_owncam_approach.py boundary test'},
            'not_gated_robot_wall_contact_runs': [x['seed'] for x in rs if x['robot_wall_contact_samples']],
            'cost': {'sim_s_mean': round(sum(x['sim_seconds'] for x in rs) / len(rs), 1),
                     'commands_mean': round(sum(x['commands'] for x in rs) / len(rs), 1),
                     'frames_mean': round(sum(x['frames'] for x in rs) / len(rs), 1),
                     'looks_mean': round(sum(x['looks'] for x in rs) / len(rs), 2),
                     'wall_s_mean': round(sum(x['wall_seconds'] for x in rs) / len(rs), 1),
                     'load_start_range': [min(x['load_average_start'][0] for x in rs), max(x['load_average_start'][0] for x in rs)]}}
    return out


def gates_stage3(runs):
    out = {}
    for arm in ARMS:
        rs = [x for x in runs if x['arm'] == arm]
        measurable = [x for x in rs if (x['drop_injection'] or {}).get('applied_at_s') is not None]
        detect = []
        for x in measurable:
            victim = x['drop_injection']['robot']
            fp = x['failure_propagation'] or {}
            ff = fp.get('first_failure') or {}
            ok = ff.get('robot') == victim and ff.get('reason') == 'LOAD_CHANGED_IN_CARRY'
            detect.append({'seed': x['seed'], 'detected': ok,
                           'latency_s': round(ff['sim_time_s'] - x['drop_injection']['applied_at_s'], 3) if ok else None})
        partner = [{'seed': x['seed'], **{k: (x['failure_propagation'] or {}).get(k) for k in
                    ('partner_terminal_state', 'partner_failure', 'partner_terminal_latency_s',
                     'partner_motion_commands_after', 'partner_arm_commands_after', 'partner_last_motion_after_s',
                     'beam_max_tilt_deg_after', 'beam_final_z_m')}} for x in measurable]
        entry = {'measurable': f'{len(measurable)}/{len(rs)}', 'not_measurable': [x['seed'] for x in rs if x not in measurable],
                 'M2-P1_detect': {'runs': detect, 'pass': len(detect) == len(rs) and
                                  all(d['detected'] and d['latency_s'] <= 1.5 for d in detect)},
                 'partner': partner}
        if arm == 'on':
            entry['M2-P2_partner_stops'] = {'pass': len(partner) == len(rs) and all(
                p['partner_failure'] == 'PARTNER_ABORT' and p['partner_terminal_latency_s'] is not None
                and p['partner_terminal_latency_s'] <= .5 and p['partner_motion_commands_after'] == 0 for p in partner)}
        else:
            moving = sum((p['partner_motion_commands_after'] or 0) >= 1 for p in partner)
            entry['M2-P3_partner_keeps_moving'] = {'value': f'{moving}/{len(partner)}', 'pass': moving >= 3}
        out[arm] = entry
    return out


def gates_stage2(runs):
    out = {}
    for arm in ARMS:
        rs = [x for x in runs if x['arm'] == arm]
        succ = sum(x['success_gt'] for x in rs)
        walls = [x['seed'] for x in rs if x['robot_wall_contact_samples'] or x['beam_wall_contact_samples']]
        arrived = sum(x['arrived_both'] for x in rs)
        clean = all(x['gt_at_runtime'] is False and x['dirty_source'] is False and x['robot_robot_contact_samples'] == 0
                    and x['weld_eq_active_max'] == 0 for x in rs)
        out[arm] = {'M2-D1_success': {'value': f'{succ}/{len(rs)}', 'pass': succ >= 4},
                    'M2-D2_no_wall_contact': {'seeds_with_contact': walls, 'pass': not walls},
                    'M2-D3_approach': {'arrived_both': f'{arrived}/{len(rs)}', 'pass': arrived >= 5},
                    'M2-D4_inputs_safety': {'pass': clean},
                    'crossing_offset_m': {x['seed']: (x['door'] or {}).get('beam_y_offset_at_crossing_m') for x in rs},
                    'final_error_m': {x['seed']: x['final_error_m'] for x in rs},
                    'failures': {x['seed']: x['failures'] for x in rs},
                    'cost': {'sim_s_mean': round(sum(x['sim_seconds'] for x in rs) / len(rs), 1),
                             'commands_mean': round(sum(x['commands'] for x in rs) / len(rs), 1),
                             'wall_s_mean': round(sum(x['wall_seconds'] for x in rs) / len(rs), 1)}}
    return out


def gates_stage2c(runs):
    """Door v3 (pre-registered in README 2c): D1-D4 as stage 2 per arm, plus L1 (lift sensitivity), L2 (specificity)."""
    main = [x for x in runs if x['seed'] in STAGES['stage2c']]
    out = gates_stage2(main)
    for arm in ARMS:
        rs = [x for x in main if x['arm'] == arm]
        fn = [{'seed': x['seed'], **v} for x in rs for v in x['lift_views'] if v['held_gt'] and v['held_iou'] < .45]
        out[arm]['M2-L1_no_lift_false_negative'] = {'lifts_held_gt': sum(v['held_gt'] for x in rs for v in x['lift_views']),
                                                    'false_negatives': fn, 'pass': not fn}
        out[arm]['unplanned_drops'] = {x['seed']: len(x['unplanned_drops'] or []) for x in rs}
        out[arm]['arrived_both'] = {x['seed']: x['arrived_both'] for x in rs}
        out[arm]['makespan_sim_s'] = {x['seed']: x['makespan_sim_s'] for x in rs}
        ok = [x['makespan_sim_s'] for x in rs if x['success_gt'] and x['makespan_sim_s'] is not None]
        out[arm]['makespan_sim_s_mean_successful'] = round(sum(ok) / len(ok), 1) if ok else None
    inter = [x for x in runs if x['seed'] in STAGE2C_INTERVENTION]
    rows = []
    for x in inter:
        inj = x['open_at_lift_injection'] or {}
        fp = x['failure_propagation'] or {}
        rows.append({'seed': x['seed'], 'injection': inj, 'applied': inj.get('applied_at_s') is not None,
                     'victim_detected_at_lift': bool(inj.get('victim_failed_at_intervened_lift')),
                     'partner_failure': fp.get('partner_failure'), 'partner_latency_s': fp.get('partner_terminal_latency_s'),
                     'partner_motion_commands_after': fp.get('partner_motion_commands_after')})
    applied = [r for r in rows if r['applied']]
    out['intervention'] = {'runs': rows, 'measurable': f'{len(applied)}/{len(STAGE2C_INTERVENTION)}',
                           'M2-L2_specificity': {'pass': len(applied) == len(STAGE2C_INTERVENTION) and
                                                 all(r['victim_detected_at_lift'] for r in applied)},
                           'M2-L3_partner_abort_on': {'pass': len(applied) == len(STAGE2C_INTERVENTION) and all(
                               r['partner_failure'] == 'PARTNER_ABORT' and r['partner_latency_s'] is not None
                               and r['partner_latency_s'] <= .5 for r in applied)}}
    return out


def gates_completion(runs):
    return {'runs': {f"{x['seed']}-{x['arm']}": {'success_gt': x['success_gt'], 'failures': x['failures'],
                                                 'arrived_both': x['arrived_both'], 'final_error_m': x['final_error_m'],
                                                 'wall_contacts': x['robot_wall_contact_samples'] + (x['beam_wall_contact_samples'] or 0),
                                                 'robot_robot_contact_samples': x['robot_robot_contact_samples'],
                                                 'makespan_sim_s': x['makespan_sim_s'], 'sim_seconds': x['sim_seconds'],
                                                 'commands': x['commands'], 'source_sha': x['source_sha'],
                                                 'dirty_source': x['dirty_source']} for x in runs}}


def collect(stage, cohort):
    runs, missing = [], []
    for seed in STAGES[stage]:
        for arm in ARMS:
            folder = RAW / cohort / f's{seed}-{arm}'
            if (folder / 'result.json').exists():
                runs.append(summarise(folder))
            else:
                missing.append({'seed': seed, 'arm': arm, 'status': 'missing/infrastructure_failure'})
    if stage == 'stage2c':
        for seed, arm in STAGE2C_INTERVENTION.items():
            folder = RAW / cohort / f's{seed}-{arm}'
            if (folder / 'result.json').exists():
                runs.append(summarise(folder))
            else:
                missing.append({'seed': seed, 'arm': arm, 'status': 'missing/infrastructure_failure'})
    return runs, missing


def main():
    stage, cohort = sys.argv[1], sys.argv[2]
    runs, missing = collect(stage, cohort)
    if missing and '--allow-incomplete' not in sys.argv:
        raise SystemExit(f'pre-registered runs missing {missing}; the cohort report is blocked')
    log = RAW / cohort / 'cohort.log'
    gates = {'stage1': gates_stage1, 'stage2': gates_stage2, 'stage2b': gates_stage2, 'stage3': gates_stage3,
             'stage2c': gates_stage2c, 'stage2b-completion': gates_completion}[stage](runs)
    dev = [summarise(p.parent) for p in sorted(RAW.glob('dev*/result.json'))]
    out = {'experiment_id': '2026-09-26-zone-m2-pair', 'stage': stage, 'cohort': cohort,
           'raw_root_local_only': str(RAW), 'raw_note': 'raw outputs are local (gitignored); not a remote backup',
           'cohort_log': {'path': str(log), 'sha256': sha(log)} if log.exists() else None,
           'runs': runs, 'missing': missing, 'report_blocked': bool(missing), 'gates': gates,
           'development_runs': [{k: d[k] for k in ('raw_dir', 'result_sha256', 'seed', 'stage', 'arm', 'source_sha',
                                                  'dirty_source', 'success_gt', 'final_error_m', 'failures')} for d in dev]}
    if stage == 'stage2c':
        out['development_runs'] = [dict(d, door_version=x['door_version'], open_at_lift_injection=x['open_at_lift_injection'],
                                        lift_views=x['lift_views'], makespan_sim_s=x['makespan_sim_s'])
                                   for d, x in zip(out['development_runs'], dev)]
    (HERE / f'results-{cohort}.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
    print(json.dumps(gates, ensure_ascii=False))
    if '--tensorboard' in sys.argv:
        tensorboard(runs, dev, stage, cohort, sys.argv[sys.argv.index('--tensorboard') + 1])


def tensorboard(runs, dev, stage, cohort, snapshot):
    view = RAW / 'tensorboard-view' / snapshot
    names = {}
    for x, label in [(r, 'test') for r in runs] + [(d, 'dev') for d in dev if d['stage'] == runs[0]['stage']
                                                   and (d['drop_injection'] is not None) == (stage == 'stage3')]:
        name = (f"m2{x['stage'][0]}{'' if label == 'test' else 'dev'}-{x['arm']}-s{x['seed']}"
                + ('-drop' if x['drop_injection'] else ''))
        if stage == 'stage2c':
            name = (f"m2c{'' if label == 'test' else 'dev'}-{x['door_version'] or 'v?'}-{x['arm']}-s{x['seed']}"
                    + ('-openlift' if x['open_at_lift_injection'] else ''))
        derived = {'derived_view_only': True, 'derived_from': x['raw_dir'], 'source_result_sha256': x['result_sha256'],
                   'success': bool(x['success_gt']),
                   'success_definition': ('GT evaluation only: both done, lifted > 0.06 m, released on floor, '
                                          + ('beam x >= 2.7 and final error <= 0.25 m' if x['stage'] == 'door'
                                             else 'final error <= 0.10 m') + '; NO GT at runtime'),
                   'mode': 'test' if label == 'test' else 'development',
                   'policy': x['profile'], 'scope': f"M2 pair long_beam, {x['stage']}, no runtime GT",
                   'case': (f"{x['stage']} door {x['door_version']} status_channel={x['arm']}"
                            + (' (approved infrastructure)' if x['arm'] == 'on' else ' (barrier-only diagnostic)')
                            + (' open_at_lift_intervention' if x['open_at_lift_injection'] else '')
                            if stage == 'stage2c' else
                            f"{x['stage']} status_channel={x['arm']} (candidate){' drop_test' if x['drop_injection'] else ''}"),
                   'config': {'contact_profile': x['contact_profile'], 'status_channel': x['arm'],
                              'on_failure': x['on_failure'], 'stage': x['stage'], 'weld': x['weld'],
                              'drop_injection': bool(x['drop_injection']),
                              **({'door_version': x['door_version'], 'lift_check': x['lift_check'],
                                  'open_at_lift': bool(x['open_at_lift_injection'])} if stage == 'stage2c' else {})},
                   **({'makespan_sim_s': x['makespan_sim_s'], 'unplanned_drops': len(x['unplanned_drops'] or []),
                       'arrived_both': int(x['arrived_both'])} if stage == 'stage2c' else {}),
                   'stop_reason': json.dumps(x['failures']),
                   'sim_s': x['sim_seconds'], 'wall_s': x['wall_seconds'], 'commands': x['commands'],
                   'model_calls': 0,
                   'evaluation': {k: x[k] for k in ('final_error_m', 'lifted_clear_gt', 'on_floor_released',
                                                    'max_tilt_deg_lifted', 'order_sheet_error', 'arrived_both',
                                                    'max_arrival_err_to_true_prestation_m', 'max_arrival_err_to_own_goal_m',
                                                    'max_arrival_localizer_err_m', 'robot_robot_contact_samples',
                                                    'robot_wall_contact_samples', 'beam_wall_contact_samples',
                                                    'failure_propagation', 'door', 'frames', 'looks')},
                   'seed': x['seed'], 'source_sha': x['source_sha']}
        (view / name).mkdir(parents=True, exist_ok=True)
        (view / name / 'result.json').write_text(json.dumps(derived, indent=1, default=str) + '\n')
        names[name] = (f"2c door {x['door_version']} {label} status_channel={x['arm']}"
                       + (' +open-at-lift' if x['open_at_lift_injection'] else '') if stage == 'stage2c' else
                       f"{x['stage']} {label} status_channel={x['arm']} (candidate, pending user decision)")
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
        entry['condition'] = names[short] + '; own robot_cam + static tag map + coarse order sheet; no runtime GT'
    (target / 'collection.json').write_text(json.dumps(collection, indent=2) + '\n')
    print(json.dumps({'snapshot': str(target), 'runs': len(collection['exported']), 'failed': collection.get('failed')}))


if __name__ == '__main__':
    main()
