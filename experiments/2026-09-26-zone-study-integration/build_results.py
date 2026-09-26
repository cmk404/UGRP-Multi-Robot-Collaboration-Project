"""Gate check and summary of the integration plumbing smoke (evaluation only, after the runs).

    python experiments/2026-09-26-zone-study-integration/build_results.py \
        --runs /Users/changmin/projects/ugrp/outputs/zone-study-integration-20260926/smoke-45999d9c \
        --out experiments/2026-09-26-zone-study-integration/results.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness import zone_study_contract as A  # noqa: E402
from harness import zone_study_eval as ev  # noqa: E402

HERE = Path(__file__).resolve().parent
CONDITIONS = A.MAIN_CONDITIONS
WRIST_LABEL = 'CURRENT OWN WRIST RGB'
IDLE_REASK_S = 10.0   # harness.zone_event_scheduler.CallPolicy().idle_reask_s, identical in every condition


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    p = Path(path)
    return [json.loads(line) for line in p.read_text().splitlines() if line.strip()] if p.is_file() else []


def load_run(run_dir):
    run_dir = Path(run_dir)
    out = {'dir': str(run_dir), 'result': None, 'manifest': None, 'trial': None}
    for key, rel in (('result', 'result.json'), ('manifest', 'manifest.json'), ('trial', 'study/trial_record.json')):
        if (run_dir / rel).is_file():
            out[key] = json.loads((run_dir / rel).read_text())
    return out


def gates_for(run):
    """P1-P6, P8 of one condition (P7 is across conditions)."""
    res, man, trial, d = run['result'], run['manifest'], run['trial'], Path(run['dir'])
    g = {}
    g['P1'] = bool(res and trial and res['failure'] is None and res['stop'] in ('horizon', 'quiescent_budget_spent'))
    if not g['P1']:
        return g, {'reason': 'no result/trial record' if not res else res.get('failure')}
    study = res['study']
    released = {c['request_id']: c['released_at_sim_s'] for c in trial['calls'] if c['status'] != 'censored'}
    dispatch = rows(d / 'study' / 'dispatch.jsonl')
    p2 = []
    for row in dispatch:
        want = released.get('req_' + row['call_id'].replace('-', '_'))
        if want is None or abs(want - row['sim_s']) > 1e-6:
            p2.append(f'{row["call_id"]}: dispatched at {row["sim_s"]}, released at {want}')
        ack = row.get('ack')
        if ack and (ack['robot_id'] != row['actor'] or abs(ack['sim_s'] - row['sim_s']) > 1e-6):
            p2.append(f'{row["call_id"]}: ack of {ack["robot_id"]} at {ack["sim_s"]}')
    executed = [a for a in trial['actions'] if a['kind'] in ('claim_order', 'wait', 'release')]
    if len(executed) != sum(1 for r in dispatch if r['api'] or r['rejected_reason']):
        p2.append(f'{len(executed)} executable actions but {len(dispatch)} dispatch rows')
    for rec in trial['calls'] + trial['messages'] + trial['actions']:
        problems = A.action_record_violations(rec) if rec['schema'] == A.ACTION_LOG_SCHEMA else []
        p2.extend(problems)
    if not study['reopen']['ok']:
        p2.extend(study['reopen']['problems'])
    g['P2'] = not p2
    g['P3'] = bool(study['channel']['ok'])
    g['P4'] = bool(study['cost']['ok']) and study['clock_drift_s'] <= 1e-3
    frames = {rid: {f['sha256'] for f in rows(d / 'robots' / rid / 'inputs' / 'frames.jsonl')} for rid in A.ROBOTS}
    p5 = []
    for row in rows(d / 'study' / 'inputs.jsonl'):
        if row['frame_sha256'] not in frames[row['robot']]:
            p5.append(f'{row["request_id"]}: image is not in {row["robot"]}\'s own frame log')
        if row['frame_t'] > row['sim_s'] + 1e-9:
            p5.append(f'{row["request_id"]}: frame after the call')
        if not (d / 'study' / 'request_images' / f'{row["frame_sha256"]}.jpg').is_file():
            p5.append(f'{row["request_id"]}: request image bytes not saved')
    for arc in trial['request_archive']:
        user = json.loads(arc['user'])
        if not arc['payload_validated'] or A.forbidden_key_hits(user) or \
                any(s in arc['user'] for s in A.FORBIDDEN_VALUE_SUBSTRINGS):
            p5.append(f'{arc["request_id"]}: payload not clean')
        if [i['label'] for i in arc['image_refs']] != [WRIST_LABEL]:
            p5.append(f'{arc["request_id"]}: images {[i["label"] for i in arc["image_refs"]]}')
    g['P5'] = bool(study['requests']['ok']) and not p5
    robots = res['robots']
    g['P6'] = (all(r['foreign_frames_fed'] == 0 and r['cameras_seen'] == ['robot_cam'] and r['pose_sources_seen']
                   and all(s.startswith('owncam_pf_v2:') for s in r['pose_sources_seen']) for r in robots.values())
               and res['eval_only']['weld_max_eq_active'] == 0
               and res['eval_only']['contact_profile']['noslip_iterations'] > 0)
    reask = {}
    events = rows(d / 'study' / 'scheduler_events.jsonl')
    for rid in A.ROBOTS:
        timers = [e['sim_s'] for e in events if e.get('kind') == 'timer' and e.get('actor') == rid]
        reask[rid] = min((b - a for a, b in zip(timers, timers[1:])), default=None)
    g['P9_reask'] = all(v is None or v >= trial_policy_idle(res) - 1e-9 for v in reask.values())
    label = {'pose_provider': 'tags_temporary', 'temporary': True, 'research_result': False,
             'note_ko': '임시, 표식 사용, 연구 결과 아님'}
    g['P8_records'] = res['pose_provider'] == label and man['pose_provider'] == label and \
        trial.get('pose_provider') == label
    return g, {'p2_problems': p2[:10], 'p5_problems': p5[:10], 'min_timer_spacing_s': reask}


def trial_policy_idle(res):
    return float(res['study'].get('call_policy_idle_reask_s') or IDLE_REASK_S)


def observed(run):
    res, trial = run['result'], run['trial']
    if not res or not trial:
        return None
    study = res['study']
    state = ev.delivery_state(trial)
    metrics = ev.trial_metrics(ev.parse_trial(trial))
    return {'stop': res['stop'], 'sim_s': res['sim_s'], 'end_reason': trial['end_reason'],
            'end_sim_s': trial['end_sim_s'], 'calls': study['calls'], 'messages': study['messages'],
            'actions': study['actions'], 'dispatch': study['dispatch'],
            'think_sim_s': study['cost']['charged_sim_s'] - study['cost']['talk_sim_s'],
            'talk_sim_s': study['cost']['talk_sim_s'], 'delivery_sim_s': study['cost']['delivery_sim_s'],
            'follower_to_follower': study['channel']['follower_to_follower'],
            'free_text_messages': study['channel']['free_text_messages'],
            'rejected_messages': study['channel']['rejected'],
            'leader_id': trial.get('leader_id'),
            'eval_only_deliveries': res['eval_only']['referee']['deliveries'],
            'eval_only_final_zone': res['eval_only']['referee']['final_zone'],
            'orders_by_eval': state['by_order'], 'orders_complete': state['orders_complete'],
            'metrics_success': metrics['efficiency'].get('success'),
            'boundary_status': ev.boundary_status(metrics['boundary']),
            'robots': {rid: {'jobs': [(j['kind'], j['outcome'], j['confirmation']) for j in r['jobs']],
                             'false_confirmations': r['false_confirmations'], 'dead': r['dead'],
                             'exception': (r['exception'] or {}).get('type'), 'aborts': len(r['aborts']),
                             'contact_steps': r['evaluation_only']['contact_steps']}
                       for rid, r in res['robots'].items()},
            'wakeups': study['wakeups'], 'wall_s': run['manifest']['wall_s'],
            'load_average': run['manifest']['load_average'], 'code_sha': run['manifest']['code']['sha'],
            'bundle_sha256': res['bundle_sha256']}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--runs', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--episode', default='smoke-i700')
    args = p.parse_args(argv)
    runs_root = Path(args.runs)
    runs = {c: load_run(runs_root / f'{c}-{args.episode}') for c in CONDITIONS}
    gates, detail, obs = {}, {}, {}
    for c, run in runs.items():
        gates[c], detail[c] = gates_for(run)
        obs[c] = observed(run)
    shas = {c: (r['result'] or {}).get('study', {}).get('pair_status_sha256') for c, r in runs.items()}
    p7 = len(set(shas.values())) == 1 and None not in shas.values()
    names = ('P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P8_records', 'P9_reask')
    summary = {n: all(gates[c].get(n, False) for c in CONDITIONS) for n in names}
    summary['P7'] = p7
    raw = {}
    for c, run in runs.items():
        d = Path(run['dir'])
        if (d / 'manifest.json').is_file():
            raw[c] = {'dir': str(d), 'manifest_sha256': sha(d / 'manifest.json'),
                      'result_sha256': sha(d / 'result.json'),
                      'trial_record_sha256': sha(d / 'study' / 'trial_record.json')
                      if (d / 'study' / 'trial_record.json').is_file() else None,
                      'files': len(list(d.rglob('*'))),
                      'bytes': sum(q.stat().st_size for q in d.rglob('*') if q.is_file())}
    out = {'schema': 'ugrp.zone_study_integration_results.v1', 'episode': args.episode,
           'runs_root': str(runs_root), 'prereg_sha256': sha(HERE / 'prereg.json'),
           'plumbing_only': True, 'pose_provider': 'tags_temporary',
           'note_ko': '배선 스모크(no-LLM fixture, 1 seed). 통신 효과·연구 결과가 아니다. 임시, 표식 사용, 연구 결과 아님.',
           'gates': {'per_condition': gates, 'P7_pair_status_sha256': shas, 'summary': summary,
                     'all_pass': all(summary.values())},
           'gate_detail': detail, 'observed': obs, 'raw': raw}
    Path(args.out).write_text(json.dumps(out, indent=1, ensure_ascii=False, default=str) + '\n')
    print(json.dumps({'summary': summary, 'all_pass': out['gates']['all_pass'],
                      'dispatch': {c: (o or {}).get('dispatch') for c, o in obs.items()},
                      'deliveries': {c: len((o or {}).get('eval_only_deliveries') or []) for c, o in obs.items()}},
                     ensure_ascii=False, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
