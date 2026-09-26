"""No-LLM 3-robot smoke v2 of the own-camera executor (package F, issue #221 fixes). Tests the executor, not dialogue.

Same frozen scenario as v1 (``run_smoke.py`` + ``prereg.json``, kept unchanged): one MuJoCo world (sync
SIM), three robots, one ``ZoneOwnExecutor`` each; a scripted layer (fixed before any run, see
``prereg_v2.json``) gives each robot one ``deliver`` job; the second and third robots first ``hold``.
The scripted layer reads only executor events.

Robot inputs: own robot_cam frames, own issued commands, the static tagged map, fixed calibrations
(camera/motion and own body model), static layout idle-spawn discs, the order sheet. Simulator truth
goes to eval_only/ only. Weld OFF; cargo_noslip_v1 (user-approved 2026-09-26).

An exception or interrupt still writes every partial log plus ``failure.json`` and closes the world;
an existing episode directory is never overwritten.

Usage (from the worktree root):
    python experiments/2026-09-26-zone-own-executor/run_smoke_v2.py --episode smoke-v2-s700 --output <dir>
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
HERE = Path(__file__).resolve().parent
SCHEMA = 'ugrp.zone_own_executor_smoke.v2'
SLOT_HALF_M = .06
ON_FLOOR_MAX_Z_M = .05
GRASP_MIN_BOTH_FINGER_FRACTION = .95
CONDITION_FOR_ACTION_RECORDS = 'no_comm'          # the scripted layer has no dialogue channel (prereg_v2)
RUNTIME_FILES = ('harness/zone_own_executor.py', 'harness/zone_own_guards.py', 'harness/zone_own_deliver.py',
                 'harness/zone_own_status.py', 'harness/zone_own_contract.py', 'harness/zone_own_team_host.py',
                 'harness/zone_study_contract.py', 'harness/zone_event_scheduler.py', 'harness/zone_map_schematic.py',
                 'harness/m1_owncam_delivery.py', 'harness/m1_owncam_contract.py', 'harness/m1_contract.py',
                 'harness/owncam_pose_source.py', 'harness/owncam_localizer.py', 'harness/owncam_drive.py',
                 'harness/owncam_drive_v2.py', 'harness/visual_arm.py', 'harness/wall_tags.py', 'harness/map_goto.py',
                 'harness/zone_color_boxes.py', 'harness/zone_own_perception.py', 'harness/wrist_zone_skill.py',
                 'harness/wrist_zone_skill_v2.py', 'harness/wrist_zone_skill_v3.py', 'harness/wrist_zone_skill_v4.py',
                 'harness/wrist_zone_skill_v5.py', 'harness/wrist_zone_skill_v6.py', 'harness/wrist_zone_skill_v7.py',
                 'harness/wrist_zone_skill_v8.py', 'harness/wrist_zone_skill_v9.py', 'harness/owncam_view.py',
                 'harness/markerless_box.py', 'harness/visual_box_skill.py', 'harness/visual_attachment.py',
                 'sim/zone_cargo_contact.py', 'sim/zone_landmarks.py', 'sim/zone_scene.py', 'sim/camera_robot_port.py',
                 'sim/multi_masterpi_production.py', 'experiments/2026-09-26-zone-own-executor/run_smoke_v2.py',
                 'experiments/2026-09-26-zone-own-executor/prereg_v2.json',
                 'experiments/2026-09-26-zone-own-executor/body_model_calibration.json')


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def sha(b):
    return hashlib.sha256(b).hexdigest()


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r, ensure_ascii=False, default=str) + '\n' for r in rows))


def build_plan(spec):
    """Order sheet from the scenario config (setup-only placement -> coarse slot) and the scripted jobs (= v1)."""
    from harness.zone_own_executor import pickup_slot_of, zone_slot
    from sim.zone_arena import episode
    from sim.zone_landmarks import tagged_map
    config = episode(spec['base_map'], spec['seed'], goal=spec['goal'], extra_boxes=spec['extra_boxes'])
    static = tagged_map(spec['map'])
    cyan = sorted(((oid, o) for oid, o in config['setup_only']['objects'].items() if o['kind'] == 'cyan'),
                  key=lambda kv: (-kv[1]['position_m'][1], kv[1]['position_m'][0]))
    spawns = config['setup_only']['spawns']
    robots = sorted(spawns, key=lambda r: -spawns[r][1])               # north first
    rule = spec['assignment']
    orders, jobs, assigned = [], {}, {}
    for i, (rid, (oid, obj)) in enumerate(zip(robots, cyan)):
        slot = pickup_slot_of(static, obj['position_m'][:2])
        zone_slot_id = rule['zone_slots_north_to_south'][i]
        zone_slot(static, zone_slot_id)
        order_id = f'o{i + 1}'
        orders.append({'order_id': order_id, 'kind': 'cyan', 'count': 1, 'required_robots': 1,
                       'destination_zone': zone_slot_id[0],
                       'initial_location': {'pickup_bay': slot.split('-')[0], 'slot': slot}})
        jobs[rid] = {'hold_s': rule['hold_s_north_to_south'][i], 'order_id': order_id, 'zone_slot': zone_slot_id}
        assigned[rid] = oid
    sheet = {'schema': 'ugrp.zone_own_executor_order_sheet.v1', 'map_id': spec['map'], 'orders': orders,
             'source': 'scenario config (sim.zone_arena.episode setup placement -> coarse pickup slot)'}
    return sheet, jobs, assigned


def scripted_layer(jobs, state):
    def study_layer(host, kind, event, now):
        """Scripted no-LLM layer: reads executor events only (never host.eval_only)."""
        if kind == 'start':
            for rid, job in jobs.items():
                if job['hold_s'] > 0:
                    host.call(rid, 'hold', job['hold_s'])
                    state['phase'][rid] = 'hold'
                else:
                    ack = host.call(rid, 'deliver', job['order_id'], job['zone_slot'])
                    state['phase'][rid] = 'deliver'
                    state['deliver_job'][rid] = ack['job_id']
            return
        rid = event['robot_id']
        if event['event'] in ('job_done', 'job_failed'):
            if state['phase'][rid] == 'hold':
                ack = host.call(rid, 'deliver', jobs[rid]['order_id'], jobs[rid]['zone_slot'])
                state['phase'][rid] = 'deliver' if ack['accepted'] else 'finished'
                state['deliver_job'][rid] = ack['job_id']
            elif state['phase'][rid] == 'deliver' and event['job_id'] == state['deliver_job'].get(rid):
                state['phase'][rid] = 'finished'
    return study_layer


def gate_uncertain_at(transitions, t):
    """Own gate state at SIM time t from the executor's own transition log (starts 'uncertain')."""
    state = 'uncertain'
    for tr in transitions:
        if tr['t'] <= t + 1e-9:
            state = 'uncertain' if tr['change'] == 'entered' else 'ok'
    return state == 'uncertain'


def robot_result(rid, slot, host, spec, jobs, assigned, sheet, prereg):
    import numpy as np  # noqa: F401 - keeps the numpy version in scope for the manifest
    from harness import m1_contract, m1_owncam_contract
    data, ev, ex = host.world.data, host.eval_only, slot.executor
    summ = ex.summary()
    deliver = next((j for j in summ['jobs'] if j['kind'] == 'deliver'), None)
    ctl_rec = next((s for s in ex._summaries if 'controller' in s), None) or {}
    ctl = ctl_rec.get('controller') or {}
    order = next(o for o in sheet['orders'] if o['order_id'] == jobs[rid]['order_id'])
    slot_xy = next(s['center_m'] for zs in host.static['zone_slots'].values() for s in zs
                   if s['slot_id'] == jobs[rid]['zone_slot'])
    boxes = {b: [float(v) for v in data.body(o['body_name']).xpos] for b, o in host.objects.items() if o['kind'] == 'cyan'}
    in_slot = sorted(b for b, (x, y, z) in boxes.items() if abs(x - slot_xy[0]) <= SLOT_HALF_M and
                     abs(y - slot_xy[1]) <= SLOT_HALF_M and z < ON_FLOOR_MAX_Z_M)
    placement = ctl.get('skill_placement') or {}
    gates = ctl.get('lookback_gates') or []
    gate = next((g for g in gates if g.get('frame_id') == placement.get('frame_id')), {})
    ret = ev['retention'][rid]
    grasp_physical = bool(ret['carry_steps']) and ret['both_finger_steps'] >= GRASP_MIN_BOTH_FINGER_FRACTION * ret['carry_steps']
    retained = bool(ret['carry_steps']) and ret['low_box_steps'] == 0
    outcome = None if deliver is None else deliver['outcome']
    judged = m1_owncam_contract.judge(
        pose_sources=summ['pose_sources_seen'], skill_reason=outcome or 'NOT_STARTED',
        skill_claim_in_slot=placement.get('reason') == 'IN_SLOT', gt_box_in_slot=bool(in_slot),
        wall_contacts=ev['kind_steps'][rid].get('wall', 0), weld_used=ev['max_eq_active'] > 0,
        face_fallback_used=bool(ctl.get('face_fallback_used')), pickup_source=ctl.get('pickup_source') or 'none',
        within_limit=deliver is not None and not str(outcome).startswith(('LOCAL_TIMEOUT', 'EPISODE_END')),
        extra_checks={'look_back_pose_gate_ok': bool(gate) and not gate.get('violations'),
                      'no_exception': slot.exception is None, 'grasp_physical_both_fingers': grasp_physical,
                      'box_retained_in_carry': retained})
    skill_summary = ctl.get('skill_summary') or {}
    block = m1_contract.outcome_fields(mode='m1', pose_sources_seen=summ['pose_sources_seen'],
                                       diagnostic_success=judged['diagnostic_success'],
                                       input_contract={'text': prereg['input_contract']},
                                       cameras_seen=sorted(set(skill_summary.get('cameras_seen') or []) |
                                                           set(summ['cameras_seen'])) or ['robot_cam'])
    strict = bool(judged['m1_success'] and block['m1_success'])
    events = ex.events
    confirmed_while_uncertain = [e['sim_s'] for e in events if e['event'] == 'job_done' and
                                 e['detail'].get('confirmation') == 'own_camera_confirmed' and
                                 gate_uncertain_at(summ['gate_transitions'], e['sim_s'])]
    legs = ctl_rec.get('legs') or []
    guard_logs = [g for leg in legs for g in leg.get('guard_log', [])] + \
        [g for s in ex._summaries for g in s.get('guard_log', []) + s.get('sweep_guard', [])]
    ctl_events = ctl_rec.get('controller_events') or []
    res = {**block, **judged, 'm1_success': strict, 'success': strict,
           'counts_as_m1': bool(judged['counts_as_m1'] and block['counts_as_m1']),
           'robot_id': rid, 'order': order, 'zone_slot': jobs[rid]['zone_slot'], 'deliver_outcome': outcome,
           'deliver_confirmation': None if deliver is None else deliver['confirmation'],
           'deliver_sim_s': None if deliver is None else [deliver['started_at_sim_s'], deliver['ended_at_sim_s']],
           'placement': placement, 'lookback_gate': gate, 'exception': slot.exception,
           'jobs': summ['jobs'], 'events': [e['event'] for e in events],
           'executor_summary': summ, 'frames': len(slot.frames), 'commands': len(slot.commands),
           'foreign_frames_fed': sum(1 for f in slot.frames if f['robot_id'] != rid or f['camera'] != 'robot_cam'),
           'guards': {'pose_uncertain_events': sum(e['event'] == 'pose_uncertain' for e in events),
                      'gate_transitions': len(summ['gate_transitions']),
                      'confirmed_while_gate_uncertain': confirmed_while_uncertain,
                      'sweeps_restricted': sum(g.get('reason') not in ('clear', 'no_own_estimate') for g in guard_logs),
                      'sweep_backoffs': sum(bool(g.get('backoff')) for g in guard_logs),
                      'm1_sweeps_restricted': sum(e['event'] == 'sweep_guard' for e in ctl_events),
                      'stall_recoveries': sum(len(leg.get('stall_keepouts', [])) for leg in legs) +
                      sum(len(s.get('stall_keepouts', [])) for s in ex._summaries if 'driver_log' in s),
                      'blocked_legs': sum(str(leg.get('outcome')) == 'blocked' for leg in legs),
                      'near_clip_retreats': sum(e['event'] == 'near_clipped' for e in ctl_events),
                      'cancellations': slot.cancellations},
           'evaluation_only': {'assigned_box': assigned[rid], 'cyan_boxes_in_slot': in_slot,
                               'assigned_box_in_slot': assigned[rid] in in_slot,
                               'box_final_xyz': {b: [round(v, 4) for v in xyz] for b, xyz in boxes.items()},
                               'contact_steps': ev['kind_steps'][rid], 'retention': ret,
                               'false_confirmation': bool(deliver and deliver['confirmation'] == 'own_camera_confirmed'
                                                          and not in_slot)}}
    m1_owncam_contract.assert_exportable(res)
    m1_contract.validate_outcome(res)
    return res


def write_logs(out, host, episode, seed):
    from harness.zone_own_executor import action_record
    for rid, slot in host.robots.items():
        ex = slot.executor
        base = out / 'robots' / rid
        jsonl(base / 'inputs' / 'commands.jsonl', slot.commands)
        jsonl(base / 'inputs' / 'frames.jsonl', slot.frames)
        jsonl(base / 'executor' / 'events.jsonl', ex.events)
        jsonl(base / 'executor' / 'api.jsonl', ex.api_log)
        jsonl(base / 'executor' / 'judgments.jsonl', ex.judgment_log)
        jsonl(base / 'executor' / 'macros.jsonl', slot.decisions)
        jsonl(base / 'executor' / 'cancellations.jsonl', slot.cancellations)
        (base / 'executor' / 'job_summaries.json').write_text(json.dumps(ex._summaries, indent=1, default=str) + '\n')
    actions = [action_record(a, run_id=episode, condition=CONDITION_FOR_ACTION_RECORDS, seed=seed,
                             request_id=f'script-{i:03d}') for i, a in enumerate(host.api_calls)]
    jsonl(out / 'study' / 'api_calls.jsonl', host.api_calls)
    jsonl(out / 'study' / 'action_records.jsonl', actions)
    jsonl(out / 'study' / 'events.jsonl', host.event_log)
    ev = host.eval_only
    jsonl(out / 'eval_only' / 'gt.jsonl', ev['gt'])
    jsonl(out / 'eval_only' / 'frames_eval.jsonl', ev['frames_eval'])
    jsonl(out / 'eval_only' / 'contacts.jsonl', ev['contacts'])
    (out / 'eval_only' / 'retention.json').write_text(json.dumps(ev['retention'], indent=1) + '\n')
    (out / 'scene.xml').write_text(host.world.scene_xml)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--episode', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--prereg', default=str(HERE / 'prereg_v2.json'))
    p.add_argument('--sim-limit-s', type=float, default=None, help='dev plumbing checks only (shorter limit)')
    args = p.parse_args(argv)
    prereg = json.loads(Path(args.prereg).read_text())
    spec = dict(next(e for e in prereg['episodes'] if e['episode_id'] == args.episode))
    student = prereg['student']
    sim_limit = float(args.sim_limit_s or prereg['sim_limit_s'])
    import cv2
    import mujoco
    import numpy as np
    from harness.zone_own_team_host import OwnCamTeamHost

    out = Path(args.output) / args.episode
    out.mkdir(parents=True, exist_ok=False)                # never overwrite an existing episode
    started, load0 = time.time(), os.getloadavg()
    code = {'sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain', '--', 'harness', 'sim', 'maps',
                                                                'experiments/2026-09-26-zone-own-executor')),
            'runtime_files_sha256': {f: sha((ROOT / f).read_bytes()) for f in RUNTIME_FILES}}
    (out / 'attempt_started.json').write_text(json.dumps({'episode': args.episode, 'code': code, 'sim_limit_s': sim_limit,
                                                          'started_unix': round(started, 3),
                                                          'load_average_start': load0}, indent=2) + '\n')
    sheet, jobs, assigned = build_plan(spec)
    spec['order_sheet'] = sheet
    state = {'phase': {r: 'start' for r in jobs}, 'deliver_job': {}}
    host, exception, run, stage = None, None, None, 'setup'
    try:
        host = OwnCamTeamHost(spec, student, root=ROOT, study_layer=scripted_layer(jobs, state), frames_dir=out / 'frames')
        host.assigned_box = dict(assigned)           # eval-only bookkeeping inside the host
        stage = 'run'
        run = host.run(sim_limit, done=lambda: all(v == 'finished' for v in state['phase'].values()))
        stage = 'results'
        robots = {rid: robot_result(rid, slot, host, spec, jobs, assigned, sheet, prereg)
                  for rid, slot in host.robots.items()}
        stage = 'logs'
        write_logs(out, host, args.episode, spec['seed'])
    except BaseException as exc:                     # noqa: BLE001 - interrupt included: record, then re-raise
        exception = {'stage': stage, 'type': type(exc).__name__, 'message': str(exc)[:2000],
                     'traceback': traceback.format_exc()[-6000:]}
        if host is not None and stage in ('run', 'results'):
            try:
                write_logs(out, host, args.episode, spec['seed'])
            except Exception as log_exc:            # noqa: BLE001
                exception['partial_log_error'] = f'{type(log_exc).__name__}: {log_exc}'
        (out / 'failure.json').write_text(json.dumps({
            'episode': args.episode, 'exception': exception,
            'sim_s': None if host is None else round(float(host.world.data.time), 3), 'code': code,
            'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
            'wall_s': round(time.time() - started, 1)}, indent=2, default=str) + '\n')
        if host is not None:
            host.close()
        raise
    pos_err = {}
    for rid in host.robots:
        errs = sorted(r['pos_err_m'] for r in host.eval_only['frames_eval'] if r['robot_id'] == rid and 'pos_err_m' in r)
        pos_err[rid] = None if not errs else {'n': len(errs), 'p50': errs[len(errs) // 2],
                                              'p90': errs[min(len(errs) - 1, int(.9 * len(errs)))]}
    result = {'schema': SCHEMA, 'episode': args.episode, 'seed': spec['seed'], 'run': run, 'host_exception': None,
              'sim_limit_s': sim_limit, 'robots': robots,
              'm1_style_success_count': sum(r['m1_success'] for r in robots.values()),
              'confirmed_count': sum(r['deliver_confirmation'] == 'own_camera_confirmed' for r in robots.values()),
              'false_confirmations': sum(r['evaluation_only']['false_confirmation'] for r in robots.values()),
              'weld_max_eq_active': host.eval_only['max_eq_active'], 'contact_profile': host.contact_record,
              'evaluation_only_pose_error': pos_err, 'order_sheet': sheet, 'scripted_jobs': jobs,
              'action_record_condition': CONDITION_FOR_ACTION_RECORDS}
    manifest = {'schema': SCHEMA, 'version': prereg['version'],
                'spec': {k: v for k, v in spec.items() if k != 'order_sheet'}, 'student': student,
                'code': code, 'prereg_sha256': sha(Path(args.prereg).read_bytes()),
                'static_map_sha256': host.scene.manifest.get('static_map_sha256'),
                'landmarks_sha256': host.scene.manifest.get('landmarks_sha256'),
                'scene_xml_sha256': sha(host.world.scene_xml.encode()), 'weld': host.scene.manifest.get('weld'),
                'contact_profile': host.contact_record, 'timestep_s': float(host.world.model.opt.timestep),
                'sync_sim': True, 'frame_period_s': host.FRAME_S, 'static_keepouts': host.keepout_records,
                'env': {'python': platform.python_version(), 'platform': platform.platform(),
                        'mujoco': mujoco.__version__, 'opencv': cv2.__version__, 'numpy': np.__version__,
                        'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                                   'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
                'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
                'wall_s': round(time.time() - started, 1)}
    manifest['files'] = {str(q.relative_to(out)): sha(q.read_bytes()) for q in sorted(out.rglob('*'))
                         if q.is_file() and q.suffix in ('.jsonl', '.xml', '.json') and q.name != 'manifest.json'}
    (out / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, default=str) + '\n')
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False, default=str) + '\n')
    host.close()
    print(json.dumps({'episode': args.episode, 'run': run, 'm1_style': result['m1_style_success_count'],
                      'confirmed': result['confirmed_count'], 'false_conf': result['false_confirmations'],
                      'per_robot': {r: (v['deliver_outcome'], v['m1_success'], v['m1_failed_checks'], v['guards']['pose_uncertain_events'])
                                    for r, v in robots.items()},
                      'wall_s': manifest['wall_s'], 'load': manifest['load_average']}, default=str), flush=True)


if __name__ == '__main__':
    main()
