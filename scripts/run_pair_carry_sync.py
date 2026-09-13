#!/usr/bin/env python3
"""Paired baseline/synchronization experiment with private timed fault injection."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from harness.camera_short_transport_student import predict_transport
from harness.grasp_student_inference import predict_student
from harness.pair_carry_policy import PairCarryPolicy, payload_skew, ROBOTS
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_approach_student import models, sha, write
from scripts.run_camera_short_transport_student import load_transport_models, choose_carry_actions


def validate_case(case):
    if not isinstance(case, dict) or not isinstance(case.get('case_id'), str):
        raise ValueError('case_id required')
    for name in ('actuation_fault', 'report_blackout'):
        fault = case.get(name)
        if fault is None:
            continue
        if set(fault) != {'robot_id', 'start_s', 'duration_s'} or fault['robot_id'] not in ROBOTS:
            raise ValueError('invalid private fault')
        for key in ('start_s', 'duration_s'):
            value = fault[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 10:
                raise ValueError('fault seconds must be finite in 0..10')
    return case


def execute_faulted(scene, commands, duration, fault, origin, stage):
    """Executor-only interception. Applied state never enters policy/coordination."""
    start = scene.time()
    cuts = [start, start + duration]
    if fault and origin is not None:
        cuts += [origin + fault['start_s'], origin + fault['start_s'] + fault['duration_s']]
    cuts = sorted(set(max(start, min(start + duration, t)) for t in cuts))
    for a, b in zip(cuts, cuts[1:]):
        if b - a <= 1e-8:
            continue
        applied = dict(commands)
        if fault and origin is not None and fault['start_s'] <= (a+b)/2-origin < fault['start_s']+fault['duration_s']:
            applied[fault['robot_id']] = 0.
        scene.carry_drive(applied, min(.25, b-a), stage=stage)


def run(case, condition, grasp_root, transport_root, out):
    validate_case(case)
    skill, grasp_models = models(grasp_root, 'student-skill.json')
    transport_skill, transport_models = load_transport_models(transport_root)
    scene = ShortTransportScene(out, grasp_root)
    policy = PairCarryPolicy(case['case_id'])
    started = time.monotonic()
    result = {'schema': 'ugrp.pair_carry_sync.v1', 'case_id': case['case_id'],
              'condition': condition, 'setup_only': case,
              'source_sha': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
              'grasp_skill_sha256': sha(grasp_root/'student-skill.json'),
              'transport_skill_sha256': sha(transport_root/'short-transport-skill.json'),
              'transport_model_sha256': {r: transport_skill['models'][r]['sha256'] for r in ROBOTS},
              'scope': 'Fixed grasp start; local RGB execution synchronization, no LLM; demonstrated placement',
              'steps': [], 'carry_ready': False, 'error': None, 'external_model_calls': 0, 'cost_usd': 0}
    try:
        import mujoco
        result['environment'] = {'python': sys.version, 'platform': platform.platform(), 'mujoco': mujoco.__version__}
        scene.open()
        result['invariants_initial'] = scene.invariant_record()
        scene.finish_grasp(predict_student, grasp_models)
        write(out/'grasp-result.json', scene.grasp_report)
        anchor = scene.capture('carry-anchor')
        result['anchor_images'] = {r: {'own': anchor[r]['own_rgb'], 'top': anchor[r]['shared_top_rgb']} for r in ROBOTS}
        histories = {r: [] for r in ROBOTS}
        origin = scene.time()
        first_motion_s = None
        confirming, consecutive = False, 0
        for index in range(150):
            request_wall = time.monotonic()-started
            frames = anchor if index == 0 else scene.capture(f'carry-{index:03d}')
            now = scene.time()-origin
            decisions = {r: predict_transport(transport_models[r], frames[r]['own_bytes'], frames[r]['top_bytes'],
                            anchor[r]['own_bytes'], anchor[r]['top_bytes'], own_command_history=list(histories[r])) for r in ROBOTS}
            skew = payload_skew(frames['r1']['top_bytes'])
            blackout = case.get('report_blackout')
            delivered = [r for r in ROBOTS if not (blackout and r == blackout['robot_id']
                         and blackout['start_s'] <= now < blackout['start_s']+blackout['duration_s'])]
            # Fault information ends at relay/executor. Policy sees only delivered reports.
            if condition == 'sync':
                control = policy.step(decisions, skew, {r: frames[r]['frame_id'] for r in ROBOTS}, now, delivered)
            else:
                base = choose_carry_actions(decisions, confirming)
                control = {**base, 'mode': 'CONFIRM' if confirming else 'CRUISE',
                           'abort': not base['valid'], 'done': False, 'permission': None,
                           'skew_error_px': None, 'recovery_count': 0}
                if confirming:
                    consecutive = consecutive + 1 if base['ready'] else 0
                    control['done'] = consecutive >= 2
                    if not base['ready']:
                        confirming = False
                elif base['ready']:
                    confirming = True
            actions = {r: {'kind': 'drive', 'forward': control['forwards'][r], 'turn': 0.,
                           'duration_s': control['duration_s']} for r in ROBOTS}
            row = {'index': index, 'observed_at_s': now, 'request_wall_s': request_wall,
                   'decision_wall_s': time.monotonic()-started,
                   'frame_ids': {r: frames[r]['frame_id'] for r in ROBOTS},
                   'images': {r: {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']} for r in ROBOTS},
                   'own_command_histories': {r: list(histories[r]) for r in ROBOTS},
                   'decisions': decisions, 'skew_px': skew, 'delivered_reports': delivered,
                   'control': control, 'actions': actions}
            for r in ROBOTS:
                histories[r].append(actions[r])
            if first_motion_s is None and any(control['forwards'].values()):
                first_motion_s = scene.time()
            stage = 'carry' if any(control['forwards'].values()) else 'carry_stop'
            row['command_issued_wall_s'] = time.monotonic()-started
            execute_faulted(scene, control['forwards'], control['duration_s'], case.get('actuation_fault'), first_motion_s, stage)
            row['execution_end_s'] = scene.time()-origin
            row['execution_end_wall_s'] = time.monotonic()-started
            result['steps'].append(row)
            if control['abort']:
                raise RuntimeError('policy stopped: '+str(control.get('permission') or 'invalid RGB'))
            if control['done']:
                result['carry_ready'] = True
                break
        if not result['carry_ready']:
            raise RuntimeError('carry decision budget exhausted')
        scene.place()
    except Exception:
        result['error'] = traceback.format_exc()
    finally:
        # Evaluation truth is consumed only after policy decisions have ended.
        result['evaluation'] = evaluate_transport_samples(scene.evaluation_samples)
        result['sync_events'] = policy.sync.events
        result['policy_events'] = policy.events
        result['weld_active_ticks'] = scene.weld_active_ticks
        result['wall_time_s'] = time.monotonic()-started
        if scene.world:
            result['invariants_final'] = scene.invariant_record()
            result['sim_time_s'] = scene.time()
        result['success'] = bool(result['error'] is None and result['carry_ready'] and result['evaluation']['success']
                                 and scene.weld_active_ticks == 0 and result.get('invariants_initial') == result.get('invariants_final'))
        scene.close()
        if out.exists():
            write(out/'result.json', result)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case-json', type=Path, required=True)
    p.add_argument('--condition', choices=('baseline','sync'), required=True)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--transport-model-dir', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    args = p.parse_args()
    d = run(json.loads(args.case_json.read_text()), args.condition, args.grasp_model_dir.resolve(),
            args.transport_model_dir.resolve(), args.out_dir.resolve())
    print(json.dumps({k: d[k] for k in ('case_id','success','error','evaluation')}), flush=True)
    return 1 if d['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
