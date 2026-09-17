#!/usr/bin/env python3
"""RGB wheel approach with two settled image confirmations before grasp."""
from __future__ import annotations
import argparse
import hashlib
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
from scripts.camera_approach_scene import ApproachScene, ROBOTS, MAX_APPROACH_ROUNDS
from scripts.run_camera_pair_transport import evaluate_grasp_samples


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def models(root, manifest):
    skill = json.loads((root / manifest).read_text())
    if set(skill['models']) != set(ROBOTS):
        raise ValueError('expected two independent robot models')
    result = {}
    for rid, rec in skill['models'].items():
        path = (root / rec['path']).resolve()
        if not path.is_relative_to(root) or sha(path) != rec['sha256']:
            raise ValueError('model path/hash mismatch')
        result[rid] = json.loads(path.read_text())
    return skill, result


def choose_actions(decisions, condition, phase, index, playback_seconds):
    """Pure controller: predictions and its fixed playback clock only."""
    if condition == 'visual':
        valid = all(bool(decisions[r]['ok']) for r in ROBOTS)
        ready = {r: bool(decisions[r]['ok'] and decisions[r]['ready']) for r in ROBOTS}
    else:
        valid = True
        ready = {r: phase != 'cruise' or index * .2 + 1e-9 >= playback_seconds for r in ROBOTS}
    confirming = phase != 'cruise'
    blocked = not valid or (confirming and not all(ready.values()))
    enter_confirmation = valid and all(ready.values())
    duration = (.0 if phase == 'confirmation-2' else .25) if blocked or enter_confirmation else .2
    forwards = {}
    for rid in ROBOTS:
        if blocked or confirming or ready[rid]:
            forwards[rid] = 0.0
        elif condition == 'playback':
            forwards[rid] = .1
        else:
            forwards[rid] = max(0.0, min(.15, float(decisions[rid]['forward'])))
    actions = {r: {'kind': 'drive', 'forward': forwards[r], 'turn': .0, 'duration_s': duration} for r in ROBOTS}
    return {'actions': actions, 'blocked': blocked, 'ready': ready,
            'enter_confirmation': enter_confirmation, 'duration_s': duration}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--approach-model-dir', type=Path, required=True)
    p.add_argument('--distance', type=float, nargs=2, required=True, metavar=('R1', 'R3'))
    p.add_argument('--condition', choices=('visual', 'playback'), required=True)
    p.add_argument('--playback-seconds', type=float, default=1.0)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--act-python', type=Path, help='optional isolated LeRobot Python')
    p.add_argument('--act-model-dir', type=Path, help='comparison directory containing r1/act and r3/act')
    args = p.parse_args()
    if bool(args.act_python) != bool(args.act_model_dir) or (args.act_python and args.condition != 'visual'):
        p.error('ACT requires both --act-python and --act-model-dir, with --condition visual')
    if any(not math.isfinite(x) or not .2 <= x <= .3 for x in args.distance):
        p.error('distance must be 0.20..0.30 m')
    if not math.isfinite(args.playback_seconds) or not 0 < args.playback_seconds <= 20:
        p.error('playback seconds must be positive and <=20')
    grasp_root, approach_root = args.grasp_model_dir.resolve(), args.approach_model_dir.resolve()
    grasp_skill, grasp_models = models(grasp_root, 'student-skill.json')
    approach_skill, approach_models = models(approach_root, 'approach-skill.json')
    out = args.out_dir.resolve()
    if out.exists():
        raise FileExistsError(out)
    started = time.monotonic()
    scene = ApproachScene(out, grasp_root)
    report = {'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'config': {'condition': args.condition, 'distance': dict(zip(ROBOTS, args.distance)),
                         'playback_seconds': args.playback_seconds, 'max_rounds': MAX_APPROACH_ROUNDS,
                         'slice_s': .2, 'confirmation_dwell_s': .25, 'seed': scene.fixture['seed'], 'weld': False},
              'approach_skill_sha256': sha(approach_root / 'approach-skill.json'),
              'approach_model_sha256': {r: approach_skill['models'][r]['sha256'] for r in ROBOTS},
              'grasp_skill_sha256': sha(grasp_root / 'student-skill.json'),
              'grasp_model_sha256': {r: grasp_skill['models'][r]['sha256'] for r in ROBOTS},
              'input_boundary': 'own RGB, fixed top RGB, saved models, own issued command and controller history only',
              'approach_calls': [], 'goal_confirmation': {r: [] for r in ROBOTS}, 'calls': [],
              'approach_ok': False, 'grasp_success': False, 'evaluation': None, 'error': None}
    act_clients = {}
    try:
        import mujoco
        from harness.camera_approach_student import predict_approach
        from harness.grasp_student_inference import predict_student
        report['environment'] = {'python': sys.version, 'platform': platform.platform(), 'mujoco': mujoco.__version__}
        if args.act_python:
            from harness.reference_act_client import ActClient
            report['approach_policy'] = 'upstream_act_small_rgb_preset'
            report['act_checkpoint_sha256'] = {}
            for rid in ROBOTS:
                model_dir = args.act_model_dir.resolve() / rid / 'act'
                report['act_checkpoint_sha256'][rid] = {
                    name: sha(model_dir / name) for name in ('config.json', 'model.safetensors')}
                act_clients[rid] = ActClient(args.act_python, model_dir)
        else:
            report['approach_policy'] = 'existing_kernel'
        scene.open(dict(zip(ROBOTS, args.distance)))
        report['evaluation_initial_state'] = scene.evaluation_snapshot()
        approach_start = scene.time()
        history = {r: [] for r in ROBOTS}
        phase, cruise_index = 'cruise', 0
        for index in range(MAX_APPROACH_ROUNDS + 2):
            frames = scene.capture(f'approach-{index:03d}')
            decisions = {r: (act_clients[r].predict(frames[r]['own_bytes'], frames[r]['top_bytes'])
                             if act_clients else predict_approach(approach_models[r], frames[r]['own_bytes'], frames[r]['top_bytes']))
                         for r in ROBOTS}
            control = choose_actions(decisions, args.condition, phase, cruise_index, args.playback_seconds)
            for rid in ROBOTS:
                call = {'index': index, 'cruise_index': cruise_index, 'phase': phase,
                        'robot_id': rid, 'frame_id': frames[rid]['frame_id'],
                        'stationary': phase != 'cruise',
                        'images': {'own': frames[rid]['own_rgb'], 'top': frames[rid]['shared_top_rgb']},
                        'decision': decisions[rid], 'action': control['actions'][rid],
                        'own_command_history': list(history[rid])}
                report['approach_calls'].append(call)
                history[rid].append(control['actions'][rid])
                if phase != 'cruise':
                    report['goal_confirmation'][rid].append({'frame_id': frames[rid]['frame_id'],
                        'stationary': True, 'ok': not control['blocked'], 'ready': control['ready'][rid], 'index': index})
            scene.drive({r: control['actions'][r]['forward'] for r in ROBOTS}, control['duration_s'])
            if control['blocked']:
                report['stop_reason'] = 'RGB unsupported or stationary confirmation failed'
                break
            if phase == 'confirmation-2':
                report['approach_ok'] = True
                report['stop_reason'] = 'two fresh fixed-dwell stationary confirmations'
                break
            if phase == 'confirmation-1':
                phase = 'confirmation-2'
            elif control['enter_confirmation']:
                phase = 'confirmation-1'
            else:
                cruise_index += 1
                if cruise_index >= MAX_APPROACH_ROUNDS:
                    scene.stop_dwell()
                    report['stop_reason'] = 'bounded approach budget exhausted'
                    break
        report['approach_elapsed_sim_s'] = scene.time() - approach_start
        report['approach_end_state'] = scene.evaluation_snapshot()
        if report['approach_ok']:
            report['calls'] = scene.finish_grasp(predict_student, grasp_models)
            scene.grasp_report['source_sha'] = report['source_sha']
            write(out / 'grasp-result.json', scene.grasp_report)
        report['evaluation'] = evaluate_grasp_samples(scene.evaluation_samples)
        report['grasp_success'] = report['evaluation']['grasp_success']
        report['final_physics'] = scene.evaluation_snapshot()
        report['approach_payload_contact_steps'] = scene.approach_payload_contact_steps
        report['approach_physics_steps'] = scene.approach_physics_steps
        report['approach_collision_events'] = scene.approach_collision_events
        report['success'] = report['approach_ok'] and report['grasp_success'] and scene.approach_payload_contact_steps == 0
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        report['traceback'] = traceback.format_exc()
    finally:
        try:
            scene.close()
        finally:
            for client in act_clients.values():
                client.close()
        report.setdefault('success', False)
        report['wall_elapsed_s'] = time.monotonic() - started
        out.mkdir(parents=True, exist_ok=True)
        write(out / 'result.json', report)
    print(json.dumps({key: report[key] for key in ('approach_ok', 'grasp_success', 'success', 'error')}), flush=True)
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
