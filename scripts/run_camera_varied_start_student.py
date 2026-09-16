#!/usr/bin/env python3
"""RGB-only phased wheel alignment and the existing local grasp student."""
from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.camera_approach_scene import ApproachScene, ROBOTS, validate_start_poses
from scripts.run_camera_approach_student import models, sha, write
from scripts.run_camera_pair_transport import evaluate_grasp_samples

from harness.camera_varied_start_student import PHASES
LIMITS = {'yaw': 90, 'lateral': 110, 'forward': 160}
AXES = {'yaw': 'turn', 'lateral': 'left', 'forward': 'forward'}


def load_stage_models(root):
    root = root.resolve()
    skill = json.loads((root / 'varied-start-skill.json').read_text())
    if set(skill['models']) != set(ROBOTS):
        raise ValueError('two independent robot models required')
    result = {}
    for rid, stages in skill['models'].items():
        if set(stages) != set(AXES):
            raise ValueError('three stage models required')
        result[rid] = {}
        for stage, rec in stages.items():
            path = (root / rec['path']).resolve()
            if not path.is_relative_to(root) or sha(path) != rec['sha256']:
                raise ValueError('stage model path/hash mismatch')
            model = json.loads(path.read_text())
            if model['robot_id'] != rid or model['stage'] != stage:
                raise ValueError('stage model identity mismatch')
            result[rid][stage] = model
    return skill, result


def choose_stage_actions(decisions, stage, confirming=False):
    """Use image predictions only. Confirmation never drives the wheels."""
    valid = all(bool(decisions[r]['ok']) for r in ROBOTS)
    ready = {r: bool(decisions[r]['ok'] and
                    (decisions[r].get('stationary_ready', decisions[r]['ready'])
                     if confirming else decisions[r]['ready'])) for r in ROBOTS}
    commands = {r: dict(forward=0., left=0., turn=0.) for r in ROBOTS}
    if valid and not confirming:
        for r in ROBOTS:
            if not ready[r]:
                commands[r][AXES[stage]] = float(decisions[r]['command'])
    enter_confirmation = valid and all(ready.values())
    return {'commands': commands, 'valid': valid, 'ready': ready,
            'enter_confirmation': enter_confirmation,
            'duration_s': .25 if confirming or enter_confirmation or not valid else .2}


def choose_alignment_refinement(decisions, confirming=False):
    """Recheck every image-derived axis after coupled final corrections."""
    valid = all(d['ok'] and d.get('precision', 'fine') == 'fine'
                for axes in decisions.values() for d in axes.values())
    ready = {r: all(d.get('stationary_ready', d['ready']) if confirming else d['ready']
                    for d in decisions[r].values()) for r in ROBOTS}
    commands = {r: dict(forward=0., left=0., turn=0.) for r in ROBOTS}
    axis = None
    if valid and not confirming:
        axis = next((s for s in ('yaw', 'lateral', 'forward')
                     if not all(decisions[r][s]['ready'] for r in ROBOTS)), None)
        if axis is not None:
            for r in ROBOTS:
                if not decisions[r][axis]['ready']:
                    commands[r][AXES[axis]] = float(decisions[r][axis]['command'])
    return dict(valid=valid, ready=ready, commands=commands, axis=axis,
                enter_confirmation=valid and all(ready.values()),
                duration_s=.25 if confirming or axis is None or not valid else .2)


def run_approach(scene, stage_models, *, condition='visual', straight_models=None,
                 reacquire_on_settle=False, final_refinement_steps=0):
    """Run the RGB-only wheel approach and return output-only audit data."""
    from harness.camera_varied_start_student import predict_stage
    from harness.camera_approach_student import predict_approach

    if condition not in ('visual', 'straight'):
        raise ValueError(f'unsupported approach condition: {condition}')
    if condition == 'straight' and straight_models is None:
        raise ValueError('straight baseline requires old models')

    if (isinstance(final_refinement_steps, bool) or not isinstance(final_refinement_steps, int)
            or not 0 <= final_refinement_steps <= 40):
        raise ValueError('final refinement budget must be 0..40 slices')

    result = {'approach_calls': [], 'stage_results': [], 'approach_ok': False}
    approach_start = scene.time()
    phases = PHASES if condition == 'visual' else ('forward',)
    history = {r: [] for r in ROBOTS}
    for phase_index, stage in enumerate(phases):
        confirming, confirmations, consecutive, movements = False, 0, 0, 0
        record = {'phase_index': phase_index, 'stage': stage, 'ok': False, 'confirmations': []}
        result['stage_results'].append(record)
        for index in range(LIMITS[stage] + 5):
            frames = scene.capture(f'phase-{phase_index}-{index:03d}')
            if condition == 'visual':
                decisions = {r: predict_stage(stage_models[r][stage], frames[r]['own_bytes'], frames[r]['top_bytes']) for r in ROBOTS}
            else:
                decisions = {}
                for r in ROBOTS:
                    d = predict_approach(straight_models[r], frames[r]['own_bytes'], frames[r]['top_bytes'])
                    decisions[r] = {**d, 'command': d['forward']}
            control = choose_stage_actions(decisions, stage, confirming)
            for rid in ROBOTS:
                action = {'kind': 'mecanum', **control['commands'][rid], 'duration_s': control['duration_s']}
                result['approach_calls'].append({'phase_index': phase_index, 'stage': stage,
                    'index': index, 'robot_id': rid, 'frame_id': frames[rid]['frame_id'],
                    'stationary': confirming, 'images': {'own': frames[rid]['own_rgb'], 'top': frames[rid]['shared_top_rgb']},
                    'decision': decisions[rid], 'action': action, 'own_command_history': list(history[rid])})
                history[rid].append(action)
            scene.drive_mecanum(control['commands'], control['duration_s'])
            if not control['valid']:
                record['reason'] = 'RGB outside learned stage support'
                break
            if confirming:
                confirmations += 1
                consecutive = consecutive + 1 if all(control['ready'].values()) else 0
                record['confirmations'].append({'frame_ids': {r: frames[r]['frame_id'] for r in ROBOTS},
                                               'ready': control['ready'], 'stationary': True})
                if consecutive >= 2:
                    record['ok'] = True
                    record['reason'] = 'two consecutive fresh stationary RGB confirmations'
                    break
                if confirmations >= 4:
                    record['reason'] = 'stationary RGB confirmation budget exhausted'
                    break
                if reacquire_on_settle and not all(control['ready'].values()):
                    # The current slice remains stationary. Resume correction
                    # only after another fresh image, under the same movement
                    # and total confirmation budgets. Never relax readiness.
                    confirming = False
                    record['reacquisitions'] = record.get('reacquisitions', 0) + 1
            elif control['enter_confirmation']:
                confirming = True
            else:
                movements += 1
                if movements >= LIMITS[stage]:
                    scene.stop_dwell()
                    record['reason'] = 'bounded phase movement budget exhausted'
                    break
        record['movement_slices'] = movements
        if not record['ok']:
            break
    result['approach_ok'] = len(result['stage_results']) == len(phases) and all(s['ok'] for s in result['stage_results'])
    # Recheck all three axes with stationary images before arm deployment.
    if result['approach_ok'] and condition == 'visual':
        result['final_alignment_checks'] = []
        for index in range(2):
            frames = scene.capture(f'final-alignment-{index}')
            checks = {r: {s: predict_stage(stage_models[r][s], frames[r]['own_bytes'], frames[r]['top_bytes']) for s in AXES} for r in ROBOTS}
            result['final_alignment_checks'].append({'frame_ids': {r: frames[r]['frame_id'] for r in ROBOTS},
                'images': {r: {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']} for r in ROBOTS}, 'decisions': checks})
            result['approach_ok'] &= all(d['ok'] and d.get('stationary_ready', d['ready']) and d.get('precision', 'fine') == 'fine'
                                         for stages in checks.values() for d in stages.values())
            scene.stop_dwell()
        if not result['approach_ok'] and final_refinement_steps:
            # A later forward correction can invalidate an earlier lateral
            # confirmation. Do not loosen its tolerance or proceed to grasp:
            # reacquire the joint ready set from new RGB under a hard budget.
            result['alignment_refinement_calls'] = []
            result['alignment_refinement_reason'] = 'bounded refinement budget exhausted'
            confirming, consecutive = False, 0
            for index in range(final_refinement_steps):
                frames = scene.capture(f'alignment-refine-{index:03d}')
                checks = {r: {s: predict_stage(stage_models[r][s], frames[r]['own_bytes'], frames[r]['top_bytes']) for s in AXES} for r in ROBOTS}
                control = choose_alignment_refinement(checks, confirming)
                result['alignment_refinement_calls'].append(dict(index=index, stationary=confirming,
                    frame_ids={r: frames[r]['frame_id'] for r in ROBOTS},
                    images={r: dict(own=frames[r]['own_rgb'], top=frames[r]['shared_top_rgb']) for r in ROBOTS},
                    decisions=checks, control=control))
                scene.drive_mecanum(control['commands'], control['duration_s'])
                if not control['valid']:
                    result['alignment_refinement_reason'] = 'RGB outside fine alignment support'
                    break
                if confirming:
                    consecutive = consecutive + 1 if all(control['ready'].values()) else 0
                    if consecutive >= 2:
                        result['approach_ok'] = True
                        result['alignment_refinement_reason'] = 'two fresh stationary confirmations of all axes'
                        break
                    if not all(control['ready'].values()):
                        confirming = False
                elif control['enter_confirmation']:
                    confirming = True
            if not result['approach_ok']:
                scene.stop_dwell()
    result['approach_elapsed_sim_s'] = scene.time() - approach_start
    result['approach_end_state'] = scene.evaluation_snapshot()
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--stage-model-dir', type=Path, required=True)
    p.add_argument('--straight-model-dir', type=Path)
    p.add_argument('--condition', choices=('visual', 'straight'), default='visual')
    p.add_argument('--case-json', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    args = p.parse_args()
    case = json.loads(args.case_json.read_text())
    starts = validate_start_poses(case['start_poses'])
    if any(not .15 <= v['distance_m'] <= .4 or abs(v['lateral_m']) > .06
           or abs(v['yaw_deg']) > 10 for v in starts.values()):
        p.error('student starts require distance 0.15..0.40 m, lateral +/-0.06 m, yaw +/-10 degrees')
    grasp_root, stage_root = args.grasp_model_dir.resolve(), args.stage_model_dir.resolve()
    grasp_skill, grasp_models = models(grasp_root, 'student-skill.json')
    stage_skill, stage_models = load_stage_models(stage_root)
    straight_models = None
    straight_skill = None
    if args.condition == 'straight':
        if args.straight_model_dir is None:
            p.error('straight baseline requires old model directory')
        straight_skill, straight_models = models(args.straight_model_dir.resolve(), 'approach-skill.json')
    out = args.out_dir.resolve()
    if out.exists():
        raise FileExistsError(out)
    started = time.monotonic()
    scene = ApproachScene(out, grasp_root)
    report = {'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'case_id': case['case_id'], 'config': {'condition': args.condition,
                  'start_poses_setup_only': starts, 'phases': list(PHASES), 'limits': LIMITS,
                  'slice_s': .2, 'stop_dwell_s': .25, 'max_confirmation_frames': 4,
                  'required_consecutive_stationary_ready_frames': 2, 'weld': False},
              'stage_skill_sha256': sha(stage_root / 'varied-start-skill.json'),
              'stage_model_sha256': {r: {s: v['sha256'] for s, v in rec.items()}
                                     for r, rec in stage_skill['models'].items()},
              'grasp_skill_sha256': sha(grasp_root / 'student-skill.json'),
              'grasp_model_sha256': {r: grasp_skill['models'][r]['sha256'] for r in ROBOTS},
              'input_boundary': 'Own RGB, fixed top RGB, saved models, own issued commands and phase history only',
              'approach_calls': [], 'stage_results': [], 'calls': [],
              'approach_ok': False, 'grasp_success': False, 'success': False, 'error': None}
    if straight_skill is not None:
        report['straight_skill_sha256'] = sha(args.straight_model_dir.resolve() / 'approach-skill.json')
        report['straight_model_sha256'] = {r: straight_skill['models'][r]['sha256'] for r in ROBOTS}
    try:
        import mujoco
        from harness.grasp_student_inference import predict_student
        report['environment'] = {'python': sys.version, 'platform': platform.platform(), 'mujoco': mujoco.__version__}
        scene.open(start_poses=starts)
        report['evaluation_initial_state'] = scene.evaluation_snapshot()
        report.update(run_approach(scene, stage_models, condition=args.condition,
                                   straight_models=straight_models))
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
        report['success'] = bool(report['approach_ok'] and report['grasp_success'] and scene.approach_payload_contact_steps == 0)
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        report['traceback'] = traceback.format_exc()
    finally:
        scene.close()
        report['wall_elapsed_s'] = time.monotonic() - started
        out.mkdir(parents=True, exist_ok=True)
        write(out / 'result.json', report)
    print(json.dumps({k: report[k] for k in ('case_id', 'approach_ok', 'grasp_success', 'success', 'error')}), flush=True)
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
