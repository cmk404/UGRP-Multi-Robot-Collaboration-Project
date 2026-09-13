#!/usr/bin/env python3
"""Independent RGB carry with demonstrated lower/open/retract commands."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.camera_approach_scene import ROBOTS, validate_start_poses
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_approach_student import models, sha, write


def choose_carry_actions(decisions, confirming=False):
    valid = set(decisions) == set(ROBOTS) and all(
        d.get('ok') is True and d.get('held_estimate') is True for d in decisions.values())
    ready = valid and all(d.get('ready') is True for d in decisions.values())
    stop = not valid or ready or confirming
    return {'valid': valid, 'ready': ready,
            'duration_s': .25 if stop else .20,
            'forwards': {r: 0. if stop else float(decisions[r]['forward']) for r in ROBOTS}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--transport-model-dir', type=Path, required=True)
    p.add_argument('--stage-model-dir', type=Path)
    p.add_argument('--case-json', type=Path, required=True)
    p.add_argument('--condition', choices=('visual', 'playback'), default='visual')
    p.add_argument('--replay-teacher-dir', type=Path)
    p.add_argument('--out-dir', type=Path, required=True)
    args = p.parse_args()
    case = json.loads(args.case_json.read_text())
    delays = case.get('departure_delay_slices', {r: 0 for r in ROBOTS})
    if set(delays) != set(ROBOTS) or any(type(v) is not int or not 0 <= v <= 10 for v in delays.values()):
        p.error('departure delay requires r1/r3 integer slices 0..10')
    starts = validate_start_poses(case['start_poses']) if 'start_poses' in case else None
    if starts is not None and (args.stage_model_dir is None or
            any(not .15 <= x['distance_m'] <= .40 or abs(x['lateral_m']) > .06
                or abs(x['yaw_deg']) > 10 for x in starts.values())):
        p.error('varied starts need stage model and prior validated start bounds')
    grasp_root, model_root, out = args.grasp_model_dir.resolve(), args.transport_model_dir.resolve(), args.out_dir.resolve()
    _, grasp_models = models(grasp_root, 'student-skill.json')
    transport_skill, transport_models = models(model_root, 'short-transport-skill.json')
    replay = None
    if args.condition == 'playback':
        if args.replay_teacher_dir is None:
            p.error('playback requires its recorded teacher directory')
        teacher = json.loads((args.replay_teacher_dir / 'teacher-report.json').read_text())
        indexes = sorted(set(c['index'] for c in teacher['calls']))
        replay = [{r: next(c['action'] for c in teacher['calls'] if c['index'] == i and c['robot_id'] == r)
                   for r in ROBOTS} for i in indexes]
    scene, started = ShortTransportScene(out, grasp_root), time.monotonic()
    report = {'schema': 'ugrp.short_transport_student.v1', 'case_id': case['case_id'],
              'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'config': {'condition': args.condition, 'setup_only': case, 'weld': False,
                         'maximum_carry_slices': 100, 'stationary_confirmations': 2},
              'input_boundary': 'Each actor receives own RGB, fixed top RGB, initial carry RGB, saved model and own issued commands only.',
              'scope': 'RGB learned carry; demonstrated local grasp and lower/open/retract command sequence',
              'transport_skill_sha256': sha(model_root / 'short-transport-skill.json'),
              'transport_model_sha256': {r: transport_skill['models'][r]['sha256'] for r in ROBOTS},
              'grasp_skill_sha256': sha(grasp_root / 'student-skill.json'),
              'approach_ok': starts is None, 'carry_calls': [], 'grasp_calls': [],
              'carry_ready': False, 'error': None, 'success': False}
    if replay is not None:
        report['playback_actions'] = replay
        report['playback_teacher_source'] = {'path': str(args.replay_teacher_dir.resolve()),
            'sha256': sha(args.replay_teacher_dir / 'teacher-report.json')}
    try:
        import mujoco
        from harness.grasp_student_inference import predict_student
        from harness.camera_short_transport_student import predict_transport
        report['environment'] = {'python': sys.version, 'platform': platform.platform(), 'mujoco': mujoco.__version__}
        scene.open(start_poses=starts) if starts is not None else scene.open()
        report['invariants_initial'] = scene.invariant_record()
        if starts is not None:
            from scripts.run_camera_varied_start_student import load_stage_models, run_approach
            _, stage_models = load_stage_models(args.stage_model_dir.resolve())
            report.update(run_approach(scene, stage_models))
        if not report['approach_ok']:
            raise RuntimeError('RGB approach did not qualify')
        report['grasp_calls'] = scene.finish_grasp(predict_student, grasp_models)
        write(out / 'grasp-result.json', scene.grasp_report)
        # The grasp referee result is intentionally never read by this actor loop.
        anchor = scene.capture('carry-anchor')
        report['actor_initial_images'] = {r: {'own': anchor[r]['own_rgb'], 'top': anchor[r]['shared_top_rgb']} for r in ROBOTS}
        report['actor_initial_issued_arm_commands'] = {r: dict(scene.commands[r]) for r in ROBOTS}
        histories = {r: [] for r in ROBOTS}
        confirming, consecutive = False, 0
        for index in range(100):
            frames = anchor if index == 0 else scene.capture(f'carry-{index:03d}')
            decisions = {r: predict_transport(transport_models[r], frames[r]['own_bytes'], frames[r]['top_bytes'],
                anchor[r]['own_bytes'], anchor[r]['top_bytes'], own_command_history=list(histories[r])) for r in ROBOTS}
            if replay is None:
                control = choose_carry_actions(decisions, confirming)
            else:
                ready = index >= len(replay)
                control = {'valid': True, 'ready': ready, 'duration_s': .25 if ready else .2,
                           'forwards': {r: 0. if ready else float(replay[index][r]['forward']) for r in ROBOTS}}
            actions = {r: {'kind': 'drive', 'forward': control['forwards'][r], 'turn': 0.,
                           'duration_s': control['duration_s']} for r in ROBOTS}
            for r in ROBOTS:
                report['carry_calls'].append({'index': index, 'robot_id': r, 'frame_id': frames[r]['frame_id'],
                    'stationary': confirming, 'images': {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']},
                    'own_command_history': list(histories[r]), 'decision': decisions[r], 'action': actions[r]})
                histories[r].append(actions[r])
            # Fault injection belongs to execution setup; never expose applied state to actors.
            applied = {r: 0. if index < delays[r] else control['forwards'][r] for r in ROBOTS}
            stage = 'carry_stop' if confirming or control['ready'] or not control['valid'] else 'carry'
            scene.carry_drive(applied, control['duration_s'], stage=stage)
            if not control['valid']:
                raise RuntimeError('RGB carry state outside learned support')
            if confirming:
                consecutive = consecutive + 1 if control['ready'] else 0
                if consecutive >= 2:
                    report['carry_ready'] = True
                    break
                if not control['ready']:
                    # Fresh images may show that coasting stopped short; bounded loop resumes.
                    confirming = False
            elif control['ready']:
                confirming = True
        if not report['carry_ready']:
            raise RuntimeError('RGB carry decision budget exhausted')
        scene.place()
    except Exception:
        report['error'] = traceback.format_exc()
    finally:
        # Output-only truth is evaluated only after every actor decision has ended.
        report['evaluation'] = evaluate_transport_samples(scene.evaluation_samples)
        report['weld_active_ticks'] = scene.weld_active_ticks
        report['referee_ticks'] = scene.referee_ticks
        report['approach_payload_contact_steps'] = scene.approach_payload_contact_steps
        report['wall_time_s'] = time.monotonic() - started
        report['sim_time_s'] = scene.time() if scene.world else None
        if scene.world:
            report['invariants_final'] = scene.invariant_record()
        report['success'] = bool(report['error'] is None and report['approach_ok'] and report['carry_ready']
            and report['evaluation']['success'] and scene.weld_active_ticks == 0
            and scene.approach_payload_contact_steps == 0
            and report.get('invariants_initial') == report.get('invariants_final'))
        scene.close()
        if out.exists():
            write(out / 'result.json', report)
    print(json.dumps({k: report[k] for k in ('case_id', 'success', 'error', 'evaluation')}), flush=True)
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
