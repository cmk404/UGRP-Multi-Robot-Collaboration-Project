#!/usr/bin/env python3
"""Privileged offline teacher diagnostic; never an RGB-only student result."""
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

from scripts.camera_approach_scene import ROBOTS
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.run_camera_approach_student import models, sha, write


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--max-forward', type=float, default=.10)
    args = p.parse_args()
    if not 0 < args.max_forward <= .10:
        p.error('max-forward must be positive and <=0.10')
    grasp_root, out = args.grasp_model_dir.resolve(), args.out_dir.resolve()
    _, grasp_models = models(grasp_root, 'student-skill.json')
    scene = ShortTransportScene(out, grasp_root)
    started = time.monotonic()
    report = {'schema': 'ugrp.short_transport_teacher.v1',
              'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'config': {'target_distance_m': .20, 'max_forward': args.max_forward,
                         'max_slices': 180, 'slice_s': .20, 'weld': False,
                         'setup': 'fixed demonstrated grasp station; no wheel approach'},
              'grasp_skill_sha256': sha(grasp_root / 'student-skill.json'),
              'teacher_privileged_inputs': 'payload/base xyz and contacts for offline demonstration only',
              'calls': [], 'grasp_calls': [], 'error': None}
    try:
        import mujoco
        from harness.grasp_student_inference import predict_student
        report['environment'] = {'python': sys.version, 'platform': platform.platform(),
                                 'mujoco': mujoco.__version__}
        scene.open()
        report['invariants_initial'] = scene.invariant_record()
        report['grasp_calls'] = scene.finish_grasp(predict_student, grasp_models)
        write(out / 'grasp-report.json', scene.grasp_report)
        origin = scene.evaluation_snapshot(full_state=False)
        report['carry_origin_evaluation_only'] = origin
        print(json.dumps({'stage': 'grasp_hold', 'evaluation': scene.grasp_report['evaluation'],
                          'pose': origin}), flush=True)
        if not scene.grasp_report['evaluation']['grasp_success']:
            raise RuntimeError('teacher initial grasp failed')
        histories = {r: [] for r in ROBOTS}
        ready = False
        for i in range(180):
            frames = scene.capture(f'carry-{i:03d}')
            truth = scene.evaluation_snapshot(full_state=False)
            progress = truth['position_m'][0] - origin['position_m'][0]
            base_progress = {r: truth['bases'][r][0] - origin['bases'][r][0] for r in ROBOTS}
            held = (truth['height_above_start_m'] >= .03 and
                    all(truth['contacts'][r]['bilateral'] for r in ROBOTS))
            ready = progress >= .197
            forwards = {r: 0. if ready else min(args.max_forward,
                           max(.02, (.20 - base_progress[r]) * 2.)) for r in ROBOTS}
            if not held:
                forwards = {r: 0. for r in ROBOTS}
            for r in ROBOTS:
                report['calls'].append({'index': i, 'robot_id': r,
                    'images': {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']},
                    'frame_id': frames[r]['frame_id'], 'own_command_history': list(histories[r]),
                    'teacher_labels': {'payload_progress_m': progress, 'base_progress_m': base_progress[r],
                                       'held': held, 'ready': ready},
                    'action': {'kind': 'drive', 'forward': forwards[r], 'turn': 0., 'duration_s': .2}})
            scene.carry_drive(forwards, stage='carry_stop' if ready or not held else 'carry')
            for r in ROBOTS:
                histories[r].append(report['calls'][-2 + ROBOTS.index(r)]['action'])
            if i % 10 == 0 or ready or not held:
                print(json.dumps({'stage': 'carry', 'index': i, 'progress_m': progress,
                                  'base_progress_m': base_progress, 'held': held,
                                  'forward': forwards}), flush=True)
            if not held:
                raise RuntimeError('teacher lost physical carry grip')
            if ready:
                break
        if not ready:
            raise RuntimeError('teacher carry slice budget exhausted')
        scene.phase = 'carry_stop'
        scene.tick(1.)
        scene.capture('carry-final')
        report['carry_final_evaluation_only'] = scene.evaluation_snapshot(full_state=False)
        scene.place()
        report['release_final_evaluation_only'] = scene.evaluation_snapshot(full_state=False)
        print(json.dumps({'stage': 'release_hold', 'pose': report['release_final_evaluation_only']}), flush=True)
    except Exception:
        report['error'] = traceback.format_exc()
        print(report['error'], flush=True)
    finally:
        report['wall_time_s'] = time.monotonic() - started
        report['sim_time_s'] = scene.time() if scene.world else None
        report['weld_active_ticks'] = scene.weld_active_ticks
        report['referee_ticks'] = scene.referee_ticks
        if scene.world:
            report['invariants_final'] = scene.invariant_record()
        scene.close()
        if out.exists():
            write(out / 'teacher-report.json', report)
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
