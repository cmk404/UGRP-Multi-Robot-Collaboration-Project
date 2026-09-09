#!/usr/bin/env python3
"""Two independent image-only actors; physical truth is an output-only referee."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def evaluate_grasp_samples(samples: list[dict], sample_period_s: float = 0.1) -> dict:
    """Score an unassisted dual grasp from output-only physics samples."""
    longest_s = 0.0
    current_s = 0.0
    previous_time = None
    previous_qualifies = False
    max_lift_m = max(
        (float(sample.get('height_above_start_m', 0.0)) for sample in samples),
        default=0.0,
    )
    for sample in samples:
        now = float(sample['sim_time_s'])
        contacts = sample.get('contacts', {})
        constraints = sample.get('constraints_active', {})
        qualifies = (
            float(sample.get('height_above_start_m', 0.0)) >= 0.03
            and all(bool(contacts.get(rid, {}).get('bilateral')) for rid in ('r1', 'r3'))
            and all(constraints.get(rid) is False for rid in ('r1', 'r3'))
        )
        contiguous = previous_time is not None and 0 < now - previous_time <= sample_period_s * 1.5
        if qualifies:
            current_s = current_s + (now - previous_time) if contiguous and previous_qualifies else 0.0
        else:
            current_s = 0.0
        previous_qualifies = qualifies
        longest_s = max(longest_s, current_s)
        previous_time = now
    return {
        'grasp_success': longest_s + 1e-9 >= 2.0,
        'longest_qualifying_duration_s': round(longest_s, 6),
        'max_lift_m': round(max_lift_m, 6),
        'sample_count': len(samples),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=8)
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--model', default='gemini-3.8-flash')
    parser.add_argument('--timeout', type=float, default=45)
    parser.add_argument('--task', choices=('carry', 'grasp'), default='carry')
    args = parser.parse_args()
    if not 1 <= args.rounds <= 30:
        parser.error('rounds must be 1..30 (2..60 bounded requests total)')
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)

    import mujoco
    from harness.camera_pair_policy import CameraPairPlanner
    from harness.gemini_proxy import GeminiProxyCompleter
    from sim.camera_robot_port import CameraRobotPort
    import sim.multi_masterpi_production as production
    from scripts.probe_dual_grasp_sync import (
        _plain_beam_contact, _plain_beam_xml, _pose_metrics, _git, Video,
    )

    # This trusted fixture owner may place initial bodies. None of these values
    # or the world object are passed to either planner or the model request.
    with patch.object(production, 'build_multi_robot_xml',
                      _plain_beam_xml(production.build_multi_robot_xml)):
        world = production.MultiMasterPiProductionV2(seed=args.seed, render=True,
                                                    width=640, height=480)
    report = {'git_sha': _git(['rev-parse', 'HEAD']), 'config': vars(args).copy(),
              'input_contract': 'own robot_cam JPEG + shared fixed overhead JPEG only',
              'calls': [], 'rounds_completed': 0, 'error': None,
              'transport_success': None, 'grasp_success': None,
              'success_claim': 'grasp evaluation pending fixed round budget'}
    report['config']['out_dir'] = str(out)
    video = None
    planners = {}
    try:
        for rid, y in [('r1', -2.325), ('r3', -1.675)]:
            world.controllers[rid].set_base_pose_for_test((-.02, y, .0324), 0.0)
        # Fixed room camera, not target tracking. Both model input and video use
        # this framing, but only the video receives presentation overlays.
        for name in ('cctv_top', 'cctv_warehouse'):
            cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            world.model.cam_pos[cid] = (.55, -2.0, 2.5)
            world.model.cam_quat[cid] = (1, 0, 0, 0)
            world.model.cam_fovy[cid] = 55
        mujoco.mj_forward(world.model, world.data)
        ports = {rid: CameraRobotPort(world, rid) for rid in ('r1', 'r3')}
        request_counts = {'r1': 0, 'r3': 0}
        for rid in ports:
            (out / rid).mkdir()
            def audited_open(request, *, timeout, _rid=rid):
                request_counts[_rid] += 1
                # Exact HTTP request body, before sending, including original
                # image bytes. No headers, endpoint credentials or secrets.
                (out / _rid / f'wire-{request_counts[_rid]:03d}.json').write_bytes(request.data)
                return urlopen(request, timeout=timeout)
            planners[rid] = CameraPairPlanner(rid, GeminiProxyCompleter(
                model=args.model, max_tokens=600, timeout=args.timeout,
                reasoning_effort='none', http_open=audited_open), task=args.task)
        video = Video(world, out / 'motion.mp4', 12)
        world.frame_callback = video.capture
        video.capture(force=True)
        samples = []
        next_referee_sample = float(world.data.time) + 0.1
        with (out / 'evaluation-only.jsonl').open('w') as referee, ThreadPoolExecutor(max_workers=2) as pool:
            for index in range(args.rounds):
                # Snapshot both actors at the same instant. Inference latency
                # pauses physics; no truth-derived intervention or correction.
                overhead = world.render_team_jpeg(camera='cctv_top')
                futures = {}
                for rid, planner in planners.items():
                    own = world.render_jpeg(robot_id=rid, camera='robot_cam')
                    for label, data in [('own', own), ('overhead', overhead)]:
                        (out / rid / f'{index:03d}-{label}.jpg').write_bytes(data)
                    futures[rid] = pool.submit(planner.decide, own, overhead)
                actions = {rid: future.result() for rid, future in futures.items()}
                now = float(world.data.time)
                for rid, action in actions.items():
                    ports[rid].apply(action, now)
                    report['calls'].append({'round': index, 'robot_id': rid,
                        'action': action, 'response': planners[rid].last_response,
                        'usage': planners[rid].completer.last_usage})
                video.stage = f'image_only_round_{index + 1}'
                # Fixed command-time slice. Servo interpolation and wheel lease
                # expiry are actuator execution, never target-position control.
                for _ in range(round(1.0 / float(world.model.opt.timestep))):
                    for port in ports.values():
                        port.tick(float(world.data.time))
                    world._physics_step_for(world.controllers['r1'])
                    video.capture()
                    if float(world.data.time) + 1e-9 >= next_referee_sample:
                        sample = {
                            'event': 'grasp_referee_sample',
                            **_pose_metrics(world),
                            'contacts': {
                                rid: _plain_beam_contact(world, rid)
                                for rid in ('r1', 'r3')
                            },
                        }
                        samples.append(sample)
                        referee.write(json.dumps(sample) + '\n')
                        referee.flush()
                        next_referee_sample += 0.1
                for port in ports.values():
                    port.tick(float(world.data.time))
                    port.stop()
                report['rounds_completed'] += 1
                (out / 'progress.json').write_text(json.dumps(report, indent=2))
        evaluation = evaluate_grasp_samples(samples)
        report.update(evaluation)
        report['success_claim'] = (
            f"evaluated grasp_success={evaluation['grasp_success']} after "
            f"{args.rounds} fixed rounds; longest qualifying hold "
            f"{evaluation['longest_qualifying_duration_s']:.1f}s, "
            f"max lift {evaluation['max_lift_m']:.3f}m"
        )
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        report['last_responses'] = {rid: {'response': planner.last_response,
            'usage': planner.completer.last_usage} for rid, planner in planners.items()}
        for rid in ('r1', 'r3'):
            world.controllers[rid].set_motor_commands(production.STOP)
        world.frame_callback = None
        cleanup_errors = []
        if video is not None:
            try:
                video.close()
            except Exception as exc:
                cleanup_errors.append(f'video.close: {type(exc).__name__}: {exc}')
        try:
            world.close()
        except Exception as exc:
            cleanup_errors.append(f'world.close: {type(exc).__name__}: {exc}')
        if cleanup_errors:
            report['cleanup_errors'] = cleanup_errors
            if report['error'] is None:
                report['error'] = '; '.join(cleanup_errors)
        report['files'] = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in out.rglob('*') if p.is_file()}
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in {'files', 'calls'}}))
    return int(report['error'] is not None)


if __name__ == '__main__':
    raise SystemExit(main())
