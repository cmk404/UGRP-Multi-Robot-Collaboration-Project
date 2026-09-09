#!/usr/bin/env python3
"""Two independent image-only actors; physical truth is an output-only referee."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


from scripts.run_camera_pair_transport import evaluate_grasp_samples


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=8)
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--model', default='gemini-3.8-flash')
    parser.add_argument('--timeout', type=float, default=45)
    parser.add_argument('--task', choices=('carry', 'grasp'), default='grasp')
    parser.add_argument('--active-robot', choices=('r1', 'r3', 'both'), default='r1')
    args = parser.parse_args()
    if not 1 <= args.rounds <= 60:
        parser.error('rounds must be 1..60 (at most 120 bounded requests)')
    if args.task != 'grasp':
        parser.error('visual controller implements grasp only')
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)

    import mujoco
    from harness.camera_visual_observer import CameraVisualObserver
    from harness.camera_grasp_controller import CameraGraspController
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
    started = time.monotonic()
    report = {'git_sha': _git(['rev-parse', 'HEAD']), 'config': vars(args).copy(),
              'input_contract': ('current/prior own+top RGB and own issued commands only; '
                                 'pixel landmarks feed state-local control; alternating isolated actions; no truth feedback'),
              'calls': [], 'rounds_completed': 0, 'error': None,
              'transport_success': None, 'grasp_success': None,
              'success_claim': 'grasp evaluation pending fixed round budget'}
    report['config']['out_dir'] = str(out)
    video = None
    planners = {}
    controls = {}
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
        ports = {rid: CameraRobotPort(world, rid, allow_reverse=True) for rid in ('r1', 'r3')}
        request_counts = {'r1': 0, 'r3': 0}
        for rid in ports:
            if args.active_robot != 'both' and rid != args.active_robot:
                continue
            (out / rid).mkdir()
            def audited_open(request, *, timeout, _rid=rid):
                request_counts[_rid] += 1
                # Exact HTTP request body, before sending, including original
                # image bytes. No headers, endpoint credentials or secrets.
                (out / _rid / f'wire-{request_counts[_rid]:03d}.json').write_bytes(request.data)
                return urlopen(request, timeout=timeout)
            controls[rid] = CameraGraspController()
            planners[rid] = CameraVisualObserver(rid, GeminiProxyCompleter(
                model=args.model, max_tokens=1200, timeout=args.timeout,
                reasoning_effort='none', http_open=audited_open))
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
                    futures[rid] = pool.submit(planner.observe, own, overhead, controls[rid].history)
                observations = {rid: future.result() for rid, future in futures.items()}
                active = args.active_robot if args.active_robot != 'both' else ('r1', 'r3')[index % 2]
                actions = {rid: controls[rid].step(obs, active=(rid == active)) for rid, obs in observations.items()}
                now = float(world.data.time)
                for rid, action in actions.items():
                    ports[rid].apply(action, now)
                    report['calls'].append({'round': index, 'robot_id': rid,
                        'action': action, 'active': rid == active, 'observation': observations[rid],
                        'decision': controls[rid].last_decision, 'response': planners[rid].last_response,
                        'usage': planners[rid].completer.last_usage,
                        'reported_model': planners[rid].completer.last_model,
                        'model_latency_ms': planners[rid].completer.last_latency_ms,
                        'learning': controls[rid].model.summary()})
                video.stage = f'visual_servo_{index + 1}_{active}'
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
        report['wall_elapsed_s'] = round(time.monotonic() - started, 3)
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
