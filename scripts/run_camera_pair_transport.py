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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=8)
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--model', default='gemini-3.8-flash')
    parser.add_argument('--timeout', type=float, default=45)
    args = parser.parse_args()
    if not 1 <= args.rounds <= 30:
        parser.error('rounds must be 1..30 (two bounded requests per round)')
    out = args.out_dir.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)

    import mujoco
    from harness.camera_pair_policy import CameraPairPlanner
    from harness.gemini_proxy import GeminiProxyCompleter
    from sim.camera_robot_port import CameraRobotPort
    import sim.multi_masterpi_production as production
    from scripts.probe_dual_grasp_sync import (
        _plain_beam_xml, _pose_metrics, _git, Video,
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
              'transport_success': False, 'success_claim': 'input boundary validation only'}
    report['config']['out_dir'] = str(out)
    video = None
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
        planners = {}
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
                reasoning_effort='none', http_open=audited_open))
        video = Video(world, out / 'motion.mp4', 12)
        world.frame_callback = video.capture
        video.capture(force=True)
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
                for port in ports.values():
                    port.tick(float(world.data.time))
                    port.stop()
                referee.write(json.dumps(_pose_metrics(world)) + '\n')
                referee.flush()
                report['rounds_completed'] += 1
                (out / 'progress.json').write_text(json.dumps(report, indent=2))
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        for rid in ('r1', 'r3'):
            world.controllers[rid].set_motor_commands(production.STOP)
        world.frame_callback = None
        if video is not None:
            video.close()
        world.close()
        report['files'] = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in out.rglob('*') if p.is_file()}
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in {'files', 'calls'}}))
    return int(report['error'] is not None)


if __name__ == '__main__':
    raise SystemExit(main())
