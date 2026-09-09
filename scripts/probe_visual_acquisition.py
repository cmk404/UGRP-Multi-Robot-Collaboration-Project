"""No-LLM acquisition regression in the camera-team physical fixture.

This explicitly scripted approach/pick test is not a transport-policy success.
The skill receives own RGB/PWM only; simulation state is logged after decisions.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path

import mujoco

from harness.llm_transport_skill import LLMTransportSkill
from harness.visual_drive_guard import validate_visual_drive
from harness.visual_macro_runtime import VisualMacroExecutor
from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--initial-delay', type=float, default=1.54)
    ap.add_argument('--seconds', type=float, default=180)
    ap.add_argument('--record', action='store_true')
    args = ap.parse_args()
    if not 0 <= args.initial_delay <= 10 or not 0 < args.seconds <= 300:
        ap.error('invalid bounded delay or duration')
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    world = MultiMasterPiProductionV2(warehouse_layout='camera_team', seed=args.seed,
                                    render=True, warehouse_cargo_ids=('small_box_01',))
    world.model.opt.impratio = 10
    world.model.opt.noslip_iterations = 3
    for zone in 'abc':
        gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM,
                              'warehouse_zone_' + zone)
        world.model.geom_group[gid] = 0
    port = CameraRobotPort(world, 'r1')
    actor = LLMTransportSkill('r1', 'small_box_01', 'B')  # unused named goal
    log = (out / 'control.jsonl').open('w')
    commands = (out / 'commands.jsonl').open('w')
    def emit(row):
        commands.write(json.dumps(row) + '\n')
    executor = VisualMacroExecutor(port, log_callback=emit,
        drive_guard=lambda action, now: validate_visual_drive(port.capture(camera='nav_cam'), action))
    video = None
    error = None
    count = 0
    picked = False
    start = float(world.data.time)
    source = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
              for base in ('harness', 'sim', 'calibration') for p in sorted(Path(base).rglob('*'))
              if p.is_file() and p.suffix in ('.py', '.json', '.xml', '.png')}
    (out / 'source-manifest.json').write_text(json.dumps(source, indent=2))
    try:
        if args.record:
            from scripts.record_visual_box import SingleBoxVideo
            video = SingleBoxVideo(world, out / 'motion-1x.mp4', actor.box)
            video.capture(force=True)
        while float(world.data.time) - start <= args.seconds:
            now = float(world.data.time)
            executor.tick(now)
            if executor.idle and now - start >= args.initial_delay:
                if actor.state == 'idle':
                    actor.request({'kind': 'approach'})
                wrist = port.capture()
                phase = actor.phase
                action = actor.advance(wrist)
                if actor.state == 'ready_to_pick' and not picked:
                    action = actor.request({'kind': 'pick'})
                    picked = True
                path = f'inputs/{count:04d}.jpg'
                (out / path).write_bytes(base64.b64decode(wrist['image']))
                log.write(json.dumps({'step': count, 'time': now, 'phase': phase,
                    'state': actor.state, 'path': path, 'sha256': wrist['sha256'],
                    'own_pose_commands': wrist['actuator_state']['servo_pulses'],
                    'target': actor.box.last_target, 'box': actor.box.last_box,
                    'surface': actor.box.last_surface, 'attachment': actor.box.last_attachment,
                    'action': action}) + '\n')
                log.flush()
                count += 1
                if actor.state in ('carrying', 'failure', 'grip_uncertain'):
                    break
                if action:
                    executor.submit(action, wrist, phase, now)
            world._physics_step_for(world.robot('r1'))
            if video and world.data.time >= video.next_frame:
                video.capture()
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        executor.cancel(float(world.data.time), 'acquisition_probe_complete')
        port.stop()
        result = {'fixture': 'camera_team_scripted_acquisition_only', 'llm_calls': 0,
            'transport_success_claimed': False, 'seed': args.seed,
            'initial_delay_s': args.initial_delay, 'state': actor.state,
            'reason': actor.reason, 'held': actor.held,
            'acquisition_passed': actor.state == 'carrying' and actor.held and error is None,
            'sim_seconds': float(world.data.time) - start, 'steps': count, 'error': error,
            'evaluation_only_final': world.warehouse_state()}
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        if video:
            video.capture(force=True)
            video.close()
        log.close()
        commands.close()
        world.close()
    print(json.dumps({k: v for k, v in result.items() if k != 'evaluation_only_final'}), flush=True)
    return 0 if result['acquisition_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
