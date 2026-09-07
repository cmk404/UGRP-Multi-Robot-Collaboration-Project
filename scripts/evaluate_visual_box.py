"""Run the isolated single-box RGB skill and score physics only offline."""
from pathlib import Path
import argparse
import base64
import hashlib
import json
import math

import mujoco

from harness.visual_box_skill import VisualBoxSkill
from harness.visual_box_evaluation import evaluate_visual_box_transfer
from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def source_hash():
    root = Path(__file__).resolve().parents[1]
    digest = hashlib.sha256()
    for name in ('scripts/evaluate_visual_box.py', 'harness/visual_box_skill.py',
                 'harness/monocular_box.py', 'harness/visual_arm.py',
                 'harness/visual_floor.py', 'harness/visual_box_surface.py', 'harness/visual_attachment.py',
                 'harness/visual_box_evaluation.py', 'sim/camera_robot_port.py',
                 'sim/multi_masterpi_production.py', 'sim/warehouse_arena.py',
                 'sim/masterpi_production_v2.py', 'sim/masterpi_scene_v2.xml',
                 'sim/masterpi_dynamics_calibration.json'):
        path = root / name
        if path.exists():
            digest.update(name.encode()); digest.update(path.read_bytes())
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--seed', type=int, default=41)
    parser.add_argument('--steps', type=int, default=340)
    parser.add_argument('--noslip-iterations', type=int, default=0)
    parser.add_argument('--impratio', type=float, default=1.)
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--negative-control-open-gripper', action='store_true')
    parser.add_argument('--task', choices=['short_transfer', 'destination_zone'], default='short_transfer')
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    cohort_hash = source_hash()
    world = MultiMasterPiProductionV2(warehouse_layout='arena', seed=args.seed, render=True,
                                    warehouse_cargo_ids=('small_box_01',))
    if not 0 <= args.noslip_iterations <= 100:
        raise ValueError('INVALID_NOSLIP_ITERATIONS')
    world.model.opt.noslip_iterations = args.noslip_iterations
    if not math.isfinite(args.impratio) or not 1 <= args.impratio <= 100:
        raise ValueError('INVALID_IMPRATIO')
    world.model.opt.impratio = args.impratio
    # Physical floor paint is part of the camera scene, not a position HUD.
    for zone in 'abc':
        geom = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, 'warehouse_zone_' + zone)
        world.model.geom_group[geom] = 0
    port = CameraRobotPort(world, 'r1')
    # A goal name is operator input. No world/port object enters the controller.
    skill = VisualBoxSkill(task=args.task, destination_zone=world.warehouse_arena.destination_zone)
    video = None
    initial = world.warehouse_state()
    snapshots = []
    reason = 'STEP_BUDGET'
    logs = (out / 'control.jsonl').open('w')
    truth = (out / 'evaluation-only.jsonl').open('w')
    commands = (out / 'commands.jsonl').open('w')

    def step(seconds):
        until = float(world.data.time) + seconds
        while world.data.time < until:
            port.tick(float(world.data.time))
            world._physics_step_for(world.robot('r1'))
            if video and world.data.time >= video.next_frame:
                video.capture()

    def apply(action, obs):
        now = float(world.data.time)
        requested = dict(action)
        if args.negative_control_open_gripper and action.get('servo_id') == 1:
            action = {**action, 'pulse': 2000}
        commands.write(json.dumps({'time': now, 'source_frame_sha256': obs['sha256'], 'requested_action': requested, 'action': action}) + '\n')
        commands.flush()
        port.apply(action, now)

    def execute(action, obs):
        if action['kind'] == 'drive':
            apply({'kind': 'drive', 'forward': action['fwd'], 'turn': action['turn'], 'duration_s': action['duration']}, obs)
            step(action['duration'] + .2)
            port.stop()
        elif action['kind'] == 'pose':
            targets = action['pulses']
            start = obs['actuator_state']['servo_pulses']
            delta = max(abs(pulse - start[str(servo)]) for servo, pulse in targets.items())
            duration = max(.25, delta / 600.)
            count = max(5, math.ceil(duration / .05))
            for sample in range(1, count + 1):
                u = sample / count
                ease = u * u * (3 - 2 * u)
                for servo, end in targets.items():
                    pulse = round(start[str(servo)] + ease * (end - start[str(servo)]))
                    raw = {'kind': 'look', 'pan_pulse': pulse} if servo == 6 else {'kind': 'arm', 'servo_id': servo, 'pulse': pulse}
                    apply(raw, obs)
                step(duration / count)
            step(.3 if skill.phase == 'approach' else .08)
        elif action['kind'] == 'wait':
            apply({'kind': 'wait'}, obs)
            step(max(.05, action['duration']))
        else:
            raise ValueError('UNKNOWN_SKILL_MACRO')

    try:
        if args.record:
            from scripts.record_visual_box import SingleBoxVideo
            video = SingleBoxVideo(world, out / 'motion-1x.mp4', skill)
            video.capture(force=True)
        for index in range(args.steps):
            obs = port.capture()
            (out / 'inputs' / f'{index:04d}.jpg').write_bytes(base64.b64decode(obs['image']))
            phase_before = skill.phase
            # The controller call receives serialized sensor data only.
            action = skill.decide(obs)
            logs.write(json.dumps({'step': index, 'phase': phase_before, 'time': obs['sim_time'],
                'frame_sha256': obs['sha256'], 'own_pose_commands': obs['actuator_state']['servo_pulses'],
                'box': skill.last_box, 'surface': skill.last_surface, 'attachment': skill.last_attachment, 'estimated_target': skill.last_target, 'action': action}) + '\n')
            logs.flush()
            # Diagnostics below run after decision and are never passed back.
            snapshot = world.warehouse_state()
            snapshots.append(snapshot)
            truth.write(json.dumps({'step': index, 'phase': phase_before, 'state': snapshot,
                'grip_site': world.robot('r1').site_xyz('grip_site').tolist(),
                'base_xyz': world.robot('r1').base_xyz().tolist()}) + '\n')
            truth.flush()
            if index % 10 == 0 or phase_before != 'approach':
                print(index, phase_before, skill.last_target, flush=True)
            if action['kind'] == 'finish':
                reason = action['reason']
                break
            execute(action, obs)
    except Exception as exc:
        reason = type(exc).__name__ + ':' + str(exc)
        print(reason, flush=True)
    finally:
        port.stop()
        step(.5)
        final = world.warehouse_state()
        score = evaluate_visual_box_transfer(initial, final, snapshots, cargo_id='small_box_01', mode=args.task)
        result = {'reason': reason, 'phase': skill.phase, 'visual_lift_confirmed': skill.held,
            'initial': initial, 'final': final, 'seed': args.seed, 'task': args.task,
            'offline_score': score, 'source_hash': cohort_hash,
            'controller': 'deterministic_RGB_execution_skill', 'active_robots': ['r1'],
            'fixture': 'isolated_single_box', 'cargo_ids': ['small_box_01'],
            'llm_calls': 0, 'communication_experiment': False,
            'negative_control_open_gripper': args.negative_control_open_gripper,
            'physics_noslip_iterations': args.noslip_iterations, 'physics_impratio': args.impratio}
        (out / 'result.json').write_text(json.dumps(result, indent=2))
        logs.close(); truth.close(); commands.close()
        if video:
            video.close()
        world.close()
    print(reason, flush=True)


if __name__ == '__main__':
    main()
