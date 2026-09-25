"""Skill-isolation runner for ``wrist_zone_skill_v1`` on zone_wide_door (open east section).

The controller receives only the r1 ``robot_cam`` observation (own wrist RGB
+ own issued PWM) and a pose estimate. In this runner the pose estimate is a
STUB read from simulator truth and labelled ``pose_source=gt_stub_eval_only``:
it isolates the grasp/carry/place skill from localisation (owned by the
``claude/zone-owncam-loc`` branch). Results of this runner are NEVER M1
successes. Truth is also used for the separate evaluation-only log (lift,
slot, weld, contacts). No nav_cam and no TOP image is captured.

Pre-registered scenarios (fixed before the first recorded run): the robot and
one cyan zone box are placed east of the divider, so no door is crossed.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.wrist_zone_skill import PROFILE, OrderSheet, PoseEstimate, WristZoneDelivery  # noqa: E402

SCHEMA = 'ugrp.zone_owncam_skill_run.v1'
POSE_SOURCE = 'gt_stub_eval_only'
VARIANT = 'zone_wide_door'
CONTACT_PROFILE = 'local_contact_fine'
SIM_LIMIT_S = 300.
STEP_LIMIT = 900
# seed -> start pose (x, y, yaw) of r1, staging cell of the cyan box (order sheet), slot id.
# 401-402 are DEVELOPMENT scenarios (tuning allowed, never reported as results);
# 501-505 are the pre-registered test scenarios.
DEV_SEEDS = (401, 402)
SCENARIOS = {
    401: {'start': (2.60, -1.80, 0.00), 'pickup_xy': (3.50, -1.80), 'slot': 'C1'},
    402: {'start': (3.60, 0.00, 0.00), 'pickup_xy': (4.00, -0.60), 'slot': 'A2'},
    501: {'start': (2.55, -2.60, 0.00), 'pickup_xy': (3.30, -2.60), 'slot': 'B1'},
    502: {'start': (2.55, 0.90, 0.10), 'pickup_xy': (3.40, 0.90), 'slot': 'A3'},
    503: {'start': (3.40, -1.00, -0.10), 'pickup_xy': (3.90, -0.85), 'slot': 'A1'},
    504: {'start': (2.50, -2.00, -0.15), 'pickup_xy': (3.70, -2.00), 'slot': 'C3'},
    505: {'start': (3.00, 1.00, 0.20), 'pickup_xy': (3.80, 0.30), 'slot': 'B2'},
}


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class GtStubPoseSource:
    """Simulator-truth pose, ONLY for skill isolation. Never an M1 input."""
    label = POSE_SOURCE

    def __init__(self, world, rid):
        self._world, self._rid = world, rid

    def estimate(self):
        robot = self._world.robot(self._rid)
        xyz = robot.base_xyz()
        return PoseEstimate(float(xyz[0]), float(xyz[1]), float(robot.base_rpy()[2]), self.label)


def build(seed):
    import mujoco
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_arena import episode
    from sim.zone_scene import ZoneScene
    scenario = SCENARIOS[seed]
    config = episode(VARIANT, seed, goal={'B': {'cyan': 1}})
    config['contact_solver_profile'] = CONTACT_PROFILE
    definition = ZoneScene.from_zone_config(config)
    world = MultiMasterPiProductionV2(seed=seed, width=640, height=480, render=True,
                                      warehouse_layout=definition.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=definition.transform)
    definition.setup(world)
    # Setup-only scenario override (recorded): the one cyan box and r1 go east of the divider.
    (oid, item), = config['setup_only']['objects'].items()
    jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, item['joint_name'])
    q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
    world.data.qpos[q:q + 7] = [*scenario['pickup_xy'], .016, 1, 0, 0, 0]
    world.data.qvel[v:v + 6] = 0
    sx, sy, syaw = scenario['start']
    world.robot('r1').set_base_pose_for_test((sx, sy, .032355118817659255), syaw)
    mujoco.mj_forward(world.model, world.data)
    static = config['static_map']
    slot_xy = next(s['center_m'] for slots in static['zone_slots'].values() for s in slots
                   if s['slot_id'] == scenario['slot'])
    order = OrderSheet('cyan', tuple(scenario['pickup_xy']), scenario['slot'], tuple(slot_xy))
    return world, definition, config, item['body_name'], order


def make_planner(static, order):
    from harness.static_keepouts import keepout_rects
    from scripts.zone_teacher import CARRY_RADIUS_M, ROBOT_RADIUS_M, plan_path
    rects = keepout_rects(static)

    def planner(start, goal, carrying):
        # Static map + the order-sheet pickup cell; no live object pose.
        discs = [] if carrying else [(order.pickup_xy_m[0], order.pickup_xy_m[1], .06)]
        path = plan_path(start, goal, static['bounds_m'], discs,
                         radius=CARRY_RADIUS_M if carrying else ROBOT_RADIUS_M, rects=rects)
        return path
    return planner


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, choices=sorted(SCENARIOS), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-dirty', action='store_true', help='development only; recorded as dirty')
    args = parser.parse_args()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip())
    if dirty and not args.allow_dirty:
        raise SystemExit('commit and freeze the source before a recorded run (or --allow-dirty)')
    import mujoco
    import numpy as np
    from sim.camera_robot_port import CameraRobotPort
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    started = time.monotonic()
    load_start = [round(v, 2) for v in os.getloadavg()]
    world, definition, config, box_body, order = build(args.seed)
    (out / 'scene.xml').write_text(world.scene_xml)
    port = CameraRobotPort(world, 'r1', allow_reverse=True, allow_mecanum=True)
    pose_source = GtStubPoseSource(world, 'r1')
    skill = WristZoneDelivery(order, planner=make_planner(config['static_map'], order))
    control = (out / 'control.jsonl').open('w')
    truth = (out / 'evaluation-only.jsonl').open('w')
    geoms = {i: mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '' for i in range(world.model.ngeom)}
    wall_ids = {i for i, n in geoms.items() if n.startswith('zone_wall')}
    r1_ids = {i for i, n in geoms.items() if n.startswith('r1__')}
    stats = {'wall_contact_steps': 0, 'max_eq_active': 0, 'max_box_z_m': 0.}

    def step(seconds):
        dt = float(world.model.opt.timestep)
        for _ in range(max(1, round(seconds / dt))):
            port.tick(float(world.data.time))
            world._physics_step_for(world.robot('r1'))
        d = world.data
        for c in d.contact[:d.ncon]:
            if (c.geom1 in wall_ids and c.geom2 in r1_ids) or (c.geom2 in wall_ids and c.geom1 in r1_ids):
                stats['wall_contact_steps'] += 1
                break
        stats['max_eq_active'] = max(stats['max_eq_active'], int(d.eq_active.max()) if len(d.eq_active) else 0)
        stats['max_box_z_m'] = max(stats['max_box_z_m'], float(d.body(box_body).xpos[2]))

    def apply(raw):
        port.apply(raw, float(world.data.time))

    def execute(action, obs):
        kind = action['kind']
        if kind == 'drive':
            apply({'kind': 'drive', 'forward': action['fwd'], 'turn': action['turn'], 'duration_s': action['duration']})
            step(action['duration'] + .2)
            port.stop()
        elif kind == 'mecanum':
            apply({'kind': 'mecanum', 'forward': action['forward'], 'left': action['left'], 'turn': action['turn'],
                   'duration_s': action['duration']})
            step(action['duration'] + .1)
            port.stop()
        elif kind == 'pose':
            targets = action['pulses']
            start = obs['actuator_state']['servo_pulses']
            delta = max(abs(p - start[str(s)]) for s, p in targets.items())
            duration = max(.25, delta / 600.)
            count = max(5, math.ceil(duration / .05))
            for sample in range(1, count + 1):
                u = sample / count
                ease = u * u * (3 - 2 * u)
                for servo, end in targets.items():
                    pulse = round(start[str(servo)] + ease * (end - start[str(servo)]))
                    apply({'kind': 'look', 'pan_pulse': pulse} if servo == 6 else
                          {'kind': 'arm', 'servo_id': servo, 'pulse': pulse})
                step(duration / count)
            step(.3 if skill.phase in ('grasp',) and skill.box.phase == 'approach' else .15)
        elif kind == 'wait':
            apply({'kind': 'wait'})
            step(max(.05, action['duration']))
        else:
            raise ValueError('UNKNOWN_MACRO')

    step(.5)
    reason, index = 'STEP_LIMIT', 0
    phase_times = {}
    try:
        for index in range(STEP_LIMIT):
            if float(world.data.time) > SIM_LIMIT_S:
                reason = 'SIM_LIMIT'
                break
            obs = port.capture()          # robot_cam only
            estimate = pose_source.estimate()
            (out / 'inputs' / f'{index:04d}.jpg').write_bytes(base64.b64decode(obs['image']))
            phase = skill.phase
            box_phase = skill.box.phase
            phase_times.setdefault(phase, round(float(world.data.time), 3))
            action = skill.decide(obs, estimate)
            control.write(json.dumps({'step': index, 'phase': phase, 'box_skill_phase': box_phase,
                                      'sim_time': obs['sim_time'], 'frame_sha256': obs['sha256'],
                                      'camera': obs['camera'], 'own_pose_commands': obs['actuator_state']['servo_pulses'],
                                      'pose_estimate': [estimate.x_m, estimate.y_m, estimate.yaw_rad],
                                      'pose_source': estimate.source, 'nav': skill.last_nav,
                                      'box': skill.box.last_box, 'attachment': skill.box.last_attachment,
                                      'action': action}) + '\n')
            # Evaluation-only truth, written after the decision, never fed back.
            gt = world.data.body(box_body)
            xyz = [round(float(v), 4) for v in gt.xpos]
            truth.write(json.dumps({'step': index, 'phase': phase, 'box_xyz': xyz,
                                    'box_quat': [round(float(v), 4) for v in gt.xquat],
                                    'base_xyz': [round(float(v), 4) for v in world.robot('r1').base_xyz()],
                                    'base_yaw': round(float(world.robot('r1').base_rpy()[2]), 4)}) + '\n')
            if action['kind'] == 'finish':
                reason = action['reason']
                break
            execute(action, obs)
    finally:
        control.close()
        truth.close()
    # ---------------- evaluation (truth, separate) ----------------
    d = world.data
    box = d.body(box_body)
    bx, by, bz = (float(v) for v in box.xpos)
    qw, qx, qy, qz = (float(v) for v in box.xquat)
    tilt = math.degrees(math.acos(max(-1., min(1., 1 - 2 * (qx * qx + qy * qy)))))
    sx, sy = order.slot_xy_m
    zone = order.slot_id[0]
    region = config['static_map']['regions']['zone_' + zone]
    (zx, zy), (hx, hy) = region['center_m'], region['half_extents_m']
    events = {e['event']: e for e in skill.events}
    lifted = stats['max_box_z_m'] > .045
    on_floor = bz < .03 and tilt < 15.
    in_slot = abs(bx - sx) <= .06 and abs(by - sy) <= .06
    in_zone = abs(bx - zx) <= hx and abs(by - zy) <= hy
    evaluation = {
        'grasp_success_gt': bool('grasp_attached' in events and lifted),
        'place_in_slot_gt': bool(in_slot and on_floor),
        'place_in_zone_gt': bool(in_zone and on_floor),
        'box_final_xyz': [round(bx, 4), round(by, 4), round(bz, 4)], 'box_tilt_deg': round(tilt, 2),
        'slot_error_m': [round(bx - sx, 4), round(by - sy, 4)],
        'max_box_z_m': round(stats['max_box_z_m'], 4), 'weld_eq_active_max': stats['max_eq_active'],
        'r1_wall_contact_steps': stats['wall_contact_steps'],
        'skill_claim_in_slot': reason == 'OWN_RGB_PLACEMENT_IN_SLOT',
        'skill_claim_agrees_with_gt': (reason == 'OWN_RGB_PLACEMENT_IN_SLOT') == bool(in_slot and on_floor),
    }
    result = {
        'schema': SCHEMA, 'profile': PROFILE, 'seed': args.seed, 'scenario': SCENARIOS[args.seed],
        'variant': VARIANT, 'contact_solver_profile': CONTACT_PROFILE, 'weld': 'off',
        'pose_source': POSE_SOURCE, 'pose_sources_seen': sorted(skill.pose_sources),
        'counts_as_m1': False, 'development_seed': args.seed in DEV_SEEDS,
        'claim_scope': ('skill isolation on the open east section of zone_wide_door with a GT pose stub; '
                        'not M1, not own-camera localisation, no door crossing'),
        'controller_inputs': 'robot_cam JPEG + own issued PWM + pose estimate (gt_stub_eval_only) + static map + order sheet',
        'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'dirty_source': dirty, 'order': order.__dict__ | {'pickup_xy_m': list(order.pickup_xy_m), 'slot_xy_m': list(order.slot_xy_m)},
        'reason': reason, 'steps': index + 1, 'sim_seconds': round(float(world.data.time), 3),
        'phase_start_sim_s': phase_times, 'events': skill.events, 'placement_own_rgb': skill.placement,
        'evaluation_only': evaluation, 'scene': {k: v for k, v in definition.manifest.items() if k != 'box_replicas'},
        'scene_xml_sha256': hashlib.sha256(world.scene_xml.encode()).hexdigest(),
        'wall_seconds': round(time.monotonic() - started, 1), 'load_average_start': load_start,
        'load_average_end': [round(v, 2) for v in os.getloadavg()]}
    (out / 'result.json').write_text(json.dumps(result, indent=1, default=str) + '\n')
    result['artifact_sha256'] = {n: sha_file(out / n) for n in ('result.json', 'control.jsonl',
                                                                'evaluation-only.jsonl', 'scene.xml')}
    (out / 'hashes.json').write_text(json.dumps(result['artifact_sha256'], indent=1) + '\n')
    print(json.dumps({'seed': args.seed, 'reason': reason, 'sim_s': result['sim_seconds'], **evaluation}))


if __name__ == '__main__':
    main()
