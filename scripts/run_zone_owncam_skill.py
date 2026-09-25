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
from harness import wrist_zone_skill_v2 as v2  # noqa: E402
from harness import wrist_zone_skill_v3 as v3  # noqa: E402

SCHEMA = 'ugrp.zone_owncam_skill_run.v1'
POSE_SOURCE = 'gt_stub_eval_only'
VARIANT = 'zone_wide_door'
CONTACT_PROFILE = 'local_contact_fine'
SIM_LIMIT_S = 300.
STEP_LIMIT = 900
# v2 (pre-registered in the experiment README before its first recorded run)
V2_SIM_LIMIT_S = 420.
V2_STEP_LIMIT = 1300
# v3 (pre-registered in the experiment README before its first recorded run): same limits as v2
V3_SIM_LIMIT_S = 420.
V3_STEP_LIMIT = 1300
PROFILES = {'v1': PROFILE, 'v2': v2.PROFILE, 'v3': v3.PROFILE}
# Contact profiles selectable per run; always recorded. local_contact_fine is the zone default;
# cargo_noslip_v1 (sim/zone_cargo_contact.py) is opt-in only and recorded with its hash.
CONTACT_PROFILES = ('local_contact_fine', 'cargo_noslip_v1')
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
    # v2 development scenarios (tuning allowed, labelled dev, never reported as v2 results)
    403: {'start': (2.55, -2.60, 0.00), 'pickup_xy': (3.35, -2.55), 'slot': 'B3'},
    404: {'start': (2.55, 0.95, 0.05), 'pickup_xy': (3.45, 0.95), 'slot': 'A2'},
    405: {'start': (3.00, 1.00, 0.10), 'pickup_xy': (3.75, 0.40), 'slot': 'B1'},
    406: {'start': (2.50, -2.00, -0.10), 'pickup_xy': (3.65, -1.95), 'slot': 'C2'},
    # v2 pre-registered test scenarios (fixed before the first v2 recorded run)
    511: {'start': (2.55, -2.70, 0.00), 'pickup_xy': (3.25, -2.70), 'slot': 'B3'},
    512: {'start': (2.60, 1.10, 0.05), 'pickup_xy': (3.35, 1.10), 'slot': 'A1'},
    513: {'start': (3.30, -1.40, -0.05), 'pickup_xy': (3.95, -1.40), 'slot': 'A3'},
    514: {'start': (2.55, -1.95, 0.10), 'pickup_xy': (3.60, -2.30), 'slot': 'C2'},
    515: {'start': (3.10, 0.70, 0.00), 'pickup_xy': (3.85, 0.95), 'slot': 'B1'},
    516: {'start': (2.50, -0.20, -0.10), 'pickup_xy': (3.55, -0.10), 'slot': 'C1'},
    517: {'start': (3.50, -2.80, 0.00), 'pickup_xy': (4.00, -2.90), 'slot': 'A2'},
    518: {'start': (2.45, 0.50, 0.15), 'pickup_xy': (3.30, 0.30), 'slot': 'B2'},
    519: {'start': (3.60, 1.20, -0.10), 'pickup_xy': (4.05, 1.15), 'slot': 'C3'},
    520: {'start': (2.70, -1.00, 0.00), 'pickup_xy': (3.60, -0.70), 'slot': 'B3'},
    # v3 development scenario (tuning allowed, labelled dev); v3 also replays 518/519 as dev
    407: {'start': (2.55, 0.80, 0.05), 'pickup_xy': (3.40, 0.60), 'slot': 'B1'},
    # v3 pre-registered test scenarios (fixed before the first v3 recorded run)
    521: {'start': (2.55, -2.50, 0.00), 'pickup_xy': (3.40, -2.50), 'slot': 'B2'},
    522: {'start': (2.60, 1.00, 0.00), 'pickup_xy': (3.50, 1.20), 'slot': 'B3'},
    523: {'start': (3.50, -2.60, 0.10), 'pickup_xy': (4.00, -0.90), 'slot': 'C1'},
    524: {'start': (2.50, -1.70, 0.00), 'pickup_xy': (3.60, -1.90), 'slot': 'A3'},
    525: {'start': (3.20, 1.20, -0.10), 'pickup_xy': (3.90, 1.00), 'slot': 'C2'},
    526: {'start': (2.45, 0.10, 0.05), 'pickup_xy': (3.55, 0.05), 'slot': 'B1'},
    527: {'start': (3.60, -2.85, 0.00), 'pickup_xy': (4.10, -2.90), 'slot': 'A1'},
    528: {'start': (2.70, -0.20, -0.05), 'pickup_xy': (3.70, -0.40), 'slot': 'A2'},
    529: {'start': (3.00, -2.85, 0.10), 'pickup_xy': (3.80, -2.70), 'slot': 'C3'},
    530: {'start': (2.60, 0.70, 0.10), 'pickup_xy': (3.45, 0.55), 'slot': 'B2'},
}
V2_DEV_SEEDS = (403, 404, 405, 406)
V2_TEST_SEEDS = tuple(range(511, 521))
V3_DEV_SEEDS = (407, 518, 519)
V3_TEST_SEEDS = tuple(range(521, 531))


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


def build(seed, contact_profile=CONTACT_PROFILE):
    import mujoco
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_arena import episode
    from sim.zone_cargo_contact import CARGO_PROFILES, apply, base_profile
    from sim.zone_scene import ZoneScene
    scenario = SCENARIOS[seed]
    config = episode(VARIANT, seed, goal={'B': {'cyan': 1}})
    if contact_profile not in CONTACT_PROFILES:
        raise ValueError(f'contact profile: choose {CONTACT_PROFILES}')
    config['contact_solver_profile'] = base_profile(contact_profile)
    definition = ZoneScene.from_zone_config(config)
    transform = definition.transform
    xml_transform = ((lambda xml: apply(transform(xml), contact_profile)) if contact_profile in CARGO_PROFILES
                     else transform)
    world = MultiMasterPiProductionV2(seed=seed, width=640, height=480, render=True,
                                      warehouse_layout=definition.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=xml_transform)
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


def _cargo_profile_record(name):
    from sim.zone_cargo_contact import CARGO_PROFILES, profile_record
    return profile_record(name) if name in CARGO_PROFILES else None


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
    parser.add_argument('--profile', choices=sorted(PROFILES), default='v1')
    parser.add_argument('--contact-profile', choices=CONTACT_PROFILES, default=CONTACT_PROFILE,
                        help='explicit, recorded contact profile (default: the zone local_contact_fine)')
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
    world, definition, config, box_body, order = build(args.seed, args.contact_profile)
    (out / 'scene.xml').write_text(world.scene_xml)
    port = CameraRobotPort(world, 'r1', allow_reverse=True, allow_mecanum=True)
    pose_source = GtStubPoseSource(world, 'r1')
    delivery = {'v1': WristZoneDelivery, 'v2': v2.WristZoneDeliveryV2, 'v3': v3.WristZoneDeliveryV3}[args.profile]
    skill = delivery(order, planner=make_planner(config['static_map'], order))
    sim_limit, step_limit = {'v1': (SIM_LIMIT_S, STEP_LIMIT), 'v2': (V2_SIM_LIMIT_S, V2_STEP_LIMIT),
                             'v3': (V3_SIM_LIMIT_S, V3_STEP_LIMIT)}[args.profile]
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
        for index in range(step_limit):
            if float(world.data.time) > sim_limit:
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
        'schema': SCHEMA, 'profile': PROFILES[args.profile], 'seed': args.seed, 'scenario': SCENARIOS[args.seed],
        'sim_limit_s': sim_limit, 'skill_summary': skill.summary() if hasattr(skill, 'summary') else None,
        'variant': VARIANT, 'contact_solver_profile': config['contact_solver_profile'],
        'contact_profile_selected': args.contact_profile,
        'cargo_contact_profile': _cargo_profile_record(args.contact_profile),
        'solver_noslip_iterations': int(world.model.opt.noslip_iterations), 'weld': 'off',
        'pose_source': POSE_SOURCE, 'pose_sources_seen': sorted(skill.pose_sources),
        'counts_as_m1': False,
        'development_seed': args.seed in DEV_SEEDS + V2_DEV_SEEDS or (args.profile == 'v3' and args.seed not in V3_TEST_SEEDS),
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
