#!/usr/bin/env python3
"""Runner v8 for ``wrist_zone_skill_v8`` (frame-relative approach cyan gate). DIAGNOSTIC.

A new runner (``zone_owncam_runner_v8``) so runner v7 stays the recorded source of
its cohort (561-572). Changes against scripts/run_zone_owncam_skill_v7.py:

* skill ``wrist_zone_skill_v8``;
* order-sheet bays W1-W4 on the light-blue PICKUP floor west of the divider
  (M1 dev-a6 failure floor), besides the east bays E1-E4. West episodes start in
  the pickup section and carry the box through door_1 to the east zones;
* peers may be parked next to the box as in v7 (static keep-outs only).

Diagnostic like v5-v7: GT pose stub, so ``m1_success`` is always False.
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
sys.path.insert(0, str(ROOT))

from harness import m1_contract  # noqa: E402
from harness import wrist_zone_skill_v5 as v5  # noqa: E402
from harness import wrist_zone_skill_v6 as v6  # noqa: E402
from harness import wrist_zone_skill_v7 as v7  # noqa: E402
from harness import wrist_zone_skill_v8 as v8  # noqa: E402
from harness.wrist_zone_skill import PoseEstimate  # noqa: E402

RUNNER = 'zone_owncam_runner_v8'
SCHEMA = 'ugrp.zone_owncam_skill.v8'
VARIANT = 'zone_wide_door'
GT_POSE_SOURCE = 'gt_stub_eval_only'
CONTACT_PROFILES = ('local_contact_fine', 'cargo_noslip_v1')
DEFAULT_CONTACT_PROFILE = 'cargo_noslip_v1'     # primary, pending the user's decision (PR 189 audit)
SIM_LIMIT_S, STEP_LIMIT = 420., 900
BAY_HALF_M = (.25, .25)
# Order-sheet bay table (static, east section of zone_wide_door). A bay is coarse on purpose.
BAYS = {'E1': (3.55, -2.35), 'E2': (3.70, -1.50), 'E3': (3.70, 0.35), 'E4': (3.60, 0.95),
        # west pickup floor (regions.pickup, light-blue paint), M1's pickup side
        'W1': (0.45, -2.10), 'W2': (0.60, -0.85), 'W3': (0.40, 0.55), 'W4': (1.20, -1.50)}
# Setup-only: box offset inside the bay and box yaw are evaluation values, never given to the skill.
SPAWN_KEEPOUT_RADIUS_M = .17       # a parked / idle MasterPi footprint
# ``parked``: robot -> (dx, dy) of its parking spot relative to the BOX centre (setup; given to the skill only
# as a static keep-out disc).
SCENARIOS = {
    # development (labelled dev, never cohort results)
    420: {'start': (-0.20, -0.85, 0.00), 'bay': 'W2', 'offset': (.05, .04), 'yaw_deg': 0., 'slot': 'B2'},
    421: {'start': (-0.30, -2.00, 0.10), 'bay': 'W1', 'offset': (-.06, .08), 'yaw_deg': -20., 'slot': 'A1'},
    422: {'start': (0.00, 0.40, -0.05), 'bay': 'W3', 'offset': (.08, -.05), 'yaw_deg': 2., 'slot': 'C2'},
    # test (pre-registered in experiments/2026-09-25-zone-owncam-skill/README.md before any v8 test run)
    581: {'start': (-0.25, -0.85, 0.00), 'bay': 'W2', 'offset': (.08, -.06), 'yaw_deg': 0., 'slot': 'B1'},
    582: {'start': (-0.20, -2.20, 0.05), 'bay': 'W1', 'offset': (-.05, .10), 'yaw_deg': 0., 'slot': 'A2'},
    583: {'start': (-0.10, 0.50, 0.00), 'bay': 'W3', 'offset': (.10, .06), 'yaw_deg': 2., 'slot': 'C1'},
    584: {'start': (0.30, -1.30, -0.10), 'bay': 'W4', 'offset': (-.08, -.08), 'yaw_deg': -2., 'slot': 'B3'},
    585: {'start': (-0.30, -0.70, 0.10), 'bay': 'W2', 'offset': (-.10, .12), 'yaw_deg': -20., 'slot': 'A3'},
    586: {'start': (-0.15, -1.90, 0.00), 'bay': 'W1', 'offset': (.12, -.04), 'yaw_deg': 15., 'slot': 'C3'},
    587: {'start': (-0.20, 0.30, 0.05), 'bay': 'W3', 'offset': (-.06, -.10), 'yaw_deg': 0., 'slot': 'B2',
          'parked': {'r2': (-.25, .30)}},
    588: {'start': (0.35, -1.70, 0.00), 'bay': 'W4', 'offset': (.05, .10), 'yaw_deg': 0., 'slot': 'A1'},
    # east regression checks
    589: {'start': (2.55, -2.30, 0.00), 'bay': 'E1', 'offset': (.06, .05), 'yaw_deg': 0., 'slot': 'B1'},
    590: {'start': (2.60, 0.30, 0.00), 'bay': 'E3', 'offset': (-.05, -.08), 'yaw_deg': -25., 'slot': 'C2'},
}
DEV_SEEDS = (420, 421, 422)
TEST_SEEDS = tuple(range(581, 591))
PARKING_IDS = {'r2': 'parking_P1', 'r3': 'parking_P2'}
DEPENDENCIES = (
    'scripts/run_zone_owncam_skill_v8.py', 'scripts/run_zone_owncam_skill_v7.py', 'scripts/run_zone_owncam_skill_v6.py', 'scripts/run_zone_owncam_skill_v5.py', 'harness/m1_contract.py', 'harness/wrist_zone_skill.py',
    'harness/wrist_zone_skill_v2.py', 'harness/wrist_zone_skill_v3.py', 'harness/wrist_zone_skill_v4.py',
    'harness/wrist_zone_skill_v5.py', 'harness/wrist_zone_skill_v6.py', 'harness/wrist_zone_skill_v7.py', 'harness/wrist_zone_skill_v8.py', 'harness/visual_box_skill.py', 'harness/visual_box_surface.py',
    'harness/markerless_box.py', 'harness/markerless_face.py', 'harness/approach_geometry.py',
    'harness/visual_arm.py', 'harness/zone_color_boxes.py', 'harness/static_keepouts.py', 'harness/owncam_view.py',
    'sim/camera_robot_port.py', 'sim/zone_arena.py', 'sim/zone_scene.py', 'sim/zone_cargo_contact.py',
    'sim/multi_masterpi_production.py', 'scripts/zone_teacher.py')


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def provenance():
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip())
    return {'head': head, 'dirty': dirty,
            'dependency_sha256': {p: sha_file(ROOT / p) for p in DEPENDENCIES if (ROOT / p).exists()},
            'missing_dependencies': [p for p in DEPENDENCIES if not (ROOT / p).exists()]}


class GtStubPoseSource:
    """Simulator-truth pose, ONLY for diagnostic skill isolation. Never an M1 input."""
    label = GT_POSE_SOURCE

    def __init__(self, world, rid):
        self._world, self._rid = world, rid

    def estimate(self):
        robot = self._world.robot(self._rid)
        xyz = robot.base_xyz()
        return PoseEstimate(float(xyz[0]), float(xyz[1]), float(robot.base_rpy()[2]), self.label)


def select_pose_source(mode, world, rid='r1'):
    m1_contract.check_mode(mode)
    source = GtStubPoseSource(world, rid)
    if mode == 'm1':
        # No own-camera pose estimator is wired into this runner yet (PR #178 interface pending).
        m1_contract.require_m1_pose_source(source.label, 'runner:pose_source')
    return source


def static_keepouts(seed):
    """Fixed peer spots from the static layout: every spawn row of the arena spec (not the per-episode
    robot assignment) and the scenario's parking spots (relative to the box; setup-defined, static)."""
    from sim.zone_arena import layout
    spec = layout(VARIANT)
    keepouts = [v6.StaticKeepout(f'spawn_row_{i}', (float(spec['spawn_x']), float(y)), SPAWN_KEEPOUT_RADIUS_M,
                                 'static_layout_idle_spawn') for i, y in enumerate(spec['spawn_rows_y'])]
    scenario = SCENARIOS[seed]
    cx, cy = BAYS[scenario['bay']]
    bx, by = cx + scenario['offset'][0], cy + scenario['offset'][1]
    for rid, (dx, dy) in sorted(scenario.get('parked', {}).items()):
        keepouts.append(v6.StaticKeepout(PARKING_IDS[rid], (round(bx + dx, 4), round(by + dy, 4)),
                                         SPAWN_KEEPOUT_RADIUS_M, 'static_map_parking'))
    return tuple(keepouts)


def order_for(seed, static_map):
    scenario = SCENARIOS[seed]
    slot_xy = next(s['center_m'] for slots in static_map['zone_slots'].values() for s in slots
                   if s['slot_id'] == scenario['slot'])
    return v5.CoarseOrderSheet('cyan', scenario['bay'], BAYS[scenario['bay']], BAY_HALF_M,
                               scenario['slot'], tuple(slot_xy))


def build(seed, contact_profile):
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
    (oid, item), = config['setup_only']['objects'].items()
    jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, item['joint_name'])
    q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
    cx, cy = BAYS[scenario['bay']]
    bx, by = cx + scenario['offset'][0], cy + scenario['offset'][1]
    half = math.radians(scenario['yaw_deg']) / 2
    world.data.qpos[q:q + 7] = [bx, by, .016, math.cos(half), 0, 0, math.sin(half)]
    world.data.qvel[v:v + 6] = 0
    sx, sy, syaw = scenario['start']
    world.robot('r1').set_base_pose_for_test((sx, sy, .032355118817659255), syaw)
    keepouts = static_keepouts(seed)
    spots = {k.keepout_id: k.xy_m for k in keepouts}
    for rid in sorted(scenario.get('parked', {})):
        px, py = spots[PARKING_IDS[rid]]
        world.robot(rid).set_base_pose_for_test((px, py, .032355118817659255), 0.)
    mujoco.mj_forward(world.model, world.data)
    setup_only = {'box_xy_m': [bx, by], 'box_yaw_deg': scenario['yaw_deg'], 'bay': scenario['bay'],
                  'offset_in_bay_m': list(scenario['offset']), 'robot_start': list(scenario['start']),
                  'parked_rel_box_m': scenario.get('parked', {}),
                  'peer_positions': {r: [round(float(v), 4) for v in world.robot(r).base_xyz()[:2]] for r in ('r2', 'r3')}}
    return (world, definition, config, item['body_name'], order_for(seed, config['static_map']), setup_only,
            tuple(keepouts))


def make_planner(static, order, keepouts=(), extra_discs=lambda: []):
    from harness.static_keepouts import keepout_rects
    from scripts.zone_teacher import CARRY_RADIUS_M, ROBOT_RADIUS_M, plan_path
    rects = keepout_rects(static)
    (cx, cy), (hx, hy) = order.pickup_bay_center_m, order.pickup_bay_half_m

    def planner(start, goal, carrying):
        # Static map + the coarse bay (kept clear while empty-handed); no live object pose.
        extra = [] if carrying else list(extra_discs())
        # re-planning around the own-RGB box: its footprint replaces the whole coarse bay
        discs = [] if carrying else (extra or [(cx, cy, max(hx, hy))])
        # static peer spots (spawn / parking), inflated by their footprint + the skill's keep-out margin
        discs += [(k.xy_m[0], k.xy_m[1], k.radius_m + v6.KEEPOUT_MARGIN_M) for k in keepouts]
        return plan_path(start, goal, static['bounds_m'], discs,
                         radius=CARRY_RADIUS_M if carrying else ROBOT_RADIUS_M, rects=rects)
    return planner


class PhysicsContactLog:
    """Evaluation-only contact classes on every physics step (never fed to the controller)."""

    CLASSES = ('cargo_wall', 'r1_wall', 'r1_cargo', 'finger_cargo', 'r1_peer')

    def __init__(self, world, box_body):
        import mujoco
        import numpy as np
        self._mj, self._np = mujoco, np
        m = world.model
        self.model, self.data = m, world.data
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or '' for i in range(m.ngeom)]
        self.names = names
        bid = m.body(box_body).id
        self.is_wall = np.array([n.startswith('zone_wall') for n in names])
        self.is_cargo = np.array([int(m.geom_bodyid[i]) == bid for i in range(m.ngeom)])
        self.is_r1 = np.array([n.startswith('r1__') for n in names])
        self.is_peer = np.array([n.startswith(('r2__', 'r3__')) for n in names])
        self.is_finger = np.array([n in ('r1__left_finger', 'r1__right_finger') for n in names])
        self.stats = {c: {'steps': 0, 'min_dist_m': None, 'max_normal_force_n': 0.} for c in self.CLASSES}
        self.episodes = {'cargo_wall': [], 'r1_wall': [], 'r1_peer': []}
        self._open = {'cargo_wall': None, 'r1_wall': None, 'r1_peer': None}
        self.physics_steps = 0
        self._force = np.zeros(6)

    def _pair(self, a, b, g):
        return (a[g[:, 0]] & b[g[:, 1]]) | (a[g[:, 1]] & b[g[:, 0]])

    def step(self, t):
        self.physics_steps += 1
        d = self.data
        n = int(d.ncon)
        masks = {}
        if n:
            g = d.contact.geom[:n]
            masks = {'cargo_wall': self._pair(self.is_cargo, self.is_wall, g),
                     'r1_wall': self._pair(self.is_r1, self.is_wall, g),
                     'r1_cargo': self._pair(self.is_r1, self.is_cargo, g),
                     'finger_cargo': self._pair(self.is_finger, self.is_cargo, g),
                     'r1_peer': self._pair(self.is_r1, self.is_peer, g)}
        for cls in self.CLASSES:
            mask = masks.get(cls)
            active = mask is not None and bool(mask.any())
            if active:
                idx = self._np.flatnonzero(mask)
                dist = float(d.contact.dist[:n][idx].min())
                force = 0.
                for i in idx:
                    self._mj.mj_contactForce(self.model, d, int(i), self._force)
                    force = max(force, float(self._force[0]))
                s = self.stats[cls]
                s['steps'] += 1
                s['min_dist_m'] = dist if s['min_dist_m'] is None else min(s['min_dist_m'], dist)
                s['max_normal_force_n'] = max(s['max_normal_force_n'], force)
                if cls in self._open:
                    ep = self._open[cls]
                    if ep is None:
                        pairs = sorted({'|'.join(sorted((self.names[int(d.contact.geom[i][0])],
                                                         self.names[int(d.contact.geom[i][1])]))) for i in idx})
                        ep = self._open[cls] = {'start_sim_s': round(t, 4), 'end_sim_s': round(t, 4), 'steps': 0,
                                                'min_dist_m': dist, 'max_normal_force_n': force, 'pairs': pairs}
                    ep['end_sim_s'] = round(t, 4)
                    ep['steps'] += 1
                    ep['min_dist_m'] = min(ep['min_dist_m'], dist)
                    ep['max_normal_force_n'] = max(ep['max_normal_force_n'], force)
            elif cls in self._open and self._open[cls] is not None:
                self.episodes[cls].append(self._open[cls])
                self._open[cls] = None

    def summary(self):
        for cls, ep in self._open.items():
            if ep is not None:
                self.episodes[cls].append(ep)
                self._open[cls] = None
        dt = float(self.model.opt.timestep)
        out = {'physics_steps': self.physics_steps, 'timestep_s': dt, 'scope': 'evaluation only, every physics step',
               'classes': {c: {**s, 'contact_s': round(s['steps'] * dt, 4),
                               'min_dist_m': None if s['min_dist_m'] is None else round(s['min_dist_m'], 6),
                               'max_normal_force_n': round(s['max_normal_force_n'], 4)} for c, s in self.stats.items()},
               'episodes': {c: [{**e, 'min_dist_m': round(e['min_dist_m'], 6),
                                 'max_normal_force_n': round(e['max_normal_force_n'], 4)} for e in eps[:200]]
                            for c, eps in self.episodes.items()},
               'episode_counts': {c: len(eps) for c, eps in self.episodes.items()}}
        return out


def input_contract(pose_label, static_map):
    static_sha = hashlib.sha256(json.dumps(static_map, sort_keys=True).encode()).hexdigest()
    return {'camera': 'robot_cam only; every observation strictly validated (N7 _validate_observation)',
            'own_commands': 'issued servo PWM in actuator_state + own issued chassis commands',
            'pose_source': pose_label, 'static_map': {'map_id': static_map.get('map_id'), 'sha256': static_sha},
            'order_sheet': 'coarse pickup bay (0.5 m) + slot id/centre; no box position or yaw',
            'bay_table_sha256': hashlib.sha256(json.dumps(BAYS, sort_keys=True).encode()).hexdigest(),
            'static_keepouts': 'fixed peer spawn / parking spots from the static layout (StaticKeepout); no live peer pose',
            'forbidden': ['nav_cam', 'cctv_top*', 'live object pose', 'live peer pose', 'measured joints',
                          'contacts/success state'],
            'runner': RUNNER, 'skill_profile': v8.PROFILE}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, choices=sorted(SCENARIOS), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=m1_contract.MODES, default='diagnostic')
    parser.add_argument('--allow-dirty', action='store_true', help='development only; recorded as dirty')
    parser.add_argument('--contact-profile', choices=CONTACT_PROFILES, default=DEFAULT_CONTACT_PROFILE)
    args = parser.parse_args()
    if args.mode == 'm1':
        try:
            select_pose_source('m1', None)
        except m1_contract.ContractViolation as exc:
            raise SystemExit(f'REFUSED: {exc}; M1 needs an own-camera pose estimator (none wired yet)')
    start_prov = provenance()
    if start_prov['dirty'] and not args.allow_dirty:
        raise SystemExit('commit and freeze the source before a recorded run (or --allow-dirty)')
    if start_prov['missing_dependencies']:
        raise SystemExit(f'missing dependencies {start_prov["missing_dependencies"]}')
    import mujoco
    from sim.camera_robot_port import CameraRobotPort
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    (out / 'provenance-start.json').write_text(json.dumps(start_prov, indent=1) + '\n')
    started = time.monotonic()
    load_start = [round(v, 2) for v in os.getloadavg()]
    world, definition, config, box_body, order, setup_only, keepouts = build(args.seed, args.contact_profile)
    pose_source = select_pose_source(args.mode, world)        # m1: raises before any control step
    (out / 'scene.xml').write_text(world.scene_xml)
    port = CameraRobotPort(world, 'r1', allow_reverse=True, allow_mecanum=True)
    holder = {}
    skill = v8.WristZoneDeliveryV8(order, mode=args.mode, static_keepouts=keepouts,
                                   static_bounds_m=config['static_map']['bounds_m'],
                                   planner=make_planner(config['static_map'], order, keepouts,
                                                        lambda: holder['skill'].planner_discs()))
    holder['skill'] = skill
    contacts = PhysicsContactLog(world, box_body)
    control = (out / 'control.jsonl').open('w')
    truth = (out / 'evaluation-only.jsonl').open('w')
    peak = {'max_eq_active': 0, 'max_box_z_m': 0.}

    def step(seconds):
        dt = float(world.model.opt.timestep)
        d = world.data
        for _ in range(max(1, round(seconds / dt))):
            port.tick(float(d.time))
            world._physics_step_for(world.robot('r1'))
            contacts.step(float(d.time))
        peak['max_eq_active'] = max(peak['max_eq_active'], int(d.eq_active.max()) if len(d.eq_active) else 0)
        peak['max_box_z_m'] = max(peak['max_box_z_m'], float(d.body(box_body).xpos[2]))

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
            begin = obs['actuator_state']['servo_pulses']
            delta = max(abs(p - begin[str(s)]) for s, p in targets.items())
            duration = max(.25, delta / 600.)
            count = max(5, math.ceil(duration / .05))
            for sample in range(1, count + 1):
                u = sample / count
                ease = u * u * (3 - 2 * u)
                for servo, end in targets.items():
                    pulse = round(begin[str(servo)] + ease * (end - begin[str(servo)]))
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
            phase, box_phase = skill.phase, skill.box.phase
            phase_times.setdefault(phase, round(float(world.data.time), 3))
            action = skill.decide(obs, estimate)
            control.write(json.dumps({'step': index, 'phase': phase, 'box_skill_phase': box_phase,
                                      'sim_time': obs['sim_time'], 'frame_id': obs['frame_id'],
                                      'frame_sha256': obs['sha256'], 'camera': obs['camera'],
                                      'own_pose_commands': obs['actuator_state']['servo_pulses'],
                                      'pose_estimate': [estimate.x_m, estimate.y_m, estimate.yaw_rad],
                                      'pose_source': estimate.source, 'nav': skill.last_nav,
                                      'box': skill.box.last_box, 'attachment': skill.box.last_attachment,
                                      'face_alignment': skill.box.last_face_alignment,
                                      'action': action}) + '\n')
            gt = world.data.body(box_body)
            truth.write(json.dumps({'step': index, 'phase': phase,
                                    'box_xyz': [round(float(v), 4) for v in gt.xpos],
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
    region = config['static_map']['regions']['zone_' + order.slot_id[0]]
    (zx, zy), (hx, hy) = region['center_m'], region['half_extents_m']
    events = {e['event']: e for e in skill.events}
    lifted = peak['max_box_z_m'] > .045
    on_floor = bz < .03 and tilt < 15.
    in_slot = abs(bx - sx) <= .06 and abs(by - sy) <= .06
    claim = reason == 'OWN_RGB_PLACEMENT_IN_SLOT'
    contact_summary = contacts.summary()
    (out / 'contacts-evaluation-only.json').write_text(json.dumps(contact_summary, indent=1) + '\n')
    evaluation = {
        'grasp_success_gt': bool('grasp_attached' in events and lifted),
        'place_in_slot_gt': bool(in_slot and on_floor),
        'place_in_zone_gt': bool(abs(bx - zx) <= hx and abs(by - zy) <= hy and on_floor),
        'box_final_xyz': [round(bx, 4), round(by, 4), round(bz, 4)], 'box_tilt_deg': round(tilt, 2),
        'slot_error_m': [round(bx - sx, 4), round(by - sy, 4)],
        'max_box_z_m': round(peak['max_box_z_m'], 4), 'weld_eq_active_max': peak['max_eq_active'],
        'contacts_per_physics_step': contact_summary['classes'],
        'contact_episode_counts': contact_summary['episode_counts'],
        'skill_claim_in_slot': claim,
        'skill_claim_agrees_with_gt': claim == bool(in_slot and on_floor),
        'face_normal_sources': [e.get('face_normal_source') for e in skill.events if e['event'] == 'grasp_attached'],
        'peer_contact_steps': contact_summary['classes']['r1_peer']['steps'],
        'peer_positions_end': {r: [round(float(v), 4) for v in world.robot(r).base_xyz()[:2]] for r in ('r2', 'r3')},
    }
    end_prov = provenance()
    outcome = m1_contract.outcome_fields(
        mode=args.mode, pose_sources_seen=skill.pose_sources, diagnostic_success=bool(in_slot and on_floor and claim),
        input_contract=input_contract(pose_source.label, config['static_map']), cameras_seen=skill.cameras_seen)
    result = {
        'schema': SCHEMA, 'runner': RUNNER, 'profile': v8.PROFILE, 'seed': args.seed,
        'development_seed': args.seed not in TEST_SEEDS, 'scenario_setup_only': setup_only,
        'order': order.record(), **outcome,
        'success': outcome['m1_success'],
        'claim_scope': ('DIAGNOSTIC skill isolation in zone_wide_door (west pickup floor with a door_1 crossing, or the '
                        'open east section) with a GT pose stub; not M1, not own-camera localisation')
                       if args.mode == 'diagnostic' else 'M1',
        'variant': VARIANT, 'contact_solver_profile': config['contact_solver_profile'],
        'contact_profile_selected': args.contact_profile,
        'contact_profile_status': 'cargo_noslip_v1 primary, pending the user decision' if args.contact_profile == 'cargo_noslip_v1' else 'zone default',
        'cargo_contact_profile': _cargo_profile_record(args.contact_profile),
        'solver_noslip_iterations': int(world.model.opt.noslip_iterations), 'weld': 'off',
        'source_sha_start': start_prov['head'], 'dirty_source_start': start_prov['dirty'],
        'source_sha_end': end_prov['head'], 'dirty_source_end': end_prov['dirty'],
        'dependency_sha256_start': start_prov['dependency_sha256'],
        'source_changed_during_run': (start_prov['head'] != end_prov['head']
                                      or start_prov['dependency_sha256'] != end_prov['dependency_sha256']),
        'reason': reason, 'steps': index + 1, 'sim_seconds': round(float(world.data.time), 3),
        'sim_limit_s': SIM_LIMIT_S, 'phase_start_sim_s': phase_times, 'events': skill.events,
        'skill_summary': skill.summary(), 'placement_own_rgb': skill.placement,
        'evaluation_only': evaluation, 'scene': {k: v for k, v in definition.manifest.items() if k != 'box_replicas'},
        'static_keepouts': [k.record() for k in keepouts],
        'scene_xml_sha256': hashlib.sha256(world.scene_xml.encode()).hexdigest(),
        'wall_seconds': round(time.monotonic() - started, 1), 'load_average_start': load_start,
        'load_average_end': [round(v, 2) for v in os.getloadavg()]}
    m1_contract.validate_outcome(result)
    (out / 'result.json').write_text(json.dumps(result, indent=1, default=str) + '\n')
    hashes = {n: sha_file(out / n) for n in ('result.json', 'control.jsonl', 'evaluation-only.jsonl', 'scene.xml',
                                             'contacts-evaluation-only.json', 'provenance-start.json')}
    (out / 'hashes.json').write_text(json.dumps(hashes, indent=1) + '\n')
    print(json.dumps({'seed': args.seed, 'reason': reason, 'sim_s': result['sim_seconds'],
                      'diagnostic_success': result['diagnostic_success'], 'm1_success': result['m1_success'],
                      'place_in_slot_gt': evaluation['place_in_slot_gt'],
                      'face': evaluation['face_normal_sources'], 'wall_s': result['wall_seconds'],
                      'cargo_wall_steps': contact_summary['classes']['cargo_wall']['steps'],
                      'peer_steps': contact_summary['classes']['r1_peer']['steps'],
                      'approach': (skill.approach_choice or {}).get('side'), 'replans': skill.keepout_replans}))


def _cargo_profile_record(name):
    from sim.zone_cargo_contact import CARGO_PROFILES, profile_record
    return profile_record(name) if name in CARGO_PROFILES else None


if __name__ == '__main__':
    main()
