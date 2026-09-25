"""Grip-hold slip probe for the zone cyan box (DIAGNOSTIC, teacher/GT only).

Question (2026-09-26): in the wrist-only zone skill v2 the held cyan box slides
in the jaws about every 60 s regardless of drive speed. Is that physics or a
contact-solver artefact? This probe grasps the 30 g box with the zone teacher
(GT), moves to the skill's issued carry posture and then (a) holds still and
(b) drives a fixed teacher square, logging the box pose in the gripper frame,
the jaw position/actuator force and the finger-box contact forces.

The same script runs under an explicitly selected contact profile:
``local_contact_fine`` (the zone solo runner) or ``cargo_noslip_v1``
(sim/zone_cargo_contact.py: local_contact_fine + noslip_iterations 10). Weld is
OFF in both. Nothing here feeds a controller; it is evaluation-only evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.probe_owncam_carry_view import Probe  # noqa: E402

SCHEMA = 'ugrp.zone_box_grip_hold_probe.v1'
SEED = 11
BOX_XY = (1.20, -1.60)                  # open floor west of the divider, no wall nearby
CARRY_POSTURE = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}  # wrist_zone_skill v1/v2 carry pose
HOLD_S = 120.
SQUARE = ((1.60, -1.60, 0.), (1.60, -1.10, math.pi / 2), (1.20, -1.10, math.pi), (1.20, -1.60, -math.pi / 2))
LOG_EVERY_S = 2.


class HoldProbe(Probe):
    def __init__(self, out, profile):
        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_arena import episode
        from sim.zone_cargo_contact import CARGO_PROFILES, apply, base_profile, profile_record
        from sim.zone_scene import ZoneScene
        from scripts.zone_teacher import ArmSequence, FOLDED
        self.out = out
        cfg = episode('zone_wide_door', SEED, goal={'B': {'cyan': 1}})
        cfg['contact_solver_profile'] = base_profile(profile)
        self.config = cfg
        self.definition = ZoneScene.from_zone_config(cfg)
        transform = self.definition.transform
        self.profile_record = profile_record(profile) if profile in CARGO_PROFILES else {'name': profile}
        xml_transform = (lambda xml: apply(transform(xml), profile)) if profile in CARGO_PROFILES else transform
        self.world = MultiMasterPiProductionV2(seed=SEED, width=320, height=240, render=True,
                                              warehouse_layout=self.definition.engine_layout,
                                              warehouse_cargo_ids=None, xml_transform=xml_transform)
        self.definition.setup(self.world)
        self.port = CameraRobotPort(self.world, 'r1', allow_reverse=True, allow_mecanum=True)
        self.arm = ArmSequence(self.port, FOLDED)
        self.box = next(iter(cfg['setup_only']['objects'].values()))['body_name']
        self.max_offset_m = 0.
        self.reference = None
        m = self.world.model
        self.noslip_iterations = int(m.opt.noslip_iterations)
        self.finger_geoms = {m.geom('r1__left_finger').id, m.geom('r1__right_finger').id}
        self.box_geoms = {g for g in range(m.ngeom) if int(m.geom_bodyid[g]) == m.body(self.box).id}
        self.jaw = [m.joint(f'r1__{s}_gripper_close') for s in ('left', 'right')]
        self.jaw_act = [m.actuator(f'r1__servo_gripper_{s}').id for s in ('left', 'right')]

    def grip_state(self):
        import mujoco
        m, d = self.world.model, self.world.data
        normal, tangential, n = 0., 0., 0
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {int(c.geom1), int(c.geom2)}
            if pair & self.finger_geoms and pair & self.box_geoms:
                mujoco.mj_contactForce(m, d, i, f6)
                normal += abs(f6[0])
                tangential += float(np.hypot(f6[1], f6[2]))
                n += 1
        jaw_q = [float(d.qpos[m.jnt_qposadr[j.id]]) for j in self.jaw]
        return {'contacts': n, 'normal_n': round(normal, 3), 'tangential_n': round(tangential, 4),
                'jaw_q_m': [round(q, 5) for q in jaw_q],
                'jaw_ctrl_m': [round(float(d.ctrl[a]), 5) for a in self.jaw_act],
                'jaw_force_n': [round(float(d.actuator_force[a]), 3) for a in self.jaw_act]}

    def sample(self, stage, t0):
        rel = self.box_in_gripper()
        delta = rel - self.reference
        return {'stage': stage, 't_s': round(float(self.world.data.time) - t0, 2),
                'box_in_gripper_mm': (rel * 1000).round(2).tolist(),
                'offset_from_grasp_mm': (delta * 1000).round(2).tolist(),
                'offset_norm_mm': round(float(np.linalg.norm(delta)) * 1000, 2),
                'box_z_m': round(self.box_z(), 4), **self.grip_state()}


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--contact-profile', choices=('local_contact_fine', 'cargo_noslip_v1'), required=True)
    parser.add_argument('--hold-s', type=float, default=HOLD_S)
    parser.add_argument('--allow-dirty', action='store_true')
    args = parser.parse_args()
    dirty = bool(git('status', '--porcelain', '--untracked-files=no', '--', 'scripts', 'sim', 'harness'))
    if dirty and not args.allow_dirty:
        raise SystemExit('commit the probe source first (or --allow-dirty for development)')
    args.output.mkdir(parents=True, exist_ok=True)
    wall0 = time.time()
    probe = HoldProbe(args.output, args.contact_profile)
    probe.place_robot_and_box(BOX_XY)
    grasp_z = probe.grasp()
    rows = [probe.sample('grasped_hover', float(probe.world.data.time))]
    probe.arm_to(CARRY_POSTURE, .6)
    probe.reference = probe.box_in_gripper()          # reference = seated in the carry posture
    t0 = float(probe.world.data.time)
    rows.append(probe.sample('carry_start', t0))
    while float(probe.world.data.time) - t0 < args.hold_s:
        probe.step(LOG_EVERY_S)
        rows.append(probe.sample('static_hold', t0))
    t1 = float(probe.world.data.time)
    for goal in SQUARE:
        start = float(probe.world.data.time)
        target = goal
        while float(probe.world.data.time) - start < 60.:
            x, y, yaw = probe.gt_pose()
            ex, ey = target[0] - x, target[1] - y
            eyaw = (target[2] - yaw + math.pi) % (2 * math.pi) - math.pi
            if math.hypot(ex, ey) < .02 and abs(eyaw) < .03:
                break
            fwd = math.cos(yaw) * ex + math.sin(yaw) * ey
            left = -math.sin(yaw) * ex + math.cos(yaw) * ey
            probe.port.apply({'kind': 'mecanum', 'forward': float(np.clip(1.2 * fwd, -.05, .12)),
                              'left': float(np.clip(1.2 * left, -.08, .08)),
                              'turn': float(np.clip(.8 * eyaw, -.12, .12)), 'duration_s': .15},
                             float(probe.world.data.time))
            probe.step(.1)
            if float(probe.world.data.time) - rows[-1]['t_s'] - t0 >= LOG_EVERY_S:
                rows.append(probe.sample('drive_square', t0))
    probe.port.hold(float(probe.world.data.time))
    probe.step(.5)
    rows.append(probe.sample('end', t0))
    hold = [r for r in rows if r['stage'] in ('carry_start', 'static_hold')]
    drive = [r for r in rows if r['stage'] in ('drive_square', 'end')]
    creep = (hold[-1]['offset_norm_mm'] - hold[0]['offset_norm_mm']) / max(1e-9, hold[-1]['t_s'] - hold[0]['t_s'])
    summary = {
        'schema': SCHEMA, 'contact_profile': probe.profile_record,
        'noslip_iterations_in_model': probe.noslip_iterations, 'weld': 'off',
        'box_mass_kg': float(probe.world.model.body(probe.box).mass[0]),
        'grip_command_pwm': CARRY_POSTURE[1], 'grasp_lift_box_z_m': round(grasp_z, 4),
        'static_hold_s': round(hold[-1]['t_s'], 1),
        'static_offset_end_mm': hold[-1]['offset_norm_mm'],
        'static_creep_mm_per_min': round(creep * 60, 3),
        'static_mean_normal_n': round(float(np.mean([r['normal_n'] for r in hold])), 3),
        'static_mean_tangential_n': round(float(np.mean([r['tangential_n'] for r in hold])), 4),
        'drive_s': round(drive[-1]['t_s'] - (t1 - t0), 1),
        'drive_offset_end_mm': drive[-1]['offset_norm_mm'],
        'drive_offset_gain_mm': round(drive[-1]['offset_norm_mm'] - hold[-1]['offset_norm_mm'], 2),
        'box_held_end': drive[-1]['box_z_m'] > .08,
        'jaw_end': {k: drive[-1][k] for k in ('jaw_q_m', 'jaw_ctrl_m', 'jaw_force_n')},
        'sim_s': round(float(probe.world.data.time), 1), 'wall_s': round(time.time() - wall0, 1),
        'load_avg': [round(x, 2) for x in os.getloadavg()],
        'git_sha': git('rev-parse', 'HEAD'), 'dirty': dirty,
        'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                  'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')},
    }
    (args.output / 'samples.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (args.output / 'result.json').write_text(json.dumps(summary, indent=2))
    (args.output / 'hashes.json').write_text(json.dumps(
        {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(args.output.glob('*.j*'))
         if p.name != 'hashes.json'}, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
