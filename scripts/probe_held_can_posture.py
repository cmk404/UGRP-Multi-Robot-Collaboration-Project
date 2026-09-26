#!/usr/bin/env python3
"""Held-can observation posture: view sweep (fixture) and retention (physics).

Package B dev finding (``experiments/2026-09-26-zone-own-perception``): in the
wrist skill's CARRY posture a held 50 mm can shows only its top rim at the frame
edge and nothing inside the held-item region, so a robot cannot verify that it is
holding a can. This probe answers two separate questions.

``view``       Where does a held can land in the own wrist frame, per posture?
               The can is *posed* between the jaws and a single frame is rendered
               without stepping physics (a perception fixture, exactly like the
               ``hold_*`` views of the package B render; weld OFF, no contact
               assistance). Ground truth segmentation is eval-only: it measures
               the answer, it never reaches a judgment.

``retention``  Does the can stay in the jaws through the proposed posture?
               A real teacher grasp (``scripts.cargo_formation_teacher``, ground
               truth poses/IK - teacher condition, never a student success),
               then CARRY -> observation posture -> hold -> CARRY, measuring the
               can's slip in the gripper frame. Run under ``cargo_noslip_v1``
               (primary, the profile the team-cargo path selects) and under
               ``local_contact_fine`` for comparison. No weld, no equality
               constraint: every sample asserts ``eq_active`` stays 0.

    .venv-sim/bin/python -m scripts.probe_held_can_posture view \
        --output outputs/held-can/view
    .venv-sim/bin/python -m scripts.probe_held_can_posture retention \
        --contact-profile cargo_noslip_v1 --output outputs/held-can/ret-noslip
"""
from __future__ import annotations

import argparse
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

from harness.visual_arm import forward_grip, tool_pose  # noqa: E402

SCHEMA = 'ugrp.held_can_posture.v1'
BASE_Z_M = .032355118817659255
CAN_KIND = 'can'
CAN_HEIGHT_M = .050
GRASP_Z_M = .024                      # sim.zone_cargo.GRASP_Z_M: jaws 24 mm up the can
HELD_Z_OFFSET_CARGO = -GRASP_Z_M      # body origin (can floor) relative to the grip site
CARRY_POSTURE = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
FOLDED = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}

# Posture candidates for the sweep. The camera and the grip site are in the same
# rigid body (``robot_cam`` at gripper (.067, 0, .0136), ``grip_site`` at
# (.100, 0, 0)), so a posture cannot move the held object in the *tool* frame.
# What a posture does change is the angle between the tool axis and the can's
# world-vertical axis, plus the background and the lighting. The sweep therefore
# varies the tool pitch (servo 3/4/5) over the safe range and reports what the
# renderer actually shows.
SWEEP = {
    'carry_p30': {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500},          # the wrist skill carry posture
    'look_p20': {1: 1500, 3: 1072, 4: 2400, 5: 1482, 6: 1500},          # package B look posture
    'tilt_p00': {1: 1500, 3: 1100, 4: 1900, 5: 1700, 6: 1500},
    'tilt_m20': {1: 1500, 3: 1320, 4: 1900, 5: 1700, 6: 1500},
    'tilt_m45': {1: 1500, 3: 1600, 4: 1900, 5: 1700, 6: 1500},
    'tilt_m70': {1: 1500, 3: 1880, 4: 1900, 5: 1700, 6: 1500},
    'high_m45': {1: 1500, 3: 1500, 4: 1700, 5: 1900, 6: 1500},
    'high_m70': {1: 1500, 3: 1780, 4: 1700, 5: 1900, 6: 1500},
    'near_m55': {1: 1500, 3: 1700, 4: 2150, 5: 1560, 6: 1500},
    'near_m80': {1: 1500, 3: 1980, 4: 2150, 5: 1560, 6: 1500},
}
# Regions scored for every posture: the package B v1 held-item region and three
# candidate v2 regions (upper band, lower band, full usable frame).
REGIONS = {'carry_roi_v1': (.18, .10, .82, .92), 'upper': (.10, .02, .90, .50),
           'lower': (.10, .50, .90, .98), 'frame': (0., 0., 1., 1.)}


def _git(*args):
    r = subprocess.run(['git', *args], cwd=ROOT, text=True, capture_output=True)
    return r.stdout.strip() if r.returncode == 0 else 'unavailable'


def posture_geometry(pose):
    """Analytic tool/grip geometry of a posture (issued pulses only)."""
    gx, gy, gz = forward_grip(pose)
    tp = tool_pose(pose)
    return {'grip_base_m': [round(gx, 4), round(gy, 4), round(gz, 4)],
            'can_floor_z_m': round(gz+HELD_Z_OFFSET_CARGO, 4),
            'can_top_z_m': round(gz+HELD_Z_OFFSET_CARGO+CAN_HEIGHT_M, 4),
            'tool_pitch_deg': round(tp.pitch_deg, 2),
            'grip_clear_of_floor': bool(gz+HELD_Z_OFFSET_CARGO > .012)}


# ---------------------------------------------------------------------- view

def _build_view_world(variant, seed, contact_profile):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo import instances, place as place_cargo
    from sim.zone_cargo_scene import CargoZoneScene
    item = {'item_id': 'held', 'kind': CAN_KIND, 'pose': [2.5, .2, 0.]}
    scene = CargoZoneScene.from_cargo_config(variant, seed, cargo=[item], goal={'A': {'cyan': 1}},
                                             contact_profile=contact_profile)
    world = MultiMasterPiProductionV2(seed=seed, width=640, height=480, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    inst = instances([item])[0]
    place_cargo(world, [inst])
    return world, scene, inst


def _can_geoms(world, inst):
    import mujoco
    out = []
    for part in inst.spec().parts:
        gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, inst.geom(part.name))
        if gid >= 0:
            out.append(gid)
    return out


def _set_can(world, inst, xyz, yaw):
    import mujoco
    jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, inst.joint)
    q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
    world.data.qpos[q:q+7] = [xyz[0], xyz[1], xyz[2], math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
    world.data.qvel[v:v+6] = 0


def _region_counts(mask, regions=REGIONS):
    height, width = mask.shape[:2]
    out = {}
    for name, (fx0, fy0, fx1, fy1) in regions.items():
        x0, y0 = int(round(fx0*width)), int(round(fy0*height))
        x1, y1 = int(round(fx1*width)), int(round(fy1*height))
        out[name] = int(mask[y0:y1, x0:x1].sum())
    return out


def run_view(args):
    from scripts.eval_zone_color_detection import _segment, _valid_own_region
    out = Path(args.output)
    (out/'frames').mkdir(parents=True, exist_ok=False)
    load = {'start': os.getloadavg()[0]}
    started = time.monotonic()
    world, scene, inst = _build_view_world(args.variant, args.seed, args.contact_profile)
    rows = []
    try:
        import mujoco
        gids = _can_geoms(world, inst)
        actor = 'r1'
        region = world.robot(actor)
        world.robot(actor).set_base_pose_for_test((2.2, .6, BASE_Z_M), 0.)
        mujoco.mj_forward(world.model, world.data)
        valid = _valid_own_region(world, actor)
        for name, pose in SWEEP.items():
            geo = posture_geometry(pose)
            world._team_joint_move_servos({actor: dict(pose)}, .35, settle_s=.35)
            grip = world.robot(actor).site_xyz('grip_site')
            yaw = float(world.robot(actor).base_rpy()[2])
            _set_can(world, inst, (float(grip[0]), float(grip[1]), float(grip[2])+HELD_Z_OFFSET_CARGO), yaw)
            mujoco.mj_forward(world.model, world.data)
            jpeg = world.render_jpeg(robot_id=actor, camera='robot_cam', quality=90)
            (out/'frames'/f'{name}.jpg').write_bytes(jpeg)
            seg = _segment(world, None, actor)
            mask = np.isin(seg, gids) & valid
            counts = _region_counts(mask)
            ys, xs = np.nonzero(mask)
            rows.append({'posture': name, 'issued_pwm': {str(k): int(v) for k, v in pose.items()},
                         **geo, 'visible_px': int(mask.sum()), 'region_px': counts,
                         'bbox': None if not len(ys) else [int(xs.min()), int(ys.min()),
                                                           int(xs.max()), int(ys.max())],
                         'centroid': None if not len(ys) else [round(float(xs.mean()), 1),
                                                               round(float(ys.mean()), 1)],
                         'grip_world_m': [round(float(v), 4) for v in grip]})
            print(json.dumps(rows[-1]))
    finally:
        world.close()
    load['end'] = os.getloadavg()[0]
    record = {'schema': SCHEMA+'.view', 'mode': 'perception fixture: can posed between the jaws,'
              ' physics not stepped, weld OFF, no contact assistance',
              'source_sha': _git('rev-parse', 'HEAD'), 'source_dirty': bool(_git('status', '--porcelain')),
              'variant': args.variant, 'seed': args.seed, 'contact_profile': args.contact_profile,
              'scene_xml_sha256': scene.manifest['scene_xml_sha256'],
              'camera_and_grip_are_one_rigid_body': True,
              'regions': {k: list(v) for k, v in REGIONS.items()},
              'labels': 'segmentation pixel counts are EVAL ONLY (they never reach a judgment)',
              'load_average_1min': load, 'wall_s': round(time.monotonic()-started, 1), 'postures': rows}
    (out/'view.json').write_text(json.dumps(record, indent=1))
    print(json.dumps({'written': str(out/'view.json'), 'load_average_1min': load,
                      'wall_s': record['wall_s']}))


# ----------------------------------------------------------------- retention

def _gripper_frame(world, rid):
    """Origin and rotation of the gripper body (eval-only truth for slip)."""
    body = world.data.body(world.robot(rid)._n('gripper'))
    return np.asarray(body.xpos, float).copy(), np.asarray(body.xmat, float).reshape(3, 3).copy()


def _can_in_gripper(world, rid, inst):
    origin, rot = _gripper_frame(world, rid)
    pos = np.asarray(world.data.body(inst.body).xpos, float)
    return rot.T @ (pos-origin)


def run_retention(args):
    from scripts.cargo_formation_teacher import FormationTeacher
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo import instances, world_grasps
    from sim.zone_cargo_contact import profile_record
    from sim.zone_cargo_scene import CargoZoneScene
    from scripts.zone_teacher import ArmSequence
    import mujoco
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    load = {'start': os.getloadavg()[0]}
    started = time.monotonic()
    item = {'item_id': 'held', 'kind': CAN_KIND, 'pose': [2.5, .2, 0.]}
    scene = CargoZoneScene.from_cargo_config(args.variant, args.seed, cargo=[item],
                                             goal={'A': {'cyan': 1}}, contact_profile=args.contact_profile)
    world = MultiMasterPiProductionV2(seed=args.seed, width=320, height=240, render=False,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    inst = instances([item])[0]
    (out/'scene.xml').write_text(world.scene_xml)
    base = world_grasps(inst)['any']['base_xyyaw']
    world.robot('r1').set_base_pose_for_test((base[0]-.10*math.cos(base[2]), base[1]-.10*math.sin(base[2]),
                                             BASE_Z_M), base[2])
    mujoco.mj_forward(world.model, world.data)
    ports = {rid: CameraRobotPort(world, rid, allow_reverse=True, allow_mecanum=True)
             for rid in ('r1', 'r2', 'r3')}
    events, trace = [], []
    teacher = FormationTeacher(world, {'r1': ports['r1']}, inst, {'r1': 'any'}, legs=[],
                              log=lambda kind, now, **d: events.append({'event': kind,
                                                                        'sim_time_s': round(now, 3), **d}),
                              hold_s=1.5)
    posture = dict(args.posture_pwm or SWEEP[args.posture])
    arm = None
    stage, stage_t = 'teacher', 0.
    reference, worst, eq_max, lost_at = None, 0., 0, None
    dt = float(world.model.opt.timestep)
    step, now = 0, 0.
    fingers = {}
    for side in ('left', 'right'):
        fingers[mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, f'r1__{side}_finger')] = side
    can_geoms = set(_can_geoms(world, inst))
    try:
        while float(world.data.time) < args.max_sim_s:
            now = float(world.data.time)
            if stage == 'teacher':
                teacher.tick(now, {'cargo_min_z': float(world.data.body(inst.body).xpos[2])})
                # Take over while the can is still held, before the teacher lowers it.
                if teacher.phase == 'hold' and now - teacher.phase_t >= 1.0:
                    arm = ArmSequence(ports['r1'], {**teacher.arms['r1'].commanded})
                    arm.queue({**CARRY_POSTURE, 1: 1500}, now, duration=1.6, settle=.6)
                    arm.queue({**posture, 1: 1500}, now, duration=1.8, settle=float(args.hold_s))
                    arm.queue({**CARRY_POSTURE, 1: 1500}, now, duration=1.8, settle=1.0)
                    stage, stage_t = 'posture', now
                    reference = _can_in_gripper(world, 'r1', inst)
                    events.append({'event': 'posture_cycle_start', 'sim_time_s': round(now, 3),
                                   'reference_can_in_gripper_m': [round(float(v), 5) for v in reference]})
                elif teacher.phase in ('lower', 'release', 'retract', 'back_off', 'settle', 'done'):
                    events.append({'event': 'teacher_finished_before_takeover', 'sim_time_s': round(now, 3),
                                   'phase': teacher.phase, 'outcome': teacher.outcome})
                    break
            else:
                if arm.tick(now) and stage == 'posture':
                    stage = 'done'
                    events.append({'event': 'posture_cycle_end', 'sim_time_s': round(now, 3)})
            for port in ports.values():
                port.tick(now)
            world._physics_step_for(world.controllers['r1'])
            step += 1
            if world.data.eq_active.any():
                raise RuntimeError('equality constraint became active: weld assistance is forbidden')
            if step % 25 == 0:   # every 50 ms of SIM time
                held = _can_in_gripper(world, 'r1', inst)
                slip = 0. if reference is None else float(np.linalg.norm(held-reference))*1000.
                contact = 0
                for i in range(world.data.ncon):
                    c = world.data.contact[i]
                    if (c.geom1 in fingers and c.geom2 in can_geoms) or \
                       (c.geom2 in fingers and c.geom1 in can_geoms):
                        contact += 1
                worst = max(worst, slip)
                if reference is not None and contact == 0 and lost_at is None:
                    lost_at = round(now, 3)
                trace.append({'sim_time_s': round(now, 3), 'stage': stage, 'phase': teacher.phase,
                              'can_in_gripper_m': [round(float(v), 5) for v in held],
                              'slip_mm': round(slip, 3), 'finger_can_contacts': contact,
                              'can_z_m': round(float(world.data.body(inst.body).xpos[2]), 4),
                              'issued_arm_pwm': None if arm is None else
                              {str(k): int(v) for k, v in arm.commanded.items()}})
            if stage == 'done':
                break
    finally:
        world.close()
    load['end'] = os.getloadavg()[0]
    record = {'schema': SCHEMA+'.retention', 'source_sha': _git('rev-parse', 'HEAD'),
              'source_dirty': bool(_git('status', '--porcelain')),
              'contact_profile': args.contact_profile,
              'contact_profile_record': (profile_record(args.contact_profile)
                                         if args.contact_profile != 'local_contact_fine' else
                                         {'name': 'local_contact_fine', 'base': 'local_contact_fine'}),
              'weld': 'off', 'eq_active_max': eq_max, 'posture': args.posture,
              'posture_pwm': {str(k): int(v) for k, v in posture.items()},
              'posture_geometry': posture_geometry(posture), 'hold_s': args.hold_s,
              'condition': 'GT teacher grasp (teacher condition, not a student success)',
              'variant': args.variant, 'seed': args.seed,
              'scene_xml_sha256': scene.manifest['scene_xml_sha256'],
              'reached_posture_cycle': reference is not None,
              'completed_cycle': stage == 'done',
              'max_slip_mm': round(worst, 3), 'lost_contact_at_sim_s': lost_at,
              'final_slip_mm': None if not trace else trace[-1]['slip_mm'],
              'teacher_outcome': teacher.outcome, 'teacher_phase': teacher.phase,
              'sim_time_s': round(now, 3), 'load_average_1min': load,
              'wall_s': round(time.monotonic()-started, 1)}
    (out/'retention.json').write_text(json.dumps(record, indent=1))
    (out/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in trace))
    (out/'events.json').write_text(json.dumps(events, indent=1))
    print(json.dumps({k: record[k] for k in ('contact_profile', 'posture', 'max_slip_mm', 'final_slip_mm',
                                             'lost_contact_at_sim_s', 'reached_posture_cycle',
                                             'completed_cycle', 'eq_active_max', 'teacher_outcome',
                                             'load_average_1min', 'wall_s')}))


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    v = sub.add_parser('view', help='render one held-can frame per posture and measure it')
    v.add_argument('--output', type=Path, required=True)
    v.add_argument('--variant', default='zone_wide')
    v.add_argument('--seed', type=int, default=11)
    v.add_argument('--contact-profile', default='local_contact_fine')
    r = sub.add_parser('retention', help='teacher grasp, then CARRY -> posture -> CARRY under physics')
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--variant', default='zone_wide')
    r.add_argument('--seed', type=int, default=11)
    r.add_argument('--contact-profile', default='cargo_noslip_v1')
    r.add_argument('--posture', default='look_p20', choices=sorted(SWEEP))
    r.add_argument('--posture-pwm', type=json.loads, default=None,
                   help='explicit {"3": .., "4": .., "5": .., "6": ..} instead of a named posture')
    r.add_argument('--hold-s', type=float, default=6.0)
    r.add_argument('--max-sim-s', type=float, default=90.0)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if args.command == 'view':
        run_view(args)
    else:
        run_retention(args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
