"""Offline body calibration for the executor's look-sweep collision guard (fixed calibration, not control input).

The guard (``harness/zone_own_guards.py``) models the robot's own arm, fingers and held box as
spheres on the forward kinematics of ``harness.visual_arm.tool_pose`` (own issued PWM). This script
measures, once, in the MuJoCo model of the SAME robot (sync SIM, weld OFF, no render), where the
arm geoms really are for the look/search/carry postures over the pan range, and writes:

* the arm yaw-axis mount offset in the base frame (``mount_xyz_m``);
* the coverage residual of the sphere model (how far any arm/finger/box geom AABB corner lies
  outside the model spheres), which the guard adds to its margin;
* the raw per-pose extents (for the record).

It is a calibration of the robot's own body (like a camera calibration). It never runs during an
episode and reads no scene object, peer or episode state. Output: ``body_model_calibration.json``.

Usage (worktree root): python experiments/2026-09-26-zone-own-executor/calibrate_body_model.py
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
HERE = Path(__file__).resolve().parent
SCHEMA = 'ugrp.zone_own_body_calibration.v1'


def corners(model, data, g):
    """World coordinates of the 8 corners of geom g's local AABB."""
    c, h = model.geom_aabb[g][:3], model.geom_aabb[g][3:]
    pts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * h + c
    return data.geom_xpos[g] + pts @ data.geom_xmat[g].reshape(3, 3).T


VALIDATION_POSES = ((2.08, -0.07), (2.05, -0.10), (2.08, 0.18), (2.20, 0.05), (1.00, -1.00))
VALIDATION_WALL_M = .40


def raise_walls(xml, height):
    """Validation-only world: every ``zone_wall_*`` box raised to ``height`` (the v3 wall height)."""
    import re

    def fix(m):
        g = re.sub(r'pos="(\S+) (\S+) [^"]+"', lambda k: f'pos="{k.group(1)} {k.group(2)} {height / 2}"', m.group(0), count=1)
        return re.sub(r'size="(\S+) (\S+) [^"]+"', lambda k: f'size="{k.group(1)} {k.group(2)} {height / 2}"', g, count=1)
    return re.sub(r'<geom name="zone_wall_[^"]*"[^>]*/>', fix, xml)


def validate_sweeps(guards, mujoco, world_cls, port_cls, scene_cls):
    """Old full sweep vs guard plan at door poses, in a 0.40 m-wall world: physical arm-wall contacts vs prediction."""
    import copy
    from harness.owncam_drive import LOOK_P20, SEARCH_POSE, WIDE_LOOK_PANS
    scene = scene_cls.from_tagged('zone_wide_door_tags_v2', 703, {'A': {'cyan': 1}}, None)
    static = copy.deepcopy(scene.config['static_map'])
    for o in static['obstacles']:
        o['height_m'] = VALIDATION_WALL_M
    static['landmarks'].pop('door_posts', None)           # v2 posts are visual-only geoms (contype 0)
    guard = guards.SweepGuard(static)
    out = []
    for x, y in VALIDATION_POSES:
        for policy in ('full_sweep_v1', 'guard_plan'):
            world = world_cls(seed=703, width=64, height=48, render=False, warehouse_layout=scene.engine_layout,
                              warehouse_cargo_ids=None,
                              xml_transform=lambda xml: raise_walls(scene.transform(xml), VALIDATION_WALL_M))
            scene.setup(world)
            m, d = world.model, world.data
            names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
            walls = {g for g, n in enumerate(names) if n.startswith('zone_wall_')}
            own = {g for g, n in enumerate(names) if n.startswith('r1__')}
            c = world.robot('r1')
            adr = c.base_qadr
            d.qpos[adr:adr + 2] = (x, y)
            d.qpos[adr + 3:adr + 7] = (1., 0., 0., 0.)
            d.qvel[c.base_dadr:c.base_dadr + 6] = 0.
            mujoco.mj_forward(m, d)
            port = port_cls(world, 'r1', allow_reverse=True, allow_mecanum=True)
            servo = {int(k): int(v) for k, v in c.servo_command_pulses.items()}
            contacts = []

            def run(seconds, label):
                end = float(d.time) + seconds
                while float(d.time) < end:
                    port.tick(float(d.time))
                    world._physics_step_for(world.controllers['r1'])
                    hit = any(({int(d.contact[i].geom1), int(d.contact[i].geom2)} & own) and
                              ({int(d.contact[i].geom1), int(d.contact[i].geom2)} & walls) for i in range(d.ncon))
                    if hit:
                        contacts.append(label)

            def move(target, label):
                for sid, want in sorted(target.items()):
                    while servo.get(sid, want) != want:
                        cur = servo[sid]
                        servo[sid] = cur + int(np.clip(want - cur, -60, 60))
                        port.apply({'kind': 'look', 'pan_pulse': servo[sid]} if sid == 6 else
                                   {'kind': 'arm', 'servo_id': sid, 'pulse': servo[sid]}, float(d.time))
                        run(.1, f'{label}@{servo.get(6)}')
                run(.6, f'{label}@{servo.get(6)}')

            move({**SEARCH_POSE, 6: 1500}, 'start')
            start_contacts = len(contacts)
            est = guards.OwnPose(x, y, 0., .01, .01)
            plan = guard.plan(servo, LOOK_P20, WIDE_LOOK_PANS, est, loaded=False)
            pans = list(WIDE_LOOK_PANS) if policy == 'full_sweep_v1' else (plan['pans'] or [1500])
            move(dict(LOOK_P20), 'look_arm')
            for pan in pans:
                move({6: pan}, 'pan')
            move({**SEARCH_POSE, 6: 1500}, 'restore')
            predicted_unsafe = sorted({p for p in range(900, 2101, 20)
                                       if guard.arm_clearance({**SEARCH_POSE, **LOOK_P20, 6: p}, est, loaded=False)[0] < 0})
            hit_pans = sorted({int(lbl.split('@')[1]) for lbl in contacts[start_contacts:] if '@' in lbl})
            base = [float(v) for v in d.qpos[adr:adr + 2]]
            out.append({'pose_xy_m': [x, y], 'policy': policy, 'pans_commanded': pans,
                        'guard_plan': {k: plan[k] for k in ('pans', 'dropped', 'reason', 'interval')},
                        'contact_steps_before_look': start_contacts, 'contact_steps': len(contacts) - start_contacts,
                        'contact_pans': hit_pans,
                        'guard_unsafe_pan_range': [predicted_unsafe[0], predicted_unsafe[-1]] if predicted_unsafe else None,
                        'contact_pans_predicted_unsafe': all(p in predicted_unsafe or any(abs(p - q) <= 20 for q in predicted_unsafe)
                                                             for p in hit_pans),
                        'base_drift_m': round(math.hypot(base[0] - x, base[1] - y), 4)})
            world.close()
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--output', default=str(HERE / 'body_model_calibration.json'))
    args = p.parse_args(argv)
    import mujoco

    from harness import zone_own_guards as guards
    from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SEARCH_POSE
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_landmarks import TaggedZoneScene

    started, load0 = time.time(), os.getloadavg()
    scene = TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 703, {'A': {'cyan': 1}}, None)
    world = MultiMasterPiProductionV2(seed=703, width=64, height=48, render=False,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    m, d = world.model, world.data
    rid = 'r1'
    port = CameraRobotPort(world, rid, allow_reverse=True, allow_mecanum=True)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
    own = [g for g, n in enumerate(names) if n.startswith(rid + '__')]
    base_bid = world.robot(rid).robot_bid
    body_of = {g: int(m.geom_bodyid[g]) for g in own}
    body_names = {g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, body_of[g]) or '' for g in own}

    def settle(pose, seconds=2.0):
        t = float(d.time)
        for s, v in pose.items():
            port.apply({'kind': 'look', 'pan_pulse': int(v)} if int(s) == 6 else
                       {'kind': 'arm', 'servo_id': int(s), 'pulse': int(v)}, t)
        end = t + seconds
        while float(d.time) < end:
            port.tick(float(d.time))
            world._physics_step_for(world.controllers[rid])
        mujoco.mj_forward(m, d)

    def to_base(pts):
        """x, y in the robot base frame (heading only); z = height above the floor (world z)."""
        yaw = math.atan2(d.xmat[base_bid][3], d.xmat[base_bid][0])
        c, s = math.cos(yaw), math.sin(yaw)
        rel = pts - d.xpos[base_bid]
        return np.stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1], pts[:, 2]], axis=1)

    # Arm geoms = every own geom whose body is not the chassis/wheel set (moves with servo 6).
    settle({**SEARCH_POSE, 6: 1500})
    ref = {g: to_base(corners(m, d, g)) for g in own}
    settle({**SEARCH_POSE, 6: 1900})
    moved = {g for g in own if np.abs(to_base(corners(m, d, g)) - ref[g]).max() > 1e-3}
    arm = sorted(moved)
    chassis = sorted(set(own) - moved)
    postures = {'look_p20': {**LOOK_P20, 1: 1500}, 'look_p20_open': {**LOOK_P20, 1: 2000},
                'search': dict(SEARCH_POSE), 'carry': dict(CARRY_POSTURE)}
    rows, worst = [], {}
    mount = None
    for name, pose in postures.items():
        for pan in range(900, 2101, 100):
            settle({**pose, 6: pan}, 1.5)
            pts = np.concatenate([to_base(corners(m, d, g)) for g in arm])
            geom_of_point = [names[g] for g in arm for _ in range(8)]
            servo = {**pose, 6: pan}
            spheres = guards.body_spheres(servo, loaded=False, mount_xyz_m=(0., 0., 0.))
            # mount: the arm yaw axis = the point of the first arm body that does not move with pan.
            if mount is None:
                yaw_body = next((b for b in range(m.nbody) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
                                                                or '').startswith(rid + '__') and
                                 m.jnt_bodyid.tolist().count(b) and
                                 any(m.jnt_bodyid[j] == b and (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or '')
                                     .endswith('arm_yaw') for j in range(m.njnt))), None)
                mount = to_base(d.xpos[yaw_body][None])[0].tolist() if yaw_body is not None else [0., 0., 0.]
                mount = [mount[0], mount[1], 0.]            # xy of the yaw axis; heights are floor-frame
            rows.append({'posture': name, 'pan': pan, 'max_x_m': round(float(pts[:, 0].max()), 4),
                         'max_abs_y_m': round(float(np.abs(pts[:, 1]).max()), 4),
                         'min_z_m': round(float(pts[:, 2].min()), 4), 'max_z_m': round(float(pts[:, 2].max()), 4),
                         'points': pts.round(4).tolist(), 'servo': servo, 'n_sphere': len(spheres),
                         'geom_of_point': geom_of_point})
    world.close()
    validation = validate_sweeps(guards, mujoco, MultiMasterPiProductionV2, CameraRobotPort, TaggedZoneScene)
    # Coverage residual of the sphere model with the measured mount.
    residuals = []
    for r in rows:
        spheres = guards.body_spheres(r['servo'], loaded=False, mount_xyz_m=tuple(mount))
        c = np.asarray([s[:3] for s in spheres])
        rad = np.asarray([s[3] for s in spheres])
        pts = np.asarray(r['points'])
        outside = (np.linalg.norm(pts[:, None, :] - c[None], axis=2) - rad[None]).min(axis=1)
        r['coverage_residual_m'] = round(float(max(0., outside.max())), 4)
        r['worst_point'] = [round(float(v), 4) for v in pts[int(outside.argmax())]]
        r['worst_geom'] = r['geom_of_point'][int(outside.argmax())]
        residuals.append(r['coverage_residual_m'])
        worst[r['posture']] = max(worst.get(r['posture'], 0.), r['coverage_residual_m'])
    out = {'schema': SCHEMA, 'robot_model': 'sim.multi_masterpi_production.MultiMasterPiProductionV2 (r1)',
           'map_for_world_only': 'zone_wide_door_tags_v2 seed 703 (no scene object is read)',
           'mount_xyz_m': [round(v, 4) for v in mount], 'arm_geoms': [names[g] for g in arm],
           'chassis_geoms': [names[g] for g in chassis], 'arm_bodies': sorted({body_names[g] for g in arm}),
           'coverage_residual_max_m': round(max(residuals), 4), 'coverage_residual_by_posture_m': worst,
           'guard_model_sha256': hashlib.sha256((ROOT / 'harness' / 'zone_own_guards.py').read_bytes()).hexdigest(),
           'sweep_validation_0p40_walls': validation,
           'rows': [{k: v for k, v in r.items() if k not in ('points', 'geom_of_point')} for r in rows],
           'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
           'wall_s': round(time.time() - started, 1)}
    Path(args.output).write_text(json.dumps(out, indent=1) + '\n')
    print(json.dumps({k: out[k] for k in ('mount_xyz_m', 'coverage_residual_max_m', 'coverage_residual_by_posture_m',
                                          'arm_bodies', 'wall_s')}))
    for v in validation:
        print(json.dumps(v))


if __name__ == '__main__':
    main()
