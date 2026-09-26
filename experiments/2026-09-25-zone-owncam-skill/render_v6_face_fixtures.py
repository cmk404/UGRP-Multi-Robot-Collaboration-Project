#!/usr/bin/env python3
"""Render controlled own-camera face-inspection frames for the v6 face-yaw tests.

Offline fixture generator (not a skill run). One cyan box is placed in bay E2
of zone_wide_door with a known yaw; r1 stands still, facing +x, at a recorded
v5 face-inspection range and arm posture, with the wrist pan pointed at the box.
For each view the script stores the exact robot_cam JPEG, the issued own PWM,
and N7's own-RGB ground-box fit (the aligner's live target input). The box's
relative yaw is stored as ``label_relative_yaw_deg_evaluation_only``: it is only
the test label and is never passed to the estimator.

Box yaws: -30 ... +30 deg including exactly 0 and +/-2 deg (PR #201 dev-a2:
v5's cuboid-yaw vote never became ready on axis-aligned boxes seen head-on).
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))

YAWS_DEG = (-30., -20., -10., -5., -2., 0., 2., 5., 10., 20., 30.)
# (range_m, lateral_m, posture) -- postures are the most frequent v5 cohort
# face-inspection PWM tuples (servo 1, 3, 4, 5) within 0.405 m of the box.
POSTURES = {'A': {1: 2000, 3: 634, 4: 2320, 5: 1320}, 'B': {1: 2000, 3: 500, 4: 2384, 5: 1320}}
VIEWS = ((.39, 0., 'A'), (.30, .04, 'B'), (.36, -.03, 'A'))
BOX_XY = (3.70, -1.50)
OUT = ROOT / 'tests' / 'fixtures' / 'wrist_zone_skill_v6'


def main():
    import mujoco
    import run_zone_owncam_skill_v5 as r5
    from harness.markerless_box import observe_ground_box
    from sim.camera_robot_port import CameraRobotPort
    OUT.mkdir(parents=True, exist_ok=True)
    world, _definition, config, _box_body, _order, _setup = r5.build(409, 'cargo_noslip_v1')
    (oid, item), = config['setup_only']['objects'].items()
    jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, item['joint_name'])
    q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
    port = CameraRobotPort(world, 'r1', allow_reverse=True, allow_mecanum=True)
    robot = world.robot('r1')
    frames = []

    def settle(seconds):
        dt = float(world.model.opt.timestep)
        for _ in range(max(1, round(seconds / dt))):
            port.tick(float(world.data.time))
            world._physics_step_for(robot)

    for yaw in YAWS_DEG:
        for vi, (rng, lat, posture) in enumerate(VIEWS):
            half = math.radians(yaw) / 2
            world.data.qpos[q:q + 7] = [BOX_XY[0], BOX_XY[1], .016, math.cos(half), 0, 0, math.sin(half)]
            world.data.qvel[v:v + 6] = 0
            robot.set_base_pose_for_test((BOX_XY[0] - rng, BOX_XY[1] - lat, .032355118817659255), 0.)
            mujoco.mj_forward(world.model, world.data)
            pan = int(round(1500 + math.degrees(math.atan2(lat, rng)) * 2000 / 180))
            for servo, pulse in POSTURES[posture].items():
                port.apply({'kind': 'arm', 'servo_id': servo, 'pulse': pulse}, float(world.data.time))
            port.apply({'kind': 'look', 'pan_pulse': pan}, float(world.data.time))
            settle(1.5)
            obs = port.capture()
            payload = base64.b64decode(obs['image'])
            pose = obs['actuator_state']['servo_pulses']
            box = observe_ground_box(obs['image'], pose, 'small_box_01')
            gt_q = world.data.body(_box_body).xquat
            rel = math.degrees(2 * math.atan2(float(gt_q[3]), float(gt_q[0])) - float(robot.base_rpy()[2]))
            name = f'yaw{yaw:+05.1f}_v{vi}.jpg'.replace('+', 'p').replace('-', 'm')
            (OUT / name).write_bytes(payload)
            frames.append({'file': name, 'sha256': hashlib.sha256(payload).hexdigest(),
                           'own_servo_pwm': pose, 'view': {'range_m': rng, 'lateral_m': lat, 'posture': posture},
                           'n7_target_xy_m': (None if not box.get('visible') else
                                              [round(float(c), 4) for c in box['estimated_box_center_base_m'][:2]]),
                           'n7_cuboid_yaw_mod90_deg': (None if box.get('estimated_yaw_mod_pi_rad') is None else
                                                       round(math.degrees(box['estimated_yaw_mod_pi_rad']) % 90, 2)),
                           'n7_iou': box.get('floor_hypothesis_projection_iou'),
                           'label_setup_yaw_deg': yaw,
                           'label_relative_yaw_deg_evaluation_only': round((rel + 45) % 90 - 45, 3)})
            print(name, frames[-1]['n7_target_xy_m'], frames[-1]['label_relative_yaw_deg_evaluation_only'],
                  frames[-1]['n7_cuboid_yaw_mod90_deg'])
    (OUT / 'frames.json').write_text(json.dumps({
        'generator': 'experiments/2026-09-25-zone-owncam-skill/render_v6_face_fixtures.py',
        'scene': 'zone_wide_door seed 409 world, box moved to bay E2, r1 stationary facing +x',
        'contact_profile': 'cargo_noslip_v1', 'labels': 'evaluation only; never estimator input',
        'frames': frames}, indent=1) + '\n')


if __name__ == '__main__':
    main()
