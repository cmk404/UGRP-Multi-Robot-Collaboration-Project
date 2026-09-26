"""How high can the MasterPi wrist camera go? (static kinematics, no physics steps)

Environment v3 sets the wall height from this analysis (2026-09-26 user request:
"벽이 너무 낮은 거 같아, 벽 너머가 보여야 하나"). Two bounds are computed:

1. every named arm posture in the code base (search, carry, look, grasp, ...);
2. the kinematic maximum over the whole commandable range: servo 3/4/5 PWM
   500..2500 (``set_servo_pulses`` clamps there) mapped by the simulator's own
   ``pulse_to_joint_targets`` and clipped to the MuJoCo joint ranges.

Both are measured in MuJoCo (``mj_forward`` after ``set_servo_pulses(...,
forward_only=True)``: joint positions only, no ``mj_step``) on a zone scene,
and cross-checked with the pure FK ``harness.visual_arm.camera_extrinsics``.
The highest point of any robot geom (arm, gripper fingers) is reported too, so
the wall can also hide other robots' raised arms. Output: camera_height.json.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

NAMED = {
    'SEARCH_POSE (sim.masterpi_production_v2, owncam nobox drive)': {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500},
    'CARRY_POSTURE = carry_p30 (wrist_zone_skill, owncam_drive loaded)': {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500},
    'LOOK_P20 (owncam_drive stop-and-look)': {1: 1500, 3: 1072, 4: 2400, 5: 1482, 6: 1500},
    'look_p10 (probe_owncam_carry_view, diagnostic only)': {1: 1500, 3: 897, 4: 1998, 5: 1598, 6: 1500},
    'HOVER_POSE / LIFT_POSE (production)': {1: 2000, 3: 650, 4: 2230, 5: 1500, 6: 1500},
    'GRASP_POSE (production)': {1: 2000, 3: 650, 4: 2230, 5: 1920, 6: 1500},
    'CARRY_POSE (production, level)': {1: 1500, 3: 960, 4: 2410, 5: 1215, 6: 1500},
    'STACK_POSE (production)': {1: 1500, 3: 650, 4: 2230, 5: 1750, 6: 1500},
    'PRECAPTURE_POSE (training env)': {1: 2000, 3: 500, 4: 2320, 5: 1320, 6: 1500},
    'v9 LOOK_DOWN (elbow 2480, wrist 560)': {1: 2000, 3: 560, 4: 2480, 5: 1320, 6: 1500},
}
GRID_STEP = 50
REFINE_HALF, REFINE_STEP = 60, 5


def build():
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.session_scenes import ROOT as SROOT
    from sim.zone_scene import ZoneScene
    scene = ZoneScene({'layout': 'zones/zone_wide_door', 'seed': 11, 'map_file': None, 'cargo_ids': None,
                       'robots': {}, 'objects': [], 'builder': None, 'contact_profile': 'local_contact_fine',
                       'params': {'goal': {'A': {'cyan': 1}}, 'extra_boxes': {}}}, SROOT)
    world = MultiMasterPiProductionV2(seed=11, width=640, height=480, render=False,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    return scene, world


def measure(world, rid, pose):
    import mujoco
    import numpy as np
    robot = world.controllers[rid]
    robot.set_servo_pulses(pose, forward_only=True)
    model, data = world.model, world.data
    cam = int(robot.robot_cam_cid)
    xyz = np.asarray(data.cam_xpos[cam], float)
    rot = np.asarray(data.cam_xmat[cam], float).reshape(3, 3)
    # MuJoCo cameras look along -z of their frame.
    forward = -rot[:, 2]
    base_z = float(data.xpos[robot.robot_bid][2])
    top, top_geom = -1., None
    for g in robot_geoms(world, rid):
        r = np.asarray(data.geom_xmat[g], float).reshape(3, 3)
        centre = np.asarray(model.geom_aabb[g][:3], float)
        half = np.asarray(model.geom_aabb[g][3:], float)
        z = float(data.geom_xpos[g][2] + r[2] @ centre + np.abs(r[2]) @ half)
        if z > top:
            top, top_geom = z, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
    return {'camera_z_m': float(xyz[2]), 'optical_pitch_deg': math.degrees(math.asin(max(-1., min(1., forward[2])))),
            'robot_top_z_m': top, 'robot_top_geom': top_geom, 'base_body_z_m': base_z}


_GEOMS = {}


def robot_geoms(world, rid):
    import mujoco
    if rid not in _GEOMS:
        model = world.model
        _GEOMS[rid] = [g for g in range(model.ngeom)
                       if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '').startswith(rid + '__')
                       and int(model.geom_rgba[g][3] * 1000) > 0]
    return _GEOMS[rid]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--output', default=str(Path(__file__).with_name('camera_height.json')))
    args = p.parse_args(argv)
    import mujoco
    import numpy as np
    from harness.visual_arm import camera_extrinsics
    started, load_start = time.time(), os.getloadavg()
    scene, world = build()
    rid = 'r1'
    try:
        named = {}
        for name, pose in NAMED.items():
            m = measure(world, rid, pose)
            origin, axes = camera_extrinsics(pose)
            m['fk_camera_z_m'] = float(origin[2])
            m['fk_optical_pitch_deg'] = math.degrees(math.asin(axes[2][2]))
            named[name] = {'pulses': {str(k): v for k, v in pose.items()}, **m}
        best = None
        grid = range(500, 2501, GRID_STEP)
        count = 0
        for a in grid:
            for b in grid:
                for c in grid:
                    pose = {1: 1500, 3: a, 4: b, 5: c, 6: 1500}
                    m = measure(world, rid, pose)
                    count += 1
                    if best is None or m['camera_z_m'] > best[0]['camera_z_m']:
                        best = (m, pose)
        coarse = best
        centre = best[1]
        fine = range(-REFINE_HALF, REFINE_HALF + 1, REFINE_STEP)
        for da in fine:
            for db in fine:
                for dc in fine:
                    pose = {1: 1500, 3: min(2500, max(500, centre[3] + da)), 4: min(2500, max(500, centre[4] + db)),
                            5: min(2500, max(500, centre[5] + dc)), 6: 1500}
                    m = measure(world, rid, pose)
                    count += 1
                    if m['camera_z_m'] > best[0]['camera_z_m']:
                        best = (m, pose)
        # Highest point of any robot geom over the same grid (arm straight up is the candidate).
        top_best = None
        for a in grid:
            for b in grid:
                for c in grid:
                    pose = {1: 1500, 3: a, 4: b, 5: c, 6: 1500}
                    m = measure(world, rid, pose)
                    if top_best is None or m['robot_top_z_m'] > top_best[0]['robot_top_z_m']:
                        top_best = (m, pose)
        origin, _ = camera_extrinsics(best[1])
        ranges = {}
        for joint in ('shoulder', 'elbow', 'wrist', 'yaw'):
            jid = world.controllers[rid].arm_joint[joint]
            ranges[joint] = [float(v) for v in world.model.jnt_range[jid]]
        result = {
            'schema': 'ugrp.zone_env_v3.camera_height.v1',
            'method': ('MuJoCo mj_forward after set_servo_pulses(pose, forward_only=True) on zones/zone_wide_door '
                       '(no mj_step); camera = r1__robot_cam world position; PWM clamp 500..2500 and joint-range '
                       'clip exactly as the simulator applies commands'),
            'named_postures': named,
            'max_named_camera_z_m': max(v['camera_z_m'] for v in named.values()),
            'kinematic_max': {'camera_z_m': best[0]['camera_z_m'], 'pulses': {str(k): v for k, v in best[1].items()},
                              'optical_pitch_deg': best[0]['optical_pitch_deg'], 'fk_camera_z_m': float(origin[2]),
                              'coarse_grid_step_pwm': GRID_STEP, 'refine': [REFINE_HALF, REFINE_STEP],
                              'coarse_camera_z_m': coarse[0]['camera_z_m']},
            'robot_top_max': {'z_m': top_best[0]['robot_top_z_m'], 'geom': top_best[0]['robot_top_geom'],
                              'pulses': {str(k): v for k, v in top_best[1].items()}, 'grid_step_pwm': GRID_STEP},
            'joint_ranges_rad': ranges, 'evaluations': count,
            'env': {'python': platform.python_version(), 'mujoco': mujoco.__version__, 'numpy': np.__version__,
                    'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                               'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
            'load_average': {'start': [round(v, 2) for v in load_start], 'end': [round(v, 2) for v in os.getloadavg()]},
            'wall_s': round(time.time() - started, 1)}
    finally:
        world.close()
    Path(args.output).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'max_named_camera_z_m': result['max_named_camera_z_m'],
                      'kinematic_max_camera_z_m': result['kinematic_max']['camera_z_m'],
                      'robot_top_max_z_m': result['robot_top_max']['z_m'], 'wall_s': result['wall_s']}))


if __name__ == '__main__':
    main()
