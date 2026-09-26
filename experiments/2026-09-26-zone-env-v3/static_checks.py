"""Environment v3 static checks ST1-ST5 (prereg.json phase 1). DIAGNOSTIC / EVALUATION TOOL.

This tool places robots and boxes with simulator state (teleport, kinematic box
placement) to measure what the wrist and TOP cameras can see. None of it is a
robot input or a student result. Physics is stepped only once per run: to record
settled arm postures and one real teacher grasp of the cyan box (weld OFF,
cargo_noslip_v1), from which the held-box pose relative to the gripper is taken.
Every view after that is ``mj_forward`` only (no ``mj_step``).

ST1 map/scene consistency, ST2 tag readability vs distance, ST3 tag coverage at
task stations, ST4 wall occlusion of the other room (v2 0.10 m vs v3 0.40 m
walls; wrist frames saved), ST5 TOP-camera floor coverage (v1/v2 vs v3 walls).
"""
from __future__ import annotations

import argparse
import hashlib
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

import numpy as np  # noqa: E402

SCHEMA = 'ugrp.zone_env_v3.static_checks.v1'
X = Path(__file__).resolve().parent
DEFAULT_OUT = Path('/Users/changmin/projects/ugrp/outputs/zone-env-v3-20260926/static')
GOAL, EXTRA = {'A': {'cyan': 1}}, {'red': 2, 'green': 1}
SEED = 700                       # setup seed of the static-check scene (box layout only)
V3 = 'zone_wide_door_tags_v3'
V2 = 'zone_wide_door_tags_v2'
PROFILE = 'cargo_noslip_v1'
LOOK_P10 = {1: 1500, 3: 897, 4: 1998, 5: 1598, 6: 1500}
ARM_UP = {1: 1500, 3: 1550, 4: 1550, 5: 1600, 6: 1500}      # highest robot geom (camera_height.json)
ST2_DISTANCES = (.3, .6, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5)
ST2_BEARINGS_DEG = (0., 30.)
ST4_DISTANCES = (.4, .8, 1.5)
GEOMGROUP_WRIST = np.array([1, 1, 1, 1, 0, 0], np.uint8)     # robot_cam scene option (groups 4, 5 hidden)
GEOMGROUP_TOP = np.array([1, 1, 1, 1, 1, 0], np.uint8)       # TOP renders all groups; 5 = invisible prototypes
GRID_M = .02


# --------------------------------------------------------------------------- helpers

def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()


def git(*args):
    import subprocess
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def yaw_quat(yaw):
    return np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)])


def quat_mul(a, b):
    import mujoco
    out = np.empty(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def mat_quat(mat):
    import mujoco
    out = np.empty(4)
    mujoco.mju_mat2Quat(out, np.asarray(mat, float).ravel())
    return out


def make_world(name, seed=SEED, render=True):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_landmarks import TaggedZoneScene
    scene = TaggedZoneScene.from_tagged(name, seed, GOAL, EXTRA, contact_profile=PROFILE)
    world = MultiMasterPiProductionV2(seed=seed, width=640, height=480, render=render,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    return scene, world


def set_free(world, body, pos, quat):
    import mujoco
    m, d = world.model, world.data
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, body + '_free')
    q, v = int(m.jnt_qposadr[jid]), int(m.jnt_dofadr[jid])
    d.qpos[q:q + 3] = pos
    d.qpos[q + 3:q + 7] = quat
    d.qvel[v:v + 6] = 0.


def clear_scene(world, scene, keep_robot, keep_box=None):
    """Move the other robots and every box except ``keep_box`` far outside the arena."""
    import mujoco
    k = 0
    for rid, robot in world.controllers.items():
        if rid != keep_robot:
            robot.set_base_pose_for_test((40. + 3*k, 40., .0324), 0.)
            k += 1
    for i, (oid, obj) in enumerate(sorted(scene.config['setup_only']['objects'].items())):
        if obj['body_name'] != keep_box:
            set_free(world, obj['body_name'], (50. + .3*i, 50., .016), (1., 0., 0., 0.))
    mujoco.mj_forward(world.model, world.data)


def robot_state(world, rid):
    m, d = world.model, world.data
    r = world.controllers[rid]
    q = np.array(d.qpos[r.base_qadr:r.base_qadr + 7], float)
    yaw = float(r.base_rpy()[2])
    return {'joints': {n: float(d.qpos[m.jnt_qposadr[j]]) for n, j in r.arm_joint.items()},
            'fingers': [float(d.qpos[m.jnt_qposadr[j]]) for j in r.gripper_joint],
            'base_z': float(q[2]), 'tilt_wxyz': quat_mul(quat_conj(yaw_quat(yaw)), q[3:]).tolist(),
            'servo_command': {str(k): int(v) for k, v in sorted(r.servo_command_pulses.items())}}


def commanded_state(world, rid, pose, like):
    """Joint state of a commanded pose without sag (forward_only), base as ``like``."""
    r = world.controllers[rid]
    r.set_servo_pulses(pose, forward_only=True)
    state = robot_state(world, rid)
    state.update(base_z=like['base_z'], tilt_wxyz=like['tilt_wxyz'],
                 servo_command={str(k): int(v) for k, v in sorted(pose.items())}, source='commanded (forward_only)')
    return state


def box_relative(world, rid, body):
    d = world.data
    g = world.controllers[rid].gripper_bid
    rg = np.array(d.xmat[g]).reshape(3, 3)
    b = world.model.body(body).id
    return {'p': (rg.T @ (np.array(d.xpos[b]) - np.array(d.xpos[g]))).tolist(),
            'R': (rg.T @ np.array(d.xmat[b]).reshape(3, 3)).tolist()}


def place(world, rid, xy, yaw, state, pan=None, box=None):
    """Teleport robot ``rid`` (and the held box) to a pose; mj_forward only."""
    import mujoco
    m, d = world.model, world.data
    r = world.controllers[rid]
    d.qpos[r.base_qadr:r.base_qadr + 3] = [xy[0], xy[1], state['base_z']]
    d.qpos[r.base_qadr + 3:r.base_qadr + 7] = quat_mul(yaw_quat(yaw), state['tilt_wxyz'])
    d.qvel[r.base_dadr:r.base_dadr + 6] = 0.
    for name, jid in r.arm_joint.items():
        value = state['joints'][name]
        if name == 'yaw' and pan is not None and int(pan) != 1500:
            value = r.pulse_to_joint_targets({6: int(pan)})['yaw']
        d.qpos[m.jnt_qposadr[jid]] = value
    for jid, value in zip(r.gripper_joint, state['fingers']):
        d.qpos[m.jnt_qposadr[jid]] = value
    mujoco.mj_forward(m, d)
    if box is not None:
        body, rel = box
        g = r.gripper_bid
        rg = np.array(d.xmat[g]).reshape(3, 3)
        set_free(world, body, np.array(d.xpos[g]) + rg @ np.array(rel['p']), mat_quat(rg @ np.array(rel['R'])))
        mujoco.mj_forward(m, d)


def camera_pose(world, rid):
    r = world.controllers[rid]
    r._sync_real_camera_mount()
    cid = int(r.robot_cam_cid)
    return np.array(world.data.cam_xpos[cid], float), np.array(world.data.cam_xmat[cid], float).reshape(3, 3)


def wrist_jpeg(world, rid):
    import cv2
    rgb = world.render_rgb(robot_id=rid, camera='robot_cam')
    ok, buf = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
    jpeg = buf.tobytes()
    seen = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    return jpeg, seen


def base_for_camera(world, rid, state, cam_xy, yaw, pan=None, box=None):
    """Base xy that puts the wrist camera at ``cam_xy`` with heading ``yaw``."""
    place(world, rid, (0., 0.), 0., state, pan=pan, box=box)
    origin, _ = camera_pose(world, rid)
    ox, oy = origin[0], origin[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (cam_xy[0] - (c*ox - s*oy), cam_xy[1] - (s*ox + c*oy))


# --------------------------------------------------------------------------- posture library (physics)

def posture_library(world, scene, log):
    """Settled arm states (unloaded SEARCH/LOOK_P20; loaded CARRY/LOOK_P20) and the held-box pose."""
    import mujoco
    from harness.owncam_drive import CARRY_POSTURE, LOOK_P20, SEARCH_POSE
    from scripts.zone_teacher import ZoneTeacherExecutor
    from sim.camera_robot_port import CameraRobotPort
    objects = scene.config['setup_only']['objects']
    spawns = scene.config['setup_only']['spawns']
    box_id = next(oid for oid, o in sorted(objects.items()) if o['kind'] == 'cyan')
    box_body = objects[box_id]['body_name']
    bx, by = objects[box_id]['position_m'][:2]
    rid = min(spawns, key=lambda r: math.hypot(spawns[r][0] - bx, spawns[r][1] - by))
    ports = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in world.controllers}
    port = ports[rid]
    m, d = world.model, world.data
    events = []
    executor = ZoneTeacherExecutor(world, ports, scene.config['static_map'], objects,
                                   lambda k, r, t, **x: events.append({'event': k, 'robot_id': r, 't': round(t, 3), **x}))

    def step(seconds, teacher=False):
        end = float(d.time) + seconds
        while float(d.time) < end:
            now = float(d.time)
            if teacher:
                executor.tick(now)
            for p in ports.values():
                p.tick(now)
            world._physics_step_for(world.controllers['r1'])

    def command(pose):
        """Issue the pose like the drivers do: at most 60 PWM per servo per 0.1 s (ARM_STEP_PWM)."""
        robot = world.controllers[rid]
        current = {int(k): float(v) for k, v in robot.servo_command_pulses.items()}
        target = {int(k): int(v) for k, v in pose.items()}
        while True:
            now = float(d.time)
            moved = False
            for servo, goal in sorted(target.items()):
                cur = current.get(servo, float(goal))
                if abs(goal - cur) < .5:
                    continue
                nxt = int(round(cur + max(-60., min(60., goal - cur))))
                current[servo] = float(nxt)
                moved = True
                if servo == 6:
                    port.apply({'kind': 'look', 'pan_pulse': nxt}, now)
                else:
                    port.apply({'kind': 'arm', 'servo_id': servo, 'pulse': nxt}, now)
            if not moved:
                return
            step(.1)

    lib = {'robot_id': rid, 'box_body': box_body}
    step(1.0)
    lib['SEARCH_unloaded'] = robot_state(world, rid)
    command(LOOK_P20)
    step(1.5)
    lib['LOOK_P20_unloaded'] = robot_state(world, rid)
    command({k: v for k, v in SEARCH_POSE.items() if k != 1})
    step(1.0)
    teacher = executor.robots[rid]
    slot = scene.config['static_map']['zone_slots']['A'][0]['center_m']
    teacher.assign({'job_id': 'grasp', 'box_body': box_body, 'slot_xy': slot}, float(d.time))
    t0 = float(d.time)
    while teacher.phase not in ('carry', 'done', 'failed') and float(d.time) - t0 < 400.:
        step(.05, teacher=True)
    if teacher.phase != 'carry':
        raise RuntimeError(f'teacher grasp failed: {teacher.phase} {teacher.outcome}')
    lib['teacher_sim_s'] = round(float(d.time) - t0, 3)
    port.hold(float(d.time))
    command(CARRY_POSTURE)
    step(2.0)
    lib['CARRY_loaded'] = robot_state(world, rid)
    lib['CARRY_loaded']['box_rel'] = box_relative(world, rid, box_body)
    lib['CARRY_loaded']['box_z_m'] = float(d.xpos[m.body(box_body).id][2])
    command(LOOK_P20)
    step(1.5)
    lib['LOOK_P20_loaded'] = robot_state(world, rid)
    lib['LOOK_P20_loaded']['box_rel'] = box_relative(world, rid, box_body)
    lib['LOOK_P20_loaded']['box_z_m'] = float(d.xpos[m.body(box_body).id][2])
    command(CARRY_POSTURE)
    step(1.5)
    lib['box_z_after_m'] = float(d.xpos[m.body(box_body).id][2])
    lib['held_after'] = lib['box_z_after_m'] > .045
    lib['teacher_events'] = events
    lib['sim_s'] = round(float(d.time), 3)
    log(f"posture library: robot {rid}, teacher {lib['teacher_sim_s']} s, box z {lib['box_z_after_m']:.3f}")
    return lib


# --------------------------------------------------------------------------- ST1

def st1(out, log):
    import mujoco
    from sim.research_dispatch_arena import digest
    from sim.zone_arena import MAP_DIR
    from sim.zone_landmarks import ENV_V3_TAGGED_MAPS, tagged_map
    rows = {}
    for name in ENV_V3_TAGGED_MAPS:
        raw = (MAP_DIR/(name + '.json')).read_bytes()
        static = tagged_map(name)
        scene, world = make_world(name, render=False)
        try:
            m = world.model
            names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '': g for g in range(m.ngeom)}
            wall_err = 0.
            for o in static['obstacles']:
                g = names['zone_' + o['id']]
                (cx, cy), (hx, hy), h = o['center_m'], o['half_extents_m'], o['height_m']
                wall_err = max(wall_err, float(np.abs(m.geom_pos[g] - [cx, cy, h/2]).max()),
                               float(np.abs(m.geom_size[g] - [hx, hy, h/2]).max()))
                assert (int(m.geom_contype[g]), int(m.geom_conaffinity[g])) == (1, 3)
            placement = static['landmarks']['placement']
            tag_err = 0.
            for t in static['landmarks']['tags']:
                g = names[f"tag_{t['id']:03d}_plate"]
                nx, ny = t['normal_xy']
                want = [t['center_m'][0] + nx*placement['plate_thickness_m']/2,
                        t['center_m'][1] + ny*placement['plate_thickness_m']/2, t['center_m'][2]]
                tag_err = max(tag_err, float(np.abs(m.geom_pos[g] - want).max()))
            tag_geoms = [g for n, g in names.items() if n.startswith('tag_')]
            visual_only = all((int(m.geom_contype[g]), int(m.geom_conaffinity[g])) == (0, 0) for g in tag_geoms)
            rows[name] = {'file_sha256': sha_bytes(raw), 'static_map_sha256': digest(static),
                          'robot_static_map_equals_file': scene.config['static_map'] == json.loads(raw),
                          'walls': len(static['obstacles']), 'wall_heights_m': sorted({o['height_m'] for o in static['obstacles']}),
                          'max_wall_geom_error_m': wall_err, 'tags': len(static['landmarks']['tags']),
                          'max_tag_plate_position_error_m': tag_err, 'tag_geoms': len(tag_geoms),
                          'manifest_tag_geoms': scene.manifest['tag_geoms'], 'tag_geoms_visual_only': visual_only,
                          'noslip_iterations': int(m.opt.noslip_iterations),
                          'weld_active': sum(int(x) for x in m.eq_active0) if m.neq else 0,
                          'wall_profile': static['wall_profile']['id'], 'scene_xml_sha256': scene.manifest['scene_xml_sha256']}
        finally:
            world.close()
        log(f"ST1 {name}: {rows[name]['tags']} tags, walls {rows[name]['wall_heights_m']}, "
            f"geom err {rows[name]['max_wall_geom_error_m']:.1e}/{rows[name]['max_tag_plate_position_error_m']:.1e}")
    ok = all(r['robot_static_map_equals_file'] and r['max_wall_geom_error_m'] < 1e-6 and r['max_tag_plate_position_error_m'] < 1e-4
             and r['tag_geoms'] == r['manifest_tag_geoms'] and r['tag_geoms_visual_only'] and r['wall_heights_m'] == [.4]
             and r['noslip_iterations'] == 10 and r['weld_active'] == 0 for r in rows.values())
    return {'pass': ok, 'maps': rows,
            'physics_identity': 'tests/test_zone_env_v3.py::V3SceneTests::test_v3_tags_do_not_change_physics (bitwise qpos)'}


# --------------------------------------------------------------------------- ST2 / ST3 (v3 door map)

def detect_frame(detector, seen, tags):
    dets = detector.detect(seen)
    return [{'id': int(x['id']), 'side_px': round(float(x['side_px']), 2),
             'range_m': round(float(np.linalg.norm(min(x['solutions'], key=lambda s: s['reproj_px'])['t_ct'])), 4),
             'center_px': np.mean(np.array(x['corners_px']), axis=0).round(2).tolist()}
            for x in dets if int(x['id']) in tags]


def ray_hits(world, origin, dirs, groups, bodyexclude=-1):
    import mujoco
    n = len(dirs)
    geomid = np.full(n, -1, np.int32)
    dist = np.full(n, -1., np.float64)
    mujoco.mj_multiRay(world.model, world.data, np.asarray(origin, float), np.ascontiguousarray(dirs, float).ravel(),
                       groups, 1, bodyexclude, geomid, dist, None, n, 1e3)
    return geomid, dist


def optical_dirs(norm_xy, rot):
    """World ray directions of normalized optical coordinates (MuJoCo camera frame = x right, y up, -z fwd)."""
    v = np.stack((norm_xy[:, 0], -norm_xy[:, 1], -np.ones(len(norm_xy))), axis=1)
    v = v @ rot.T
    return v/np.linalg.norm(v, axis=1, keepdims=True)


def pixel_norm(pixels):
    import cv2
    from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix
    k = scaled_camera_matrix(640, 480)
    dd = np.asarray(CAMERA_FISHEYE_D, float).reshape(4, 1)
    return cv2.fisheye.undistortPoints(np.asarray(pixels, float).reshape(-1, 1, 2), k, dd).reshape(-1, 2)


def st2_st3(world, scene, lib, out, log):
    import mujoco
    from harness.owncam_drive import WIDE_LOOK_PANS
    from harness.visual_arm import forward_grip
    from harness.wall_tags import TagDetector
    from harness.owncam_drive import CARRY_POSTURE
    from sim.zone_arena import layout
    static = scene.config['static_map']
    tags = {int(t['id']): t for t in static['landmarks']['tags']}
    detector = TagDetector.for_map(static)
    rid, box_body = lib['robot_id'], lib['box_body']
    clear_scene(world, scene, rid, keep_box=box_body)
    frames = out/'frames'
    frames.mkdir(parents=True, exist_ok=True)
    names = [mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(world.model.ngeom)]
    configs = {'SEARCH_unloaded': (lib['SEARCH_unloaded'], None), 'LOOK_P20_unloaded': (lib['LOOK_P20_unloaded'], None),
               'CARRY_loaded': (lib['CARRY_loaded'], (box_body, lib['CARRY_loaded']['box_rel'])),
               'LOOK_P20_loaded': (lib['LOOK_P20_loaded'], (box_body, lib['LOOK_P20_loaded']['box_rel']))}
    ray_check = {'tags_checked': 0, 'hit_own_tag': 0}

    def park_box():
        set_free(world, box_body, (60., 60., .016), (1., 0., 0., 0.))

    def view(name, state_key, base_xy, yaw, pan=None, save=False):
        state, box = configs[state_key]
        if box is None:
            park_box()
        place(world, rid, base_xy, yaw, state, pan=pan, box=box)
        jpeg, seen = wrist_jpeg(world, rid)
        det = detect_frame(detector, seen, tags)
        origin, rot = camera_pose(world, rid)
        if det:                                      # ray-model check: a ray through the detected centre hits that tag
            dirs = optical_dirs(pixel_norm([x['center_px'] for x in det]), rot)
            geomid, _ = ray_hits(world, origin, dirs, GEOMGROUP_WRIST)
            for x, g in zip(det, geomid):
                ray_check['tags_checked'] += 1
                ray_check['hit_own_tag'] += int(g >= 0 and names[g].startswith(f"tag_{x['id']:03d}_"))
        if save:
            (frames/f'{name}.jpg').write_bytes(jpeg)
        return det, origin

    # ST2: the door-frame site column site_00 (west face of wall_divider_1)
    site = next(s for s in static['landmarks']['sites'] if s['id'] == 'site_00')
    column = site['tag_ids']
    sx, sy = site['center_m']
    n_ang = math.atan2(site['normal_xy'][1], site['normal_xy'][0])
    st2 = []
    for state_key in configs:
        for bearing in ST2_BEARINGS_DEG:
            for dist in ST2_DISTANCES:
                phi = n_ang + math.radians(bearing)          # rotate the normal towards -y (south)
                cam = (sx + dist*math.cos(phi), sy + dist*math.sin(phi))
                yaw = math.atan2(sy - cam[1], sx - cam[0])
                state, box = configs[state_key]
                base = base_for_camera(world, rid, state, cam, yaw, box=box)
                det, origin = view(f'st2_{state_key}_b{int(bearing)}_d{dist:.1f}', state_key, base, yaw,
                                   save=dist in (.6, 1.5, 2.5, 3.5) and bearing == 0.)
                row = {'posture': state_key, 'bearing_deg': bearing, 'distance_m': dist, 'per_tag': {}}
                for tid in column:
                    t = tags[tid]
                    true_range = float(np.linalg.norm(np.array(t['center_m']) - origin))
                    hit = next((x for x in det if x['id'] == tid), None)
                    row['per_tag'][f"z{t['center_m'][2]:.2f}"] = (
                        None if hit is None else {'side_px': hit['side_px'], 'range_error_m': round(hit['range_m'] - true_range, 4),
                                                  'true_range_m': round(true_range, 4)})
                row['other_tags_detected'] = sorted(x['id'] for x in det if x['id'] not in column)
                st2.append(row)
    log(f'ST2 done: {len(st2)} views')

    # ST3: task stations
    arena = layout('zone_wide_door')
    door = next(p for p in static['passages'] if p['kind'] == 'door')
    dx, dy = door['center_m']
    gx = float(forward_grip(CARRY_POSTURE)[0])
    stations = []
    for y in arena['spawn_rows_y']:
        stations.append(('spawn', (arena['spawn_x'], y), 0., ('unloaded',), True))
    for x in arena['pickup_columns_x']:
        for y in arena['pickup_rows_y']:
            stations.append(('pickup_approach', (x - .40, y), 0., ('unloaded',), True))
    for label, xy in (('door_west_1.5', (dx - 1.5, dy)), ('door_west_0.6', (dx - .6, dy)), ('door_exit_W', (dx + .45, dy))):
        stations.append((label, xy, 0., ('unloaded', 'loaded'), True))
    for label, xy in (('return_east_0.6', (dx + .6, dy)), ('return_east_1.5', (dx + 1.5, dy))):
        stations.append((label, xy, math.pi, ('unloaded',), False))        # extra, not gated
    for zone, slots in static['zone_slots'].items():
        for s in slots:
            stations.append(('preplace_' + s['slot_id'], (s['center_m'][0] - gx, s['center_m'][1]), 0., ('loaded',), True))
    pans = tuple(dict.fromkeys(WIDE_LOOK_PANS))
    st3 = []
    for label, xy, yaw, loads, gated in stations:
        for load in loads:
            drive_key = 'SEARCH_unloaded' if load == 'unloaded' else 'CARRY_loaded'
            look_key = 'LOOK_P20_' + load
            views = {}
            det, _ = view(f'st3_{label}_{load}_drive', drive_key, xy, yaw, save=True)
            views['drive'] = {str(x['id']): x['range_m'] for x in det}
            for pan in pans:
                det, _ = view(f'st3_{label}_{load}_look{pan}', look_key, xy, yaw, pan=pan, save=pan == 1500)
                views[f'look_{pan}'] = {str(x['id']): x['range_m'] for x in det}
            seen_ids = sorted({int(i) for v in views.values() for i in v})
            ranges = [r for v in views.values() for r in v.values()]
            st3.append({'station': label, 'base_xy': [round(xy[0], 4), round(xy[1], 4)], 'yaw_rad': round(yaw, 4),
                        'load': load, 'gated': gated, 'views_pnp_range_m': views, 'distinct_tags': seen_ids,
                        'max_tags_in_one_view': max(len(v) for v in views.values()),
                        'nearest_pnp_range_m': min(ranges) if ranges else None,
                        'sites_seen': sorted({tags[i]['site'] for i in seen_ids}), 'pass': bool(seen_ids)})
        log(f"ST3 {label}: " + ', '.join(f"{r['load']} {len(r['distinct_tags'])}" for r in st3 if r['station'] == label))
    gate = all(r['pass'] for r in st3 if r['gated'])
    return {'site': site['id'], 'column_tag_ids': column, 'views': st2}, \
           {'pass': gate, 'stations': st3, 'preplace_offset_m': gx, 'look_pans': list(pans)}, ray_check


# --------------------------------------------------------------------------- ST4

def level_max_camera_posture(world, rid, max_pitch_deg=10.):
    from importlib import util
    spec = util.spec_from_file_location('camera_height', X/'camera_height.py')
    mod = util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    best = None
    for step, span in ((100, None), (10, 100)):
        centre = best[1] if best else None
        rng = (range(500, 2501, step) if span is None else None)
        grid = []
        if span is None:
            grid = [(a, b, c) for a in rng for b in rng for c in rng]
        else:
            off = range(-span, span + 1, step)
            grid = [(min(2500, max(500, centre[3] + a)), min(2500, max(500, centre[4] + b)), min(2500, max(500, centre[5] + c)))
                    for a in off for b in off for c in off]
        for a, b, c in grid:
            pose = {1: 1500, 3: a, 4: b, 5: c, 6: 1500}
            m = mod.measure(world, rid, pose)
            if abs(m['optical_pitch_deg']) <= max_pitch_deg and (best is None or m['camera_z_m'] > best[0]['camera_z_m']):
                best = (m, pose)
    return best


def st4(out, lib, log):
    import mujoco
    from sim.masterpi_camera_profile import raw_fisheye_remap, scaled_camera_matrix
    from sim.masterpi_production_v2 import CARRY_POSE
    frames = out/'frames'
    frames.mkdir(parents=True, exist_ok=True)
    k = scaled_camera_matrix(640, 480)
    map_x, map_y = raw_fisheye_remap(640, 480)
    valid = (map_x >= 0) & (map_x <= 639) & (map_y >= 0) & (map_y <= 479)
    vs, us = np.nonzero(valid)
    norm = np.stack(((map_x[vs, us] - k[0, 2])/k[0, 0], (map_y[vs, us] - k[1, 2])/k[1, 1]), axis=1)
    rows, extra = [], {}
    for name in (V2, V3):
        scene, world = make_world(name)
        try:
            rid = lib['robot_id']
            clear_scene(world, scene, rid)
            static = scene.config['static_map']
            divider = next(o for o in static['obstacles'] if o['id'] == 'wall_divider_1')
            east_face = divider['center_m'][0] + divider['half_extents_m'][0]
            west_face = divider['center_m'][0] - divider['half_extents_m'][0]
            door = next(p for p in static['passages'] if p['kind'] == 'door')
            d_lo, d_hi = door['center_m'][1] - door['width_m']/2, door['center_m'][1] + door['width_m']/2
            wall_h = divider['height_m']
            m, d = world.model, world.data
            gnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
            bodyname = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or '' for g in range(m.ngeom)]
            # probes in the east room: peer r2 with its arm straight up, a red box
            peer = next(r for r in world.controllers if r != rid)
            base = lib['SEARCH_unloaded']
            arm_up = commanded_state(world, peer, ARM_UP, base)
            place(world, peer, (east_face + .75, -1.6), math.pi, arm_up)
            red = next(o['body_name'] for _, o in sorted(scene.config['setup_only']['objects'].items()) if o['kind'] == 'red')
            set_free(world, red, (east_face + .35, -1.6, .016), (1., 0., 0., 0.))
            mujoco.mj_forward(m, d)
            if 'level_max' not in extra:
                best = level_max_camera_posture(world, rid)
                extra['level_max'] = {'pulses': {str(a): b for a, b in best[1].items()}, 'camera_z_m': best[0]['camera_z_m'],
                                      'optical_pitch_deg': best[0]['optical_pitch_deg'], 'search': 'PWM grid 100 then +-100 step 10'}
            postures = {'SEARCH_POSE': base,
                        'CARRY_POSE_level': commanded_state(world, rid, CARRY_POSE, base),
                        'look_p10': commanded_state(world, rid, LOOK_P10, base),
                        'level_max_camera': commanded_state(world, rid, {int(a): b for a, b in extra['level_max']['pulses'].items()}, base)}
            for pname, state in postures.items():
                for dist in ST4_DISTANCES:
                    cam = (west_face - dist, -1.6)
                    xy = base_for_camera(world, rid, state, cam, 0.)
                    place(world, rid, xy, 0., state)
                    jpeg, _ = wrist_jpeg(world, rid)
                    tag = f"st4_{name.split('_tags_')[1]}_{pname}_d{dist:.1f}"
                    (frames/f'{tag}.jpg').write_bytes(jpeg)
                    origin, rot = camera_pose(world, rid)
                    dirs = optical_dirs(norm, rot)
                    geomid, dist_hit = ray_hits(world, origin, dirs, GEOMGROUP_WRIST)
                    hit = dist_hit >= 0
                    pts = origin + dirs*np.where(hit, dist_hit, 0.)[:, None]
                    own = np.array([bodyname[g].startswith(rid + '__') if g >= 0 else False for g in geomid])
                    east = hit & (pts[:, 0] > east_face + 1e-3)
                    # where each ray crosses the divider plane (x = divider centre)
                    with np.errstate(divide='ignore', invalid='ignore'):
                        t = (divider['center_m'][0] - origin[0])/dirs[:, 0]
                    cross = (t > 0) & (~hit | (dist_hit > t))
                    zc = origin[2] + t*dirs[:, 2]
                    yc = origin[1] + t*dirs[:, 1]
                    over_wall = cross & (zc > wall_h) & ~((yc > d_lo) & (yc < d_hi))
                    through_door = cross & (zc <= wall_h) & (yc > d_lo) & (yc < d_hi) & (zc >= 0)
                    east_kinds = {}
                    for g in geomid[east]:
                        key = ('peer_robot' if bodyname[g].startswith(peer + '__') else 'probe_box' if bodyname[g] == red
                               else 'zone_paint_or_slot' if gnames[g].startswith(('zone_zone_', 'zone_slot_')) else
                               'wall' if gnames[g].startswith('zone_wall_') else 'tag' if gnames[g].startswith('tag_')
                               else 'floor' if gnames[g] == 'floor' else 'other')
                        east_kinds[key] = east_kinds.get(key, 0) + 1
                    rows.append({'map': name, 'wall_height_m': wall_h, 'posture': pname, 'camera_distance_m': dist,
                                 'camera_z_m': round(float(origin[2]), 4), 'valid_pixels': int(len(norm)),
                                 'own_robot_or_box_pixels': int(own.sum()), 'sky_pixels': int((~hit).sum()),
                                 'east_room_pixels': int(east.sum()), 'east_room_by_kind': east_kinds,
                                 'over_wall_pixels': int(over_wall.sum()), 'door_pixels': int(through_door.sum()),
                                 'frame': f'frames/{tag}.jpg', 'frame_sha256': sha_bytes(jpeg)})
                    log(f"ST4 {name} {pname} d{dist}: east {int(east.sum())} over {int(over_wall.sum())} door {int(through_door.sum())}")
        finally:
            world.close()
    v3 = [r for r in rows if r['map'] == V3]
    gate = all(r['east_room_pixels'] == 0 for r in v3 if r['door_pixels'] == 0) and all(r['door_pixels'] == 0 for r in v3)
    return {'pass': gate, 'views': rows, **extra,
            'probes': 'peer robot 0.75 m east of the divider with its arm straight up (highest robot geom), red box 0.35 m east'}


# --------------------------------------------------------------------------- ST5

def st5(out, log):
    import mujoco
    from sim.zone_tag_rule_v3 import pickup_bays
    results, maps_img = {}, {}
    for base in ('zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor'):
        for family, name in (('walls_0.10_v1', base + '_tags_v1'), ('walls_0.40_v3', base + '_tags_v3')):
            scene, world = make_world(name, render=False)
            try:
                clear_scene(world, scene, keep_robot=None)
                static = scene.config['static_map']
                m, d = world.model, world.data
                x0, x1, y0, y1 = static['bounds_m']
                xs = np.arange(x0 + .025 + GRID_M/2, x1 - .025, GRID_M)
                ys = np.arange(y0 + .025 + GRID_M/2, y1 - .025, GRID_M)
                gx, gy = np.meshgrid(xs, ys)
                walls = [o for o in static['obstacles'] if o.get('kind') == 'wall']
                inside_wall = np.zeros_like(gx, bool)
                for o in walls:
                    # 5 mm margin: grid points on a wall face (or inside a 2 mm tag plate) are not floor
                    (cx, cy), (hx, hy) = o['center_m'], o['half_extents_m']
                    inside_wall |= (np.abs(gx - cx) <= hx + .005) & (np.abs(gy - cy) <= hy + .005)
                cams = [c['name'] for c in static['top_cameras']] + ['cctv_warehouse']
                for z in (.003, .032):
                    pts = np.stack((gx.ravel(), gy.ravel(), np.full(gx.size, z)), axis=1)
                    seen = {}
                    blocker = {}
                    gnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
                    for cname in cams:
                        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cname)
                        c = np.array(d.cam_xpos[cid]); rot = np.array(d.cam_xmat[cid]).reshape(3, 3)
                        pc = (pts - c) @ rot
                        depth = -pc[:, 2]
                        f = 240./math.tan(math.radians(float(m.cam_fovy[cid]))/2)
                        with np.errstate(divide='ignore', invalid='ignore'):
                            u = 320. + f*pc[:, 0]/depth
                            v = 240. - f*pc[:, 1]/depth
                        in_img = (depth > 0) & (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
                        vec = pts - c
                        target = np.linalg.norm(vec, axis=1)
                        dirs = vec/target[:, None]
                        vis = np.zeros(len(pts), bool)
                        idx = np.nonzero(in_img)[0]
                        if len(idx):
                            geomid, dist = ray_hits(world, c, dirs[idx], GEOMGROUP_TOP)
                            ok = (dist < 0) | (dist >= target[idx] - 1e-3)
                            vis[idx] = ok
                            if cname != 'cctv_warehouse':
                                for i, g in zip(idx[~ok], geomid[~ok]):
                                    blocker.setdefault(int(i), set()).add(gnames[g])
                        seen[cname] = vis
                    tops = np.sum([seen[c] for c in cams[:-1]], axis=0)
                    regions = region_masks(static, gx, gy, pickup_bays)
                    floor = ~inside_wall.ravel()
                    row = {}
                    for rname, mask in regions.items():
                        mask = mask.ravel() & floor
                        if not mask.any():
                            continue
                        unseen = np.nonzero(mask & (tops == 0))[0]
                        walls_blocking = {}
                        for i in unseen:
                            for g in blocker.get(int(i), ()):
                                walls_blocking[g] = walls_blocking.get(g, 0) + 1
                        row[rname] = {'points': int(mask.sum()), 'top_visible_fraction': round(float((tops[mask] > 0).mean()), 5),
                                      'top_2plus_fraction': round(float((tops[mask] > 1).mean()), 5),
                                      'observer_visible_fraction': round(float(seen['cctv_warehouse'][mask].mean()), 5)}
                        if len(unseen):
                            row[rname]['unseen_points'] = int(len(unseen))
                            row[rname]['unseen_bbox_m'] = [round(float(pts[unseen, 0].min()), 3), round(float(pts[unseen, 0].max()), 3),
                                                           round(float(pts[unseen, 1].min()), 3), round(float(pts[unseen, 1].max()), 3)]
                            row[rname]['blocking_geoms'] = dict(sorted(walls_blocking.items(), key=lambda kv: -kv[1])[:6])
                    results.setdefault(base, {}).setdefault(family, {})[f'z{z:.3f}'] = row
                    if z == .003:
                        maps_img[(base, family)] = (tops.reshape(gx.shape), inside_wall)
                    log(f"ST5 {name} z{z}: floor {row['floor_all']['top_visible_fraction']:.4f}, "
                        f"min region {min(r['top_visible_fraction'] for r in row.values()):.4f}")
            finally:
                world.close()
    draw_coverage(maps_img, out/'top_coverage.png')
    return {'results': results, 'image': 'top_coverage.png',
            'cameras': 'four fixed TOPs (straight down, fovy 55, 640x480, z 2.5 m); cctv_warehouse is the viewer camera (not evaluation)'}


def region_masks(static, gx, gy, pickup_bays):
    def rect(c, h):
        return (np.abs(gx - c[0]) <= h[0]) & (np.abs(gy - c[1]) <= h[1])
    out = {'floor_all': np.ones_like(gx, bool)}
    for key, r in static['regions'].items():
        out[key] = rect(r['center_m'], r['half_extents_m'])
    slots = np.zeros_like(gx, bool)
    for zs in static['zone_slots'].values():
        for s in zs:
            slots |= rect(s['center_m'], s['half_extents_m'])
    out['zone_slots'] = slots
    for bay in pickup_bays(static):
        for s in bay['slots']:
            out['pickup_' + s['slot_id']] = rect(s['center_m'], s['half_extents_m'])
    for p in static.get('passages', []):
        if p['kind'] == 'door':
            out['door_region_' + p['id']] = rect(p['center_m'], (.6, .5))
        else:
            out[p['kind'] + '_' + p['id']] = rect(p['center_m'], p['half_extents_m'])
    from sim.zone_arena import layout
    arena = layout(static['base_map']['map_id'])
    spawn = np.zeros_like(gx, bool)
    for y in arena['spawn_rows_y']:
        spawn |= np.hypot(gx - arena['spawn_x'], gy - y) <= .2
    out['spawn_discs'] = spawn
    near = np.zeros_like(gx, bool)
    for o in static['obstacles']:
        if o['id'] in ('wall_north', 'wall_south', 'wall_west', 'wall_east'):
            continue
        (cx, cy), (hx, hy) = o['center_m'], o['half_extents_m']
        near |= (np.abs(gx - cx) <= hx + .30) & (np.abs(gy - cy) <= hy + .30)
    out['within_0.30m_of_interior_walls'] = near
    return out


def draw_coverage(maps_img, path):
    from PIL import Image, ImageDraw
    tiles = []
    for base in ('zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor'):
        row = []
        for family in ('walls_0.10_v1', 'walls_0.40_v3'):
            tops, wall = maps_img[(base, family)]
            img = np.zeros(tops.shape + (3,), np.uint8)
            img[tops == 0] = (220, 40, 40)
            img[tops == 1] = (240, 200, 60)
            img[tops >= 2] = (70, 170, 80)
            img[wall] = (40, 40, 40)
            im = Image.fromarray(img[::-1]).resize((tops.shape[1]*2, tops.shape[0]*2), Image.NEAREST)
            ImageDraw.Draw(im).text((6, 4), f'{base} {family}', fill=(0, 0, 0))
            row.append(np.asarray(im))
        tiles.append(np.concatenate(row, axis=1))
    Image.fromarray(np.concatenate(tiles, axis=0)).save(path)


# --------------------------------------------------------------------------- summary and media

def summarize(result):
    """Compact tables for the record (computed from the detailed results only)."""
    out = {}
    if 'ST2' in result:
        rows = {}
        for v in result['ST2']['views']:
            key = f"{v['posture']} bearing {int(v['bearing_deg'])}"
            for z, hit in v['per_tag'].items():
                r = rows.setdefault(key, {}).setdefault(z, {'detected_at_m': [], 'abs_range_error_m': {}})
                if hit is not None:
                    r['detected_at_m'].append(v['distance_m'])
                    r['abs_range_error_m'][str(v['distance_m'])] = abs(hit['range_error_m'])
        for key, zs in rows.items():
            for z, r in zs.items():
                r['max_detected_m'] = max(r['detected_at_m']) if r['detected_at_m'] else None
                near = [e for d, e in r['abs_range_error_m'].items() if float(d) <= 1.5]
                far = [e for d, e in r['abs_range_error_m'].items() if 2.0 <= float(d) <= 2.5]
                r['max_abs_error_le_1.5m'] = max(near) if near else None
                r['max_abs_error_2.0_2.5m'] = max(far) if far else None
        out['ST2'] = rows
    if 'ST3' in result:
        st = result['ST3']['stations']
        out['ST3'] = {'stations': len(st), 'gated': sum(r['gated'] for r in st), 'pass': sum(r['pass'] for r in st if r['gated']),
                      'min_distinct_tags': min((len(r['distinct_tags']), r['station'], r['load']) for r in st if r['gated']),
                      'max_nearest_range_m': max((r['nearest_pnp_range_m'] or 99, r['station'], r['load']) for r in st if r['gated'])}
    if 'ST4' in result:
        out['ST4'] = [{k: v[k] for k in ('map', 'posture', 'camera_distance_m', 'camera_z_m', 'east_room_pixels',
                                        'over_wall_pixels', 'door_pixels', 'valid_pixels')} for v in result['ST4']['views']]
    if 'ST5' in result:
        t = {}
        for base, fams in result['ST5']['results'].items():
            for fam, zs in fams.items():
                row = zs['z0.003']
                t[f'{base} {fam}'] = {k: row[k]['top_visible_fraction'] for k in row
                                      if k.startswith(('floor_all', 'door_region', 'corridor', 'passing', 'zone_', 'pickup', 'spawn',
                                                       'within'))}
                t[f'{base} {fam}']['observer_floor_all'] = row['floor_all']['observer_visible_fraction']
        out['ST5'] = t
    return out


def make_media(out, media):
    from PIL import Image
    media.mkdir(parents=True, exist_ok=True)
    frames = out/'frames'

    def mosaic(rows, path, size=(320, 240)):
        tiles = []
        for row in rows:
            ims = [np.asarray(Image.open(frames/f'{name}.jpg').convert('RGB').resize(size)) for name in row]
            tiles.append(np.concatenate(ims, axis=1))
        Image.fromarray(np.concatenate(tiles, axis=0)).save(path, quality=85)
        return path.name
    made = []
    made.append(mosaic([[f'st4_v2_{p}_d0.8' for p in ('SEARCH_POSE', 'look_p10', 'level_max_camera')],
                        [f'st4_v3_{p}_d0.8' for p in ('SEARCH_POSE', 'look_p10', 'level_max_camera')]],
                       media/'st4_wall_occlusion_v2_top_v3_bottom.jpg'))
    made.append(mosaic([['st3_door_west_1.5_loaded_drive', 'st3_door_west_1.5_loaded_look1500', 'st3_door_exit_W_loaded_look1500'],
                        ['st3_return_east_0.6_unloaded_look1500', 'st3_preplace_A2_loaded_look1500', 'st3_preplace_C2_loaded_look1500']],
                       media/'st3_station_views.jpg'))
    made.append(mosaic([[f'st2_LOOK_P20_loaded_b0_d{d:.1f}' for d in (.6, 1.5, 2.5, 3.5)]], media/'st2_distance_look_p20_loaded.jpg'))
    im = Image.open(out/'top_coverage.png')
    im.save(media/'st5_top_coverage.png', optimize=True)
    made.append('st5_top_coverage.png')
    return made


# --------------------------------------------------------------------------- main

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--output', default=str(DEFAULT_OUT))
    p.add_argument('--only', default='ST1,ST2,ST3,ST4,ST5')
    p.add_argument('--record', default='', help='write the compact summary here (and media/ next to it)')
    args = p.parse_args(argv)
    import cv2
    import mujoco
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    only = set(args.only.split(','))
    started, load_start = time.time(), os.getloadavg()
    lines = []

    def log(msg):
        lines.append(f'{time.time() - started:8.1f}s {msg}')
        print(lines[-1], flush=True)

    code = {'sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain', '--', 'sim', 'harness', 'scripts', 'maps',
                                                                'experiments/2026-09-26-zone-env-v3'))}
    result = {'schema': SCHEMA, 'code': code, 'seed': SEED, 'contact_profile': PROFILE,
              'note': 'diagnostic placement with simulator state; not a robot input, not a student result'}
    if 'ST1' in only:
        result['ST1'] = st1(out, log)
    lib = None
    if only & {'ST2', 'ST3', 'ST4'}:
        scene, world = make_world(V3)
        try:
            lib = posture_library(world, scene, log)
            (out/'posture_library.json').write_text(json.dumps(lib, indent=1) + '\n')
            result['posture_library'] = {k: lib[k] for k in ('robot_id', 'teacher_sim_s', 'box_z_after_m', 'held_after')}
            result['posture_library']['file_sha256'] = sha_bytes((out/'posture_library.json').read_bytes())
            if only & {'ST2', 'ST3'}:
                st2_res, st3_res, ray_check = st2_st3(world, scene, lib, out, log)
                result['ST2'], result['ST3'], result['ray_model_check'] = st2_res, st3_res, ray_check
        finally:
            world.close()
    if 'ST4' in only:
        result['ST4'] = st4(out, lib, log)
    if 'ST5' in only:
        result['ST5'] = st5(out, log)
    result['env'] = {'python': platform.python_version(), 'mujoco': mujoco.__version__, 'opencv': cv2.__version__,
                     'numpy': np.__version__, 'platform': platform.platform(),
                     'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                                'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}}
    result['load_average'] = {'start': [round(v, 2) for v in load_start], 'end': [round(v, 2) for v in os.getloadavg()]}
    result['wall_s'] = round(time.time() - started, 1)
    (out/'static_checks.json').write_text(json.dumps(result, indent=1, ensure_ascii=False) + '\n')
    (out/'static_checks.log').write_text('\n'.join(lines) + '\n')
    frames = sorted((out/'frames').glob('*.jpg')) if (out/'frames').exists() else []
    (out/'frames_index.json').write_text(json.dumps({f.name: sha_bytes(f.read_bytes()) for f in frames}, indent=0) + '\n')
    if args.record:
        record = Path(args.record)
        media = record.parent/'media'
        made = make_media(out, media) if only == {'ST1', 'ST2', 'ST3', 'ST4', 'ST5'} else []
        summary = {'schema': SCHEMA + '.summary', 'code': code, 'raw': str(out),
                   'raw_static_checks_sha256': sha_bytes((out/'static_checks.json').read_bytes()),
                   'raw_frames_index_sha256': sha_bytes((out/'frames_index.json').read_bytes()), 'frames': len(frames),
                   'gates': {k: result[k].get('pass') for k in ('ST1', 'ST3', 'ST4') if k in result},
                   'ray_model_check': result.get('ray_model_check'), 'posture_library': result.get('posture_library'),
                   'ST4_level_max_camera': result.get('ST4', {}).get('level_max'),
                   'tables': summarize(result), 'media': made, 'load_average': result['load_average'],
                   'wall_s': result['wall_s'], 'env': result['env']}
        record.write_text(json.dumps(summary, indent=1, ensure_ascii=False) + '\n')
    print(json.dumps({k: result[k].get('pass') for k in ('ST1', 'ST3', 'ST4') if k in result}))


if __name__ == '__main__':
    main()
