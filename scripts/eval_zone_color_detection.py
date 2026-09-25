"""Offline evaluation of zone box colour detection (own RGB + TOP RGB).

Two steps, both local and finite:

``render``  builds the standard ``ZoneScene`` (and the standard ``dispatch/open``
            scene for beam confusion) in synchronous SIM, re-poses robots and
            boxes per view, and writes the RGB frames a robot would receive
            (``frames/``) plus the commanded arm pose. Ground truth (poses,
            segmentation, visibility) goes ONLY to ``eval-labels/`` and is used
            for scoring, never by a detector.
``score``   runs the detectors on ``frames/`` and scores them against
            ``eval-labels/``.

Seeds come from the split file committed before any held-out scoring.
Cameras, robot appearance and box paint are never changed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SPLIT_FILE = ROOT/'experiments/2026-09-25-zone-rgb-color/split.json'
SCHEMA = 'ugrp.zone_color_eval.v1'
FOLDED = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
HOVER_Z_M = .095
# Fixed goal shape: two of every kind in the goal, spares drawn per seed.
EVAL_GOAL = {'A': {'red': 1, 'yellow': 1, 'cyan': 1}, 'B': {'green': 1, 'cyan': 1, 'red': 1},
             'C': {'yellow': 1, 'green': 1}}
VIEW_PLAN = (('layout', 2), ('approach', 4), ('zones', 2), ('edge', 1), ('occluded', 2),
             ('under_gripper', 1), ('empty', 1))
DISPATCH_VIEW_PLAN = (('dispatch_layout', 1), ('dispatch_approach', 3), ('dispatch_beam', 2))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_split():
    return json.loads(SPLIT_FILE.read_text())


# ---------------------------------------------------------------- geometry

def _yaw_quat(yaw):
    return [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]


def _to_base(pose, xy):
    x, y, yaw = pose
    dx, dy = xy[0]-x, xy[1]-y
    return (math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy)


def _from_base(pose, bxy):
    x, y, yaw = pose
    return (x+math.cos(yaw)*bxy[0]-math.sin(yaw)*bxy[1], y+math.sin(yaw)*bxy[0]+math.cos(yaw)*bxy[1])


def _box_ok_near_robot(pose, xy):
    """Keep boxes off the chassis; the free space ahead of the folded arm is allowed."""
    bx, by = _to_base(pose, xy)
    if bx >= .13 and abs(by) <= .10:
        return True
    return math.hypot(bx, by) >= .19


class Placer:
    def __init__(self, rng, bounds, margin=.14):
        self.rng = rng
        self.lo = (bounds[0]+margin, bounds[2]+margin)
        self.hi = (bounds[1]-margin, bounds[3]-margin)

    def inside(self, xy, margin=0.):
        return (self.lo[0]+margin <= xy[0] <= self.hi[0]-margin and
                self.lo[1]+margin <= xy[1] <= self.hi[1]-margin)

    def free_xy(self, boxes, robots, *, region=None, tries=400, min_box=.07):
        for _ in range(tries):
            if region is None:
                xy = (self.rng.uniform(self.lo[0], self.hi[0]), self.rng.uniform(self.lo[1], self.hi[1]))
            else:
                (cx, cy), (hx, hy) = region
                xy = (self.rng.uniform(cx-hx, cx+hx), self.rng.uniform(cy-hy, cy+hy))
            if not self.inside(xy):
                continue
            if all(math.dist(xy, b) >= min_box for b in boxes) and all(_box_ok_near_robot(p, xy) for p in robots):
                return xy
        return None


# ---------------------------------------------------------------- world

def build_world(scene_id, seed, rng):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    if scene_id == 'zones/zone_wide':
        from sim.zone_arena import episode
        from sim.zone_scene import ZoneScene
        extras = {k: rng.randint(0, 1) for k in ('cyan', 'green', 'red', 'yellow')}
        extras = {k: v for k, v in extras.items() if v}
        config = episode(scene_id.split('/', 1)[1], seed, goal=EVAL_GOAL, extra_boxes=extras)
        config['contact_solver_profile'] = 'local_contact_fine'
        config['extra_boxes'] = extras
        definition = ZoneScene.from_zone_config(config)
        world = MultiMasterPiProductionV2(seed=seed, width=960, height=720, render=True,
                                          warehouse_layout=definition.engine_layout, warehouse_cargo_ids=None,
                                          xml_transform=definition.transform)
        definition.setup(world)
        static = config['static_map']
        tops = [c['name'] for c in static['top_cameras']]
        boxes = {oid: {'kind': o['kind'], 'body': o['body_name'], 'joint': o['joint_name']}
                 for oid, o in config['setup_only']['objects'].items()}
        info = {'config': config, 'static_map': static, 'bounds': static['bounds_m'],
                'layout_xy': {oid: o['position_m'][:2] for oid, o in config['setup_only']['objects'].items()},
                'spawns': config['setup_only']['spawns'], 'scene_xml_sha256': definition.manifest['scene_xml_sha256']}
    elif scene_id == 'dispatch/open':
        from sim.session_config import validate_config
        from sim.session_scenes import Scene
        scene_cfg = validate_config({'version': 1, 'scene': {'layout': scene_id, 'seed': seed}})
        definition = Scene(scene_cfg['scene'], ROOT)
        world = MultiMasterPiProductionV2(seed=seed, width=960, height=720, render=True,
                                          warehouse_layout=definition.engine_layout,
                                          warehouse_cargo_ids=scene_cfg['scene']['cargo_ids'],
                                          xml_transform=definition.transform)
        definition.setup(world)
        static = definition.config['static_map']
        tops = ['cctv_top']
        boxes = {'dispatch_box': {'kind': 'cyan', 'body': 'dispatch_box', 'joint': 'dispatch_box_free'}}
        cargo = definition.config['setup_only']['cargo']
        info = {'config': definition.config, 'static_map': static, 'bounds': definition.bounds,
                'layout_xy': {'dispatch_box': cargo['box'][:2]}, 'beam_xy': cargo['beam'][:2],
                'spawns': definition.config['setup_only']['spawns'],
                'scene_xml_sha256': (definition.manifest or {}).get('scene_xml_sha256')}
    else:
        raise ValueError(scene_id)
    return world, tops, boxes, info


def _geom_table(world, boxes):
    import mujoco
    body_to_box = {b['body']: oid for oid, b in boxes.items()}
    table = {}
    for gid in range(world.model.ngeom):
        name = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ''
        body = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY, int(world.model.geom_bodyid[gid])) or ''
        root = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY,
                                 int(world.model.body_rootid[int(world.model.geom_bodyid[gid])])) or ''
        if body in body_to_box:
            table[gid] = {'category': 'box', 'ref': body_to_box[body], 'kind': boxes[body_to_box[body]]['kind']}
        elif root.endswith('__robot') or name.split('__')[0] in ('r1', 'r2', 'r3'):
            table[gid] = {'category': 'robot', 'ref': root.split('__')[0] or name.split('__')[0]}
        elif root == 'team_beam' or body.startswith('team_beam'):
            table[gid] = {'category': 'beam', 'ref': name}
        elif name == 'floor':
            table[gid] = {'category': 'floor', 'ref': name}
        elif name.startswith(('zone_wall', 'dispatch_wall')) or 'wall' in name:
            table[gid] = {'category': 'wall', 'ref': name}
        elif name.startswith(('zone_', 'dispatch_')):
            table[gid] = {'category': 'floor_paint', 'ref': name}
        else:
            table[gid] = {'category': 'other', 'ref': name or body}
    return table


def _segment(world, camera, rid=None):
    """Segmentation with the exact actor/observer scene options (eval only)."""
    import cv2
    import mujoco

    def job():
        with world.physics_lock, world.render_lock:
            if rid is not None:
                robot = world.robot(rid)
                robot._sync_real_camera_mount()
                renderer = world.renderer
                renderer.update_scene(world.data, camera=robot._n('robot_cam'),
                                      scene_option=robot._robot_sensor_scene_option)
            else:
                renderer = world.observer_renderer
                option = mujoco.MjvOption()
                option.geomgroup[:] = 1
                renderer.update_scene(world.data, camera=camera, scene_option=option)
            renderer.enable_segmentation_rendering()
            try:
                seg = renderer.render().copy()
            finally:
                renderer.disable_segmentation_rendering()
        ids = np.where(seg[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM), seg[..., 0], -1).astype(np.float32)
        if rid is not None:
            map_x, map_y = world.robot(rid)._robot_fisheye_map
            ids = cv2.remap(ids, map_x, map_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=-1)
        return ids.astype(np.int32)
    return world._render_executor.submit(job).result(timeout=60.)


def _camera_projector(world, camera_name, rid=None):
    """Pinhole (TOP) or pinhole+raw fisheye (own) projector from the true camera pose."""
    import cv2
    import mujoco
    from sim.masterpi_camera_profile import CAMERA_FISHEYE_D, scaled_camera_matrix
    name = world.robot(rid)._n('robot_cam') if rid is not None else camera_name
    cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    pos = np.array(world.data.cam_xpos[cid])
    mat = np.array(world.data.cam_xmat[cid]).reshape(3, 3)
    h, w = (world.height, world.width) if rid is not None else (world.observer_height, world.observer_width)
    fovy = float(world.model.cam_fovy[cid])

    def project(points):
        rel = (np.asarray(points, float)-pos) @ mat           # MuJoCo camera frame (-z forward)
        optical = np.column_stack((rel[:, 0], -rel[:, 1], -rel[:, 2]))
        if np.any(optical[:, 2] <= 1e-4):
            return None
        norm = optical[:, :2]/optical[:, 2:3]
        if rid is None:
            f = (h/2)/math.tan(math.radians(fovy)/2)
            return np.column_stack((norm[:, 0]*f+w/2-.5, norm[:, 1]*f+h/2-.5))
        k = scaled_camera_matrix(w, h)
        d = np.asarray(CAMERA_FISHEYE_D, np.float64).reshape(4, 1)
        return cv2.fisheye.distortPoints(norm.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    return project, (w, h)


def _box_corners(world, body, half):
    xpos = np.array(world.data.body(body).xpos)
    xmat = np.array(world.data.body(body).xmat).reshape(3, 3)
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    return xpos + (signs*np.asarray(half)) @ xmat.T, xpos, xmat


def _valid_own_region(world, rid):
    map_x, map_y = world.robot(rid)._robot_fisheye_map
    h, w = map_x.shape
    return (map_x >= 0) & (map_x <= w-1) & (map_y >= 0) & (map_y <= h-1)


def label_camera(world, seg, table, boxes, half, camera, rid=None, valid=None):
    """Per-box visibility from segmentation + expected footprint from projection."""
    import cv2
    project, (w, h) = _camera_projector(world, camera, rid)
    if valid is None:
        valid = np.ones((h, w), bool)
    edge = cv2.dilate((~valid).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    edge[:3, :] = edge[-3:, :] = True
    edge[:, :3] = edge[:, -3:] = True
    box_gids = {}
    for gid, row in table.items():
        if row['category'] == 'box':
            box_gids.setdefault(row['ref'], []).append(gid)
    rows = {}
    for oid, spec in boxes.items():
        corners, xpos, xmat = _box_corners(world, spec['body'], half)
        mask = np.isin(seg, box_gids.get(oid, []))
        visible = int(mask.sum())
        pixels = project(corners)
        expected, partial, top_center = 0, False, None
        if pixels is not None and np.all(np.isfinite(pixels)):
            hull = cv2.convexHull(pixels.astype(np.float32))
            big = np.zeros((h+400, w+400), np.uint8)
            cv2.fillConvexPoly(big, (hull.reshape(-1, 2)+200).round().astype(np.int32), 1)
            full_area = int(big.sum())
            inframe = big[200:200+h, 200:200+w].astype(bool) & valid
            expected = int(inframe.sum())
            partial = bool(expected < .97*full_area or (inframe & edge).any())
            tc = project((xpos + np.array([0, 0, half[2]]) @ np.eye(3))[None, :])
            top_center = [float(v) for v in tc[0]] if tc is not None else None
        ys, xs = np.nonzero(mask)
        yaw = math.atan2(xmat[1, 0], xmat[0, 0])
        row = {'kind': spec['kind'], 'world_xyz': [float(v) for v in xpos], 'yaw_rad': yaw,
               'visible_px': visible, 'expected_px': expected, 'partial_frame': partial,
               'visible_fraction': (visible/expected) if expected else 0.,
               'seg_centroid_px': [float(xs.mean()), float(ys.mean())] if visible else None,
               'seg_bbox_px': [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())] if visible else None,
               'projected_top_center_px': top_center}
        rows[oid] = row
    return rows


# ---------------------------------------------------------------- views

def _pose_all(world, robots, box_xy, boxes, arm, yaw_by_box):
    import mujoco
    for rid, (x, y, yaw) in robots.items():
        world.robot(rid).set_base_pose_for_test((x, y, .032355118817659255), yaw)
    for oid, xy in box_xy.items():
        jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, boxes[oid]['joint'])
        q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
        world.data.qpos[q:q+7] = [xy[0], xy[1], .016, *_yaw_quat(yaw_by_box.get(oid, 0.))]
        world.data.qvel[v:v+6] = 0
    mujoco.mj_forward(world.model, world.data)
    world._team_joint_move_servos(arm, .3, settle_s=.3)


def _robot_facing(target, d, lateral, yaw):
    """Robot pose whose base frame sees ``target`` at (d, lateral)."""
    x = target[0]-math.cos(yaw)*d+math.sin(yaw)*lateral
    y = target[1]-math.sin(yaw)*d-math.cos(yaw)*lateral
    return (x, y, yaw)


def plan_view(kind, rng, info, boxes, robot_ids):
    """Returns robots {rid: (x, y, yaw)}, box xy, box yaw, arm poses; or None."""
    placer = Placer(rng, info['bounds'])
    oids = list(boxes)
    box_xy, box_yaw, robots = {}, {}, {}
    arm = {r: dict(FOLDED) for r in robot_ids}
    for r in robot_ids:
        arm[r][6] = rng.choice((1500, 1500, 1420, 1580))

    def robot_ok(p):
        return placer.inside(p[:2], -.04) and all(math.dist(p[:2], q[:2]) >= .42 for q in robots.values())

    def scatter(rest, region=None):
        for oid in rest:
            xy = placer.free_xy(list(box_xy.values()), list(robots.values()), region=region)
            if xy is None:
                return False
            box_xy[oid] = xy
            box_yaw[oid] = rng.uniform(-math.pi/4, math.pi/4)
        return True

    if kind in ('layout', 'dispatch_layout'):
        for rid, s in info['spawns'].items():
            robots[rid] = (s[0]+rng.uniform(-.05, .05), s[1]+rng.uniform(-.05, .05), s[3]+rng.uniform(-.35, .35))
        box_xy = {oid: tuple(info['layout_xy'][oid]) for oid in oids}
        return robots, box_xy, box_yaw, arm
    static = info['static_map']
    pickup = static.get('regions', {}).get('pickup')
    pickup_region = ((pickup['center_m'], pickup['half_extents_m']) if pickup else None)
    rng.shuffle(oids)
    if kind in ('approach', 'edge', 'occluded', 'dispatch_approach'):
        n_targets = 1 if kind == 'occluded' else len(robot_ids)
        targets = oids[:min(n_targets, len(oids))]
        used = set(targets)
        for i, rid in enumerate(robot_ids[:len(targets)]):
            for _ in range(200):
                t = placer.free_xy(list(box_xy.values()), list(robots.values()), region=pickup_region if kind != 'dispatch_approach' else None)
                if t is None:
                    return None
                yaw = rng.uniform(-math.pi, math.pi)
                if kind == 'edge':
                    ang = rng.choice((-1, 1))*rng.uniform(math.radians(30), math.radians(55))
                    d = rng.uniform(.18, .45)
                    pose = _robot_facing(t, d*math.cos(ang), d*math.sin(ang), yaw)
                elif kind == 'occluded':
                    d = rng.uniform(.55, .75)
                    pose = _robot_facing(t, d, rng.uniform(-.04, .04), yaw)
                else:
                    pose = _robot_facing(t, rng.uniform(.14, .50), rng.uniform(-.07, .07), yaw)
                if robot_ok(pose) and all(_box_ok_near_robot(pose, b) for b in box_xy.values()):
                    robots[rid] = pose
                    box_xy[targets[i]] = t
                    box_yaw[targets[i]] = rng.uniform(-math.pi/4, math.pi/4)
                    break
            else:
                return None
            if kind in ('approach', 'dispatch_approach') and rng.random() < .6:
                # Adjacent boxes of other colours around the target.
                for oid in [o for o in oids if o not in used][:rng.randint(1, 2)]:
                    for _ in range(100):
                        a, r = rng.uniform(-math.pi, math.pi), rng.uniform(.055, .085)
                        xy = (t[0]+r*math.cos(a), t[1]+r*math.sin(a))
                        if (placer.inside(xy) and all(math.dist(xy, b) >= .055 for b in box_xy.values())
                                and all(_box_ok_near_robot(p, xy) for p in robots.values())):
                            box_xy[oid] = xy
                            box_yaw[oid] = rng.uniform(-math.pi/4, math.pi/4)
                            used.add(oid)
                            break
        if kind == 'occluded':
            # A second robot stands between the viewer and the target.
            viewer = robots[robot_ids[0]]
            t = box_xy[targets[0]]
            for _ in range(200):
                f = rng.uniform(.45, .6)
                mid = (viewer[0]+(t[0]-viewer[0])*f, viewer[1]+(t[1]-viewer[1])*f)
                lat = rng.uniform(-.06, .06)
                pose = (mid[0]-math.sin(viewer[2])*lat, mid[1]+math.cos(viewer[2])*lat, rng.uniform(-math.pi, math.pi))
                if placer.inside(pose[:2], -.04) and math.dist(pose[:2], viewer[:2]) >= .36 and \
                        all(_box_ok_near_robot(pose, b) for b in box_xy.values()):
                    robots[robot_ids[1]] = pose
                    break
            else:
                return None
            for rid in robot_ids[2:]:
                for _ in range(200):
                    xy = placer.free_xy([], list(robots.values()))
                    pose = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
                    if robot_ok(pose) and all(_box_ok_near_robot(pose, b) for b in box_xy.values()):
                        robots[rid] = pose
                        break
        if not scatter([o for o in oids if o not in box_xy], pickup_region if kind != 'dispatch_approach' else None):
            return None
    elif kind == 'zones':
        slots = [s for zs in static['zone_slots'].values() for s in zs]
        rng.shuffle(slots)
        n = min(len(oids), rng.randint(3, 6))
        for oid, slot in zip(oids[:n], slots):
            c = slot['center_m']
            box_xy[oid] = (c[0]+rng.uniform(-.02, .02), c[1]+rng.uniform(-.02, .02))
            box_yaw[oid] = rng.uniform(-math.pi/4, math.pi/4)
        in_zone = oids[:n]
        for rid in robot_ids:
            for _ in range(300):
                t = box_xy[rng.choice(in_zone)]
                pose = _robot_facing(t, rng.uniform(.18, .6), rng.uniform(-.1, .1), rng.uniform(-math.pi, math.pi))
                if robot_ok(pose) and all(_box_ok_near_robot(pose, b) for b in box_xy.values()):
                    robots[rid] = pose
                    break
            else:
                return None
        if not scatter(oids[n:], pickup_region):
            return None
    elif kind == 'under_gripper':
        from harness.visual_arm import solve_grip_ik
        t = placer.free_xy([], [], region=pickup_region)
        yaw = rng.uniform(-math.pi, math.pi)
        robots[robot_ids[0]] = _robot_facing(t, .155, 0., yaw)
        box_xy[oids[0]] = t
        box_yaw[oids[0]] = rng.uniform(-.3, .3)
        hover = solve_grip_ik(.155, 0., HOVER_Z_M, -75)
        arm[robot_ids[0]] = {**arm[robot_ids[0]], **{int(k): int(v) for k, v in hover.items()}}
        for rid in robot_ids[1:]:
            for _ in range(300):
                xy = placer.free_xy([], list(robots.values()))
                pose = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
                if robot_ok(pose) and _box_ok_near_robot(pose, t):
                    robots[rid] = pose
                    break
            else:
                return None
        if not scatter(oids[1:], pickup_region):
            return None
    elif kind == 'empty':
        # Robots look at empty floor, floor paint or a nearby robot; boxes elsewhere.
        zones = [r for k, r in static.get('regions', {}).items() if k.startswith('zone_')]
        for i, rid in enumerate(robot_ids):
            for _ in range(300):
                if zones and i < 2:
                    z = rng.choice(zones)
                    xy = (z['center_m'][0]+rng.uniform(-.6, .6), z['center_m'][1]+rng.uniform(-.6, .6))
                    yaw = math.atan2(z['center_m'][1]-xy[1], z['center_m'][0]-xy[0])
                else:
                    xy = placer.free_xy([], list(robots.values()))
                    yaw = rng.uniform(-math.pi, math.pi)
                pose = (xy[0], xy[1], yaw)
                if robot_ok(pose):
                    robots[rid] = pose
                    break
            else:
                return None
        # Keep boxes far outside every robot's forward 1.2 m.
        for oid in oids:
            for _ in range(400):
                xy = placer.free_xy(list(box_xy.values()), list(robots.values()), region=pickup_region)
                if xy is None:
                    return None
                if all(not (0 < _to_base(p, xy)[0] < 1.2 and abs(_to_base(p, xy)[1]) < 1.0) for p in robots.values()):
                    box_xy[oid] = xy
                    break
            else:
                box_xy[oid] = xy
    elif kind == 'dispatch_beam':
        # Face the beam (the main non-box coloured object) from 0.25-0.6 m.
        beam = info['beam_xy']
        for rid in robot_ids:
            for _ in range(300):
                pose = _robot_facing((beam[0]+rng.uniform(-.2, .2), beam[1]+rng.uniform(-.2, .2)),
                                     rng.uniform(.25, .6), rng.uniform(-.1, .1), rng.uniform(-math.pi, math.pi))
                if robot_ok(pose) and math.dist(pose[:2], beam) >= .3:
                    robots[rid] = pose
                    break
            else:
                return None
        if not scatter(oids):
            return None
    else:
        raise ValueError(kind)
    for rid in robot_ids:
        if rid in robots:
            continue
        for _ in range(300):
            xy = placer.free_xy([], list(robots.values()))
            if xy is None:
                return None
            pose = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
            if robot_ok(pose) and all(_box_ok_near_robot(pose, b) for b in box_xy.values()):
                robots[rid] = pose
                break
        else:
            return None
    beam = info.get('beam_xy')
    if beam is not None and (any(math.dist(p[:2], beam) < .30 for p in robots.values())
                             or any(math.dist(b, beam) < .16 for b in box_xy.values())):
        return None
    return robots, box_xy, box_yaw, arm


def render(args):
    import cv2
    split = load_split()
    specs = split['splits'][args.split]
    out = Path(args.output)
    (out/'frames').mkdir(parents=True, exist_ok=False)
    (out/'eval-labels').mkdir()
    from harness.zone_color_boxes import BOX_HALF_M
    load = {'start': os.getloadavg()[0]}
    manifest = {'schema': SCHEMA, 'split': args.split, 'split_file_sha256': sha256(SPLIT_FILE),
                'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'source_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
                'clock': 'synchronous SIM; render after settle', 'views': [], 'scenes': []}
    started = time.monotonic()
    for spec in specs:
        rng = random.Random(f"{spec['scene']}:{spec['seed']}")
        world, tops, boxes, info = build_world(spec['scene'], spec['seed'], rng)
        try:
            table = _geom_table(world, boxes)
            robot_ids = list(world.robot_ids)
            manifest['scenes'].append({'scene': spec['scene'], 'seed': spec['seed'],
                                       'scene_xml_sha256': info['scene_xml_sha256'],
                                       'boxes': {k: v['kind'] for k, v in boxes.items()}})
            plan = DISPATCH_VIEW_PLAN if spec['scene'] == 'dispatch/open' else VIEW_PLAN
            for kind, count in plan:
                for i in range(count):
                    planned = None
                    for _ in range(20):
                        planned = plan_view(kind, rng, info, boxes, robot_ids)
                        if planned is not None:
                            break
                    view_id = f"{spec['scene'].replace('/', '-')}-s{spec['seed']}-{kind}-{i}"
                    if planned is None:
                        manifest['views'].append({'view_id': view_id, 'skipped': 'no valid placement'})
                        continue
                    robots, box_xy, box_yaw, arm = planned
                    _pose_all(world, robots, box_xy, boxes, arm, box_yaw)
                    vdir = out/'frames'/view_id
                    vdir.mkdir()
                    labels = {'view_id': view_id, 'scene': spec['scene'], 'seed': spec['seed'], 'case': kind,
                              'robots': {}, 'cameras': {}, 'geom_table': {str(k): v for k, v in table.items()}}
                    actor = {'view_id': view_id, 'commanded_arm_pwm': {}, 'top_cameras': tops}
                    for cam in tops:
                        jpeg = world.render_team_jpeg(camera=cam, quality=95)
                        (vdir/f'{cam}.jpg').write_bytes(jpeg)
                        seg = _segment(world, cam)
                        labels['cameras'][cam] = {'boxes': label_camera(world, seg, table, boxes, BOX_HALF_M, cam)}
                        cv2.imwrite(str(out/'eval-labels'/f'{view_id}--{cam}.png'), (seg+1).astype(np.uint16))
                    for rid in robot_ids:
                        jpeg = world.render_jpeg(robot_id=rid, camera='robot_cam', quality=90)
                        (vdir/f'{rid}.jpg').write_bytes(jpeg)
                        actor['commanded_arm_pwm'][rid] = {str(k): int(v) for k, v in arm[rid].items()}
                        seg = _segment(world, None, rid)
                        rows = label_camera(world, seg, table, boxes, BOX_HALF_M, rid, rid,
                                            valid=_valid_own_region(world, rid))
                        robot = world.robot(rid)
                        xyz = robot.base_xyz()
                        pose = (float(xyz[0]), float(xyz[1]), float(robot.base_rpy()[2]))
                        for row in rows.values():
                            row['base_xy'] = list(_to_base(pose, row['world_xyz'][:2]))
                        labels['robots'][rid] = {'base_pose': list(pose)}
                        labels['cameras'][rid] = {'boxes': rows, 'own_robot': rid}
                        cv2.imwrite(str(out/'eval-labels'/f'{view_id}--{rid}.png'), (seg+1).astype(np.uint16))
                    if 'beam_xy' in info:
                        labels['beam_world_xyz'] = [float(v) for v in world.data.body('team_beam').xpos]
                    (vdir/'actor-inputs.json').write_text(json.dumps(actor, indent=1))
                    (out/'eval-labels'/f'{view_id}.json').write_text(json.dumps(labels))
                    manifest['views'].append({'view_id': view_id, 'case': kind, 'scene': spec['scene'],
                                              'seed': spec['seed']})
        finally:
            world.close()
    load['end'] = os.getloadavg()[0]
    manifest['load_average_1min'] = load
    manifest['wall_s'] = round(time.monotonic()-started, 1)
    (out/'manifest.json').write_text(json.dumps(manifest, indent=1))
    print(json.dumps({'views': len([v for v in manifest['views'] if 'skipped' not in v]),
                      'skipped': len([v for v in manifest['views'] if 'skipped' in v]),
                      'load_average_1min': load, 'wall_s': manifest['wall_s']}))



# ---------------------------------------------------------------- scoring

# zone_wide only; dispatch/open is a separate beam-confusion appendix (the
# zone scenes hide the beam). zone_open is retired and not evaluated.
GROUP_OF = {'zones/zone_wide': 'primary_zone_wide', 'dispatch/open': 'appendix_beam_confusion_dispatch_open'}
TOP_FULL_FRACTION = .6      # TOP boxes are ~10 px; rasterised footprints over-count edges
OWN_FULL_FRACTION = .8
MIN_VISIBLE_TOP_PX = 20
PIXEL_MATCH_PAD = 4


def _visibility(row, top):
    if row['visible_px'] < (MIN_VISIBLE_TOP_PX if top else 90):
        return 'not_visible'
    if row['partial_frame']:
        return 'partial_frame'
    return 'full' if row['visible_fraction'] >= (TOP_FULL_FRACTION if top else OWN_FULL_FRACTION) else 'occluded'


def _match(det_px, gt_rows, kind):
    hits = []
    for oid, row in gt_rows.items():
        if not row['seg_bbox_px']:
            continue
        x0, y0, x1, y1 = row['seg_bbox_px']
        if x0-PIXEL_MATCH_PAD <= det_px[0] <= x1+PIXEL_MATCH_PAD and y0-PIXEL_MATCH_PAD <= det_px[1] <= y1+PIXEL_MATCH_PAD:
            hits.append((row['kind'] != kind, row['visible_px'] == 0, oid))
    return sorted(hits)[0] if hits else None


def _bg_category(seg, table, px):
    h, w = seg.shape
    x, y = int(round(min(max(px[0], 0), w-1))), int(round(min(max(px[1], 0), h-1)))
    patch = seg[max(0, y-3):y+4, max(0, x-3):x+4].ravel()-1
    cats = [table.get(str(int(g)), {'category': 'none'})['category'] if g >= 0 else 'none' for g in patch]
    for c in ('beam', 'robot', 'floor_paint', 'wall', 'box'):
        if c in cats:
            return c
    return max(set(cats), key=cats.count)


def _stats(values):
    if not values:
        return None
    a = np.asarray(values, float)
    return {'n': int(a.size), 'median': float(np.median(a)), 'p90': float(np.percentile(a, 90)),
            'max': float(a.max()), 'mean': float(a.mean())}


PROFILE_SETS = {'baseline': ('own_production_v1', 'zone_perception_v1'),
                'zone': ('own_zone_v2', 'top_zone_v2')}


def score_views(frames_dir, profile_set='baseline'):
    """Per-detection records; detectors see frames + commanded arm pose only."""
    import cv2
    from harness.zone_color_boxes import KINDS, detect_own, detect_top
    own_profile, top_profile = PROFILE_SETS[profile_set]
    frames_dir = Path(frames_dir)
    manifest = json.loads((frames_dir/'manifest.json').read_text())
    records = []
    static_cache = {}
    for view in manifest['views']:
        if 'skipped' in view or view['scene'] not in GROUP_OF:
            continue
        vid = view['view_id']
        labels = json.loads((frames_dir/'eval-labels'/f'{vid}.json').read_text())
        actor = json.loads((frames_dir/'frames'/vid/'actor-inputs.json').read_text())
        key = (view['scene'], view['seed'])
        if key not in static_cache:
            static_cache[key] = _static_for(view['scene'], view['seed'])
        cameras = static_cache[key]
        table = labels['geom_table']
        for cam, lab in labels['cameras'].items():
            top = cam not in actor['commanded_arm_pwm']
            jpeg = (frames_dir/'frames'/vid/f'{cam}.jpg').read_bytes()
            seg = cv2.imread(str(frames_dir/'eval-labels'/f'{vid}--{cam}.png'), cv2.IMREAD_UNCHANGED).astype(np.int32)
            if top:
                h, w = seg.shape
                dets = []
                for d in detect_top(jpeg, cameras[cam], KINDS, profile=top_profile):
                    dets.append({**d, 'px': [d['pixel'][0]*w, d['pixel'][1]*h]})
            else:
                pose = {int(k): v for k, v in actor['commanded_arm_pwm'][cam].items()}
                dets = [{**d, 'px': d['pixel_centroid']} for d in detect_own(jpeg, pose, KINDS, profile=own_profile)['detections']]
            matched = {}
            for d in dets:
                hit = _match(d['px'], lab['boxes'], d['kind'])
                rec = {'view_id': vid, 'scene': view['scene'], 'group': GROUP_OF[view['scene']], 'case': view['case'],
                       'camera': cam, 'camera_type': 'top' if top else 'own', 'record': 'detection', 'kind': d['kind'],
                       'px': [float(v) for v in d['px']], 'range_class': d.get('range_class')}
                if hit and not hit[0]:
                    oid = hit[2]
                    row = lab['boxes'][oid]
                    if oid in matched:
                        rec['outcome'] = 'duplicate'
                    else:
                        matched[oid] = d
                        rec['outcome'] = 'tp'
                    rec['box'] = oid
                    if top:
                        tc = row['projected_top_center_px']
                        rec['pixel_error_px'] = float(math.dist(d['px'], tc)) if tc else None
                        rec['floor_error_m'] = float(math.dist(d['floor_xy_m'], row['world_xyz'][:2]))
                    elif d.get('estimated_box_center_base_m'):
                        rec['base_error_m'] = float(math.dist(d['estimated_box_center_base_m'][:2], row['base_xy']))
                        rec['gt_range_m'] = float(math.hypot(*row['base_xy']))
                elif hit:
                    rec['outcome'] = 'fp_colour_confusion'
                    rec['true_kind'] = lab['boxes'][hit[2]]['kind']
                    rec['box'] = hit[2]
                else:
                    rec['outcome'] = 'fp_background'
                    rec['background'] = _bg_category(seg, table, d['px'])
                    if rec['background'] == 'robot' and not top:
                        own = lab.get('own_robot')
                        pix = seg[int(min(max(d['px'][1], 0), seg.shape[0]-1)), int(min(max(d['px'][0], 0), seg.shape[1]-1))]-1
                        ref = table.get(str(int(pix)), {}).get('ref')
                        rec['background'] = 'own_robot' if ref == own else 'robot'
                records.append(rec)
            for oid, row in lab['boxes'].items():
                vis = _visibility(row, top)
                if vis == 'not_visible':
                    continue
                rec = {'view_id': vid, 'scene': view['scene'], 'group': GROUP_OF[view['scene']], 'case': view['case'],
                       'camera': cam, 'camera_type': 'top' if top else 'own', 'record': 'gt', 'kind': row['kind'],
                       'box': oid, 'visibility': vis, 'detected': oid in matched,
                       'visible_px': row['visible_px']}
                if not top:
                    rec['gt_range_m'] = float(math.hypot(*row['base_xy']))
                records.append(rec)
        # Team-level TOP merge (zone_perception.detect_all semantics).
        merged = _merged_top(frames_dir, vid, labels, cameras, actor, top_profile)
        records.extend(merged)
    return records, manifest


def _static_for(scene, seed):
    if scene.startswith('zones/'):
        from sim.zone_arena import authored_map
        static = authored_map(scene.split('/', 1)[1])
        return {c['name']: c for c in static['top_cameras']}
    from sim.session_config import validate_config
    from sim.session_scenes import Scene
    definition = Scene(validate_config({'version': 1, 'scene': {'layout': scene, 'seed': seed}})['scene'], ROOT)
    top = dict(definition.config['static_map']['top_camera'])
    top.setdefault('name', 'cctv_top')
    return {'cctv_top': top}


def _merged_top(frames_dir, vid, labels, cameras, actor, profile):
    from harness.zone_color_boxes import KINDS, detect_top
    tops = [c for c in actor['top_cameras']]
    rows = []
    for cam in tops:
        jpeg = (frames_dir/'frames'/vid/f'{cam}.jpg').read_bytes()
        for d in detect_top(jpeg, cameras[cam], KINDS, profile=profile):
            d['_off'] = math.hypot(d['pixel'][0]-.5, d['pixel'][1]-.5)
            rows.append(d)
    rows.sort(key=lambda r: r['_off'])
    kept = []
    for r in rows:
        if any(k['kind'] == r['kind'] and math.dist(k['floor_xy_m'], r['floor_xy_m']) < .05 for k in kept):
            continue
        kept.append(r)
    gt = {}
    for cam in tops:
        for oid, row in labels['cameras'][cam]['boxes'].items():
            vis = _visibility(row, True)
            best = gt.get(oid, {'visibility': 'not_visible'})['visibility']
            order = ['not_visible', 'occluded', 'partial_frame', 'full']
            if order.index(vis) >= order.index(best):
                gt[oid] = {'visibility': vis, 'kind': row['kind'], 'xy': row['world_xyz'][:2]}
    out = []
    used = set()
    for oid, g in gt.items():
        if g['visibility'] == 'not_visible':
            continue
        cand = sorted((math.dist(k['floor_xy_m'], g['xy']), i) for i, k in enumerate(kept)
                      if k['kind'] == g['kind'] and i not in used)
        det = cand[0] if cand and cand[0][0] <= .05 else None
        if det:
            used.add(det[1])
        out.append({'view_id': vid, 'scene': labels['scene'], 'group': GROUP_OF[labels['scene']], 'case': labels['case'],
                    'camera': 'merged_top', 'camera_type': 'top_merged', 'record': 'gt', 'kind': g['kind'], 'box': oid,
                    'visibility': g['visibility'], 'detected': bool(det), 'floor_error_m': det[0] if det else None})
    for i, k in enumerate(kept):
        if i in used:
            continue
        near = sorted((math.dist(k['floor_xy_m'], g['xy']), g['kind']) for g in gt.values())
        outcome = 'fp_background'
        true_kind = None
        if near and near[0][0] <= .05:
            outcome, true_kind = ('duplicate', None) if near[0][1] == k['kind'] else ('fp_colour_confusion', near[0][1])
        out.append({'view_id': vid, 'scene': labels['scene'], 'group': GROUP_OF[labels['scene']], 'case': labels['case'],
                    'camera': 'merged_top', 'camera_type': 'top_merged', 'record': 'detection', 'kind': k['kind'],
                    'outcome': outcome, 'true_kind': true_kind, 'floor_xy_m': k['floor_xy_m']})
    return out


def summarize(records, frames_per_group):
    from harness.zone_color_boxes import KINDS
    out = {}
    groups = sorted({r['group'] for r in records})
    for group in groups:
        g = {}
        for ctype in ('own', 'top', 'top_merged'):
            rs = [r for r in records if r['group'] == group and r['camera_type'] == ctype]
            gts = [r for r in rs if r['record'] == 'gt']
            dets = [r for r in rs if r['record'] == 'detection']
            per_kind = {}
            for kind in KINDS:
                row = {}
                for vis in ('full', 'partial_frame', 'occluded'):
                    sel = [r for r in gts if r['kind'] == kind and r['visibility'] == vis]
                    row[vis] = {'positives': len(sel), 'detected': sum(r['detected'] for r in sel),
                                'rate': (sum(r['detected'] for r in sel)/len(sel)) if sel else None}
                kd = [r for r in dets if r['kind'] == kind]
                row['detections'] = len(kd)
                row['tp'] = sum(r['outcome'] == 'tp' for r in kd) if ctype != 'top_merged' else None
                row['duplicates'] = sum(r['outcome'] == 'duplicate' for r in kd)
                conf = [r for r in kd if r['outcome'] == 'fp_colour_confusion']
                row['fp_colour_confusion'] = {k: sum(r['true_kind'] == k for r in conf) for k in KINDS if any(r['true_kind'] == k for r in conf)}
                bg = [r for r in kd if r['outcome'] == 'fp_background']
                row['fp_background'] = {c: sum(r.get('background') == c for r in bg) for c in sorted({str(r.get('background')) for r in bg})}
                if ctype == 'top':
                    row['pixel_error_px'] = _stats([r['pixel_error_px'] for r in kd if r['outcome'] == 'tp' and r.get('pixel_error_px') is not None])
                    row['floor_error_m'] = _stats([r['floor_error_m'] for r in kd if r['outcome'] == 'tp'])
                elif ctype == 'top_merged':
                    row['floor_error_m'] = _stats([r['floor_error_m'] for r in gts if r['kind'] == kind and r['detected']])
                else:
                    row['base_error_m'] = _stats([r['base_error_m'] for r in kd if r['outcome'] == 'tp' and 'base_error_m' in r])
                    row['base_error_m_within_0p5m'] = _stats([r['base_error_m'] for r in kd if r['outcome'] == 'tp' and 'base_error_m' in r and r['gt_range_m'] <= .5])
                    row['base_error_m_far_coarse'] = _stats([r['base_error_m'] for r in kd if r['outcome'] == 'tp' and 'base_error_m' in r and r['range_class'] == 'far_coarse'])
                    row['fp_by_range_class'] = {c: sum(r.get('range_class') == c for r in kd if r['outcome'].startswith('fp_')) for c in ('near', 'far_coarse')}
                    row['full_by_range_m'] = {}
                    for lo, hi in ((0, .6), (.6, 1.2), (1.2, 2.), (2., 9.)):
                        sel = [r for r in gts if r['kind'] == kind and r['visibility'] == 'full' and lo <= r['gt_range_m'] < hi]
                        row['full_by_range_m'][f'{lo}-{hi}'] = {'positives': len(sel), 'detected': sum(r['detected'] for r in sel)}
                per_kind[kind] = row
            fp = [r for r in dets if r['outcome'].startswith('fp_')]
            g[ctype] = {'frames': frames_per_group.get((group, ctype), 0), 'per_kind': per_kind,
                        'false_positives_total': len(fp),
                        'false_positives_per_frame': (len(fp)/frames_per_group[(group, ctype)]) if frames_per_group.get((group, ctype)) else None}
        out[group] = g
    return out


def score(args):
    frames_dir = Path(args.frames)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    load = {'start': os.getloadavg()[0]}
    profiles = args.profile_set or list(PROFILE_SETS)
    results = {}
    for profile in profiles:
        records, manifest = score_views(frames_dir, profile)
        frames = {}
        for view in manifest['views']:
            if 'skipped' in view or view['scene'] not in GROUP_OF:
                continue
            labels = json.loads((frames_dir/'eval-labels'/f"{view['view_id']}.json").read_text())
            group = GROUP_OF[view['scene']]
            for cam in labels['cameras']:
                ctype = 'own' if cam in labels['robots'] else 'top'
                frames[(group, ctype)] = frames.get((group, ctype), 0)+1
            frames[(group, 'top_merged')] = frames.get((group, 'top_merged'), 0)+1
        (out/f'records-{profile}.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
        results[profile] = summarize(records, frames)
    load['end'] = os.getloadavg()[0]
    summary = {'schema': SCHEMA+'.score', 'frames_dir': str(frames_dir), 'split': manifest['split'],
               'render_source_sha': manifest['source_sha'],
               'score_source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
               'score_source_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
               'load_average_1min': load, 'profile_sets': {k: {'own': PROFILE_SETS[k][0], 'top': PROFILE_SETS[k][1]} for k in profiles},
               'results': results}
    (out/'summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps({'written': str(out/'summary.json'), 'load_average_1min': load}))


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    r = sub.add_parser('render', help='render frames + eval labels for one split')
    r.add_argument('--split', choices=('dev', 'test'), required=True)
    r.add_argument('--output', type=Path, required=True)
    s = sub.add_parser('score', help='run detectors on rendered frames and score them')
    s.add_argument('--frames', type=Path, required=True, help='output directory of render')
    s.add_argument('--output', type=Path, required=True)
    s.add_argument('--profile-set', action='append', choices=sorted(PROFILE_SETS),
                   help='detector profile set(s) (own, top); default: all')
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if args.command == 'render':
        render(args)
    else:
        score(args)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
