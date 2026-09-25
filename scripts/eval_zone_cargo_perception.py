"""Offline evaluation of kind-aware TOP-RGB cargo detection (profile top_cargo_v1).

Two steps, both local and finite (same structure as eval_zone_color_detection):

``render``  builds ``zones/zone_wide`` with the catalogue cargo mixed in with
            the four box colours (``sim.zone_cargo_scene.CargoZoneScene``, weld
            OFF), re-poses robots, boxes and cargo per view in synchronous SIM
            and writes the TOP JPEGs the robots receive (``frames/``). Ground
            truth (poses, segmentation, visibility, grip points) goes ONLY to
            ``eval-labels/`` and is used for scoring, never by a detector.
``score``   runs the detectors on ``frames/`` only and scores them against
            ``eval-labels/``.

Seeds come from the split file committed before any held-out rendering.
Cameras, robot appearance, cargo and box paint are never changed.
"""
from __future__ import annotations

import argparse
import itertools
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

from scripts.eval_zone_color_detection import (  # noqa: E402
    EVAL_GOAL, FOLDED, HOVER_Z_M, _camera_projector, _segment, _yaw_quat, sha256)

SPLIT_FILE = ROOT/'experiments/2026-09-25-zone-cargo-perception/split.json'
SCHEMA = 'ugrp.zone_cargo_perception_eval.v1'
SCENE = 'zones/zone_wide'
CARGO_SET = (('can1', 'can'), ('can2', 'can'), ('tile1', 'tile'), ('tile2', 'tile'),
             ('beam1', 'long_beam'), ('beam2', 'long_beam'), ('crate1', 'heavy_crate'),
             ('crate2', 'heavy_crate'), ('frame1', 'tri_frame'))
VIEW_PLAN = (('scatter', 2), ('paint', 2), ('near_robot', 2), ('formation', 2), ('adjacent', 1),
             ('straddle', 1), ('robots_on_paint', 1))
ROBOT_RADIUS_M = .17
SYMMETRY = {'can': None, 'tile': 180., 'long_beam': 180., 'heavy_crate': 180., 'tri_frame': 120.}


def load_split():
    return json.loads(SPLIT_FILE.read_text())


def _cls(kind, colour=None):
    return f'box_{colour}' if kind == 'box' else kind


# ---------------------------------------------------------------- footprints

def _discs(kind, pose):
    """World discs (x, y, r) covering an item's footprint."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    if kind == 'box':
        local = [(0., 0., .03)]
    elif kind == 'can':
        local = [(0., 0., .025)]
    elif kind == 'tile':
        local = [(0., 0., .037)]
    elif kind == 'heavy_crate':
        local = [(-.07, 0., .06), (0., 0., .06), (.07, 0., .06), (-.10, 0., .035), (.10, 0., .035)]
    elif kind == 'long_beam':
        local = [(t, 0., .03) for t in np.linspace(-.28, .28, 9)]
    elif kind == 'tri_frame':
        local = []
        verts = [(.2*math.cos(a), .2*math.sin(a)) for a in (0., 2*math.pi/3, 4*math.pi/3)]
        for i in range(3):
            (x0, y0), (x1, y1) = verts[i], verts[(i+1) % 3]
            local += [(x0+(x1-x0)*f, y0+(y1-y0)*f, .03) for f in np.linspace(0, 1, 7)]
            local.append((1.1*x0, 1.1*y0, .04))
    else:
        raise ValueError(kind)
    return [(x+c*a-s*b, y+s*a+c*b, r) for a, b, r in local]


class Layout:
    def __init__(self, rng, bounds, margin=.08):
        self.rng = rng
        self.lo = (bounds[0]+margin, bounds[2]+margin)
        self.hi = (bounds[1]-margin, bounds[3]-margin)
        self.items = {}        # oid -> (kind, pose)
        self.robots = {}       # rid -> pose

    def _inside(self, discs):
        return all(self.lo[0]+r <= x <= self.hi[0]-r and self.lo[1]+r <= y <= self.hi[1]-r for x, y, r in discs)

    def item_ok(self, kind, pose, *, gap=.015, ignore_robots=(), ignore_items=()):
        discs = _discs(kind, pose)
        if not self._inside(discs):
            return False
        for oid, (k, p) in self.items.items():
            if oid in ignore_items:
                continue
            for x, y, r in _discs(k, p):
                if any(math.hypot(x-a, y-b) < r+q+gap for a, b, q in discs):
                    return False
        for rid, p in self.robots.items():
            if rid in ignore_robots:
                continue
            if any(math.hypot(p[0]-a, p[1]-b) < ROBOT_RADIUS_M+q for a, b, q in discs):
                return False
        return True

    def robot_ok(self, pose, *, ignore_items=()):
        if not (self.lo[0]+.1 <= pose[0] <= self.hi[0]-.1 and self.lo[1]+.1 <= pose[1] <= self.hi[1]-.1):
            return False
        if any(math.dist(pose[:2], q[:2]) < .42 for q in self.robots.values()):
            return False
        for oid, (k, p) in self.items.items():
            if oid in ignore_items:
                continue
            if any(math.hypot(pose[0]-a, pose[1]-b) < ROBOT_RADIUS_M+r for a, b, r in _discs(k, p)):
                return False
        return True

    def random_xy(self, region=None):
        if region is None:
            return (self.rng.uniform(*[self.lo[0], self.hi[0]]), self.rng.uniform(*[self.lo[1], self.hi[1]]))
        (cx, cy), (hx, hy) = region
        return (self.rng.uniform(cx-hx, cx+hx), self.rng.uniform(cy-hy, cy+hy))

    def place(self, oid, kind, *, region=None, tries=400, yaw=None, **kw):
        for _ in range(tries):
            xy = self.random_xy(region)
            pose = (xy[0], xy[1], self.rng.uniform(-math.pi, math.pi) if yaw is None else yaw)
            if self.item_ok(kind, pose, **kw):
                self.items[oid] = (kind, pose)
                return pose
        return None


# ---------------------------------------------------------------- world

def build_world(seed, rng):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo_scene import CargoZoneScene
    extras = {k: rng.randint(0, 1) for k in ('cyan', 'green', 'red', 'yellow')}
    extras = {k: v for k, v in extras.items() if v}
    # Setup-only start poses in a row along the north wall; re-posed per view.
    cargo = [{'item_id': iid, 'kind': kind, 'pose': [-.4+.62*i, 1.05, 0.]} for i, (iid, kind) in enumerate(CARGO_SET)]
    scene = CargoZoneScene.from_cargo_config('zone_wide', seed, cargo=cargo, goal=EVAL_GOAL, extra_boxes=extras)
    world = MultiMasterPiProductionV2(seed=seed, width=960, height=720, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    static = scene.config['static_map']
    items = {}
    for oid, o in scene.config['setup_only']['objects'].items():
        items[oid] = {'kind': 'box', 'colour': o['kind'], 'class': _cls('box', o['kind']),
                      'body': o['body_name'], 'joint': o['joint_name'], 'z': .016}
    for inst in scene.cargo:
        items[inst.item_id] = {'kind': inst.kind, 'colour': None, 'class': inst.kind, 'body': inst.body,
                               'joint': inst.joint, 'z': .0005}
    info = {'static_map': static, 'bounds': static['bounds_m'], 'spawns': scene.config['setup_only']['spawns'],
            'layout_xy': {oid: o['position_m'][:2] for oid, o in scene.config['setup_only']['objects'].items()},
            'scene_xml_sha256': scene.manifest['scene_xml_sha256'],
            'catalogue_sha256': scene.config['cargo_set']['catalogue_sha256'], 'extra_boxes': extras}
    return world, items, info


def _geom_table(world, items):
    import mujoco
    body_to_item = {v['body']: oid for oid, v in items.items()}
    table = {}
    for gid in range(world.model.ngeom):
        name = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ''
        body = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY, int(world.model.geom_bodyid[gid])) or ''
        root = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY,
                                 int(world.model.body_rootid[int(world.model.geom_bodyid[gid])])) or ''
        if body in body_to_item:
            oid = body_to_item[body]
            table[gid] = {'category': 'item', 'ref': oid, 'class': items[oid]['class']}
        elif root.endswith('__robot') or name.split('__')[0] in ('r1', 'r2', 'r3'):
            table[gid] = {'category': 'robot', 'ref': root.split('__')[0] or name.split('__')[0]}
        elif name == 'floor':
            table[gid] = {'category': 'floor', 'ref': name}
        elif 'wall' in name:
            table[gid] = {'category': 'wall', 'ref': name}
        elif name.startswith('zone_zone_'):
            table[gid] = {'category': 'paint_' + name[10:], 'ref': name}
        elif name.startswith('zone_slot_'):
            table[gid] = {'category': 'paint_' + name[10], 'ref': name}
        elif name == 'zone_pickup':
            table[gid] = {'category': 'paint_pickup', 'ref': name}
        else:
            table[gid] = {'category': 'other', 'ref': name or body}
    return table


def _pose_all(world, robots, item_poses, items, arm):
    import mujoco
    for rid, (x, y, yaw) in robots.items():
        world.robot(rid).set_base_pose_for_test((x, y, .032355118817659255), yaw)
    for oid, (x, y, yaw) in item_poses.items():
        jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, items[oid]['joint'])
        q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
        world.data.qpos[q:q+7] = [x, y, items[oid]['z'], *_yaw_quat(yaw)]
        world.data.qvel[v:v+6] = 0
    mujoco.mj_forward(world.model, world.data)
    world._team_joint_move_servos(arm, .3, settle_s=.3)


# ---------------------------------------------------------------- views

def _facing(target, d, lateral, yaw):
    return (target[0]-math.cos(yaw)*d+math.sin(yaw)*lateral, target[1]-math.sin(yaw)*d-math.cos(yaw)*lateral, yaw)


def _hover():
    from harness.visual_arm import solve_grip_ik
    return {int(k): int(v) for k, v in solve_grip_ik(.155, 0., HOVER_Z_M, -75).items()}


def plan_view(case, index, rng, info, items, robot_ids):
    """Robots {rid: pose}, item poses {oid: (x, y, yaw)}, arm poses; or None."""
    static = info['static_map']
    lay = Layout(rng, info['bounds'])
    arm = {r: dict(FOLDED) for r in robot_ids}
    for r in robot_ids:
        arm[r][6] = rng.choice((1500, 1500, 1420, 1580))
    boxes = [o for o, v in items.items() if v['kind'] == 'box']
    cargo = [o for o, v in items.items() if v['kind'] != 'box']
    regions = static['regions']
    zone = {k: (r['center_m'], r['half_extents_m']) for k, r in regions.items()}
    pickup = zone['pickup']
    cams = static['top_cameras']
    xs = sorted({c['position_m'][0] for c in cams})
    ys = sorted({c['position_m'][1] for c in cams})
    seam_x, seam_y = (xs[0]+xs[-1])/2, (ys[0]+ys[-1])/2

    def spawn_robots(jitter=True):
        for rid, s in info['spawns'].items():
            pose = (s[0]+rng.uniform(-.05, .05), s[1]+rng.uniform(-.05, .05), s[3]+rng.uniform(-.35, .35))
            if not lay.robot_ok(pose):
                return False
            lay.robots[rid] = pose
        return True

    def free_robots():
        for rid in robot_ids:
            if rid in lay.robots:
                continue
            for _ in range(300):
                xy = lay.random_xy()
                pose = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
                if lay.robot_ok(pose):
                    lay.robots[rid] = pose
                    break
            else:
                return False
        return True

    def boxes_at_layout():
        for oid in boxes:
            x, y = info['layout_xy'][oid]
            pose = (x, y, rng.uniform(-math.pi/4, math.pi/4))
            if not lay.item_ok('box', pose, gap=0.):
                return False
            lay.items[oid] = ('box', pose)
        return True

    def scatter(oids, region=None):
        for oid in oids:
            if oid in lay.items:
                continue
            kind = items[oid]['kind']
            yaw = rng.uniform(-math.pi/4, math.pi/4) if kind == 'box' else None
            if lay.place(oid, kind, region=region, yaw=yaw) is None and lay.place(oid, kind, yaw=yaw) is None:
                return False
        return True

    if case == 'scatter':
        if not (spawn_robots() and boxes_at_layout() and scatter(cargo)):
            return None
    elif case == 'paint':
        # Cargo mostly on zone paint (C twice as likely: purple is closest to violet/magenta/pink).
        names = ['zone_C', 'zone_C', 'zone_A', 'zone_B', 'pickup']
        for oid in rng.sample(cargo, len(cargo)):
            kind = items[oid]['kind']
            region = zone[rng.choice(names)] if rng.random() < .8 else None
            if lay.place(oid, kind, region=region) is None and lay.place(oid, kind) is None:
                return None
        if not (scatter(boxes, pickup) and free_robots()):
            return None
    elif case == 'near_robot':
        targets = rng.sample(cargo, len(robot_ids))
        for rid, oid in zip(robot_ids, targets):
            kind = items[oid]['kind']
            if lay.place(oid, kind) is None:
                return None
            _, pose = lay.items[oid]
            for _ in range(400):
                a = rng.uniform(-math.pi, math.pi)
                d = rng.uniform(.0, .06)
                # robot centre just outside the item footprint on a random bearing
                ref = rng.choice(_discs(kind, pose))
                dist = ROBOT_RADIUS_M + ref[2] + d
                rpose = (ref[0]+dist*math.cos(a), ref[1]+dist*math.sin(a), rng.uniform(-math.pi, math.pi))
                if lay.robot_ok(rpose):
                    lay.robots[rid] = rpose
                    if rng.random() < .5:
                        arm[rid] = {**arm[rid], **_hover()}
                    break
            else:
                return None
        if not (scatter(boxes, pickup) and scatter(cargo)):
            return None
    elif case == 'formation':
        from sim.zone_cargo import CargoInstance, world_grasps
        pool = [o for o in cargo if items[o]['kind'] in ('long_beam', 'heavy_crate')] if index % 2 == 0 else \
            [o for o in cargo if items[o]['kind'] == 'tri_frame']
        oid = rng.choice(pool)
        kind = items[oid]['kind']
        for _ in range(200):
            lay.items.pop(oid, None)
            lay.robots.clear()
            if lay.place(oid, kind) is None:
                return None
            pose = lay.items[oid][1]
            grasps = world_grasps(CargoInstance(oid, kind, pose))
            poses = [g['base_xyyaw'] for g in grasps.values()]
            if all(lay.robot_ok(p, ignore_items=(oid,)) for p in poses) and \
                    all(math.dist(p[:2], q[:2]) >= .30 for p, q in itertools.combinations(poses, 2)):
                for rid, p in zip(robot_ids, poses):
                    lay.robots[rid] = tuple(p)
                    arm[rid] = {**arm[rid], **_hover()}
                break
        else:
            return None
        if not (free_robots() and scatter(boxes, pickup) and scatter(cargo)):
            return None
    elif case == 'adjacent':
        # Boxes 5-10 cm from small cargo and crate edges (similar-size neighbours).
        small = [o for o in cargo if items[o]['kind'] in ('can', 'tile', 'heavy_crate')]
        free = list(boxes)
        rng.shuffle(free)
        for oid in small:
            kind = items[oid]['kind']
            if lay.place(oid, kind, region=pickup if rng.random() < .5 else None) is None:
                return None
            _, pose = lay.items[oid]
            for _ in range(rng.randint(1, 2)):
                if not free:
                    break
                b = free.pop()
                ref = rng.choice(_discs(kind, pose))
                for _ in range(200):
                    a = rng.uniform(-math.pi, math.pi)
                    dist = ref[2] + .03 + rng.uniform(.005, .04)
                    bp = (ref[0]+dist*math.cos(a), ref[1]+dist*math.sin(a), rng.uniform(-math.pi/4, math.pi/4))
                    if lay.item_ok('box', bp, gap=.004):
                        lay.items[b] = ('box', bp)
                        break
        if not (scatter(boxes, pickup) and scatter(cargo) and free_robots()):
            return None
    elif case == 'straddle':
        # Items across the TOP view seams (x = seam_x or y = seam_y).
        for oid in rng.sample(cargo, len(cargo)):
            kind = items[oid]['kind']
            for _ in range(300):
                if rng.random() < .5:
                    xy = (seam_x+rng.uniform(-.12, .12), rng.uniform(lay.lo[1], lay.hi[1]))
                else:
                    xy = (rng.uniform(lay.lo[0], lay.hi[0]), seam_y+rng.uniform(-.12, .12))
                pose = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
                if lay.item_ok(kind, pose):
                    lay.items[oid] = (kind, pose)
                    break
        if not (scatter(cargo) and boxes_at_layout() and free_robots()):
            return None
    elif case == 'robots_on_paint':
        for rid, name in zip(robot_ids, ('zone_C', rng.choice(('zone_A', 'zone_B', 'zone_C')), 'pickup')):
            for _ in range(300):
                xy = lay.random_xy(zone[name])
                pose = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
                if lay.robot_ok(pose):
                    lay.robots[rid] = pose
                    if rng.random() < .5:
                        arm[rid] = {**arm[rid], **_hover()}
                    break
            else:
                return None
        if not (boxes_at_layout() and scatter(cargo)):
            return None
    else:
        raise ValueError(case)
    if not free_robots():
        return None
    poses = {oid: pose for oid, (_, pose) in lay.items.items()}
    if set(poses) != set(items):
        return None
    return dict(lay.robots), poses, arm


# ---------------------------------------------------------------- labels

def _geom_polygon(world, gid, project):
    """Projected convex outline of one geom (box / cylinder)."""
    import mujoco
    gtype = int(world.model.geom_type[gid])
    size = np.array(world.model.geom_size[gid])
    pos = np.array(world.data.geom_xpos[gid])
    mat = np.array(world.data.geom_xmat[gid]).reshape(3, 3)
    if gtype == int(mujoco.mjtGeom.mjGEOM_BOX):
        local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)*size
    elif gtype == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        a = np.linspace(0, 2*math.pi, 16, endpoint=False)
        ring = np.column_stack([np.cos(a)*size[0], np.sin(a)*size[0]])
        local = np.vstack([np.column_stack([ring, np.full(16, z)]) for z in (-size[1], size[1])])
    else:
        return None
    return project(pos + local @ mat.T)


def label_top(world, seg, table, items, camera):
    import cv2
    project, (w, h) = _camera_projector(world, camera)
    item_gids = {}
    for gid, row in table.items():
        if row['category'] == 'item':
            item_gids.setdefault(row['ref'], []).append(gid)
    rows = {}
    for oid, spec in items.items():
        gids = item_gids.get(oid, [])
        big = np.zeros((h+800, w+800), np.uint8)
        for gid in gids:
            if world.model.geom_rgba[gid][3] <= 0:
                continue
            px = _geom_polygon(world, gid, project)
            if px is None or not np.all(np.isfinite(px)):
                continue
            hull = cv2.convexHull(px.astype(np.float32)).reshape(-1, 2)
            cv2.fillConvexPoly(big, (hull+400).round().astype(np.int32), 1)
        full = int(big.sum())
        inframe = big[400:400+h, 400:400+w].astype(bool)
        expected = int(inframe.sum())
        mask = np.isin(seg, gids)
        visible = int(mask.sum())
        edge = np.zeros((h, w), bool)
        edge[:4, :] = edge[-4:, :] = True
        edge[:, :4] = edge[:, -4:] = True
        hull_px = None
        if visible:
            ys, xs = np.nonzero(mask)
            hull_px = cv2.convexHull(np.column_stack([xs, ys]).astype(np.float32)).reshape(-1, 2).round(1).tolist()
        rows[oid] = {'class': spec['class'], 'visible_px': visible, 'expected_px': expected,
                     'partial_frame': bool(expected and (expected < .97*full or (inframe & edge).any())),
                     'visible_fraction': visible/expected if expected else 0., 'hull_px': hull_px}
    return rows


def _item_truth(world, items):
    from sim.zone_cargo import CargoInstance, world_grasps
    out = {}
    for oid, spec in items.items():
        body = world.data.body(spec['body'])
        xmat = np.array(body.xmat).reshape(3, 3)
        yaw = math.atan2(xmat[1, 0], xmat[0, 0])
        tilt = math.degrees(math.acos(max(-1., min(1., float(xmat[2, 2])))))
        row = {'class': spec['class'], 'kind': spec['kind'], 'colour': spec['colour'],
               'xy': [float(body.xpos[0]), float(body.xpos[1])], 'yaw_rad': yaw, 'tilt_deg': tilt}
        if spec['kind'] != 'box':
            grips = world_grasps(CargoInstance(oid, spec['kind'], (row['xy'][0], row['xy'][1], yaw)))
            row['grip_xy'] = {k: [float(v['grip_xyz'][0]), float(v['grip_xyz'][1])] for k, v in grips.items()}
        out[oid] = row
    return out


def render(args):
    import cv2
    split = load_split()
    specs = [{'seed': s} for s in split['splits'][args.split]]
    out = Path(args.output)
    (out/'frames').mkdir(parents=True, exist_ok=False)
    (out/'eval-labels').mkdir()
    load = {'start': os.getloadavg()[0]}
    manifest = {'schema': SCHEMA, 'split': args.split, 'split_file_sha256': sha256(SPLIT_FILE),
                'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'source_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
                'clock': 'synchronous SIM; render after settle', 'views': [], 'scenes': []}
    started = time.monotonic()
    for spec in specs:
        seed = spec['seed']
        rng = random.Random(f'{SCENE}:cargo:{seed}')
        world, items, info = build_world(seed, rng)
        try:
            table = _geom_table(world, items)
            robot_ids = list(world.robot_ids)
            tops = [c['name'] for c in info['static_map']['top_cameras']]
            manifest['scenes'].append({'scene': SCENE, 'seed': seed, 'scene_xml_sha256': info['scene_xml_sha256'],
                                       'catalogue_sha256': info['catalogue_sha256'],
                                       'items': {k: v['class'] for k, v in items.items()}})
            for case, count in VIEW_PLAN:
                for i in range(count):
                    planned = None
                    for _ in range(30):
                        planned = plan_view(case, i, rng, info, items, robot_ids)
                        if planned is not None:
                            break
                    view_id = f'zone_wide-s{seed}-{case}-{i}'
                    if planned is None:
                        manifest['views'].append({'view_id': view_id, 'skipped': 'no valid placement'})
                        continue
                    robots, poses, arm = planned
                    _pose_all(world, robots, poses, items, arm)
                    vdir = out/'frames'/view_id
                    vdir.mkdir()
                    truth = _item_truth(world, items)
                    robot_truth = {}
                    for rid in robot_ids:
                        xyz = world.robot(rid).base_xyz()
                        robot_truth[rid] = [float(xyz[0]), float(xyz[1]), float(world.robot(rid).base_rpy()[2])]
                    labels = {'view_id': view_id, 'scene': SCENE, 'seed': seed, 'case': case,
                              'items': truth, 'robots': robot_truth, 'cameras': {},
                              'geom_table': {str(k): v for k, v in table.items()}}
                    for cam in tops:
                        jpeg = world.render_team_jpeg(camera=cam, quality=95)
                        (vdir/f'{cam}.jpg').write_bytes(jpeg)
                        seg = _segment(world, cam)
                        labels['cameras'][cam] = {'items': label_top(world, seg, table, items, cam)}
                        cv2.imwrite(str(out/'eval-labels'/f'{view_id}--{cam}.png'), (seg+1).astype(np.uint16))
                    (vdir/'actor-inputs.json').write_text(json.dumps({'view_id': view_id, 'top_cameras': tops}, indent=1))
                    (out/'eval-labels'/f'{view_id}.json').write_text(json.dumps(labels))
                    manifest['views'].append({'view_id': view_id, 'case': case, 'scene': SCENE, 'seed': seed})
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

FULL_FRACTION = .6
MIN_VISIBLE_PX = 20
PIXEL_MATCH_PAD = 4
MATCH_RADIUS_M = {'box': .05, 'can': .05, 'tile': .05, 'heavy_crate': .08, 'long_beam': .10, 'tri_frame': .10}
PROFILES = ('top_cargo_v1', 'top_zone_v2', 'zone_perception_v1')
CARGO_CLASSES = ('can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')
BOX_CLASSES = ('box_cyan', 'box_green', 'box_red', 'box_yellow')
CLASSES = BOX_CLASSES + CARGO_CLASSES


def _visibility(row):
    if row['visible_px'] < MIN_VISIBLE_PX:
        return 'not_visible'
    if row['partial_frame']:
        return 'partial_frame'
    return 'full' if row['visible_fraction'] >= FULL_FRACTION else 'occluded'


def _kind_of(cls):
    return 'box' if cls.startswith('box_') else cls


def _yaw_error(est, true, sym):
    if est is None or sym is None:
        return None
    p = math.radians(sym)
    d = (est-true) % p
    return math.degrees(min(d, p-d))


def _grip_error(item, truth):
    """Worst grip-point error after the best role assignment (roles swap under symmetry)."""
    from harness.zone_cargo_perception import grasp_handles
    est = [h['grip_xyz_m'][:2] for h in grasp_handles(item)['handles']]
    true = list(truth['grip_xy'].values())
    if not est or len(est) != len(true):
        return None
    best = None
    for perm in itertools.permutations(range(len(true))):
        worst = max(math.dist(est[i], true[j]) for i, j in enumerate(perm))
        best = worst if best is None else min(best, worst)
    return best


def _bg_category(seg, table, px):
    h, w = seg.shape
    x, y = int(round(min(max(px[0], 0), w-1))), int(round(min(max(px[1], 0), h-1)))
    patch = seg[max(0, y-3):y+4, max(0, x-3):x+4].ravel()-1
    cats = [table.get(str(int(g)), {'category': 'none'})['category'] if g >= 0 else 'none' for g in patch]
    for c in ('robot', 'item', 'paint_C', 'paint_A', 'paint_B', 'paint_pickup', 'wall'):
        if c in cats:
            return c
    return max(set(cats), key=cats.count)


def _detect(profile, jpeg, camera):
    from harness import zone_color_boxes as zcb
    from harness.zone_cargo_perception import detect_cargo_top
    if profile == 'top_cargo_v1':
        rows = detect_cargo_top(jpeg, camera)
        return [{**r, 'class': _cls(r['kind'], r.get('colour'))} for r in rows]
    rows = zcb.detect_top(jpeg, camera, zcb.KINDS, profile=profile)
    return [{'class': _cls('box', r['kind']), 'kind': 'box', 'colour': r['kind'], 'pixel': r['pixel'],
             'floor_xy_m': r['floor_xy_m'], 'camera': r['camera'], 'yaw_rad': None, 'confidence': None}
            for r in rows]


def _merged(profile, tops, static):
    from harness import zone_perception as zp
    from harness.zone_cargo_perception import detect_all_cargo
    if profile == 'top_cargo_v1':
        items = detect_all_cargo(tops, static)['items']
        return [{**r, 'class': _cls(r['kind'], r.get('colour'))} for r in items]
    if profile == 'zone_perception_v1':
        rows = zp.detect_all(tops, static)
    else:
        from harness import zone_color_boxes as zcb
        rows = []
        for cam in static['top_cameras']:
            for d in zcb.detect_top(tops[cam['name']], cam, zcb.KINDS, profile=profile):
                d['_off'] = math.hypot(d['pixel'][0]-.5, d['pixel'][1]-.5)
                rows.append(d)
        rows.sort(key=lambda r: r['_off'])
        kept = []
        for r in rows:
            if not any(k['kind'] == r['kind'] and math.dist(k['floor_xy_m'], r['floor_xy_m']) < .05 for k in kept):
                kept.append(r)
        rows = kept
    return [{'class': _cls('box', r['kind']), 'kind': 'box', 'colour': r['kind'], 'floor_xy_m': r['floor_xy_m'],
             'yaw_rad': None, 'confidence': None} for r in rows]


def _in_hull(hull, px, pad):
    import cv2
    if not hull:
        return None
    h = np.asarray(hull, np.float32).reshape(-1, 1, 2)
    d = cv2.pointPolygonTest(h, (float(px[0]), float(px[1])), True)
    return d >= -pad, float(cv2.contourArea(h))


def score_views(frames_dir, profile):
    import cv2
    from sim.zone_arena import authored_map
    frames_dir = Path(frames_dir)
    manifest = json.loads((frames_dir/'manifest.json').read_text())
    static = authored_map('zone_wide')
    cameras = {c['name']: c for c in static['top_cameras']}
    records = []
    for view in manifest['views']:
        if 'skipped' in view:
            continue
        vid = view['view_id']
        labels = json.loads((frames_dir/'eval-labels'/f'{vid}.json').read_text())
        table = labels['geom_table']
        tops = {}
        for cam, lab in labels['cameras'].items():
            jpeg = (frames_dir/'frames'/vid/f'{cam}.jpg').read_bytes()
            tops[cam] = jpeg
            seg = cv2.imread(str(frames_dir/'eval-labels'/f'{vid}--{cam}.png'), cv2.IMREAD_UNCHANGED).astype(np.int32)
            h, w = seg.shape
            matched = {}
            for d in _detect(profile, jpeg, cameras[cam]):
                px = [d['pixel'][0]*w, d['pixel'][1]*h]
                cands = []
                for oid, row in lab['items'].items():
                    hit = _in_hull(row['hull_px'], px, PIXEL_MATCH_PAD)
                    if hit and hit[0]:
                        cands.append((row['class'] != d['class'], hit[1], oid))
                rec = {'view_id': vid, 'case': view['case'], 'camera': cam, 'level': 'camera', 'record': 'detection',
                       'class': d['class'], 'confidence': d.get('confidence')}
                if cands:
                    cands.sort()
                    oid = cands[0][2]
                    rec['item'] = oid
                    if not cands[0][0]:
                        rec['outcome'] = 'duplicate' if oid in matched else 'tp'
                        matched.setdefault(oid, d)
                    else:
                        rec['outcome'] = 'confusion'
                        rec['true_class'] = lab['items'][oid]['class']
                else:
                    rec['outcome'] = 'fp_background'
                    rec['background'] = _bg_category(seg, table, px)
                records.append(rec)
            for oid, row in lab['items'].items():
                vis = _visibility(row)
                if vis == 'not_visible':
                    continue
                rec = {'view_id': vid, 'case': view['case'], 'camera': cam, 'level': 'camera', 'record': 'gt',
                       'class': row['class'], 'item': oid, 'visibility': vis, 'detected': oid in matched}
                if oid in matched:
                    d, t = matched[oid], labels['items'][oid]
                    rec['xy_error_m'] = math.dist(d['floor_xy_m'], t['xy'])
                    rec['yaw_error_deg'] = _yaw_error(d.get('yaw_rad'), t['yaw_rad'], SYMMETRY.get(row['class']))
                records.append(rec)
        records.extend(_score_merged(profile, vid, view['case'], labels, tops, static))
    return records, manifest


def _score_merged(profile, vid, case, labels, tops, static):
    dets = _merged(profile, tops, static)
    order = ['not_visible', 'occluded', 'partial_frame', 'full']
    vis = {}
    for cam, lab in labels['cameras'].items():
        for oid, row in lab['items'].items():
            v = _visibility(row)
            if order.index(v) > order.index(vis.get(oid, 'not_visible')):
                vis[oid] = v
    truth = labels['items']
    out = []
    used_gt, det_outcome = {}, {}
    dets = sorted(enumerate(dets), key=lambda x: -(x[1].get('confidence') or 1.))
    # 1) same-class nearest within radius
    pairs = []
    for i, d in dets:
        for oid, t in truth.items():
            if t['class'] != d['class']:
                continue
            dist = math.dist(d['floor_xy_m'], t['xy'])
            if dist <= MATCH_RADIUS_M[_kind_of(t['class'])]:
                pairs.append((dist, i, oid))
    for dist, i, oid in sorted(pairs):
        if i in det_outcome or oid in used_gt:
            continue
        used_gt[oid] = i
        det_outcome[i] = ('tp', oid)
    dmap = dict(dets)
    for i, d in dets:
        if i in det_outcome:
            continue
        near = sorted((math.dist(d['floor_xy_m'], t['xy']), oid) for oid, t in truth.items()
                      if math.dist(d['floor_xy_m'], t['xy']) <= max(MATCH_RADIUS_M[_kind_of(t['class'])],
                                                                      MATCH_RADIUS_M[_kind_of(d['class'])]))
        same = [oid for _, oid in near if truth[oid]['class'] == d['class']]
        if same:
            det_outcome[i] = ('duplicate', same[0])
        elif near:
            det_outcome[i] = ('confusion', near[0][1])
        else:
            det_outcome[i] = ('fp_background', None)
    regions = static['regions']
    for i, (outcome, oid) in det_outcome.items():
        d = dmap[i]
        rec = {'view_id': vid, 'case': case, 'camera': 'merged_top', 'level': 'merged', 'record': 'detection',
               'class': d['class'], 'outcome': outcome, 'item': oid, 'confidence': d.get('confidence')}
        if outcome == 'confusion':
            rec['true_class'] = truth[oid]['class']
        if outcome == 'fp_background':
            xy = d['floor_xy_m']
            bg = 'floor'
            if any(math.dist(xy, r[:2]) <= .20 for r in labels['robots'].values()):
                bg = 'robot'
            elif any(math.dist(xy, t['xy']) <= .30 for t in truth.values()):
                bg = 'near_item'
            else:
                for rid, r in regions.items():
                    (cx, cy), (hx, hy) = r['center_m'], r['half_extents_m']
                    if abs(xy[0]-cx) <= hx and abs(xy[1]-cy) <= hy:
                        bg = 'paint_' + (rid.split('_', 1)[1] if rid.startswith('zone_') else rid)
                        break
            rec['background'] = bg
            rec['floor_xy_m'] = xy
        out.append(rec)
    for oid, t in truth.items():
        v = vis.get(oid, 'not_visible')
        if v == 'not_visible':
            continue
        rec = {'view_id': vid, 'case': case, 'camera': 'merged_top', 'level': 'merged', 'record': 'gt',
               'class': t['class'], 'item': oid, 'visibility': v, 'detected': oid in used_gt,
               'tilt_deg': t['tilt_deg']}
        if oid in used_gt:
            d = dmap[used_gt[oid]]
            rec['xy_error_m'] = math.dist(d['floor_xy_m'], t['xy'])
            rec['yaw_error_deg'] = _yaw_error(d.get('yaw_rad'), t['yaw_rad'], SYMMETRY.get(t['class']))
            rec['confidence'] = d.get('confidence')
            if t['kind'] != 'box' and profile == 'top_cargo_v1':
                rec['grip_error_m'] = _grip_error(d, t)
                rec['centre_mode'] = (d.get('evidence') or {}).get('centre_mode')
        else:
            # what, if anything, was reported near it
            rec['nearest_detection'] = None
        out.append(rec)
    return out


def _stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    a = np.asarray(values, float)
    return {'n': int(a.size), 'median': round(float(np.median(a)), 4), 'p90': round(float(np.percentile(a, 90)), 4),
            'max': round(float(a.max()), 4)}


def summarize(records, frames):
    out = {}
    for level in ('camera', 'merged'):
        rs = [r for r in records if r['level'] == level]
        gts = [r for r in rs if r['record'] == 'gt']
        dets = [r for r in rs if r['record'] == 'detection']
        per_class = {}
        for cls in CLASSES:
            row = {}
            for vis in ('full', 'partial_frame', 'occluded'):
                sel = [r for r in gts if r['class'] == cls and r['visibility'] == vis]
                n = sum(r['detected'] for r in sel)
                row[vis] = {'positives': len(sel), 'detected': n, 'rate': round(n/len(sel), 4) if sel else None}
            tp = [r for r in gts if r['class'] == cls and r['detected']]
            row['xy_error_m'] = _stats([r.get('xy_error_m') for r in tp])
            row['xy_error_m_full'] = _stats([r.get('xy_error_m') for r in tp if r['visibility'] == 'full'])
            row['yaw_error_deg'] = _stats([r.get('yaw_error_deg') for r in tp])
            row['yaw_error_deg_full'] = _stats([r.get('yaw_error_deg') for r in tp if r['visibility'] == 'full'])
            if level == 'merged':
                row['grip_error_m'] = _stats([r.get('grip_error_m') for r in tp])
                row['grip_error_m_full'] = _stats([r.get('grip_error_m') for r in tp if r['visibility'] == 'full'])
                modes = {}
                for r in tp:
                    if r.get('centre_mode'):
                        modes[r['centre_mode']] = modes.get(r['centre_mode'], 0)+1
                if modes:
                    row['centre_modes'] = modes
                row['full_by_case'] = {}
                for case in sorted({r['case'] for r in gts}):
                    sel = [r for r in gts if r['class'] == cls and r['visibility'] == 'full' and r['case'] == case]
                    if sel:
                        row['full_by_case'][case] = f"{sum(r['detected'] for r in sel)}/{len(sel)}"
            kd = [r for r in dets if r['class'] == cls]
            row['detections'] = len(kd)
            row['duplicates'] = sum(r['outcome'] == 'duplicate' for r in kd)
            conf = [r for r in kd if r['outcome'] == 'confusion']
            row['confused_as_this_from'] = {c: sum(r['true_class'] == c for r in conf)
                                            for c in sorted({r['true_class'] for r in conf})}
            bg = [r for r in kd if r['outcome'] == 'fp_background']
            row['fp_background'] = {c: sum(r['background'] == c for r in bg) for c in sorted({r['background'] for r in bg})}
            per_class[cls] = row
        # confusion matrix: true class -> predicted class (tp + confusion), plus missed and background FPs
        matrix = {c: {} for c in CLASSES + ('background',)}
        for r in dets:
            if r['outcome'] == 'tp':
                matrix[r['class']][r['class']] = matrix[r['class']].get(r['class'], 0)+1
            elif r['outcome'] == 'confusion':
                matrix[r['true_class']][r['class']] = matrix[r['true_class']].get(r['class'], 0)+1
            elif r['outcome'] == 'fp_background':
                matrix['background'][r['class']] = matrix['background'].get(r['class'], 0)+1
        for r in gts:
            if not r['detected']:
                matrix[r['class']]['missed'] = matrix[r['class']].get('missed', 0)+1
        box_cargo = {'cargo_reported_as_box': sum(r['outcome'] == 'confusion' and r['class'].startswith('box_')
                                                  and not r['true_class'].startswith('box_') for r in dets),
                     'box_reported_as_cargo': sum(r['outcome'] == 'confusion' and not r['class'].startswith('box_')
                                                  and r['true_class'].startswith('box_') for r in dets),
                     'cargo_kind_swaps': sum(r['outcome'] == 'confusion' and not r['class'].startswith('box_')
                                             and not r['true_class'].startswith('box_') for r in dets),
                     'box_colour_swaps': sum(r['outcome'] == 'confusion' and r['class'].startswith('box_')
                                             and r['true_class'].startswith('box_') for r in dets)}
        fp = [r for r in dets if r['outcome'] == 'fp_background']
        fp_by_bg = {c: sum(r['background'] == c for r in fp) for c in sorted({r['background'] for r in fp})}
        out[level] = {'frames': frames[level], 'per_class': per_class, 'confusion_matrix': matrix,
                      'box_cargo_confusion': box_cargo, 'false_positives_total': len(fp),
                      'false_positives_by_background': fp_by_bg,
                      'false_positives_per_frame': round(len(fp)/frames[level], 4) if frames[level] else None}
    return out


def score(args):
    frames_dir = Path(args.frames)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    load = {'start': os.getloadavg()[0]}
    results = {}
    manifest = None
    for profile in args.profile or PROFILES:
        records, manifest = score_views(frames_dir, profile)
        views = [v for v in manifest['views'] if 'skipped' not in v]
        frames = {'camera': 4*len(views), 'merged': len(views)}
        (out/f'records-{profile}.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
        results[profile] = summarize(records, frames)
    load['end'] = os.getloadavg()[0]
    from sim.zone_cargo import catalogue_record
    summary = {'schema': SCHEMA+'.score', 'frames_dir': str(frames_dir), 'split': manifest['split'],
               'render_source_sha': manifest['source_sha'], 'render_source_dirty': manifest['source_dirty'],
               'score_source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
               'score_source_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip()),
               'catalogue_sha256': catalogue_record()['sha256'], 'load_average_1min': load, 'results': results}
    (out/'summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps({'written': str(out/'summary.json'), 'load_average_1min': load}))


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    r = sub.add_parser('render', help='render TOP frames + eval labels for one split')
    r.add_argument('--split', choices=('dev', 'test'), required=True)
    r.add_argument('--output', type=Path, required=True)
    s = sub.add_parser('score', help='run detectors on rendered frames and score them')
    s.add_argument('--frames', type=Path, required=True, help='output directory of render')
    s.add_argument('--output', type=Path, required=True)
    s.add_argument('--profile', action='append', choices=PROFILES, help='TOP profile(s); default: all')
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
