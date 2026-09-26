"""Offline evaluation of the four wrist-RGB-only zone judgments (render / score).

Two finite local steps, as in ``scripts/eval_zone_color_detection.py``
(experiments/2026-09-25-zone-rgb-color):

``render``  builds the standard ``ZoneScene`` in synchronous SIM, poses robots,
            cargo and one unmapped obstruction per view, and writes only what a
            robot may receive to ``frames/``: its own wrist RGB, its own issued
            arm pulses, the static map id/hash and the scenario order sheet.
            Ground truth (world poses, segmentation, the truth answer of each
            judgment) goes ONLY to ``eval-labels/``.
``score``   runs ``harness.zone_own_perception`` on ``frames/`` and scores it
            against ``eval-labels/``. Multi-tick views additionally go through
            ``harness.zone_own_outcome.JudgmentTracker``.

The ground-truth teacher is used here for dataset generation and scoring only.
No detector call receives a label; the score step reads labels after the answer
is produced.

Render notes that must stay visible in every report:
* ``hold_*`` views pose the item between the jaws for a single frame and render
  without stepping physics. Weld stays OFF and no contact assistance is added:
  this is a *perception* fixture, never evidence that a grasp held.
* ``block_*`` views add one extra body, ``unmapped_block``, that is deliberately
  absent from the static map. That is the condition under test; its size, colour
  and pose are recorded in the manifest with the scene hash.
* Cameras, robot appearance, box paint and the cargo catalogue are unchanged.

Own-pose belief: map-relative questions need the robot's own pose estimate. It
is *injected* here at three declared levels (``gt_stub_eval_only``,
``noise_30mm``, ``noise_60mm``), the same pattern as the wrist skill cohorts.
The zero-noise level is a stub, not a localization result.
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

# Read-only reuse of the rgb-color render helpers (that file is not modified).
from scripts.eval_zone_color_detection import (_segment, _valid_own_region, _to_base, _yaw_quat,  # noqa: E402
                                              sha256)

SPLIT_FILE = ROOT/'experiments/2026-09-26-zone-own-perception/split.json'
SCHEMA = 'ugrp.zone_own_perception_eval.v1'

EVAL_GOAL = {'A': {'cyan': 1, 'red': 1}, 'B': {'cyan': 1, 'green': 1}, 'C': {'yellow': 1, 'green': 1}}
EXTRA_BOXES = {'cyan': 1, 'yellow': 1}
CARGO_ITEMS = (('can1', 'can'), ('tile1', 'tile'))
UNMAPPED_BLOCK = {'name': 'unmapped_block', 'half_extents_m': (.085, .085, .110),
                  'rgba': '.36 .28 .21 1', 'mass_kg': .60, 'park_xy': (-7.0, -9.0)}

FOLDED = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
BASE_Z_M = .032355118817659255       # same chassis height the rgb-color render uses
HELD_Z_OFFSET = {'box': -.008, 'cargo': -.024}
BOX_HALF_Z_M = .016

# View plan: (case, issued pan pulses, one per observation tick). The robot stays
# where it is and observes on the fixed cadence; between ticks its base drifts by
# ``TICK_JITTER`` (its own micro-motion), so repeated ticks are genuinely
# different frames rather than a re-render of one. The agreed off-axis pan is
# used for the third look-back tick only: swinging the arm away from the target
# is the tracker's *re-look plan*, not a better view of the same spot.
STRAIGHT2 = (1500, 1500)
LOOK_BACK3 = (1500, 1500, 1770)
TICK_JITTER = {'xy_m': .015, 'yaw_rad': math.radians(1.2)}
VIEW_PLAN = (
    ('slot_expected_box', STRAIGHT2), ('slot_expected_can', STRAIGHT2), ('slot_wrong_kind', STRAIGHT2),
    ('slot_empty', STRAIGHT2), ('slot_occluded', STRAIGHT2), ('slot_far', STRAIGHT2),
    ('hold_expected_box', STRAIGHT2), ('hold_expected_can', STRAIGHT2), ('hold_expected_tile', STRAIGHT2),
    ('hold_wrong_kind', STRAIGHT2), ('hold_empty', STRAIGHT2),
    ('placed_zone_A', LOOK_BACK3), ('placed_zone_B', LOOK_BACK3), ('placed_zone_C', LOOK_BACK3),
    ('placed_outside_slot', LOOK_BACK3), ('placed_empty', LOOK_BACK3), ('placed_occluded', LOOK_BACK3),
    ('block_unmapped_passage', STRAIGHT2), ('block_unmapped_near', STRAIGHT2), ('clear_passage', STRAIGHT2),
    ('clear_open', STRAIGHT2), ('mapped_wall', STRAIGHT2), ('peer_in_lane', STRAIGHT2),
)
JUDGMENT_OF = {'slot': 'slot_item', 'hold': 'holding_item', 'placed': 'placed_in_slot',
               'block': 'route_blockage', 'clear': 'route_blockage', 'mapped': 'route_blockage',
               'peer': 'route_blockage'}
POSE_BELIEF_LEVELS = {'gt_stub_eval_only': (0., 0.), 'noise_30mm': (.030, math.radians(3.)),
                      'noise_60mm': (.060, math.radians(6.))}
PRIMARY_BELIEF = 'gt_stub_eval_only'

# Truth conventions (PREREGISTERED in the experiment README).
TRUTH_MATCH_M = .10          # an item this close to the slot centre counts as "at the slot"
MIN_VISIBLE_PX = {'slot_item': 90, 'holding_item': 500, 'placed_in_slot': 90, 'route_blockage': 260}
# 'holding_item' counts pixels inside ``zone_own_perception.CARRY_ROI`` only.


def _git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def load_split():
    return json.loads(SPLIT_FILE.read_text())


# ------------------------------------------------------------------ scenario config

def pickup_slots(variant):
    """Pickup-bay slot table from the scenario configuration (never simulator state).

    ``sim.zone_arena`` declares the pickup grid that the episode generator uses
    to place items; the same declaration names the slots. The authored map file
    is not modified (another package owns ``maps/zones/``).
    """
    from sim.zone_arena import layout
    spec = layout(variant)
    slots = {}
    for ci, x in enumerate(spec['pickup_columns_x']):
        for ri, y in enumerate(spec['pickup_rows_y']):
            slots[f'P{ci+1}-{ri+1}'] = [round(float(x), 4), round(float(y), 4)]
    return slots


def order_sheet(variant, seed, config, slots, cargo_poses):
    """Immutable order sheet built from the scenario config, not from the simulator."""
    orders, used = [], {}
    for item_id, spec in config['setup_only']['objects'].items():
        xy = tuple(round(float(v), 4) for v in spec['position_m'][:2])
        used[xy] = item_id
    rows = []
    for item_id, spec in config['setup_only']['objects'].items():
        xy = [round(float(v), 4) for v in spec['position_m'][:2]]
        slot = next((sid for sid, c in slots.items() if math.dist(c, xy) < 1e-3), None)
        rows.append((item_id, spec['kind'], slot, xy, 'box'))
    for item_id, kind in CARGO_ITEMS:
        xy = [round(float(v), 4) for v in cargo_poses[item_id][:2]]
        slot = next((sid for sid, c in slots.items() if math.dist(c, xy) < 1e-3), None)
        rows.append((item_id, kind, slot, xy, 'cargo'))
    zones = ('A', 'B', 'C')
    for index, (item_id, kind, slot, xy, family) in enumerate(sorted(rows)):
        orders.append({'order_id': f'order-{index+1}', 'item_ids': [item_id], 'kind': kind, 'count': 1,
                       'required_robots': 1, 'destination_zone': zones[index % 3],
                       'initial_location': {'pickup_bay': 'P', 'slot': slot, 'declared_center_m': xy},
                       'item_family': family})
    return {'schema': 'ugrp.zone_order.v1', 'map_id': 'zone_'+variant.split('_', 1)[1],
            'variant': variant, 'seed': seed, 'orders': orders,
            'note': ('initial_location is where the scenario placed the item at setup; it is not a '
                     'guarantee about the current state')}


# ------------------------------------------------------------------ world

def _cargo_free_cells(config, slots):
    taken = {tuple(round(float(v), 4) for v in spec['position_m'][:2])
             for spec in config['setup_only']['objects'].values()}
    free = [c for c in slots.values() if tuple(c) not in taken]
    if len(free) < len(CARGO_ITEMS):
        raise ValueError('not enough free pickup cells for the catalogue cargo')
    return free


def _unmapped_block_xml(xml):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    half = ' '.join(f'{v:.6f}' for v in UNMAPPED_BLOCK['half_extents_m'])
    body = ET.SubElement(world, 'body', name=UNMAPPED_BLOCK['name'],
                         pos=f"{UNMAPPED_BLOCK['park_xy'][0]} {UNMAPPED_BLOCK['park_xy'][1]} "
                             f"{UNMAPPED_BLOCK['half_extents_m'][2]:.6f}")
    ET.SubElement(body, 'freejoint', name=UNMAPPED_BLOCK['name']+'_free')
    ET.SubElement(body, 'geom', {'name': UNMAPPED_BLOCK['name']+'_geom', 'type': 'box', 'size': half,
                                 'rgba': UNMAPPED_BLOCK['rgba'], 'mass': f"{UNMAPPED_BLOCK['mass_kg']:.6f}",
                                 'contype': '1', 'conaffinity': '3', 'friction': '1.0 .2 .01', 'group': '0'})
    return ET.tostring(root, encoding='unicode')


def build_world(variant, seed, rng):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_arena import episode
    from sim.zone_cargo import add_cargo_xml, instances, place as place_cargo
    from sim.zone_scene import ZoneScene
    config = episode(variant, seed, goal=EVAL_GOAL, extra_boxes=dict(EXTRA_BOXES))
    config['contact_solver_profile'] = 'local_contact_fine'
    config['extra_boxes'] = dict(EXTRA_BOXES)
    definition = ZoneScene.from_zone_config(config)
    slots = pickup_slots(variant)
    free = _cargo_free_cells(config, slots)
    rng.shuffle(free)
    cargo_poses = {item_id: (free[i][0], free[i][1], 0.) for i, (item_id, _) in enumerate(CARGO_ITEMS)}
    cargo = instances([{'item_id': item_id, 'kind': kind, 'pose': list(cargo_poses[item_id])}
                       for item_id, kind in CARGO_ITEMS])
    base_transform = definition.transform
    mirrored = {'finger_contact_pairs': None}

    def transform(xml):
        xml = base_transform(xml)
        try:
            xml, manifest = add_cargo_xml(xml, cargo, mirror_pairs=True)
        except ValueError:
            xml, manifest = add_cargo_xml(xml, cargo, mirror_pairs=False)
        mirrored.update(manifest)
        xml = _unmapped_block_xml(xml)
        mirrored['scene_xml_with_extras_sha256'] = hashlib.sha256(xml.encode()).hexdigest()
        return xml

    world = MultiMasterPiProductionV2(seed=seed, width=640, height=480, render=True,
                                      warehouse_layout=definition.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=transform)
    definition.setup(world)
    place_cargo(world, cargo)
    static = config['static_map']
    items = {oid: {'kind': o['kind'], 'body': o['body_name'], 'joint': o['joint_name'], 'family': 'box'}
             for oid, o in config['setup_only']['objects'].items()}
    for inst in cargo:
        items[inst.item_id] = {'kind': inst.kind, 'body': inst.body, 'joint': inst.joint, 'family': 'cargo'}
    info = {'config': config, 'static_map': static, 'slots': slots, 'variant': variant,
            'order_sheet': order_sheet(variant, seed, config, slots, cargo_poses),
            'scene_xml_sha256': definition.manifest['scene_xml_sha256'],
            'scene_extras': dict(mirrored), 'cargo_setup_poses': {k: list(v) for k, v in cargo_poses.items()},
            'spawns': config['setup_only']['spawns']}
    return world, items, info


# ------------------------------------------------------------------ posing

def _set_base(world, rid, pose):
    world.robot(rid).set_base_pose_for_test((pose[0], pose[1], BASE_Z_M), pose[2])


def _set_item(world, items, item_id, xy, z, yaw):
    import mujoco
    joint = items[item_id]['joint']
    jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
    world.data.qpos[q:q+7] = [xy[0], xy[1], z, *_yaw_quat(yaw)]
    world.data.qvel[v:v+6] = 0


def _set_block(world, xy):
    import mujoco
    jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, UNMAPPED_BLOCK['name']+'_free')
    q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
    world.data.qpos[q:q+7] = [xy[0], xy[1], UNMAPPED_BLOCK['half_extents_m'][2], 1., 0., 0., 0.]
    world.data.qvel[v:v+6] = 0


def _floor_z(items, item_id):
    return BOX_HALF_Z_M if items[item_id]['family'] == 'box' else .0005


def _robot_facing(target, distance, lateral, yaw):
    return (target[0]-math.cos(yaw)*distance+math.sin(yaw)*lateral,
            target[1]-math.sin(yaw)*distance-math.cos(yaw)*lateral, yaw)


def _park_all(world, items, info, rng):
    """Every item and the unmapped block off the stage; the view puts back what it needs."""
    bounds = info['static_map']['bounds_m']
    park = []
    for index, item_id in enumerate(sorted(items)):
        x = bounds[0]+.25+(index % 4)*.22
        y = bounds[2]+.25+(index//4)*.22
        park.append((item_id, (x, y)))
        _set_item(world, items, item_id, (x, y), _floor_z(items, item_id), rng.uniform(-.4, .4))
    _set_block(world, UNMAPPED_BLOCK['park_xy'])
    return dict(park)


def _lane_target(info, rng):
    """A point in the first declared passage (door or corridor), plus its id and width."""
    passages = [p for p in info['static_map'].get('passages', []) if p['kind'] != 'passing_bay']
    if not passages:
        return None
    passage = passages[rng.randrange(len(passages))]
    return {'id': passage['id'], 'center_m': list(passage['center_m']),
            'width_m': float(passage['width_m']), 'kind': passage['kind']}


def plan_view(case, rng, info, items, robot_ids):
    """Robot poses, item placements, arm pulses and the question for one view."""
    static = info['static_map']
    slots = info['slots']
    orders = info['order_sheet']['orders']
    family = case.split('_')[0]
    judgment = JUDGMENT_OF[family]
    actor = robot_ids[0]
    plan = {'judgment': judgment, 'case': case, 'actor': actor, 'items': {}, 'robots': {},
            'arm': {}, 'question': {}, 'block_xy': None, 'held': None}
    boxes = [o for o in orders if o['item_family'] == 'box']
    cans = [o for o in orders if o['kind'] == 'can']
    tiles = [o for o in orders if o['kind'] == 'tile']

    if judgment == 'slot_item':
        order = {'slot_expected_box': boxes[0], 'slot_expected_can': cans[0], 'slot_wrong_kind': boxes[1],
                 'slot_empty': boxes[2], 'slot_occluded': boxes[3 % len(boxes)],
                 'slot_far': boxes[4 % len(boxes)]}[case]
        slot_id = order['initial_location']['slot']
        centre = slots[slot_id]
        if case == 'slot_wrong_kind':
            other = next(o for o in orders if o['kind'] != order['kind'] and o['item_ids'][0] != order['item_ids'][0])
            plan['items'][other['item_ids'][0]] = (centre, rng.uniform(-.4, .4))
        elif case != 'slot_empty':
            plan['items'][order['item_ids'][0]] = (centre, rng.uniform(-.4, .4))
        distance = rng.uniform(1.55, 1.85) if case == 'slot_far' else rng.uniform(.42, .80)
        yaw = rng.uniform(-math.pi, math.pi)
        plan['robots'][actor] = _robot_facing(centre, distance, rng.uniform(-.05, .05), yaw)
        plan['arm'][actor] = {**FOLDED, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
        if case == 'slot_occluded':
            mid = rng.uniform(.45, .62)
            base = plan['robots'][actor]
            peer = (base[0]+(centre[0]-base[0])*mid, base[1]+(centre[1]-base[1])*mid, rng.uniform(-math.pi, math.pi))
            plan['robots'][robot_ids[1]] = peer
        plan['question'] = {'slot_id': slot_id, 'expected_kind': order['kind'],
                            'slot_world_m': list(centre), 'order_id': order['order_id']}
    elif judgment == 'holding_item':
        order = {'hold_expected_box': boxes[0], 'hold_expected_can': cans[0], 'hold_expected_tile': tiles[0],
                 'hold_wrong_kind': boxes[0], 'hold_empty': boxes[1]}[case]
        held = None
        if case == 'hold_wrong_kind':
            held = next(o for o in orders if o['kind'] != order['kind'])['item_ids'][0]
        elif case != 'hold_empty':
            held = order['item_ids'][0]
        region = static['regions']['pickup']
        xy = (region['center_m'][0]+rng.uniform(-.6, .6), region['center_m'][1]+rng.uniform(-1.2, 1.2))
        plan['robots'][actor] = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
        plan['arm'][actor] = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
        plan['held'] = held
        plan['question'] = {'expected_kind': order['kind'], 'order_id': order['order_id']}
    elif judgment == 'placed_in_slot':
        zone = {'placed_zone_A': 'A', 'placed_zone_B': 'B', 'placed_zone_C': 'C'}.get(case)
        if zone is None:
            zone = ('A', 'B', 'C')[rng.randrange(3)]
        zone_slots = static['zone_slots'][zone]
        slot = zone_slots[rng.randrange(len(zone_slots))]
        centre = slot['center_m']
        # The 518 pair (cyan on zone B blue paint) is always exercised on zone B.
        kind = 'cyan' if zone == 'B' else ('yellow' if zone == 'A' else 'green')
        order = next((o for o in orders if o['kind'] == kind), boxes[0])
        item_id = order['item_ids'][0]
        if case == 'placed_outside_slot':
            offset = rng.choice((-1, 1))*rng.uniform(.17, .26)
            plan['items'][item_id] = ((centre[0], centre[1]+offset), rng.uniform(-.4, .4))
        elif case != 'placed_empty':
            plan['items'][item_id] = ((centre[0]+rng.uniform(-.015, .015), centre[1]+rng.uniform(-.015, .015)),
                                      rng.uniform(-.4, .4))
        yaw = rng.uniform(-math.pi, math.pi)
        plan['robots'][actor] = _robot_facing(centre, rng.uniform(.44, .82), rng.uniform(-.05, .05), yaw)
        plan['arm'][actor] = {**FOLDED, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
        if case == 'placed_occluded':
            base = plan['robots'][actor]
            mid = rng.uniform(.45, .62)
            plan['robots'][robot_ids[1]] = (base[0]+(centre[0]-base[0])*mid, base[1]+(centre[1]-base[1])*mid,
                                            rng.uniform(-math.pi, math.pi))
        plan['question'] = {'slot_id': slot['slot_id'], 'kind': order['kind'], 'zone': zone,
                            'slot_world_m': list(centre), 'order_id': order['order_id']}
    else:                                    # route_blockage
        lane = _lane_target(info, rng)
        if lane is None or case == 'clear_open':
            region = static['regions']['pickup']
            target = (region['center_m'][0]+rng.uniform(-.4, .4), region['center_m'][1]+rng.uniform(-1., 1.))
            lane = {'id': None, 'center_m': list(target), 'width_m': 2*.22, 'kind': 'open'}
        centre = lane['center_m']
        yaw = math.atan2(rng.uniform(-.12, .12), 1.) + (0. if centre[0] >= 0 else math.pi)
        distance = {'block_unmapped_near': rng.uniform(.60, .85)}.get(case, rng.uniform(1.0, 1.35))
        plan['robots'][actor] = _robot_facing(centre, distance, rng.uniform(-.06, .06), yaw)
        if case in ('block_unmapped_passage', 'block_unmapped_near'):
            plan['block_xy'] = (centre[0]+rng.uniform(-.04, .04), centre[1]+rng.uniform(-.05, .05))
        elif case == 'mapped_wall':
            wall = _nearest_interior_wall(static, centre)
            if wall is None:
                return None
            plan['robots'][actor] = _robot_facing(wall, rng.uniform(.45, .75), rng.uniform(-.05, .05),
                                                  math.atan2(0., 1.))
        elif case == 'peer_in_lane':
            plan['robots'][robot_ids[1]] = (centre[0]+rng.uniform(-.05, .05), centre[1]+rng.uniform(-.05, .05),
                                            rng.uniform(-math.pi, math.pi))
        plan['arm'][actor] = ({1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500} if rng.random() < .5
                              else {**FOLDED, 3: 1072, 4: 2400, 5: 1482, 6: 1500})
        plan['question'] = {'passage_id': lane['id'], 'lane_center_m': list(centre),
                            'lane_width_m': lane['width_m'], 'lane_kind': lane['kind']}
    bounds = static['bounds_m']
    for pose in plan['robots'].values():
        if not (bounds[0]+.22 <= pose[0] <= bounds[1]-.22 and bounds[2]+.22 <= pose[1] <= bounds[3]-.22):
            return None
    poses = list(plan['robots'].values())
    for i, a in enumerate(poses):
        for b in poses[i+1:]:
            if math.dist(a[:2], b[:2]) < .34:
                return None
    for index, rid in enumerate(robot_ids):
        if rid in plan['robots']:
            continue
        plan['robots'][rid] = (bounds[0]+.35+index*.30, bounds[3]-.35, 0.)
        plan['arm'].setdefault(rid, dict(FOLDED))
    for rid in robot_ids:
        plan['arm'].setdefault(rid, dict(FOLDED))
    return plan


def _nearest_interior_wall(static, near_xy):
    interior = [o for o in static['obstacles']
                if not o['id'].startswith(('wall_north', 'wall_south', 'wall_west', 'wall_east'))]
    if not interior:
        return None
    wall = min(interior, key=lambda o: math.dist(o['center_m'], near_xy))
    (cx, cy), (hx, hy) = wall['center_m'], wall['half_extents_m']
    if hy >= hx:                       # a north-south wall; stand west of a point on it
        return (cx-.02, max(cy-hy+.1, min(near_xy[1], cy+hy-.1)))
    return (max(cx-hx+.1, min(near_xy[0], cx+hx-.1)), cy-.02)


# ------------------------------------------------------------------ labels

def _geom_categories(world, items):
    import mujoco
    body_to_item = {spec['body']: oid for oid, spec in items.items()}
    table, item_gids = {}, {}
    for gid in range(world.model.ngeom):
        name = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ''
        bid = int(world.model.geom_bodyid[gid])
        body = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY, bid) or ''
        root = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY, int(world.model.body_rootid[bid])) or ''
        if body in body_to_item:
            oid = body_to_item[body]
            table[gid] = {'category': 'item', 'ref': oid}
            item_gids.setdefault(oid, []).append(gid)
        elif body == UNMAPPED_BLOCK['name']:
            table[gid] = {'category': 'unmapped_block', 'ref': UNMAPPED_BLOCK['name']}
            item_gids.setdefault(UNMAPPED_BLOCK['name'], []).append(gid)
        elif root.endswith('__robot') or name.split('__')[0] in ('r1', 'r2', 'r3'):
            table[gid] = {'category': 'robot', 'ref': (root.split('__')[0] or name.split('__')[0])}
        elif name == 'floor':
            table[gid] = {'category': 'floor', 'ref': name}
        elif 'wall' in name:
            table[gid] = {'category': 'wall', 'ref': name}
        elif name.startswith(('zone_', 'dispatch_')):
            table[gid] = {'category': 'floor_paint', 'ref': name}
        else:
            table[gid] = {'category': 'other', 'ref': name or body}
    return table, item_gids


def _visible(seg, gids):
    from harness.zone_own_perception import CARRY_ROI
    mask = np.isin(seg, gids)
    count = int(mask.sum())
    height, width = seg.shape[:2]
    x0, y0, x1, y1 = (int(round(CARRY_ROI[0]*width)), int(round(CARRY_ROI[1]*height)),
                      int(round(CARRY_ROI[2]*width)), int(round(CARRY_ROI[3]*height)))
    in_roi = int(mask[y0:y1, x0:x1].sum())
    if not count:
        return {'visible_px': 0, 'visible_in_carry_roi_px': 0, 'seg_centroid_px': None}
    ys, xs = np.nonzero(mask)
    return {'visible_px': count, 'visible_in_carry_roi_px': in_roi,
            'seg_centroid_px': [float(xs.mean()), float(ys.mean())]}


def _truth(plan, world, items, item_gids, seg, base_pose, static):
    """The truth answer of this view's judgment from ground truth (eval only)."""
    judgment = plan['judgment']
    question = plan['question']
    world_xy = {oid: [float(v) for v in world.data.body(spec['body']).xpos[:2]] for oid, spec in items.items()}
    block_xy = [float(v) for v in world.data.body(UNMAPPED_BLOCK['name']).xpos[:2]]
    visible = {oid: _visible(seg, gids) for oid, gids in item_gids.items()}
    row = {'judgment': judgment, 'case': plan['case'], 'world_xy': world_xy, 'block_world_xy': block_xy,
           'visible': visible, 'base_pose': list(base_pose)}
    if judgment in ('slot_item', 'placed_in_slot'):
        centre = question['slot_world_m']
        kind = question.get('expected_kind') or question.get('kind')
        at_slot = [(oid, items[oid]['kind']) for oid, xy in world_xy.items()
                   if math.dist(xy, centre) <= TRUTH_MATCH_M]
        same = [oid for oid, k in at_slot if k == kind]
        row['items_at_slot'] = at_slot
        row['truth'] = 'yes' if same else 'no'
        row['truth_observed'] = kind if same else (items[at_slot[0][0]]['kind'] if at_slot else 'empty')
        target = same[0] if same else (at_slot[0][0] if at_slot else None)
        base_xy = _to_base(base_pose, centre)
        row['slot_base_m'] = [round(v, 4) for v in base_xy]
        in_range = math.hypot(*base_xy) <= 1.20 and base_xy[0] > .05
        enough = target is None or visible[target]['visible_px'] >= MIN_VISIBLE_PX[judgment]
        row['observable'] = bool(in_range and enough and not _slot_occluded(plan, seg, base_pose, centre))
    elif judgment == 'holding_item':
        held = plan['held']
        row['held_item'] = held
        row['truth'] = 'yes' if held is not None and items[held]['kind'] == question['expected_kind'] else 'no'
        row['truth_observed'] = items[held]['kind'] if held is not None else 'empty'
        # Observability for this judgment is "visible in the region the carry view
        # actually looks at". Dev finding: a held can shows only its top rim,
        # entirely outside that region, so those views are not observable.
        row['observable'] = bool(held is None
                                 or visible[held]['visible_in_carry_roi_px'] >= MIN_VISIBLE_PX[judgment])
    else:
        base_xy = _to_base(base_pose, block_xy)
        half = max(.10, question['lane_width_m']/2-.02)
        in_lane = .05 < base_xy[0] <= 2.0 and abs(base_xy[1]) <= half+.05
        row['block_base_m'] = [round(v, 4) for v in base_xy]
        row['truth'] = 'yes' if (plan['block_xy'] is not None and in_lane) else 'no'
        row['truth_observed'] = 'unmapped_obstruction' if row['truth'] == 'yes' else 'clear'
        block_seen = visible.get(UNMAPPED_BLOCK['name'], {'visible_px': 0})['visible_px']
        row['observable'] = bool(block_seen >= MIN_VISIBLE_PX[judgment] if row['truth'] == 'yes' else True)
        row['peer_in_lane'] = plan['case'] == 'peer_in_lane'
        row['mapped_wall_view'] = plan['case'] == 'mapped_wall'
    return row


def _slot_occluded(plan, seg, base_pose, centre):
    """True when a robot body covers the slot centre in this frame."""
    return False if plan['case'] not in ('slot_occluded', 'placed_occluded') else True


# ------------------------------------------------------------------ render

def render(args):
    split = load_split()
    specs = split['splits'][args.split]
    out = Path(args.output)
    (out/'frames').mkdir(parents=True, exist_ok=False)
    (out/'eval-labels').mkdir()
    import cv2
    load = {'start': os.getloadavg()[0]}
    manifest = {'schema': SCHEMA, 'split': args.split, 'split_file_sha256': sha256(SPLIT_FILE),
                'source_sha': _git('rev-parse', 'HEAD'),
                'source_dirty': bool(_git('status', '--porcelain')),
                'clock': 'synchronous SIM; arm settled, then one render per tick',
                'weld': 'off', 'hold_views': 'item posed between the jaws for one frame, physics not stepped',
                'unmapped_block': {k: (list(v) if isinstance(v, tuple) else v)
                                   for k, v in UNMAPPED_BLOCK.items()},
                'pose_belief_levels': {k: list(v) for k, v in POSE_BELIEF_LEVELS.items()},
                'views': [], 'scenes': []}
    started = time.monotonic()
    for spec in specs:
        variant, seed = spec['variant'], spec['seed']
        rng = random.Random(f'{variant}:{seed}:own-perception')
        world, items, info = build_world(variant, seed, rng)
        try:
            table, item_gids = _geom_categories(world, items)
            robot_ids = list(world.robot_ids)
            manifest['scenes'].append({'variant': variant, 'seed': seed,
                                       'scene_xml_sha256': info['scene_xml_sha256'],
                                       'scene_extras': info['scene_extras'],
                                       'items': {k: v['kind'] for k, v in items.items()},
                                       'order_sheet_sha256': hashlib.sha256(
                                           json.dumps(info['order_sheet'], sort_keys=True).encode()).hexdigest()})
            (out/'frames'/f'order-sheet-{variant}-s{seed}.json').write_text(
                json.dumps(info['order_sheet'], indent=1))
            for case, pans in VIEW_PLAN:
                plan = None
                for _ in range(40):
                    plan = plan_view(case, rng, info, items, robot_ids)
                    if plan is not None:
                        break
                view_id = f'{variant}-s{seed}-{case}'
                if plan is None:
                    manifest['views'].append({'view_id': view_id, 'skipped': 'no valid placement'})
                    continue
                _park_all(world, items, info, rng)
                for item_id, (xy, yaw) in plan['items'].items():
                    _set_item(world, items, item_id, xy, _floor_z(items, item_id), yaw)
                if plan['block_xy'] is not None:
                    _set_block(world, plan['block_xy'])
                for rid, pose in plan['robots'].items():
                    _set_base(world, rid, pose)
                import mujoco
                mujoco.mj_forward(world.model, world.data)
                world._team_joint_move_servos(plan['arm'], .3, settle_s=.3)
                actor = plan['actor']
                if plan['held'] is not None:
                    grip = world.robot(actor).site_xyz('grip_site')
                    offset = HELD_Z_OFFSET['box' if items[plan['held']]['family'] == 'box' else 'cargo']
                    _set_item(world, items, plan['held'], (float(grip[0]), float(grip[1])),
                              float(grip[2])+offset, world.robot(actor).base_rpy()[2])
                    mujoco.mj_forward(world.model, world.data)
                vdir = out/'frames'/view_id
                vdir.mkdir()
                robot = world.robot(actor)
                xyz, rpy = robot.base_xyz(), robot.base_rpy()
                base_pose = (float(xyz[0]), float(xyz[1]), float(rpy[2]))
                actor_inputs = {'view_id': view_id, 'variant': variant, 'seed': seed, 'case': case,
                                'judgment': plan['judgment'], 'actor': actor,
                                'question': plan['question'], 'ticks': [],
                                'order_sheet_file': f'order-sheet-{variant}-s{seed}.json',
                                'map_id': info['static_map']['map_id'],
                                'map_version': info['static_map']['version']}
                labels = {'view_id': view_id, 'variant': variant, 'seed': seed, 'case': case,
                          'geom_table': {str(k): v for k, v in table.items()}, 'ticks': []}
                for tick, pan in enumerate(pans):
                    if tick:
                        jitter = (base_pose[0]+rng.uniform(-TICK_JITTER['xy_m'], TICK_JITTER['xy_m']),
                                  base_pose[1]+rng.uniform(-TICK_JITTER['xy_m'], TICK_JITTER['xy_m']),
                                  base_pose[2]+rng.uniform(-TICK_JITTER['yaw_rad'], TICK_JITTER['yaw_rad']))
                        _set_base(world, actor, jitter)
                        mujoco.mj_forward(world.model, world.data)
                    arm = {**plan['arm'][actor], 6: pan}
                    world._team_joint_move_servos({actor: arm}, .25, settle_s=.25)
                    xyz, rpy = world.robot(actor).base_xyz(), world.robot(actor).base_rpy()
                    tick_pose = (float(xyz[0]), float(xyz[1]), float(rpy[2]))
                    if plan['held'] is not None:
                        grip = world.robot(actor).site_xyz('grip_site')
                        offset = HELD_Z_OFFSET['box' if items[plan['held']]['family'] == 'box' else 'cargo']
                        _set_item(world, items, plan['held'], (float(grip[0]), float(grip[1])),
                                  float(grip[2])+offset, tick_pose[2])
                        mujoco.mj_forward(world.model, world.data)
                    name = f'{actor}--tick{tick}--pan{pan}'
                    (vdir/f'{name}.jpg').write_bytes(world.render_jpeg(robot_id=actor, camera='robot_cam',
                                                                      quality=90))
                    seg = _segment(world, None, actor)
                    cv2.imwrite(str(out/'eval-labels'/f'{view_id}--{name}.png'), (seg+1).astype(np.uint16))
                    actor_inputs['ticks'].append({'tick': tick, 'frame': f'{name}.jpg', 'sim_s': float(tick),
                                                  'commanded_arm_pwm': {str(k): int(v) for k, v in arm.items()}})
                    labels['ticks'].append({'tick': tick, 'frame': f'{name}.jpg', 'pan_pwm': pan,
                                           **_truth(plan, world, items, item_gids, seg, tick_pose,
                                                    info['static_map'])})
                (vdir/'actor-inputs.json').write_text(json.dumps(actor_inputs, indent=1))
                (out/'eval-labels'/f'{view_id}.json').write_text(json.dumps(labels))
                manifest['views'].append({'view_id': view_id, 'case': case, 'judgment': plan['judgment'],
                                          'variant': variant, 'seed': seed, 'ticks': len(pans),
                                          'pans': list(pans)})
        finally:
            world.close()
    load['end'] = os.getloadavg()[0]
    manifest['load_average_1min'] = load
    manifest['wall_s'] = round(time.monotonic()-started, 1)
    (out/'manifest.json').write_text(json.dumps(manifest, indent=1))
    print(json.dumps({'views': len([v for v in manifest['views'] if 'skipped' not in v]),
                      'skipped': len([v for v in manifest['views'] if 'skipped' in v]),
                      'load_average_1min': load, 'wall_s': manifest['wall_s']}))


# ------------------------------------------------------------------ scoring

def _belief(level, base_pose, rng):
    sigma_m, sigma_rad = POSE_BELIEF_LEVELS[level]
    confidence = {'gt_stub_eval_only': 'high', 'noise_30mm': 'medium', 'noise_60mm': 'low'}[level]
    return {'x_m': base_pose[0]+rng.gauss(0., sigma_m), 'y_m': base_pose[1]+rng.gauss(0., sigma_m),
            'yaw_rad': base_pose[2]+rng.gauss(0., sigma_rad), 'confidence': confidence,
            'source': 'injected_'+level}


def _slot_base_from_belief(belief, slot_world_m):
    dx, dy = slot_world_m[0]-belief['x_m'], slot_world_m[1]-belief['y_m']
    c, s = math.cos(belief['yaw_rad']), math.sin(belief['yaw_rad'])
    return (c*dx+s*dy, -s*dx+c*dy)


def score_views(frames_dir, level):
    from harness import zone_own_outcome as outcome
    from harness import zone_own_perception as perception
    frames_dir = Path(frames_dir)
    manifest = json.loads((frames_dir/'manifest.json').read_text())
    static_cache = {}
    records = []
    for view in manifest['views']:
        if 'skipped' in view:
            continue
        vid = view['view_id']
        labels = json.loads((frames_dir/'eval-labels'/f'{vid}.json').read_text())
        actor_inputs = json.loads((frames_dir/'frames'/vid/'actor-inputs.json').read_text())
        judgment = actor_inputs['judgment']
        question = actor_inputs['question']
        variant = actor_inputs['variant']
        if variant not in static_cache:
            from sim.zone_arena import authored_map
            static_cache[variant] = authored_map(variant)
        static = static_cache[variant]
        rng = random.Random(f'{vid}:{level}')
        observations = []
        for tick_input, tick_label in zip(actor_inputs['ticks'], labels['ticks']):
            frame = (frames_dir/'frames'/vid/tick_input['frame']).read_bytes()
            pose = {int(k): int(v) for k, v in tick_input['commanded_arm_pwm'].items()}
            belief = _belief(level, tick_label['base_pose'], rng)
            if judgment == 'slot_item':
                answer = perception.judge_slot_item(
                    frame, pose, slot_base_xy=_slot_base_from_belief(belief, question['slot_world_m']),
                    expected_kind=question['expected_kind'], slot_id=question['slot_id'])
            elif judgment == 'holding_item':
                answer = perception.judge_holding_item(frame, pose, expected_kind=question['expected_kind'])
            elif judgment == 'placed_in_slot':
                answer = perception.judge_placed_in_slot(
                    frame, pose, slot_base_xy=_slot_base_from_belief(belief, question['slot_world_m']),
                    kind=question['kind'], zone=question['zone'], slot_id=question['slot_id'])
            else:
                answer = perception.judge_route_blockage(
                    frame, pose, static_map=static, pose_belief=belief,
                    passage_id=question['passage_id'],
                    lane_half_width_m=max(.10, question['lane_width_m']/2-.02))
            observations.append({**answer, 'pan_pwm': tick_label['pan_pwm']})
            records.append({'record': 'tick', 'view_id': vid, 'case': labels['case'], 'judgment': judgment,
                            'variant': variant, 'seed': labels['seed'], 'belief_level': level,
                            'tick': tick_input['tick'], 'answer': answer['answer'],
                            'confidence': answer['confidence'], 'reason': answer['reason'],
                            'observed': answer.get('observed'), 'truth': tick_label['truth'],
                            'truth_observed': tick_label['truth_observed'],
                            'observable': tick_label['observable'],
                            'peer_in_lane': tick_label.get('peer_in_lane', False),
                            'mapped_wall_view': tick_label.get('mapped_wall_view', False)})
        decision = outcome.track(judgment, observations, question=question)
        final = labels['ticks'][-1]
        records.append({'record': 'view', 'view_id': vid, 'case': labels['case'], 'judgment': judgment,
                        'variant': variant, 'seed': labels['seed'], 'belief_level': level,
                        'ticks': len(observations), 'status': decision['status'],
                        'answer': decision['answer'], 'confidence': decision['confidence'],
                        'reason': decision['reason'], 'observed': decision.get('observed'),
                        'unknown_ticks': decision['unknown_ticks'], 'truth': final['truth'],
                        'truth_observed': final['truth_observed'], 'observable': final['observable'],
                        'peer_in_lane': final.get('peer_in_lane', False),
                        'mapped_wall_view': final.get('mapped_wall_view', False)})
    return records, manifest


CONFIDENT = .65      # PREREGISTERED: the tracker commit threshold


def summarize(records):
    out = {}
    for scope in ('tick', 'view'):
        rows = [r for r in records if r['record'] == scope]
        per = {}
        for judgment in sorted({r['judgment'] for r in rows}):
            sel = [r for r in rows if r['judgment'] == judgment]
            # A peer robot in the lane is not an unmapped object; that discrimination
            # is out of this package's scope and is reported on its own.
            main = [r for r in sel if not r.get('peer_in_lane')]
            per[judgment] = {'by_case': {}, **_metrics(main),
                             'observable': _metrics([r for r in main if r['observable']]),
                             'not_observable': _metrics([r for r in main if not r['observable']]),
                             'peer_in_lane_reported_separately': _metrics([r for r in sel
                                                                          if r.get('peer_in_lane')]),
                             'mapped_wall_view': _metrics([r for r in sel if r.get('mapped_wall_view')])}
            for case in sorted({r['case'] for r in sel}):
                per[judgment]['by_case'][case] = _metrics([r for r in sel if r['case'] == case])
        out[scope] = per
    return out


def _metrics(rows):
    if not rows:
        return {'n': 0}
    unknown = [r for r in rows if r['answer'] == 'unknown']
    decided = [r for r in rows if r['answer'] != 'unknown']
    correct = [r for r in decided if r['answer'] == r['truth']]
    wrong = [r for r in decided if r['answer'] != r['truth']]
    confident_wrong = [r for r in wrong if r['confidence'] >= CONFIDENT]
    return {'n': len(rows), 'unknown': len(unknown), 'unknown_rate': round(len(unknown)/len(rows), 4),
            'decided': len(decided), 'correct': len(correct), 'wrong': len(wrong),
            'accuracy_of_decided': round(len(correct)/len(decided), 4) if decided else None,
            'accuracy_of_all': round(len(correct)/len(rows), 4),
            'false_confident': len(confident_wrong),
            'false_confident_rate': round(len(confident_wrong)/len(rows), 4),
            'false_confident_cases': sorted({r['case'] for r in confident_wrong}),
            'wrong_examples': [{'view_id': r['view_id'], 'answer': r['answer'], 'truth': r['truth'],
                                'reason': r['reason'], 'confidence': r['confidence']} for r in wrong[:6]]}


def score(args):
    frames_dir = Path(args.frames)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    load = {'start': os.getloadavg()[0]}
    levels = args.belief_level or list(POSE_BELIEF_LEVELS)
    results, manifest = {}, None
    for level in levels:
        records, manifest = score_views(frames_dir, level)
        (out/f'records-{level}.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
        results[level] = summarize(records)
    load['end'] = os.getloadavg()[0]
    summary = {'schema': SCHEMA+'.score', 'frames_dir': str(frames_dir), 'split': manifest['split'],
               'render_source_sha': manifest['source_sha'],
               'score_source_sha': _git('rev-parse', 'HEAD'),
               'score_source_dirty': bool(_git('status', '--porcelain')),
               'confident_threshold': CONFIDENT, 'primary_belief_level': PRIMARY_BELIEF,
               'load_average_1min': load, 'results': results}
    (out/'summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps({'written': str(out/'summary.json'), 'load_average_1min': load}))


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    r = sub.add_parser('render', help='render wrist frames + eval labels for one split')
    r.add_argument('--split', choices=('dev', 'test'), required=True)
    r.add_argument('--output', type=Path, required=True)
    s = sub.add_parser('score', help='run the judgments on rendered frames and score them')
    s.add_argument('--frames', type=Path, required=True)
    s.add_argument('--output', type=Path, required=True)
    s.add_argument('--belief-level', action='append', choices=sorted(POSE_BELIEF_LEVELS))
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
