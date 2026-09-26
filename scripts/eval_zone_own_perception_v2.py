"""Offline evaluation of the v2 wrist-RGB-only zone judgments (render / score).

Same two finite local steps as package B v1 (``scripts/eval_zone_own_perception.py``,
``experiments/2026-09-26-zone-own-perception``), for the three follow-up
judgments:

* ``peer_in_lane``           a peer MasterPi ahead, or an unmapped object?
* ``team_cargo_identity``    ``long_beam`` / ``heavy_crate`` / ``tri_frame`` from
                             partial views: at a pickup slot, while approaching a
                             handle, and while carrying.
* ``team_cargo_handle``      "my handle is here".
* ``held_item``              what am I holding, in the held-check posture that
                             ``scripts/probe_held_can_posture.py`` selected (v1
                             could not answer this for a can).

``render``  builds the standard ``CargoZoneScene`` (``cargo_noslip_v1``, the
            profile the team-cargo path selects; weld OFF), poses robots, cargo
            and one unmapped obstruction per view, and writes only what a robot
            may receive to ``frames/``: its own wrist RGB, its own issued arm
            pulses, the map id/version and the scenario order sheet. Ground truth
            (world poses, segmentation, the truth answer) goes ONLY to
            ``eval-labels/``.
``score``   runs ``harness.zone_own_perception_v2`` on ``frames/`` and scores it
            against ``eval-labels/``; multi-tick views go through
            ``harness.zone_own_outcome_v2.JudgmentTrackerV2``.

Render notes that must stay visible in every report:
* ``carry_*`` and ``held_*`` views pose the item between the jaws for a single
  frame and render without stepping physics. Weld stays OFF and no contact
  assistance is added: a *perception* fixture, never evidence that a grasp held.
  Whether the held-check posture keeps a real grasp is a separate physics result
  (``scripts/probe_held_can_posture.py retention``).
* ``object_*`` views add one extra body, ``unmapped_block``, deliberately absent
  from the static map; ``peer_*`` views put a second MasterPi in the lane instead.
  Those are the conditions under test and are recorded in the manifest.
* Cameras, robot appearance, box paint and the cargo catalogue are unchanged.
  ``maps/zones/*``, ``sim/zone_scene.py`` and ``sim/zone_arena.py`` are not
  modified (other packages own them).
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

# Read-only reuse of the v1 render helpers (those files are not modified).
from scripts.eval_zone_color_detection import (_segment, _to_base, _valid_own_region,  # noqa: E402
                                              _yaw_quat, sha256)
from scripts.eval_zone_own_perception import (UNMAPPED_BLOCK, _nearest_interior_wall,  # noqa: E402
                                              pickup_slots)

SPLIT_FILE = ROOT/'experiments/2026-09-26-zone-own-perception-v2/split.json'
SCHEMA = 'ugrp.zone_own_perception_v2_eval.v1'
CONTACT_PROFILE = 'cargo_noslip_v1'

EVAL_GOAL = {'A': {'cyan': 1, 'red': 1}, 'B': {'cyan': 1, 'green': 1}, 'C': {'yellow': 1, 'green': 1}}
EXTRA_BOXES = {'cyan': 1, 'yellow': 1}
# Solo cargo stays in the scene as a distractor; the team items are the subject.
CARGO_ITEMS = (('can1', 'can'), ('tile1', 'tile'), ('beam1', 'long_beam'),
               ('crate1', 'heavy_crate'), ('frame1', 'tri_frame'))
TEAM_ITEMS = {'long_beam': 'beam1', 'heavy_crate': 'crate1', 'tri_frame': 'frame1'}
HELD_ITEMS = {'can': 'can1', 'tile': 'tile1'}

BASE_Z_M = .032355118817659255
HELD_Z_OFFSET = {'box': -.008, 'cargo': -.024}
BOX_HALF_Z_M = .016
FOLDED = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
LOOK = {1: 2000, 3: 1072, 4: 2400, 5: 1482, 6: 1500}
# Approach-stage views use the pre-grasp look posture: the travel look posture
# only sees the floor from about 0.45 m, so a handle inside the reach band is
# below its frame (dev finding).
APPROACH_LOOK = {1: 2000, 3: 973, 4: 2212, 5: 1959, 6: 1500}

# Every v2 view gets three observation ticks at the fixed cadence. A partial view
# has to agree with itself twice before the tracker commits, and dev showed both a
# handle and a held can that appear in one tick and not in the next. The cadence,
# the pan and the number of ticks are identical for every view, so no
# communication condition can buy a cheaper observation.
STRAIGHT3 = (1500, 1500, 1500)
TICK_JITTER = {'xy_m': .015, 'yaw_rad': math.radians(1.2)}
VIEW_PLAN = (
    # peer_in_lane: a robot in the lane, an object in the lane, or nothing
    ('peer_in_passage', STRAIGHT3), ('peer_in_open', STRAIGHT3), ('peer_near_wall', STRAIGHT3),
    ('object_in_passage', STRAIGHT3), ('object_in_open', STRAIGHT3), ('clear_lane', STRAIGHT3),
    # team cargo, four stages per kind. The approach standoff is rendered twice
    # because the two judgments need different postures (dev finding): identity
    # from the travel look posture (``near_*``), the handle from the pre-grasp look
    # posture (``grasp_*``), which is the only one that sees the reach band floor.
    ('slot_long_beam', STRAIGHT3), ('slot_heavy_crate', STRAIGHT3), ('slot_tri_frame', STRAIGHT3),
    ('near_long_beam', STRAIGHT3), ('near_heavy_crate', STRAIGHT3), ('near_tri_frame', STRAIGHT3),
    ('grasp_long_beam', STRAIGHT3), ('grasp_heavy_crate', STRAIGHT3), ('grasp_tri_frame', STRAIGHT3),
    ('carry_long_beam', STRAIGHT3), ('carry_heavy_crate', STRAIGHT3), ('carry_tri_frame', STRAIGHT3),
    ('slot_wrong_team_kind', STRAIGHT3), ('wrong_end_long_beam', STRAIGHT3), ('slot_no_team_cargo', STRAIGHT3),
    # held_item: the held-check posture, with CARRY as the recorded comparison
    ('held_can', STRAIGHT3), ('held_can_absent', STRAIGHT3), ('held_can_wrong_kind', STRAIGHT3),
    ('held_box_carry_posture', STRAIGHT3), ('held_can_carry_posture', STRAIGHT3),
    ('held_box_held_check', STRAIGHT3),
)
PEER_CASES = ('peer_in_passage', 'peer_in_open', 'peer_near_wall', 'object_in_passage',
              'object_in_open', 'clear_lane')
TEAM_STAGE = {'slot': 'slot', 'near': 'near', 'grasp': 'grasp', 'carry': 'carry', 'wrong': 'grasp'}
POSE_BELIEF_LEVELS = {'gt_stub_eval_only': (0., 0.), 'noise_30mm': (.030, math.radians(3.)),
                      'noise_60mm': (.060, math.radians(6.))}
PRIMARY_BELIEF = 'gt_stub_eval_only'

# Truth conventions (PREREGISTERED in the experiment README).
MIN_VISIBLE_PX = {'peer_in_lane': 300, 'team_cargo_identity': 400, 'team_cargo_handle': 400,
                  'held_item': 500}
MIN_ITEM_SATURATION_P90 = 60      # PREREGISTERED: an observable team item still carries colour
MIN_HANDLE_VISIBLE_PX = 120       # PREREGISTERED: a black lug/grip band of the item is in frame
# 'held_item' counts pixels inside zone_own_perception_v2.HELD_CHECK_ROI only.
# Pre-grasp reach band, identical to zone_own_perception_v2.HANDLE_REACH_BAND_M:
# "the handle I would take is in front of me, within one approach step".
HANDLE_TRUTH_BAND_M = (.18, .45)
HANDLE_TRUTH_LATERAL_M = .10
PEER_TRUTH_RANGE_M = 2.0
CONFIDENT = .65      # PREREGISTERED: the tracker commit threshold


def _git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def load_split():
    return json.loads(SPLIT_FILE.read_text())


# ------------------------------------------------------------------ scenario config

def order_sheet(variant, seed, config, slots, cargo_poses):
    """Immutable order sheet from the scenario config, never from the simulator."""
    from sim.zone_cargo import kind as cargo_kind
    rows = []
    for item_id, spec in config['setup_only']['objects'].items():
        xy = [round(float(v), 4) for v in spec['position_m'][:2]]
        slot = next((sid for sid, c in slots.items() if math.dist(c, xy) < 1e-3), None)
        rows.append((item_id, spec['kind'], slot, xy, 'box', 1))
    for item_id, kind in CARGO_ITEMS:
        xy = [round(float(v), 4) for v in cargo_poses[item_id][:2]]
        slot = next((sid for sid, c in slots.items() if math.dist(c, xy) < 1e-3), None)
        rows.append((item_id, kind, slot, xy, 'cargo', cargo_kind(kind).required_carriers))
    zones = ('A', 'B', 'C')
    orders = []
    for index, (item_id, kind, slot, xy, family, carriers) in enumerate(sorted(rows)):
        orders.append({'order_id': f'order-{index+1}', 'item_ids': [item_id], 'kind': kind, 'count': 1,
                       'required_robots': int(carriers), 'destination_zone': zones[index % 3],
                       'initial_location': {'pickup_bay': 'P', 'slot': slot, 'declared_center_m': xy},
                       'item_family': family})
    return {'schema': 'ugrp.zone_order.v1', 'map_id': 'zone_'+variant.split('_', 1)[1],
            'variant': variant, 'seed': seed, 'orders': orders,
            'note': ('initial_location is where the scenario placed the item at setup; it is not a '
                     'guarantee about the current state')}


# ------------------------------------------------------------------ world

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


def _team_park(bounds, index):
    """Team items are long: park them along the far edge, well apart."""
    return (bounds[0]+.5+index*.9, bounds[3]-.6)


def build_world(variant, seed, rng):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo_scene import CargoZoneScene
    slots = pickup_slots(variant)
    free = list(slots.values())
    rng.shuffle(free)
    cargo_poses = {}
    for index, (item_id, kind) in enumerate(CARGO_ITEMS):
        cell = free[index % len(free)]
        cargo_poses[item_id] = (round(cell[0], 4), round(cell[1], 4), 0.)
    items_spec = [{'item_id': item_id, 'kind': kind, 'pose': list(cargo_poses[item_id])}
                  for item_id, kind in CARGO_ITEMS]
    scene = CargoZoneScene.from_cargo_config(variant, seed, cargo=items_spec, goal=EVAL_GOAL,
                                            extra_boxes=dict(EXTRA_BOXES),
                                            contact_profile=CONTACT_PROFILE)
    base_transform = scene.transform
    extras = {}

    def transform(xml):
        xml = base_transform(xml)
        xml = _unmapped_block_xml(xml)
        extras['scene_xml_with_extras_sha256'] = hashlib.sha256(xml.encode()).hexdigest()
        return xml

    world = MultiMasterPiProductionV2(seed=seed, width=640, height=480, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=transform)
    scene.setup(world)
    config = scene.config
    items = {oid: {'kind': o['kind'], 'body': o['body_name'], 'joint': o['joint_name'], 'family': 'box'}
             for oid, o in config['setup_only']['objects'].items()}
    for inst in scene.cargo:
        items[inst.item_id] = {'kind': inst.kind, 'body': inst.body, 'joint': inst.joint,
                               'family': 'cargo'}
    info = {'config': config, 'static_map': config['static_map'], 'slots': slots, 'variant': variant,
            'order_sheet': order_sheet(variant, seed, config, slots, cargo_poses),
            'scene_xml_sha256': scene.manifest['scene_xml_sha256'],
            'scene_extras': {**extras, 'cargo': scene.manifest.get('cargo'),
                             'cargo_contact_profile': scene.manifest.get('cargo_contact_profile')},
            'cargo_setup_poses': {k: list(v) for k, v in cargo_poses.items()},
            'cargo_instances': {inst.item_id: inst for inst in scene.cargo},
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
    bounds = info['static_map']['bounds_m']
    team = 0
    for index, item_id in enumerate(sorted(items)):
        if items[item_id]['kind'] in TEAM_ITEMS:
            xy = _team_park(bounds, team)
            team += 1
        else:
            xy = (bounds[0]+.25+(index % 4)*.22, bounds[2]+.25+(index//4)*.22)
        _set_item(world, items, item_id, xy, _floor_z(items, item_id), 0.)
    _set_block(world, UNMAPPED_BLOCK['park_xy'])


def _lane_target(info, rng):
    passages = [p for p in info['static_map'].get('passages', []) if p['kind'] != 'passing_bay']
    if not passages:
        return None
    passage = passages[rng.randrange(len(passages))]
    return {'id': passage['id'], 'center_m': list(passage['center_m']),
            'width_m': float(passage['width_m']), 'kind': passage['kind']}


def _open_target(info, rng):
    region = info['static_map']['regions']['pickup']
    return {'id': None, 'center_m': [region['center_m'][0]+rng.uniform(-.4, .4),
                                     region['center_m'][1]+rng.uniform(-1., 1.)],
            'width_m': 2*.22, 'kind': 'open'}


def _free_floor_xy(info, rng):
    """An open patch of floor on the pickup side, away from walls and zone paint."""
    region = info['static_map']['regions']['pickup']
    cx, cy = region['center_m']
    return (cx+rng.uniform(-.5, .5), cy+rng.uniform(-1.1, 1.1))


def plan_view(case, rng, info, items, robot_ids, instances):
    """Robot poses, item placements, arm pulses and the question for one view."""
    from sim.zone_cargo import world_grasps
    static = info['static_map']
    actor = robot_ids[0]
    plan = {'case': case, 'actor': actor, 'items': {}, 'robots': {}, 'arm': {}, 'question': {},
            'block_xy': None, 'held': None, 'held_posture': 'held_check', 'judgments': [],
            'team_stage': None, 'team_item': None}
    if case in PEER_CASES:
        plan['judgments'] = ['peer_in_lane']
        lane = _lane_target(info, rng) if 'passage' in case or case == 'peer_near_wall' else None
        if lane is None:
            lane = _open_target(info, rng)
        centre = lane['center_m']
        yaw = math.atan2(rng.uniform(-.12, .12), 1.) + (0. if centre[0] >= 0 else math.pi)
        distance = rng.uniform(.75, 1.25)
        plan['robots'][actor] = _robot_facing(centre, distance, rng.uniform(-.05, .05), yaw)
        if case.startswith('peer'):
            spot = centre
            if case == 'peer_near_wall':
                wall = _nearest_interior_wall(static, centre)
                if wall is None:
                    return None
                spot = (wall[0]-.12, wall[1])
                plan['robots'][actor] = _robot_facing(spot, rng.uniform(.65, 1.0),
                                                      rng.uniform(-.04, .04), math.atan2(0., 1.))
            plan['robots'][robot_ids[1]] = (spot[0]+rng.uniform(-.04, .04), spot[1]+rng.uniform(-.04, .04),
                                           rng.uniform(-math.pi, math.pi))
        elif case.startswith('object'):
            plan['block_xy'] = (centre[0]+rng.uniform(-.04, .04), centre[1]+rng.uniform(-.05, .05))
        plan['arm'][actor] = ({1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500} if rng.random() < .5
                              else {**FOLDED, 3: 1072, 4: 2400, 5: 1482, 6: 1500})
        plan['question'] = {'passage_id': lane['id'], 'lane_center_m': list(centre),
                            'lane_width_m': lane['width_m'], 'lane_kind': lane['kind']}
        return _validate(plan, static, robot_ids)

    if case.startswith('held'):
        plan['judgments'] = ['held_item']
        kind = {'held_can': 'can', 'held_can_absent': 'can', 'held_can_wrong_kind': 'can',
                'held_box_carry_posture': 'cyan', 'held_can_carry_posture': 'can',
                'held_box_held_check': 'cyan'}[case]
        held = None
        if case == 'held_can_wrong_kind':
            held = HELD_ITEMS['tile']
        elif case in ('held_box_carry_posture', 'held_box_held_check'):
            held = next((oid for oid, spec in items.items()
                         if spec['family'] == 'box' and spec['kind'] == 'cyan'), None)
            if held is None:
                return None
        elif case != 'held_can_absent':
            held = HELD_ITEMS[kind]
        plan['held'] = held
        plan['held_posture'] = 'carry' if case.endswith('carry_posture') else 'held_check'
        from harness.zone_own_perception_v2 import HELD_CHECK_POSTURE
        from harness.zone_own_perception import CARRY_POSTURE
        posture = CARRY_POSTURE if plan['held_posture'] == 'carry' else HELD_CHECK_POSTURE
        plan['arm'][actor] = {1: 1500, **{k: v for k, v in posture.items() if k != 1}}
        xy = _free_floor_xy(info, rng)
        plan['robots'][actor] = (xy[0], xy[1], rng.uniform(-math.pi, math.pi))
        plan['question'] = {'expected_kind': kind, 'posture_name': plan['held_posture']}
        return _validate(plan, static, robot_ids)

    # team cargo
    family = case.split('_')[0]
    stage = TEAM_STAGE[family]
    kind = ('heavy_crate' if case == 'slot_no_team_cargo' else
            'long_beam' if case in ('slot_wrong_team_kind', 'wrong_end_long_beam') else
            case.split('_', 1)[1])
    plan['team_stage'] = stage
    plan['question'] = {'expected_kind': kind, 'stage': stage}
    present = kind
    if case == 'slot_wrong_team_kind':
        present = 'tri_frame'
    elif case == 'slot_no_team_cargo':
        present = None
    plan['team_item'] = None if present is None else TEAM_ITEMS[present]
    spot = _free_floor_xy(info, rng)
    if case.startswith('carry'):
        plan['judgments'] = ['team_cargo_identity']
        plan['held'] = plan['team_item']
        plan['arm'][actor] = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
        plan['robots'][actor] = (spot[0], spot[1], rng.uniform(-math.pi, math.pi))
        return _validate(plan, static, robot_ids)
    plan['judgments'] = (['team_cargo_identity'] if case.startswith('near')
                         else ['team_cargo_identity', 'team_cargo_handle'])
    item_yaw = rng.uniform(-math.pi, math.pi)
    if present is not None:
        plan['items'][plan['team_item']] = ((spot[0], spot[1]), item_yaw)
    if case.startswith(('near', 'grasp')):
        # Standing at the RGB approach standoff, not with the jaws already around
        # the handle: at the final grasp base pose the item is 2-5 cm from the lens
        # and the render blows out to white (dev: item saturation median 3-7).
        grasps = world_grasps(instances[plan['team_item']], (spot[0], spot[1], item_yaw))
        role = sorted(grasps)[rng.randrange(len(grasps))]
        base = grasps[role]['base_xyyaw']
        grip = grasps[role]['grip_xyz']
        standoff = rng.uniform(.24, .40)
        plan['robots'][actor] = (grip[0]-standoff*math.cos(base[2])+rng.uniform(-.012, .012),
                                 grip[1]-standoff*math.sin(base[2])+rng.uniform(-.012, .012),
                                 base[2]+rng.uniform(-.03, .03))
        plan['question']['grasp_role'] = role
        plan['question']['standoff_m'] = round(standoff, 4)
    elif case == 'wrong_end_long_beam':
        # Facing the middle of the beam: the item is identifiable, no handle is in reach.
        yaw = item_yaw + math.pi/2
        plan['robots'][actor] = _robot_facing((spot[0], spot[1]), rng.uniform(.30, .42),
                                              rng.uniform(-.03, .03), yaw)
    else:
        yaw = item_yaw + rng.choice((math.pi/2, -math.pi/2))
        plan['robots'][actor] = _robot_facing((spot[0], spot[1]), rng.uniform(.52, .78),
                                              rng.uniform(-.05, .05), yaw)
    near_stage = case.startswith(('grasp', 'wrong_end'))
    plan['arm'][actor] = dict(APPROACH_LOOK if near_stage else LOOK)
    plan['question']['look_posture'] = 'approach_look' if near_stage else 'travel_look'
    return _validate(plan, static, robot_ids)


def _validate(plan, static, robot_ids):
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
        plan['robots'][rid] = (bounds[0]+.35+index*.30, bounds[2]+1.2, 0.)
    for rid in robot_ids:
        plan['arm'].setdefault(rid, dict(FOLDED))
    return plan


# ------------------------------------------------------------------ labels

def _handle_geoms(world, items, instances):
    """Geom ids of the black lugs and grip bands of each team item (eval only).

    These are exactly the catalogue parts painted ``HANDLE_RGBA``: the two grip
    bands of ``long_beam``, the two end lugs of ``heavy_crate`` and the three
    vertex lugs of ``tri_frame``. A handle question is only observable when one of
    them is actually in the frame.
    """
    import mujoco
    from sim.zone_cargo import HANDLE_RGBA
    out = {}
    for item_id, inst in instances.items():
        gids = []
        for part in inst.spec().parts:
            if part.rgba != HANDLE_RGBA:
                continue
            gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM, inst.geom(part.name))
            if gid >= 0:
                gids.append(gid)
        if gids:
            out[item_id] = gids
    return out


def _geom_categories(world, items):
    import mujoco
    body_to_item = {spec['body']: oid for oid, spec in items.items()}
    table, item_gids, robot_gids = {}, {}, {}
    for gid in range(world.model.ngeom):
        name = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ''
        bid = int(world.model.geom_bodyid[gid])
        body = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY, bid) or ''
        root = mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY,
                                 int(world.model.body_rootid[bid])) or ''
        item = next((oid for b, oid in body_to_item.items() if body == b or body.startswith(b+'__')), None)
        if item is not None:
            table[gid] = {'category': 'item', 'ref': item}
            item_gids.setdefault(item, []).append(gid)
        elif body == UNMAPPED_BLOCK['name']:
            table[gid] = {'category': 'unmapped_block', 'ref': UNMAPPED_BLOCK['name']}
            item_gids.setdefault(UNMAPPED_BLOCK['name'], []).append(gid)
        elif root.endswith('__robot') or name.split('__')[0] in ('r1', 'r2', 'r3'):
            ref = root.split('__')[0] or name.split('__')[0]
            table[gid] = {'category': 'robot', 'ref': ref}
            robot_gids.setdefault(ref, []).append(gid)
        elif name == 'floor':
            table[gid] = {'category': 'floor', 'ref': name}
        elif 'wall' in name:
            table[gid] = {'category': 'wall', 'ref': name}
        elif name.startswith(('zone_', 'dispatch_')):
            table[gid] = {'category': 'floor_paint', 'ref': name}
        else:
            table[gid] = {'category': 'other', 'ref': name or body}
    return table, item_gids, robot_gids


def _item_colour_p90(frame_bgr, seg, gids):
    """90th-percentile saturation inside the item silhouette (eval only).

    Observability rule for the team judgments: the wrist camera sits 2-5 cm from a
    *carried* item, where the ceiling light either blows the surface out to white
    or leaves it in the arm's shadow. Dev measurement inside the silhouette: a
    carried beam keeps S 186-190, a carried crate drops to S 0 (V 7-13) and is
    unreadable. An item is called observable only when some of it still carries
    saturated colour; that is a property of the frame, not a detector threshold.
    """
    import cv2
    mask = np.isin(seg, gids)
    if not mask.any():
        return 0.
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    return float(np.percentile(hsv[..., 1][mask], 90))


def _visible(seg, gids):
    from harness.zone_own_perception_v2 import HELD_CHECK_ROI
    mask = np.isin(seg, gids)
    count = int(mask.sum())
    height, width = seg.shape[:2]
    x0, y0, x1, y1 = (int(round(HELD_CHECK_ROI[0]*width)), int(round(HELD_CHECK_ROI[1]*height)),
                      int(round(HELD_CHECK_ROI[2]*width)), int(round(HELD_CHECK_ROI[3]*height)))
    in_roi = int(mask[y0:y1, x0:x1].sum())
    if not count:
        return {'visible_px': 0, 'visible_in_held_roi_px': 0, 'seg_centroid_px': None}
    ys, xs = np.nonzero(mask)
    return {'visible_px': count, 'visible_in_held_roi_px': in_roi,
            'seg_centroid_px': [float(xs.mean()), float(ys.mean())]}


def _handle_truth(plan, world, items, instances, base_pose):
    """Is a handle of the present item inside the calibrated reach band (eval only)?"""
    from sim.zone_cargo import world_grasps
    item_id = plan['team_item']
    if item_id is None:
        return {'truth': 'no', 'handles_base_m': [], 'note': 'no team cargo present'}
    if items[item_id]['kind'] != plan['question']['expected_kind']:
        return {'truth': 'no', 'handles_base_m': [],
                'note': 'the item present is not the expected kind, so no handle of it is here'}
    body = world.data.body(items[item_id]['body'])
    mat = np.asarray(body.xmat, float).reshape(3, 3)
    pose = (float(body.xpos[0]), float(body.xpos[1]), float(math.atan2(mat[1, 0], mat[0, 0])))
    rows, best = [], 'no'
    for role, grasp in world_grasps(instances[item_id], pose).items():
        gx, gy, _ = grasp['grip_xyz']
        bx, by = _to_base(base_pose, (gx, gy))
        inside = (HANDLE_TRUTH_BAND_M[0] <= bx <= HANDLE_TRUTH_BAND_M[1]
                  and abs(by) <= HANDLE_TRUTH_LATERAL_M)
        rows.append({'role': role, 'base_m': [round(bx, 4), round(by, 4)], 'in_reach': bool(inside)})
        if inside:
            best = 'yes'
    return {'truth': best, 'handles_base_m': rows}


def _truth(plan, world, items, instances, item_gids, handle_gids, robot_gids, seg, base_pose, frame_bgr):
    """Truth answers for this view's judgments, from ground truth (eval only)."""
    world_xy = {oid: [float(v) for v in world.data.body(spec['body']).xpos[:2]]
                for oid, spec in items.items()}
    block_xy = [float(v) for v in world.data.body(UNMAPPED_BLOCK['name']).xpos[:2]]
    visible = {oid: _visible(seg, gids) for oid, gids in item_gids.items()}
    peers = {rid: _visible(seg, gids) for rid, gids in robot_gids.items() if rid != plan['actor']}
    row = {'case': plan['case'], 'judgments': list(plan['judgments']), 'world_xy': world_xy,
           'block_world_xy': block_xy, 'visible': visible, 'peer_visible': peers,
           'base_pose': list(base_pose), 'truth': {}, 'truth_observed': {}, 'observable': {}}
    if 'peer_in_lane' in plan['judgments']:
        half = max(.10, plan['question']['lane_width_m']/2-.02)

        def in_lane(xy):
            bx, by = _to_base(base_pose, xy)
            return .05 < bx <= PEER_TRUTH_RANGE_M and abs(by) <= half+.05

        peer_ids = [rid for rid in robot_gids if rid != plan['actor']]
        peer_in = [rid for rid in peer_ids
                   if in_lane([float(v) for v in world.data.body(f'{rid}__robot').xpos[:2]])]
        block_in = plan['block_xy'] is not None and in_lane(block_xy)
        truth = 'yes' if peer_in else 'no'
        row['truth']['peer_in_lane'] = truth
        row['truth_observed']['peer_in_lane'] = ('peer_robot' if peer_in else
                                                 ('unmapped_object' if block_in else 'clear'))
        seen = max([peers[rid]['visible_px'] for rid in peer_in], default=0)
        block_seen = visible.get(UNMAPPED_BLOCK['name'], {'visible_px': 0})['visible_px']
        row['observable']['peer_in_lane'] = bool(
            seen >= MIN_VISIBLE_PX['peer_in_lane'] if peer_in else
            (block_seen >= MIN_VISIBLE_PX['peer_in_lane'] if block_in else True))
        row['peer_in_lane_detail'] = {'peers_in_lane': peer_in, 'block_in_lane': bool(block_in),
                                      'lane_half_width_m': round(half, 4)}
    if 'team_cargo_identity' in plan['judgments']:
        expected = plan['question']['expected_kind']
        present_id = plan['team_item']
        present_kind = None if present_id is None else items[present_id]['kind']
        row['truth']['team_cargo_identity'] = 'yes' if present_kind == expected else 'no'
        row['truth_observed']['team_cargo_identity'] = present_kind or 'empty'
        seen = 0 if present_id is None else visible[present_id]['visible_px']
        sat90 = (0. if present_id is None
                 else _item_colour_p90(frame_bgr, seg, item_gids[present_id]))
        row['observable']['team_cargo_identity'] = bool(
            present_id is not None and seen >= MIN_VISIBLE_PX['team_cargo_identity']
            and sat90 >= MIN_ITEM_SATURATION_P90)
        row['team_cargo_detail'] = {'present_item': present_id, 'present_kind': present_kind,
                                    'visible_px': seen, 'saturation_p90': round(sat90, 1),
                                    'stage': plan['team_stage']}
    if 'team_cargo_handle' in plan['judgments']:
        handle = _handle_truth(plan, world, items, instances, base_pose)
        row['truth']['team_cargo_handle'] = handle['truth']
        row['truth_observed']['team_cargo_handle'] = ('handle_here' if handle['truth'] == 'yes'
                                                      else 'handle_elsewhere')
        present_id = plan['team_item']
        seen = 0 if present_id is None else visible[present_id]['visible_px']
        sat90 = (0. if present_id is None
                 else _item_colour_p90(frame_bgr, seg, item_gids[present_id]))
        marker_px = (0 if present_id is None or present_id not in handle_gids
                     else _visible(seg, handle_gids[present_id])['visible_px'])
        row['observable']['team_cargo_handle'] = bool(
            present_id is not None and seen >= MIN_VISIBLE_PX['team_cargo_handle']
            and sat90 >= MIN_ITEM_SATURATION_P90 and marker_px >= MIN_HANDLE_VISIBLE_PX
            and items[present_id]['kind'] == plan['question']['expected_kind'])
        row['handle_detail'] = {**handle, 'saturation_p90': round(sat90, 1),
                                'handle_marker_visible_px': marker_px}
    if 'held_item' in plan['judgments']:
        held = plan['held']
        expected = plan['question']['expected_kind']
        row['truth']['held_item'] = ('yes' if held is not None and items[held]['kind'] == expected else 'no')
        row['truth_observed']['held_item'] = items[held]['kind'] if held is not None else 'empty'
        in_roi = 0 if held is None else visible[held]['visible_in_held_roi_px']
        row['observable']['held_item'] = bool(held is None or in_roi >= MIN_VISIBLE_PX['held_item'])
        row['held_detail'] = {'held_item': held, 'posture': plan['held_posture'],
                              'visible_in_held_roi_px': in_roi,
                              'visible_px': 0 if held is None else visible[held]['visible_px']}
    return row


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
                'contact_profile': CONTACT_PROFILE,
                'clock': 'synchronous SIM; arm settled, then one render per tick',
                'weld': 'off',
                'hold_views': 'item posed between the jaws for one frame, physics not stepped',
                'unmapped_block': {k: (list(v) if isinstance(v, tuple) else v)
                                   for k, v in UNMAPPED_BLOCK.items()},
                'pose_belief_levels': {k: list(v) for k, v in POSE_BELIEF_LEVELS.items()},
                'views': [], 'scenes': []}
    started = time.monotonic()
    for spec in specs:
        variant, seed = spec['variant'], spec['seed']
        rng = random.Random(f'{variant}:{seed}:own-perception-v2')
        world, items, info = build_world(variant, seed, rng)
        instances = info['cargo_instances']
        try:
            table, item_gids, robot_gids = _geom_categories(world, items)
            handle_gids = _handle_geoms(world, items, instances)
            robot_ids = list(world.robot_ids)
            manifest['scenes'].append({'variant': variant, 'seed': seed,
                                       'scene_xml_sha256': info['scene_xml_sha256'],
                                       'scene_extras': info['scene_extras'],
                                       'items': {k: v['kind'] for k, v in items.items()},
                                       'order_sheet_sha256': hashlib.sha256(
                                           json.dumps(info['order_sheet'], sort_keys=True).encode()
                                       ).hexdigest()})
            (out/'frames'/f'order-sheet-{variant}-s{seed}.json').write_text(
                json.dumps(info['order_sheet'], indent=1))
            for case, pans in VIEW_PLAN:
                plan = None
                for _ in range(40):
                    plan = plan_view(case, rng, info, items, robot_ids, instances)
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
                vdir = out/'frames'/view_id
                vdir.mkdir()
                xyz, rpy = world.robot(actor).base_xyz(), world.robot(actor).base_rpy()
                base_pose = (float(xyz[0]), float(xyz[1]), float(rpy[2]))
                actor_inputs = {'view_id': view_id, 'variant': variant, 'seed': seed, 'case': case,
                                'judgments': list(plan['judgments']), 'actor': actor,
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
                        family = items[plan['held']]['family']
                        offset = HELD_Z_OFFSET['box' if family == 'box' else 'cargo']
                        _set_item(world, items, plan['held'], (float(grip[0]), float(grip[1])),
                                  float(grip[2])+offset, tick_pose[2])
                        mujoco.mj_forward(world.model, world.data)
                    name = f'{actor}--tick{tick}--pan{pan}'
                    (vdir/f'{name}.jpg').write_bytes(world.render_jpeg(robot_id=actor, camera='robot_cam',
                                                                      quality=90))
                    seg = _segment(world, None, actor)
                    cv2.imwrite(str(out/'eval-labels'/f'{view_id}--{name}.png'), (seg+1).astype(np.uint16))
                    actor_inputs['ticks'].append({'tick': tick, 'frame': f'{name}.jpg',
                                                  'sim_s': float(tick),
                                                  'commanded_arm_pwm': {str(k): int(v)
                                                                        for k, v in arm.items()}})
                    bgr = cv2.imdecode(np.frombuffer((vdir/f'{name}.jpg').read_bytes(), np.uint8), 1)
                    labels['ticks'].append({'tick': tick, 'frame': f'{name}.jpg', 'pan_pwm': pan,
                                            **_truth(plan, world, items, instances, item_gids,
                                                     handle_gids, robot_gids, seg, tick_pose,
                                                     bgr)})
                (vdir/'actor-inputs.json').write_text(json.dumps(actor_inputs, indent=1))
                (out/'eval-labels'/f'{view_id}.json').write_text(json.dumps(labels))
                manifest['views'].append({'view_id': view_id, 'case': case,
                                          'judgments': list(plan['judgments']), 'variant': variant,
                                          'seed': seed, 'ticks': len(pans), 'pans': list(pans)})
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


def score_views(frames_dir, level):
    from harness import zone_own_outcome_v2 as outcome
    from harness import zone_own_perception_v2 as perception
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
        question = actor_inputs['question']
        variant = actor_inputs['variant']
        if variant not in static_cache:
            from sim.zone_arena import authored_map
            static_cache[variant] = authored_map(variant)
        static = static_cache[variant]
        series = {j: [] for j in actor_inputs['judgments']}
        for tick_input, tick_label in zip(actor_inputs['ticks'], labels['ticks']):
            frame = (frames_dir/'frames'/vid/tick_input['frame']).read_bytes()
            pose = {int(k): int(v) for k, v in tick_input['commanded_arm_pwm'].items()}
            rng = random.Random(f'{vid}:{level}:{tick_input["tick"]}')
            belief = _belief(level, tick_label['base_pose'], rng)
            for judgment in actor_inputs['judgments']:
                if judgment == 'peer_in_lane':
                    answer = perception.judge_peer_in_lane(
                        frame, pose, static_map=static, pose_belief=belief,
                        passage_id=question['passage_id'],
                        lane_half_width_m=max(.10, question['lane_width_m']/2-.02))
                elif judgment == 'team_cargo_identity':
                    answer = perception.judge_team_cargo_identity(
                        frame, pose, expected_kind=question['expected_kind'])
                elif judgment == 'team_cargo_handle':
                    answer = perception.judge_team_cargo_handle(
                        frame, pose, expected_kind=question['expected_kind'])
                else:
                    answer = perception.judge_held_item(
                        frame, pose, expected_kind=question['expected_kind'],
                        posture_name=question['posture_name'])
                series[judgment].append({**answer, 'pan_pwm': tick_label['pan_pwm']})
                records.append({'record': 'tick', 'view_id': vid, 'case': labels['case'],
                                'judgment': judgment, 'variant': variant, 'seed': labels['seed'],
                                'belief_level': level, 'tick': tick_input['tick'],
                                'answer': answer['answer'], 'confidence': answer['confidence'],
                                'reason': answer['reason'], 'observed': answer.get('observed'),
                                'truth': tick_label['truth'][judgment],
                                'truth_observed': tick_label['truth_observed'][judgment],
                                'observable': tick_label['observable'][judgment],
                                'stage': (tick_label.get('team_cargo_detail') or {}).get('stage'),
                                'posture': (tick_label.get('held_detail') or {}).get('posture')})
        final = labels['ticks'][-1]
        for judgment, observations in series.items():
            decision = outcome.track(judgment, observations, question=question)
            records.append({'record': 'view', 'view_id': vid, 'case': labels['case'],
                            'judgment': judgment, 'variant': variant, 'seed': labels['seed'],
                            'belief_level': level, 'ticks': len(observations),
                            'status': decision['status'], 'answer': decision['answer'],
                            'confidence': decision['confidence'], 'reason': decision['reason'],
                            'observed': decision.get('observed'),
                            'unknown_ticks': decision['unknown_ticks'],
                            'truth': final['truth'][judgment],
                            'truth_observed': final['truth_observed'][judgment],
                            'observable': final['observable'][judgment],
                            'stage': (final.get('team_cargo_detail') or {}).get('stage'),
                            'posture': (final.get('held_detail') or {}).get('posture')})
    return records, manifest


def summarize(records):
    out = {}
    for scope in ('tick', 'view'):
        rows = [r for r in records if r['record'] == scope]
        per = {}
        for judgment in sorted({r['judgment'] for r in rows}):
            sel = [r for r in rows if r['judgment'] == judgment]
            per[judgment] = {'by_case': {}, 'by_stage': {}, **_metrics(sel),
                             'observable': _metrics([r for r in sel if r['observable']]),
                             'not_observable': _metrics([r for r in sel if not r['observable']])}
            for case in sorted({r['case'] for r in sel}):
                per[judgment]['by_case'][case] = _metrics([r for r in sel if r['case'] == case])
            for stage in sorted({r['stage'] or r['posture'] or '-' for r in sel}):
                per[judgment]['by_stage'][stage] = _metrics(
                    [r for r in sel if (r['stage'] or r['posture'] or '-') == stage])
        out[scope] = per
    return out


def _metrics(rows):
    if not rows:
        return {'n': 0}
    unknown = [r for r in rows if r['answer'] == 'unknown']
    decided = [r for r in rows if r['answer'] != 'unknown']
    correct = [r for r in decided if r['answer'] == r['truth']]
    wrong = [r for r in decided if r['answer'] != r['truth']]
    observed_ok = [r for r in decided if r['answer'] == r['truth']
                   and (r.get('observed') in (None, r.get('truth_observed'))
                        or r['truth'] == 'no')]
    confident_wrong = [r for r in wrong if r['confidence'] >= CONFIDENT]
    return {'n': len(rows), 'unknown': len(unknown), 'unknown_rate': round(len(unknown)/len(rows), 4),
            'decided': len(decided), 'correct': len(correct), 'wrong': len(wrong),
            'accuracy_of_decided': round(len(correct)/len(decided), 4) if decided else None,
            'accuracy_of_all': round(len(correct)/len(rows), 4),
            'observed_label_agrees': len(observed_ok),
            'false_confident': len(confident_wrong),
            'false_confident_rate': round(len(confident_wrong)/len(rows), 4),
            'false_confident_cases': sorted({r['case'] for r in confident_wrong}),
            'wrong_examples': [{'view_id': r['view_id'], 'answer': r['answer'], 'truth': r['truth'],
                                'observed': r.get('observed'), 'reason': r['reason'],
                                'confidence': r['confidence']} for r in wrong[:6]]}


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
    s = sub.add_parser('score', help='run the v2 judgments on rendered frames and score them')
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
