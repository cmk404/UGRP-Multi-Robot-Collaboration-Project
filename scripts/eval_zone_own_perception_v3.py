"""Offline evaluation of the v3 wrist-RGB-only judgments (render / score).

Same two finite local steps as package B v1/v2
(``scripts/eval_zone_own_perception_v2.py``, which is imported read-only):

* ``held_item_at_grip``     is the item at my own grip the ordered kind? The
                            region is the at-grip silhouette predicted from the
                            issued pulses (v2 W7 fix). v2's ``held_item`` is
                            scored on the same frames for the side-by-side.
* ``team_cargo_at_grip``    which team item am I carrying, including the
                            ``heavy_crate`` that renders black in the arm's
                            shadow. v2's ``team_cargo_identity`` is scored on the
                            same frames.
* grasp stage               v2's ``team_cargo_identity`` and
                            ``team_cargo_handle`` at the grasp standoff, once in
                            v2's pre-grasp look posture and once in each v3
                            candidate/selected grasp-look posture (same robot,
                            same item, same tick jitter).

``render``  standard ``CargoZoneScene`` (``cargo_noslip_v1``, weld OFF), writes
            only robot-receivable data to ``frames/`` (own wrist JPEG, own issued
            pulses, map id/version, order sheet) and ground truth ONLY to
            ``eval-labels/``.
``score``   runs the judgments on ``frames/`` and scores them against
            ``eval-labels/``; multi-tick views go through the v1 commit policy.

Render notes that must stay visible in every report:
* ``held_*`` and ``carry_*`` views pose the item at the grip for a single frame
  without stepping physics, by the catalogue hold convention (the grasp role's
  grip point at the grip site, the robot facing the role's approach heading),
  with a small hold jitter. Weld OFF, no contact assistance: a *perception*
  fixture, never evidence that a grasp held.
* v2's fixture held team items by their centroid. v3's fixture holds them by the
  grasp role (lug / beam end / vertex), which is where a robot actually holds
  them; the v2 carry numbers are therefore not directly comparable.
* Cameras, robot appearance, box paint and the cargo catalogue are unchanged.
  ``maps/zones/*``, ``sim/zone_scene.py`` and ``sim/zone_arena.py`` are not
  modified.
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

from scripts import eval_zone_own_perception_v2 as ev2  # noqa: E402  (read-only reuse)
from scripts.eval_zone_color_detection import _segment, sha256  # noqa: E402

SPLIT_FILE = ROOT/'experiments/2026-09-26-zone-own-perception-v3/split.json'
SCHEMA = 'ugrp.zone_own_perception_v3_eval.v1'
CONTACT_PROFILE = ev2.CONTACT_PROFILE

CARRY = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}
HELD_CHECK = {1: 1500, 3: 1500, 4: 1700, 5: 1900, 6: 1500}
APPROACH_LOOK = dict(ev2.APPROACH_LOOK)                       # v2 pre-grasp look posture
# Grasp-stage look candidates for the dev scan. B/C only raise the wrist pitch of
# the v2 posture; D/E are calibrated IK with the grip site 0.10 m ahead at
# 0.16 m, tool pitch -42 / -39 deg (camera higher, floor seen from ~0.21 m).
GRASP_CANDIDATES = {
    'approach_look_v2': APPROACH_LOOK,
    'wrist_m50': {1: 2000, 3: 1033, 4: 2212, 5: 1959, 6: 1500},
    'wrist_m44': {1: 2000, 3: 1093, 4: 2212, 5: 1959, 6: 1500},
    'ik_r10_h16_m42': {1: 2000, 3: 639, 4: 2321, 5: 1373, 6: 1500},
    'ik_r10_h16_m39': {1: 2000, 3: 699, 4: 2400, 5: 1321, 6: 1500},
}
# Hold jitter of the fixture (PREREGISTERED): a real hold is not exactly on the
# catalogue convention. Applied per tick, along the tool heading, in height and
# in yaw.
HOLD_JITTER = {'forward_m': .004, 'z_m': .003, 'yaw_deg': 4.}
STRAIGHT3 = ev2.STRAIGHT3
TICK_JITTER = ev2.TICK_JITTER

# (case, posture, held kind or None, expected kind). ``held_box`` means a cyan box.
HELD_CASES = (
    ('held_can', 'held_check', 'can', 'can'),
    ('held_can_absent', 'held_check', None, 'can'),
    ('held_can_wrong_kind', 'held_check', 'tile', 'can'),
    ('held_box_carry', 'carry', 'cyan', 'cyan'),
    ('held_tile_carry', 'carry', 'tile', 'tile'),
    ('held_nothing_carry', 'carry', None, 'cyan'),
    ('held_tile_expect_box_carry', 'carry', 'tile', 'cyan'),
    ('held_can_carry', 'carry', 'can', 'can'),           # W7 condition: can not framed by CARRY
    ('held_box_held_check', 'held_check', 'cyan', 'cyan'),  # control: box not framed by held-check
)
CARRY_CASES = (
    ('carry_long_beam', 'long_beam', 'long_beam'),
    ('carry_heavy_crate', 'heavy_crate', 'heavy_crate'),
    ('carry_tri_frame', 'tri_frame', 'tri_frame'),
    ('carry_crate_expect_beam', 'heavy_crate', 'long_beam'),
    ('carry_beam_expect_crate', 'long_beam', 'heavy_crate'),
    ('carry_nothing_expect_crate', None, 'heavy_crate'),
)
# The v2 centroid hold (``convention='centroid'``) is not in the cohort: it puts
# the lens inside the crate body (half length 0.07 m, lens 0.033 m behind the grip
# site), which is physically impossible. Dev scan: that is v2's "black crate".
HOLD_CONVENTION: dict[str, str] = {}
# (case, v2 plan case, expected kind override)
GRASP_CASES = (
    ('grasp_long_beam', 'grasp_long_beam', None),
    ('grasp_heavy_crate', 'grasp_heavy_crate', None),
    ('grasp_tri_frame', 'grasp_tri_frame', None),
    ('grasp_wrong_kind', 'grasp_tri_frame', 'long_beam'),
    ('wrong_end_long_beam', 'wrong_end_long_beam', None),
)
# Which kinds a posture frames at the grip (PREREGISTERED, from the v1/v2
# measurements and the v3 silhouette check): absence of the ordered kind is only
# decidable when the posture would show it.
FRAMED = {'carry': ('cyan', 'green', 'red', 'yellow', 'tile', 'long_beam', 'heavy_crate', 'tri_frame'),
          'held_check': ('can',)}
MIN_HELD_VISIBLE_PX = 500
MIN_TEAM_VISIBLE_PX = 400
CONFIDENT = ev2.CONFIDENT


def _git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def load_split():
    return json.loads(SPLIT_FILE.read_text())


# ------------------------------------------------------------------ hold fixture

def _hold_item(world, items, item_id, kind, rid, rng, convention='grasp_role'):
    """Pose ``item_id`` at the robot's grip by the catalogue hold convention (fixture).

    ``convention='centroid'`` reproduces the v2 fixture (item centroid at the grip,
    item yaw = robot yaw): physically implausible for a team item, kept only as the
    recorded condition in which v2 saw a black, colourless carried crate.
    """
    import mujoco
    from harness.zone_own_perception_v3 import held_parts
    robot = world.robot(rid)
    grip = np.asarray(robot.site_xyz('grip_site'), float)
    yaw = float(robot.base_rpy()[2])
    parts, grip_obj, approach = held_parts(kind)
    if convention == 'centroid':
        grip_obj, approach = (0., 0., grip_obj[2]), 0.
    fwd = rng.uniform(-HOLD_JITTER['forward_m'], HOLD_JITTER['forward_m'])
    dz = rng.uniform(-HOLD_JITTER['z_m'], HOLD_JITTER['z_m'])
    obj_yaw = yaw-approach+math.radians(rng.uniform(-HOLD_JITTER['yaw_deg'], HOLD_JITTER['yaw_deg']))
    c, s = math.cos(obj_yaw), math.sin(obj_yaw)
    ox = grip[0]-(c*grip_obj[0]-s*grip_obj[1])+fwd*math.cos(yaw)
    oy = grip[1]-(s*grip_obj[0]+c*grip_obj[1])+fwd*math.sin(yaw)
    oz = grip[2]-grip_obj[2]+dz
    body_z = oz+(.016 if items[item_id]['family'] == 'box' else 0.)
    ev2._set_item(world, items, item_id, (ox, oy), body_z, obj_yaw)
    mujoco.mj_forward(world.model, world.data)
    return {'fwd_m': round(fwd, 4), 'dz_m': round(dz, 4), 'yaw_offset_deg': round(math.degrees(obj_yaw-(yaw-approach)), 2)}


def _item_for_kind(items, kind):
    if kind in ('can', 'tile'):
        return ev2.HELD_ITEMS[kind]
    if kind in ev2.TEAM_ITEMS:
        return ev2.TEAM_ITEMS[kind]
    return next((oid for oid, spec in items.items() if spec['family'] == 'box' and spec['kind'] == kind), None)


# ------------------------------------------------------------------ plans

def plan_held(case_row, rng, info, items, robot_ids):
    case, posture, held_kind, expected = case_row
    actor = robot_ids[0]
    held = None if held_kind is None else _item_for_kind(items, held_kind)
    if held_kind is not None and held is None:
        return None
    xy = ev2._free_floor_xy(info, rng)
    plan = {'case': case, 'family': 'held', 'actor': actor, 'items': {}, 'robots': {actor: (xy[0], xy[1],
            rng.uniform(-math.pi, math.pi))}, 'arm': {actor: dict(CARRY if posture == 'carry' else HELD_CHECK)},
            'held': held, 'held_kind': held_kind, 'posture': posture, 'block_xy': None,
            'question': {'expected_kind': expected, 'posture_name': posture},
            'judgments': ['held_item_at_grip'], 'team_item': None, 'team_stage': None}
    return ev2._validate(plan, info['static_map'], robot_ids)


def plan_carry(case_row, rng, info, items, robot_ids):
    case, held_kind, expected = case_row
    actor = robot_ids[0]
    xy = ev2._free_floor_xy(info, rng)
    held = None if held_kind is None else ev2.TEAM_ITEMS[held_kind]
    plan = {'case': case, 'family': 'carry', 'actor': actor, 'items': {},
            'robots': {actor: (xy[0], xy[1], rng.uniform(-math.pi, math.pi))}, 'arm': {actor: dict(CARRY)},
            'held': held, 'held_kind': held_kind, 'posture': 'carry', 'block_xy': None,
            'question': {'expected_kind': expected, 'stage': 'carry'},
            'judgments': ['team_cargo_at_grip'], 'team_item': held, 'team_stage': 'carry'}
    return ev2._validate(plan, info['static_map'], robot_ids)


def plan_grasp(case_row, rng, info, items, robot_ids, instances):
    case, v2_case, expected = case_row
    plan = ev2.plan_view(v2_case, rng, info, items, robot_ids, instances)
    if plan is None:
        return None
    plan['case'] = case
    plan['family'] = 'grasp'
    plan['judgments'] = ['team_cargo_identity', 'team_cargo_handle']
    plan['held_kind'] = None
    if expected is not None:
        plan['question']['expected_kind'] = expected
    return plan


# ------------------------------------------------------------------ labels

def _seg_visible(seg, gids, usable):
    return int((np.isin(seg, gids) & usable).sum())


def _truth_held(plan, items, item_gids, seg, usable):
    expected = plan['question']['expected_kind']
    held = plan['held']
    kind = None if held is None else items[held]['kind']
    seen = 0 if held is None else _seg_visible(seg, item_gids[held], usable)
    framed = expected in FRAMED[plan['posture']]
    if kind == expected:
        observable = seen >= MIN_HELD_VISIBLE_PX
    else:
        observable = framed
    return {'truth': 'yes' if kind == expected else 'no',
            'truth_observed': kind or ('expected_kind_absent' if held is None else kind),
            'observable': bool(observable),
            'detail': {'held_item': held, 'held_kind': kind, 'visible_px': seen, 'posture': plan['posture'],
                       'expected_framed_by_posture': framed}}


def _truth_carry(plan, items, item_gids, seg, usable, frame_bgr):
    expected = plan['question']['expected_kind']
    held = plan['held']
    kind = None if held is None else items[held]['kind']
    seen = 0 if held is None else _seg_visible(seg, item_gids[held], usable)
    sat90 = 0. if held is None else ev2._item_colour_p90(frame_bgr, seg, item_gids[held])
    observable = True if held is None else seen >= MIN_TEAM_VISIBLE_PX
    return {'truth': 'yes' if kind == expected else 'no', 'truth_observed': kind or 'empty',
            'observable': bool(observable),
            # v2's colour-based observability, recorded for the side-by-side
            'observable_colour_v2': bool(held is not None and seen >= MIN_TEAM_VISIBLE_PX
                                         and sat90 >= ev2.MIN_ITEM_SATURATION_P90),
            'detail': {'held_item': held, 'held_kind': kind, 'visible_px': seen,
                       'saturation_p90': round(sat90, 1)}}


# ------------------------------------------------------------------ render

def _render_ticks(world, plan, items, instances, item_gids, handle_gids, robot_gids, rng_jitter, rng_hold,
                  arm, vdir, labels_dir, view_id, usable, pans):
    import cv2
    import mujoco
    actor = plan['actor']
    xyz, rpy = world.robot(actor).base_xyz(), world.robot(actor).base_rpy()
    base_pose = (float(xyz[0]), float(xyz[1]), float(rpy[2]))
    ticks_in, ticks_lab = [], []
    for tick, pan in enumerate(pans):
        if tick:
            jitter = (base_pose[0]+rng_jitter.uniform(-TICK_JITTER['xy_m'], TICK_JITTER['xy_m']),
                      base_pose[1]+rng_jitter.uniform(-TICK_JITTER['xy_m'], TICK_JITTER['xy_m']),
                      base_pose[2]+rng_jitter.uniform(-TICK_JITTER['yaw_rad'], TICK_JITTER['yaw_rad']))
            ev2._set_base(world, actor, jitter)
            mujoco.mj_forward(world.model, world.data)
        tick_arm = {**arm, 6: pan}
        world._team_joint_move_servos({actor: tick_arm}, .25, settle_s=.25)
        xyz, rpy = world.robot(actor).base_xyz(), world.robot(actor).base_rpy()
        tick_pose = (float(xyz[0]), float(xyz[1]), float(rpy[2]))
        hold = None
        if plan['held'] is not None:
            hold = _hold_item(world, items, plan['held'], items[plan['held']]['kind'], actor, rng_hold,
                              HOLD_CONVENTION.get(plan['case'], 'grasp_role'))
        name = f'{actor}--tick{tick}--pan{pan}'
        (vdir/f'{name}.jpg').write_bytes(world.render_jpeg(robot_id=actor, camera='robot_cam', quality=90))
        seg = _segment(world, None, actor)
        cv2.imwrite(str(labels_dir/f'{view_id}--{name}.png'), (seg+1).astype(np.uint16))
        bgr = cv2.imdecode(np.frombuffer((vdir/f'{name}.jpg').read_bytes(), np.uint8), 1)
        ticks_in.append({'tick': tick, 'frame': f'{name}.jpg', 'sim_s': float(tick),
                         'commanded_arm_pwm': {str(k): int(v) for k, v in tick_arm.items()}})
        row = {'tick': tick, 'frame': f'{name}.jpg', 'pan_pwm': pan, 'base_pose': list(tick_pose),
               'hold_jitter': hold, 'truth': {}, 'truth_observed': {}, 'observable': {}}
        if plan['family'] == 'held':
            t = _truth_held(plan, items, item_gids, seg, usable)
            row['truth']['held_item_at_grip'] = t['truth']
            row['truth_observed']['held_item_at_grip'] = t['truth_observed']
            row['observable']['held_item_at_grip'] = t['observable']
            row['held_detail'] = t['detail']
        elif plan['family'] == 'carry':
            t = _truth_carry(plan, items, item_gids, seg, usable, bgr)
            row['truth']['team_cargo_at_grip'] = t['truth']
            row['truth_observed']['team_cargo_at_grip'] = t['truth_observed']
            row['observable']['team_cargo_at_grip'] = t['observable']
            row['observable_colour_v2'] = t['observable_colour_v2']
            row['carry_detail'] = t['detail']
        else:
            v2row = ev2._truth(plan, world, items, instances, item_gids, handle_gids, robot_gids, seg,
                               tick_pose, bgr)
            for key in ('truth', 'truth_observed', 'observable'):
                row[key] = v2row[key]
            row['team_cargo_detail'] = v2row.get('team_cargo_detail')
            row['handle_detail'] = v2row.get('handle_detail')
        ticks_lab.append(row)
    return ticks_in, ticks_lab


def render(args):
    import mujoco
    from harness import zone_own_perception as v1
    split = load_split()
    specs = split['splits'][args.split][:args.max_scenes] if args.max_scenes else split['splits'][args.split]
    out = Path(args.output)
    (out/'frames').mkdir(parents=True, exist_ok=False)
    (out/'eval-labels').mkdir()
    families = set(args.families or ('held', 'carry', 'grasp'))
    grasp_postures = args.grasp_posture or ['approach_look_v2', 'grasp_look_v3']
    posture_table = dict(GRASP_CANDIDATES)
    from harness.zone_own_perception_v3 import GRASP_LOOK_POSTURE
    posture_table['grasp_look_v3'] = dict(GRASP_LOOK_POSTURE)
    load = {'start': os.getloadavg()[0]}
    manifest = {'schema': SCHEMA, 'split': args.split, 'split_file_sha256': sha256(SPLIT_FILE),
                'source_sha': _git('rev-parse', 'HEAD'), 'source_dirty': bool(_git('status', '--porcelain')),
                'contact_profile': CONTACT_PROFILE, 'weld': 'off',
                'clock': 'synchronous SIM; arm settled, then one render per tick',
                'hold_views': ('item posed at the grip by the catalogue hold convention for one frame, '
                               'physics not stepped'),
                'hold_jitter': HOLD_JITTER, 'families': sorted(families),
                'grasp_postures': {name: posture_table[name] for name in grasp_postures},
                'max_scenes': args.max_scenes,
                'views': [], 'scenes': []}
    usable = v1._usable_region(640, 480)
    started = time.monotonic()
    for spec in specs:
        variant, seed = spec['variant'], spec['seed']
        rng = random.Random(f'{variant}:{seed}:own-perception-v3')
        world, items, info = ev2.build_world(variant, seed, rng)
        instances = info['cargo_instances']
        try:
            table, item_gids, robot_gids = ev2._geom_categories(world, items)
            handle_gids = ev2._handle_geoms(world, items, instances)
            robot_ids = list(world.robot_ids)
            manifest['scenes'].append({'variant': variant, 'seed': seed,
                                       'scene_xml_sha256': info['scene_xml_sha256'],
                                       'scene_extras': info['scene_extras'],
                                       'items': {k: v['kind'] for k, v in items.items()},
                                       'order_sheet_sha256': hashlib.sha256(
                                           json.dumps(info['order_sheet'], sort_keys=True).encode()).hexdigest()})
            (out/'frames'/f'order-sheet-{variant}-s{seed}.json').write_text(json.dumps(info['order_sheet'], indent=1))
            jobs = []
            if 'held' in families:
                jobs += [('held', row) for row in HELD_CASES]
            if 'carry' in families:
                jobs += [('carry', row) for row in CARRY_CASES]
            if 'grasp' in families:
                jobs += [('grasp', row) for row in GRASP_CASES]
            for family, row in jobs:
                plan = None
                for _ in range(40):
                    if family == 'held':
                        plan = plan_held(row, rng, info, items, robot_ids)
                    elif family == 'carry':
                        plan = plan_carry(row, rng, info, items, robot_ids)
                    else:
                        plan = plan_grasp(row, rng, info, items, robot_ids, instances)
                    if plan is not None:
                        break
                case = row[0]
                if plan is None:
                    manifest['views'].append({'view_id': f'{variant}-s{seed}-{case}', 'skipped': 'no placement'})
                    continue
                postures = ([(name, posture_table[name]) for name in grasp_postures] if family == 'grasp'
                            else [(plan['posture'], plan['arm'][plan['actor']])])
                view_seed = rng.random()
                for posture_name, arm in postures:
                    view_id = (f'{variant}-s{seed}-{case}' if family != 'grasp'
                               else f'{variant}-s{seed}-{case}@{posture_name}')
                    ev2._park_all(world, items, info, rng)
                    for item_id, (xy, yaw) in plan['items'].items():
                        ev2._set_item(world, items, item_id, xy, ev2._floor_z(items, item_id), yaw)
                    for rid, pose in plan['robots'].items():
                        ev2._set_base(world, rid, pose)
                    mujoco.mj_forward(world.model, world.data)
                    arms = {**plan['arm'], plan['actor']: dict(arm)}
                    world._team_joint_move_servos(arms, .3, settle_s=.3)
                    vdir = out/'frames'/view_id
                    vdir.mkdir()
                    # identical tick jitter for every posture of the same grasp view
                    rng_jitter = random.Random(f'{view_seed}:jitter')
                    rng_hold = random.Random(f'{view_seed}:hold')
                    ticks_in, ticks_lab = _render_ticks(world, plan, items, instances, item_gids, handle_gids,
                                                        robot_gids, rng_jitter, rng_hold, arm, vdir,
                                                        out/'eval-labels', view_id, usable, STRAIGHT3)
                    question = {**plan['question'], 'posture_name': posture_name}
                    actor_inputs = {'view_id': view_id, 'variant': variant, 'seed': seed, 'case': case,
                                    'family': family, 'judgments': list(plan['judgments']),
                                    'actor': plan['actor'], 'question': question, 'ticks': ticks_in,
                                    'order_sheet_file': f'order-sheet-{variant}-s{seed}.json',
                                    'map_id': info['static_map']['map_id'],
                                    'map_version': info['static_map']['version']}
                    (vdir/'actor-inputs.json').write_text(json.dumps(actor_inputs, indent=1))
                    labels = {'view_id': view_id, 'variant': variant, 'seed': seed, 'case': case,
                              'family': family, 'posture': posture_name,
                              'geom_table': {str(k): v for k, v in table.items()}, 'ticks': ticks_lab}
                    (out/'eval-labels'/f'{view_id}.json').write_text(json.dumps(labels))
                    manifest['views'].append({'view_id': view_id, 'case': case, 'family': family,
                                              'posture': posture_name, 'judgments': list(plan['judgments']),
                                              'variant': variant, 'seed': seed, 'ticks': len(STRAIGHT3)})
        finally:
            world.close()
    load['end'] = os.getloadavg()[0]
    manifest['load_average_1min'] = load
    manifest['wall_s'] = round(time.monotonic()-started, 1)
    (out/'manifest.json').write_text(json.dumps(manifest, indent=1))
    print(json.dumps({'views': len([v for v in manifest['views'] if 'skipped' not in v]),
                      'skipped': len([v for v in manifest['views'] if 'skipped' in v]),
                      'load_average_1min': load, 'wall_s': manifest['wall_s']}))


# ------------------------------------------------------------------ score

def _answers_for_tick(family, posture, frame, pose, question, v2, v3):
    """(version, judgment, answer) rows for one tick: v3 and the v2 side-by-side."""
    expected = question['expected_kind']
    rows = []
    if family == 'held':
        rows.append(('v3', 'held_item_at_grip', v3.judge_held_item(frame, pose, expected_kind=expected)))
        v2ans = v2.judge_held_item(frame, pose, expected_kind=expected, posture_name=question['posture_name'])
        rows.append(('v2', 'held_item_at_grip', {**v2ans, 'judgment': 'held_item'}))
    elif family == 'carry':
        rows.append(('v3', 'team_cargo_at_grip', v3.judge_team_cargo_at_grip(frame, pose, expected_kind=expected)))
        v2ans = v2.judge_team_cargo_identity(frame, pose, expected_kind=expected)
        rows.append(('v2', 'team_cargo_at_grip', v2ans))
    else:
        version = 'v2' if posture == 'approach_look_v2' else 'v3'
        if version == 'v3' and posture == 'grasp_look_v3':
            both = v3.judge_team_cargo_grasp_stage(frame, pose, expected_kind=expected)
            identity, handle = both['identity'], both['handle']
            identity = {**identity, 'judgment': 'team_cargo_identity'}
            handle = {**handle, 'judgment': 'team_cargo_handle'}
        else:
            identity = v2.judge_team_cargo_identity(frame, pose, expected_kind=expected)
            handle = v2.judge_team_cargo_handle(frame, pose, expected_kind=expected)
        rows.append((version, 'team_cargo_identity', identity))
        rows.append((version, 'team_cargo_handle', handle))
    return rows


def score_views(frames_dir):
    from harness import zone_own_outcome_v3 as outcome
    from harness import zone_own_perception_v2 as v2
    from harness import zone_own_perception_v3 as v3
    frames_dir = Path(frames_dir)
    manifest = json.loads((frames_dir/'manifest.json').read_text())
    records = []
    for view in manifest['views']:
        if 'skipped' in view:
            continue
        vid = view['view_id']
        labels = json.loads((frames_dir/'eval-labels'/f'{vid}.json').read_text())
        inputs = json.loads((frames_dir/'frames'/vid/'actor-inputs.json').read_text())
        family, posture, question = inputs['family'], labels['posture'], inputs['question']
        series = {}
        for tick_in, tick_lab in zip(inputs['ticks'], labels['ticks']):
            frame = (frames_dir/'frames'/vid/tick_in['frame']).read_bytes()
            pose = {int(k): int(v) for k, v in tick_in['commanded_arm_pwm'].items()}
            for version, judgment, answer in _answers_for_tick(family, posture, frame, pose, question, v2, v3):
                key = (version, judgment)
                series.setdefault(key, []).append(answer)
                records.append({'record': 'tick', 'view_id': vid, 'case': labels['case'], 'family': family,
                                'posture': posture, 'version': version, 'judgment': judgment,
                                'variant': inputs['variant'], 'seed': inputs['seed'], 'tick': tick_in['tick'],
                                'answer': answer['answer'], 'confidence': answer['confidence'],
                                'reason': answer['reason'], 'observed': answer.get('observed'),
                                'cue': answer.get('cue'),
                                'truth': tick_lab['truth'][judgment],
                                'truth_observed': tick_lab['truth_observed'][judgment],
                                'observable': tick_lab['observable'][judgment],
                                'observable_colour_v2': tick_lab.get('observable_colour_v2'),
                                'diag': _diag(answer)})
        final = labels['ticks'][-1]
        for (version, judgment), observations in series.items():
            name = observations[0]['judgment']
            decision = outcome.track(name, [{**o, 'judgment': name} for o in observations], question=question)
            records.append({'record': 'view', 'view_id': vid, 'case': labels['case'], 'family': family,
                            'posture': posture, 'version': version, 'judgment': judgment,
                            'variant': inputs['variant'], 'seed': inputs['seed'], 'ticks': len(observations),
                            'status': decision['status'], 'answer': decision['answer'],
                            'confidence': decision['confidence'], 'reason': decision['reason'],
                            'observed': decision.get('observed'), 'unknown_ticks': decision['unknown_ticks'],
                            'truth': final['truth'][judgment], 'truth_observed': final['truth_observed'][judgment],
                            'observable': final['observable'][judgment],
                            'observable_colour_v2': final.get('observable_colour_v2')})
    return records, manifest


def _diag(answer):
    keep = {}
    for key in ('per_kind', 'colour_in_expected_silhouette', 'shape_margin', 'expected_silhouette_lit_share',
                'coverage', 'margin', 'best_kind', 'silhouette_value_p75'):
        if key in answer:
            keep[key] = answer[key]
    return keep


def summarize(records):
    out = {}
    for scope in ('tick', 'view'):
        rows = [r for r in records if r['record'] == scope]
        per = {}
        for key in sorted({(r['version'], r['judgment'], r['family'], r['posture']) for r in rows}):
            sel = [r for r in rows if (r['version'], r['judgment'], r['family'], r['posture']) == key]
            name = ':'.join(key)
            per[name] = {**ev2._metrics(sel), 'observable': ev2._metrics([r for r in sel if r['observable']]),
                         'not_observable': ev2._metrics([r for r in sel if not r['observable']]),
                         'by_case': {c: ev2._metrics([r for r in sel if r['case'] == c])
                                     for c in sorted({r['case'] for r in sel})}}
        out[scope] = per
    return out


def score(args):
    frames_dir = Path(args.frames)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    load = {'start': os.getloadavg()[0]}
    records, manifest = score_views(frames_dir)
    (out/'records.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    load['end'] = os.getloadavg()[0]
    summary = {'schema': SCHEMA+'.score', 'frames_dir': str(frames_dir), 'split': manifest['split'],
               'render_source_sha': manifest['source_sha'], 'render_source_dirty': manifest['source_dirty'],
               'score_source_sha': _git('rev-parse', 'HEAD'),
               'score_source_dirty': bool(_git('status', '--porcelain')),
               'confident_threshold': CONFIDENT, 'load_average_1min': load, 'results': summarize(records)}
    (out/'summary.json').write_text(json.dumps(summary, indent=1))
    print(json.dumps({'written': str(out/'summary.json'), 'load_average_1min': load}))


def parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='command', required=True)
    r = sub.add_parser('render', help='render wrist frames + eval labels for one split')
    r.add_argument('--split', choices=('dev', 'test'), required=True)
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--families', action='append', choices=('held', 'carry', 'grasp'))
    r.add_argument('--max-scenes', type=int, default=0, help='smoke runs only: render the first N scenes')
    r.add_argument('--grasp-posture', action='append',
                   choices=sorted(GRASP_CANDIDATES) + ['grasp_look_v3'],
                   help='grasp-stage postures to render (default: approach_look_v2 and grasp_look_v3)')
    s = sub.add_parser('score', help='run v3 (and v2 side-by-side) judgments and score them')
    s.add_argument('--frames', type=Path, required=True)
    s.add_argument('--output', type=Path, required=True)
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
