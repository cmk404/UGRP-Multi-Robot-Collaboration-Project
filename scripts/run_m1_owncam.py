"""M1 run: one robot delivers the cyan box from the pickup area through the door to a zone slot on its own camera.

The runner is the physics owner. The controller (``harness.m1_owncam_delivery``)
receives only the robot's own ``robot_cam`` observations and its own issued
commands (plus the static tagged map, fixed calibrations and the order sheet:
colour, pickup area rows, destination slot). No pose stub of any kind: the
M1 contract rejects every non own-camera pose source in the controller, the
judge and the export (Codex review #1). Simulator truth is written to
``eval_only/`` for scoring only. Sync SIM only; weld OFF.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCHEMA = 'ugrp.m1_owncam_run.v1'
FRAME_S = .2
TICK_S = .1
GT_S = .05
SIM_LIMIT_S = 720.
SLOT_HALF_M = .06
ON_FLOOR_MAX_Z_M = .05
SKILLS = {'v4': ('harness.wrist_zone_skill_v4', 'WristZoneDeliveryV4')}
RUNTIME_FILES = ('harness/m1_owncam_delivery.py', 'harness/m1_contract.py', 'harness/owncam_pose_source.py',
                 'harness/owncam_localizer.py', 'harness/owncam_drive.py', 'harness/owncam_drive_v2.py',
                 'harness/wall_tags.py', 'harness/map_goto.py', 'harness/zone_color_boxes.py',
                 'harness/wrist_zone_skill.py', 'harness/wrist_zone_skill_v2.py', 'harness/wrist_zone_skill_v3.py',
                 'harness/wrist_zone_skill_v4.py', 'harness/visual_box_skill.py', 'scripts/run_m1_owncam.py')


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in rows))


def run(spec, out, student):
    import importlib

    import cv2
    import mujoco
    import numpy as np
    from harness import m1_contract
    from harness.m1_owncam_delivery import M1OwnCamDelivery
    from harness.map_goto import plan_path
    from harness.owncam_drive import LOADED_ENVELOPE
    from harness.map_goto import UNLOADED_ENVELOPE
    from harness.wrist_zone_skill import PoseEstimate
    from scripts.record_owncam_localization import LoggingPort
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.research_dispatch_arena import digest
    from sim.zone_arena import LAYOUTS
    from sim.zone_landmarks import TaggedZoneScene

    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    started, load_start = time.time(), os.getloadavg()
    code = {'sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain', '--', 'sim', 'harness',
                                                                 'scripts', 'maps', 'experiments')),
            'stamped': 'at launch', 'runtime_files_sha256': {f: sha_bytes((ROOT/f).read_bytes()) for f in RUNTIME_FILES}}
    scene = TaggedZoneScene.from_tagged(spec['map'], spec['seed'], spec['goal'], spec.get('extra_boxes'),
                                        contact_profile=spec.get('contact_profile', 'local_contact_fine'))
    world = MultiMasterPiProductionV2(seed=spec['seed'], width=640, height=480, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    static = scene.config['static_map']
    objects = scene.config['setup_only']['objects']
    spawns = scene.config['setup_only']['spawns']
    rid = min(spawns, key=lambda r: abs(spawns[r][1] - spec['spawn_y']))
    slot_xy = next(s['center_m'] for slots in static['zone_slots'].values() for s in slots if s['slot_id'] == spec['slot_id'])
    calibration_path = ROOT/student['calibration']
    calibration = json.loads(calibration_path.read_text())
    module, name = SKILLS[student['skill']]
    skill_cls = getattr(importlib.import_module(module), name)
    rows_y = LAYOUTS['zone_wide']['pickup_rows_y']

    ctl_ref = {}

    def planner(start, goal, carrying):
        ctl = ctl_ref['ctl']
        result = plan_path(static, start, goal, LOADED_ENVELOPE if carrying else UNLOADED_ENVELOPE,
                           obstacles=ctl._keepouts(), escape_start_m=.25)
        return None if result is None else [tuple(p) for p in result['waypoints_m'][1:]]

    ctl = M1OwnCamDelivery(static, calibration['params'], box_kind='cyan', slot_id=spec['slot_id'], slot_xy=slot_xy,
                           skill_factory=lambda order: skill_cls(order, planner=planner),
                           pose_estimate_cls=PoseEstimate, search_rows_y=rows_y, robot_id=rid, seed=spec['seed'])
    ctl_ref['ctl'] = ctl
    commands = []

    def sink(row):
        commands.append(row)
        ctl.on_command(row)
    initial = {int(k): int(v) for k, v in world.robot(rid).servo_command_pulses.items()}
    sink({'t': 0.0, 'kind': 'initial_servo_command', 'pulses': dict(initial)})
    raw = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in ('r1', 'r2', 'r3')}
    port = LoggingPort(raw[rid], sink)
    box = next(oid for oid, o in sorted(objects.items()) if o['kind'] == 'cyan')
    box_body = objects[box]['body_name']
    model, data = world.model, world.data
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(model.ngeom)]
    wall_geoms = {g for g, n in enumerate(names) if n.startswith('zone_wall_')}
    own_geoms = {g for g, n in enumerate(names) if n.startswith(rid + '__')}
    peer_geoms = {g for g, n in enumerate(names) if n.startswith(('r1__', 'r2__', 'r3__')) and g not in own_geoms}
    box_geom = {g for g, n in enumerate(names) if n == box_body + '_geom'}
    other_boxes = {g for g, n in enumerate(names) if n.startswith('cargo_box_') and g not in box_geom}
    frames_dir = out/'frames'
    frames_dir.mkdir()
    frames, frame_eval, gt, contacts, decisions = [], [], [], [], []
    state = {'next_frame': 0., 'next_gt': 0., 'max_eq_active': 0, 'phase_times': {}}

    def truth():
        r = world.robot(rid)
        xyz, rpy = r.base_xyz(), r.base_rpy()
        return float(xyz[0]), float(xyz[1]), float(rpy[2])

    def capture(now):
        obs = port.capture()
        jpeg = base64.b64decode(obs['image'])
        rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        index = len(frames)
        (frames_dir/f'{index:05d}.jpg').write_bytes(jpeg)
        report = ctl.on_frame(now, obs, rgb)
        frames.append({'frame': index, 't': round(now, 4), 'file': f'frames/{index:05d}.jpg', 'frame_id': obs['frame_id'],
                       'sha256': obs['sha256'], 'camera': obs['camera'], 'phase': ctl.phase,
                       'skill_phase': getattr(ctl.skill, 'phase', None),
                       'commanded_servo': obs['actuator_state']['servo_pulses'], 'report': report.as_dict()})
        x, y, yaw = truth()
        row = {'frame': index, 't': round(now, 4), 'gt': [round(x, 5), round(y, 5), round(yaw, 6)],
               'box_xyz': [round(float(v), 4) for v in data.body(box_body).xpos], 'phase': ctl.phase,
               'skill_phase': getattr(ctl.skill, 'phase', None)}
        if report.initialized:
            row.update(pos_err_m=round(math.hypot(report.x_m - x, report.y_m - y), 5),
                       yaw_err_deg=round(abs(math.degrees((report.yaw_rad - yaw + math.pi) % (2*math.pi) - math.pi)), 4),
                       std_xy_m=round(report.std_xy_m, 5))
        frame_eval.append(row)
        state['next_frame'] = now + FRAME_S
        return obs

    def physics(seconds):
        n = max(1, round(seconds/float(model.opt.timestep)))
        for _ in range(n):
            now = float(data.time)
            for p in raw.values():
                p.tick(now)
            world._physics_step_for(world.controllers[rid])
            now = float(data.time)
            for i in range(data.ncon):
                c = data.contact[i]
                pair = {int(c.geom1), int(c.geom2)}
                mine = pair & (own_geoms | box_geom)
                if not mine:
                    continue
                other = next(iter(pair - mine), None)
                kind = ('wall' if other in wall_geoms else 'peer_robot' if other in peer_geoms and mine & own_geoms else
                        'other_box' if other in other_boxes else None)
                if kind and (not contacts or contacts[-1]['t'] < now - .1 or contacts[-1]['kind'] != kind):
                    contacts.append({'t': round(now, 4), 'kind': kind, 'dist_m': round(float(c.dist), 5),
                                     'geoms': sorted(names[g] for g in pair), 'phase': ctl.phase})
            if len(data.eq_active):
                state['max_eq_active'] = max(state['max_eq_active'], int(data.eq_active.max()))
            if now + 1e-9 >= state['next_gt']:
                x, y, yaw = truth()
                gt.append({'t': round(now, 4), 'x': round(x, 5), 'y': round(y, 5), 'yaw': round(yaw, 6),
                           'box_xyz': [round(float(v), 4) for v in data.body(box_body).xpos]})
                state['next_gt'] = now + GT_S
            if now + 1e-9 >= state['next_frame']:
                capture(now)

    def apply(action):
        port.apply(action, float(data.time))

    def execute_macro(action, obs):
        """Skill macros (as scripts/run_zone_owncam_skill.py executes them), frames at 5 Hz throughout."""
        kind = action['kind']
        if kind == 'drive':
            apply({'kind': 'drive', 'forward': action['fwd'], 'turn': action['turn'], 'duration_s': action['duration']})
            physics(action['duration'] + .2)
            port.hold(float(data.time))
        elif kind == 'mecanum':
            apply({'kind': 'mecanum', 'forward': action['forward'], 'left': action['left'], 'turn': action['turn'],
                   'duration_s': action['duration']})
            physics(action['duration'] + .1)
            port.hold(float(data.time))
        elif kind == 'pose':
            targets = action['pulses']
            start = obs['actuator_state']['servo_pulses']
            delta = max(abs(p - start[str(s)]) for s, p in targets.items())
            duration = max(.25, delta/600.)
            count = max(5, math.ceil(duration/.05))
            for sample in range(1, count + 1):
                u = sample/count
                ease = u*u*(3 - 2*u)
                for servo, end in targets.items():
                    pulse = round(start[str(servo)] + ease*(end - start[str(servo)]))
                    apply({'kind': 'look', 'pan_pulse': pulse} if int(servo) == 6 else
                          {'kind': 'arm', 'servo_id': int(servo), 'pulse': pulse})
                physics(duration/count)
            physics(.3 if ctl.skill is not None and ctl.skill.phase == 'grasp' and ctl.skill.box.phase == 'approach' else .15)
        elif kind == 'wait':
            apply({'kind': 'wait'})
            physics(max(.05, action['duration']))
        else:
            raise ValueError('UNKNOWN_MACRO')

    physics(.5)
    outcome = None
    while True:
        now = float(data.time)
        if now > SIM_LIMIT_S:
            outcome = 'SIM_LIMIT'
            break
        state['phase_times'].setdefault(f'{ctl.phase}:{getattr(ctl.skill, "phase", "")}', round(now, 2))
        decision = ctl.decide(now)
        mode = decision['mode']
        if mode == 'done':
            outcome = decision['outcome']
            break
        if mode == 'capture':
            capture(now)
            continue
        if mode == 'tick':
            for cmd in decision['commands']:
                if cmd['kind'] == 'hold':
                    port.hold(now)
                else:
                    apply(cmd)
            physics(TICK_S)
        elif mode == 'macro':
            decisions.append({'t': round(now, 3), 'skill_phase': ctl.skill.phase, 'action': decision['action']})
            execute_macro(decision['action'], ctl.last_obs)
            capture(float(data.time))
    port.hold(float(data.time))
    physics(.5)                       # settle before the final truth sample
    summary = ctl.summary()
    bx, by, bz = (float(v) for v in data.body(box_body).xpos)
    gt_in_slot = abs(bx - slot_xy[0]) <= SLOT_HALF_M and abs(by - slot_xy[1]) <= SLOT_HALF_M and bz < ON_FLOOR_MAX_Z_M
    placement = summary['skill_placement'] or {}
    claim = placement.get('reason') == 'IN_SLOT'
    gate = summary['lookback_gate'] or {}
    wall_contacts = sum(1 for c in contacts if c['kind'] == 'wall')
    judged = m1_contract.judge(
        pose_sources=summary['pose_sources'], skill_reason=outcome, skill_claim_in_slot=claim,
        gt_box_in_slot=gt_in_slot, wall_contacts=wall_contacts, weld_used=state['max_eq_active'] > 0,
        face_fallback_used=summary['face_fallback_used'], pickup_source=summary['pickup_source'] or 'none',
        within_limit=outcome != 'SIM_LIMIT',
        extra_checks={'look_back_pose_gate_ok': bool(gate) and not gate.get('violations')})
    target = summary['target_xy']
    box0 = objects[box]['position_m']
    result = {'schema': SCHEMA, 'episode': spec['episode_id'], 'split': spec['split'], 'robot_id': rid,
              'outcome': outcome, **judged,
              'input_contract': 'own robot_cam JPEG + own issued commands + static tagged map v2 + fixed calibrations + '
                                'order sheet (cyan, pickup area rows, destination slot); no pose stub; GT only in eval_only/',
              'evaluation_only': {'gt_box_final_xyz': [round(bx, 4), round(by, 4), round(bz, 4)],
                                  'slot_xy': slot_xy, 'gt_box_in_slot': gt_in_slot,
                                  'search_target_error_m': None if target is None else
                                  round(math.hypot(target[0] - box0[0], target[1] - box0[1]), 4),
                                  'wall_contacts': wall_contacts,
                                  'contacts': {k: sum(1 for c in contacts if c['kind'] == k)
                                               for k in ('wall', 'peer_robot', 'other_box')},
                                  'max_eq_active': state['max_eq_active']},
              'sim_s': round(float(data.time), 2), 'looks': summary['looks'], 'placement': placement,
              'lookback_gate': gate, 'controller': summary, 'phase_times': state['phase_times'],
              'commands': len(commands), 'frames': len(frames)}
    m1_contract.assert_exportable(result)
    jsonl(out/'inputs'/'commands.jsonl', commands)
    jsonl(out/'inputs'/'frames.jsonl', frames)
    jsonl(out/'controller_events.jsonl', ctl.events)
    jsonl(out/'skill_events.jsonl', getattr(ctl.skill, 'events', []) or [])
    jsonl(out/'macros.jsonl', decisions)
    jsonl(out/'eval_only'/'frames_eval.jsonl', frame_eval)
    jsonl(out/'eval_only'/'gt_trajectory.jsonl', gt)
    jsonl(out/'eval_only'/'contacts.jsonl', contacts)
    (out/'scene.xml').write_text(world.scene_xml)
    manifest = {'schema': SCHEMA, 'spec': spec, 'student': student, 'code': code, 'map_id': spec['map'],
                'static_map_sha256': digest(static), 'landmarks_sha256': scene.manifest['landmarks_sha256'],
                'scene_xml_sha256': scene.manifest['scene_xml_sha256'],
                'calibration_sha256': sha_bytes(calibration_path.read_bytes()), 'pose_source': ctl.pose.source,
                'weld': scene.manifest['weld'], 'contact_profile': scene.manifest.get('contact_solver_profile'),
                'timestep_s': float(model.opt.timestep), 'frame_period_s': FRAME_S, 'tick_s': TICK_S, 'sync_sim': True,
                'env': {'python': platform.python_version(), 'platform': platform.platform(),
                        'mujoco': mujoco.__version__, 'opencv': cv2.__version__, 'numpy': np.__version__,
                        'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                                   'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
                'load_average': {'start': [round(v, 2) for v in load_start], 'end': [round(v, 2) for v in os.getloadavg()]},
                'wall_s': round(time.time() - started, 1)}
    manifest['files'] = {str(p.relative_to(out)): sha_bytes(p.read_bytes())
                         for p in sorted(out.rglob('*')) if p.is_file() and p.suffix in ('.jsonl', '.xml')}
    (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, default=str) + '\n')
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    world.close()
    return result, manifest


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--prereg', required=True)
    p.add_argument('--only', default='')
    p.add_argument('--output', required=True)
    args = p.parse_args(argv)
    prereg = json.loads(Path(args.prereg).read_text())
    only = {s for s in args.only.split(',') if s}
    for spec in prereg['episodes']:
        if only and spec['episode_id'] not in only:
            continue
        result, manifest = run(spec, Path(args.output)/spec['episode_id'], prereg['student'])
        print(json.dumps({'episode': spec['episode_id'], 'outcome': result['outcome'], 'm1_success': result['m1_success'],
                          'failed': result['m1_failed_checks'], 'diagnostic_success': result['diagnostic_success'],
                          'false_success': result['false_success'], 'sim_s': result['sim_s'], 'looks': result['looks'],
                          'wall_s': manifest['wall_s'], 'load': manifest['load_average']}), flush=True)


if __name__ == '__main__':
    main()
