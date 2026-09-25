"""Closed-loop own-camera drive through the door (student on its own estimate).

The runner is the physics owner. It renders the robot's wrist frames, feeds
the student (``harness.owncam_drive.OwnCamDriver``) only robot-side inputs
(issued commands, JPEG frames, static map, calibrations, static keep-outs) and
applies the student's commands to the robot's port. Simulator truth is written
to ``eval_only/`` for scoring and never passed to the student.

Box condition: the GT zone teacher (read-only reuse of scripts/zone_teacher.py)
drives to the cyan box, grasps and lifts it (weld OFF); the student takes over
at the teacher's carry phase. That teacher part is not a student result.
Sync SIM only.
"""
from __future__ import annotations

import argparse
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

SCHEMA = 'ugrp.owncam_loop_run.v1'
FRAME_S = .2
STUDENT_LIMIT_S = 240.
TEACHER_LIMIT_S = 400.
DOOR_REGION = (.6, .5)
CALIBRATION = ROOT/'experiments'/'2026-09-25-zone-owncam-loop'/'calibration_loop.json'
KEEPOUT_HALF_M = .06


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in rows))


def pickup_keepouts():
    """Static layout: every pickup-grid cell where a box may stand (all episodes)."""
    from sim.zone_arena import LAYOUTS
    spec = LAYOUTS['zone_wide']
    return [{'id': f'pickup_cell_{i}_{j}', 'center_m': [x, y], 'half_extents_m': [KEEPOUT_HALF_M, KEEPOUT_HALF_M],
             'source': 'static pickup-grid layout cell (possible box), not a live pose'}
            for i, x in enumerate(spec['pickup_columns_x']) for j, y in enumerate(spec['pickup_rows_y'])]


def run(spec, out):
    import cv2
    import mujoco
    import numpy as np
    from harness.owncam_drive import OwnCamDriver
    from scripts.record_owncam_localization import LoggingPort
    from scripts.zone_teacher import ZoneTeacherExecutor
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.research_dispatch_arena import digest
    from sim.zone_landmarks import TaggedZoneScene

    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    started, load_start = time.time(), os.getloadavg()
    code = {'sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain', '--', 'sim', 'harness',
                                                                 'scripts', 'maps')), 'stamped': 'at launch'}
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
    door = next(p for p in static['passages'] if p['kind'] == 'door')
    door_xy = door['center_m']
    goal = [door_xy[0] + .45, door_xy[1]]
    loaded = spec['condition'] == 'box'
    calibration = json.loads(CALIBRATION.read_text())
    keepouts = pickup_keepouts()
    initial = {int(k): int(v) for k, v in world.robot(rid).servo_command_pulses.items()}
    student = OwnCamDriver(static, calibration['params'], loaded=loaded, goal_xy=goal, door_xy=door_xy,
                           keepouts=keepouts, initial_servo=initial, seed=spec['seed'])
    commands = []

    def sink(row):
        commands.append(row)
        student.on_command(row)
    sink({'t': 0.0, 'kind': 'initial_servo_command', 'pulses': dict(initial)})
    raw = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in ('r1', 'r2', 'r3')}
    ports = dict(raw)
    port = ports[rid] = LoggingPort(raw[rid], sink)
    events = []
    executor = ZoneTeacherExecutor(world, ports, static, objects, lambda k, r, t, **d: events.append(
        {'event': k, 'robot_id': r, 'sim_time_s': round(t, 3), **d}))
    teacher = executor.robots[rid]
    box = next(oid for oid, o in sorted(objects.items()) if o['kind'] == spec.get('box_kind', 'cyan'))
    box_body = objects[box]['body_name']
    model, data = world.model, world.data
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(model.ngeom)]
    wall_geoms = {g for g, n in enumerate(names) if n.startswith('zone_wall_')}
    own_geoms = {g for g, n in enumerate(names) if n.startswith(rid + '__')}
    peer_geoms = {g for g, n in enumerate(names) if n.startswith(('r1__', 'r2__', 'r3__')) and g not in own_geoms}
    box_geom = {g for g, n in enumerate(names) if n == box_body + '_geom'}
    other_boxes = {g for g, n in enumerate(names) if n.startswith('cargo_box_') and g not in box_geom}
    dt = float(model.opt.timestep)
    frames, frame_eval, gt, contacts = [], [], [], []
    state = {'phase': 'teacher' if loaded else 'student', 'student_start': None, 'next_frame': 0., 'next_tick': 0.,
             'next_gt': 0., 'min_box_z_student': None}

    def truth():
        r = world.robot(rid)
        xyz, rpy = r.base_xyz(), r.base_rpy()
        return float(xyz[0]), float(xyz[1]), float(rpy[2])

    def observe(now):
        rgb = world.render_rgb(robot_id=rid, camera='robot_cam')
        ok, buf = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        jpeg = buf.tobytes()
        index = len(frames)
        name = f'frames/{index:05d}.jpg'
        (out/'frames').mkdir(exist_ok=True)
        (out/name).write_bytes(jpeg)
        seen = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        est = student.observe(now, seen)          # the student sees exactly the saved JPEG
        frames.append({'frame': index, 't': round(now, 4), 'file': name,
                       'sha256': hashlib.sha256(jpeg).hexdigest(), 'phase': state['phase'],
                       'student_state': student.state,
                       'commanded_servo': {str(k): v for k, v in sorted(student.servo.items())}})
        x, y, yaw = truth()
        row = {'frame': index, 't': round(now, 4), 'gt': [round(x, 5), round(y, 5), round(yaw, 6)],
               'phase': state['phase'], 'student_state': student.state, 'tags': est.get('tags', []),
               'initialized': est.get('initialized', False),
               'door_region': abs(x - door_xy[0]) <= DOOR_REGION[0] and abs(y - door_xy[1]) <= DOOR_REGION[1],
               'box_z': round(float(data.body(box_body).xpos[2]), 4)}
        if est.get('initialized'):
            row.update(est=[round(est['x'], 5), round(est['y'], 5), round(est['yaw'], 6)],
                       pos_err_m=round(math.hypot(est['x'] - x, est['y'] - y), 5),
                       yaw_err_deg=round(abs(math.degrees((est['yaw'] - yaw + math.pi) % (2*math.pi) - math.pi)), 4),
                       std_xy_m=round(est['std_xy_m'], 5))
        frame_eval.append(row)

    def check_contacts(now):
        for i in range(data.ncon):
            c = data.contact[i]
            pair = {int(c.geom1), int(c.geom2)}
            mine = pair & (own_geoms | (box_geom if loaded else set()))
            if not mine:
                continue
            other = next(iter(pair - mine), None)
            kind = ('wall' if other in wall_geoms else 'peer_robot' if other in peer_geoms else
                    'other_box' if other in other_boxes else None)
            if kind and (not contacts or contacts[-1]['t'] < now - .1 or contacts[-1]['kind'] != kind):
                contacts.append({'t': round(now, 4), 'kind': kind, 'phase': state['phase'],
                                 'geoms': sorted(names[g] for g in pair)})

    if loaded:
        slot = static['zone_slots']['A'][0]['center_m']
        teacher.assign({'job_id': 'grasp', 'box_body': box_body, 'slot_xy': slot}, float(data.time))
    outcome = None
    while True:
        now = float(data.time)
        if state['phase'] == 'teacher':
            executor.tick(now)
            if teacher.phase == 'carry':
                port.hold(now)
                state['phase'], state['student_start'] = 'student', now
                state['next_tick'] = now
            elif teacher.phase in ('done', 'failed') or now > TEACHER_LIMIT_S:
                outcome = 'teacher_failure:' + str(teacher.outcome or teacher.phase)
                break
        elif state['student_start'] is None:
            state['student_start'] = now
        if now + 1e-9 >= state['next_frame']:
            observe(now)
            state['next_frame'] = now + FRAME_S
        if state['phase'] == 'student' and now + 1e-9 >= state['next_tick']:
            for cmd in student.tick(now):
                if cmd['kind'] == 'hold':
                    port.hold(now)
                else:
                    port.apply(cmd, now)
            state['next_tick'] = now + .1
            if student.outcome:
                outcome = student.outcome
                break
            if now - state['student_start'] > STUDENT_LIMIT_S:
                outcome = 'time_limit'
                break
        for p in raw.values():
            p.tick(now)
        world._physics_step_for(world.controllers['r1'])
        now = float(data.time)
        check_contacts(now)
        if state['phase'] == 'student':
            z = float(data.body(box_body).xpos[2])
            state['min_box_z_student'] = z if state['min_box_z_student'] is None else min(state['min_box_z_student'], z)
        if now + 1e-9 >= state['next_gt']:
            x, y, yaw = truth()
            gt.append({'t': round(now, 4), 'x': round(x, 5), 'y': round(y, 5), 'yaw': round(yaw, 6),
                       'phase': state['phase']})
            state['next_gt'] = now + .05
    x, y, yaw = truth()
    end_box_z = float(data.body(box_body).xpos[2])
    student_frames = [r for r in frame_eval if r['phase'] == 'student']
    door_errs = sorted(r['pos_err_m'] for r in student_frames if r['door_region'] and 'pos_err_m' in r)
    door_yaw = sorted(r['yaw_err_deg'] for r in student_frames if r['door_region'] and 'yaw_err_deg' in r)
    door_uninit = sum(1 for r in student_frames if r['door_region'] and 'pos_err_m' not in r)

    def p90(values):
        return None if not values else float(np.percentile(values, 90))
    arrived = outcome == 'arrived'
    exit_err = math.hypot(x - goal[0], y - goal[1])
    wall = [c for c in contacts if c['kind'] == 'wall' and c['phase'] == 'student']
    gates = {'R1_exit_waypoint': {'pass': bool(arrived and exit_err <= .10), 'gt_distance_m': round(exit_err, 4),
                                  'declared_arrival': arrived},
             'R2_no_wall_contact': {'pass': not wall, 'wall_contacts': len(wall)},
             'R3_estimate_near_door': {'pass': bool(door_errs and door_uninit == 0 and p90(door_errs) < .06),
                                       'p90_pos_m': p90(door_errs), 'p90_yaw_deg': p90(door_yaw),
                                       'frames': len(door_errs), 'uninitialized': door_uninit}}
    if loaded:
        mz = state['min_box_z_student']
        gates['R4_box_held'] = {'pass': bool(mz is not None and mz > .045 and end_box_z > .045),
                                'min_box_z_m': None if mz is None else round(mz, 4), 'end_box_z_m': round(end_box_z, 4)}
    passed = (not outcome.startswith('teacher_failure')) and all(g['pass'] for g in gates.values())
    vis = sum(bool(r['tags']) for r in student_frames)/len(student_frames) if student_frames else None
    result = {'schema': SCHEMA, 'episode': spec['episode_id'], 'split': spec['split'], 'condition': spec['condition'],
              'robot_id': rid, 'outcome': outcome, 'episode_pass': passed, 'gates': gates,
              'student_sim_s': None if state['student_start'] is None else round(float(data.time) - state['student_start'], 3),
              'teacher_sim_s': round(state['student_start'], 3) if loaded and state['student_start'] else None,
              'looks': student.looks, 'look_reasons': [e.get('reason') for e in student.log if e['event'] == 'state'
                                                       and e.get('reason')],
              'student_commands': sum(1 for c in commands if c['t'] >= (state['student_start'] or 0)),
              'student_frames': len(student_frames), 'student_tag_visibility': None if vis is None else round(vis, 3),
              'contacts_student': {k: sum(1 for c in contacts if c['kind'] == k and c['phase'] == 'student')
                                   for k in ('wall', 'peer_robot', 'other_box')},
              'final_gt': [round(x, 4), round(y, 4), round(yaw, 5)], 'goal_m': goal}
    jsonl(out/'inputs'/'commands.jsonl', commands)
    jsonl(out/'inputs'/'frames.jsonl', frames)
    jsonl(out/'student_log.jsonl', student.log)
    jsonl(out/'eval_only'/'frames_eval.jsonl', frame_eval)
    jsonl(out/'eval_only'/'gt_trajectory.jsonl', gt)
    jsonl(out/'eval_only'/'contacts.jsonl', contacts)
    jsonl(out/'eval_only'/'teacher_events.jsonl', events)
    (out/'scene.xml').write_text(world.scene_xml)
    import cv2 as _cv2
    manifest = {'schema': SCHEMA, 'spec': spec, 'code': code, 'map_id': spec['map'],
                'static_map_sha256': digest(static), 'landmarks_sha256': scene.manifest['landmarks_sha256'],
                'base_static_map_sha256': scene.manifest['base_static_map_sha256'],
                'scene_xml_sha256': scene.manifest['scene_xml_sha256'],
                'calibration_sha256': hashlib.sha256(CALIBRATION.read_bytes()).hexdigest(),
                'keepouts_sha256': digest(keepouts), 'weld': scene.manifest['weld'],
                'contact_profile': scene.manifest.get('contact_solver_profile'), 'timestep_s': dt,
                'frame_period_s': FRAME_S, 'control_period_s': .1, 'sync_sim': True,
                'env': {'python': platform.python_version(), 'platform': platform.platform(),
                        'mujoco': mujoco.__version__, 'opencv': _cv2.__version__, 'numpy': np.__version__,
                        'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                                   'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
                'load_average': {'start': [round(v, 2) for v in load_start],
                                 'end': [round(v, 2) for v in os.getloadavg()]},
                'wall_s': round(time.time() - started, 1)}
    manifest['files'] = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in sorted(out.rglob('*')) if p.is_file() and p.suffix in ('.jsonl', '.xml')}
    (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    world.close()
    return result, manifest


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--prereg', required=True, help='pre-registration JSON with the episode list')
    p.add_argument('--only', default='', help='comma-separated episode ids')
    p.add_argument('--output', required=True)
    args = p.parse_args(argv)
    specs = json.loads(Path(args.prereg).read_text())['episodes']
    only = {s for s in args.only.split(',') if s}
    for spec in specs:
        if only and spec['episode_id'] not in only:
            continue
        result, manifest = run(spec, Path(args.output)/spec['episode_id'])
        print(json.dumps({'episode': spec['episode_id'], 'outcome': result['outcome'], 'pass': result['episode_pass'],
                          'gates': {k: v['pass'] for k, v in result['gates'].items()},
                          'student_sim_s': result['student_sim_s'], 'looks': result['looks'],
                          'wall_s': manifest['wall_s'], 'load': manifest['load_average']}), flush=True)


if __name__ == '__main__':
    main()
