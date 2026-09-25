"""Record own-camera localization datasets on a tagged zone map (teacher drives).

The ground-truth zone teacher (scripts/zone_teacher.py, reused read-only) drives
ONE robot along a route through the door. The dataset keeps two parts apart:

* ``inputs/``: what the robot itself has - its wrist RGB frames (raw fisheye
  JPEG), every command it ISSUED (base mecanum/hold, arm/look servo pulses),
  the commanded servo state at each frame (replayed from its own command log),
  and the posture it commanded. The localizer reads only this part and the
  static map.
* ``eval_only/``: simulator truth for offline evaluation and offline noise
  calibration (base pose every physics control step, wrist camera pose and
  box height at each frame). The localizer never reads it at run time.

Modes (the posture the robot commands while moving):
* ``teacher_carry``: the unchanged teacher job (search pose to the box, grasp,
  carry in the teacher's low hover pose, place).
* ``look_search`` / ``look_level``: no cargo; drive the route in the search pose
  (camera -21 deg) or the level carry pose (camera +0.5 deg), and at stops
  sweep the wrist pan centre/left/right.
* ``carry_level``: the teacher grasps the cyan box, then the arm moves to the
  production level carry pose (``CARRY_POSE``) and the robot drives the route
  holding the box, with pan sweeps at stops.
* ``carry_pitch`` (post-hoc diagnostic, 2026-09-25): as ``carry_level`` but the
  wrist (servo 3) is pitched down by ``carry_pitch_deg`` so that wall tags
  appear above the held box in the image.

Sync SIM only (no realtime). Weld OFF (scene builder refuses otherwise).
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

SCHEMA = 'ugrp.owncam_loc_dataset.v1'
CAPTURE_S = .25
SETTLE_S = .6
PANS = {'c': 1500, 'l': 1900, 'r': 1100}
SWEEP = ('c', 'l', 'r', 'c')
STOP_SPACING_M = .6
# Extra stops on both sides of every door (door axis x), metres from its centre.
DOOR_STOPS_M = (-.45, .45)
LEG_LIMIT_S = 150.
ROBOT_IDS = ('r1', 'r2', 'r3')


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as fh:
        for row in rows:
            fh.write(json.dumps(row, ensure_ascii=False) + '\n')


class LoggingPort:
    """Pass-through port that records every command the robot issues."""

    def __init__(self, port, sink):
        self._port, self._sink = port, sink

    def apply(self, action, sim_time):
        self._sink({'t': round(float(sim_time), 4), **{k: v for k, v in action.items()}})
        return self._port.apply(action, sim_time)

    def hold(self, sim_time):
        self._sink({'t': round(float(sim_time), 4), 'kind': 'hold'})
        return self._port.hold(sim_time)

    def __getattr__(self, name):
        return getattr(self._port, name)


class Recorder:
    def __init__(self, spec, out):
        import mujoco
        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_landmarks import TaggedZoneScene
        from scripts.zone_teacher import ZoneTeacherExecutor

        self.spec, self.out = spec, Path(out)
        self.out.mkdir(parents=True, exist_ok=False)
        self.scene = TaggedZoneScene.from_tagged(spec['map'], spec['seed'], spec['goal'], spec.get('extra_boxes'),
                                                 contact_profile=spec.get('contact_profile', 'local_contact_fine'))
        self.world = MultiMasterPiProductionV2(
            seed=spec['seed'], width=640, height=480, render=True, warehouse_layout=self.scene.engine_layout,
            warehouse_cargo_ids=None, xml_transform=self.scene.transform)
        self.scene.setup(self.world)
        self.static = self.scene.config['static_map']
        self.objects = self.scene.config['setup_only']['objects']
        spawns = self.scene.config['setup_only']['spawns']
        # The recorded robot: the one spawned on the requested spawn row.
        self.rid = min(spawns, key=lambda r: abs(spawns[r][1] - spec['spawn_y']))
        self.commands = []
        self.servo = {int(k): int(v) for k, v in self.world.robot(self.rid).servo_command_pulses.items()}
        self.commands.append({'t': 0.0, 'kind': 'initial_servo_command', 'pulses': dict(self.servo)})
        raw = {r: CameraRobotPort(self.world, r, allow_reverse=True, allow_mecanum=True) for r in ROBOT_IDS}
        self.ports = dict(raw)
        self.ports[self.rid] = LoggingPort(raw[self.rid], self._issued)
        self.raw_ports = raw
        self.events = []
        self.executor = ZoneTeacherExecutor(self.world, self.ports, self.static, self.objects, self._log)
        self.robot = self.executor.robots[self.rid]
        self.cam_id = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_CAMERA, f'{self.rid}__robot_cam')
        self.frames = []
        self.gt = []
        self.frame_gt = []
        self.next_capture = 0.
        self.next_gt = 0.
        self.posture = 'initial'
        self.box = next(oid for oid, o in sorted(self.objects.items()) if o['kind'] == spec.get('box_kind', 'cyan'))
        self.box_body = self.objects[self.box]['body_name']
        self.teacher_controls = False
        self.dt = float(self.world.model.opt.timestep)

    # --- robot-side logging (issued commands only) ---
    def _issued(self, row):
        self.commands.append(row)
        if row['kind'] == 'arm':
            self.servo[int(row['servo_id'])] = int(row['pulse'])
        elif row['kind'] == 'look':
            self.servo[6] = int(row['pan_pulse'])

    def _log(self, kind, rid, now, **detail):
        self.events.append({'event': kind, 'robot_id': rid, 'sim_time_s': round(now, 3), **detail})

    # --- teacher/eval-only truth ---
    def _truth(self):
        robot = self.world.robot(self.rid)
        xyz, rpy = robot.base_xyz(), robot.base_rpy()
        return float(xyz[0]), float(xyz[1]), float(rpy[2])

    def now(self):
        return float(self.world.data.time)

    def step(self, seconds):
        for _ in range(max(1, round(seconds/self.dt))):
            now = self.now()
            if self.teacher_controls:
                self.executor.tick(now)
            else:
                self.robot.arm.tick(now)
            for port in self.raw_ports.values():
                port.tick(now)
            self.world._physics_step_for(self.world.controllers['r1'])
            now = self.now()
            if now + 1e-9 >= self.next_gt:
                x, y, yaw = self._truth()
                self.gt.append({'t': round(now, 4), 'x': round(x, 5), 'y': round(y, 5), 'yaw': round(yaw, 6)})
                self.next_gt = now + .05

    def capture(self, label):
        import cv2
        import numpy as np
        now = self.now()
        rgb = self.world.render_rgb(robot_id=self.rid, camera='robot_cam')
        ok, buf = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        data = buf.tobytes()
        index = len(self.frames)
        name = f'frames/{index:05d}.jpg'
        (self.out/name).parent.mkdir(exist_ok=True)
        (self.out/name).write_bytes(data)
        self.frames.append({'frame': index, 't': round(now, 4), 'file': name, 'sha256': sha256(data),
                            'commanded_servo': {str(k): v for k, v in sorted(self.servo.items())},
                            'posture': label})
        x, y, yaw = self._truth()
        cam_pos = self.world.data.cam_xpos[self.cam_id]
        cam_mat = np.asarray(self.world.data.cam_xmat[self.cam_id]).reshape(3, 3)
        self.frame_gt.append({'frame': index, 't': round(now, 4), 'x': round(x, 5), 'y': round(y, 5),
                              'yaw': round(yaw, 6), 'cam_pos': [round(float(v), 5) for v in cam_pos],
                              'cam_mat_mujoco': [round(float(v), 6) for v in cam_mat.ravel()],
                              'box_z': round(float(self.world.data.body(self.box_body).xpos[2]), 4),
                              'phase': self.robot.phase})

    def run_until(self, done, label_fn, limit_s):
        start = self.now()
        while not done() and self.now() - start < limit_s:
            self.step(.05)
            if self.now() + 1e-9 >= self.next_capture:
                self.capture(label_fn())
                self.next_capture = self.now() + CAPTURE_S
        return done()

    # --- postures and routes ---
    def queue_pose(self, pose, settle=SETTLE_S):
        self.robot.arm.queue(pose, self.now())
        self.run_until(lambda: self.now() >= self.robot.arm.until and not self.robot.arm.events,
                       lambda: 'arm_moving', 10.)
        self.hold_for(settle)

    def hold_for(self, seconds):
        self.ports[self.rid].hold(self.now())
        end = self.now() + seconds
        while self.now() < end - 1e-9:
            self.step(.05)

    def sweep(self, base_label):
        for key in SWEEP:
            if self.servo.get(6) != PANS[key]:
                self.robot.arm.queue({6: PANS[key]}, self.now(), duration=.5, settle=0.)
                self.run_until(lambda: self.now() >= self.robot.arm.until and not self.robot.arm.events,
                               lambda: 'arm_moving', 5.)
            self.hold_for(SETTLE_S)
            self.capture(f'{base_label}_{key}')

    def route_stops(self, start, goal, carrying):
        from scripts.zone_teacher import plan_path
        discs = self.executor.discs_for(self.robot, exclude=self.box_body if carrying else None, carrying=carrying)
        path = plan_path(start, goal, self.static['bounds_m'], discs, radius=self.robot.radius(carrying),
                         rects=self.robot.rects)
        if path is None:
            raise RuntimeError(f'no teacher path {start} -> {goal}')
        pts = [tuple(start)] + [tuple(p) for p in path]
        stops, travelled, last = [], 0., pts[0]
        for p in pts[1:]:
            travelled += math.dist(last, p)
            last = p
            near_door = any(abs(p[1] - d['center_m'][1]) < .05 and any(
                abs(p[0] - (d['center_m'][0] + off)) < .03 for off in DOOR_STOPS_M)
                for d in self.static.get('passages', []) if d['kind'] == 'door')
            if travelled >= STOP_SPACING_M or near_door:
                stops.append(p)
                travelled = 0.
        if not stops or math.dist(stops[-1], goal) > .05:
            stops.append(tuple(goal))
        return stops

    def drive_route(self, goal, carrying, label):
        stops = self.route_stops(self._truth()[:2], goal, carrying)
        self._log('route', self.rid, self.now(), goal=list(goal), stops=[[round(v, 3) for v in s] for s in stops])
        for i, stop in enumerate(stops):
            nxt = stops[i+1] if i + 1 < len(stops) else None
            heading = math.atan2(nxt[1]-stop[1], nxt[0]-stop[0]) if nxt else 0.
            state = {'next': 0., 'ok': False}
            self.robot.path = None

            def arrived():
                now = self.now()
                if state['ok'] or now + 1e-9 < state['next']:
                    return state['ok']
                state['next'] = now + .1
                discs = self.executor.discs_for(self.robot, exclude=self.box_body if carrying else None,
                                                carrying=carrying)
                state['ok'] = self.robot._drive_to(stop, heading, now, discs, carrying=carrying, tol=.04)
                return state['ok']
            ok = self.run_until(arrived, lambda: f'{label}_drive', LEG_LIMIT_S)
            self._log('stop', self.rid, self.now(), index=i, reached=ok)
            self.hold_for(.3)
            self.sweep(label)
        return True

    def run(self):
        from scripts.zone_teacher import CLOSED, FOLDED
        from sim.masterpi_production_v2 import CARRY_POSE, SEARCH_POSE
        spec = self.spec
        slot = next(s for s in self.static['zone_slots'][spec['zone']] if s['slot_id'] == spec['slot'])
        sx, sy = slot['center_m']
        approach = (sx - .155 - .08, sy)
        mode = spec['mode']
        self.hold_for(.5)
        outcome = {}
        if mode in ('teacher_carry', 'carry_level', 'carry_pitch'):
            self.teacher_controls = True
            self.robot.assign({'job_id': 'j1', 'box_body': self.box_body, 'slot_xy': [sx, sy]}, self.now())
            labels = {'to_box': 'search_drive', 'align_box': 'search_drive', 'grasp': 'grasp', 'lift': 'grasp',
                      'carry': 'teacher_carry_drive', 'align_slot': 'teacher_carry_drive', 'release': 'place',
                      'retract': 'place', 'back_off': 'place', 'done': 'done', 'failed': 'failed'}
            stop_phase = ('carry',) if mode in ('carry_level', 'carry_pitch') else ('done', 'failed')
            self.run_until(lambda: self.robot.phase in stop_phase or self.robot.phase in ('done', 'failed'),
                           lambda: labels.get(self.robot.phase, self.robot.phase), 400.)
            outcome['teacher_phase'] = self.robot.phase
            outcome['teacher_outcome'] = self.robot.outcome
            self.teacher_controls = False
            if mode in ('carry_level', 'carry_pitch') and self.robot.phase == 'carry':
                self.ports[self.rid].hold(self.now())
                pose = {**CARRY_POSE, 1: CLOSED}
                label = 'carry_level'
                if mode == 'carry_pitch':
                    deg = float(spec['carry_pitch_deg'])
                    pose[3] = CARRY_POSE[3] - round(deg*2000/180)
                    label = f'carry_pitch{int(deg)}'
                self.queue_pose(pose)
                self.drive_route(approach, True, label)
            if mode == 'teacher_carry':
                self.hold_for(.5)
                self.capture('done')
        else:
            pose = SEARCH_POSE if mode == 'look_search' else {**CARRY_POSE, 1: CLOSED}
            label = 'search' if mode == 'look_search' else 'level'
            self.queue_pose(pose)
            self.drive_route(approach, False, label)
            back = spec.get('return_to')
            if back:
                self.drive_route(tuple(back), False, label)
            self.queue_pose(FOLDED)
        outcome['final_box_z'] = round(float(self.world.data.body(self.box_body).xpos[2]), 4)
        return outcome


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def record(spec, out):
    started = time.time()
    load_start = os.getloadavg()
    # Code identity at launch (the run imports its sources now).
    code = {'sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain', '--', 'sim', 'harness',
                                                                 'scripts', 'maps')), 'stamped': 'at launch'}
    rec = Recorder(spec, out)
    outcome = rec.run()
    root = rec.out
    jsonl(root/'inputs'/'commands.jsonl', rec.commands)
    jsonl(root/'inputs'/'frames.jsonl', rec.frames)
    jsonl(root/'eval_only'/'gt_trajectory.jsonl', rec.gt)
    jsonl(root/'eval_only'/'frames_gt.jsonl', rec.frame_gt)
    jsonl(root/'eval_only'/'events.jsonl', rec.events)
    (root/'scene.xml').write_text(rec.world.scene_xml)
    import cv2
    import mujoco
    import numpy
    manifest = {
        'schema': SCHEMA, 'spec': spec, 'robot_id': rec.rid, 'box': rec.box, 'outcome': outcome,
        'frames': len(rec.frames), 'commands': len(rec.commands), 'sim_time_s': round(rec.now(), 3),
        'map_id': spec['map'], 'static_map_sha256': rec.scene.manifest['static_map_sha256'],
        'base_static_map_sha256': rec.scene.manifest['base_static_map_sha256'],
        'landmarks_sha256': rec.scene.manifest['landmarks_sha256'],
        'scene_xml_sha256': rec.scene.manifest['scene_xml_sha256'],
        'scene_file_sha256': sha256((root/'scene.xml').read_bytes()),
        'contact_profile': rec.scene.manifest.get('contact_solver_profile'), 'weld': rec.scene.manifest['weld'],
        'timestep_s': rec.dt, 'capture_period_s': CAPTURE_S, 'settle_s': SETTLE_S, 'pans': PANS,
        'image': {'camera': 'robot_cam', 'width': 640, 'height': 480, 'encoding': 'jpeg q90',
                  'geometry': 'raw fisheye (sim pinhole render remapped by raw_fisheye_remap)'},
        'split_note': 'robot inputs in inputs/, simulator truth in eval_only/ (evaluation/offline calibration only)',
        'code': code,
        'env': {'python': platform.python_version(), 'platform': platform.platform(), 'mujoco': mujoco.__version__,
                'opencv': cv2.__version__, 'numpy': numpy.__version__},
        'load_average': {'start': [round(v, 2) for v in load_start],
                         'end': [round(v, 2) for v in os.getloadavg()]},
        'wall_s': round(time.time() - started, 1), 'sync_sim': True,
    }
    manifest['files'] = {str(p.relative_to(root)): sha256(p.read_bytes())
                         for p in sorted(root.rglob('*')) if p.is_file() and p.suffix in ('.jsonl', '.xml')}
    (root/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    rec.world.close()
    return manifest


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--episodes', required=True, help='JSON file with a list of episode specs')
    p.add_argument('--only', default='', help='comma-separated episode ids (default: all)')
    p.add_argument('--output', required=True, help='output root; one directory per episode')
    args = p.parse_args(argv)
    specs = json.loads(Path(args.episodes).read_text())['episodes']
    only = {s for s in args.only.split(',') if s}
    for spec in specs:
        if only and spec['episode_id'] not in only:
            continue
        manifest = record(spec, Path(args.output)/spec['episode_id'])
        print(json.dumps({'episode': spec['episode_id'], 'frames': manifest['frames'],
                          'sim_time_s': manifest['sim_time_s'], 'outcome': manifest['outcome'],
                          'wall_s': manifest['wall_s'], 'load': manifest['load_average']}), flush=True)


if __name__ == '__main__':
    main()
