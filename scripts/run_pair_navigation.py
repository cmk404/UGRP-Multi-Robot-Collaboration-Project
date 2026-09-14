#!/usr/bin/env python3
"""Fixed-grasp RGB paired rotation / authored-wall detour experiment."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))

from harness.grasp_student_inference import predict_student
from harness.pair_carry_sync import PairCarrySync
from harness.pair_navigation import PairNavigator, ROBOTS, authorize_pair, digest, validate_map, wrap
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.run_camera_approach_student import models, sha, write


class PairNavigationScene(ShortTransportScene):
    """Private setup, raw actuators, and output-only referee. Never an actor API."""
    def __init__(self, out, grasp_root, data, *, impratio=1):
        super().__init__(out, grasp_root)
        if impratio not in (1, 10, 100): raise ValueError('explicit contact impedance profile required')
        self.impratio = impratio
        self.map = data
        self.wall_ids = set()
        self.wall_contact_ticks = 0
        self.nav_start_s = None
        self.xml_sha = None

    def open(self):
        import mujoco
        import numpy as np
        import sim.multi_masterpi_production as production
        from sim.camera_robot_port import CameraRobotPort
        original = production.build_multi_robot_xml
        def builder(*args, **kwargs):
            root = ET.fromstring(original(*args, **kwargs))
            root.find('option').set('impratio', str(self.impratio))
            world = root.find('worldbody')
            for box in self.map['obstacles']:
                x, y = box['center_m']; hx, hy = box['half_extents_m']; height = box['height_m']
                ET.SubElement(world, 'geom', name='pair_wall_'+box['id'], type='box',
                              pos=f'{x} {y} {height/2}', size=f'{hx} {hy} {height/2}',
                              rgba='.35 .38 .42 1', contype='1', conaffinity='1')
            xml = ET.tostring(root, encoding='unicode')
            self.xml_sha = hashlib.sha256(xml.encode()).hexdigest()
            return xml
        with patch.object(production, 'build_multi_robot_xml', builder):
            super().open()
        self.ports = {r: CameraRobotPort(self.world, r, allow_reverse=True, allow_mecanum=True) for r in ROBOTS}
        self.wall_ids = {i for i in range(self.world.model.ngeom)
            if (mujoco.mj_id2name(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('pair_wall_')}
        # Presentation camera only. Actor camera parameters stay byte-for-byte.
        m = self.world.model
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_warehouse')
        position, target = np.array([.25, -3.9, 2.4]), np.array([.25, -2., .1])
        forward = target-position; forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
        up = np.cross(right, forward); quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, np.column_stack((right, up, -forward)).ravel())
        m.cam_pos[cid] = position; m.cam_quat[cid] = quat; m.cam_fovy[cid] = 55.
        mujoco.mj_forward(m, self.world.data)
        return self

    def invariant_record(self):
        record = super().invariant_record()
        opt = self.world.model.opt
        record['contact_solver'] = {k: float(getattr(opt, k)) for k in (
            'impratio', 'cone', 'solver', 'iterations', 'tolerance', 'noslip_iterations', 'timestep')}
        return record

    def _referee_tick(self):
        if self.wall_ids and self.phase in ('carry', 'carry_stop'):
            for c in self.world.data.contact[:self.world.data.ncon]:
                pair = {int(c.geom1), int(c.geom2)}
                if pair & self.wall_ids and pair & (self.robot_geoms | {self.beam_geom}):
                    self.wall_contact_ticks += 1
                    break
        super()._referee_tick()

    def evaluation_snapshot(self, *, full_state=True):
        from sim.cooperative_payload import beam_pose
        result = super().evaluation_snapshot(full_state=full_state)
        result['yaw_rad'] = float(beam_pose(self.world.data, self.world.model)['yaw_rad'])
        result['arm_joints'] = {rid: {name: float(self.world.data.qpos[self.world.model.jnt_qposadr[jid]])
                               for name, jid in self.world.controllers[rid].arm_joint.items()} for rid in ROBOTS}
        return result

    def execute(self, actions):
        if set(actions) != set(ROBOTS): raise ValueError('two commands required')
        self.phase = 'carry' if any(any(abs(a[k]) > 1e-12 for k in ('forward', 'left', 'turn')) for a in actions.values()) else 'carry_stop'
        start = self.time()
        for rid in ROBOTS:
            self.ports[rid].apply(actions[rid], start)
        self.tick(.2)
        self.trace.append({'stage': self.phase, 'actions': actions,
                           'start_sim_time_s': start, 'end_sim_time_s': self.time()})


def evaluate(scene, report):
    """Only called after the actor stops. No result is sent back to either actor."""
    rows = [r for r in scene.evaluation_samples if r['phase'] in ('carry', 'carry_stop')]
    if not rows: return {'success': False, 'reason': 'no navigation samples'}
    first, last = rows[0], rows[-1]
    bilateral = [all(r['contacts'][rid]['bilateral'] for rid in ROBOTS) for r in rows]
    lifted = [r['height_above_start_m'] >= .03 for r in rows]
    angle = wrap(last['yaw_rad']-first['yaw_rad'])
    goal = scene.map['goal']
    displacement = math.dist(last['position_m'][:2], goal['center_m'])
    error_yaw = abs(wrap(angle-math.radians(goal['relative_yaw_deg'])))
    gates = {'actor_arrived': report['arrived'], 'bilateral_every_sample': all(bilateral),
             'lifted_every_sample': all(lifted), 'wall_contact_free': scene.wall_contact_ticks == 0,
             'weld_off': scene.weld_active_ticks == 0,
             'cameras_geometry_unchanged': report['invariants_initial'] == report['invariants_final'],
             'goal_position': displacement <= .08, 'goal_orientation': error_yaw <= math.radians(10)}
    return {'success': all(gates.values()), 'gates': gates, 'samples': len(rows),
            'bilateral_fraction': sum(bilateral)/len(rows), 'lifted_fraction': sum(lifted)/len(rows),
            'min_lift_m': min(r['height_above_start_m'] for r in rows),
            'payload_turn_deg': math.degrees(angle), 'goal_position_error_m': displacement,
            'goal_yaw_error_deg': math.degrees(error_yaw), 'wall_contact_ticks': scene.wall_contact_ticks,
            'navigation_sim_seconds': last['sim_time_s']-first['sim_time_s'],
            'sample_gap_max_s': max((b['sim_time_s']-a['sim_time_s'] for a,b in zip(rows,rows[1:])), default=0.)}


def run(data, grasp_root, out, budget=750, *, impratio=1):
    validate_map(data)
    if out.exists(): raise FileExistsError(out)
    skill, grasp_models = models(grasp_root, 'student-skill.json')
    scene = PairNavigationScene(out, grasp_root, data, impratio=impratio)
    actors = {r: PairNavigator(data, r) for r in ROBOTS}
    sync = PairCarrySync(task_id=data['map_id'])
    started = time.monotonic()
    report = {'schema': 'ugrp.pair_navigation_trial.v1', 'map': data, 'map_sha256': digest(data),
              'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'grasp_skill_sha256': sha(grasp_root/'student-skill.json'),
              'environment': {'python': sys.version, 'platform': platform.platform()},
              'scope': 'Fixed-grasp classical RGB pair navigation; zero LLM calls; placement is demonstration replay',
              'budget': budget, 'steps': [], 'arrived': False, 'error': None,
              'contact_impratio': impratio,
              'external_model_calls': 0, 'cost_usd': 0}
    try:
        import mujoco
        report['environment']['mujoco'] = mujoco.__version__
        scene.open()
        report['scene_xml_sha256'] = scene.xml_sha
        report['invariants_initial'] = scene.invariant_record()
        scene.finish_grasp(predict_student, grasp_models)
        # Fixed settling interval for all numerical profiles. No contact/pose
        # condition controls this wait or the start of navigation.
        scene.phase = 'grasp_hold'
        scene.tick(5.)
        report['post_grasp_settle_s'] = 5.
        write(out/'grasp-result.json', scene.grasp_report)
        for index in range(budget):
            frames = scene.capture(f'nav-{index:04d}')
            decisions = {r: actors[r].decide(frames[r]['own_bytes'], frames[r]['top_bytes']) for r in ROBOTS}
            permission = authorize_pair(sync, decisions, {r: frames[r]['frame_id'] for r in ROBOTS}, index)
            actions = {r: dict(decisions[r]['action']) for r in ROBOTS}
            if permission['phase'] != 'GO':
                for a in actions.values(): a.update(forward=0., left=0., turn=0.)
            row = {'index': index, 'images': {r: {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']} for r in ROBOTS},
                   'frame_ids': {r: frames[r]['frame_id'] for r in ROBOTS}, 'decisions': decisions,
                   'permission': permission, 'issued_actions': actions}
            report['steps'].append(row)
            scene.execute(actions)
            if not all(d['ready'] for d in decisions.values()):
                if all(d['status'] == 'no_route' for d in decisions.values()):
                    report['refused_no_route'] = True
                    break
                raise RuntimeError('policy stopped: '+json.dumps({r: d['status'] for r,d in decisions.items()}))
            if all(d['done'] for d in decisions.values()):
                report['arrived'] = True
                break
        else:
            raise RuntimeError('RGB navigation budget exhausted')
        if report['arrived']:
            for _ in range(5):
                scene.execute({r: {'kind':'mecanum', 'forward':0., 'left':0., 'turn':0., 'duration_s':.2} for r in ROBOTS})
            scene.capture('navigation-settled')
            scene.place()
    except Exception:
        report['error'] = traceback.format_exc()
    finally:
        if scene.world:
            report['invariants_final'] = scene.invariant_record()
            report['weld_active_ticks'] = scene.weld_active_ticks
            report['evaluation'] = evaluate(scene, report)
            report['sim_seconds'] = scene.time()
        report['sync_events'] = sync.events
        report['wall_seconds'] = time.monotonic()-started
        scene.close()
        out.mkdir(parents=True, exist_ok=True)
        write(out/'result.json', report)
    print(json.dumps({'out': str(out), 'evaluation': report.get('evaluation'), 'error': report['error']}, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--grasp-model-dir', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--budget', type=int, default=750)
    parser.add_argument('--impratio', type=int, choices=(1, 10, 100), default=1,
                        help='explicit friction impedance comparison; default preserves main')
    args = parser.parse_args()
    if not 1 <= args.budget <= 1200: parser.error('budget must be 1..1200')
    result = run(json.loads(args.map.read_text()), args.grasp_model_dir.resolve(), args.out_dir.resolve(), args.budget,
                 impratio=args.impratio)
    return int(bool(result['error']))


if __name__ == '__main__':
    raise SystemExit(main())
