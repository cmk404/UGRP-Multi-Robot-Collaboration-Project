#!/usr/bin/env python3
"""Fixed-grasp RGB paired rotation / authored-wall detour experiment."""
from __future__ import annotations

import argparse
import hashlib
import json
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
from harness.pair_navigation import PairNavigator, ROBOTS, authorize_pair, digest, validate_map, retryable_visual_hold
from harness.pair_grasp_spacing import PairGraspSpacing, HOLD_DT
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.run_camera_approach_student import models, sha, write
from scripts.evaluate_pair_navigation import evaluate_samples, evaluate_grasp_stability


class PairNavigationScene(ShortTransportScene):
    """Private setup, raw actuators, and output-only referee. Never an actor API."""
    def __init__(self, out, grasp_root, data, *, impratio=1):
        super().__init__(out, grasp_root)
        if impratio not in (1, 10, 100): raise ValueError('explicit contact impedance profile required')
        self.impratio = impratio
        self.map = data
        self.wall_ids = set()
        self.wall_contact_ticks = 0
        self.unexpected_contact_ticks = 0
        self.contact_events = []
        self.nav_start_s = None
        self.xml_sha = None
        self.spacing_actors = {r:PairGraspSpacing(data,r) for r in ROBOTS}
        self.spacing_sync = PairCarrySync(task_id=data['map_id']+'-grasp-spacing')
        self.spacing_steps = []

    def spacing_frame(self, *, execute=True):
        index = len(self.spacing_steps)
        frames = self.capture(f'spacing-{index:04d}')
        decisions = {r:self.spacing_actors[r].decide(frames[r]['own_bytes'],frames[r]['top_bytes']) for r in ROBOTS}
        permission = authorize_pair(self.spacing_sync,decisions,{r:frames[r]['frame_id'] for r in ROBOTS},index,interval_s=HOLD_DT)
        actions = {r:dict(decisions[r]['action']) for r in ROBOTS}
        if permission['phase'] != 'GO':
            for action in actions.values():action.update(forward=0.,left=0.,turn=0.)
        self.spacing_steps.append({'index':index,'images':{r:{'own':frames[r]['own_rgb'],'top':frames[r]['shared_top_rgb']} for r in ROBOTS},
            'frame_ids':{r:frames[r]['frame_id'] for r in ROBOTS},'decisions':decisions,
            'permission':permission,'issued_actions':actions,'sim_time_s':self.time(),'executed':execute})
        for r in ROBOTS:self.ports[r].apply(actions[r],self.time())
        if not all(d['ready'] for d in decisions.values()):
            raise RuntimeError('grasp spacing stopped: '+json.dumps({r:d.get('error') for r,d in decisions.items()}))
        if execute:
            self.phase = 'grasp_hold'
            self.tick(HOLD_DT)

    def anchor_spacing(self):
        self.spacing_frame(execute=False)

    def hold_spacing(self, duration_s):
        for _ in range(round(duration_s/HOLD_DT)):
            self.spacing_frame()

    def finish_spacing(self):
        self.spacing_frame(execute=False)
        for r in ROBOTS:self.ports[r].stop()
        if not all(d['stable_frames'] >= 5 for d in self.spacing_steps[-1]['decisions'].values()):
            raise RuntimeError('grasp spacing did not visually settle before departure')

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
        self.geom_names = {i: mujoco.mj_id2name(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
                           for i in range(self.world.model.ngeom)}
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
        # Save the final compiled scene, after the inherited plain-beam wrapper.
        mujoco.mj_saveLastXML(str(self.out/'scene.xml'), m)
        self.xml_sha = sha(self.out/'scene.xml')
        return self

    def invariant_record(self):
        record = super().invariant_record()
        opt = self.world.model.opt
        record['contact_solver'] = {k: float(getattr(opt, k)) for k in (
            'impratio', 'cone', 'solver', 'iterations', 'tolerance', 'noslip_iterations', 'timestep')}
        return record

    def _referee_tick(self):
        if hasattr(self, 'geom_names') and self.phase in ('carry', 'carry_stop'):
            wall_hit, unexpected = False, False
            for c in self.world.data.contact[:self.world.data.ncon]:
                pair = {int(c.geom1), int(c.geom2)}
                if pair & self.wall_ids and pair & (self.robot_geoms | {self.beam_geom}):
                    wall_hit = True
                a, b = int(c.geom1), int(c.geom2)
                na, nb = self.geom_names[a], self.geom_names[b]
                if 'floor' in (na, nb) and self.beam_geom not in pair:
                    continue
                if self.beam_geom in pair:
                    if not pair & self.robot_geoms: unexpected = True
                    continue
                ra, rb = na.split('__')[0], nb.split('__')[0]
                if (ra in ROBOTS or rb in ROBOTS) and ra != rb:
                    unexpected = True
            self.wall_contact_ticks += int(wall_hit)
            self.unexpected_contact_ticks += int(unexpected)
            if wall_hit or unexpected:
                self.contact_events.append({'sim_time_s':self.time(), 'wall':wall_hit, 'unexpected':unexpected})
        super()._referee_tick()

    def evaluation_snapshot(self, *, full_state=True):
        from sim.cooperative_payload import beam_pose
        result = super().evaluation_snapshot(full_state=full_state)
        result['yaw_rad'] = float(beam_pose(self.world.data, self.world.model)['yaw_rad'])
        result['arm_joints'] = {rid: {name: float(self.world.data.qpos[self.world.model.jnt_qposadr[jid]])
                               for name, jid in self.world.controllers[rid].arm_joint.items()} for rid in ROBOTS}
        # Conservative world AABBs of the actual physical robot/payload geoms.
        import numpy as np
        m, d = self.world.model, self.world.data
        inside = True
        xmin, xmax, ymin, ymax = self.map['bounds_m']
        for gid in self.robot_geoms | {self.beam_geom}:
            if not (m.geom_contype[gid] or m.geom_conaffinity[gid]): continue
            size = m.geom_size[gid]
            # Boxes: exact AABB. Other types: bounding sphere, conservatively.
            if int(m.geom_type[gid]) == 6:
                extent = np.abs(d.geom_xmat[gid].reshape(3,3)) @ size
            else:
                extent = np.full(3, m.geom_rbound[gid])
            x, y, _ = d.geom_xpos[gid]
            inside &= xmin <= x-extent[0] and x+extent[0] <= xmax and ymin <= y-extent[1] and y+extent[1] <= ymax
        result['within_authored_bounds'] = bool(inside)
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
    return evaluate_samples(scene.evaluation_samples, scene.map, arrived=report['arrived'],
                            invariants_match=report.get('invariants_initial') == report.get('invariants_final'),
                            weld_ticks=scene.weld_active_ticks, wall_contact_ticks=scene.wall_contact_ticks,
                            unexpected_contact_ticks=scene.unexpected_contact_ticks,
                            require_full_grasp=report.get('grasp_spacing')=='visual')


def run(data, grasp_root, out, budget=750, *, impratio=1, vision_mode='legacy', grasp_spacing=None, grasp_only=False, close_pulse=None):
    validate_map(data)
    grasp_spacing = grasp_spacing or ('visual' if vision_mode=='robust' else 'passive')
    if grasp_spacing not in ('visual','passive'):raise ValueError('unknown grasp spacing mode')
    if close_pulse is None and grasp_spacing=='visual':close_pulse=1700
    if out.exists(): raise FileExistsError(out)
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit the complete execution source and protocol before an experiment')
    skill, grasp_models = models(grasp_root, 'student-skill.json')
    scene = PairNavigationScene(out, grasp_root, data, impratio=impratio)
    actors = {r: PairNavigator(data, r, vision_mode=vision_mode) for r in ROBOTS}
    sync = PairCarrySync(task_id=data['map_id'])
    started = time.monotonic()
    report = {'schema': 'ugrp.pair_navigation_trial.v1', 'map': data, 'map_sha256': digest(data),
              'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
              'grasp_skill_sha256': sha(grasp_root/'student-skill.json'),
              'appearance_manifest_sha256': sha(ROOT/'harness/assets/pair_navigation/manifest.json'),
              'environment': {'python': sys.version, 'platform': platform.platform()},
              'scope': 'Fixed-grasp classical RGB pair navigation; zero LLM calls; placement is demonstration replay',
              'budget': budget, 'steps': [], 'arrived': False, 'error': None,
              'contact_impratio': impratio,
              'vision_mode': vision_mode,
              'grasp_spacing':grasp_spacing,'grasp_only':bool(grasp_only),
              'close_command_override':close_pulse,
              'external_model_calls': 0, 'cost_usd': 0}
    try:
        import mujoco
        report['environment']['mujoco'] = mujoco.__version__
        scene.open()
        report['scene_xml_sha256'] = scene.xml_sha
        report['invariants_initial'] = scene.invariant_record()
        scene.finish_grasp(predict_student, grasp_models,
                          after_close=scene.anchor_spacing if grasp_spacing=='visual' else None,
                          hold=scene.hold_spacing if grasp_spacing=='visual' else None, close_pulse=close_pulse)
        # Fixed settling interval for all numerical profiles. No contact/pose
        # condition controls this wait or the start of navigation.
        scene.phase = 'grasp_hold'
        (scene.hold_spacing if grasp_spacing=='visual' else scene.tick)(8.)
        if grasp_spacing=='visual':scene.finish_spacing()
        report['post_grasp_settle_s'] = 8.
        write(out/'grasp-result.json', scene.grasp_report)
        for index in range(0 if grasp_only else budget):
            frames = scene.capture(f'nav-{index:04d}')
            decisions = {r: actors[r].decide(frames[r]['own_bytes'], frames[r]['top_bytes']) for r in ROBOTS}
            permission = authorize_pair(sync, decisions, {r: frames[r]['frame_id'] for r in ROBOTS}, index)
            actions = {r: dict(decisions[r]['action']) for r in ROBOTS}
            if permission['phase'] != 'GO':
                for a in actions.values(): a.update(forward=0., left=0., turn=0.)
            row = {'index': index, 'images': {r: {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']} for r in ROBOTS},
                   'frame_ids': {r: frames[r]['frame_id'] for r in ROBOTS}, 'decisions': decisions,
                   'permission': permission, 'issued_actions': actions, 'sim_time_s': scene.time()}
            report['steps'].append(row)
            scene.execute(actions)
            if not all(d['ready'] for d in decisions.values()):
                if retryable_visual_hold(decisions):
                    continue  # Both commands were zero; capture a fresh pair of images.
                if all(d['status'] == 'no_route' for d in decisions.values()):
                    report['refused_no_route'] = True
                    break
                raise RuntimeError('policy stopped: '+json.dumps({r: d['status'] for r,d in decisions.items()}))
            if all(d['done'] for d in decisions.values()):
                report['arrived'] = True
                break
        else:
            if not grasp_only:
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
            report['wall_contact_ticks'] = scene.wall_contact_ticks
            report['unexpected_contact_ticks'] = scene.unexpected_contact_ticks
            report['evaluation'] = evaluate(scene, report)
            report['grasp_stability'] = evaluate_grasp_stability(scene.evaluation_samples)
            report['sim_seconds'] = scene.time()
        report['sync_events'] = sync.events
        if scene.grasp_report is not None:
            write(out/'grasp-result.json',scene.grasp_report)
        report['spacing_steps'] = scene.spacing_steps
        report['spacing_sync_events'] = scene.spacing_sync.events
        report['wall_seconds'] = time.monotonic()-started
        report['success'] = bool(not report['error'] and report.get('evaluation',{}).get('success'))
        if grasp_only:report['success'] = bool(not report['error'] and report.get('grasp_stability',{}).get('success'))
        write(out/'contact-events-evaluation-only.json', scene.contact_events)
        try:
            scene.close()
        except Exception:
            report['cleanup_error'] = traceback.format_exc()
            report['success'] = False
        finally:
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
    parser.add_argument('--grasp-spacing',choices=('passive','visual'),help='robust defaults to visual; passive retains the previous comparison')
    parser.add_argument('--grasp-only',action='store_true',help='bounded pre-drive diagnostic; does not claim navigation success')
    parser.add_argument('--close-pulse',type=int,choices=(1500,1600,1700,1800),help='visual spacing defaults to 1700; passive retains the skill command; no contact feedback')
    parser.add_argument('--vision-mode', choices=('legacy','temporal','temporal-edges','robust'), default='legacy',
                        help='explicit vision/control comparison; robust adds wheel geometry and own-view carry guard')
    parser.add_argument('--impratio', type=int, choices=(1, 10, 100), default=1,
                        help='explicit friction impedance comparison; default preserves main')
    args = parser.parse_args()
    if not 1 <= args.budget <= 1200: parser.error('budget must be 1..1200')
    result = run(json.loads(args.map.read_text()), args.grasp_model_dir.resolve(), args.out_dir.resolve(), args.budget,
        impratio=args.impratio, vision_mode=args.vision_mode, grasp_spacing=args.grasp_spacing, grasp_only=args.grasp_only,close_pulse=args.close_pulse)
    return int(bool(result['error']))


if __name__ == '__main__':
    raise SystemExit(main())
