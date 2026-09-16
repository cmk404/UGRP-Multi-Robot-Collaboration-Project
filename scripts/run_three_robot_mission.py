#!/usr/bin/env python3
"""Three-peer target selection -> concurrent pair and solo physical transport."""
from __future__ import annotations

import base64
from functools import partial
import json
import math
from pathlib import Path
import sys
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.three_robot_plan import ROBOTS, TeamAgreement
from harness.three_robot_mission import (GOALS, validate_mission, mission_fixture,
    validate_mission_reply, build_mission_request)
from harness.solo_box_transport import SoloBoxTransport
from harness.visual_macro_runtime import VisualMacroExecutor
from scripts.run_camera_goal_transport import GoalScene, build_parser, run
from scripts.camera_approach_scene import image_record
from scripts.camera_skill_gate import CameraSkillGate
from scripts.three_robot_runtime import ThreeRobotRuntime, write

# Scene construction/referee constants. Never passed into either motor policy.
SOLO_START = (.50, -3.0, .016)
SOLO_GOALS = {'near_magenta': (1.0, -3.0), 'far_magenta': (1.45, -3.0)}
GOAL_HALF = (.16, .14)


def mission_xml(builder):
    def build(*args, **kwargs):
        from sim.multi_masterpi_production import DEFAULT_SPAWNS
        kwargs['spawns'] = {**kwargs.get('spawns', DEFAULT_SPAWNS),
                            'r2': (.0, -3.0, .032355118817659255, 0.)}
        root = ET.fromstring(builder(*args, **kwargs))
        world = root.find('worldbody')
        from sim.warehouse_mission import CargoSpec, _cargo_body
        spec = CargoSpec('small_box_01', 'small_box', 'small_box', 'mission_cyan_box',
                         'mission_cyan_free', (.034,.040,.032), .03,
                         SOLO_START, (*SOLO_GOALS['near_magenta'],.016), .020,
                         required_carriers=1)
        world.append(_cargo_body(spec))  # Unmodified production box appearance/physics.
        for name, (x,y) in SOLO_GOALS.items():
            ET.SubElement(world, 'geom', name='mission_'+name, type='box',
                pos=f'{x} {y} .002', size=f'{GOAL_HALF[0]} {GOAL_HALF[1]} .002',
                rgba='.8 .12 .7 .5', contype='0', conaffinity='0')
        return ET.tostring(root, encoding='unicode')
    return build


def evaluate_solo(samples, goal):
    """Output-only referee; invoked after all action generation ends."""
    if not samples:
        return {'success':False, 'reason':'no_samples'}
    target = SOLO_GOALS[goal]
    def inside(row):
        x,y,z = row['box_position']
        # Conservative footprint radius covers any horizontal box orientation.
        return (abs(x-target[0])+.027 <= GOAL_HALF[0]
                and abs(y-target[1])+.027 <= GOAL_HALF[1]
                and .012 <= z <= .021 and row['floor_contact']
                and not row['robot_contact'])
    tail = samples[-11:]
    lifted = max(row['box_position'][2] for row in samples)-SOLO_START[2]
    movement = math.dist(samples[0]['robot_position'][:2], samples[-1]['robot_position'][:2])
    carried = any(row['box_position'][0]-SOLO_START[0] > .20
                  and row['box_position'][2] > .04 for row in samples)
    stable = (len(tail)==11 and all(inside(s) for s in tail)
              and max(math.dist(s['box_position'],tail[-1]['box_position']) for s in tail)<.004)
    return {'success': bool(lifted>.035 and carried and stable and movement>.25),
            'max_lift_m':lifted, 'transported_while_lifted':carried,
            'released_stable_inside':stable, 'robot_displacement_m':movement,
            'final_box_position':samples[-1]['box_position'], 'selected_goal':goal}


class MissionScene(GoalScene):
    team = solo = solo_executor = solo_log = None

    def configure_world_setup(self):
        # The production constructor resets XML spawns to its own layout.
        # Apply this authored initial pose once, before folded setup and RGB.
        self.world.controllers['r2'].set_base_pose_for_test((0.,-3.,.032355118817659255),0.)

    def open(self, *args, **kwargs):
        from unittest.mock import patch
        import sim.multi_masterpi_production as production
        self.solo_rows, self.solo_samples, self.solo_raw = [], [], []
        self.all_weld_ticks = 0
        self.observer_frame_id = 0
        with patch.object(production, 'build_multi_robot_xml', mission_xml(production.build_multi_robot_xml)):
            super().open(*args, **kwargs)
        from sim.camera_robot_port import CameraRobotPort
        self.solo_port = CameraRobotPort(self.world, 'r2')
        import mujoco
        self.solo_body = mujoco.mj_name2id(self.world.model,mujoco.mjtObj.mjOBJ_BODY,'mission_cyan_box')
        self.solo_geom = mujoco.mj_name2id(self.world.model,mujoco.mjtObj.mjOBJ_GEOM,'mission_cyan_box_geom')
        # Presentation camera only: include all three robots and both task lanes.
        import numpy as np
        cid = mujoco.mj_name2id(self.world.model,mujoco.mjtObj.mjOBJ_CAMERA,'cctv_warehouse')
        position, target = np.array((.4,-4.5,2.6)), np.array((.65,-2.35,.05))
        forward = target-position; forward /= np.linalg.norm(forward)
        right = np.cross(forward,(0.,0.,1.)); right /= np.linalg.norm(right)
        up = np.cross(right,forward); quat=np.empty(4)
        mujoco.mju_mat2Quat(quat,np.column_stack((right,up,-forward)).ravel())
        self.world.model.cam_pos[cid],self.world.model.cam_quat[cid]=position,quat
        self.world.model.cam_fovy[cid]=55.
        mujoco.mj_forward(self.world.model,self.world.data)
        return self

    def team_capture(self, tag):
        frames = self.capture(tag)
        own = self.world.render_jpeg(robot_id='r2',camera='robot_cam',quality=95)
        self.observer_frame_id += 1
        frames['r2'] = {'own_bytes':own, 'top_bytes':frames['r1']['top_bytes'],
            'own_rgb':image_record(self.out/'rgb'/f'{tag}-r2-own.jpg',self.out,own),
            'shared_top_rgb':frames['r1']['shared_top_rgb'], 'frame_id':self.observer_frame_id}
        return frames

    def invariant_record(self):
        import mujoco
        row = super().invariant_record()
        cid=mujoco.mj_name2id(self.world.model,mujoco.mjtObj.mjOBJ_CAMERA,'r2__robot_cam')
        row['policy_cameras']['r2__robot_cam']={
            'position':self.world.model.cam_pos[cid].tolist(),
            'quaternion':self.world.model.cam_quat[cid].tolist(),
            'fov_y_deg':float(self.world.model.cam_fovy[cid])}
        return row

    def configure_run(self, args, report):
        self.report = report
        self.preference = args.solo_goal
        run_id = uuid.uuid4().hex[:12]
        self.team = ThreeRobotRuntime(self.out/'team',run_id=run_id,mode=args.team_planner,
            agreement=TeamAgreement(run_id,plan_validator=validate_mission),
            request_builder=partial(build_mission_request,preference=self.preference),
            reply_validator=validate_mission_reply,
            plan_fixture=mission_fixture('near_magenta' if self.preference=='auto' else self.preference))
        history={r:[] for r in ROBOTS}
        for row in self.trace:
            for rid,targets in row.get('command',{}).get('targets',{}).items():
                history[rid].append({'stage':row['stage'],'targets':targets,
                                    'duration_s':row['command']['duration_s']})
        for turn in range(6):
            if self.team.negotiate(self.team_capture(f'mission-plan-{turn}'),history,turn,self.time()):
                break
            self.tick(.2)
        else:
            raise RuntimeError('no unanimous executable three-robot mission')
        committed=self.team.agreement.committed
        chosen=committed['plan']['solo']['goal']
        if self.preference!='auto' and chosen!=self.preference:
            raise RuntimeError('plan violates operator destination requirement')
        self.solo = SoloBoxTransport(chosen)
        self.solo_log=(self.out/'solo-decisions.jsonl').open('w')
        self.solo_executor=VisualMacroExecutor(self.solo_port,log_callback=self.solo_raw.append)
        self.solo_started=None
        self.team.event('MISSION_COMMITTED',self.time(),plan=committed)
        if args.planner=='llm':
            self.gate=CameraSkillGate(self.out/'llm',team_plan=committed)
        report['config'].update(team_planner=args.team_planner,solo_goal_preference=self.preference)
        report['scope']='three RGB agents agree on two physical cargo tasks and a selected box destination; concurrent r1/r3 pair and r2 solo; skill-constrained roles, RGB local control, not arbitrary role/route planning'

    def _solo_tick(self):
        now=self.time()
        if self.solo_started is None:
            return
        if now-self.solo_started>240:
            raise RuntimeError('solo simulation budget exhausted')
        committed=self.team.agreement.committed
        if not committed or not self.team.agreement.authorize(committed['proposal_id'],committed['plan_hash']):
            self.solo_executor.cancel(now,'plan_revoked')
            raise RuntimeError('mission plan revoked')
        self.solo_executor.tick(now)
        if self.solo.done or not self.solo_executor.idle:
            return
        obs=self.solo_port.capture()
        own=base64.b64decode(obs['image'])
        # Existing monocular skill calibration is 640x480; preserve FOV and
        # image content by resizing the full image, never changing the camera.
        import cv2
        import numpy as np
        import hashlib
        native=cv2.imdecode(np.frombuffer(own,np.uint8),cv2.IMREAD_COLOR)
        own=cv2.imencode('.jpg',cv2.resize(native,(640,480)),[cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
        obs['image']=base64.b64encode(own).decode()
        obs['sha256']=hashlib.sha256(own).hexdigest()
        top=self.world.render_team_jpeg(camera='cctv_top',quality=95)
        index=len(self.solo_rows)
        refs={'own':image_record(self.out/'rgb'/f'solo-{index:04d}-own.jpg',self.out,own),
              'top':image_record(self.out/'rgb'/f'solo-{index:04d}-top.jpg',self.out,top)}
        before=self.solo.phase
        action,evidence=self.solo.decide(obs,top)
        row={'index':index,'sim_time_s':now,'phase_before':before,'phase_after':self.solo.phase,
             'observation':{k:v for k,v in obs.items() if k!='image'},'images':refs,
             'action':action,'top_evidence':evidence}
        self.solo_rows.append(row)
        self.solo_log.write(json.dumps(row)+'\n');self.solo_log.flush()
        self.solo_executor.submit(action,obs,self.solo.phase,now)
        if before!=self.solo.phase or index%25==0 or self.solo.done:
            print(json.dumps({'solo_step':index,'phase':self.solo.phase,'action':action}),flush=True)
        if self.solo.done:
            self.team.event('SOLO_STOPPED',now,reason=self.solo.reason)
            if self.solo.reason!='VISUAL_RELEASE_CONFIRMED':
                raise RuntimeError('solo stopped: '+str(self.solo.reason))

    def _step_with_referee(self,active,commands=None):
        if self.solo_executor is not None:
            self._solo_tick()
        super()._step_with_referee(active,commands)
        # Separate output-only truth path: never fed into the actors above.
        if hasattr(self,'solo_body') and (not self.solo_samples or self.time()-self.solo_samples[-1]['sim_time_s']>=.099):
            import mujoco
            others=[]
            for c in self.world.data.contact[:self.world.data.ncon]:
                a,b=int(c.geom1),int(c.geom2)
                if self.solo_geom in (a,b):others.append(b if a==self.solo_geom else a)
            row={'sim_time_s':self.time(),
                 'box_position':self.world.data.xpos[self.solo_body].tolist(),
                 'robot_position':self.world.controllers['r2'].base_xyz().tolist(),
                 'floor_contact':any(int(self.world.model.geom_bodyid[g])==0 for g in others),
                 'robot_contact':any((mujoco.mj_id2name(self.world.model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').startswith(('r1__','r2__','r3__')) for g in others)}
            self.solo_samples.append(row)
        self.all_weld_ticks += int(any(self.world.data.eq_active))

    def checkpoint(self,skill):
        if self.team:
            committed=self.team.agreement.committed
            if not committed or not self.team.agreement.authorize(committed['proposal_id'],committed['plan_hash']):
                raise RuntimeError('mission revoked')
        result=super().checkpoint(skill)
        if self.team:
            self.team.event('PAIR_AUTHORIZED',self.time(),skill=skill)
            if skill=='APPROACH':
                self.solo_started=self.time()
                self.team.event('SOLO_STARTED',self.time())
            if skill=='FINISH':
                self.team.event('PAIR_FINISHED',self.time())
                while not self.solo.done:
                    self.tick(.2)
                self.tick(1.2)
        return result

    def extra_report(self):
        goal=self.solo.goal if self.solo else 'near_magenta'
        evaluation=evaluate_solo(self.solo_samples,goal)
        return {'mission_team':self.team.snapshot() if self.team else None,
                'solo':{'goal':goal,'visual_reason':self.solo.reason if self.solo else None,
                        'decision_count':len(self.solo_rows),'evaluation':evaluation},
                'all_weld_active_ticks':self.all_weld_ticks,
                'pair_success':self.report['success'],
                'success':bool(self.report['success'] and evaluation['success']
                               and self.all_weld_ticks==0)}

    def close(self):
        if self.solo_executor:
            self.solo_executor.cancel(self.time(),'experiment_ended')
        if self.solo_log:self.solo_log.close()
        if self.team:self.team.close(self.time())
        if self.out.exists():
            write(self.out/'solo-raw-actions.json',self.solo_raw)
            write(self.out/'solo-evaluation-only.json',self.solo_samples)
        super().close()


def main():
    parser=build_parser()
    parser.description=__doc__
    parser.set_defaults(planner='llm')
    parser.add_argument('--team-planner',choices=('llm','fixture'),default='llm')
    parser.add_argument('--solo-goal',choices=('auto',*GOALS),default='auto')
    return run(parser.parse_args(),scene_factory=MissionScene)


if __name__=='__main__':raise SystemExit(main())
