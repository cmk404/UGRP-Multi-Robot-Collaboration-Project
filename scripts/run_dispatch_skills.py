#!/usr/bin/env python3
"""Actual existing RGB skill integration for the three-peer dispatch mission.

Shared physics interleaves the solo macro executor while the pair executes its
existing staged approach/grasp code. Evaluation is a separate output sink.
"""
from __future__ import annotations
import base64
import copy
from functools import partial
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from harness.camera_motion_identity import ImageMotionIdentity
from harness.dispatch_plan import build_dispatch_request, validate_dispatch_plan, validate_dispatch_reply
from harness.dispatch_skill_binding import SkillBindings, ImageRoute
from harness.dispatch_feasibility import negotiate_executable
from harness.dispatch_yield import SoloYield,WheelObserver
from harness.three_robot_plan import ROBOTS, TeamAgreement, images
from harness.solo_box_transport import SoloBoxTransport
from harness.grasp_student_inference import predict_student
from harness.visual_macro_runtime import VisualMacroExecutor
from sim.research_dispatch_arena import episode, actor_task
from scripts.research_dispatch_scene import DispatchScene
from scripts.run_dispatch_e2e import Referee
from scripts.run_research_dispatch import opaque_run_id
from scripts.three_robot_runtime import ThreeRobotRuntime, write
from scripts.run_camera_approach_student import models, sha
from scripts.run_camera_varied_start_student import load_stage_models
from scripts.run_three_robot_mission import prepare_grasp_models
from scripts.dispatch_pair_skill import BoundPairSkill
from scripts.camera_approach_scene import image_record


class SkillScene(DispatchScene):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.video=self.referee=self.bindings=self.team=None
        self.solo=self.solo_executor=None;self.solo_lease=0.
        self.solo_rows=[];self.solo_raw=[];self.next_sample=0.;self.pair_phase='SETUP'
        self.yield_rows=[];self.yield_policy=None;self.yield_folded=False
        self.identity=None
        self.original_step=None;self.deadline=None;self.last_frames=None
        self.efficient_capture=False

    def time(self):return float(self.world.data.time)
    def open(self):
        super().open()
        self.original_step=self.world._physics_step_for
        self.world._physics_step_for=self._physics
        return self
    def _physics(self,active,commands=None):
        self._solo_tick()
        now=self.time()
        for p in self.ports.values():p.tick(now)
        self.original_step(active,commands)
        self.physics_steps+=1;self.weld_steps+=bool(self.world.data.eq_active.any())
        self.obstacle_contact_steps+=any(
            (int(c.geom1) in self.robot_ids and int(c.geom2) in self.obstacle_ids)
            or (int(c.geom2) in self.robot_ids and int(c.geom1) in self.obstacle_ids)
            for c in self.world.data.contact[:self.world.data.ncon] if c.dist<0)
        if self.video:
            self.video.stage='PAIR '+self.pair_phase+' | '+(self.bindings.solo+' '+self.solo.phase if self.solo else 'SETUP/END')
            self.video.capture()
        if self.referee and self.time()+1e-9>=self.next_sample:
            self.referee.sample();self.next_sample=self.time()+.1
    def step(self,seconds):
        for _ in range(round(seconds/self.world.model.opt.timestep)):
            if self.deadline and time.monotonic()>self.deadline:raise RuntimeError('skill wall budget exhausted')
            self.world._physics_step_for(self.world.controllers['r1'])
    def hold(self):
        for p in self.ports.values():p.hold(self.time())
    def capture(self,label,*,own_robots=None,overview=True):
        if not self.efficient_capture:own_robots,overview=None,True
        self.last_frames=super().capture(label,own_robots=own_robots,overview=overview)
        return self.last_frames
    def authorize(self):
        if self.bindings:self.bindings.authorize(self.team.agreement.committed)
    def raw(self,rid,action,stage):
        self.authorize();self.ports[rid].validate_action(action)
        self.ports[rid].apply(action,self.time())
        self.command_history[rid].append({'stage':stage,'action':copy.deepcopy(action),
            'issued_at_s':self.time(),'plan_hash':self.bindings.committed['plan_hash'] if self.bindings else None})
    def pair_drive(self,commands,duration,stage):
        self.authorize();self.pair_phase=stage
        if set(commands)!=set(self.bindings.pair.values()):raise ValueError('pair endpoint mismatch')
        for r,a in commands.items():self.ports[r].validate_action(a)
        if stage=='TRANSIT' and any(any(abs(a.get(k,0.))>0 for k in ('forward','left','turn')) for a in commands.values()):
            self.bindings.note_transit_command('beam')
        for r,a in commands.items():self.raw(r,a,stage)
        self.step(duration)
    def pair_arm(self,targets,duration,settle,stage):
        self.authorize();self.pair_phase=stage
        if not set(targets)<=set(self.bindings.pair.values()):raise ValueError('pair arm endpoint mismatch')
        if stage.startswith('grasp'):self.bindings.note_grasp_command('beam')
        for r,p in targets.items():
            self.command_history[r].append({'stage':stage,'issued_servo_targets':p,
                'duration_s':duration,'settle_s':settle,'issued_at_s':self.time(),
                'plan_hash':self.bindings.committed['plan_hash']})
        # Reuse the successful baseline interpolation and shared-physics clock.
        # This helper reads its issued-command cache, not measured joints.
        self.world._team_joint_move_servos(targets,duration,settle_s=settle)
    def start_solo(self):
        self.solo=SoloBoxTransport(robot_id=self.bindings.solo,navigator=ImageRoute(self.bindings,'box'),attachment_min_saturation=150,release_refine_ground_fit=True)
        self.solo_executor=VisualMacroExecutor(self.ports[self.bindings.solo],
            log_callback=self.solo_raw.append,drive_settle_by_phase={'carry':0.})
        self.solo_started=None
    def other_robot_observation(self,top):
        if self.yield_policy:
            return self.yield_policy.vision.observe(top),{'source':'solo_yield_history'}
        frame=cv2.imdecode(np.frombuffer(top,np.uint8),cv2.IMREAD_COLOR)
        claim=self.identity[self.bindings.solo]['claim']
        if not claim['valid']:raise RuntimeError('waiting robot identity unresolved')
        hint=np.array(claim['center'])*[frame.shape[1]-1,frame.shape[0]-1]
        return WheelObserver(self.bindings.static_map,hint).observe(top),{'source':'own_identity_probe','hint_px':hint.tolist()}
    def _yield_tick(self,now):
        if self.bindings.tasks['beam']['id'] in self.bindings.finished:
            self.bindings.finish('box');return
        obs=self.ports[self.bindings.solo].capture()
        own=base64.b64decode(obs['image'])
        top=self.world.render_team_jpeg(camera='cctv_top',quality=95)
        index=len(self.yield_rows)
        refs={'own':image_record(self.out/'rgb'/f'yield-{index}-own.jpg',self.out,own),
              'top':image_record(self.out/'rgb'/f'yield-{index}-top.jpg',self.out,top)}
        if not self.yield_folded:
            action={'kind':'pose','pulses':{1:2000,3:740,4:2320,5:1320,6:1500}}
            evidence={'phase':'fold_open_arm_after_visual_release'}
            self.solo_executor.submit(action,obs,'yield_fold',now);self.yield_folded=True
        else:
            if self.yield_policy is None:
                self.yield_policy=SoloYield(self.bindings.static_map,self.solo.navigator.box_center)
            action,evidence=self.yield_policy.decide(top)
            evidence['initial_cargo_center_px']=self.solo.navigator.box_center.tolist()
            self.raw(self.bindings.solo,action,'YIELD');self.solo_lease=now+action['duration_s']
            if self.yield_policy.done:self.bindings.finish('box')
        self.yield_rows.append({'index':index,'sim_time_s':now,'robot_id':self.bindings.solo,
            'images':refs,'observation':{k:v for k,v in obs.items() if k!='image'},
            'action':action,'evidence':evidence,'box_job_finished':self.bindings.tasks['box']['id'] in self.bindings.finished})
        if index%25==0 or self.yield_policy and self.yield_policy.done:
            print(json.dumps({'solo_yield_step':index,'evidence':evidence}),flush=True)
    def _solo_tick(self):
        if self.solo is None:return
        self.authorize();now=self.time()
        self.solo_executor.tick(now)
        if self.bindings.tasks['box']['id'] in self.bindings.finished:return
        if not self.solo_executor.idle or now<self.solo_lease:return
        if self.solo.done:
            self._yield_tick(now);return
        stage='APPROACH' if self.solo.phase=='approach' else 'TRANSIT' if self.solo.phase=='carry' else 'GRASP'
        if not self.bindings.permission('box',stage):return
        if self.solo_started is None:self.solo_started=now
        if now-self.solo_started>300:raise RuntimeError('solo SIM budget exhausted')
        obs=self.ports[self.bindings.solo].capture()
        native=base64.b64decode(obs['image'])
        decoded=cv2.imdecode(np.frombuffer(native,np.uint8),cv2.IMREAD_COLOR)
        own=cv2.imencode('.jpg',cv2.resize(decoded,(640,480)),[cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
        obs['image']=base64.b64encode(own).decode();obs['sha256']=hashlib.sha256(own).hexdigest()
        top=self.world.render_team_jpeg(camera='cctv_top',quality=95)
        index=len(self.solo_rows)
        refs={'own':image_record(self.out/'rgb'/f'solo-{index}-own.jpg',self.out,own),
              'top':image_record(self.out/'rgb'/f'solo-{index}-top.jpg',self.out,top)}
        before=self.solo.phase
        approach_state=copy.deepcopy(self.solo.box) if before=='approach' else None
        action,evidence=self.solo.decide(obs,top)
        if evidence.get('waiting_for_resource'):self.solo.steps-=1
        if before=='approach' and self.solo.phase=='lower' and not self.bindings.permission('box','GRASP'):
            # Stay at the pregrasp visual boundary and reobserve after waiting.
            # Never hold a lifted cargo merely to queue for the apron.
            self.solo.box=approach_state
            self.solo.steps-=1  # Resource waiting is not an executed skill decision.
            action={'kind':'wait','duration':.3}
            evidence={**evidence,'waiting_before_grasp':True}
        if action['kind']=='mecanum' and not self.bindings.permission('box','TRANSIT'):
            action={'kind':'wait','duration':.1}
            self.solo.steps-=1
            evidence={**evidence,'waiting_for_resource':True}
        self.solo_rows.append({'index':index,'sim_time_s':now,'robot_id':self.bindings.solo,
            'phase_before':before,'phase_after':self.solo.phase,'observation':{k:v for k,v in obs.items() if k!='image'},
            'images':refs,'action':action,'top_evidence':evidence,
            'own_attachment_evidence':copy.deepcopy(self.solo.box.last_attachment)})
        if self.video:self.video.stage='PAIR '+self.pair_phase+' | '+self.bindings.solo+' '+self.solo.phase
        if action['kind']=='mecanum':
            self.raw(self.bindings.solo,action,'TRANSIT');self.solo_lease=now+action['duration_s']
        else:self.solo_executor.submit(action,obs,self.solo.phase,now)
        if index%25==0 or before!=self.solo.phase:
            print(json.dumps({'solo_step':index,'robot':self.bindings.solo,'phase':self.solo.phase,'action':action}),flush=True)
        if self.solo.done:
            if self.solo.reason!='VISUAL_RELEASE_CONFIRMED':raise RuntimeError('solo stopped: '+str(self.solo.reason))
            if self.bindings.tasks['beam']['id'] in self.bindings.finished:self.bindings.finish('box')
    def close(self):
        if self.world:
            if self.solo_executor:self.solo_executor.cancel(self.time(),'trial_end')
            if self.original_step:self.world._physics_step_for=self.original_step
        super().close()


def grasp_transfer_options(skill):
    if skill.get('rgb_support_scope') in {'full_calibrated_views', 'local_grasp_top_v1'}:
        return {'background_band': False, 'top_roi': None}
    return {'background_band': skill.get('task_domain') != 'dispatch_open_v1',
            'top_roi': [12,6,20,19] if skill.get('task_domain') == 'dispatch_open_v1' else None}


def run(args):
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit and freeze source before a trial')
    import mujoco
    from scripts.probe_dual_grasp_sync import Video
    source_skill=json.loads((args.grasp_model_dir/'student-skill.json').read_text())
    grasp_root=prepare_grasp_models(args.grasp_model_dir,args.output.with_name(args.output.name+'-grasp-models'),
        **grasp_transfer_options(source_skill)).resolve()
    skill,grasp=models(grasp_root,'student-skill.json')
    stage_skill,stages=load_stage_models(args.stage_model_dir)
    reference=args.reference_top.read_bytes()
    config=episode(args.variant,args.seed)
    config['contact_solver_profile']=args.contact_profile
    import math
    dx,dy,yaw=getattr(args,'spawn_offset',[0.,0.,0.])
    if max(abs(dx),abs(dy))>.03 or abs(yaw)>3:raise ValueError('bounded spawn perturbation')
    for pose in config['setup_only']['spawns'].values():
        pose[0]+=dx;pose[1]+=dy;pose[3]+=math.radians(yaw)
    scene=SkillScene(config,args.output)
    scene.efficient_capture=getattr(args,'efficient_capture',False)
    started=time.monotonic();pair=team=None
    result={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'scope':'three LLM peers choose allocation; actual saved approach/grasp models and existing box skill execute',
        'config':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        'model_provenance':{'grasp_manifest_sha256':sha(grasp_root/'student-skill.json'),
            'grasp_models':skill['models'],'stage_manifest_sha256':sha(args.stage_model_dir/'varied-start-skill.json'),
            'stage_models':stage_skill['models'],'reference_sha256':sha(args.reference_top)},
        'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__},
        'plan_committed':False,'protocol_complete':False,'physical_success':False,'error':None,
        'phase':'SETUP','cost_usd':None,
        'input_boundary':'own fixed RGB + common fixed TOP RGB + static authored map + own issued commands + peer claims; referee output only'}
    try:
        scene.open();scene.deadline=started+args.max_wall_s
        write(args.output/'episode-setup-only.json',scene.config)
        write(args.output/'scene-manifest.json',scene.manifest)
        scene.video=Video(scene.world,args.output/'execution.mp4',getattr(args,'video_fps',10))
        scene.referee=Referee(scene);scene.referee.sample()
        identity={}
        for rid in ROBOTS:
            scene.command_history[rid].append({'stage':'SETUP','issued_servo_targets':{1:2000,3:740,4:2320,5:1320,6:1500},
                'meaning':'issued commands, not measured state'})
            scene.hold();scene.step(.4);before=scene.capture('identity-'+rid+'-before')[rid]
            probe={'kind':'drive','forward':.10,'turn':0.,'duration_s':.6}
            scene.raw(rid,probe,'IDENTIFY');scene.step(.9);scene.hold()
            after=scene.capture('identity-'+rid+'-after')[rid]
            tracker=ImageMotionIdentity();tracker.update(before['top_bytes'],None)
            evidence=[]
            for label,f in [('BEFORE',before),('AFTER',after)]:
                evidence += [{'label':label+'_'+i['label'],'image':i['image']} for i in images(f['own_bytes'],f['top_bytes'])]
            identity[rid]={'claim':tracker.update(after['top_bytes'],probe),'images':evidence}
        write(args.output/'identity-evidence.json',identity)
        scene.identity=identity
        task=actor_task(scene.config['static_map'],required_dock=getattr(args,'required_dock',None))
        task['capability_scope']='RGB pair approach/grasp plus loaded rotation and complete-footprint path checking; existing VisualBoxSkill. Parallel envelope 0.99m; rotated envelope 0.45m. Loaded terrain is unvalidated and avoided. In clutter, pickup preparation is exclusive. Waiting cargo and robots remain occupied space. If you select beam.after=[box_job] and box.after=[], the box executor releases its cargo and visually clears the unloading bay before finishing box_job. Role binding, routes and task dependencies follow your plan. All skills remain experimental; no raw-action fallback.'
        write(args.output/'actor-mission.json',task)
        run_id=opaque_run_id()
        def planner(rid,**kwargs):
            return build_dispatch_request(rid,task=task,execution_pilot=True,identity_evidence=identity[rid],**kwargs)
        replay_plan=None
        if args.plan_replay:
            saved=json.loads(args.plan_replay.read_text())
            SkillBindings(saved,scene.config['static_map'])
            replay_plan=saved['plan']
            result['scope']='recorded-plan diagnostic with fixture votes; existing RGB physical skills, not fresh LLM E2E'
            result['plan_replay_sha256']=sha(args.plan_replay)
            if getattr(args,'live_replan',False):
                result['scope']='fixture-initialized recovery diagnostic; actual LLM re-negotiation after capability rejection, not fresh initial LLM E2E'
        team=ThreeRobotRuntime(args.output/'team',run_id=run_id,mode='fixture' if replay_plan else 'llm',
            plan_fixture=replay_plan,
            agreement=TeamAgreement(run_id,plan_validator=partial(validate_dispatch_plan,
                required_dock=getattr(args,'required_dock',None))),request_builder=planner,
            reply_validator=validate_dispatch_reply,request_timeout=args.timeout,max_tokens=1600,
            roles_fixed_by_skill=False,planning_only=False,max_wall_s=args.max_wall_s)
        scene.team=team;frames=scene.capture('planning')
        result['phase']='NEGOTIATE'
        result['plan_feasibility']=negotiate_executable(team,frames,scene.command_history,task,
            scene.config['static_map'],scene.time(),max_tokens=args.max_input_tokens,
            live_replan=getattr(args,'live_replan',False),identity=identity,reference_top=reference)
        scene.bindings=SkillBindings(team.agreement.committed,scene.config['static_map'],
                                    route_overlap=getattr(args,'route_overlap',False),
                                    overlap_start=getattr(args,'overlap_start','transit'))
        result.update(plan_committed=True,plan=scene.bindings.plan,bindings=scene.bindings.capabilities())
        write(args.output/'committed-plan.json',team.agreement.committed)
        write(args.output/'robot-programs.json',scene.bindings.programs)
        write(args.output/'skill-bindings.json',scene.bindings.capabilities())
        # Preserve the accepted plan. Do not silently replace its route or move
        # a loaded formation through a known insufficient static clearance.
        scene.bindings.check_route()
        scene.start_solo()
        pair=BoundPairSkill(scene,scene.bindings,skill,grasp,stages,grasp_root,reference,identity)
        while not scene.bindings.permission('beam','APPROACH'):scene.step(.2)
        result['phase']='APPROACH';result['pair_approach']=pair.approach()
        while not scene.bindings.permission('beam','GRASP'):scene.step(.2)
        result['phase']='GRASP';result['pair_grasp']=pair.finish_grasp(predict_student,grasp)
        pair.grasp_report.pop('evaluation',None)
        pair.grasp_report['evaluation_source']='separate referee-only.jsonl after control ends'
        while not scene.bindings.permission('beam','TRANSIT'):scene.step(.2)
        result['phase']='TRANSIT'
        carry_steps=getattr(args,'carry_max_steps',None)
        if getattr(args,'carry_act_model',None):
            from scripts.dispatch_act_carry import carry
            result['carry_policy']='ACT own RGB + raw top RGB + static task + own last issued motion'
            result['carry_model_sha256']=sha(args.carry_act_model/'model.safetensors')
            carry(pair,args.carry_act_python,args.carry_act_model,
                  args.carry_act_max_steps if carry_steps is None else carry_steps)
        else:pair.carry(ImageRoute(scene.bindings,'beam'),max_steps=carry_steps)
        if scene.bindings.route_overlap and not scene.bindings.permission('beam','UNLOAD'):
            raise RuntimeError('shared unload resource unavailable')
        result['phase']='RELEASE';pair.place();pair.verify_placement();scene.bindings.finish('beam')
        while scene.bindings.tasks['box']['id'] not in scene.bindings.finished:scene.step(.2)
        result['protocol_complete']=True;result['phase']='FINISHED'
    except (Exception,KeyboardInterrupt) as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
        if args.output.exists():(args.output/'exception.txt').write_text(traceback.format_exc())
    finally:
        try:
            if scene.world:
                # Freeze decisions before final referee-only settling.
                if scene.solo_executor:scene.solo_executor.cancel(scene.time(),'trial_end')
                if scene.solo:result['solo_status']={'phase':scene.solo.phase,'reason':scene.solo.reason,'done':scene.solo.done}
                if scene.bindings:result['resource_events']=scene.bindings.resource_events
                scene.solo=None;scene.deadline=None;scene.hold()
                try:
                    scene.step(1.3);scene.capture('final')
                except Exception as exc:
                    result['cleanup_error']=str(exc)
                result['camera_geometry_unchanged']=scene.initial_invariants==scene.invariants()
                result['obstacle_contact_steps']=scene.obstacle_contact_steps
                if scene.referee:
                    result['evaluation']=scene.referee.finish(result.get('plan'))
                    result['physical_success']=result['evaluation']['physical_success']
                    write(args.output/'evaluation-only.json',result['evaluation'])
                write(args.output/'issued-commands.json',scene.command_history)
                write(args.output/'solo-decisions.json',scene.solo_rows);write(args.output/'solo-raw-actions.json',scene.solo_raw)
                write(args.output/'solo-yield.json',scene.yield_rows)
                if pair:
                    write(args.output/'pair-decisions.json',pair.calls)
                    write(args.output/'pair-grasp.json',pair.grasp_report)
                    write(args.output/'pair-replay.json',pair.trace)
        finally:
            for cleanup in (lambda:team.close(scene.time() if scene.world else 0.) if team else None,
                            lambda:scene.video.close() if scene.video else None,scene.close):
                try:cleanup()
                except Exception as exc:result.setdefault('cleanup_errors',[]).append(str(exc))
        result['wall_s']=time.monotonic()-started
        if team:
            result['protocol_calls']=len(team.calls)
            result['llm_calls']=sum(c.get('model')!='scripted-fixture-not-llm' for c in team.calls)
            result['usage']={k:sum((c.get('usage') or {}).get(k,0) for c in team.calls)
                for k in ('prompt_tokens','completion_tokens','total_tokens')}
        args.output.mkdir(parents=True,exist_ok=True);write(args.output/'result.json',result)
    print(json.dumps({k:result.get(k) for k in ('phase','error','plan_committed','protocol_complete','physical_success','wall_s')}),flush=True)
    return 0 if result['physical_success'] and result['protocol_complete'] else 1
