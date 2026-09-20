#!/usr/bin/env python3
"""Matched rule/Gemini/Jev direct motor-choice experiment. SIM pauses during inference."""
from __future__ import annotations
import argparse
import copy
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from harness.jev_motion import (RGBObserver, action, at_goal, policy_state, rule, jev_request,
                               gemini_request, post, validate_jev, validate_gemini, GOAL)
from harness.camera_motion_identity import ImageMotionIdentity
from scripts.research_dispatch_scene import DispatchScene
from sim.research_dispatch_arena import episode

CASES = {'dev':[-.80,-2.65,0.], 'dev_yaw':[-.78,-2.70,8.], 'straight':[-.86,-2.65,0.],
         'left_offset':[-.84,-2.75,12.], 'right_offset':[-.76,-2.55,-12.]}

def model_request(state, policy, variant, model):
    if variant == 'numeric':
        return jev_request(state,model) if policy=='jev' else gemini_request(state,model)
    if variant != 'semantic': raise ValueError('unknown state variant')
    # Reuse exactly the successful archived-state condition, without explicit rule criteria.
    from scripts.probe_jev_motion_representation import request
    body=request(state,'semantic_state');body['model']=model
    if policy=='jev':return body
    result=gemini_request(body['state'],model)
    question=body['questions']['action']
    result['messages'][0]['content']=question['instructions']+'\nReturn only a JSON object with one key action and one of these choices: '+json.dumps(question['criteria'])
    return result

def write(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

class Scene(DispatchScene):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.video=None;self.samples=[];self.next_sample=0.;self.contacts=0;self.peer_contacts=0
    def sample(self):
        # Output-only evaluator. No sampled field reaches perception or policy.
        import mujoco
        import numpy as np
        w=self.world;r=w.controllers['r2'];p=list(map(float,r.base_xyz()))
        gid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'dispatch_box_geom')
        bid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_BODY,'r2__robot')
        yaw=math.atan2(w.data.xmat[bid].reshape(3,3)[1,0],w.data.xmat[bid].reshape(3,3)[0,0])
        delta=w.data.geom_xpos[gid,:2]-p[:2]
        bearing=(math.atan2(delta[1],delta[0])-yaw+math.pi)%(2*math.pi)-math.pi
        row={'sim_s':float(w.data.time),'position':p,'heading_deg':math.degrees(yaw),
             'range_m':float(np.linalg.norm(delta)),'bearing_deg':math.degrees(bearing),
             'box_position':w.data.geom_xpos[gid].tolist()}
        self.samples.append(row)
    def step(self,seconds):
        import mujoco
        w=self.world
        gid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'dispatch_box_geom')
        own={g for g in self.robot_ids if (mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').startswith('r2__')}
        for _ in range(round(seconds/w.model.opt.timestep)):
            super().step(float(w.model.opt.timestep))
            self.contacts+=any((int(c.geom1)==gid and int(c.geom2) in self.robot_ids)
                or (int(c.geom2)==gid and int(c.geom1) in self.robot_ids) for c in w.data.contact[:w.data.ncon] if c.dist<0)
            self.peer_contacts+=any((int(c.geom1) in own and int(c.geom2) in self.robot_ids-own) or (int(c.geom2) in own and int(c.geom1) in self.robot_ids-own) for c in w.data.contact[:w.data.ncon] if c.dist<0)
            if float(w.data.time)>=self.next_sample:
                self.sample();self.next_sample=float(w.data.time)+.05
            if self.video:self.video.capture()
    def execute(self,name):
        a=action(name);now=float(self.world.data.time)
        self.ports['r2'].apply_bounded(a,now,.2)
        self.command_history['r2'].append({'action':a,'issued_at_s':now})
        self.step(.25)
        self.ports['r2'].hold(float(self.world.data.time))
    def close(self):
        try:
            if self.video:self.video.close();self.video=None
        finally:super().close()


def trial(args, policy, case, key, source):
    from scripts.probe_dual_grasp_sync import Video
    config=episode('open',11)
    x,y,deg=getattr(args,'case_setup',CASES)[case]
    config['setup_only']['spawns']['r2']=[x,y,.032355118817659255,math.radians(deg)]
    # Other robots and beam retain their authored parked positions; only r2 acts.
    variant=getattr(args,'state_variant','numeric')
    out=args.output/getattr(args,'trial_id',policy+'-'+case)
    scene=Scene(config,out);started=time.perf_counter();rows=[];history=[]
    result={'source_sha':source,'policy':policy,'case':case,'state_variant':variant,'goal':GOAL,'success':False,
            'clock':'paused SIM during inference; not real-time control','scope':'single robot approach and alignment, no grasp or carry',
            'stop_reason':None,'error':None,'model_calls':0,'input_tokens':0,'output_tokens':0,'cost_usd':None}
    try:
        scene.open();write(out/'setup-only.json',config);write(out/'manifest.json',scene.manifest)
        scene.video=Video(scene.world,out/'motion.mp4',8);scene.video.stage=policy+' / '+case
        scene.sample()
        before=scene.capture('probe-before')['r2']
        for _ in range(6):scene.execute('forward')
        scene.step(.25)
        after=scene.capture('probe-after')['r2']
        identity=ImageMotionIdentity();identity.update(before['top_bytes'],None)
        anchor=identity.update(after['top_bytes'],{'kind':'drive','forward':.1,'turn':0.,'duration_s':.2})
        if not anchor['valid']:raise ValueError('own motion identity unresolved')
        import cv2
        import numpy as np
        frame=cv2.imdecode(np.frombuffer(after['top_bytes'],np.uint8),cv2.IMREAD_COLOR)
        observer=RGBObserver(config['static_map'],before['top_bytes'],after['top_bytes'],np.array(anchor['center'])*[frame.shape[1]-1,frame.shape[0]-1])
        write(out/'identity.json',{'claim':anchor,'probe':observer.probe_evidence})
        confirmations=0
        for turn in range(args.max_steps):
            if time.perf_counter()-started>args.max_wall_s:result['stop_reason']='wall_budget';break
            round_start=time.perf_counter()
            frames=scene.capture(f'{turn:03d}')['r2']
            observation=observer.observe(frames['own_bytes'],frames['top_bytes'])
            state=policy_state(observation,history)
            row={'turn':turn,'observed_at_sim_s':float(scene.world.data.time),
                 'images':{k:frames[k] for k in ('own_rgb','shared_top_rgb')},'observation':observation,'state':state}
            confirmations=confirmations+1 if at_goal(observation) else 0
            if at_goal(observation):
                name='stop';row['decision_source']='common_RGB_goal_confirmation'
            elif policy=='rule':
                name=rule(state);row['decision_source']='rule'
            else:
                # Reserve a bounded request before sending; never silently overshoot by a batch.
                if result['input_tokens']+5000>args.max_input_tokens:
                    result['stop_reason']='input_budget';break
                body=model_request(state,policy,variant,args.jev_model if policy=='jev' else args.gemini_model)
                row['request']=body
                write(out/f'{turn:03d}-request.json',body)
                response=post(body,'https://api.typesafe.ai/v1/systemone' if policy=='jev' else args.gemini_url,
                              key if policy=='jev' else None,args.timeout)
                write(out/f'{turn:03d}-response.json',response)
                row['response']=response;result['model_calls']+=1
                if response['status']!='ok':raise RuntimeError('model_'+response['status'])
                b=response['body'];u=b.get('usage',{})
                result['input_tokens']+=u.get('input_tokens',u.get('prompt_tokens',0))
                result['output_tokens']+=u.get('output_tokens',u.get('completion_tokens',0))
                result['model']=b.get('model')
                name=validate_jev(b) if policy=='jev' else validate_gemini(b)
                if policy=='jev':
                    a=b['answers']['action']
                    row['choice_probability_mismatch']=a['probabilities'][name] < max(a['probabilities'].values())-1e-6
                row['decision_source']=policy
            row['action']=name;row['observation_to_issue_wall_s']=time.perf_counter()-round_start
            scene.execute(name)
            history.append({'action':name,'range_m':observation['range_m'],'bearing_deg':observation['bearing_deg']})
            rows.append(row);write(out/'turns.json',rows)
            if turn%10==0:print(json.dumps({'policy':policy,'case':case,'turn':turn,'action':name,'range_m':observation['range_m'],'bearing_deg':observation['bearing_deg']}),flush=True)
            if confirmations>=3:result['stop_reason']='RGB_goal_confirmed';break
        else:result['stop_reason']='step_budget'
        scene.execute('stop');scene.step(.5);scene.sample()
    except Exception as exc:
        result['error']={'type':type(exc).__name__,'message':str(exc) if key is None else str(exc).replace(key,'[REDACTED]')}
        result['stop_reason']='error'
    finally:
        if scene.world:
            result['camera_geometry_unchanged']=scene.invariants()==scene.initial_invariants
            result['weld_steps']=scene.weld_steps
            result['cargo_contact_steps']=scene.contacts
            result['peer_contact_steps']=scene.peer_contacts
            result['obstacle_contact_steps']=scene.obstacle_contact_steps
            write(out/'referee-only.json',scene.samples)
            if scene.samples:
                tail=[s for s in scene.samples if s['sim_s']>=scene.samples[-1]['sim_s']-.4]
                physical=(tail[-1]['sim_s']-tail[0]['sim_s'] >= .35) and all(.26<=s['range_m']<=.30 and abs(s['bearing_deg'])<=6 for s in tail)
                result['final_evaluation']=scene.samples[-1]
                result['success']=bool(result['stop_reason']=='RGB_goal_confirmed' and physical and not scene.contacts
                    and not scene.obstacle_contact_steps and not scene.peer_contacts and not scene.weld_steps and result['camera_geometry_unchanged'])
            result['sim_s']=float(scene.world.data.time)
            result['commands']=len(scene.command_history['r2'])
        scene.close();result['wall_s']=time.perf_counter()-started;result['turns']=len(rows)
        if out.exists():write(out/'result.json',result)
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policies',nargs='+',choices=['rule','gemini','jev'],default=['rule'])
    p.add_argument('--cases',nargs='+',choices=list(CASES),default=['dev'])
    p.add_argument('--execute',action='store_true');p.add_argument('--prompt-key',action='store_true')
    p.add_argument('--jev-model',default='jev-1.13.0');p.add_argument('--gemini-model',default='gemini-3.8-flash')
    p.add_argument('--gemini-url',default=os.environ.get('GEMINI_PROXY_URL','http://127.0.0.1:8391/v1/chat/completions'))
    p.add_argument('--max-steps',type=int,default=70);p.add_argument('--max-input-tokens',type=int,default=180000)
    p.add_argument('--max-wall-s',type=float,default=360);p.add_argument('--timeout',type=float,default=30)
    args=p.parse_args()
    if not args.execute:p.error('--execute required for physical simulation and optional paid model calls')
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():p.error('commit experiment source first')
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if args.output.exists():p.error('output must be new')
    key=None
    if 'jev' in args.policies:
        key=getpass.getpass('Jev API key (hidden): ') if args.prompt_key else os.environ.get('TYPESAFE_API_KEY')
        if not key:p.error('Jev key missing')
    import mujoco, cv2
    args.output.mkdir(parents=True)
    write(args.output/'protocol.json',{'source_sha':source,'cases':args.cases,'policies':args.policies,
        'max_steps':args.max_steps,'max_input_tokens_per_episode':args.max_input_tokens,'max_wall_s_per_episode':args.max_wall_s,
        'case_setup':CASES,'goal':GOAL,'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__,'opencv':cv2.__version__},
        'jev_model':args.jev_model,'gemini_model':args.gemini_model,'request_timeout_s':args.timeout,
        'probe_commands':6,'probe_duration_each_s':.2,'action_duration_s':.2,
        'evaluation_goal':{'range_m':[.26,.30],'absolute_bearing_deg_max':6,'stable_tail_min_s':.35,'zero_contacts_and_weld':True},
        'no_model_confidence_actuation_threshold':True,'model_confidence_scope':'recorded only, not safety probability'})
    results=[]
    for case in args.cases:
        for policy in args.policies:
            results.append(trial(args,policy,case,key,source));write(args.output/'results.json',results)
    return 0
if __name__=='__main__':raise SystemExit(main())
