#!/usr/bin/env python3
"""RGB short-skill comparison; scene/referee are isolated from controller state."""
from __future__ import annotations
import argparse
import copy
import concurrent.futures
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.jev_motion import post
from harness.jev_skill_motion import (SkillObserver,SkillController,request,parse_answer,
    reference_choice,bounded_command,request_compatible,DT)
from harness.camera_motion_identity import ImageMotionIdentity
from scripts.run_jev_motion import Scene,write,physical_goal_confirmed
from sim.research_dispatch_arena import episode,digest

DEVELOPMENT={
 'dev_open':{'pose':[-.81,-2.65,0],'target':[-.06,-2.65]},
 'dev_left':{'pose':[-.79,-2.70,26],'target':[-.06,-2.65]},
 'dev_right':{'pose':[-.79,-2.58,-26],'target':[-.06,-2.65]},
 'dev_blocked':{'pose':[-.83,-2.65,0],'target':[.10,-2.65],'barrier':[-.42,-2.65,.06,.11],'known':True},
 'dev_unknown':{'pose':[-.83,-2.65,0],'target':[.10,-2.65],'barrier':[-.42,-2.67,.06,.10]},
 'dev_occlusion':{'pose':[-.81,-2.65,0],'target':[.04,-2.65],'occlusion':[6.,8.]},
}
# Qualification v1 was demoted after a common RGB association defect.
QUALIFICATION={
 'open01':{'pose':[-.85,-2.66,4],'target':[-.02,-2.65]},
 'open02':{'pose':[-.78,-2.61,-4],'target':[.03,-2.63]},
 'yaw01':{'pose':[-.84,-2.71,31],'target':[.02,-2.65]},
 'yaw02':{'pose':[-.82,-2.59,-31],'target':[.02,-2.65]},
 'known01':{'pose':[-.85,-2.64,3],'target':[.18,-2.65],'barrier':[-.39,-2.66,.065,.10],'known':True},
 'known02':{'pose':[-.82,-2.68,-3],'target':[.23,-2.66],'barrier':[-.34,-2.65,.075,.10],'known':True},
 'unknown01':{'pose':[-.85,-2.63,5],'target':[.16,-2.64],'barrier':[-.38,-2.65,.06,.12]},
 'unknown02':{'pose':[-.82,-2.68,-5],'target':[.22,-2.66],'barrier':[-.32,-2.66,.08,.09]},
 'occluded01':{'pose':[-.84,-2.68,7],'target':[.05,-2.65],'occlusion':[7.,9.5]},
 'occluded02':{'pose':[-.80,-2.60,-7],'target':[.11,-2.64],'occlusion':[8.,11.]},
 'temporary01':{'pose':[-.85,-2.65,0],'target':[.19,-2.65],'barrier':[-.35,-2.65,.06,.10],'barrier_window':[5.,12.]},
 'temporary02':{'pose':[-.81,-2.63,0],'target':[.25,-2.65],'barrier':[-.30,-2.63,.07,.11],'barrier_window':[6.,14.]},
}


DEVELOPMENT.update({f'qualification_{name}':copy.deepcopy(QUALIFICATION[name])
                    for name in ('known02','temporary02')})
# Replacement evaluation was drawn with seed 210923 before repairing the
# qualification defects. The literal layouts stay frozen for the whole cohort.
HOLDOUT={'open03': {'pose': [-0.8316, -2.6513, 6.182], 'target': [-0.0195, -2.6431]},
 'open04': {'pose': [-0.7821, -2.6155, -3.767], 'target': [0.0158, -2.6227]},
 'yaw03': {'pose': [-0.835, -2.6945, 32.782], 'target': [0.0137, -2.6359]},
 'yaw04': {'pose': [-0.8275, -2.5731, -31.329], 'target': [-0.0026, -2.6383]},
 'known03': {'pose': [-0.8474, -2.6559, 2.191],
             'target': [0.1794, -2.6462],
             'barrier': [-0.3906, -2.6562, 0.0645, 0.1029],
             'known': True},
 'known04': {'pose': [-0.8351, -2.6885, -0.546],
             'target': [0.2083, -2.672],
             'barrier': [-0.3617, -2.662, 0.0716, 0.101],
             'known': True},
 'unknown03': {'pose': [-0.8492, -2.6113, 6.01],
               'target': [0.1448, -2.6304],
               'barrier': [-0.3952, -2.6404, 0.0586, 0.1203]},
 'unknown04': {'pose': [-0.812, -2.6977, -7.479],
               'target': [0.2375, -2.6597],
               'barrier': [-0.3025, -2.6597, 0.0811, 0.0867]},
 'occluded03': {'pose': [-0.85, -2.6768, 5.028], 'target': [0.0717, -2.6407], 'occlusion': [6.915, 9.528]},
 'occluded04': {'pose': [-0.7915, -2.5979, -4.778],
                'target': [0.1062, -2.6368],
                'occlusion': [7.806, 10.955]},
 'temporary03': {'pose': [-0.85, -2.6369, -1.724],
                 'target': [0.2077, -2.6385],
                 'barrier': [-0.3323, -2.6385, 0.061, 0.1023],
                 'barrier_window': [4.767, 11.817]},
 'temporary04': {'pose': [-0.797, -2.6383, 1.029],
                 'target': [0.2483, -2.6417],
                 'barrier': [-0.3017, -2.6217, 0.0663, 0.1076],
                 'barrier_window': [5.933, 14.062]}}

def configuration(case):
    c=episode('open',11)
    x,y,heading=case['pose']
    c['setup_only']['spawns']['r2']=[x,y,.032355118817659255,math.radians(heading)]
    c['setup_only']['cargo']['box']=[*case['target'],.016]
    if 'barrier' in case:
        x,y,hx,hy=case['barrier']
        obstacle={'id':'motion_barrier','center_m':[x,y],'half_extents_m':[hx,hy],'height_m':.16,'kind':'barrier'}
        if case.get('known'):c['static_map']['obstacles'].append(obstacle)
        else:c['setup_only']['unexpected_obstacles'].append(obstacle)
    c['static_map_sha256']=digest(c['static_map'])
    return c


class ChallengeScene(Scene):
    def __init__(self,config,out,case):
        super().__init__(config,out)
        self.case=case;self.events=[];self.event_geoms={};self.last_events={}

    def open(self):
        import scripts.research_dispatch_scene as dispatch
        original=dispatch.build_scene_xml
        def build(source,config):
            xml,manifest=original(source,config)
            if 'occlusion' in self.case:
                tree=ET.fromstring(xml);world=tree.find('worldbody')
                x,y=self.case['target']
                ET.SubElement(world,'geom',name='motion_target_cover',type='box',
                    pos=f'{x} {y} .12',size='.065 .065 .006',rgba='.23 .28 .33 1',
                    contype='0',conaffinity='0',group='0')
                xml=ET.tostring(tree,encoding='unicode')
                manifest['scene_xml_sha256']=hashlib.sha256(xml.encode()).hexdigest()
            return xml,manifest
        with patch.object(dispatch,'build_scene_xml',build):super().open()
        import mujoco
        w=self.world
        for name,window in [('motion_target_cover',self.case.get('occlusion')),
                            ('dispatch_motion_barrier',self.case.get('barrier_window'))]:
            if window:
                gid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,name)
                self.event_geoms[name]=(gid,window,w.model.geom_pos[gid].copy(),int(w.model.geom_contype[gid]),int(w.model.geom_conaffinity[gid]))
        self.update_environment()


        return self

    def update_environment(self):
        # Authored environment clock, never sent to a controller/model request.
        import mujoco
        w=self.world
        for name,(gid,window,position,contype,affinity) in self.event_geoms.items():
            active=window[0]<=float(w.data.time)<window[1]
            if self.last_events.get(name) is active:continue
            w.model.geom_pos[gid]=position
            if not active:w.model.geom_pos[gid,2]=-2
            w.model.geom_rgba[gid,3]=1 if active else 0
            w.model.geom_contype[gid]=contype if active else 0
            w.model.geom_conaffinity[gid]=affinity if active else 0
            self.last_events[name]=active
            self.events.append({'sim_s':float(w.data.time),'fixture':name,'active':active})
            mujoco.mj_forward(w.model,w.data)

    def issue(self,command):
        now=float(self.world.data.time)
        self.ports['r2'].apply_bounded(command,now,.2)
        self.command_history['r2'].append({'action':command,'issued_at_s':now})
        self.step(DT)
        self.ports['r2'].hold(float(self.world.data.time))
        self.update_environment()


def continuous_disposition(control,latest,state,now,observed_at,deadline):
    """Local RGB completion wins over an unused delayed model decision."""
    if latest is not None and control.done:return 'complete'
    if now+DT>deadline+1e-9:return 'budget'
    fresh=control.state(latest[1]) if latest else None
    if fresh is None or now-observed_at>4. or not request_compatible(state,fresh):return 'discard'
    return 'execute'


def post_continuous(scene,observer,control,body,url,key,rows,label,sim_deadline,initial=None,transport=None):
    """Keep physics and RGB alive while one network request is outstanding.

    An expired decision is braked during inference; this is a continuous-clock
    stress test, not a claim of uninterrupted high-rate autonomous driving.
    """
    # A response can finish before the first physics step. In that case the
    # original RGB is still current; an invalid later frame must erase it.
    latest=initial
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
        pending=pool.submit(transport or post,body,url,key,30)
        i=0
        while (not pending.done() and not control.done
               and float(scene.world.data.time)+DT<=sim_deadline+1e-9):
            start=time.monotonic()
            scene.issue(bounded_command())
            frames=scene.capture(f'{label}-wait-{i:03}')['r2']
            row={'tick':label,'wait_index':i,'observed_at_sim_s':float(scene.world.data.time),
                 'images':{k:frames[k] for k in ('own_rgb','shared_top_rgb')},
                 'command':bounded_command(),'decision_source':'inference_brake','execution_source':'inference_brake'}
            try:
                o=observer.observe(frames['own_bytes'],frames['top_bytes'])
                control.done=control.gate.update(o)
                latest=(frames,o,float(scene.world.data.time))
                row['observation']=o
            except ValueError as exc:
                row['observation']={'valid':False,'reason':str(exc)}
                observer.recent.clear();control.gate.count=0;control.done=False;latest=None
            rows.append(row);write(scene.out/'turns.json',rows)
            i+=1
            time.sleep(max(0.,DT-(time.monotonic()-start)))
        response=pending.result()
    return response,latest

def run_trial(args,case,policy,key,source):
    import cv2
    import numpy as np
    import mujoco
    from harness.model_mailbox import MailboxTransport
    transport=MailboxTransport(args.model_mailbox) if getattr(args,"model_mailbox",None) else post
    from scripts.probe_dual_grasp_sync import Video
    config=configuration(case);out=Path(args.output)
    scene=ChallengeScene(config,out,case)
    rows=[];started=time.monotonic()
    result={'source_sha':source,'policy':policy,'arm':args.arm,'case':args.case,'success':False,
        'stop_reason':None,'error':None,'model_calls':0,'input_tokens':0,'output_tokens':0,'cost_usd':None,
        'transport':'colab_private_mailbox' if getattr(args,'model_mailbox',None) else 'direct',
        'clock':getattr(args,'clock','paused'),'scope':'RGB approach, route selection and recovery; no grasp/carry',
        'interventions':{},'limits':{'sim_s':args.max_sim_s,'calls':args.max_calls,'wall_s':args.max_wall_s,'input_tokens':args.max_input_tokens}}
    try:
        scene.open();write(out/'setup-only.json',config);write(out/'case-setup-only.json',case)
        write(out/'manifest.json',scene.manifest)
        write(out/'environment.json',{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__,'opencv':cv2.__version__})
        scene.video=Video(scene.world,out/'motion.mp4',8);scene.video.stage=policy+' / '+args.case+' / '+args.arm
        before=scene.capture('probe-before')['r2']
        for _ in range(6):scene.execute('forward')
        scene.step(.25);after=scene.capture('probe-after')['r2']
        identity=ImageMotionIdentity();identity.update(before['top_bytes'],None)
        anchor=identity.update(after['top_bytes'],{'kind':'drive','forward':.1,'turn':0.,'duration_s':.2})
        if not anchor['valid']:raise ValueError('own motion identity unresolved')
        observer=SkillObserver(config['static_map'],before['top_bytes'],after['top_bytes'],np.array(anchor['center'])*[959,719])
        write(out/'identity.json',{'claim':anchor,'probe':observer.probe_evidence})
        control=SkillController(copy.deepcopy(config['static_map']),always_query=args.arm=='always',primitive=args.arm=='primitive')
        start_sim=float(scene.world.data.time);lost=0
        for tick in range(math.ceil(args.max_sim_s/DT)):
            if time.monotonic()-started>args.max_wall_s:result['stop_reason']='wall_budget';break
            if float(scene.world.data.time)+DT>start_sim+args.max_sim_s+1e-9:result['stop_reason']='sim_budget';break
            frames=scene.capture(f'{tick:03}')['r2']
            row={'tick':tick,'observed_at_sim_s':float(scene.world.data.time),
                 'images':{k:frames[k] for k in ('own_rgb','shared_top_rgb')}}
            try:
                o=observer.observe(frames['own_bytes'],frames['top_bytes']);lost=0
            except ValueError as exc:
                lost+=1;control.active=None;control.path=[];control.gate.count=0
                observer.recent.clear()
                row.update(observation={'valid':False,'reason':str(exc)},command=bounded_command(),decision_source='RGB_visibility_hold')
                rows.append(row);write(out/'turns.json',rows);scene.issue(row['command'])
                if lost>=24:result['stop_reason']='vision_recovery_exhausted';break
                continue
            row['observation']=o;control.observe_progress(o)
            state=control.state(o);row['state']=state
            if control.done:
                row.update(command=bounded_command(),decision_source='RGB_completion')
                rows.append(row);write(out/'turns.json',rows);scene.issue(row['command'])
                result['stop_reason']='RGB_goal_confirmed';break
            inside=.27<=o['range_m']<=.28 and abs(o['bearing_deg'])<=3
            reason=None if inside else control.need_query(state)
            if reason:
                row['query_reason']=reason
                if policy=='rule':
                    chosen=reference_choice(state);confidence=None;row['decision_source']='rule'
                else:
                    if result['model_calls']>=args.max_calls:result['stop_reason']='call_budget';break
                    if result['input_tokens']+6000>args.max_input_tokens:result['stop_reason']='input_budget';break
                    body=request(state,policy,decomposed=args.arm!='single')
                    row['request']=body;write(out/f'{tick:03}-request.json',body)
                    url='https://api.typesafe.ai/v1/systemone' if policy=='jev' else args.gemini_url
                    origin=row
                    if getattr(args,'clock','paused')=='continuous':
                        origin['execution_source']='request_only';rows.append(origin)
                        response,latest=post_continuous(scene,observer,control,body,url,key if policy=='jev' else None,
                            rows,f'{tick:03}',start_sim+args.max_sim_s,initial=(frames,o,origin['observed_at_sim_s']),transport=transport)
                    else:
                        response=transport(body,url,key if policy=='jev' else None,30)
                        latest=None
                    row['response']=response;write(out/f'{tick:03}-response.json',response)
                    result['model_calls']+=1
                    b=response.get('body') or {};usage=b.get('usage',{}) if isinstance(b,dict) else {}
                    result['input_tokens']+=usage.get('input_tokens',usage.get('prompt_tokens',0))
                    result['output_tokens']+=usage.get('output_tokens',usage.get('completion_tokens',0))
                    if getattr(args,'clock','paused')=='continuous':
                        now=float(scene.world.data.time)
                        origin['response_received_sim_s']=now
                        disposition=continuous_disposition(control,latest,state,now,origin['observed_at_sim_s'],start_sim+args.max_sim_s)
                        if disposition in ('complete','budget'):
                            origin['response_unused_reason']='RGB_completion' if disposition=='complete' else 'sim_budget'
                            if disposition=='complete':
                                frames,o,observed=latest
                                rows.append({'tick':tick,'observed_at_sim_s':observed,
                                    'images':{k:frames[k] for k in ('own_rgb','shared_top_rgb')},
                                    'observation':o,'state':control.state(o),'command':bounded_command(),
                                    'execution_source':'RGB_completion','request_origin_tick':origin['tick'],
                                    'response_age_sim_s':now-origin['observed_at_sim_s']})
                            write(out/'turns.json',rows)
                            result['stop_reason']='RGB_goal_confirmed' if disposition=='complete' else 'sim_budget'
                            break
                    if response['status']!='ok':
                        if not any(row is saved for saved in rows):rows.append(row)
                        raise RuntimeError('model_'+response['status'])
                    answers,confidence=parse_answer(b,policy,state,decomposed=args.arm!='single')
                    row['answers']=answers;row['confidence']=confidence;chosen=answers['action']
                    row['decision_source']=policy
                    if getattr(args,'clock','paused')=='continuous':
                        fresh=control.state(latest[1]) if latest else None
                        age=float(scene.world.data.time)-origin['observed_at_sim_s']
                        origin['response_received_sim_s']=float(scene.world.data.time)
                        if disposition=='discard':
                            origin['discard_reason']='missing_changed_or_expired_RGB_context'
                            control.active=None;control.path=[]
                            write(out/'turns.json',rows)
                            continue
                        frames,o,observed=latest;state=fresh
                        row={'tick':tick,'observed_at_sim_s':observed,'images':{k:frames[k] for k in ('own_rgb','shared_top_rgb')},
                             'observation':o,'state':state,'answers':answers,'confidence':confidence,
                             'decision_source':policy,'request_origin_tick':origin['tick'],'response_age_sim_s':age}
                    if answers.get('evidence')=='observe_again' and args.arm!='primitive':
                        chosen='hold_and_observe';row['decision_source']='model_requested_reobserve'
                control.select(chosen,o,confidence if args.arm!='no_confidence' else None)
                if policy!='rule' and answers.get('progress')=='reconsider':control.cautious=True
                row['chosen_skill']=chosen
            command,execution=control.command(o)
            row['command']=command;row['execution_source']=execution;row['active_skill']=control.active
            result['interventions'][execution]=result['interventions'].get(execution,0)+1
            rows.append(row);write(out/'turns.json',rows);scene.issue(command)
            if tick%25==0:print(json.dumps({'case':args.case,'policy':policy,'tick':tick,'calls':result['model_calls'],'range':o['range_m'],'angle':o['bearing_deg'],'skill':control.active,'execution':execution}),flush=True)
        else:result['stop_reason']='sim_budget'
        scene.issue(bounded_command());scene.step(.5);scene.sample()
    except KeyboardInterrupt:
        result['stop_reason']='diagnostic_interrupted'
        raise
    except Exception as exc:
        if 'row' in locals() and not any(row is saved for saved in rows):rows.append(row)
        result['error']={'type':type(exc).__name__,'message':str(exc) if not key else str(exc).replace(key,'[REDACTED]')}
        result['stop_reason']='error'
    finally:
        if scene.world:
            result.update(camera_geometry_unchanged=scene.invariants()==scene.initial_invariants,weld_steps=scene.weld_steps,
                cargo_contact_steps=scene.contacts,peer_contact_steps=scene.peer_contacts,obstacle_contact_steps=scene.obstacle_contact_steps,
                sim_s=float(scene.world.data.time),commands=len(scene.command_history['r2']))
            write(out/'referee-only.json',scene.samples);write(out/'environment-events-only.json',scene.events)
            if scene.samples:
                result['final_evaluation']=scene.samples[-1]
                result['success']=bool(result['stop_reason']=='RGB_goal_confirmed' and physical_goal_confirmed(scene.samples)
                    and not(scene.contacts or scene.peer_contacts or scene.obstacle_contact_steps or scene.weld_steps)
                    and result['camera_geometry_unchanged'])
            write(out/'turns.json',rows)
        scene.close();result['wall_s']=time.monotonic()-started;result['ticks']=len(rows)
        if out.exists():write(out/'result.json',result)
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--case',choices=list(DEVELOPMENT)+list(HOLDOUT),default='dev_open')
    p.add_argument('--policy',choices=['rule','jev','gemini'],default='rule')
    p.add_argument('--arm',choices=['full','always','primitive','single','no_confidence'],default='full')
    p.add_argument('--clock',choices=['paused','continuous'],default='paused')
    p.add_argument('--execute',action='store_true');p.add_argument('--stdin-key',action='store_true')
    p.add_argument('--max-sim-s',type=float,default=90);p.add_argument('--max-wall-s',type=float,default=600)
    p.add_argument('--max-calls',type=int,default=100);p.add_argument('--max-input-tokens',type=int,default=240000)
    p.add_argument('--gemini-url',default='http://127.0.0.1:8391/v1/chat/completions')
    args=p.parse_args()
    if not args.execute:p.error('--execute required')
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():p.error('commit source before experiments')
    if args.output.exists():p.error('output must be new')
    key=None
    if args.policy=='jev':
        if args.stdin_key:key=json.load(sys.stdin)['key']
        else:
            import getpass
            key=getpass.getpass('Jev API key (hidden): ')
        if not key:p.error('Jev key missing')
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    result=run_trial(args,{**DEVELOPMENT,**HOLDOUT}[args.case],args.policy,key,source)
    return int(result['error'] is not None)

if __name__=='__main__':raise SystemExit(main())
