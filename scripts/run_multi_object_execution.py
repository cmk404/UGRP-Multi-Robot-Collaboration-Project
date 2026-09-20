#!/usr/bin/env python3
"""Bounded live RGB pilot: unanimous multi-task plan -> identity -> local stages.

SIM pauses during inference. Privileged cargo positions are read only after
control ends for a separate evaluator; they never reach a producer or gate.
"""
from __future__ import annotations

import argparse
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

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from harness.camera_motion_identity import ImageMotionIdentity
from harness.dispatch_execution import build_request,validate_reply
from harness.multi_object_execution import MultiObjectExecution
from harness.multi_object_plan import validate_plan
from harness.three_robot_plan import ROBOTS,TeamAgreement,images,validate_plan_reply
from scripts.multi_object_scene_preview import MultiObjectScene
from scripts.three_robot_runtime import ThreeRobotRuntime,write
from sim.camera_robot_port import CameraRobotPort
from sim.multi_object_scene import configuration,static_task
from sim.multi_object_suite import load_pilot


class ExecutionScene(MultiObjectScene):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.execution=self.video=None

    def open(self):
        super().open()
        self.ports={r:CameraRobotPort(self.world,r,allow_reverse=True,allow_mecanum=True) for r in ROBOTS}
        self.execution=None
        self.video=None
        return self

    def step(self,seconds):
        w=self.world
        for _ in range(round(seconds/w.model.opt.timestep)):
            now=float(w.data.time)
            if self.execution:self.execution.tick(now)
            for p in self.ports.values():p.tick(now)
            w._physics_step_for(w.controllers['r1'])
            self.physics_steps+=1;self.weld_steps+=bool(w.data.eq_active.any())
            if self.video:self.video.capture()

    def time(self):return float(self.world.data.time)

    def hold(self):
        for p in self.ports.values():p.hold(self.time())


def plan_request(rid,*,task,identity,request_id,own_rgb,top_rgb,agreement,inbox,own_history):
    prompt='''Choose and unanimously acknowledge a multi-object transport plan from RGB.
Your dynamic observations are only OWN and fixed TOP RGB, your issued commands,
and received peer claims. Authored static map and destinations are allowed.
Object IDs are grounded ONCE from the initial TOP: within each kind, columns from
left to right, then bottom to top inside a column (x separation <= .025 image width).
Do not re-sort after motion. Same-looking cargo must retain its initial ID.
Your robot ID is not a screen location; use your own issued probe and its RGB evidence.
Plan schema: {"schema":"ugrp.multi_object_plan.v1","mission_sha256":MISSION_HASH,
"tasks":[{"task_id":TASK_ID,"participants":{"r1":"end_a","r3":"end_b"},
"route_id":"lane_a","after":[DEPENDENCY_IDS]}]}.
Include every mission task exactly once. Boxes need one participant with role solo;
beams need two distinct robots, end_a at the upper TOP endpoint and end_b lower.
Pick robots yourself. Keep all required dependencies; extra ordering is permitted.
lane_a/lane_b are logical resource reservations on this open-map pilot; they do
not establish a collision-free motor path. Inspect all cargo, peers and obstacles.
Copy a pending proposal exactly to accept. Before one exists, propose a complete
plan or reject with plan null. All three peers must ACK the same proposal.
Return only {request_id,proposal_id,plan_hash,accept,plan,reason,message}.
Copy proposal_id/plan_hash from pending proposal; both null before proposal.
This is an experimental raw-action executor, not a demonstrated grasp skill.'''
    context={'robot_id':rid,'request_id':request_id,'authored_task':task,'agreement':agreement,
             'inbox':inbox,'own_issued_commands':own_history,'own_motion_identity':identity[rid]['claim']}
    return {'request_id':request_id,'messages':[{'role':'system','content':prompt},
            {'role':'user','content':json.dumps(context)}],
            'images':images(own_rgb,top_rgb)+identity[rid]['images']}


def evaluate(scene):
    """Call after decisions are disabled. Output only, independent of claims."""
    import mujoco
    import numpy as np
    w=scene.world; result={}
    for oid,item in scene.config['setup_only']['objects'].items():
        task=next(t for t in scene.config['mission']['tasks'] if t['object_id']==oid and
                  scene.config['static_map']['destinations'][t['destination_id']]['kind']=='final')
        slot=scene.config['static_map']['destinations'][task['destination_id']]
        bid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_BODY,item['body_name'])
        gid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,item['body_name']+'_geom')
        center=w.data.xpos[bid];half=np.abs(w.data.geom_xmat[gid].reshape(3,3))@w.model.geom_size[gid]
        inside=bool(np.all(np.abs(center[:2]-slot['center_m'])+half[:2]<=np.array(slot['half_extents_m'])+.003))
        floor=abs(center[2]-half[2])<.006
        touching_robot=any((int(c.geom1)==gid and 'r'==str(mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,int(c.geom2)) or '')[:1]) or
                           (int(c.geom2)==gid and 'r'==str(mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,int(c.geom1)) or '')[:1])
                           for c in w.data.contact[:w.data.ncon])
        result[oid]={'center_m':center.tolist(),'inside_final_slot':inside,'on_floor':bool(floor),
                     'touching_robot':touching_robot,'final_placement':bool(inside and floor and not touching_robot)}
    return {'objects':result,'final_objects':sum(v['final_placement'] for v in result.values()),
            'whole_mission_final_placement':all(v['final_placement'] for v in result.values()),
            'scope':'terminal placement only; required intermediate jobs need independent trajectory evidence'}


def run(args):
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise ValueError('commit and freeze source first')
    if args.output.exists():raise ValueError('new output directory required')
    import mujoco
    from scripts.probe_dual_grasp_sync import Video
    _,cases=load_pilot();case=next(c for c in cases if c['id']==args.case)
    config=configuration(case);task=static_task(config)
    scene=ExecutionScene(config,args.output/'scene');team=gate=None;started=time.monotonic()
    result={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'case':args.case,'error':None,'stop_reason':None,'transport_success':False,
            'clock':'paused SIM during inference; not real-time control','model':args.model,
            'limits':{'turns':args.turns,'calls':args.max_calls,'input_tokens':args.max_input_tokens,'wall_s':args.max_wall_s},
            'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__},
            'cost_usd':None,'scope':'experimental RGB raw-action execution, not ACT or teacher replay'}
    try:
        scene.open();initial_invariants=scene.invariants()
        write(args.output/'setup-evaluation-only.json',config);write(args.output/'actor-static-task.json',task)
        scene.video=Video(scene.world,args.output/'execution.mp4',8)
        identity={}
        for rid in ROBOTS:
            before=scene.capture('probe-'+rid+'-before')[rid]
            probe={'kind':'drive','forward':.1,'turn':0.,'duration_s':.2}
            for _ in range(3):scene.ports[rid].apply_bounded(probe,scene.time(),.2);scene.step(.2)
            scene.hold();scene.step(.3)
            after=scene.capture('probe-'+rid+'-after')[rid]
            tracker=ImageMotionIdentity();tracker.update(before['top_bytes'],None)
            claim=tracker.update(after['top_bytes'],dict(probe,duration_s=.6))
            identity[rid]={'claim':claim,'images':[
                {'label':prefix+'_'+v['label'],'image':v['image']}
                for prefix,f in [('OWN_PROBE_BEFORE',before),('OWN_PROBE_AFTER',after)]
                for v in images(f['own_bytes'],f['top_bytes'])]}
            scene.command_history[rid]=[{'stage':'SETUP','issued_servo_targets':{1:2000,3:740,4:2320,5:1320,6:1500},
                                         'meaning':'issued commands only'},
                                        {'stage':'IDENTIFY','action':probe,'repetitions':3,'duration_s':.2}]
        write(args.output/'motion-identity.json',identity)
        if not all(x['claim']['valid'] for x in identity.values()):
            raise RuntimeError('own robot identity unresolved')
        mission=task['mission'];validator=partial(validate_plan,mission=mission)
        team=ThreeRobotRuntime(args.output/'team',run_id='multi-'+str(time.time_ns()),
            agreement=TeamAgreement('multi-'+str(time.time_ns()),plan_validator=validator),
            request_builder=partial(plan_request,task=task,identity=identity),
            reply_validator=partial(validate_plan_reply,plan_validator=validator),
            request_timeout=args.timeout,max_tokens=1800,roles_fixed_by_skill=False,max_wall_s=args.max_wall_s)
        for client in team.clients.values():client.model_name=args.model
        def budget(extra):
            return (len(team.calls)+extra<=args.max_calls and time.monotonic()-started<args.max_wall_s and
                    sum((c.get('usage') or {}).get('prompt_tokens',0) for c in team.calls)<args.max_input_tokens)
        frames=scene.capture('planning')
        for turn in range(6):
            if not budget(3):break
            if team.negotiate(frames,scene.command_history,turn,scene.time()):break
        if not team.agreement.committed:raise RuntimeError('no unanimous executable plan within budget')
        write(args.output/'committed-plan.json',team.agreement.committed)
        gate=MultiObjectExecution(task,team.agreement,scene.ports,args.output/'runtime',now_s=scene.time())
        gate.history=copy.deepcopy(scene.command_history);scene.execution=gate
        previous={r:None for r in ROBOTS};quiet=0
        for turn in range(args.turns):
            scene.hold();frames=scene.capture(f'execution-{turn:03}')
            requests=gate.observe(frames,now_s=scene.time());futures={}
            if not budget(len(requests)):result['stop_reason']='model_budget';break
            for rid,req in requests.items():
                tid=req['task_id'];row=gate.protocol.tasks[tid]
                program={**row,'stage':req['stage'],**gate._meta(tid)}
                model_request=build_request(rid,request_id=req['request_id'],committed=team.agreement.committed,
                    program=program,task=task,frame=frames[rid],previous=previous[rid],own_history=gate.history[rid],
                    inbox=team.inbox[rid],identity=identity[rid]['claim'],feedback={'target_rgb_binding':req['target'],
                    'visual_inventory':req['visual_inventory'],'last_issued_command':req['last_command']})
                model_request['messages'][0]['content']+='\nMULTI-OBJECT OVERRIDE: the current object_id is bound to target_rgb_binding, an RGB estimate. Other same-color cargo is NOT your target. Follow this exact target continuously. Reobserve or BLOCKED on ambiguity. All drive/mecanum commands are exactly .2 seconds. Goal is the destination_id of this task in the authored static map. Keep task dependencies. Do not declare completion from issued commands. No learned grasp or ACT model is being executed.'
                futures[rid]=team.pool.submit(team._invoke,rid,model_request,
                    partial(validate_reply,plan_hash=req['plan_hash'],stage=req['stage'],robot_id=rid))
            batch={r:f.result() for r,f in futures.items()};replies={r:v[0] for r,v in batch.items()}
            for reply,stop,records in batch.values():team.calls.extend(records)
            gate.batch(replies,now_s=scene.time());team.save()
            previous=frames
            row={'turn':turn,'at_s':scene.time(),'tasks':{tid:j['phase'] if j['phase']=='APPROACH' else j['stage'].sync.stage for tid,j in gate.active.items()},
                 'replies':replies,'protocol':gate.protocol.summary()}
            write(args.output/f'turn-{turn:03}.json',row)
            print(json.dumps({'turn':turn,'tasks':row['tasks'],'statuses':{r:v['status'] if v else None for r,v in replies.items()},'calls':len(team.calls)}),flush=True)
            if len(gate.protocol.completed)==len(gate.protocol.tasks):result['stop_reason']='all_jobs_claimed';break
            quiet=quiet+1 if not replies or all(not v or v['status'] in ('UNCERTAIN','BLOCKED') for v in replies.values()) else 0
            if quiet>=3:result['stop_reason']='unresolved_visual_evidence';break
            scene.step(.2)
        else:result['stop_reason']='turn_budget'
    except (Exception,KeyboardInterrupt) as e:
        result['error']=f'{type(e).__name__}: {e}'
        write(args.output/'exception.json',{'traceback':traceback.format_exc()})
    finally:
        try:
            if gate:
                result['protocol']=gate.protocol.summary();result['issued_commands']=sum(len(h) for h in gate.history.values())
                gate.close(now_s=scene.time());scene.execution=None
            if scene.world:
                scene.hold();scene.step(1.3);scene.capture('final')
                result['evaluation']=evaluate(scene)
                result['camera_geometry_unchanged']=initial_invariants==scene.invariants()
                result['weld_steps']=scene.weld_steps
                write(args.output/'evaluation-only.json',result['evaluation'])
        finally:
            try:
                if team:team.close(scene.time() if scene.world else 0.)
            finally:
                try:
                    if scene.video:scene.video.close()
                finally:scene.close()
        if team:
            result['model_calls']=len(team.calls)
            result['usage']={k:sum((c.get('usage') or {}).get(k,0) for c in team.calls) for k in ('prompt_tokens','completion_tokens','total_tokens')}
        result['wall_s']=time.monotonic()-started
        write(args.output/'result.json',result)
        hashes={str(p.relative_to(args.output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.rglob('*') if p.is_file()}
        write(args.output/'artifact-hashes.json',hashes)
    print(json.dumps(result),flush=True)
    return 0 if result['stop_reason']=='all_jobs_claimed' else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--case',choices=['single_box','single_beam','staging_three'],default='staging_three')
    p.add_argument('--model',required=True)
    p.add_argument('--turns',type=int,default=60)
    p.add_argument('--max-calls',type=int,default=60)
    p.add_argument('--max-input-tokens',type=int,default=160000)
    p.add_argument('--max-wall-s',type=float,default=480.)
    p.add_argument('--timeout',type=float,default=30.)
    sys.exit(run(p.parse_args()))
