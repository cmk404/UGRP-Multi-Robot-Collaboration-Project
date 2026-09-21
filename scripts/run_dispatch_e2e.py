#!/usr/bin/env python3
"""Three live RGB peers: identity -> common plan -> bounded physical dispatch.

Default: actual saved RGB skills. --executor raw retains the earlier diagnostic. Physics
pauses during parallel LLM requests. Output-only referee never gates commands.
"""
from __future__ import annotations

import argparse
import copy
from functools import partial
import itertools
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))

from harness.camera_motion_identity import ImageMotionIdentity
from harness.dispatch_plan import build_dispatch_request, validate_dispatch_plan, validate_dispatch_reply
from harness.dispatch_execution import DispatchExecution, build_request, validate_reply
from harness.three_robot_plan import ROBOTS, TeamAgreement, images
from sim.research_dispatch_arena import episode, actor_task, VARIANTS
from scripts.research_dispatch_scene import DispatchScene
from scripts.run_research_dispatch import opaque_run_id
from scripts.three_robot_runtime import ThreeRobotRuntime, write


class Referee:
    """Output sink. The controller never receives samples or intermediate verdicts."""
    def __init__(self,scene):
        import mujoco
        self.scene = scene
        self.samples = []
        self.file = (scene.out/'referee-only.jsonl').open('w')
        self.geoms = {obj:mujoco.mj_name2id(scene.world.model,mujoco.mjtObj.mjOBJ_GEOM,name)
            for obj,name in [('beam','team_beam_geom'),('box','dispatch_box_geom')]}
        self.floor = mujoco.mj_name2id(scene.world.model,mujoco.mjtObj.mjOBJ_GEOM,'floor')
        if min(self.geoms.values()) < 0:raise ValueError('missing cargo geom')

    def sample(self):
        import numpy as np
        w = self.scene.world
        row = {'sim_time_s':float(w.data.time),'robots':self.scene.evaluate_positions(),
               'weld':bool(w.data.eq_active.any()),'cargo':{}}
        for obj,g in self.geoms.items():
            corners = np.array(list(itertools.product((-1,1),repeat=3))) * w.model.geom_size[g]
            corners = corners @ w.data.geom_xmat[g].reshape(3,3).T + w.data.geom_xpos[g]
            contact = set()
            for c in w.data.contact[:w.data.ncon]:
                if c.dist <= .001:
                    if int(c.geom1)==g:contact.add(int(c.geom2))
                    if int(c.geom2)==g:contact.add(int(c.geom1))
            row['cargo'][obj] = {'position':w.data.geom_xpos[g].tolist(),
                'corners':corners.tolist(),'robot_contact':bool(contact & self.scene.robot_ids),
                'floor_contact':self.floor in contact}
        self.samples.append(row)
        self.file.write(json.dumps(row)+'\n');self.file.flush()

    def finish(self,plan):
        self.file.close()
        if not self.samples:return {'physical_success':False,'reason':'no_samples'}
        import numpy as np
        samples = self.samples
        end = samples[-1]['sim_time_s']
        tail = [s for s in samples if s['sim_time_s'] >= end-1.05]
        stable_window = len(tail)>1 and tail[-1]['sim_time_s']-tail[0]['sim_time_s']>=.99
        from harness.dispatch_evaluation import carry_clearance
        clearance = carry_clearance(samples, self.scene.command_history, plan)
        results = {}
        for obj in self.geoms:
            initial = np.array(samples[0]['cargo'][obj]['position'])
            slot = self.scene.config['static_map']['docks'][plan['dock']]['slots'][obj] if plan else None
            def inside(s):
                if not slot:return False
                points = np.array(s['cargo'][obj]['corners'])[:,:2]
                return bool(np.all(np.abs(points-np.array(slot['center_m'])) <= np.array(slot['half_extents_m'])))
            positions = np.array([s['cargo'][obj]['position'] for s in tail])
            lifted = any(s['cargo'][obj]['position'][2]-initial[2]>.015 and not s['cargo'][obj]['floor_contact'] for s in samples)
            retained = stable_window and np.max(np.linalg.norm(positions-positions[-1],axis=1)) < .005
            r = {'max_lift_m':max(s['cargo'][obj]['position'][2]-initial[2] for s in samples),
                 'displacement_m':float(np.linalg.norm(np.array(samples[-1]['cargo'][obj]['position'])[:2]-initial[:2])),
                 'lift_observed_by_referee':lifted,
                 'inside_slot_at_end':inside(samples[-1]),
                 'released_supported_stable':bool(retained and all(inside(s) and s['cargo'][obj]['floor_contact']
                     and not s['cargo'][obj]['robot_contact'] for s in tail))}
            r['carry_clearance'] = clearance.get(obj, {'sampled_continuous_clearance': False})
            r['physical_success'] = bool(lifted and r['released_supported_stable']
                and r['carry_clearance']['sampled_continuous_clearance']
                and not any(s['weld'] for s in samples))
            results[obj] = r
        distance = {r:sum(np.linalg.norm(np.array(b['robots'][r])[:2]-np.array(a['robots'][r])[:2])
                         for a,b in zip(samples,samples[1:])) for r in ROBOTS}
        concurrent = sum(b['sim_time_s']-a['sim_time_s'] for a,b in zip(samples,samples[1:])
            if all(np.linalg.norm(np.array(b['robots'][r])[:2]-np.array(a['robots'][r])[:2]) > .001 for r in ROBOTS))
        return {'physical_success':all(r['physical_success'] for r in results.values()),'cargo':results,
                'robot_path_length_m':distance,'all_three_body_motion_sim_s':concurrent,
                'weld_steps':self.scene.weld_steps,'samples':len(samples),
                'stable_window_s':tail[-1]['sim_time_s']-tail[0]['sim_time_s']}


class TrialScene(DispatchScene):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.video = None;self.referee = None;self.expiry = {};self.next_sample = 0.

    def hold(self):
        for p in self.ports.values():p.hold(float(self.world.data.time))
        self.expiry.clear()

    def issue(self,rid,command):
        now = float(self.world.data.time)
        self.ports[rid].apply(command['action'],now)
        self.expiry[rid] = now+command['duration_s']
        self.command_history[rid].append(copy.deepcopy(command))

    def step(self,seconds):
        w = self.world;end = float(w.data.time)+seconds
        while float(w.data.time)+1e-9 < end:
            now = float(w.data.time)
            for rid,deadline in list(self.expiry.items()):
                if now+1e-9 >= deadline:
                    self.ports[rid].hold(now);self.expiry.pop(rid)
            super().step(float(w.model.opt.timestep))
            if self.video:self.video.capture()
            if self.referee and float(w.data.time)+1e-9 >= self.next_sample:
                self.referee.sample();self.next_sample = float(w.data.time)+.1


def run(args):
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit and freeze source before a trial')
    import mujoco
    from scripts.probe_dual_grasp_sync import Video
    scene = TrialScene(episode(args.variant,args.seed),args.output)
    team = gate = None;identity = {};previous = {r:None for r in ROBOTS}
    started = time.monotonic()
    result = {'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'scope':'three live RGB agents and raw-action physical execution pilot',
        'config':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},'error':None,'stop_reason':None,
        'plan_committed':False,'protocol_complete':False,'turns':[],
        'physical_success':False,'cost_usd':None,
        'clock':'SIM pauses during parallel inference; independent job gates, not real-time distributed control',
        'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__}}
    try:
        scene.open()
        write(args.output/'episode-setup-only.json',scene.config)
        write(args.output/'scene-manifest.json',scene.manifest)
        task = actor_task(scene.config['static_map'],required_dock=args.required_dock)
        write(args.output/'actor-mission.json',task)
        scene.video = Video(scene.world,args.output/'execution.mp4',10)
        scene.referee = Referee(scene);scene.referee.sample()
        scene.video.stage = 'IDENTIFY | issued probe, not cargo work'
        for rid in ROBOTS:
            scene.command_history[rid].append({'command_id':rid+'-setup','stage':'SETUP',
                'issued_servo_targets':{'1':2000,'3':740,'4':2320,'5':1320,'6':1500},
                'meaning':'fixed issued setup commands, not measured state'})
            scene.hold();scene.step(.4)
            before = scene.capture('identity-'+rid+'-before')[rid]
            probe = {'kind':'drive','forward':.10,'turn':0.,'duration_s':.6}
            scene.issue(rid,{'command_id':rid+'-identify','stage':'IDENTIFY','action':probe,
                'duration_s':.6,'issued_at_s':float(scene.world.data.time)})
            scene.step(.9);scene.hold()
            after = scene.capture('identity-'+rid+'-after')[rid]
            motion = ImageMotionIdentity();motion.update(before['top_bytes'],None)
            claim = motion.update(after['top_bytes'],probe)
            evidence = []
            for label,frame in [('OWN_PROBE_BEFORE',before),('OWN_PROBE_AFTER',after)]:
                evidence += [{'label':label+'_'+i['label'],'image':i['image']}
                             for i in images(frame['own_bytes'],frame['top_bytes'])]
            identity[rid] = {'claim':claim,'images':evidence}
        write(args.output/'identity-evidence.json',identity)
        run_id = opaque_run_id()
        def planner(rid,**kwargs):
            return build_dispatch_request(rid,task=task,execution_pilot=True,
                identity_evidence=identity[rid],**kwargs)
        team = ThreeRobotRuntime(args.output/'team',run_id=run_id,mode='llm',
            agreement=TeamAgreement(run_id,plan_validator=partial(validate_dispatch_plan,required_dock=args.required_dock)),
            request_builder=planner,reply_validator=validate_dispatch_reply,
            request_timeout=args.timeout,max_tokens=1600,roles_fixed_by_skill=False,
            planning_only=False,max_wall_s=args.max_wall_s)
        frames = scene.capture('planning')
        scene.video.stage = 'NEGOTIATE | all three peers'
        for turn in range(8):
            if team.negotiate(frames,scene.command_history,turn,float(scene.world.data.time)):break
        if not team.agreement.committed:raise RuntimeError('no valid unanimous dispatch plan')
        committed = team.agreement.committed
        write(args.output/'committed-plan.json',committed)
        gate = DispatchExecution(committed,scene.config['static_map'])
        write(args.output/'robot-programs.json',gate.programs)
        result['plan_committed'] = True;result['plan'] = committed['plan']
        quiet = 0
        for turn in range(args.rounds):
            if time.monotonic()-started >= args.max_wall_s:
                result['stop_reason'] = 'wall_budget';break
            tokens = sum((c.get('usage') or {}).get('prompt_tokens',0) for c in team.calls)
            if tokens >= args.max_input_tokens:result['stop_reason'] = 'input_token_budget';break
            scene.hold();frames = scene.capture(f'execute-{turn:03d}')
            futures = {};rows = {r:gate.current(r) for r in ROBOTS}
            for rid,row in rows.items():
                if not row:continue
                request_id = f'{run_id}-{rid}-execute-{turn}'
                req = build_request(rid,request_id=request_id,committed=committed,
                    program=row,task=task,frame=frames[rid],previous=previous[rid],
                    own_history=scene.command_history[rid],inbox=team.inbox[rid],
                    identity=identity[rid]['claim'],feedback=gate.feedback[rid])
                futures[rid] = team.pool.submit(team._invoke,rid,req,
                    partial(validate_reply,plan_hash=committed['plan_hash'],stage=row['stage'],robot_id=rid))
            batch = {r:f.result() for r,f in futures.items()}
            replies = {r:v[0] for r,v in batch.items()}
            for reply,stop,records in batch.values():team.calls.extend(records)
            for rid,reply in replies.items():
                if reply and reply['message']:
                    for peer in ROBOTS:
                        if peer!=rid:team.inbox[peer].append({'from_robot':rid,'message':reply['message'],'turn':turn})
            frame_id = frames['r1']['frame_id'];now = float(scene.world.data.time)
            commands = gate.batch(replies,frame_id=frame_id,now_s=now)
            row = {'turn':turn,'at_s':now,'stages':{r:p['stage'] if p else 'FINISHED' for r,p in rows.items()},
                'replies':replies,'stops':{r:v[1] for r,v in batch.items()},'commands':commands,
                'feedback':copy.deepcopy(gate.feedback)}
            result['turns'].append(row)
            for rid,command in commands.items():scene.ports[rid].validate_action(command['action'])
            for rid,command in commands.items():scene.issue(rid,command)
            scene.video.stage = ' | '.join(r+':'+(p['stage'] if p else 'DONE') for r,p in rows.items())
            scene.step(max([c['duration_s'] for c in commands.values()]+[.2])+.1)
            scene.hold()
            previous = frames
            team.save();write(args.output/'execution-events.json',gate.events)
            write(args.output/'progress.json',result)
            print(json.dumps({'execution_turn':turn,'stages':row['stages'],
                'actions':{r:c['action'] for r,c in commands.items()},'feedback':gate.feedback}),flush=True)
            quiet = quiet+1 if not any(c['action']['kind']!='wait' for c in commands.values()) else 0
            if gate.complete:result['stop_reason'] = 'visual_protocol_complete';break
            if gate.revoked:result['stop_reason'] = 'visual_replan_required';break
            if quiet>=args.max_quiet_rounds:result['stop_reason'] = 'no_action_progress_budget';break
        if not result['stop_reason']:result['stop_reason'] = 'decision_round_budget'
        result['protocol_complete'] = gate.complete
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        if args.output.exists():(args.output/'exception.txt').write_text(traceback.format_exc())
    finally:
        try:
            if scene.world:
                scene.hold()
                if scene.video:scene.video.stage = 'FINAL HOLD | referee output only'
                scene.step(1.3)
                scene.capture('final')
                result['camera_geometry_unchanged'] = scene.initial_invariants == scene.invariants()
                result['obstacle_contact_steps'] = scene.obstacle_contact_steps
                if scene.referee:
                    verdict = scene.referee.finish(result.get('plan'))
                    write(args.output/'evaluation-only.json',verdict)
                    result['physical_success'] = verdict['physical_success']
                    result['evaluation'] = verdict
                if gate:
                    result['final_stages'] = {r:(gate.current(r) or {}).get('stage','FINISHED') for r in ROBOTS}
                    result['retained_resources'] = gate.locks
                write(args.output/'issued-commands.json',scene.command_history)
        finally:
            try:
                if team:team.close(float(scene.world.data.time) if scene.world else 0.)
            finally:
                try:
                    if scene.video:scene.video.close()
                finally:scene.close()
        result['wall_seconds'] = time.monotonic()-started
        if team:
            result['model_attempts'] = len(team.calls)
            result['usage'] = {k:sum((c.get('usage') or {}).get(k,0) for c in team.calls)
                               for k in ('prompt_tokens','completion_tokens','total_tokens')}
        if args.output.exists():write(args.output/'result.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='turns'},ensure_ascii=False),flush=True)
    return 1 if result['error'] else 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--variant',choices=VARIANTS,default='shared_crossing')
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--required-dock',choices=('dock_a','dock_b'),help='authored mission destination; allocation and routes still require peer agreement')
    p.add_argument('--rounds',type=int,default=24)
    p.add_argument('--timeout',type=float,default=60.)
    p.add_argument('--max-wall-s',type=float,default=1200.)
    p.add_argument('--max-input-tokens',type=int,default=500000)
    p.add_argument('--max-quiet-rounds',type=int,default=4)
    p.add_argument('--plan-replay',type=Path,help='diagnostic only: replay a saved agreed plan with fixture votes')
    p.add_argument('--live-replan',action='store_true',help='diagnostic only: after rejecting a fixture plan, require actual LLM re-negotiation')
    p.add_argument('--executor',choices=('skills','raw'),default='skills')
    p.add_argument('--contact-profile',choices=('legacy','global_noslip','local_contact','local_contact_fine'),default='local_contact_fine',help='skills only: explicit simulation contact solver profile')
    p.add_argument('--grasp-model-dir',type=Path)
    p.add_argument('--stage-model-dir',type=Path)
    p.add_argument('--carry-act-model',type=Path,help='replace loaded beam motion only')
    p.add_argument('--carry-act-python',type=Path)
    p.add_argument('--carry-act-max-steps',type=int,default=900)
    p.add_argument('--carry-max-steps',type=int,help='common loaded-motion decision cap for RGB and ACT')
    p.add_argument('--spawn-offset',type=float,nargs=3,default=[0.,0.,0.],metavar=('DX','DY','YAW_DEG'),help='setup-only paired comparison perturbation; never actor input')
    p.add_argument('--video-fps',type=int,default=10)
    p.add_argument('--reference-top',type=Path,default=ROOT/'tests/fixtures/camera_goal_transport/reference-top.jpg')
    args = p.parse_args()
    if args.carry_act_model and not args.carry_act_python:p.error('ACT interpreter required')
    if args.carry_max_steps is not None and args.carry_max_steps<=0:p.error('positive carry-max-steps required')
    if args.live_replan and (not args.plan_replay or args.executor!='skills'):
        p.error('--live-replan requires a skills --plan-replay diagnostic')
    if args.output.exists():p.error('output exists; choose a new directory')
    if min(args.rounds,args.timeout,args.max_wall_s,args.max_input_tokens,args.max_quiet_rounds)<=0:p.error('positive budgets required')
    if args.executor=='skills':
        if not args.grasp_model_dir or not args.stage_model_dir:
            p.error('skills execution requires --grasp-model-dir and --stage-model-dir; raw diagnostics require explicit --executor raw')
        from scripts.run_dispatch_skills import run as run_skills
        return run_skills(args)
    return run(args)


if __name__=='__main__':raise SystemExit(main())
