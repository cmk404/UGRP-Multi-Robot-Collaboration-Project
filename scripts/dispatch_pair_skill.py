"""Port-bound facade reusing the actual approach and grasp implementations.

Only RGB/callbacks/own issued commands are visible here. Legacy r1/r3 keys are
model slots, remapped at the driver boundary using the committed plan.
"""
from __future__ import annotations
from pathlib import Path
from harness.dispatch_skill_binding import canonical_pair_top, beam_feature
from harness.camera_goal_transport import coarse_approach, dock_command, preclose_supported, own_payload
from harness.camera_varied_start_student import predict_stage
from harness.grasp_student_inference import predict_student
from harness.pair_carry_policy import PairCarryPolicy, payload_skew
from scripts.camera_approach_scene import ApproachScene, image_record, normalize_replay, ROBOTS
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.run_camera_varied_start_student import run_approach
import math


class BoundPairSkill:
    finish_grasp = ApproachScene.finish_grasp
    place = ShortTransportScene.place

    def __init__(self,io,bindings,skill,grasp,stages,model_root,reference):
        self.io,self.bindings=io,bindings
        self.skill,self.grasp_models,self.stage_models=skill,grasp,stages
        self.models_root=Path(model_root);self.reference=reference
        self.out=io.out;self.commands={r:dict(skill['initialization_replay'][0]['targets'][r]) for r in ROBOTS}
        self.commands={r:{int(c):int(v) for c,v in p.items()} for r,p in self.commands.items()}
        self.trace=[];self.evaluation_samples=[];self.grasp_report={};self.phase='APPROACH'
        self.last_capture=None;self.count=0;self.calls=[]

    def time(self):return self.io.time()
    def tick(self,seconds):self.io.step(seconds)
    def evaluation_snapshot(self):
        # Legacy run_approach stores this as output only. Keep the facade free
        # of simulator state even for reporting; the driver owns the referee.
        return {'source':'separate referee-only.jsonl'}

    def capture(self,tag):
        self.count+=1
        frames=self.io.capture('pair-'+str(self.count)+'-'+tag)
        top, transform=canonical_pair_top(frames['r1']['top_bytes'],self.reference)
        top_ref=image_record(self.out/'rgb'/f'pair-{self.count}-canonical-top.jpg',self.out,top)
        mapped={slot:{**frames[rid], 'top_bytes':top,'shared_top_rgb':top_ref,
                      'raw_top_rgb':frames[rid]['shared_top_rgb'],'physical_robot_id':rid}
                for slot,rid in self.bindings.pair.items()}
        self.calls.append({'kind':'image_binding','frame_id':frames['r1']['frame_id'],
            'pair_binding':self.bindings.pair,'transform':transform,'derived_top':top_ref,
            'raw_top':frames['r1']['shared_top_rgb'],
            'own':{slot:frames[rid]['own_rgb'] for slot,rid in self.bindings.pair.items()}})
        self.last_capture=mapped
        return mapped

    def drive_mecanum(self,commands,duration_s=.2):
        self.io.pair_drive({self.bindings.pair[r]:dict(kind='mecanum',**c,duration_s=duration_s)
                            for r,c in commands.items()},duration_s,self.phase)
    def drive(self,forwards,duration_s=.2):
        self.drive_mecanum({r:dict(forward=v,left=0.,turn=0.) for r,v in forwards.items()},duration_s)
    def stop_dwell(self):self.drive({r:0. for r in ROBOTS},.25)

    def replay(self,commands,stage):
        self.phase=stage
        if stage=='grasp_close':
            frames=self.capture('preclose-support')
            d={r:predict_student(self.grasp_models[r],frames[r]['own_bytes'],frames[r]['top_bytes'],max_step=25)
               for r in ROBOTS}
            self.calls.append({'kind':'preclose','predictions':d})
            if not preclose_supported(d):raise RuntimeError('preclose RGB outside learned grasp support')
        for command in commands:
            targets=normalize_replay(command)
            mapped={self.bindings.pair[r]:pose for r,pose in targets.items()}
            self.io.pair_arm(mapped,float(command['duration_s']),float(command.get('settle_s',0)),stage)
            for r,pose in targets.items():self.commands[r].update(pose)
            self.trace.append({'stage':stage,'command':command})

    def approach(self):
        report={'coarse_calls':[]}
        for index in range(120):
            frames=self.capture('coarse')
            decisions={r:coarse_approach(frames[r]['top_bytes'],self.reference,r) for r in ROBOTS}
            report['coarse_calls'].append(decisions);self.calls.append({'kind':'coarse','decisions':decisions})
            if not all(d['ok'] for d in decisions.values()):raise RuntimeError('coarse RGB model convention unresolved')
            self.drive_mecanum({r:dict(forward=d['forward'],left=0.,turn=d['turn']) for r,d in decisions.items()})
            if all(d['ready'] for d in decisions.values()):self.stop_dwell();break
        else:raise RuntimeError('coarse RGB approach budget exhausted')
        report.update(run_approach(self,self.stage_models,reacquire_on_settle=True,final_refinement_steps=40))
        self.calls.append({'kind':'learned_approach','report':report})
        if not report['approach_ok']:raise RuntimeError('fine RGB alignment outside saved skill support')
        ready_count=0
        for index in range(30):
            frames=self.capture('dock')
            predictions={r:predict_stage(self.stage_models[r]['forward'],frames[r]['own_bytes'],frames[r]['top_bytes']) for r in ROBOTS}
            commands={r:dock_command(p) for r,p in predictions.items()}
            self.calls.append({'kind':'dock','predictions':predictions,'commands':commands})
            if not all(c['ok'] for c in commands.values()):raise RuntimeError('fine docking outside saved support')
            self.drive({r:c['forward'] for r,c in commands.items()})
            ready_count=ready_count+1 if all(c['ready'] for c in commands.values()) else 0
            if ready_count>=2:return report
        raise RuntimeError('fine docking confirmation budget exhausted')

    def carry(self,navigator):
        self.phase='TRANSIT'
        anchor=self.capture('carry-anchor')
        policy=PairCarryPolicy('dispatch-'+self.bindings.committed['plan_hash'][:12])
        for index in range(900):
            frames=self.capture('carry')
            raw=self.io.last_frames['r1']['top_bytes']
            motion,evidence=navigator.observe(raw)
            decisions={}
            for r in ROBOTS:
                current,initial=own_payload(frames[r]['own_bytes']),own_payload(anchor[r]['own_bytes'])
                held=bool(current and initial and .25<=current[0]/initial[0]<=4
                          and math.dist(current[1:],initial[1:])<=.15)
                decisions[r]={'ok':held,'held_estimate':held,'ready':evidence['done'],
                              'forward':abs(motion['forward'])}
            control=policy.step(decisions,payload_skew(frames['r1']['top_bytes']),
                {r:f['frame_id'] for r,f in frames.items()},self.time())
            self.calls.append({'kind':'carry','decisions':decisions,'control':control,'route':evidence})
            if control['abort']:raise RuntimeError('existing pair carry guard stopped: '+control['mode'])
            if control['done']:return
            moving=control['mode']=='CRUISE' and control['valid']
            # Common translation only. The original skew recovery stays on its
            # calibrated forward axis; lateral travel stops during recovery.
            commands={r:dict(forward=(math.copysign(v,motion['forward']) if moving and motion['forward'] else v),
                      left=motion['left'] if moving else 0.,turn=0.) for r,v in control['forwards'].items()}
            self.drive_mecanum(commands,control['duration_s'])
        raise RuntimeError('pair route decision budget exhausted')
