"""Port-bound facade reusing the actual approach and grasp implementations.

Only RGB/callbacks/own issued commands are visible here. Legacy r1/r3 keys are
model slots, remapped at the driver boundary using the committed plan.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from harness.dispatch_skill_binding import canonical_pair_top, beam_feature, PairCoarsePixels, pixel_from_map, BeamContinuity, released_beam_envelope
from harness.camera_goal_transport import coarse_approach, dock_command, preclose_supported, own_payload
from harness.camera_varied_start_student import predict_stage
from harness.grasp_student_inference import predict_student
from harness.pair_carry_policy import PairCarryPolicy, payload_skew
from scripts.camera_approach_scene import ApproachScene, image_record, normalize_replay, ROBOTS
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.run_camera_varied_start_student import run_approach
import math
import copy
import time
import numpy as np

COARSE_LEAD_LIMIT_PX = 16.
# Open approach renews one moving command across its usual RGB refresh gap.
# Zero/dwell stays settled; the port's .25s max and capture-time .6s TTL cap it.
OPEN_APPROACH_RENEWAL_LEASE_S = .25
OPEN_CARRY_RENEWAL_LEASE_S = .25


def _visual_timing(frame, *, perception_wall_s=None, policy_wall_s=None):
    """Output-only wall breakdown; missing legacy timing fields stay absent."""
    spans={
        'snapshot_submit':('capture_requested_wall_s','snapshot_submitted_wall_s'),
        'render_wait':('snapshot_submitted_wall_s','render_completed_wall_s'),
        'jpeg_encode':('render_completed_wall_s','encode_completed_wall_s'),
        'image_record':('encode_completed_wall_s','materialized_wall_s'),
        'pair_bind':('bind_started_wall_s','bind_completed_wall_s'),
    }
    result={name+'_wall_s':max(0.,float(frame[end])-float(frame[start]))
            for name,(start,end) in spans.items() if start in frame and end in frame}
    if perception_wall_s is not None:result['perception_wall_s']=perception_wall_s
    if policy_wall_s is not None:result['policy_wall_s']=policy_wall_s
    return result


def coordinated_coarse_commands(decisions):
    """Let RGB alignment finish before either beam partner advances past the other."""
    if set(decisions) != set(ROBOTS) or not all(decisions[r].get('ok') is True for r in ROBOTS):
        raise ValueError('both coarse RGB decisions are required')
    gaps = {}
    for r in ROBOTS:
        d = decisions[r]
        image_error = d.get('image_error')
        gap = image_error[0] if isinstance(image_error, (list, tuple)) and image_error else d.get('image_gap')
        if gap is None or not math.isfinite(float(gap)):
            raise ValueError('finite RGB-relative forward gap required for paired approach')
        gaps[r] = float(gap)
    aligning = any(abs(float(decisions[r].get(axis, 0.))) > 1e-9
                   for r in ROBOTS for axis in ('left', 'turn'))
    farthest_gap = max(gaps.values())
    commands = {}
    held_forward = []
    for r in ROBOTS:
        d = decisions[r]
        forward = float(d['forward'])
        # All error terms come from the same fresh fixed TOP RGB. A partner
        # correcting heading/lateral pose must not let the other travel ahead.
        # Once aligned, the rear partner may catch up on its own if necessary.
        if forward > 0 and (aligning or (farthest_gap - gaps[r]) * 960 >= COARSE_LEAD_LIMIT_PX):
            forward = 0.
            held_forward.append(r)
        commands[r] = dict(forward=forward, left=float(d.get('left', 0.)),
                           turn=float(d['turn']))
    return commands, {'forward_gap_px': {r: gaps[r] * 960 for r in ROBOTS},
                      'lead_limit_px': COARSE_LEAD_LIMIT_PX,
                      'alignment_in_progress': aligning, 'held_forward': held_forward}


class BoundPairSkill:
    finish_grasp = ApproachScene.finish_grasp
    place = ShortTransportScene.place

    def __init__(self,io,bindings,skill,grasp,stages,model_root,reference,identity=None):
        self.io,self.bindings=io,bindings
        self.skill,self.grasp_models,self.stage_models=skill,grasp,stages
        self.models_root=Path(model_root);self.reference=reference
        self.out=io.out;self.commands={r:dict(skill['initialization_replay'][0]['targets'][r]) for r in ROBOTS}
        self.commands={r:{int(c):int(v) for c,v in p.items()} for r,p in self.commands.items()}
        self.trace=[];self.evaluation_samples=[];self.grasp_report={};self.phase='APPROACH'
        self.last_capture=None;self.count=0;self.calls=[]
        self.grasp_translation=None;self.latest_translation=None;self.transport_started=False
        # Only the open RGB carry path has the owner-step pump contract through
        # release. Rotation and other transports retain their capture timing.
        self.realtime_open_capture=False
        self.beam_continuity=BeamContinuity()
        from harness.dispatch_beam_tracker import CarriedBeamTracker
        self.carried_beam=CarriedBeamTracker()
        self.coarse=PairCoarsePixels(identity,bindings,reference) if identity is not None else None

    def time(self):return self.io.time()
    def tick(self,seconds):
        self.io.pair_phase=self.phase
        self.io.step(seconds)
    def evaluation_snapshot(self):
        # Legacy run_approach stores this as output only. Keep the facade free
        # of simulator state even for reporting; the driver owns the referee.
        return {'source':'separate referee-only.jsonl'}

    def capture(self,tag):
        if (not getattr(self.io,'realtime_control',False) or
                (self.transport_started and not getattr(self,'realtime_open_capture',False))):
            self.count+=1
            frames=self.io.capture('pair-'+str(self.count)+'-'+tag,
                own_robots=tuple(self.bindings.pair.values()),overview=False)
            return self._bind_capture(frames,self.count)
        from sim.snapshot_contract import SnapshotBackpressure
        for _ in range(3):
            self.count+=1;count=self.count
            while True:
                try:
                    future=self.io.capture_async('pair-'+str(count)+'-'+tag,
                        own_robots=tuple(self.bindings.pair.values()),overview=False)
                    break
                except SnapshotBackpressure:
                    self.io.realtime_stats['pair_backpressure']+=1
                    self.tick(.02)
            frames=self.io.await_visual(future)
            self.io.last_frames=frames
            observed=float(frames['r1']['observed_at_s'])
            if self.time()-observed<=.6:
                mapped=self.io.compute_visual(lambda:self._bind_capture(frames,count))
                if self.time()-observed<=.6:return mapped
            self.io.realtime_stats['pair_stage_stale_rgb']+=1
            self.calls.append({'kind':'pair_stage_stale_rgb','stage':self.phase,
                'frame_id':frames['r1']['frame_id'],'observed_at_s':observed,
                'received_at_s':self.time(),'wall_timing':_visual_timing(frames['r1'])})
            self._hold_pair()
        raise RuntimeError('pair stage RGB remained stale after bounded reobservation')

    def _hold_pair(self):
        now=self.time()
        for rid in self.bindings.pair.values():self.io.ports[rid].hold(now)

    def observe_and_compute(self,tag,predict):
        """Commit no paired action from a stale capture or delayed predictor."""
        if not getattr(self.io,'realtime_control',False):
            frames=self.capture(tag)
            return frames,predict(frames)
        for _ in range(3):
            frames=self.capture(tag)
            def measured_predict():
                started=time.monotonic()
                return predict(frames),time.monotonic()-started
            decisions,predict_wall_s=self.io.compute_visual(measured_predict)
            observed=float(frames['r1']['observed_at_s'])
            if self.time()-observed<=.6:
                self.io.realtime_stats['pair_stage_samples']+=1
                self.calls.append({'kind':'pair_stage_sample','stage':self.phase,
                    'frame_id':frames['r1']['frame_id'],'observed_at_s':observed,
                    'received_at_s':self.time(),'decision_age_s':self.time()-observed,
                    'wall_timing':_visual_timing(frames['r1'],
                        perception_wall_s=predict_wall_s)})
                return frames,decisions
            self.io.realtime_stats['pair_stage_stale_rgb']+=1
            self.calls.append({'kind':'pair_stage_stale_prediction','stage':self.phase,
                'frame_id':frames['r1']['frame_id'],'observed_at_s':observed,
                'received_at_s':self.time(),'wall_timing':_visual_timing(frames['r1'],
                    perception_wall_s=predict_wall_s)})
            self._hold_pair()
        raise RuntimeError('pair stage prediction remained stale after bounded reobservation')

    def _bind_capture(self,frames,count):
        bind_started_wall_s=time.monotonic()
        top, transform=canonical_pair_top(frames['r1']['top_bytes'],self.reference,
            translation_px=self.grasp_translation if self.phase.startswith('grasp') else None,
            hue_upper=35 if self.transport_started else 24,
            observed_beam=self.carried_beam.observe(frames['r1']['top_bytes']) if self.transport_started else None)
        if self.transport_started:self.beam_continuity.observe(transform['observed_beam'])
        self.latest_translation=transform['translation_px']
        top_ref=image_record(self.out/'rgb'/f'pair-{count}-canonical-top.jpg',self.out,top)
        bind_completed_wall_s=time.monotonic()
        mapped={slot:{**frames[rid], 'top_bytes':top,'shared_top_rgb':top_ref,
                      'raw_top_rgb':frames[rid]['shared_top_rgb'],'raw_top_bytes':frames[rid]['top_bytes'],'physical_robot_id':rid}
                for slot,rid in self.bindings.pair.items()}
        for frame in mapped.values():
            frame['bind_started_wall_s']=bind_started_wall_s
            frame['bind_completed_wall_s']=bind_completed_wall_s
        self.calls.append({'kind':'image_binding','frame_id':frames['r1']['frame_id'],
            'pair_binding':self.bindings.pair,'transform':transform,'derived_top':top_ref,
            'raw_top':frames['r1']['shared_top_rgb'],
            'own':{slot:frames[rid]['own_rgb'] for slot,rid in self.bindings.pair.items()},
            'wall_timing':_visual_timing(mapped['r1'])})
        self.last_capture=mapped
        return mapped

    def issue_mecanum_bounded(self,commands,duration_s=.2,*,observed_at_s=None):
        if getattr(self.io,'realtime_control',False):
            now=self.time()
            if observed_at_s is None or not math.isfinite(float(observed_at_s)) or float(observed_at_s)>now+1e-9:
                self._hold_pair()
                raise ValueError('bounded pair action requires a valid RGB capture time')
            duration_s=min(duration_s,.25,float(observed_at_s)+.6-now)
            if duration_s<=0:
                self._hold_pair()
                self.calls.append({'kind':'bounded_pair_stale_rgb','stage':self.phase,
                                   'observed_at_s':float(observed_at_s),'received_at_s':now})
                return 0.
        if self.transport_started and self.carried_beam.previous is not None:
            self.bindings.reserve_beam_apron(self.carried_beam.previous)
        self.io.pair_issue_bounded(
            {self.bindings.pair[r]:dict(kind='mecanum',**c,duration_s=duration_s)
             for r,c in commands.items()},duration_s,self.phase)
        return duration_s

    def drive_mecanum(self,commands,duration_s=.2):
        moving=any(abs(c[k])>1e-9 for c in commands.values()
                   for k in ('forward','left','turn'))
        if (getattr(self.io,'realtime_control',False) and not self.bindings.cluttered
                and not self.transport_started and self.phase=='APPROACH' and moving):
            # A short lease continues while the next RGB batch is rendered and
            # interpreted. The owner advances only enough to start that batch.
            frame=self.last_capture['r1'] if self.last_capture else None
            observed=frame.get('observed_at_s') if frame else None
            self.issue_mecanum_bounded(commands,OPEN_APPROACH_RENEWAL_LEASE_S,
                                       observed_at_s=observed)
            self.tick(.02)
            return
        if self.transport_started and self.carried_beam.previous is not None:
            self.bindings.reserve_beam_apron(self.carried_beam.previous)
        self.io.pair_drive({self.bindings.pair[r]:dict(kind='mecanum',**c,duration_s=duration_s)
                            for r,c in commands.items()},duration_s,self.phase)
    def drive(self,forwards,duration_s=.2):
        self.drive_mecanum({r:dict(forward=v,left=0.,turn=0.) for r,v in forwards.items()},duration_s)
    def stop_dwell(self):self.drive({r:0. for r in ROBOTS},.25)

    def replay(self,commands,stage):
        if stage=='grasp_initialization':self.grasp_translation=self.latest_translation
        self.phase=stage
        if stage=='grasp_close':
            frames,d=self.observe_and_compute('preclose-support',lambda frames:{
                r:predict_student(self.grasp_models[r],frames[r]['own_bytes'],
                                  frames[r]['top_bytes'],max_step=25)
                for r in ROBOTS})
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
            def predict_coarse(frames):
                candidate=(copy.deepcopy(self.coarse) if getattr(self.io,'realtime_control',False)
                           else self.coarse)
                raw=(frames['r1']['raw_top_bytes'] if frames else
                     self.io.last_frames['r1']['top_bytes'])
                decisions={r:(candidate.decide(raw,r)
                              if candidate is not None else
                              coarse_approach(frames[r]['top_bytes'],self.reference,r))
                           for r in ROBOTS}
                return decisions,candidate
            frames,(decisions,candidate)=self.observe_and_compute('coarse',predict_coarse)
            if self.coarse is not None:self.coarse=candidate
            if not all(d['ok'] for d in decisions.values()):raise RuntimeError('coarse RGB model convention unresolved')
            commands,coordination=coordinated_coarse_commands(decisions)
            report['coarse_calls'].append(decisions)
            self.calls.append({'kind':'coarse','decisions':decisions,'commands':commands,
                               'coordination':coordination})
            self.drive_mecanum(commands)
            if all(d['ready'] for d in decisions.values()):self.stop_dwell();break
        else:raise RuntimeError('coarse RGB approach budget exhausted')
        report.update(run_approach(self,self.stage_models,reacquire_on_settle=True,
                                   final_refinement_steps=40,invalid_reobserve_budget=1))
        self.calls.append({'kind':'learned_approach','report':report})
        if not report['approach_ok']:raise RuntimeError('fine RGB alignment outside saved skill support')
        ready_count=0
        for index in range(30):
            frames,predictions=self.observe_and_compute('dock',lambda frames:{
                r:predict_stage(self.stage_models[r]['forward'],frames[r]['own_bytes'],frames[r]['top_bytes'])
                for r in ROBOTS})
            commands={r:dock_command(p) for r,p in predictions.items()}
            self.calls.append({'kind':'dock','predictions':predictions,'commands':commands})
            if not all(c['ok'] for c in commands.values()):raise RuntimeError('fine docking outside saved support')
            self.drive({r:c['forward'] for r,c in commands.items()})
            ready_count=ready_count+1 if all(c['ready'] for c in commands.values()) else 0
            if ready_count>=2:return report
        raise RuntimeError('fine docking confirmation budget exhausted')

    def carry(self,navigator,max_steps=None):
        if self.bindings.cluttered:return self.carry_with_rotation(max_steps)
        self.phase='TRANSIT';self.transport_started=True
        anchor=self.capture('carry-anchor')
        policy=PairCarryPolicy('dispatch-'+self.bindings.committed['plan_hash'][:12])
        for index in range(900 if max_steps is None else max_steps):
            frames=self.capture('carry')
            raw=self.io.last_frames['r1']['top_bytes']
            motion,evidence=navigator.observe(raw)
            decisions={}
            for r in ROBOTS:
                current,initial=own_payload(frames[r]['own_bytes'],hue_upper=35),own_payload(anchor[r]['own_bytes'],hue_upper=35)
                held=bool(current and initial and .25<=current[0]/initial[0]<=4
                          and math.dist(current[1:],initial[1:])<=.15)
                decisions[r]={'ok':held,'held_estimate':held,'ready':evidence['done'],
                              'forward':abs(motion['forward']),'current_own_rgb_features':current,'anchor_own_rgb_features':initial,
                              'appearance':'orange-to-yellow beam hue 3..35; same shape/consistency gates'}
            beam=self.carried_beam.previous
            from harness.dispatch_translation_skew import translation_skew
            try:skew,skew_evidence=translation_skew(raw,beam)
            except ValueError as error:
                skew=None;skew_evidence={'unresolved':str(error)}
            now=self.time()
            control=policy.step(decisions,skew,
                {r:f['frame_id'] for r,f in frames.items()},now)
            self.calls.append({'kind':'carry','decisions':decisions,'control':control,'route':evidence,
                'skew_evidence':skew_evidence,'sim_time_s':now,
                'frame_ids':{r:f['frame_id'] for r,f in frames.items()}})
            if control['abort']:raise RuntimeError('existing pair carry guard stopped: '+control['mode'])
            if control['done']:return
            moving=control['mode']=='CRUISE' and control['valid']
            # Common translation only. The original skew recovery stays on its
            # calibrated forward axis; lateral travel stops during recovery.
            commands={r:dict(forward=(math.copysign(v,motion['forward']) if moving and motion['forward'] else v),
                      left=motion['left'] if moving else 0.,turn=0.) for r,v in control['forwards'].items()}
            self.drive_mecanum(commands,control['duration_s'])
        raise RuntimeError('pair route decision budget exhausted')

    def carry_realtime(self,navigator,max_steps=None):
        """Keep physics advancing while one bounded RGB batch is processed."""
        if self.bindings.cluttered:
            # Rotation/grasp stages have distinct motion safety contracts.
            return self.carry_with_rotation(max_steps)
        self.phase='TRANSIT';self.transport_started=True
        self.realtime_open_capture=True
        anchor=self.capture('carry-anchor')
        policy=PairCarryPolicy('dispatch-'+self.bindings.committed['plan_hash'][:12])
        with ThreadPoolExecutor(max_workers=1,thread_name_prefix='dispatch-pair-decision') as worker:
            next_capture=None

            def submit_capture(*,retry):
                # Only the physics owner requests actor snapshots. A speculative
                # request is bounded to one; backpressure is retried when it
                # becomes the current iteration, without skipping frame IDs.
                from sim.snapshot_contract import SnapshotBackpressure
                while True:
                    count=self.count+1
                    try:
                        future=self.io.capture_async('pair-'+str(count)+'-carry',
                            own_robots=tuple(self.bindings.pair.values()),overview=False)
                    except SnapshotBackpressure:
                        self.io.realtime_stats['pair_backpressure']+=1
                        if not retry:return None
                        self.tick(.02)
                    else:
                        self.count=count
                        return count,future

            try:
                for index in range(900 if max_steps is None else max_steps):
                    current=next_capture or submit_capture(retry=True)
                    next_capture=None
                    count,frame_future=current
                    while not frame_future.done():
                        self.tick(.02)
                    raw_frames=frame_future.result()

                    def analyze(raw_frames=raw_frames,count=count):
                        frames=self._bind_capture(raw_frames,count)
                        perception_started_wall_s=time.monotonic()
                        raw=frames['r1']['raw_top_bytes']
                        motion,evidence=navigator.observe(raw)
                        decisions={}
                        for r in ROBOTS:
                            current=own_payload(frames[r]['own_bytes'],hue_upper=35)
                            initial=own_payload(anchor[r]['own_bytes'],hue_upper=35)
                            held=bool(current and initial and .25<=current[0]/initial[0]<=4
                                      and math.dist(current[1:],initial[1:])<=.15)
                            decisions[r]={'ok':held,'held_estimate':held,'ready':evidence['done'],
                                'forward':abs(motion['forward']),
                                'current_own_rgb_features':current,
                                'anchor_own_rgb_features':initial,
                                'appearance':'orange-to-yellow beam hue 3..35; same shape/consistency gates'}
                        from harness.dispatch_translation_skew import translation_skew
                        try:skew,skew_evidence=translation_skew(raw,self.carried_beam.previous)
                        except ValueError as error:
                            skew=None;skew_evidence={'unresolved':str(error)}
                        perception_wall_s=time.monotonic()-perception_started_wall_s
                        return frames,motion,evidence,decisions,skew,skew_evidence,perception_wall_s

                    pending=worker.submit(analyze)
                    if index+1<(900 if max_steps is None else max_steps):
                        # The current immutable RGB batch is materialized.
                        # Rendering one next batch may now overlap its ordered
                        # tracker/perception worker. Its original SIM timestamp
                        # remains authoritative even if it predates this action.
                        next_capture=submit_capture(retry=False)
                    while not pending.done():
                        # Only the owner advances physics. Port leases expire
                        # independently if RGB/render/decision work stalls.
                        self.tick(.02)
                    frames,motion,evidence,decisions,skew,skew_evidence,perception_wall_s=pending.result()
                    now=self.time();observed=float(frames['r1']['observed_at_s'])
                    self.io.realtime_stats['pair_decisions']+=1
                    self.io.realtime_stats['max_decision_age_s']=max(
                        self.io.realtime_stats['max_decision_age_s'],now-observed)
                    policy_started_wall_s=time.monotonic()
                    control=policy.step(decisions,skew,
                        {r:f['frame_id'] for r,f in frames.items()},now,
                        observed_at_s=observed)
                    policy_wall_s=time.monotonic()-policy_started_wall_s
                    carry_record={'kind':'carry','decisions':decisions,'control':control,
                        'route':evidence,'skew_evidence':skew_evidence,
                        'sim_time_s':now,'observed_at_s':observed,
                        'decision_age_s':now-observed,
                        'wall_timing':_visual_timing(frames['r1'],
                            perception_wall_s=perception_wall_s,
                            policy_wall_s=policy_wall_s),
                        'frame_ids':{r:f['frame_id'] for r,f in frames.items()}}
                    self.calls.append(carry_record)
                    if control['abort']:
                        raise RuntimeError('existing pair carry guard stopped: '+control['mode'])
                    if control['done']:
                        carry_record['effective_lease_s']=self.issue_mecanum_bounded(
                            {r:dict(forward=0.,left=0.,turn=0.) for r in ROBOTS},.2,
                            observed_at_s=observed)
                        return
                    moving=control['mode']=='CRUISE' and control['valid']
                    commands={r:dict(
                        forward=(math.copysign(v,motion['forward']) if moving and motion['forward'] else v),
                        left=motion['left'] if moving else 0.,turn=0.)
                        for r,v in control['forwards'].items()}
                    renewing=any(abs(command[axis])>1e-9 for command in commands.values()
                                 for axis in ('forward','left','turn'))
                    requested_lease_s=(OPEN_CARRY_RENEWAL_LEASE_S if renewing
                                       else control['duration_s'])
                    carry_record['requested_lease_s']=requested_lease_s
                    effective_lease_s=self.issue_mecanum_bounded(
                        commands,requested_lease_s,observed_at_s=observed)
                    carry_record['effective_lease_s']=effective_lease_s
                    # At least one physical control interval precedes the next
                    # image. The remaining lease may overlap RGB processing.
                    self.tick(.02 if renewing and effective_lease_s>0 else .05)
            finally:
                now=self.time()
                for rid in self.bindings.pair.values():self.io.ports[rid].hold(now)
                if next_capture is not None:
                    future=next_capture[1]
                    if not future.cancel():
                        try:future.result(timeout=30.)
                        except Exception:pass
        raise RuntimeError('pair route decision budget exhausted')

    def carry_with_rotation(self,max_steps=None):
        from harness.dispatch_navigation_map import navigation_map
        from harness.dispatch_pair_navigation import PairNavigator,authorize_pair
        from harness.pair_carry_sync import PairCarrySync
        from harness.dispatch_own_hold import OwnHoldContinuity
        self.phase='TRANSIT';self.transport_started=True
        anchor=self.capture('carry-anchor')
        other,other_source=self.io.other_robot_observation(anchor['r1']['raw_top_bytes'])
        data=navigation_map(self.bindings,anchor['r1']['raw_top_bytes'],other_robot_center_px=other['center_px'])
        agents={r:PairNavigator(data,r) for r in ROBOTS}
        own_guards={r:OwnHoldContinuity(anchor[r]['own_bytes']) for r in ROBOTS}
        sync=PairCarrySync('dispatch-'+self.bindings.committed['plan_hash'])
        self.calls.append({'kind':'navigation_map','map':data,'other_robot_observation':other,'other_robot_source':other_source})
        for index in range(1200 if max_steps is None else max_steps):
            frames=self.capture('rotate-carry')
            decisions={}
            for r in ROBOTS:
                decision=agents[r].decide(frames[r]['own_bytes'],frames[r]['raw_top_bytes'])
                decision['own_attachment']=own_guards[r].observe(frames[r]['own_bytes'])
                decision['ready']=decision['ready'] and decision['own_attachment']['held_estimate']
                decisions[r]=decision
            permission=authorize_pair(sync,decisions,{r:f['frame_id'] for r,f in frames.items()},index)
            self.calls.append({'kind':'rotating_carry','decisions':decisions,'permission':permission,
                'frame_ids':{r:f['frame_id'] for r,f in frames.items()},
                'images':{r:{'own':f['own_rgb'],'top':f['raw_top_rgb']} for r,f in frames.items()}})
            if permission['phase']!='GO':
                raise RuntimeError('paired navigation stopped: '+str({r:d.get('status') for r,d in decisions.items()}))
            if all(d['done'] for d in decisions.values()):return
            self.drive_mecanum({r:{k:d['action'][k] for k in ('forward','left','turn')}
                for r,d in decisions.items()},.2)
        raise RuntimeError('rotating pair route decision budget exhausted')

    def verify_placement(self):
        """Fresh visual slot/stability claim; physical release stays referee-only."""
        slot=self.bindings.static_map['docks'][self.bindings.plan['dock']]['slots']['beam']
        samples=[]
        def assess(frames):
            # Capture/binding has completed before this runs. The pair tracker
            # is not mutated by owner physics while this RGB work is pending.
            b=(copy.deepcopy(self.carried_beam.previous)
               if getattr(self,'realtime_open_capture',False) else self.carried_beam.previous)
            envelope=released_beam_envelope(frames['r1']['raw_top_bytes'],b,self.bindings.static_map)
            w,h=b['image_size'];corners=np.array(envelope['corners_px'])
            a=pixel_from_map(np.array(slot['center_m'])-slot['half_extents_m'],self.bindings.static_map,(h,w),height=.04)
            z=pixel_from_map(np.array(slot['center_m'])+slot['half_extents_m'],self.bindings.static_map,(h,w),height=.04)
            inside=bool(np.all(corners>=np.minimum(a,z)) and np.all(corners<=np.maximum(a,z)))
            return {'beam':b,'released_envelope':envelope,'inside_visible_slot':inside}
        for index in range(2):
            if getattr(self,'realtime_open_capture',False):
                frames,assessment=self.observe_and_compute('placement-confirmation',assess)
            else:
                frames=self.capture('placement-confirmation')
                assessment=assess(frames)
            samples.append({'frame_id':frames['r1']['frame_id'],**assessment})
            self.tick(.5)
        stable=math.dist(samples[0]['beam']['center'],samples[1]['beam']['center'])<.003
        self.calls.append({'kind':'visual_placement','samples':samples,'stable':stable,
            'meaning':'RGB estimate; contact/release truth is separate output only'})
        if not stable or not all(s['inside_visible_slot'] for s in samples):
            raise RuntimeError('visual placement confirmation failed')
