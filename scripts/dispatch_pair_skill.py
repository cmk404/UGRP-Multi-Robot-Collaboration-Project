"""Port-bound facade reusing the actual approach and grasp implementations.

Only RGB/callbacks/own issued commands are visible here. Legacy r1/r3 keys are
model slots, remapped at the driver boundary using the committed plan.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib
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
_CURRENT_BEAM_FEATURE = object()


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


def _partial_pipeline_timing(frame,result):
    needed=('top_render_completed_wall_s','render_completed_wall_s',
            'top_materialized_wall_s','materialized_wall_s')
    if not all(key in frame and frame[key] is not None for key in needed):
        return None
    start=float(result['top_analysis_started_wall_s'])
    end=float(result['top_analysis_completed_wall_s'])
    top=float(frame['top_render_completed_wall_s'])
    full=float(frame['render_completed_wall_s'])
    return {'top_render_completed_wall_s':top,
            'top_materialized_wall_s':float(frame['top_materialized_wall_s']),
            'top_analysis_started_wall_s':start,
            'top_analysis_completed_wall_s':end,
            'full_render_completed_wall_s':full,
            'own_materialized_wall_s':float(frame['materialized_wall_s']),
            'top_cpu_wall_s':max(0.,end-start),
            'top_cpu_overlap_own_render_wall_s':max(0.,min(end,full)-max(start,top))}


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
        self.process_pair_perception=bool(getattr(io,'realtime_control',False))
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

    def _adopt_carry_analysis(self,raw_frames,count,result):
        """Commit ordered RGB evidence on the owner; permissions stay live here."""
        frame=raw_frames['r1']
        frame_id=frame['frame_id']
        observed=float(frame['observed_at_s'])
        if (result['frame_id']!=frame_id or
                abs(float(result['observed_at_s'])-observed)>1e-9 or
                any(raw_frames[rid]['frame_id']!=frame_id or
                    abs(float(raw_frames[rid]['observed_at_s'])-observed)>1e-9
                    for rid in self.bindings.pair.values())):
            raise RuntimeError('pair perception result/capture provenance mismatch')
        top=result['canonical_top']
        transform=result['transform']
        if transform['source_sha256']!=hashlib.sha256(frame['top_bytes']).hexdigest():
            raise RuntimeError('pair perception returned a different TOP RGB source')
        top_ref=image_record(self.out/'rgb'/f'pair-{count}-canonical-top.jpg',self.out,top)
        mapped={slot:{**raw_frames[rid], 'top_bytes':top,'shared_top_rgb':top_ref,
                      'raw_top_rgb':raw_frames[rid]['shared_top_rgb'],
                      'raw_top_bytes':raw_frames[rid]['top_bytes'],'physical_robot_id':rid,
                      'bind_started_wall_s':result['bind_started_wall_s'],
                      'bind_completed_wall_s':result['bind_completed_wall_s']}
                for slot,rid in self.bindings.pair.items()}
        pipeline_timing=_partial_pipeline_timing(mapped['r1'],result)
        self.carried_beam.previous=copy.deepcopy(result['carried_previous'])
        self.beam_continuity.previous=copy.deepcopy(result['continuity_previous'])
        self.latest_translation=transform['translation_px']
        binding_record={'kind':'image_binding','frame_id':frame_id,
            'pair_binding':self.bindings.pair,'transform':transform,'derived_top':top_ref,
            'raw_top':frame['shared_top_rgb'],
            'own':{slot:raw_frames[rid]['own_rgb'] for slot,rid in self.bindings.pair.items()},
            'analysis_backend':'spawn_cpu',
            'pair_bind_timing_scope':'child RGB tracking and canonicalization; excludes IPC and owner image record',
            'top_cpu_timing_scope':'spawn child tracker, canonicalization, route and skew; excludes IPC, own payload and file record',
            'wall_timing':_visual_timing(mapped['r1'])}
        if pipeline_timing is not None:
            binding_record['partial_pipeline_wall_timing']=pipeline_timing
            for mapped_frame in mapped.values():
                mapped_frame['partial_pipeline_wall_timing']=pipeline_timing
        self.calls.append(binding_record)
        self.last_capture=mapped
        return (mapped,result['motion'],result['route'],result['decisions'],
                result['skew'],result['skew_evidence'],result['perception_wall_s'])

    def issue_mecanum_bounded(self,commands,duration_s=.2,*,observed_at_s=None,
                              reservation_feature=_CURRENT_BEAM_FEATURE):
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
        feature=(self.carried_beam.previous if reservation_feature is _CURRENT_BEAM_FEATURE
                 else reservation_feature)
        if self.transport_started and feature is not None:
            self.bindings.reserve_beam_apron(feature)
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
        process_mode=getattr(self,'process_pair_perception',False)
        if process_mode:
            from harness.dispatch_pair_perception import PairPerceptionProcess, detached_beam_route
            self._hold_pair()
            initialization={'reference':self.reference,
                'anchor_own':{r:anchor[r]['own_bytes'] for r in ROBOTS},
                'carried_previous':copy.deepcopy(self.carried_beam.previous),
                'continuity_previous':copy.deepcopy(self.beam_continuity.previous),
                'route_state':detached_beam_route(navigator)}
            worker_context=PairPerceptionProcess(initialization)
        else:
            worker_context=ThreadPoolExecutor(max_workers=1,thread_name_prefix='dispatch-pair-decision')
        with worker_context as worker:
            if process_mode:
                # Startup occurs before requesting the first carry RGB. The
                # anchor is a fixed appearance reference, not a command lease.
                worker.wait_ready(lambda:self.tick(.02))
            next_capture=None
            pending_lease=None

            def advance_pending():
                nonlocal pending_lease
                lease=pending_lease
                if lease is not None:
                    now=self.time()
                    # The last accepted RGB decision, plan and route permission
                    # remain the only authority while its next image is pending.
                    # A missing authority interface is fail-closed for renewal
                    # (and keeps older non-runtime callers unchanged).
                    authorize=getattr(self.io,'authorize',None)
                    permission=getattr(self.bindings,'permission',None)
                    sync=getattr(policy,'sync',None)
                    if (not callable(authorize) or not callable(permission)
                            or not callable(getattr(sync,'authorize',None))):
                        pending_lease=None
                    else:
                        try:
                            authorize()
                            if self.bindings.committed['plan_hash']!=lease['plan_hash']:
                                raise RuntimeError('pending pair plan changed')
                            authority=sync.authorize(now)
                            if (authority['phase']!='GO'
                                    or authority['epoch']!=lease['sync_epoch']):
                                pending_lease=None
                                self._hold_pair()
                            elif not permission('beam','TRANSIT'):
                                raise RuntimeError('pending pair transit permission revoked')
                            elif lease['accepted_beam'] is not None:
                                self.bindings.reserve_beam_apron(lease['accepted_beam'])
                        except BaseException:
                            pending_lease=None
                            self._hold_pair()
                            raise
                        deadline=lease['observed_at_s']+.6
                        if pending_lease is None:
                            pass
                        elif now>=lease['valid_until_s']-1e-9:
                            # A stopped/expired command may never be revived
                            # from old RGB, even when part of its TTL remains.
                            pending_lease=None
                            self._hold_pair()
                        elif now>=deadline-1e-9:
                            pending_lease=None
                            self._hold_pair()
                        elif (now+.02>=lease['valid_until_s']-1e-9
                              and lease['valid_until_s']<deadline-1e-9):
                            prior_until=lease['valid_until_s']
                            effective=self.issue_mecanum_bounded(
                                lease['commands'],min(.25,deadline-now),
                                observed_at_s=lease['observed_at_s'],
                                reservation_feature=lease['accepted_beam'])
                            if effective<=0:
                                pending_lease=None
                            else:
                                lease['valid_until_s']=now+effective
                                receipt={'kind':'carry_pending_renewal',
                                    'source_frame_ids':dict(lease['frame_ids']),
                                    'source_observed_at_s':lease['observed_at_s'],
                                    'source_decision_sim_time_s':lease['decision_sim_time_s'],
                                    'original_rgb_deadline_s':deadline,
                                    'previous_valid_until_s':prior_until,
                                    'issued_at_s':now,'valid_until_s':now+effective,
                                    'duration_s':effective,'plan_hash':lease['plan_hash'],
                                    'reason':'same_accepted_cruise_while_next_rgb_pending'}
                                self.calls.append(receipt)
                                history=getattr(self.io,'command_history',None)
                                if history is not None:
                                    for rid in self.bindings.pair.values():
                                        history[rid][-1].update(
                                            pending_renewal=True,
                                            source_frame_ids=dict(lease['frame_ids']),
                                            observed_at_s=lease['observed_at_s'],
                                            original_rgb_deadline_s=deadline)
                self.tick(.02)

            def submit_capture(*,retry):
                # Only the physics owner requests actor snapshots. A speculative
                # request is bounded to one; backpressure is retried when it
                # becomes the current iteration, without skipping frame IDs.
                from sim.snapshot_contract import SnapshotBackpressure
                while True:
                    count=self.count+1
                    try:
                        if process_mode:
                            future=self.io.capture_pair_partial_async('pair-'+str(count)+'-carry',
                                own_robots=tuple(self.bindings.pair.values()))
                        else:
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
                    if process_mode:
                        while not frame_future.top.done():
                            advance_pending()
                        top_frame=frame_future.top.result()
                        top_pending=worker.submit_top(top_frame['frame_id'],
                            float(top_frame['observed_at_s']),top_frame['top_bytes'])
                        if index+1<(900 if max_steps is None else max_steps):
                            next_capture=submit_capture(retry=False)
                        while not frame_future.full.done():
                            advance_pending()
                        raw_frames=frame_future.full.result()
                        observed=float(top_frame['observed_at_s'])
                        if (any(raw_frames[rid]['frame_id']!=top_frame['frame_id'] or
                                abs(float(raw_frames[rid]['observed_at_s'])-observed)>1e-9 or
                                raw_frames[rid]['top_bytes']!=top_frame['top_bytes']
                                for rid in self.bindings.pair.values()) or
                                raw_frames['r1']['shared_top_rgb']['sha256']!=
                                top_frame['shared_top_rgb']['sha256']):
                            raise RuntimeError('partial pair TOP/own provenance mismatch')
                        while not top_pending.done():
                            advance_pending()
                        top_result=top_pending.result()
                        if (top_result['frame_id']!=top_frame['frame_id'] or
                                top_result['observed_at_s']!=observed):
                            raise RuntimeError('partial pair TOP analysis provenance mismatch')
                        own_pending=worker.submit_own(top_frame['frame_id'],observed,
                            {slot:raw_frames[rid]['own_bytes']
                             for slot,rid in self.bindings.pair.items()})
                        while not own_pending.done():
                            advance_pending()
                        result=own_pending.result()
                        (frames,motion,evidence,decisions,skew,skew_evidence,
                         perception_wall_s)=self._adopt_carry_analysis(raw_frames,count,result)
                    else:
                        while not frame_future.done():
                            advance_pending()
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
                            next_capture=submit_capture(retry=False)
                        while not pending.done():
                            advance_pending()
                        frames,motion,evidence,decisions,skew,skew_evidence,perception_wall_s=pending.result()
                    pending_lease=None  # A new decision replaces the older RGB authority.
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
                    if process_mode:
                        carry_record['analysis_backend']='spawn_cpu'
                        carry_record['pair_bind_timing_scope']=(
                            'child RGB tracking and canonicalization; excludes IPC and owner image record')
                        carry_record['top_cpu_timing_scope']=(
                            'spawn child tracker, canonicalization, route and skew; excludes IPC, own payload and file record')
                        if 'partial_pipeline_wall_timing' in frames['r1']:
                            carry_record['partial_pipeline_wall_timing']=(
                                frames['r1']['partial_pipeline_wall_timing'])
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
                    permission=control.get('permission',{})
                    if (moving and renewing and effective_lease_s>0
                            and permission.get('phase')=='GO'
                            and callable(getattr(getattr(policy,'sync',None),'authorize',None))):
                        pending_lease={'commands':copy.deepcopy(commands),
                            'observed_at_s':observed,'frame_ids':dict(carry_record['frame_ids']),
                            'decision_sim_time_s':now,'valid_until_s':now+effective_lease_s,
                            'plan_hash':self.bindings.committed['plan_hash'],
                            'sync_epoch':permission['epoch'],
                            'accepted_beam':copy.deepcopy(self.carried_beam.previous)}
                    # At least one physical control interval precedes the next
                    # image. The remaining lease may overlap RGB processing.
                    self.tick(.02 if renewing and effective_lease_s>0 else .05)
            finally:
                pending_lease=None
                now=self.time()
                for rid in self.bindings.pair.values():self.io.ports[rid].hold(now)
                if next_capture is not None:
                    capture=next_capture[1]
                    futures=(capture.top,capture.full) if process_mode else (capture,)
                    for future in futures:
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
