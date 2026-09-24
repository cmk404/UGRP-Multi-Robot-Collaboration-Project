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
from harness.camera_varied_start_student import PHASES, predict_stage
from harness.camera_varied_start_pose_student import SCHEMA as POSE_STAGE_SCHEMA, TOLERANCES as POSE_STAGE_TOLERANCES
from harness.grasp_student_inference import predict_student
from harness.pair_carry_policy import PairCarryPolicy, payload_skew
from scripts.camera_approach_scene import ApproachScene, image_record, normalize_replay, ROBOTS
from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.run_camera_varied_start_student import run_approach
import math
import copy
import time
import numpy as np
from sim.research_dispatch_arena import authored_map, digest as static_map_digest

COARSE_LEAD_LIMIT_PX = 16.
COARSE_FINE_HANDOFF_GAP = .065
# Cluttered maps issue one existing .2s coarse command per RGB batch. Admit it
# only while that whole command fits within the original capture + .6s TTL.
COARSE_CONCURRENT_MAX_CAPTURE_AGE_S = .4
COARSE_CONCURRENT_COMMAND_S = .2
RGB_ACTION_TTL_S = .6
# Open approach renews one moving command across its usual RGB refresh gap.
# Zero/dwell stays settled; the port's .25s max and capture-time .6s TTL cap it.
OPEN_APPROACH_RENEWAL_LEASE_S = .25
FINE_APPROACH_BRIDGE_EXTENSION_S = .05
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


def _fresh_common_coarse_top(frames, now_s):
    """Admit concurrent motion only from one current TOP capture for both roles."""
    try:
        if set(frames) != set(ROBOTS) or type(now_s) not in (int, float):
            return None
        now = float(now_s)
        first, second = (frames[r] for r in ROBOTS)
        frame_id = first['frame_id']
        observed = first['observed_at_s']
        top = first['raw_top_bytes']
        if (type(frame_id) is not int or type(second['frame_id']) is not int
                or frame_id != second['frame_id']
                or type(observed) not in (int, float)
                or type(second['observed_at_s']) not in (int, float)
                or not all(math.isfinite(v) for v in (now, float(observed),
                                                     float(second['observed_at_s'])))
                or abs(float(observed)-float(second['observed_at_s'])) > 1e-9
                or not 0 <= now-float(observed) <= COARSE_CONCURRENT_MAX_CAPTURE_AGE_S
                or not isinstance(top, bytes) or not top
                or top != second['raw_top_bytes']):
            return None
        return {'frame_id': frame_id, 'observed_at_s': float(observed),
                'top_sha256': hashlib.sha256(top).hexdigest(),
                'decision_age_s': now-float(observed)}
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def _coarse_concurrent_issue_age(observed_at_s, issued_at_s, duration_s):
    """Check the whole issued command against its original RGB deadline."""
    try:
        observed, issued, duration = map(float, (observed_at_s, issued_at_s, duration_s))
        if (not all(math.isfinite(v) for v in (observed, issued, duration))
                or duration <= 0 or issued < observed
                or issued + duration > observed + RGB_ACTION_TTL_S + 1e-9):
            return None
        return issued - observed
    except (TypeError, ValueError, OverflowError):
        return None


def _rgb_sha(value):
    return hashlib.sha256(value).hexdigest() if isinstance(value,bytes) and value else None


def coarse_command_audit(index,frames,decisions,bindings,*,source_sha=None,map_sha=None):
    """Output-only RGB/proposal identity; never feeds a later policy decision."""
    source={}
    for slot in ROBOTS:
        frame=frames.get(slot,{}) if isinstance(frames,dict) else {}
        if not isinstance(frame,dict):frame={}
        source[slot]={'physical_robot_id':bindings.pair[slot],
                      'frame_id':frame.get('frame_id'),
                      'observed_at_s':frame.get('observed_at_s'),
                      'own_rgb_sha256':_rgb_sha(frame.get('own_bytes')),
                      'raw_top_sha256':_rgb_sha(frame.get('raw_top_bytes')),
                      'model_top_sha256':_rgb_sha(frame.get('top_bytes'))}
    top_hashes={source[r]['raw_top_sha256'] for r in ROBOTS}
    return {'kind':'coarse_command_receipt','schema':'ugrp.pair_coarse_command_receipt.v1',
            'coarse_index':index,'batch_id':f'pair-coarse-{index:03d}',
            'source_sha':source_sha,'static_map_sha256':map_sha,
            'source_rgb':source,
            'common_raw_top_sha256':(next(iter(top_hashes)) if len(top_hashes)==1 else None),
            'model_proposals':{r:copy.deepcopy(decisions.get(r)) for r in ROBOTS},
            'coordinated_commands':None,'coordination':None,
            'pre_issue_sim_time_s':None,'requested_duration_s':None,
            'effective_duration_s':None,'port_receipts':{},
            'issued_parallel_window_s':None,'physical_motion_inferred':False,
            'status':'decision_only'}


def complete_coarse_command_audit(audit,receipts,bindings):
    """Attach frozen port acks; issued overlap never asserts physical travel."""
    if not isinstance(receipts,dict):
        audit['status']='port_receipt_unavailable'
        return
    rows={}
    for slot in ROBOTS:
        rid=bindings.pair[slot]
        source=receipts.get(rid)
        rows[slot]={'physical_robot_id':rid,**copy.deepcopy(source)} if isinstance(source,dict) \
                   else {'physical_robot_id':rid,'acknowledged':False}
    audit['port_receipts']=rows
    if not all(rows[r].get('acknowledged') is True for r in ROBOTS):
        audit['status']='port_receipt_incomplete'
        return
    audit['status']='issued'
    starts=[rows[r]['actual_issued_at_s'] for r in ROBOTS]
    ends=[rows[r]['effective_expiry_s'] for r in ROBOTS]
    audit['issued_duration_s_by_slot']={r:rows[r]['effective_expiry_s']-
                                        rows[r]['actual_issued_at_s'] for r in ROBOTS}
    if audit['effective_duration_s'] is None:
        durations=list(audit['issued_duration_s_by_slot'].values())
        audit['effective_duration_s']=durations[0] if durations[0]==durations[1] else None
    if all(not any(abs(rows[r]['applied_action'].get(k,0.))>1e-9
                   for k in ('forward','left','turn')) for r in ROBOTS):
        audit['status']='issued_hold'
    audit['rgb_ttl_audit']={}
    audit['lead_gap_audit']={}
    proposals=audit['model_proposals']
    try:
        gaps={r:float(proposals[r]['image_error'][0]) for r in ROBOTS}
        finite_gaps=all(math.isfinite(gap) for gap in gaps.values())
    except (KeyError,TypeError,ValueError,IndexError,OverflowError):
        gaps={};finite_gaps=False
    for slot in ROBOTS:
        observed=audit['source_rgb'][slot]['observed_at_s']
        issued=rows[slot]['actual_issued_at_s']
        expiry=rows[slot]['effective_expiry_s']
        try:
            age=float(issued)-float(observed)
            ttl_ok=(all(math.isfinite(float(value)) for value in (observed,issued,expiry))
                    and age>=0 and float(expiry)<=float(observed)+RGB_ACTION_TTL_S+1e-9)
        except (TypeError,ValueError,OverflowError):
            age=None;ttl_ok=False
        audit['rgb_ttl_audit'][slot]={'capture_age_at_issue_s':age,
                                     'original_deadline_s':(float(observed)+RGB_ACTION_TTL_S
                                         if isinstance(observed,(int,float)) and math.isfinite(observed)
                                         else None),
                                     'whole_issued_window_within_ttl':ttl_ok}
        moving=rows[slot]['applied_action'].get('forward',0.)>0
        lead=(max(gaps.values())-gaps[slot])*960 if finite_gaps else None
        audit['lead_gap_audit'][slot]={'issued_positive_forward':moving,
                                      'lead_px':lead,
                                      'below_16px_when_moving':(lead<COARSE_LEAD_LIMIT_PX
                                          if moving and lead is not None else not moving)}
    parallel=any(
        rows[r]['applied_action'].get('forward',0.)>0
        and any(abs(rows[other]['applied_action'].get(k,0.))>1e-9
                for k in ('left','turn'))
        for r in ROBOTS for other in ROBOTS if other!=r)
    if (parallel
            and all(any(abs(p)>1e-9 for p in rows[r]['port_ack_motor_pwm']) for r in ROBOTS)
            and max(starts)<min(ends)):
        audit['issued_parallel_window_s']=[max(starts),min(ends)]


def _bounded_coarse_values(decision):
    """Check the own RGB policy output before allowing its forward proposal."""
    if not isinstance(decision, dict) or decision.get('ok') is not True \
            or type(decision.get('ready')) is not bool:
        return None
    error = decision.get('image_error')
    heading = decision.get('heading')
    if (not isinstance(error, (list, tuple)) or len(error) != 2
            or not isinstance(heading, dict)):
        return None
    values = [*error, heading.get('angle_deg'), *(decision.get(k) for k in ('forward','left','turn'))]
    if any(type(value) not in (int, float) for value in values):
        return None
    try:
        gap_x, gap_y, angle, forward, left, turn = map(float, values)
    except (TypeError, ValueError, OverflowError):
        return None
    if not all(math.isfinite(value) for value in (gap_x,gap_y,angle,forward,left,turn)):
        return None
    if not (gap_x >= 0 and 0 <= forward <= .12 and abs(left) <= .05 and abs(turn) <= .10):
        return None
    if decision['ready'] and any(abs(value) > 1e-9 for value in (forward,left,turn)):
        return None
    if (forward > 0 and (decision['ready'] or gap_x <= COARSE_FINE_HANDOFF_GAP
                         or abs(gap_y) > .003 or abs(angle) > 1.5
                         or abs(left) > 1e-9 or abs(turn) > 1e-9)):
        return None
    return gap_x, forward, left, turn


def coordinated_coarse_commands(decisions, *, concurrent_alignment=False,
                                frames=None, now_s=None):
    """Keep the 16px lead bound; opt in to moving while a peer aligns."""
    if concurrent_alignment:
        capture = _fresh_common_coarse_top(frames, now_s)
        values = ({r: _bounded_coarse_values(decisions[r]) for r in ROBOTS}
                  if isinstance(decisions, dict) and set(decisions) == set(ROBOTS)
                  else None)
        if capture is None or values is None or any(v is None for v in values.values()):
            return ({r: dict(forward=0., left=0., turn=0.) for r in ROBOTS},
                    {'coarse_concurrent_alignment': True,
                     'admitted': False, 'reason': 'coarse_capture_or_policy_unresolved',
                     'held_forward': list(ROBOTS), 'lead_limit_px': COARSE_LEAD_LIMIT_PX})
        gaps = {r: values[r][0] for r in ROBOTS}
        aligning = any(abs(values[r][axis]) > 1e-9 for r in ROBOTS for axis in (2, 3))
        farthest_gap = max(gaps.values())
        commands = {}
        held_forward = []
        for r in ROBOTS:
            gap, forward, left, turn = values[r]
            if forward > 0 and (farthest_gap-gap)*960 >= COARSE_LEAD_LIMIT_PX:
                forward = 0.
                held_forward.append(r)
            commands[r] = dict(forward=forward, left=left, turn=turn)
        return commands, {'forward_gap_px': {r: gaps[r]*960 for r in ROBOTS},
                          'lead_limit_px': COARSE_LEAD_LIMIT_PX,
                          'alignment_in_progress': aligning,
                          'held_forward': held_forward,
                          'coarse_concurrent_alignment': True, 'admitted': True,
                          'common_top_capture': capture,
                          'concurrent_forward_with_peer_alignment': [
                              r for r in ROBOTS if commands[r]['forward'] > 0 and
                              any(abs(commands[other][axis]) > 1e-9
                                  for other in ROBOTS if other != r for axis in ('left','turn'))]}
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


def _handoff_gaps(decisions):
    """Only valid coarse or its exact heading dropout may supply RGB gaps."""
    gaps = {}
    if set(decisions) != set(ROBOTS):
        return None
    for r in ROBOTS:
        d = decisions[r]
        error = d.get('image_error')
        if (not (d.get('ok') is True or
                 (d.get('ok') is False and d.get('reason')=='own_wheel_heading_unresolved'))
                or not isinstance(error, (list, tuple)) or not error):
            return None
        try:
            gap = float(error[0])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(gap) or gap > COARSE_FINE_HANDOFF_GAP:
            return None
        gaps[r] = gap
    return gaps


def _fresh_handoff_frames(before, after):
    """The stopped pair must be seen together in a strictly newer RGB batch."""
    try:
        old = [before[r] for r in ROBOTS]
        new = [after[r] for r in ROBOTS]
        old_ids = [int(f['frame_id']) for f in old]
        new_ids = [int(f['frame_id']) for f in new]
        old_times = [float(f['observed_at_s']) for f in old]
        new_times = [float(f['observed_at_s']) for f in new]
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    return (old_ids[0] == old_ids[1] and new_ids[0] == new_ids[1]
            and new_ids[0] > old_ids[0]
            and all(math.isfinite(t) for t in old_times + new_times)
            and old_times[0] == old_times[1] and new_times[0] == new_times[1]
            and new_times[0] > old_times[0])


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
        # Explicit experimental far-field fine alignment command schedule.
        self.fine_gain_schedule=bool(getattr(io,'fine_gain_schedule',False))
        self.coarse_concurrent_alignment=bool(getattr(io,'coarse_concurrent_alignment',False))
        self.grasp_translation=None;self.latest_translation=None;self.transport_started=False
        # Only the open RGB carry path has the owner-step pump contract through
        # release. Rotation and other transports retain their capture timing.
        self.realtime_open_capture=False
        self.process_pair_perception=bool(getattr(io,'realtime_control',False))
        self.beam_continuity=BeamContinuity()
        from harness.dispatch_beam_tracker import CarriedBeamTracker
        self.carried_beam=CarriedBeamTracker()
        self.coarse=PairCoarsePixels(identity,bindings,reference) if identity is not None else None
        self._approach_pending_lease=None
        self._fine_pending_lease=None
        self._fine_candidate=None

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
                    if getattr(self,'_fine_pending_lease',None) is not None:
                        self._advance_fine_pending()
                    elif getattr(self,'_approach_pending_lease',None) is not None:
                        self._advance_approach_pending()
                    else:self.tick(.02)
            frames=self._await_approach_visual(future)
            self.io.last_frames=frames
            observed=float(frames['r1']['observed_at_s'])
            if self.time()-observed<=.6:
                mapped=self._compute_approach_visual(lambda:self._bind_capture(frames,count))
                if self.time()-observed<=.6:return mapped
            self.io.realtime_stats['pair_stage_stale_rgb']+=1
            self.calls.append({'kind':'pair_stage_stale_rgb','stage':self.phase,
                'frame_id':frames['r1']['frame_id'],'observed_at_s':observed,
                'received_at_s':self.time(),'wall_timing':_visual_timing(frames['r1'])})
            self._hold_pair()
        raise RuntimeError('pair stage RGB remained stale after bounded reobservation')

    def _hold_pair(self):
        self._approach_pending_lease=None
        self._fine_pending_lease=None
        self._fine_candidate=None
        now=self.time()
        for rid in self.bindings.pair.values():self.io.ports[rid].hold(now)

    def _record_coarse_hold(self,audit,status):
        """Save the issued HOLD's port cache without reading measured state."""
        audit['status']=status
        audit['held_at_s']=self.time()
        audit['post_hold_motor_pwm']={
            slot:list(getattr(self.io.ports[rid],'_motor_commands',()) or ())
            for slot,rid in self.bindings.pair.items()}

    def _clear_approach_pending(self,*,hold=False):
        lease=getattr(self,'_approach_pending_lease',None)
        self._approach_pending_lease=None
        if hold and lease is not None:
            self._hold_pair()

    def _clear_fine_pending(self,*,hold=False):
        lease=getattr(self,'_fine_pending_lease',None)
        self._fine_pending_lease=None
        if hold and lease is not None:
            self._hold_pair()

    def _open_coarse_scope(self):
        static=self.bindings.static_map
        return (getattr(self.io,'realtime_control',False)
                and static.get('map_id')=='dispatch_open'
                and not self.bindings.cluttered and not static.get('terrain')
                and all(o.get('id') in {'wall_north','wall_south','wall_west','wall_east'}
                        for o in static.get('obstacles',[])))

    def _concurrent_coarse_scope(self):
        """The fixed teacher arena and open map use the same RGB admission."""
        static=self.bindings.static_map
        map_id=static.get('map_id')
        config=getattr(self.io,'config',None)
        variant=config.get('variant') if isinstance(config,dict) else None
        if variant not in {'open','shared_crossing'}:
            return False
        try:
            authored_hash=static_map_digest(static)
            expected_hash=static_map_digest(authored_map(variant))
        except (TypeError,ValueError,KeyError,OSError):
            return False
        allowed={'wall_north','wall_south','wall_west','wall_east'}
        if map_id=='dispatch_shared_crossing':
            allowed.add('service_island')
        return (getattr(self.io,'realtime_control',False)
                and getattr(self,'phase',None)=='APPROACH'
                and not getattr(self,'transport_started',False)
                and map_id in {'dispatch_open','dispatch_shared_crossing'}
                and map_id=='dispatch_'+variant
                and config.get('static_map_sha256')==authored_hash==expected_hash
                and not static.get('terrain')
                and all(o.get('id') in allowed for o in static.get('obstacles',[])))

    def _approach_port_matches(self,lease):
        for slot,rid in self.bindings.pair.items():
            port=self.io.ports[rid]
            if (getattr(port,'_command_expires_at',None)!=lease['valid_until_s']
                    or getattr(port,'_drive_expires_at',None)!=lease['valid_until_s']
                    or tuple(getattr(port,'_motor_commands',()) or ())!=lease['motor_commands'][slot]):
                return False
        return True

    def _accept_coarse_pending(self,frames,decisions,commands):
        """Record only an issued, valid open coarse command as renewal authority."""
        self._approach_pending_lease=None
        if (not self._open_coarse_scope() or getattr(self,'phase',None)!='APPROACH'
                or getattr(self,'transport_started',False)
                or any(decisions[r].get('ok') is not True or
                       decisions[r].get('ready') is True for r in ROBOTS)
                or _handoff_gaps(decisions) is not None
                or not any(abs(commands[r][axis])>1e-9 for r in ROBOTS
                           for axis in ('forward','left','turn'))):
            return
        history=getattr(self.io,'command_history',None)
        if history is None:return
        try:
            observed=float(frames['r1']['observed_at_s'])
            frame_ids={r:int(frames[r]['frame_id']) for r in ROBOTS}
            if (not math.isfinite(observed) or
                    any(float(frames[r]['observed_at_s'])!=observed for r in ROBOTS)
                    or len(set(frame_ids.values()))!=1):return
            rows={slot:history[rid][-1] for slot,rid in self.bindings.pair.items()}
            issued=float(rows['r1']['issued_at_s'])
            valid=float(rows['r1']['valid_until_s'])
            if (any(row['stage']!='APPROACH' or row['issued_at_s']!=issued
                    or row['valid_until_s']!=valid for row in rows.values())
                    or not 0.<valid-issued<=.25+1e-9
                    or valid>observed+.6+1e-9
                    or self.time()>=valid-1e-9
                    or any(any(rows[slot]['action'][axis]!=commands[slot][axis]
                               for axis in ('forward','left','turn'))
                           for slot in ROBOTS)):return
            motors={slot:tuple(self.io.ports[rid]._motor_commands)
                    for slot,rid in self.bindings.pair.items()}
        except (KeyError,IndexError,TypeError,ValueError,AttributeError):
            return
        lease={'commands':copy.deepcopy(commands),'frame_ids':frame_ids,
               'observed_at_s':observed,'decision_sim_time_s':issued,
               'valid_until_s':valid,'plan_hash':self.bindings.committed['plan_hash'],
               'motor_commands':motors}
        if self._approach_port_matches(lease):
            for row in rows.values():
                row.update(source_frame_ids=dict(frame_ids),observed_at_s=observed)
            self._approach_pending_lease=lease

    def _record_fine_candidate(self,tag,frames,decisions):
        """Keep only a supported, clearly far fine slice for its next issue."""
        self._fine_candidate=None
        parts=tag.split('-')
        if (not self._open_coarse_scope() or self.phase!='APPROACH'
                or self.transport_started or len(parts)!=3 or parts[0]!='phase'
                or not parts[1].isdigit() or not parts[2].isdigit()):return
        phase_index=int(parts[1])
        if (phase_index>=len(PHASES) or tag!=f'phase-{phase_index}-{int(parts[2]):03d}'
                or PHASES[phase_index] not in ('lateral','forward')):return
        axis=PHASES[phase_index]
        tolerance=POSE_STAGE_TOLERANCES[axis]
        far_errors={}
        expected={}
        try:
            observed=float(frames['r1']['observed_at_s'])
            frame_ids={r:int(frames[r]['frame_id']) for r in ROBOTS}
            if (not math.isfinite(observed) or len(set(frame_ids.values()))!=1
                    or any(float(frames[r]['observed_at_s'])!=observed for r in ROBOTS)):
                return
            for slot in ROBOTS:
                model=self.stage_models[slot][axis]
                prediction=decisions[slot]
                settings=model['settings']
                error=float(prediction['diagnostics']['image_derived_error'])
                command=float(prediction['command'])
                if (model.get('schema')!=POSE_STAGE_SCHEMA
                        or model.get('robot_id')!=slot or model.get('stage')!=axis
                        or settings.get('tolerance')!=tolerance
                        or prediction.get('ok') is not True
                        or prediction.get('precision')!='fine'
                        or not math.isfinite(error) or not math.isfinite(command)):
                    return
                if prediction.get('ready') is True:
                    if command!=0.:return
                    expected[slot]=0.
                elif abs(error)>2*tolerance and abs(command)>1e-9:
                    far_errors[slot]=error
                    expected[slot]=command
                else:return
        except (KeyError,TypeError,ValueError,OverflowError,AttributeError):
            return
        if far_errors:
            self._fine_candidate={'phase_tag':tag,'phase_index':phase_index,
                'axis':axis,'far_error_m':far_errors,'outer_tolerance_m':tolerance,
                'source_ready':{r:decisions[r]['ready'] for r in ROBOTS},
                'expected':expected,'frame_ids':frame_ids,'observed_at_s':observed}

    def _same_fine_phase_tag(self,tag,lease):
        parts=tag.split('-')
        return (len(parts)==3 and parts[0]=='phase'
                and parts[1]==str(lease['phase_index']) and parts[2].isdigit()
                and tag==f"phase-{lease['phase_index']}-{int(parts[2]):03d}"
                and PHASES[lease['phase_index']]==lease['axis'])

    def _accept_fine_pending(self,commands):
        """The issued motor lease, not a worker prediction, owns the bridge."""
        candidate=getattr(self,'_fine_candidate',None)
        self._fine_candidate=None
        self._fine_pending_lease=None
        if candidate is None:return
        axis=candidate['axis']
        component='left' if axis=='lateral' else 'forward'
        if any(commands[r].get(component)!=candidate['expected'][r]
               or any(abs(commands[r].get(other,0.))>1e-9
                      for other in ('forward','left','turn') if other!=component)
               for r in ROBOTS):return
        history=getattr(self.io,'command_history',None)
        if history is None:return
        try:
            rows={slot:history[rid][-1] for slot,rid in self.bindings.pair.items()}
            issued=float(rows['r1']['issued_at_s'])
            valid=float(rows['r1']['valid_until_s'])
            observed=candidate['observed_at_s']
            if (any(row['stage']!='APPROACH' or row['issued_at_s']!=issued
                    or row['valid_until_s']!=valid for row in rows.values())
                    or not 0.<valid-issued<=.25+1e-9
                    or valid>observed+.6+1e-9
                    or self.time()>=valid-1e-9
                    or any(any(rows[slot]['action'][key]!=commands[slot][key]
                               for key in ('forward','left','turn'))
                           for slot in ROBOTS)):
                return
            motors={slot:tuple(self.io.ports[rid]._motor_commands)
                    for slot,rid in self.bindings.pair.items()}
        except (KeyError,IndexError,TypeError,ValueError,AttributeError):
            return
        lease={'commands':copy.deepcopy(commands),'frame_ids':candidate['frame_ids'],
            'observed_at_s':observed,'decision_sim_time_s':issued,
            'valid_until_s':valid,'original_valid_until_s':valid,
            'plan_hash':self.bindings.committed['plan_hash'],'motor_commands':motors,
            'phase_tag':candidate['phase_tag'],'phase_index':candidate['phase_index'],
            'axis':axis,'far_error_m':candidate['far_error_m'],
            'outer_tolerance_m':candidate['outer_tolerance_m'],
            'source_ready':candidate['source_ready'],'bridged':False}
        if self._approach_port_matches(lease):
            for row in rows.values():
                row.update(source_frame_ids=dict(candidate['frame_ids']),
                    observed_at_s=observed,phase_tag=candidate['phase_tag'],
                    fine_axis=axis,fine_bridge_eligible=True)
            self._fine_pending_lease=lease

    def _advance_approach_pending(self):
        lease=getattr(self,'_approach_pending_lease',None)
        if lease is not None:
            now=self.time();deadline=lease['observed_at_s']+.6
            if (self.phase!='APPROACH' or self.transport_started
                    or not self._open_coarse_scope()
                    or now>=lease['valid_until_s']-1e-9
                    or now>=deadline-1e-9
                    or not self._approach_port_matches(lease)):
                self._clear_approach_pending(hold=True)
            else:
                try:
                    self.io.authorize()
                    if self.bindings.committed['plan_hash']!=lease['plan_hash']:
                        raise RuntimeError('pending approach plan changed')
                    if not self.bindings.permission('beam','APPROACH'):
                        raise RuntimeError('pending approach permission revoked')
                    if (now+.02>=lease['valid_until_s']-1e-9
                            and lease['valid_until_s']<deadline-1e-9):
                        prior=lease['valid_until_s']
                        effective=self.issue_mecanum_bounded(
                            lease['commands'],min(.25,deadline-now),
                            observed_at_s=lease['observed_at_s'])
                        if effective<=0:
                            self._clear_approach_pending(hold=True)
                        else:
                            lease['valid_until_s']=now+effective
                            lease['motor_commands']={
                                slot:tuple(self.io.ports[rid]._motor_commands)
                                for slot,rid in self.bindings.pair.items()}
                            if not self._approach_port_matches(lease):
                                raise RuntimeError('renewed approach port state changed')
                            receipt={'kind':'approach_coarse_pending_renewal',
                                'source_frame_ids':dict(lease['frame_ids']),
                                'source_observed_at_s':lease['observed_at_s'],
                                'source_decision_sim_time_s':lease['decision_sim_time_s'],
                                'original_rgb_deadline_s':deadline,
                                'previous_valid_until_s':prior,'issued_at_s':now,
                                'valid_until_s':now+effective,'duration_s':effective,
                                'plan_hash':lease['plan_hash'],
                                'permission_request':['beam','APPROACH'],
                                'permission_verified_at_s':now,
                                'phase':self.phase,'commands':copy.deepcopy(lease['commands'])}
                            self.calls.append(receipt)
                            history=getattr(self.io,'command_history',None)
                            if history is not None:
                                for rid in self.bindings.pair.values():
                                    history[rid][-1].update(
                                        pending_renewal=True,
                                        source_frame_ids=dict(lease['frame_ids']),
                                        observed_at_s=lease['observed_at_s'],
                                        original_rgb_deadline_s=deadline,
                                        source_decision_sim_time_s=lease['decision_sim_time_s'],
                                        previous_valid_until_s=prior,
                                        duration_s=effective,
                                        permission_request=['beam','APPROACH'],
                                        permission_verified_at_s=now)
                except BaseException:
                    self._clear_approach_pending(hold=True)
                    raise
        self.tick(.02)

    def _advance_fine_pending(self):
        lease=getattr(self,'_fine_pending_lease',None)
        if lease is not None:
            now=self.time()
            deadline=lease['observed_at_s']+.6
            ceiling=min(lease['original_valid_until_s']+FINE_APPROACH_BRIDGE_EXTENSION_S,
                        deadline)
            if (self.phase!='APPROACH' or self.transport_started
                    or not self._open_coarse_scope()
                    or now>=lease['valid_until_s']-1e-9
                    or now>=deadline-1e-9
                    or not self._approach_port_matches(lease)):
                self._clear_fine_pending(hold=True)
            else:
                try:
                    self.io.authorize()
                    if self.bindings.committed['plan_hash']!=lease['plan_hash']:
                        raise RuntimeError('pending fine approach plan changed')
                    if not self.bindings.permission('beam','APPROACH'):
                        raise RuntimeError('pending fine approach permission revoked')
                    if (not lease['bridged'] and now+.02>=lease['valid_until_s']-1e-9
                            and ceiling>lease['valid_until_s']+1e-9):
                        prior=lease['valid_until_s']
                        effective=self.issue_mecanum_bounded(
                            lease['commands'],min(.25,ceiling-now),
                            observed_at_s=lease['observed_at_s'])
                        if effective<=0:
                            self._clear_fine_pending(hold=True)
                        else:
                            lease['valid_until_s']=now+effective
                            lease['bridged']=True
                            lease['motor_commands']={
                                slot:tuple(self.io.ports[rid]._motor_commands)
                                for slot,rid in self.bindings.pair.items()}
                            if (lease['valid_until_s']>ceiling+1e-9
                                    or not self._approach_port_matches(lease)):
                                raise RuntimeError('renewed fine approach port state changed')
                            extension=max(0.,lease['valid_until_s']-lease['original_valid_until_s'])
                            receipt={'kind':'approach_fine_pending_bridge',
                                'source_frame_ids':dict(lease['frame_ids']),
                                'source_observed_at_s':lease['observed_at_s'],
                                'source_decision_sim_time_s':lease['decision_sim_time_s'],
                                'phase_tag':lease['phase_tag'],
                                'phase_index':lease['phase_index'],'axis':lease['axis'],
                                'source_ready':dict(lease['source_ready']),
                                'source_stationary':False,
                                'far_error_m':dict(lease['far_error_m']),
                                'outer_tolerance_m':lease['outer_tolerance_m'],
                                'near_ready_multiplier':2,
                                'original_rgb_deadline_s':deadline,
                                'original_valid_until_s':lease['original_valid_until_s'],
                                'previous_valid_until_s':prior,'issued_at_s':now,
                                'valid_until_s':lease['valid_until_s'],
                                'duration_s':effective,'total_extension_s':extension,
                                'plan_hash':lease['plan_hash'],
                                'permission_request':['beam','APPROACH'],
                                'permission_verified_at_s':now,
                                'phase':self.phase,
                                'commands':copy.deepcopy(lease['commands'])}
                            self.calls.append(receipt)
                            history=getattr(self.io,'command_history',None)
                            if history is not None:
                                for rid in self.bindings.pair.values():
                                    history[rid][-1].update(
                                        pending_renewal=True,
                                        source_frame_ids=dict(lease['frame_ids']),
                                        observed_at_s=lease['observed_at_s'],
                                        source_decision_sim_time_s=lease['decision_sim_time_s'],
                                        phase_tag=lease['phase_tag'],fine_axis=lease['axis'],
                                        original_rgb_deadline_s=deadline,
                                        original_valid_until_s=lease['original_valid_until_s'],
                                        previous_valid_until_s=prior,duration_s=effective,
                                        total_extension_s=extension,
                                        permission_request=['beam','APPROACH'],
                                        permission_verified_at_s=now)
                except BaseException:
                    self._clear_fine_pending(hold=True)
                    raise
        self.tick(.02)

    def _await_approach_visual(self,future):
        if (getattr(self,'_approach_pending_lease',None) is None
                and getattr(self,'_fine_pending_lease',None) is None):
            return self.io.await_visual(future)
        try:
            while not future.done():
                if getattr(self,'_fine_pending_lease',None) is not None:
                    self._advance_fine_pending()
                else:self._advance_approach_pending()
            return future.result()
        except BaseException:
            self._clear_approach_pending(hold=True)
            self._clear_fine_pending(hold=True)
            raise

    def _compute_approach_visual(self,fn):
        if (getattr(self,'_approach_pending_lease',None) is None
                and getattr(self,'_fine_pending_lease',None) is None):
            return self.io.compute_visual(fn)
        worker=getattr(self.io,'_decision_workers',None)
        if worker is None:
            self._clear_approach_pending(hold=True)
            self._clear_fine_pending(hold=True)
            raise RuntimeError('approach renewal requires owner visual worker')
        return self._await_approach_visual(worker.submit(fn))

    def observe_and_compute(self,tag,predict):
        try:return self._observe_and_compute_owned(tag,predict)
        except BaseException:
            self._clear_approach_pending(hold=True)
            self._clear_fine_pending(hold=True)
            raise

    def _observe_and_compute_owned(self,tag,predict):
        """Commit no paired action from a stale capture or delayed predictor."""
        self._fine_candidate=None
        lease=getattr(self,'_fine_pending_lease',None)
        if lease is not None and not self._same_fine_phase_tag(tag,lease):
            self._clear_fine_pending(hold=True)
        if not getattr(self.io,'realtime_control',False):
            frames=self.capture(tag)
            return frames,predict(frames)
        for _ in range(3):
            frames=self.capture(tag)
            def measured_predict():
                started=time.monotonic()
                return predict(frames),time.monotonic()-started
            decisions,predict_wall_s=self._compute_approach_visual(measured_predict)
            observed=float(frames['r1']['observed_at_s'])
            if self.time()-observed<=.6:
                self._clear_approach_pending(hold=True)
                self._clear_fine_pending(hold=True)
                self._record_fine_candidate(tag,frames,decisions)
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
                              reservation_feature=_CURRENT_BEAM_FEATURE,
                              receipt_sink=None):
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
        applied=self.io.pair_issue_bounded(
            {self.bindings.pair[r]:dict(kind='mecanum',**c,duration_s=duration_s)
             for r,c in commands.items()},duration_s,self.phase)
        if receipt_sink is not None and isinstance(applied,dict):
            receipt_sink.update(applied)
        return duration_s

    def drive_mecanum(self,commands,duration_s=COARSE_CONCURRENT_COMMAND_S,*,
                      coarse_observed_at_s=None,coarse_audit=None):
        self._clear_approach_pending(hold=True)
        self._clear_fine_pending(hold=True)
        moving=any(abs(c[k])>1e-9 for c in commands.values()
                   for k in ('forward','left','turn'))
        if (getattr(self.io,'realtime_control',False) and not self.bindings.cluttered
                and not self.transport_started and self.phase=='APPROACH' and moving):
            # A short lease continues while the next RGB batch is rendered and
            # interpreted. The owner advances only enough to start that batch.
            frame=self.last_capture['r1'] if self.last_capture else None
            observed=(coarse_observed_at_s if coarse_observed_at_s is not None else
                      frame.get('observed_at_s') if frame else None)
            applied={}
            if coarse_audit is not None:
                coarse_audit['pre_issue_sim_time_s']=self.time()
                coarse_audit['requested_duration_s']=OPEN_APPROACH_RENEWAL_LEASE_S
            effective=self.issue_mecanum_bounded(commands,OPEN_APPROACH_RENEWAL_LEASE_S,
                                                 observed_at_s=observed,
                                                 receipt_sink=applied if coarse_audit is not None else None)
            if coarse_audit is not None:
                coarse_audit['effective_duration_s']=effective
                coarse_audit['duration_clipped_for_rgb_ttl']=effective<OPEN_APPROACH_RENEWAL_LEASE_S
                if effective>0:complete_coarse_command_audit(coarse_audit,applied,self.bindings)
                else:self._record_coarse_hold(coarse_audit,'bounded_rgb_ttl_rejected_held')
            if effective>0:self._accept_fine_pending(commands)
            else:self._fine_candidate=None
            self.tick(.02)
            return
        self._fine_candidate=None
        if self.transport_started and self.carried_beam.previous is not None:
            self.bindings.reserve_beam_apron(self.carried_beam.previous)
        pre_issue=(self.time() if coarse_audit is not None or
                   coarse_observed_at_s is not None else None)
        if coarse_audit is not None:
            coarse_audit['pre_issue_sim_time_s']=pre_issue
            coarse_audit['requested_duration_s']=duration_s
        if coarse_observed_at_s is not None and moving:
            # The owner thread does not advance SIM time between this check
            # and pair_drive/raw issuance. Include all compute-to-issue delay.
            issue_age=_coarse_concurrent_issue_age(
                coarse_observed_at_s,pre_issue,duration_s)
            if issue_age is None:
                self._hold_pair()
                if coarse_audit is not None:
                    self._record_coarse_hold(coarse_audit,'coarse_issue_ttl_rejected_held')
                self.calls.append({'kind':'coarse_concurrent_stale_issue',
                                   'observed_at_s':coarse_observed_at_s,
                                   'issued_check_at_s':pre_issue,
                                   'duration_s':duration_s})
                return False
            self.calls.append({'kind':'coarse_concurrent_issue_guard',
                               'source_observed_at_s':coarse_observed_at_s,
                               'issue_age_s':issue_age,'duration_s':duration_s,
                               'rgb_deadline_s':coarse_observed_at_s+RGB_ACTION_TTL_S})
        try:
            applied=self.io.pair_drive({self.bindings.pair[r]:dict(kind='mecanum',**c,duration_s=duration_s)
                                        for r,c in commands.items()},duration_s,self.phase)
        except BaseException:
            if coarse_audit is not None:coarse_audit['status']='issue_exception_receipt_unknown'
            raise
        if coarse_audit is not None:
            complete_coarse_command_audit(coarse_audit,applied,self.bindings)
    def drive(self,forwards,duration_s=.2):
        self.drive_mecanum({r:dict(forward=v,left=0.,turn=0.) for r,v in forwards.items()},duration_s)
    def stop_dwell(self,*,coarse_audit=None):
        self.drive_mecanum({r:dict(forward=0.,left=0.,turn=0.) for r in ROBOTS},
                           .25,coarse_audit=coarse_audit)

    def replay(self,commands,stage):
        self._clear_approach_pending(hold=True)
        self._clear_fine_pending(hold=True)
        self._fine_candidate=None
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
        # The extra admission is confined to the live open map and may stop
        # for one stationary probe only. It does not change the coarse budget
        # or any of the seven learned alignment phases that follow.
        handoff_available=self._open_coarse_scope()
        handoff_attempted=False
        def fine_support(frames):
            return {r:{axis:predict_stage(self.stage_models[r][axis],
                        frames[r]['own_bytes'],frames[r]['top_bytes'])
                       for axis in ('yaw','lateral','forward')} for r in ROBOTS}
        def supported(predictions):
            return (predictions is not None and all(
                predictions[r][axis].get('ok') is True
                and predictions[r][axis].get('precision')=='fine'
                for r in ROBOTS for axis in ('yaw','lateral','forward')))
        for index in range(120):
            def predict_coarse(frames, *, check_fine=False, require_not_ready=True):
                candidate=(copy.deepcopy(self.coarse) if getattr(self.io,'realtime_control',False)
                           else self.coarse)
                raw=(frames['r1']['raw_top_bytes'] if frames else
                     self.io.last_frames['r1']['top_bytes'])
                decisions={r:(candidate.decide(raw,r)
                              if candidate is not None else
                              coarse_approach(frames[r]['top_bytes'],self.reference,r))
                           for r in ROBOTS}
                predictions=(fine_support(frames) if check_fine and _handoff_gaps(decisions) is not None
                             and (not require_not_ready or not all(d['ready'] for d in decisions.values()))
                             else None)
                return decisions,candidate,predictions
            frames,(decisions,candidate,predictions)=self.observe_and_compute(
                'coarse',lambda frames:predict_coarse(
                    frames,check_fine=handoff_available and not handoff_attempted))
            if self.coarse is not None:self.coarse=candidate
            unresolved=[r for r in ROBOTS if decisions[r].get('ok') is not True]
            gaps=_handoff_gaps(decisions)
            config=getattr(self.io,'config',{})
            audit=coarse_command_audit(
                index,frames,decisions,self.bindings,
                source_sha=getattr(self.io,'source_sha',None),
                map_sha=config.get('static_map_sha256') if isinstance(config,dict) else None)
            if unresolved:
                if not (handoff_available and not handoff_attempted
                        and gaps is not None and supported(predictions)):
                    held={r:dict(forward=0.,left=0.,turn=0.) for r in ROBOTS}
                    audit['coordinated_commands']=held
                    audit['coordination']={'unresolved_heading':unresolved,
                                           'reason':'coarse_rgb_model_convention_unresolved'}
                    self.calls.append({'kind':'coarse','decisions':decisions,
                                       'commands':held,'coordination':audit['coordination']})
                    self.calls.append(audit)
                    self._hold_pair()
                    self._record_coarse_hold(audit,'policy_unresolved_held')
                    raise RuntimeError('coarse RGB model convention unresolved')
                # The coarse heading has no motion authority. Hold both while
                # only the independent fine models can admit a stopped handoff.
                commands={r:dict(forward=0.,left=0.,turn=0.) for r in ROBOTS}
                coordination={'unresolved_heading':unresolved,'held_forward':list(ROBOTS),
                              'forward_gap_px':{r:gaps[r]*960 for r in ROBOTS}}
            else:
                concurrent=getattr(self,'coarse_concurrent_alignment',False)
                if concurrent and (not self._concurrent_coarse_scope() or self.coarse is None):
                    commands={r:dict(forward=0.,left=0.,turn=0.) for r in ROBOTS}
                    coordination={'coarse_concurrent_alignment':True,'admitted':False,
                                  'reason':'coarse_scope_or_own_identity_unresolved',
                                  'held_forward':list(ROBOTS),'lead_limit_px':COARSE_LEAD_LIMIT_PX}
                else:
                    commands,coordination=coordinated_coarse_commands(
                        decisions,concurrent_alignment=concurrent,
                        frames=frames if concurrent else None,
                        now_s=self.time() if concurrent else None)
            report['coarse_calls'].append(decisions)
            self.calls.append({'kind':'coarse','decisions':decisions,'commands':commands,
                               'coordination':coordination})
            audit['coordinated_commands']=copy.deepcopy(commands)
            audit['coordination']=copy.deepcopy(coordination)
            self.calls.append(audit)
            if predictions is not None and supported(predictions):
                handoff_attempted=True
                initial={'frame_ids':{r:frames[r]['frame_id'] for r in ROBOTS},
                         'observed_at_s':{r:frames[r]['observed_at_s'] for r in ROBOTS},
                         'images':{r:{'own':frames[r]['own_rgb'],'top':frames[r]['shared_top_rgb']}
                                   for r in ROBOTS},
                         'coarse_decisions':decisions,'forward_gaps':gaps,
                         'unresolved_heading_slots':unresolved,
                         'fine_predictions':predictions}
                self.stop_dwell(coarse_audit=audit)
                stationary,(checks,checked_coarse,checked_fine)=self.observe_and_compute(
                    'coarse-fine-handoff-stationary',
                    lambda frames:predict_coarse(frames,check_fine=True,require_not_ready=False))
                fresh=_fresh_handoff_frames(frames,stationary)
                if fresh and self.coarse is not None:self.coarse=checked_coarse
                gaps=_handoff_gaps(checks)
                accepted=fresh and gaps is not None and supported(checked_fine)
                stationary_unresolved=[r for r in ROBOTS if checks[r].get('ok') is not True]
                evidence={'kind':'coarse_fine_handoff','coarse_index':index,
                          'reason':(('fine_supported_heading_unresolved_handoff'
                                     if unresolved or stationary_unresolved else 'fine_supported_handoff')
                                    if accepted else
                                    'stationary_rgb_not_fresh' if not fresh else
                                    'stationary_coarse_gap_or_signature_outside_band' if gaps is None else
                                    'stationary_fine_support_missing'),
                          'initial':initial,
                          'stationary':{'frame_ids':{r:stationary[r].get('frame_id') for r in ROBOTS},
                                        'observed_at_s':{r:stationary[r].get('observed_at_s') for r in ROBOTS},
                                        'images':{r:{'own':stationary[r].get('own_rgb'),
                                                     'top':stationary[r].get('shared_top_rgb')}
                                                  for r in ROBOTS},
                                        'fresh':fresh,'coarse_decisions':checks,
                                        'forward_gaps':gaps,
                                        'unresolved_heading_slots':stationary_unresolved,
                                        'fine_predictions':checked_fine}}
                self.calls.append(evidence)
                report['coarse_fine_handoff']=evidence
                if accepted:break
                if unresolved or stationary_unresolved:
                    self._hold_pair()
                    raise RuntimeError('coarse RGB model convention unresolved')
                continue
            issued=self.drive_mecanum(
                commands,coarse_observed_at_s=(frames['r1']['observed_at_s']
                    if concurrent and coordination.get('admitted') else None),
                coarse_audit=audit)
            if issued is False:
                coordination.update(admitted=False,reason='coarse_issue_ttl_expired',
                                    held_forward=list(ROBOTS))
                audit['coordination']=copy.deepcopy(coordination)
                for r in ROBOTS:commands[r]=dict(forward=0.,left=0.,turn=0.)
            if not unresolved and issued is not False:
                self._accept_coarse_pending(frames,decisions,commands)
            if all(d['ready'] for d in decisions.values()):self.stop_dwell();break
        else:
            self._clear_approach_pending(hold=True)
            raise RuntimeError('coarse RGB approach budget exhausted')
        report.update(run_approach(self,self.stage_models,reacquire_on_settle=True,
                                   final_refinement_steps=40,invalid_reobserve_budget=1,
                                   fine_gain_schedule=getattr(self,'fine_gain_schedule',False)))
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
