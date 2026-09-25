#!/usr/bin/env python3
"""Actual existing RGB skill integration for the three-peer dispatch mission.

Shared physics interleaves the solo macro executor while the pair executes its
existing staged approach/grasp code. Evaluation is a separate output sink.
"""
from __future__ import annotations
import base64
import copy
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import hashlib
import json
import math
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
from harness.dispatch_plan import build_dispatch_request, validate_dispatch_plan, validate_dispatch_reply, navigation_mode
from harness.dispatch_skill_binding import SkillBindings, ImageRoute
from harness.dispatch_skill_binding import (_CARRIER_RELINK_MAX_OCCLUDED_FRAMES,
    _CARRIER_RELINK_MAX_ELAPSED_S,_CARRIER_RELINK_MAX_MOTION_PX)
from harness.dispatch_feasibility import negotiate_executable
from harness.dispatch_yield import SoloYield,WheelObserver
from harness.dispatch_goto import MapGoToYield
from harness.dispatch_plan_guidance import PlanGuidance
from harness.three_robot_plan import ROBOTS, TeamAgreement, images
from harness.solo_box_transport import SoloBoxTransport, normalize_own_rgb
from harness.grasp_student_inference import predict_student
from harness.visual_macro_runtime import VisualMacroExecutor
from harness.rolling_visual_servo import (FINAL_ENTRY_HANDOFF_M,
    FINAL_ENTRY_SETTLE_S, NEAR_MOTION_REOBSERVE_BUFFER_M,
    SettledViewRecovery, RollingApproachLease, approach_drive_support,
    guard_near_entry_from_issued_commands, VIEW_RECOVERY_MAX_POSES,
    VIEW_RECOVERY_MAX_ELAPSED_S, VIEW_RECOVERY_MAX_EPISODES,
    VIEW_RECOVERY_WRIST_STEP_PWM, VIEW_RECOVERY_MAX_PAN_DELTA_PWM)
from sim.research_dispatch_arena import episode, actor_task
from scripts.research_dispatch_scene import DispatchScene
from scripts.run_dispatch_e2e import Referee
from scripts.run_research_dispatch import opaque_run_id
from scripts.three_robot_runtime import ThreeRobotRuntime, write
from scripts.run_camera_approach_student import models, sha
from scripts.run_camera_varied_start_student import load_stage_models
from scripts.run_three_robot_mission import prepare_grasp_models
from scripts.dispatch_pair_skill import (BoundPairSkill, COARSE_LEAD_LIMIT_PX,
    COARSE_CONCURRENT_MAX_CAPTURE_AGE_S, COARSE_CONCURRENT_COMMAND_S)
from scripts.camera_approach_scene import image_record

RGB_ACTION_TTL_S=.6
REALTIME_MOTOR_RENEWAL_S=.25
REALTIME_CAPTURE_OVERLAP_S=.02
SOLO_ACTIVE_SIM_BUDGET_S=300.
# Dynamic box retry: reverse slices after the fold. E3/V3 (v51/v56): 10
# slices moved r2 ~8 cm and left the box ~0.25 m away, below the folded
# camera's view, so the fresh search never saw it.
BOX_RETRY_REVERSE_SLICES=20


def fast_servo_map_supported(static_map, *, realtime_control):
    boundaries={'wall_north','wall_south','wall_west','wall_east'}
    return bool(realtime_control and static_map.get('map_id')=='dispatch_open'
                and not static_map.get('terrain')
                and all(item.get('id') in boundaries for item in static_map.get('obstacles',[])))


def coarse_concurrency_status(pair, *, requested):
    if not requested:
        return {'applied':False,'reason':'disabled'}
    if pair.coarse is None:
        return {'applied':False,'reason':'own_motion_identity_unresolved'}
    if not pair._concurrent_coarse_scope():
        return {'applied':False,'reason':'unsupported_map_or_runtime_scope'}
    return {'applied':True,'reason':'eligible_for_rgb_decision_admission'}


def port_issue_receipt(applied,port,action,*,bounded):
    """Freeze only the returned port acknowledgement before physics advances."""
    state=applied.get('actuator_state') if isinstance(applied,dict) else None
    pwm=state.get('motor_commands') if isinstance(state,dict) else None
    issued=applied.get('sim_time') if isinstance(applied,dict) else None
    expiry=getattr(port,'_command_expires_at' if bounded else '_drive_expires_at',None)
    acknowledged=(type(issued) in (int,float) and math.isfinite(issued)
                  and type(expiry) in (int,float) and math.isfinite(expiry) and expiry>=issued
                  and isinstance(pwm,(list,tuple)) and len(pwm)==4
                  and all(type(v) in (int,float) and math.isfinite(v) for v in pwm))
    return {'acknowledged':acknowledged,
            'actual_issued_at_s':issued if acknowledged else None,
            'effective_expiry_s':expiry if acknowledged else None,
            'port_ack_motor_pwm':list(pwm) if acknowledged else None,
            'applied_action':copy.deepcopy(action) if acknowledged else None}


def _solo_skill(scene,fast_servo,*,search_turn=.12):
    return SoloBoxTransport(robot_id=scene.bindings.solo,search_turn=search_turn,
        navigator=ImageRoute(scene.bindings,'box',time_aware_box_reacquisition=fast_servo,
                             bounded_carrier_relink=getattr(scene,'bounded_carrier_relink',False)),
        attachment_min_saturation=150,release_refine_ground_fit=True,
        fast_near_field_servo=fast_servo)


class SkillScene(DispatchScene):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.video=self.referee=self.bindings=self.team=None
        self.solo=self.solo_executor=None;self.solo_lease=0.
        self.solo_rows=[];self.solo_raw=[];self.next_sample=0.;self.pair_phase='SETUP'
        self.yield_rows=[];self.yield_policy=None;self.yield_folded=False
        self.identity=None
        self.original_step=None;self.deadline=None;self.last_frames=None
        self.native_view=None
        self.replay=None
        self.efficient_capture=False
        self.realtime_control=False
        self.coarse_concurrent_alignment=False
        self.rolling_visual_servo=False
        self.bounded_carrier_relink=False
        self.rolling_view_recovery=False
        self._view_recovery=None
        self._decision_workers=None
        self._solo_pending=None
        self._solo_pending_lease=None
        self._solo_approach_lease=None
        self._solo_approach_last_lease=None
        self._solo_approach_last_source=None
        self.solo_renewals=[]
        self._yield_pending=None
        self._solo_motion_accept_after_s=0.
        self._solo_phase_label='SETUP'
        self._contact_audit=None
        self.realtime_stats={'solo_backpressure':0,'solo_stale_rgb':0,
                             'solo_decisions':0,'pair_backpressure':0,
                             'pair_decisions':0,'max_decision_age_s':0.,
                             'solo_total_sim_s':0.,'solo_active_sim_s':0.,
                             'solo_resource_wait_sim_s':0.,
                             'solo_resource_waiting':False,
                             'solo_resource_wait_intervals':0,
                             'solo_active_budget_s':SOLO_ACTIVE_SIM_BUDGET_S}
        self._solo_retry_at=0.
        self._solo_budget_last_s=None
        self._solo_budget_active_s=0.
        self._solo_budget_wait_s=0.
        self._solo_resource_waiting=False
        self._solo_resource_wait_reason=None
        self._solo_budget_ended=False
        self._in_physics=False
        self.realtime_stats.update(pair_stage_stale_rgb=0,pair_stage_samples=0)
        # Dynamic coordination only (synchronous): a box failure pauses the box
        # robot; the team decides outside the physics callback (see step()).
        self.team_event_handler=None
        self.diagnostic_fail_box_once=False
        self._solo_event=None
        self._solo_recovery=None
        self.solo_dropped=False
        self.solo_events=[]

    def time(self):return float(self.world.data.time)
    def open(self):
        super().open()
        if self.realtime_control:
            from sim.contact_audit_kernel import ContactAuditKernel
            from sim.physics_drive_kernel import PhysicsDriveKernel
            self._contact_audit=ContactAuditKernel(
                self.world.model.ngeom,self.robot_ids,self.obstacle_ids)
            self.world._fast_drive_kernel=PhysicsDriveKernel(self.world)
            # Build the extra frozen render context before the first control
            # decision, not while the pair is carrying a live payload.
            self.world.render_snapshot_async([(None,'cctv_top')]).result(timeout=30.)
        self.original_step=self.world._physics_step_for
        self.world._physics_step_for=self._physics
        return self
    def _physics(self,active,commands=None):
        if getattr(self,'_in_physics',False):raise RuntimeError('recursive physics owner step')
        self._in_physics=True
        try:return self._physics_owned(active,commands)
        finally:self._in_physics=False

    def _physics_owned(self,active,commands=None):
        if self.native_view:self.native_view.tick()
        try:
            if self.realtime_control:self._solo_tick_realtime()
            else:self._solo_tick()
        except Exception as exc:
            from harness.skill_errors import ComponentExecutionError
            raise ComponentExecutionError('solo_box',exc) from exc
        now=self.time()
        for p in self.ports.values():p.tick(now)
        self.original_step(active,commands)
        self.physics_steps+=1;self.weld_steps+=bool(self.world.data.eq_active.any())
        if self._contact_audit is not None:
            self.obstacle_contact_steps+=self._contact_audit.has_penetrating_contact(self.world.data)
        else:
            self.obstacle_contact_steps+=any(
                (int(c.geom1) in self.robot_ids and int(c.geom2) in self.obstacle_ids)
                or (int(c.geom2) in self.robot_ids and int(c.geom1) in self.obstacle_ids)
                for c in self.world.data.contact[:self.world.data.ncon] if c.dist<0)
        if self.video:
            self.video.stage='PAIR '+self.pair_phase+' | '+(self.bindings.solo+' '+
                (self._solo_phase_label if self.realtime_control else self.solo.phase) if self.solo else 'SETUP/END')
            self.video.capture()
        # Observer-only post-run replay; never read by actors or the referee.
        replay=getattr(self,'replay',None)
        if replay:replay.sample(self.video.stage if self.video else None)
        if self.referee and self.time()+1e-9>=self.next_sample:
            self.referee.sample();self.next_sample=self.time()+.1
    def step(self,seconds):
        for _ in range(round(seconds/self.world.model.opt.timestep)):
            if self.deadline and time.monotonic()>self.deadline:raise RuntimeError('skill wall budget exhausted')
            self.world._physics_step_for(self.world.controllers['r1'])
            event=getattr(self,'_solo_event',None)
            if event is not None and not event.get('handling'):
                self._handle_solo_event()
    def _handle_solo_event(self):
        """Team decision on a paused box failure, outside the physics callback."""
        event=self._solo_event;event['handling']=True
        saved=getattr(self,'last_frames',None)
        try:decision=self.team_event_handler(event)
        finally:self.last_frames=saved
        self.solo_events.append({**{k:v for k,v in event.items() if k!='handling'},'decision':decision})
        if decision=='retry':
            self._solo_recovery=[{'kind':'pose','pulses':{1:2000,3:740,4:2320,5:1320,6:1500}}]+[
                {'kind':'mecanum','forward':-.05,'left':0.,'turn':0.,'duration_s':.2}]*BOX_RETRY_REVERSE_SLICES
            # The fresh skill first searches toward the side where the failed
            # skill last saw the box in its own RGB (robot frame, +y = left).
            seen=getattr(getattr(getattr(self,'solo',None),'box',None),'last_target',None)
            self._solo_search_turn=-.12 if seen is not None and seen[1]<0 else .12
        elif decision=='dropped':self.solo_dropped=True
        else:raise RuntimeError(f"box stopped: {event['reason']}; team decided to stop all")
        self._solo_event=None
    def _solo_failed(self,reason,phase,now,*,injected=False):
        self.solo_executor.cancel(now,'team_recovery');self.ports[self.bindings.solo].hold(now)
        self._solo_event={'reason':str(reason),'phase':phase,'sim_time_s':now,
                          'diagnostic_injection':injected}
    def _solo_recovery_tick(self,now):
        """Issued fold pose then a short reverse; then a fresh box skill."""
        if not self.solo_executor.idle or now<self.solo_lease:return
        if not self._solo_recovery:
            self.solo=_solo_skill(self,fast_servo_map_supported(
                self.bindings.static_map,realtime_control=self.realtime_control),
                search_turn=getattr(self,'_solo_search_turn',.12))
            self._solo_recovery=None
            self._solo_phase_label=self.solo.phase;return
        action=self._solo_recovery.pop(0)
        if action['kind']=='pose':
            self.solo_executor.submit(action,self.ports[self.bindings.solo].capture(),'recovery_fold',now)
        else:
            self.raw(self.bindings.solo,action,'RECOVERY');self.solo_lease=now+action['duration_s']
    def hold(self):
        self._solo_pending_lease=None
        if getattr(self,'_solo_approach_lease',None) is not None:
            self._clear_solo_approach_lease(self.time(),'scene_hold')
        for p in self.ports.values():p.hold(self.time())
    def capture(self,label,*,own_robots=None,overview=True):
        if not self.efficient_capture:own_robots,overview=None,True
        self.last_frames=super().capture(label,own_robots=own_robots,overview=overview)
        return self.last_frames

    def await_visual(self,future):
        """Pump the single physics owner while one bounded visual job runs."""
        if getattr(self,'_in_physics',False):raise RuntimeError('visual wait inside physics callback')
        while not future.done():self.step(.02)
        return future.result()

    def compute_visual(self,fn):
        """Run only RGB/model arithmetic off owner; never issue actuators here."""
        if not self.realtime_control:return fn()
        if self._decision_workers is None:
            raise RuntimeError('realtime visual workers not started')
        return self.await_visual(self._decision_workers.submit(fn))
    def authorize(self):
        if self.bindings:self.bindings.authorize(self.team.agreement.committed)
    def raw(self,rid,action,stage):
        self.authorize();self.ports[rid].validate_action(action)
        applied=self.ports[rid].apply(action,self.time())
        self.command_history[rid].append({'stage':stage,'action':copy.deepcopy(action),
            'issued_at_s':(applied['sim_time'] if isinstance(applied,dict)
                           and 'sim_time' in applied else self.time()),
            'plan_hash':self.bindings.committed['plan_hash'] if self.bindings else None})
        return applied
    def _permission_snapshot(self):
        """A private permission state for RGB workers and owner preflight."""
        snapshot=copy.copy(self.bindings)
        snapshot.locks=dict(self.bindings.locks)
        snapshot.finished=set(self.bindings.finished)
        snapshot.dropped=set(getattr(self.bindings,'dropped',()))
        snapshot.grasp_started=set(self.bindings.grasp_started)
        snapshot.transit_started=set(self.bindings.transit_started)
        snapshot.resource_events=[]
        return snapshot
    def _commit_permissions(self,requests):
        """Check every requested gate without side effects, then acquire on owner."""
        requests=list(dict.fromkeys(requests))
        probe=self._permission_snapshot()
        if not all(probe.permission(obj,stage) for obj,stage in requests):return False
        for obj,stage in requests:
            if not self.bindings.permission(obj,stage):
                raise RuntimeError('resource permission changed during owner commit')
        return True
    def _issue_realtime_motor(self,rid,action,stage,observed_at_s,*,now,
                              max_duration_s=None,source_evidence=None):
        """Owner-only bounded motor renewal from one fresh RGB instant."""
        requested=float(action['duration_s'])
        renewal=REALTIME_MOTOR_RENEWAL_S
        effective=min(renewal,float(observed_at_s)+RGB_ACTION_TTL_S-now)
        if max_duration_s is not None:effective=min(effective,float(max_duration_s))
        if effective<=0:
            self.ports[rid].hold(now)
            return None
        issued={**action,'duration_s':effective}
        self.authorize();self.ports[rid].validate_bounded(issued,effective)
        self.ports[rid].apply_bounded(issued,now,effective)
        self.command_history[rid].append({'stage':stage,'action':copy.deepcopy(issued),
            'issued_at_s':now,'valid_until_s':now+effective,
            'observed_at_s':float(observed_at_s),'requested_duration_s':requested,
            'renewal_request_s':renewal,
            'plan_hash':self.bindings.committed['plan_hash']})
        if source_evidence is not None:
            self.command_history[rid][-1]['source_rgb_evidence']=copy.deepcopy(source_evidence)
        return effective
    def _solo_budget_tick(self,now,*,enforce=True):
        """Charge every owner SIM interval to active work or a denied gate."""
        started=getattr(self,'solo_started',None)
        if started is None or getattr(self,'_solo_budget_ended',False):return
        if not math.isfinite(now) or now<started-1e-9:
            raise RuntimeError('solo SIM clock invalid or moved backwards')
        previous=getattr(self,'_solo_budget_last_s',None)
        if previous is None:previous=started
        if now<previous-1e-9:
            raise RuntimeError('solo SIM clock moved backwards')
        elapsed=max(0.,now-previous)
        if getattr(self,'_solo_resource_waiting',False):
            self._solo_budget_wait_s=getattr(self,'_solo_budget_wait_s',0.)+elapsed
            self._solo_budget_active_s=getattr(self,'_solo_budget_active_s',0.)
        else:
            self._solo_budget_active_s=getattr(self,'_solo_budget_active_s',0.)+elapsed
            self._solo_budget_wait_s=getattr(self,'_solo_budget_wait_s',0.)
        self._solo_budget_last_s=now
        self.realtime_stats.update(solo_total_sim_s=now-started,
            solo_active_sim_s=self._solo_budget_active_s,
            solo_resource_wait_sim_s=self._solo_budget_wait_s,
            solo_resource_waiting=getattr(self,'_solo_resource_waiting',False),
            solo_active_budget_s=SOLO_ACTIVE_SIM_BUDGET_S)
        if enforce and self._solo_budget_active_s>SOLO_ACTIVE_SIM_BUDGET_S:
            raise RuntimeError('solo active SIM budget exhausted')
    def _solo_set_resource_wait(self,now,waiting,reason=None):
        self._solo_budget_tick(now)
        previous=getattr(self,'_solo_resource_waiting',False)
        self._solo_resource_waiting=bool(waiting)
        self._solo_resource_wait_reason=reason if waiting else None
        if waiting and not previous:
            self.realtime_stats['solo_resource_wait_intervals']=(
                self.realtime_stats.get('solo_resource_wait_intervals',0)+1)
        self.realtime_stats['solo_resource_waiting']=bool(waiting)
    def _solo_finish_budget(self,now):
        self._solo_budget_tick(now,enforce=False)
        self._solo_budget_ended=True
        self._solo_resource_waiting=False
        self._solo_resource_wait_reason=None
        self.realtime_stats['solo_resource_waiting']=False
    def _adopt_waiting_carry_visual(self,candidate,action,evidence,observation):
        """Advance only a proven RGB guard frame while an unload gate is closed."""
        if (self.solo.phase!='carry' or candidate.phase!='carry'
                or not evidence.get('waiting_for_resource')
                or action['kind']!='mecanum'
                or any(abs(action[k])>1e-9 for k in ('forward','left','turn'))):
            return False
        current=self.solo.box;checked=candidate.box
        if (not hasattr(current,'_carry_previous_image')
                or checked._carry_previous_image!=observation['image']
                or not (checked.last_attachment or {}).get('attached')
                or checked._last_frame_id!=observation['frame_id']
                or checked._last_frame_id<=current._last_frame_id
                or checked._last_sim_time!=observation['sim_time']
                or checked._last_sim_time<current._last_sim_time):
            return False
        # VisualBoxSkill updates _carry_previous_image only after its strict
        # pairwise, absolute anchor, and ground checks pass. Its navigation
        # candidate, policy step and any requested locks are never adopted.
        for field in ('_carry_previous_image','last_attachment',
                      '_surface_drop_probe_validated','_last_frame_id',
                      '_last_sim_time','_hashes','_history','last_box',
                      'last_target','last_target_provenance','last_surface'):
            setattr(current,field,copy.deepcopy(getattr(checked,field)))
        return True
    def pair_drive(self,commands,duration,stage):
        self.authorize();self.pair_phase=stage
        if set(commands)!=set(self.bindings.pair.values()):raise ValueError('pair endpoint mismatch')
        for r,a in commands.items():self.ports[r].validate_action(a)
        if stage=='TRANSIT' and any(any(abs(a.get(k,0.))>0 for k in ('forward','left','turn')) for a in commands.values()):
            self.bindings.note_transit_command('beam')
        receipts={}
        for r,a in commands.items():
            applied=self.raw(r,a,stage)
            receipts[r]=port_issue_receipt(applied,self.ports[r],a,bounded=False)
        self.step(duration)
        return receipts
    def pair_issue_bounded(self,commands,duration,stage):
        """Issue one atomic pair lease; physics remains on the calling owner."""
        self.authorize();self.pair_phase=stage
        if set(commands)!=set(self.bindings.pair.values()):raise ValueError('pair endpoint mismatch')
        if not 0.<duration<=.25:raise ValueError('bounded pair duration required')
        for r,a in commands.items():self.ports[r].validate_bounded(a,duration)
        if stage=='TRANSIT' and any(any(abs(a.get(k,0.))>0 for k in ('forward','left','turn')) for a in commands.values()):
            self.bindings.note_transit_command('beam')
        now=self.time()
        receipts={}
        try:
            for r,a in commands.items():
                applied=self.ports[r].apply_bounded(a,now,duration)
                self.command_history[r].append({'stage':stage,'action':copy.deepcopy(a),
                    'issued_at_s':now,'valid_until_s':now+duration,
                    'plan_hash':self.bindings.committed['plan_hash']})
                receipts[r]=port_issue_receipt(applied,self.ports[r],a,bounded=True)
        except BaseException:
            for r in commands:self.ports[r].hold(now)
            raise
        return receipts
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
        authored=self.bindings.static_map
        fast_servo=fast_servo_map_supported(authored,realtime_control=self.realtime_control)
        if getattr(self,'rolling_visual_servo',False) and not fast_servo:
            raise ValueError('rolling visual servo requires realtime dispatch_open without internal obstacles')
        if getattr(self,'bounded_carrier_relink',False) and not fast_servo:
            raise ValueError('bounded carrier relink requires realtime dispatch_open without internal obstacles')
        if getattr(self,'rolling_view_recovery',False) and not fast_servo:
            raise ValueError('rolling view recovery requires realtime dispatch_open without internal obstacles')
        self.solo=_solo_skill(self,fast_servo)
        self.solo_executor=VisualMacroExecutor(self.ports[self.bindings.solo],
            log_callback=self.solo_raw.append,drive_settle_by_phase={'carry':0.})
        self.solo_started=None
        self._solo_budget_last_s=None
        self._solo_budget_active_s=self._solo_budget_wait_s=0.
        self._solo_resource_waiting=False
        self._solo_resource_wait_reason=None
        self._solo_budget_ended=False
        self._solo_phase_label=self.solo.phase
        self._solo_motion_accept_after_s=0.
        self._solo_pending_lease=None
        self._solo_approach_lease=None
        self._solo_approach_last_lease=None
        self._solo_approach_last_source=None
        self._view_recovery=SettledViewRecovery() if getattr(self,'rolling_view_recovery',False) else None
        self._solo_pose_log_cursor=0
        self._solo_completed_pose_receipts={}
        self.solo_renewals=[]
        if self.realtime_control:
            self._decision_workers=ThreadPoolExecutor(max_workers=2,thread_name_prefix='dispatch-decision')
    def other_robot_observation(self,top):
        if self.yield_policy:
            return self.yield_policy.vision.observe(top),{'source':'solo_yield_history'}
        frame=cv2.imdecode(np.frombuffer(top,np.uint8),cv2.IMREAD_COLOR)
        claim=self.identity[self.bindings.solo]['claim']
        if not claim['valid']:raise RuntimeError('waiting robot identity unresolved')
        hint=np.array(claim['center'])*[frame.shape[1]-1,frame.shape[0]-1]
        return WheelObserver(self.bindings.static_map,hint).observe(top),{'source':'own_identity_probe','hint_px':hint.tolist()}
    def _new_yield_policy(self,top):
        if getattr(self.bindings,'navigation','authored')!='planned':
            return SoloYield(self.bindings.static_map,self.solo.navigator.box_center)
        # The models chose the park place; the feasibility verdict is reused.
        planned=self.bindings.box_route or {}
        return MapGoToYield(self.bindings.static_map,self.bindings.plan,self.solo.navigator.box_center,
            top_jpeg=top,beam_center=self.bindings.planning_beam_center,destination=planned.get('park'))
    def _yield_tick(self,now):
        if self.bindings.settled('beam'):
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
                self.yield_policy=self._new_yield_policy(top)
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
        if self.bindings.settled('box') or getattr(self,'_solo_event',None) is not None:return
        if getattr(self,'_solo_recovery',None) is not None:
            self._solo_recovery_tick(now);return
        if not self.solo_executor.idle or now<self.solo_lease:return
        if self.solo.done:
            self._yield_tick(now);return
        stage='APPROACH' if self.solo.phase=='approach' else 'TRANSIT' if self.solo.phase=='carry' else 'GRASP'
        if not self.bindings.permission('box',stage):return
        if self.solo_started is None:self.solo_started=now
        if now-self.solo_started>300:raise RuntimeError('solo SIM budget exhausted')
        obs=self.ports[self.bindings.solo].capture()
        native=base64.b64decode(obs['image'])
        obs, input_transform=normalize_own_rgb(obs)
        own=base64.b64decode(obs['image'],validate=True)
        top=self.world.render_team_jpeg(camera='cctv_top',quality=95)
        index=len(self.solo_rows)
        refs={'raw_own':image_record(self.out/'rgb'/f'solo-{index}-raw-own.jpg',self.out,native),
              'own':image_record(self.out/'rgb'/f'solo-{index}-own.jpg',self.out,own),
              'top':image_record(self.out/'rgb'/f'solo-{index}-top.jpg',self.out,top)}
        before=self.solo.phase
        approach_state=copy.deepcopy(self.solo.box) if before=='approach' else None
        action,evidence=self.solo.decide(obs,top)
        injected=False
        if (getattr(self,'diagnostic_fail_box_once',False) and before=='approach'
                and self.solo.phase not in ('approach','finished')):
            # Output-only diagnostic: the robots see the real stop reason.
            self.diagnostic_fail_box_once=False;injected=True
            action={'kind':'finish','reason':'APPROACH_OVERSHOT'}
            evidence={**evidence,'diagnostic_injection':True}
        if evidence.get('waiting_for_resource'):self.solo.steps-=1
        if (not injected and before=='approach' and self.solo.phase=='lower'
                and not self.bindings.permission('box','GRASP')):
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
            'input_transform':input_transform,
            'phase_before':before,'phase_after':self.solo.phase,'observation':{k:v for k,v in obs.items() if k!='image'},
            'images':refs,'action':action,'top_evidence':evidence,
            'own_attachment_evidence':copy.deepcopy(self.solo.box.last_attachment),
                'approach_adjustment':copy.deepcopy(getattr(self.solo.box,'last_approach_adjustment',None))})
        if self.video:self.video.stage='PAIR '+self.pair_phase+' | '+self.bindings.solo+' '+self.solo.phase
        if injected:
            self._solo_failed('APPROACH_OVERSHOT',before,now,injected=True);return
        if (self.solo.done and self.solo.reason!='VISUAL_RELEASE_CONFIRMED'
                and getattr(self,'team_event_handler',None) is not None):
            self._solo_failed(self.solo.reason,before,now);return
        if action['kind']=='mecanum':
            self.raw(self.bindings.solo,action,'TRANSIT');self.solo_lease=now+action['duration_s']
        else:self.solo_executor.submit(action,obs,self.solo.phase,now)
        if index%25==0 or before!=self.solo.phase:
            print(json.dumps({'solo_step':index,'robot':self.bindings.solo,'phase':self.solo.phase,'action':action}),flush=True)
        if self.solo.done:
            if self.solo.reason!='VISUAL_RELEASE_CONFIRMED':raise RuntimeError('solo stopped: '+str(self.solo.reason))
            if self.bindings.settled('beam'):self.bindings.finish('box')

    def _clear_solo_pending_lease(self,now,*,hold=False):
        lease=getattr(self,'_solo_pending_lease',None)
        self._solo_pending_lease=None
        if hold and lease is not None:self.ports[self.bindings.solo].hold(now)

    def _solo_port_lease_matches(self,lease):
        """Inspect issued port state only; never infer motion from measured joints."""
        port=self.ports[self.bindings.solo]
        if not hasattr(port,'_command_expires_at'):return False
        return (getattr(port,'_command_expires_at',None)==lease['valid_until_s']
                and getattr(port,'_drive_expires_at',None)==lease['valid_until_s']
                and tuple(getattr(port,'_motor_commands',()))==lease['motor_commands'])

    def _renew_solo_pending_lease(self,now,*,reason):
        """Owner-only replay of the last accepted carry action while RGB is pending."""
        lease=getattr(self,'_solo_pending_lease',None)
        if lease is None:return
        deadline=lease['observed_at_s']+RGB_ACTION_TTL_S
        if (self.solo.phase!='carry' or self.solo.done
                or self.bindings.tasks['box']['id'] in self.bindings.finished
                or now>=lease['valid_until_s']-1e-9 or now>=deadline-1e-9
                or not self._solo_port_lease_matches(lease)):
            self._clear_solo_pending_lease(now,hold=True)
            return
        nearing_expiry=now+REALTIME_CAPTURE_OVERLAP_S>=lease['valid_until_s']-1e-9
        if not nearing_expiry and now<lease['next_permission_check_s']:
            return
        self.authorize()
        if self.bindings.committed['plan_hash']!=lease['plan_hash']:
            self._clear_solo_pending_lease(now,hold=True)
            raise RuntimeError('pending solo plan changed')
        probe=self._permission_snapshot()
        requests=lease['permission_requests']
        if not all(probe.permission(obj,stage) for obj,stage in requests):
            self._clear_solo_pending_lease(now,hold=True)
            return
        lease['next_permission_check_s']=now+REALTIME_CAPTURE_OVERLAP_S
        if not nearing_expiry:return
        if lease['valid_until_s']>=deadline-1e-9:return
        # A copy-only probe must not invent a live lane or apron reservation.
        if not self._commit_permissions(requests):
            self._clear_solo_pending_lease(now,hold=True)
            return
        prior_until=lease['valid_until_s']
        effective=self._issue_realtime_motor(self.bindings.solo,lease['action'],
                                             'TRANSIT',lease['observed_at_s'],now=now)
        if effective is None:
            self._clear_solo_pending_lease(now,hold=True)
            return
        lease['valid_until_s']=now+effective
        port=self.ports[self.bindings.solo]
        if hasattr(port,'_motor_commands'):
            lease['motor_commands']=tuple(port._motor_commands)
        receipt={'kind':'solo_carry_pending_renewal',
                 'source_frame_id':lease['frame_id'],
                 'source_observed_at_s':lease['observed_at_s'],
                 'source_decision_sim_time_s':lease['decision_sim_time_s'],
                 'original_rgb_deadline_s':deadline,
                 'previous_valid_until_s':prior_until,
                 'issued_at_s':now,'valid_until_s':now+effective,
                 'duration_s':effective,'plan_hash':lease['plan_hash'],
                 'permission_requests':list(requests),
                 'permission_verified_at_s':now,
                 'route_overlap':bool(getattr(self.bindings,'route_overlap',False)),
                 'apron_permission_required':('box','UNLOAD') in requests,
                 'phase':self.solo.phase,'reason':reason}
        self.solo_renewals.append(receipt)
        self.command_history[self.bindings.solo][-1].update(
            pending_renewal=True,source_frame_id=lease['frame_id'],
            source_decision_sim_time_s=lease['decision_sim_time_s'],
            original_rgb_deadline_s=deadline,
            permission_requests=list(requests),
            permission_verified_at_s=now,
            apron_permission_required=('box','UNLOAD') in requests,
            renewal_reason=reason)

    def _clear_solo_approach_lease(self,now,reason):
        """Stop an issued rolling motor lease and retain its abort context."""
        lease=getattr(self,'_solo_approach_lease',None)
        self._solo_approach_lease=None
        if lease is None:return
        scheduled_windows=list(lease.issued_windows)
        lease.issued_windows=[(start,min(end,now)) for start,end in scheduled_windows
                              if start < now and min(end,now) > start]
        lease.hold_at_s=now
        self._solo_approach_last_lease=lease
        self.ports[self.bindings.solo].hold(now)
        self.solo_renewals.append({'kind':'solo_approach_rolling_stop',
            'time_s':now,'reason':reason,'source_frame_id':lease.evidence['source_frame_id'],
            'source_own_sha256':lease.evidence['source_own_sha256'],
            'source_top_sha256':lease.evidence['source_top_sha256'],
            'source_observed_at_s':lease.evidence['source_observed_at_s'],
            'original_horizon_s':lease.horizon_s,'rgb_deadline_s':lease.rgb_deadline_s,
            'last_valid_until_s':lease.valid_until_s,'plan_hash':lease.plan_hash,
            'scheduled_lease_windows':scheduled_windows,
            'effective_command_windows':list(lease.issued_windows),
            'hold_at_s':now,'issued_command_not_measured_motion':True})
        self.solo_raw.append({'event':'rolling_motor_hold','time':now,
            'reason':reason,'robot_id':self.bindings.solo,
            'source_frame_sha256':lease.evidence['source_own_sha256'],
            'source_top_sha256':lease.evidence['source_top_sha256'],
            'source_frame_id':lease.evidence['source_frame_id']})

    def _rolling_view_sample(self,result,pending,obs,now):
        """Only paired current RGB and the camera port's issued-command cache."""
        return {'frame_id':result['frame_id'],
                'capture_started_at_s':pending['capture_started_at_s'],
                'observed_at_s':result['observed_at_s'],'decision_at_s':now,
                'own_sha256':obs['sha256'],'top_sha256':result['top_sha256'],
                'paired_top_sha256':result['images']['top']['sha256'],
                'pose':obs['actuator_state']['servo_pulses'],
                'box':self.solo.box.last_box,'target':self.solo.box.last_target,
                'cargo_id':self.solo.box.cargo_id,
                'completed_pose':self.solo_executor.last_execution,
                'completed_pose_receipts':copy.deepcopy(self._solo_completed_pose_receipts),
                'phase':self.solo.phase,
                'plan_hash':self.bindings.committed['plan_hash']}

    def _refresh_completed_issued_pose_receipts(self):
        """Index completed own poses and every actually issued solo wheel command."""
        if getattr(self,'_view_recovery',None) is None:return
        for item in self.solo_raw[self._solo_pose_log_cursor:]:
            if item.get('robot_id')==self.bindings.solo and (
                    item.get('event')=='rolling_raw_action'
                    or (item.get('event')=='raw_action'
                        and (item.get('raw_action') or {}).get('kind') in ('drive','mecanum'))):
                self._view_recovery.note_wheel_issue()
            if item.get('event')!='macro_finished':continue
            macro=item.get('macro') or {}
            receipt=item.get('execution') or {}
            if macro.get('kind')!='pose' or receipt.get('status')!='completed':continue
            pulses=macro.get('pulses') or {}
            for servo in (3,6):
                if servo in pulses or str(servo) in pulses:
                    self._solo_completed_pose_receipts[str(servo)]=copy.deepcopy(receipt)
        self._solo_pose_log_cursor=len(self.solo_raw)

    def _issue_rolling_view_reobserve(self,event,row,now):
        """Hold first, then let the regular RGB worker capture after settling."""
        port=self.ports[self.bindings.solo]
        port.hold(now)
        if any(abs(value)>1e-12 for value in port._motor_commands):
            raise RuntimeError('rolling settled observation could not hold wheels')
        self._solo_retry_at=event['capture_not_before_s']
        self.solo_raw.append({'event':'rolling_view_recovery_settle_hold','time':now,
            'robot_id':self.bindings.solo,'source_frame_id':event['frame_id'],
            'source_frame_sha256':event['own_sha256'],
            'source_top_sha256':event['top_sha256'],
            'capture_not_before_s':event['capture_not_before_s'],
            'episode':event['episode'],'motor_commands':list(port._motor_commands)})
        row['view_recovery']=event
        row['issued_action']={'kind':'hold','motor_commands':list(port._motor_commands)}
        row['rolling_dropped']='hold_for_settled_rgb_instead_of_weak_drive'
        self.solo_rows.append(row)

    def _issue_rolling_view_pose(self,event,sample,obs,row,now):
        """Hold wheels before the existing interpolated pose/settle scheduler."""
        port=self.ports[self.bindings.solo]
        port.hold(now)
        if any(abs(value)>1e-12 for value in port._motor_commands):
            raise RuntimeError('rolling view recovery could not hold wheels')
        pose={'kind':'pose','pulses':event['issued_pose']}
        self.solo.box.lock_rolling_approach_view(
            int(event['issued_pose'].get('6',sample['pose']['6'])),
            math.hypot(*sample['target'][:2]))
        self.solo_raw.append({'event':'rolling_view_recovery_hold','time':now,
            'robot_id':self.bindings.solo,'source_frame_id':sample['frame_id'],
            'source_frame_sha256':sample['own_sha256'],
            'source_top_sha256':sample['top_sha256'],
            'motor_commands':list(port._motor_commands),
            'reason':event['reason']})
        self.solo_executor.submit(pose,obs,self.solo.phase,now)
        event['issued_at_s']=now
        event['macro_source_own_sha256']=obs['sha256']
        row['view_recovery']=event
        row['issued_action']=pose
        row['rolling_dropped']='active_view_pose_instead_of_weak_drive'
        self.solo_rows.append(row)

    def _fail_rolling_view(self,event,row,now):
        self.ports[self.bindings.solo].hold(now)
        row['view_recovery']=event
        row['rolling_dropped']=event['reason']
        self.solo_rows.append(row)
        self.solo_raw.append({'event':'rolling_view_recovery_stop','time':now,
            'robot_id':self.bindings.solo,'reason':event['reason'],
            'source_frame_id':event.get('frame_id'),
            'source_frame_sha256':event.get('own_sha256'),
            'motor_commands':list(self.ports[self.bindings.solo]._motor_commands)})
        raise RuntimeError('solo RGB active-view recovery failed: '+event['reason'])

    def _clear_rolling_view_recovery(self,now,reason):
        recovery=getattr(self,'_view_recovery',None)
        if recovery is None or recovery.episode is None:return
        recovery.episode=None
        self.ports[self.bindings.solo].hold(now)
        self.solo_raw.append({'event':'rolling_view_recovery_stop','time':now,
            'robot_id':self.bindings.solo,'reason':reason,
            'motor_commands':list(self.ports[self.bindings.solo]._motor_commands)})

    def _start_solo_approach_lease(self,action,support,now,requests):
        """Issue the first <=.25 s slice of exactly one RGB-selected drive."""
        raw={'kind':'drive','forward':float(action['fwd']),
             'turn':float(action['turn']),'duration_s':float(action['duration'])}
        horizon=now+support['original_duration_s']
        duration=min(REALTIME_MOTOR_RENEWAL_S,horizon-now,
                     support['source_observed_at_s']+RGB_ACTION_TTL_S-now)
        if duration<=0:return None
        effective=self._issue_realtime_motor(
            self.bindings.solo,raw,'APPROACH',support['source_observed_at_s'],
            now=now,max_duration_s=duration,source_evidence={
                **support,'decision_at_s':now,'original_horizon_s':horizon,
                'lease_kind':'rolling_approach'})
        if effective is None:return None
        port=self.ports[self.bindings.solo]
        lease=RollingApproachLease(
            action=raw,evidence=copy.deepcopy(support),decision_at_s=now,
            first_issued_at_s=now,plan_hash=self.bindings.committed['plan_hash'],
            valid_until_s=now+effective,
            motor_commands=tuple(getattr(port,'_motor_commands',())),
            permission_requests=tuple(dict.fromkeys(requests)),
            issued_windows=[(now,now+effective)])
        self._solo_approach_lease=lease
        self._solo_approach_last_source=(support['source_frame_id'],
            support['source_observed_at_s'],support['source_own_sha256'])
        self.solo_renewals.append({'kind':'solo_approach_rolling_issue',
            'issued_at_s':now,'valid_until_s':lease.valid_until_s,
            'source_frame_id':support['source_frame_id'],
            'source_own_sha256':support['source_own_sha256'],
            'source_top_sha256':support['source_top_sha256'],
            'source_observed_at_s':support['source_observed_at_s'],
            'original_horizon_s':lease.horizon_s,'rgb_deadline_s':lease.rgb_deadline_s,
            'plan_hash':lease.plan_hash,'permission_requests':list(lease.permission_requests)})
        self.solo_raw.append({'event':'rolling_raw_action','time':now,
            'robot_id':self.bindings.solo,
            'source_frame_sha256':support['source_own_sha256'],
            'source_top_sha256':support['source_top_sha256'],
            'source_frame_id':support['source_frame_id'],
            'raw_action':{**raw,'duration_s':effective},
            'valid_until_s':lease.valid_until_s,
            'original_horizon_s':lease.horizon_s,
            'rgb_deadline_s':lease.rgb_deadline_s})
        return effective

    def _tick_solo_approach_lease(self,now):
        """Owner-check and, only while a new RGB job runs, finish this action's horizon."""
        lease=getattr(self,'_solo_approach_lease',None)
        if lease is None:return
        if (self.solo.phase!='approach' or self.solo.done
                or self.bindings.tasks['box']['id'] in self.bindings.finished):
            self._clear_solo_approach_lease(now,'phase_exit')
            return
        if self.bindings.committed['plan_hash']!=lease.plan_hash:
            self._clear_solo_approach_lease(now,'plan_changed')
            raise RuntimeError('rolling approach plan changed')
        # The port can expire and clear its own command fields before this
        # owner tick. Normal expiry is a hold/reobserve, not a foreign write.
        if (now>=lease.valid_until_s-1e-9 or now>=lease.horizon_s-1e-9
                or now>=lease.rgb_deadline_s-1e-9):
            self._clear_solo_approach_lease(now,'original_horizon_or_rgb_ttl')
            return
        if not self._solo_port_lease_matches({
                'valid_until_s':lease.valid_until_s,
                'motor_commands':lease.motor_commands}):
            self._clear_solo_approach_lease(now,'port_command_replaced')
            raise RuntimeError('rolling approach motor command changed outside owner lease')
        probe=self._permission_snapshot()
        if not all(probe.permission(obj,stage) for obj,stage in lease.permission_requests):
            self._clear_solo_approach_lease(now,'permission_revoked')
            return
        pending=self._solo_pending
        if (pending is None or pending['future'].done()
                or now+REALTIME_CAPTURE_OVERLAP_S<lease.valid_until_s-1e-9):
            return
        remaining=lease.next_duration_s(now)
        if remaining<=1e-9:
            self._clear_solo_approach_lease(now,'original_horizon_or_rgb_ttl')
            return
        if not self._commit_permissions(lease.permission_requests):
            self._clear_solo_approach_lease(now,'permission_revoked_at_renewal')
            return
        previous_until=lease.valid_until_s
        effective=self._issue_realtime_motor(
            self.bindings.solo,lease.action,'APPROACH',
            lease.evidence['source_observed_at_s'],now=now,max_duration_s=remaining,
            source_evidence={**lease.evidence,'decision_at_s':lease.decision_at_s,
                'original_horizon_s':lease.horizon_s,'lease_kind':'rolling_approach',
                'renewed_with_same_rgb':True})
        if effective is None:
            self._clear_solo_approach_lease(now,'rgb_ttl_at_renewal')
            return
        lease.valid_until_s=now+effective
        lease.motor_commands=tuple(self.ports[self.bindings.solo]._motor_commands)
        lease.issued_windows.append((now,now+effective))
        self.solo_renewals.append({'kind':'solo_approach_rolling_renewal',
            'source_frame_id':lease.evidence['source_frame_id'],
            'source_own_sha256':lease.evidence['source_own_sha256'],
            'source_top_sha256':lease.evidence['source_top_sha256'],
            'source_observed_at_s':lease.evidence['source_observed_at_s'],
            'decision_at_s':lease.decision_at_s,'issued_at_s':now,
            'previous_valid_until_s':previous_until,'valid_until_s':lease.valid_until_s,
            'original_horizon_s':lease.horizon_s,'rgb_deadline_s':lease.rgb_deadline_s,
            'permission_requests':list(lease.permission_requests),
            'plan_hash':lease.plan_hash})
        self.solo_raw.append({'event':'rolling_raw_action','time':now,
            'robot_id':self.bindings.solo,
            'source_frame_sha256':lease.evidence['source_own_sha256'],
            'source_top_sha256':lease.evidence['source_top_sha256'],
            'source_frame_id':lease.evidence['source_frame_id'],
            'raw_action':{**lease.action,'duration_s':effective},
            'valid_until_s':lease.valid_until_s,
            'original_horizon_s':lease.horizon_s,
            'rgb_deadline_s':lease.rgb_deadline_s,
            'renewed_with_same_rgb':True})

    def _solo_tick_realtime(self):
        try:self._solo_tick_realtime_owned()
        except BaseException:
            now=self.time()
            recovery=getattr(self,'_view_recovery',None)
            if recovery is not None and recovery.episode is not None:
                if self.solo_executor:self.solo_executor.cancel(now,'active_view_owner_exception')
                self.ports[self.bindings.solo].hold(now)
                recovery.episode=None
                self.solo_raw.append({'event':'rolling_view_recovery_stop','time':now,
                    'robot_id':self.bindings.solo,'reason':'owner_exception',
                    'motor_commands':list(self.ports[self.bindings.solo]._motor_commands)})
            self._clear_solo_pending_lease(now,hold=True)
            self._clear_solo_approach_lease(now,'owner_exception')
            raise

    def _solo_tick_realtime_owned(self):
        """Poll one RGB decision without making the physics owner wait for it."""
        if self.solo is None:return
        self.authorize();now=self.time()
        recovery=getattr(self,'_view_recovery',None)
        if recovery is not None and recovery.episode is not None:
            if now>=recovery.episode['started_at_s']+VIEW_RECOVERY_MAX_ELAPSED_S:
                self.solo_executor.cancel(now,'active_view_elapsed_budget')
                self._clear_rolling_view_recovery(now,'active_view_elapsed_budget')
                raise RuntimeError('solo RGB active-view elapsed budget exhausted')
            if (self.bindings.committed['plan_hash']!=recovery.episode['plan_hash']
                    or not self.bindings.permission('box','APPROACH')):
                self.solo_executor.cancel(now,'active_view_plan_or_permission_changed')
                self.ports[self.bindings.solo].hold(now)
                recovery.episode=None
                self.solo_raw.append({'event':'rolling_view_recovery_stop','time':now,
                    'robot_id':self.bindings.solo,
                    'reason':'plan_or_permission_changed',
                    'motor_commands':list(self.ports[self.bindings.solo]._motor_commands)})
                raise RuntimeError('solo RGB active-view recovery lost plan or permission')
        self.solo_executor.tick(now)
        self._refresh_completed_issued_pose_receipts()
        if getattr(self,'rolling_visual_servo',False):self._tick_solo_approach_lease(now)
        tasks=getattr(getattr(self,'bindings',None),'tasks',None)
        finished=bool(tasks and tasks['box']['id'] in self.bindings.finished)
        self._solo_budget_tick(now,enforce=not finished)
        if finished:
            self._clear_solo_pending_lease(now,hold=True)
            self._clear_solo_approach_lease(now,'box_job_finished')
            self._solo_finish_budget(now)
            return
        if self._solo_pending is not None:
            pending=self._solo_pending
            if not pending['future'].done():
                self._renew_solo_pending_lease(now,reason='next_rgb_pending')
                return
            if 'result' not in pending:pending['result']=pending['future'].result()
            pending.setdefault('result_ready_at_s',now)
            result=pending['result']
            observed=result['observed_at_s']
            if now-observed>RGB_ACTION_TTL_S:
                self._clear_solo_pending_lease(now)
                self._clear_solo_approach_lease(now,'stale_rgb')
                self._solo_pending=None
                self._solo_motion_accept_after_s=0.
                self._solo_set_resource_wait(now,False)
                self.ports[self.bindings.solo].hold(now)
                self.realtime_stats['solo_stale_rgb']+=1
                self.solo_rows.append({'index':pending['index'],'sim_time_s':now,
                    'observed_at_s':observed,'dropped':'stale_rgb',
                    'frame_id':result['frame_id']})
                if recovery is not None and recovery.episode is not None:
                    recovery.episode=None
                    self.solo_raw.append({'event':'rolling_view_recovery_stop','time':now,
                        'robot_id':self.bindings.solo,'reason':'stale_reobservation',
                        'source_frame_id':result['frame_id'],
                        'motor_commands':list(self.ports[self.bindings.solo]._motor_commands)})
                    raise RuntimeError('solo RGB active-view recovery stale reobservation')
                return
            candidate=result['candidate']
            action,evidence=result['action'],result['evidence']
            before=result['before']
            if (getattr(self,'rolling_visual_servo',False) and before!=self.solo.phase):
                self._clear_solo_approach_lease(now,'worker_phase_mismatch')
                self._solo_pending=None
                self.ports[self.bindings.solo].hold(now)
                raise RuntimeError('rolling approach worker phase changed before owner commit')
            requests=[('box',pending['stage']),*result['permission_requests']]
            if before=='approach' and candidate.phase=='lower':
                requests.append(('box','GRASP'))
            if action['kind']=='mecanum':requests.append(('box','TRANSIT'))
            moving_carry=((before=='carry' or candidate.phase=='carry')
                          and action['kind']=='mecanum'
                          and any(abs(action[k])>1e-9 for k in ('forward','left','turn')))
            if moving_carry and observed+RGB_ACTION_TTL_S-now<=0:
                self._clear_solo_pending_lease(now)
                self._solo_pending=None
                self._solo_motion_accept_after_s=0.
                self._solo_set_resource_wait(now,False)
                self.ports[self.bindings.solo].hold(now)
                self.realtime_stats['solo_stale_rgb']+=1
                self.solo_rows.append({'index':pending['index'],'sim_time_s':now,
                    'observed_at_s':observed,'dropped':'expired_motion_lease',
                    'frame_id':result['frame_id']})
                return
            worker_wait=bool(evidence.get('waiting_for_resource'))
            # The legacy RGB skill counts one moving decision per requested
            # 0.2 SIM seconds. Finish the next RGB worker early, but admit its
            # next moving action only after that nominal interval while the
            # prior bounded lease remains valid. A TTL-shortened lease caps it.
            cadence_due=getattr(self,'_solo_motion_accept_after_s',0.)
            waiting_for_cadence=(moving_carry and before=='carry'
                                 and candidate.phase=='carry'
                                 and now+1e-9<cadence_due)
            if worker_wait or waiting_for_cadence:
                probe=self._permission_snapshot()
                owner_denied=not all(probe.permission(obj,stage)
                                     for obj,stage in dict.fromkeys(requests))
            else:owner_denied=not self._commit_permissions(requests)
            if waiting_for_cadence and not worker_wait and not owner_denied:
                # No live lock or route state is committed while waiting. The
                # original RGB timestamp is checked again on every owner tick.
                self._renew_solo_pending_lease(now,reason='fresh_rgb_waiting_for_cadence')
                return
            self._solo_pending=None
            if worker_wait or owner_denied:
                self._clear_solo_pending_lease(now)
                self._clear_solo_approach_lease(now,'resource_permission')
                self._solo_motion_accept_after_s=0.
                self._solo_set_resource_wait(now,owner_denied,
                                             'candidate' if owner_denied else None)
                previous_guard_at=getattr(self.solo.box,'_last_sim_time',None)
                vision_refreshed=self._adopt_waiting_carry_visual(
                    candidate,action,evidence,result['observation'])
                self.ports[self.bindings.solo].hold(now)
                self._solo_retry_at=now+(.3 if ('box','GRASP') in requests else .1)
                self.solo_rows.append({'index':pending['index'],'sim_time_s':now,
                    'observed_at_s':observed,'dropped':'resource_permission',
                    'frame_id':result['frame_id'],'requests':requests,
                    'owner_permission_denied':owner_denied,
                    'state_only':vision_refreshed,
                    'phase_before':before,'phase_after':self.solo.phase,
                    'observation':{k:v for k,v in result['observation'].items()
                                   if k!='image'},
                    'images':result['images'],
                    'action':action,'top_evidence':evidence,
                    'own_attachment_evidence':copy.deepcopy(candidate.box.last_attachment),
                    'approach_adjustment':copy.deepcopy(getattr(candidate.box,'last_approach_adjustment',None)),
                    'carry_visual_refreshed':vision_refreshed,
                    'guard_previous_observed_at_s':previous_guard_at,
                    'guard_observation_gap_s':(
                        observed-previous_guard_at if vision_refreshed else None)})
                return
            self._clear_solo_pending_lease(now,hold=True)
            previous_approach_lease=(getattr(self,'_solo_approach_lease',None)
                or getattr(self,'_solo_approach_last_lease',None))
            self._clear_solo_approach_lease(now,'fresh_rgb_decision')
            self._solo_set_resource_wait(now,False)
            # The committed navigator must no longer retain the worker's
            # private permission snapshot when it is cloned next time.
            if candidate.navigator is not None:
                candidate.navigator._permission=self.bindings.permission
            self.solo=candidate
            self.realtime_stats['solo_decisions']+=1
            self.realtime_stats['max_decision_age_s']=max(
                self.realtime_stats['max_decision_age_s'],now-observed)
            self._solo_phase_label=self.solo.phase
            obs=result['observation']
            row={'index':pending['index'],'sim_time_s':now,
                'observed_at_s':observed,'frame_id':result['frame_id'],
                'decision_age_s':now-observed,'robot_id':self.bindings.solo,
                'input_transform':result['input_transform'],'phase_before':before,
                'phase_after':self.solo.phase,
                'observation':{k:v for k,v in obs.items() if k!='image'},
                'images':result['images'],'action':action,'top_evidence':evidence,
                'own_attachment_evidence':copy.deepcopy(self.solo.box.last_attachment),
                'approach_adjustment':copy.deepcopy(getattr(self.solo.box,'last_approach_adjustment',None))}
            if getattr(self,'rolling_visual_servo',False) and before=='approach':
                row['rolling_capture_evidence']={
                    'capture_started_at_s':pending['capture_started_at_s'],
                    'observed_at_s':observed,'frame_id':result['frame_id'],
                    'own_sha256':obs['sha256'],'top_sha256':result['top_sha256'],
                    'frame_within_issued_command_window':bool(previous_approach_lease and
                        previous_approach_lease.frame_within_issued_command_window(observed)),
                    'command_window_is_not_measured_motion':True}
            view_sample=None
            if recovery is not None and before=='approach':
                view_sample=self._rolling_view_sample(result,pending,obs,now)
                view_event=recovery.advance(view_sample)
                if view_event is not None:
                    if view_event['kind']=='pose_issued':
                        self._issue_rolling_view_pose(view_event,view_sample,obs,row,now)
                        return
                    if view_event['kind']=='failed':
                        self._fail_rolling_view(view_event,row,now)
                    row['view_recovery']=view_event
                if (not row['rolling_capture_evidence']['frame_within_issued_command_window']
                        and recovery.episode is None):
                    previous_end=(previous_approach_lease.last_command_end_s()
                                  if previous_approach_lease is not None else None)
                    recovery.remember(view_sample,view_sample['completed_pose_receipts'],
                                      previous_end)
            if action['kind']=='mecanum':
                if moving_carry:
                    effective=self._issue_realtime_motor(
                        self.bindings.solo,action,'TRANSIT',observed,now=now)
                    nominal_cadence=min(float(action['duration_s']),
                                        REALTIME_MOTOR_RENEWAL_S)
                    # A TTL-clipped lease must never force a later motor gap.
                    admission_cadence=min(nominal_cadence,effective)
                    self._solo_motion_accept_after_s=(
                        now+admission_cadence if self.solo.phase=='carry' else 0.)
                    if effective is not None and self.solo.phase=='carry':
                        renewal_requests=list(dict.fromkeys([*requests,('box','TRANSIT')]))
                        navigator=getattr(candidate,'navigator',None)
                        if (getattr(self.bindings,'route_overlap',False)
                                and navigator is not None
                                and getattr(navigator,'index',0)>=2):
                            renewal_requests=list(dict.fromkeys([*renewal_requests,
                                                                   ('box','UNLOAD')]))
                        self._solo_pending_lease={
                            'action':copy.deepcopy(action),'observed_at_s':observed,
                            'frame_id':result['frame_id'],'decision_sim_time_s':now,
                            'valid_until_s':now+effective,
                            'motor_commands':tuple(getattr(
                                self.ports[self.bindings.solo], '_motor_commands', ())),
                            'next_permission_check_s':now+REALTIME_CAPTURE_OVERLAP_S,
                            'plan_hash':self.bindings.committed['plan_hash'],
                            'permission_requests':tuple(renewal_requests)}
                    row.update(requested_duration_s=action['duration_s'],
                               renewal_request_s=REALTIME_MOTOR_RENEWAL_S,
                               effective_lease_s=effective,
                               nominal_motion_cadence_s=nominal_cadence,
                               motion_admission_cadence_s=admission_cadence,
                               cadence_wait_s=max(0.,now-pending['result_ready_at_s']))
                    self.solo_lease=now+REALTIME_CAPTURE_OVERLAP_S
                else:
                    self._solo_motion_accept_after_s=0.
                    self.raw(self.bindings.solo,action,'TRANSIT')
                    self.solo_lease=now+action['duration_s']
            elif (getattr(self,'rolling_visual_servo',False) and before=='approach'
                    and self.solo.phase=='approach' and action['kind']=='drive'):
                prior=self._solo_approach_last_source
                support=approach_drive_support(
                    action=action,box=self.solo.box.last_box,
                    target=self.solo.box.last_target,
                    cargo_id=self.solo.box.cargo_id,frame_id=result['frame_id'],
                    observed_at_s=observed,decision_at_s=now,
                    capture_started_at_s=pending['capture_started_at_s'],
                    own_sha256=obs['sha256'],top_sha256=result['top_sha256'],
                    paired_top_sha256=result['images']['top']['sha256'],
                    prior_frame_id=prior[0] if prior else None,
                    prior_observed_at_s=prior[1] if prior else None,
                    prior_own_sha256=prior[2] if prior else None)
                support=guard_near_entry_from_issued_commands(
                    support,previous_approach_lease,observed,now)
                settle_until=support['previous_command_settle_until_s']
                row['rolling_support']=support
                if support['allowed']:
                    effective=self._start_solo_approach_lease(action,support,now,requests)
                    if effective is None:
                        row['rolling_dropped']='lease_expired_before_issue'
                        self.ports[self.bindings.solo].hold(now)
                        self._solo_retry_at=now+REALTIME_CAPTURE_OVERLAP_S
                    else:
                        row.update(rolling_effective_lease_s=effective,
                                   rolling_original_horizon_s=now+support['original_duration_s'])
                        self.solo_lease=now+REALTIME_CAPTURE_OVERLAP_S
                elif support['reason']=='final_entry_handoff':
                    # Preserve the existing settled final-entry/grasp routine.
                    self.solo_executor.submit(action,obs,self.solo.phase,now)
                else:
                    if (recovery is not None and support['reason']=='weak_direct_cyan_rgb'):
                        view_event=recovery.start(view_sample)
                        if view_event['kind']=='hold_reobserve':
                            row['rolling_support']=support
                            self._issue_rolling_view_reobserve(view_event,row,now)
                            return
                        if view_event['kind']=='pose_issued':
                            row['rolling_support']=support
                            self._issue_rolling_view_pose(view_event,view_sample,obs,row,now)
                            return
                        self._fail_rolling_view(view_event,row,now)
                    row['rolling_dropped']=support['reason']
                    self.ports[self.bindings.solo].hold(now)
                    self._solo_retry_at=max(now+.1,settle_until or now)
            else:
                self._solo_motion_accept_after_s=0.
                self.solo_executor.submit(action,obs,self.solo.phase,now)
            self.solo_rows.append(row)
            if pending['index']%25==0 or before!=self.solo.phase:
                print(json.dumps({'solo_step':pending['index'],'robot':self.bindings.solo,
                    'phase':self.solo.phase,'action':action}),flush=True)
            if self.solo.done:
                if self.solo.reason!='VISUAL_RELEASE_CONFIRMED':
                    raise RuntimeError('solo stopped: '+str(self.solo.reason))
                if self.bindings.tasks['beam']['id'] in self.bindings.finished:self.bindings.finish('box')
            return
        if self.bindings.tasks['box']['id'] in self.bindings.finished:return
        if not self.solo_executor.idle or now<max(self.solo_lease,self._solo_retry_at):return
        if self.solo.done:
            self._yield_tick_realtime(now);return
        stage='APPROACH' if self.solo.phase=='approach' else 'TRANSIT' if self.solo.phase=='carry' else 'GRASP'
        if self.solo_started is None:self.solo_started=now
        self._solo_budget_tick(now)
        rid=self.bindings.solo;index=len(self.solo_rows)
        observation_state=copy.deepcopy(self.ports[rid]._actuator_state())
        permission_snapshot=self._permission_snapshot()
        if not permission_snapshot.permission('box',stage):
            self._clear_solo_pending_lease(now)
            self._clear_solo_approach_lease(now,'capture_stage_permission_denied')
            self._solo_set_resource_wait(now,True,'stage')
            self.ports[rid].hold(now)
            return
        if getattr(self,'_solo_resource_wait_reason',None)=='stage':
            self._solo_set_resource_wait(now,False)
        candidate=copy.deepcopy(self.solo,{id(self.bindings):self.bindings})
        permission_requests=[]
        if candidate.navigator is not None:
            def worker_permission(obj,requested_stage):
                permission_requests.append((obj,requested_stage))
                return permission_snapshot.permission(obj,requested_stage)
            candidate.navigator._permission=worker_permission
        from sim.snapshot_render import SnapshotBackpressure
        try:frames=self.capture_async(f'solo-{index}',own_robots=(rid,),overview=False)
        except SnapshotBackpressure:
            self._solo_set_resource_wait(now,False)
            self.realtime_stats['solo_backpressure']+=1
            self._solo_retry_at=now+.02
            return

        def decide():
            frame=frames.result(timeout=30.)[rid]
            native=frame['own_bytes']
            obs={'robot_id':rid,'frame_id':frame['frame_id'],
                 'sim_time':frame['observed_at_s'],
                 'image':base64.b64encode(native).decode('ascii'),
                 'sha256':hashlib.sha256(native).hexdigest(),
                 'camera':'robot_cam','actuator_state':observation_state}
            normalized,input_transform=normalize_own_rgb(obs)
            own=base64.b64decode(normalized['image'],validate=True)
            refs={'raw_own':frame['own_rgb'],
                  'own':image_record(self.out/'rgb'/f'solo-{index}-own.jpg',self.out,own),
                  'top':frame['shared_top_rgb']}
            before=candidate.phase
            approach_state=copy.deepcopy(candidate.box) if before=='approach' else None
            action,evidence=candidate.decide(normalized,frame['top_bytes'])
            return {'candidate':candidate,'action':action,'evidence':evidence,
                    'before':before,'approach_state':approach_state,
                    'observation':normalized,'input_transform':input_transform,
                    'images':refs,'observed_at_s':frame['observed_at_s'],
                    'frame_id':frame['frame_id'],
                    'top_sha256':hashlib.sha256(frame['top_bytes']).hexdigest(),
                    'permission_requests':tuple(permission_requests)}
        self._solo_pending={'future':self._decision_workers.submit(decide),
                            'index':index,'stage':stage,
                            'capture_started_at_s':now}

    def _yield_tick_realtime(self,now):
        if self.bindings.tasks['beam']['id'] in self.bindings.finished:
            self.bindings.finish('box');return
        if self._yield_pending is not None:
            pending=self._yield_pending
            if not pending['future'].done():return
            self._yield_pending=None
            result=pending['future'].result()
            observed=result['observed_at_s']
            if now-observed>RGB_ACTION_TTL_S:
                self.ports[self.bindings.solo].hold(now)
                self.realtime_stats['solo_stale_rgb']+=1
                self.yield_rows.append({'index':pending['index'],'sim_time_s':now,
                    'observed_at_s':observed,'dropped':'stale_rgb',
                    'frame_id':result['frame_id']})
                return
            action,evidence=result['action'],result['evidence']
            moving=(self.yield_folded and action['kind']=='mecanum'
                    and any(abs(action[k])>1e-9 for k in ('forward','left','turn')))
            if moving and observed+RGB_ACTION_TTL_S-now<=0:
                self.ports[self.bindings.solo].hold(now)
                self.realtime_stats['solo_stale_rgb']+=1
                self.yield_rows.append({'index':pending['index'],'sim_time_s':now,
                    'observed_at_s':observed,'dropped':'expired_motion_lease',
                    'frame_id':result['frame_id']})
                return
            if result['policy'] is not None:self.yield_policy=result['policy']
            row={'index':pending['index'],'sim_time_s':now,
                'observed_at_s':observed,'decision_age_s':now-observed,
                'frame_id':result['frame_id'],'robot_id':self.bindings.solo,
                'images':result['images'],
                'observation':{k:v for k,v in result['observation'].items() if k!='image'},
                'action':action,'evidence':evidence,
                'box_job_finished':self.bindings.tasks['box']['id'] in self.bindings.finished}
            if not self.yield_folded:
                self.solo_executor.submit(action,result['observation'],'yield_fold',now)
                self.yield_folded=True
            else:
                if moving:
                    effective=self._issue_realtime_motor(
                        self.bindings.solo,action,'YIELD',observed,now=now)
                    row.update(requested_duration_s=action['duration_s'],
                               renewal_request_s=REALTIME_MOTOR_RENEWAL_S,
                               effective_lease_s=effective)
                    self.solo_lease=now+REALTIME_CAPTURE_OVERLAP_S
                else:
                    self.raw(self.bindings.solo,action,'YIELD')
                    row.update(requested_duration_s=action['duration_s'],
                               effective_lease_s=action['duration_s'])
                    self.solo_lease=now+action['duration_s']
                if self.yield_policy.done:self.bindings.finish('box')
            self.yield_rows.append(row)
            self.realtime_stats['solo_decisions']+=1
            self.realtime_stats['max_decision_age_s']=max(
                self.realtime_stats['max_decision_age_s'],now-observed)
            if pending['index']%25==0 or self.yield_policy and self.yield_policy.done:
                print(json.dumps({'solo_yield_step':pending['index'],'evidence':evidence}),flush=True)
            return
        if now<self._solo_retry_at:return
        rid=self.bindings.solo;index=len(self.yield_rows)
        observation_state=copy.deepcopy(self.ports[rid]._actuator_state())
        cargo_center=copy.deepcopy(self.solo.navigator.box_center)
        folded=self.yield_folded
        policy=(copy.deepcopy(self.yield_policy) if self.yield_policy is not None
                else SoloYield(self.bindings.static_map,cargo_center))
        from sim.snapshot_render import SnapshotBackpressure
        try:frames=self.capture_async(f'yield-{index}',own_robots=(rid,),overview=False)
        except SnapshotBackpressure:
            self.realtime_stats['solo_backpressure']+=1
            self._solo_retry_at=now+.02
            return

        def decide():
            frame=frames.result(timeout=30.)[rid]
            own=frame['own_bytes']
            obs={'robot_id':rid,'frame_id':frame['frame_id'],
                 'sim_time':frame['observed_at_s'],
                 'image':base64.b64encode(own).decode('ascii'),
                 'sha256':hashlib.sha256(own).hexdigest(),
                 'camera':'robot_cam','actuator_state':observation_state}
            if not folded:
                action={'kind':'pose','pulses':{1:2000,3:740,4:2320,5:1320,6:1500}}
                evidence={'phase':'fold_open_arm_after_visual_release'}
                used_policy=None
            else:
                action,evidence=policy.decide(frame['top_bytes'])
                evidence['initial_cargo_center_px']=cargo_center.tolist()
                used_policy=policy
            return {'action':action,'evidence':evidence,'policy':used_policy,
                    'observation':obs,'observed_at_s':frame['observed_at_s'],
                    'frame_id':frame['frame_id'],
                    'images':{'own':frame['own_rgb'],'top':frame['shared_top_rgb']}}
        self._yield_pending={'future':self._decision_workers.submit(decide),'index':index}

    def close(self):
        if self.native_view:self.native_view.close();self.native_view=None
        if self.world:
            self._clear_rolling_view_recovery(self.time(),'trial_end')
            self._clear_solo_approach_lease(self.time(),'trial_end')
            self._clear_solo_pending_lease(self.time(),hold=True)
            if self.solo_executor:self.solo_executor.cancel(self.time(),'trial_end')
            if self.original_step:self.world._physics_step_for=self.original_step
        if self._decision_workers:
            self._decision_workers.shutdown(wait=True,cancel_futures=True)
            self._decision_workers=None
        super().close()


def grasp_transfer_options(skill):
    if skill.get('rgb_support_scope') in {'full_calibrated_views', 'local_grasp_top_v1'}:
        return {'background_band': False, 'top_roi': None}
    return {'background_band': skill.get('task_domain') != 'dispatch_open_v1',
            'top_roi': [12,6,20,19] if skill.get('task_domain') == 'dispatch_open_v1' else None}


class TeamStop(RuntimeError):
    """The robots chose to stop the whole team (dynamic coordination)."""


def _pair_participants(scene):
    return [scene.bindings.pair[slot] for slot in sorted(scene.bindings.pair)]


def _inject(args,result,where):
    """Output-only diagnostic failure, at most once per kind and run."""
    if (getattr(args,'coordination','plan_first')!='dynamic'
            or not getattr(args,f'diagnostic_fail_{where}_once',False)):return False
    done=result['coordination'].setdefault('injections',[])
    if where in done:return False
    done.append(where);result['_injected']=True
    return True


def _team_decide(scene,team,task,result,kind,participants,failure,*,injected=False,hold=None,
                 settle_s=.5,**details):
    """Ask the affected robots; record the event, replies and decision."""
    from harness.dynamic_coordination import consult,make_event
    coordination=result['coordination']
    if hold is not None:
        hold();scene.step(settle_s)
    # Numbered after the settle: a paused box event may be decided meanwhile.
    index=len(coordination['recoveries'])
    frames=scene.capture(f'recovery-{index}')
    event=make_event(kind,f'{kind}-{index}',participants=participants,failure=failure,**details)
    decision,rounds=consult(team,participants,event,frames,scene.command_history,task,
                            scene.time(),turn=index)
    coordination['recoveries'].append({**event,'sim_time_s':scene.time(),
        'diagnostic_injection':injected,'decision':decision,'rounds':rounds})
    team.event('RECOVERY_DECISION',scene.time(),event_id=event['event_id'],decision=decision)
    print(f'RECOVERY {event["event_id"]}: {decision}',flush=True)
    return decision


def _job_failed(scene,team,task,result,job,exc,*,set_down,hold=None):
    """A job is given up: all three decide to continue the others or stop."""
    other='box' if job=='beam' else 'beam'
    if other in [row['job'] for row in result['coordination'].get('dropped_jobs',[])]:
        raise TeamStop(f'{exc}; no agreed job left') from exc
    decision=_team_decide(scene,team,task,result,'job_failed',list(ROBOTS),exc,hold=hold,
        failed_job=job,other_job=other,
        other_job_state='finished' if scene.bindings.settled(other) else 'in_progress')
    if decision!='continue_others':
        raise TeamStop(f'{exc}; team decided to stop all') from exc
    set_down();scene.bindings.drop(job)
    result['coordination'].setdefault('dropped_jobs',[]).append(
        {'job':job,'reason':str(exc),'sim_time_s':scene.time()})
    print(f'JOB DROPPED {job}: {exc}',flush=True)


def _box_event(scene,team,task,args,result,event):
    """Paused box failure: the box robot may retry; otherwise the team decides."""
    from harness.dynamic_coordination import RECOVERABLE_BOX_FAILURES
    retries=getattr(args,'box_retries',2)
    used=sum(row.get('decision')=='retry' for row in scene.solo_events)
    solo=scene.bindings.solo
    if event['reason'] in RECOVERABLE_BOX_FAILURES and used<retries:
        decision=_team_decide(scene,team,task,result,'box_failure',[solo],event['reason'],
            injected=event.get('diagnostic_injection',False),stage=event['phase'],
            retries_left=retries-used)
        if decision=='retry':return 'retry'
    # The box robot is already holding; it keeps its gripper as issued.
    _job_failed(scene,team,task,result,'box',RuntimeError('box stopped: '+event['reason']),
                set_down=lambda:None)
    return 'dropped'


def _approach(pair,scene,team,task,args,result):
    """Pair approach; in dynamic coordination a supported failure is discussed.

    The affected carriers decide unanimously to retry (bounded back-off and a
    fresh RGB approach) or abort. Plan-first coordination keeps fail-closed.
    """
    from harness.dynamic_coordination import recoverable
    dynamic=getattr(args,'coordination','plan_first')=='dynamic'
    retries=getattr(args,'approach_retries',2) if dynamic else 0
    attempt=0
    while True:
        try:
            report=pair.approach()
            if dynamic and _inject(args,result,'approach'):
                # The robots see a real controller stop reason, so the team's
                # decision rests on its images; the injection is output-only.
                raise RuntimeError('coarse RGB approach budget exhausted')
            return report
        except RuntimeError as exc:
            if not dynamic or not recoverable(exc) or attempt>=retries:raise
            decision=_team_decide(scene,team,task,result,'pair_approach_failure',
                _pair_participants(scene),exc,injected=result.pop('_injected',False),
                hold=pair._hold_pair,stage='APPROACH',retries_left=retries-attempt)
            if decision!='retry':raise RuntimeError(f'{exc}; team decided to abort') from exc
            pair.back_off();attempt+=1


def _beam_job(pair,scene,team,task,args,result,grasp):
    while not scene.bindings.permission('beam','APPROACH'):scene.step(.2)
    result['phase']='APPROACH';result['pair_approach']=_approach(pair,scene,team,task,args,result)
    regrasps=0
    while True:
        while not scene.bindings.permission('beam','GRASP'):scene.step(.2)
        result['phase']='GRASP'
        try:
            result['pair_grasp']=pair.finish_grasp(predict_student,grasp)
            if _inject(args,result,'grasp'):
                raise RuntimeError('existing pair carry guard stopped: ABORT')
            pair.grasp_report.pop('evaluation',None)
            pair.grasp_report['evaluation_source']='separate referee-only.jsonl after control ends'
            while not scene.bindings.permission('beam','TRANSIT'):scene.step(.2)
            result['phase']='TRANSIT'
            _carry(pair,scene,args,result)
            break
        except RuntimeError as exc:
            from harness.dynamic_coordination import recoverable,RECOVERABLE_GRASP_FAILURES
            retries=getattr(args,'approach_retries',2)
            # Regrasp only at the pickup: before any loaded transit command.
            if (getattr(args,'coordination','plan_first')!='dynamic'
                    or not recoverable(exc,RECOVERABLE_GRASP_FAILURES)
                    or 'beam' in scene.bindings.transit_started or regrasps>=retries):raise
            decision=_team_decide(scene,team,task,result,'pair_grasp_failure',
                _pair_participants(scene),exc,stage=result['phase'],retries_left=retries-regrasps,
                injected=result.pop('_injected',False),hold=pair._hold_pair)
            if decision!='regrasp':raise RuntimeError(f'{exc}; team decided to abort') from exc
            pair.reset_for_regrasp();pair.back_off();regrasps+=1
            result['phase']='APPROACH'
            result['pair_approach']=_approach(pair,scene,team,task,args,result)
    if scene.bindings.route_overlap and not scene.bindings.permission('beam','UNLOAD'):
        raise RuntimeError('shared unload resource unavailable')
    result['phase']='RELEASE';pair.place();pair.verify_placement();scene.bindings.finish('beam')


def _carry(pair,scene,args,result):
    carry_steps=getattr(args,'carry_max_steps',None)
    if getattr(args,'carry_act_model',None):
        from scripts.dispatch_act_carry import carry
        result['carry_policy']='ACT own RGB + raw top RGB + static task + own last issued motion'
        result['carry_stop_mode']=getattr(args,'carry_act_stop_mode','rgb_guarded')
        result['pure_act']=result['carry_stop_mode']=='learned'
        result['act_motion_only']=result['carry_stop_mode']!='rgb_refined'
        if result['carry_stop_mode']=='rgb_guarded':
            result['carry_policy']='ACT motion with independent RGB stop admission'
        elif result['carry_stop_mode']=='rgb_refined':
            result['carry_policy']='ACT transit plus explicit RGB final alignment (hybrid)'
        result['carry_model_sha256']=sha(args.carry_act_model/'model.safetensors')
        carry(pair,args.carry_act_python,args.carry_act_model,
              args.carry_act_max_steps if carry_steps is None else carry_steps,
              stop_mode=result['carry_stop_mode'])
    elif scene.realtime_control:
        pair.carry_realtime(ImageRoute(scene.bindings,'beam'),max_steps=carry_steps)
    else:pair.carry(ImageRoute(scene.bindings,'beam'),max_steps=carry_steps)


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
    scene.realtime_control=bool(getattr(args,'realtime_control',False))
    scene.coarse_concurrent_alignment=bool(getattr(args,'coarse_concurrent_alignment',False))
    scene.rolling_visual_servo=bool(getattr(args,'rolling_visual_servo',False))
    scene.bounded_carrier_relink=bool(getattr(args,'bounded_carrier_relink',False))
    scene.rolling_view_recovery=bool(getattr(args,'rolling_view_recovery',False))
    if scene.rolling_visual_servo and not scene.realtime_control:
        raise ValueError('rolling visual servo requires realtime control')
    if scene.coarse_concurrent_alignment and not scene.realtime_control:
        raise ValueError('coarse concurrent alignment requires realtime control')
    if scene.bounded_carrier_relink and not (scene.rolling_visual_servo and scene.realtime_control):
        raise ValueError('bounded carrier relink requires rolling realtime control')
    if scene.rolling_view_recovery and not (scene.rolling_visual_servo and scene.realtime_control):
        raise ValueError('rolling view recovery requires rolling realtime control')
    scene.fine_gain_schedule=bool(getattr(args,'fine_gain_schedule',False))
    started=time.monotonic();pair=team=None
    motion_started_wall=motion_started_sim=None
    result={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'scope':'three LLM peers choose allocation; actual saved approach/grasp models and existing box skill execute',
        'config':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        'model_provenance':{'grasp_manifest_sha256':sha(grasp_root/'student-skill.json'),
            'grasp_models':skill['models'],'stage_manifest_sha256':sha(args.stage_model_dir/'varied-start-skill.json'),
            'stage_models':stage_skill['models'],'reference_sha256':sha(args.reference_top)},
        'environment':{'python':sys.version,'platform':platform.platform(),'mujoco':mujoco.__version__},
        'plan_committed':False,'protocol_complete':False,'physical_success':False,'error':None,
        'phase':'SETUP','cost_usd':None,
        'coarse_concurrent_alignment':{
            'requested':scene.coarse_concurrent_alignment,'applied':False,
            'reason':'not_evaluated',
            'scope':'realtime paired APPROACH on fixed dispatch_open or dispatch_shared_crossing RGB maps',
            'static_map_sha256':config['static_map_sha256'],
            'arena_variant':config['variant'],
            'same_fresh_top_required':True,'max_capture_age_s':COARSE_CONCURRENT_MAX_CAPTURE_AGE_S,
            'rgb_ttl_s':RGB_ACTION_TTL_S,'existing_coarse_command_s':COARSE_CONCURRENT_COMMAND_S,
            'lead_limit_px':COARSE_LEAD_LIMIT_PX,'coarse_decision_cap':120,
            'lease_policy':'unchanged existing 0.25s wire cap and 0.6s RGB TTL'},
        'rolling_approach':{'requested':scene.rolling_visual_servo,'applied':False,
            'scope':'solo approach only, realtime dispatch_open without internal obstacles',
            'max_wire_lease_s':REALTIME_MOTOR_RENEWAL_S,
            'rgb_ttl_s':RGB_ACTION_TTL_S,
            'final_entry_handoff_m':FINAL_ENTRY_HANDOFF_M,
            'near_motion_reobserve_buffer_m':NEAR_MOTION_REOBSERVE_BUFFER_M,
            'final_entry_settle_s':FINAL_ENTRY_SETTLE_S,
            'issued_command_integral_is_not_measured_travel':True},
        'view_recovery':{'requested':scene.rolling_view_recovery,'applied':False,
            'scope':'solo rolling approach on realtime dispatch_open; current paired own/TOP RGB and completed issued pose only',
            'minimum_drive_rgb_confidence':0.75,
            'max_poses_per_episode':VIEW_RECOVERY_MAX_POSES,
            'max_elapsed_s':VIEW_RECOVERY_MAX_ELAPSED_S,
            'max_episodes':VIEW_RECOVERY_MAX_EPISODES,
            'wrist_step_pwm':VIEW_RECOVERY_WRIST_STEP_PWM,
            'max_pan_delta_pwm':VIEW_RECOVERY_MAX_PAN_DELTA_PWM,
            'episodes_started':0,
            'poses_issued':0,
            'issued_pwm_not_measured_joint':True},
        'bounded_carrier_relink':{'requested':scene.bounded_carrier_relink,'applied':False,
            'scope':'solo carry only, realtime dispatch_open with rolling approach',
            'max_total_occluded_frames':_CARRIER_RELINK_MAX_OCCLUDED_FRAMES,
            'max_elapsed_s':_CARRIER_RELINK_MAX_ELAPSED_S,
            'max_cumulative_visual_motion_px':_CARRIER_RELINK_MAX_MOTION_PX,
            'final_slot_requires_direct_top_cargo':True},
        'input_boundary':'own fixed RGB + common fixed TOP RGB + static authored map + own issued commands + peer claims; referee output only'}
    scene.source_sha=result['source_sha']
    try:
        scene.open();scene.deadline=started+args.max_wall_s
        if getattr(args,'viewer',False):
            if scene.realtime_control:
                from scripts.dispatch_native_process import IsolatedDispatchNativeView
                scene.native_view=IsolatedDispatchNativeView(scene,realtime_factor=args.realtime_factor)
            else:
                from scripts.dispatch_native_view import DispatchNativeView
                scene.native_view=DispatchNativeView(scene,realtime_factor=args.realtime_factor)
        elif scene.realtime_control:
            from scripts.dispatch_native_view import HeadlessPacer
            scene.native_view=HeadlessPacer(scene,realtime_factor=args.realtime_factor)
        if getattr(args,'record_replay',False):
            from scripts.dispatch_replay import ReplayRecorder
            scene.replay=ReplayRecorder(scene.world,args.output)
        result['timing']={'scene_and_observer_setup_wall_s':time.monotonic()-started,
                          'scope':'output-only SIM/wall measurements; wall_s includes finalization'}
        write(args.output/'episode-setup-only.json',scene.config)
        write(args.output/'scene-manifest.json',scene.manifest)
        if scene.realtime_control:
            from scripts.dispatch_async_video import AsyncVideo
            scene.video=AsyncVideo(scene.world,args.output/'execution.mp4',getattr(args,'video_fps',10))
        else:
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
        if getattr(args,'task',None):task['operator_instruction']=args.task
        guidance=PlanGuidance(getattr(args,'plan_guidance','legacy'),scene.config['static_map'],
                              route_overlap=getattr(args,'route_overlap',False),
                              auto_route_overlap=getattr(args,'auto_route_overlap',False),
                              overlap_start=getattr(args,'overlap_start','transit'))
        task['capability_scope']=guidance.capability_scope
        scene.guidance=guidance
        navigation=getattr(args,'navigation','authored')
        if navigation=='planned':
            task['capability_scope']+=(' Planned navigation: you choose the dock and the box robot park place '
                '(authored region name or xy_m); map A* plans the loaded box path and the park move. '
                'The beam keeps its agreed corridor. Box and beam transport run one at a time.')
        write(args.output/'actor-mission.json',task)
        run_id=opaque_run_id()
        def planner(rid,**kwargs):
            return build_dispatch_request(rid,task=task,execution_pilot=True,identity_evidence=identity[rid],
                                          navigation=navigation,guidance=guidance,**kwargs)
        replay_plan=None
        if args.plan_replay:
            saved=json.loads(args.plan_replay.read_text())
            if navigation_mode(saved['plan'])!=navigation:
                raise ValueError('replayed plan was written for '+navigation_mode(saved['plan'])+' navigation')
            SkillBindings(saved,scene.config['static_map'])
            replay_plan=saved['plan']
            result['scope']='recorded-plan diagnostic with fixture votes; existing RGB physical skills, not fresh LLM E2E'
            result['plan_replay_sha256']=sha(args.plan_replay)
            if getattr(args,'live_replan',False):
                result['scope']='fixture-initialized recovery diagnostic; actual LLM re-negotiation after capability rejection, not fresh initial LLM E2E'
        team=ThreeRobotRuntime(args.output/'team',run_id=run_id,mode='fixture' if replay_plan else 'llm',
            plan_fixture=replay_plan,
            agreement=TeamAgreement(run_id,plan_validator=partial(validate_dispatch_plan,
                required_dock=getattr(args,'required_dock',None),navigation=navigation)),request_builder=planner,
            reply_validator=partial(validate_dispatch_reply,navigation=navigation),request_timeout=args.timeout,max_tokens=1600,
            roles_fixed_by_skill=False,planning_only=False,max_wall_s=args.max_wall_s,
            model=getattr(args,'model','gemini-3.8-flash'),
            idle_callback=scene.native_view.poll if scene.native_view else None)
        scene.team=team;frames=scene.capture('planning')
        negotiation=dict(max_tokens=args.max_input_tokens,
            max_rounds=getattr(args,'planning_rounds',8),max_replans=getattr(args,'max_replans',2),
            live_replan=getattr(args,'live_replan',False))
        if getattr(args,'coordination','plan_first')=='dynamic':
            # Start from independent self-claims; talk only on conflict/rejection.
            from harness.dynamic_coordination import start as dynamic_start
            result['phase']='CLAIM'
            log=dynamic_start(team,frames,scene.command_history,task,scene.config['static_map'],
                scene.time(),identity=identity,reference_top=reference,
                required_dock=getattr(args,'required_dock',None),**negotiation)
            result['plan_feasibility']=log.pop('feasibility')
            result['coordination']={'mode':'dynamic','start':log,'recoveries':[]}
            print('DYNAMIC START '+log['path'],flush=True)
        else:
            result['phase']='NEGOTIATE'
            result['plan_feasibility']=negotiate_executable(team,frames,scene.command_history,task,
                scene.config['static_map'],scene.time(),identity=identity,reference_top=reference,
                feedback_instruction=guidance.feasibility_feedback,**negotiation)
        scene.bindings=SkillBindings(team.agreement.committed,scene.config['static_map'],
                                    route_overlap=getattr(args,'route_overlap',False),
                                    auto_route_overlap=getattr(args,'auto_route_overlap',False),
                                    overlap_start=getattr(args,'overlap_start','transit'),
                                    box_route=result['plan_feasibility'].get('planned_box_route'))
        result.update(plan_committed=True,plan=scene.bindings.plan,bindings=scene.bindings.capabilities(),
                      overlap_selection=scene.bindings.overlap_selection)
        write(args.output/'committed-plan.json',team.agreement.committed)
        write(args.output/'robot-programs.json',scene.bindings.programs)
        write(args.output/'skill-bindings.json',scene.bindings.capabilities())
        print('PLAN COMMITTED '+team.agreement.committed['plan_hash'],flush=True)
        for rid,program in scene.bindings.programs.items():
            row=program[0]
            print(f"{rid}: {row['object']} | {row['role']} | partners={row['participants']} | route={row['route']} | after={row['after']}",flush=True)
        # Preserve the accepted plan. Do not silently replace its route or move
        # a loaded formation through a known insufficient static clearance.
        scene.bindings.check_route()
        motion_started_wall=time.monotonic();motion_started_sim=scene.time()
        scene.start_solo()
        result['rolling_approach']['applied']=scene.rolling_visual_servo
        result['bounded_carrier_relink']['applied']=scene.bounded_carrier_relink
        result['view_recovery']['applied']=scene.rolling_view_recovery
        pair=BoundPairSkill(scene,scene.bindings,skill,grasp,stages,grasp_root,reference,identity)
        result['coarse_concurrent_alignment'].update(coarse_concurrency_status(
            pair,requested=scene.coarse_concurrent_alignment))
        dynamic=getattr(args,'coordination','plan_first')=='dynamic'
        if dynamic:
            scene.team_event_handler=lambda event:_box_event(scene,team,task,args,result,event)
            scene.diagnostic_fail_box_once=getattr(args,'diagnostic_fail_box_once',False)
        try:
            _beam_job(pair,scene,team,task,args,result,grasp)
        except Exception as exc:
            if not dynamic or isinstance(exc,TeamStop) or getattr(exc,'component',None):raise
            _job_failed(scene,team,task,result,'beam',exc,
                        set_down=pair.set_down,hold=pair._hold_pair)
        while not scene.bindings.settled('box'):scene.step(.2)
        dropped=result.get('coordination',{}).get('dropped_jobs')
        if dropped:
            result['phase']='FINISHED_PARTIAL'
        else:
            result['protocol_complete']=True;result['phase']='FINISHED'
    except (Exception,KeyboardInterrupt) as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
        result['failed_component']=getattr(exc,'component',
            'pair' if result['phase'] in ('APPROACH','GRASP','TRANSIT','RELEASE') else 'runtime')
        if args.output.exists():(args.output/'exception.txt').write_text(traceback.format_exc())
    finally:
        result.pop('_injected',None)
        if getattr(scene,'solo_events',None):
            result.setdefault('coordination',{})['box_events']=copy.deepcopy(scene.solo_events)
        cleanup_started=time.monotonic()
        if scene.world:
            timing=result.setdefault('timing',{})
            timing.update(control_end_sim_s=scene.time(),control_wall_s=cleanup_started-started)
            if motion_started_wall is not None:
                motion_wall_s=cleanup_started-motion_started_wall
                motion_sim_s=scene.time()-motion_started_sim
                timing.update(motion_wall_s=motion_wall_s,motion_sim_s=motion_sim_s,
                              motion_sim_to_wall=motion_sim_s/motion_wall_s)
        try:
            if scene.world:
                if scene.native_view:
                    scene.native_view.close();scene.native_view=None
                # Freeze decisions before final referee-only settling.
                scene._clear_rolling_view_recovery(scene.time(),'trial_end')
                scene._clear_solo_approach_lease(scene.time(),'trial_end')
                if scene.solo_executor:scene.solo_executor.cancel(scene.time(),'trial_end')
                if scene.solo:result['solo_status']={'phase':scene.solo.phase,'reason':scene.solo.reason,'done':scene.solo.done}
                if scene._view_recovery is not None:
                    result['view_recovery']['episodes_started']=scene._view_recovery.episodes_started
                    result['view_recovery']['poses_issued']=scene._view_recovery.poses_issued
                    result['view_recovery']['holds_started']=scene._view_recovery.holds_started
                    result['view_recovery']['settled_observations']=scene._view_recovery.settled_observations
                if scene.bindings:result['resource_events']=scene.bindings.resource_events
                if scene.realtime_control:result['realtime_control_stats']=dict(scene.realtime_stats)
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
                if getattr(scene,'guidance',None):
                    result['plan_guidance']=scene.guidance.record()
                    write(args.output/'plan-previews.json',scene.guidance.preview_log)
                write(args.output/'issued-commands.json',scene.command_history)
                write(args.output/'solo-decisions.json',scene.solo_rows);write(args.output/'solo-raw-actions.json',scene.solo_raw)
                write(args.output/'solo-renewals.json',scene.solo_renewals)
                write(args.output/'solo-yield.json',scene.yield_rows)
                if pair:
                    write(args.output/'pair-decisions.json',pair.calls)
                    write(args.output/'pair-grasp.json',pair.grasp_report)
                    write(args.output/'pair-replay.json',pair.trace)
        finally:
            for cleanup in (lambda:team.close(scene.time() if scene.world else 0.) if team else None,
                            lambda:scene.video.close() if scene.video else None,
                            lambda:result.__setitem__('replay',scene.replay.close()) if scene.replay else None,
                            scene.close):
                try:cleanup()
                except Exception as exc:result.setdefault('cleanup_errors',[]).append(str(exc))
        result['wall_s']=time.monotonic()-started
        result.setdefault('timing',{})['cleanup_wall_s']=time.monotonic()-cleanup_started
        if team:
            result['protocol_calls']=len(team.calls)
            result['llm_calls']=sum(c.get('model')!='scripted-fixture-not-llm' for c in team.calls)
            result['usage']={k:sum((c.get('usage') or {}).get(k,0) for c in team.calls)
                for k in ('prompt_tokens','completion_tokens','total_tokens')}
        args.output.mkdir(parents=True,exist_ok=True);write(args.output/'result.json',result)
    print(json.dumps({k:result.get(k) for k in ('phase','error','plan_committed','protocol_complete','physical_success','wall_s')}),flush=True)
    return 0 if result['physical_success'] and result['protocol_complete'] else 1
