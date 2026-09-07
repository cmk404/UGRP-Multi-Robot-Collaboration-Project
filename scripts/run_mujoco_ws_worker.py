#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
import os
import secrets
import time
from pathlib import Path

import cv2
from websockets.sync.client import connect

from sim.multi_masterpi_production import MultiMasterPiProductionV2, ROBOT_IDS
from sim.self_observer import diagnose_action
from sim.worker_contract import REMOTE_WORKER_CONTRACT

SPEED_FILE = Path(os.environ.get('UGRP_SIM_SPEED_FILE','/content/ugrp_sim_speed'))
FIRST_PERSON_CAMERA = 'robot_cam'
THIRD_PERSON_CAMERA = 'cctv_front_left'
TEAM_OVERVIEW_CAMERA = 'cctv_front_right'
FIRST_PERSON_JPEG_QUALITY = int(os.environ.get('UGRP_SIM_FIRST_PERSON_JPEG_QUALITY', '76'))
THIRD_PERSON_JPEG_QUALITY = int(os.environ.get('UGRP_SIM_THIRD_PERSON_JPEG_QUALITY', '86'))
STREAM_INTERVAL_S = max(0.04, float(os.environ.get('UGRP_SIM_GPU_STREAM_INTERVAL', '0.05')))
OBSERVER_INTERVAL_S = max(STREAM_INTERVAL_S, float(os.environ.get('UGRP_SIM_GPU_OBSERVER_INTERVAL', '0.20')))
# The fixed TEAM overview is presentation-only.  Rendering it several times per
# second made the long physical tower path take ~330 s wall-clock even at 3x,
# while the same dynamics path finishes in ~106-131 s without presentation
# overhead.  ~1.3 fps still shows continuous motion in the TEAM panel without
# starving MuJoCo physics callbacks.
TEAM_OVERVIEW_INTERVAL_S = max(0.10, float(os.environ.get('UGRP_SIM_TEAM_OVERVIEW_INTERVAL', '0.75')))
# Presentation is best-effort.  These token buckets keep an observer or team
# overview render from being scheduled back-to-back when MuJoCo is already
# busy with an action.  A stream_control message may lower these rates without
# changing the controller's sensor cadence.
OBSERVER_RENDER_RATE = max(0.5, float(os.environ.get('UGRP_SIM_OBSERVER_RENDER_RATE', str(1.0 / OBSERVER_INTERVAL_S))))
TEAM_OVERVIEW_RENDER_RATE = max(0.5, float(os.environ.get('UGRP_SIM_TEAM_OVERVIEW_RENDER_RATE', str(1.0 / TEAM_OVERVIEW_INTERVAL_S))))
# One 640x480 robot-camera render costs ~40-50 ms on the Mac worker, i.e. about
# as long as STREAM_INTERVAL_S, so a pure wall-clock gate let rendering eat
# more than half of every action (a 1.6 s headless approach took 40+ s and hit
# the bridge timeout). After each render the next one is deferred by this
# factor times the render cost, bounding presentation to ~1/(1+factor) of wall.
RENDER_DUTY_FACTOR = max(0.0, float(os.environ.get('UGRP_SIM_RENDER_DUTY_FACTOR', '3.0')))
TRACE_INTERVAL_S = max(0.20, float(os.environ.get('UGRP_SIM_TRACE_INTERVAL', '0.40')))
TRACE_MAX_FRAMES = max(4, int(os.environ.get('UGRP_SIM_TRACE_MAX_FRAMES', '36')))
COMPLETED_RESULTS_MAX = 8
PROVIDER = os.environ.get('UGRP_GPU_PROVIDER','remote').strip().lower() or 'remote'
MACHINE = os.environ.get('UGRP_GPU_MACHINE','GPU').strip().upper() or 'GPU'
INSTANCE_ID = os.environ.get('UGRP_GPU_INSTANCE_ID','').strip() or secrets.token_hex(8)
ACTIVITY_FILE = Path(os.environ.get('UGRP_USER_ACTIVITY_FILE','/tmp/ugrp_sim_user_activity'))
# ``sync`` is an authority/state fence.  New bridges follow it with the
# subscriber-aware stream_control message.  Keep an opt-in escape hatch for
# an older bridge that has no stream_control support; the default must remain
# headless so authority handoff with zero browser subscribers does not wake the
# expensive renderer.
LEGACY_SYNC_STREAM = os.environ.get('UGRP_SIM_LEGACY_SYNC_STREAM', '0').strip().lower() in {'1', 'true', 'yes'}


class PresentationTokenBucket:
    """Small, single-threaded token bucket for droppable presentation work."""

    def __init__(self, rate_per_s: float, capacity: float = 1.0):
        self.rate_per_s = max(0.0, float(rate_per_s))
        self.capacity = max(1.0, float(capacity))
        self.tokens = self.capacity
        self.last_refill = time.monotonic()

    def try_consume(self, cost: float = 1.0, *, force: bool = False) -> bool:
        now = time.monotonic()
        elapsed = max(0.0, now - self.last_refill)
        self.last_refill = now
        self.tokens = min(self.capacity, self.tokens + elapsed * self.rate_per_s)
        amount = max(0.0, float(cost))
        if force:
            # Boundary frames are explicit snapshots, not a continuous stream.
            # Do not let a stale bucket suppress the one frame users expect.
            self.tokens = max(0.0, self.tokens - min(amount, self.capacity))
            return True
        if self.tokens + 1e-9 < amount:
            return False
        self.tokens -= amount
        return True


def touch_activity():
    try:
        ACTIVITY_FILE.parent.mkdir(parents=True,exist_ok=True)
        ACTIVITY_FILE.touch()
    except Exception:
        pass


def speed():
    try: x=float(SPEED_FILE.read_text().strip())
    except Exception: x=1.0
    return x if x in (1.0,2.0,3.0) else 1.0


def cached_robot_jpeg(world, robot_id="r1", quality=FIRST_PERSON_JPEG_QUALITY):
    view=world.robot(robot_id) if hasattr(world, "robot") else world
    bgr=getattr(view,'_latest_robot_bgr',None)
    if bgr is not None:
        ok,encoded=cv2.imencode('.jpg',bgr,[int(cv2.IMWRITE_JPEG_QUALITY),int(quality)])
        if ok:
            return encoded.tobytes()
    try:
        return world.render_jpeg(robot_id=robot_id,camera=FIRST_PERSON_CAMERA,quality=quality)
    except TypeError:
        return world.render_jpeg(camera=FIRST_PERSON_CAMERA,quality=quality)

def frame_msg(world, robot_id="r1", *, state_seq=None, robot_jpeg=None, generated_wall_s=None, render_ms=None):
    """Build one robot's first-person publication in the shared team world."""
    robot_jpeg=robot_jpeg if robot_jpeg is not None else cached_robot_jpeg(world,robot_id,FIRST_PERSON_JPEG_QUALITY)
    obj={
        'type':'frame','robot_id':robot_id,
        'jpeg_b64':base64.b64encode(robot_jpeg).decode(),
        'state':world.robot_state(robot_id) if hasattr(world, 'robot_state') else world.state(),
    }
    if state_seq is not None: obj['state_seq']=int(state_seq)
    if generated_wall_s is not None: obj['generated_wall_s']=float(generated_wall_s)
    if render_ms is not None: obj['render_ms']=round(float(render_ms),3)
    return obj


def live_camera_msg(world, robot_id="r1", *, state_seq=None):
    started=time.perf_counter()
    # SimCamera.read() already rendered the actor-visible sensor frame and
    # stores it on the namespaced controller.  Re-rendering the same camera for
    # the browser was the dominant idle/action presentation cost.  Fall back to
    # a fresh render only before the first controller sample exists.
    view=world.robot(robot_id) if hasattr(world, "robot") else world
    cached=getattr(view, '_latest_robot_bgr', None)
    cached_seq=getattr(view, '_latest_robot_frame_seq', None)
    cached_jpeg=getattr(view, '_latest_robot_jpeg', None)
    cached_jpeg_seq=getattr(view, '_latest_robot_jpeg_seq', None)
    cached_jpeg_quality=getattr(view, '_latest_robot_jpeg_quality', None)
    presentation_dirty=bool(getattr(view, '_presentation_dirty', False))
    if (
        not presentation_dirty
        and
        cached_jpeg is not None
        and cached_seq is not None
        and cached_jpeg_seq == cached_seq
        and cached_jpeg_quality == FIRST_PERSON_JPEG_QUALITY
    ):
        robot_jpeg=cached_jpeg
    elif cached is not None and not presentation_dirty:
        ok, encoded=cv2.imencode('.jpg', cached, [int(cv2.IMWRITE_JPEG_QUALITY), int(FIRST_PERSON_JPEG_QUALITY)])
        if ok:
            robot_jpeg=encoded.tobytes()
            if cached_seq is not None:
                # The controller's sensor sequence is immutable for this
                # sample, so later stream ticks can reuse the encoded bytes as
                # well as the underlying BGR frame.
                try:
                    view._latest_robot_jpeg=robot_jpeg
                    view._latest_robot_jpeg_seq=cached_seq
                    view._latest_robot_jpeg_quality=FIRST_PERSON_JPEG_QUALITY
                except Exception:
                    pass
        else:
            robot_jpeg=None
    else:
        robot_jpeg=None
    if robot_jpeg is None:
        try:
            robot_jpeg=world.render_jpeg(robot_id=robot_id,camera=FIRST_PERSON_CAMERA,quality=FIRST_PERSON_JPEG_QUALITY)
        except TypeError:
            robot_jpeg=world.render_jpeg(camera=FIRST_PERSON_CAMERA,quality=FIRST_PERSON_JPEG_QUALITY)
        # A static idle stream can reuse this presentation snapshot until the
        # next physics step or controller sensor sample marks it dirty.
        try:
            seq=int(getattr(view, '_latest_robot_frame_seq', 0) or 0) + 1
            view._latest_robot_frame_seq=seq
            view._latest_robot_jpeg=robot_jpeg
            view._latest_robot_jpeg_seq=seq
            view._latest_robot_jpeg_quality=FIRST_PERSON_JPEG_QUALITY
            view._presentation_dirty=False
        except Exception:
            pass
    render_ms=(time.perf_counter()-started)*1000.0
    return frame_msg(
        world,robot_id,state_seq=state_seq,robot_jpeg=robot_jpeg,
        generated_wall_s=time.time(),render_ms=render_ms,
    )


def observer_camera_msg(world, robot_id='r2', *, name=THIRD_PERSON_CAMERA):
    """One shared presentation camera follows the selected robot; never planner input."""
    started=time.perf_counter()
    try:
        jpeg=world.render_jpeg(robot_id=robot_id,camera=name,quality=THIRD_PERSON_JPEG_QUALITY)
    except TypeError:
        jpeg=world.render_jpeg(camera=name,quality=THIRD_PERSON_JPEG_QUALITY)
    render_ms=(time.perf_counter()-started)*1000.0
    return {
        'type':'observer_frame','robot_id':robot_id,'name':name,
        'jpeg_b64':base64.b64encode(jpeg).decode(),
        'generated_wall_s':time.time(),'render_ms':round(render_ms,3),
    }

def team_overview_camera_msg(world, *, name='team_overview'):
    """Fixed shared-world view used only by the TEAM presentation panel."""
    started=time.perf_counter()
    camera = 'cctv_warehouse' if getattr(world, 'warehouse_mission', None) else TEAM_OVERVIEW_CAMERA
    jpeg=world.render_team_jpeg(camera=camera,quality=THIRD_PERSON_JPEG_QUALITY)
    return {
        'type':'observer_frame','name':name,
        'jpeg_b64':base64.b64encode(jpeg).decode(),
        'generated_wall_s':time.time(),
        'render_ms':round((time.perf_counter()-started)*1000.0,3),
    }

def run(url, token, seed):
    # One authoritative MuJoCo model/data contains all three robots. Controller
    # actions remain serialized by MultiMasterPiProductionV2 so the migrated REAL
    # stack's process-global compatibility shims cannot leak between robot slots.
    world=MultiMasterPiProductionV2(seed=seed, warehouse_layout=os.environ.get('UGRP_WAREHOUSE_LAYOUT','mixed'))
    world.set_speed_multiplier(speed())
    last_state=0.0
    last_frame={rid:0.0 for rid in ROBOT_IDS}
    next_frame_at={rid:0.0 for rid in ROBOT_IDS}
    last_observer=0.0
    last_team_overview=0.0
    next_observer_at={'value':0.0}
    next_team_overview_at={'value':0.0}
    observer_index={'value':0}
    stream_enabled={'value':False}
    # ``sync`` is followed by stream_control on the current bridge.  Keep the
    # default headless; set UGRP_SIM_LEGACY_SYNC_STREAM=1 only for an older
    # bridge that cannot publish subscriber state.
    stream_config={'robot_ids':None,'observer':True,'team_overview':True}
    observer_bucket=PresentationTokenBucket(OBSERVER_RENDER_RATE)
    team_overview_bucket=PresentationTokenBucket(TEAM_OVERVIEW_RENDER_RATE)
    publication_seq={'value':time.time_ns()}
    def next_seq():
        publication_seq['value']=max(publication_seq['value']+1,time.time_ns())
        return publication_seq['value']
    ws_box={'ws':None}
    command_active={'value':False,'robot_id':'r1','action':''}
    active_trace={'frames':[], 'last_capture':0.0}
    last_activity_touch={'value':0.0}

    def emit_live(*, force=False, robot_ids=None):
        nonlocal last_state,last_observer,last_team_overview
        now=time.monotonic(); ws=ws_box['ws']
        if ws is None:
            return
        # Forensic traces are independent of presentation subscribers. Reuse
        # only a controller-observed frame here: headless execution must never
        # wake MuJoCo rendering just to populate a trace.
        if not stream_enabled['value']:
            active_rid=str(command_active.get('robot_id') or 'r1')
            view=world.robot(active_rid) if hasattr(world, 'robot') else world
            if (
                command_active['value']
                and getattr(view, '_latest_robot_bgr', None) is not None
                and len(active_trace['frames']) < TRACE_MAX_FRAMES
                and (force or now-active_trace['last_capture'] >= TRACE_INTERVAL_S)
            ):
                live=live_camera_msg(world,active_rid,state_seq=next_seq())
                # Keep the planner-facing /snapshot current without enabling a
                # presentation stream. This reuses the exact controller frame
                # and therefore adds no MuJoCo render.
                ws.send(json.dumps(live,separators=(',',':')))
                active_trace['frames'].append({
                    't_monotonic':round(now,6),'robot_id':active_rid,
                    'robot_jpeg_b64':live.get('jpeg_b64'),
                    'observer_jpeg_b64':None,
                    'state':live.get('state') or {},
                })
                active_trace['last_capture']=now
            return
        # ``None`` means the normal all-robot publication set.  An explicit
        # empty tuple is useful for long shared actions such as team_tower,
        # where the fixed overview is the important live view and rendering a
        # first-person camera on every physics callback would dominate wall
        # time.
        configured_ids=stream_config['robot_ids']
        if robot_ids is None:
            ids=tuple(ROBOT_IDS if configured_ids is None else configured_ids)
        else:
            requested=tuple(robot_ids)
            ids=tuple(rid for rid in requested if configured_ids is None or rid in configured_ids)
        try:
            if command_active['value'] and now-last_activity_touch['value'] >= 20.0:
                touch_activity(); last_activity_touch['value']=now
            if now-last_state >= 0.10:
                ws.send(json.dumps({'type':'state','state':world.state(),'state_seq':next_seq()},separators=(',',':')))
                last_state=now
            live_for_trace=None
            for rid in ids:
                if force or (now-last_frame[rid] >= STREAM_INTERVAL_S and now >= next_frame_at[rid]):
                    render_started=time.monotonic()
                    live=live_camera_msg(world,rid,state_seq=next_seq())
                    ws.send(json.dumps(live,separators=(',',':')))
                    last_frame[rid]=now
                    next_frame_at[rid]=time.monotonic()+RENDER_DUTY_FACTOR*(time.monotonic()-render_started)
                    if rid == command_active.get('robot_id'):
                        live_for_trace=live
            observer=None
            active_rid=str(command_active.get('robot_id') or 'r1')
            # cctv_front_left is a movable follow camera in the shared world.
            # Publishing only active_rid made every robot page show whichever
            # robot moved most recently.  On forced boundaries publish the
            # requested robot slots; while idle/action callbacks run, refresh one
            # slot per interval round-robin so all three views remain current
            # without tripling every first-person render.
            if not stream_config['observer']:
                observer_rids=()
            elif force:
                observer_rids=ids
            elif ids and (
                now-last_observer >= OBSERVER_INTERVAL_S
                and now >= next_observer_at['value']
                and observer_bucket.try_consume()
            ):
                observer_pool=ids
                i=observer_index['value'] % len(observer_pool)
                observer_rids=(observer_pool[i],)
                observer_index['value'] += 1
            else:
                observer_rids=()
            for observer_rid in observer_rids:
                render_started=time.monotonic()
                candidate=observer_camera_msg(world,observer_rid)
                ws.send(json.dumps(candidate,separators=(',',':')))
                if observer_rid == active_rid:
                    observer=candidate
                last_observer=now
                next_observer_at['value']=time.monotonic()+RENDER_DUTY_FACTOR*(time.monotonic()-render_started)
            # TEAM tower is a long, visibly coordinated action. Publish its
            # fixed overview throughout execution so the browser shows the
            # robots driving, grasping and carrying instead of jumping from a
            # start frame to a completed tower. Other actions keep the cheaper
            # boundary-only overview behavior.
            publish_team_overview = stream_config['team_overview'] and (force or (
                command_active['value']
                and command_active.get('action') == 'team_tower'
                and now-last_team_overview >= TEAM_OVERVIEW_INTERVAL_S
                and now >= next_team_overview_at['value']
                and team_overview_bucket.try_consume()
            ))
            if publish_team_overview:
                render_started=time.monotonic()
                ws.send(json.dumps(team_overview_camera_msg(world),separators=(',',':')))
                last_team_overview=now
                next_team_overview_at['value']=time.monotonic()+RENDER_DUTY_FACTOR*(time.monotonic()-render_started)
            if command_active['value'] and live_for_trace is not None and len(active_trace['frames']) < TRACE_MAX_FRAMES and (
                force or now-active_trace['last_capture'] >= TRACE_INTERVAL_S
            ):
                if observer is None and stream_config['observer']:
                    observer=observer_camera_msg(world,active_rid)
                    ws.send(json.dumps(observer,separators=(',',':')))
                    last_observer=now
                if observer is not None:
                    active_trace['frames'].append({
                    't_monotonic':round(now,6),'robot_id':active_rid,
                    'robot_jpeg_b64':live_for_trace.get('jpeg_b64'),
                    'observer_jpeg_b64':observer.get('jpeg_b64'),
                    'state':live_for_trace.get('state') or {},
                    })
                    active_trace['last_capture']=now
        except Exception:
            pass

    # The composed production world owns one callback because MuJoCo time is
    # shared. During an action the callback publishes only that active robot;
    # idle ticks still refresh all three slots independently.
    world.frame_callback=lambda: emit_live(
        force=False,
        robot_ids=(
            () if command_active.get('action') == 'team_tower'
            else (str(command_active.get('robot_id') or 'r1'),)
        ),
    )

    completed_results: dict[str, dict] = {}
    while True:
        try:
            with connect(url,open_timeout=15,close_timeout=5,max_size=32*1024*1024,compression=None,ping_interval=None,ping_timeout=None) as ws:
                ws_box['ws']=ws; stream_enabled['value']=False
                ws.send(json.dumps({'type':'auth','token':token,'provider':PROVIDER,'machine':MACHINE,'instance_id':INSTANCE_ID,'worker_contract':REMOTE_WORKER_CONTRACT}))
                auth=json.loads(ws.recv())
                if auth.get('type')!='auth_ok': raise RuntimeError('auth failed')
                print('UGRP WS 3x GPU worker connected',{'speed':speed(),'robots':list(ROBOT_IDS)},flush=True)
                while True:
                    try: raw=ws.recv(timeout=STREAM_INTERVAL_S)
                    except TimeoutError:
                        emit_live(force=False); continue
                    msg=json.loads(raw); typ=msg.get('type')
                    if typ=='ping':
                        try:
                            if float(msg.get('user_activity_age_s',1e9)) < 75.0: touch_activity()
                        except Exception: pass
                        ws.send(json.dumps({'type':'pong','t':msg.get('t'),'worker_wall_s':time.time()})); continue
                    if typ=='sync':
                        stream_enabled['value']=LEGACY_SYNC_STREAM
                        stream_config.update(robot_ids=None,observer=True,team_overview=True)
                        if stream_enabled['value']:
                            emit_live(force=True)
                        else:
                            # Authority handoff clears Bridge frame caches. Pay
                            # a one-time robot-camera render per slot so headless
                            # planner snapshots are immediately usable; ongoing
                            # idle presentation remains disabled.
                            for rid in ROBOT_IDS:
                                try:
                                    jpeg=world.render_jpeg(
                                        robot_id=rid,camera=FIRST_PERSON_CAMERA,
                                        quality=FIRST_PERSON_JPEG_QUALITY,
                                    )
                                except TypeError:
                                    jpeg=world.render_jpeg(
                                        camera=FIRST_PERSON_CAMERA,
                                        quality=FIRST_PERSON_JPEG_QUALITY,
                                    )
                                ws.send(json.dumps(frame_msg(
                                    world,rid,state_seq=next_seq(),robot_jpeg=jpeg,
                                    generated_wall_s=time.time(),
                                ),separators=(',',':')))
                                try:
                                    view=world.robot(rid) if hasattr(world,'robot') else world
                                    seq=int(getattr(view,'_latest_robot_frame_seq',0) or 0)+1
                                    view._latest_robot_frame_seq=seq
                                    view._latest_robot_jpeg=jpeg
                                    view._latest_robot_jpeg_seq=seq
                                    view._latest_robot_jpeg_quality=FIRST_PERSON_JPEG_QUALITY
                                    view._presentation_dirty=False
                                except Exception:
                                    pass
                        # Frame messages carry one robot-local state. Publish a
                        # higher-sequence shared state after all three so Bridge
                        # health/TEAM mission UI retains robots + payload/zones
                        # instead of ending startup on the last R3 frame.
                        ws.send(json.dumps({
                            'type':'state','state':world.state(),'state_seq':next_seq(),
                        },separators=(',',':')))
                        continue
                    if typ=='stream_control':
                        # Subscriber-aware presentation control.  All fields are
                        # optional so a bridge can change only one surface:
                        # {enabled, robot_ids, observer, team_overview}.
                        previous_enabled=stream_enabled['value']
                        previous_config=(stream_config['robot_ids'], stream_config['observer'], stream_config['team_overview'])
                        stream_enabled['value']=bool(msg.get('enabled', True))
                        raw_ids=msg.get('robot_ids')
                        if raw_ids is None:
                            stream_config['robot_ids']=None
                        elif isinstance(raw_ids, (list, tuple, set)):
                            stream_config['robot_ids']=tuple(
                                rid for rid in ROBOT_IDS if str(rid) in {str(v) for v in raw_ids}
                            )
                        stream_config['observer']=bool(msg.get('observer', stream_config['observer']))
                        stream_config['team_overview']=bool(msg.get('team_overview', stream_config['team_overview']))
                        next_config=(stream_config['robot_ids'], stream_config['observer'], stream_config['team_overview'])
                        if stream_enabled['value'] and (not previous_enabled or next_config != previous_config):
                            emit_live(force=True)
                        continue
                    if typ!='command': continue
                    cmd=msg.get('command') or {}; cid=str(cmd.get('id')); action=str(cmd.get('action'))
                    if cid in completed_results:
                        # The bridge redelivers an inflight command after a socket
                        # blip. The world already moved; replaying pick/carry/place
                        # would double-apply it, so resend the stored result instead.
                        ws.send(json.dumps({'type':'result','id':cid,'result':completed_results[cid]},separators=(',',':')))
                        ws.send(json.dumps({'type':'state','state':world.state(),'state_seq':next_seq()},separators=(',',':')))
                        print('redelivered result for',cid,action,flush=True); continue
                    if isinstance(cmd.get('commands'), list) and hasattr(world, 'act_parallel'):
                        touch_activity(); last_activity_touch['value']=time.monotonic()
                        incoming_speed=float(msg.get('sim_speed') or speed()); world.set_speed_multiplier(incoming_speed)
                        parallel_commands=list(cmd.get('commands') or [])
                        command_by_robot={
                            str(item.get('robot_id') or ''):item
                            for item in parallel_commands if isinstance(item,dict)
                        }
                        def emit_partial_result(rid,result,timing):
                            member=command_by_robot.get(rid) or {}
                            member_id=str(member.get('id') or '')
                            if not member_id:
                                return
                            item={
                                'ok':result.ok,'action':result.action,'robot_id':rid,
                                'reason':result.reason,'state':result.state,
                                'worker_action_s':timing.get('duration_s'),
                                'parallel_member_timing':timing,
                            }
                            ws.send(json.dumps({
                                'type':'partial_result','id':cid,
                                'member_id':member_id,'result':item,
                            },separators=(',',':')))
                        t=time.perf_counter()
                        parallel_results=world.act_parallel(
                            parallel_commands,on_result=emit_partial_result,
                        )
                        action_s=time.perf_counter()-t
                        payload={
                            'ok':all(bool(r.ok) for r in parallel_results.values()),
                            'action':'parallel',
                            'robot_ids':list(parallel_results.keys()),
                            'reason':'parallel multi-robot command completed',
                            'results':{
                                rid:{
                                    'ok':r.ok,'action':r.action,'reason':r.reason,'state':r.state,
                                    'worker_action_s':(
                                        (getattr(world,'last_parallel_timing',{}) or {})
                                        .get('robots',{}).get(rid,{}).get('duration_s')
                                    ),
                                }
                                for rid,r in parallel_results.items()
                            },
                            'state':world.state(),
                            'worker_action_s':round(action_s,3),
                            'parallel_timing':dict(getattr(world,'last_parallel_timing',{}) or {}),
                        }
                        completed_results[cid]=payload
                        ws.send(json.dumps({'type':'result','id':cid,'result':payload},separators=(',',':')))
                        ws.send(json.dumps({'type':'state','state':world.state(),'state_seq':next_seq()},separators=(',',':')))
                        print('parallel',list(parallel_results.keys()),'action_s',round(action_s,3),flush=True)
                        continue
                    robot_id=str(cmd.get('robot_id') or 'r1').strip().lower()
                    if robot_id not in ROBOT_IDS:
                        payload={'ok':False,'action':action,'robot_id':robot_id,'reason':'invalid robot_id','failure_code':'INVALID_ROBOT_ID'}
                        ws.send(json.dumps({'type':'result','id':cid,'result':payload},separators=(',',':'))); continue
                    touch_activity(); command_active.update(value=True,robot_id=robot_id,action=action); last_activity_touch['value']=time.monotonic()
                    active_trace['frames']=[]; active_trace['last_capture']=0.0
                    incoming_speed=float(msg.get('sim_speed') or speed()); world.set_speed_multiplier(incoming_speed)
                    if action == 'warehouse_research':
                        emit_live(force=True, robot_ids=ROBOT_IDS)
                        t=time.perf_counter()
                        result=world.act(robot_id, action, request=cmd.get('request'))
                        payload={
                            'ok':result.ok,'action':action,'robot_id':robot_id,
                            'reason':result.reason,'research':result.state.get('research', {}),
                            'worker_action_s':round(time.perf_counter()-t,3),
                        }
                        completed_results[cid]=payload
                        ws.send(json.dumps({'type':'result','id':cid,'result':payload},separators=(',',':')))
                        ws.send(json.dumps({'type':'state','state':world.state(),'state_seq':next_seq()},separators=(',',':')))
                        emit_live(force=True, robot_ids=ROBOT_IDS)
                        command_active['value']=False
                        continue
                    before_state=world.robot_state(robot_id)
                    before_detections=world.scene_detections(robot_id)
                    emit_live(force=True,robot_ids=(robot_id,))
                    t=time.perf_counter()
                    call_params={k:v for k,v in cmd.items() if k not in {
                        'id','action','created','robot_id','team_batch_id','team_batch_expected'
                    }}
                    result=world.act(robot_id,action,**call_params)
                    action_s=time.perf_counter()-t
                    detections=world.scene_detections(robot_id)
                    vision=detections.get('red'); yellow=detections.get('yellow'); blue=detections.get('blue')
                    after_state=world.robot_state(robot_id)
                    diagnostic=diagnose_action(
                        action=action,params=call_params,before=before_state,after=after_state,
                        result_ok=bool(result.ok),result_reason=str(result.reason),
                        before_vision=before_detections,after_vision=detections,
                    )
                    emit_live(force=True,robot_ids=(robot_id,))
                    result_seq=next_seq()
                    payload={
                        'ok':result.ok,'action':result.action,'robot_id':robot_id,'reason':result.reason,
                        'state':after_state,'state_seq':result_seq,'sim_speed':incoming_speed,
                        'worker_action_s':round(action_s,3),'transport':'websocket',
                    }
                    payload['_sim_diagnostic']=diagnostic
                    payload['_sim_trace']={
                        'schema':'ugrp.sim.trace.v1','command_id':cid,'action':action,'robot_id':robot_id,
                        'params':call_params,'seed':after_state.get('seed'),'world_seed':after_state.get('world_seed'),'before_state':before_state,
                        'after_state':after_state,'frames':list(active_trace['frames']),'worker_action_s':round(action_s,6),
                    }
                    if vision is not None: payload['vision']=vision
                    if yellow is not None: payload['yellow_vision']=yellow
                    if blue is not None: payload['blue_vision']=blue
                    payload['spatial_memory']=world.spatial_memory_public(robot_id)
                    completed_results[cid]=payload
                    while len(completed_results) > COMPLETED_RESULTS_MAX:
                        completed_results.pop(next(iter(completed_results)))
                    ws.send(json.dumps({'type':'result','id':cid,'result':payload},separators=(',',':')))
                    ws.send(json.dumps({'type':'state','state':world.state(),'state_seq':next_seq()},separators=(',',':')))
                    command_active.update(value=False,action=''); active_trace['frames']=[]; touch_activity(); emit_live(force=True,robot_ids=(robot_id,))
                    print(robot_id,action,result.ok,'action_s',round(action_s,3),flush=True)
        except Exception as e:
            stream_enabled['value']=False; command_active.update(value=False,action=''); ws_box['ws']=None
            print('ws reconnect',repr(e),flush=True); time.sleep(1)

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--url',default=os.environ.get('UGRP_SIM_WS_URL')); ap.add_argument('--token',default=os.environ.get('UGRP_SIM_TOKEN')); ap.add_argument('--seed',type=int,default=11); a=ap.parse_args()
    if not a.url or not a.token: raise SystemExit('set UGRP_SIM_WS_URL and UGRP_SIM_TOKEN')
    run(a.url,a.token,a.seed)
