"""ACT replaces only loaded motion. Existing task grants and RGB guards remain."""
import json
import time
import copy
import math
from pathlib import Path
from collections import deque
from harness.pair_carry_act_contract import context,AXES
from harness.pair_carry_act_client import CarryClient
from harness.dispatch_own_hold import OwnHoldContinuity
from harness.pair_carry_sync import PairCarrySync
from harness.dispatch_pair_navigation import authorize_pair


def observe_act(pair, tag, predict, *, not_before=None):
    """Keep owner physics running; accept only a coherent, fresh raw RGB batch."""
    robots=tuple(pair.bindings.pair.values())
    if not getattr(pair.io,'realtime_control',False):
        frames=pair.io.capture(tag,own_robots=robots,overview=False)
        return frames,predict(frames)
    from sim.snapshot_contract import SnapshotBackpressure
    for attempt in range(3):
        # Image records are immutable; each reobservation needs a fresh path.
        capture_tag=tag+'-attempt-'+str(attempt)
        deadline=pair.time()+2.
        while True:
            try:
                future=pair.io.capture_async(capture_tag,own_robots=robots,overview=False)
                if future is not None:break
            except SnapshotBackpressure:
                pass
            if pair.time()>=deadline:
                pair._hold_pair()
                raise RuntimeError('ACT capture backpressure exceeded 2 SIM seconds')
            pair.tick(.02)
        frames=pair.io.await_visual(future)
        first=frames[robots[0]]
        observed=float(first['observed_at_s'])
        if (not math.isfinite(observed) or observed>pair.time()+1e-9
                or any(f['frame_id']!=first['frame_id']
                       or float(f['observed_at_s'])!=observed for f in frames.values())):
            pair._hold_pair()
            raise ValueError('ACT requires coherent capture timestamps and frame IDs')
        if pair.time()-observed>.6:
            pair._hold_pair()
            pair.calls.append({'kind':'act_stale_capture','tag':tag,
                'frame_id':first['frame_id'],'observed_at_s':observed,
                'received_at_s':pair.time()})
            continue
        prediction=pair.io.compute_visual(lambda:predict(frames))
        if isinstance(prediction,dict) and prediction.get('error'):
            pair._hold_pair()
            pair.calls.append({'kind':'act_inference_error','tag':tag,
                'index':prediction['index'],'frame_id':first['frame_id'],
                'observed_at_s':observed,'received_at_s':pair.time(),
                'inputs':prediction['inputs'],'decisions':prediction['decisions'],
                'error':prediction['error']})
            raise RuntimeError('ACT inference failed: '+prediction['error'])
        while not_before is not None and pair.time()<not_before-1e-9:
            pair.tick(min(.02,not_before-pair.time()))
        if pair.time()-observed<=.6:
            return frames,prediction
        pair._hold_pair()
        # Preserve even unused model requests; stale inference never issues.
        record={'kind':'act_stale_prediction','tag':tag,'attempt':attempt,
                'frame_id':first['frame_id'],'observed_at_s':observed,
                'received_at_s':pair.time()}
        if isinstance(prediction,dict):
            record.update(index=prediction.get('index'),inputs=prediction.get('inputs'),
                          decisions=prediction.get('decisions'))
        pair.calls.append(record)
    raise RuntimeError('ACT RGB/prediction remained stale after bounded reobservation')


def issue_act(pair, frames, actions, index):
    if not getattr(pair.io,'realtime_control',False):
        pair.drive_mecanum(actions,.2)
        return None
    frame=frames[next(iter(pair.bindings.pair.values()))]
    now=pair.time()
    duration=pair.issue_mecanum_bounded(actions,.25,observed_at_s=frame['observed_at_s'])
    if duration<=0:
        raise RuntimeError('ACT accepted RGB expired before command commit')
    pair.calls.append({'kind':'act_issue','index':index,'issued_at_s':now,
        'observed_at_s':frame['observed_at_s'],'valid_until_s':now+duration,
        'source_frame_ids':{s:frames[r]['frame_id'] for s,r in pair.bindings.pair.items()},
        'plan_hash':pair.bindings.committed['plan_hash'],'duration_s':duration,
        'actions':copy.deepcopy(actions)})
    moving=any(abs(v)>1e-9 for action in actions.values() for v in action.values())
    pair.tick(.02 if moving else min(.2,duration))
    return now+.2

def track_release_rgb(pair, frames):
    """Maintain RGB shaft identity for the shared release stage, not ACT inputs."""
    frame=frames[next(iter(pair.bindings.pair.values()))]
    feature=pair.carried_beam.observe(frame['top_bytes'])
    return {'frame_id':frame['frame_id'],'image':frame['shared_top_rgb'],
            'center':feature['center'],'purpose':'release continuity only'}

def carry(pair,python,model_dir,max_steps=900,stop_mode='rgb_guarded'):
    if stop_mode not in ('learned', 'rgb_guarded', 'rgb_refined'):
        raise ValueError('unknown ACT stop mode')
    pair.phase='TRANSIT';pair.transport_started=True
    if getattr(pair.io,'realtime_control',False):pair._hold_pair()
    # ACT still receives raw fixed cameras. The shared release stage needs a
    # continuous RGB identity; its estimate never enters the model or steering.
    anchor,_=observe_act(pair,'act-carry-anchor',lambda _frames:None)
    track_release_rgb(pair,anchor)
    own_guards={slot:OwnHoldContinuity(anchor[rid]['own_bytes']) for slot,rid in pair.bindings.pair.items()}
    goal=pair.bindings.static_map['docks'][pair.bindings.plan['dock']]['slots']['beam']['center_m']
    route=pair.bindings.tasks['beam']['route'];previous={r:[0.,0.,0.] for r in pair.bindings.pair}
    sync=PairCarrySync('act-'+pair.bindings.committed['plan_hash']);ready_count=0
    adapter=json.loads((Path(model_dir)/'adapter.json').read_text())
    temporal=adapter.get('kind')=='carry_input_ablation'
    if temporal:
        from harness.carry_input_client import InputCarryClient
        factory=lambda:InputCarryClient(python,model_dir)
    else:factory=lambda:CarryClient(python,model_dir)
    client=(pair.io.compute_visual(factory) if getattr(pair.io,'realtime_control',False) else factory())
    histories={s:deque(maxlen=client.history if temporal else 1) for s in pair.bindings.pair}
    not_before=None
    try:
        for index in range(max_steps):
            plan_hash=pair.bindings.committed['plan_hash']
            call_time=pair.time()
            def predict(frames):
                # Workers update private image/history state only. The owner
                # adopts it after freshness and plan authority are checked.
                tracker=copy.deepcopy(pair.carried_beam)
                guards=copy.deepcopy(own_guards)
                candidate_histories=copy.deepcopy(histories)
                raw=frames[next(iter(pair.bindings.pair.values()))]
                feature=tracker.observe(raw['top_bytes'])
                tracking={'frame_id':raw['frame_id'],'image':raw['shared_top_rgb'],
                          'center':feature['center'],'purpose':'release continuity only'}
                decisions={};actor_inputs={}
                for slot,rid in pair.bindings.pair.items():
                    f=frames[rid];ctx=context(goal,route,slot,previous[slot])
                    began=time.monotonic()
                    if temporal:
                        current={'own_rgb':f['own_bytes'],'top_rgb':f['top_bytes'],'context':ctx}
                        record={'images':{'own':f['own_rgb'],'top':f['shared_top_rgb']},
                                'context':ctx,'sim_time_s':f.get('observed_at_s',call_time),
                                'frame_id':f['frame_id']}
                        candidate_histories[slot].append((current,record))
                        padded=([candidate_histories[slot][0]]*(client.history-len(candidate_histories[slot]))
                                +list(candidate_histories[slot]))
                    failure=None
                    try:
                        d=(client.predict([v[0] for v in padded]) if temporal else
                           client.predict(f['own_bytes'],f['top_bytes'],ctx))
                    except Exception as error:
                        failure=type(error).__name__+': '+str(error)
                    inference_wall_s=time.monotonic()-began
                    actor_inputs[slot]={'images':{'own':f['own_rgb'],'top':f['shared_top_rgb']},
                        'context':ctx,'wire_sha256':getattr(client,'last_request_sha256',None),'physical_robot_id':rid,
                        'frame_id':f['frame_id'],'observed_at_s':f.get('observed_at_s',call_time),
                        'inference_wall_s':inference_wall_s,'response_received':failure is None}
                    if temporal:actor_inputs[slot].update(history=[v[1] for v in padded])
                    if failure:
                        return {'index':index,'inputs':actor_inputs,'decisions':decisions,'error':failure}
                    decisions[slot]=d
                    try:
                        held=guards[slot].observe(f['own_bytes'])
                    except Exception as error:
                        return {'index':index,'inputs':actor_inputs,'decisions':decisions,
                                'error':'RGB attachment validation: '+type(error).__name__+': '+str(error)}
                    decisions[slot]={**d,'ready':held['held_estimate'],'own_attachment':held,'plan_hash':plan_hash}
                return {'index':index,'decisions':decisions,'inputs':actor_inputs,'tracker':tracker,
                        'guards':guards,'histories':candidate_histories,'tracking':tracking}
            frames,prediction=observe_act(pair,'act-carry-'+str(index),predict,not_before=not_before)
            if prediction.get('error'):
                first=frames[next(iter(pair.bindings.pair.values()))]
                pair.calls.append({'kind':'act_inference_error','index':index,
                    'frame_id':first['frame_id'],'observed_at_s':first.get('observed_at_s',call_time),
                    'received_at_s':pair.time(),'sim_time_s':pair.time(),'inputs':prediction['inputs'],
                    'decisions':prediction['decisions'],'error':prediction['error']})
                raise RuntimeError('ACT inference failed: '+prediction['error'])
            if pair.bindings.committed['plan_hash']!=plan_hash:
                raise RuntimeError('ACT plan changed during inference')
            pair.carried_beam=prediction['tracker'];own_guards=prediction['guards']
            histories=prediction['histories'];release_tracking=prediction['tracking']
            decisions=prediction['decisions'];actor_inputs=prediction['inputs']
            permission=authorize_pair(sync,decisions,{s:frames[r]['frame_id'] for s,r in pair.bindings.pair.items()},index)
            ready_count=ready_count+1 if all(d['done'] for d in decisions.values()) else 0
            # Hold BOTH if either proposes arrival. Never let the partner drag it.
            pause=any(d['done'] for d in decisions.values())
            actions={s:dict.fromkeys(AXES,0.) if pause else d['action'] for s,d in decisions.items()}
            pair.calls.append({'kind':'act_carry','index':index,'sim_time_s':pair.time(),'inputs':actor_inputs,'decisions':decisions,'permission':permission,'actions':actions,'ready_count':ready_count,'release_tracking':release_tracking})
            if index%50==0:print(json.dumps({'act_carry_step':index,'sim_time_s':pair.time(),'stop_scores':{r:d['stop_score'] for r,d in decisions.items()},'actions':actions}),flush=True)
            if permission['phase']!='GO':raise RuntimeError('ACT carry RGB attachment guard stopped')
            if stop_mode=='learned' and ready_count>=3:return
            if pause and stop_mode!='learned':
                from harness.carry_arrival import arrival_evidence, PrematureCarryStop
                evidence=arrival_evidence(pair.carried_beam.previous,pair.bindings.static_map,pair.bindings.plan['dock'])
                pair.calls.append({'kind':'act_stop_admission','mode':stop_mode,
                                   'model_requested_stop':True,
                                   'requesting_slots':[s for s,d in decisions.items() if d['done']],
                                   'ready_count':ready_count,'rgb':evidence})
                if evidence['arrived']:
                    if stop_mode=='rgb_refined' or ready_count>=3:return
                    # The strict condition still requires three joint votes.
                    # A local stop outside the goal is rejected immediately.
                    not_before=issue_act(pair,frames,actions,index)
                    previous={s:[a[k] for k in AXES] for s,a in actions.items()}
                    continue
                if stop_mode=='rgb_refined':
                    # Explicit hybrid condition: ACT transit plus the existing
                    # RGB final approach, reported separately from ACT alone.
                    from harness.dispatch_skill_binding import ImageRoute
                    navigator=ImageRoute(pair.bindings,'beam')
                    navigator.observe(frames[next(iter(pair.bindings.pair.values()))]['top_bytes'])
                    # Only final alignment in the destination neighbourhood.
                    # Do not silently repair an arbitrary incorrect route.
                    if max(abs(v) for v in evidence['error_px']) > 80:
                        raise PrematureCarryStop('ACT stop too far from destination for RGB refinement')
                    navigator.index=len(navigator.points)-1
                    navigator.confirmations=0
                    pair.calls.append({'kind':'act_rgb_refinement','controller':'existing RGB final waypoint',
                                       'counts_as_pure_act':False})
                    pair.carry(navigator,max_steps=min(max_steps,160))
                    return
                raise PrematureCarryStop('ACT premature stop rejected by RGB arrival guard')
            not_before=issue_act(pair,frames,actions,index)
            previous={s:[a[k] for k in AXES] for s,a in actions.items()}
        raise RuntimeError('ACT carry decision budget exhausted')
    finally:
        if getattr(pair.io,'realtime_control',False):pair._hold_pair()
        client.close()
