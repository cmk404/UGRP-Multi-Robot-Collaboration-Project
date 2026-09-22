"""ACT replaces only loaded motion. Existing task grants and RGB guards remain."""
import json
import time
from pathlib import Path
from collections import deque
from harness.pair_carry_act_contract import context,AXES
from harness.pair_carry_act_client import CarryClient
from harness.dispatch_own_hold import OwnHoldContinuity
from harness.pair_carry_sync import PairCarrySync
from harness.dispatch_pair_navigation import authorize_pair

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
    # ACT still receives raw fixed cameras. The shared release stage needs a
    # continuous RGB identity; its estimate never enters the model or steering.
    anchor=pair.io.capture('act-carry-anchor',own_robots=tuple(pair.bindings.pair.values()),overview=False)
    track_release_rgb(pair,anchor)
    own_guards={slot:OwnHoldContinuity(anchor[rid]['own_bytes']) for slot,rid in pair.bindings.pair.items()}
    goal=pair.bindings.static_map['docks'][pair.bindings.plan['dock']]['slots']['beam']['center_m']
    route=pair.bindings.tasks['beam']['route'];previous={r:[0.,0.,0.] for r in pair.bindings.pair}
    sync=PairCarrySync('act-'+pair.bindings.committed['plan_hash']);ready_count=0
    adapter=json.loads((Path(model_dir)/'adapter.json').read_text())
    temporal=adapter.get('kind')=='carry_input_ablation'
    if temporal:
        from harness.carry_input_client import InputCarryClient
        client=InputCarryClient(python,model_dir)
    else:client=CarryClient(python,model_dir)
    histories={s:deque(maxlen=client.history if temporal else 1) for s in pair.bindings.pair}
    try:
        for index in range(max_steps):
            frames=pair.io.capture('act-carry-'+str(index),
                own_robots=tuple(pair.bindings.pair.values()),overview=False);decisions={};actor_inputs={}
            release_tracking=track_release_rgb(pair,frames)
            for slot,rid in pair.bindings.pair.items():
                f=frames[rid];ctx=context(goal,route,slot,previous[slot])
                began=time.monotonic()
                if temporal:
                    current={'own_rgb':f['own_bytes'],'top_rgb':f['top_bytes'],'context':ctx}
                    record={'images':{'own':f['own_rgb'],'top':f['shared_top_rgb']},'context':ctx,'sim_time_s':pair.time(),'frame_id':f['frame_id']}
                    histories[slot].append((current,record))
                    padded=[histories[slot][0]]*(client.history-len(histories[slot]))+list(histories[slot])
                    d=client.predict([v[0] for v in padded])
                else:d=client.predict(f['own_bytes'],f['top_bytes'],ctx)
                inference_wall_s=time.monotonic()-began
                held=own_guards[slot].observe(f['own_bytes'])
                decisions[slot]={**d,'ready':held['held_estimate'],'own_attachment':held,'plan_hash':pair.bindings.committed['plan_hash']}
                actor_inputs[slot]={'images':{'own':f['own_rgb'],'top':f['shared_top_rgb']},'context':ctx,'wire_sha256':client.last_request_sha256,'physical_robot_id':rid,'inference_wall_s':inference_wall_s}
                if temporal:actor_inputs[slot].update(history=[v[1] for v in padded])
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
                    pair.drive_mecanum(actions,.2)
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
            pair.drive_mecanum(actions,.2)
            previous={s:[a[k] for k in AXES] for s,a in actions.items()}
        raise RuntimeError('ACT carry decision budget exhausted')
    finally:client.close()
