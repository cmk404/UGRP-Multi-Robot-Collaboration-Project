"""ACT replaces only loaded motion. Existing task grants and RGB guards remain."""
import json
from harness.pair_carry_act_contract import context,AXES
from harness.pair_carry_act_client import CarryClient
from harness.dispatch_own_hold import OwnHoldContinuity
from harness.pair_carry_sync import PairCarrySync
from harness.dispatch_pair_navigation import authorize_pair

def carry(pair,python,model_dir,max_steps=900):
    pair.phase='TRANSIT';pair.transport_started=True
    # Raw fixed cameras; do not run teacher localization/canonicalization here.
    anchor=pair.io.capture('act-carry-anchor')
    own_guards={slot:OwnHoldContinuity(anchor[rid]['own_bytes']) for slot,rid in pair.bindings.pair.items()}
    goal=pair.bindings.static_map['docks'][pair.bindings.plan['dock']]['slots']['beam']['center_m']
    route=pair.bindings.tasks['beam']['route'];previous={r:[0.,0.,0.] for r in pair.bindings.pair}
    sync=PairCarrySync('act-'+pair.bindings.committed['plan_hash']);ready_count=0
    client=CarryClient(python,model_dir)
    try:
        for index in range(max_steps):
            frames=pair.io.capture('act-carry-'+str(index));decisions={};actor_inputs={}
            for slot,rid in pair.bindings.pair.items():
                f=frames[rid];ctx=context(goal,route,slot,previous[slot])
                d=client.predict(f['own_bytes'],f['top_bytes'],ctx)
                held=own_guards[slot].observe(f['own_bytes'])
                decisions[slot]={**d,'ready':held['held_estimate'],'own_attachment':held,'plan_hash':pair.bindings.committed['plan_hash']}
                actor_inputs[slot]={'images':{'own':f['own_rgb'],'top':f['shared_top_rgb']},'context':ctx,'wire_sha256':client.last_request_sha256,'physical_robot_id':rid}
            permission=authorize_pair(sync,decisions,{s:frames[r]['frame_id'] for s,r in pair.bindings.pair.items()},index)
            ready_count=ready_count+1 if all(d['done'] for d in decisions.values()) else 0
            # Hold BOTH if either proposes arrival. Never let the partner drag it.
            pause=any(d['done'] for d in decisions.values())
            actions={s:dict.fromkeys(AXES,0.) if pause else d['action'] for s,d in decisions.items()}
            pair.calls.append({'kind':'act_carry','index':index,'sim_time_s':pair.time(),'inputs':actor_inputs,'decisions':decisions,'permission':permission,'actions':actions,'ready_count':ready_count})
            if index%50==0:print(json.dumps({'act_carry_step':index,'sim_time_s':pair.time(),'stop_scores':{r:d['stop_score'] for r,d in decisions.items()},'actions':actions}),flush=True)
            if permission['phase']!='GO':raise RuntimeError('ACT carry RGB attachment guard stopped')
            if ready_count>=3:return
            pair.drive_mecanum(actions,.2)
            previous={s:[a[k] for k in AXES] for s,a in actions.items()}
        raise RuntimeError('ACT carry decision budget exhausted')
    finally:client.close()
