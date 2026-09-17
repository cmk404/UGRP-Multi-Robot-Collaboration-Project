#!/usr/bin/env python3
"""Replay exact three-peer inputs, selected targets, all local controls and referee."""
from __future__ import annotations
import argparse
import base64
import copy
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.audit_camera_goal_transport import audit as audit_pair, require, same
from scripts.audit_camera_grasp_student import _image
from scripts.run_three_robot_mission import evaluate_solo
from harness.gemini_proxy import _to_gemini_multi_image_messages
from harness.three_robot_plan import ROBOTS, TeamAgreement
from harness.three_robot_mission import validate_mission, validate_mission_reply, build_mission_request
from harness.solo_box_transport import SoloBoxTransport


def movement_evidence(pair,solo):
    t=np.asarray([s['sim_time_s'] for s in solo])
    positions=np.asarray([s['robot_position'][:2] for s in solo])
    interpolated=np.column_stack([np.interp([s['sim_time_s'] for s in pair],t,positions[:,i]) for i in (0,1)])
    overlap=0.
    for i in range(1,len(pair)):
        dt=pair[i]['sim_time_s']-pair[i-1]['sim_time_s']
        if not 0<dt<.16:continue
        speeds=[np.linalg.norm(np.asarray(pair[i]['bases'][r][:2])-pair[i-1]['bases'][r][:2])/dt for r in ('r1','r3')]
        speeds.append(float(np.linalg.norm(interpolated[i]-interpolated[i-1])/dt))
        if min(speeds)>.015:overlap+=dt
    return {'all_three_moving_sim_s':overlap,'moving_threshold_m_per_s':.015,
            'net_displacement_m':{**{r:float(np.linalg.norm(np.asarray(pair[-1]['bases'][r][:2])-pair[0]['bases'][r][:2])) for r in ('r1','r3')},
                                  'r2':float(np.linalg.norm(positions[-1]-positions[0]))},
            'scope':'output-only sampled body motion; r2 interpolated to pair timestamps'}


def audit(root):
    root=Path(root).resolve();report=json.loads((root/'result.json').read_text())
    for path in ('harness/three_robot_mission.py','harness/three_robot_plan.py',
                 'harness/solo_box_transport.py','harness/visual_box_skill.py',
                 'harness/visual_macro_runtime.py','sim/camera_robot_port.py',
                 'scripts/three_robot_runtime.py','scripts/run_three_robot_mission.py'):
        require(subprocess.check_output(['git','show',f'{report["source_sha"]}:{path}'],cwd=ROOT)==(ROOT/path).read_bytes(),f'changed source: {path}')
    pair=audit_pair(root)
    saved=json.loads((root/'team/team.json').read_text())
    same(saved,report['mission_team'],'persisted mission ledger')
    team=TeamAgreement(saved['run_id'],plan_validator=validate_mission)
    inbox={r:[] for r in ROBOTS};used=set();wires=0
    trace=json.loads((root/'execution-trace.json').read_text())
    history={r:[] for r in ROBOTS}
    for row in trace:
        if row['stage']!='folded_setup':break
        for r,targets in row['command']['targets'].items():
            history[r].append({'stage':row['stage'],'targets':targets,'duration_s':row['command']['duration_s']})
    for row in saved['rounds']:
        context=team.context();same(context,row['agreement'],'agreement version')
        same(history,row['own_history'],'only actual own setup commands')
        replies={}
        for rid in ROBOTS:
            frame=row['images'][rid]
            request=build_mission_request(rid,request_id=f'{saved["run_id"]}-{rid}-plan-{row["turn"]}',
                own_rgb=_image(root,frame['own']),top_rgb=_image(root,frame['top']),
                agreement=context,inbox=inbox[rid],own_history=history[rid],
                preference=report['config']['solo_goal_preference'])
            calls=[c for c in saved['calls'] if c['request_id']==request['request_id']]
            require(len(calls)==1,'exactly one recorded attempt per request')
            c=calls[0];used.add(c['request_id'])
            same(json.loads((root/'team'/c['request']).read_text()),request,'exact RGB/task request')
            if saved['mode']=='llm':
                wire=root/'team'/rid/f'wire-{c["wire_index"]:03d}.json'
                same(json.loads(wire.read_text())['messages'],_to_gemini_multi_image_messages(request['messages'],request['images']),'actual wire input boundary')
                wires+=1
                if 'reply' in c:
                    response=json.loads(wire.with_name(wire.stem+'-response.json').read_text())
                    same(response['choices'][0]['message']['content'].strip(),c['raw_response'],'actual model reply')
                    same(response.get('usage'),c['usage'],'actual provider usage')
            else:require(c['model']=='scripted-fixture-not-llm','fixture must be labeled')
            reply=validate_mission_reply(c['raw_response'],request['request_id'],context) if 'reply' in c else None
            same(reply,c.get('reply'),'parsed reply');replies[rid]=reply
        same(replies,row['replies'],'delivered peer replies')
        same(team.receive(replies,row['turn']),row['committed'],'unanimous commit')
        for sender,reply in replies.items():
            if reply and reply['message']:
                for receiver in ROBOTS:
                    if receiver!=sender:inbox[receiver].append({'from_robot':sender,'turn':row['turn'],'message':reply['message']})
    require(team.committed is not None,'missing committed mission')
    same(team.committed,saved['committed'],'frozen plan');same(team.events,saved['agreement_events'],'transition log')
    require(len(used)==len(saved['calls']),'unaccounted team calls')
    committed_at=saved['rounds'][-1]['sim_time_s']
    require(all(r.get('start_sim_time_s',committed_at)>=committed_at for r in trace if r['stage']!='folded_setup'),'pair moved before commit')
    goal=team.committed['plan']['solo']['goal']
    same(goal,report['solo']['goal'],'executed selected goal')
    preference=report['config']['solo_goal_preference']
    require(preference=='auto' or preference==goal,'operator destination violated')
    actor=SoloBoxTransport(goal)
    rows=[json.loads(line) for line in (root/'solo-decisions.jsonl').read_text().splitlines()]
    events=json.loads((root/'solo-raw-actions.json').read_text())
    require(rows and rows[0]['sim_time_s']>=committed_at,'solo moved before commit')
    state=copy.deepcopy(rows[0]['observation']['actuator_state'])
    raw=[e for e in events if e['event']=='raw_action'];cursor=0;expires=None
    for row in rows:
        now=row['sim_time_s']
        while cursor<len(raw) and raw[cursor]['time']<now-1e-9:
            e=raw[cursor];a=e['raw_action'];cursor+=1
            if a['kind']=='drive':
                f,t=a['forward'],a['turn'];state['motor_commands']=[f-t,f+t,f-t,f+t]
                expires=e['time']+a['duration_s']
            elif a['kind']=='arm':state['servo_pulses'][str(a['servo_id'])]=a['pulse']
            elif a['kind']=='look':state['servo_pulses']['6']=a['pan_pulse']
            else:state['motor_commands']=[0.,0.,0.,0.];expires=None
        if expires is not None and now>=expires-1e-9:state['motor_commands']=[0.,0.,0.,0.]
        # A completed drive macro explicitly stops; it never supplies measured joints.
        same(state,row['observation']['actuator_state'],'solo own issued command history')
        obs=copy.deepcopy(row['observation']);own=_image(root,row['images']['own'])
        obs['image']=base64.b64encode(own).decode()
        require(obs['sha256']==row['images']['own']['sha256'],'own frame hash')
        same(actor.phase,row['phase_before'],'solo phase before')
        action,evidence=actor.decide(obs,_image(root,row['images']['top']))
        same(action,row['action'],'RGB solo action replay');same(evidence,row['top_evidence'],'TOP features replay')
        same(actor.phase,row['phase_after'],'solo phase after')
        kind='explicit_stop' if action['kind']=='finish' else 'macro_submitted'
        matches=[e for e in events if e['event']==kind and abs(e['time']-now)<1e-8 and e['source_frame_sha256']==obs['sha256']]
        require(len(matches)==1,'missing or duplicate dispatched solo macro')
        same(matches[0]['macro'],action,'dispatched macro equals visual decision')
    require(len([e for e in events if e['event']=='macro_submitted'])==sum(r['action']['kind']!='finish' for r in rows),'unrecorded solo macros')
    samples=json.loads((root/'solo-evaluation-only.json').read_text())
    same(evaluate_solo(samples,goal),report['solo']['evaluation'],'output-only selected-goal referee')
    pair_samples=[json.loads(s) for s in (root/'evaluation-only.jsonl').read_text().splitlines()]
    motion=movement_evidence(pair_samples,samples)
    require(min(motion['net_displacement_m'].values())>.25,'one robot did not move')
    require(motion['all_three_moving_sim_s']>.2,'three physical motions did not overlap')
    require(actor.done and actor.reason=='VISUAL_RELEASE_CONFIRMED','solo not visually finished')
    require(report['all_weld_active_ticks']==0,'weld active')
    require(report['success'] and report['evaluation']['success'] and report['solo']['evaluation']['success'],'both cargo deliveries required')
    return {'ok':True,'source_sha':report['source_sha'],'pair':pair,'mission_wire_requests':wires,
            'selected_solo_goal':goal,'solo_decisions_replayed':len(rows),'solo_raw_actions':len(raw),
            'movement':motion,'scope':'RGB/own-command/ACK replay and separate physical outcomes; not OS isolation or general task planning'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);args=p.parse_args()
    result=audit(args.run_dir);(args.run_dir/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='pair'},indent=2))
