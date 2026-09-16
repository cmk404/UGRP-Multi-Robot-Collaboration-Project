#!/usr/bin/env python3
"""Replay actual RGB decisions, model requests, and execution from saved bytes."""
from __future__ import annotations
import argparse
import inspect
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from scripts.audit_camera_grasp_student import _image, _sha, _canonical, audit as audit_grasp
from scripts.run_camera_varied_start_student import load_stage_models, run_approach
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_approach_student import models
from harness.pair_carry_policy import PairCarryPolicy, payload_skew
from harness.gemini_proxy import _to_gemini_multi_image_messages
from harness.camera_skill_actor import build_skill_request, validate_skill_reply, pair_skill_ready
from harness.camera_varied_start_student import predict_stage
from harness.grasp_student_inference import predict_student

ROBOTS=('r1','r3')


def require(value,reason):
    if not value: raise ValueError(reason)


def same(a,b,reason):
    require(_canonical(a)==_canonical(b),reason)


def source_module(sha,path):
    # Execute only the repository's own locally committed experiment source.
    raw=subprocess.check_output(['git','show',f'{sha}:{path}'],cwd=ROOT,text=True)
    module=types.ModuleType('experiment_replay')
    module.__file__ = str(ROOT/path)
    exec(compile(raw,path,'exec'),module.__dict__)
    return module


def audit(root):
    root=Path(root).resolve()
    report=json.loads((root/'result.json').read_text())
    source=report['source_sha']
    dependencies=['harness/pair_carry_policy.py','harness/pair_carry_sync.py',
        'harness/camera_beam_features.py','harness/camera_varied_start_student.py',
        'harness/camera_varied_start_pose_student.py','harness/camera_varied_start_geometry.py',
        'harness/camera_varied_start_heading.py','harness/grasp_student_inference.py',
        'harness/camera_recovery_student.py',
        'scripts/evaluate_camera_short_transport.py']
    for path in dependencies:
        raw=subprocess.check_output(['git','show',f'{source}:{path}'],cwd=ROOT)
        require(raw==(ROOT/path).read_bytes(),f'replay dependency changed: {path}')
    vision=source_module(source,'harness/camera_goal_transport.py')
    approach=source_module(source,'scripts/run_camera_varied_start_student.py')
    assets=report['assets']
    grasp_root=Path(assets['grasp_skill']['path'])
    stage_root=Path(assets['stage_skill']['path'])
    require(_sha(grasp_root/'student-skill.json')==assets['grasp_skill']['sha256'],'grasp skill hash')
    require(_sha(stage_root/'varied-start-skill.json')==assets['stage_skill']['sha256'],'alignment skill hash')
    require(_sha(root/'reference-top.jpg')==assets['reference_top']['sha256'],'reference hash')
    skill,grasp_models=models(grasp_root,'student-skill.json')
    _,stages=load_stage_models(stage_root)
    reference=(root/'reference-top.jpg').read_bytes()
    expected=[]
    for row in report['coarse_calls']:
        decisions={r:vision.coarse_approach(_image(root,row['images'][r]['top']),reference,r) for r in ROBOTS}
        for r in ROBOTS: _image(root,row['images'][r]['own'])
        same(decisions,row['decisions'],'coarse RGB decision')
        if not all(d['ok'] for d in decisions.values()): break
        if report['config'].get('coarse_control') == 'rgb-wheel-heading-v1':
            expected.append({r:dict(kind='mecanum',forward=decisions[r]['forward'],left=0.,
                turn=decisions[r]['turn'],duration_s=.2) for r in ROBOTS})
        else:
            require('coarse_control' not in report['config'],'unknown coarse control')
            expected.append({r:dict(kind='drive',forward=decisions[r]['forward'],turn=0.,duration_s=.2) for r in ROBOTS})
        if all(d['ready'] for d in decisions.values()):
            expected.append({r:dict(kind='drive',forward=0.,turn=0.,duration_s=.25) for r in ROBOTS})
    image_map={}
    for row in report.get('approach_calls',[]):
        tag=Path(row['images']['top']['path']).stem.removesuffix('-top')
        image_map.setdefault(tag,{})[row['robot_id']]={**row['images'],'frame_id':row['frame_id']}
    for row in report.get('final_alignment_checks',[]):
        for r in ROBOTS:
            tag=Path(row['images'][r]['top']['path']).stem.removesuffix('-top')
            image_map.setdefault(tag,{})[r]={**row['images'][r],'frame_id':row['frame_ids'][r]}
    class Replay:
        elapsed=0.
        def time(self): return self.elapsed
        def capture(self,tag):
            records=image_map[tag]
            return {r:dict(own_bytes=_image(root,x['own']),top_bytes=_image(root,x['top']),
                own_rgb=x['own'],shared_top_rgb=x['top'],frame_id=x['frame_id']) for r,x in records.items()}
        def drive_mecanum(self,commands,duration_s):
            expected.append({r:dict(kind='mecanum',**commands[r],duration_s=duration_s) for r in ROBOTS})
            self.elapsed+=duration_s
        def stop_dwell(self):
            expected.append({r:dict(kind='drive',forward=0.,turn=0.,duration_s=.25) for r in ROBOTS})
            self.elapsed+=.25
        def evaluation_snapshot(self): return {}  # output-only, deliberately no truth in replay
    if image_map:
        options = ({'reacquire_on_settle': True} if report['config'].get('reacquire_on_settle') is True else {})
        replayed=approach.run_approach(Replay(),stages,**options)
        for key in ('approach_calls','stage_results','approach_ok','final_alignment_checks'):
            same(replayed.get(key),report.get(key),'fine approach replay '+key)
    for row in report.get('dock_calls',[]):
        predictions={r:predict_stage(stages[r]['forward'],_image(root,row['images'][r]['own']),
                                    _image(root,row['images'][r]['top'])) for r in ROBOTS}
        same(predictions,row['predictions'],'dock RGB prediction')
        decisions={r:vision.dock_command(p) for r,p in predictions.items()}
        same(decisions,row['decisions'],'dock command')
        if not all(d['ok'] for d in decisions.values()): break
        expected.append({r:dict(kind='drive',forward=decisions[r]['forward'],turn=0.,duration_s=.2) for r in ROBOTS})
    preclose=report.get('preclose_check')
    if preclose is not None:
        decisions={r:predict_student(grasp_models[r],_image(root,preclose['images'][r]['own']),
                                    _image(root,preclose['images'][r]['top']),max_step=25) for r in ROBOTS}
        same(decisions,preclose['decisions'],'preclose RGB support')
        same(vision.preclose_supported(decisions),preclose['ok'],'preclose readiness')
    grasp=None
    if (root/'grasp-result.json').exists(): grasp=audit_grasp(root,grasp_root,report_name='grasp-result.json')
    policy=PairCarryPolicy('visible-goal-carry')
    for row in report['carry_calls']:
        decisions={}
        for r in ROBOTS:
            args=[_image(root,row['images'][r]['own']),_image(root,row['images'][r]['top']),
                  _image(root,report['carry_anchor'][r]['own'])]
            if len(inspect.signature(vision.goal_carry).parameters)==4:
                args.append(_image(root,report['carry_anchor'][r]['top']))
            decisions[r]=vision.goal_carry(*args)
        same(decisions,row['decisions'],'carry RGB decision')
        skew=payload_skew(_image(root,row['images']['r1']['top']))
        same(skew,row['skew'],'carry skew')
        control=policy.step(decisions,skew,row['frame_ids'],row['sim_time_s'])
        same(control,row['control'],'paired carry barrier')
        expected.append({r:dict(kind='drive',forward=control['forwards'][r],turn=0.,duration_s=control['duration_s']) for r in ROBOTS})
    trace=json.loads((root/'execution-trace.json').read_text())
    same(expected,[r['actions'] for r in trace if 'actions' in r],'issued wheel commands')
    # Record all arm sequences; pregrasp recovery deltas are independently
    # replayed above. Verify playback against the immutable teacher skill.
    for phase,commands in [('folded_setup',skill['initialization_replay'][:1]),
                           ('grasp_initialization',skill['initialization_replay'][1:])]:
        actual=[r['command'] for r in trace if r['stage']==phase]
        if actual: same(actual,commands,'demonstrated '+phase)
    llm=audit_llm(root,report) if report.get('llm') is not None else None
    samples=[json.loads(line) for line in (root/'evaluation-only.jsonl').read_text().splitlines()]
    evaluation=evaluate_transport_samples(samples,target_distance_m=.5)
    same(evaluation,report['evaluation'],'output-only physical evaluation')
    require(report['weld_active_ticks']==0,'active weld')
    same(report['invariants_initial'],report['invariants_final'],'camera/geometry changed')
    return dict(ok=True,source_sha=source,physical_success=report['success'],
        coarse_decisions=2*len(report['coarse_calls']),fine_decisions=len(report.get('approach_calls',[])),
        dock_decisions=2*len(report.get('dock_calls',[])),
        grasp=grasp,carry_decisions=2*len(report['carry_calls']),llm=llm,
        replayed_wheel_batches=len(expected),scope='saved input/command replay, not OS isolation')


def audit_llm(root,report):
    for path in ('harness/camera_skill_actor.py','scripts/camera_skill_gate.py',
                 'harness/research_execution_recovery.py','harness/gemini_proxy.py'):
        raw=subprocess.check_output(['git','show',f'{report["source_sha"]}:{path}'],cwd=ROOT)
        require(raw==(ROOT/path).read_bytes(),f'LLM replay dependency changed: {path}')
    records=json.loads((root/'llm/gate.json').read_text())
    trace=json.loads((root/'execution-trace.json').read_text())
    previous={r:None for r in ROBOTS};inbox={r:[] for r in ROBOTS};calls=records['calls']
    wire_count=0
    for event in records['events']:
        before={'APPROACH':'approach','PREPARE_GRASP':'grasp_initialization',
                'CLOSE':'grasp_close','LIFT':'grasp_lift','CARRY':'carry',
                'LOWER':'place_lower','RELEASE':'place_open','RETRACT':'place_retract',
                'FINISH':None}[event['skill']]
        boundary=next((i for i,row in enumerate(trace) if row['stage']==before),len(trace))
        histories={r:[] for r in ROBOTS}
        for row in trace[:boundary]:
            for r in ROBOTS:
                if 'actions' in row:
                    histories[r].append(dict(stage=row['stage'],action=row['actions'][r]))
                elif r in row['command']['targets']:
                    histories[r].append(dict(stage=row['stage'],targets=row['command']['targets'][r],
                        duration_s=row['command']['duration_s']))
        same(event['own_commands'],{r:h[-16:] for r,h in histories.items()},'own commands from actual trace')
        replies={}
        for rid in ROBOTS:
            own=_image(root,event['images'][rid]['own']);top=_image(root,event['images'][rid]['top'])
            rows=[c for c in calls if c['event']==event['index'] and c['robot_id']==rid]
            retry=None
            for row in rows:
                saved=json.loads((root/'llm'/row['request']).read_text())
                request=build_skill_request(rid,event['skill'],request_id=row['request_id'],own_rgb=own,top_rgb=top,
                    previous=previous[rid],own_commands=event['own_commands'][rid],
                    peer_claims=inbox[rid][-4:],retry=retry)
                same(request,saved,'LLM allowed request reconstruction')
                wire=json.loads((root/'llm'/rid/f'wire-{row["wire_index"]:03d}.json').read_text())
                same(wire['messages'],_to_gemini_multi_image_messages(request['messages'],request['images']),'exact model wire')
                wire_count+=1
                if 'reply' in row:
                    response=json.loads((root/'llm'/rid/f'wire-{row["wire_index"]:03d}-response.json').read_text())
                    same(response['choices'][0]['message']['content'].strip(),row['raw_response'],
                         'actual model response bytes')
                    same(response.get('usage'),row.get('usage'),'actual model token usage')
                    reply=validate_skill_reply(row['raw_response'],row['request_id'],event['skill'])
                    same(reply,row['reply'],'LLM schema replay');replies[rid]=reply
                elif row['error_kind']=='reply_schema':
                    retry=dict(kind='reply_schema',detail=row['error'].split(': ',1)[1],
                               previous_response=row.get('raw_response',''))
                else:
                    retry=dict(kind=row['error_kind'],detail='Previous inference attempt failed; return a fresh reply for this request.')
            previous[rid]=(own,top)
        same(pair_skill_ready(replies,event['skill']),event['ready'],'LLM paired permission')
        for rid,reply in replies.items():
            peer='r3' if rid=='r1' else 'r1'
            inbox[peer].append(dict(from_robot=rid,skill=event['skill'],message=reply['message']))
    return dict(wire_requests=wire_count,events=len(records['events']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path)
    a=p.parse_args();result=audit(a.run_dir)
    (a.run_dir/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='grasp'},indent=2))
