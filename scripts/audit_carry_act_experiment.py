"""Post-run audit; never imported into the live policy or training process."""
import argparse,base64,hashlib,io,json,math,subprocess,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from harness.pair_carry_act_contract import context,AXES
from harness.dispatch_own_hold import OwnHoldContinuity
from harness.pair_carry_act_client import CarryClient
from scripts.run_dispatch_e2e import Referee
from scripts.build_carry_act_data import extract
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--protocol',type=Path,required=True)
parser.add_argument('--act-python',type=Path,required=True)
args=parser.parse_args()
OUT=args.out.resolve()
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,r):p.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
report=read(OUT/'report.json');assert report['complete']
protocol=read(args.protocol)
assert report['protocol']==protocol
freeze=read(OUT/'final-freeze.json');assert freeze['protocol_sha256']==report['protocol_sha256']
dataset=read(OUT/'dataset.json')
assert len(dataset['train'])==6 and len(dataset['development'])==2
assert not {v['root'] for v in dataset['train']}&{v['root'] for v in dataset['development']}
for split in ('train','development'):
 for episode in dataset[split]:assert extract(Path(episode['root']))==episode
models={}
for seed,h in freeze['models'].items():
 p=OUT/f'model-{seed}';r=read(p/'report.json')
 assert r['complete'] and sha(p/'act/model.safetensors')==h==r['model_sha256']
 assert r['dataset_sha256']==sha(OUT/'dataset.json')
 assert r['source_sha']==report.get('training_reuse',{}).get('source_sha',report['source_sha'])
 models[seed]=r
rows=[];checked_images=checked_wires=replayed=0;entries={};initials={}
for run in report['runs']:
 root=Path(run['output']);r=read(root/'result.json');setup=read(root/'episode-setup-only.json')
 assert r['source_sha']==(report['training_reuse']['source_sha'] if run.get('reused') else report['source_sha']);assert r['camera_geometry_unchanged']
 samples=[json.loads(line) for line in (root/'referee-only.jsonl').open()]
 assert not any(s['weld'] for s in samples)
 commands=read(root/'issued-commands.json');bind=read(root/'skill-bindings.json')['pair_model_slots']
 ref=Referee.__new__(Referee);ref.file=io.StringIO();ref.samples=samples;ref.geoms={'beam':0,'box':1};ref.scene=SimpleNamespace(config=setup,command_history=commands,weld_steps=0)
 expected=ref.finish(r['plan']);assert expected==r['evaluation'],root
 plan=read(root/'committed-plan.json')['plan'];decisions=read(root/'pair-decisions.json')
 act=[x for x in decisions if x['kind']=='act_carry']
 starts=[c['issued_at_s'] for s,rid in bind.items() for c in commands[rid] if c['stage']=='TRANSIT']
 start=act[0]['sim_time_s'] if act else min(starts) if starts else None
 entry=min(samples,key=lambda x:abs(x['sim_time_s']-start)) if start is not None else None
 summary={'condition':run['condition'],'case':run['case']['id'],'phase':run['phase'],'whole_success':bool(r['physical_success'] and r['protocol_complete'] and not r['error'] and not r['obstacle_contact_steps']),'beam_success':bool(expected['cargo']['beam']['physical_success'] and not r['obstacle_contact_steps']),'physical_only_whole_success':r['physical_success'],'protocol_complete':r['protocol_complete'],'error':r['error'],'phase_at_end':r['phase'],'obstacle_contact_steps':r['obstacle_contact_steps'],'weld_steps':0,'beam_displacement_m':expected['cargo']['beam']['displacement_m'],'carry_window':expected['cargo']['beam']['carry_clearance']['window_s'],'carry_entered':entry is not None,'prerequisite_failure':entry is None and r['phase']!='TRANSIT','carry_decisions':len(act),'raw':str(root),'source_sha':r['source_sha']}
 if summary['carry_window']:summary['carry_sim_s']=summary['carry_window'][1]-summary['carry_window'][0]
 if run['phase']=='final':
  key=run['case']['id'];initials.setdefault(key,{})[run['condition']]={'first_referee_sample':samples[0],'setup':setup,'plan':plan}
  entries.setdefault(key,{})[run['condition']]={'referee_at_entry':entry}
 if act:
  assert not any(x['kind'] in ('carry','rotating_carry','navigation_map') for x in decisions)
  assert r['carry_model_sha256']==freeze['models'][run['condition']]
  goal=setup['static_map']['docks'][plan['dock']]['slots']['beam']['center_m'];route=next(t['route'] for t in plan['tasks'] if t['object']=='beam')
  previous={s:[0.,0.,0.] for s in bind};guards={s:OwnHoldContinuity((root/'rgb'/f'act-carry-anchor-{rid}.jpg').read_bytes()) for s,rid in bind.items()};client=CarryClient(args.act_python,OUT/f"model-{run['condition']}"/'act');count=0
  try:
   for i,a in enumerate(act):
    all_done=all(d['done'] for d in a['decisions'].values());count=count+1 if all_done else 0;assert a['ready_count']==count
    pause=any(d['done'] for d in a['decisions'].values())
    for slot,record in a['inputs'].items():
     assert record['physical_robot_id']==bind[slot]
     assert record['context']==context(goal,route,slot,previous[slot])
     rgb={}
     for view,im in record['images'].items():
      p=(root/im['path']).resolve();assert p.is_relative_to(root.resolve()) and sha(p)==im['sha256'];rgb[view]=p.read_bytes();checked_images+=1
     wire=json.dumps({'own_rgb':base64.b64encode(rgb['own']).decode('ascii'),'top_rgb':base64.b64encode(rgb['top']).decode('ascii'),'context':record['context']})+'\n'
     assert hashlib.sha256(wire.encode()).hexdigest()==record['wire_sha256'];checked_wires+=1
     guard=guards[slot].observe(rgb['own']);assert guard==a['decisions'][slot]['own_attachment']
     assert a['decisions'][slot]['ready']==guard['held_estimate']
     if i in {0,len(act)-1} or i%max(1,len(act)//12)==0:
      d=client.predict(rgb['own'],rgb['top'],record['context']);assert all(d[k]==a['decisions'][slot][k] for k in ('action','done','stop_score'));replayed+=1
     expected_action=dict.fromkeys(AXES,0.) if pause else a['decisions'][slot]['action'];assert a['actions'][slot]==expected_action
     issued=[c for c in commands[bind[slot]] if c['stage']=='TRANSIT' and abs(c['issued_at_s']-a['sim_time_s'])<1e-7]
     if a['permission']['phase']=='GO' and count<3:
      assert len(issued)==1 and all(issued[0]['action'][k]==expected_action[k] for k in AXES)
     else:assert not issued
     previous[slot]=[expected_action[k] for k in AXES]
    if run['phase']=='final' and i==0:entries[run['case']['id']][run['condition']]['rgb_sha256']={s:{v:im['sha256'] for v,im in rr['images'].items()} for s,rr in a['inputs'].items()}
   summary['arrival_declared']=count>=3;summary['last_stop_scores']={s:d['stop_score'] for s,d in act[-1]['decisions'].items()}
  finally:client.close()
 else:
  summary['arrival_declared']=False
  if run['condition']!='teacher' and r['phase']=='TRANSIT':summary['runtime_before_first_decision_failure']=True
  if run['phase']=='final' and run['condition']=='teacher':
   captures={d['frame_id']:d for d in decisions if d['kind']=='image_binding'}
   first=next((d for d in decisions if d['kind'] in ('carry','rotating_carry')),None)
   if first:
    im=captures[first['frame_ids']['r1']];entries[run['case']['id']][run['condition']]['rgb_sha256']={s:{'own':im['own'][s]['sha256'],'top':im['raw_top']['sha256']} for s in bind}
 summary['file_hashes']={n:sha(root/n) for n in ('result.json','issued-commands.json','pair-decisions.json','referee-only.jsonl','episode-setup-only.json','execution.mp4')}
 rows.append(summary)
for key,conditions in initials.items():
 assert set(conditions)=={'teacher','20260918','20260919'}
 teacher=conditions['teacher']
 for other in conditions.values():assert other==teacher,(key,'unmatched initial physics/plan/setup')
for key, conditions in entries.items():
 if all(v.get('rgb_sha256') for v in conditions.values()):
  assert all(v['rgb_sha256']==conditions['teacher']['rgb_sha256'] for v in conditions.values()),(key,'different carry-entry actor images')
write(OUT/'audit.json',{'passed':True,'source_sha':report['source_sha'],'training_source_sha':report.get('training_reuse',{}).get('source_sha',report['source_sha']),'dataset_raw_provenance_verified':True,'rows':rows,'initial_pairs_verified':len(initials),'carry_entry':entries,'image_references_verified':checked_images,'wire_requests_verified':checked_wires,'native_decisions_replayed':replayed,'training':{seed:{k:m[k] for k in ('seed','steps','samples','groups','selected','train_metrics','development_metrics','wall_s','model_sha256')} for seed,m in models.items()},'scope':'All final actor wires/context/issued commands and referee outcomes audited; native ACT decisions sampled. Six successful nominal teacher episodes: limited feasibility evidence, no recovery generalization claim.'})
print(json.dumps({'passed':True,'runs':len(rows),'images':checked_images,'wires':checked_wires,'native':replayed}))
