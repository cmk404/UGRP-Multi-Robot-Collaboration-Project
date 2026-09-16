#!/usr/bin/env python3
"""Read-only verification of final recovery comparison and dataset provenance."""
import argparse,json,hashlib,subprocess,sys,statistics,base64
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.recovery_teacher import AXES,command,errors,heldout_region,score_alignment
from scripts.approach_speed_teacher import teacher_command
from scripts.run_camera_pair_transport import evaluate_grasp_samples

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def audit(a):
 import torch
 from harness.recovery_act import RecoveryAct
 torch.set_num_threads(2)
 if a.out.exists():raise FileExistsError(a.out)
 spec=read(a.spec);cases=read(Path(spec['test_cases']));goals=read(a.grasp/'evaluation-fixture.json')['base_poses']
 evidence={'audit_source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'scope':'All file/image hashes and actions, paired initial physics/RGB, recomputed common alignment and grasp; sampled first/middle/last and ready-transition native ACT decisions. Teacher/data information never used to correct live students.','raw_files_verified':0,'image_references_verified':0,'actions_verified':0,'model_decisions_replayed':0,'teacher_decisions_replayed':0,'runs':[],'pairs':[],'groups':{},'sources':{},'training':{},'external_model_calls':0}
 records={};actors={}
 for condition,entry in spec['conditions'].items():
  folder=Path(entry['cohort']);summary=read(folder/'summary.json')
  assert summary['complete'] and summary['cases_sha256']==sha(Path(spec['test_cases']))
  evidence['sources'][condition]={'source_sha':summary['source_sha'],'summary':str(folder/'summary.json'),'summary_sha256':sha(folder/'summary.json'),'manifest_sha256':sha(folder/'raw-manifest.json')}
  for rel,h in read(folder/'raw-manifest.json').items():
   p=(folder/rel).resolve();assert p.is_relative_to(folder.resolve()) and sha(p)==h;evidence['raw_files_verified']+=1
  assert {r['id'] for r in summary['cases']}=={c['id'] for c in cases}
  if entry.get('model'):
   root=Path(entry['model']);training=read(root/'report.json');assert training['complete']
   for rel,h in training['artifacts'].items():assert sha(root/rel)==h
   for source in training['sources']+training['development_sources']:
    p=Path(source['path'])
    for name,key in [('result.json','result_sha256'),('actor_samples.json','actor_sha256'),('teacher_labels.json','label_sha256')]:assert sha(p/name)==source[key]
    assert read(p/'result.json')['training_eligible']
    labels=read(p/'teacher_labels.json');assert not any(l['heldout_region'] for l in labels)
    raw=read(p/'result.json');observations={x['index']:x for x in raw['teacher_observations']}
    for label in labels:
     index=int(label['id'].split(':')[-2]);rid=label['robot_id'];e=errors(observations[index]['state'],goals,rid)
     assert not heldout_region(e)
     teacher=command(e);target={k:0. if teacher['ready'] else teacher[k] for k in AXES};target['stop']=teacher['ready']
     assert label['target']==target
    assert not ({l['case_id'] for l in labels}&{c['id'] for c in cases})
   actors[condition]={rid:RecoveryAct.load(root/rid/'act') for rid in ('r1','r3')}
   evidence['training'][condition]={'source_sha':training['source_sha'],'report_sha256':sha(root/'report.json'),'seed':training['seed'],'steps':training['steps'],'train_trajectories':len(training['sources']),'robots':training['robots'],'wall_s':training['wall_s']}
  for row in summary['cases']:
   assert row['returncode']==0 and not row['timeout'] and not row['error'];records[(condition,row['id'])]=row
 for case in cases:
  reference=None
  for condition,entry in spec['conditions'].items():
   row=records[(condition,case['id'])];folder=Path(row['raw_dir']);r=read(folder/'result.json')
   assert r['case']==case and r['takeover_step'] is None and not r['config']['weld']
   assert r['source_sha']==evidence['sources'][condition]['source_sha']
   for state in ('initial_state','approach_end_state','final_state'):assert not any(r[state]['constraints_active'].values())
   pair=(r['initial_state'],[{k:c['images'][k]['sha256'] for k in ('own','top')} for c in r['calls'][:2]])
   if reference is None:reference=pair
   else:assert pair==reference
   if r['policy']=='act':
    assert r['teacher_observations'] is None and not any(c['teaching'] for c in r['calls'])
    for c in r['calls']:
     payload=json.dumps({k+'_rgb':base64.b64encode((folder/c['images'][k]['path']).read_bytes()).decode('ascii') for k in ('own','top')})+'\n'
     assert hashlib.sha256(payload.encode()).hexdigest()==c['wire_request_sha256']
    for rid,hashes in r['model_hashes'].items():
     for name,h in hashes.items():assert sha(Path(entry['model'])/rid/'act'/name)==h
   def images(x):
    if isinstance(x,dict):
     if isinstance(x.get('path'),str) and x['path'].startswith('rgb/') and x.get('sha256'):
      p=(folder/x['path']).resolve();assert p.is_relative_to(folder.resolve()) and sha(p)==x['sha256'];evidence['image_references_verified']+=1
     for v in x.values():images(v)
    elif isinstance(x,list):
     for v in x:images(v)
   images(r);hist={rid:[] for rid in ('r1','r3')}
   consecutive=0
   for i in range(0,len(r['calls']),2):
    calls=r['calls'][i:i+2];assert [c['robot_id'] for c in calls]==['r1','r3']
    both=all(c['decision']['ok'] and c['decision']['ready'] for c in calls);consecutive=consecutive+1 if both else 0
    for c in calls:
     rid=c['robot_id'];d=c['decision'];assert c['history']==hist[rid] and set(c['images'])=={'own','top'}
     expected=dict.fromkeys(AXES,0.) if d['ready'] else {k:d[k] for k in AXES};assert c['action']==expected
     hist[rid].append(expected);evidence['actions_verified']+=1
     if r['policy']!='act':
      obs=r['teacher_observations'][i//2];e=errors(obs['state'],goals,rid);assert e==obs['errors'][rid]
      expected=command(e) if r['policy']=='recovery_teacher' else {**teacher_command(e['x'],2.),'left':0.,'turn':0.}
      assert d==expected;evidence['teacher_decisions_replayed']+=1
   assert r['approach_ok']==(consecutive>=3)
   if r['policy']=='act':
    for rid in ('r1','r3'):
     calls=[c for c in r['calls'] if c['robot_id']==rid];indices={0,len(calls)//2,len(calls)-1}
     indices.update(i for i in range(1,len(calls)) if calls[i]['decision']['ready']!=calls[i-1]['decision']['ready'])
     for i in sorted(indices):
      c=calls[i];actual=actors[condition][rid].predict(*[(folder/c['images'][k]['path']).read_bytes() for k in ('own','top')]);assert actual==c['decision'];evidence['model_decisions_replayed']+=1
   samples=[json.loads(x) for x in (folder/'evaluation-only.jsonl').read_text().splitlines()]
   alignment=score_alignment(samples,goals,r['approach_end_state']['sim_time_s']);grasp=evaluate_grasp_samples(samples)
   assert alignment==r['alignment'] and grasp==r['evaluation']
   assert all(not any(sample['constraints_active'].values()) for sample in samples)
   success=bool(r['approach_ok'] and alignment['success'] and grasp['grasp_success'] and not r['contact_steps'])
   assert success==r['success']==row['success']
   evidence['runs'].append({**row,'condition':condition,'physical_grasp_success':grasp['grasp_success'],'total_sim_s':r['final_state']['sim_time_s']-r['initial_state']['sim_time_s'],'action_count':len(r['calls']),'alignment':alignment})
  evidence['pairs'].append({'case_id':case['id'],'all_initial_physics_and_rgb_match':True})
 for g in sorted({c['group'] for c in cases}):
  evidence['groups'][g]={}
  for condition in spec['conditions']:
   rows=[r for r in evidence['runs'] if r['group']==g and r['condition']==condition];good=[r for r in rows if r['success']]
   evidence['groups'][g][condition]={'trials':len(rows),'successes':len(good),'physical_grasp_successes':sum(r['physical_grasp_success'] for r in rows),'mean_success_approach_sim_s':statistics.mean(r['approach_sim_s'] for r in good) if good else None,'mean_success_total_sim_s':statistics.mean(r['total_sim_s'] for r in good) if good else None,'mean_actions_all':statistics.mean(r['action_count'] for r in rows),'mean_wall_s_all':statistics.mean(r['wall_s'] for r in rows)}
 a.out.write_text(json.dumps(evidence,indent=2)+'\n');print(json.dumps(evidence['groups'],indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ('spec','grasp','out'):p.add_argument('--'+k,type=Path,required=True)
 audit(p.parse_args())
