from pathlib import Path
import json,hashlib,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scripts.audit_camera_grasp_student import audit
from scripts.run_camera_pair_transport import evaluate_grasp_samples
base=Path('/Users/changmin/projects/ugrp/outputs');models=base/'grasp-student-model-20260910-v1'
for cohort in ['grasp-student-cohort-20260910-v1','grasp-student-stress-20260910-v1']:
 root=base/cohort;rows=[]
 for p in sorted(root.iterdir()):
  if not p.is_dir():continue
  r=json.loads((p/'result.json').read_text());a=audit(p,models);(p/'audit.json').write_text(json.dumps(a,indent=2)+'\n');es=[json.loads(x) for x in (p/'evaluation-only.jsonl').read_text().splitlines()];assert evaluate_grasp_samples(es)==r['evaluation'];assert r['evaluation_fixture_sha256']==hashlib.sha256((models/'evaluation-fixture.json').read_bytes()).hexdigest()
  rows.append({'id':p.name,'evaluation':r['evaluation'],'source_sha':r['source_sha'],'config':r['config'],'applied_perturbation':r['applied_perturbation'],'wall_elapsed_s':r['wall_elapsed_s'],'action_count':sum(len(c['actions']) for c in r['calls']),'initial_feature_errors':{c['robot_id']:c['decision']['feature_error'] for c in r['calls'][:2]},'final_feature_errors':{rid:v['feature_error'] for rid,v in r['final_visual_errors'].items()},'audit':{'ok':a['ok'],'calls':a['calls']},'sim_time_range':[es[0]['sim_time_s'],es[-1]['sim_time_s']]})
 for x in rows:
  if x['config']['condition']=='visual':
   name=x['id'].removesuffix('-visual');a=json.loads((root/(name+'-visual')/'result.json').read_text());b=json.loads((root/(name+'-playback')/'result.json').read_text());assert all(c['images']==d['images'] for c,d in zip(a['calls'][:2],b['calls'][:2]));x['matched_initial_rgb']=True
   y=next(y for y in rows if y['id']==name+'-playback');assert x['sim_time_range']==y['sim_time_range'];assert x['applied_perturbation']==y['applied_perturbation'];assert a['initialization_commands']==b['initialization_commands'];assert a['skill_sha256']==b['skill_sha256'];assert a['source_sha']==b['source_sha'];assert all(list(v)==x['config']['perturb'] for v in x['applied_perturbation'].values())
 (root/'summary.json').write_text(json.dumps(rows,indent=2)+'\n');print(cohort,'verified',len(rows),'runs',sum(r['audit']['calls'] for r in rows),'calls',{c:sum(r['evaluation']['grasp_success'] for r in rows if r['config']['condition']==c) for c in ['visual','playback']},flush=True)
