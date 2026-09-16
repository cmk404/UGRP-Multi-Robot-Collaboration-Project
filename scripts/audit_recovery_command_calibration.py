"""Audit a paired development-only actuator diagnostic; never choose on final tests."""
import argparse,base64,hashlib,json,sys,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from harness.recovery_commands import issued_command
from scripts.recovery_teacher import score_alignment
from scripts.run_camera_pair_transport import evaluate_grasp_samples

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 import torch
 from harness.recovery_act import RecoveryAct
 torch.set_num_threads(2)
 p=argparse.ArgumentParser()
 for k in ('spec','grasp','out'):p.add_argument('--'+k,type=Path,required=True)
 a=p.parse_args();assert not a.out.exists();spec=read(a.spec);goals=read(a.grasp/'evaluation-fixture.json')['base_poses']
 result={'scope':'Development diagnostic only; weights/ready rule fixed; paired physics/RGB and all issued commands and actual requests verified, sampled native predictions replayed.','audit_source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'runs':[],'requests_verified':0,'predictions_replayed':0,'raw_files_verified':0};paired={};models={}
 for name,e in spec.items():
  root=Path(e['cohort']);summary=read(root/'summary.json');assert summary['complete']
  m=Path(e['model']);training=read(m/'report.json');assert training['complete']
  for rel,h in training['artifacts'].items():assert sha(m/rel)==h
  models.setdefault(str(m),{r:RecoveryAct.load(m/r/'act') for r in ('r1','r3')})
  for rel,h in read(root/'raw-manifest.json').items():assert sha(root/rel)==h;result['raw_files_verified']+=1
  for row in summary['cases']:
   assert row['returncode']==0 and not row['timeout'];folder=Path(row['raw_dir']);r=read(folder/'result.json');assert not r['error'] and r['teacher_observations'] is None and r['takeover_step'] is None
   pair=[r['initial_state'],[c['images'] for c in r['calls'][:2]]]
   assert paired.setdefault(row['id'],pair)==pair
   for c in r['calls']:
    assert not c['teaching'];assert c['action']==issued_command(c['decision'],e['decoder'])
    payload=json.dumps({k+'_rgb':base64.b64encode((folder/c['images'][k]['path']).read_bytes()).decode('ascii') for k in ('own','top')})+'\n'
    assert hashlib.sha256(payload.encode()).hexdigest()==c['wire_request_sha256'];result['requests_verified']+=1
   for rid in ('r1','r3'):
    calls=[c for c in r['calls'] if c['robot_id']==rid]
    for i in {0,len(calls)//2,len(calls)-1}:
     c=calls[i];d=models[str(m)][rid].predict(*[(folder/c['images'][k]['path']).read_bytes() for k in ('own','top')]);assert d==c['decision'];result['predictions_replayed']+=1
   samples=[json.loads(l) for l in (folder/'evaluation-only.jsonl').read_text().splitlines()];al=score_alignment(samples,goals,r['approach_end_state']['sim_time_s']);gr=evaluate_grasp_samples(samples)
   assert al==r['alignment'] and gr==r['evaluation'];assert bool(r['approach_ok'] and al['success'] and gr['grasp_success'] and not r['contact_steps'])==r['success']
   result['runs'].append({'condition':name,'case':row['id'],'success':r['success'],'contact_steps':r['contact_steps'],'physical_grasp':gr['grasp_success'],'approach_sim_s':r['approach_sim_s'],'raw_dir':str(folder),'result_sha256':sha(folder/'result.json'),'source_sha':r['source_sha'],'model_report_sha256':sha(m/'report.json')})
 raw=sum(x['success'] for x in result['runs'] if x['condition'].endswith('_raw'));cal=sum(x['success'] for x in result['runs'] if x['condition'].endswith('_calibrated'))
 retained=all(next(x for x in result['runs'] if x['condition']=='recovery_calibrated' and x['case']==c)['success'] for c in ('nominal-dev-01','overshoot-dev-01'))
 result.update(raw_successes=raw,calibrated_successes=cal,previous_recovery_successes_retained=retained,selected_decoder='calibrated' if cal>raw and retained else 'raw',selection='Preregistered pooled improvement AND retain both prior recovery successes. No threshold tuning.')
 a.out.write_text(json.dumps(result,indent=2)+'\n');print(result['selected_decoder'],raw,cal,retained)
if __name__=='__main__':main()
