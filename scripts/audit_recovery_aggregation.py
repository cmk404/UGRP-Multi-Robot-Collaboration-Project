"""Read-only audit of all intervention attempts, including excluded failures."""
import argparse,base64,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.recovery_teacher import AXES,command,errors,heldout_region,score_alignment
from scripts.run_camera_pair_transport import evaluate_grasp_samples

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def audit(a):
 if a.out.exists():raise FileExistsError(a.out)
 goals=read(a.grasp/'evaluation-fixture.json')['base_poses'];summary=read(a.root/'summary.json');assert summary['complete']
 result={'summary_sha256':sha(a.root/'summary.json'),'manifest_sha256':sha(a.root/'raw-manifest.json'),'attempts':[],'raw_files_verified':0,'actual_rgb_requests_verified':0}
 for rel,h in read(a.root/'raw-manifest.json').items():
  p=(a.root/rel).resolve();assert p.is_relative_to(a.root.resolve()) and sha(p)==h;result['raw_files_verified']+=1
 for row in summary['cases']:
  folder=Path(row['raw_dir']);r=read(folder/'result.json');assert not r['error'] and row['returncode']==0 and not row['timeout']
  prefix=[c for c in r['calls'] if not c['teaching']];tail=[c for c in r['calls'] if c['teaching']]
  t=r['takeover'];assert len(prefix)==2*t['index']==2*r['takeover_step']
  assert abs(t['state']['sim_time_s']-r['initial_state']['sim_time_s']-.2*t['index'])<.0001
  for c in prefix:
   payload=json.dumps({k+'_rgb':base64.b64encode((folder/c['images'][k]['path']).read_bytes()).decode('ascii') for k in ('own','top')})+'\n'
   assert hashlib.sha256(payload.encode()).hexdigest()==c['wire_request_sha256'];result['actual_rgb_requests_verified']+=1
  actors=read(folder/'actor_samples.json');labels=read(folder/'teacher_labels.json');assert len(actors)==len(labels)==len(tail)
  obs={x['index']:x for x in r['teacher_observations']};held=0
  for actor,label,c in zip(actors,labels,tail):
   rid=c['robot_id'];index=c['index'];assert index>=t['index'];assert actor['id']==label['id']==f'{r["case"]["id"]}:{index:03d}:{rid}'
   assert actor['images']==c['images'] and set(actor['images'])=={'own','top'}
   for im in actor['images'].values():assert sha(folder/im['path'])==im['sha256']
   e=errors(obs[index]['state'],goals,rid);d=command(e);assert d==c['decision'];assert heldout_region(e)==label['heldout_region'];held+=int(label['heldout_region'])
   assert label['target']=={**{k:0. if d['ready'] else d[k] for k in AXES},'stop':d['ready']}
  samples=[json.loads(l) for l in (folder/'evaluation-only.jsonl').read_text().splitlines()]
  assert all(not any(s['constraints_active'].values()) for s in samples)
  alignment=score_alignment(samples,goals,r['approach_end_state']['sim_time_s']);grasp=evaluate_grasp_samples(samples);assert alignment==r['alignment'] and grasp==r['evaluation']
  eligible=bool(r['approach_ok'] and alignment['success'] and grasp['grasp_success'] and r['teacher_tail_contact_steps']==0 and held==0);assert eligible==r['training_eligible']
  reasons=[]
  if not r['approach_ok']:reasons.append('teacher did not declare aligned')
  if not alignment['success']:reasons.append('common alignment/stability failed')
  if not grasp['grasp_success']:reasons.append('physical grasp failed')
  if r['teacher_tail_contact_steps']:reasons.append('teacher tail contact')
  if held:reasons.append('held-out combination region in teacher labels')
  result['attempts'].append({'id':row['id'],'group':row['group'],'eligible':eligible,'full_run_strict_success':r['success'],'physical_grasp_success':grasp['grasp_success'],'prefix_steps':t['index'],'prefix_declared_ready':t['prefix_declared_ready'],'prefix_contact_steps':t['prefix_contact_steps'],'takeover_errors':t['errors'],'teacher_tail_contact_steps':r['teacher_tail_contact_steps'],'teacher_label_rows':len(labels),'heldout_label_rows':held,'exclusions':reasons,'raw_dir':str(folder),'result_sha256':sha(folder/'result.json')})
 a.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser()
 for k in ('root','grasp','out'):p.add_argument('--'+k,type=Path,required=True)
 audit(p.parse_args())
