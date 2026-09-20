"""Package completed endurance diagnostics without dropping failures."""
from pathlib import Path
import argparse,copy,gzip,hashlib,json,math,shutil
ROOT=Path.cwd()
def read(p):return json.loads(p.read_text())
def write(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--group',action='append',required=True,help='label=raw-directory');a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True);summary={'groups':[],'scope':'Fixed-start simulation; all trials, including failures; raw data local only'};hashes=[]
 for spec in a.group:
  label,path=spec.split('=',1);base=Path(path).resolve();co=read(base/'cohort.json');group={'label':label,'source_sha':co['source_sha'],'raw_directory':str(base),'trials':[]};write(out/label/'cohort.json',co)
  for log in base.glob('*.log'):shutil.copy2(log,out/label/log.name)
  for t in co['trials']:
   case=t.get('mode',t.get('id'));raw=base/case;r=read(raw/'result.json');dest=out/label/case;dest.mkdir(parents=True,exist_ok=True)
   rows=[json.loads(x) for x in (raw/'evaluation-only.jsonl').read_text().splitlines()];c=[x for x in rows if x['phase'] in ('carry','carry_stop')]
   entry={k:r.get(k) for k in ['success','error','cleanup_error','evaluation','grasp_stability','environment','source_sha','sim_seconds','wall_seconds','external_model_calls','cost_usd','impratio','contact_impratio','noslip_iterations','spacing_integral','weld_active_ticks','wall_contact_ticks','unexpected_contact_ticks','refused_no_route']};entry.update(case=case,raw_directory=str(raw),input_audit=read(raw/'input-audit.json'),decision_rounds=len(r['steps']),spacing_rounds=len(r.get('spacing_steps',[])))
   if c:
    anchor=math.dist(c[0]['bases']['r1'][:2],c[0]['bases']['r3'][:2]);entry.update(carry_sim_seconds=c[-1]['sim_time_s']-c[0]['sim_time_s'],max_beam_drop_m=c[0]['position_m'][2]-min(x['position_m'][2] for x in c),max_carry_spacing_change_m=max(abs(math.dist(x['bases']['r1'][:2],x['bases']['r3'][:2])-anchor) for x in c))
   group['trials'].append(entry)
   for name in ['result.json','grasp-result.json','execution-trace.json','evaluation-only.jsonl','input-audit.json','contact-events-evaluation-only.json']:
    with (raw/name).open('rb') as source,gzip.open(dest/(name+'.gz'),'wb') as target:shutil.copyfileobj(source,target)
   shutil.copy2(raw/'scene.xml',dest/'scene.xml')
  for f in sorted(base.rglob('*')):
   if f.is_file():hashes.append({'path':str(f),'size_bytes':f.stat().st_size,'sha256':hashlib.file_digest(f.open('rb'),'sha256').hexdigest()})
  summary['groups'].append(group)
 write(out/'summary.json',summary)
 with gzip.open(out/'raw-hashes.jsonl.gz','wt') as f:
  for h in hashes:f.write(json.dumps(h)+'\n')
 write(out/'raw-manifest.json',{'file_count':len(hashes),'total_bytes':sum(x['size_bytes'] for x in hashes),'remote_backup':False,'locations':[g['raw_directory'] for g in summary['groups']]})
 print(json.dumps({'groups':len(summary['groups']),'trials':sum(len(g['trials']) for g in summary['groups']),'raw_files':len(hashes)}))
if __name__=='__main__':main()
