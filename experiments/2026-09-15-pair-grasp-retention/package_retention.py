"""Archive all diagnostics, pilots and final trials; raw media stays local."""
from pathlib import Path
import argparse,gzip,hashlib,json,math,shutil
ROOT=Path.cwd()
def read(p):return json.loads(p.read_text())
def write(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def main():
 p=argparse.ArgumentParser();p.add_argument('--destination',type=Path,required=True);a=p.parse_args();out=a.destination.resolve();out.mkdir(parents=True,exist_ok=True)
 shutil.copytree(ROOT/'outputs/retention-records-staging',out,dirs_exist_ok=True)
 summary={'scope':'Fixed-start simulation, zero external model calls; all failures retained; raw media local only','groups':[]};hashes=[]
 for label,base in [('pilot',ROOT/'outputs/retention-recovery-pilot-20260915'),('final',ROOT/'outputs/retention-final-20260915')]:
  group={'label':label,'raw_directory':str(base),'trials':[]}
  for result in sorted(base.glob('*/result.json')):
   raw=result.parent;r=read(result);dest=out/label/raw.name;dest.mkdir(parents=True,exist_ok=True)
   rows=[json.loads(x) for x in (raw/'evaluation-only.jsonl').read_text().splitlines()];carry=[x for x in rows if x['phase'] in ('carry','carry_stop')]
   entry={k:r.get(k) for k in ('source_sha','environment','success','error','cleanup_error','evaluation','grasp_stability','recovery_evaluation','sim_seconds','wall_seconds','contact_profile','weld_active_ticks','wall_contact_ticks','unexpected_contact_ticks','refused_no_route')}
   entry.update(case=raw.name,raw_directory=str(raw),decision_rounds=len(r['steps']),recovery_count=len(r.get('recoveries',[])),input_audit=read(raw/'input-audit.json'))
   if carry:
    anchor=math.dist(carry[0]['bases']['r1'][:2],carry[0]['bases']['r3'][:2]);entry.update(carry_elapsed_s=carry[-1]['sim_time_s']-carry[0]['sim_time_s'],max_beam_drop_m=carry[0]['position_m'][2]-min(x['position_m'][2] for x in carry),max_carry_spacing_change_m=max(abs(math.dist(x['bases']['r1'][:2],x['bases']['r3'][:2])-anchor) for x in carry),max_tilt_deg=max(x['tilt_deg'] for x in carry))
   group['trials'].append(entry)
   for f in raw.glob('*'):
    if f.is_file() and f.suffix in ('.json','.jsonl'):
     with f.open('rb') as src,gzip.open(dest/(f.name+'.gz'),'wb') as dst:shutil.copyfileobj(src,dst)
   shutil.copy2(raw/'scene.xml',dest/'scene.xml')
  for f in sorted(base.glob('*')):
   if f.is_file():
    (out/label).mkdir(exist_ok=True);shutil.copy2(f,out/label/(f.name+'.txt' if f.suffix=='.log' else f.name))
  for f in sorted(base.rglob('*')):
   if f.is_file():
    with f.open('rb') as src:digest=hashlib.file_digest(src,'sha256').hexdigest()
    hashes.append({'path':str(f),'size_bytes':f.stat().st_size,'sha256':digest})
  summary['groups'].append(group)
 write(out/'summary.json',summary)
 with gzip.open(out/'pilot-final-raw-hashes.jsonl.gz','wt') as f:
  for h in hashes:f.write(json.dumps(h)+'\n')
 write(out/'raw-manifest.json',{'pilot_final_file_count':len(hashes),'pilot_final_bytes':sum(x['size_bytes'] for x in hashes),'remote_backup':False,'diagnostic_hashes':'diagnostic-raw-hashes.jsonl.gz','locations':[g['raw_directory'] for g in summary['groups']]})
 print(json.dumps({'groups':len(summary['groups']),'trials':sum(len(g['trials']) for g in summary['groups']),'raw_files':len(hashes)}))
if __name__=='__main__':main()
