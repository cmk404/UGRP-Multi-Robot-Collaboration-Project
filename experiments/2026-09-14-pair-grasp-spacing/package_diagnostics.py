"""Read-only summaries/hashes for bounded grasp diagnostics."""
from pathlib import Path
import json,hashlib,gzip,cv2,sys
ROOT=Path(__file__).resolve().parents[2]
dest=Path(sys.argv[1]);dest.mkdir(parents=True,exist_ok=True)
summary=[]
for name in ['pair-spacing-diagnostic-2026-09-14','pair-spacing-diagnostic2-2026-09-14','pair-spacing-diagnostic3-2026-09-14','pair-spacing-passive1700-2026-09-14','pair-spacing-1600-short-2026-09-14','pair-spacing-1600-long-2026-09-14/narrow-door']:
 raw=ROOT/'outputs'/name;r=json.loads((raw/'result.json').read_text())
 ident=name.replace('-2026-09-14','').replace('/narrow-door','')
 out=dest/ident;out.mkdir(exist_ok=True)
 stripped={k:v for k,v in r.items() if k not in ['steps','sync_events','spacing_steps','spacing_sync_events']}
 if (raw/'input-audit.json').exists():stripped['input_audit']=json.loads((raw/'input-audit.json').read_text())
 stripped['raw_directory']=str(raw)
 (out/'result-summary.json').write_text(json.dumps(stripped,ensure_ascii=False,indent=2)+'\n')
 files=[{'path':p.relative_to(raw).as_posix(),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(raw.rglob('*')) if p.is_file()]
 h=out/'raw-hashes.jsonl.gz'
 with gzip.open(h,'wt') as f:
  for row in files:f.write(json.dumps(row)+'\n')
 manifest={'raw_root':str(raw),'raw_remote_backup':False,'files':len(files),'bytes':sum(x['bytes'] for x in files),'hash_index':'raw-hashes.jsonl.gz','hash_index_sha256':hashlib.sha256(h.read_bytes()).hexdigest()}
 (out/'raw-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 v=cv2.VideoCapture(str(raw/'motion.mp4'));v.set(cv2.CAP_PROP_POS_FRAMES,int(v.get(cv2.CAP_PROP_FRAME_COUNT))-1);ok,im=v.read();v.release()
 if ok:cv2.imwrite(str(out/'last-observer.jpg'),im)
 summary.append({'id':ident,**{k:r.get(k) for k in ['source_sha','grasp_spacing','close_command_override','grasp_only','success','error','wall_seconds','grasp_stability']},'manifest':manifest,'input_audit_passed':stripped.get('input_audit',{}).get('passed')})
(dest/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print('diagnostics packaged',len(summary))
