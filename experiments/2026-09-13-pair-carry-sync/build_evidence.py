"""Package exact run records and deduplicated camera inputs after a frozen cohort."""
import argparse,gzip,hashlib,json,pathlib,shutil,zipfile

def sha(data): return hashlib.sha256(data).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--cohort',type=pathlib.Path,required=True);p.add_argument('--development',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
 cohort=json.loads((a.cohort/'cohort-report.json').read_text());assert cohort['complete'] and len(cohort['runs'])==52
 a.out.mkdir(parents=True,exist_ok=True)
 index={'schema':'ugrp.pair_carry_evidence.v1','runs':{},'archives':[]};unique={};raw=[]
 for group,root in [('comparison',a.cohort),('development',a.development)]:
  for result in sorted(root.glob('*/result.json')):
   run=result.parent;name=f'{group}/{run.name}';entry={'raw_directory':str(run.resolve()),'files':{}}
   for f in sorted(run.rglob('*')):
    if not f.is_file():continue
    rel=str(f.relative_to(run));data=f.read_bytes();digest=sha(data)
    raw.append({'run':name,'path':str(f.resolve()),'sha256':digest,'bytes':len(data)})
    if f.suffix=='.jpg' and rel.startswith('rgb/'):
     unique.setdefault(digest,f);entry['files'][rel]={'sha256':digest,'kind':'rgb'}
    elif f.suffix in ('.json','.jsonl'):
     dest=a.out/'records'/name/(rel+'.gz');dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(gzip.compress(data,mtime=0))
     entry['files'][rel]={'sha256':digest,'kind':'record','archive_path':str(dest.relative_to(a.out))}
   index['runs'][name]=entry
 parts=[];batch=[];size=0
 for digest,source in sorted(unique.items()):
  n=source.stat().st_size
  if batch and size+n>35*1024*1024:parts.append(batch);batch=[];size=0
  batch.append((digest,source));size+=n
 if batch:parts.append(batch)
 for i,part in enumerate(parts,1):
  dest=a.out/f'rgb-{i:02d}.zip'
  with zipfile.ZipFile(dest,'w',compression=zipfile.ZIP_STORED) as z:
   for digest,source in part:z.write(source,f'{digest}.jpg')
  index['archives'].append({'path':dest.name,'sha256':sha(dest.read_bytes()),'count':len(part),'bytes':dest.stat().st_size})
 (a.out/'evidence-index.json.gz').write_bytes(gzip.compress(json.dumps(index,ensure_ascii=False,sort_keys=True).encode(),mtime=0))
 (a.out/'raw-files.jsonl.gz').write_bytes(gzip.compress(('\n'.join(json.dumps(r,ensure_ascii=False) for r in raw)+'\n').encode(),mtime=0))
 shutil.copy2(a.cohort/'cohort-report.json',a.out/'cohort-report.json')
 summary={'run_count':len(index['runs']),'unique_rgb':len(unique),'rgb_references':sum(x['kind']=='rgb' for r in index['runs'].values() for x in r['files'].values()),'archive_bytes':sum(x['bytes'] for x in index['archives']),'raw_file_count':len(raw)}
 (a.out/'packaging.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
if __name__=='__main__':main()
