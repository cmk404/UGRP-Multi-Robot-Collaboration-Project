#!/usr/bin/env python3
"""Restore committed paired-carry JPEG/record evidence into a new directory."""
import gzip,hashlib,json,pathlib,sys,zipfile

def sha(data):return hashlib.sha256(data).hexdigest()
def restore(out):
 root=pathlib.Path(__file__).resolve().parent
 index=json.loads(gzip.decompress((root/'evidence-index.json.gz').read_bytes()))
 out=pathlib.Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
 images={}
 for a in index['archives']:
  data=(root/a['path']).read_bytes();assert sha(data)==a['sha256'],a['path']
  with zipfile.ZipFile(root/a['path']) as z:
   assert len(z.namelist())==a['count']
   for name in z.namelist():
    data=z.read(name);digest=sha(data);assert name==digest+'.jpg';images[digest]=data
 count=0
 for run,entry in index['runs'].items():
  for relative,meta in entry['files'].items():
   dest=(out/run/relative).resolve();assert dest.is_relative_to(out)
   data=images[meta['sha256']] if meta['kind']=='rgb' else gzip.decompress((root/meta['archive_path']).read_bytes())
   assert sha(data)==meta['sha256'],relative
   dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data);count+=1
 print(json.dumps({'runs':len(index['runs']),'files_verified_and_restored':count,'destination':str(out)}))
if __name__=='__main__':restore(sys.argv[1])
