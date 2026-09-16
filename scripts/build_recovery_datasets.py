#!/usr/bin/env python3
"""Build explicit source lists; never replace excluded demonstration slots."""
import argparse,json,hashlib
from pathlib import Path

def read(p):return json.loads(p.read_text())
def main():
 p=argparse.ArgumentParser();p.add_argument('--curriculum',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--aggregation',type=Path);p.add_argument('--nominal-only',action='store_true');a=p.parse_args();a.out.mkdir(exist_ok=True,parents=True)
 roots={name:a.curriculum/name for name in ('nominal','recovery','development')}
 def selection(name,predicate=lambda c:True):
  root=roots[name];summary=read(root/'summary.json')
  if not summary['complete']:raise ValueError('incomplete '+name)
  chosen=[r for r in summary['cases'] if predicate(r)]
  return {'root':str(root.resolve()),'cases':[r['id'] for r in chosen if r['training_eligible']]},[r['id'] for r in chosen if not r['training_eligible']]
 development,excluded_dev=selection('development');nominal,excluded_nominal=selection('nominal')
 configs={'nominal':{'train':[nominal],'development':[development],'exclusions':{'training':excluded_nominal,'development':excluded_dev}}}
 if not a.nominal_only:
  common,ecommon=selection('nominal',lambda c:int(c['id'].split('-')[-1])<=24)
  core,ecore=selection('recovery',lambda c:int(c['id'].split('-')[-1])<=4)
  recovery,erecovery=selection('recovery')
  configs['bootstrap']={'train':[common,core],'development':[development],'exclusions':{'training':ecommon+ecore,'development':excluded_dev}}
  configs['recovery']={'train':[common,recovery],'development':[development],'exclusions':{'training':ecommon+erecovery,'development':excluded_dev}}
  if a.aggregation:
   roots['aggregation']=a.aggregation;aggregate,eaggregate=selection('aggregation')
   configs['aggregated']={'train':[common,core,aggregate],'development':[development],'exclusions':{'training':ecommon+ecore+eaggregate,'development':excluded_dev}}
 for name,c in configs.items():
  c['condition']=name;c['scope']='Successful eligible trajectories only; all excluded attempt IDs retained. Teacher combination-region screening applies to every labeled frame.'
  path=a.out/(name+'.json')
  if path.exists() and read(path)!=c:raise FileExistsError('would change dataset '+str(path))
  path.write_text(json.dumps(c,indent=2)+'\n');print(name,[(len(x['cases']),x['root']) for x in c['train']],c['exclusions'])
if __name__=='__main__':main()
