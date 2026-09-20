#!/usr/bin/env python3
"""Compare observers on recorded RGB; this does not execute their new actions."""
from pathlib import Path
import argparse
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from harness.pair_navigation import PairVision, TemporalPairVision, ROBOTS
from scripts.audit_pair_carry_sync import _rgb


def replay(root, vision_mode='temporal'):
    record = json.loads((root/'result.json').read_text())
    if vision_mode not in ('temporal','temporal-edges'): raise ValueError('unknown candidate vision mode')
    observers = {'legacy':PairVision(record['map']),
                 'temporal':TemporalPairVision(record['map'],edge_axis=vision_mode=='temporal-edges')}
    results = {name:{'accepted':0,'errors':[],'memory_matches':0} for name in observers}
    maximum_delta = 0.
    last = {}
    started = time.monotonic()
    for row in record['steps']:
        own = _rgb(root,row['images']['r1']['own'])
        top = _rgb(root,row['images']['r1']['top'])
        outputs = {}
        for name,observer in observers.items():
            try:
                outputs[name] = observer.observe(own,top)
                results[name]['accepted'] += 1
                results[name]['memory_matches'] += sum(o.get('appearance_source')=='memory' for o in outputs[name].values())
            except ValueError as error:
                results[name]['errors'].append({'index':row['index'],'error':str(error)})
        if len(outputs)==2:
            import math
            maximum_delta=max(maximum_delta,*(math.dist(outputs['legacy'][r]['xy_m'],outputs['temporal'][r]['xy_m']) for r in ROBOTS))
        last = outputs
    return {'map_id':record['map']['map_id'],'frames':len(record['steps']),'candidate_vision_mode':vision_mode,
            'baseline_source_sha':record['source_sha'],'observers':results,
            'max_position_difference_when_both_valid_m':maximum_delta,
            'last_observations':last,'wall_seconds':time.monotonic()-started,
            'scope':'Recorded baseline RGB only; revised actions and physical success are untested here'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_dirs',nargs='+',type=Path)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--vision-mode',choices=('temporal','temporal-edges'),default='temporal')
    args=p.parse_args()
    if args.out.exists(): raise FileExistsError(args.out)
    records=[]
    for root in args.run_dirs:
        result=replay(root.resolve(),args.vision_mode);records.append(result)
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({k:result[k] for k in ('map_id','frames','observers','max_position_difference_when_both_valid_m')},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
