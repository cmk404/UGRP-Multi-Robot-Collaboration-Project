"""Offline episode-level termination audit; no model tuning or simulator calls."""
import argparse
import hashlib
import json
from pathlib import Path


def audit(rows,threshold=.65):
    episodes={}
    for row in rows:
        root,slot,index=row['id'].rsplit(':',2)
        value=float(row['prediction'][3]);target=float(row['target'][3])
        import math
        if not math.isfinite(value) or target not in (0.,1.):raise ValueError('invalid stop sample')
        episodes.setdefault(root,{}).setdefault(slot,[]).append((int(index),value,target))
    results=[]
    for root,robots in episodes.items():
        if set(robots)!={'r1','r3'}:raise ValueError('both model slots required')
        observations={}
        for slot,samples in robots.items():
            samples.sort()
            if [x[0] for x in samples]!=list(range(len(samples))):raise ValueError('noncontiguous sequence')
            if sum(x[2] for x in samples)!=1 or samples[-1][2]!=1:raise ValueError('one final terminal required')
            premature=[i for i,p,t in samples if p>=threshold and not t]
            observations[slot]={'samples':len(samples),'terminal_score':samples[-1][1],
                'premature_stop_indices':premature,'first_stop_remaining_commands':
                    len(samples)-1-premature[0] if premature else 0}
        if len(robots['r1'])!=len(robots['r3']):raise ValueError('unmatched pair sequence')
        unsafe=any(s['premature_stop_indices'] for s in observations.values())
        missed=any(s['terminal_score']<threshold for s in observations.values())
        results.append({'episode':root,'robots':observations,'premature_pair_hold':unsafe,'terminal_missed':missed})
    if not results:raise ValueError('empty predictions')
    return {'episodes':len(results),'premature_pair_hold_episodes':sum(r['premature_pair_hold'] for r in results),
            'missed_terminal_episodes':sum(r['terminal_missed'] for r in results),
            'offline_termination_pass':all(not r['premature_pair_hold'] and not r['terminal_missed'] for r in results),
            'eligible_for_default':False,
            'scope':'Teacher-trajectory replay audit only. Any unilateral false done freezes the coupled pair. An offline pass still requires independent physical qualification; false-done frame percentage alone is not admission.',
            'threshold':threshold,'results':results}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--predictions',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    result=audit(json.loads(args.predictions.read_text()))
    result.update(predictions=str(args.predictions.resolve()),predictions_sha256=hashlib.sha256(args.predictions.read_bytes()).hexdigest())
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with args.out.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='results'}))


if __name__=='__main__':main()
