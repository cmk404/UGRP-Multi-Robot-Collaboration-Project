"""Join disjoint, completed trial shards without altering their raw reports."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.carry_failure_metrics import schedule,aggregate


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def combine(protocol_path,report_paths):
    protocol_path=Path(protocol_path);protocol=json.loads(protocol_path.read_text())
    source=protocol['expected_source_sha'];jobs=schedule(protocol,list(protocol['conditions']))
    expected={j['trial_id']:j for j in jobs};rows={};exports=[];shards=[]
    for path in map(Path,report_paths):
        r=json.loads(path.read_text());p=r['protocol']
        if not r['complete'] or r['source_sha']!=source:raise ValueError('shard incomplete or source mismatch')
        for key in ('test','controls','grasp','stages','asset_sha256','expected_source_sha','evaluation'):
            if p.get(key)!=protocol.get(key):raise ValueError('shard protocol mismatch: '+key)
        if not p['conditions'] or any(k not in protocol['conditions'] or v!=protocol['conditions'][k]
                                    for k,v in p['conditions'].items()):raise ValueError('shard condition mismatch')
        declared=schedule(p,list(p['conditions']))
        if r['jobs']!=declared or {j['trial_id'] for j in declared}!={x['trial_id'] for x in r['runs']}:
            raise ValueError('shard run set mismatch')
        for row in r['runs']:
            tid=row['trial_id']
            if tid in rows or tid not in expected:raise ValueError('duplicate or unexpected trial')
            if any(row[k]!=expected[tid][k] for k in ('case','condition','repeat')):raise ValueError('trial metadata mismatch')
            rows[tid]=row
        exports.extend(r.get('tensorboard_exports',[]))
        shards.append({'report':str(path.resolve()),'sha256':sha(path),'protocol_sha256':r['protocol_sha256']})
    if rows.keys()!=expected.keys():raise ValueError('missing planned trials')
    ordered=[rows[j['trial_id']] for j in jobs];counts=aggregate(jobs,ordered)
    return {'source_sha':source,'protocol_sha256':sha(protocol_path),'protocol':protocol,
            'jobs':jobs,'runs':ordered,'not_attempted':[],'complete':counts['complete'],
            'failure_estimates':counts,'tensorboard_exports':exports,'shards':shards,
            'scope':'Post-run aggregation only; immutable raw shard reports retain launch order, commands and timing. This does not qualify a controller; use assess_research_cohort for raw evidence readback.'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--protocol',type=Path,required=True);p.add_argument('--report',type=Path,action='append',required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();r=combine(a.protocol,a.report)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    with a.out.open('x') as f:json.dump(r,f,indent=2);f.write('\n')
    print(json.dumps({'complete':r['complete'],'trials':len(r['runs']),'shards':len(r['shards'])}))


if __name__=='__main__':main()
