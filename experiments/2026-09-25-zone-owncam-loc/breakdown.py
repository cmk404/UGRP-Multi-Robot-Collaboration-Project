import sys, json, copy, numpy as np, collections
sys.path.insert(0,'/Users/changmin/projects/ugrp-wt/zone-owncam-loc')
from pathlib import Path
import scripts.eval_owncam_localization as E
data=Path(sys.argv[1]); eps=sys.argv[2].split(','); p=json.load(open(sys.argv[3]))['params']
for k,v in json.loads(sys.argv[4]).items():
    a,b=k.split('.')
    if b=='noise_mult': p['motion']['noise_rel']=[x*v for x in p['motion']['noise_rel']]; p['motion']['noise_abs']=[x*v for x in p['motion']['noise_abs']]
    else: p[a][b]=v
rows=[]
for ep in eps:
    static,inp=E.static_for(data/ep)[1],E.load_inputs(data/ep)
    est,s=E.localize(static,inp,p,seed=0); rows+=E.score_episode(data/ep,est)
def st(rs):
    rs=[r for r in rs if r['initialized']]
    if not rs: return 'none'
    pe=np.array([r['pos_err_m'] for r in rs]); ye=np.array([r['yaw_err_deg'] for r in rs])
    return f'n={len(rs):4d} vis={np.mean([r["visible"] for r in rs]):.2f} pos p50 {np.median(pe):.3f} p90 {np.percentile(pe,90):.3f} yaw p50 {np.median(ye):.2f} p90 {np.percentile(ye,90):.2f}'
for key in sys.argv[5].split(','):
    for v in sorted({r[key] for r in rows}):
        sel=[r for r in rows if r[key]==v]
        print(f'{key}={v:24s} all  {st(sel)}')
        print(f'{"":30s} door {st([r for r in sel if r["door_region"]])}')
# error vs time since tag
for lo,hi in ((0,0.01),(0.01,2),(2,10),(10,30),(30,1e9)):
    sel=[r for r in rows if r['initialized'] and r['since_tag_s'] is not None and lo<=r['since_tag_s']<hi]
    print(f'since_tag {lo}-{hi}s', st(sel))
