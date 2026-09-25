import sys, json, math, copy, numpy as np
sys.path.insert(0,'/Users/changmin/projects/ugrp-wt/zone-owncam-loc')
from pathlib import Path
import scripts.eval_owncam_localization as E
data=Path(sys.argv[1]); eps=sys.argv[2].split(','); cal=json.load(open(sys.argv[3]))['params']
variants=json.loads(sys.argv[4]); thr=json.load(open('/Users/changmin/projects/ugrp-wt/zone-owncam-loc/experiments/2026-09-25-zone-owncam-loc/thresholds.json'))
seeds=(0,1)
cache={ep:(E.static_for(data/ep)[1], E.load_inputs(data/ep)) for ep in eps}
for name,over in variants.items():
    p=copy.deepcopy(cal)
    for k,v in over.items():
        a,b=k.split('.')
        if b=='noise_mult': p['motion']['noise_rel']=[x*v for x in p['motion']['noise_rel']]; p['motion']['noise_abs']=[x*v for x in p['motion']['noise_abs']]
        elif a=='top': p[b]=v
        else: p[a][b]=v
    rows=[]; resets=0
    for ep in eps:
        static,inp=cache[ep]
        for seed in seeds:
            est,s=E.localize(static,inp,p,seed=seed); resets+=s['resets']
            rows+=E.score_episode(data/ep,est)
    g=E.gates(rows,thr)
    allr=[r for r in rows if r['initialized'] and r['mode']!='teacher_carry']
    f=lambda rs,q: round(float(np.percentile([r['pos_err_m'] for r in rs],q)),3)
    print(f'{name:22s} rs={resets:4d} '+' '.join(f"{k[:2]}:{v.get('pos_m')}/{v.get('yaw_deg')}{'P' if v['pass'] else 'F'}" for k,v in g.items())+f' | all p50 {f(allr,50)} p90 {f(allr,90)} max {f(allr,100)}', flush=True)
