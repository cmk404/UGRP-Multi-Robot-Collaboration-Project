import sys, json, math, numpy as np, collections
sys.path.insert(0,'/Users/changmin/projects/ugrp-wt/zone-owncam-loc')
from pathlib import Path
import scripts.eval_owncam_localization as E
data=Path(sys.argv[1]); p=json.load(open(sys.argv[3]))['params']['motion']
G=np.array(p['gain']); tau=p['tau_s']
res=collections.defaultdict(list); fits=collections.defaultdict(lambda:[[],[]])
for ep in sys.argv[2].split(','):
    gt=E.read_jsonl(data/ep/'eval_only/gt_trajectory.jsonl'); cmds=E.read_jsonl(data/ep/'inputs/commands.jsonl')
    fr=E.read_jsonl(data/ep/'inputs/frames.jsonl'); post=[(f['t'],f['posture']) for f in fr]
    t,dt,v=E._gt_body_velocity(gt); u=E._active_series(cmds,t); ul=E._lagged(u,dt,tau)
    def lab(tt):
        best='?'
        for a,b in post:
            if a<=tt: best=b
        return best.split('_drive')[0] if best.endswith('_drive') else 'stopped/other'
    labs=[lab(x) for x in t]
    for i in range(len(t)): 
        if np.any(ul[i]!=0): fits[labs[i]][0].append(ul[i]); fits[labs[i]][1].append(v[i])
    step=200
    for i in range(0,len(t)-step,50):
        if not np.any(u[i:i+step]): continue
        x=y=0.;th=gt[i]['yaw']
        for k in range(i,i+step):
            vv=G@ul[k]; x+=(math.cos(th)*vv[0]-math.sin(th)*vv[1])*dt[k]; y+=(math.sin(th)*vv[0]+math.cos(th)*vv[1])*dt[k]; th+=vv[2]*dt[k]
        ex=gt[i+step]['x']-gt[i]['x']-x; ey=gt[i+step]['y']-gt[i]['y']-y; eth=math.degrees((gt[i+step]['yaw']-th+math.pi)%(2*math.pi)-math.pi)
        res[labs[i]].append((math.hypot(ex,ey),abs(eth)))
for k,v in res.items():
    a=np.array(v); print(k,len(a),'10s DR pos p50 %.3f p90 %.3f yaw p50 %.1f p90 %.1f'%(np.median(a[:,0]),np.percentile(a[:,0],90),np.median(a[:,1]),np.percentile(a[:,1],90)))
for k,(U,V) in fits.items():
    U=np.array(U);V=np.array(V); g,*_=np.linalg.lstsq(U,V,rcond=None); print(k,'gain diag',np.round(np.diag(g.T),3), 'n',len(U))
