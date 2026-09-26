"""Dev-only motion row-scale/tau fit on closed-loop dev drive segments (GT offline; never test).
Usage: python3 fit_motion_rowscale.py  (reads outputs/owncam-loop-20260925/dev-a*)"""
import sys,json,math,numpy as np
from pathlib import Path
def integrate(cmds,t0,t1,G,tau,dt=.01):
    # body-frame displacement (start-yaw frame) of the mean model
    vel=np.zeros(3); x=y=th=0.; i=0; cmd=np.zeros(3); exp=-1; t=t0
    cs=[c for c in cmds if c['t']<t1]
    # state at t0: take last command before t0
    for c in cs:
        if c['t']<=t0:
            if c['kind']=='mecanum': cmd=np.array([c['forward'],c['left'],c['turn']]); exp=c['t']+c['duration_s']
            elif c['kind'] in('hold','stop'): cmd=np.zeros(3); exp=-1
    cs=[c for c in cs if c['t']>t0]
    while t<t1:
        while cs and cs[0]['t']<=t:
            c=cs.pop(0)
            if c['kind']=='mecanum': cmd=np.array([c['forward'],c['left'],c['turn']]); exp=c['t']+c['duration_s']
            elif c['kind'] in('hold','stop'): cmd=np.zeros(3); exp=-1
        u=cmd if t<exp else np.zeros(3)
        vel+= (1-math.exp(-dt/tau))*(G@u-vel)
        x+=(math.cos(th)*vel[0]-math.sin(th)*vel[1])*dt; y+=(math.sin(th)*vel[0]+math.cos(th)*vel[1])*dt; th+=vel[2]*dt; t+=dt
    return np.array([x,y,th])
CAL=json.load(open(Path(__file__).with_name('calibration_loop.json')))
O=Path('/Users/changmin/projects/ugrp/outputs/owncam-loop-20260925')
def segs(d):
    cmds=[json.loads(l) for l in open(d/'inputs/commands.jsonl')]
    ev=[e for e in map(json.loads,open(d/'eval_only/frames_eval.jsonl')) if e['phase']=='student']
    cur=[]
    for e in ev+[{'student_state':'end'}]:
        if e['student_state']=='drive': cur.append(e)
        else:
            if len(cur)>5:
                a,b=cur[0],cur[-1]; g0=np.array(a['gt']);g1=np.array(b['gt']);c,sn=math.cos(g0[2]),math.sin(g0[2]);dx,dy=g1[:2]-g0[:2]
                yield cmds,a['t'],b['t'],np.array([c*dx+sn*dy,-sn*dx+c*dy])
            cur=[]
for key,pat in (('motion_loaded','dev-a*/dev-box-s3*'),('motion','dev-a*/dev-nobox-s3*')):
    G0=np.array(CAL['motion_refit_2026_09_26'][key]['gain_before'])
    S=[s for d in sorted(O.glob(pat)) for s in segs(d)]; gt=np.array([s[3] for s in S])
    for tau in (.2,.3,.44,.6,.8,1.0,1.2,1.5):
        pr=np.array([integrate(s[0],s[1],s[2],G0,tau)[:2] for s in S])
        k=[(gt[:,i]*pr[:,i]).sum()/(pr[:,i]**2).sum() for i in (0,1)]
        print(key,len(S),'tau',tau,'scale',np.round(k,3),'rms',np.sqrt(((gt-pr*np.array(k))**2).mean(0)).round(4))
