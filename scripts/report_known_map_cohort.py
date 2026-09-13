"""Offline output-only trajectory plots and replay audits for a completed cohort."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from scripts.audit_known_map_navigation import audit_run


def plot_pair(map_run, direct_run, target):
    """Draw authored geometry and referee trajectories, never actor inputs."""
    data = json.loads((map_run / 'actor-map.json').read_text())
    xmin,xmax,ymin,ymax = data['bounds_m']
    width,height,pad = 1100,880,75
    scale = min((width-2*pad)/(xmax-xmin), (height-210)/(ymax-ymin))
    def p(xy):
        return round(pad+(xy[0]-xmin)*scale), round(130+(ymax-xy[1])*scale)
    img = np.full((height,width,3), 247, np.uint8)
    cv2.putText(img, 'Known map + camera feedback', (pad,42), cv2.FONT_HERSHEY_SIMPLEX, 1., (35,35,35),2,cv2.LINE_AA)
    cv2.putText(img, 'Output-only referee trajectories; one unloaded robot per run', (pad,77), cv2.FONT_HERSHEY_SIMPLEX,.6,(70,70,70),1,cv2.LINE_AA)
    cv2.rectangle(img,p((xmin,ymax)),p((xmax,ymin)),(180,180,180),2)
    inflate = sum(data['footprint'].values())
    for obstacle in data['obstacles']:
        x,y = obstacle['center_m']; hx,hy = obstacle['half_extents_m']
        cv2.rectangle(img,p((x-hx-inflate,min(ymax,y+hy+inflate))),p((x+hx+inflate,max(ymin,y-hy-inflate))),(219,225,233),-1)
    for obstacle in data['obstacles']:
        x,y = obstacle['center_m']; hx,hy = obstacle['half_extents_m']
        cv2.rectangle(img,p((x-hx,y+hy)),p((x+hx,y-hy)),(92,83,77),-1)
    for name,color in [('start',(190,130,45)),('goal',(100,180,80))]:
        z=data['zones'][name]
        cv2.circle(img,p(z['center_m']),round(z['radius_m']*scale),color,2,cv2.LINE_AA)
        q=p(z['center_m']);cv2.putText(img,name,(q[0]-25,q[1]+75),cv2.FONT_HERSHEY_SIMPLEX,.6,color,2,cv2.LINE_AA)
    summaries = []
    for run,label,color in [(direct_run,'direct',(80,100,210)),(map_run,'map',(195,140,20))]:
        if run is None: continue
        rows=[json.loads(line) for line in (run/'evaluation-only.jsonl').read_text().splitlines()]
        points=np.asarray([p(r['robot_xyz_m'][:2]) for r in rows],np.int32)
        cv2.polylines(img,[points],False,color,3,cv2.LINE_AA)
        cv2.circle(img,tuple(points[-1]),6,color,-1,cv2.LINE_AA)
        result=json.loads((run/'result.json').read_text()); ev=result['evaluation']
        summaries.append((f"{label}: {result['actor_status']} | contacts={ev['metrics']['collision_steps']} | {result['decisions']} decisions",color))
    for i,(line,color) in enumerate(summaries):
        cv2.putText(img,line,(pad,height-55+i*26),cv2.FONT_HERSHEY_SIMPLEX,.65,color,2,cv2.LINE_AA)
    if not cv2.imwrite(str(target),img): raise RuntimeError('plot write failed')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cohort',type=Path)
    a=parser.parse_args()
    summaries=[]
    for row in json.loads((a.cohort/'results.json').read_text()):
        root=a.cohort/row['name']
        try:
            audit=audit_run(root)
            entry={k:v for k,v in audit.items() if k!='replay'}
        except Exception as exc:
            entry={'audit_pass':False,'error':str(exc)}
        entry['name']=row['name'];summaries.append(entry)
    (a.cohort/'audits.json').write_text(json.dumps(summaries,indent=2)+'\n')
    for rid in ('r1','r3'):
        map_run=a.cohort/f'heldout_{rid}-slalom-map'
        direct_run=a.cohort/f'heldout_{rid}-slalom-direct'
        if map_run.exists() and direct_run.exists():
            plot_pair(map_run,direct_run,a.cohort/f'{rid}-slalom-comparison.png')
    print(json.dumps(summaries,indent=2))


if __name__=='__main__':
    main()
