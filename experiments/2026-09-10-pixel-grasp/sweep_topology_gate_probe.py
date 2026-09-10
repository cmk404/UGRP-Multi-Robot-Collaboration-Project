#!/usr/bin/env python3
"""Offline-only RGB sweep-topology probe; reads saved images/results, writes /tmp only."""
import cv2, numpy as np, json, math
from pathlib import Path
ROOT=Path('/Users/changmin/projects/ugrp/outputs')
CASES=[('v13',218,219),('v13',302,303),('v14',695,696),('v14',990,991)]

def beam_geom(call,w,h):
 b=call['observation']['beam']; poly=np.array([[x*w,y*h] for x,y in b['corners4']],np.float32)
 eps=np.array([[x*w,y*h] for x,y in b['endpoints']],np.float64)
 return poly,eps

def supported_candidates(old,new):
 h,w=new.shape[:2]; delta=np.max(cv2.absdiff(old,new),axis=2)
 base=(delta>15).astype(np.uint8); _,_,s0,_=cv2.connectedComponentsWithStats(base,8)
 candidates=[]
 for threshold in range(15,26):
  mask=(delta>threshold).astype(np.uint8); count,labels,stats,cents=cv2.connectedComponentsWithStats(mask,8)
  if count<=1: continue
  largest=int(np.max(stats[1:,cv2.CC_STAT_AREA])); minimum=max(8,int(round(h*w*.000025)),int(math.ceil(largest*.12)))
  ids=[i for i in range(1,len(stats)) if int(stats[i,cv2.CC_STAT_AREA])>=minimum]
  if not 2<=len(ids)<=8: continue
  centers=cents[ids].astype(float); q=centers-centers.mean(0); cov=q.T@q/max(1,len(ids)-1); vals,vecs=np.linalg.eigh(cov)
  linearity=float(vals[-1]/max(vals[-2],1e-6)); sizes=[int(stats[i,4]) for i in ids]; axis=vecs[:,-1]; span=float(np.ptp(centers@axis)); balance=min(sizes)/max(sizes)
  if linearity<1.8 or span<5 or balance<.35: continue
  score=min(linearity,50)*balance*sum(sizes)
  candidates.append(dict(score=score,threshold=threshold,labels=labels,ids=ids,axis=axis,component_centers=centers,raw_center=centers.mean(0)))
 supported=[]
 for c in candidates:
  supporters=sum(abs(float(c['axis']@d['axis']))>=.95 and float(np.linalg.norm(c['raw_center']-d['raw_center']))<=3 for d in candidates)
  if supporters>=3: c['supporters']=supporters; supported.append(c)
 return supported

def evaluate(c,oldpoly,newpoly,endpoint,shape):
 h,w=shape[:2]; selected=np.isin(c['labels'],c['ids']); block=np.zeros((h,w),np.uint8)
 cv2.fillPoly(block,[np.round(oldpoly).astype(np.int32),np.round(newpoly).astype(np.int32)],1)
 external=selected & (block==0); y,x=np.nonzero(external); pts=np.c_[x,y].astype(float)
 axis=c['axis'].astype(float); axis/=np.linalg.norm(axis)
 if axis[0]<0: axis=-axis
 rawy,rawx=np.nonzero(selected); rawpts=np.c_[rawx,rawy].astype(float); center=rawpts.mean(0)
 proj=(pts-center)@axis if len(pts) else np.array([])
 left=pts[proj<0]; right=pts[proj>0]
 reasons=[]
 if len(left)==0 or len(right)==0: reasons.append('beam subtraction removed one side')
 hull=None; inside=False; interior_area=0.0; bracket=False; side_centers=None
 if len(left) and len(right):
  lc=left.mean(0); rc=right.mean(0); side_centers=[lc.tolist(),rc.tolist()]
  eu=float(endpoint@axis); bracket=float(lc@axis)<eu<float(rc@axis)
  if not bracket: reasons.append('shaft endpoint is not strictly between side-support centroids')
  hull=cv2.convexHull(pts.astype(np.float32)); inside_distance=float(cv2.pointPolygonTest(hull,tuple(endpoint.astype(float)),True))
  inside=inside_distance>0
  if not inside: reasons.append('shaft endpoint is not strictly inside external-support hull')
  # Positive-area intersection with the full detected shaft polygon; endpoint-inside is stricter at terminal end.
  try: interior_area=float(cv2.intersectConvexConvex(hull.astype(np.float32),newpoly.reshape(-1,1,2).astype(np.float32))[0])
  except cv2.error: interior_area=0.0
  if interior_area<=0: reasons.append('no positive-area shaft/support-hull intersection')
 return dict(inside_distance_px=(inside_distance if hull is not None else None),lateral_bracket_margins_px=([float(endpoint@axis-left.mean(0)@axis),float(right.mean(0)@axis-endpoint@axis)] if len(left) and len(right) else None),threshold=c['threshold'],supporters=c['supporters'],raw_pixels=int(selected.sum()),external_pixels=int(external.sum()),removed_beam_pixels=int((selected&(block>0)).sum()),left_pixels=len(left),right_pixels=len(right),side_centers=side_centers,endpoint_bracketed=bool(bracket),endpoint_strictly_inside_hull=bool(inside),intersection_area_px2=interior_area,passed=not reasons,reasons=reasons), external,hull,block

reports=[]; viewrows=[]
for ver,a,b in CASES:
 folder=ROOT/f'pixel-grasp-20260910-r1-{ver}'; result=json.loads((folder/'result.json').read_text()); calls={c['round']:c for c in result['calls']}
 old=cv2.imread(str(folder/'r1'/f'{a:03d}-top.jpg')); new=cv2.imread(str(folder/'r1'/f'{b:03d}-top.jpg')); h,w=new.shape[:2]
 oldpoly,oldeps=beam_geom(calls[a],w,h); newpoly,neweps=beam_geom(calls[b],w,h)
 cs=supported_candidates(old,new)
 # Use the endpoint already selected by the saved RGB alignment observation.
 endpoint=np.array(calls[b]['observation']['alignment']['endpoint'],dtype=float)*np.array([w,h],dtype=float)
 endpoint_match_error=float(np.min(np.linalg.norm(neweps-endpoint,axis=1)))
 evals=[]; render=None
 for c in cs:
  ev,external,hull,block=evaluate(c,oldpoly,newpoly,endpoint,new.shape); evals.append(ev)
  if render is None or c['threshold']==max(cs,key=lambda z:(z['supporters'],z['score'],z['threshold']))['threshold']:
   render=(c,external,hull,block)
 pairpass=bool(evals) and all(e['passed'] for e in evals)
 reports.append(dict(case=[ver,a,b],transition={'old_frame_follows_action':calls[a-1]['action'],'new_frame_follows_action':calls[b-1]['action']},supported_candidate_count=len(evals),all_supported_candidates_pass=pairpass,endpoint=endpoint.tolist(),endpoint_match_error_px=endpoint_match_error,candidates=evals))
 c,external,hull,block=render; vis=new.copy(); vis[block>0]=(vis[block>0]*.65+np.array([0,80,255])*.35).astype(np.uint8); vis[external]=(vis[external]*.2+np.array([255,0,255])*.8).astype(np.uint8)
 cv2.polylines(vis,[np.round(newpoly).astype(np.int32)],True,(0,255,255),1); cv2.circle(vis,tuple(np.round(endpoint).astype(int)),4,(0,0,255),-1)
 if hull is not None: cv2.polylines(vis,[np.round(hull).astype(np.int32)],True,(0,255,0),1)
 crop=vis[525:610,590:720]; crop=cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_NEAREST); cv2.putText(crop,f'{ver} {a}->{b} '+('PASS' if pairpass else 'FAIL'),(4,18),0,.5,(255,255,255),1,cv2.LINE_AA); viewrows.append(crop)
Path('/tmp/sweep_topology_gate_probe.json').write_text(json.dumps(reports,indent=2)); cv2.imwrite('/tmp/sweep_topology_gate_probe.jpg',np.vstack(viewrows))
print(json.dumps([{'case':r['case'],'candidate_count':r['supported_candidate_count'],'pass':r['all_supported_candidates_pass'],'summary':[{'t':e['threshold'],'pass':e['passed'],'ext':e['external_pixels'],'removed':e['removed_beam_pixels'],'L':e['left_pixels'],'R':e['right_pixels'],'bracket':e['endpoint_bracketed'],'inside':e['endpoint_strictly_inside_hull'],'area':round(e['intersection_area_px2'],2),'why':e['reasons']} for e in r['candidates']]} for r in reports],indent=2))
