import cv2,numpy as np,json,math
from pathlib import Path
CASES=[('v13',218,219),('v13',302,303),('v14',695,696),('v14',990,991)]
ROOT=Path('/Users/changmin/projects/ugrp/outputs')

def calib(old,new):
 h,w=new.shape[:2]; d=np.max(cv2.absdiff(old,new),2); base=(d>15).astype(np.uint8); _,_,s0,_=cv2.connectedComponentsWithStats(base,8); largest=int(s0[1:,4].max()); mn=max(8,round(h*w*.000025),math.ceil(largest*.12)); cs=[]
 for th in range(15,26):
  m=(d>th).astype(np.uint8); _,lab,st,ce=cv2.connectedComponentsWithStats(m,8); ids=[i for i in range(1,len(st)) if st[i,4]>=mn]
  if not 2<=len(ids)<=8:continue
  cen=ce[ids]; q=cen-cen.mean(0); val,vec=np.linalg.eigh(q.T@q/max(1,len(ids)-1)); lin=val[-1]/max(val[-2],1e-6); sizes=st[ids,4]; ax=vec[:,-1]; span=np.ptp(cen@ax); bal=sizes.min()/sizes.max()
  if lin>=1.8 and span>=5 and bal>=.35:cs.append((min(lin,50)*bal*sizes.sum(),th,lab,st,ce,ids,ax,cen.mean(0)))
 sup=[]
 for c in cs:
  n=sum(abs(c[6]@z[6])>=.95 and np.linalg.norm(c[7]-z[7])<=3 for z in cs)
  if n>=3:sup.append((n,c))
 _,z=max(sup,key=lambda x:(x[0],x[1][0],x[1][1])); _,th,lab,st,ce,ids,ax,_=z
 if ax[0]<0:ax=-ax
 sel=np.isin(lab,ids); yy,xx=np.nonzero(sel); ctr=np.array([xx.mean(),yy.mean()]); return d,sel,ctr,ax,th

def warm(im,variant=0):
 hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV); H,S,V=cv2.split(hsv)
 pars=[(5,35,60,50),(7,30,80,70),(3,40,45,40)][variant]
 lo,hi,ss,vv=pars; return (H>=lo)&(H<=hi)&(S>=ss)&(V>=vv)

def side_support(im,old,sel,ctr,ax,dil=6,var=0):
 h,w=sel.shape; yy,xx=np.indices((h,w)); proj=(xx-ctr[0])*ax[0]+(yy-ctr[1])*ax[1]
 W=warm(im,var); Wo=warm(old,var); out=[]
 for sign in [-1,1]:
  seed=sel & (proj*sign>0)
  roi=cv2.dilate(seed.astype(np.uint8),np.ones((2*dil+1,2*dil+1),np.uint8))>0
  # side half-plane prevents central beam crossover
  roi &= proj*sign > 0
  cand=W&roi
  # Retain all current warm components touching a 2px dilation of motion seed.
  n,lab,st,ce=cv2.connectedComponentsWithStats(cand.astype(np.uint8),8)
  touch=cv2.dilate(seed.astype(np.uint8),np.ones((5,5),np.uint8))>0
  ids=[i for i in range(1,n) if np.any(touch&(lab==i)) and st[i,4]>=2]
  keep=np.isin(lab,ids)
  y,x=np.nonzero(keep); p=np.c_[x,y].astype(float)
  oldkeep=Wo&roi; oy,ox=np.nonzero(oldkeep); op=np.c_[ox,oy].astype(float)
  out.append((keep,p,op))
 return out

report=[]; views=[]
for ver,a,b in CASES:
 folder=ROOT/f'pixel-grasp-20260910-r1-{ver}'/'r1'; old=cv2.imread(str(folder/f'{a:03d}-top.jpg')); cur=cv2.imread(str(folder/f'{b:03d}-top.jpg')); d,sel,ctr,ax,th=calib(old,cur)
 variants={}
 for var in range(3):
  for dil in [4,6,8]:
   ss=side_support(cur,old,sel,ctr,ax,dil,var); vals=[]
   for keep,p,op in ss:
    vals.append(None if len(p)==0 else {'n':len(p),'bbox':[int(p[:,0].min()),int(p[:,1].min()),int(p[:,0].max()),int(p[:,1].max())],'u':[float((p@ax).min()),float((p@ax).max())],'center':p.mean(0).tolist()})
   variants[f'v{var}_d{dil}']=vals
 base=side_support(cur,old,sel,ctr,ax,6,0)
 vis=np.hstack([old.copy(),cur.copy()]); colors=[(255,0,255),(0,255,255)]
 for si,(keep,p,op) in enumerate(base):
  layer=vis[:,cur.shape[1]:]; layer[keep]=(layer[keep]*.25+np.array(colors[si])*.75).astype(np.uint8)
  if len(p): cv2.rectangle(layer,(int(p[:,0].min()),int(p[:,1].min())),(int(p[:,0].max()),int(p[:,1].max())),colors[si],1)
 # motion magenta/green contours on current
 cc=vis[:,cur.shape[1]:]; cv2.drawContours(cc,cv2.findContours(sel.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0],-1,(0,255,0),1)
 crop=vis[530:610,590:720*2 if False else 1280+720] # wrong combined crop custom below
 combined=np.hstack([old[530:610,590:720],cc[530:610,590:720]]); combined=cv2.resize(combined,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST); cv2.putText(combined,f'{ver} {a} closed -> {b} open',(4,16),0,.45,(255,255,255),1); views.append(combined)
 # Diagnose whether the apparent current-color extents are real object boundaries.
 diagnostics=[]
 W,Wo=warm(cur,0),warm(old,0)
 hsv=cv2.cvtColor(cur,cv2.COLOR_BGR2HSV); H,S,V=cv2.split(hsv)
 orange=((H>=8)&(H<=25)&(S>=100)&(V>=130)).astype(np.uint8)
 _,olab,_,_=cv2.connectedComponentsWithStats(orange,8)
 shaft=(olab==olab[520,650])
 yy0,xx0=np.indices(sel.shape); proj0=(xx0-ctr[0])*ax[0]+(yy0-ctr[1])*ax[1]
 for si,(keep,pix,_) in enumerate(base):
  sign=(-1,1)[si]; seed=sel&(proj0*sign>0)
  roi=cv2.dilate(seed.astype(np.uint8),np.ones((13,13),np.uint8))>0; roi&=proj0*sign>0
  boundary=cv2.morphologyEx(roi.astype(np.uint8),cv2.MORPH_GRADIENT,np.ones((3,3),np.uint8))>0
  diagnostics.append({'pixels':int(np.count_nonzero(keep)),'new_only_vs_closed':int(np.count_nonzero(keep&~Wo)),'already_warm_in_closed':int(np.count_nonzero(keep&Wo)),'shaft_component_overlap':int(np.count_nonzero(keep&shaft)),'touches_seed_roi_boundary':bool(np.any(keep&boundary))})
 report.append({'case':[ver,a,b],'pose':'open: current frame follows prior issued servo1 pulse 2000','threshold':th,'tracker_center':ctr.tolist(),'axis':ax.tolist(),'base_current_support':variants['v0_d6'],'diagnostics':diagnostics,'sensitivity':variants})
cv2.imwrite('/tmp/current_open_pad_probe.jpg',np.vstack(views)); Path('/tmp/current_open_pad_probe.json').write_text(json.dumps(report,indent=2)); print(json.dumps([{'case':x['case'],'base':x['base_current_support'],'sens':x['sensitivity']} for x in report],indent=2))
