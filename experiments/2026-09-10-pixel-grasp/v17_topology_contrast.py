#!/usr/bin/env python3
# Visualizes low/high supported contrast topology using helpers from the offline diagnostic.
exec(open('/tmp/v17_sweep_failure_diag.py').read().split('reports=[];')[0])
CASES2=[(226,227),(237,238),(317,318),(538,539)]
folder=ROOT/'pixel-grasp-20260910-r1-v17'; result=json.loads((folder/'progress.json').read_text()); calls={c['round']:c for c in result['calls']}; panels=[]; nums=[]
for a,b in CASES2:
 old=cv2.imread(str(folder/'r1'/f'{a:03d}-top.jpg')); new=cv2.imread(str(folder/'r1'/f'{b:03d}-top.jpg')); h,w=new.shape[:2]; op,_=beam_geom(calls[a],w,h); npoly,_=beam_geom(calls[b],w,h); endpoint=np.array(calls[b]['observation']['alignment']['endpoint'])*[w,h]; cs=supported_candidates(old,new)
 row=[]
 for c in [cs[0],cs[-1]]:
  ev,ext,hull,block=evaluate(c,op,npoly,endpoint,new.shape); vis=new.copy(); vis[block>0]=(vis[block>0]*.65+np.array([0,80,255])*.35).astype(np.uint8); vis[ext]=(vis[ext]*.2+np.array([255,0,255])*.8).astype(np.uint8); cv2.polylines(vis,[np.round(hull).astype(np.int32)],True,(0,255,0),1); cv2.circle(vis,tuple(np.round(endpoint).astype(int)),4,(0,0,255),-1); crop=cv2.resize(vis[525:610,590:720],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST); cv2.putText(crop,f'{a}->{b} t{c["threshold"]} '+('PASS' if ev['passed'] else 'FAIL'),(4,18),0,.5,(255,255,255),1,cv2.LINE_AA); row.append(crop); nums.append({'pair':[a,b],**ev})
 panels.append(np.hstack(row))
cv2.imwrite('/tmp/v17_topology_contrast.jpg',np.vstack(panels)); Path('/tmp/v17_topology_contrast.json').write_text(json.dumps(nums,indent=2))
