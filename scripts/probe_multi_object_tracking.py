#!/usr/bin/env python3
"""Privileged rendered pose sweep for RGB tracker QA; NOT physical transport.

The fixture owner changes free-body poses to generate a visual sequence. The
tracker sees only JPEGs and authored mission IDs. The pose evaluator runs after
each decision and never corrects identity. No robot policy or contact success.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from harness.multi_object_tracking import CargoTracker
from scripts.multi_object_scene_preview import MultiObjectScene
from sim.multi_object_scene import configuration
from sim.multi_object_suite import load_pilot


def run(output):
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise ValueError('commit source first')
    import mujoco
    import numpy as np
    case=next(c for c in load_pilot()[1] if c['id']=='staging_three')
    scene=MultiObjectScene(configuration(case),output/'scene')
    result={'scope':'rendered pose sweep, not physical manipulation, learned control or transport',
            'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'frames':[],'identity_errors':0}
    try:
        scene.open();tracker=CargoTracker(case['mission']);w=scene.world
        base={oid:list(x['position_m']) for oid,x in scene.config['setup_only']['objects'].items()}
        for i in range(16):
            # Privileged fixture generation only. Do not pass this motion to tracker.
            for index,(oid,item) in enumerate(scene.config['setup_only']['objects'].items()):
                jid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_JOINT,item['joint_name'])
                q=int(w.model.jnt_qposadr[jid]);pos=base[oid][:]
                pos[0]+=.01*i
                if item['kind']=='box':pos[1]+=(.008 if index==1 else -.008)*i
                w.data.qpos[q:q+3]=pos
            mujoco.mj_forward(w.model,w.data)
            f=scene.capture(f'pose-{i:02}')['r1']
            tracked=tracker.observe(f['top_bytes'],frame_id=i+1,now_s=i*.2)
            # Truth is computed only AFTER the RGB decision for this frame.
            cid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_CAMERA,'cctv_top')
            truth={}
            for oid,item in scene.config['setup_only']['objects'].items():
                bid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_BODY,item['body_name'])
                p=(w.data.xpos[bid]-w.data.cam_xpos[cid])@w.data.cam_xmat[cid].reshape(3,3)
                focal=720/(2*np.tan(np.radians(w.model.cam_fovy[cid])/2))
                truth[oid]=[(480+focal*p[0]/-p[2])/960,(360-focal*p[1]/-p[2])/720]
            mistakes=[]
            if tracked['valid']:
                for oid,t in tracked['tracks'].items():
                    closest=min(truth,key=lambda key:np.linalg.norm(np.array(truth[key])-t['center']))
                    if oid!=closest:mistakes.append(oid)
            else:mistakes=list(truth)
            result['identity_errors']+=len(mistakes)
            result['frames'].append({'index':i,'rgb':f['shared_top_rgb'],'tracker':tracked,
                                     'evaluation_only_centers':truth,'mismatches':mistakes})
        result['weld_active']=bool(w.data.eq_active.any())
    finally:
        scene.close()
        output.mkdir(parents=True,exist_ok=True)
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        hashes={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest() for p in output.rglob('*') if p.is_file()}
        (output/'artifact-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='frames'}))
    return 0 if len(result['frames'])==16 and result['identity_errors']==0 else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('new output required')
    sys.exit(run(a.output))
