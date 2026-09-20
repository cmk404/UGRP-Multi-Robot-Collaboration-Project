#!/usr/bin/env python3
"""Offline wheel appearance export from annotated RGB; no simulator state."""
import argparse,hashlib,json,sys
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from harness.dispatch_pair_navigation import PairVision

def export(source,output,source_sha):
    raw=source.read_bytes();frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    mask=PairVision._mask(None,frame)
    annotations={'r1':[275,354],'r3':[274,169]}
    output.mkdir(parents=True,exist_ok=True)
    meta={'schema':'ugrp.pair_wheel_appearance.v1','payload_length_m':.45,
        'source_sha':source_sha,'source_path':str(source.resolve()),
        'source_image_sha256':hashlib.sha256(raw).hexdigest(),
        'labels':'Offline wheel-envelope centres annotated in archived carry RGB, not simulator poses', 'templates':{}}
    for rid,(x,y) in annotations.items():
        image=mask[y-30:y+31,x-30:x+31]
        if image.shape!=(61,61) or np.count_nonzero(image)<80:raise ValueError('bad annotated crop')
        data=cv2.imencode('.png',cv2.GaussianBlur(image,(3,3),.7))[1].tobytes()
        (output/(rid+'.png')).write_bytes(data)
        meta['templates'][rid]={'path':rid+'.png','sha256':hashlib.sha256(data).hexdigest(),'offline_center_uv':[x,y]}
    (output/'manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--source-sha',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();export(a.source,a.output,a.source_sha)
