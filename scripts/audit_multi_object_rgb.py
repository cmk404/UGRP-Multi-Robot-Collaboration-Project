#!/usr/bin/env python3
"""Audit first-frame RGB binding on exported scenes. No motion-success claim."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.multi_object_tracking import CargoTracker


def audit(folder):
    results=[]
    for file in sorted(folder.glob('*/static-task.json')):
        task=json.loads(file.read_text());image=file.parent/'preview/rgb/preview-top.jpg'
        tracker=CargoTracker(task['mission']);data=image.read_bytes()
        initial=tracker.observe(data,frame_id=1,now_s=0.)
        repeated=tracker.observe(data,frame_id=2,now_s=.2)
        results.append({'case':file.parent.name,'initial':initial,'repeated':repeated,
                        'input_path':str(image.resolve()),'input_sha256':hashlib.sha256(data).hexdigest()})
    return {'scope':'recorded initial RGB and identical-frame continuity only; no physical tracking or transport',
            'cases':len(results),'passed':sum(r['initial']['valid'] and r['repeated']['valid'] for r in results),
            'results':results}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('preserve existing audit; choose new output')
    report=audit(a.input);a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'}))
    sys.exit(0 if report['cases']==report['passed']==12 else 1)
