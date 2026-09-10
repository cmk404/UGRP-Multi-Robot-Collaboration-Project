"""Output-only fine command response analysis from RGB; never reads evaluator."""
from pathlib import Path
import json,sys,hashlib
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from harness.camera_gripper_motion import GripperMotionTracker
from harness.camera_beam_features import extract_beams,select_beam
from harness.camera_pixel_grasp import alignment_features
root=Path(sys.argv[1]);orig=Path(sys.argv[2])
mapping=json.loads(Path(__file__).with_name('probe-v13-fine-mapping.json').read_text());report=json.loads((root/'result.json').read_text());caps={c['index']:c for c in report['captures'] if c['phase']=='after'}
def capbytes(n,k):return (root/caps[n]['files'][k]['path']).read_bytes()
def features(closed,opened,own):
 t=GripperMotionTracker(initial_gripper_pulse=1500);t.update(closed,None);g=t.update(opened,{'kind':'arm','servo_id':1,'pulse':2000});b=select_beam(extract_beams(opened,robust_shaft=True),g.get('center'),[.508,.5]);os=extract_beams(own);o=max(os,key=lambda b:b['area_px']) if os else None
 return {'gripper':g,'beam':b,'alignment':alignment_features(g,b,None,o),'own_present':o is not None}
before=features((orig/'r1/302-top.jpg').read_bytes(),(orig/'r1/303-top.jpg').read_bytes(),(orig/'r1/303-own.jpg').read_bytes());rows=[]
for m in mapping['probes']:
 n=m['measurement_after_probe_index'];z=m['restored_after_probe_index'];after=features(capbytes(n-1,'top'),capbytes(n,'top'),capbytes(n,'own'));restored=features(capbytes(z-1,'top'),capbytes(z,'top'),capbytes(z,'own'));rows.append({**m,'before':before,'after':after,'restored':restored})
 def show(f):
  a=f['alignment'];return None if a is None else [round(a['top_cost'],3),[round(v,3) for v in a['offset_px']],round(a['axis_error_rad'],3),f['own_present']]
 print(m['label'],show(before),'->',show(after),'restore',show(restored));before=restored
Path(sys.argv[3]).write_text(json.dumps(rows,indent=2)+'\n')
