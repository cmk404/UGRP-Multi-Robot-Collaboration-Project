from pathlib import Path
import sys,json,hashlib,shutil,copy
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from harness.pair_transport_vision import GeometryPairVision
base=Path('/Users/changmin/projects/ugrp-worktrees/pair-vision-recovery/outputs/pair-vision-recovery-v2-2026-09-14')
out=Path('tests/fixtures/pair_transport_robustness');out.mkdir(parents=True,exist_ok=True)
cases={'narrow-door':451,'l-corner':132,'s-bends':400,'staggered-obstacles':400,'blocked-branch':373}
manifest={'source_execution_sha':'cdac7b7e5154cdc531169a46c90732461b5ae06e','provenance':'Preceding observer states reconstructed only from saved top RGB. Wheel angle checks use visible wheel rectangles; drop examples are manually identified own RGB silhouettes. No runtime ground truth inputs.','cases':{},'files':{}}
keys=('centers','angles','geometry_angles','wheel_origins','payload','payload_angle','payload_origin_angle')
for case,idx in cases.items():
 raw=base/case;r=json.loads((raw/'result.json').read_text());v=GeometryPairVision(r['map']);v.carry_monitor.observe=lambda f:None
 folder=out/case;folder.mkdir(exist_ok=True)
 for s in r['steps'][:idx+1]:
  own=(raw/s['images']['r1']['own']['path']).read_bytes();top=(raw/s['images']['r1']['top']['path']).read_bytes()
  if s['index']==idx:
   state={k:copy.deepcopy(getattr(v,k)) for k in keys}
   (folder/'state.json').write_text(json.dumps(state,indent=2)+'\n')
  v.observe(own,top)
 for name,source in [('start-own.jpg',raw/'rgb/nav-0000-r1-own.jpg'),('start-top.jpg',raw/'rgb/nav-0000-top.jpg'),('target-own.jpg',raw/f'rgb/nav-{idx:04d}-r1-own.jpg'),('target-top.jpg',raw/f'rgb/nav-{idx:04d}-top.jpg')]:
  shutil.copyfile(source,folder/name)
 (folder/'map.json').write_text(json.dumps(r['map'],indent=2)+'\n')
 manifest['cases'][case]={'source_directory':str(raw),'index':idx,'manual_wheel_axis_deg':{'narrow-door':0,'l-corner':62,'s-bends':0,'staggered-obstacles':45,'blocked-branch':90}[case],'angle_tolerance_deg':3}
for case in ['s-bends','staggered-obstacles']:
 for rid in ['r1','r3']:
  shutil.copyfile(base/case/f'rgb/nav-0749-{rid}-own.jpg',out/case/f'dropped-{rid}-own.jpg')
for p in sorted(out.rglob('*')):
 if p.is_file():manifest['files'][str(p.relative_to(out))]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('fixtures',len(manifest['files']))
