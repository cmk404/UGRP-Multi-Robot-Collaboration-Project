"""Compare observer-only source invariants against the preceding full cohort."""
from pathlib import Path
import json,hashlib,sys
root,out=map(Path,sys.argv[1:]);baseline=Path('/Users/changmin/projects/ugrp-worktrees/pair-transport-robustness/outputs/pair-robust-final-2026-09-14')
c=json.loads((root/'cohort.json').read_text());checks=[]
for t in c['trials']:
 p=root/t['id'];b=baseline/t['id'];r=json.loads((p/'result.json').read_text());old=json.loads((b/'result.json').read_text())
 equal={k:r[k]==old[k] for k in ['scene_xml_sha256','map_sha256','appearance_manifest_sha256','grasp_skill_sha256','invariants_initial','invariants_final']}
 a=json.loads((p/'execution-trace.json').read_text());o=json.loads((b/'execution-trace.json').read_text())
 for stage in ['grasp_initialization','grasp_pre_recovery_settle','grasp_rgb_recovery','grasp_lift']:
  equal[stage+'_commands']= [x.get('command') for x in a if x['stage']==stage]==[x.get('command') for x in o if x['stage']==stage]
 assert all(equal.values()),(t['id'],equal)
 assert r['weld_active_ticks']==0 and r['external_model_calls']==0 and r['cost_usd']==0
 checks.append({'id':t['id'],'unchanged':equal,'actual_close_command':[x['command'] for x in a if x['stage']=='grasp_close'],'source_sha':r['source_sha']})
res={'passed':True,'scope':'Output-only exact scene/map/camera/grasp-model and fixed arm command comparison; actual fixed closure differs intentionally','baseline_source_sha':old['source_sha'],'candidate_source_sha':c['source_sha'],'cases':checks}
out.write_text(json.dumps(res,indent=2)+'\n');print('unchanged scene/model/cameras/lift',len(checks))
