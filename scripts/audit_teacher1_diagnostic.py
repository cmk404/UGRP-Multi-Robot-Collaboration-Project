#!/usr/bin/env python3
"""Read-only audit of the post hoc original-speed teacher diagnostic."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.approach_speed_teacher import teacher_command
from scripts.run_camera_pair_transport import evaluate_grasp_samples

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser()
    for key in ('diagnostic','baseline','teacher','out'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():raise FileExistsError(a.out)
    d=read(a.diagnostic/'summary.json');b=read(a.baseline/'summary.json')
    assert d['complete'] and d['baseline_sha256']==sha(a.baseline/'summary.json')
    expected={r['case_id']:r for r in b['cases'] if r['condition']=='teacher2' and not r['success']}
    assert set(expected)==set(d['selected_case_ids'])=={r['case_id'] for r in d['cases']}
    result={'scope':d['scope'],'source_sha':d['source_sha'],'audit_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'summary_sha256':sha(a.diagnostic/'summary.json'),'manifest_sha256':sha(a.diagnostic/'raw-manifest.json'),
            'files_verified':0,'images_verified':0,'teacher_decisions_replayed':0,'runs':[]}
    for rel,h in read(a.diagnostic/'raw-manifest.json').items():
        f=(a.diagnostic/rel).resolve();assert f.is_relative_to(a.diagnostic.resolve()) and sha(f)==h
        result['files_verified']+=1
    goals=read(a.teacher/'report.json')['goal_references']
    for row in d['cases']:
        assert row['returncode']==0 and not row['timeout'] and not row['error']
        folder=Path(row['raw_dir']);r=read(folder/'result.json');old=read(Path(expected[row['case_id']]['raw_dir'])/'result.json')
        assert r['source_sha']==d['source_sha'] and r['approach_policy']=='privileged_x_teacher'
        assert r['teacher_reference_sha256']==sha(a.teacher/'report.json')
        assert r['evaluation_initial_state']==old['evaluation_initial_state']
        assert not r['config']['weld']
        for state in ('evaluation_initial_state','approach_end_state','final_physics'):
            assert not any(r[state]['constraints_active'].values())
        for i in (0,1):
            for key in ('own','top'):assert r['approach_calls'][i]['images'][key]['sha256']==old['approach_calls'][i]['images'][key]['sha256']
        for i,c in enumerate(r['approach_calls']):
            assert teacher_command(r['teacher_observations'][i//2]['remaining_m'][c['robot_id']],1.)==c['decision']
            result['teacher_decisions_replayed']+=1
            for im in c['images'].values():assert sha(folder/im['path'])==im['sha256'];result['images_verified']+=1
        ev=evaluate_grasp_samples([json.loads(l) for l in (folder/'evaluation-only.jsonl').read_text().splitlines()])
        assert ev==r['evaluation']
        success=bool(r['approach_ok'] and ev['grasp_success'] and r['approach_payload_contact_steps']==0 and not r['error'])
        assert success==row['success']==r['success']
        result['runs'].append({**row,'stop_x_error_m':{robot:r['approach_end_state']['bases'][robot][0]-goals[robot]['base_x'] for robot in goals}})
    result['groups']={g:{'trials':sum(r['group']==g for r in result['runs']),'successes':sum(r['group']==g and r['success'] for r in result['runs'])} for g in sorted({r['group'] for r in result['runs']})}
    a.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['groups'],indent=2))
if __name__=='__main__':main()
