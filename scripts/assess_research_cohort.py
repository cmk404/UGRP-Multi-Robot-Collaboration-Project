"""Read-only admission assessment for one declared, completed physical cohort."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.carry_failure_metrics import aggregate, outcome


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path,kind):
    try:value=json.loads(path.read_text())
    except (OSError,ValueError):return None
    return value if isinstance(value,kind) else None


def assess(report_path):
    report_path=Path(report_path)
    report=json.loads(report_path.read_text())
    jobs=report['jobs'];rows=[];scenes=defaultdict(set);evidence=[]
    for saved in report['runs']:
        root=Path(saved['output'])
        result=read_json(root/'result.json',dict) or {}
        verdict=outcome(root,saved['exit_code'],saved.get('timed_out',False))
        if any(verdict.get(key)!=value for key,value in saved['outcome'].items()):
            # Old reports may predate extra classification fields. Keep the
            # fresh readback authoritative, while recording the disagreement.
            consistent=False
        else:consistent=True
        artifacts={name:sha(root/name) for name in ('result.json','scene.xml','pair-decisions.json',
                   'issued-commands.json','evaluation-only.json','execution.mp4') if (root/name).exists()}
        if 'scene.xml' in artifacts:scenes[saved['case']['id']].add(artifacts['scene.xml'])
        required={'result.json','scene.xml','pair-decisions.json','issued-commands.json','evaluation-only.json','execution.mp4'}
        evaluation=result.get('evaluation',{})
        contacts=evaluation.get('robot_robot_contact_samples')
        boundary=(result.get('source_sha')==report['source_sha'] and evaluation.get('weld_steps')==0
                  and result.get('obstacle_contact_steps')==0 and contacts==0)
        commands=read_json(root/'issued-commands.json',dict)
        decisions=read_json(root/'pair-decisions.json',list)
        valid_json=(bool(result) and commands is not None and all(isinstance(v,list) for v in commands.values())
                    and decisions is not None and all(isinstance(d,dict) for d in decisions)
                    and read_json(root/'evaluation-only.json',dict) is not None)
        decisions=decisions if decisions is not None and all(isinstance(d,dict) for d in decisions) else []
        act=[d for d in decisions if d.get('kind')=='act_carry']
        latency=[v['inference_wall_s'] for d in act for v in d.get('inputs',{}).values() if 'inference_wall_s' in v]
        rows.append({**saved,'outcome':verdict})
        evidence.append({'trial_id':saved['trial_id'],'condition':saved['condition'],
            'raw':str(root),'artifact_sha256':artifacts,'required_evidence_complete':required<=artifacts.keys(),
            'artifact_json_valid':valid_json,
            'report_matches_readback':consistent,'source_physics_checks_pass':boundary,
            'outcome':verdict,'wall_s':result.get('wall_s'),
            'commands':sum(map(len,commands.values())) if commands is not None and all(isinstance(v,list) for v in commands.values()) else None,
            'act_model_calls':sum(len(d.get('inputs',{})) for d in act),
            'mean_act_inference_s':statistics.mean(latency) if latency else None,
            'external_llm_calls':result.get('llm_calls'),'cost_usd':result.get('cost_usd'),
            'simultaneous_loaded_motion_s':evaluation.get('concurrent_transport',{}).get('simultaneous_loaded_motion_s'),
            'controller':result.get('carry_policy','RGB'),'stop_mode':result.get('carry_stop_mode'),
            'pure_act':result.get('pure_act',False)})
    counts=aggregate(jobs,rows)
    matching=all(len(hashes)==1 for hashes in scenes.values())
    conditions={}
    for name,count in counts['conditions'].items():
        trials=[e for e in evidence if e['condition']==name]
        qualified=(counts['complete'] and count['attempted']==count['planned'] and count['failures']==0
                   and matching and all(t['required_evidence_complete'] and t['artifact_json_valid'] and t['report_matches_readback']
                   and t['source_physics_checks_pass'] for t in trials))
        success_times=[t['wall_s'] for t in trials if t['outcome']['whole_success'] and t['wall_s'] is not None]
        conditions[name]={**count,'qualified_for_declared_cases':qualified,
            'successful_wall_s_median':statistics.median(success_times) if success_times else None}
    return {'source_sha':report['source_sha'],'protocol_sha256':report['protocol_sha256'],
            'report':str(report_path.resolve()),'report_sha256':sha(report_path),
            'complete':counts['complete'],'conditions':conditions,'same_scene_per_case':matching,
            'trials':evidence,'scope':'Admission applies only to the declared fixed case suite. RGB-guarded and RGB-refined ACT are separately named. Physical completion, component carry entry, runtime errors and unattempted trials remain distinct. No communication, model-generalization or physical-robot claim.'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();result=assess(args.report)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with args.out.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='trials'},indent=2))
    return 0 if result['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
