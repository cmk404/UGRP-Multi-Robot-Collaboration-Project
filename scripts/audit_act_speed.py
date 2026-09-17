#!/usr/bin/env python3
"""Post-run audit; never available as an observation to the actor."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def audit(protocol_path,cohorts,models,teacher,out):
    if out.exists():raise FileExistsError(out)
    import torch
    from harness.reference_act import RGBAct
    from scripts.run_camera_approach_student import choose_actions
    from scripts.approach_speed_teacher import teacher_command
    from scripts.run_camera_pair_transport import evaluate_grasp_samples
    torch.set_num_threads(2)
    protocol=read(protocol_path)
    evidence={'scope':'All original file/RGB hashes and controller actions; first/middle/last and ready-transition model decisions; exact paired initial state and RGB; recomputed grasp evaluation. No runtime truth correction.',
              'image_references_verified':0,'raw_files_verified':0,'control_actions_replayed':0,
              'model_decisions_replayed':0,'teacher_decisions_replayed':0,'pairs':[],
              'runs':[],'groups':{},'errors':[],'sources':{},'external_model_calls':0,'external_model_tokens':0}
    evidence['audit_source_sha']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    records={}
    for cohort in cohorts:
        summary=read(cohort/'summary.json')
        if not summary['complete'] or summary['protocol_sha256']!=sha(protocol_path):raise ValueError('incomplete or changed protocol')
        subprocess.run(['git','diff','--exit-code',summary['source_sha'],'HEAD','--','harness',
                        'scripts/run_camera_approach_student.py','scripts/camera_approach_scene.py',
                        'scripts/approach_speed_teacher.py'],cwd=ROOT,check=True)
        evidence['sources'][str(cohort.resolve())]={'source_sha':summary['source_sha'],'summary_sha256':sha(cohort/'summary.json'),'manifest_sha256':sha(cohort/'raw-manifest.json')}
        for rel,expected in read(cohort/'raw-manifest.json').items():
            p=(cohort/rel).resolve()
            if not p.is_relative_to(cohort.resolve()) or sha(p)!=expected:raise ValueError('raw hash mismatch')
            evidence['raw_files_verified']+=1
        for row in summary['cases']:
            key=(row['case_id'],row['condition'])
            if key in records:raise ValueError('duplicate test run')
            if row['returncode']!=0 or row['timeout'] or row['error']:raise ValueError('runtime error; cannot certify cohort')
            records[key]=row
    expected={(c['id'],condition) for c in protocol['test_cases'] for condition in protocol['conditions']}
    if set(records)!=expected:raise ValueError('missing or additional test runs')
    actors={name:{r:RGBAct.load(root/r/'act') for r in ('r1','r3')} for name,root in models.items()}
    teacher_report=read(teacher/'report.json')
    goals={r:teacher_report['goal_references'][r]['base_x'] for r in ('r1','r3')}
    for case in protocol['test_cases']:
        reference=None;case_results={}
        for condition in protocol['conditions']:
            record=records[(case['id'],condition)];folder=Path(record['raw_dir']);r=read(folder/'result.json')
            if r['config']['start_poses']!=case['start_poses'] or r['config']['weld']:raise ValueError('fixture mismatch')
            if r['source_sha']!=record['source_sha']:raise ValueError('runner source mismatch')
            for state in ('evaluation_initial_state','approach_end_state','final_physics'):
                if any(r[state]['constraints_active'].values()):raise ValueError('weld enabled')
            calls=r['approach_calls']
            first_images=[{k:x['images'][k]['sha256'] for k in ('own','top')} for x in calls[:2]]
            pair=(r['evaluation_initial_state'],first_images)
            if reference is None:reference=pair
            elif pair!=reference:raise ValueError('unmatched initial state or RGB')
            if condition=='teacher2':
                if r['approach_policy']!='privileged_x_teacher' or r['teacher_reference_sha256']!=sha(teacher/'report.json'):
                    raise ValueError('teacher provenance mismatch')
            else:
                if 'teacher_observations' in r or r['approach_policy']!='upstream_act_rgb_preset':raise ValueError('privileged student')
                for robot,files in r['act_checkpoint_sha256'].items():
                    for name,h in files.items():
                        if sha(models[condition]/robot/'act'/name)!=h:raise ValueError('checkpoint mismatch')
            def images(v):
                if isinstance(v,dict):
                    if isinstance(v.get('path'),str) and v['path'].startswith('rgb/') and v.get('sha256'):
                        p=(folder/v['path']).resolve()
                        if not p.is_relative_to(folder.resolve()) or sha(p)!=v['sha256']:raise ValueError('RGB mismatch')
                        evidence['image_references_verified']+=1
                    for sub in v.values():images(sub)
                elif isinstance(v,list):
                    for sub in v:images(sub)
            images(r)
            hist={robot:[] for robot in ('r1','r3')}
            for j in range(0,len(calls),2):
                rows=calls[j:j+2]
                if [v['robot_id'] for v in rows]!=['r1','r3']:raise ValueError('robot pairing mismatch')
                control=choose_actions({v['robot_id']:v['decision'] for v in rows},'visual',rows[0]['phase'],rows[0]['cruise_index'],r['config']['playback_seconds'])
                for call in rows:
                    robot=call['robot_id']
                    if set(call['images'])!={'own','top'} or call['action']!=control['actions'][robot] or call['own_command_history']!=hist[robot]:raise ValueError('control or history mismatch')
                    hist[robot].append(call['action']);evidence['control_actions_replayed']+=1
                    if condition=='teacher2':
                        decision=teacher_command(r['teacher_observations'][j//2]['remaining_m'][robot],2.)
                        if decision!=call['decision']:raise ValueError('teacher command mismatch')
                        evidence['teacher_decisions_replayed']+=1
            if condition!='teacher2':
                for robot in ('r1','r3'):
                    rows=[c for c in calls if c['robot_id']==robot]
                    indices={0,len(rows)//2,len(rows)-1}
                    indices.update(i for i in range(1,len(rows)) if rows[i]['decision']['ready']!=rows[i-1]['decision']['ready'])
                    for i in sorted(indices):
                        c=rows[i];actual=actors[condition][robot].predict(*[(folder/c['images'][key]['path']).read_bytes() for key in ('own','top')])
                        if actual!=c['decision']:raise ValueError('model prediction mismatch')
                        evidence['model_decisions_replayed']+=1
            samples=[json.loads(line) for line in (folder/'evaluation-only.jsonl').read_text().splitlines()]
            evaluation=evaluate_grasp_samples(samples)
            if evaluation!=r['evaluation']:raise ValueError('grasp evaluation mismatch')
            success=bool(r['approach_ok'] and evaluation['grasp_success'] and r['approach_payload_contact_steps']==0 and not r['error'])
            if success!=record['success'] or success!=r['success']:raise ValueError('success mismatch')
            info={**record,'total_sim_s':r['final_physics']['sim_time_s']-r['evaluation_initial_state']['sim_time_s'],
                  'stop_x_error_m':{robot:r['approach_end_state']['bases'][robot][0]-goals[robot] for robot in ('r1','r3')},
                  'grasp_evaluation':evaluation}
            info['within_teacher_stop_tolerance'] = all(abs(value) <= .004 for value in info['stop_x_error_m'].values())
            info['success_with_common_stop_tolerance'] = success and info['within_teacher_stop_tolerance']
            evidence['runs'].append(info);case_results[condition]=info
        evidence['pairs'].append({'case_id':case['id'],'initial_physics_and_rgb_identical':True,
                                  'teacher_failed':not case_results['teacher2']['success'],
                                  'student_successes':[c for c in models if case_results[c]['success']]})
    for group in sorted({c['group'] for c in protocol['test_cases']}):
        evidence['groups'][group]={}
        for condition in protocol['conditions']:
            rows=[r for r in evidence['runs'] if r['group']==group and r['condition']==condition]
            successes=[r for r in rows if r['success']]
            evidence['groups'][group][condition]={'trials':len(rows),'successes':len(successes),
                'successes_with_common_4mm_stop':sum(r['success_with_common_stop_tolerance'] for r in rows),
                'approach_ready':sum(r['approach_ok'] for r in rows),'contact_failures':sum(bool(r['contact_steps']) for r in rows),
                'mean_approach_sim_s_success_only':statistics.mean(r['sim_s'] for r in successes) if successes else None,
                'mean_total_sim_s_success_only':statistics.mean(r['total_sim_s'] for r in successes) if successes else None}
    evidence['paired_speed']={}
    for fast in ('fast_act16','fast_act17'):
        pairs=[(records[(c['id'],'slow_act')],records[(c['id'],fast)]) for c in protocol['test_cases'] if c['group']=='in_domain']
        good=[(a,b) for a,b in pairs if a['success'] and b['success']]
        evidence['paired_speed'][fast]={'both_success_pairs':len(good),'all_pairs':len(pairs),
            'mean_slow_sim_s':statistics.mean(a['sim_s'] for a,b in good) if good else None,
            'mean_fast_sim_s':statistics.mean(b['sim_s'] for a,b in good) if good else None,
            'mean_pairwise_speedup':statistics.mean(a['sim_s']/b['sim_s'] for a,b in good) if good else None}
    hard=[p for p in evidence['pairs'] if p['teacher_failed']]
    evidence['teacher_failure_subset']={'cases':len(hard),'case_ids':[p['case_id'] for p in hard],
        'student_successes':{c:sum(c in p['student_successes'] for p in hard) for c in models},
        'both_fast_seeds_success':[p['case_id'] for p in hard if all(c in p['student_successes'] for c in ('fast_act16','fast_act17'))]}
    evidence['common_stop_diagnostic'] = {
        'scope': 'Post hoc diagnostic after first far-start student success: apply the teacher 4mm x tolerance equally to all methods at settled approach end. Does not replace preregistered physical task success or alter any controller.',
        'teacher_failure_subset_student_successes_with_4mm_stop': {
            c: sum(r['success_with_common_stop_tolerance'] for r in evidence['runs']
                   if r['condition']==c and r['case_id'] in evidence['teacher_failure_subset']['case_ids'])
            for c in models},
        'both_fast_seeds_success_with_4mm_stop': [
            cid for cid in evidence['teacher_failure_subset']['case_ids']
            if all(any(r['case_id']==cid and r['condition']==c and r['success_with_common_stop_tolerance']
                       for r in evidence['runs']) for c in ('fast_act16','fast_act17'))]}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(evidence,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:evidence[k] for k in ('groups','paired_speed','teacher_failure_subset')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('protocol','teacher','slow-model','fast16-model','fast17-model','out'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--cohorts',type=Path,nargs='+',required=True)
    a=p.parse_args();audit(a.protocol,a.cohorts,{'slow_act':a.slow_model,'fast_act16':a.fast16_model,'fast_act17':a.fast17_model},a.teacher,a.out)
