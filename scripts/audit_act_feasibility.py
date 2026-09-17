#!/usr/bin/env python3
"""Read-only audit of fixed ACT final cohorts; never used for actor control."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def audit(first, second, models1, models2, out):
    if out.exists():
        raise FileExistsError(out)
    import torch
    from harness.reference_act import RGBAct
    from harness.camera_approach_student import predict_approach
    from scripts.run_camera_approach_student import choose_actions
    torch.set_num_threads(2)
    summaries = [read(root/'summary.json') for root in (first, second)]
    if not all(s['complete'] for s in summaries):
        raise ValueError('both final cohorts must be complete')
    distances = summaries[0]['protocol']['physical_pilot_distances']
    if distances != summaries[1]['protocol']['physical_pilot_distances']:
        raise ValueError('different final start conditions')
    if len(summaries[0]['cases']) != len(distances)*2 or len(summaries[1]['cases']) != len(distances):
        raise ValueError('incomplete case coverage')
    # The second seed reuses only the exactly identical fitted kernel baseline.
    for rid in ('r1','r3'):
        if digest(models1/rid/'kernel.json') != digest(models2/rid/'kernel.json'):
            raise ValueError('kernel differs between training seeds')
    actors = [{rid:RGBAct.load(root/rid/'act') for rid in ('r1','r3')} for root in (models1,models2)]
    kernels = {rid:read(models1/rid/'kernel.json') for rid in ('r1','r3')}
    evidence={'image_references_checked':0,'decision_samples_replayed':0,'all_control_actions_replayed':0,
              'pairs':[], 'groups':{}, 'errors':[], 'all_initial_final_weld_off':True,
              'scope':'All RGB references and controller actions; first/middle/last learned decisions per robot per run. Not every video frame.'}
    evidence['audit_source_sha'] = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    evidence['actor_source_shas'] = sorted({s['source_sha'] for s in summaries})
    for source in evidence['actor_source_shas']:
        subprocess.run(['git', 'diff', '--exit-code', source, 'HEAD', '--',
                        'harness', 'scripts/run_camera_approach_student.py',
                        'scripts/camera_approach_scene.py'], cwd=ROOT, check=True)
    evidence['raw_files_verified'] = 0
    for root in (first,second):
        for name,expected in read(root/'raw-manifest.json').items():
            path=(root/name).resolve()
            if not path.is_relative_to(root.resolve()) or digest(path)!=expected:
                raise ValueError('raw manifest mismatch')
            evidence['raw_files_verified']+=1
    groups = {'kernel':[], 'act_seed16':[], 'act_seed17':[]}
    for i, distance in enumerate(distances,1):
        reports=[]
        for group,root,name,actor_index in [('kernel',first,f'{i:02d}-kernel',None),
                                           ('act_seed16',first,f'{i:02d}-act',0),
                                           ('act_seed17',second,f'{i:02d}-act',1)]:
            folder=root/name;result=read(folder/'result.json');reports.append(result)
            if result['error'] or [result['config']['distance'][rid] for rid in ('r1','r3')] != distance:
                raise ValueError(f'{group}/{name}: error or mismatched distance')
            if result['config']['weld']:
                raise ValueError('weld enabled')
            for phase in ('evaluation_initial_state','final_physics'):
                if any(result[phase]['constraints_active'].values()):raise ValueError('weld enabled')
            if actor_index is not None:
                modelroot=(models1,models2)[actor_index]
                for rid,files in result['act_checkpoint_sha256'].items():
                    for filename,h in files.items():
                        if digest(modelroot/rid/'act'/filename)!=h:raise ValueError('checkpoint mismatch')
            def images(value):
                if isinstance(value,dict):
                    if isinstance(value.get('path'),str) and value.get('sha256') and value['path'].startswith('rgb/'):
                        path=(folder/value['path']).resolve()
                        if not path.is_relative_to(folder.resolve()) or digest(path)!=value['sha256']:
                            raise ValueError('RGB path or hash mismatch')
                        evidence['image_references_checked']+=1
                    for sub in value.values():images(sub)
                elif isinstance(value,list):
                    for sub in value:images(sub)
            images(result)
            calls=result['approach_calls']
            histories={rid:[] for rid in ('r1','r3')}
            for j in range(0,len(calls),2):
                pair=calls[j:j+2]
                if [x['robot_id'] for x in pair] != ['r1','r3']:raise ValueError('robot pairing mismatch')
                control=choose_actions({x['robot_id']:x['decision'] for x in pair},'visual',pair[0]['phase'],pair[0]['cruise_index'],result['config']['playback_seconds'])
                for call in pair:
                    rid=call['robot_id']
                    if set(call['images'])!={'own','top'}:raise ValueError('wrong actor image fields')
                    if call['action']!=control['actions'][rid] or call['own_command_history']!=histories[rid]:
                        raise ValueError('control action/history mismatch')
                    histories[rid].append(call['action']);evidence['all_control_actions_replayed']+=1
            for rid in ('r1','r3'):
                rows=[c for c in calls if c['robot_id']==rid]
                for ix in sorted({0,len(rows)//2,len(rows)-1}):
                    call=rows[ix];own=(folder/call['images']['own']['path']).read_bytes();top=(folder/call['images']['top']['path']).read_bytes()
                    actual=actors[actor_index][rid].predict(own,top) if actor_index is not None else predict_approach(kernels[rid],own,top)
                    expected=call['decision']
                    for key in ('ok','ready','forward'):
                        if actual[key]!=expected[key]:raise ValueError(f'prediction mismatch {group} {name} {rid} {ix} {key}')
                    if actor_index is not None and actual['stop_score']!=expected['stop_score']:raise ValueError('stop prediction mismatch')
                    evidence['decision_samples_replayed']+=1
            groups[group].append({'case':i,'distance':distance,'success':bool(result['success']),
                                  'approach_ok':result['approach_ok'],'grasp_success':result['grasp_success'],
                                  'sim_s':result['approach_elapsed_sim_s'],'wall_s':result['wall_elapsed_s'],
                                  'commands':len(calls),'contact_steps':result['approach_payload_contact_steps'],
                                  'stop_reason':result['stop_reason']})
        same_state=all(r['evaluation_initial_state']==reports[0]['evaluation_initial_state'] for r in reports[1:])
        same_rgb=all(all(r['approach_calls'][j]['images'][k]['sha256']==reports[0]['approach_calls'][j]['images'][k]['sha256'] for j in (0,1) for k in ('own','top')) for r in reports[1:])
        if not same_state or not same_rgb:raise ValueError('unmatched initial conditions')
        evidence['pairs'].append({'case':i,'initial_state_identical':same_state,'initial_rgb_identical':same_rgb})
    for group,rows in groups.items():
        evidence['groups'][group]={'trials':len(rows),'successes':sum(r['success'] for r in rows),
                                  'mean_sim_s':statistics.mean(r['sim_s'] for r in rows),
                                  'mean_wall_s':statistics.mean(r['wall_s'] for r in rows),
                                  'mean_commands':statistics.mean(r['commands'] for r in rows),
                                  'contact_steps':sum(r['contact_steps'] for r in rows),'cases':rows}
    evidence['sources']={str(p.resolve()):digest(p) for root in (first,second) for p in (root/'summary.json',root/'raw-manifest.json')}
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps({k:{m:v for m,v in info.items() if m!='cases'} for k,info in evidence['groups'].items()},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('first','second','models1','models2','out'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();audit(a.first,a.second,a.models1,a.models2,a.out)
