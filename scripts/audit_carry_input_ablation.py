"""Post-run only: reconstruct causal actor wires and independently score physics."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import numpy as np
from harness.carry_input_history import window_indices,wire_request
from harness.carry_input_client import InputCarryClient
from harness.pair_carry_act_contract import context,AXES
from scripts.run_dispatch_e2e import Referee
from scripts.build_carry_act_data import extract


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--act-python',type=Path,required=True);a=p.parse_args();out=a.out.resolve()
    report=read(out/'report.json');freeze=read(out/'final-freeze.json');data=read(out/'dataset.json')
    if not report['complete']:raise ValueError('cohort incomplete')
    assert freeze['source_sha']==report['source_sha'] and freeze['protocol_sha256']==report['protocol_sha256']
    assert sha(out/'dataset.json')==report['protocol']['dataset_sha256']
    for split in ('train','development'):
        for episode in data[split]:assert extract(Path(episode['root']))==episode
    for name,model in freeze['models'].items():
        assert sha(Path(model['path'])/'model.safetensors')==model['sha256']
        training=read(Path(model['path']).parent/'report.json')
        assert training['complete'] and training['dataset_sha256']==sha(out/'dataset.json')
    rows=[];initials={};entries={};checked_wires=checked_refs=native_replays=0
    image_hashes={}
    def image(root,ref):
        nonlocal checked_refs
        path=(root/ref['path']).resolve();assert path.is_relative_to(root)
        if path not in image_hashes:image_hashes[path]=sha(path)
        assert image_hashes[path]==ref['sha256'];checked_refs+=1
        return path.read_bytes()
    for run in report['runs']:
        root=Path(run['output']).resolve()
        if not (root/'result.json').exists():
            rows.append({'case':run['case']['id'],'condition':run['condition'],'completed_report':False,'error':'missing result','exit_code':run['exit_code']});continue
        r=read(root/'result.json');setup=read(root/'episode-setup-only.json');commands=read(root/'issued-commands.json')
        assert r['source_sha']==report['source_sha'] and r['camera_geometry_unchanged']
        assert r['llm_calls']==0 and r['evaluation']['weld_steps']==0
        samples=[json.loads(line) for line in (root/'referee-only.jsonl').open()]
        assert not any(s['weld'] for s in samples)
        ref=Referee.__new__(Referee);ref.file=io.StringIO();ref.samples=samples;ref.geoms={'beam':0,'box':1}
        ref.scene=SimpleNamespace(config=setup,command_history=commands,weld_steps=0)
        evaluation=ref.finish(r['plan']);assert evaluation==r['evaluation']
        plan=read(root/'committed-plan.json')['plan'];bindings=read(root/'skill-bindings.json')['pair_model_slots']
        decisions=read(root/'pair-decisions.json');act=[v for v in decisions if v['kind']=='act_carry']
        carry_window=evaluation['cargo']['beam']['carry_clearance']['window_s']
        row={'case':run['case']['id'],'condition':run['condition'],'completed_report':True,
             'whole_success':bool(r['physical_success'] and r['protocol_complete'] and not r['error'] and not r['obstacle_contact_steps']),
             'beam_success':bool(evaluation['cargo']['beam']['physical_success'] and not r['error'] and not r['obstacle_contact_steps']),
             'carry_entered':bool(act or carry_window),'phase_at_end':r['phase'],'error':r['error'],
             'protocol_complete':r['protocol_complete'],'physical_success':r['physical_success'],
             'carry_decisions':len(act),'carry_sim_s':carry_window[1]-carry_window[0] if carry_window else None,
             'wall_s':r['wall_s'],'obstacle_contact_steps':r['obstacle_contact_steps'],'external_model_calls':r['llm_calls'],
             'cost_usd':r['cost_usd'],'commands':sum(map(len,commands.values())),'raw':str(root)}
        slot=setup['static_map']['docks'][plan['dock']]['slots']['beam']
        corners=np.array(samples[-1]['cargo']['beam']['corners'])[:,:2]
        row['minimum_slot_margin_m']=float((np.array(slot['half_extents_m'])-np.abs(corners-slot['center_m'])).min())
        row['one_robot_done_rounds']=sum(sum(d['done'] for d in v['decisions'].values())==1 for v in act)
        row['both_done_rounds']=sum(all(d['done'] for d in v['decisions'].values()) for v in act)
        row['declared_done']=bool(act and act[-1]['ready_count']>=3)
        row['false_done']=bool(row['declared_done'] and not evaluation['cargo']['beam']['physical_success'])
        initials.setdefault(run['case']['id'],{})[run['condition']]={'sample':samples[0],'setup':setup,'plan':plan}
        entries.setdefault(run['case']['id'],{})[run['condition']]={}
        if carry_window:
            entry=min(samples,key=lambda s:abs(s['sim_time_s']-carry_window[0]))
            entries[run['case']['id']][run['condition']]['referee_at_carry']=entry
            source=next(Path(e['root']) for e in data['train'] if Path(e['root']).name==run['case']['teacher_case'])
            tr=read(source/'result.json');t0=tr['evaluation']['cargo']['beam']['carry_clearance']['window_s'][0]
            original=min((json.loads(line) for line in (source/'referee-only.jsonl').open()),key=lambda s:abs(s['sim_time_s']-t0))
            row['entry_beam_difference_from_training_m']=float(np.linalg.norm(np.array(entry['cargo']['beam']['position'])-original['cargo']['beam']['position']))
        if act:
            model=freeze['models'][run['condition']];assert r['carry_model_sha256']==model['sha256']
            length=model['arm']['history'];goal=slot['center_m'];route=next(t['route'] for t in plan['tasks'] if t['object']=='beam')
            previous={s:[0.,0.,0.] for s in bindings};past={s:[] for s in bindings};ready_count=0;latencies=[]
            client=InputCarryClient(a.act_python,Path(model['path']))
            issued_by_time={s:{round(c['issued_at_s'],6):c for c in commands[rid] if c['stage']=='TRANSIT'} for s,rid in bindings.items()}
            try:
                for i,v in enumerate(act):
                    ready_count=ready_count+1 if all(d['done'] for d in v['decisions'].values()) else 0
                    assert ready_count==v['ready_count'];pause=any(d['done'] for d in v['decisions'].values())
                    for s,inp in v['inputs'].items():
                        assert inp['physical_robot_id']==bindings[s] and inp['context']==context(goal,route,s,previous[s])
                        current={'images':inp['images'],'context':inp['context'],'sim_time_s':v['sim_time_s'],'frame_id':v['decisions'][s]['own_attachment'].get('frame_id')}
                        # frame_id is a capture sequence, present in the recorded window.
                        current['frame_id']=inp['history'][-1]['frame_id'];past[s].append(current)
                        expected=[past[s][j] for j in window_indices(i,length)]
                        assert inp['history']==expected
                        frames=[{'own_rgb':image(root,f['images']['own']),'top_rgb':image(root,f['images']['top']),'context':f['context']} for f in expected]
                        payload=json.dumps(wire_request(frames,length))+'\n'
                        assert hashlib.sha256(payload.encode()).hexdigest()==inp['wire_sha256'];checked_wires+=1
                        if i in (0,len(act)-1):
                            actual=client.predict(frames)
                            assert all(actual[k]==v['decisions'][s][k] for k in ('action','done','stop_score'));native_replays+=1
                        action=dict.fromkeys(AXES,0.) if pause else v['decisions'][s]['action'];assert action==v['actions'][s]
                        command=issued_by_time[s].get(round(v['sim_time_s'],6))
                        if v['permission']['phase']=='GO' and ready_count<3:
                            assert command and all(command['action'][k]==action[k] for k in AXES)
                        else:assert command is None
                        previous[s]=[action[k] for k in AXES];latencies.append(inp['inference_wall_s'])
                    if i==0:entries[run['case']['id']][run['condition']]['rgb_sha256']={s:{k:ref['sha256'] for k,ref in inp['images'].items()} for s,inp in v['inputs'].items()}
            finally:client.close()
            row['inference_wall_s']={'median':float(np.median(latencies)),'p95':float(np.percentile(latencies,95)),'max':max(latencies)}
        elif run['condition']=='teacher':
            captures={d['frame_id']:d for d in decisions if d['kind']=='image_binding'}
            first=next((d for d in decisions if d['kind'] in ('carry','rotating_carry')),None)
            if first:
                image_record=captures[first['frame_ids']['r1']]
                entries[run['case']['id']]['teacher']['rgb_sha256']={s:{'own':image_record['own'][s]['sha256'],'top':image_record['raw_top']['sha256']} for s in bindings}
        row['raw_hashes']={name:sha(root/name) for name in ('result.json','pair-decisions.json','issued-commands.json','referee-only.jsonl','execution.mp4')}
        rows.append(row)
    expected_conditions={'teacher',*freeze['models']}
    for case,values in initials.items():
        assert set(values)==expected_conditions
        assert all(v==values['teacher'] for v in values.values()),(case,'initial state mismatch')
    entry_matches={}
    for case,values in entries.items():
        images={c:v['rgb_sha256'] for c,v in values.items() if v.get('rgb_sha256')}
        entry_matches[case]={'policies_with_entry_images':len(images),'all_identical':len({json.dumps(v,sort_keys=True) for v in images.values()})<=1}
    counts={}
    for condition in sorted(expected_conditions):
        rs=[r for r in rows if r['condition']==condition];entered=[r for r in rs if r.get('carry_entered')]
        counts[condition]={'whole_successes':sum(r.get('whole_success',False) for r in rs),'runs':len(rs),
                           'carry_successes':sum(r['beam_success'] for r in entered),'carry_entries':len(entered)}
    result={'passed':True,'source_sha':report['source_sha'],'rows':rows,'summary':counts,'carry_entry':entries,
            'entry_image_comparison':entry_matches,'initial_states_matched':len(initials),'image_refs_checked':checked_refs,
            'wires_reconstructed':checked_wires,'sampled_native_decisions_replayed':native_replays,
            'scope':'All input history/wires/commands and post-run evaluator reconstructed; first/last native inference replay only; no physical rerun in audit.'}
    (out/'audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'passed':True,'summary':counts,'wires':checked_wires,'native':native_replays},indent=2))


if __name__=='__main__':main()
