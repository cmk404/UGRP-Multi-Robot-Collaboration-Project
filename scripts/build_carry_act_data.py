"""Join successful teacher RGB with actual issued commands, by decision order."""
import argparse,hashlib,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from harness.pair_carry_act_contract import context,AXES,SCALES

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
def extract(root):
    root=root.resolve();result=read(root/'result.json')
    if not result['physical_success'] or not result['protocol_complete']:raise ValueError('unsuccessful teacher '+str(root))
    if result.get('carry_policy'):raise ValueError('student is not teacher')
    decisions=read(root/'pair-decisions.json');commands=read(root/'issued-commands.json')
    bindings=read(root/'skill-bindings.json')['pair_model_slots'];plan=read(root/'committed-plan.json')['plan']
    static=read(root/'episode-setup-only.json')['static_map']
    goal=static['docks'][plan['dock']]['slots']['beam']['center_m']
    route=next(t['route'] for t in plan['tasks'] if t['object']=='beam')
    image_bind={v['frame_id']:v for v in decisions if v['kind']=='image_binding'}
    rows=[v for v in decisions if v['kind'] in ('carry','rotating_carry')]
    if len({v['kind'] for v in rows})!=1:raise ValueError('mixed/no carry trajectory')
    sequences={};image_hashes={}
    for slot,rid in bindings.items():
        actions=[v for v in commands[rid] if v['stage']=='TRANSIT']
        if len(actions)!=len(rows)-1:raise ValueError('unaligned actions '+str(root))
        previous=[0.,0.,0.];sequence=[]
        for i,row in enumerate(rows):
            done=row['control']['done'] if row['kind']=='carry' else all(d['done'] for d in row['decisions'].values())
            if done!=(i==len(rows)-1):raise ValueError('only terminal arrival expected')
            bound=image_bind[row['frame_ids'][slot]]
            refs={'own':bound['own'][slot],'top':bound['raw_top']}
            target=[0.,0.,0.] if done else [actions[i]['action'][k] for k in AXES]
            if not done:
                cmd=actions[i]
                if cmd['action']['kind']!='mecanum' or not .19<=cmd['action']['duration_s']<=.26:raise ValueError('wrong command interval')
                if row['kind']=='carry' and abs(cmd['issued_at_s']-row['sim_time_s'])>1e-6:raise ValueError('time mismatch')
                if row['kind']=='rotating_carry' and any(abs(cmd['action'][k]-row['decisions'][slot]['action'][k])>1e-10 for k in AXES):raise ValueError('rotating label mismatch')
            for ref in refs.values():
                p=(root/ref['path']).resolve()
                if not p.is_relative_to(root) or sha(p)!=ref['sha256']:raise ValueError('image provenance')
                image_hashes[ref['path']]=ref['sha256']
            sequence.append({'id':str(root)+':'+slot+':'+str(i),'images':refs,'context':context(goal,route,slot,previous),
                'action':[v/s for v,s in zip(target,SCALES)]+[float(done)],'done':done,'frame_id':row['frame_ids'][slot]})
            previous=target
        sequences[slot]=sequence
    files=['result.json','pair-decisions.json','issued-commands.json','skill-bindings.json','committed-plan.json','episode-setup-only.json']
    return {'root':str(root),'source_sha':result['source_sha'],'files':{n:sha(root/n) for n in files},'image_hashes':image_hashes,'sequences':sequences}

def main():
    p=argparse.ArgumentParser();p.add_argument('--train',type=Path,nargs='+',required=True);p.add_argument('--development',type=Path,nargs='+',required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    if {x.resolve() for x in a.train}&{x.resolve() for x in a.development}:raise ValueError('split overlap')
    result={k:[extract(v) for v in getattr(a,k)] for k in ('train','development')}
    a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:{'episodes':len(v),'rows':sum(len(s) for e in v for s in e['sequences'].values())} for k,v in result.items()}))
if __name__=='__main__':main()
