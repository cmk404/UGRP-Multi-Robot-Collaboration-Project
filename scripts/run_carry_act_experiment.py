"""Bounded owned experiment: development, two training seeds, paired final trials."""
import argparse,concurrent.futures,hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def write(p,v):p.write_text(json.dumps(v,indent=2)+'\n')
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--act-python',type=Path,required=True);p.add_argument('--mjpython',type=Path,required=True);p.add_argument('--grasp',type=Path,required=True);p.add_argument('--stages',type=Path,required=True);a=p.parse_args();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
    protocol_path=ROOT/'experiments/2026-09-18-act-pair-carry/protocol.json';protocol=json.loads(protocol_path.read_text());source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    report={'complete':False,'source_sha':source,'protocol_sha256':hashlib.sha256(protocol_path.read_bytes()).hexdigest(),'protocol':protocol,'runs':[]};write(a.out/'report.json',report)
    def frozen():
        assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==source
        assert not subprocess.check_output(['git','status','--porcelain'],cwd=ROOT)
    def run(cmd,name):
        frozen();start=time.time();log=a.out/(name+'.log')
        with log.open('w') as f:proc=subprocess.run(list(map(str,cmd)),cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        frozen();return {'name':name,'command':list(map(str,cmd)),'exit_code':proc.returncode,'wall_s':time.time()-start,'log':str(log)}
    def trial(case,condition,phase,model=None):
        output=a.out/phase/condition/case['id'];base=Path(protocol['train'][int(case['teacher_case'][1:])-1])
        cmd=[a.mjpython,ROOT/'scripts/run_dispatch_e2e.py','--executor','skills','--variant',case['variant'],'--seed','11','--required-dock','dock_a','--plan-replay',base/'committed-plan.json','--grasp-model-dir',a.grasp,'--stage-model-dir',a.stages,'--output',output,'--max-wall-s','2400','--video-fps','4','--spawn-offset',*map(str,case['offset'])]
        if model:cmd+=['--carry-act-model',model,'--carry-act-python',a.act_python,'--carry-act-max-steps',str(protocol['constraints']['max_carry_steps'])]
        result=run(cmd,phase+'-'+condition+'-'+case['id']);result.update(case=case,condition=condition,phase=phase,output=str(output))
        if (output/'result.json').exists():
            r=json.loads((output/'result.json').read_text());result['summary']={k:r.get(k) for k in ('physical_success','protocol_complete','phase','error','wall_s','carry_model_sha256')}
        return result
    def batch(tasks):
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures=[pool.submit(trial,*t) for t in tasks]
            for future in concurrent.futures.as_completed(futures):
                r=future.result();report['runs'].append(r);write(a.out/'report.json',report);print(json.dumps(r),flush=True)
    batch([(c,'teacher','development') for c in protocol['development']])
    if not all(r.get('summary',{}).get('physical_success') for r in report['runs']):
        report['blocked']='development teacher failed; no training or final selection performed';write(a.out/'report.json',report);return 1
    data=a.out/'dataset.json'
    cmd=[sys.executable,ROOT/'scripts/build_carry_act_data.py','--train',*protocol['train'],'--development',*[a.out/'development/teacher'/c['id'] for c in protocol['development']],'--out',data]
    row=run(cmd,'dataset');report['dataset_build']=row;write(a.out/'report.json',report)
    if row['exit_code']:return 1
    for seed in protocol['training']['seeds']:
        model=a.out/('model-'+str(seed));row=run([a.act_python,ROOT/'scripts/train_carry_act.py','--dataset',data,'--out',model,'--steps',str(protocol['training']['steps']),'--seed',str(seed)],'train-'+str(seed));report.setdefault('training',[]).append(row);write(a.out/'report.json',report)
        if row['exit_code']:return 1
    freeze={str(seed):hashlib.sha256((a.out/('model-'+str(seed))/'act/model.safetensors').read_bytes()).hexdigest() for seed in protocol['training']['seeds']};write(a.out/'final-freeze.json',{'source_sha':source,'models':freeze,'protocol_sha256':report['protocol_sha256'],'frozen_unix':time.time()})
    tasks=[]
    for c in protocol['test']:
        tasks.append((c,'teacher','final'))
        for seed in protocol['training']['seeds']:tasks.append((c,str(seed),'final',a.out/('model-'+str(seed))/'act'))
    batch(tasks)
    report['complete']=True;write(a.out/'report.json',report);return 0
if __name__=='__main__':raise SystemExit(main())
