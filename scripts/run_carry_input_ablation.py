"""Owned, fixed-source training and paired full-physics ablation cohort."""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
ROOT = Path(__file__).resolve().parents[1]


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write(p, value):
    p.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def validate_training(root, arm, seed, protocol):
    """Reject a complete but wrong-arm/seed/budget checkpoint before physics."""
    report = json.loads((root/'report.json').read_text())
    expected = {'complete': True, 'dataset_sha256': protocol['dataset_sha256'],
                'seed': seed, 'steps': protocol['training']['steps'],
                'batch_size': protocol['training']['batch']}
    for key, value in expected.items():
        if report.get(key) != value:
            raise ValueError(f'training {key} mismatch: {root}')
    adapter = json.loads((root/'act/adapter.json').read_text())
    if (adapter != report.get('adapter') or adapter.get('kind') != 'carry_input_ablation'
            or adapter.get('size') != arm['size'] or adapter.get('history') != arm['history']):
        raise ValueError(f'training arm mismatch: {root}')
    if sha(root/'act/model.safetensors') != report.get('model_sha256'):
        raise ValueError(f'training model hash mismatch: {root}')
    if (not report.get('initial_cache_verification')
            or set(report.get('selected_cache_verification', {})) != {'train', 'development'}):
        raise ValueError(f'missing native/cache validation: {root}')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--act-python', type=Path, required=True)
    p.add_argument('--mjpython', type=Path, required=True)
    p.add_argument('--grasp', type=Path, required=True)
    p.add_argument('--stages', type=Path, required=True)
    p.add_argument('--training-only', action='store_true')
    p.add_argument('--reuse-training', type=Path)
    p.add_argument('--protocol', type=Path, default=ROOT/'experiments/2026-09-21-carry-input-ablation/protocol.json')
    a = p.parse_args(); a.out = a.out.resolve(); a.out.mkdir(parents=True, exist_ok=False)
    protocol_path = a.protocol
    protocol = json.loads(protocol_path.read_text())
    source = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    report = {'complete':False, 'source_sha':source, 'protocol_sha256':sha(protocol_path),
              'protocol':protocol,'training':[], 'runs':[], 'external_model_calls':0}
    write(a.out/'report.json', report)

    def frozen():
        if subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()!=source or subprocess.check_output(['git','status','--porcelain'],cwd=ROOT):
            raise RuntimeError('cohort source changed')

    def run(cmd, name):
        frozen(); started=time.monotonic()
        log=a.out/(name+'.log')
        with log.open('w') as stream:
            proc=subprocess.run(list(map(str,cmd)),cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
        frozen()
        return {'name':name,'command':list(map(str,cmd)),'exit_code':proc.returncode,
                'wall_s':time.monotonic()-started,'log':str(log)}

    data=Path(protocol['dataset'])
    if sha(data)!=protocol['dataset_sha256']:
        raise ValueError('original dataset changed')
    shutil.copyfile(data,a.out/'dataset.json')
    models={}
    for seed in protocol['training']['seeds']:
        for arm in protocol['arms']:
            name=f"{arm['id']}-s{seed}"
            output=a.out/('model-'+name)
            if a.reuse_training:
                origin=a.reuse_training/('model-'+name)
                old=validate_training(origin,arm,seed,protocol)
                # Keep raw provenance without duplicating large checkpoints.
                output.symlink_to(origin.resolve(),target_is_directory=True)
                row={'name':name,'reused':str(origin),'training_source_sha':old['source_sha']}
            else:
                row=run([a.act_python,ROOT/'scripts/train_carry_input_act.py','--dataset',a.out/'dataset.json',
                         '--out',output,'--size',arm['size'],'--history',arm['history'],
                         '--seed',seed,'--steps',protocol['training']['steps']], 'train-'+name)
                if row['exit_code']:
                    report['training'].append(row);report['blocked']='training process failed';write(a.out/'report.json',report);return 1
            mr=validate_training(output,arm,seed,protocol)
            models[name]={'path':str(output/'act'),'sha256':sha(output/'act/model.safetensors'),
                          'source_sha':mr['source_sha'],'arm':arm,'seed':seed}
            row['model']=models[name];report['training'].append(row);write(a.out/'report.json',report)
            print(json.dumps({'event':'trained','name':name,'wall_s':mr['wall_s'],'development':mr['development_metrics']}),flush=True)
    write(a.out/'final-freeze.json',{'source_sha':source,'protocol_sha256':sha(protocol_path),'models':models,'frozen_unix':time.time()})
    if a.training_only:
        report['training_complete']=True;write(a.out/'report.json',report);return 0
    dataset=json.loads(data.read_text())

    def trial(case, condition):
        base=next(Path(e['root']) for e in dataset['train'] if Path(e['root']).name==case['teacher_case'])
        output=a.out/'final'/condition/case['id']
        cmd=[a.mjpython,ROOT/'scripts/run_dispatch_e2e.py','--executor','skills','--variant',case['variant'],
             '--seed','11','--required-dock','dock_a','--plan-replay',base/'committed-plan.json',
             '--grasp-model-dir',a.grasp,'--stage-model-dir',a.stages,'--output',output,
             '--max-wall-s',protocol['controls']['max_wall_s'],'--video-fps','4','--spawn-offset',*case['offset']]
        if condition!='teacher':
            cmd+=['--carry-act-model',models[condition]['path'],'--carry-act-python',a.act_python,
                  '--carry-act-max-steps',protocol['controls']['max_carry_steps']]
        row=run(cmd,'final-'+condition+'-'+case['id'])
        row.update(phase='final',condition=condition,case=case,output=str(output))
        if (output/'result.json').exists():
            result=json.loads((output/'result.json').read_text())
            row['summary']={k:result.get(k) for k in ('physical_success','protocol_complete','phase','error','wall_s')}
        return row

    tasks=[]
    names=['teacher',*models]
    for i,case in enumerate(protocol['test']):
        order=names[i:]+names[:i]
        tasks.extend((case,name) for name in order)
    with concurrent.futures.ThreadPoolExecutor(max_workers=protocol['controls']['workers']) as pool:
        futures=[pool.submit(trial,*task) for task in tasks]
        for future in concurrent.futures.as_completed(futures):
            row=future.result();report['runs'].append(row);write(a.out/'report.json',report)
            print(json.dumps({'event':'trial','case':row['case']['id'],'condition':row['condition'],'summary':row.get('summary'),'exit_code':row['exit_code']}),flush=True)
    report['complete']=True;write(a.out/'report.json',report)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
