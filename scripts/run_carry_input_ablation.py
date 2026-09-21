"""Owned, fixed-source training and paired full-physics ablation cohort."""
import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import shutil
import os
import signal
import subprocess
import sys
import time
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.carry_failure_metrics import schedule, outcome, aggregate
from scripts.cloud_progress import emit,run_logged


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
    p.add_argument('--dataset', type=Path, help='Hash-verified portable dataset; canonical protocol remains unchanged')
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

    def run(cmd, name, timeout=None):
        frozen(); started=time.monotonic()
        log=a.out/(name+'.log')
        try:
            result=run_logged(cmd,log,name=name,cwd=ROOT,timeout=timeout)
            exit_code,timed_out=result['exit_code'],result['timed_out']
        except OSError as exc:
            log.write_text(f'Process launch failed: {type(exc).__name__}\n')
            exit_code,timed_out=127,False
        frozen()
        return {'name':name,'command':list(map(str,cmd)),'exit_code':exit_code,
                'wall_s':time.monotonic()-started,'log':str(log),'timed_out':timed_out}

    from scripts.colab_carry_bundle import verify_dataset
    data=a.dataset or Path(protocol['dataset'])
    provenance=verify_dataset(data)
    if provenance['dataset_sha256']!=protocol['dataset_sha256']:
        raise ValueError('original dataset changed')
    report['dataset_provenance']=provenance
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

    def trial(job):
        minimum = protocol['controls'].get('min_free_gib', 0)
        available = shutil.disk_usage(a.out).free / 2**30
        if available < minimum:
            return {**job, 'not_attempted': True, 'reason': 'disk_reserve',
                    'free_gib': available, 'min_free_gib': minimum}
        case,condition,repeat=job['case'],job['condition'],job['repeat']
        root_map=provenance.get('relocation_provenance',{}).get('root_map',{})
        original_roots={v:k for k,v in root_map.items()}
        base=next(Path(e['root']) for e in dataset['train']
                  if Path(original_roots.get(e['root'],e['root'])).name==case['teacher_case'])
        output=a.out/'final'/condition/f"{case['id']}-r{repeat}"
        cmd=[a.mjpython,ROOT/'scripts/run_dispatch_e2e.py','--executor','skills','--variant',case['variant'],
             '--seed','11','--required-dock','dock_a','--plan-replay',base/'committed-plan.json',
             '--grasp-model-dir',a.grasp,'--stage-model-dir',a.stages,'--output',output,
             '--max-wall-s',protocol['controls']['max_wall_s'],
             '--carry-max-steps',protocol['controls']['max_carry_steps'],
             '--video-fps','4','--spawn-offset',*case['offset']]
        if condition!='teacher':
            cmd+=['--carry-act-model',models[condition]['path'],'--carry-act-python',a.act_python,
                  '--carry-act-max-steps',protocol['controls']['max_carry_steps']]
        row=run(cmd,'final-'+job['trial_id'],timeout=protocol['controls']['max_wall_s']+60)
        row.update(job,phase='final',output=str(output))
        if (output/'result.json').exists():
            try:
                result=json.loads((output/'result.json').read_text())
                row['summary']={k:result.get(k) for k in ('physical_success','protocol_complete','phase','error','wall_s')}
            except (OSError,ValueError,AttributeError):pass
        row['outcome']=outcome(output,row['exit_code'],row['timed_out'])
        return row

    names=['teacher',*models]
    tasks=schedule(protocol,names)
    report['evaluation_jobs']=tasks
    report['failure_estimates']=aggregate(tasks,[])
    write(a.out/'report.json',report)
    def progress():
        failures=sum(not r['outcome']['whole_success'] for r in report['runs'])
        emit('evaluation_progress',phase='physics128256',planned=len(tasks),attempted=len(report['runs']),
             successes=len(report['runs'])-failures,failures=failures,pending=len(tasks)-len(report['runs']))
    progress()
    with concurrent.futures.ThreadPoolExecutor(max_workers=protocol['controls']['workers']) as pool:
        futures=[pool.submit(trial,task) for task in tasks]
        for future in concurrent.futures.as_completed(futures):
            row=future.result()
            if row.get('not_attempted'):
                report.setdefault('not_attempted', []).append(row)
                write(a.out/'report.json', report)
                emit('evaluation_not_attempted', trial_id=row['trial_id'], phase=row['reason'])
                continue
            report['runs'].append(row)
            report['failure_estimates']=aggregate(tasks,report['runs']);write(a.out/'report.json',report)
            progress()
            print(json.dumps({'event':'trial','case':row['case']['id'],'condition':row['condition'],'summary':row.get('summary'),'exit_code':row['exit_code']}),flush=True)
    report['complete']=report['failure_estimates']['complete'];write(a.out/'report.json',report)
    return 0 if report['complete'] else 2


if __name__=='__main__':
    raise SystemExit(main())
