"""Run a predeclared ACT protocol on CUDA, resuming trusted checkpoints."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.colab_carry_bundle import PROTOCOL, digest, source_identity, verify_dataset, write


def require_cuda():
    # The orchestration process must not retain another full PyTorch runtime
    # while a 512px trainer is close to the Colab host RAM limit.
    subprocess.run([sys.executable, '-c',
                    'import torch; assert torch.cuda.is_available(), "select a Colab GPU runtime first"'],
                   check=True, timeout=60)


def validate_report(report, protocol, arm, seed, source_sha, directory):
    expected={'source_sha':source_sha,'dataset_sha256':protocol['dataset_sha256'],
              'seed':seed,'steps':protocol['training']['steps'],'batch_size':protocol['training']['batch']}
    if any(report.get(k)!=v for k,v in expected.items()):raise ValueError('cohort report provenance mismatch')
    if report['adapter']['size']!=arm['size'] or report['adapter']['history']!=arm['history']:
        raise ValueError('cohort arm mismatch')
    if report.get('complete') and digest(directory/'act/model.safetensors')!=report['model_sha256']:
        raise ValueError('completed checkpoint changed')
    if report.get('complete') and report.get('completed_steps') != protocol['training']['steps']:
        raise ValueError('completed checkpoint has wrong update count')
    execution = protocol.get('execution', {})
    if report.get('activation_checkpointing', False) != execution.get('checkpoint_encoder', False):
        raise ValueError('activation checkpointing mode differs')
    if report.get('cpu_evaluation_batch_size', 32) != execution.get('cpu_evaluation_batch_size', 32):
        raise ValueError('CPU export batch differs')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--resume',action='store_true')
    p.add_argument('--protocol',type=Path,default=ROOT/PROTOCOL)
    p.add_argument('--diagnostic',action='store_true',help='Only 2/8000 steps of the largest arm; not a cohort result')
    a=p.parse_args();protocol=json.loads(a.protocol.read_text());source_sha=source_identity()
    provenance=verify_dataset(a.dataset)
    if provenance['dataset_sha256']!=protocol['dataset_sha256']:raise ValueError('fixed data mismatch')
    require_cuda()
    a.out.mkdir(parents=True,exist_ok=a.resume)
    expected=len(protocol['arms'])*len(protocol['training']['seeds'])
    summary={'complete':False,'diagnostic':a.diagnostic,'source_sha':source_sha,'dataset':provenance,
             'protocol_sha256':digest(a.protocol),'expected_models':expected,'models':[]}
    arms=protocol['arms'][-1:] if a.diagnostic else protocol['arms']
    seeds=protocol['training']['seeds'][:1] if a.diagnostic else protocol['training']['seeds']
    for arm in arms:
        for seed in seeds:
            directory=a.out/f"model-{arm['id']}-s{seed}"
            cmd=[sys.executable,str(ROOT/'scripts/train_carry_input_act.py'),'--dataset',str(a.dataset),
                 '--out',str(directory),'--size',str(arm['size']),'--history',str(arm['history']),
                 '--steps',str(protocol['training']['steps']),'--seed',str(seed),'--device','cuda']
            execution = protocol.get('execution', {})
            if execution.get('checkpoint_encoder'):
                cmd.append('--checkpoint-encoder')
            if 'cpu_evaluation_batch_size' in execution:
                cmd.extend(['--cpu-evaluation-batch-size', str(execution['cpu_evaluation_batch_size'])])
            if directory.exists():
                if not a.resume:raise ValueError('existing model needs --resume')
                report=json.loads((directory/'report.json').read_text())
                validate_report(report,protocol,arm,seed,source_sha,directory)
                if report.get('complete'):
                    summary['models'].append({'directory':directory.name,'complete':True,
                                              'completed_steps':report['completed_steps'],'model_sha256':report['model_sha256']});continue
                cmd.append('--resume')
            if a.diagnostic:cmd.extend(['--stop-after-step','2'])
            subprocess.run(cmd,check=True,cwd=ROOT)
            report=json.loads((directory/'report.json').read_text())
            validate_report(report,protocol,arm,seed,source_sha,directory)
            summary['models'].append({'directory':directory.name,'complete':report['complete'],
                                      'model_sha256':report.get('model_sha256'),'completed_steps':report['completed_steps']})
            write(a.out/'cohort.json',summary)
    summary['complete']=not a.diagnostic and len(summary['models'])==expected and all(m['complete'] for m in summary['models'])
    write(a.out/'cohort.json',summary)

if __name__=='__main__':main()
