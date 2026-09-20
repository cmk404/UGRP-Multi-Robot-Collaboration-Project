"""Run the predeclared eight ACT models on CUDA, resuming trusted checkpoints."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.colab_carry_bundle import PROTOCOL, digest, source_identity, verify_dataset, write


def validate_report(report, protocol, arm, seed, source_sha, directory):
    expected={'source_sha':source_sha,'dataset_sha256':protocol['dataset_sha256'],
              'seed':seed,'steps':protocol['training']['steps'],'batch_size':protocol['training']['batch']}
    if any(report.get(k)!=v for k,v in expected.items()):raise ValueError('cohort report provenance mismatch')
    if report['adapter']['size']!=arm['size'] or report['adapter']['history']!=arm['history']:
        raise ValueError('cohort arm mismatch')
    if report.get('complete') and digest(directory/'act/model.safetensors')!=report['model_sha256']:
        raise ValueError('completed checkpoint changed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--resume',action='store_true')
    p.add_argument('--diagnostic',action='store_true',help='Only 2/8000 steps of the largest arm; not a cohort result')
    a=p.parse_args();protocol=json.loads((ROOT/PROTOCOL).read_text());source_sha=source_identity()
    provenance=verify_dataset(a.dataset)
    if provenance['dataset_sha256']!=protocol['dataset_sha256']:raise ValueError('fixed data mismatch')
    import torch
    if not torch.cuda.is_available():raise RuntimeError('select a Colab GPU runtime first')
    a.out.mkdir(parents=True,exist_ok=a.resume)
    summary={'complete':False,'diagnostic':a.diagnostic,'source_sha':source_sha,'dataset':provenance,'models':[]}
    arms=protocol['arms'][-1:] if a.diagnostic else protocol['arms']
    seeds=protocol['training']['seeds'][:1] if a.diagnostic else protocol['training']['seeds']
    for arm in arms:
        for seed in seeds:
            directory=a.out/f"model-{arm['id']}-s{seed}"
            cmd=[sys.executable,str(ROOT/'scripts/train_carry_input_act.py'),'--dataset',str(a.dataset),
                 '--out',str(directory),'--size',str(arm['size']),'--history',str(arm['history']),
                 '--steps',str(protocol['training']['steps']),'--seed',str(seed),'--device','cuda']
            if directory.exists():
                if not a.resume:raise ValueError('existing model needs --resume')
                report=json.loads((directory/'report.json').read_text())
                validate_report(report,protocol,arm,seed,source_sha,directory)
                if report.get('complete'):
                    summary['models'].append({'directory':directory.name,'model_sha256':report['model_sha256']});continue
                cmd.append('--resume')
            if a.diagnostic:cmd.extend(['--stop-after-step','2'])
            subprocess.run(cmd,check=True,cwd=ROOT)
            report=json.loads((directory/'report.json').read_text())
            validate_report(report,protocol,arm,seed,source_sha,directory)
            summary['models'].append({'directory':directory.name,'complete':report['complete'],
                                      'model_sha256':report.get('model_sha256'),'completed_steps':report['completed_steps']})
            write(a.out/'cohort.json',summary)
    summary['complete']=not a.diagnostic and len(summary['models'])==8
    write(a.out/'cohort.json',summary)

if __name__=='__main__':main()
