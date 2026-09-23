"""Finite teacher -> expanded-data ACT training -> matched repeated evaluation.

No live referee value is sent to an actor. Completed teacher verdicts admit
offline demonstrations only. Every declared evaluation runs despite failures.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.run_matched_carry_cohort import sha,write
from scripts.cloud_progress import run_logged
from scripts.build_carry_act_data import extract
from scripts.run_carry_input_ablation import validate_training


def validate_split_cases(protocol):
    def fingerprint(c):return (c['variant'],c['seed'],c['dock'],tuple(c['offset']))
    train={fingerprint(c) for c in [*protocol['teacher_cases'],protocol['reuse_parallel_case']]}
    test=[fingerprint(c) for c in protocol['test_cases']]
    if len(set(test))!=len(test) or train.intersection(test):raise ValueError('teacher/test physical-case overlap')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('protocol','out','mjpython','act-python','tensorboard-dir'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();cfg=json.loads(a.protocol.read_text());validate_split_cases(cfg)
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    def frozen():
        if (subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()!=source
                or subprocess.check_output(['git','status','--porcelain'],cwd=ROOT)):
            raise RuntimeError('study source changed')
        for path,digest in cfg['asset_sha256'].items():
            if sha(path)!=digest:raise ValueError('study asset changed: '+path)
    frozen();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
    report={'complete':False,'source_sha':source,'protocol_sha256':sha(a.protocol),'protocol':cfg,'stages':[]}
    def save():write(a.out/'study.json',report)
    def stage(name,cmd,timeout=None):
        frozen()
        if shutil.disk_usage(a.out).free/2**30<cfg['controls']['min_free_gib']:raise RuntimeError('disk reserve; next stage not attempted')
        report['active_stage']=name;save();print(json.dumps({'stage':name,'state':'running'}),flush=True)
        row=run_logged(cmd,a.out/(name+'.log'),name=name,cwd=ROOT,timeout=timeout)
        report['stages'].append({'name':name,**row,'command':list(map(str,cmd))});save();frozen()
        if row['exit_code']!=0 or row['timed_out']:raise RuntimeError(name+' process incomplete; recorded child evidence retained')
    def cohort(name,protocol):
        path=a.out/(name+'-protocol.json');write(path,protocol)
        stage(name,[sys.executable,ROOT/'scripts/run_matched_carry_cohort.py','--protocol',path,
                    '--out',a.out/name,'--mjpython',a.mjpython,'--act-python',a.act_python,
                    '--tensorboard-dir',a.tensorboard_dir/name])
        return json.loads((a.out/name/'report.json').read_text())
    try:
        base={k:copy.deepcopy(cfg[k]) for k in ('controls','grasp','stages','asset_sha256')}
        teachers=cohort('teachers',{**base,'scope':'offline RGB demonstrations for parallel visual distribution',
                                  'conditions':{'RGB':{}},'test':cfg['teacher_cases']})
        # All teacher attempts complete before admission; never stop on the first failure.
        if any(not row['outcome']['whole_success'] for row in teachers['runs']):
            raise RuntimeError('teacher admission failed after all declared teacher trials; no failed demo admitted')
        dataset=json.loads(Path(cfg['base_dataset']).read_text())
        old_counts={split:len(dataset[split]) for split in ('train','development')}
        reused=Path(cfg['reuse_parallel_train']);demo=extract(reused)
        reuse_result=json.loads((reused/'result.json').read_text())
        if (reuse_result['source_sha']!=cfg['reuse_parallel_source_sha']
                or reuse_result['evaluation']['robot_robot_contact_samples']!=0
                or reuse_result['evaluation']['concurrent_transport']['simultaneous_loaded_motion_s']<=0):
            raise ValueError('reused parallel demonstration does not match declared admission')
        dataset['train'].append(demo)
        for row in teachers['runs']:
            case=next(c for c in cfg['teacher_cases'] if c['id']==row['case']['id'])
            result=json.loads((Path(row['output'])/'result.json').read_text())
            if result['evaluation']['concurrent_transport']['simultaneous_loaded_motion_s']<=0:
                raise ValueError('demonstration did not contain actual concurrent transport')
            dataset[case['split']].append(extract(Path(row['output'])))
        if {e['root'] for e in dataset['train']} & {e['root'] for e in dataset['development']}:
            raise ValueError('dataset split overlap')
        data=a.out/'expanded-dataset.json';write(data,dataset)
        report['dataset']={'sha256':sha(data),'old_episode_counts':old_counts,
                           'episode_counts':{s:len(dataset[s]) for s in old_counts},
                           'input_boundary':'own/top RGB + static context + issued commands; teacher verdicts are offline admission only'};save()
        t=cfg['training'];model=a.out/'model-expanded'
        stage('training',[a.act_python,ROOT/'scripts/train_carry_input_act.py','--dataset',data,'--out',model,
              '--size',t['size'],'--history',t['history'],'--seed',t['seed'],'--steps',t['steps'],'--device','cpu'],
              timeout=t['timeout_s'])
        validation=validate_training(model,t,t['seed'],{'dataset_sha256':sha(data),'training':{'steps':t['steps'],'batch':32}})
        report['training']={'model_sha256':validation['model_sha256'],'selected':validation['selected'],
                            'development_metrics':validation['development_metrics']};save()
        from scripts.tensorboard_tools.export import convert
        exported=convert(model,a.tensorboard_dir/'training',max_images=0,media_port=6009)
        report['training_export']={'complete':exported['complete'],'warnings':exported['warnings']};save()
        assets=copy.deepcopy(cfg['asset_sha256'])
        for f in (model/'act').iterdir():
            if f.is_file():assets[str(f)]=sha(f)
        evaluated=cohort('evaluation',{**base,'asset_sha256':assets,'scope':cfg['interpretation'],
              'conditions':{'RGB':{},'ACT-before':{'model':cfg['baseline_model']},'ACT-expanded':{'model':str(model/'act')}},
              'test':cfg['test_cases']})
        report['failure_estimates']=evaluated['failure_estimates'];report['complete']=evaluated['complete']
        report['active_stage']=None;save()
    except Exception as error:
        report['blocked']=type(error).__name__+': '+str(error);save();raise
    return 0 if report['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
