"""Fixed-source endurance, terrain and recovery cohort inside an owned session."""
from pathlib import Path
import argparse,hashlib,json,subprocess,time
ROOT=Path(__file__).resolve().parents[2]
def write(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()
def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('mjpython','grasp-model-dir','out-dir'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();out=a.out_dir.resolve();models=a.grasp_model_dir.resolve()
    if git('status','--porcelain'):raise RuntimeError('commit complete source and protocol first')
    out.mkdir(parents=True,exist_ok=False);sha=git('rev-parse','HEAD')
    cases=[{'id':mode,'mode':mode,'duration':300,'profile':'retention','expected':'continuous_hold'} for mode in ('shuttle','stationary')]
    catalog=json.loads((ROOT/'maps/pair_navigation/catalog.json').read_text())
    cases += [{'id':e['id'],'map':e['map'],'expected':'delivery' if e['expected_geometric_route'] else 'no_route'} for e in catalog['entries']]
    cases += [{'id':'baseline-recovery','mode':'stationary','duration':60,'profile':'baseline','expected':'one_regrasp_then_place_stop'}]
    record={'source_sha':sha,'cases':cases,'trials':[],'model_files':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(models.iterdir()) if f.is_file()},'external_model_calls':0,'cost_usd':0}
    write(out/'cohort.json',record)
    for case in cases:
        if git('rev-parse','HEAD')!=sha or git('status','--porcelain'):raise RuntimeError('source changed during cohort')
        endurance='mode' in case
        command=[str(a.mjpython.resolve()),str(ROOT/'scripts'/('run_pair_grasp_endurance.py' if endurance else 'run_pair_navigation.py')),
            '--map',str(ROOT/'maps/pair_navigation'/case.get('map','narrow-door.json')),'--grasp-model-dir',str(models),'--out-dir',str(out/case['id'])]
        command += (['--mode',case['mode'],'--duration',str(case['duration']),'--contact-profile',case['profile']] if endurance else
            ['--budget','750','--impratio','10','--vision-mode','robust','--grasp-spacing','visual','--close-pulse','1600','--contact-profile','retention'])
        trial={'id':case['id'],'expected':case['expected'],'command':command};started=time.monotonic()
        print('START '+case['id'],flush=True)
        with (out/(case['id']+'.log')).open('w') as log:
            try:trial['returncode']=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=1200).returncode
            except subprocess.TimeoutExpired:
                trial.update(timeout=True,wall_seconds=time.monotonic()-started);record['trials'].append(trial);write(out/'cohort.json',record);raise
        trial['wall_seconds']=time.monotonic()-started
        result=out/case['id']/'result.json'
        if result.exists():
            r=json.loads(result.read_text())
            trial.update(success=r['success'],error=r['error'],refused_no_route=r.get('refused_no_route',False),evaluation=r.get('evaluation'),recovery_evaluation=r.get('recovery_evaluation'))
            if case['expected']=='no_route':passed=not r['error'] and trial['refused_no_route'] and r['grasp_stability']['success']
            elif case['expected']=='one_regrasp_then_place_stop':passed=bool(r['recovery_evaluation']['success'] and len(r['recoveries'])==2 and 'placed and stopped' in (r['error'] or '') and not r['success'])
            else:passed=r['success'] and not r.get('recoveries')
            trial['expected_outcome_passed']=bool(passed)
        else:trial.update(infrastructure_error='result.json missing',expected_outcome_passed=False)
        record['trials'].append(trial);write(out/'cohort.json',record)
        print('END '+case['id']+' '+json.dumps({k:trial[k] for k in ('returncode','wall_seconds','expected_outcome_passed')}),flush=True)
        if not trial['expected_outcome_passed']:raise RuntimeError('candidate failed; preserve results and diagnose before continuing')
    record['complete']=True;write(out/'cohort.json',record);print('COHORT COMPLETE',flush=True)
if __name__=='__main__':main()
