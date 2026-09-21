"""Import/startup, package and artifact checks only; never runs a simulator."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.cloud_collection import write_json
from scripts.cloud_progress import emit,run_logged


def probe_python(python, modules, *, env=None):
    code='import importlib,json; print(json.dumps({name: bool(importlib.import_module(name)) for name in '+repr(modules)+'}))'
    process=subprocess.run([str(python),'-c',code],env=env,capture_output=True,text=True,timeout=60)
    startup_error='Error in sitecustomize' in process.stderr or 'ModuleNotFoundError' in process.stderr
    return {'ready':process.returncode==0 and not startup_error,'exit_code':process.returncode,
            'startup_error':startup_error,'modules':modules,
            'stdout':process.stdout[-4000:],'stderr':process.stderr[-4000:]}


def inspect_environment(*, python=sys.executable, require_gpu=False, act_python=None):
    env={**os.environ,'PYTHONPATH':str(ROOT),'PYTHONUNBUFFERED':'1'}
    modules=['numpy','cv2','mujoco','PIL','OpenGL']
    if Path('/kaggle').is_dir():modules.append('wrapt')
    checks={'sim_imports':probe_python(python,modules,env=env)}
    packages=subprocess.run([str(python),'-m','pip','check'],env=env,capture_output=True,text=True,timeout=60)
    checks['sim_packages']={'ready':packages.returncode==0,'exit_code':packages.returncode,
                            'detail':(packages.stdout+packages.stderr)[-4000:]}
    if act_python:
        checks['act_imports']=probe_python(act_python,['torch','torchvision','lerobot.policies.act.modeling_act'],env=env)
    if require_gpu:
        from scripts.probe_kaggle_gpu import inventory
        device=inventory(env=env)
        checks['gpu_assignment']={'ready':device.get('nvidia_smi_exit')==0 and bool(device.get('gpus','').strip()),'inventory':device}
    return {'scope':'Preflight imports/packages/device inventory only; no simulation or training',
            'ready':all(c['ready'] for c in checks.values()),'checks':checks,'checked_unix':time.time()}


def require_ready(report):
    if report.get('ready') is not True:raise RuntimeError('environment preflight failed; simulation not started')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--require-gpu',action='store_true')
    p.add_argument('--act-python',type=Path)
    p.add_argument('--with-act',action='store_true',help='Install the pinned ACT environment for import checks only')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    deadline=time.monotonic()+720
    if a.with_act:
        if not Path('/kaggle').is_dir():raise ValueError('ACT preflight installation is restricted to the disposable Kaggle job')
        # Keep installed packages outside the run artifact tree; archive only evidence.
        act=ROOT.parent/'preflight-act-env';a.act_python=act/'bin/python'
        for name,command in [('install-uv',[sys.executable,'-m','pip','install','uv']),
                             ('create-act',[sys.executable,'-m','uv','venv','--seed','--python',sys.executable,str(act)]),
                             ('install-act',[str(a.act_python),'-m','pip','install','-r',str(ROOT/'requirements-reference-act.txt'),'wrapt==2.2.2']),
                             ('patch-act',[str(a.act_python),str(ROOT/'scripts/patch_reference_act.py')])]:
            remaining=deadline-time.monotonic()
            if remaining<=0:
                write_json(a.output/'preflight.json',{'ready':False,'phase':name,'reason':'installation_deadline'})
                return 1
            step=run_logged(command,a.output/(name+'.log'),name=name,timeout=min(600,remaining))
            if step['exit_code'] or step['timed_out']:
                write_json(a.output/'preflight.json',{'ready':False,'phase':name,'step':step})
                return 1
    report=inspect_environment(require_gpu=a.require_gpu,act_python=a.act_python)
    write_json(a.output/'preflight.json',report)
    emit('preflight',ready=report['ready'],checks={name:c['ready'] for name,c in report['checks'].items()})
    # Exercise the same log path used by real work, with no robot work.
    emit('preflight_finished',ready=report['ready'])
    return 0 if report['ready'] else 1


if __name__=='__main__':raise SystemExit(main())
