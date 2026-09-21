"""Finite GPU-only ACT recovery: calibrate RGB, validate handoff, then fixed cohort.

No model credentials or online teacher correction. All resets and truth labels
remain inside the offline calibration program. 512px is deliberately excluded.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.cloud_collection import digest,write_json
from scripts.colab_carry_bundle import unpack
from scripts.setup_kaggle_egl import configure


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--input-root',type=Path,default=Path('/kaggle/input'))
    p.add_argument('--data-sha',required=True);p.add_argument('--assets-sha',required=True)
    a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    state={'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
           'complete':False,'phase':'setup','scope':'RGB recalibration diagnostics followed by fixed 128/256 evaluation; no 512px'}
    def save():write_json(out/'recovery-status.json',state)
    def run(command,name,timeout):
        with (out/(name+'.log')).open('w') as stream:
            return subprocess.run(list(map(str,command)),cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=timeout).returncode
    env={**os.environ,'MUJOCO_GL':'egl','PYOPENGL_PLATFORM':'egl','OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','PYTHONPATH':str(ROOT)}
    save()
    try:
        gpu,render_environment=configure(sys.executable,out/'renderer')
        env.update(render_environment)
        state['renderer']=gpu;save()
        def asset(name,sha):
            paths=list(a.input_root.rglob(name))
            if len(paths)!=1 or digest(paths[0])!=sha:raise ValueError('input archive missing or changed: '+name)
            return paths[0]
        data=unpack(asset('carry-data.bin',a.data_sha),ROOT/'outputs/recovered-data',a.data_sha)
        assets=ROOT/'outputs/recovered-models';assets.mkdir(parents=True)
        with zipfile.ZipFile(asset('physics-assets.bin',a.assets_sha)) as bundle:
            for name in bundle.namelist():
                path=Path(name)
                if path.is_absolute() or '..' in path.parts:raise ValueError('unsafe model member')
            bundle.extractall(assets)
        act=ROOT.parent/'act-env'
        subprocess.run([sys.executable,'-m','pip','install','uv'],check=True)
        subprocess.run([sys.executable,'-m','uv','venv','--seed','--python',sys.executable,str(act)],check=True)
        actpy=act/'bin/python'
        if run([actpy,'-m','pip','install','-r',ROOT/'requirements-reference-act.txt'],'act-install',1500):raise RuntimeError('ACT dependencies failed')
        if run([actpy,ROOT/'scripts/patch_reference_act.py'],'act-patch',60):raise RuntimeError('ACT compatibility patch failed')
        state['phase']='approach_calibration';save()
        calibration=out/'approach-calibration'
        if run([sys.executable,'-m','scripts.train_dispatch_transfer','--output',calibration,'--base-grasp',assets/'grasp','--approach-only'],'approach-calibration',1800):raise RuntimeError('RGB calibration failed')
        stages=calibration/'models/varied';grasp=assets/'grasp'
        plan=Path(data).parent/'episodes/train-0/committed-plan.json'
        # Locate the relocated training episode using the verified dataset itself.
        dataset=json.loads(Path(data).read_text());plan=Path(dataset['train'][0]['root'])/'committed-plan.json'
        def diagnostic(name):
            output=out/name
            code=run([sys.executable,'-m','scripts.run_dispatch_e2e','--executor','skills','--variant','open','--seed','11','--required-dock','dock_a',
                '--plan-replay',plan,'--grasp-model-dir',grasp,'--stage-model-dir',stages,'--output',output,'--max-wall-s','1500','--video-fps','4','--spawn-offset','-.008','.003','0'],name,1560)
            result=json.loads((output/'result.json').read_text())
            state[name]={'exit_code':code,'physical_success':result.get('evaluation',{}).get('physical_success'),
                'protocol_complete':result.get('protocol_complete'),'error':result.get('error')};save()
            return bool(result.get('protocol_complete') and result.get('evaluation',{}).get('physical_success'))
        state['phase']='handoff_diagnostic';save()
        if not diagnostic('diagnostic-approach'):
            state['phase']='grasp_calibration';save()
            transfer=out/'grasp-calibration'
            if run([sys.executable,'-m','scripts.train_dispatch_transfer','--output',transfer,'--base-grasp',assets/'grasp','--approach-models',stages],'grasp-calibration',3600):raise RuntimeError('grasp calibration failed')
            grasp=transfer/'models/grasp';stages=transfer/'models/varied'
            if not diagnostic('diagnostic-grasp'):raise RuntimeError('handoff remains invalid; full ACT cohort not started')
        state['phase']='physics128256';save()
        code=run([sys.executable,'-m','scripts.run_carry_input_ablation','--out',out/'physics128256','--act-python',actpy,'--mjpython',sys.executable,
            '--grasp',grasp,'--stages',stages,'--reuse-training',assets/'training','--dataset',data],'physics128256',7200)
        state.update(complete=code==0,exit_code=code)
        return code
    except Exception as exc:
        state.update(error_type=type(exc).__name__,error=str(exc),exit_code=1)
        raise
    finally:save()

if __name__=='__main__':raise SystemExit(main())
