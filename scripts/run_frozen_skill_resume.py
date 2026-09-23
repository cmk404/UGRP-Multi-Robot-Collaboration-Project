"""Finite remaining-trial runner around an unchanged, committed actor checkout.

The recovery control script has its own recorded hash. Existing trial evidence
stays on the collecting host; no completed success OR failure is rerun.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def remaining_protocol(protocol, completed):
    jobs = protocol['jobs']
    by_id = {job['trial_id']: job for job in jobs}
    if len(by_id) != len(jobs) or len(set(completed)) != len(completed):
        raise ValueError('duplicate trial identity')
    if not set(completed) <= set(by_id):
        raise ValueError('unknown completed trial')
    return {**protocol, 'jobs': [job for job in jobs if job['trial_id'] not in completed],
            'resume': {'completed_trial_ids': list(completed), 'original_planned': len(jobs)}}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--manifest-sha', required=True)
    p.add_argument('--seconds', type=int, default=12600)
    a = p.parse_args()
    if not 60 <= a.seconds <= 14400: p.error('invalid finite deadline')
    raw = a.manifest.read_bytes()
    if hashlib.sha256(raw).hexdigest() != a.manifest_sha: raise ValueError('resume manifest changed')
    manifest = json.loads(raw)
    root = a.base/'source'; py = a.base/'sim-env/bin/python'
    source = subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
    if source != manifest['actor_source_sha']: raise ValueError('actor source changed')
    if subprocess.check_output(['git','status','--porcelain'],cwd=root,text=True).strip(): raise ValueError('dirty actor source')
    state_path = a.base/'state-model-v4.json'
    state = {'source_sha': source, 'started_unix': time.time(), 'complete': False,
             'phase': None, 'exports': {}, 'resume_manifest_sha256': a.manifest_sha,
             'control_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    def save():
        temp = state_path.with_suffix('.tmp'); temp.write_text(json.dumps(state,indent=2)+'\n'); temp.replace(state_path)
    env = {**os.environ, 'MUJOCO_GL':'egl', 'PYOPENGL_PLATFORM':'egl', 'PYTHONPATH':str(root),
           'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1',
           '__EGL_VENDOR_LIBRARY_FILENAMES':'/usr/share/glvnd/egl_vendor.d/10_nvidia.json'}
    deadline = time.monotonic() + a.seconds
    save()
    try:
        for phase in ('development','holdout','regression','ablation','continuous'):
            original = manifest['phases'][phase]['protocol']
            if original['source_sha'] != source or original['phase'] != phase: raise ValueError('protocol identity mismatch')
            protocol = remaining_protocol(original, manifest['phases'][phase]['completed'])
            protocol['model_mailbox'] = str(a.base/'mailbox')
            out = root/'outputs'/('gpu-'+phase)/'result'; out.mkdir(parents=True,exist_ok=False)
            (out/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
            state['phase'] = phase; save()
            def worker(index):
                seconds = int(deadline-time.monotonic())
                if seconds <= 0: return 124
                with (out/f'worker-{index}.log').open('w') as log:
                    proc = subprocess.run(['timeout','--signal=INT','--kill-after=15',str(seconds),str(py),
                        str(root/'scripts/run_jev_skill_cohort.py'),'--output',str(out),'--worker',str(index)],
                        input=json.dumps({'key':None}),text=True,stdout=log,stderr=subprocess.STDOUT,cwd=root,env=env)
                return proc.returncode
            with concurrent.futures.ThreadPoolExecutor(max_workers=protocol['workers']) as pool:
                codes = list(pool.map(worker,range(protocol['workers']))) if protocol['jobs'] else []
            results = [json.loads((out/job['trial_id']/'result.json').read_text()) for job in protocol['jobs']
                       if (out/job['trial_id']/'result.json').exists()]
            (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
            (out/'worker-exit-codes.json').write_text(json.dumps(codes)+'\n')
            state['exports'][phase] = {'new_completed': len(results), 'new_planned': len(protocol['jobs']),
                                       'previous_completed': len(protocol['resume']['completed_trial_ids'])}
            state['phase_exit_code'] = int(any(codes) or len(results)!=len(protocol['jobs'])); save()
            if state['phase_exit_code']:
                state['stop_reason'] = 'phase_process_failed'; break
    except Exception as exc:
        state['error_type'] = type(exc).__name__
        raise
    finally:
        state.update(complete=True,ended_unix=time.time()); save()


if __name__ == '__main__': main()
