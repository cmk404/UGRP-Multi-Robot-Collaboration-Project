"""Finite serial Mac cohort: isolated trials, first failures preserved, no trial retries."""
from __future__ import annotations

import argparse
import importlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.cloud_collection import checkpoint_trial, verify_checkpoint, write_json
from scripts.cloud_progress import emit, run_logged


def source_sha():
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip():
        raise RuntimeError('source must remain committed and clean')
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()


def worker(output, index):
    from types import SimpleNamespace
    from scripts.run_jev_skill_motion import run_trial
    protocol = json.loads((output/'protocol.json').read_text())
    if source_sha() != protocol['source_sha']:
        raise RuntimeError('frozen source changed')
    job = protocol['jobs'][index]
    key = json.load(sys.stdin)['key'] if job['policy'] == 'jev' else None
    args = SimpleNamespace(**job, **protocol['limits'], output=output/job['trial_id'],
                           gemini_url=protocol['gemini_url'], model_mailbox=None)
    result = run_trial(args, protocol['case_setup'][job['case']], job['policy'], key,
                       protocol['source_sha'])
    result.update(repeat=job['repeat'], trial_id=job['trial_id'], partition=protocol['phase'])
    write_json(args.output/'result.json', result)


def summary(protocol, rows, started, active=None, stop=None):
    counts = {policy: {'attempted': 0, 'successes': 0, 'task_failures': 0,
                      'infrastructure_failures': 0} for policy in ('rule', 'jev', 'gemini')}
    for row in rows:
        c = counts[row['policy']]
        c['attempted'] += 1
        c['successes' if row['success'] else row['failure_kind']] += 1
    return {'source_sha': protocol['source_sha'], 'started_unix': started,
            'updated_unix': time.time(), 'deadline_unix': started+protocol['budget_s'],
            'planned': len(protocol['jobs']), 'attempted': len(rows),
            'pending': len(protocol['jobs'])-len(rows), 'active_trial': active,
            'complete': len(rows) == len(protocol['jobs']), 'stop_reason': stop,
            'by_policy': counts, 'trials': rows}


def evaluate(output, protocol, key, mjpython, *, launch=run_logged,
             frozen=source_sha, clock=time.monotonic):
    started = time.time()
    deadline = clock()+protocol['budget_s']
    rows = []
    stop = None
    env = {k: v for k, v in os.environ.items()
           if k not in ('MUJOCO_GL', 'PYOPENGL_PLATFORM', '__EGL_VENDOR_LIBRARY_FILENAMES')}
    for index, job in enumerate(protocol['jobs']):
        if clock() >= deadline:
            stop = 'cohort_deadline'
            break
        if frozen() != protocol['source_sha']:
            stop = 'source_changed'
            break
        trial = output/job['trial_id']
        if trial.exists():
            raise FileExistsError('never overwrite or retry an existing trial')
        write_json(output/'status.json', summary(protocol, rows, started, job['trial_id']))
        emit('trial_start', trial_id=job['trial_id'], planned=len(protocol['jobs']),
             attempted=len(rows), pending=len(protocol['jobs'])-len(rows))
        command = [mjpython, str(Path(__file__).resolve()), '--output', str(output),
                   '--worker', str(index)]
        try:
            process = launch(command, output/(job['trial_id']+'.log'), name=job['trial_id'],
                             cwd=ROOT, env=env,
                             timeout=min(protocol['limits']['max_wall_s']+45, deadline-clock()),
                             stdin_text=json.dumps({'key': key}) if job['policy'] == 'jev' else None)
        except Exception as exc:
            # Do not serialize exception text: subprocess errors can contain private inputs.
            process = {'exit_code': None, 'timed_out': False, 'error_type': type(exc).__name__}
        trial.mkdir(exist_ok=True)
        result_path = trial/'result.json'
        raw_valid = False
        if result_path.exists():
            try:
                result = json.loads(result_path.read_text())
                raw_valid = (result.get('source_sha') == protocol['source_sha']
                             and result.get('policy') == job['policy']
                             and result.get('case') == job['case']
                             and isinstance(result.get('success'), bool))
            except (ValueError, TypeError):
                pass
        if not raw_valid:
            if result_path.exists():
                result_path.rename(trial/'invalid-result.raw')
            result = {'source_sha': protocol['source_sha'], **job, 'success': False,
                      'evaluation_available': False, 'stop_reason': 'worker_incomplete',
                      'error': {'type': 'WorkerIncomplete'}, 'partition': protocol['phase']}
            write_json(result_path, result)
        infrastructure = (not raw_valid or process.get('exit_code') != 0
                          or process.get('timed_out') or bool(result.get('error')))
        row = {**job, 'success': result['success'] and not infrastructure,
               'failure_kind': 'infrastructure_failures' if infrastructure else 'task_failures',
               'stop_reason': result.get('stop_reason'), 'process': process,
               'result_path': str(result_path), 'evaluation_available': raw_valid}
        write_json(trial/'attempt.json', row)
        manifest = checkpoint_trial(trial, output/'checkpoints', protocol['source_sha'])
        verify_checkpoint(output/'checkpoints'/manifest['archive'], manifest, protocol['source_sha'])
        row['checkpoint'] = manifest
        rows.append(row)
        report = summary(protocol, rows, started)
        write_json(output/'status.json', report)
        emit('trial_complete', trial_id=job['trial_id'], planned=report['planned'],
             attempted=report['attempted'], pending=report['pending'],
             successes=sum(r['success'] for r in rows), failures=sum(not r['success'] for r in rows))
    report = summary(protocol, rows, started, stop=stop or 'all_trials_attempted')
    write_json(output/'status.json', report)
    write_json(output/'report.json', report)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--worker', type=int)
    p.add_argument('--mjpython', type=Path)
    p.add_argument('--keychain-helper', type=Path)
    p.add_argument('--gemini-url', default='http://127.0.0.1:8391/v1/chat/completions')
    p.add_argument('--budget-s', type=float, default=14400)
    p.add_argument('--execute', action='store_true')
    a = p.parse_args()
    a.output = a.output.resolve()
    if a.worker is not None:
        return worker(a.output, a.worker)
    if not a.execute or not a.mjpython or not a.keychain_helper:
        p.error('--execute, --mjpython and --keychain-helper required')
    if platform.system() != 'Darwin' or not 0 < a.budget_s <= 14400:
        p.error('native Mac and a finite budget up to four hours required')
    source = source_sha()
    a.output.mkdir(parents=True, exist_ok=False)
    from scripts.model_connectivity_preflight import check, validate_report
    from scripts.run_jev_skill_cohort import plan
    key = subprocess.check_output([str(a.keychain_helper.resolve())],
                                  stderr=subprocess.DEVNULL, timeout=15).decode().strip()
    if not key:
        raise ValueError('missing Jev credential')
    preflight = check(key, a.gemini_url)
    write_json(a.output/'model-preflight.json', preflight)
    if not validate_report(preflight):
        raise RuntimeError('model preflight failed; simulation not started')
    jobs, cases = plan('holdout')
    protocol = {'source_sha': source, 'phase': 'holdout', 'jobs': jobs, 'case_setup': cases,
                'workers': 1, 'budget_s': a.budget_s, 'gemini_url': a.gemini_url,
                'limits': {'max_sim_s': 90., 'max_wall_s': 1200., 'max_calls': 400,
                           'max_input_tokens': 1000000, 'clock': 'paused'},
                'hardware': {'system': platform.platform(), 'machine': platform.machine(),
                             'python': sys.version, 'mjpython': str(a.mjpython.resolve()),
                             'packages': {name: importlib.import_module(name).__version__
                                          for name in ('mujoco', 'numpy', 'cv2')}},
                'repeats': 'identical fixed layouts; fresh API responses, not independent maps',
                'failures': 'one attempt per trial; no retries; infrastructure and task failures separate',
                'scope': 'RGB approach/route/recovery; not grasp/carry; separate Mac cohort',
                'cost_usd': None,
                'boundary': 'own RGB, shared top RGB, own issued commands, authored static map; weld off'}
    write_json(a.output/'protocol.json', protocol)
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    report = evaluate(a.output, protocol, key, str(a.mjpython.resolve()))
    return 0 if report['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
