#!/usr/bin/env python3
"""One finite standard-manager diagnostic; no retries or background monitoring.

Use --prepare once from a clean checkout, then --trial 1 or --trial 2.
Trial 2 additionally needs a root-reviewed first-trial audit.json with passed=true.
Only this launcher's process group is ever stopped; foreign work blocks launch.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PRIMARY = Path('/Users/changmin/projects/ugrp')
PROTOCOL = Path(__file__).with_name('physical-protocol.json')


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value, *, new=False):
    with Path(path).open('x' if new else 'w') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    sys.modules[name] = value
    spec.loader.exec_module(value)
    return value


def identity(protocol):
    if git('status', '--porcelain'):
        raise RuntimeError('source must be clean before prepare/launch and unchanged afterwards')
    models = Path(protocol['models'])
    model_hashes = {str(p.relative_to(models)): digest(p)
                    for p in sorted(models.rglob('*')) if p.is_file()}
    model_tree = hashlib.sha256(''.join(f'{p} {h}\n' for p, h in model_hashes.items()).encode()).hexdigest()
    if model_tree != protocol['models_tree_sha256']:
        raise RuntimeError('model tree differs from preregistered input')
    paths = [PROTOCOL, Path(__file__), Path(protocol['host_guard']), ROOT/'scripts/agent_lock.py',
             ROOT/'config/rgb_execution_bundles'/f"{protocol['expected_bundle']}.json",
             ROOT/protocol['plan'], ROOT/protocol['reference_top']]
    hashes = {str(p): digest(p) for p in paths}
    for path, expected in ((Path(protocol['host_guard']), protocol['host_guard_sha256']),
                           (ROOT/protocol['plan'], protocol['plan_sha256']),
                           (ROOT/protocol['reference_top'], protocol['reference_top_sha256'])):
        if hashes[str(path)] != expected:
            raise RuntimeError(f'preregistered input drift: {path}')
    return {'source_sha': git('rev-parse', 'HEAD'), 'file_hashes': hashes,
            'models_tree_sha256': model_tree, 'model_files': model_hashes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--records', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--prepare', action='store_true')
    mode.add_argument('--trial', type=int, choices=(1, 2))
    args = parser.parse_args()
    protocol = read(PROTOCOL)
    records = args.records.resolve()
    freeze_path = records/'freeze.json'
    current = identity(protocol)
    if args.prepare:
        records.mkdir(parents=True, exist_ok=False)
        save(freeze_path, current, new=True)
        save(records/'closure.json', {'status': 'prepared', 'trials': [
            {'id': name, 'status': 'NOT_RUN'} for name in protocol['planned_trials']]}, new=True)
        print(json.dumps({'prepared': str(records), 'source_sha': current['source_sha']}))
        return
    if current != read(freeze_path):
        raise RuntimeError('frozen source/input changed; this cohort cannot be resumed')
    closure = read(records/'closure.json')
    index = args.trial-1
    if closure['trials'][index]['status'] != 'NOT_RUN':
        raise RuntimeError('trial already reserved/started; no replacement run')
    if index:
        prior = closure['trials'][0]
        audit_path = records/protocol['planned_trials'][0]/'audit.json'
        audit = read(audit_path)
        if (prior['status'] != 'completed' or not prior['physical_success']
                or not prior['protocol_complete'] or audit.get('passed') is not True
                or audit.get('result_sha256') != prior['result_sha256']
                or audit.get('freeze_sha256') != digest(freeze_path)):
            raise RuntimeError('first diagnostic has not passed physical and postrun audit gates')
    guard = module('settled_diagnostic_host_guard', Path(protocol['host_guard']))
    lock = module('settled_diagnostic_host_lock', ROOT/'scripts/agent_lock.py')
    lock.acquire(lock.DEFAULT_ROOT, owner='codex', branch=git('branch', '--show-current'),
                 purpose=f'v46 original diagnostic {args.trial}, finite 370s',
                 pid=os.getpid(), expected_minutes=7)
    launch = None
    run_started = False
    owned = None
    try:
        blocker, ignored = guard.classify_host_load(guard.foreign_snapshot(), policy='strict')
        if blocker is not None or ignored:
            raise RuntimeError(f'foreign host work blocks launch: {blocker}')
        if identity(protocol) != current:
            raise RuntimeError('source/input changed before launch')
        record_dir = records/protocol['planned_trials'][index]
        record_dir.mkdir(exist_ok=False)
        raw = record_dir/'raw'
        models = Path(protocol['models'])
        cmd = ['bash', 'scripts/open_simulation.command', 'dispatch',
               *protocol['fixed_args'], '--plan-replay', protocol['plan'],
               '--reference-top', protocol['reference_top'],
               '--grasp-model-dir', str(models/'grasp'), '--stage-model-dir', str(models/'varied'),
               '--max-wall-s', str(protocol['budget']['internal_wall_s']), '--output', str(raw)]
        launch = {'id': protocol['planned_trials'][index], 'status': 'started',
                  'source_sha': current['source_sha'], 'freeze_sha256': digest(freeze_path),
                  'command': cmd, 'cwd': str(ROOT), 'output': str(raw),
                  'started_unix': time.time(), 'loadavg_before': os.getloadavg()}
        closure['trials'][index] = launch
        save(records/'closure.json', closure)
        save(record_dir/'launch.json', launch, new=True)
        environment = dict(os.environ, UGRP_SIM_PYTHON=str(PRIMARY/'.venv-sim-worker-mac/bin/python'),
                           OMP_NUM_THREADS='2', MKL_NUM_THREADS='2')
        run_started = True
        owned = guard.run_owned(cmd, cwd=ROOT, environment=environment, log=record_dir/'console.log',
                                timeout=protocol['budget']['owned_process_wall_s'], host_load_policy='strict')
        launch.update(owned, process_wall_s=owned['wall_s'], loadavg_after=os.getloadavg(),
                      owned_group_cleaned=not guard.group_exists(owned['pid']),
                      frozen_identity_unchanged=identity(protocol) == current)
        matches = [p for p in (ROOT/'outputs/simulation-runs').glob('*/manifest.json')
                   if read(p).get('output') == str(raw)]
        if len(matches) != 1:
            raise RuntimeError(f'expected one standard manager manifest, found {len(matches)}')
        manager = read(matches[0])
        launch.update(manager_manifest=str(matches[0]), manager_sha256=digest(matches[0]),
                      manager_argv_matches=manager.get('argv') == cmd[2:],
                      manager_source_matches=manager.get('source', {}).get('source_sha') == current['source_sha'],
                      manager_source_unchanged=manager.get('source_changed_during_run') is False,
                      manager_inputs_unchanged=manager.get('inputs_changed_during_run') is False,
                      manager_exit_matches=manager.get('exit_code') == owned['exit_code'])
        result = read(raw/'result.json')
        launch.update({k: result.get(k) for k in ('physical_success', 'protocol_complete', 'wall_s', 'error')})
        launch['result_sha256'] = digest(raw/'result.json')
        launch['status'] = 'completed' if (owned['foreign_interference'] is None and not owned['timed_out']
            and owned['wall_s'] <= protocol['budget']['allocation_per_trial_s']
            and all(launch[k] for k in ('owned_group_cleaned', 'frozen_identity_unchanged',
                'manager_argv_matches', 'manager_source_matches', 'manager_source_unchanged',
                'manager_inputs_unchanged', 'manager_exit_matches'))) else 'infrastructure_failed'
        print(json.dumps(launch, ensure_ascii=False), flush=True)
    except BaseException as exc:
        if launch is not None:
            launch.update(status='interrupted_or_error', launcher_error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if launch is not None:
            save(record_dir/'launch.json', launch)
            closure['status'] = 'awaiting_root_audit' if launch['status'] == 'completed' else 'stopped'
            save(records/'closure.json', closure)
        # A guard exception may hide the child PID, including failed cleanup.
        # Retain the reservation until root inventories processes in that case.
        cleanup_verified = not run_started or (owned is not None and not guard.group_exists(owned['pid']))
        if cleanup_verified:
            lock.release(lock.DEFAULT_ROOT, owner='codex')
        else:
            closure['cleanup_unverified_lock_retained'] = True
            save(records/'closure.json', closure)
            print('Cleanup not confirmed; lock retained for root process inspection.', file=sys.stderr)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: sys.exit('launcher received SIGTERM'))
    main()
