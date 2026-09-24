#!/usr/bin/env python3
"""Run the preregistered ABBA cohort sequentially (protocol.json).

Each run uses the standard dispatch entry point under ugrp_session so the
whole child tree is cleaned up. A B failure skips remaining B runs; A2 still
runs. Existing output directories are never reused.
"""
import argparse, hashlib, json, os, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTS = {'v1': Path('/Users/changmin/projects/ugrp/outputs/fine-gain-schedule-20260924'),
        'v2': Path('/Users/changmin/projects/ugrp/outputs/fine-gain-schedule-20260924-sync')}
MODELS = Path('/Users/changmin/.codex/worktrees/faster-dispatch/ugrp/outputs/dispatch-models/e78a5a5777f5bc48/models')
ORDER = [('A1', False), ('B1', True), ('B2', True), ('A2', False)]
BASE = ['--realtime-control', '--plan-replay', 'experiments/2026-09-22-parallel-transport/independent-plan.json',
        '--variant', 'open', '--seed', '11', '--contact-profile', 'local_contact_fine',
        '--realtime-factor', '1', '--max-wall-s', '350', '--overlap-start', 'grasp', '--video-fps', '10',
        '--grasp-model-dir', str(MODELS / 'grasp'), '--stage-model-dir', str(MODELS / 'varied')]
# protocol-v2: synchronous (non-realtime) headless execution; physics waits
# for each capture/decision, so host CPU load changes wall time only.
BASE_V2 = ['--headless', '--plan-replay', 'experiments/2026-09-22-parallel-transport/independent-plan.json',
           '--variant', 'open', '--seed', '11', '--contact-profile', 'local_contact_fine',
           '--max-wall-s', '1200', '--overlap-start', 'grasp', '--video-fps', '10',
           '--grasp-model-dir', str(MODELS / 'grasp'), '--stage-model-dir', str(MODELS / 'varied')]


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', choices=('v1', 'v2'), default='v1')
    version = parser.parse_args().protocol
    OUT, base = OUTS[version], (BASE if version == 'v1' else BASE_V2)
    if git('status', '--porcelain'):
        raise SystemExit('runtime source must be committed and clean before the cohort')
    OUT.mkdir(parents=True, exist_ok=True)
    sha = git('rev-parse', 'HEAD')
    b_failed = False
    for name, schedule in ORDER:
        output = OUT / name
        if output.exists():
            raise SystemExit(f'output exists: {output}')
        if schedule and b_failed:
            (OUT / f'{name}.skipped.json').write_text(json.dumps(
                {'run': name, 'reason': 'protocol stop: earlier B failure'}, indent=2) + '\n')
            continue
        # The standard launcher owns its own ugrp_session process group.
        command = ['bash', 'scripts/open_simulation.command', 'dispatch', *base,
                   *(['--fine-gain-schedule'] if schedule else []), '--output', str(output)]
        launch = {'protocol': version, 'run': name, 'fine_gain_schedule': schedule, 'source_sha': sha, 'cwd': str(ROOT),
                  'command': command, 'loadavg_before': os.getloadavg(), 'started_unix': time.time()}
        log = OUT / f'{name}.console.log'
        start = time.monotonic()
        with log.open('xb') as stream:
            code = subprocess.call(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        launch.update(exit_code=code, process_wall_s=time.monotonic() - start, finished_unix=time.time(),
                      console_log_sha256=hashlib.sha256(log.read_bytes()).hexdigest())
        result_path = output / 'result.json'
        ok = False
        if result_path.is_file():
            result = json.loads(result_path.read_text())
            ok = bool(result.get('physical_success')) and bool(result.get('protocol_complete'))
            launch.update(result_sha256=hashlib.sha256(result_path.read_bytes()).hexdigest(),
                          physical_success=result.get('physical_success'),
                          protocol_complete=result.get('protocol_complete'),
                          error=result.get('error'), wall_s=result.get('wall_s'))
        launch['success'] = ok
        (OUT / f'{name}.launch.json').write_text(json.dumps(launch, indent=2) + '\n')
        print(json.dumps({k: launch.get(k) for k in ('run', 'exit_code', 'success', 'wall_s', 'error')}), flush=True)
        if schedule and not ok:
            b_failed = True


if __name__ == '__main__':
    main()
