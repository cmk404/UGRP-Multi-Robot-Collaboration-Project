"""Planning-guidance cohort: C0 legacy / C1 objective / C2 objective_preview.

Same scene (open, seed 11), same operator task, synchronous headless dispatch.
Conditions are interleaved so that time-of-day or endpoint drift does not
line up with one condition. Each run is a separate cleaned-up session.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TASK = '서로 역할과 순서를 합의해서 beam과 box를 dock_b로 옮겨'
CONDITIONS = ('legacy', 'objective', 'objective_preview')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--python', default=str(Path('/Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python')))
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if subprocess.check_output(['git', 'status', '--porcelain', '--', 'harness', 'scripts', 'sim', 'configs', 'config'],
                               cwd=ROOT, text=True).strip():
        raise SystemExit('source tree dirty; commit before the cohort')
    log = args.output / 'cohort.jsonl'
    for k in range(args.repeats):
        order = CONDITIONS[k % 3:] + CONDITIONS[:k % 3]
        for cond in order:
            name = f'{cond}-{k + 1}'
            out = args.output / name
            if (out / 'result.json').exists():
                continue
            row = {'run': name, 'condition': cond, 'repeat': k + 1, 'source_sha': sha,
                   'loadavg_before': os.getloadavg(), 'started_unix': time.time()}
            command = [sys.executable, 'scripts/ugrp_session.py', 'run', f'plan-guidance-{name}', '--',
                       args.python, '-m', 'scripts.sim_cli', 'dispatch', '--headless', '--variant', 'open',
                       '--plan-guidance', cond, '--task', TASK, '--output', str(out)]
            with (args.output / f'{name}.log').open('w') as stream:
                code = subprocess.call(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
            row.update(exit_code=code, wall_s=time.time() - row['started_unix'], loadavg_after=os.getloadavg())
            with log.open('a') as stream:
                stream.write(json.dumps(row) + '\n')
            print(json.dumps(row), flush=True)


if __name__ == '__main__':
    main()
