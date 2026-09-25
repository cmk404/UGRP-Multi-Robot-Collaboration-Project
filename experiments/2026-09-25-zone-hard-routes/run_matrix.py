"""Fixture matrix for the hard-route zone maps (teacher executor, LLM 0 calls).

Runs scripts.run_zone_dispatch --mode fixture on synchronous SIM with
local_contact_fine and --record-replay (weld stays OFF in the runner), a few
at a time, and records the 1-minute load average at each run's start and end.
Load does not change SIM results; it is recorded per AGENTS.md.
"""
from __future__ import annotations

import argparse
import itertools
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GOALS = {'G5': ({'A': {'red': 2}, 'B': {'cyan': 1}, 'C': {'green': 1, 'red': 1}}, {}),
         'G8': ({'A': {'red': 2}, 'B': {'cyan': 2}, 'C': {'green': 1, 'yellow': 1}}, {'red': 1, 'cyan': 1})}
VARIANTS = ('zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor')
SHORT = {'zone_wide_door': 'door', 'zone_wide_two_doors': 'two', 'zone_wide_corridor': 'corr'}
MODES = ('plan_first', 'dynamic', 'independent')
SEEDS = (11, 12, 13)


def one(out, variant, goal, seed, mode):
    rid = f'{SHORT[variant]}-{goal}-{mode}-s{seed}'
    target = out/rid
    if (target/'result.json').is_file():
        return json.loads((out/'runs'/f'{rid}.json').read_text())
    g, extra = GOALS[goal]
    cmd = [sys.executable, '-m', 'scripts.run_zone_dispatch', '--output', str(target), '--variant', variant,
           '--coordination', mode, '--mode', 'fixture', '--goal', json.dumps(g), '--extra-boxes', json.dumps(extra),
           '--seed', str(seed), '--contact-profile', 'local_contact_fine', '--record-replay']
    start_load, t0 = os.getloadavg(), time.monotonic()
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    row = {'run': rid, 'variant': variant, 'goal': goal, 'seed': seed, 'coordination': mode, 'returncode': proc.returncode,
           'wall_s': round(time.monotonic()-t0, 1), 'load_1min_start': round(start_load[0], 2),
           'load_1min_end': round(os.getloadavg()[0], 2), 'command': cmd[1:]}
    (out/'runs').mkdir(exist_ok=True)
    (out/'runs'/f'{rid}.log').write_text(proc.stdout + proc.stderr)
    (out/'runs'/f'{rid}.json').write_text(json.dumps(row, indent=2) + '\n')
    print(json.dumps({k: row[k] for k in ('run', 'returncode', 'wall_s', 'load_1min_start')}), flush=True)
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--jobs', type=int, default=3)
    p.add_argument('--variants', nargs='*', default=VARIANTS)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    plan = list(itertools.product(args.variants, GOALS, SEEDS, MODES))
    with ThreadPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(lambda a: one(args.output, *a), plan))
    (args.output/'matrix.json').write_text(json.dumps(rows, indent=2) + '\n')


if __name__ == '__main__':
    main()
