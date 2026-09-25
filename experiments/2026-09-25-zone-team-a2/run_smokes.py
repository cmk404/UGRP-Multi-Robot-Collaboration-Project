"""Zone team A2 fixture smokes (GT teacher, LLM 0 calls, synchronous SIM, weld OFF, --record-replay).

At most two runs at a time (shared host). Each run's 1-minute load average is
recorded at its start and end; load does not change SIM results.

  .venv-sim/bin/python experiments/2026-09-25-zone-team-a2/run_smokes.py --output outputs/zone-team-a2-v2-20260925

Cohort 1 (source 143360d, before --perception-profile existed) ran top_cargo_v1 into
outputs/zone-team-a2-20260925; cohort 2 passes --perception-profile top_cargo_v2 (default).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MIXED = {'A': {'long_beam': 1}, 'B': {'heavy_crate': 1, 'red': 1}, 'C': {'can': 1, 'green': 1, 'tile': 1}}
TRI = {'A': {'tri_frame': 1, 'red': 1}, 'C': {'green': 1}}
# (run id, variant, goal, seed, coordination, extra args)
# Research conditions (user, 2026-09-25): independent and dynamic (leader not implemented yet);
# plan_first is legacy: one run checks its minimal v2 support (cohort 1 ran it on both seeds).
RUNS = [
    *[(f'a-two-{m}-s{s}', 'zone_wide_two_doors', MIXED, s, m, []) for s in (11, 12)
      for m in ('dynamic', 'independent')],
    ('a-two-plan_first-s11', 'zone_wide_two_doors', MIXED, 11, 'plan_first', []),
    ('b-door-tri-dynamic-s11', 'zone_wide_door', TRI, 11, 'dynamic', []),
    ('b-two-tri-dynamic-s11', 'zone_wide_two_doors', TRI, 11, 'dynamic', []),
    ('c-two-graspfail-dynamic-s11', 'zone_wide_two_doors', MIXED, 11, 'dynamic',
     ['--inject-team-grasp-failure', 'long_beam']),
]


def one(out, perception, rid, variant, goal, seed, mode, extra):
    target = out/rid
    row_file = out/'runs'/f'{rid}.json'
    if (target/'result.json').is_file() and row_file.is_file():
        return json.loads(row_file.read_text())
    cmd = [sys.executable, '-m', 'scripts.run_zone_dispatch', '--output', str(target), '--variant', variant,
           '--coordination', mode, '--mode', 'fixture', '--goal', json.dumps(goal), '--seed', str(seed),
           '--contact-profile', 'cargo_noslip_v1', '--perception-profile', perception, '--record-replay', *extra]
    start_load, t0 = os.getloadavg(), time.monotonic()
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    row = {'run': rid, 'variant': variant, 'goal': goal, 'seed': seed, 'coordination': mode, 'extra': extra,
           'perception_profile': perception,
           'returncode': proc.returncode, 'wall_s': round(time.monotonic()-t0, 1),
           'load_1min_start': round(start_load[0], 2), 'load_1min_end': round(os.getloadavg()[0], 2),
           'command': cmd[1:]}
    (out/'runs').mkdir(exist_ok=True)
    (out/'runs'/f'{rid}.log').write_text(proc.stdout + proc.stderr)
    row_file.write_text(json.dumps(row, indent=2) + '\n')
    print(json.dumps({k: row[k] for k in ('run', 'returncode', 'wall_s', 'load_1min_start')}), flush=True)
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--jobs', type=int, default=2)
    p.add_argument('--only', nargs='*')
    p.add_argument('--perception-profile', default='top_cargo_v2', choices=('top_cargo_v1', 'top_cargo_v2'))
    args = p.parse_args()
    if args.jobs > 2:
        raise SystemExit('shared host: at most 2 concurrent runs')
    args.output.mkdir(parents=True, exist_ok=True)
    plan = [r for r in RUNS if not args.only or r[0] in args.only]
    with ThreadPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(lambda a: one(args.output, args.perception_profile, *a), plan))
    (args.output/'matrix.json').write_text(json.dumps(rows, indent=2) + '\n')


if __name__ == '__main__':
    main()
