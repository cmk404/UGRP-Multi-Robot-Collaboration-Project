"""Zone teacher fix cohort (2026-09-26): GT TEACHER feasibility on new seeds + blocker re-runs.

TEACHER FEASIBILITY ONLY (never robot, RGB-skill or student success). Fixture claims, LLM 0 calls,
synchronous SIM, weld OFF, cargo_noslip_v1 (pending user approval), --record-replay, one BLAS/OpenMP
thread per run, at most two runs at a time; the 1-minute load average is recorded per run.

  python3 scripts/ugrp_session.py run kiro-teacher-fix-cohort -- \\
      /Users/changmin/projects/ugrp/.venv-sim-worker-mac/bin/python \\
      experiments/2026-09-26-zone-teacher-fix/run_cohort.py --output outputs/zone-teacher-fix-20260926
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
THREAD_ENV = {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'VECLIB_MAXIMUM_THREADS': '1',
              'MKL_NUM_THREADS': '1'}
MIXED = {'A': {'long_beam': 1}, 'B': {'heavy_crate': 1, 'red': 1}, 'C': {'can': 1, 'green': 1, 'tile': 1}}
TRI = {'A': {'tri_frame': 1, 'red': 1}, 'C': {'green': 1}}
TWO = 'zone_wide_two_doors'
# (run id, variant, goal, seed, coordination, extra args) -- pre-registered in README.md section 2.
RUNS = [
    ('d-b3-mix-dynamic-s12', TWO, MIXED, 12, 'dynamic', []),
    ('d-b4-mix-dynamic-s12-b3off', TWO, MIXED, 12, 'dynamic',
     ['--teacher-fix', json.dumps({'b3_claim_yield': False}), '--max-sim-s', '900']),
    *[(f'mix-{m}-s{s}', TWO, MIXED, s, m, []) for s in (21, 22, 23) for m in ('dynamic', 'independent')],
    *[(f'tri-dynamic-s{s}', TWO, TRI, s, 'dynamic', []) for s in (21, 22)],
]
# Gate-only runs (the runner refuses before any physics; exit code 3 expected).
GATE_RUNS = [
    ('g-b2-door-tri-s11', 'zone_wide_door', TRI, 11, 'dynamic', []),
    ('g-b8-two-tri-s11', TWO, TRI, 11, 'dynamic', []),
]


def one(out, rid, variant, goal, seed, mode, extra):
    target = out/rid
    row_file = out/'runs'/f'{rid}.json'
    if row_file.is_file():
        return json.loads(row_file.read_text())
    cmd = [sys.executable, '-m', 'scripts.run_zone_teacher_fix', '--output', str(target), '--variant', variant,
           '--coordination', mode, '--mode', 'fixture', '--goal', json.dumps(goal), '--seed', str(seed),
           '--contact-profile', 'cargo_noslip_v1', '--perception-profile', 'top_cargo_v2_track',
           '--record-replay', *extra]
    start_load, t0 = os.getloadavg(), time.monotonic()
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, env={**os.environ, **THREAD_ENV})
    end_load = os.getloadavg()
    row = {'run': rid, 'variant': variant, 'goal': goal, 'seed': seed, 'coordination': mode, 'extra': extra,
           'perception_profile': 'top_cargo_v2_track', 'contact_profile': 'cargo_noslip_v1 (pending user approval)',
           'thread_env': THREAD_ENV, 'load_avg_start': [round(v, 2) for v in start_load],
           'load_avg_end': [round(v, 2) for v in end_load], 'returncode': proc.returncode,
           'wall_s': round(time.monotonic()-t0, 1), 'command': cmd[1:]}
    (out/'runs').mkdir(exist_ok=True)
    (out/'runs'/f'{rid}.log').write_text(proc.stdout + proc.stderr)
    row_file.write_text(json.dumps(row, indent=2) + '\n')
    print(json.dumps({k: row[k] for k in ('run', 'returncode', 'wall_s', 'load_avg_start')}), flush=True)
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--jobs', type=int, default=2)
    p.add_argument('--only', nargs='*')
    args = p.parse_args()
    if args.jobs > 2:
        raise SystemExit('shared host: at most 2 concurrent runs')
    args.output.mkdir(parents=True, exist_ok=True)
    rows = [one(args.output, *r) for r in GATE_RUNS if not args.only or r[0] in args.only]
    plan = [r for r in RUNS if not args.only or r[0] in args.only]
    with ThreadPoolExecutor(args.jobs) as pool:
        rows += list(pool.map(lambda a: one(args.output, *a), plan))
    (args.output/'matrix.json').write_text(json.dumps(rows, indent=2) + '\n')


if __name__ == '__main__':
    main()
