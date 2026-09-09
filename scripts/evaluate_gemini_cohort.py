"""Run fresh matched trials; stop scheduling after an inference outage.

Trial concurrency is separate from the three asynchronous robots inside each
trial. Existing outputs are never overwritten or silently omitted.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import subprocess
import sys


def infrastructure_problem(run: Path, returncode: int) -> str | None:
    if returncode:
        return f'PROCESS_EXIT_{returncode}'
    path = run / 'result.json'
    if not path.exists():
        return 'MISSING_RESULT'
    result = json.loads(path.read_text())
    if result.get('error'):
        return 'RUNNER_ERROR'
    if any(result.get('inference_unresolved', {}).values()):
        return 'INFERENCE_UNAVAILABLE'
    for outcome in result.get('outcomes', {}).values():
        if str(outcome.get('reason', '')).startswith('LLM_ERROR:'):
            return 'INFERENCE_UNAVAILABLE'
    return None


def run_cohort(root: Path, runs: list[dict], *, jobs: int, execute) -> dict:
    completed = []
    blocked = None
    for offset in range(0, len(runs), jobs):
        batch = runs[offset:offset + jobs]
        with ThreadPoolExecutor(max_workers=jobs) as pool:
            codes = list(pool.map(execute, batch))
        for run, code in zip(batch, codes):
            problem = infrastructure_problem(root / run['path'], code)
            completed.append({**run, 'returncode': code, 'infrastructure_problem': problem})
            blocked = blocked or problem
        state = {'status': 'infrastructure_blocked' if blocked else 'running',
                 'attempted': completed, 'started_runs': [r['path'] for r in completed],
                 'not_started': runs[offset + len(batch):],
                 'blocking_reason': blocked}
        (root / 'cohort-status.json').write_text(json.dumps(state, indent=2))
        if blocked:
            return state
    state['status'] = 'finished'
    (root / 'cohort-status.json').write_text(json.dumps(state, indent=2))
    return state


def build_run_command(args, run):
    """Use the same explicitly recorded model and budgets in every cell."""
    return [sys.executable, '-m', 'scripts.evaluate_gemini_team',
            '--output', str(args.output / run['path']), '--seed', str(run['seed']),
            '--robots', '3', '--seconds', str(args.seconds),
            '--model', args.model, '--max-calls', str(args.max_calls),
            '--max-input-tokens', str(args.max_input_tokens),
            '--input-request-estimate', str(args.input_request_estimate),
            '--impratio', '10', '--noslip-iterations', '3',
            '--communication', run['communication'], '--record']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[41, 89, 97])
    parser.add_argument('--jobs', type=int, choices=(1, 2, 3), default=1)
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--max-calls', type=int, default=40)
    parser.add_argument('--max-input-tokens', type=int, default=120000)
    parser.add_argument('--input-request-estimate', type=int, default=6000)
    parser.add_argument('--model', default='gemini-3.8-flash')
    args = parser.parse_args()
    if (min(args.max_calls,args.max_input_tokens,args.input_request_estimate) <= 0
            or not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("budgets must be positive")
    if len(set(args.seeds)) != len(args.seeds):
        parser.error('seeds must be unique')
    args.output.mkdir(parents=True, exist_ok=False)
    runs = [{'path': f'team-{seed}-{mode}', 'seed': seed, 'communication': mode}
            for seed in args.seeds for mode in ('none', 'status', 'natural')]
    manifest = {'purpose': 'matched exploratory communication comparison',
                'seeds': args.seeds, 'modes': ['none', 'status', 'natural'],
                'robots': 3, 'model': args.model, 'seconds': args.seconds,
                'max_calls_per_robot': args.max_calls, 'trial_jobs': args.jobs,
                'max_input_tokens_per_robot': args.max_input_tokens,
                'input_request_estimate': args.input_request_estimate,
                'budget_mode':'estimated_preflight','runs': runs}
    (args.output / 'cohort-manifest.json').write_text(json.dumps(manifest, indent=2))

    def execute(run):
        with (args.output / (run['path'] + '.log')).open('w') as log:
            return subprocess.run(build_run_command(args, run),
                stdout=log, stderr=subprocess.STDOUT).returncode

    state = run_cohort(args.output, runs, jobs=args.jobs, execute=execute)
    subprocess.run([sys.executable, '-m', 'scripts.render_visual_team_report', str(args.output)], check=True)
    print(json.dumps(state, ensure_ascii=False))
    return 2 if state['blocking_reason'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
