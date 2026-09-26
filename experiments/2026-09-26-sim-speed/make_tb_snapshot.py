"""TensorBoard snapshot for the sim-speed runs (native event protobufs via the repo's export Writer).

Reads the raw sim_profile / pair_prof outputs read-only and writes one run per source under
``outputs/tensorboard/<snapshot>``. CPU numbers are ``speed/*``; wall time is informational only
(no agent lock). ``speed/equivalent`` is 1 only when the named equivalence report says identical.

    .venv-sim-worker-mac/bin/python experiments/2026-09-26-sim-speed/make_tb_snapshot.py \
        --raw /Users/changmin/projects/ugrp/outputs/sim-speed-20260926 \
        --output /Users/changmin/projects/ugrp/outputs/tensorboard/0926-sim-speed
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from scripts.tensorboard_tools.export import Writer  # noqa: E402

# run name -> (raw dir, case, runner, seed, speedups, cohort, equivalence report or None)
RUNS = {
    'm1-s93-none-120': ('base120i-s93', 'M1 dev s93, first 120 SIM s', 'm1', 93, 'none', 'cpu-120', None),
    'm1-s93-exact1-full': ('exact1-full-s93', 'M1 dev s93, full mission', 'm1', 93, 'exact-v1', 'full-equivalence',
                           'equivalence/s93_full_deva8_vs_exact1.json'),
    'm1-s93-exact1cv1-120': ('exact1-cv1-120-s93', 'M1 dev s93, first 120 SIM s, cv2 threads 1', 'm1', 93,
                             'exact-v1+cv1', 'cpu-120', 'equivalence/s93_120_base_vs_exact1_cv1.json'),
    'm1-s95-none-120': ('base120i-s95', 'M1 dev s95, first 120 SIM s', 'm1', 95, 'none', 'cpu-120', None),
    'm1-s95-exact1-full': ('exact1-full-s95b', 'M1 dev s95, full mission', 'm1', 95, 'exact-v1', 'full-equivalence',
                           'equivalence/s95_full_deva8_vs_exact1.json'),
    'm1-s93-slice-to-carry': ('slice-s93-to-carry', 'M1 dev s93 DEV-ONLY prefix slice to skill:to_carry_posture',
                              'm1', 93, 'exact-v1', 'dev-slice', 'equivalence/s93_slice_to_carry_vs_exact1_full.json'),
    'm1-s93-origrunner-120': ('base120-s93', 'M1 dev s93, first 120 SIM s, runner before speedups arg', 'm1', 93,
                              'none (pre-arg runner)', 'cpu-120', 'equivalence/s93_120_origrunner_vs_none.json'),
    'm1-s95-origrunner-120': ('base120-s95', 'M1 dev s95, first 120 SIM s, runner before speedups arg', 'm1', 95,
                              'none (pre-arg runner)', 'cpu-120', 'equivalence/s95_120_origrunner_vs_none.json'),
    'm1-s93-sections-highload-120': ('prof-s93-120-sections-base', 'M1 dev s93, 120 SIM s, section timers, load 12-90 '
                                     '(account-1 session)', 'm1', 93, 'none', 'profile-sections', None),
    'pair701-none': ('pair701-base', 'M2 pair open_floor dev 701 (PR #205 6990a6e)', 'm2_pair', 701, 'none',
                     'pair', None),
    'pair701-kernel': ('pair701-kernel', 'M2 pair open_floor dev 701 (PR #205 6990a6e)', 'm2_pair', 701,
                       'drive_kernel', 'pair', 'equivalence/pair701_base_vs_kernel.json'),
    'pair701-none-cprofile': ('pair701-base-cprof-b', 'M2 pair dev 701 under cProfile (CPU not comparable)', 'm2_pair',
                              701, 'none', 'profile-cprofile', 'equivalence/pair701_base_cprof_vs_base.json'),
    'm1-s95-exact1-aborted': ('exact1-full-s95', 'aborted within seconds (code.dirty=true from an untracked tool)',
                              'm1', 95, 'exact-v1', 'aborted', None),
    'pair701-cprofile-importfail': ('pair701-base-cprof', 'wrapper import failure before physics', 'm2_pair', 701,
                                    'none', 'aborted', None),
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--raw', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise SystemExit('output must be new (existing snapshots are never modified)')
    args.output.mkdir(parents=True)
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    at = time.time()
    exported = []
    for name, (raw, case, runner, seed, speedups, cohort, eq) in RUNS.items():
        src = args.raw/raw
        prof_path = src/'profile.json'
        prof = json.loads(prof_path.read_text()) if prof_path.exists() else {}
        run_dir = src/'run'
        result_path = run_dir/'result.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else {}
        started = run_dir/'attempt_started.json'
        code = json.loads(started.read_text())['code'] if started.exists() else {}
        w = Writer(args.output/name, at)
        mark = prof.get('cpu_at_sim_mark') or {}
        ctr = prof.get('counters') or {}
        mctr = mark.get('counters') or {}
        cpu = prof.get('cpu_s') or {}
        metrics = {
            'result/sim_s': result.get('sim_s', result.get('sim_seconds')),
            'result/commands': (result.get('commands') if isinstance(result.get('commands'), int)
                                else sum(result['commands'].values()) if isinstance(result.get('commands'), dict) else None),
            'result/wall_s': prof.get('wall_s_informational'),
            'speed/cpu_process_s_total': cpu.get('process'),
            'speed/cpu_main_thread_s_total': cpu.get('main_thread'),
            'speed/cpu_render_thread_s_total': cpu.get('render_thread'),
            'speed/instructions_g_total': ctr.get('instructions_g'),
            'speed/p_core_cpu_share_total': ctr.get('p_core_cpu_share'),
            'speed/cpu_process_s_at_120s': mark.get('process_s'),
            'speed/cpu_main_thread_s_at_120s': mark.get('main_thread_s'),
            'speed/instructions_g_at_120s': mctr.get('instructions_g'),
            'speed/p_core_cpu_share_at_120s': mctr.get('p_core_cpu_share'),
            'speed/load_avg_1m_start': (prof.get('load_average') or {}).get('start', [None])[0],
            'speed/mj_steps': prof.get('mj_steps'),
        }
        if runner == 'm1' and cohort == 'full-equivalence' and isinstance(result.get('m1_success'), bool):
            metrics['evaluation/reported_success'] = int(result['m1_success'])
        eq_report = None
        if eq:
            eq_report = json.loads((args.raw/eq).read_text())
            metrics['speed/equivalent'] = int(bool(eq_report['equivalent']))
            q = eq_report.get('qpos_checkpoints') or (eq_report.get('checks') or {}).get('qpos_checkpoints')
            if isinstance(q, dict) and 'identical' in q:
                metrics['speed/qpos_checkpoints_identical'] = int(bool(q['identical']))
        for tag, value in metrics.items():
            w.scalar(tag, value)
        provenance = {'raw_dir': str(src), 'case': case, 'cohort': cohort, 'speedups': speedups,
                      'profile_json_sha256': sha(prof_path) if prof_path.exists() else None,
                      'result_json_sha256': sha(result_path) if result_path.exists() else None,
                      'run_code_sha': code.get('sha') or prof.get('pair_worktree_head'), 'run_code_dirty': code.get('dirty'),
                      'outcome': result.get('outcome'), 'profile_error': prof.get('error'),
                      'dev_slice': prof.get('dev_slice'), 'load_average': prof.get('load_average'),
                      'equivalence_report': eq, 'equivalence_checks': None if eq_report is None else
                      ({k: v.get('identical') for k, v in eq_report['checks'].items()} if 'checks' in eq_report else
                       {k: eq_report.get(k) for k in ('equivalent', 'same', 'differ', 'result_differing_keys',
                                                       'qpos_checkpoints')}),
                      'note': 'wall_s is informational (no agent lock); compare speed/* CPU and instructions. '
                              'reported_success = result.json m1_success (full missions only); dev slices and '
                              'truncated runs are never M1 outcomes.',
                      'snapshot_builder_sha': head}
        w.text('provenance/run', provenance)
        w.hparams({'case': case, 'runner': runner, 'seed': seed, 'speedups': speedups, 'cohort': cohort,
                   'source_sha': (provenance['run_code_sha'] or '')[:9], 'outcome': result.get('outcome', 'none')},
                  [k for k, v in metrics.items() if isinstance(v, (int, float))])
        w.close()
        manifest = {'name': name, 'source': str(src), 'files': {
            str(f.relative_to(src)): sha(f) for f in (prof_path, result_path, started) if f.exists()},
            'counts': w.counts, 'exported_at': at, 'builder_sha': head}
        (args.output/name/'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        exported.append({'name': name, 'source': str(src), 'counts': w.counts, 'condition': f'{cohort}: {speedups}'})
        print(json.dumps(exported[-1]), flush=True)
    (args.output/'collection.json').write_text(json.dumps({'exported': exported, 'failed': []}, indent=2) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
