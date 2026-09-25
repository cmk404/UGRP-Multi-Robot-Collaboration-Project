#!/usr/bin/env python3
"""Collect the cargo_noslip_v1 side-effect audit A/B into results.json.

Reads every ``<scenario>-<profile>-<seed>/result.json`` under the run root and
applies the pre-registered comparison (``scripts.audit_contact_profiles.compare``)
per scenario and seed. Records source hashes so the raw location is identifiable.

  PYTHONPATH=. .venv-sim/bin/python experiments/2026-09-26-noslip-side-effects/collect.py \
      outputs/noslip-audit/<sha>
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.audit_contact_profiles import PROFILES, SCENARIOS, TOLERANCES, compare  # noqa: E402

HERE = Path(__file__).resolve().parent
BASE, CAND = PROFILES


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__.splitlines()[0])
    run_root = Path(sys.argv[1])
    root = run_root if run_root.is_absolute() else ROOT/run_root
    runs, raw = {}, {}
    for path in sorted(root.glob('*/result.json')):
        data = json.loads(path.read_text())
        runs[(data['scenario'], data['contact_profile']['name'], data['seed'])] = data
        for item in sorted(path.parent.iterdir()):
            raw[str(item.relative_to(root))] = {
                'sha256': hashlib.sha256(item.read_bytes()).hexdigest(), 'bytes': item.stat().st_size}
    scenarios = {}
    for name in sorted(SCENARIOS):
        seeds = sorted({seed for (s, _, seed) in runs if s == name})
        per_seed, gate_fail, first_digest = {}, [], None
        for seed in seeds:
            a, b = runs.get((name, BASE, seed)), runs.get((name, CAND, seed))
            if not (a and b):
                per_seed[str(seed)] = {'status': 'missing',
                                       'have': [p for p in PROFILES if (name, p, seed) in runs]}
                continue
            gates = compare(name, a['metrics'], b['metrics'])
            failed = [g for g in gates if not g['ok']]
            gate_fail += [(seed, g) for g in failed]
            metrics = {BASE: a['metrics'], CAND: b['metrics']}
            digest = hashlib.sha256(json.dumps(metrics, sort_keys=True).encode()).hexdigest()
            row = {
                'status': 'compared', 'gates': len(gates), 'failed': len(failed),
                'failed_gates': failed,
                'hard_gates_ok': all(g['ok'] for run in (a, b) for g in run['hard_gates']),
                'applied': {BASE: a['applied_solver_options'], CAND: b['applied_solver_options']},
                'sim_time_s': {BASE: a['sim_time_s'], CAND: b['sim_time_s']},
                'scene_xml_sha256': {BASE: a['scene_xml_sha256'], CAND: b['scene_xml_sha256']},
                'metrics_sha256': digest,
                'load_avg_start': {BASE: a['host']['load_avg_start'], CAND: b['host']['load_avg_start']},
                'load_avg_end': {BASE: a['host']['load_avg_end'], CAND: b['host']['load_avg_end']}}
            # zone_wide_door seeds only shuffle the west pickup box, so the audited
            # physics is seed-independent: later seeds are a determinism check.
            # Keep the full metrics once and a hash for the rest (raw files keep everything).
            if first_digest is None:
                first_digest = digest
                row['metrics'] = metrics
            else:
                row['identical_to_first_seed'] = digest == first_digest
                if digest != first_digest:
                    row['metrics'] = metrics
            per_seed[str(seed)] = row
        scenarios[name] = {'question': SCENARIOS[name]['question'], 'metric': SCENARIOS[name]['metric'],
                           'tolerance': TOLERANCES[name], 'seeds': per_seed,
                           'side_effect_detected': bool(gate_fail),
                           'failed_gate_names': sorted({g['gate'] for _, g in gate_fail})}
    slip = {}
    for name in ('solo_carry', 'solo_hold_load', 'pair_beam_hold'):
        rows = {}
        for profile in PROFILES:
            for (s, p, seed), data in sorted(runs.items()):
                if s == name and p == profile:
                    m = data['metrics']
                    rows[f'{profile}-{seed}'] = {
                        'hold_span_s': m['hold_span_s'], 'hold_slip_mm': m['hold_slip_mm'],
                        'hold_creep_mm_per_min': m['hold_creep_mm_per_min'],
                        'hold_creep_r2': m['hold_creep_r2'], 'carry_slip_mm': m['carry_slip_mm'],
                        'max_slip_mm': m['max_slip_mm'], 'dropped': m['dropped'],
                        'outcome': m['outcome'], 'placement_err_mm': m['placement_err_mm'],
                        'hold_finger_total_n': m['hold_finger_total_n']}
        slip[name] = rows
    git = lambda *a: subprocess.run(['git', *a], cwd=ROOT, text=True, capture_output=True).stdout.strip()
    out = {'schema': 'ugrp.contact_profile_side_effect_audit.results.v1',
           'experiment': '2026-09-26-noslip-side-effects',
           'profiles': {'baseline': BASE, 'candidate': CAND},
           'git': {'sha': git('rev-parse', 'HEAD'), 'branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
                   'dirty_run_sources': bool(git('status', '--porcelain', '--untracked-files=no',
                                                 '--', 'scripts', 'sim', 'harness'))},
           'runs': len(runs), 'raw_root_local_only': str(root), 'raw_files': raw,
           'scenarios': scenarios, 'grip_slip_intended_effect': slip,
           'side_effect_detected': any(v['side_effect_detected'] for v in scenarios.values())}
    (HERE/'results.json').write_text(json.dumps(out, indent=1)+'\n')
    print(json.dumps({'runs': len(runs),
                      'side_effects': {k: v['failed_gate_names'] for k, v in scenarios.items()
                                       if v['side_effect_detected']},
                      'missing': [f'{k}:{s}' for k, v in scenarios.items()
                                  for s, r in v['seeds'].items() if r['status'] == 'missing']}, indent=1))


if __name__ == '__main__':
    main()
