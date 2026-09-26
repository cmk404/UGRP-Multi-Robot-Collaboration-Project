"""Derived TensorBoard view for the v9 cohort (same M1 outcome contract as v5; v8 cohort as baseline).

Reuses build_tensorboard_v5.derived_run (outcome block validated, success = m1_success) with its own
derived-view folder. Raw results are never modified; snapshots are never overwritten. Usage:
  python experiments/2026-09-25-zone-owncam-skill/build_tensorboard_v9.py <cohort-dir-name> <snapshot-name>
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import build_tensorboard_v5 as b5  # noqa: E402

VIEW = b5.RAW / 'tensorboard-view-v9'
V9_TEST_SEEDS = tuple(range(601, 611))
V8_BASELINE = 'cohort-v8-39f214c'
V8_TEST_SEEDS = tuple(range(581, 592))


def main():
    cohort, snapshot = sys.argv[1], sys.argv[2]
    b5.VIEW = VIEW
    missing = [s for s in V9_TEST_SEEDS if not (b5.RAW / cohort / 'P' / str(s) / 'result.json').exists()]
    if missing:
        raise SystemExit(f'pre-registered v9 seeds {missing} missing/infrastructure_failure; export blocked')
    runs, conditions = [], {}

    def add(src, name, case, condition):
        runs.append(b5.derived_run(src, name, case, 'diagnostic'))
        conditions[name] = condition

    for s in V9_TEST_SEEDS:
        r = json.loads((b5.RAW / cohort / 'P' / str(s) / 'result.json').read_text())
        yaw = r['scenario_setup_only']['box_yaw_deg']
        parked = ','.join(sorted((r['scenario_setup_only'].get('parked_rel_box_m') or {}).keys())) or 'none'
        add(b5.RAW / cohort / 'P' / str(s), f'v9P-s{s}', 'v9 P diagnostic (gt_stub, NOT M1)',
            f'v9 pre-registered arm P, bay {r["scenario_setup_only"]["bay"]}, box yaw {yaw:+.0f} deg, parked {parked}, source {cohort[-7:]}')
    for src in sorted((b5.RAW / 'dev-v9').glob('*/result.json')):
        add(src.parent, 'v9dev-' + src.parent.name, 'v9 dev (not a result)', 'v9 development, not a result')
    for s in V8_TEST_SEEDS:
        add(b5.RAW / V8_BASELINE / 'P' / str(s), f'v8P-s{s}', 'v8 P baseline (gt_stub, NOT M1)',
            'v8 baseline arm P (39f214c), west pickup floor 581-588 + east 589-591')
    target = b5.TB / snapshot
    if target.exists():
        raise SystemExit(f'{target} exists; snapshots are never overwritten')
    cmd = [sys.executable, str(b5.ROOT / 'scripts/export_tensorboard.py'), '--output', str(target), '--max-images', '0']
    for name in runs:
        cmd += ['--source', str(VIEW / name)]
    subprocess.run(cmd, check=True)
    collection = json.loads((target / 'collection.json').read_text())
    for entry in collection['exported']:
        short = Path(entry['source']).name
        old = target / entry['name']
        if old.exists() and entry['name'] != short:
            old.rename(target / short)
        entry['original_name'], entry['name'] = entry['name'], short
        derived = json.loads((VIEW / short / 'result.json').read_text())
        entry['condition'] = (conditions[short] + f'; mode={derived["mode"]}, counts_as_m1={derived["counts_as_m1"]}, '
                              f'success=m1_success={derived["m1_success"]}, diagnostic_success={derived["diagnostic_success"]}')
    (target / 'collection.json').write_text(json.dumps(collection, indent=2) + '\n')
    print(json.dumps({'snapshot': str(target), 'runs': len(collection['exported']), 'failed': collection.get('failed')}))


if __name__ == '__main__':
    main()
