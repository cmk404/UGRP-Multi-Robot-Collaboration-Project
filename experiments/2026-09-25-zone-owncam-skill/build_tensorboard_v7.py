"""Derived TensorBoard view for the v7 cohort (same M1 outcome contract as v5; v6 cohort as baseline).

Reuses build_tensorboard_v5.derived_run (outcome block validated, success = m1_success) with its own
derived-view folder. Raw results are never modified; snapshots are never overwritten. Usage:
  python experiments/2026-09-25-zone-owncam-skill/build_tensorboard_v7.py <cohort-dir-name> <snapshot-name>
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import build_tensorboard_v5 as b5  # noqa: E402

VIEW = b5.RAW / 'tensorboard-view-v7'
V7_TEST_SEEDS = tuple(range(561, 573))
V6_BASELINE = 'cohort-v6-ac34651'
V6_TEST_SEEDS = tuple(range(551, 561))


def main():
    cohort, snapshot = sys.argv[1], sys.argv[2]
    b5.VIEW = VIEW
    missing = [s for s in V7_TEST_SEEDS if not (b5.RAW / cohort / 'P' / str(s) / 'result.json').exists()]
    if missing:
        raise SystemExit(f'pre-registered v7 seeds {missing} missing/infrastructure_failure; export blocked')
    runs, conditions = [], {}

    def add(src, name, case, condition):
        runs.append(b5.derived_run(src, name, case, 'diagnostic'))
        conditions[name] = condition

    for s in V7_TEST_SEEDS:
        r = json.loads((b5.RAW / cohort / 'P' / str(s) / 'result.json').read_text())
        yaw = r['scenario_setup_only']['box_yaw_deg']
        parked = ','.join(sorted((r['scenario_setup_only'].get('parked_rel_box_m') or {}).keys())) or 'none'
        add(b5.RAW / cohort / 'P' / str(s), f'v7P-s{s}', 'v7 P diagnostic (gt_stub, NOT M1)',
            f'v7 pre-registered arm P, box yaw {yaw:+.0f} deg, parked {parked} next to box, source {cohort[-7:]}')
    for src in sorted((b5.RAW / 'dev-v7').glob('*/result.json')):
        add(src.parent, 'v7dev-' + src.parent.name, 'v7 dev (not a result)', 'v7 development, not a result')
    for s in V6_TEST_SEEDS:
        add(b5.RAW / V6_BASELINE / 'P' / str(s), f'v6P-s{s}', 'v6 P baseline (gt_stub, NOT M1)',
            'v6 baseline arm P (ac34651), peers parked relative to the v5 approach point, different scenarios')
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
