"""Derived TensorBoard view for the v5 cohort with the M1 outcome contract (Codex review issue 1).

Every derived result carries the required outcome fields of harness.m1_contract
(mode, counts_as_m1, m1_success, diagnostic_success, pose_source,
pose_sources_seen, input_contract, success_semantics) and "success" =
m1_success. Legacy v4 results (no outcome block) are relabelled the same way as
DIAGNOSTIC runs, so a GT-pose run can never show success:true (the v2 s518 /
v4 derived-view mistake). Every derived result is validated before export; with
--view m1 the exporter refuses any result that does not count as M1.
Raw results are never modified; snapshots are never overwritten. Usage:
  python experiments/2026-09-25-zone-owncam-skill/build_tensorboard_v5.py <cohort-dir-name> <snapshot-name> [--view diagnostic|m1]
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from harness import m1_contract  # noqa: E402

RAW = Path('/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925')
TB = Path('/Users/changmin/projects/ugrp/outputs/tensorboard')
VIEW = RAW / 'tensorboard-view-v5'
V5_TEST_SEEDS = tuple(range(541, 549))
V4_TEST_SEEDS = tuple(range(531, 541))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def outcome_of(r):
    """The v5 outcome block, or a DIAGNOSTIC relabel of a legacy (v1-v4) result."""
    if all(k in r for k in m1_contract.REQUIRED_OUTCOME_KEYS):
        return {k: r[k] for k in m1_contract.REQUIRED_OUTCOME_KEYS}
    ev = r['evaluation_only']
    sources = r.get('pose_sources_seen') or [r['pose_source']]
    return {k: v for k, v in m1_contract.outcome_fields(
        mode='diagnostic', pose_sources_seen=sources,
        diagnostic_success=bool(ev['place_in_slot_gt'] and ev.get('skill_claim_in_slot')),
        input_contract={'legacy_profile': r['profile'], 'camera': 'robot_cam (not re-validated at look_back in v1-v4)',
                        'order_sheet': 'EXACT pickup_xy (v1-v4; Codex review issue 3)', 'pose_source': sources},
    ).items() if k in m1_contract.REQUIRED_OUTCOME_KEYS}


def derived_run(src, name, case, view):
    r = json.loads((src / 'result.json').read_text())
    outcome = outcome_of(r)
    out = {'derived_view_only': True, 'derived_from': str(src), 'source_result_sha256': sha(src / 'result.json'),
           **outcome, 'success': outcome['m1_success'],
           'stop_reason': r['reason'], 'scope': r['claim_scope'], 'policy': r['profile'], 'case': case,
           'config': {'contact_profile': r.get('contact_profile_selected') or 'local_contact_fine',
                      'mode': outcome['mode'], 'counts_as_m1': outcome['counts_as_m1']},
           'sim_s': r['sim_seconds'], 'wall_s': r['wall_seconds'], 'commands': r['steps'] - 1, 'model_calls': 0,
           'evaluation': {**r['evaluation_only'], 'diagnostic_success': outcome['diagnostic_success'],
                          'm1_success': outcome['m1_success'], 'counts_as_m1': outcome['counts_as_m1']},
           'seed': r['seed'], 'development_seed': r['development_seed'],
           'source_sha': r.get('source_sha_start') or r.get('source_sha'), 'skill_summary': r.get('skill_summary')}
    m1_contract.validate_outcome(out)
    if view == 'm1' and not out['counts_as_m1']:
        raise m1_contract.ContractViolation(f'{src}: pose sources {out["pose_sources_seen"]} do not count as M1; '
                                            'refusing to export it into an M1 view')
    folder = VIEW / name
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'result.json').write_text(json.dumps(out, indent=1) + '\n')
    return name


def main():
    cohort, snapshot = sys.argv[1], sys.argv[2]
    view = sys.argv[sys.argv.index('--view') + 1] if '--view' in sys.argv else 'diagnostic'
    m1_contract.check_mode(view)
    runs, conditions = [], {}
    missing = [s for s in V5_TEST_SEEDS if not (RAW / cohort / 'P' / str(s) / 'result.json').exists()]
    if missing:
        raise SystemExit(f'pre-registered v5 seeds {missing} missing/infrastructure_failure; export blocked')
    for s in V5_TEST_SEEDS:
        name = derived_run(RAW / cohort / 'P' / str(s), f'v5P-s{s}', 'v5 P diagnostic (gt_stub, NOT M1)', view)
        runs.append(name)
        conditions[name] = f'v5 pre-registered arm P, coarse bay, source {cohort[-7:]}'
    if view == 'diagnostic':
        for src in sorted((RAW / 'dev-v5').glob('*/result.json')):
            name = derived_run(src.parent, 'v5dev-' + src.parent.name, 'v5 dev (not a result)', view)
            runs.append(name)
            conditions[name] = 'v5 development, dirty source, not a result'
        for s in V4_TEST_SEEDS:
            name = derived_run(RAW / 'cohort-v4-6664425' / 'P' / str(s), f'v4P-s{s}',
                               'v4 P diagnostic relabel (gt_stub, exact pickup, NOT M1)', view)
            runs.append(name)
            conditions[name] = 'v4 baseline arm P (6664425), exact pickup_xy order, different scenarios; relabelled'
    target = TB / snapshot
    if target.exists():
        raise SystemExit(f'{target} exists; snapshots are never overwritten')
    cmd = [sys.executable, str(ROOT / 'scripts/export_tensorboard.py'), '--output', str(target), '--max-images', '0']
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
