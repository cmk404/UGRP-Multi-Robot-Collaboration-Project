"""Derived TensorBoard view for the v4 cohort (+ v4 dev, v3 P baseline).

Writes derived result.json files (source path + SHA-256, never modifying raw
results) under outputs/zone-owncam-skill-20260925/tensorboard-view-v3/<run>/,
exports them with scripts/export_tensorboard.py into a NEW snapshot and renames
the exported runs to short names. Usage:
  python experiments/2026-09-25-zone-owncam-skill/build_tensorboard_v4.py <cohort-dir-name> <snapshot-name>
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RAW = Path('/Users/changmin/projects/ugrp/outputs/zone-owncam-skill-20260925')
TB = Path('/Users/changmin/projects/ugrp/outputs/tensorboard')
VIEW = RAW / 'tensorboard-view-v4'
DEFINITION = ('evaluation-only GT: box upright on floor inside the ordered slot (+-0.06 m); '
              'pose_source=gt_stub_eval_only, NOT M1')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def derived_run(src, name, case, condition):
    r = json.loads((src / 'result.json').read_text())
    profile = r.get('contact_profile_selected') or 'local_contact_fine'
    out = {'derived_view_only': True, 'derived_from': str(src), 'source_result_sha256': sha(src / 'result.json'),
           'success': bool(r['evaluation_only']['place_in_slot_gt']), 'success_definition': DEFINITION,
           'stop_reason': r['reason'], 'scope': r['claim_scope'], 'policy': r['profile'], 'case': case,
           'config': {'contact_profile': profile}, 'sim_s': r['sim_seconds'], 'wall_s': r['wall_seconds'],
           'commands': r['steps'] - 1, 'model_calls': 0, 'evaluation': r['evaluation_only'], 'seed': r['seed'],
           'development_seed': r['development_seed'], 'source_sha': r['source_sha'],
           'skill_summary': r.get('skill_summary')}
    folder = VIEW / name
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'result.json').write_text(json.dumps(out, indent=1) + '\n')
    return name, condition


def main():
    cohort, snapshot = sys.argv[1], sys.argv[2]
    runs = []
    for arm, s_range, label in (('P', range(531, 541), 'P cargo_noslip_v1'),
                                ('D', (531, 533), 'D drop-safety (fault injection)')):
        for s in s_range:
            src = RAW / cohort / arm / str(s)
            if (src / 'result.json').exists():
                runs.append(derived_run(src, f'v4{arm}-s{s}', label,
                                        f'v4 pre-registered arm {arm} ({label}), source {cohort[-7:]}'))
    for src in sorted((RAW / 'dev-v4').glob('*/result.json')):
        parts = src.parent.name.split('-')
        runs.append(derived_run(src.parent, 'v4dev-' + '-'.join(parts[:-1]), 'dev', 'v4 development (not a result), a8b9ee2'))
    for s in range(521, 531):
        src = RAW / 'cohort-v3-7322a96' / 'P' / str(s)
        runs.append(derived_run(src, f'v3P-s{s}', 'v3 P cargo_noslip_v1', 'v3 baseline arm P (7322a96), different scenarios'))
    target = TB / snapshot
    if target.exists():
        raise SystemExit(f'{target} exists; snapshots are never overwritten')
    cmd = [sys.executable, str(ROOT / 'scripts/export_tensorboard.py'), '--output', str(target), '--max-images', '0']
    for name, _ in runs:
        cmd += ['--source', str(VIEW / name)]
    subprocess.run(cmd, check=True)
    collection = json.loads((target / 'collection.json').read_text())
    conditions = dict(runs)
    for entry in collection['exported']:
        short = Path(entry['source']).name
        old = target / entry['name']
        if old.exists() and entry['name'] != short:
            old.rename(target / short)
        entry['original_name'], entry['name'] = entry['name'], short
        entry['condition'] = conditions[short] + '; zone_wide_door east section, gt_stub_eval_only, NOT M1'
    (target / 'collection.json').write_text(json.dumps(collection, indent=2) + '\n')
    print(json.dumps({'snapshot': str(target), 'runs': len(collection['exported']), 'failed': collection.get('failed')}))


if __name__ == '__main__':
    main()
