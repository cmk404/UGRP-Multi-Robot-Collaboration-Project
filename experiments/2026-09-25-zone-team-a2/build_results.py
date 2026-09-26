"""Build experiments/2026-09-25-zone-team-a2/results.json from both smoke cohorts (observer-only).

  .venv-sim/bin/python experiments/2026-09-25-zone-team-a2/build_results.py \
      --cohort c1=/Users/.../zone-team-a2/outputs/zone-team-a2-20260925@143360d \
      --cohort c2=/Users/.../zone-team-a2-dev/outputs/zone-team-a2-v2-20260925@1f25d15
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _analyze():
    spec = importlib.util.spec_from_file_location('a2_analyze', HERE/'analyze.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cohort', action='append', required=True, help='name=path@source_sha')
    p.add_argument('--media', type=Path)
    p.add_argument('--out', type=Path, default=HERE/'results.json')
    args = p.parse_args()
    an = _analyze()
    cohorts = []
    for spec in args.cohort:
        name, rest = spec.split('=', 1)
        path, sha = rest.rsplit('@', 1)
        root = Path(path)
        runs, skipped = [], []
        for rf in sorted((root/'runs').glob('*.json')):
            row = json.loads(rf.read_text())
            if row.get('skipped'):
                skipped.append({'run': row['run'], 'reason': row['reason']})
                continue
            if not (root/row['run']/'result.json').is_file():
                skipped.append({'run': row['run'], 'reason': 'no result.json'})
                continue
            m = an.metrics(root/row['run'], row)
            m.pop('robot_events', None)
            m['blockers_n'] = len(m.pop('blockers'))
            runs.append(m)
        cohorts.append({'cohort': name, 'source_sha': sha, 'raw_root': str(root), 'raw_is_local_only': True,
                        'runs': runs, 'skipped': skipped})
    media = {}
    if args.media and args.media.is_dir():
        for f in sorted(args.media.glob('*.mp4')):
            media[f.name] = {'sha256': an.sha(f), 'bytes': f.stat().st_size}
    out = {'experiment': '2026-09-25-zone-team-a2',
           'claim_scope': ('TEACHER feasibility only: ground-truth drive/IK teacher (weld OFF), fixture claims, '
                           '0 LLM calls, synchronous SIM. Not a robot, RGB-skill or student success. The final '
                           'study executor is own-camera only (user, 2026-09-25); these runs judge whether the '
                           'teacher can produce demonstrations on these maps and goals.'),
           'robot_facing_outcome_source': 'teacher_receipt_L4 (audit L4 unresolved)',
           'cohorts': cohorts, 'media': media}
    args.out.write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    print(args.out, sum(len(c['runs']) for c in cohorts), 'runs')


if __name__ == '__main__':
    main()
