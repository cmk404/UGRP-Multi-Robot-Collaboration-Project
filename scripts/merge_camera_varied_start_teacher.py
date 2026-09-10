#!/usr/bin/env python3
"""Bind completed calibration and physical teacher evidence for RGB fitting."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def link_or_copy(source, destination):
    try:
        return os.link(source, destination)
    except OSError:
        return shutil.copy2(source, destination)


def merge(sources, out):
    sources, out = [Path(p).resolve() for p in sources], Path(out).resolve()
    if out.exists() or len(sources) != len(set(sources)):
        raise ValueError('output must be new and source directories distinct')
    reports = [json.loads((p / 'report.json').read_text()) for p in sources]
    if not reports or any(r.get('complete') is not True for r in reports):
        raise ValueError('every input teacher collection must be complete')
    seen, successful, excluded = set(), [], []
    for report in reports:
        names = report['successful_cases'] + report['excluded_cases']
        if len(names) != len(set(names)) or seen.intersection(names):
            raise ValueError('case IDs must be unique across teacher inputs')
        seen.update(names)
        successful.extend(report['successful_cases'])
        excluded.extend(report['excluded_cases'])
    out.mkdir(parents=True)
    actors, labels, provenance = [], [], []
    for index, (root, report) in enumerate(zip(sources, reports)):
        prefix = f'sources/{index}/'
        # Inputs are completed immutable evidence; hardlinks avoid duplicating RGB
        # disk usage while retaining in-root paths and original per-source reports.
        shutil.copytree(root, out / prefix, copy_function=link_or_copy)
        source_actors = [json.loads(line) for line in (root / 'actor-samples.jsonl').read_text().splitlines() if line.strip()]
        source_labels = [json.loads(line) for line in (root / 'teacher-labels.jsonl').read_text().splitlines() if line.strip()]
        for actor in source_actors:
            for record in actor['observations'].values():
                path = (root / record['path']).resolve()
                if not path.is_relative_to(root) or sha(path) != record['sha256']:
                    raise ValueError('input RGB path/hash mismatch')
                record['path'] = prefix + record['path']
        actors.extend(source_actors)
        labels.extend(source_labels)
        provenance.append({'raw_dir': str(root), 'mode': report['mode'],
                           'source_sha': report['source_sha'],
                           'input_hashes': {name: sha(root / name) for name in
                               ('report.json', 'actor-samples.jsonl', 'teacher-labels.jsonl')}})
    for name, values in [('actor-samples.jsonl', actors), ('teacher-labels.jsonl', labels)]:
        (out / name).write_text(''.join(json.dumps(row, sort_keys=True, allow_nan=False) + '\n' for row in values))
    goals = reports[0]['goal_references']
    for views in goals.values():
        for record in views.values():
            record['path'] = 'sources/0/' + record['path']
    report = {'complete': True, 'mode': 'merged_teacher', 'sources': provenance,
              'successful_cases': successful, 'excluded_cases': excluded,
              'goal_references': goals, 'actor_count': len(actors),
              'provenance': 'successful_cases means eligible calibration/trajectory training cases; synthetic resets are not physical successes'}
    (out / 'report.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--teacher-dirs', nargs='+', type=Path, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    result = merge(args.teacher_dirs, args.out_dir)
    print(json.dumps({'complete': result['complete'], 'actor_count': result['actor_count'],
                      'eligible_cases': len(result['successful_cases'])}))
