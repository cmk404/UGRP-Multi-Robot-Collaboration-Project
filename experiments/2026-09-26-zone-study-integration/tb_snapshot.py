"""TensorBoard snapshot of the integration plumbing smoke (native TensorBoard events, evaluation only).

Uses the study's own converter (``scripts/zone_study_report.py`` build + ``--tb-events``) on the
four trial records, then adds one Text card per run with the pose-provider label, writes
``collection.json`` and re-reads every event file to check the scalar values.

    python experiments/2026-09-26-zone-study-integration/tb_snapshot.py \
        --runs /Users/changmin/projects/ugrp/outputs/zone-study-integration-20260926/smoke-45999d9c \
        --snapshot /Users/changmin/projects/ugrp/outputs/tensorboard/0926-zone-study-integration-tags-temporary \
        --report /Users/changmin/projects/ugrp/outputs/zone-study-integration-20260926/smoke-45999d9c-report
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.zone_study_contract import MAIN_CONDITIONS  # noqa: E402
from scripts import zone_study_report as report  # noqa: E402

LABEL = {'pose_provider': 'tags_temporary', 'temporary': True, 'research_result': False,
         'note_ko': '임시, 표식 사용, 연구 결과 아님'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def add_label_text(run_dir, at, text):
    from tensorboard.compat.proto.event_pb2 import Event
    from tensorboard.compat.proto.summary_pb2 import Summary
    from tensorboard.plugins.text import metadata as text_metadata
    from tensorboard.summary.writer.event_file_writer import EventFileWriter
    from tensorboard.util.tensor_util import make_tensor_proto
    writer = EventFileWriter(str(run_dir), max_queue_size=10, flush_secs=5, filename_suffix='.label')
    value = Summary.Value(tag='provenance/pose_provider', metadata=text_metadata.create_summary_metadata('pose_provider', 'temporary provider label'),
                          tensor=make_tensor_proto([text.encode()], dtype='string'))
    writer.add_event(Event(wall_time=at, step=0, summary=Summary(value=[value])))
    writer.close()


def reread(snapshot, scalars):
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    problems, checked = [], 0
    for run in scalars['runs']:
        acc = EventAccumulator(str(Path(snapshot) / run['run']), size_guidance={'scalars': 0, 'tensors': 0})
        acc.Reload()
        for tag, value in run['scalars'].items():
            got = [e.value for e in acc.Scalars(tag)] if tag in acc.Tags()['scalars'] else []
            if len(got) != 1 or abs(got[0] - value) > 1e-6 * max(1., abs(value)):
                problems.append(f'{run["run"]}:{tag} {got} != {value}')
            checked += 1
        if not run['run'].startswith('cohort/') and 'provenance/pose_provider' not in acc.Tags()['tensors']:
            problems.append(f'{run["run"]}: no pose provider label')
    return {'checked_scalars': checked, 'problems': problems}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--runs', required=True)
    p.add_argument('--snapshot', required=True)
    p.add_argument('--report', required=True)
    p.add_argument('--episode', default='smoke-i700')
    p.add_argument('--conditions', nargs='+', default=list(MAIN_CONDITIONS), help='dev checks only')
    args = p.parse_args(argv)
    snapshot = Path(args.snapshot)
    if snapshot.exists():
        raise SystemExit(f'{snapshot} exists: snapshots are never overwritten')
    records = [Path(args.runs) / f'{c}-{args.episode}' / 'study' / 'trial_record.json' for c in args.conditions]
    missing = [str(r) for r in records if not r.is_file()]
    if missing:
        raise SystemExit(f'missing trial records: {missing}')
    at = time.time()
    built = report.build([str(r) for r in records], args.report, tb_events=str(snapshot), now=at)
    scalars = json.loads((Path(args.report) / 'scalars.json').read_text())
    text = ('pose_provider: tags_temporary — 임시, 표식 사용, 연구 결과 아님. no-LLM fixture 배선 스모크(1 seed); '
            '통신 효과·조건 우열의 근거가 아니다. 성공·배송은 eval-only 시뮬레이터 정답 판정이다.')
    for run in scalars['runs']:
        add_label_text(snapshot / run['run'], at, text)
    check = reread(snapshot, scalars)
    exported = []
    for run, record in zip([r for r in scalars['runs'] if not r['run'].startswith('cohort/')], records):
        files = sorted((snapshot / run['run']).glob('events.out.tfevents.*'))
        exported.append({'name': run['run'], 'hparams': run['hparams'], 'scalars': len(run['scalars']),
                         'event_files': {f.name: sha(f) for f in files}, 'origin': str(record),
                         'origin_sha256': sha(record)})
    collection = {'schema': 'ugrp.zone_study_tensorboard_collection.v1', 'snapshot': snapshot.name,
                  'pose_provider': LABEL, 'plumbing_only': True,
                  'converter': 'scripts/zone_study_report.py build --tb-events + Text label '
                               '(experiments/2026-09-26-zone-study-integration/tb_snapshot.py)',
                  'raw_root': str(Path(args.runs)), 'report': {f.name: sha(f) for f in sorted(Path(args.report).glob('*'))
                                                                 if f.is_file()},
                  'scope_ko': '통합 러너 배선 스모크(no-LLM fixture, 주 4조건 × seed 700, 임시 태그 자세 제공자). '
                              '통신 효과·연구 결과가 아니다.', 'exported': exported, 'reread': check,
                  'built': built}
    (snapshot / 'collection.json').write_text(json.dumps(collection, indent=2, ensure_ascii=False, default=str) + '\n')
    print(json.dumps({'runs': len(scalars['runs']), 'reread': check, 'collection_sha256': sha(snapshot / 'collection.json')},
                     ensure_ascii=False))
    return 0 if not check['problems'] else 1


if __name__ == '__main__':
    sys.exit(main())
