#!/usr/bin/env python3
"""자기 카메라 폐루프 스냅샷을 다시 읽어 원본 JSON과 대조한다.

이벤트를 TensorBoard `EventAccumulator`로 다시 열고, 각 scalar·text·HParams 값을
**파생 뷰가 아니라 원본 실행 기록**(`outputs/owncam-loop-20260925/<시도>/<에피소드>/`의
`result.json`·`manifest.json`)에서 다시 계산해 비교한다. `--server`를 주면 실행 중인
공용 TensorBoard의 HTTP API로 같은 run·태그 값을 한 번 더 확인한다. 서버를 시작·종료하지
않고 읽기만 한다.

사용:
    python3 scripts/verify_owncam_loop_snapshot.py \
        --snapshot outputs/tensorboard/0926-zone-owncam-loop \
        --index <파생 뷰 index.json> --raw outputs/owncam-loop-20260925 \
        [--server http://127.0.0.1:6006] [--out experiments/<ID>/verification.json]
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import re
import urllib.parse
import urllib.request
from pathlib import Path

SCALAR_TAGS = ('result/sim_s', 'result/wall_s', 'result/commands', 'result/model_calls',
               'evaluation/reported_success', 'evaluation/robot_robot_contact_samples',
               'claims/protocol_complete')
TEXT_TAGS = ('result/summary', 'evaluation/referee_only', 'provenance/source')
FLOAT32_TOLERANCE = 1e-5


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path):
    return json.loads(path.read_text())


def expected_scalars(raw: dict, manifest: dict) -> dict:
    """원본 실행 기록에서 기대 scalar 값을 다시 만든다(파생 뷰를 보지 않는다)."""
    return {
        'result/sim_s': float(raw['student_sim_s']),
        'result/wall_s': float(manifest['wall_s']),
        'result/commands': float(raw['student_commands']),
        'result/model_calls': 0.0,
        'evaluation/reported_success': float(int(raw['episode_pass'])),
        'evaluation/robot_robot_contact_samples': float(raw['contacts_student']['peer_robot']),
        'claims/protocol_complete': float(int(raw['gates']['R1_exit_waypoint']['declared_arrival'])),
    }


def close(a: float, b: float) -> bool:
    return abs(a - b) <= FLOAT32_TOLERANCE * max(1.0, abs(b))


def text_payload(rendered: str):
    """`<pre>`로 감싼 텍스트 카드에서 JSON 본문을 되돌린다."""
    body = re.sub(r'^<pre>|</pre>$', '', rendered.strip())
    return json.loads(html.unescape(body))


def read_events(run_dir: Path) -> dict:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    from tensorboard.plugins.hparams import plugin_data_pb2

    acc = EventAccumulator(str(run_dir), size_guidance={'scalars': 10000, 'tensors': 500, 'images': 0})
    acc.Reload()
    tags = acc.Tags()
    scalars = {tag: [(s.step, float(s.value)) for s in acc.Scalars(tag)] for tag in tags['scalars']}
    texts = {}
    for tag in tags['tensors']:
        if tag.startswith('_hparams_/'):
            continue
        texts[tag] = [event.tensor_proto.string_val[0].decode() for event in acc.Tensors(tag)]
    hparams = {}
    if '_hparams_/session_start_info' in tags['tensors']:
        content = acc.SummaryMetadata('_hparams_/session_start_info').plugin_data.content
        info = plugin_data_pb2.HParamsPluginData.FromString(content).session_start_info
        for key, value in info.hparams.items():
            hparams[key] = (value.string_value if value.HasField('string_value')
                            else value.number_value if value.HasField('number_value')
                            else value.bool_value)
    return {'scalar_tags': sorted(tags['scalars']), 'text_tags': sorted(texts),
            'scalars': scalars, 'texts': texts, 'hparams': hparams}


def fetch(server: str, path: str, body: dict | None = None):
    url = server.rstrip('/') + path
    data = json.dumps(body).encode() if body is not None else None
    request = urllib.request.Request(
        url, data=data, headers={'Content-Type': 'application/json'} if data else {})
    with urllib.request.urlopen(request, timeout=60) as response:
        return json.loads(response.read().decode())


def verify(snapshot: Path, index_path: Path, raw_root: Path, server: str | None) -> dict:
    index = load(index_path)
    collection = load(snapshot / 'collection.json')
    by_run = {entry['name']: entry for entry in collection['exported']}
    report = {'schema': 'ugrp.owncam_loop_snapshot_verification.v1',
              'snapshot': str(snapshot), 'derived_index': str(index_path),
              'derived_index_sha256': sha256(index_path), 'raw_root': str(raw_root),
              'collection': {'exported': len(collection['exported']),
                             'failed': len(collection.get('failed') or [])},
              'runs': [], 'mismatches': [], 'totals': {}}
    compared = text_checks = 0
    for row in index['rows']:
        run = row['run']
        run_dir = snapshot / run
        entry = by_run.get(run)
        record = {'run': run, 'episode': row['episode'], 'attempt': row['attempt'],
                  'split': row['split'], 'carry': row['carry'], 'seed': row['seed'],
                  'cohort': row['cohort'], 'origin': row['origin']}
        if entry is None:
            report['mismatches'].append({'run': run, 'problem': 'missing from collection.json'})
            continue
        raw_path = Path(row['origin'])
        manifest_path = raw_path.parent / 'manifest.json'
        raw, manifest = load(raw_path), load(manifest_path)
        record['origin_sha256'] = sha256(raw_path)
        if record['origin_sha256'] != row['origin_sha256']:
            report['mismatches'].append({'run': run, 'problem': 'original result.json changed since build'})
        export_manifest = load(run_dir / 'manifest.json')
        record['export_complete'] = export_manifest.get('complete')
        record['export_warnings'] = export_manifest.get('warnings') or []
        record['exporter'] = export_manifest.get('exporter')
        if export_manifest.get('complete') is not True:
            report['mismatches'].append({'run': run, 'problem': 'export manifest not complete'})
        if record['export_warnings']:
            report['mismatches'].append({'run': run, 'problem': 'export warnings',
                                         'warnings': record['export_warnings']})
        for relative, recorded in (export_manifest.get('source_files') or {}).items():
            current = Path(export_manifest['source']) / relative
            if not current.is_file() or sha256(current) != recorded['sha256']:
                report['mismatches'].append({'run': run, 'problem': 'derived view changed since export',
                                             'file': relative})
        events = read_events(run_dir)
        record['scalar_tags'] = events['scalar_tags']
        record['text_tags'] = events['text_tags']
        record['hparams'] = events['hparams']
        expected = expected_scalars(raw, manifest)
        values = {}
        for tag in SCALAR_TAGS:
            points = events['scalars'].get(tag)
            if not points:
                report['mismatches'].append({'run': run, 'tag': tag, 'problem': 'scalar missing'})
                continue
            if len(points) != 1:
                report['mismatches'].append({'run': run, 'tag': tag, 'problem': 'expected one step',
                                             'steps': [p[0] for p in points]})
            value = points[0][1]
            values[tag] = value
            compared += 1
            if not close(value, expected[tag]):
                report['mismatches'].append({'run': run, 'tag': tag, 'problem': 'value differs from original',
                                             'event': value, 'original': expected[tag]})
        record['scalars'] = values
        record['expected_scalars'] = expected
        # 텍스트 카드: 평가 전용 게이트 원문이 원본과 같은지 확인한다.
        for tag in TEXT_TAGS:
            if tag not in events['texts']:
                report['mismatches'].append({'run': run, 'tag': tag, 'problem': 'text card missing'})
        referee = events['texts'].get('evaluation/referee_only')
        if referee:
            payload = text_payload(referee[0])
            text_checks += 1
            if payload.get('gates') != raw['gates']:
                report['mismatches'].append({'run': run, 'tag': 'evaluation/referee_only',
                                             'problem': 'gates differ from original'})
            for key, original in (('student_sim_s', raw['student_sim_s']),
                                  ('teacher_sim_s', raw.get('teacher_sim_s')),
                                  ('looks', raw['looks']),
                                  ('student_frames', raw['student_frames']),
                                  ('student_tag_visibility', raw['student_tag_visibility']),
                                  ('contacts_student', raw['contacts_student']),
                                  ('final_gt', raw['final_gt'])):
                text_checks += 1
                if payload.get(key) != original:
                    report['mismatches'].append({'run': run, 'tag': 'evaluation/referee_only',
                                                 'problem': f'{key} differs from original',
                                                 'event': payload.get(key), 'original': original})
        summary = events['texts'].get('result/summary')
        if summary:
            payload = text_payload(summary[0])
            text_checks += 1
            if payload.get('success') is not raw['episode_pass'] or payload.get('stop_reason') != raw['outcome']:
                report['mismatches'].append({'run': run, 'tag': 'result/summary',
                                             'problem': 'success or stop_reason differs from original'})
        for key, expected_value in (('case', row['split']), ('seed', str(row['seed']))):
            actual = str(events['hparams'].get(key, ''))
            if key == 'seed':
                if actual not in (expected_value, expected_value + '.0', str(float(row['seed']))):
                    report['mismatches'].append({'run': run, 'problem': 'hparam seed differs',
                                                 'hparam': actual, 'expected': expected_value})
            elif not actual.startswith(expected_value):
                report['mismatches'].append({'run': run, 'problem': 'hparam case split differs',
                                             'hparam': actual, 'expected': expected_value})
        record['collection_condition'] = entry.get('condition')
        report['runs'].append(record)

    if server:
        report['server'] = server_checks(server, snapshot.name, report['runs'], server_runs(server))
    report['totals'] = {'runs': len(report['runs']), 'scalars_compared': compared,
                        'text_field_checks': text_checks, 'mismatches': len(report['mismatches'])}
    return report


def server_runs(server: str) -> list:
    return fetch(server, '/data/runs')


def server_checks(server: str, snapshot_name: str, runs: list, listed: list) -> dict:
    """실행 중인 서버가 새 run을 나열하고 같은 값을 주는지 확인한다(재시작 없음)."""
    result = {'endpoint': server, 'total_runs_listed': len(listed), 'snapshot_runs_listed': 0,
              'scalars_compared': 0, 'text_cards_served': 0, 'hparams_sessions': 0, 'mismatches': []}
    for record in runs:
        name = f'{snapshot_name}/{record["run"]}'
        if name not in listed:
            result['mismatches'].append({'run': name, 'problem': 'not listed by the running server'})
            continue
        result['snapshot_runs_listed'] += 1
        query = urllib.parse.urlencode({'run': name})
        for tag, value in record['scalars'].items():
            points = fetch(server, f'/data/plugin/scalars/scalars?{query}&'
                                   + urllib.parse.urlencode({'tag': tag}))
            served = float(points[0][2])
            result['scalars_compared'] += 1
            if not close(served, value):
                result['mismatches'].append({'run': name, 'tag': tag, 'problem': 'server value differs',
                                             'server': served, 'event': value})
        for tag in TEXT_TAGS:
            served = fetch(server, f'/data/plugin/text/text?{query}&'
                                   + urllib.parse.urlencode({'tag': tag}))
            if not served or 'text' not in served[0]:
                result['mismatches'].append({'run': name, 'tag': tag, 'problem': 'server served no text'})
            else:
                result['text_cards_served'] += 1
    groups = fetch(server, '/data/plugin/hparams/session_groups',
                   {'experimentName': '', 'allowedStatuses': [
                       'STATUS_UNKNOWN', 'STATUS_SUCCESS', 'STATUS_FAILURE', 'STATUS_RUNNING'],
                    'colParams': [], 'startIndex': 0, 'sliceSize': 100000})
    session_groups = groups.get('sessionGroups', []) if isinstance(groups, dict) else groups
    sessions = {session['name'] for group in session_groups for session in group.get('sessions', [])}
    wanted = {f'{snapshot_name}/{record["run"]}' for record in runs}
    result['hparams_sessions'] = len(wanted & sessions)
    missing = sorted(wanted - sessions)
    if missing:
        result['mismatches'].append({'problem': 'runs missing from HParams sessions', 'runs': missing[:5],
                                     'missing': len(missing)})
    experiment = fetch(server, '/data/plugin/hparams/experiment', {'experimentName': ''})
    result['hparams_columns'] = sorted(info.get('name', '') for info in experiment.get('hparamInfos', []))
    result['hparams_metric_columns'] = sorted(
        (info.get('name') or {}).get('tag', '') for info in experiment.get('metricInfos', []))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--index', type=Path, required=True, help='파생 뷰 index.json')
    parser.add_argument('--raw', type=Path, required=True, help='원본 실행 루트(읽기 전용)')
    parser.add_argument('--server', help='실행 중인 TensorBoard 주소(읽기 전용 확인)')
    parser.add_argument('--out', type=Path, help='검증 결과 JSON 경로')
    args = parser.parse_args()
    report = verify(args.snapshot.resolve(), args.index.resolve(), args.raw.resolve(), args.server)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'totals': report['totals'], 'server': (report.get('server') or {}).get('mismatches'),
                      'snapshot_runs_listed': (report.get('server') or {}).get('snapshot_runs_listed'),
                      'first_mismatches': report['mismatches'][:3]}, ensure_ascii=False))
    return 1 if report['mismatches'] or (report.get('server') or {}).get('mismatches') else 0


if __name__ == '__main__':
    raise SystemExit(main())
