#!/usr/bin/env python3
"""`0926-zone-perception-noslip` 스냅샷을 TensorBoard 자체 리더로 다시 읽어 원본과 대조한다.

확인하는 것:

1. 스냅샷의 run 수·이름이 파생 뷰 index와 같은지, 각 run manifest가 `complete`인지.
2. 이벤트에 실제로 들어간 scalar 값이 **파생 뷰 선언값**과 같은지.
3. 그 선언값이 **원본 기록**(인식 채점 `summary.json`, 감사 `results.json`·실행 `result.json`)의
   값과 같은지. 즉 변환기가 새로 만든 숫자가 없는지.
4. 시계열 표본 수와 표본 3개(첫·중간·끝)가 원본 `trace.jsonl`과 같은지.
5. Text 카드와 HParams 세션이 run마다 있는지.
6. 원본 파일이 변환 뒤에도 그대로인지(manifest의 SHA-256 재확인).

원본을 수정하지 않으며 시뮬레이션·재채점을 하지 않는다. 실행 중인 TensorBoard 서버는 건드리지 않는다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

TOLERANCE = 1e-6


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path):
    return json.loads(path.read_text())


def close(a, b) -> bool:
    if type(a) in (int, float) and type(b) in (int, float):
        return abs(float(a) - float(b)) <= TOLERANCE + TOLERANCE * abs(float(b))
    return a == b


def accumulate(run: Path):
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    accumulator = EventAccumulator(str(run), size_guidance={'scalars': 100000, 'tensors': 500})
    accumulator.Reload()
    return accumulator


def leaf(row, path):
    value = row
    for key in path:
        if isinstance(value, dict):
            value = value.get(key)
        elif isinstance(value, list) and isinstance(key, int) and -len(value) <= key < len(value):
            value = value[key]
        else:
            return None
    return value


def original_value(view: dict, tag: str, sources: dict):
    """선언된 스칼라가 원본 기록에 실제로 있는 값인지 되짚는다.

    되짚을 규칙이 있는 태그만 검사하고, 규칙이 없으면 None을 돌려 '미검사'로 센다.
    """
    pointer = view.get('offline_source_pointer') or ''
    origin = sources[view['offline_source']['path']]
    family = view.get('family')
    if family == 'zone-own-perception' and pointer.startswith('results.'):
        belief, judgment = view['belief'], view['judgment']
        if judgment == 'all' or tag.startswith('gate/'):
            return None  # 코호트 합계·게이트 판정은 이 스냅샷이 사전 등록 기준으로 계산한 값이다
        scope = 'tick' if tag.startswith('offline/tick/') else 'view'
        rest = tag.replace('offline/tick/', '', 1) if scope == 'tick' else tag
        rest = rest[len('offline/'):] if rest.startswith('offline/') else None
        if rest is None:
            return None
        stats = origin['results'][belief][scope][judgment]
        rename = {'views': 'n', 'confident_errors': 'false_confident',
                  'confident_error_rate': 'false_confident_rate'}
        parts = rest.split('/')
        if parts[0] == 'case':
            stats, parts = stats['by_case'][parts[1]], parts[2:]
        elif parts[0] in ('observable', 'not_observable'):
            stats, parts = stats[parts[0]], parts[1:]
        elif parts[0] == 'peer_in_lane':
            stats, parts = stats['peer_in_lane_reported_separately'], parts[1:]
        elif parts[0] == 'mapped_wall':
            stats, parts = stats['mapped_wall_view'], parts[1:]
        if len(parts) != 1:
            return None
        return stats.get(rename.get(parts[0], parts[0]))
    if family == 'noslip-side-effects' and pointer.startswith('result.json'):
        if tag.startswith('offline/applied/'):
            return origin['applied_solver_options'][tag.split('/')[-1]]
        if tag in ('offline/sim_time_s', 'offline/physics_steps', 'offline/hold_s', 'offline/limit_s'):
            return origin[tag.split('/')[-1]]
        if tag.startswith('gate/hard/'):
            _, _, name, field = tag.split('/')
            gate = next(g for g in origin['hard_gates'] if g['gate'] == name)
            return int(gate['ok']) if field == 'ok' else gate[field]
        if tag.startswith('offline/segment/') or tag.startswith('offline/drop/') \
                or tag.startswith('offline/host/'):
            return None
        if tag in ('offline/dropped', 'offline/lifted_clear'):
            return int(origin['metrics'][tag.split('/')[-1]])
        path = [int(p) if p.isdigit() else p for p in tag[len('offline/'):].split('/')]
        return leaf(origin['metrics'], path)
    if family == 'noslip-side-effects' and pointer.startswith('scenarios.'):
        parts = pointer.split('.')
        scenario = parts[1]
        row = origin['scenarios'][scenario] if len(parts) == 2 else \
            origin['scenarios'][scenario]['seeds'][parts[3]]
        if tag == 'offline/gates_total' and len(parts) > 2:
            return row['gates']
        if tag == 'offline/gates_failed' and len(parts) > 2:
            return row['failed']
        if tag == 'gate/hard_gates_ok' and len(parts) > 2:
            return int(row['hard_gates_ok'])
        return None
    return None


def verify_run(snapshot: Path, entry: dict, sources: dict) -> dict:
    run = snapshot / entry['name']
    manifest = load(run / 'manifest.json')
    view = load(Path(entry['source']) / 'result.json')
    report = {'run': entry['name'], 'complete': manifest.get('complete') is True,
              'scalar_mismatches': [], 'origin_mismatches': [], 'origin_checked': 0,
              'series': {}, 'texts': 0, 'hparam_session': False, 'source_files_ok': True}
    for relative, record in manifest['source_files'].items():
        path = Path(manifest['source']) / relative
        if not path.is_file() or sha256(path) != record['sha256']:
            report['source_files_ok'] = False
    for declared in (manifest['metadata'].get('offline_source'),
                     (manifest['metadata'].get('offline_scalars') or {}).get('series')):
        if declared:
            path = Path(declared['path'])
            if not path.is_file() or sha256(path) != declared['sha256']:
                report['source_files_ok'] = False
    accumulator = accumulate(run)
    tags = accumulator.Tags()
    report['texts'] = len(tags.get('tensors') or [])
    report['hparam_session'] = any('hparams' in t for t in (tags.get('tensors') or []))
    declared_scalars = view['offline_scalars']
    for tag, expected in declared_scalars.items():
        events = accumulator.Scalars(tag)
        if len(events) != 1 or not close(events[0].value, expected):
            report['scalar_mismatches'].append(
                {'tag': tag, 'events': len(events),
                 'value': events[0].value if events else None, 'declared': expected})
        recorded = original_value(view, tag, sources)
        if recorded is not None:
            report['origin_checked'] += 1
            if not close(expected, recorded):
                report['origin_mismatches'].append(
                    {'tag': tag, 'declared': expected, 'original': recorded})
    for field, standard in (('sim_s', 'result/sim_s'), ('wall_s', 'result/wall_s'),
                            ('commands', 'result/commands'), ('model_calls', 'result/model_calls')):
        if field in view:
            events = accumulator.Scalars(standard)
            if len(events) != 1 or not close(events[0].value, view[field]):
                report['scalar_mismatches'].append({'tag': standard, 'declared': view[field],
                                                    'events': len(events)})
    if view.get('success') is not None:
        events = accumulator.Scalars('evaluation/reported_success')
        if len(events) != 1 or events[0].value != int(view['success']):
            report['scalar_mismatches'].append({'tag': 'evaluation/reported_success',
                                               'declared': view['success'], 'events': len(events)})
    series = view.get('offline_series')
    if series:
        trace = Path(series['source']['path'])
        rows = [json.loads(line) for line in trace.read_text().splitlines() if line.strip()]
        report['series'] = {'rows': len(rows), 'tags': {}, 'mismatches': []}
        indices = sorted({0, len(rows) // 2, len(rows) - 1})
        for spec in series['tags']:
            events = accumulator.Scalars(spec['tag'])
            report['series']['tags'][spec['tag']] = len(events)
            if len(events) != len(rows):
                report['series']['mismatches'].append(
                    {'tag': spec['tag'], 'events': len(events), 'rows': len(rows)})
                continue
            for index in indices:
                value = leaf(rows[index], spec['path'])
                if spec.get('reduce') == 'sum' and isinstance(value, list):
                    value = sum(value)
                if not close(events[index].value, value):
                    report['series']['mismatches'].append(
                        {'tag': spec['tag'], 'step': index, 'event': events[index].value,
                         'original': value})
        clock = accumulator.Scalars('execution/sim_time_s')
        report['series']['sim_time_points'] = len(clock)
        for index in indices:
            if not close(clock[index].value, rows[index][series['sim_time_field']]):
                report['series']['mismatches'].append(
                    {'tag': 'execution/sim_time_s', 'step': index, 'event': clock[index].value,
                     'original': rows[index][series['sim_time_field']]})
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--index', type=Path, required=True, help='파생 뷰 index.json')
    parser.add_argument('--report', type=Path, help='검증 결과 JSON 경로')
    args = parser.parse_args()
    snapshot, index = args.snapshot.resolve(), load(args.index.resolve())
    collection = load(snapshot / 'collection.json')
    names = {row['run'] for row in index['rows']}
    exported = {entry['name'] for entry in collection['exported']}
    reports, sources = [], {}
    for entry in collection['exported']:
        view = load(Path(entry['source']) / 'result.json')
        path = view['offline_source']['path']
        if path not in sources:
            sources[path] = load(Path(path))
    for entry in collection['exported']:
        reports.append(verify_run(snapshot, entry, sources))
    problems = [r for r in reports if not r['complete'] or r['scalar_mismatches']
                or r['origin_mismatches'] or not r['source_files_ok']
                or (r['series'].get('mismatches') if r['series'] else False)
                or not r['hparam_session']]
    summary = {
        'snapshot': str(snapshot),
        'runs_in_index': len(names), 'runs_exported': len(exported),
        'runs_missing': sorted(names - exported), 'runs_extra': sorted(exported - names),
        'failed_exports': collection.get('failed') or [],
        'scalars_checked': sum(len(load(Path(e['source']) / 'result.json')['offline_scalars'])
                               for e in collection['exported']),
        'scalars_traced_to_original': sum(r['origin_checked'] for r in reports),
        'series_points': sum(sum(r['series']['tags'].values()) + r['series'].get('sim_time_points', 0)
                             for r in reports if r['series']),
        'runs_with_problems': [r['run'] for r in problems],
        'all_complete': all(r['complete'] for r in reports),
        'all_hparams': all(r['hparam_session'] for r in reports),
        'all_sources_unchanged': all(r['source_files_ok'] for r in reports),
        'pass': not problems and not (collection.get('failed') or []) and names == exported,
        'runs': reports,
    }
    if args.report:
        args.report.write_text(json.dumps(summary, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'runs'}, ensure_ascii=False, indent=1))
    return 0 if summary['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
