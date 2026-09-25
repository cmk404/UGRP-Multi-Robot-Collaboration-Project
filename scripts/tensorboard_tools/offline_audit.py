"""오프라인 감사·채점 기록을 TensorBoard 파생 뷰로 읽는 변환기.

`scripts/tensorboard_tools/export.py`는 로봇 실행·학습 기록을 변환한다. 이 모듈은 그 파일을
고치지 않고, **오프라인 인식 채점·접촉 프로필 A/B 감사처럼 실행 기록이 아닌 평가 산출물**을
같은 안전 규칙으로 변환한다(원본 불변, 기존 폴더 덮어쓰기 거부, manifest에 원본 SHA-256 기록).

읽는 것은 파생 뷰 1폴더의 `result.json` 하나다. 파생 뷰는 **어떤 수치를 보여줄지 스스로 선언**하고,
모든 수치는 해시로 고정한 원본 기록에 이미 있는 값이어야 한다. 원본이 없거나 해시가 다르면
변환을 거부한다. 시계열은 파생 뷰가 지정한 원본 JSONL의 표본을 그대로 옮기며 보간하지 않는다.

이 변환기가 만드는 숫자는 **오프라인 측정**이다. 로봇 임무 성공·실물 성능·실행 시간이 아니다.
`evaluation/reported_success`는 파생 뷰가 명시한 bool 게이트 판정이며 그 정의를 함께 기록한다.

PR #183이 `export.py`에 넣은 `offline_scalars` 선언 형식과 같은 키를 쓴다. 그 변경이 main에
들어오면 이 모듈은 은퇴하고 표준 변환기 하나로 합칠 수 있다(중복 경로를 남기지 않기 위한 전환용).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import tempfile
import time

from scripts.tensorboard_tools.export import (
    HP_METRICS, MAX_BYTES, Source, Writer, exporter_version, finite, inside, obj,
    redact, sha, stable_digest,
)

SCHEMA = 'ugrp.offline_audit_view.v1'
# 파생 뷰가 선언할 수 있는 스칼라 태그. 오프라인 측정(offline/)과 사전 등록 게이트(gate/)만 허용한다.
# 구역 문자(A/B/C)와 로봇 ID는 원본 그대로 두므로 경로 조각에 대문자를 허용한다.
SEGMENT = '[A-Za-z0-9_]+'
SCALAR_TAG = re.compile(rf'(?:offline|gate)/{SEGMENT}(?:/{SEGMENT}){{0,5}}')
# 원본 표본을 그대로 옮기는 시계열 태그.
SERIES_TAG = re.compile(rf'trace/{SEGMENT}(?:/{SEGMENT}){{0,2}}')
MAX_SERIES_ROWS = 40000
HEX64 = re.compile(r'[0-9a-f]{64}')

# 파생 뷰가 표준 이름으로 옮길 수 있는 실행 지표. 없는 값은 0으로 채우지 않는다.
STANDARD = {'sim_s': 'result/sim_s', 'wall_s': 'result/wall_s',
            'commands': 'result/commands', 'model_calls': 'result/model_calls',
            'model_latency_s': 'result/model_latency_s'}
HPARAM_KEYS = ('family', 'policy', 'case', 'condition', 'split', 'judgment', 'belief',
               'scenario', 'contact_profile', 'seed', 'outcome', 'source_sha', 'run_id')


def hashed_original(spec, what):
    """선언된 원본 파일을 해시까지 확인하고 (경로, 통계)를 돌려준다."""
    spec = obj(spec)
    path, digest = spec.get('path'), spec.get('sha256')
    if not isinstance(path, str) or not isinstance(digest, str) or not HEX64.fullmatch(digest):
        raise ValueError(f'{what} requires an absolute path and sha256')
    original = Path(path)
    if not original.is_absolute():
        raise ValueError(f'{what} requires an absolute path and sha256: {path}')
    if original.is_symlink() or not original.is_file():
        raise ValueError(f'{what} missing or linked: {path}')
    if original.stat().st_size > MAX_BYTES:
        raise ValueError(f'{what} exceeds 64 MiB: {path}')
    if sha(original.read_bytes()) != digest:
        raise ValueError(f'{what} changed since the derived view was built: {path}')
    stat = original.stat()
    return original, {'path': str(original), 'sha256': digest, 'size': stat.st_size,
                      'mtime_s': stat.st_mtime}


def leaf(row, path):
    """행에서 선언된 경로의 값을 꺼낸다. 없는 값은 None이며 만들어내지 않는다."""
    value = row
    for key in path:
        if isinstance(value, dict):
            value = value.get(key)
        elif isinstance(value, list) and isinstance(key, int) and -len(value) <= key < len(value):
            value = value[key]
        else:
            return None
    return value


def export_series(w, spec):
    """파생 뷰가 지정한 원본 JSONL 표본을 그대로 옮긴다(보간·재계산 없음)."""
    spec = obj(spec)
    original, provenance = hashed_original(spec.get('source'), 'offline_series source')
    if spec.get('format') != 'jsonl':
        raise ValueError('offline_series supports the recorded jsonl format only')
    rows = [json.loads(line) for line in original.read_bytes().decode().splitlines() if line.strip()]
    if not rows or len(rows) > MAX_SERIES_ROWS:
        raise ValueError(f'offline_series row count out of range: {len(rows)}')
    time_field = spec.get('sim_time_field')
    tags = spec.get('tags')
    if not isinstance(tags, list) or not tags:
        raise ValueError('offline_series requires at least one tag')
    relative = obj(spec.get('relative_time')) or None
    if relative and not (SERIES_TAG.fullmatch(str(relative.get('tag', ''))) and finite(relative.get('origin_s'))):
        raise ValueError('offline_series relative_time requires a trace/ tag and a recorded origin_s')
    emitted = {}
    for step, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError('offline_series rows must be objects')
        if isinstance(time_field, str):
            value = row.get(time_field)
            if finite(value):
                w.scalar('execution/sim_time_s', value, step)
                emitted['execution/sim_time_s'] = emitted.get('execution/sim_time_s', 0) + 1
        for entry in tags:
            entry = obj(entry)
            tag, path = str(entry.get('tag', '')), entry.get('path')
            if not SERIES_TAG.fullmatch(tag) or not isinstance(path, list) or not path:
                raise ValueError('offline_series tag must be trace/<name> with a recorded path')
            value = leaf(row, path)
            if entry.get('reduce') == 'sum':
                if isinstance(value, list) and value and all(finite(v) for v in value):
                    value = sum(value)
                else:
                    value = None
            if finite(value):
                w.scalar(tag, value, step)
                emitted[tag] = emitted.get(tag, 0) + 1
        if relative and row.get(spec.get('phase_field')) == relative.get('phase'):
            value = row.get(time_field)
            if finite(value):
                w.scalar(relative['tag'], value - relative['origin_s'], step)
                emitted[relative['tag']] = emitted.get(relative['tag'], 0) + 1
    provenance.update(rows=len(rows), points={k: v for k, v in sorted(emitted.items())},
                      step_axis='원본 JSONL 표본 순번입니다. SIM 시각은 execution/sim_time_s에 있습니다.',
                      relative_time=relative,
                      limit='기록된 표본을 그대로 옮겼습니다. 보간·평활·재계산은 하지 않았습니다.')
    return provenance


def export_offline_audit(src, w, view):
    """파생 뷰 1개를 이벤트로 옮기고 (메타데이터, 지표)를 돌려준다."""
    if view.get('schema') != SCHEMA:
        raise ValueError(f'not an offline audit view: {view.get("schema")}')
    if view.get('derived_view_only') is not True:
        raise ValueError('offline audit view requires derived_view_only evidence')
    scope = view.get('offline_scalar_scope')
    if not isinstance(scope, str) or not scope.strip():
        raise ValueError('offline audit view requires offline_scalar_scope text')
    original, provenance = hashed_original(view.get('offline_source'), 'offline_source')
    declared = obj(view.get('offline_scalars'))
    if not declared:
        raise ValueError('offline audit view declares no offline scalars')
    metrics = {}
    for tag, value in declared.items():
        if not SCALAR_TAG.fullmatch(str(tag)):
            raise ValueError('offline scalar tag must match offline/<name> or gate/<name>: ' + str(tag))
        if not finite(value):
            raise ValueError('offline scalar must be a finite number: ' + str(tag))
        metrics[str(tag)] = value
    for field, tag in STANDARD.items():
        if field in view:
            if not finite(view[field]):
                raise ValueError(f'{field} must be a finite number when present')
            metrics[tag] = view[field]
    success, definition = view.get('success'), view.get('success_definition')
    if success is not None:
        if type(success) is not bool or not isinstance(definition, str) or not definition.strip():
            raise ValueError('success requires a bool value and an explicit success_definition')
        metrics['evaluation/reported_success'] = int(success)
    texts = obj(view.get('texts'))
    for tag, value in texts.items():
        if not re.fullmatch(r'[a-z_]+/[a-z0-9_]+(?:/[a-z0-9_]+){0,2}', str(tag)):
            raise ValueError('text tag must be <group>/<name>: ' + str(tag))
        w.text(str(tag), value)
    series = export_series(w, view['offline_series']) if view.get('offline_series') else None
    for tag, value in metrics.items():
        w.scalar(tag, value)
    offline_provenance = {
        'scope': scope, 'tags': sorted(declared), 'original': provenance,
        'pointer': view.get('offline_source_pointer'),
        'records': view.get('records'),
        'series': series,
        'limit': ('원본 기록에 있는 오프라인 측정값입니다. 로봇 임무 성공·실물 성능·실행 시간이 아닙니다. '
                  'evaluation/reported_success는 파생 뷰가 명시한 게이트 판정이며 그 정의를 함께 봅니다.'),
    }
    w.text('provenance/offline_scalars', offline_provenance)
    w.text('result/summary', {'success': success, 'success_definition': definition,
                              'outcome': view.get('outcome'), 'stop_reason': view.get('stop_reason'),
                              'scope': view.get('scope'), 'limits': view.get('limits')})
    if view.get('evaluation') is not None:
        w.text('evaluation/referee_only', view['evaluation'])
    meta = {k: view.get(k) for k in ('family', 'policy', 'case', 'condition', 'split', 'judgment',
                                     'belief', 'scenario', 'contact_profile', 'seed', 'outcome',
                                     'source_sha', 'run_id', 'clock', 'goal', 'scope', 'limits',
                                     'records')}
    meta['success_source_field'] = 'declared_gate_verdict' if success is not None else None
    meta['offline_scalars'] = offline_provenance
    meta['offline_source'] = provenance
    declared_hp = view.get('hparam_metrics')
    if declared_hp is not None:
        if (not isinstance(declared_hp, list)
                or any(tag not in metrics for tag in declared_hp)):
            raise ValueError('hparam_metrics must list tags this view actually emits')
        meta['hparam_metrics'] = list(declared_hp)
    return meta, metrics


def convert(source, output):
    """파생 뷰 1개를 한 번만 내보낸다. 기존 출력 폴더는 거부한다(중복 step 방지)."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if not source.is_dir():
        raise ValueError(f'Not a derived view directory: {source}')
    if output == source or output.is_relative_to(source):
        raise ValueError('Export must be outside the source directory')
    src = Source(source)
    view = src.read('result.json', required=True)
    if not isinstance(view, dict):
        raise ValueError('Derived view result.json must be an object')
    at = time.time()
    output.mkdir(parents=True, exist_ok=False)
    temporary = tempfile.TemporaryDirectory(prefix='ugrp-offline-audit-export-')
    w = Writer(Path(temporary.name), at)
    manifest = {'schema': 'ugrp.tensorboard-export.v1', 'kind': SCHEMA, 'source': str(source),
                'exported_at_s': at, 'exporter': exporter_version(), 'complete': False,
                'event_wall_time': 'export time, not historical execution time'}
    try:
        meta, metrics = export_offline_audit(src, w, view)
        w.text('provenance/source', {
            'source_directory': str(source), 'export_time_s': at,
            'event_wall_time': '변환 시각입니다. 실행 시작·종료 시각이 아닙니다. 가로축은 STEP으로 보세요.',
            'step_axis': 'offline audit: 파생 뷰의 단일 요약은 step 0, 시계열은 원본 표본 순번',
            'source_metadata': meta, 'warnings': src.warnings,
            'limits': ('오프라인 채점·물리 감사 기록입니다. 성공률 집계·조건 동등성·실물 성능을 '
                       '자동 주장하지 않습니다. 미기록 값은 0으로 채우지 않습니다.')})
        hp = {k: str(meta.get(k) if meta.get(k) is not None else 'unrecorded') for k in HPARAM_KEYS}
        hp['condition_fingerprint'] = stable_digest(
            {k: meta.get(k) for k in ('family', 'case', 'condition', 'split', 'judgment', 'belief',
                                      'scenario', 'contact_profile', 'seed', 'source_sha', 'scope')})
        # HParams 표에는 파생 뷰가 고른 핵심 태그만 등록한다(전부 등록하면 표가 읽히지 않는다).
        curated = meta.get('hparam_metrics')
        if curated is None:
            curated = sorted(k for k in metrics if k.startswith(('offline/', 'gate/')))
        w.hparams(hp, HP_METRICS + tuple(curated))
        manifest.update(metadata=meta, source_files=src.files, warnings=src.warnings,
                        counts=w.counts, hparams=hp)
        w.close()
        for relative, record in src.files.items():
            current = inside(src.root, relative)
            if current is None or sha(current.read_bytes()) != record['sha256']:
                raise ValueError(f'Source changed during export: {relative}; no event file published')
        for declared in (meta.get('offline_source'), obj(meta.get('offline_scalars')).get('series')):
            declared = obj(declared)
            if declared:
                current = Path(declared['path'])
                if not current.is_file() or sha(current.read_bytes()) != declared['sha256']:
                    raise ValueError('Original changed during export: ' + declared['path'])
        for event_file in Path(temporary.name).iterdir():
            shutil.copy2(event_file, output / event_file.name)
        manifest['complete'] = True
    except Exception:
        w.close()
        manifest.update(source_files=src.files, warnings=src.warnings)
        (output / 'manifest.json').write_text(json.dumps(redact(manifest), ensure_ascii=False, indent=2) + '\n')
        raise
    finally:
        temporary.cleanup()
    (output / 'manifest.json').write_text(json.dumps(redact(manifest), ensure_ascii=False, indent=2) + '\n')
    return manifest


def main():
    p = argparse.ArgumentParser(description='오프라인 감사 파생 뷰 → TensorBoard 이벤트')
    p.add_argument('--source', type=Path, action='append', required=True,
                   help='파생 뷰 1폴더(result.json); 반복 지정 가능')
    p.add_argument('--output', type=Path, required=True, help='새 export collection 폴더')
    args = p.parse_args()
    output = args.output.resolve()
    if output.exists() and (output / 'collection.json').exists():
        raise SystemExit('기존 collection에 덧붙이지 않는다; 새 스냅샷 ID로 변환한다')
    output.mkdir(parents=True, exist_ok=True)
    exported, failed = [], []
    for source in args.source:
        name = Path(source).resolve().name
        try:
            manifest = convert(source, output / name)
            exported.append({'source': str(Path(source).resolve()), 'name': name,
                             'counts': manifest['counts']})
        except Exception as error:  # 실패도 남긴다; 하나가 실패해도 나머지는 계속 변환한다
            failed.append({'source': str(Path(source).resolve()), 'name': name,
                           'error': f'{type(error).__name__}: {error}'})
    collection = {'schema': 'ugrp.tensorboard-collection.v1', 'kind': SCHEMA,
                  'exported': exported, 'failed': failed, 'at_s': time.time()}
    (output / 'collection.json').write_text(json.dumps(collection, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'exported': len(exported), 'failed': len(failed), 'output': str(output)},
                     ensure_ascii=False))
    return 1 if failed else 0
