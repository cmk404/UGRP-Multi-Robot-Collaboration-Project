"""오프라인 감사 파생 뷰 변환기 회귀 검사.

이벤트를 TensorBoard 자체 리더로 다시 읽어 스칼라·Text·HParams가 실제로 들어갔는지, 선언하지
않은 값을 만들어내지 않는지, 원본 해시가 다르거나 변환 중 바뀌면 거부하는지 확인한다.
선택 의존성(tensorboard)이 없으면 그 검사만 건너뛴다.
"""
import hashlib
import json
from pathlib import Path

import pytest

from scripts.build_perception_noslip_views import exact, leaves, near
from scripts.tensorboard_tools.offline_audit import SCALAR_TAG, SERIES_TAG, convert


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def original(tmp_path: Path, payload=None) -> tuple[Path, str]:
    payload = {'measured': {'accuracy': 0.75, 'unknown': 3}} if payload is None else payload
    path = tmp_path / 'origin' / 'summary.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(payload).encode()
    path.write_bytes(data)
    return path, sha256(data)


def trace(tmp_path: Path) -> tuple[Path, str]:
    rows = [{'t': 1.0, 'phase': 'lift', 'z': 0.05, 'finger': [1.0, 2.0]},
            {'t': 1.5, 'phase': 'hold', 'z': 0.04, 'finger': [1.5, 1.5]},
            {'t': 2.0, 'phase': 'hold', 'z': -0.01, 'finger': [0.0, 0.0]}]
    path = tmp_path / 'origin' / 'trace.jsonl'
    path.parent.mkdir(parents=True, exist_ok=True)
    data = ''.join(json.dumps(r) + '\n' for r in rows).encode()
    path.write_bytes(data)
    return path, sha256(data)


def view(tmp_path: Path, **overrides) -> Path:
    source_path, digest = original(tmp_path)
    payload = {
        'schema': 'ugrp.offline_audit_view.v1', 'derived_view_only': True,
        'offline_source': {'path': str(source_path), 'sha256': digest},
        'offline_source_pointer': 'measured',
        'offline_scalar_scope': '오프라인 채점 수치',
        'offline_scalars': {'offline/accuracy': 0.75, 'offline/unknown': 3,
                            'offline/case/zone_A/views': 4, 'gate/g1_pass': 1},
        'family': 'test-family', 'case': 'unit', 'condition': 'unit condition',
        'outcome': 'gates_pass', 'source_sha': 'abc1234', 'seed': 11,
        'success': True, 'success_definition': '단위 검사용 게이트 정의',
        'scope': '검사용', 'limits': '검사용',
        'texts': {'evaluation/detail': {'note': 'ok'}},
    }
    payload.update(overrides)
    directory = tmp_path / 'derived' / 'run'
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'result.json').write_text(json.dumps(payload, ensure_ascii=False))
    return directory


def accumulate(run: Path):
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    accumulator = EventAccumulator(str(run), size_guidance={'scalars': 1000, 'tensors': 100})
    accumulator.Reload()
    return accumulator


def test_tag_patterns_allow_literal_zone_letters_and_reject_other_namespaces():
    assert SCALAR_TAG.fullmatch('offline/case/placed_zone_A/views')
    assert SCALAR_TAG.fullmatch('gate/finger_force_r1/diff')
    assert SCALAR_TAG.fullmatch('offline/ab/hold_slip_mm/r1/base')
    assert not SCALAR_TAG.fullmatch('result/sim_s')
    assert not SCALAR_TAG.fullmatch('offline/with space')
    assert not SCALAR_TAG.fullmatch('offline')
    assert SERIES_TAG.fullmatch('trace/finger_total_n/r1')
    assert not SERIES_TAG.fullmatch('offline/finger')


def test_builder_helpers_skip_unsafe_keys_and_non_numbers():
    out, skipped = {}, []
    leaves({'ok': 1, 'flag': True, 'text': 'x', 'nested': {'zone_A': 2.5, 'bad key': 3},
            'list': [1, None]}, 'offline', out, skipped)
    assert out == {'offline/ok': 1, 'offline/nested/zone_A': 2.5, 'offline/list/0': 1}
    assert skipped == ['offline/nested/bad key']
    assert near(0.4583, 0.458, 3) and not near(0.4583, 0.457, 3)
    assert near(None, None) and not near(0.1, None)
    assert exact(3, 3.0) and not exact(3, 4)


def test_events_carry_declared_scalars_texts_and_hparams(tmp_path):
    pytest.importorskip('tensorboard')
    source = view(tmp_path)
    output = tmp_path / 'snapshot' / 'run'
    manifest = convert(source, output)
    assert manifest['complete'] is True
    assert manifest['source_files']['result.json']['sha256']
    accumulator = accumulate(output)
    values = {tag: accumulator.Scalars(tag)[0].value for tag in accumulator.Tags()['scalars']}
    assert values['offline/accuracy'] == pytest.approx(0.75)
    assert values['offline/unknown'] == 3
    assert values['offline/case/zone_A/views'] == 4
    assert values['gate/g1_pass'] == 1
    assert values['evaluation/reported_success'] == 1
    # 선언하지 않은 표준 지표는 0으로 채우지 않는다.
    assert 'result/sim_s' not in values
    tensors = accumulator.Tags()['tensors']
    assert 'evaluation/detail' in tensors and 'provenance/offline_scalars' in tensors
    assert any('hparams' in tag for tag in tensors)
    assert manifest['hparams']['family'] == 'test-family'
    assert manifest['hparams']['contact_profile'] == 'unrecorded'


def test_standard_metrics_only_when_declared(tmp_path):
    pytest.importorskip('tensorboard')
    source = view(tmp_path, sim_s=136.9002, commands=11)
    manifest = convert(source, tmp_path / 'snapshot' / 'run')
    accumulator = accumulate(tmp_path / 'snapshot' / 'run')
    assert accumulator.Scalars('result/sim_s')[0].value == pytest.approx(136.9002)
    assert accumulator.Scalars('result/commands')[0].value == 11
    assert 'result/wall_s' not in accumulator.Tags()['scalars']
    assert manifest['metadata']['success_source_field'] == 'declared_gate_verdict'


def test_series_transfers_recorded_samples_without_interpolation(tmp_path):
    pytest.importorskip('tensorboard')
    trace_path, digest = trace(tmp_path)
    source = view(tmp_path, offline_series={
        'source': {'path': str(trace_path), 'sha256': digest}, 'format': 'jsonl',
        'sim_time_field': 't', 'phase_field': 'phase',
        'tags': [{'tag': 'trace/cargo_z_m', 'path': ['z']},
                 {'tag': 'trace/finger_total_n/r1', 'path': ['finger'], 'reduce': 'sum'}],
        'relative_time': {'tag': 'trace/hold_elapsed_s', 'origin_s': 1.5, 'phase': 'hold'}})
    convert(source, tmp_path / 'snapshot' / 'run')
    accumulator = accumulate(tmp_path / 'snapshot' / 'run')
    z = accumulator.Scalars('trace/cargo_z_m')
    assert [event.step for event in z] == [0, 1, 2]
    assert [round(event.value, 4) for event in z] == [0.05, 0.04, -0.01]
    assert [round(e.value, 4) for e in accumulator.Scalars('trace/finger_total_n/r1')] == [3.0, 3.0, 0.0]
    assert [round(e.value, 4) for e in accumulator.Scalars('execution/sim_time_s')] == [1.0, 1.5, 2.0]
    # hold 상대 시각은 hold 표본에만 있고 다른 phase를 채워 넣지 않는다.
    relative = accumulator.Scalars('trace/hold_elapsed_s')
    assert [(e.step, round(e.value, 4)) for e in relative] == [(1, 0.0), (2, 0.5)]


@pytest.mark.parametrize('overrides,message', [
    ({'schema': 'other'}, 'not an offline audit view'),
    ({'derived_view_only': False}, 'derived_view_only'),
    ({'offline_scalar_scope': '  '}, 'offline_scalar_scope'),
    ({'offline_scalars': {}}, 'declares no offline scalars'),
    ({'offline_scalars': {'result/sim_s': 1.0}}, 'offline scalar tag must match'),
    ({'offline_scalars': {'offline/a': 'text'}}, 'finite number'),
    ({'offline_scalars': {'offline/a': float('inf')}}, 'finite number'),
    ({'success_definition': None}, 'success_definition'),
    ({'texts': {'Bad/Tag': {}}}, 'text tag must be'),
    ({'hparam_metrics': ['offline/missing']}, 'hparam_metrics'),
    ({'sim_s': 'later'}, 'finite number'),
])
def test_rejects_malformed_declarations(tmp_path, overrides, message):
    pytest.importorskip('tensorboard')
    source = view(tmp_path, **overrides)
    with pytest.raises(ValueError, match=message):
        convert(source, tmp_path / 'snapshot' / 'run')
    assert not (tmp_path / 'snapshot' / 'run' / 'events.out.tfevents').exists()


def test_rejects_wrong_hash_missing_and_relative_original(tmp_path):
    pytest.importorskip('tensorboard')
    source_path, digest = original(tmp_path)
    with pytest.raises(ValueError, match='changed since the derived view was built'):
        convert(view(tmp_path, offline_source={'path': str(source_path), 'sha256': '0' * 64}),
                tmp_path / 'a')
    with pytest.raises(ValueError, match='missing or linked'):
        convert(view(tmp_path, offline_source={'path': str(tmp_path / 'absent.json'),
                                               'sha256': digest}), tmp_path / 'b')
    with pytest.raises(ValueError, match='absolute path and sha256'):
        convert(view(tmp_path, offline_source={'path': 'summary.json', 'sha256': digest}),
                tmp_path / 'c')


def test_refuses_existing_output_and_source_inside_output(tmp_path):
    pytest.importorskip('tensorboard')
    source = view(tmp_path)
    output = tmp_path / 'snapshot' / 'run'
    convert(source, output)
    with pytest.raises(FileExistsError):
        convert(source, output)
    with pytest.raises(ValueError, match='outside the source directory'):
        convert(source, source / 'inner')


def test_original_changed_during_export_leaves_failure_manifest(tmp_path, monkeypatch):
    pytest.importorskip('tensorboard')
    source_path, digest = original(tmp_path)
    source = view(tmp_path)
    from scripts.tensorboard_tools import offline_audit

    real = offline_audit.export_offline_audit

    def mutate(src, writer, payload):
        result = real(src, writer, payload)
        source_path.write_bytes(json.dumps({'measured': {'accuracy': 0.0}}).encode())
        return result

    monkeypatch.setattr(offline_audit, 'export_offline_audit', mutate)
    output = tmp_path / 'snapshot' / 'run'
    with pytest.raises(ValueError, match='Original changed during export'):
        offline_audit.convert(source, output)
    manifest = json.loads((output / 'manifest.json').read_text())
    assert manifest['complete'] is False
    assert not list(output.glob('events.out.tfevents*'))
