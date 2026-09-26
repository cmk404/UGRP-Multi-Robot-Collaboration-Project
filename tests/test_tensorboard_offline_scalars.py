"""파생 뷰가 선언한 오프라인 평가 수치만 스칼라로 내보내는지 확인한다.

원본 기록의 경로·SHA-256을 다시 확인하고, 태그·값·범위 설명이 없으면 변환을
거부한다. 이 수치는 오프라인 측정이며 로봇 임무 성공이 아니다.
"""
import hashlib
import json
from pathlib import Path

import pytest

from scripts.tensorboard_tools.export import offline_scalars


def origin(tmp_path: Path, payload=None):
    path = tmp_path / 'results.json'
    path.write_text(json.dumps(payload if payload is not None else {'pos_m': 0.0912}))
    return path, hashlib.sha256(path.read_bytes()).hexdigest()


def view(tmp_path: Path, **overrides):
    path, digest = origin(tmp_path)
    result = {'derived_view_only': True,
              'offline_source': {'path': str(path), 'sha256': digest},
              'offline_scalar_scope': 'recorded localisation quantile; not robot success',
              'offline_scalars': {'offline/pos_err_m': 0.0912, 'offline/frames': 104}}
    result.update(overrides)
    return result


@pytest.fixture
def export_api():
    pytest.importorskip('tensorboard')
    pytest.importorskip('PIL')
    from scripts.tensorboard_tools.export import convert
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    return convert, EventAccumulator


def test_absent_declaration_keeps_the_previous_behaviour(tmp_path):
    assert offline_scalars({'success': True}) == ({}, None)


def test_declared_values_are_read_with_original_hash(tmp_path):
    values, provenance = offline_scalars(view(tmp_path))
    assert values == {'offline/pos_err_m': 0.0912, 'offline/frames': 104}
    assert provenance['tags'] == ['offline/frames', 'offline/pos_err_m']
    assert provenance['original']['sha256'] == origin(tmp_path)[1]
    assert 'not robot success' in provenance['scope']


@pytest.mark.parametrize('overrides', [
    {'derived_view_only': False},
    {'offline_scalar_scope': '  '},
    {'offline_scalars': {'result/sim_s': 1.0}},
    {'offline_scalars': {'offline/Pos': 1.0}},
    {'offline_scalars': {'offline/pos_err_m': 'low'}},
    {'offline_scalars': {'offline/pass': True}},
    {'offline_scalars': {'offline/pos_err_m': float('inf')}},
])
def test_unverifiable_declarations_are_refused(tmp_path, overrides):
    with pytest.raises(ValueError):
        offline_scalars(view(tmp_path, **overrides))


def test_changed_or_missing_original_is_refused(tmp_path):
    result = view(tmp_path)
    (tmp_path / 'results.json').write_text(json.dumps({'pos_m': 0.5}))
    with pytest.raises(ValueError):
        offline_scalars(result)
    (tmp_path / 'results.json').unlink()
    with pytest.raises(ValueError):
        offline_scalars(result)
    missing = view(tmp_path)
    missing['offline_source'] = {'path': str(tmp_path / 'results.json')}
    with pytest.raises(ValueError):
        offline_scalars(missing)


def test_snapshot_shows_offline_scalars_condition_and_provenance(tmp_path, export_api):
    convert, EA = export_api
    source = tmp_path / 'loc-test-G1'
    source.mkdir()
    record = view(tmp_path)
    record.update(success=False, stop_reason='gate_evaluated', case='G1_door_stop_look_p90',
                  condition='owncam localisation preregistered test', clock='offline frames',
                  scope='offline evaluation of recorded frames; no SIM ran')
    before = (tmp_path / 'results.json').read_bytes()
    (source / 'result.json').write_text(json.dumps(record, ensure_ascii=False))
    manifest = convert(source, tmp_path / 'export', max_images=0)
    accumulator = EA(str(tmp_path / 'export')).Reload()
    assert accumulator.Scalars('offline/pos_err_m')[0].value == pytest.approx(0.0912)
    assert accumulator.Scalars('offline/frames')[0].value == 104
    assert accumulator.Scalars('evaluation/reported_success')[0].value == 0
    assert 'provenance/offline_scalars' in accumulator.Tags()['tensors']
    assert manifest['metadata']['condition'] == 'owncam localisation preregistered test'
    assert manifest['metadata']['offline_scalars']['original']['sha256'] == hashlib.sha256(before).hexdigest()
    assert (tmp_path / 'results.json').read_bytes() == before


def test_declaration_without_a_hashed_original_publishes_no_events(tmp_path, export_api):
    convert, _ = export_api
    source = tmp_path / 'broken'
    source.mkdir()
    record = view(tmp_path)
    record['offline_source'] = {'path': str(tmp_path / 'results.json'), 'sha256': 'f' * 64}
    (source / 'result.json').write_text(json.dumps(record))
    output = tmp_path / 'export-broken'
    with pytest.raises(ValueError):
        convert(source, output, max_images=0)
    assert json.loads((output / 'manifest.json').read_text())['complete'] is False
    assert not list(output.glob('events.out.tfevents.*'))
