"""Offline Korean dialogue pilot: raw records are append-only and resume never overwrites them.

No model call and no SIM: the proxy client's opener is redirected to a fake
urlopen inside the pilot module.
"""
import json

import pytest

from scripts import pilot_korean_dialogue as pilot
from scripts import pilot_korean_dialogue_analysis as analysis

SETTINGS = {'model': 'gemini-3.8-flash', 'temperature': 0.2, 'max_tokens': 1400, 'reasoning_effort': 'none'}
REPLY = json.dumps({'request_id': 'zone-x-r1-claim-1-0-1', 'claim': {'box': 'cyan-1', 'zone': 'B'},
                    'reason': '가까움', 'message': 'cyan-1을 B로 옮기겠습니다.', 'recipients': ['r2', 'r3']},
                   ensure_ascii=False)


class _FakeResponse:
    def __init__(self, body):
        self._body = body

    def read(self):
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


@pytest.fixture
def offline(tmp_path, monkeypatch):
    """Pilot with its own raw/experiment dirs and a fake HTTP transport."""
    raw, exp = tmp_path/'raw', tmp_path/'exp'
    raw.mkdir()
    exp.mkdir()
    monkeypatch.setattr(pilot, 'RAW', raw)
    monkeypatch.setattr(pilot, 'EXP', exp)
    monkeypatch.setattr(pilot, 'ROOT', tmp_path)
    calls = []

    def fake_urlopen(request, timeout=None):
        calls.append(json.loads(request.data))
        body = {'model': 'gemini-3.8-flash', 'choices': [{'message': {'content': REPLY}}],
                'usage': {'prompt_tokens': 10, 'completion_tokens': 5, 'total_tokens': 15}}
        return _FakeResponse(json.dumps(body).encode())

    monkeypatch.setattr(pilot, 'urlopen', fake_urlopen)
    return {'raw': raw, 'exp': exp, 'http': calls}


def _request():
    return {'request_id': 'zone-x-r1-claim-1-0-1',
            'messages': [{'role': 'system', 'content': 'system text'},
                         {'role': 'user', 'content': '{"robot_id": "r1"}'}],
            'images': [{'label': 'CURRENT OWN RGB',
                        'image': 'data:image/jpeg;base64,/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAE='}]}


def _call(offline, call_id='single:p1:V2:0', limit=10):
    return pilot.call_model(_request(), call_id, SETTINGS, pilot.Budget(0, limit),
                            {'kind': 'single', 'point_id': 'p1', 'variant': 'V2', 'rep': 0})


def test_raw_wire_and_response_are_written_once(offline):
    row = _call(offline)
    wire = offline['raw']/'single_p1_V2_0-a0-wire.json'
    resp = offline['raw']/'single_p1_V2_0-a0-response.json'
    assert wire.exists() and resp.exists()
    assert row['raw_occurrence'] == 0
    assert row['wire']['body_sha256'] == pilot.sha(wire.read_bytes())
    assert row['response']['sha256'] == pilot.sha(resp.read_bytes())


def test_resume_never_overwrites_an_existing_raw_record(offline):
    first = _call(offline)
    wire = offline['raw']/(pilot.raw_stem('single:p1:V2:0', 0, 0) + '-wire.json')
    resp = offline['raw']/(pilot.raw_stem('single:p1:V2:0', 0, 0) + '-response.json')
    before = (wire.read_bytes(), resp.read_bytes())

    second = _call(offline)                      # same call id: a resumed run

    assert (wire.read_bytes(), resp.read_bytes()) == before, 'existing raw record was modified'
    assert second['raw_occurrence'] == 1
    assert second['wire']['raw_path'] != first['wire']['raw_path']
    assert second['response']['raw_path'] != first['response']['raw_path']
    new_wire = offline['raw']/(pilot.raw_stem('single:p1:V2:0', 0, 1) + '-wire.json')
    assert new_wire.exists()
    assert len(list(offline['raw'].glob('*-wire.json'))) == 2


def test_write_raw_refuses_to_overwrite(offline):
    path = offline['raw']/'kept.json'
    path.write_bytes(b'original')
    with pytest.raises(SystemExit, match='refusing to overwrite'):
        pilot.write_raw(path, b'new')
    assert path.read_bytes() == b'original'


def test_reserve_raw_stops_instead_of_reusing_names(offline, monkeypatch):
    monkeypatch.setattr(pilot, 'RAW_OCCURRENCE_LIMIT', 2)
    for occurrence in range(2):
        (offline['raw']/(pilot.raw_stem('single:p1:V2:0', 0, occurrence) + '-wire.json')).write_bytes(b'x')
    with pytest.raises(SystemExit, match='refusing to overwrite'):
        pilot.reserve_raw('single:p1:V2:0', 0)


def test_calls_jsonl_is_append_only(offline):
    _call(offline)
    _call(offline)
    rows = pilot.load_calls(offline['exp'])
    assert len(rows) == 2
    assert [r['raw_occurrence'] for r in rows] == [0, 1]
    assert len({r['wire']['raw_path'] for r in rows}) == 2


def test_interrupted_window_resumes_as_a_new_episode(offline):
    calls = [{'kind': 'dialogue', 'scenario': 'M1', 'variant': 'V2', 'turn': t, 'call_id':
              pilot.window_call_id('M1', 'V2', 1, t, 'r1')} for t in (1, 2, 3)]
    assert pilot.window_episode(calls, 'M1', 'V2') == 2          # rows without 'episode' are episode 1
    assert pilot.window_episode(calls, 'M2', 'V2') == 1
    assert pilot.window_episode(calls + [{'kind': 'dialogue', 'scenario': 'M1', 'variant': 'V2',
                                          'episode': 2}], 'M1', 'V2') == 3
    assert pilot.window_call_id('M1', 'V2', 1, 1, 'r1') == 'dialogue:M1:V2:t1:r1'   # recorded run unchanged
    assert pilot.window_call_id('M1', 'V2', 2, 1, 'r1') == 'dialogue:M1:V2:e2:t1:r1'


def test_new_episode_keeps_the_earlier_raw_files(offline):
    old = _call(offline, call_id=pilot.window_call_id('M1', 'V2', 1, 1, 'r1'))
    old_path = offline['raw']/(pilot.raw_stem(old['call_id'], 0, 0) + '-wire.json')
    kept = old_path.read_bytes()
    new = _call(offline, call_id=pilot.window_call_id('M1', 'V2', 2, 1, 'r1'))
    assert old_path.read_bytes() == kept
    assert new['wire']['raw_path'] != old['wire']['raw_path']
    assert 'e2' in new['call_id']


def test_partial_window_checkpoint_keeps_every_episode(offline):
    path = offline['exp']/'dialogues-partial.json'
    pilot.save_partial(path, {'scenario': 'M1', 'variant': 'V2', 'episode': 1, 'turns': [1, 2, 3],
                              'complete': False})
    pilot.save_partial(path, {'scenario': 'M1', 'variant': 'V2', 'episode': 2, 'turns': [1],
                              'complete': False})
    pilot.save_partial(path, {'scenario': 'M1', 'variant': 'V2', 'episode': 2, 'turns': [1, 2],
                              'complete': False})
    done = json.loads(path.read_text())
    assert [(d['episode'], len(d['turns'])) for d in done] == [(1, 3), (2, 2)]


def test_analyze_refuses_to_overwrite_recorded_results(tmp_path):
    out = tmp_path/'results.json'
    out.write_text('{"schema": "ugrp.zone_dialogue_ko_pilot.results.v1"}\n')
    with pytest.raises(SystemExit, match='already exists'):
        analysis.analyze(tmp_path, tmp_path, out=out)
    assert json.loads(out.read_text())['schema'] == 'ugrp.zone_dialogue_ko_pilot.results.v1'


def test_results_schema_is_versioned_with_the_act_rules():
    assert analysis.SCHEMA == 'ugrp.zone_dialogue_ko_pilot.results.v2'
    assert analysis.ACT_RULES == 'v2'
    assert analysis.acts_of('Accepted proposal.') == {'acts': ['agree'], 'acts_rules': 'v2',
                                                     'acts_rules_v1': ['propose', 'agree']}
