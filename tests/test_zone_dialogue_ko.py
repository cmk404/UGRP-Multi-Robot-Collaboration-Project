"""Offline Korean dialogue pilot: prompt builders and evaluation-only metrics."""
import base64
import json
import re

import pytest

from harness import zone_dialogue_ko as z
from harness import zone_dialogue_metrics as m

INSTRUCTION = next(iter(z.KO_INSTRUCTIONS))
GOAL = '{"A": {"red": 2}, "B": {"cyan": 2}, "C": {"green": 1, "yellow": 1}}'
IMAGES = ('CURRENT OWN RGB, TOP_SW (pickup, south), TOP_NW (pickup, north), TOP_SE (zones, south) '
          'and TOP_NE (zones, north)')
LABELS = ('cyan-1', 'cyan-2', 'green-1', 'red-1', 'red-2', 'yellow-1')


def _uri(seed):
    return 'data:image/jpeg;base64,' + base64.b64encode(bytes([255, 216, seed]) * 7).decode()


def _request(mode='claim', tie=True, rid='r2'):
    facts = z.SystemFacts(mode, rid, INSTRUCTION, GOAL, IMAGES, tie)
    user = {'request_id': f'zone-x-{rid}-claim-1-1-1', 'robot_id': rid,
            'box_labels': {k: {'kind': k.split('-')[0], 'rgb_floor_xy_m': [0.0, 0.0]} for k in LABELS},
            'peer_messages': [{'from_robot': 'r3', 'message': 'Claiming cyan-1 for zone B.', 'turn': 1}],
            'conflict': [{'box': 'cyan-1', 'kind': 'same_box', 'robots': ['r2', 'r3']}]}
    if mode == 'plan':
        user['agreement'] = {'proposal': None, 'proposer': 'r1'}
    return {'request_id': user['request_id'],
            'messages': [{'role': 'system', 'content': z.render_en(facts)},
                         {'role': 'user', 'content': json.dumps(user, sort_keys=True)}],
            'images': [{'label': lab, 'image': _uri(i)} for i, lab in
                       enumerate(('CURRENT OWN RGB', 'TOP_SW', 'TOP_NW', 'TOP_SE', 'TOP_NE'))]}


@pytest.mark.parametrize('mode,tie', [('claim', True), ('claim', False), ('plan', False)])
def test_recorded_prompt_round_trips_exactly(mode, tie):
    req = _request(mode, tie)
    facts = z.parse_recorded_system(req['messages'][0]['content'])
    assert (facts.mode, facts.tie_break, facts.goal, facts.images) == (mode, tie, GOAL, IMAGES)
    assert z.render_en(facts) == req['messages'][0]['content']


def test_unknown_prompt_is_rejected():
    req = _request()
    with pytest.raises(ValueError):
        z.parse_recorded_system(req['messages'][0]['content'] + ' Be nice.')


@pytest.mark.parametrize('variant', z.VARIANTS)
@pytest.mark.parametrize('mode', ['claim', 'plan'])
def test_images_user_json_and_request_id_unchanged(variant, mode):
    req = _request(mode, mode == 'claim')
    out = z.build_variant(req, variant)
    assert out['images'] == req['images']
    assert z.image_digests(out) == z.image_digests(req)
    assert out['messages'][1] == req['messages'][1]
    assert out['request_id'] == req['request_id']
    if variant == 'V0':
        assert out == req


@pytest.mark.parametrize('variant', z.VARIANTS)
@pytest.mark.parametrize('mode', ['claim', 'plan'])
def test_literal_ids_and_keys_preserved(variant, mode):
    text = z.build_variant(_request(mode, mode == 'claim'), variant)['messages'][0]['content']
    for token in ('r1', 'r2', 'r3', GOAL, '"A|B|C"', 'CURRENT OWN RGB', 'TOP_SW', 'TOP_NW', 'TOP_SE',
                  'TOP_NE', '"request_id"', '"reason"', '"message"', 'null', 'rgb_view', 'box_labels'):
        assert token in text, (variant, token)
    keys = ('"claim"', '"box"', '"zone"') if mode == 'claim' else (
        '"proposal_id"', '"plan_hash"', '"accept"', '"plan"', '"assignments"', 'true|false')
    for token in keys:
        assert token in text, (variant, token)


def test_v2_is_korean_and_keeps_only_literals_in_latin():
    text = z.build_variant(_request(), 'V2')['messages'][0]['content']
    ratio = m.hangul_ratio(text, LABELS, extra=('rgb_view', 'box_labels', 'team_board', 'peer_messages',
        'pickup_boxes_still_visible', 'zone_counts_seen', 'active', 'conflict', 'robot_id', 'kind',
        'request_id', 'claim', 'box', 'zone', 'reason', 'message', 'label'))
    assert ratio is not None and ratio > 0.97
    # No English prose sentences survive in V2.
    assert not re.search(r'\b(the|and|you|your|with|must|robot)\b', text)


@pytest.mark.parametrize('variant', z.VARIANTS)
def test_no_scripted_phrases_or_added_rules(variant):
    base = _request('claim', tie=False)
    text = z.build_variant(base, variant)['messages'][0]['content']
    assert 'lowest robot_id' not in text and '가장 낮은' not in text
    for cue in ('e.g.', 'for example', 'example', '예:', '예시', '예를 들', 'say "', '라고 말'):
        assert cue not in text.lower(), (variant, cue)
    # Nothing added beyond a language rule / structured schema: no quoted Korean utterance.
    assert not re.search(r'"[^"]*[가-힣][^"]*(합니다|할게|하겠)[^"]*"', text)


def test_tie_break_kept_only_when_recorded():
    with_tie = z.build_variant(_request('claim', True), 'V2')['messages'][0]['content']
    assert 'robot_id가 가장 낮은' in with_tie
    without = z.build_variant(_request('claim', False), 'V2')['messages'][0]['content']
    assert '가장 낮은' not in without
    facts = z.parse_recorded_system(_request('claim', False)['messages'][0]['content'])
    with pytest.raises(ValueError):
        z.render_ko(facts, tie_break=True)


def test_v1_changes_only_a_language_rule():
    req = _request()
    v1 = z.build_variant(req, 'V1')['messages'][0]['content']
    assert v1.startswith(req['messages'][0]['content'])
    assert v1[len(req['messages'][0]['content']):] == z.EN_KOREAN_MESSAGE_RULE


def test_v3_structured_schema():
    text = z.build_variant(_request(), 'V3')['messages'][0]['content']
    for field in z.STRUCT_FIELDS:
        assert f'"{field}"' in text
    assert 'brief message to\npeers' not in text
    labels = [i['label'] for i in _request()['images']]
    ok = {'act': 'yield', 'item': 'cyan-1', 'zone': 'B', 'role': None, 'passage': None,
          'observed_at': 'TOP_SW', 'confidence': 0.8, 'reply_to': {'from_robot': 'r3', 'turn': 1}}
    assert z.validate_struct_message(ok, labels) == ok
    assert z.validate_struct_message(None, labels) is None
    for bad in ({**ok, 'act': 'chat'}, {**ok, 'text': 'hi'}, {**ok, 'confidence': 2},
                {**ok, 'observed_at': 'MY CAMERA'}, 'free text'):
        with pytest.raises(ValueError):
            z.validate_struct_message(bad, labels)


def test_dialogue_request_relays_only_received_utterances():
    req = _request('claim', tie=True)
    window = {'max_turns': 6, 'per_robot': 2}
    received = [{'from_robot': 'r1', 'to': ['r2'], 'turn': 1, 'message': 'r1은 red-1을 A로 옮깁니다.'}]
    out = z.dialogue_request(req, window=window, received=received, sent=[], turn=2)
    user = json.loads(out['messages'][1]['content'])
    original = json.loads(req['messages'][1]['content'])
    assert user['dialogue_window']['received'] == received
    assert user['dialogue_window']['your_turns_left'] == 2
    assert {k: v for k, v in user.items() if k not in ('dialogue_window', 'request_id')} == \
        {k: v for k, v in original.items() if k != 'request_id'}
    assert out['images'] == req['images']
    system = out['messages'][0]['content']
    assert '가장 낮은' not in system          # tie-break dropped in windows unless asked
    assert '"recipients"' in system and '최대\n6번' in system
    v3 = z.dialogue_request(req, window=window, received=[], sent=[], turn=1, variant='V3')
    assert '"reply_to"' in v3['messages'][0]['content']


def test_metrics_hangul_ratio_code_switching_and_id_issues():
    assert m.hangul_ratio('r2가 cyan-1을 B 구역으로 옮깁니다.', LABELS) == 1.0
    assert m.hangul_ratio('Claiming cyan-1 for zone B.', LABELS) == 0.0
    assert m.hangul_ratio('', LABELS) is None and m.hangul_ratio('r1 cyan-1 A', LABELS) is None
    mixed = 'cyan-1은 제가 pick up 하겠습니다'
    assert 0 < m.hangul_ratio(mixed, LABELS) < 1
    assert m.english_words(mixed, LABELS) == ['pick', 'up']
    issues = m.id_issues('로봇 2가 시안-1과 cyan-9를 에이 구역으로, R3도', LABELS)
    assert {i['kind'] for i in issues} == {'robot_id_variant', 'translated_label', 'unknown_label',
                                          'translated_zone'}
    assert m.id_issues('r2가 cyan-1을 A 구역으로', LABELS) == []


def test_metrics_dialogue_acts():
    assert m.dialogue_acts('') == ['silence']
    assert 'yield' in m.dialogue_acts('cyan-1은 r2에게 양보하고 green-1을 맡겠습니다.')
    assert 'claim' in m.dialogue_acts('cyan-1은 r2에게 양보하고 green-1을 맡겠습니다.')
    assert 'question' in m.dialogue_acts('제가 red-2를 가져가도 될까요?')
    assert m.references('r3, 알겠습니다.', 'cyan-1 가져갈게요', 'r3', LABELS)
    assert m.references('cyan-1은 양보합니다', 'cyan-1 가져갈게요', 'r3', LABELS)
    assert not m.references('green-1을 맡겠습니다', 'cyan-1 가져갈게요', 'r3', LABELS)


def test_act_rule_versions_are_explicit():
    assert m.ACT_RULES_VERSIONS == ('v1', 'v2') and m.ACT_RULES_VERSION == 'v2'
    assert m.ACT_RULES is m.ACT_RULES_V1                      # frozen rules that labelled results.v1
    assert set(m.ACT_RULES_V1) == set(m.ACT_RULES_V2) == set(m.ACT_LABELS) == set(m.ACT_DEFINITIONS)
    with pytest.raises(ValueError):
        m.dialogue_acts('아무 말', rules='v3')


def test_structured_and_free_text_act_vocabularies_are_consistent():
    assert m.STRUCTURED_ACTS == z.ACTS                        # V3 schema enum, not edited by the metrics
    assert set(m.STRUCTURED_ACTS) < set(m.ACT_LABELS)
    assert tuple(sorted(set(m.ACT_LABELS) - set(m.STRUCTURED_ACTS))) == tuple(sorted(m.ACTS_FREE_TEXT_ONLY))
    assert m.struct_acts(None) == ['silence'] and m.struct_acts('free text') == ['invalid']
    assert m.struct_acts({'act': 'yield'}) == ['yield'] and m.struct_acts({'act': 'chat'}) == ['other']


@pytest.mark.parametrize('text,expected', [
    # a mention of someone else's act is not the act (v1 called these `propose`)
    ('Accepted proposal. Ready to execute.', ['agree']),
    ('Plan looks great and balanced, accepting proposal.', ['agree']),
    ("Standing by for r1's proposal.", ['standby']),
    ('r1의 제안을 수락합니다.', ['agree']),
    # proposing, including submitting one's own proposal
    ('Proposing balanced plan: r1 takes C, r2 takes A.', ['propose']),
    ('Confirming my proposal: 2 boxes per robot.', ['propose']),
    ('r1의 제안대로 계획을 확정하여 제출합니다.', ['propose']),
    # negation
    ('I disagree with that plan.', ['refuse']),
    ('I do not agree with r1.', ['refuse']),
    ('r1의 제안에 동의하지 않습니다.', ['refuse']),
    ('아직 완료하지 않았습니다.', ['other']),
    # asking a peer to accept is a request, not agreement
    ('계획을 올립니다. 확인 후 동의해 주세요.', ['propose', 'request']),
    # own claim vs a third-person assignment
    ('I will take red-1 to zone A.', ['claim']),
    ('cyan-1 is claimed by r2, so I take green-1.', ['claim']),
    ('red-1 배달 완료했습니다.', ['report']),
])
def test_act_rules_v2_separates_act_from_mention_and_negation(text, expected):
    assert m.dialogue_acts(text) == expected


@pytest.mark.parametrize('text,v1', [
    ('Accepted proposal. Ready to execute.', ['propose', 'agree']),
    ("Standing by for r1's proposal.", ['propose', 'standby']),
    ('I disagree with that plan.', ['agree']),
])
def test_act_rules_v1_still_reproduces_the_recorded_labels(text, v1):
    assert m.dialogue_acts(text, rules='v1') == v1

