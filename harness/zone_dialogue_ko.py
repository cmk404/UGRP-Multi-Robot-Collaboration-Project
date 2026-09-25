"""Prompt variants for the OFFLINE Korean dialogue pilot on recorded zone decisions.

Offline only: this module never runs SIM and is not wired into the zone runner.
It rewrites the *system text* of a recorded ZC2 zone request (commit 7ccd6c3,
``harness/zone_coordination.py``) into prompt variants and leaves the user JSON
string and every image byte-identical:

V0  the recorded English prompt, unchanged (replay control).
V1  the recorded English prompt plus a language rule: ``message`` in Korean.
V2  the full Korean prompt: task, output description, reason and message in Korean.
V3  the recorded English prompt with ``message`` replaced by a structured-only
    object (condition B fields; no free text).

Literal tokens are never translated: robot ids, box labels, zone letters,
colour kinds, image labels, JSON keys and enum values. No example utterances
and no coordination rules are added. The C2 tie-break (``lowest robot_id``)
is kept only when the recorded prompt had it and is never added.

``dialogue_request`` builds one turn of a short multi-turn window (offline
pilot): the robot sees the peers' actual utterances addressed to it in this
window. The host only relays messages to the named recipients; it never
arbitrates, merges claims or ends a disagreement.
"""
from __future__ import annotations

import copy
import hashlib
import json
import re
from dataclasses import dataclass

ROBOTS = ('r1', 'r2', 'r3')
VARIANTS = ('V0', 'V1', 'V2', 'V3')
ZC2_SOURCE_SHA = '7ccd6c3'

# Frozen copy of the ZC2 English templates (harness/zone_coordination.py at
# 7ccd6c3; unchanged at 0add360). Kept here because the live module is being
# rewired; ``parse_recorded_system`` proves each recorded prompt re-renders
# byte-for-byte from these templates.
EN_COMMON = '''You are robot {rid}, an equal peer in a three-robot team (r1, r2, r3).
Mission: {instruction}
Goal (zone -> colour -> count): {goal}
The pickup area is west, zones A, B, C are painted floor areas east. You carry
one box at a time. A motion executor moves you when you are given a job; you
coordinate WHICH box goes to WHICH zone and WHO does it. Ground choices in the
images: {images}. box_labels are
fixed names from the first TOP images (per colour, west to east then south to
north). rgb_view is the current TOP-RGB estimate, not ground truth. Never claim
simulator coordinates or physical success.'''

EN_PLAN_BODY = '''
PLAN FIRST: agree on the COMPLETE assignment before anyone moves. Only
agreement.proposer may propose while agreement.proposal is null; the others
then reply accept=false, plan=null with a useful peer message. Once
agreement.proposal exists, every robot (the proposer too) either ACCEPTS by
replying accept=true with plan set to an exact copy of agreement.proposal.plan,
or REJECTS with accept=false, plan=null and a reason. accept=true with
plan=null is invalid. Copy proposal_id and plan_hash from agreement.proposal.
A plan lists, for every robot, its ordered jobs {"box": label, "zone": "A|B|C"}
so that every zone receives exactly its goal count per colour, each box is used
at most once and each box colour matches the zone's need. Balance the work;
robots run their lists in parallel.
'''
EN_PLAN_REPLY = '''Reply JSON only: {"request_id": copied, "proposal_id": copied or null,
"plan_hash": copied or null, "accept": true|false, "plan": {"assignments":
{"r1": [...], "r2": [...], "r3": [...]}} (your proposal, or the exact accepted
plan) or null when rejecting/waiting, "reason": "brief",
"message": "brief message to peers"}. reason/message under 240 characters.'''

EN_CLAIM_BODY = '''
TALK WHEN NEEDED: there is no global plan. You are idle now: claim ONE next job
for yourself, or null if nothing useful remains for you. Choose a box that is
still in pickup (rgb_view.pickup_boxes_still_visible), not claimed in
team_board.active, and a zone that still needs that colour (goal minus
rgb_view.zone_counts_seen minus active peer claims to that zone and colour).
Prefer jobs that keep the team busy and avoid peers' current paths. If
conflict is present, you and a peer claimed the same box or the same last need:
agree who takes it (read peer_messages) and choose again.'''
EN_TIE_BREAK = ''' Everyone answers at
once, so if you all yield nobody takes it: unless a peer's message already
gives it to a specific robot, the conflicting robot with the lowest robot_id
keeps it and the others choose a different job or null.'''
EN_CLAIM_REPLY = '''
Reply JSON only: {"request_id": copied, "claim": {"box": label or null,
"zone": "A|B|C" or null}, "reason": "brief", "message": "brief message to
peers"}. reason/message under 240 characters.'''

# V1: the only change to the English prompt is this language rule.
EN_KOREAN_MESSAGE_RULE = ('\nLANGUAGE: write "message" in Korean (한국어). Copy robot ids (r1, r2, r3), '
                          'box labels, zone letters (A, B, C), colour kinds, JSON keys and values '
                          '(null, true, false) exactly as given; never translate them.')

# V3: condition B structured message; replaces only the reply paragraph.
ACTS = ('claim', 'request', 'inform_obstacle', 'report', 'agree', 'yield', 'question', 'correct', 'refuse')
STRUCT_FIELDS = ('act', 'item', 'zone', 'role', 'passage', 'observed_at', 'confidence', 'reply_to')


def _struct_rule(image_labels):
    return ('STRUCTURED MESSAGES ONLY: "message" is null (silence) or one object with every field '
            '{"act": "' + '|'.join(ACTS) + '", "item": box label or null, "zone": "A|B|C" or null, '
            '"role": "carrier" or null, "passage": passage label or null, "observed_at": "'
            + '|'.join(image_labels) + '" or null, "confidence": number 0..1 or null, '
            '"reply_to": {"from_robot": "r1|r2|r3", "turn": integer} or null}. '
            'No free text in "message".')


EN_PLAN_REPLY_V3 = '''Reply JSON only: {"request_id": copied, "proposal_id": copied or null,
"plan_hash": copied or null, "accept": true|false, "plan": {"assignments":
{"r1": [...], "r2": [...], "r3": [...]}} (your proposal, or the exact accepted
plan) or null when rejecting/waiting, "reason": "brief",
"message": null or one structured message}. reason under 240 characters.
'''
EN_CLAIM_REPLY_V3 = '''
Reply JSON only: {"request_id": copied, "claim": {"box": label or null,
"zone": "A|B|C" or null}, "reason": "brief", "message": null or one structured
message}. reason under 240 characters.
'''

# V2: full Korean prompt. Same content and order as the English template.
KO_INSTRUCTIONS = {
    'Deliver boxes so that each zone ends with exactly the requested number of boxes of each colour. '
    'Any box of the right colour counts. Boxes start in the pickup area (west). Zones A, B and C are '
    'painted floor areas (east). Each robot carries one box at a time.':
    '각 구역에 색별로 요청된 개수의 상자가 정확히 놓이도록 상자를 배달한다. 색이 맞으면 어느 상자든 '
    '인정된다. 상자는 집하 구역(서쪽)에서 시작한다. 구역 A, B, C는 바닥에 칠해진 영역(동쪽)이다. '
    '각 로봇은 한 번에 상자 하나를 운반한다.',
}
KO_VIEW_ROLES = {'pickup': '집하 구역', 'zones': '목표 구역', 'south': '남쪽', 'north': '북쪽',
                 'west': '서쪽', 'east': '동쪽'}

KO_COMMON = '''너는 로봇 {rid}이며, 세 로봇 팀(r1, r2, r3)의 동등한 구성원이다.
임무: {instruction}
목표 (구역 -> 색 -> 개수): {goal}
집하 구역은 서쪽에 있고, 구역 A, B, C는 동쪽 바닥에 칠해진 영역이다. 너는 한 번에
상자 하나만 운반한다. 작업을 받으면 동작 실행기가 너를 움직인다. 너는 어느 상자를
어느 구역으로 누가 옮길지를 조정한다. 선택의 근거는 이미지에서 찾는다: {images}.
box_labels는 처음 TOP 이미지에서 정한 고정 이름이다(색마다 서쪽에서 동쪽으로, 그다음
남쪽에서 북쪽으로). rgb_view는 현재 TOP-RGB 추정값이며 정답이 아니다. 시뮬레이터
좌표나 물리적 성공을 주장하지 않는다.'''

KO_PLAN_BODY = '''
먼저 계획하기: 누구도 움직이기 전에 완전한 배정에 합의한다. agreement.proposal이
null인 동안에는 agreement.proposer만 제안할 수 있고, 나머지 로봇은 accept=false,
plan=null과 함께 동료에게 쓸모 있는 메시지로 답한다. agreement.proposal이 생기면 모든
로봇(제안자 포함)은 accept=true와 agreement.proposal.plan을 정확히 복사한 plan으로
수락하거나, accept=false, plan=null과 reason으로 거절한다. accept=true인데 plan=null이면
무효다. proposal_id와 plan_hash는 agreement.proposal에서 복사한다.
계획은 로봇마다 순서가 있는 작업 목록 {"box": 라벨, "zone": "A|B|C"}을 나열하며, 모든
구역이 색별 목표 개수를 정확히 받고, 각 상자는 최대 한 번만 쓰이며, 상자 색이 구역에
필요한 색과 맞아야 한다. 일을 고르게 나눈다. 로봇들은 각자의 목록을 동시에 실행한다.
'''
KO_PLAN_REPLY = '''JSON으로만 답한다: {"request_id": 그대로 복사, "proposal_id": 복사 또는 null,
"plan_hash": 복사 또는 null, "accept": true|false, "plan": {"assignments":
{"r1": [...], "r2": [...], "r3": [...]}} (너의 제안 또는 정확히 복사한 수락 계획) 또는
거절·대기 때 null, "reason": "짧은 판단 근거", "message": "동료에게 보내는 짧은 메시지"}.
reason과 message는 각각 240자 이내.'''

KO_CLAIM_BODY = '''
필요할 때 대화하기: 전체 계획은 없다. 너는 지금 할 일이 없다. 다음 작업 하나를 직접
맡겠다고 선언(claim)하거나, 너에게 더 쓸모 있는 일이 없으면 null로 답한다. 아직 집하
구역에 있고(rgb_view.pickup_boxes_still_visible) team_board.active에서 선언되지 않은
상자와, 그 색이 아직 필요한 구역(목표 − rgb_view.zone_counts_seen − 그 구역·색에 대한
동료의 active 선언)을 고른다. 팀이 계속 일할 수 있고 동료의 현재 경로를 피하는 작업을
우선한다. conflict가 있으면 너와 동료가 같은 상자나 같은 마지막 필요분을 선언한
것이다. 누가 가져갈지 합의하고(peer_messages를 읽는다) 다시 고른다.'''
KO_TIE_BREAK = ''' 모두가 동시에 답하므로
모두 양보하면 아무도 가져가지 않는다. 동료의 메시지가 이미 특정 로봇에게 넘긴 경우가
아니면, 충돌한 로봇 중 robot_id가 가장 낮은 로봇이 가져가고 나머지는 다른 작업이나
null을 고른다.'''
KO_CLAIM_REPLY = '''
JSON으로만 답한다: {"request_id": 그대로 복사, "claim": {"box": 라벨 또는 null,
"zone": "A|B|C" 또는 null}, "reason": "짧은 판단 근거", "message": "동료에게 보내는 짧은
메시지"}. reason과 message는 각각 240자 이내.'''
KO_LANGUAGE_RULE = ('\n언어: reason과 message는 한국어로 쓴다. 로봇 ID(r1, r2, r3), 상자 라벨, '
                    '구역 문자(A, B, C), 색 kind 값, 이미지 이름, JSON 키와 값(null, true, false)은 '
                    '번역하지 않고 주어진 그대로 쓴다.')

# Multi-turn window protocol (offline pilot). Describes the channel only:
# who can hear what, how many turns, what counts. No strategy, no phrases.
KO_WINDOW = '''
대화 창: 지금 세 로봇이 차례대로 말하는 짧은 대화 창이 열려 있다. 창 전체에서 최대
{max_turns}번, 로봇마다 최대 {per_robot}번 말할 수 있다. dialogue_window.received에는 이
창에서 네가 실제로 받은 동료 발화가 순서대로 있고, dialogue_window.sent에는 네가 이 창에서
보낸 발화가 있다. 매 차례 너의 현재 claim을 함께 낸다. 창이 끝났을 때 너의 마지막 claim이
너의 실행기로 전달되며, 동료의 claim을 대신 정해 주는 사람은 없다. message를 빈 문자열로
두면 침묵이다. recipients에는 message를 받을 동료 ID를 적는다(침묵이면 []). 받는 사람으로
지정되지 않은 로봇은 그 발화를 받지 못한다.'''
KO_WINDOW_REPLY = '''
JSON으로만 답한다: {"request_id": 그대로 복사, "claim": {"box": 라벨 또는 null,
"zone": "A|B|C" 또는 null}, "reason": "짧은 판단 근거", "message": "동료에게 보내는 짧은
메시지" 또는 "", "recipients": ["r1"|"r2"|"r3", ...]}. reason과 message는 각각 240자 이내.'''
KO_WINDOW_REPLY_V3 = '''
JSON으로만 답한다: {"request_id": 그대로 복사, "claim": {"box": 라벨 또는 null,
"zone": "A|B|C" 또는 null}, "reason": "짧은 판단 근거", "message": null 또는 정형 메시지
하나, "recipients": ["r1"|"r2"|"r3", ...]}. reason은 240자 이내.'''


@dataclass(frozen=True)
class SystemFacts:
    mode: str            # 'claim' | 'plan'
    rid: str
    instruction: str
    goal: str            # the exact goal JSON text of the recorded prompt
    images: str          # the exact image list text of the recorded prompt
    tie_break: bool


def render_en(f: SystemFacts) -> str:
    head = EN_COMMON.format(rid=f.rid, instruction=f.instruction, goal=f.goal, images=f.images)
    if f.mode == 'plan':
        return head + EN_PLAN_BODY + EN_PLAN_REPLY
    return head + EN_CLAIM_BODY + (EN_TIE_BREAK if f.tie_break else '') + EN_CLAIM_REPLY


_FIELD = {'rid': r'(?P<rid>r[123])', 'instruction': r'(?P<instruction>[^\n]+)',
          'goal': r'(?P<goal>[^\n]+)', 'images': r'(?P<images>.+?)'}


def _pattern(template):
    out, pos = [], 0
    for m in re.finditer(r'\{(rid|instruction|goal|images)\}', template):
        out.append(re.escape(template[pos:m.start()]))
        out.append(_FIELD[m.group(1)])
        pos = m.end()
    out.append(re.escape(template[pos:]))
    return re.compile(''.join(out), re.S)


def parse_recorded_system(text: str) -> SystemFacts:
    """Recover the template fields of a recorded ZC2 system prompt.

    Raises ValueError unless re-rendering the English template reproduces the
    recorded text byte-for-byte, so no prompt content can be silently lost.
    """
    shapes = (('plan', False, EN_PLAN_BODY + EN_PLAN_REPLY),
              ('claim', True, EN_CLAIM_BODY + EN_TIE_BREAK + EN_CLAIM_REPLY),
              ('claim', False, EN_CLAIM_BODY + EN_CLAIM_REPLY))
    head = _pattern(EN_COMMON)
    for mode, tie, tail in shapes:
        if not text.endswith(tail):
            continue
        m = head.fullmatch(text[:-len(tail)])
        if not m:
            continue
        facts = SystemFacts(mode, m['rid'], m['instruction'], m['goal'], m['images'], tie)
        if render_en(facts) == text:
            return facts
    raise ValueError('system prompt does not match a frozen ZC2 zone template')


def image_text_ko(images_en: str) -> str:
    """'CURRENT OWN RGB, TOP_SW (pickup, south) and TOP_NE (zones, north)' -> Korean; labels literal."""
    def role(m):
        parts = [KO_VIEW_ROLES.get(p.strip()) for p in m.group(2).split(',')]
        if None in parts:
            raise ValueError(f'unknown view role in {m.group(0)!r}')
        return f'{m.group(1)} ({", ".join(parts)})'
    text = re.sub(r'([A-Z_]+) \(([a-z, ]+)\)', role, images_en)
    text = re.sub(r' and (?=[A-Z_]+ \()', ', ', text)
    if re.search(r'[a-z]{3,}', re.sub(r'[A-Z_]+', '', text)):
        raise ValueError(f'untranslated image text: {text!r}')
    return text


def render_ko(f: SystemFacts, *, tie_break=None, window=None, structured=False, image_labels=()) -> str:
    try:
        instruction = KO_INSTRUCTIONS[f.instruction]
    except KeyError:
        raise ValueError('no Korean translation for this mission instruction') from None
    head = KO_COMMON.format(rid=f.rid, instruction=instruction, goal=f.goal, images=image_text_ko(f.images))
    tie = f.tie_break if tie_break is None else tie_break
    if tie and not f.tie_break:
        raise ValueError('the C2 tie-break is never added to a prompt that lacked it')
    if f.mode == 'plan':
        if window is not None:
            raise ValueError('dialogue windows are built for claim prompts only')
        return head + KO_PLAN_BODY + KO_PLAN_REPLY + KO_LANGUAGE_RULE
    body = head + KO_CLAIM_BODY + (KO_TIE_BREAK if tie else '')
    if window is None:
        return body + KO_CLAIM_REPLY + KO_LANGUAGE_RULE
    body += KO_WINDOW.format(**window)
    if structured:
        return (body + KO_WINDOW_REPLY_V3 + '\n' + _struct_rule(image_labels)
                + '\n언어: reason은 한국어로 쓴다. 로봇 ID, 상자 라벨, 구역 문자, 색 kind 값, 이미지 이름, '
                  'JSON 키와 값은 번역하지 않고 주어진 그대로 쓴다.')
    return body + KO_WINDOW_REPLY + KO_LANGUAGE_RULE


def _labels_of(request):
    return [item['label'] for item in request['images']]


def system_variant(request, variant):
    text = request['messages'][0]['content']
    facts = parse_recorded_system(text)
    if variant == 'V0':
        return text
    if variant == 'V1':
        return text + EN_KOREAN_MESSAGE_RULE
    if variant == 'V2':
        return render_ko(facts)
    if variant == 'V3':
        if facts.mode == 'plan':
            head = text[:-len(EN_PLAN_REPLY)]
            return head + EN_PLAN_REPLY_V3 + _struct_rule(_labels_of(request))
        head = text[:-len(EN_CLAIM_REPLY)]
        return head + EN_CLAIM_REPLY_V3 + _struct_rule(_labels_of(request))
    raise ValueError(f'unknown variant {variant!r}')


def build_variant(request, variant):
    """Same request_id, same user JSON string, same images; only the system text may differ."""
    if [m['role'] for m in request['messages']] != ['system', 'user']:
        raise ValueError('expected one system and one user message')
    out = {'request_id': request['request_id'],
           'messages': [{'role': 'system', 'content': system_variant(request, variant)},
                        copy.deepcopy(request['messages'][1])],
           'images': copy.deepcopy(request['images'])}
    return out


def dialogue_request(request, *, window, received, sent, turn, variant='V2', tie_break=False):
    """One turn of an offline multi-turn window for a recorded claim request.

    ``received``: the peers' actual utterances delivered to this robot in this
    window (already filtered by recipients). ``sent``: this robot's own
    utterances. The recorded user JSON is kept and one ``dialogue_window`` key
    is added; images are unchanged.
    """
    facts = parse_recorded_system(request['messages'][0]['content'])
    if facts.mode != 'claim':
        raise ValueError('dialogue windows use claim decision points')
    if variant not in ('V2', 'V3'):
        raise ValueError('dialogue windows use V2 (Korean) or V3 (structured)')
    system = render_ko(facts, tie_break=tie_break, window=window, structured=variant == 'V3',
                       image_labels=_labels_of(request))
    user = json.loads(request['messages'][1]['content'])
    if 'dialogue_window' in user:
        raise ValueError('recorded user JSON already has dialogue_window')
    request_id = f"{request['request_id']}-dlg{turn}"
    user['request_id'] = request_id
    user['dialogue_window'] = {'turn': turn, 'max_turns': window['max_turns'],
                               'your_turns_left': window['per_robot'] - len(sent),
                               'received': copy.deepcopy(received), 'sent': copy.deepcopy(sent)}
    return {'request_id': request_id,
            'messages': [{'role': 'system', 'content': system},
                         {'role': 'user', 'content': json.dumps(user, sort_keys=True, ensure_ascii=False)}],
            'images': copy.deepcopy(request['images'])}


def image_digests(request):
    """sha256 of each data URI and of its decoded bytes, for exact provenance."""
    import base64
    out = []
    for item in request['images']:
        uri = item['image']
        head, b64 = uri.split(',', 1)
        out.append({'label': item['label'], 'media': head,
                    'data_uri_sha256': hashlib.sha256(uri.encode()).hexdigest(),
                    'bytes_sha256': hashlib.sha256(base64.b64decode(b64)).hexdigest(),
                    'bytes': len(base64.b64decode(b64))})
    return out


def validate_struct_message(value, image_labels):
    """V3 schema check (evaluation and relay format); never repairs a message."""
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != set(STRUCT_FIELDS):
        raise ValueError('structured message needs exactly ' + ', '.join(STRUCT_FIELDS))
    if value['act'] not in ACTS:
        raise ValueError(f"unknown act {value['act']!r}")
    for key in ('item', 'passage'):
        if value[key] is not None and not isinstance(value[key], str):
            raise ValueError(f'{key} must be a string or null')
    if value['zone'] not in (None, 'A', 'B', 'C'):
        raise ValueError('zone must be A, B, C or null')
    if value['role'] not in (None, 'carrier'):
        raise ValueError('role must be carrier or null')
    if value['observed_at'] is not None and value['observed_at'] not in image_labels:
        raise ValueError('observed_at must be an image label or null')
    c = value['confidence']
    if c is not None and (isinstance(c, bool) or not isinstance(c, (int, float)) or not 0 <= c <= 1):
        raise ValueError('confidence must be 0..1 or null')
    r = value['reply_to']
    if r is not None and (not isinstance(r, dict) or set(r) != {'from_robot', 'turn'}
                          or r['from_robot'] not in ROBOTS or isinstance(r['turn'], bool)
                          or not isinstance(r['turn'], int)):
        raise ValueError('reply_to must be {from_robot, turn} or null')
    return copy.deepcopy(value)


__all__ = ['VARIANTS', 'SystemFacts', 'parse_recorded_system', 'render_en', 'render_ko', 'build_variant',
           'dialogue_request', 'image_digests', 'validate_struct_message', 'image_text_ko', 'ACTS',
           'STRUCT_FIELDS', 'ZC2_SOURCE_SHA']
