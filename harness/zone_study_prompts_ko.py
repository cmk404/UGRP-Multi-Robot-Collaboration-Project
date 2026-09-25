"""Korean prompt skeletons of the zone dialogue study (package C).

One skeleton per condition — ``no_comm``, ``peer_ko``, ``leader_ko`` (leader and
follower variants), ``structured`` and the ``reference_R`` commander. Only the
communication block and the ``messages`` part of the output schema differ; the
task text, the input boundary and the language rule are identical, so a
condition difference cannot be a prompt-language difference (pilot
2026-09-25 recommendation 1, ``experiments/2026-09-25-zone-dialogue-ko-pilot``).

Every call carries the versioned static map, the order sheet built from the
scenario config, the robot's own wrist RGB, its own issued commands and its own
belief. The TOP camera is evaluation-only and the sim-only ``nav_cam`` is not a
robot input, so ``build_request`` refuses such an image label.

Literal tokens stay literal: ``r1``/``r2``/``r3``, zone letters ``A``/``B``/``C``,
``order_id``/``item_id``/passage IDs, item kinds, role names, JSON keys and enum
values are never translated.

Package A (``harness/zone_study_contract.py`` + ``harness/zone_study_inputs.py``)
owns the input boundary. ``StudyInputs`` is a thin, immutable wrapper around ONE
A payload (``ugrp.zone_study_call_input.v1``) built by
``harness.zone_study_inputs.build_call_input``: the JSON a model receives is that
validated payload, so the earlier local order-sheet/map/belief/command validators
of this module are gone and cannot drift from A's contract.
"""
from __future__ import annotations

import base64
import copy
import json
from dataclasses import dataclass

from harness import zone_study_protocol as zp
from harness.zone_study_contract import (ORDER_SHEET_SCHEMA, PAYLOAD_SCHEMA, ROLE_NAMES,
                                         validate_robot_payload)
from harness.zone_study_inputs import INPUT_PROFILE, payload_sha256, vocabulary

PROMPT_VERSION = 'ugrp.zone_study_prompts_ko.v1'
#: Which A schema this prompt builder consumes. A bump here is a prompt change.
CONTRACT_PAYLOAD_SCHEMA = PAYLOAD_SCHEMA

IMAGE_OWN = 'CURRENT OWN WRIST RGB'
IMAGE_MAP = 'STATIC MAP FIGURE'
FORBIDDEN_IMAGE_TOKENS = ('top', 'nav_cam', 'cctv')
#: Extra top-level key of the request body: the channel BUDGET of the open
#: dialogue window. It carries no observation — the delivered messages are A's
#: ``inbox`` — so it cannot widen the input boundary.
WINDOW_KEY = 'dialogue_window'
WINDOW_FIELDS = ('window_id', 'max_utterances', 'max_your_utterances', 'your_utterances_left', 'sent')


@dataclass(frozen=True)
class StudyInputs:
    """One validated package A per-call payload plus the images it references.

    The boundary is A's: ``validate_robot_payload`` runs at construction, so a
    TOP frame, a ground-truth pose, a teacher receipt, a peer camera or a
    completion flag cannot reach a prompt. This class adds only the image bytes
    and the read-only views the prompt builder needs.
    """

    payload: dict
    wrist_jpeg: bytes | None = None
    map_figure_jpeg: bytes | None = None
    robot_views: dict | None = None
    seed: int | None = None

    def __post_init__(self):
        set_ = object.__setattr__
        if not isinstance(self.payload, dict) or self.payload.get('schema') != PAYLOAD_SCHEMA:
            raise zp.ProtocolError(f'inputs.payload must carry schema {PAYLOAD_SCHEMA} '
                                   '(harness.zone_study_inputs.build_call_input)')
        payload = copy.deepcopy(self.payload)
        try:
            validate_robot_payload(payload, seed=self.seed)
        except Exception as exc:                    # ContractViolation and friends
            raise zp.ProtocolError(f'payload violates the package A contract: {exc}') from None
        set_(self, 'payload', payload)
        commander = payload['robot_id'] == zp.COMMANDER
        if commander:
            if self.wrist_jpeg is not None:
                raise zp.ProtocolError('the reference_R commander has no own wrist RGB')
            refs = [r['ref'] for r in payload.get('team_rgb_refs', ())]
            views = dict(self.robot_views or {})
            if sorted(views) != sorted({ref.split('-')[1] for ref in refs}):
                raise zp.ProtocolError('robot_views must match the team_rgb_refs of the payload')
            set_(self, 'robot_views', {rid: _image_bytes(jpeg, f'robot_views[{rid}]')
                                       for rid, jpeg in sorted(views.items())})
        else:
            if self.robot_views is not None:
                raise zp.ProtocolError('only the reference_R commander receives every robot wrist RGB')
            set_(self, 'wrist_jpeg', _image_bytes(self.wrist_jpeg, 'wrist_jpeg'))
        if self.map_figure_jpeg is not None:
            set_(self, 'map_figure_jpeg', _image_bytes(self.map_figure_jpeg, 'map_figure_jpeg'))

    # -- A payload views ---------------------------------------------------
    @property
    def condition(self) -> str:
        return self.payload['condition']

    @property
    def robot_id(self) -> str:
        return self.payload['robot_id']

    @property
    def request_id(self) -> str:
        return self.payload['request_id']

    @property
    def sim_time_s(self) -> float:
        return self.payload['sim_time_s']

    @property
    def static_map(self) -> dict:
        return self.payload['static_map']

    @property
    def map_public(self) -> dict:
        return self.payload['static_map']['public_map']

    @property
    def map_sha256(self) -> str:
        return self.payload['static_map']['public_map_sha256']

    @property
    def order_sheet(self) -> dict:
        return self.payload['order_sheet']

    @property
    def order_sheet_sha256(self) -> str:
        return payload_sha256(self.payload['order_sheet'])

    @property
    def payload_sha256(self) -> str:
        return payload_sha256(self.payload)

    @property
    def own_commands(self) -> tuple:
        return tuple(self.payload.get('own_command_history', ()))

    @property
    def own_belief(self) -> dict:
        return self.payload.get('self_belief', {})

    @property
    def issued_orders(self) -> tuple:
        return tuple(self.payload.get('issued_orders', ()))

    @property
    def inbox(self) -> tuple:
        """The messages the condition actually delivered (A envelopes)."""
        return tuple(self.payload.get('inbox', ()))

    def order_ids(self) -> tuple:
        return tuple(o['order_id'] for o in self.order_sheet['orders'])

    def item_ids(self) -> tuple:
        return tuple(i for o in self.order_sheet['orders'] for i in o.get('item_ids') or ())

    def kinds(self) -> tuple:
        return tuple(dict.fromkeys(o['kind'] for o in self.order_sheet['orders']))

    def roles_by_order(self) -> dict:
        """Grasp roles per order, from A's static ``kinds`` table."""
        table = self.order_sheet.get('kinds') or {}
        out = {}
        for order in self.order_sheet['orders']:
            roles = (table.get(order['kind']) or {}).get('roles')
            out[order['order_id']] = tuple(roles) if roles else ROLE_NAMES
        return out

    def passages(self) -> tuple:
        return tuple(p['id'] for p in self.map_public.get('passages', ()))

    def location_refs(self) -> tuple:
        """Static location references a message may name (A's vocabulary)."""
        return tuple(sorted(self.vocabulary().location_refs))

    def vocabulary(self):
        """Package A ``Vocabulary``: the only IDs a message may name."""
        return vocabulary(self.order_sheet, self.map_public)


def _image_bytes(jpeg, where) -> bytes:
    if not isinstance(jpeg, (bytes, bytearray)) or not jpeg:
        raise zp.ProtocolError(f'{where} must be non-empty JPEG bytes')
    return bytes(jpeg)


# --- Korean prompt text ---------------------------------------------------

KO_ROLE_LINE = {
    'peer': '세 로봇 r1, r2, r3은 동등한 동료입니다.',
    'leader': '세 로봇 r1, r2, r3 가운데 이번 seed의 leader는 당신입니다. 지휘와 자기 운반을 함께 맡습니다.',
    'follower': '세 로봇 r1, r2, r3 가운데 이번 seed의 leader는 {leader}입니다. 당신은 follower이며 자기 운반을 맡습니다.',
}

KO_HEAD = '''당신은 로봇 {rid}입니다. {role_line}

목표:
order_sheet의 물건을 지정된 destination_zone(A, B, C)으로 배송하십시오.
안전하게 완료한 배송을 우선하고, 추론·발화 대기를 포함한 SIM 시간을 줄이십시오.

매 호출 제공되는 static_map과 order_sheet는 초기 계획 정보입니다.
현재 위치, 현재 재고, 배송 완료를 보장하지 않습니다.
현재 상황의 근거는 자기 wrist RGB 1장, own_commands, own_belief와
이 조건에서 실제로 받은 수신 메시지뿐입니다.
공용 TOP 카메라, 다른 로봇의 영상·명령 기록, 시뮬레이터 상태, 정답 좌표,
성공·완료 판정은 제공되지 않습니다.
명령을 보냈다는 사실을 실제 이동·파지·배달 성공으로 간주하지 마십시오.
보이지 않거나 식별할 수 없으면 unknown으로 두십시오.'''

KO_HEAD_COMMANDER = '''당신은 지휘자 commander입니다. 로봇 몸체가 없습니다.

목표:
order_sheet의 물건이 지정된 destination_zone(A, B, C)에 배송되도록
r1, r2, r3에게 작업을 지시하십시오. 안전하게 완료한 배송을 우선하고,
추론 대기를 포함한 SIM 시간을 줄이십시오.

당신은 static_map, order_sheet, 세 로봇의 wrist RGB와 당신이 보낸 지시 이력을 받습니다.
로봇에는 LLM이 없으며 당신의 지시만 실행합니다.
공용 TOP 카메라, 시뮬레이터 상태, 정답 좌표, 성공·완료 판정은 제공되지 않습니다.
지시를 보냈다는 사실을 실제 이동·파지·배달 성공으로 간주하지 마십시오.
보이지 않거나 식별할 수 없으면 unknown으로 두십시오.
이 조건은 주 비교 조건이 아니라 정보가 많은 중앙 제어의 참고 상한입니다.'''

KO_CHANNEL = {
    'no_comm': '''
통신: 이 조건에는 메시지 채널이 없습니다. 보낼 수도, 받을 수도 없습니다.
messages는 반드시 빈 배열이어야 하고 decision_sources에 message를 쓸 수 없습니다.''',
    'peer_ko': '''
통신: 필요하면 동료에게 한국어로 관측·의도·질문·요청·양보·정정을 전달하십시오.
recipients에 받을 로봇 ID를 명시합니다. 지정되지 않은 로봇은 그 발화를 받지 못합니다.
이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.
각 로봇은 자기 행동을 스스로 결정합니다.''',
    'leader': '''
통신: 팀 배정을 한국어 메시지로 지시하고 보고를 확인하십시오.
recipients에 받을 follower ID를 명시합니다. follower끼리는 서로 말할 수 없으므로,
한 follower가 알아야 하는 정보는 당신이 그 follower에게 직접 전달해야 합니다.
이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.
action은 당신 자신의 행동만 지정합니다. 지시는 상대가 스스로 판단해 따릅니다.''',
    'follower': '''
통신: {leader}의 한국어 지시를 해석하십시오. 질문·거절·관측 보고·양보는 {leader}에게만
보낼 수 있습니다. 다른 follower를 recipients에 넣은 메시지는 전달되지 않고 거절로 기록됩니다.
이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.
자기 wrist RGB의 근거와 어긋나는 지시는 되묻거나 거절할 수 있습니다.
안전하지 않다고 판단하면 멈추고 그 이유를 보고하십시오.''',
    'structured': '''
통신: 메시지는 자유 문장 없이 고정 필드로만 씁니다. 필드는 {fields}입니다.
act는 {acts} 중 하나입니다. state는 {states} 중 하나이거나 null,
confidence는 {confidence} 중 하나이거나 null, observed_at_sim_s는 0 이상의 수이거나 null입니다.
item, role, passage, location_ref는 static_map과 order_sheet에 있는 ID만 씁니다.
text, reason, note 같은 자유 문자열 필드를 넣으면 메시지가 거절됩니다.
recipients에 받을 로봇 ID를 명시합니다. 지정되지 않은 로봇은 그 발화를 받지 못합니다.
이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.
각 로봇은 자기 행동을 스스로 결정합니다.''',
    'commander': '''
통신: 로봇과 주고받는 메시지 채널은 없습니다. messages는 반드시 빈 배열이어야 합니다.
지시는 messages가 아니라 action(kind "order")으로 내립니다.''',
}

KO_OUTPUT_HEAD = '''
출력 형식:
JSON 하나만 출력합니다. 최상위 키는 request_id, action, decision_sources, messages입니다.
- request_id: 주어진 값을 그대로 복사합니다.'''

KO_ACTION_ROBOT = '''
- action: 당신 자신의 행동 하나입니다. 다음 중 하나를 씁니다.
  {"kind": "claim", "order_id": order_sheet의 order_id, "role": 그 주문의 roles 중 하나,
   "destination_zone": "A"|"B"|"C"}
  {"kind": "continue"}
  {"kind": "wait"}
  {"kind": "release", "order_id": 놓아줄 order_id}'''

KO_ACTION_COMMANDER = '''
- action: 세 로봇에 대한 지시입니다.
  {"kind": "order", "assignments": {"r1": 지시 또는 null, "r2": 지시 또는 null,
   "r3": 지시 또는 null}}
  지시는 {"order_id": order_sheet의 order_id, "role": 그 주문의 roles 중 하나,
  "destination_zone": "A"|"B"|"C"}입니다.'''

KO_SOURCES = '''
- decision_sources: 실제로 사용한 근거만 나열합니다. 가능한 값은
  static_map, order_sheet, own_rgb, own_commands, own_belief, message입니다.'''

KO_MESSAGES_NONE = '''
- messages: 빈 배열 [].'''
KO_MESSAGES_KO = '''
- messages: 보낼 발화 목록입니다. 각 항목은
  {"recipients": ["r1"|"r2"|"r3", ...], "text": "한국어 발화", "reply_to": message_id 또는 null}
  입니다. 보낼 말이 없으면 빈 배열 []을 씁니다. text는 __CHARS__자 이내로 씁니다.
  reply_to에는 당신이 실제로 받은 메시지의 message_id만 씁니다.'''
KO_MESSAGES_STRUCT = '''
- messages: 보낼 발화 목록입니다. 각 항목은
  {"recipients": ["r1"|"r2"|"r3", ...], "message": 정형 메시지 하나,
   "reply_to": message_id 또는 null}
  입니다. 보낼 말이 없으면 빈 배열 []을 씁니다.
  reply_to에는 당신이 실제로 받은 메시지의 message_id만 씁니다.'''

KO_SEPARATION = '''
action과 messages는 분리됩니다. 메시지는 다른 로봇의 행동, 작업 배정, 예약을
직접 바꾸지 않습니다. 상대는 자기 관측과 판단으로 스스로 결정합니다.'''

KO_LANGUAGE = '''
언어: 자유 메시지 본문은 한국어로 씁니다. 로봇 ID(r1, r2, r3), 구역 문자(A, B, C),
order_id와 item_id, passage ID, 물건 kind, role 이름, JSON 키와 값(null, true, false),
enum 값은 번역하거나 바꾸지 않고 주어진 그대로 씁니다.'''

KO_LANGUAGE_STRUCT = '''
언어: 메시지는 정형 필드뿐이므로 자유 문장이 없습니다. 로봇 ID(r1, r2, r3),
구역 문자(A, B, C), order_id와 item_id, passage ID, 물건 kind, role 이름,
JSON 키와 값(null, true, false), enum 값은 번역하거나 바꾸지 않고 그대로 씁니다.'''


def _channel_block(condition, role, *, spec, leader=None):
    caps = {'cap_window': spec.max_window_utterances, 'cap_robot': spec.max_robot_utterances}
    if condition == 'no_comm':
        return KO_CHANNEL['no_comm']
    if condition == 'reference_R':
        return KO_CHANNEL['commander']
    if condition == 'structured':
        return KO_CHANNEL['structured'].format(
            fields=', '.join(zp.STRUCT_FIELDS), acts=', '.join(zp.STRUCT_ACTS),
            states=', '.join(zp.STRUCT_STATES), confidence=', '.join(zp.CONFIDENCE), **caps)
    if condition == 'peer_ko':
        return KO_CHANNEL['peer_ko'].format(**caps)
    return KO_CHANNEL[role].format(leader=leader, **caps)


def system_prompt(condition, rid, *, seed=None, leader=None, robots=zp.ROBOTS) -> str:
    """Korean system prompt of one condition and actor. Literals stay literal."""
    s = zp.spec(condition)
    role = zp.role_of(condition, rid, seed=seed, leader=leader, robots=robots)
    lead = zp.leader_of(condition, seed=seed, leader=leader, robots=robots)
    if role == 'commander':
        text = KO_HEAD_COMMANDER
    else:
        role_line = KO_ROLE_LINE[role].format(leader=lead)
        text = KO_HEAD.format(rid=rid, role_line=role_line)
    text += '\n' + _channel_block(condition, role, spec=s, leader=lead).lstrip('\n')
    text += '\n' + KO_OUTPUT_HEAD.lstrip('\n')
    text += KO_ACTION_COMMANDER if role == 'commander' else KO_ACTION_ROBOT
    text += KO_SOURCES
    if not s.channel_open:
        text += KO_MESSAGES_NONE
    elif s.channel_open and s.encoding == 'schema':
        text += KO_MESSAGES_STRUCT
    else:
        text += KO_MESSAGES_KO.replace('__CHARS__', str(zp.PROMPT_TEXT_CHARS))
    text += '\n' + KO_SEPARATION.lstrip('\n')
    text += '\n' + (KO_LANGUAGE_STRUCT if s.channel_open and s.encoding == 'schema'
                     else KO_LANGUAGE).lstrip('\n')
    return text


def _images(inputs):
    out = []
    if inputs.robot_views is None:
        out.append({'label': IMAGE_OWN, 'image': _uri(inputs.wrist_jpeg)})
    else:
        for rid, jpeg in sorted(inputs.robot_views.items()):
            out.append({'label': f'WRIST RGB {rid}', 'image': _uri(jpeg)})
    if inputs.map_figure_jpeg is not None:
        out.append({'label': IMAGE_MAP, 'image': _uri(inputs.map_figure_jpeg)})
    for item in out:
        low = item['label'].lower()
        if any(token in low for token in FORBIDDEN_IMAGE_TOKENS):
            raise zp.ProtocolError(f'{item["label"]}: TOP and nav_cam images are not robot input')
    return out


def _uri(jpeg):
    if not isinstance(jpeg, (bytes, bytearray)) or not jpeg:
        raise zp.ProtocolError('an image must be non-empty JPEG bytes')
    return 'data:image/jpeg;base64,' + base64.b64encode(bytes(jpeg)).decode()


def build_request(inputs, *, seed=None, leader=None, window=None, sent=(), robots=zp.ROBOTS) -> dict:
    """One model request: Korean system text, package A's payload JSON and images.

    ``inputs`` is a :class:`StudyInputs`, i.e. ONE validated A payload, so the
    condition, actor, request id, static map, order sheet, own observations and
    inbox all come from the contract. The user message is that payload verbatim,
    plus ``dialogue_window`` when the channel is open: only the window id and the
    utterance budget, never an extra observation. Preferably pass
    ``Transport.window_context(rid)`` so the stated budget is the transport's
    real one. This function never filters a channel itself and never receives
    host claims, reservations or another robot's state.
    """
    if not isinstance(inputs, StudyInputs):
        raise zp.ProtocolError('inputs must be a StudyInputs wrapping a validated package A payload '
                               '(harness.zone_study_inputs.build_call_input)')
    condition, rid, request_id = inputs.condition, inputs.robot_id, inputs.request_id
    s = zp.spec(condition)
    seed = inputs.seed if seed is None else seed
    role = zp.role_of(condition, rid, seed=seed, leader=leader, robots=robots)
    window = dict(window or {})
    # ``received`` is package A's ``inbox``: drop a transport copy instead of
    # sending the same messages twice under two key names.
    window.pop('received', None)
    issued = list(window.pop('sent', None) or sent)
    if not s.channel_open and (issued or window):
        raise zp.ProtocolError(f'{condition} has no dialogue channel: sent and window must be empty')
    if not s.channel_open and inputs.inbox:
        raise zp.ProtocolError(f'{condition} delivers no message, so the payload carries no inbox')
    body = copy.deepcopy(inputs.payload)
    if s.channel_open:
        cap_window = window.pop('max_utterances', s.max_window_utterances)
        cap_robot = window.pop('max_your_utterances', s.max_robot_utterances)
        left = window.pop('your_utterances_left', max(0, cap_robot - len(issued)))
        window_id = window.pop('window_id', None)
        if window:
            raise zp.ProtocolError(f'unknown dialogue window fields {sorted(window)}')
        if not zp.is_message_id(window_id or ''):
            raise zp.ProtocolError('an open dialogue window needs its window_id')
        for message_id in issued:
            if not zp.is_message_id(message_id):
                raise zp.ProtocolError(f'sent must be message_ids, got {message_id!r}')
        body[WINDOW_KEY] = {'window_id': window_id, 'max_utterances': cap_window,
                            'max_your_utterances': cap_robot, 'your_utterances_left': left,
                            'sent': issued}
    return {'request_id': request_id, 'condition': condition, 'actor': rid, 'prompt_role': role,
            'prompt_version': PROMPT_VERSION, 'protocol_version': zp.PROTOCOL_VERSION,
            'payload_schema': PAYLOAD_SCHEMA, 'input_sha256': inputs.payload_sha256,
            'input_profile_id': INPUT_PROFILE['profile_id'],
            'messages': [{'role': 'system',
                          'content': system_prompt(condition, rid, seed=seed, leader=leader, robots=robots)},
                         {'role': 'user',
                          'content': json.dumps(body, sort_keys=True, ensure_ascii=False)}],
            'images': _images(inputs)}


__all__ = ['PROMPT_VERSION', 'CONTRACT_PAYLOAD_SCHEMA', 'StudyInputs', 'system_prompt', 'build_request',
           'IMAGE_OWN', 'IMAGE_MAP', 'ORDER_SHEET_SCHEMA', 'FORBIDDEN_IMAGE_TOKENS', 'WINDOW_KEY',
           'WINDOW_FIELDS']
