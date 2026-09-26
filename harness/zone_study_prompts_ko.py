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
import hashlib
import json
import re
from collections.abc import Mapping
from dataclasses import dataclass

from harness import zone_study_protocol as zp
from harness.zone_study_contract import (MAIN_CONDITIONS, ORDER_SHEET_SCHEMA, PAYLOAD_SCHEMA,
                                         ROLE_NAMES, validate_robot_payload)
from harness.zone_study_inputs import INPUT_PROFILE, payload_sha256, vocabulary

PROMPT_VERSION = 'ugrp.zone_study_prompts_ko.v2'
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
#: Schema of the digest that covers the WHOLE final request (system text, user
#: JSON and the actual image bytes), added for review finding 1.
REQUEST_DIGEST_SCHEMA = 'ugrp.zone_study_request_digest.v1'


class _Frozen(Mapping):
    """Deeply immutable view of one validated payload (review finding 4).

    ``frozen=True`` on the dataclass only stopped attribute rebinding: the
    counterexample added ``teacher_receipt`` to the payload dict AFTER
    validation and it reached the user JSON. Mutating this view raises, and
    ``copy.deepcopy``/:func:`thaw` hand out a plain copy instead of the original.
    """

    __slots__ = ('_data',)

    def __init__(self, data):
        object.__setattr__(self, '_data', {k: freeze(v) for k, v in data.items()})

    def __getitem__(self, key):
        return self._data[key]

    def __iter__(self):
        return iter(self._data)

    def __len__(self):
        return len(self._data)

    def __repr__(self):
        return f'_Frozen({self._data!r})'

    def __deepcopy__(self, memo):
        return thaw(self)

    def __copy__(self):
        return thaw(self)


def freeze(value):
    """Deeply immutable copy: dict -> ``_Frozen``, list -> tuple, scalars as-is."""
    if isinstance(value, Mapping):
        return _Frozen(value)
    if isinstance(value, (list, tuple)):
        return tuple(freeze(item) for item in value)
    return value


def thaw(value):
    """Plain JSON-serialisable copy of a frozen structure."""
    if isinstance(value, Mapping):
        return {k: thaw(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [thaw(item) for item in value]
    return value


@dataclass(frozen=True)
class StudyInputs:
    """One validated package A per-call payload plus the images it references.

    The boundary is A's: ``validate_robot_payload`` runs at construction, so a
    TOP frame, a ground-truth pose, a teacher receipt, a peer camera or a
    completion flag cannot reach a prompt. This class adds only the image bytes
    and the read-only views the prompt builder needs.

    Two rules come from the 2026-09-26 review:

    * **finding 1** — every image is bound to a VALIDATED reference: the sha256
      of the actual bytes must equal the digest the payload declared for that
      ref, and a map figure is only accepted against
      ``static_map.schematic_ref.png_sha256``, i.e. an artefact of the frozen
      map. So another camera's bytes can no longer be relabelled ``wrist_jpeg``.
    * **finding 4** — the payload is deeply frozen, and ``build_request``
      re-validates the thawed copy right before serialising it.

    ``pinned`` optionally carries the digests of the frozen order sheet and map
    bundle (``OrderSheetSource.pinned``).
    """

    payload: dict
    wrist_jpeg: bytes | None = None
    map_figure_jpeg: bytes | None = None
    robot_views: dict | None = None
    seed: int | None = None
    pinned: dict | None = None

    def __post_init__(self):
        set_ = object.__setattr__
        if not isinstance(self.payload, Mapping) or self.payload.get('schema') != PAYLOAD_SCHEMA:
            raise zp.ProtocolError(f'inputs.payload must carry schema {PAYLOAD_SCHEMA} '
                                   '(harness.zone_study_inputs.build_call_input)')
        payload = thaw(self.payload)
        try:
            validate_robot_payload(payload, seed=self.seed, pinned=self.pinned)
        except Exception as exc:                    # ContractViolation and friends
            raise zp.ProtocolError(f'payload violates the package A contract: {exc}') from None
        commander = payload['robot_id'] == zp.COMMANDER
        if commander:
            if self.wrist_jpeg is not None:
                raise zp.ProtocolError('the reference_R commander has no own wrist RGB')
            refs = {ref['ref'].split('-')[1]: ref for ref in payload.get('team_rgb_refs', ())}
            views = dict(self.robot_views or {})
            if sorted(views) != sorted(refs):
                raise zp.ProtocolError('robot_views must match the team_rgb_refs of the payload')
            # second review: an immutable view, so the r1 frame cannot be swapped
            # for the r2 frame after the bytes were bound to their refs.
            set_(self, 'robot_views', freeze({rid: _bound_bytes(jpeg, refs[rid], f'robot_views[{rid}]')
                                              for rid, jpeg in sorted(views.items())}))
        else:
            if self.robot_views is not None:
                raise zp.ProtocolError('only the reference_R commander receives every robot wrist RGB')
            own = list(payload.get('own_rgb_refs', ()))
            if not own:
                raise zp.ProtocolError('a wrist image needs the own_rgb_ref it was captured for')
            set_(self, 'wrist_jpeg', _bound_bytes(self.wrist_jpeg, own[-1], 'wrist_jpeg'))
        if self.map_figure_jpeg is not None:
            ref = (payload.get('static_map') or {}).get('schematic_ref')
            if not isinstance(ref, Mapping):
                raise zp.ProtocolError('a map figure is only allowed with static_map.schematic_ref, so the '
                                       'figure is an artefact of the frozen map')
            # Second review, finding 1: a self-declared ``schematic_ref`` proves
            # nothing (a wrist photo with its own hash passed). The figure is only
            # accepted against the run's PINNED map bundle.
            if not self.pinned or self.pinned.get('schematic_png_sha256') is None:
                raise zp.ProtocolError('a map figure needs pinned=OrderSheetSource.pinned of a map bundle '
                                       'rendered with its schematic (schematic_png_sha256)')
            if ref.get('png_sha256') != self.pinned['schematic_png_sha256']:
                raise zp.ProtocolError('static_map.schematic_ref is not the pinned map schematic')
            set_(self, 'map_figure_jpeg',
                 _bound_bytes(self.map_figure_jpeg, {'ref': ref['ref'], 'sha256': ref['png_sha256']},
                              'map_figure_jpeg'))
        set_(self, 'payload', freeze(payload))
        set_(self, '_payload_sha256', payload_sha256(payload))
        set_(self, 'pinned', dict(self.pinned) if self.pinned else None)

    # -- A payload views ---------------------------------------------------
    def payload_dict(self) -> dict:
        """A plain, mutable copy. Mutating it cannot change this input."""
        return thaw(self.payload)

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
        return thaw(self.payload['static_map'])

    @property
    def map_public(self) -> dict:
        return thaw(self.payload['static_map']['public_map'])

    @property
    def map_sha256(self) -> str:
        return self.payload['static_map']['public_map_sha256']

    @property
    def order_sheet(self) -> dict:
        return thaw(self.payload['order_sheet'])

    @property
    def order_sheet_sha256(self) -> str:
        return payload_sha256(thaw(self.payload['order_sheet']))

    @property
    def payload_sha256(self) -> str:
        """Digest of the validated payload, pinned at construction time."""
        return self._payload_sha256

    @property
    def own_commands(self) -> tuple:
        return tuple(thaw(self.payload.get('own_command_history', ())))

    @property
    def own_belief(self) -> dict:
        return thaw(self.payload.get('self_belief', {}))

    @property
    def issued_orders(self) -> tuple:
        return tuple(thaw(self.payload.get('issued_orders', ())))

    @property
    def inbox(self) -> tuple:
        """The messages the condition actually delivered (A envelopes)."""
        return tuple(thaw(self.payload.get('inbox', ())))

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


def image_sha256(jpeg) -> str:
    return hashlib.sha256(bytes(jpeg)).hexdigest()


def _bound_bytes(jpeg, ref, where) -> bytes:
    """Image bytes bound to the reference the payload declared (review finding 1).

    The ref already passed A's shape checks (own wrist ref pattern, capture time
    not after the call, sha256 hex), so comparing the digest of the ACTUAL bytes
    is what stops another camera's frame from being relabelled.
    """
    data = _image_bytes(jpeg, where)
    got = image_sha256(data)
    want = ref.get('sha256') if isinstance(ref, Mapping) else None
    if got != want:
        raise zp.ProtocolError(f'{where}: the bytes hash to {got[:12]}… but the validated reference '
                               f'{ref.get("ref") if isinstance(ref, Mapping) else ref!r} declares '
                               f'{str(want)[:12]}…; an image must be the frame its reference names')
    return data


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

#: Behaviour guidance that must be IDENTICAL in every condition and for every
#: role (2026-09-26 review finding 18). It used to live only in the ``follower``
#: channel block, so the leader condition got extra stop/refuse instructions and
#: a longer prompt, which confounded the channel effect with a guidance effect.
#: Second review: it asked every condition to "leave a reason and report it",
#: but no_comm and structured have no field for a reason. The guidance now asks
#: only for what every condition's output can express (the action and
#: ``decision_sources``); a reply schema identical across conditions is kept, so
#: the structured condition still carries no free text anywhere.
KO_BEHAVIOUR = '''행동 판단:
자기 wrist RGB의 근거와 어긋나는 계획이나 지시는 따르지 마십시오. 근거는 decision_sources에 적습니다.
안전하지 않다고 판단하면 action을 {"kind": "wait"}로 하여 멈추십시오.
같은 자리에서 같은 명령을 반복하지 말고 관측을 먼저 갱신하십시오.
각 로봇은 자기 행동을 스스로 결정합니다.'''

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

# --- channel blocks -------------------------------------------------------
# One fixed 5-line template. Only the slot texts differ between conditions, so
# the fixed instruction volume is the same and the measured difference is the
# CHANNEL, not the amount of guidance (review finding 18).
KO_CHANNEL_TEMPLATE = '''통신: {headline}
표현: {form}
수신자: {recipients}
예산: {budget}
분리: {separation}'''

KO_CHANNEL_SLOTS = {
    'no_comm': {
        'headline': '이 조건에는 메시지 채널이 없습니다. 보낼 수도, 받을 수도 없습니다.',
        'form': 'messages는 반드시 빈 배열 []이어야 합니다.',
        'recipients': '지정할 수 있는 수신자가 없습니다.',
        'budget': '이 조건의 발화 예산은 0입니다.',
        'separation': 'decision_sources에 message를 쓸 수 없습니다.',
    },
    'peer_ko': {
        'headline': '동료에게 한국어로 관측·의도·질문·요청·양보·정정을 전달할 수 있습니다.',
        'form': '본문은 한국어 자유 문장 한 개입니다.',
        'recipients': 'recipients에 받을 로봇 ID를 명시합니다. 지정되지 않은 로봇은 그 발화를 받지 못합니다.',
        'budget': '이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.',
        'separation': '메시지는 상대의 행동을 직접 바꾸지 않습니다.',
    },
    'leader': {
        'headline': '팀 배정을 한국어 메시지로 지시하고 보고를 확인할 수 있습니다.',
        'form': '본문은 한국어 자유 문장 한 개입니다.',
        'recipients': 'recipients에 받을 follower ID를 명시합니다. follower끼리는 서로 말할 수 없으므로 '
                      '한 follower가 알아야 하는 정보는 당신이 직접 전달해야 합니다.',
        'budget': '이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.',
        'separation': 'action은 당신 자신의 행동만 지정하고, 지시는 상대가 스스로 판단해 따릅니다.',
    },
    'follower': {
        'headline': '{leader}의 한국어 지시를 해석하고 질문·거절·관측 보고·양보를 보낼 수 있습니다.',
        'form': '본문은 한국어 자유 문장 한 개입니다.',
        'recipients': 'recipients에는 {leader}만 넣습니다. 다른 follower를 넣은 메시지는 전달되지 않고 '
                      '거절로 기록됩니다.',
        'budget': '이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.',
        'separation': '지시는 당신의 action을 자동으로 바꾸지 않습니다.',
    },
    'structured': {
        'headline': '동료에게 고정 필드 메시지로 관측·의도·요청·양보·정정을 전달할 수 있습니다.',
        'form': '본문은 자유 문장 없이 고정 필드 {fields}입니다. act는 {acts} 중 하나, '
                'state는 {states} 중 하나이거나 null, confidence는 {confidence} 중 하나이거나 null, '
                'observed_at_sim_s는 0 이상의 수이거나 null입니다. item, role, passage, location_ref는 '
                'static_map과 order_sheet에 있는 ID만 씁니다. text, reason, note 같은 자유 문자열 필드를 '
                '넣으면 메시지가 거절됩니다.',
        'recipients': 'recipients에 받을 로봇 ID를 명시합니다. 지정되지 않은 로봇은 그 발화를 받지 못합니다.',
        'budget': '이 대화 창에서 팀 전체 최대 {cap_window}발화, 당신은 최대 {cap_robot}발화입니다.',
        'separation': '메시지는 상대의 행동을 직접 바꾸지 않습니다.',
    },
    'commander': {
        'headline': '로봇과 주고받는 메시지 채널은 없습니다.',
        'form': 'messages는 반드시 빈 배열 []이어야 합니다.',
        'recipients': '지시는 messages가 아니라 action(kind "order")의 assignments로 내립니다.',
        'budget': '이 조건의 발화 예산은 0입니다.',
        'separation': '로봇은 당신의 지시만 실행합니다.',
    },
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
    """The ONLY part of the fixed prompt that differs between conditions."""
    key = {'no_comm': 'no_comm', 'reference_R': 'commander', 'structured': 'structured',
           'peer_ko': 'peer_ko'}.get(condition, role)
    slots = KO_CHANNEL_SLOTS[key]
    values = {'cap_window': spec.max_window_utterances, 'cap_robot': spec.max_robot_utterances,
              'leader': leader, 'fields': ', '.join(zp.STRUCT_FIELDS), 'acts': ', '.join(zp.STRUCT_ACTS),
              'states': ', '.join(zp.STRUCT_STATES), 'confidence': ', '.join(zp.CONFIDENCE)}
    return KO_CHANNEL_TEMPLATE.format(**{name: text.format(**values) for name, text in slots.items()})


def system_prompt(condition, rid, *, seed=None, leader=None, robots=zp.ROBOTS,
                  allow_leader_override=False) -> str:
    """Korean system prompt of one condition and actor. Literals stay literal.

    Every block except the channel section is identical across the conditions
    (review finding 18), so a condition difference is a CHANNEL difference and
    not a difference in the amount of task, safety or behaviour guidance.
    """
    parts = prompt_parts(condition, rid, seed=seed, leader=leader, robots=robots,
                         allow_leader_override=allow_leader_override)
    return '\n\n'.join(parts[name] for name in PROMPT_BLOCKS)


#: Assembly order of the system prompt. ``CHANNEL_BLOCKS`` is the only part a
#: condition may change; ``COMMON_BLOCKS`` must be token-identical everywhere
#: (review finding 18, checked by ``tests/test_zone_study_prompts_ko.py``).
COMMON_BLOCKS = ('head', 'behaviour', 'output', 'action', 'sources', 'separation')
CHANNEL_BLOCKS = ('channel', 'messages', 'language')
PROMPT_BLOCKS = ('head', 'channel', 'behaviour', 'output', 'action', 'sources', 'messages',
                 'separation', 'language')


def prompt_parts(condition, rid, *, seed=None, leader=None, robots=zp.ROBOTS,
                 allow_leader_override=False) -> dict:
    """The system prompt as named blocks, so the fixed cost can be measured."""
    s = zp.spec(condition)
    role = zp.role_of(condition, rid, seed=seed, leader=leader, robots=robots,
                      allow_override=allow_leader_override)
    lead = zp.leader_of(condition, seed=seed, leader=leader, robots=robots,
                        allow_override=allow_leader_override)
    if role == 'commander':
        head = KO_HEAD_COMMANDER
    else:
        head = KO_HEAD.format(rid=rid, role_line=KO_ROLE_LINE[role].format(leader=lead))
    if not s.channel_open:
        messages = KO_MESSAGES_NONE
    elif s.encoding == 'schema':
        messages = KO_MESSAGES_STRUCT
    else:
        messages = KO_MESSAGES_KO.replace('__CHARS__', str(zp.PROMPT_TEXT_CHARS))
    return {'head': head,
            'channel': _channel_block(condition, role, spec=s, leader=lead),
            'behaviour': KO_BEHAVIOUR,
            'output': KO_OUTPUT_HEAD.strip('\n'),
            'action': (KO_ACTION_COMMANDER if role == 'commander' else KO_ACTION_ROBOT).strip('\n'),
            'sources': KO_SOURCES.strip('\n'),
            'messages': messages.strip('\n'),
            'separation': KO_SEPARATION.strip('\n'),
            'language': (KO_LANGUAGE_STRUCT if s.channel_open and s.encoding == 'schema'
                         else KO_LANGUAGE).strip('\n')}


# --- frozen tokenizer for prompt-cost control (review finding 18) ----------

#: Version of the deterministic local tokenizer. It is NOT a provider tokenizer:
#: it is a frozen, reproducible proxy so the FIXED guidance cost can be compared
#: across conditions offline, without a network call. A real cohort records the
#: provider's own counts next to these.
TOKENIZER_VERSION = 'ugrp.zone_study_tokens.v1'
_TOKEN_SPLIT = re.compile(r'[A-Za-z0-9_]+|[가-힣]{1,2}|[^\sA-Za-z0-9_가-힣]')


def count_tokens(text) -> int:
    """Deterministic token count of one string (frozen proxy tokenizer).

    Latin/number runs and punctuation are one token each; Korean is counted in
    two-syllable chunks, which is the usual order of magnitude for Korean
    sub-word vocabularies. Frozen with ``TOKENIZER_VERSION``.
    """
    if not isinstance(text, str):
        raise zp.ProtocolError('count_tokens needs a string')
    return len(_TOKEN_SPLIT.findall(text))


def request_tokens(system, user, images=()) -> dict:
    """Token counts of one final request, split into fixed and variable parts."""
    return {'tokenizer': TOKENIZER_VERSION, 'system': count_tokens(system),
            'user': count_tokens(user), 'images': len(list(images)),
            'total_text': count_tokens(system) + count_tokens(user)}


#: How the FIXED system prompt is billed in SIM time (second review, finding 18).
#: The structured condition must list its fields, so its fixed text is longer
#: than the free-text conditions'. Billing the actual length would charge that
#: description as if it were dialogue. Under this policy every main-condition
#: call is billed the SAME fixed-prompt size — the largest main-condition system
#: prompt at that seed — while the variable part (payload, inbox, window) is
#: billed as counted. The actual count stays in the record next to the billed one.
FIXED_PROMPT_POLICY = 'fixed_prompt_equalized.v1'


def fixed_prompt_reference_tokens(*, seed=None, robots=zp.ROBOTS) -> int:
    """Billed fixed-prompt size: the largest main-condition system prompt."""
    return _fixed_reference(seed, tuple(robots))


def _fixed_reference(seed, robots):
    # The leader's identity only changes an ID token, not the size, so a call
    # without a seed (no rotating leader) is billed against seed 0's prompts.
    seed = 0 if seed is None else seed
    key = (seed, robots)
    if key not in _FIXED_CACHE:
        _FIXED_CACHE[key] = max(count_tokens(system_prompt(name, rid, seed=seed, robots=robots))
                                for name in MAIN_CONDITIONS for rid in zp.actors(name, robots))
    return _FIXED_CACHE[key]


_FIXED_CACHE: dict = {}


def billed_prompt_tokens(request, *, seed=None, robots=zp.ROBOTS) -> dict:
    """Actual vs billed text tokens of one request under ``FIXED_PROMPT_POLICY``."""
    tokens = request['tokens']
    reference = fixed_prompt_reference_tokens(seed=seed, robots=robots)
    return {'policy': FIXED_PROMPT_POLICY, 'tokenizer': tokens['tokenizer'],
            'system_actual': tokens['system'], 'system_billed': reference,
            'user': tokens['user'], 'images': tokens['images'],
            'total_text_billed': reference + tokens['user']}


def prompt_token_report(*, seed=11, robots=zp.ROBOTS) -> dict:
    """Fixed-prompt cost per condition and actor, split common vs channel.

    Used to hold the FIXED guidance equal across conditions: the common blocks
    must be token-identical for every main-condition robot and only the channel
    section may differ (review finding 18).
    """
    rows = {}
    for name in zp.CONDITIONS:
        for rid in zp.actors(name, robots):
            parts = prompt_parts(name, rid, seed=seed, robots=robots)
            common = sum(count_tokens(parts[b]) for b in COMMON_BLOCKS)
            channel = sum(count_tokens(parts[b]) for b in CHANNEL_BLOCKS)
            text = system_prompt(name, rid, seed=seed, robots=robots)
            rows[f'{name}:{rid}'] = {
                'condition': name, 'actor': rid,
                'role': zp.role_of(name, rid, seed=seed, robots=robots),
                'chars': len(text), 'tokens': count_tokens(text),
                'common_tokens': common, 'channel_tokens': channel,
                'channel_chars': sum(len(parts[b]) for b in CHANNEL_BLOCKS),
                'blocks': {b: count_tokens(parts[b]) for b in PROMPT_BLOCKS}}
    main = [row for row in rows.values() if row['condition'] in MAIN_CONDITIONS]
    return {'tokenizer': TOKENIZER_VERSION, 'seed': seed,
            'common_blocks': list(COMMON_BLOCKS), 'channel_blocks': list(CHANNEL_BLOCKS),
            # Measured control of the FIXED prompt cost: the common guidance must
            # not differ at all (head excluded: that is the role sentence), and
            # the residual difference is the channel section itself.
            'common_token_spread': (max(r['common_tokens'] - r['blocks']['head'] for r in main)
                                    - min(r['common_tokens'] - r['blocks']['head'] for r in main)),
            'channel_token_spread': (max(r['channel_tokens'] for r in main)
                                     - min(r['channel_tokens'] for r in main)),
            'rows': rows}


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


def build_request(inputs, *, seed=None, leader=None, window=None, sent=(), robots=zp.ROBOTS,
                  allow_leader_override=False) -> dict:
    """One model request: Korean system text, package A's payload JSON and images.

    ``inputs`` is a :class:`StudyInputs`, i.e. ONE validated A payload, so the
    condition, actor, request id, static map, order sheet, own observations and
    inbox all come from the contract. The user message is that payload verbatim,
    plus ``dialogue_window`` when the channel is open: only the window id and the
    utterance budget, never an extra observation. Preferably pass
    ``Transport.window_context(rid)`` so the stated budget is the transport's
    real one. This function never filters a channel itself and never receives
    host claims, reservations or another robot's state.

    2026-09-26 review:

    * finding 4 — the payload is re-validated here, right before it is
      serialised, so a mutation after construction cannot reach the model.
    * finding 9 — an explicit ``leader`` must agree with the seed rotation and
      with the payload's ``leader_id`` unless the caller says it is a
      diagnostic (``allow_leader_override``).
    * finding 1 — ``request_sha256`` covers the WHOLE final request: system
      text, user JSON and the digests of the actual image bytes.
    * finding 18 — ``tokens`` reports the frozen-tokenizer cost of the fixed
      guidance versus the variable part, per condition.
    """
    if not isinstance(inputs, StudyInputs):
        raise zp.ProtocolError('inputs must be a StudyInputs wrapping a validated package A payload '
                               '(harness.zone_study_inputs.build_call_input)')
    condition, rid, request_id = inputs.condition, inputs.robot_id, inputs.request_id
    s = zp.spec(condition)
    seed = inputs.seed if seed is None else seed
    resolved = zp.leader_of(condition, seed=seed, leader=leader, robots=robots,
                            allow_override=allow_leader_override)
    payload_leader = inputs.payload.get('leader_id')
    if payload_leader is not None and resolved != payload_leader:
        raise zp.ProtocolError(f'the payload names leader {payload_leader!r} but this request would use '
                               f'{resolved!r}: payload, prompt and transport must agree')
    role = zp.role_of(condition, rid, seed=seed, leader=leader, robots=robots,
                      allow_override=allow_leader_override)
    window = dict(window or {})
    # ``received`` is package A's ``inbox``: drop a transport copy instead of
    # sending the same messages twice under two key names.
    window.pop('received', None)
    issued = list(window.pop('sent', None) or sent)
    if not s.channel_open and (issued or window):
        raise zp.ProtocolError(f'{condition} has no dialogue channel: sent and window must be empty')
    if not s.channel_open and inputs.inbox:
        raise zp.ProtocolError(f'{condition} delivers no message, so the payload carries no inbox')
    body = inputs.payload_dict()
    # finding 4: the payload that is about to be serialised is validated again,
    # and its digest must still be the one recorded at construction.
    try:
        validate_robot_payload(body, seed=seed, pinned=inputs.pinned)
    except Exception as exc:
        raise zp.ProtocolError(f'the payload changed after validation: {exc}') from None
    if payload_sha256(body) != inputs.payload_sha256:
        raise zp.ProtocolError('the payload changed after validation (digest mismatch)')
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
    system = system_prompt(condition, rid, seed=seed, leader=leader, robots=robots,
                           allow_leader_override=allow_leader_override)
    user = json.dumps(body, sort_keys=True, ensure_ascii=False)
    images = _images(inputs)
    manifest = image_manifest(inputs)
    # second review, finding 1: every attached image is re-bound to its validated
    # reference right before sending, whichever path produced it.
    for row in manifest:
        if row['bytes_sha256'] != row['sha256']:
            raise zp.ProtocolError(f'{row["label"]}: the attached bytes are not the frame {row["ref"]} names')
    request = {'request_id': request_id, 'condition': condition, 'actor': rid, 'prompt_role': role,
               'prompt_version': PROMPT_VERSION, 'protocol_version': zp.PROTOCOL_VERSION,
               'payload_schema': PAYLOAD_SCHEMA, 'input_sha256': inputs.payload_sha256,
               'input_profile_id': INPUT_PROFILE['profile_id'],
               'messages': [{'role': 'system', 'content': system},
                            {'role': 'user', 'content': user}],
               'images': images}
    request['image_refs'] = manifest
    request['request_sha256'] = request_digest(system, user, images)
    request['tokens'] = request_tokens(system, user, images)
    request['billed_tokens'] = billed_prompt_tokens(request, seed=seed, robots=robots)
    return request


def image_manifest(inputs) -> list:
    """Which validated reference each attached image belongs to (review finding 1)."""
    payload = inputs.payload
    out = []
    if inputs.robot_views is None:
        own = list(payload.get('own_rgb_refs', ()))
        if own:
            ref = own[-1]
            out.append({'label': IMAGE_OWN, 'ref': ref['ref'], 'sha256': ref['sha256'],
                        'captured_at_sim_s': ref['captured_at_sim_s'],
                        'bytes_sha256': image_sha256(inputs.wrist_jpeg)})
    else:
        refs = {r['ref'].split('-')[1]: r for r in payload.get('team_rgb_refs', ())}
        for rid, jpeg in sorted(inputs.robot_views.items()):
            ref = refs[rid]
            out.append({'label': f'WRIST RGB {rid}', 'ref': ref['ref'], 'sha256': ref['sha256'],
                        'captured_at_sim_s': ref['captured_at_sim_s'],
                        'bytes_sha256': image_sha256(jpeg)})
    if inputs.map_figure_jpeg is not None:
        ref = payload['static_map']['schematic_ref']
        out.append({'label': IMAGE_MAP, 'ref': ref['ref'], 'sha256': ref['png_sha256'],
                    'captured_at_sim_s': None, 'bytes_sha256': image_sha256(inputs.map_figure_jpeg)})
    return out


def request_digest(system, user, images) -> str:
    """Digest of the WHOLE final request (review finding 1).

    ``input_sha256`` only covered the validated payload, so the dialogue window,
    the system text and the actual image bytes were outside the archived hash.
    The image bytes enter through their own sha256, which keeps the digest small
    and still binds the exact frames.
    """
    value = {'schema': REQUEST_DIGEST_SCHEMA, 'system': system, 'user': user,
             'images': [{'label': item['label'], 'bytes_sha256': image_sha256(_from_uri(item['image']))}
                        for item in images]}
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def request_digest_from_refs(system, user, image_refs) -> str:
    """The same digest recomputed from an ARCHIVED request (second review, finding 1).

    An archive keeps the final system and user text plus the image manifest
    (label + ``bytes_sha256``), not the base64 frames, so the digest a model
    request carried can be re-derived from the stored log alone.
    """
    value = {'schema': REQUEST_DIGEST_SCHEMA, 'system': system, 'user': user,
             'images': [{'label': row['label'], 'bytes_sha256': row['bytes_sha256']} for row in image_refs]}
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def archive_request(request, *, provider_usage=None) -> dict:
    """What an offline/online log keeps of one final request (second review, finding 1).

    The exact system and user text, the image manifest with the byte digests
    and the request digest. ``verify_archived_request`` re-derives the digest,
    so an archive that dropped or edited any part of the request is detected.

    Three sizes are kept APART (third review): ``tokens`` is the frozen local
    count of the text that actually went out, ``billed_tokens`` the
    standardised size the SIM cost charges (``FIXED_PROMPT_POLICY``), and
    ``provider_usage`` the provider's own usage report, None when no provider
    answered (offline fixture) or it reported nothing.
    """
    return {'request_id': request['request_id'], 'request_sha256': request['request_sha256'],
            'input_sha256': request['input_sha256'], 'prompt_version': request['prompt_version'],
            'system': request['messages'][0]['content'], 'user': request['messages'][1]['content'],
            'image_refs': copy.deepcopy(request['image_refs']),
            'tokens': dict(request['tokens']), 'billed_tokens': dict(request['billed_tokens']),
            'provider_usage': None if provider_usage is None else dict(provider_usage)}


def verify_archived_request(row) -> list:
    """Problems of one archived request (empty list = the archive is the request)."""
    problems = []
    for key in ('request_sha256', 'system', 'user', 'image_refs'):
        if key not in row:
            problems.append(f'archived request {row.get("request_id")!r} misses {key}')
    if problems:
        return problems
    if request_digest_from_refs(row['system'], row['user'], row['image_refs']) != row['request_sha256']:
        problems.append(f'archived request {row.get("request_id")!r}: content does not hash to request_sha256')
    for image in row['image_refs']:
        if image.get('bytes_sha256') != image.get('sha256'):
            problems.append(f'archived request {row.get("request_id")!r}: {image.get("label")} bytes differ '
                            'from their reference')
    problems.extend(_count_problems(row))
    try:
        body = json.loads(row['user'])
    except ValueError:
        return problems + [f'archived request {row.get("request_id")!r}: user text is not JSON']
    if 'input_sha256' in row:
        body.pop(WINDOW_KEY, None)
        if payload_sha256(body) != row['input_sha256']:
            problems.append(f'archived request {row.get("request_id")!r}: user JSON does not match input_sha256')
    return problems


def _count_problems(row) -> list:
    """The stored token counts re-derive from the stored text (third review, finding 1).

    An archive that kept the text but edited ``tokens`` or ``billed_tokens``
    would change the SIM cost without changing the request digest, so the
    counts are recomputed from the archived system/user text and compared.
    """
    rid = row.get('request_id')
    tokens, billed = row.get('tokens'), row.get('billed_tokens')
    if not isinstance(tokens, Mapping) or not isinstance(billed, Mapping):
        return [f'archived request {rid!r} misses its token counts (tokens, billed_tokens)']
    if tokens.get('tokenizer') != TOKENIZER_VERSION:
        return [f'archived request {rid!r}: tokenizer {tokens.get("tokenizer")!r} is not {TOKENIZER_VERSION}']
    want = request_tokens(row['system'], row['user'], row['image_refs'])
    out = [f'archived request {rid!r}: tokens.{k} {tokens.get(k)!r} != recount {v!r}'
           for k, v in want.items() if tokens.get(k) != v]
    for key, value in (('system_actual', want['system']), ('user', want['user']),
                       ('images', want['images']), ('tokenizer', TOKENIZER_VERSION),
                       ('policy', FIXED_PROMPT_POLICY)):
        if billed.get(key) != value:
            out.append(f'archived request {rid!r}: billed_tokens.{key} {billed.get(key)!r} != {value!r}')
    reference = billed.get('system_billed')
    if isinstance(reference, bool) or not isinstance(reference, int) or reference < 0:
        out.append(f'archived request {rid!r}: billed_tokens.system_billed {reference!r} is not a size')
    elif billed.get('total_text_billed') != reference + want['user']:
        out.append(f'archived request {rid!r}: billed_tokens.total_text_billed != system_billed + user')
    usage = row.get('provider_usage')
    if usage is not None and (not isinstance(usage, Mapping) or any(
            isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in usage.values())):
        out.append(f'archived request {rid!r}: provider_usage must be None or non-negative ints')
    return out


def _from_uri(uri) -> bytes:
    prefix = 'data:image/jpeg;base64,'
    if not isinstance(uri, str) or not uri.startswith(prefix):
        raise zp.ProtocolError('an image must be a base64 JPEG data URI')
    return base64.b64decode(uri[len(prefix):])


__all__ = ['PROMPT_VERSION', 'CONTRACT_PAYLOAD_SCHEMA', 'StudyInputs', 'system_prompt', 'build_request',
           'IMAGE_OWN', 'IMAGE_MAP', 'ORDER_SHEET_SCHEMA', 'FORBIDDEN_IMAGE_TOKENS', 'WINDOW_KEY',
           'WINDOW_FIELDS']
