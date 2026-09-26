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

Package A (``harness/zone_study_contract.py``, branch ``kiro/zone-study-contract``)
is not merged yet. ``StudyInputs`` below is a MINIMAL LOCAL ADAPTER so package C
can be built and tested now; ``from_contract`` accepts a duck-typed contract
object. Both must be aligned with package A once it lands — see ``ADAPTER_NOTE``.
"""
from __future__ import annotations

import base64
import copy
import json
from dataclasses import dataclass, field

from harness import zone_study_protocol as zp

PROMPT_VERSION = 'ugrp.zone_study_prompts_ko.v1'
ADAPTER_NOTE = ('package A (harness/zone_study_contract.py) 미병합 상태의 최소 로컬 어댑터. '
                'A가 병합되면 StudyInputs 필드·해시·주문서 schema를 A의 계약에 맞춰 정렬해야 한다.')

IMAGE_OWN = 'CURRENT OWN WRIST RGB'
IMAGE_MAP = 'STATIC MAP FIGURE'
ORDER_SHEET_SCHEMA = 'ugrp.zone_order.v1'

# Public projection of a versioned static map. ``top_cameras`` is dropped: the
# TOP views are evaluation-only.
MAP_PUBLIC_KEYS = ('schema', 'map_id', 'version', 'frame', 'bounds_m', 'regions', 'zone_slots',
                   'pickup_bays', 'passages', 'obstacles', 'terrain', 'box_kinds',
                   'approach_convention', 'landmarks')
MAP_FORBIDDEN_KEYS = ('top_cameras', 'setup_only', 'box_positions', 'robot_positions', 'digest')
# An order sheet comes from the scenario config, never from simulator state.
ORDER_KEYS = ('order_id', 'kind', 'item_ids', 'count', 'required_robots', 'roles',
              'destination_zone', 'initial_location')
ORDER_FORBIDDEN_KEYS = ('pose', 'xyz', 'xyz_m', 'held_by', 'carried_by', 'delivered', 'status',
                        'current_location', 'current_zone', 'body', 'body_name', 'qpos', 'progress')
FORBIDDEN_IMAGE_TOKENS = ('top', 'nav_cam', 'cctv')
# Own belief: only fields a robot can build from its own allowed inputs (the
# design's belief example). Package A/B may widen this on purpose, never by
# accident, so an unknown key is refused instead of forwarded.
BELIEF_KEYS = ('region', 'last_visual_anchor', 'last_requested_destination',
               'last_visually_confirmed_region', 'confidence', 'sources')
BELIEF_CONFIDENCE = ('low', 'medium', 'high')
# Own issued commands: the command and its own queue state, never a measured
# joint, a contact, a success judgement or another robot's command.
OWN_COMMAND_KEYS = ('command', 'args', 'issued_at_sim_s', 'status', 'request_id', 'window_id')
OWN_COMMAND_STATUS = ('command_issued', 'queue_empty', 'hold_requested', 'local_timeout')


@dataclass(frozen=True)
class StudyInputs:
    """Minimal local stand-in for the package A per-call input bundle.

    The input boundary is enforced here, at construction: the map is projected
    through ``public_map``, the order sheet through ``validate_order_sheet`` and
    the belief and command history through their allowlists. Building the bundle
    by hand therefore cannot smuggle a TOP camera, a ground-truth pose, another
    robot's state or a completion judgement into a prompt.
    """
    map_public: dict
    map_sha256: str
    order_sheet: dict
    order_sheet_sha256: str
    wrist_jpeg: bytes
    own_commands: tuple = ()
    own_belief: dict = field(default_factory=dict)
    map_figure_jpeg: bytes | None = None
    sim_time_s: float = 0.0
    adapter: str = 'local_minimal_v1'

    def __post_init__(self):
        set_ = object.__setattr__
        set_(self, 'map_public', public_map(self.map_public))
        set_(self, 'order_sheet', validate_order_sheet(self.order_sheet))
        set_(self, 'own_belief', validate_belief(self.own_belief))
        set_(self, 'own_commands', validate_own_commands(self.own_commands))
        for name in ('map_sha256', 'order_sheet_sha256'):
            if not isinstance(getattr(self, name), str) or len(getattr(self, name)) != 64:
                raise zp.ProtocolError(f'{name} must be a sha256 hex digest')
        # copy the image bytes: a caller keeping a bytearray must not be able to
        # change the picture a later prompt carries
        set_(self, 'wrist_jpeg', _image_bytes(self.wrist_jpeg, 'wrist_jpeg'))
        if self.map_figure_jpeg is not None:
            set_(self, 'map_figure_jpeg', _image_bytes(self.map_figure_jpeg, 'map_figure_jpeg'))

    def order_ids(self) -> tuple:
        return tuple(o['order_id'] for o in self.order_sheet['orders'])

    def item_ids(self) -> tuple:
        return tuple(i for o in self.order_sheet['orders'] for i in o['item_ids'])

    def roles_by_order(self) -> dict:
        return {o['order_id']: tuple(o['roles']) for o in self.order_sheet['orders']}

    def passages(self) -> tuple:
        return tuple(p['id'] for p in self.map_public.get('passages', ()))

    def location_refs(self) -> tuple:
        """Static location references a message may name."""
        regions = tuple(self.map_public.get('regions', ()))
        bays = tuple(self.map_public.get('pickup_bays', ()) or ())
        slots = tuple(str(s) for bay in (self.map_public.get('pickup_bays') or {}).values()
                      for s in (bay.get('slots', ()) if isinstance(bay, dict) else ()))
        return tuple(dict.fromkeys(regions + bays + slots + self.passages() + zp.ZONES))


def from_contract(obj) -> StudyInputs:
    """Adapt a package A contract object (duck-typed) to ``StudyInputs``."""
    if isinstance(obj, StudyInputs):
        return obj
    get = (lambda k, d=None: obj.get(k, d)) if isinstance(obj, dict) else (lambda k, d=None: getattr(obj, k, d))
    missing = [k for k in ('map_public', 'map_sha256', 'order_sheet', 'order_sheet_sha256', 'wrist_jpeg')
               if get(k) is None]
    if missing:
        raise zp.ProtocolError(f'contract object is missing {missing}; {ADAPTER_NOTE}')
    return StudyInputs(map_public=get('map_public'), map_sha256=get('map_sha256'),
                       order_sheet=get('order_sheet'),
                       order_sheet_sha256=get('order_sheet_sha256'), wrist_jpeg=get('wrist_jpeg'),
                       own_commands=tuple(get('own_commands', ()) or ()),
                       own_belief=dict(get('own_belief', {}) or {}),
                       map_figure_jpeg=get('map_figure_jpeg'), sim_time_s=float(get('sim_time_s', 0.0) or 0.0),
                       adapter='from_contract_v1')


def public_map(static_map: dict) -> dict:
    """Keep only static, robot-allowed map content; drop the TOP cameras.

    Keys outside the allowlist are dropped. A forbidden key NESTED inside an
    allowed block cannot be dropped safely, so it is refused instead.
    """
    if not isinstance(static_map, dict) or 'map_id' not in static_map:
        raise zp.ProtocolError('static map must be the authored map object')
    out = {k: copy.deepcopy(static_map[k]) for k in MAP_PUBLIC_KEYS if k in static_map}
    leaked = sorted(_nested_keys(out) & set(MAP_FORBIDDEN_KEYS))
    if leaked:
        raise zp.ProtocolError(f'public map must not carry {leaked}')
    return out


def _nested_keys(value, depth=12) -> set:
    if depth <= 0:
        raise zp.ProtocolError('input is nested too deeply to audit')
    if isinstance(value, dict):
        return set(value) | {k for v in value.values() for k in _nested_keys(v, depth - 1)}
    if isinstance(value, (list, tuple)):
        return {k for v in value for k in _nested_keys(v, depth - 1)}
    return set()


def _image_bytes(jpeg, where) -> bytes:
    if not isinstance(jpeg, (bytes, bytearray)) or not jpeg:
        raise zp.ProtocolError(f'{where} must be non-empty JPEG bytes')
    return bytes(jpeg)


def validate_belief(belief) -> dict:
    """Own belief: allowlisted fields only, so no ground truth can ride along."""
    if belief is None:
        return {}
    if not isinstance(belief, dict):
        raise zp.ProtocolError('own_belief must be an object')
    extra = sorted(set(belief) - set(BELIEF_KEYS))
    if extra:
        raise zp.ProtocolError(f'own_belief carries unknown fields {extra}; allowed: {BELIEF_KEYS}')
    for key in ('region', 'last_visual_anchor', 'last_requested_destination',
                'last_visually_confirmed_region'):
        if belief.get(key) is not None and not isinstance(belief[key], str):
            raise zp.ProtocolError(f'own_belief.{key} must be a string or null')
    if belief.get('confidence') is not None and belief['confidence'] not in BELIEF_CONFIDENCE:
        raise zp.ProtocolError(f'own_belief.confidence must be one of {BELIEF_CONFIDENCE} or null')
    sources = belief.get('sources')
    if sources is not None and (not isinstance(sources, list)
                                or not all(isinstance(s, str) for s in sources)):
        raise zp.ProtocolError('own_belief.sources must be a list of source ids')
    return copy.deepcopy(belief)


def validate_own_commands(commands) -> tuple:
    """The robot's own issued commands: no measurement, contact or peer record."""
    out = []
    for entry in tuple(commands or ()):
        if not isinstance(entry, dict):
            raise zp.ProtocolError('each own command must be an object')
        extra = sorted(set(entry) - set(OWN_COMMAND_KEYS))
        if extra:
            raise zp.ProtocolError(f'own_commands carries unknown fields {extra}; '
                                   f'allowed: {OWN_COMMAND_KEYS}')
        if entry.get('status') is not None and entry['status'] not in OWN_COMMAND_STATUS:
            raise zp.ProtocolError(f'own_commands status must be one of {OWN_COMMAND_STATUS}; '
                                   'an executor success or contact judgement is not a robot input')
        nested = sorted(_nested_keys(list(entry.values())) & set(ORDER_FORBIDDEN_KEYS))
        if nested:                      # e.g. a measured pose hidden inside args
            raise zp.ProtocolError(f'own_commands hides live state {nested}')
        out.append(copy.deepcopy(entry))
    return tuple(out)


def validate_order_sheet(sheet: dict) -> dict:
    """Config-built order sheet: no live state, no simulator names."""
    if not isinstance(sheet, dict) or sheet.get('schema') != ORDER_SHEET_SCHEMA:
        raise zp.ProtocolError(f'order sheet schema must be {ORDER_SHEET_SCHEMA}')
    if set(sheet) != {'schema', 'map_id', 'map_sha256', 'orders'}:
        raise zp.ProtocolError('order sheet needs schema, map_id, map_sha256 and orders only')
    orders = sheet['orders']
    if not isinstance(orders, list) or not orders:
        raise zp.ProtocolError('order sheet needs at least one order')
    seen = set()
    for order in orders:
        if not isinstance(order, dict):
            raise zp.ProtocolError('each order must be an object')
        bad = [k for k in order if k in ORDER_FORBIDDEN_KEYS]
        if bad:
            raise zp.ProtocolError(f'order {order.get("order_id")!r} carries live state {bad}')
        if set(order) != set(ORDER_KEYS):
            raise zp.ProtocolError('each order needs exactly ' + ', '.join(ORDER_KEYS))
        nested = sorted(_nested_keys(list(order.values())) & set(ORDER_FORBIDDEN_KEYS))
        if nested:
            raise zp.ProtocolError(f'order {order["order_id"]!r} hides live state {nested}')
        for key in ('order_id', 'kind'):
            if not isinstance(order[key], str) or not order[key]:
                raise zp.ProtocolError(f'{key} must be a non-empty string')
        if order['order_id'] in seen:
            raise zp.ProtocolError(f'duplicate order_id {order["order_id"]!r}')
        seen.add(order['order_id'])
        if order['destination_zone'] not in zp.ZONES:
            raise zp.ProtocolError(f'destination_zone must be one of {zp.ZONES}')
        if not isinstance(order['item_ids'], list) or not order['item_ids'] \
                or not all(isinstance(i, str) and i for i in order['item_ids']):
            raise zp.ProtocolError('item_ids must be a non-empty list of ids')
        if not isinstance(order['roles'], list) or len(order['roles']) != order['required_robots'] \
                or not all(isinstance(r, str) and r for r in order['roles']):
            raise zp.ProtocolError('roles must list one role name per required robot')
        loc = order['initial_location']
        if not isinstance(loc, dict) or set(loc) != {'pickup_bay', 'slot'}:
            raise zp.ProtocolError('initial_location needs pickup_bay and slot only')
    return copy.deepcopy(sheet)


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
    elif s.encoding == 'structured':
        text += KO_MESSAGES_STRUCT
    else:
        text += KO_MESSAGES_KO.replace('__CHARS__', str(zp.PROMPT_TEXT_CHARS))
    text += '\n' + KO_SEPARATION.lstrip('\n')
    text += '\n' + (KO_LANGUAGE_STRUCT if s.encoding == 'structured' else KO_LANGUAGE).lstrip('\n')
    return text


def _images(inputs, *, robot_views=None):
    out = []
    if robot_views is None:
        out.append({'label': IMAGE_OWN, 'image': _uri(inputs.wrist_jpeg)})
    else:
        for rid, jpeg in sorted(robot_views.items()):
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


def build_request(condition, rid, *, request_id, inputs, seed=None, leader=None, window=None,
                  inbox=(), sent=(), robot_views=None, robots=zp.ROBOTS) -> dict:
    """One model request: Korean system text, the per-call input JSON and images.

    ``inputs`` must be a ``StudyInputs``, so the input boundary is always the
    validated one. ``window`` is preferably ``Transport.window_context(rid)``:
    then the stated budget is the transport's real budget, not the condition
    default. ``inbox``/``sent`` are what the transport actually delivered and
    accepted; this function never filters a channel itself and never receives
    host claims, reservations or another robot's state.
    """
    s = zp.spec(condition)
    if not isinstance(inputs, StudyInputs):
        raise zp.ProtocolError('inputs must be a StudyInputs bundle (the validated input boundary); '
                               f'{ADAPTER_NOTE}')
    role = zp.role_of(condition, rid, seed=seed, leader=leader, robots=robots)
    if (robot_views is not None) != (role == 'commander'):
        raise zp.ProtocolError('only the reference_R commander receives every robot wrist RGB')
    if robot_views is not None and set(robot_views) != set(robots):
        raise zp.ProtocolError(f'robot_views must be exactly {tuple(robots)}')
    window = dict(window or {})
    received = [dict(r) for r in (window.pop('received', None) or inbox)]
    issued = list(window.pop('sent', None) or sent)
    if not s.channel_open and (received or issued or window):
        raise zp.ProtocolError(f'{condition} has no dialogue channel: inbox, sent and window must be empty')
    body_key = 'text' if s.encoding == 'ko_free' else 'message'
    for record in received:
        extra = set(record) - set(zp.INBOX_FIELDS)
        if extra:
            raise zp.ProtocolError(f'inbox record carries non-robot-facing fields {sorted(extra)}')
        if s.channel_open and body_key not in record:
            raise zp.ProtocolError(f'{condition} delivers {body_key}; got {sorted(record)}')
        if 'text' in record and 'message' in record:
            raise zp.ProtocolError('a delivered message is free text or structured, never both')
    body = {'request_id': request_id, 'robot_id': rid, 'condition': condition,
            'sim_time_s': inputs.sim_time_s,
            'static_map': copy.deepcopy(inputs.map_public), 'static_map_sha256': inputs.map_sha256,
            'order_sheet': copy.deepcopy(inputs.order_sheet),
            'order_sheet_sha256': inputs.order_sheet_sha256}
    images = _images(inputs, robot_views=robot_views)
    if role == 'commander':
        # No body, no own camera: the commander sees every robot's wrist RGB and
        # the orders it issued itself.
        body['issued_orders'] = copy.deepcopy(list(inputs.own_commands)[-16:])
        body['robot_views'] = [item['label'] for item in images if item['label'].startswith('WRIST RGB')]
    else:
        body['own_commands'] = copy.deepcopy(list(inputs.own_commands)[-16:])
        body['own_belief'] = copy.deepcopy(inputs.own_belief)
    if s.rotating_leader:
        body['leader'] = zp.leader_of(condition, seed=seed, leader=leader, robots=robots)
    if s.channel_open:
        cap_window = window.pop('max_utterances', s.max_window_utterances)
        cap_robot = window.pop('max_your_utterances', s.max_robot_utterances)
        left = window.pop('your_utterances_left', max(0, cap_robot - len(issued)))
        window_id = window.pop('window_id', None)
        if window:
            raise zp.ProtocolError(f'unknown dialogue window fields {sorted(window)}')
        if not zp.is_message_id(window_id or ''):
            raise zp.ProtocolError('an open dialogue window needs its window_id')
        body['dialogue_window'] = {'window_id': window_id, 'max_utterances': cap_window,
                                   'max_your_utterances': cap_robot, 'your_utterances_left': left,
                                   'received': received, 'sent': issued}
    return {'request_id': request_id, 'condition': condition, 'actor': rid, 'prompt_role': role,
            'prompt_version': PROMPT_VERSION, 'protocol_version': zp.PROTOCOL_VERSION,
            'messages': [{'role': 'system',
                          'content': system_prompt(condition, rid, seed=seed, leader=leader, robots=robots)},
                         {'role': 'user',
                          'content': json.dumps(body, sort_keys=True, ensure_ascii=False)}],
            'images': images}


__all__ = ['PROMPT_VERSION', 'ADAPTER_NOTE', 'StudyInputs', 'from_contract', 'public_map',
           'validate_order_sheet', 'validate_belief', 'validate_own_commands', 'system_prompt',
           'build_request', 'IMAGE_OWN', 'IMAGE_MAP', 'ORDER_SHEET_SCHEMA', 'ORDER_KEYS',
           'MAP_PUBLIC_KEYS', 'MAP_FORBIDDEN_KEYS', 'FORBIDDEN_IMAGE_TOKENS', 'BELIEF_KEYS',
           'OWN_COMMAND_KEYS', 'OWN_COMMAND_STATUS']
