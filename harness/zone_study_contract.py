"""Study contract for the Korean-dialogue zone study: conditions, the robot-facing
schema, the payload validator and the call/message/action log schema (package A).

Research question (user decisions 2026-09-25/26): does natural-language (Korean)
dialogue between robots change multi-robot task efficiency? Only the
COMMUNICATION CHANNEL differs between the main conditions; every other input is
identical.

Conditions (``CONDITIONS``):

===============  ==========================================================
``no_comm``      no channel at all; ``inbox`` must be absent.
``peer_ko``      decentralised mesh, free Korean text.
``leader_ko``    one ROBOT doubles as leader, rotating r1/r2/r3 by seed.
                 Hub-and-spoke: leader<->each follower, never follower<->follower.
                 Followers may report and object in Korean.
``structured``   the same mesh and the same information as ``peer_ko`` in a
                 fixed schema; free text is rejected.
``reference_R``  reference ceiling, NOT a main condition: an all-seeing
                 commander that receives every robot's own wrist RGB and
                 commands no-LLM robots. Never reported as a study condition.
===============  ==========================================================

What a robot may receive on every call (``BASE_ALLOWLIST`` + the condition's
extra keys): the static map projection and its schematic, the order sheet built
from the scenario config, its OWN wrist RGB refs, its OWN issued-command history,
its own belief and the messages its condition actually delivered to it.

Evaluation-only, never in a robot payload: the TOP cameras and anything derived
from them, ground-truth poses, measured joints, contacts, teacher receipts,
completion/success flags, peer state the host would know, hidden event
schedules, referee counts and metrics. ``validate_robot_payload`` rejects those
keys (``FORBIDDEN_KEYS``/``FORBIDDEN_KEY_SUBSTRINGS``), foreign camera refs and
any key that is not on the condition's allowlist.

Korean prompts keep IDs, zone letters, enum values and JSON keys literal, so the
validator also rejects non-ASCII keys.

This module has no simulator import, no model call and no file write.
"""
from __future__ import annotations

import hashlib
import json
import re
import unicodedata
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

CONTRACT_VERSION = 'ugrp.zone_study_contract.v1'
PAYLOAD_SCHEMA = 'ugrp.zone_study_call_input.v1'
MESSAGE_ENVELOPE_SCHEMA = 'ugrp.zone_study_message.v1'
CALL_LOG_SCHEMA = 'ugrp.zone_study_call.v1'
MESSAGE_LOG_SCHEMA = 'ugrp.zone_study_message_log.v1'
ACTION_LOG_SCHEMA = 'ugrp.zone_study_action.v1'
ORDER_SHEET_SCHEMA = 'ugrp.zone_order.v1'

ROBOTS = ('r1', 'r2', 'r3')
COMMANDER = 'commander'
ZONE_IDS = ('A', 'B', 'C')


class ContractViolation(ValueError):
    """A payload, message or log record that breaks the study contract."""


def digest(value: object) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


# ---------------------------------------------------------------------------
# Condition registry

BASE_ALLOWLIST = ('schema', 'request_id', 'robot_id', 'condition', 'sim_time_s', 'static_map',
                  'order_sheet', 'own_rgb_refs', 'own_command_history', 'self_belief', 'channel')
REQUIRED_KEYS = ('schema', 'request_id', 'robot_id', 'condition', 'sim_time_s', 'static_map',
                 'order_sheet', 'own_rgb_refs', 'own_command_history', 'channel')


@dataclass(frozen=True)
class Condition:
    """One row of the study design: who may talk to whom, in what form, on what input."""

    name: str
    korean_label: str
    actors: tuple[str, ...]
    topology: str                      # none | mesh | star | commander_downlink
    encoding: str                      # none | free_ko | schema
    robot_llm: bool
    is_main: bool
    leader_rotation: bool = False
    extra_input_keys: tuple[str, ...] = ()
    dropped_input_keys: tuple[str, ...] = ()
    notes: str = ''

    @property
    def input_allowlist(self) -> frozenset[str]:
        return frozenset(BASE_ALLOWLIST).union(self.extra_input_keys).difference(self.dropped_input_keys)

    @property
    def required_keys(self) -> tuple[str, ...]:
        return tuple(k for k in REQUIRED_KEYS if k not in self.dropped_input_keys)

    @property
    def free_text_allowed(self) -> bool:
        return self.encoding == 'free_ko'


CONDITIONS: dict[str, Condition] = {
    'no_comm': Condition(
        name='no_comm', korean_label='무통신', actors=ROBOTS, topology='none', encoding='none',
        robot_llm=True, is_main=True,
        notes='No high-level message is sent or received; the minimum execution synchronisation of a team '
              'carry is not a message channel and is documented separately.'),
    'peer_ko': Condition(
        name='peer_ko', korean_label='자유 한국어 동료 대화', actors=ROBOTS, topology='mesh',
        encoding='free_ko', robot_llm=True, is_main=True, extra_input_keys=('inbox',),
        notes='Decentralised: every robot decides its own action; nobody assigns teammates.'),
    'leader_ko': Condition(
        name='leader_ko', korean_label='한국어 지휘 겸임', actors=ROBOTS, topology='star',
        encoding='free_ko', robot_llm=True, is_main=True, leader_rotation=True,
        extra_input_keys=('inbox', 'leader_id', 'role'),
        notes='One robot doubles as leader, rotating r1/r2/r3 by seed. Hub-and-spoke only; followers may '
              'report and object in Korean and still choose their own action.'),
    'structured': Condition(
        name='structured', korean_label='정형 메시지 대조', actors=ROBOTS, topology='mesh',
        encoding='schema', robot_llm=True, is_main=True, extra_input_keys=('inbox',),
        notes='Same observations, recipients and budget as peer_ko; the message meaning travels only in the '
              'fixed schema (no free text, no free-text smuggling fields).'),
    'reference_R': Condition(
        name='reference_R', korean_label='전지적 지휘 참조 상한', actors=(COMMANDER,) + ROBOTS,
        topology='commander_downlink', encoding='schema', robot_llm=False, is_main=False,
        extra_input_keys=('team_rgb_refs', 'issued_orders'),
        dropped_input_keys=('own_rgb_refs', 'own_command_history', 'self_belief'),
        notes='Reference ceiling, not a main condition: the commander receives all robots own wrist RGB and '
              'the robots carry no LLM. Never summed with the main conditions and never called optimal.'),
}
MAIN_CONDITIONS = tuple(name for name, c in CONDITIONS.items() if c.is_main)


def condition(name: str) -> Condition:
    if name not in CONDITIONS:
        raise ContractViolation(f'unknown condition: {name!r} (known: {sorted(CONDITIONS)})')
    return CONDITIONS[name]


def leader_for_seed(name: str, seed: int, *, robots: Sequence[str] = ROBOTS) -> str:
    """Rotating leader: seed 11 -> r3, 12 -> r1, 13 -> r2 (seed % 3 over r1, r2, r3)."""
    spec = condition(name)
    if not spec.leader_rotation:
        raise ContractViolation(f'{name} has no leader')
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ContractViolation(f'seed must be an int, got {seed!r}')
    return robots[seed % len(robots)]


def allowed_edges(name: str, seed: int | None = None) -> frozenset[tuple[str, str]]:
    """(sender, recipient) pairs the condition permits."""
    spec = condition(name)
    if spec.topology == 'none':
        return frozenset()
    if spec.topology == 'mesh':
        return frozenset((a, b) for a in ROBOTS for b in ROBOTS if a != b)
    if spec.topology == 'star':
        if seed is None:
            raise ContractViolation(f'{name} needs a seed to place the rotating leader')
        leader = leader_for_seed(name, seed)
        return frozenset([(leader, f) for f in ROBOTS if f != leader]
                         + [(f, leader) for f in ROBOTS if f != leader])
    return frozenset((COMMANDER, r) for r in ROBOTS)


def role_of(name: str, actor: str, seed: int | None = None) -> str:
    """leader | follower | peer | commander | executor for the run manifest and the prompt."""
    spec = condition(name)
    if actor == COMMANDER:
        return 'commander'
    if spec.topology == 'commander_downlink':
        return 'executor'
    if spec.leader_rotation:
        return 'leader' if actor == leader_for_seed(name, seed) else 'follower'
    return 'peer'


def channel_section(name: str, actor: str, seed: int | None = None) -> dict:
    """The robot-facing description of its own channel (a condition rule, not live state)."""
    spec = condition(name)
    edges = allowed_edges(name, seed)
    return {'condition': name, 'topology': spec.topology, 'encoding': spec.encoding,
            'free_text_allowed': spec.free_text_allowed,
            'can_send_to': sorted(b for a, b in edges if a == actor),
            'can_receive_from': sorted(a for a, b in edges if b == actor),
            'role': role_of(name, actor, seed)}


def condition_manifest(name: str, seed: int | None = None) -> dict:
    """What a run bundle records about the condition (docs/execution_versioning.md)."""
    spec = condition(name)
    value = {'contract_version': CONTRACT_VERSION, 'condition': name, 'korean_label': spec.korean_label,
             'topology': spec.topology, 'encoding': spec.encoding, 'robot_llm': spec.robot_llm,
             'is_main': spec.is_main, 'actors': list(spec.actors),
             'input_allowlist': sorted(spec.input_allowlist), 'notes': spec.notes}
    if spec.leader_rotation and seed is None:
        value['edges'] = 'star: leader<->each follower; the leader rotates r1/r2/r3 as robots[seed % 3]'
    else:
        value['edges'] = sorted(allowed_edges(name, seed))
    if spec.leader_rotation and seed is not None:
        value['seed'], value['leader_id'] = seed, leader_for_seed(name, seed)
    return value


def registry_sha256() -> str:
    """Hash of the whole registry; pin it next to the code SHA of a cohort."""
    return digest({name: condition_manifest(name) for name in sorted(CONDITIONS)})


# ---------------------------------------------------------------------------
# Structured messages (condition ``structured`` and the commander downlink)

STRUCTURED_ACTS = ('propose', 'request', 'accept', 'reject', 'inform', 'correct', 'yield', 'cancel')
STRUCTURED_STATES = ('unknown', 'suspected', 'clear', 'blocked', 'present', 'absent', 'held', 'placed')
CONFIDENCE = ('low', 'medium', 'high')
STRUCTURED_FIELDS = ('act', 'item', 'zone', 'role', 'passage', 'location_ref', 'state', 'confidence',
                     'observed_at_sim_s', 'reply_to')
STRUCTURED_ID_FIELDS = {'item': 'items', 'zone': 'zones', 'role': 'roles', 'passage': 'passages',
                        'location_ref': 'location_refs'}
# Free-text smuggling fields: rejected in schema messages.
FREE_TEXT_FIELDS = ('text', 'reason', 'note', 'notes', 'comment', 'other', 'message', 'body', 'extra')
MESSAGE_ENVELOPE = ('message_id', 'sender', 'recipients', 'encoding', 'created_at_sim_s', 'reply_to', 'body')
HANGUL = re.compile(r'[\uac00-\ud7a3\u1100-\u11ff\u3130-\u318f]')
LITERAL_TOKEN = re.compile(r'^[A-Za-z0-9_.:\-\[\]{}"]+$')
LATIN_RUN = re.compile(r'[A-Za-z][A-Za-z0-9_\-]*')
# Tokens that stay literal in a Korean message no matter the scenario.
ALWAYS_LITERAL = frozenset(ROBOTS) | frozenset(ZONE_IDS) | frozenset(STRUCTURED_ACTS) \
    | frozenset(STRUCTURED_STATES) | frozenset(CONFIDENCE) | {COMMANDER, 'null', 'true', 'false',
                                                              'pickup', 'JSON', 'sim', 'SIM', 'RGB'}


@dataclass(frozen=True)
class Vocabulary:
    """The IDs a structured message may name: from the order sheet and the public map only."""

    items: frozenset[str] = frozenset()
    zones: frozenset[str] = frozenset(ZONE_IDS)
    roles: frozenset[str] = frozenset()
    passages: frozenset[str] = frozenset()
    location_refs: frozenset[str] = frozenset()
    extra: Mapping[str, frozenset[str]] = field(default_factory=dict)

    def allowed(self, name: str) -> frozenset[str]:
        return frozenset(getattr(self, name, frozenset()))


def structured_violations(body: Mapping, *, vocabulary: Vocabulary | None = None) -> list[str]:
    """Why a schema message is invalid (empty list = valid)."""
    out: list[str] = []
    if not isinstance(body, Mapping):
        return ['structured message must be an object']
    unknown = [k for k in body if k not in STRUCTURED_FIELDS]
    if unknown:
        out.append(f'unknown structured field(s): {sorted(unknown)}')
    smuggled = [k for k in body if k in FREE_TEXT_FIELDS]
    if smuggled:
        out.append(f'free-text field(s) are not allowed in a schema message: {sorted(smuggled)}')
    act = body.get('act')
    if act not in STRUCTURED_ACTS:
        out.append(f'act must be one of {STRUCTURED_ACTS}, got {act!r}')
    state = body.get('state')
    if state is not None and state not in STRUCTURED_STATES:
        out.append(f'state must be one of {STRUCTURED_STATES}, got {state!r}')
    confidence = body.get('confidence')
    if confidence is not None and confidence not in CONFIDENCE:
        out.append(f'confidence must be one of {CONFIDENCE}, got {confidence!r}')
    observed = body.get('observed_at_sim_s')
    if observed is not None and (isinstance(observed, bool) or not isinstance(observed, (int, float))
                                 or observed < 0):
        out.append('observed_at_sim_s must be a non-negative number or null')
    reply_to = body.get('reply_to')
    if reply_to is not None and not (isinstance(reply_to, str) and reply_to):
        out.append('reply_to must be a message_id string or null')
    for field_name, vocab_name in STRUCTURED_ID_FIELDS.items():
        value = body.get(field_name)
        if value is None:
            continue
        if not isinstance(value, str) or not LITERAL_TOKEN.match(value):
            out.append(f'{field_name} must be a literal ID token, got {value!r}')
        elif vocabulary is not None and value not in vocabulary.allowed(vocab_name):
            out.append(f'{field_name}={value!r} is not in the run vocabulary')
    return out


def free_text_report(text: object, *, literals: Sequence[str] = ()) -> dict:
    """Korean-compliance report for a free message (metrics for package I).

    ``literals`` are the run's literal tokens (robot ids, order/item ids, zone
    letters, passage ids, enum values) that must NOT be translated. Every other
    latin word is reported as a language slip; only a message with no Korean at
    all is a contract violation.
    """
    if not isinstance(text, str) or not text.strip():
        return {'ok': False, 'reasons': ['free message body needs non-empty text'], 'has_korean': False,
                'latin_words': [], 'chars': 0}
    allowed = set(literals) | ALWAYS_LITERAL
    latin_words = [run for run in LATIN_RUN.findall(text)
                   if run not in allowed and len(re.findall('[A-Za-z]', run)) > 1]
    has_korean = bool(HANGUL.search(text))
    reasons = [] if has_korean else ['free message body has no Korean text']
    return {'ok': not reasons, 'reasons': reasons, 'has_korean': has_korean,
            'latin_words': latin_words, 'chars': len(text)}


def message_violations(name: str, sender: str, recipients: Sequence[str], body: object, *,
                       seed: int | None = None, vocabulary: Vocabulary | None = None) -> list[str]:
    """Topology + encoding check for one outgoing message (empty list = allowed)."""
    spec = condition(name)
    out: list[str] = []
    if not isinstance(recipients, Sequence) or isinstance(recipients, str) or not recipients:
        return [f'recipients must be a non-empty list, got {recipients!r}']
    if len(set(recipients)) != len(recipients):
        out.append('recipients must be unique')
    if sender in recipients:
        out.append('a robot cannot send to itself')
    edges = allowed_edges(name, seed)
    for recipient in recipients:
        if (sender, recipient) not in edges:
            reason = 'no channel in this condition' if spec.topology == 'none' else \
                ('hub-and-spoke: leader<->follower only' if spec.topology == 'star' else 'edge not allowed')
            out.append(f'{sender}->{recipient} is not allowed ({reason})')
    if spec.encoding == 'none':
        out.append(f'{name} allows no messages at all')
    elif spec.encoding == 'free_ko':
        if not isinstance(body, Mapping) or set(body) - {'text'} or 'text' not in body:
            out.append('a free message body must be exactly {"text": "..."}')
        else:
            out.extend(free_text_report(body['text'])['reasons'])
    else:
        out.extend(structured_violations(body if isinstance(body, Mapping) else {}, vocabulary=vocabulary))
    return out


def check_message(name: str, sender: str, recipients: Sequence[str], body: object, *,
                  seed: int | None = None, vocabulary: Vocabulary | None = None) -> None:
    problems = message_violations(name, sender, recipients, body, seed=seed, vocabulary=vocabulary)
    if problems:
        raise ContractViolation('; '.join(problems))


def message_envelope(name: str, message_id: str, sender: str, recipients: Sequence[str], body: object, *,
                     created_at_sim_s: float, seed: int | None = None, reply_to: str | None = None,
                     vocabulary: Vocabulary | None = None) -> dict:
    """A checked envelope; the meaning stays in ``body`` (free Korean text or the schema object)."""
    check_message(name, sender, recipients, body, seed=seed, vocabulary=vocabulary)
    return {'schema': MESSAGE_ENVELOPE_SCHEMA, 'message_id': str(message_id), 'sender': sender,
            'recipients': list(recipients), 'encoding': condition(name).encoding,
            'created_at_sim_s': float(created_at_sim_s), 'reply_to': reply_to,
            'body': json.loads(json.dumps(body))}


# ---------------------------------------------------------------------------
# Public/private boundary

FORBIDDEN_KEYS = frozenset({
    # ground truth / simulator state
    'pose', 'poses', 'qpos', 'qvel', 'ctrl', 'measured_joints', 'joint_positions', 'joint_angles',
    'body_id', 'body_name', 'geom_id', 'site_id', 'xpos', 'xquat', 'ground_truth', 'truth', 'gt',
    'simulator', 'sim_state', 'mj_model', 'mj_data', 'contacts', 'contact_forces', 'forces', 'wrench',
    'weld', 'referee', 'referee_v2',
    # evaluation-only cameras
    'top', 'top_frame', 'top_frames', 'top_image', 'top_images', 'top_rgb', 'top_view', 'top_views',
    'top_camera', 'top_cameras', 'cctv', 'cctv_top', 'nav_cam', 'nav_cam_rgb', 'nav_camera',
    # teacher and completion judgements
    'teacher', 'teacher_receipt', 'teacher_receipts', 'receipt', 'receipts', 'grasp_success', 'placed',
    'placed_at', 'delivered', 'deliveries', 'delivery_confirmed', 'completion', 'completed', 'complete',
    'finished', 'success', 'succeeded', 'done',
    # host-side peer/global state
    'zone_counts', 'zone_counts_seen', 'remaining_need', 'global_progress', 'progress', 'team_board',
    'peer_board', 'peer_status', 'peer_states', 'active_claims', 'peer_claims', 'peer_commands',
    'peer_command_history', 'peer_rgb', 'peer_images', 'other_robots', 'busy',
    # hidden events and metrics
    'hidden_event', 'hidden_events', 'event_schedule', 'injected_failures', 'eval', 'evaluation',
    'eval_only', 'score', 'scores', 'makespan', 'metrics',
})
FORBIDDEN_KEY_SUBSTRINGS = ('_pose', 'pose_', 'ground_truth', 'teacher', 'receipt', 'top_rgb', 'top_frame',
                            'top_image', 'cctv', 'nav_cam', 'qpos', 'qvel', 'hidden_event', 'body_id',
                            'xpos', 'sim_state', 'peer_', 'referee', 'weld', 'grasp_success')
FORBIDDEN_VALUE_SUBSTRINGS = ('cctv', 'nav_cam', 'top_rgb', 'top_frame', 'top_west', 'top_east', 'top_sw',
                              'top_nw', 'top_se', 'top_ne', 'ground_truth', 'teacher_receipt')
OWN_RGB_REF = re.compile(r'^own-(r1|r2|r3)-\d{3,6}$')
MAP_SCHEMATIC_REF = re.compile(r'^map-[A-Za-z0-9_\-]+-schematic$')
LOCAL_STATES = ('command_issued', 'queue_empty', 'hold_requested', 'local_timeout', 'command_rejected')
BELIEF_KEYS = ('region', 'last_visual_anchor', 'last_requested_destination', 'last_visually_confirmed_region',
               'confidence', 'sources', 'held_item_guess', 'blocked_passages', 'notes_ko')


def _walk(value, path='$'):
    if isinstance(value, Mapping):
        for key, item in value.items():
            yield path, key, item
            yield from _walk(item, f'{path}.{key}')
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, item in enumerate(value):
            yield from _walk(item, f'{path}[{index}]')


def forbidden_key_hits(value: object) -> list[str]:
    """Every evaluation-only key found anywhere in a robot-facing payload."""
    hits = []
    for path, key, _ in _walk(value):
        if not isinstance(key, str):
            hits.append(f'{path}: non-string key {key!r}')
            continue
        low = key.lower()
        if low in FORBIDDEN_KEYS:
            hits.append(f'{path}.{key}: evaluation-only key')
        elif any(part in low for part in FORBIDDEN_KEY_SUBSTRINGS):
            hits.append(f'{path}.{key}: evaluation-only key pattern')
    return hits


def non_ascii_keys(value: object) -> list[str]:
    """Korean prompts keep JSON keys literal; a non-ASCII key breaks that rule."""
    return [f'{path}.{key}' for path, key, _ in _walk(value)
            if isinstance(key, str) and not key.isascii()]


def _value_hits(value: object) -> list[str]:
    hits = []
    for path, key, item in _walk(value):
        if not isinstance(item, str):
            continue
        low = unicodedata.normalize('NFKC', item).lower()
        for part in FORBIDDEN_VALUE_SUBSTRINGS:
            if part in low:
                hits.append(f'{path}.{key}: value names {part}')
    return hits


def _ref_hits(payload: Mapping, spec: Condition) -> list[str]:
    hits, robot = [], payload.get('robot_id')
    for entry in payload.get('own_rgb_refs', ()) or ():
        ref = entry.get('ref') if isinstance(entry, Mapping) else None
        if not isinstance(ref, str) or not OWN_RGB_REF.match(ref):
            hits.append(f'own_rgb_refs: {ref!r} is not an own wrist RGB ref (own-<robot>-<index>)')
        elif not ref.startswith(f'own-{robot}-'):
            hits.append(f'own_rgb_refs: {ref} belongs to another robot')
    for entry in payload.get('team_rgb_refs', ()) or ():
        ref = entry.get('ref') if isinstance(entry, Mapping) else None
        if not isinstance(ref, str) or not OWN_RGB_REF.match(ref):
            hits.append(f'team_rgb_refs: {ref!r} is not a wrist RGB ref')
        elif spec.name != 'reference_R':
            hits.append('team_rgb_refs is only allowed for reference_R')
    schematic = (payload.get('static_map') or {}).get('schematic_ref')
    if isinstance(schematic, Mapping):
        ref = schematic.get('ref')
        if not isinstance(ref, str) or not MAP_SCHEMATIC_REF.match(ref):
            hits.append(f'static_map.schematic_ref: {ref!r} is not a map schematic ref')
    return hits


def _shape_hits(payload: Mapping, spec: Condition, seed: int | None) -> list[str]:
    hits = []
    for entry in payload.get('own_command_history', ()) or ():
        if not isinstance(entry, Mapping) or not {'command_id', 'issued_at_sim_s', 'kind'} <= set(entry):
            hits.append('own_command_history entries need command_id, issued_at_sim_s and kind')
        elif entry.get('local_state') not in (None,) + LOCAL_STATES:
            hits.append(f'own_command_history: local_state {entry.get("local_state")!r} is not a self state')
    belief = payload.get('self_belief')
    if belief is not None:
        if not isinstance(belief, Mapping):
            hits.append('self_belief must be an object')
        else:
            unknown = [k for k in belief if k not in BELIEF_KEYS]
            if unknown:
                hits.append(f'self_belief has non-contract key(s): {sorted(unknown)}')
    inbox = payload.get('inbox')
    if inbox is not None:
        if spec.topology == 'none':
            hits.append('no_comm must not carry an inbox')
        for envelope in inbox if isinstance(inbox, Sequence) and not isinstance(inbox, str) else []:
            if not isinstance(envelope, Mapping):
                hits.append('inbox entries must be message envelopes')
                continue
            missing = [k for k in ('message_id', 'sender', 'recipients', 'encoding', 'body') if k not in envelope]
            if missing:
                hits.append(f'inbox envelope misses {missing}')
                continue
            if envelope['encoding'] != spec.encoding:
                hits.append(f'inbox envelope encoding {envelope["encoding"]!r} differs from the condition')
            if payload.get('robot_id') not in envelope['recipients']:
                hits.append(f'inbox envelope {envelope["message_id"]} was not addressed to this robot')
            hits.extend(message_violations(spec.name, envelope['sender'], [payload.get('robot_id')],
                                           envelope['body'], seed=seed))
    if spec.leader_rotation:
        if seed is not None and payload.get('leader_id') != leader_for_seed(spec.name, seed):
            hits.append('leader_id differs from the seed rotation')
        if payload.get('role') not in ('leader', 'follower'):
            hits.append('leader_ko payloads carry role=leader|follower')
    order_sheet = payload.get('order_sheet')
    if not isinstance(order_sheet, Mapping) or order_sheet.get('schema') != ORDER_SHEET_SCHEMA:
        hits.append(f'order_sheet must carry schema {ORDER_SHEET_SCHEMA}')
    static_map = payload.get('static_map')
    if not isinstance(static_map, Mapping) or 'public_map' not in static_map:
        hits.append('static_map must carry the public_map projection')
    elif static_map.get('public_map_sha256') and digest(static_map['public_map']) != static_map['public_map_sha256']:
        hits.append('static_map.public_map_sha256 does not match the projection')
    channel = payload.get('channel')
    if not isinstance(channel, Mapping) or channel.get('condition') != spec.name:
        hits.append('channel must describe this condition')
    return hits


def payload_violations(payload: object, *, seed: int | None = None) -> list[str]:
    """Every contract violation of a robot-facing per-call payload (empty list = clean)."""
    if not isinstance(payload, Mapping):
        return ['payload must be an object']
    out: list[str] = []
    if payload.get('schema') != PAYLOAD_SCHEMA:
        out.append(f'payload schema must be {PAYLOAD_SCHEMA}, got {payload.get("schema")!r}')
    name = payload.get('condition')
    if name not in CONDITIONS:
        return out + [f'unknown condition: {name!r}']
    spec = CONDITIONS[name]
    actor = payload.get('robot_id')
    if actor not in spec.actors:
        out.append(f'{actor!r} is not an actor of {name}')
    if spec.topology == 'commander_downlink' and actor != COMMANDER:
        out.append('reference_R robots carry no LLM, so they receive no payload')
    allow = spec.input_allowlist
    extra = [k for k in payload if k not in allow]
    if extra:
        out.append(f'key(s) outside the {name} input allowlist: {sorted(extra)}')
    missing = [k for k in spec.required_keys if k not in payload]
    if missing:
        out.append(f'missing required key(s): {missing}')
    time_s = payload.get('sim_time_s')
    if isinstance(time_s, bool) or not isinstance(time_s, (int, float)) or time_s < 0:
        out.append('sim_time_s must be a non-negative number')
    out.extend(forbidden_key_hits(payload))
    out.extend(non_ascii_keys(payload))
    out.extend(_value_hits(payload))
    out.extend(_ref_hits(payload, spec))
    out.extend(_shape_hits(payload, spec, seed))
    return out


def validate_robot_payload(payload: object, *, seed: int | None = None) -> Mapping:
    """Raise ``ContractViolation`` unless the payload is inside the robot-facing boundary."""
    problems = payload_violations(payload, seed=seed)
    if problems:
        raise ContractViolation('robot-facing payload violates the study contract: ' + '; '.join(problems))
    return payload


# ---------------------------------------------------------------------------
# Log schema (written by the runner, read by package D and package I)

CALL_STATUS = ('ok', 'invalid_json', 'rejected_message', 'timeout', 'http_error', 'budget_exhausted',
               'policy_refusal')
ACTION_KINDS = ('claim_order', 'goto', 'observe', 'grasp', 'place', 'release', 'wait', 'yield_passage',
                'abort_job', 'noop')
DELIVERY_STATUS = ('delivered', 'pending', 'rejected', 'dropped_budget')
CALL_FIELDS = {
    'schema': 'literal ugrp.zone_study_call.v1',
    'run_id': 'str; one physical trial',
    'condition': 'str; key of CONDITIONS',
    'seed': 'int; paired across conditions',
    'actor': 'str; r1|r2|r3|commander',
    'role': 'str; leader|follower|peer|commander|executor',
    'request_id': 'str; unique inside the run',
    'call_index': 'int; per-actor order, from 0',
    'trigger': 'str; start|own_view_change|own_timer|message_received|idle_review|execution_review',
    'requested_at_sim_s': 'float; when the input was captured',
    'released_at_sim_s': 'float; when the answer reached the executor (>= requested_at_sim_s)',
    'sim_cost_s': 'float; released - requested, from the deterministic cost model (package D)',
    'cost_terms': 'object; {alpha_s, beta_s_per_token, gamma_s_per_utterance, output_tokens, utterances}',
    'input_sha256': 'str; digest of the validated payload',
    'input_tokens': 'object; {text, image, cached}',
    'output_tokens': 'int',
    'wall_latency_s': 'float|null; measured API latency, never used for SIM order',
    'http_attempts': 'int; >= 1 for a logical call that reached a provider',
    'status': f'str; one of {CALL_STATUS}',
    'action_id': 'str|null; the action this call submitted',
    'message_ids': 'list[str]; messages this call emitted',
    'decision_sources': 'list[str]; own refs, own command ids and message ids the answer cited',
    'payload_validated': 'bool; validate_robot_payload passed before the call',
}
MESSAGE_FIELDS = {
    'schema': 'literal ugrp.zone_study_message_log.v1',
    'run_id': 'str', 'condition': 'str', 'seed': 'int',
    'message_id': 'str', 'sender': 'str', 'recipients': 'list[str]',
    'encoding': 'str; none|free_ko|schema',
    'reply_to': 'str|null; message_id',
    'created_at_sim_s': 'float', 'delivered_at_sim_s': 'float|null',
    'delivery_delay_s': 'float; deterministic transport delay (package D)',
    'status': f'str; one of {DELIVERY_STATUS}',
    'rejected_reason': 'str|null; contract violation text when status=rejected',
    'body': 'object; {"text": str} for free_ko, the schema object for structured',
    'body_sha256': 'str',
    'act': 'str|null; structured act, or the labelled act of a free message (package I)',
    'chars': 'int; free-text length, 0 for schema messages',
    'korean_ok': 'bool|null; free_text_report(...)["ok"], null for schema messages',
}
ACTION_FIELDS = {
    'schema': 'literal ugrp.zone_study_action.v1',
    'run_id': 'str', 'condition': 'str', 'seed': 'int', 'actor': 'str',
    'action_id': 'str', 'request_id': 'str; the call that produced it',
    'submitted_at_sim_s': 'float; = the call release time',
    'kind': f'str; one of {ACTION_KINDS}',
    'arguments': 'object; own-frame targets, order ids, roles; no ground-truth pose',
    'order_id': 'str|null', 'role': 'str|null',
    'accepted': 'bool; the executor accepted the command',
    'rejected_reason': 'str|null',
    'local_state': f'str; one of {LOCAL_STATES}',
}
LOG_SCHEMAS = {CALL_LOG_SCHEMA: CALL_FIELDS, MESSAGE_LOG_SCHEMA: MESSAGE_FIELDS,
               ACTION_LOG_SCHEMA: ACTION_FIELDS}
_NUMBER = (int, float)


def _require(record: Mapping, fields: Mapping, schema: str) -> tuple[list[str], bool]:
    """(problems, complete): ``complete`` is False when required fields are missing."""
    if not isinstance(record, Mapping):
        return ['record must be an object'], False
    missing = sorted(set(fields) - set(record))
    out = [f'missing field(s): {missing}'] if missing else []
    extra = sorted(set(record) - set(fields))
    if extra:
        out.append(f'unknown field(s): {extra}')
    if record.get('schema') != schema:
        out.append(f'schema must be {schema}')
    if record.get('condition') not in CONDITIONS:
        out.append(f'unknown condition: {record.get("condition")!r}')
    if isinstance(record.get('seed'), bool) or not isinstance(record.get('seed'), int):
        out.append('seed must be an int')
    return out, not missing


def call_record_violations(record: Mapping) -> list[str]:
    out, complete = _require(record, CALL_FIELDS, CALL_LOG_SCHEMA)
    if not complete:
        return out
    for key in ('requested_at_sim_s', 'released_at_sim_s', 'sim_cost_s'):
        value = record.get(key)
        if isinstance(value, bool) or not isinstance(value, _NUMBER) or value < 0:
            out.append(f'{key} must be a non-negative number')
    if all(isinstance(record.get(k), _NUMBER) for k in ('requested_at_sim_s', 'released_at_sim_s', 'sim_cost_s')):
        if record['released_at_sim_s'] < record['requested_at_sim_s']:
            out.append('released_at_sim_s must not precede requested_at_sim_s')
        if abs((record['released_at_sim_s'] - record['requested_at_sim_s']) - record['sim_cost_s']) > 1e-9:
            out.append('sim_cost_s must equal released_at_sim_s - requested_at_sim_s')
    if record.get('status') not in CALL_STATUS:
        out.append(f'status must be one of {CALL_STATUS}')
    if isinstance(record.get('http_attempts'), bool) or not isinstance(record.get('http_attempts'), int) \
            or record.get('http_attempts', -1) < 0:
        out.append('http_attempts must be a non-negative int')
    if record.get('payload_validated') is not True:
        out.append('payload_validated must be True: the payload passed validate_robot_payload')
    if not isinstance(record.get('message_ids'), list) or not isinstance(record.get('decision_sources'), list):
        out.append('message_ids and decision_sources must be lists')
    return out


def message_record_violations(record: Mapping) -> list[str]:
    out, complete = _require(record, MESSAGE_FIELDS, MESSAGE_LOG_SCHEMA)
    if not complete:
        return out
    if record.get('status') not in DELIVERY_STATUS:
        out.append(f'status must be one of {DELIVERY_STATUS}')
    spec = CONDITIONS.get(record.get('condition'))
    if spec is not None and record.get('encoding') != spec.encoding:
        out.append('encoding must match the condition')
    created, delivered = record.get('created_at_sim_s'), record.get('delivered_at_sim_s')
    if isinstance(created, bool) or not isinstance(created, _NUMBER) or created < 0:
        out.append('created_at_sim_s must be a non-negative number')
    if delivered is not None:
        if isinstance(delivered, bool) or not isinstance(delivered, _NUMBER):
            out.append('delivered_at_sim_s must be a number or null')
        elif isinstance(created, _NUMBER) and delivered < created:
            out.append('delivered_at_sim_s must not precede created_at_sim_s')
    elif record.get('status') == 'delivered':
        out.append('a delivered message needs delivered_at_sim_s')
    if record.get('body_sha256') and isinstance(record.get('body'), (Mapping, str)):
        if digest(record['body']) != record['body_sha256']:
            out.append('body_sha256 does not match body')
    return out


def action_record_violations(record: Mapping) -> list[str]:
    out, complete = _require(record, ACTION_FIELDS, ACTION_LOG_SCHEMA)
    if not complete:
        return out
    if record.get('kind') not in ACTION_KINDS:
        out.append(f'kind must be one of {ACTION_KINDS}')
    if record.get('local_state') not in LOCAL_STATES:
        out.append(f'local_state must be one of {LOCAL_STATES}')
    submitted = record.get('submitted_at_sim_s')
    if isinstance(submitted, bool) or not isinstance(submitted, _NUMBER) or submitted < 0:
        out.append('submitted_at_sim_s must be a non-negative number')
    if not isinstance(record.get('arguments'), Mapping):
        out.append('arguments must be an object')
    else:
        out.extend(forbidden_key_hits({'arguments': record['arguments']}))
    if record.get('accepted') not in (True, False):
        out.append('accepted must be a bool')
    return out


_VALIDATORS = {CALL_LOG_SCHEMA: call_record_violations, MESSAGE_LOG_SCHEMA: message_record_violations,
               ACTION_LOG_SCHEMA: action_record_violations}


def validate_log_record(record: Mapping) -> Mapping:
    """Raise unless the record matches its declared log schema."""
    schema = record.get('schema') if isinstance(record, Mapping) else None
    if schema not in _VALIDATORS:
        raise ContractViolation(f'unknown log schema: {schema!r} (known: {sorted(_VALIDATORS)})')
    problems = _VALIDATORS[schema](record)
    if problems:
        raise ContractViolation(f'{schema} record is invalid: ' + '; '.join(problems))
    return record
