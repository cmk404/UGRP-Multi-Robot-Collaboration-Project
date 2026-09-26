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


#: Length of the opaque scenario reference a robot receives.
SCENARIO_REF_HEX = 12
SCENARIO_REF = re.compile(r'^sc_[0-9a-f]{%d}$' % SCENARIO_REF_HEX)


def scenario_ref(scenario_id: str) -> str:
    """Opaque robot-facing reference of a scenario (2026-09-26 review finding 17).

    The descriptive config id (``s5_moved_dropped_item``) names the hidden event
    KIND before the robot could observe anything, so the order sheet carries this
    derived token instead. The mapping back stays in the run manifest and the
    trial record, which are evaluation data.
    """
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ContractViolation('scenario_id must be a non-empty string')
    if SCENARIO_REF.match(scenario_id):
        # Idempotent: E's ``public_part()`` already carries the opaque ref
        # (second review, finding 17), and the sheet built from it must equal the
        # sheet built from the full config.
        return scenario_id
    return 'sc_' + hashlib.sha256(scenario_id.encode()).hexdigest()[:SCENARIO_REF_HEX]


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
    latin word is reported as a language slip.

    User decision 2026-09-26 (review finding 8): a non-Korean body is DELIVERED,
    flagged and costed, never blocking. So ``blocking_reasons`` carries only the
    structural failures (no body at all) that make an utterance unsendable, and
    the language slip lives in ``language_reasons``/``ok``. A/C share this split,
    so a sender's language mistake can no longer make the RECIPIENT's next call
    fail to build.
    """
    if not isinstance(text, str) or not text.strip():
        return {'ok': False, 'reasons': ['free message body needs non-empty text'],
                'blocking_reasons': ['free message body needs non-empty text'],
                'language_reasons': [], 'has_korean': False, 'latin_words': [], 'chars': 0}
    allowed = set(literals) | ALWAYS_LITERAL
    latin_words = [run for run in LATIN_RUN.findall(text)
                   if run not in allowed and len(re.findall('[A-Za-z]', run)) > 1]
    has_korean = bool(HANGUL.search(text))
    language = [] if has_korean else ['free message body has no Korean text']
    return {'ok': not language, 'reasons': language, 'blocking_reasons': [],
            'language_reasons': language, 'has_korean': has_korean,
            'latin_words': latin_words, 'chars': len(text)}


def language_violations(name: str, body: object, *, literals: Sequence[str] = ()) -> list[str]:
    """Language slips of one message body. Flagged and costed, never blocking."""
    if condition(name).encoding != 'free_ko' or not isinstance(body, Mapping):
        return []
    return list(free_text_report(body.get('text'), literals=literals)['language_reasons'])


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
            # Language slips are flagged (``language_violations``), not blocking.
            out.extend(free_text_report(body['text'])['blocking_reasons'])
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
SHA256_HEX = re.compile(r'^[0-9a-f]{64}$')
ID_TOKEN = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_.\-]*$')
LOCAL_STATES = ('command_issued', 'queue_empty', 'hold_requested', 'local_timeout', 'command_rejected')
BELIEF_KEYS = ('region', 'last_visual_anchor', 'last_requested_destination', 'last_visually_confirmed_region',
               'confidence', 'sources', 'held_item_guess', 'blocked_passages', 'notes_ko')
# Closed sub-schemas: a robot-facing section may carry NO key beyond these, so an
# evaluation-only field cannot slip in under a new name (renamed poses, overhead
# stills, peer views, host boards, hidden schedules, metrics). Extending a tuple
# is a contract change: bump CONTRACT_VERSION and re-run the boundary tests.
STATIC_MAP_KEYS = ('map_id', 'map_file_sha256', 'public_map', 'public_map_sha256', 'schematic_ref')
STATIC_MAP_REQUIRED = ('map_id', 'public_map', 'public_map_sha256')
SCHEMATIC_REF_KEYS = ('ref', 'kind', 'png_sha256', 'width_px', 'height_px', 'px_per_m')
PUBLIC_MAP_KEYS = ('schema', 'map_id', 'version', 'frame', 'bounds_m', 'walls', 'terrain', 'passages',
                   'regions', 'zone_slots', 'pickup_bays', 'item_kinds_painted', 'approach_convention',
                   'landmark_detail', 'landmarks', 'base_map_id')
# Closed sub-schemas INSIDE the map projection (2026-09-26 review finding 3): a
# renamed live field cannot ride along in a wall, a passage, a slot or a tag just
# because the provider recomputed the projection hash.
WALL_KEYS = ('id', 'center_m', 'half_extents_m', 'height_m', 'perimeter')
TERRAIN_KEYS = ('id', 'kind', 'center_m', 'half_extents_m', 'height_m', 'slope_deg', 'passable')
PASSAGE_KEYS = ('id', 'kind', 'axis', 'center_m', 'half_extents_m', 'connects', 'opens_to', 'lanes',
                'width_m')
REGION_KEYS = ('map_key', 'center_m', 'half_extents_m', 'paint_rgba')
SLOT_KEYS = ('slot_id', 'center_m', 'half_extents_m')
BAY_KEYS = ('bay_id', 'center_m', 'half_extents_m', 'slots')
LANDMARKS_KEYS = ('schema', 'family', 'placement', 'tag_count', 'tag_frame', 'tags')
TAG_KEYS = ('id', 'wall', 'center_m', 'normal_xy', 'size_m', 'yaw_rad')
ORDER_KEYS = ('order_id', 'kind', 'count', 'item_ids', 'required_robots', 'destination_zone',
              'initial_location', 'identity')
#: How an order names what must be delivered. A closed enum, not free text.
ORDER_IDENTITIES = ('specific_item', 'kind_fungible')
ORDER_SHEET_KEYS = ('schema', 'scenario_id', 'map_id', 'map_file_sha256', 'public_map_sha256', 'orders',
                    'kinds', 'team_size', 'note_ko')
#: ``order_sheet.kinds.<kind>``: the static team requirement of one kind.
KIND_KEYS = ('required_robots', 'roles')
RGB_REF_KEYS = ('ref', 'kind', 'captured_at_sim_s', 'sha256')
COMMAND_KEYS = ('command_id', 'issued_at_sim_s', 'kind', 'arguments', 'local_state')
# Arguments of the robot's OWN commands: own-frame targets and map/order ids only.
COMMAND_ARGUMENT_KEYS = ('target_ref', 'target_zone', 'order_id', 'item', 'role', 'passage', 'distance_m',
                         'turn_deg', 'speed', 'duration_s', 'gripper', 'observe', 'waypoints', 'reason_code')
ISSUED_ORDER_KEYS = ('order_ref', 'to', 'at_sim_s', 'instruction')
CHANNEL_KEYS = ('condition', 'topology', 'encoding', 'free_text_allowed', 'can_send_to',
                'can_receive_from', 'role')
ENVELOPE_KEYS = ('schema', 'message_id', 'sender', 'recipients', 'encoding', 'created_at_sim_s',
                 'reply_to', 'body')
ENVELOPE_REQUIRED = ('message_id', 'sender', 'recipients', 'encoding', 'created_at_sim_s', 'body')
# Item kinds, their team size and their grasp roles (static task information).
ROLE_NAMES = ('west', 'east', 'any', 'end_neg', 'end_pos', 'v0', 'v1', 'v2')


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
    """Evaluation-only source names in HOST-built values.

    2026-09-26 second review: the text of a delivered message is what another
    ROBOT wrote (``"nav_cam은 사용하지 말고 자기 카메라만 보세요."``), not a
    data source the host attached, so C accepted it and A then refused the
    recipient's payload. Message bodies are checked by the channel rules
    (``message_violations``) instead; every other value is still scanned.
    """
    if isinstance(value, Mapping) and isinstance(value.get('inbox'), Sequence) \
            and not isinstance(value.get('inbox'), (str, bytes)):
        value = {**value, 'inbox': [{k: v for k, v in env.items() if k != 'body'}
                                    if isinstance(env, Mapping) else env for env in value['inbox']]}
    hits = []
    for path, key, item in _walk(value):
        if not isinstance(item, str):
            continue
        low = unicodedata.normalize('NFKC', item).lower().replace('-', '_').replace(' ', '_')
        for part in FORBIDDEN_VALUE_SUBSTRINGS:
            if part in low:
                hits.append(f'{path}.{key}: value names {part}')
    return hits


def _closed(value: object, keys: Sequence[str], label: str, *, required: Sequence[str] = ()) -> list[str]:
    """Reject any key outside ``keys`` (closed sub-schema) and any missing required key."""
    if not isinstance(value, Mapping):
        return [f'{label} must be an object']
    out = []
    extra = sorted(set(value) - set(keys))
    if extra:
        out.append(f'{label} carries key(s) outside the contract: {extra}')
    missing = [k for k in required if k not in value]
    if missing:
        out.append(f'{label} misses {missing}')
    return out


def _is_num(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _vec(*lengths: int):
    def check(value, label):
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) \
                and len(value) in lengths and all(_is_num(v) for v in value):
            return []
        return [f'{label} must be a list of {"/".join(map(str, lengths))} numbers, got {value!r}']
    return check


def _num(value, label):
    return [] if _is_num(value) else [f'{label} must be a number, got {value!r}']


def _int(value, label):
    return [] if isinstance(value, int) and not isinstance(value, bool) else \
        [f'{label} must be a whole number, got {value!r}']


def _bool(value, label):
    return [] if isinstance(value, bool) else [f'{label} must be true/false, got {value!r}']


def _str(value, label):
    return [] if isinstance(value, str) else [f'{label} must be a string, got {value!r}']


def _opt(check):
    return lambda value, label: [] if value is None else check(value, label)


def _tokens(value, label):
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [p for i, item in enumerate(value) for p in _token(item, f'{label}[{i}]')]
    return [f'{label} must be a list of ID tokens, got {value!r}']


def _strs(value, label):
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) \
            and all(isinstance(v, str) for v in value):
        return []
    return [f'{label} must be a list of strings, got {value!r}']


def _rgba(value, label):
    return _str(value, label) if isinstance(value, str) else _vec(4)(value, label)


#: Value type of every leaf key a map sub-object may carry (2026-09-26 second
#: review, finding 3). Closing the KEYS was not enough: ``center_m`` could carry
#: ``{"survey_xy_m": [...]}`` and a recomputed hash made it pass. A key listed in
#: a closed sub-schema but missing here is a contract bug (``_typed`` refuses it).
MAP_LEAF_TYPES = {
    'center_m': _vec(2, 3), 'half_extents_m': _vec(2, 3), 'height_m': _num, 'slope_deg': _num,
    'passable': _bool, 'perimeter': _bool, 'axis': _str, 'lanes': _int, 'width_m': _num,
    'connects': _strs, 'opens_to': _str, 'map_key': _str,
    'paint_rgba': _rgba, 'normal_xy': _vec(2, 3), 'size_m': _num, 'yaw_rad': _num,
    'wall': lambda v, l: _token(v, l), 'kind': lambda v, l: _token(v, l),
    'schema': _str, 'family': _str, 'tag_count': _int, 'tag_frame': _str,
    'version': _int, 'frame': _str, 'bounds_m': _vec(4), 'item_kinds_painted': _tokens,
    'approach_convention': _str, 'landmark_detail': _str, 'base_map_id': _opt(lambda v, l: _token(v, l)),
    'map_id': lambda v, l: _token(v, l),
    'width_px': _int, 'height_px': _int, 'px_per_m': _num,
}
#: ``landmarks.placement``: the fixed tag mounting geometry, numbers only.
PLACEMENT_KEYS = ('cell_thickness_m', 'center_height_m', 'end_margin_m', 'faces', 'plate_m',
                  'plate_thickness_m', 'size_m', 'spacing_m')
PLACEMENT_TYPES = {**{k: _num for k in PLACEMENT_KEYS}, 'faces': _str}


def _either(*checks):
    def check(value, label):
        results = [c(value, label) for c in checks]
        return [] if any(not r for r in results) else results[0]
    return check


_TOKEN = lambda v, l: _token(v, l)   # noqa: E731 - late binding: _token is defined below


def _points(value, label):
    """Own-frame waypoints: a list of ID tokens or of 2/3-number vectors, never objects."""
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [p for i, item in enumerate(value)
                for p in _either(_TOKEN, _vec(2, 3))(item, f'{label}[{i}]')]
    return [f'{label} must be a list, got {value!r}']


#: Value types of the robot's own belief, own command arguments and the
#: commander's issued orders (second review, finding 3): no nested object can
#: ride along under an allowed key.
BELIEF_TYPES = {'region': _str, 'last_visual_anchor': _opt(_str), 'last_requested_destination': _opt(_str),
                'last_visually_confirmed_region': _opt(_str), 'confidence': _either(_str, _num), 'sources': _strs,
                'held_item_guess': _opt(_str), 'blocked_passages': _tokens, 'notes_ko': _str}
COMMAND_ARGUMENT_TYPES = {
    'target_ref': _opt(_TOKEN), 'target_zone': _opt(_TOKEN), 'order_id': _opt(_TOKEN), 'item': _opt(_TOKEN),
    'role': _opt(_TOKEN), 'passage': _opt(_TOKEN), 'distance_m': _num, 'turn_deg': _num, 'speed': _num,
    'duration_s': _num, 'gripper': _either(_str, _num), 'observe': _either(_bool, _str),
    'waypoints': _points, 'reason_code': _opt(_TOKEN)}
ISSUED_ORDER_TYPES = {'order_ref': _TOKEN, 'to': _TOKEN, 'at_sim_s': _num, 'instruction': _str}


def _typed(value: object, label: str, *, skip: Sequence[str] = (), types: Mapping = MAP_LEAF_TYPES) -> list[str]:
    """Type check every present leaf of one closed sub-object."""
    if not isinstance(value, Mapping):
        return []
    out = []
    for key, item in value.items():
        if key in skip:
            continue
        check = types.get(key)
        if check is None:
            out.append(f'{label}.{key} has no declared value type in the contract')
            continue
        out.extend(check(item, f'{label}.{key}'))
    return out


def _sha(value: object, label: str, *, required: bool = True) -> list[str]:
    if value is None and not required:
        return []
    return [] if isinstance(value, str) and SHA256_HEX.match(value) else [f'{label} must be a sha256 hex digest']


def vocabulary_from_payload(payload: Mapping) -> Vocabulary:
    """The IDs a message may name, read from the payload's own order sheet and public map."""
    sheet = payload.get('order_sheet') if isinstance(payload.get('order_sheet'), Mapping) else {}
    static = payload.get('static_map') if isinstance(payload.get('static_map'), Mapping) else {}
    public = static.get('public_map') if isinstance(static.get('public_map'), Mapping) else {}
    items: set = set()
    for order in sheet.get('orders', ()) or ():
        if isinstance(order, Mapping):
            # Malformed values are reported by the shape checks; the vocabulary
            # must not raise on them, or a violation would become a crash.
            candidates = [order.get('order_id'), order.get('kind')]
            ids = order.get('item_ids')
            if isinstance(ids, Sequence) and not isinstance(ids, (str, bytes)):
                candidates.extend(ids)
            items |= {v for v in candidates if isinstance(v, str)}
    refs = set(ZONE_IDS) | {'pickup'}
    for bay in public.get('pickup_bays', ()) or ():
        if isinstance(bay, Mapping):
            refs.add(bay.get('bay_id'))
            refs |= {s.get('slot_id') for s in bay.get('slots', ()) or () if isinstance(s, Mapping)}
    for slots in (public.get('zone_slots') or {}).values():
        refs |= {s.get('slot_id') for s in slots or () if isinstance(s, Mapping)}
    passages = {p.get('id') for p in public.get('passages', ()) or () if isinstance(p, Mapping)}
    keep = lambda values: frozenset(v for v in values if isinstance(v, str))   # noqa: E731
    return Vocabulary(items=keep(items), zones=frozenset(ZONE_IDS), roles=frozenset(ROLE_NAMES),
                      passages=keep(passages), location_refs=keep(refs))


def _rgb_hits(entries: object, label: str, allowed_robots: Sequence[str], now: float | None) -> list[str]:
    hits = []
    if not isinstance(entries, Sequence) or isinstance(entries, str):
        return [f'{label} must be a list']
    for entry in entries:
        hits.extend(_closed(entry, RGB_REF_KEYS, label, required=('ref', 'captured_at_sim_s', 'sha256')))
        if not isinstance(entry, Mapping):
            continue
        ref = entry.get('ref')
        if not isinstance(ref, str) or not OWN_RGB_REF.match(ref):
            hits.append(f'{label}: {ref!r} is not a wrist RGB ref (own-<robot>-<index>)')
        elif not any(ref.startswith(f'own-{robot}-') for robot in allowed_robots):
            hits.append(f'{label}: {ref} belongs to another robot')
        if entry.get('kind') not in (None, 'own_wrist_rgb'):
            hits.append(f'{label}: kind must be own_wrist_rgb')
        hits.extend(_sha(entry.get('sha256'), f'{label}.sha256'))
        taken = entry.get('captured_at_sim_s')
        if isinstance(taken, bool) or not isinstance(taken, (int, float)) or taken < 0:
            hits.append(f'{label}.captured_at_sim_s must be a non-negative number')
        elif now is not None and taken > now + 1e-9:
            hits.append(f'{label}: {ref} was captured after the call time')
    return hits


def _token(value: object, label: str) -> list[str]:
    """A literal ID token, i.e. never a coordinate, a number or a nested object."""
    if not isinstance(value, str) or not ID_TOKEN.match(value):
        return [f'{label} must be a literal ID token, got {value!r}']
    return []


def _whole(value: object, label: str, *, minimum: int = 0) -> list[str]:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        return [f'{label} must be a whole number >= {minimum}, got {value!r}']
    return []


def _seq(value: object, label: str) -> tuple[list[str], list]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return [f'{label} must be a list'], []
    return [], list(value)


def map_slot_ids(public: object) -> frozenset[str]:
    """Every pickup-bay and zone slot id the public map declares."""
    out: set[str] = set()
    if not isinstance(public, Mapping):
        return frozenset()
    for bay in public.get('pickup_bays', ()) or ():
        if isinstance(bay, Mapping):
            for slot in bay.get('slots', ()) or ():
                if isinstance(slot, Mapping) and isinstance(slot.get('slot_id'), str):
                    out.add(slot['slot_id'])
    for slots in (public.get('zone_slots') or {}).values():
        for slot in slots or ():
            if isinstance(slot, Mapping) and isinstance(slot.get('slot_id'), str):
                out.add(slot['slot_id'])
    return frozenset(out)


def map_bay_ids(public: object) -> frozenset[str]:
    if not isinstance(public, Mapping):
        return frozenset()
    return frozenset(bay['bay_id'] for bay in public.get('pickup_bays', ()) or ()
                     if isinstance(bay, Mapping) and isinstance(bay.get('bay_id'), str))


def _public_map_hits(public: object) -> list[str]:
    """Close the map projection down to its leaf objects, keys AND value types."""
    hits = _closed(public, PUBLIC_MAP_KEYS, 'static_map.public_map',
                   required=('map_id', 'bounds_m', 'walls', 'regions'))
    if not isinstance(public, Mapping):
        return hits
    hits.extend(_typed(public, 'static_map.public_map',
                       skip=('walls', 'terrain', 'passages', 'regions', 'zone_slots', 'pickup_bays',
                             'landmarks')))
    for name, keys, required in (('walls', WALL_KEYS, ('id',)), ('terrain', TERRAIN_KEYS, ('id',)),
                                 ('passages', PASSAGE_KEYS, ('id',))):
        if name not in public:
            continue
        problems, rows = _seq(public.get(name), f'static_map.public_map.{name}')
        hits.extend(problems)
        for row in rows:
            label = f'static_map.public_map.{name}[]'
            hits.extend(_closed(row, keys, label, required=required))
            if isinstance(row, Mapping):
                hits.extend(_token(row.get('id'), f'{label}.id'))
                hits.extend(_typed(row, label, skip=('id',)))
    for name in ('regions', 'zone_slots'):
        section = public.get(name)
        if name in public and not isinstance(section, Mapping):
            hits.append(f'static_map.public_map.{name} must be an object')
            continue
        for key, value in (section or {}).items():
            label = f'static_map.public_map.{name}.{key}'
            hits.extend(_token(key, f'static_map.public_map.{name} key'))
            if name == 'regions':
                hits.extend(_closed(value, REGION_KEYS, label))
                hits.extend(_typed(value, label))
                continue
            problems, rows = _seq(value, label)
            hits.extend(problems)
            for slot in rows:
                hits.extend(_closed(slot, SLOT_KEYS, f'{label}[]', required=('slot_id',)))
                if isinstance(slot, Mapping):
                    hits.extend(_token(slot.get('slot_id'), f'{label}[].slot_id'))
                    hits.extend(_typed(slot, f'{label}[]', skip=('slot_id',)))
    if 'pickup_bays' in public:
        problems, bays = _seq(public.get('pickup_bays'), 'static_map.public_map.pickup_bays')
        hits.extend(problems)
        for bay in bays:
            label = 'static_map.public_map.pickup_bays[]'
            hits.extend(_closed(bay, BAY_KEYS, label, required=('bay_id',)))
            if not isinstance(bay, Mapping):
                continue
            hits.extend(_token(bay.get('bay_id'), f'{label}.bay_id'))
            hits.extend(_typed(bay, label, skip=('bay_id', 'slots')))
            problems, slots = _seq(bay.get('slots', ()), f'{label}.slots')
            hits.extend(problems)
            for slot in slots:
                hits.extend(_closed(slot, SLOT_KEYS, f'{label}.slots[]', required=('slot_id',)))
                if isinstance(slot, Mapping):
                    hits.extend(_token(slot.get('slot_id'), f'{label}.slots[].slot_id'))
                    hits.extend(_typed(slot, f'{label}.slots[]', skip=('slot_id',)))
    if 'landmarks' in public:
        marks = public['landmarks']
        hits.extend(_closed(marks, LANDMARKS_KEYS, 'static_map.public_map.landmarks'))
        if isinstance(marks, Mapping):
            hits.extend(_typed(marks, 'static_map.public_map.landmarks', skip=('placement', 'tags')))
            if 'placement' in marks:
                hits.extend(_closed(marks['placement'], PLACEMENT_KEYS,
                                    'static_map.public_map.landmarks.placement'))
                hits.extend(_typed(marks['placement'], 'static_map.public_map.landmarks.placement',
                                   types=PLACEMENT_TYPES))
            problems, tags = _seq(marks.get('tags', ()), 'static_map.public_map.landmarks.tags')
            hits.extend(problems)
            for tag in tags:
                hits.extend(_closed(tag, TAG_KEYS, 'static_map.public_map.landmarks.tags[]',
                                    required=('id',)))
                if isinstance(tag, Mapping):
                    hits.extend(_int(tag.get('id'), 'static_map.public_map.landmarks.tags[].id'))
                    hits.extend(_typed(tag, 'static_map.public_map.landmarks.tags[]', skip=('id',)))
    return hits


def _static_map_hits(payload: Mapping) -> list[str]:
    static = payload.get('static_map')
    hits = _closed(static, STATIC_MAP_KEYS, 'static_map', required=STATIC_MAP_REQUIRED)
    if not isinstance(static, Mapping):
        return hits
    hits.extend(_sha(static.get('public_map_sha256'), 'static_map.public_map_sha256'))
    hits.extend(_sha(static.get('map_file_sha256'), 'static_map.map_file_sha256', required=False))
    public = static.get('public_map')
    hits.extend(_public_map_hits(public))
    if isinstance(public, Mapping) and isinstance(static.get('public_map_sha256'), str) \
            and digest(public) != static['public_map_sha256']:
        hits.append('static_map.public_map_sha256 does not match the projection')
    if 'schematic_ref' in static:
        hits.extend(_closed(static['schematic_ref'], SCHEMATIC_REF_KEYS, 'static_map.schematic_ref',
                            required=('ref', 'png_sha256')))
        if isinstance(static['schematic_ref'], Mapping):
            schematic = static['schematic_ref']
            ref = schematic.get('ref')
            if not isinstance(ref, str) or not MAP_SCHEMATIC_REF.match(ref):
                hits.append(f'static_map.schematic_ref: {ref!r} is not a map schematic ref')
            elif ref != f'map-{static.get("map_id")}-schematic':
                hits.append(f'static_map.schematic_ref: {ref!r} is not the schematic of map '
                            f'{static.get("map_id")!r}')
            if schematic.get('kind', 'map_schematic') != 'map_schematic':
                hits.append('static_map.schematic_ref.kind must be map_schematic')
            hits.extend(_sha(schematic.get('png_sha256'), 'static_map.schematic_ref.png_sha256'))
            hits.extend(_typed(schematic, 'static_map.schematic_ref', skip=('ref', 'kind', 'png_sha256')))
    return hits


def _order_sheet_hits(payload: Mapping) -> list[str]:
    sheet = payload.get('order_sheet')
    hits = _closed(sheet, ORDER_SHEET_KEYS, 'order_sheet', required=('schema', 'orders'))
    if not isinstance(sheet, Mapping):
        return hits
    if sheet.get('schema') != ORDER_SHEET_SCHEMA:
        hits.append(f'order_sheet must carry schema {ORDER_SHEET_SCHEMA}')
    # The robot-facing sheet names the scenario only by its opaque ref: a
    # descriptive id would announce the hidden event kind (review finding 17).
    if 'scenario_id' in sheet and not SCENARIO_REF.match(str(sheet.get('scenario_id'))):
        hits.append('order_sheet.scenario_id must be the opaque scenario_ref '
                    '(sc_<12 hex>), not the descriptive config id')
    static = payload.get('static_map') if isinstance(payload.get('static_map'), Mapping) else {}
    public = static.get('public_map') if isinstance(static.get('public_map'), Mapping) else {}
    bays, slots = map_bay_ids(public), map_slot_ids(public)
    # second review, finding 3: the non-order fields are typed too.
    for key in ('map_file_sha256', 'public_map_sha256'):
        if key in sheet:
            hits.extend(_sha(sheet.get(key), f'order_sheet.{key}'))
    if 'map_id' in sheet:
        hits.extend(_token(sheet.get('map_id'), 'order_sheet.map_id'))
    if 'team_size' in sheet:
        hits.extend(_whole(sheet.get('team_size'), 'order_sheet.team_size', minimum=1))
    if 'note_ko' in sheet:
        hits.extend(_str(sheet.get('note_ko'), 'order_sheet.note_ko'))
    if 'kinds' in sheet:
        table = sheet.get('kinds')
        if not isinstance(table, Mapping):
            hits.append('order_sheet.kinds must be an object')
        else:
            for kind, row in table.items():
                label = f'order_sheet.kinds.{kind}'
                hits.extend(_token(kind, 'order_sheet.kinds key'))
                hits.extend(_closed(row, KIND_KEYS, label, required=KIND_KEYS))
                if isinstance(row, Mapping):
                    hits.extend(_whole(row.get('required_robots'), f'{label}.required_robots', minimum=1))
                    roles = row.get('roles')
                    hits.extend(_tokens(roles, f'{label}.roles'))
                    if isinstance(roles, Sequence) and not isinstance(roles, (str, bytes)):
                        hits.extend(f'{label}.roles: {r!r} is not a role name' for r in roles
                                    if r not in ROLE_NAMES)
    orders = sheet.get('orders')
    if not isinstance(orders, Sequence) or isinstance(orders, str):
        return hits + ['order_sheet.orders must be a list']
    for order in orders:
        hits.extend(_closed(order, ORDER_KEYS, 'order_sheet.orders[]',
                            required=('order_id', 'kind', 'count', 'destination_zone', 'initial_location')))
        if not isinstance(order, Mapping):
            continue
        label = f'order_sheet.orders[{order.get("order_id")!r}]'
        hits.extend(_token(order.get('order_id'), f'{label}.order_id'))
        hits.extend(_token(order.get('kind'), f'{label}.kind'))
        hits.extend(_whole(order.get('count'), f'{label}.count', minimum=1))
        if 'required_robots' in order:
            hits.extend(_whole(order.get('required_robots'), f'{label}.required_robots', minimum=1))
        if 'identity' in order and order.get('identity') not in ORDER_IDENTITIES:
            hits.append(f'{label}.identity must be one of {ORDER_IDENTITIES}, got {order.get("identity")!r}')
        if 'item_ids' in order:
            problems, items = _seq(order.get('item_ids'), f'{label}.item_ids')
            hits.extend(problems)
            for item in items:
                hits.extend(_token(item, f'{label}.item_ids[]'))
        if order.get('destination_zone') not in ZONE_IDS:
            hits.append(f'order {order.get("order_id")!r} has a destination outside {ZONE_IDS}')
        where = order.get('initial_location')
        hits.extend(_closed(where, ('pickup_bay', 'slot'), f'{label}.initial_location',
                            required=('pickup_bay',)))
        if not isinstance(where, Mapping):
            continue
        # A coarse location is map VOCABULARY, never a coordinate: a list, a
        # number or an unknown id here would be exact-pose smuggling.
        hits.extend(_token(where.get('pickup_bay'), f'{label}.initial_location.pickup_bay'))
        if bays and isinstance(where.get('pickup_bay'), str) and where['pickup_bay'] not in bays:
            hits.append(f'{label}.initial_location.pickup_bay {where["pickup_bay"]!r} is not a bay of the '
                        'static map')
        if where.get('slot') is not None:
            hits.extend(_token(where.get('slot'), f'{label}.initial_location.slot'))
            if slots and isinstance(where.get('slot'), str) and where['slot'] not in slots:
                hits.append(f'{label}.initial_location.slot {where["slot"]!r} is not a slot of the static map')
    return hits


def _history_hits(payload: Mapping, now: float | None) -> list[str]:
    entries = payload.get('own_command_history')
    if not isinstance(entries, Sequence) or isinstance(entries, str):
        return ['own_command_history must be a list']
    hits, previous = [], None
    for entry in entries:
        hits.extend(_closed(entry, COMMAND_KEYS, 'own_command_history[]',
                            required=('command_id', 'issued_at_sim_s', 'kind')))
        if not isinstance(entry, Mapping):
            continue
        if not isinstance(entry.get('command_id'), str) or not ID_TOKEN.match(str(entry.get('command_id'))):
            hits.append('own_command_history[].command_id must be a literal id token')
        if entry.get('local_state') not in (None,) + LOCAL_STATES:
            hits.append(f'own_command_history: local_state {entry.get("local_state")!r} is not a self state')
        hits.extend(_closed(entry.get('arguments', {}), COMMAND_ARGUMENT_KEYS,
                            'own_command_history[].arguments'))
        hits.extend(_typed(entry.get('arguments', {}), 'own_command_history[].arguments',
                           types=COMMAND_ARGUMENT_TYPES))
        hits.extend(_typed({k: v for k, v in entry.items() if k in ('kind',)}, 'own_command_history[]',
                           types={'kind': lambda v, l: _token(v, l)}))
        issued = entry.get('issued_at_sim_s')
        if isinstance(issued, bool) or not isinstance(issued, (int, float)) or issued < 0:
            hits.append('own_command_history[].issued_at_sim_s must be a non-negative number')
            continue
        if now is not None and issued > now + 1e-9:
            hits.append(f'own_command_history: {entry.get("command_id")} was issued after the call time')
        if previous is not None and issued < previous - 1e-9:
            hits.append('own_command_history must be ordered oldest first')
        previous = issued
    return hits


def _inbox_hits(payload: Mapping, spec: Condition, seed: int | None, now: float | None) -> list[str]:
    inbox = payload.get('inbox')
    if inbox is None:
        return []
    if spec.topology == 'none':
        return ['no_comm must not carry an inbox']
    if not isinstance(inbox, Sequence) or isinstance(inbox, str):
        return ['inbox must be a list']
    if spec.topology == 'star' and seed is None:
        return ['validating a leader_ko inbox needs the seed that places the rotating leader']
    hits, robot = [], payload.get('robot_id')
    vocabulary = vocabulary_from_payload(payload)
    for envelope in inbox:
        hits.extend(_closed(envelope, ENVELOPE_KEYS, 'inbox[]', required=ENVELOPE_REQUIRED))
        if not isinstance(envelope, Mapping) or not {'sender', 'recipients', 'body'} <= set(envelope):
            continue
        if envelope.get('encoding') != spec.encoding:
            hits.append(f'inbox envelope encoding {envelope.get("encoding")!r} differs from the condition')
        recipients = envelope['recipients']
        if not isinstance(recipients, Sequence) or isinstance(recipients, str):
            hits.append('inbox[].recipients must be a list')
            continue
        if robot not in recipients:
            hits.append(f'inbox envelope {envelope.get("message_id")} was not addressed to this robot')
        unknown = [r for r in recipients if r not in spec.actors]
        if unknown:
            hits.append(f'inbox envelope names recipient(s) outside this condition: {unknown}')
        hits.extend(message_violations(spec.name, envelope['sender'], recipients, envelope['body'],
                                      seed=seed, vocabulary=vocabulary))
        created = envelope.get('created_at_sim_s')
        if isinstance(created, bool) or not isinstance(created, (int, float)) or created < 0:
            hits.append('inbox[].created_at_sim_s must be a non-negative number')
        elif now is not None and created > now + 1e-9:
            hits.append('an inbox message cannot be created after the call time')
    return hits


def _shape_hits(payload: Mapping, spec: Condition, seed: int | None) -> list[str]:
    time_s = payload.get('sim_time_s')
    now = time_s if isinstance(time_s, (int, float)) and not isinstance(time_s, bool) else None
    hits = _static_map_hits(payload) + _order_sheet_hits(payload)
    if not isinstance(payload.get('request_id'), str) or not ID_TOKEN.match(str(payload.get('request_id'))):
        hits.append('request_id must be a literal id token')
    if 'own_rgb_refs' in payload:
        hits.extend(_rgb_hits(payload['own_rgb_refs'], 'own_rgb_refs', [payload.get('robot_id')], now))
    if 'team_rgb_refs' in payload:
        hits.extend(_rgb_hits(payload['team_rgb_refs'], 'team_rgb_refs', ROBOTS, now)
                    if spec.name == 'reference_R' else ['team_rgb_refs is only allowed for reference_R'])
    if 'own_command_history' in payload:
        hits.extend(_history_hits(payload, now))
    if 'issued_orders' in payload:
        for entry in payload['issued_orders'] if isinstance(payload['issued_orders'], Sequence) else []:
            hits.extend(_closed(entry, ISSUED_ORDER_KEYS, 'issued_orders[]', required=('order_ref', 'to')))
            hits.extend(_typed(entry, 'issued_orders[]', types=ISSUED_ORDER_TYPES))
    if 'self_belief' in payload:
        hits.extend(_closed(payload['self_belief'], BELIEF_KEYS, 'self_belief'))
        hits.extend(_typed(payload['self_belief'], 'self_belief', types=BELIEF_TYPES))
    hits.extend(_inbox_hits(payload, spec, seed, now))
    if spec.leader_rotation:
        if seed is None:
            hits.append('validating a leader_ko payload needs the seed that places the rotating leader')
        elif payload.get('leader_id') != leader_for_seed(spec.name, seed):
            hits.append('leader_id differs from the seed rotation')
        if payload.get('role') not in ('leader', 'follower'):
            hits.append('leader_ko payloads carry role=leader|follower')
    channel = payload.get('channel')
    hits.extend(_closed(channel, CHANNEL_KEYS, 'channel', required=CHANNEL_KEYS))
    if isinstance(channel, Mapping) and not (spec.leader_rotation and seed is None):
        expected = channel_section(spec.name, payload.get('robot_id'), seed)
        if channel != expected:
            hits.append('channel does not match the condition rule for this actor')
    return hits


ROBOT_FACING = {
    'static_map': 'the static map projection and its schematic, re-sent on every call',
    'order_sheet': 'the immutable order sheet built from the scenario config',
    'own_rgb_refs': "this robot's own wrist fisheye frames (own-<robot>-<index>)",
    'own_command_history': "this robot's own issued commands and its own command states",
    'self_belief': 'a belief built only from the inputs above',
    'inbox': 'the messages this condition actually delivered to this robot',
    'channel': "the condition's own send/receive rule",
}
EVALUATION_ONLY = {
    'cameras': 'TOP/cctv frames, their calibration and anything derived from them; the sim-only nav_cam',
    'ground_truth': 'object and robot poses, measured joints, contacts, forces, simulator identifiers, weld',
    'judgements': 'teacher receipts, grasp/placement/delivery confirmations, completion and success flags',
    'peer_state': "other robots' cameras, raw commands, claims, busy state, host boards, global progress",
    'schedule': 'hidden event schedules and injected failures',
    'metrics': 'referee counts, scores, makespan and every evaluation metric',
}


def boundary_manifest() -> dict:
    """The public/private schema as data: what a robot may see and what stays in evaluation."""
    return {'contract_version': CONTRACT_VERSION, 'payload_schema': PAYLOAD_SCHEMA,
            'robot_facing': dict(ROBOT_FACING),
            'input_allowlist': {name: sorted(spec.input_allowlist) for name, spec in CONDITIONS.items()},
            'evaluation_only': dict(EVALUATION_ONLY), 'forbidden_keys': sorted(FORBIDDEN_KEYS),
            'forbidden_key_substrings': list(FORBIDDEN_KEY_SUBSTRINGS),
            'forbidden_value_substrings': list(FORBIDDEN_VALUE_SUBSTRINGS),
            'closed_sub_schemas': {'static_map': list(STATIC_MAP_KEYS),
                                   'static_map.public_map': list(PUBLIC_MAP_KEYS),
                                   'static_map.public_map.walls[]': list(WALL_KEYS),
                                   'static_map.public_map.terrain[]': list(TERRAIN_KEYS),
                                   'static_map.public_map.passages[]': list(PASSAGE_KEYS),
                                   'static_map.public_map.regions.*': list(REGION_KEYS),
                                   'static_map.public_map.zone_slots.*[]': list(SLOT_KEYS),
                                   'static_map.public_map.pickup_bays[]': list(BAY_KEYS),
                                   'static_map.public_map.pickup_bays[].slots[]': list(SLOT_KEYS),
                                   'static_map.public_map.landmarks': list(LANDMARKS_KEYS),
                                   'static_map.public_map.landmarks.tags[]': list(TAG_KEYS),
                                   'static_map.schematic_ref': list(SCHEMATIC_REF_KEYS),
                                   'order_sheet': list(ORDER_SHEET_KEYS),
                                   'order_sheet.orders[]': list(ORDER_KEYS),
                                   'order_sheet.orders[].initial_location': ['pickup_bay', 'slot'],
                                   'own_rgb_refs[]': list(RGB_REF_KEYS),
                                   'own_command_history[]': list(COMMAND_KEYS),
                                   'own_command_history[].arguments': list(COMMAND_ARGUMENT_KEYS),
                                   'inbox[]': list(ENVELOPE_KEYS), 'channel': list(CHANNEL_KEYS),
                                   'self_belief': list(BELIEF_KEYS),
                                   'issued_orders[]': list(ISSUED_ORDER_KEYS)},
            'order_identities': list(ORDER_IDENTITIES), 'pinned_keys': list(PINNED_KEYS),
            'own_rgb_ref_pattern': OWN_RGB_REF.pattern, 'map_schematic_ref_pattern': MAP_SCHEMATIC_REF.pattern,
            'own_command_states': list(LOCAL_STATES), 'belief_keys': list(BELIEF_KEYS)}


#: What a caller may pin so a payload is compared with the FROZEN originals
#: instead of only with the provider's own recomputed hashes (review finding 3).
PINNED_KEYS = ('order_sheet_sha256', 'public_map_sha256', 'map_file_sha256', 'map_id', 'scenario_id',
               'schematic_png_sha256')


def _pinned_hits(payload: Mapping, pinned: Mapping) -> list[str]:
    """Compare the payload with the pinned frozen order sheet and static map."""
    unknown = sorted(set(pinned) - set(PINNED_KEYS))
    if unknown:
        return [f'unknown pinned key(s): {unknown}']
    out = []
    sheet = payload.get('order_sheet') if isinstance(payload.get('order_sheet'), Mapping) else None
    static = payload.get('static_map') if isinstance(payload.get('static_map'), Mapping) else None
    public = (static or {}).get('public_map')
    if 'order_sheet_sha256' in pinned:
        got = digest(sheet) if sheet is not None else None
        if got != pinned['order_sheet_sha256']:
            out.append('order_sheet does not match the pinned frozen order sheet')
    if 'public_map_sha256' in pinned:
        if not isinstance(public, Mapping) or digest(public) != pinned['public_map_sha256']:
            out.append('static_map.public_map does not match the pinned map projection')
        if (static or {}).get('public_map_sha256') != pinned['public_map_sha256']:
            out.append('static_map.public_map_sha256 does not match the pinned map projection')
    for key in ('map_file_sha256', 'map_id'):
        if key in pinned and (static or {}).get(key) != pinned[key]:
            out.append(f'static_map.{key} does not match the pinned map bundle')
    if 'scenario_id' in pinned and (sheet or {}).get('scenario_id') != pinned['scenario_id']:
        out.append('order_sheet.scenario_id does not match the pinned scenario reference')
    # Second review, finding 1: the map FIGURE is part of the frozen map bundle.
    # ``None`` pins "this run has no schematic", so any schematic_ref is foreign.
    if 'schematic_png_sha256' in pinned:
        schematic = (static or {}).get('schematic_ref')
        got = schematic.get('png_sha256') if isinstance(schematic, Mapping) else None
        if got != pinned['schematic_png_sha256']:
            out.append('static_map.schematic_ref does not match the pinned map schematic')
    return out


def thaw_for_json(value: object) -> object:
    """Plain ``dict``/``list`` copy of any mapping/sequence tree.

    The validator must accept a DEEPLY FROZEN payload (review finding 4:
    ``harness.zone_study_prompts_ko`` hands out an immutable view), so the JSON
    check looks at the structure, not at the concrete container type. A value
    that is neither a mapping, a sequence nor a JSON scalar is left as-is and
    ``json.dumps`` still rejects it.
    """
    if isinstance(value, Mapping):
        return {k: thaw_for_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)) or (isinstance(value, Sequence)
                                            and not isinstance(value, (str, bytes))):
        return [thaw_for_json(item) for item in value]
    return value


def payload_violations(payload: object, *, seed: int | None = None,
                       pinned: Mapping | None = None) -> list[str]:
    """Every contract violation of a robot-facing per-call payload (empty list = clean).

    ``pinned`` optionally carries the digests of the FROZEN order sheet and map
    bundle (``PINNED_KEYS``). Without it the validator can only check that the
    provider's hashes are self-consistent, which a provider that recomputes both
    the data and the hash would pass (review finding 3).
    """
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
    try:                                  # everything a model receives must be plain JSON
        json.dumps(thaw_for_json(payload), allow_nan=False)
    except (TypeError, ValueError) as error:
        return out + [f'payload is not JSON serialisable: {error}']
    out.extend(forbidden_key_hits(payload))
    out.extend(non_ascii_keys(payload))
    out.extend(_value_hits(payload))
    out.extend(_shape_hits(payload, spec, seed))
    if pinned:
        out.extend(_pinned_hits(payload, pinned))
    return out


def validate_robot_payload(payload: object, *, seed: int | None = None,
                           pinned: Mapping | None = None) -> Mapping:
    """Raise ``ContractViolation`` unless the payload is inside the robot-facing boundary."""
    problems = payload_violations(payload, seed=seed, pinned=pinned)
    if problems:
        raise ContractViolation('robot-facing payload violates the study contract: ' + '; '.join(problems))
    return payload


# ---------------------------------------------------------------------------
# Log schema (written by the runner, read by package D and package I)

CALL_STATUS = ('ok', 'invalid_json', 'rejected_message', 'timeout', 'http_error', 'budget_exhausted',
               'policy_refusal', 'input_rejected', 'censored')
ACTION_KINDS = ('claim_order', 'goto', 'observe', 'grasp', 'place', 'release', 'wait', 'yield_passage',
                'abort_job', 'noop')
DELIVERY_STATUS = ('delivered', 'pending', 'rejected', 'dropped_budget')
# What every call record pins so a cohort can be re-identified (AGENTS.md,
# docs/execution_versioning.md). ``model``/``provider``/``model_settings_sha256``
# identify the model; the hashes identify the inputs and the contract.
PROVENANCE_KEYS = ('registry_sha256', 'order_sheet_sha256', 'map_file_sha256', 'public_map_sha256',
                   'code_sha', 'execution_bundle_id', 'model', 'provider', 'model_settings_sha256',
                   'prompt_template_sha256', 'cost_profile_id', 'input_profile_id')
DELIVERY_KEYS = ('recipient', 'delivered_at_sim_s', 'status')
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
    'status': f'str; one of {CALL_STATUS}; censored = still outstanding at the horizon',
    'action_id': 'str|null; the action this call submitted',
    'message_ids': 'list[str]; messages this call emitted',
    'decision_sources': 'list[str]; own refs, own command ids and message ids the answer cited',
    'payload_validated': 'bool; validate_robot_payload passed; False only with status=input_rejected',
    'provenance': f'object; keys of {PROVENANCE_KEYS}',
}
MESSAGE_FIELDS = {
    'schema': 'literal ugrp.zone_study_message_log.v1',
    'run_id': 'str', 'condition': 'str', 'seed': 'int',
    'message_id': 'str', 'sender': 'str', 'recipients': 'list[str]',
    'encoding': 'str; none|free_ko|schema',
    'reply_to': 'str|null; message_id',
    'created_at_sim_s': 'float', 'delivered_at_sim_s': 'float|null; earliest delivery',
    'deliveries': f'list of objects with {DELIVERY_KEYS}; one per recipient (per-spoke delays)',
    'delivery_delay_s': 'float; deterministic transport delay of the earliest delivery (package D)',
    'status': f'str; aggregate, one of {DELIVERY_STATUS}',
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
    if record.get('payload_validated') is not True and record.get('status') != 'input_rejected':
        out.append('payload_validated must be True unless status=input_rejected')
    if not isinstance(record.get('message_ids'), list) or not isinstance(record.get('decision_sources'), list):
        out.append('message_ids and decision_sources must be lists')
    out.extend(_closed(record.get('provenance'), PROVENANCE_KEYS, 'provenance',
                       required=('registry_sha256', 'order_sheet_sha256', 'map_file_sha256', 'code_sha',
                                 'model')))
    if not isinstance(record.get('cost_terms'), Mapping) or not isinstance(record.get('input_tokens'), Mapping):
        out.append('cost_terms and input_tokens must be objects')
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
    out.extend(_delivery_hits(record, created, delivered))
    if record.get('body_sha256') and isinstance(record.get('body'), (Mapping, str)):
        if digest(record['body']) != record['body_sha256']:
            out.append('body_sha256 does not match body')
    return out


def _delivery_hits(record: Mapping, created, delivered) -> list[str]:
    entries = record.get('deliveries')
    if not isinstance(entries, list):
        return ['deliveries must be a list, one entry per recipient']
    recipients = record.get('recipients') if isinstance(record.get('recipients'), list) else []
    out, seen, times = [], set(), []
    for entry in entries:
        out.extend(_closed(entry, DELIVERY_KEYS, 'deliveries[]', required=DELIVERY_KEYS))
        if not isinstance(entry, Mapping):
            continue
        recipient = entry.get('recipient')
        if recipient not in recipients:
            out.append(f'deliveries[] names {recipient!r}, which is not a recipient of the message')
        if recipient in seen:
            out.append(f'deliveries[] repeats {recipient!r}')
        seen.add(recipient)
        if entry.get('status') not in DELIVERY_STATUS:
            out.append(f'deliveries[].status must be one of {DELIVERY_STATUS}')
        at = entry.get('delivered_at_sim_s')
        if at is None:
            if entry.get('status') == 'delivered':
                out.append('a delivered recipient needs delivered_at_sim_s')
            continue
        if isinstance(at, bool) or not isinstance(at, _NUMBER):
            out.append('deliveries[].delivered_at_sim_s must be a number or null')
        elif isinstance(created, _NUMBER) and at < created:
            out.append('deliveries[].delivered_at_sim_s must not precede created_at_sim_s')
        else:
            times.append(at)
    if recipients and set(recipients) - seen:
        out.append(f'deliveries misses recipient(s): {sorted(set(recipients) - seen)}')
    if times:
        if delivered is None or abs(min(times) - delivered) > 1e-9:
            out.append('delivered_at_sim_s must be the earliest delivery time')
        delay = record.get('delivery_delay_s')
        if isinstance(created, _NUMBER) and isinstance(delay, _NUMBER) \
                and abs((created + delay) - min(times)) > 1e-9:
            out.append('delivery_delay_s must equal the earliest delivery minus created_at_sim_s')
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
