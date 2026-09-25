"""Communication protocol of the Korean zone dialogue study (package C).

Four main conditions differ ONLY in the communication channel; the reference
ceiling R is not a main condition:

``no_comm``      no high-level messages at all: 0 sent, 0 received.
``peer_ko``      decentralised free Korean peer dialogue over a mesh.
``leader_ko``    one robot doubles as leader, ROTATING r1/r2/r3 by seed.
                 Hub-and-spoke: leader<->follower edges only, never
                 follower<->follower. Followers may report and object.
``structured``   same acts, recipients and budget as ``peer_ko`` but a fixed
                 schema with no free text.
``reference_R``  an all-seeing commander with no-LLM robots. The commander's
                 decision is an ``action`` of kind ``order``; there is no
                 message channel, so every ``send`` is rejected.

Scope and boundaries this module keeps:

* A message is only ever relayed. ``Transport`` has no claim, reservation or
  job ledger, takes no such argument (``FORBIDDEN_TRANSPORT_PARAMS``) and the
  robot-facing inbox record is restricted to ``INBOX_FIELDS``. Nothing here can
  change a host claim, a reservation or another robot's action.
* Rejections and language drift are recorded, never repaired. A non-Korean
  utterance is flagged in the evaluation log and still delivered byte-identical;
  flags never enter a robot-facing record.
* Validators parse and check the model reply; they never fill in a decision.
  ``action`` (own job claim, or the commander's order) is separate from
  ``messages``.

Literal tokens (``r1``/``r2``/``r3``, zone letters ``A``/``B``/``C``, order and
item IDs, passage IDs, JSON keys, enum values) stay literal everywhere.

Threat model: an honest runner plus an UNTRUSTED model reply. Every check here
defends against a malformed, over-talkative or boundary-crossing reply and
against a caller mistake (a reused window id, an unvalidated input bundle, a
record edited after delivery). It does not defend against a hostile in-process
caller that subclasses these types, uses ``object.__setattr__`` on a frozen
value, or passes a mapping that lies in ``keys()``; that is out of scope and
would need a different design.

Package A (``harness/zone_study_contract.py``) owns the condition registry, the
message envelope, the structured-message schema and the input boundary. This
module derives ``SPECS`` from ``A.CONDITIONS``, delivers A envelopes
(``ugrp.zone_study_message.v1``) into the inbox and delegates the structured
schema to ``A.structured_violations``; it adds only what A leaves to the
protocol: dialogue windows, utterance budgets, rejection names, the model reply
schema and the evaluation-only language report.
"""
from __future__ import annotations

import copy
import inspect
import re
from dataclasses import dataclass, field

from harness.three_robot_plan import parse
from harness.zone_dialogue_metrics import english_words, hangul_ratio, id_issues
from harness.zone_study_contract import (CONDITIONS as CONTRACT_CONDITIONS, CONFIDENCE,
                                         ENVELOPE_KEYS, FREE_TEXT_FIELDS,
                                         MESSAGE_ENVELOPE_SCHEMA, ROBOTS as CONTRACT_ROBOTS,
                                         STRUCTURED_ACTS, STRUCTURED_FIELDS, STRUCTURED_STATES,
                                         Vocabulary, language_violations as contract_language_violations,
                                         structured_violations)

ROBOTS = CONTRACT_ROBOTS
ZONES = ('A', 'B', 'C')
CONDITIONS = tuple(CONTRACT_CONDITIONS)
COMMANDER = 'commander'
PROTOCOL_VERSION = 'ugrp.zone_study_protocol.v1'

# Utterance budget of one dialogue window (pilot 2026-09-25: 6 per window and
# 2 per robot were enough; the second round was silence or repetition).
MAX_WINDOW_UTTERANCES = 6
MAX_ROBOT_UTTERANCES = 2
# The prompt asks for <=240 characters; >600 is a schema violation, never truncated.
PROMPT_TEXT_CHARS = 240
MAX_TEXT_CHARS = 600
DELIVERY_DELAY_SIM_S = 0.1
# Korean compliance threshold of the evaluation-only flag (pilot V2: 0.992).
KOREAN_MIN_RATIO = 0.9

# Structured-message schema (condition 4), from package A. This module keeps its
# stricter rule that ALL fields are present, so a reply cannot omit a field.
STRUCT_FIELDS = STRUCTURED_FIELDS
STRUCT_ACTS = STRUCTURED_ACTS
STRUCT_STATES = STRUCTURED_STATES
# Free-text smuggling: package A's list plus two names this protocol also refuses.
FREE_TEXT_KEYS = tuple(dict.fromkeys(FREE_TEXT_FIELDS + ('detail', 'description')))

REPLY_FIELDS = ('request_id', 'action', 'decision_sources', 'messages')
ROBOT_ACTION_KINDS = ('claim', 'continue', 'release', 'wait')
COMMANDER_ACTION_KINDS = ('order',)
DECISION_SOURCES = ('static_map', 'order_sheet', 'own_rgb', 'own_commands', 'own_belief', 'message')

# Robot-facing fields of a delivered message: package A's closed message
# envelope. Evaluation data (language flags, the sender's reason, any action) can
# therefore never appear in an inbox record, and neither can the delivery time.
INBOX_FIELDS = ENVELOPE_KEYS
# No public callable of this module may take host decision state.
FORBIDDEN_TRANSPORT_PARAMS = ('claims', 'claim', 'reservation', 'reservations', 'board', 'ledger',
                              'jobs', 'actions', 'assignments')

REJECTIONS = ('channel_closed', 'no_follower_to_follower', 'unknown_sender', 'unknown_recipient',
              'self_recipient', 'no_recipients', 'free_text_not_allowed', 'structured_not_allowed',
              'empty_text', 'text_too_long', 'schema', 'window_cap', 'robot_cap', 'no_window',
              'unknown_reply_to', 'total_cap')

# A message_id is an identifier, never a place to hide prose: the structured
# condition would otherwise carry free text in ``reply_to``.
MESSAGE_ID_MAX = 64
_MESSAGE_ID = re.compile(r'[A-Za-z0-9_.:-]{1,%d}\Z' % MESSAGE_ID_MAX)


def is_message_id(value) -> bool:
    return isinstance(value, str) and bool(_MESSAGE_ID.fullmatch(value))


class ProtocolError(ValueError):
    """Malformed model reply or malformed protocol use. Never repaired."""


@dataclass(frozen=True)
class ConditionSpec:
    """One study condition. Only the channel fields differ between 1-4.

    ``topology`` and ``encoding`` carry package A's literals
    (``none|mesh|star|commander_downlink`` and ``none|free_ko|schema``); the
    utterance caps are this protocol's own budget, which A does not define.
    """
    name: str
    topology: str            # A: 'none' | 'mesh' | 'star' | 'commander_downlink'
    encoding: str            # A: 'none' | 'free_ko' | 'schema'
    rotating_leader: bool
    robot_llm: bool
    commander_llm: bool
    main_condition: bool
    max_window_utterances: int = MAX_WINDOW_UTTERANCES
    max_robot_utterances: int = MAX_ROBOT_UTTERANCES

    @property
    def channel_open(self) -> bool:
        return self.topology in ('mesh', 'star')


def _spec_from_contract(name, contract):
    """One ``ConditionSpec`` derived from package A's ``Condition`` row."""
    open_channel = contract.topology in ('mesh', 'star')
    return ConditionSpec(name=name, topology=contract.topology, encoding=contract.encoding,
                         rotating_leader=contract.leader_rotation, robot_llm=contract.robot_llm,
                         commander_llm=contract.topology == 'commander_downlink',
                         main_condition=contract.is_main,
                         max_window_utterances=MAX_WINDOW_UTTERANCES if open_channel else 0,
                         max_robot_utterances=MAX_ROBOT_UTTERANCES if open_channel else 0)


#: Derived from package A: a new condition or a changed topology/encoding there
#: reaches this protocol automatically instead of drifting.
SPECS = {name: _spec_from_contract(name, contract) for name, contract in CONTRACT_CONDITIONS.items()}


def spec(condition: str) -> ConditionSpec:
    try:
        return SPECS[condition]
    except KeyError:
        raise ProtocolError(f'unknown condition {condition!r}; use one of {CONDITIONS}') from None


def leader_for_seed(seed: int, robots=ROBOTS) -> str:
    """Rotate the leader r1/r2/r3 across seeds (user decision 2026-09-26).

    The leader is one of the robots and keeps carrying; there is no separate
    commander in condition 3.
    """
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ProtocolError('seed must be an int')
    return robots[seed % len(robots)]


def actors(condition: str, robots=ROBOTS) -> tuple[str, ...]:
    """Every LLM actor of the condition. Only R adds a commander."""
    return (COMMANDER,) if spec(condition).commander_llm else tuple(robots)


def leader_of(condition: str, *, seed=None, leader=None, robots=ROBOTS, allow_override=False):
    """Resolve the leader from the SEED rotation. None when the condition has none.

    2026-09-26 review finding 9: an explicit ``leader`` used to win over the
    seed, so a whole cohort could run with a fixed leader while the payload and
    the transport disagreed. A study run may now only pass the leader the seed
    rotation places; ``allow_override=True`` is for diagnostics that say so
    explicitly and must never be used for a reported condition.
    """
    if not spec(condition).rotating_leader:
        if leader is not None:
            raise ProtocolError(f'condition {condition!r} has no leader')
        return None
    if leader is not None:
        if leader not in robots:
            raise ProtocolError(f'unknown leader {leader!r}')
        if seed is None:
            if not allow_override:
                raise ProtocolError('leader_ko rotates the leader by seed: pass the seed, or set '
                                    'allow_override=True for an explicitly labelled diagnostic')
            return leader
        rotated = leader_for_seed(seed, robots)
        if leader != rotated and not allow_override:
            raise ProtocolError(f'seed {seed} rotates the leader to {rotated!r}, not {leader!r}; the '
                                'rotation is a user decision (2026-09-26) and cannot be overridden in a '
                                'study run')
        return leader
    if seed is None:
        raise ProtocolError('leader_ko needs a seed (leader rotation) or an explicit leader')
    return leader_for_seed(seed, robots)


def allowed_edges(condition: str, *, seed=None, leader=None, robots=ROBOTS,
                  allow_override=False) -> frozenset:
    """Directed (sender, recipient) pairs the channel allows."""
    s = spec(condition)
    if not s.channel_open:
        return frozenset()
    if s.topology == 'mesh':
        return frozenset((a, b) for a in robots for b in robots if a != b)
    lead = leader_of(condition, seed=seed, leader=leader, robots=robots, allow_override=allow_override)
    return frozenset([*((lead, b) for b in robots if b != lead),
                      *((b, lead) for b in robots if b != lead)])


def role_of(condition: str, rid: str, *, seed=None, leader=None, robots=ROBOTS,
            allow_override=False) -> str:
    """Prompt role of an actor: 'peer' | 'leader' | 'follower' | 'commander'."""
    if rid == COMMANDER:
        if not spec(condition).commander_llm:
            raise ProtocolError(f'condition {condition!r} has no commander')
        return 'commander'
    if rid not in robots:
        raise ProtocolError(f'unknown robot {rid!r}')
    lead = leader_of(condition, seed=seed, leader=leader, robots=robots, allow_override=allow_override)
    if lead is None:
        return 'peer'
    return 'leader' if rid == lead else 'follower'


# --- language drift (evaluation only) -------------------------------------

def language_report(text, *, literals=()) -> dict:
    """Flag Korean-compliance drift of one utterance. Never rewrites the text.

    ``flags`` may contain ``silence``, ``non_korean``, ``code_switch`` and
    ``literal_id_issue``. Evaluation data only: callers keep it out of every
    robot-facing record (see ``INBOX_FIELDS``).
    """
    literals = tuple(literals)
    ratio = hangul_ratio(text or '', literals)
    latin = english_words(text or '', literals)
    ids = id_issues(text or '', literals)
    flags = []
    if not (text or '').strip():
        flags.append('silence')
    elif ratio is None:
        flags.append('non_korean')          # literal tokens only, no Korean body
    else:
        if ratio < KOREAN_MIN_RATIO:
            flags.append('non_korean')
        if latin:
            flags.append('code_switch')
    if ids:
        flags.append('literal_id_issue')
    return {'hangul_ratio': ratio, 'korean': bool(ratio is not None and ratio >= KOREAN_MIN_RATIO),
            'latin_words': latin, 'id_issues': ids, 'flags': tuple(flags)}


# --- message transport ----------------------------------------------------

@dataclass(frozen=True)
class Envelope:
    message_id: str
    sender: str
    recipients: tuple
    sent_at_sim_s: float
    text: str | None = None
    structured: dict | None = None
    reply_to: str | None = None

    def body(self) -> dict:
        """The message body as package A sees it: ``{"text": ...}`` or the schema object."""
        return {'text': self.text} if self.text is not None else copy.deepcopy(self.structured)

    def record(self, *, delivered_at_sim_s=None) -> dict:
        """Robot-facing record: package A's closed message envelope
        (``ugrp.zone_study_message.v1``), with a copy of the body so a caller
        editing the record cannot change what was delivered.

        ``delivered_at_sim_s`` is accepted and deliberately NOT forwarded: A's
        envelope carries no delivery time, and the SIM delivery order belongs to
        the scheduler and the evaluation log, not to a robot input.
        """
        del delivered_at_sim_s
        body = {'text': self.text} if self.text is not None else copy.deepcopy(self.structured)
        return {'schema': MESSAGE_ENVELOPE_SCHEMA, 'message_id': self.message_id, 'sender': self.sender,
                'recipients': list(self.recipients),
                'encoding': 'free_ko' if self.text is not None else 'schema',
                'created_at_sim_s': float(self.sent_at_sim_s), 'reply_to': self.reply_to, 'body': body}


@dataclass(frozen=True)
class Receipt:
    accepted: bool
    rejection: str | None = None
    detail: str | None = None
    envelope: Envelope | None = None
    deliveries: tuple = ()
    language: dict | None = None
    #: The utterance was accepted and delivered but breaks the Korean rule
    #: (user decision 2026-09-26: flag and cost it, never block).
    language_violation: bool = False


@dataclass
class Window:
    window_id: str
    opened_at_sim_s: float
    utterances: int = 0
    per_robot: dict = field(default_factory=dict)


class Transport:
    """Relay-only message transport with channel isolation and window caps.

    It owns inboxes and an evaluation log, nothing else. There is no claim,
    reservation or job state here, and no method takes one, so a message can
    never directly change a host claim, a reservation or another robot's action.

    **One owner of the message bus** (2026-09-26 review finding 2). C used to
    deliver into its own inbox at ``at_sim_s + delivery_delay`` while D minted a
    second id and delivered into a second inbox, so the two clocks and the two id
    spaces could disagree and a message could be visible before its call was
    charged. With ``delivery_owner`` set, this transport validates, costs, logs
    and CREATES the canonical envelope, but only that owner may commit a
    delivery (``commit_delivery``). ``delivery_owner=None`` keeps the standalone
    behaviour used by the package-C unit tests and by callers that have no
    scheduler.
    """

    def __init__(self, condition, *, seed=None, leader=None, robots=ROBOTS,
                 item_ids=(), order_ids=(), roles=(), passages=(), location_refs=(),
                 vocabulary=None, delivery_owner=None, allow_leader_override=False,
                 delivery_delay_sim_s=DELIVERY_DELAY_SIM_S, max_window_utterances=None,
                 max_robot_utterances=None, max_total_utterances=None):
        self.spec = spec(condition)
        self.condition = self.spec.name
        self.robots = tuple(robots)
        self.leader = leader_of(self.condition, seed=seed, leader=leader, robots=self.robots,
                                allow_override=allow_leader_override)
        self.edges = allowed_edges(self.condition, seed=seed, leader=self.leader, robots=self.robots,
                                   allow_override=allow_leader_override)
        self.delivery_owner = delivery_owner
        self.delivery_delay_sim_s = float(delivery_delay_sim_s)
        self.cap_window = self.spec.max_window_utterances if max_window_utterances is None else max_window_utterances
        self.cap_robot = self.spec.max_robot_utterances if max_robot_utterances is None else max_robot_utterances
        # Optional episode budget. Re-opening windows must not be a way around
        # the per-window cap, so the runner can bound the whole run here.
        self.cap_total = max_total_utterances
        if vocabulary is not None:
            # Package A's Vocabulary (order sheet + public map) is the preferred
            # source: then a message can only name an ID the robot was given.
            self.vocab = {'item': frozenset(vocabulary.items), 'zone': frozenset(vocabulary.zones),
                          'role': frozenset(vocabulary.roles), 'passage': frozenset(vocabulary.passages),
                          'location_ref': frozenset(vocabulary.location_refs)}
        else:
            self.vocab = {'item': frozenset(item_ids) | frozenset(order_ids), 'zone': frozenset(ZONES),
                          'role': frozenset(roles), 'passage': frozenset(passages),
                          'location_ref': frozenset(location_refs)}
        self._inbox = {rid: [] for rid in self.robots}
        self.window = None
        self.windows = []      # every window id opened, in order
        self.log = []          # evaluation only: every attempt, accepted or not
        self.rejections = []   # evaluation only
        self.truncations = []  # evaluation only: inbox records a caller cut off
        self.language_flags = []   # evaluation only: delivered but flagged
        # single-owner bus bookkeeping (review finding 2)
        self.pending = {}      # accepted, waiting for the owner to commit delivery
        self.delivered = {}    # message_id -> envelope, once committed
        self._committed = set()

    def _received_ids(self, rid, at_sim_s=None) -> frozenset:
        """Message ids actually delivered to ``rid`` (by ``at_sim_s`` if given).

        A robot may only reply to a message it has received, so an id it could
        not know yet is not a valid reference either.
        """
        return frozenset(env.message_id for at, env in self._inbox.get(rid, ())
                         if at_sim_s is None or at <= float(at_sim_s))

    # -- windows ----------------------------------------------------------
    def open_window(self, window_id, *, at_sim_s) -> Window:
        """Open a new window. A window id is never reused, so the per-window
        cap cannot be reset by re-opening the same window.

        The id must be an identifier: it becomes part of every ``message_id``,
        which is a robot-facing field, so free text is refused here too.
        """
        if not is_message_id(window_id):
            raise ProtocolError(f'window id must be at most {MESSAGE_ID_MAX} id characters')
        if window_id in self.windows:
            raise ProtocolError(f'window {window_id!r} was already opened; use a new id')
        window = Window(window_id, float(at_sim_s))
        self.windows.append(window_id)
        self.window = window
        return window

    def close_window(self):
        self.window = None

    def remaining(self, sender=None) -> int:
        """Utterances still allowed in this window (0 when the channel is shut)."""
        if not self.spec.channel_open or self.window is None:
            return 0
        left = self.cap_window - self.window.utterances
        if self.cap_total is not None:
            left = min(left, self.cap_total - self.sent_count())
        if sender is not None:
            left = min(left, self.cap_robot - self.window.per_robot.get(sender, 0))
        return max(0, left)

    # -- sending ----------------------------------------------------------
    def send(self, sender, *, recipients, text=None, structured=None, reply_to=None, at_sim_s) -> Receipt:
        """Attempt one utterance. Rejections are recorded, never repaired.

        ``no_comm`` and ``reference_R`` reject every attempt (channel_closed),
        so those conditions send and receive exactly 0.
        """
        attempt = {'condition': self.condition, 'sender': sender,
                   'recipients': list(recipients) if isinstance(recipients, (list, tuple)) else recipients,
                   'at_sim_s': at_sim_s, 'window': self.window.window_id if self.window else None,
                   'encoding': self.spec.encoding}
        language = None
        if text is not None:
            language = language_report(text, literals=sorted(self.vocab['item'] | self.vocab['role']
                                                             | self.vocab['passage']
                                                             | self.vocab['location_ref']))
        try:
            envelope = self._build(sender, recipients, text, structured, reply_to, at_sim_s)
        except _Reject as exc:
            receipt = Receipt(False, exc.reason, exc.detail, language=language)
            self.log.append({**attempt, 'accepted': False, 'rejection': exc.reason,
                             'detail': exc.detail, 'language': language})
            self.rejections.append({**attempt, 'rejection': exc.reason, 'detail': exc.detail})
            return receipt
        delivered_at = float(at_sim_s) + self.delivery_delay_sim_s
        if self.delivery_owner is None:
            for rid in envelope.recipients:
                self._inbox[rid].append((delivered_at, envelope))
        else:
            # The SIM scheduler owns delivery: it calls ``commit_delivery`` at the
            # SIM time it charged, so nothing is readable before the call is paid.
            self.pending[envelope.message_id] = envelope
            delivered_at = None
        self.window.utterances += 1
        self.window.per_robot[sender] = self.window.per_robot.get(sender, 0) + 1
        slip = contract_language_violations(self.condition, envelope.body(),
                                            literals=self._literals()) if text is not None else []
        self.log.append({**attempt, 'accepted': True, 'message_id': envelope.message_id,
                         'delivered_at_sim_s': delivered_at, 'edges': len(envelope.recipients),
                         'broadcast': len(envelope.recipients) > 1, 'language': language,
                         'language_violation': bool(slip), 'language_reasons': slip,
                         'delivery_owner': self.delivery_owner})
        if slip:
            # Delivered, flagged, costed — never blocking (user decision
            # 2026-09-26, review finding 8).
            self.language_flags.append({'message_id': envelope.message_id, 'sender': sender,
                                        'reasons': slip})
        return Receipt(True, envelope=envelope,
                       deliveries=tuple((rid, delivered_at) for rid in envelope.recipients),
                       language=language, language_violation=bool(slip))

    def _literals(self):
        return sorted(self.vocab['item'] | self.vocab['role'] | self.vocab['passage']
                      | self.vocab['location_ref'])

    def commit_delivery(self, message_id, *, at_sim_s, owner, recipients=None) -> tuple:
        """Put one accepted envelope into its recipients' inboxes (single owner).

        Only the registered ``delivery_owner`` may call this, and only once per
        (message, recipient): the canonical id created by ``send`` stays the id
        the robot sees, the SIM scheduler decides WHEN, and nothing else can add,
        duplicate or reorder an inbox entry (review finding 2).
        """
        if self.delivery_owner is None:
            raise ProtocolError('this transport delivers its own messages; construct it with '
                                'delivery_owner=... to hand delivery to a SIM scheduler')
        if owner != self.delivery_owner:
            raise ProtocolError(f'{owner!r} does not own this message bus ({self.delivery_owner!r})')
        envelope = self.pending.get(message_id) or self.delivered.get(message_id)
        if envelope is None:
            raise ProtocolError(f'unknown message_id {message_id!r}: only an accepted envelope is delivered')
        targets = tuple(recipients) if recipients is not None else envelope.recipients
        out = []
        for rid in targets:
            if rid not in envelope.recipients:
                raise ProtocolError(f'{message_id} was not addressed to {rid!r}')
            if (message_id, rid) in self._committed:
                raise ProtocolError(f'{message_id} was already delivered to {rid!r}')
            self._committed.add((message_id, rid))
            self._inbox[rid].append((float(at_sim_s), envelope))
            out.append((rid, float(at_sim_s)))
        self.delivered[message_id] = envelope
        if set(self._committed) >= {(message_id, r) for r in envelope.recipients}:
            self.pending.pop(message_id, None)
        return tuple(out)

    def _build(self, sender, recipients, text, structured, reply_to, at_sim_s) -> Envelope:
        if not self.spec.channel_open:
            raise _Reject('channel_closed', f'{self.condition} has no message channel')
        if sender not in self.robots:
            raise _Reject('unknown_sender', str(sender))
        if self.window is None:
            raise _Reject('no_window', 'no dialogue window is open')
        if not isinstance(recipients, (list, tuple)) or not recipients:
            raise _Reject('no_recipients', 'recipients must be an explicit non-empty list')
        seen, targets = set(), []
        for rid in recipients:
            if rid == sender:
                raise _Reject('self_recipient', str(rid))
            if rid not in self.robots:
                raise _Reject('unknown_recipient', str(rid))
            if (sender, rid) not in self.edges:
                reason = 'no_follower_to_follower' if self.spec.topology == 'star' else 'unknown_recipient'
                raise _Reject(reason, f'{sender}->{rid}')
            if rid not in seen:
                seen.add(rid)
                targets.append(rid)
        body_text, body_struct = self._body(text, structured)
        known = self._received_ids(sender, at_sim_s)
        for candidate in (reply_to, (body_struct or {}).get('reply_to')):
            if candidate is not None and candidate not in known:
                raise _Reject('unknown_reply_to', str(candidate))
        if self.cap_total is not None and self.sent_count() >= self.cap_total:
            raise _Reject('total_cap', f'{self.cap_total} per run')
        if self.window.utterances >= self.cap_window:
            raise _Reject('window_cap', f'{self.cap_window} per window')
        if self.window.per_robot.get(sender, 0) >= self.cap_robot:
            raise _Reject('robot_cap', f'{self.cap_robot} per robot per window')
        mid = f'{self.window.window_id}-{sender}-{self.window.utterances + 1}'
        return Envelope(mid, sender, tuple(targets), float(at_sim_s), body_text, body_struct, reply_to)

    def _body(self, text, structured):
        if self.spec.encoding == 'free_ko':
            if structured is not None:
                raise _Reject('structured_not_allowed', 'free Korean channel carries text')
            if not isinstance(text, str) or not text.strip():
                raise _Reject('empty_text', 'an utterance needs Korean text')
            if len(text) > MAX_TEXT_CHARS:
                raise _Reject('text_too_long', f'{len(text)} > {MAX_TEXT_CHARS}')
            return text, None
        if text is not None:
            raise _Reject('free_text_not_allowed', 'structured channel carries no free text')
        try:
            return None, validate_structured(structured, vocab=self.vocab)
        except _Reject:
            raise
        except ProtocolError as exc:
            raise _Reject('schema', str(exc)) from None

    # -- receiving --------------------------------------------------------
    def inbox(self, rid, *, now_sim_s, last=None) -> tuple:
        """Messages actually delivered to ``rid`` by ``now_sim_s``.

        Every delivered message is returned by default: a silently dropped
        utterance would be an unrecorded information loss. ``last`` is an
        explicit caller choice, must be a positive count, and the drop is
        reported in ``truncations``.
        """
        if rid not in self._inbox:
            raise ProtocolError(f'unknown robot {rid!r}')
        if last is not None and (isinstance(last, bool) or not isinstance(last, int) or last < 1):
            raise ProtocolError('last must be a positive whole number of messages, or None')
        out = [env.record(delivered_at_sim_s=at) for at, env in self._inbox[rid] if at <= float(now_sim_s)]
        if last is not None and len(out) > last:
            self.truncations.append({'robot_id': rid, 'now_sim_s': float(now_sim_s),
                                     'delivered': len(out), 'kept': last})
            return tuple(out[-last:])
        return tuple(out)

    def sent_ids(self, rid, window_id=None) -> tuple:
        """message_ids this robot actually got accepted (optionally one window)."""
        return tuple(e['message_id'] for e in self.log if e['accepted'] and e['sender'] == rid
                     and (window_id is None or e['window'] == window_id))

    def window_context(self, rid, *, now_sim_s) -> dict | None:
        """The truthful dialogue-window block for a prompt.

        Caps and the remaining budget come from this transport, not from the
        condition defaults, so an overridden cap cannot make a prompt state a
        budget the robot does not have. ``None`` when the channel is shut.
        """
        if not self.spec.channel_open:
            return None
        if self.window is None:
            raise ProtocolError('no dialogue window is open')
        return {'window_id': self.window.window_id, 'max_utterances': self.cap_window,
                'max_your_utterances': self.cap_robot, 'your_utterances_left': self.remaining(rid),
                'received': list(self.inbox(rid, now_sim_s=now_sim_s)),
                'sent': list(self.sent_ids(rid, self.window.window_id))}

    def delivered_count(self, rid) -> int:
        return len(self._inbox[rid])

    def sent_count(self, rid=None) -> int:
        return sum(1 for e in self.log if e['accepted'] and (rid is None or e['sender'] == rid))


class _Reject(ProtocolError):
    """A named transport rejection. Subclasses ProtocolError so a direct
    validator call still raises ValueError."""

    def __init__(self, reason, detail=None):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        if reason not in REJECTIONS:                 # checked in -O too
            raise ProtocolError(f'unknown rejection reason {reason!r}')
        self.reason, self.detail = reason, detail


# --- validators -----------------------------------------------------------

def _contract_vocabulary(vocab) -> Vocabulary:
    """Package A ``Vocabulary`` from this module's internal per-field mapping."""
    if isinstance(vocab, Vocabulary):
        return vocab
    return Vocabulary(items=frozenset(vocab.get('item', ())), zones=frozenset(vocab.get('zone', ZONES)),
                      roles=frozenset(vocab.get('role', ())), passages=frozenset(vocab.get('passage', ())),
                      location_refs=frozenset(vocab.get('location_ref', ())))


def validate_structured(value, *, vocab) -> dict:
    """Condition-4 message: package A's schema, with every field required here.

    Field names, enums and the ID vocabulary are A's
    (``harness.zone_study_contract.structured_violations``). This protocol adds
    two rules A leaves open: the full field set must be present, and ``reply_to``
    must be a ``message_id`` of this transport's id shape.
    """
    if not isinstance(value, dict):
        raise ProtocolError('structured message must be an object')
    for key in value:
        if key in FREE_TEXT_KEYS:
            raise _Reject('free_text_not_allowed', f'field {key!r}')
    if set(value) != set(STRUCT_FIELDS):
        raise ProtocolError('structured message needs exactly ' + ', '.join(STRUCT_FIELDS))
    problems = structured_violations(value, vocabulary=_contract_vocabulary(vocab))
    if problems:
        raise ProtocolError('; '.join(problems))
    if value['reply_to'] is not None and not is_message_id(value['reply_to']):
        raise ProtocolError(f'reply_to must be a message_id (at most {MESSAGE_ID_MAX} id characters) or null')
    return copy.deepcopy(value)


def validate_action(value, *, condition, actor, order_ids=(), roles_by_order=None, robots=ROBOTS) -> dict:
    """``action`` is the actor's own job claim, or the commander's order.

    A robot claims only for itself. Only the R commander may issue orders, and
    an order is an action, not a message.
    """
    if not isinstance(value, dict) or 'kind' not in value:
        raise ProtocolError('action must be an object with kind')
    kind = value['kind']
    commander = actor == COMMANDER
    allowed = COMMANDER_ACTION_KINDS if commander else ROBOT_ACTION_KINDS
    if kind not in allowed:
        raise ProtocolError(f'action kind {kind!r} is not allowed for {actor}; use {allowed}')
    if kind == 'order':
        if set(value) != {'kind', 'assignments'}:
            raise ProtocolError('order action needs kind and assignments only')
        got = value['assignments']
        if not isinstance(got, dict) or set(got) != set(robots):
            raise ProtocolError(f'assignments need exactly {robots}')
        for rid, job in got.items():
            if job is not None:
                _check_job(job, order_ids=order_ids, roles_by_order=roles_by_order, where=rid)
        return copy.deepcopy(value)
    if kind == 'claim':
        if set(value) != {'kind', 'order_id', 'role', 'destination_zone'}:
            raise ProtocolError('claim needs kind, order_id, role and destination_zone only')
        _check_job(value, order_ids=order_ids, roles_by_order=roles_by_order, where=actor, claim=True)
        return copy.deepcopy(value)
    if kind == 'release':
        if set(value) != {'kind', 'order_id'}:
            raise ProtocolError('release needs kind and order_id only')
        if value['order_id'] not in tuple(order_ids):
            raise ProtocolError(f"unknown order_id {value['order_id']!r}")
        return copy.deepcopy(value)
    if set(value) != {'kind'}:
        raise ProtocolError(f'{kind} takes no other field')
    return copy.deepcopy(value)


def _check_job(job, *, order_ids, roles_by_order, where, claim=False):
    keys = {'kind', 'order_id', 'role', 'destination_zone'} if claim else {'order_id', 'role', 'destination_zone'}
    if not isinstance(job, dict) or set(job) != keys:
        raise ProtocolError(f'{where}: a job needs order_id, role and destination_zone')
    if job['order_id'] not in tuple(order_ids):
        raise ProtocolError(f"{where}: unknown order_id {job['order_id']!r}")
    if job['destination_zone'] not in ZONES:
        raise ProtocolError(f"{where}: destination_zone must be one of {ZONES}")
    if roles_by_order is not None:
        allowed = tuple(roles_by_order.get(job['order_id'], ()))
        if job['role'] not in allowed:
            raise ProtocolError(f"{where}: role {job['role']!r} is not in {allowed}")
    elif not isinstance(job['role'], str) or not job['role']:
        raise ProtocolError(f'{where}: role must be a non-empty string')


def validate_reply(raw, *, request_id, condition, actor, order_ids=(), item_ids=(), kinds=(),
                   roles_by_order=None, passages=(), location_refs=(), vocabulary=None,
                   robots=ROBOTS) -> dict:
    """Parse and check one model reply. Raises ProtocolError; never repairs.

    Shape for every condition and actor:
    ``{"request_id", "action", "decision_sources", "messages"}``. ``messages``
    is [] unless the condition's channel is open, so ``no_comm`` and
    ``reference_R`` cannot smuggle an utterance through the reply either.

    ``vocabulary`` is package A's :class:`~harness.zone_study_contract.Vocabulary`
    and is the preferred input: without it the structured validator only knew the
    order and item ids, so an item KIND that A allows (``item="red"`` of a
    fungible order) was rejected here and the structured condition could not
    express what the free-text conditions could (2026-09-26 review finding 7).
    """
    s = spec(condition)
    try:
        value = parse(raw) if isinstance(raw, str) else copy.deepcopy(raw)
    except Exception as exc:                        # malformed JSON
        raise ProtocolError(f'reply is not JSON: {exc}') from None
    if not isinstance(value, dict) or set(value) != set(REPLY_FIELDS):
        raise ProtocolError('reply needs exactly ' + ', '.join(REPLY_FIELDS))
    if value['request_id'] != request_id:
        raise ProtocolError('stale reply: request_id does not match')
    value['action'] = validate_action(value['action'], condition=condition, actor=actor,
                                     order_ids=order_ids, roles_by_order=roles_by_order, robots=robots)
    sources = value['decision_sources']
    if not isinstance(sources, list) or not sources or len(set(map(str, sources))) != len(sources):
        raise ProtocolError('decision_sources must be a non-empty list without duplicates')
    for src in sources:
        if src not in DECISION_SOURCES:
            raise ProtocolError(f'unknown decision source {src!r}; use {DECISION_SOURCES}')
        if src == 'message' and not s.channel_open:
            raise ProtocolError(f'{condition} receives no message, so it cannot be a decision source')
    messages = value['messages']
    if not isinstance(messages, list):
        raise ProtocolError('messages must be a list')
    if not s.channel_open and messages:
        raise ProtocolError(f'{condition} sends no message: messages must be []')
    if actor == COMMANDER and messages:
        raise ProtocolError('the reference commander orders through action, not messages')
    if vocabulary is not None:
        vocab = {'item': frozenset(vocabulary.items), 'zone': frozenset(vocabulary.zones),
                 'role': frozenset(vocabulary.roles), 'passage': frozenset(vocabulary.passages),
                 'location_ref': frozenset(vocabulary.location_refs)}
    else:
        vocab = {'item': frozenset(item_ids) | frozenset(order_ids) | frozenset(kinds),
                 'zone': frozenset(ZONES),
                 'role': frozenset(r for roles in (roles_by_order or {}).values() for r in roles),
                 'passage': frozenset(passages), 'location_ref': frozenset(location_refs)}
    value['messages'] = [_check_message(m, spec=s, robots=robots, vocab=vocab, actor=actor)
                         for m in messages]
    return value


def _check_message(value, *, spec, robots, vocab, actor):
    """Schema of one outgoing message.

    Deliberately NOT a topology check: which recipient a robot may reach is
    decided and recorded by ``Transport`` (``no_follower_to_follower``). A wrong
    recipient must cost the utterance, not void the whole reply and its action,
    so the mistake stays measurable.
    """
    keys = {'recipients', 'reply_to', 'text' if spec.encoding == 'free_ko' else 'message'}
    if not isinstance(value, dict) or set(value) != keys:
        raise ProtocolError('each message needs exactly ' + ', '.join(sorted(keys)))
    recipients = value['recipients']
    if not isinstance(recipients, list) or not recipients:
        raise ProtocolError('recipients must be an explicit non-empty list')
    for rid in recipients:
        if rid not in robots:
            raise ProtocolError(f'unknown recipient {rid!r}')
        if rid == actor:
            raise ProtocolError('a robot does not address itself')
    if value['reply_to'] is not None and not is_message_id(value['reply_to']):
        raise ProtocolError(f'reply_to must be a message_id (at most {MESSAGE_ID_MAX} id characters) or null')
    if spec.encoding == 'free_ko':
        text = value['text']
        if not isinstance(text, str) or not text.strip():
            raise ProtocolError('text must be a non-empty string')
        if len(text) > MAX_TEXT_CHARS:
            raise ProtocolError(f'text is longer than {MAX_TEXT_CHARS} characters')
        return copy.deepcopy(value)
    try:
        body = validate_structured(value['message'], vocab=vocab)
    except _Reject as exc:
        raise ProtocolError(f'{exc.reason}: {exc.detail}') from None
    return {'recipients': list(recipients), 'reply_to': value['reply_to'], 'message': body}


def relay(transport: Transport, actor, reply, *, at_sim_s) -> tuple:
    """Offer a validated reply's messages to the transport, in order.

    Returns the receipts. Rejected utterances stay rejected: the transport does
    not rewrite them, and nothing here touches the actor's action.
    """
    out = []
    for message in reply['messages']:
        out.append(transport.send(actor, recipients=message['recipients'], text=message.get('text'),
                                  structured=message.get('message'), reply_to=message['reply_to'],
                                  at_sim_s=at_sim_s))
    return tuple(out)


def transport_parameter_names() -> frozenset:
    """Parameter names of the whole transport surface (audit helper).

    Used by the tests to prove no transport entry point can be handed a host
    claim, reservation or job ledger, so a message cannot change one.
    """
    names = set()
    for obj in (Transport, Envelope, Receipt, Window):
        for _, member in inspect.getmembers(obj, inspect.isfunction):
            names.update(inspect.signature(member).parameters)
    for func in (relay, allowed_edges, leader_of, role_of, language_report):
        names.update(inspect.signature(func).parameters)
    return frozenset(names)


__all__ = ['PROTOCOL_VERSION', 'CONDITIONS', 'SPECS', 'ConditionSpec', 'ProtocolError', 'ROBOTS',
           'ZONES', 'COMMANDER', 'Envelope', 'Receipt', 'Transport', 'Window', 'spec', 'actors',
           'leader_for_seed', 'leader_of', 'allowed_edges', 'role_of', 'language_report',
           'validate_structured', 'validate_action', 'validate_reply', 'relay', 'STRUCT_FIELDS',
           'STRUCT_ACTS', 'STRUCT_STATES', 'CONFIDENCE', 'REPLY_FIELDS', 'DECISION_SOURCES',
           'ROBOT_ACTION_KINDS', 'COMMANDER_ACTION_KINDS', 'INBOX_FIELDS', 'REJECTIONS',
           'FORBIDDEN_TRANSPORT_PARAMS', 'MAX_WINDOW_UTTERANCES', 'MAX_ROBOT_UTTERANCES',
           'PROMPT_TEXT_CHARS', 'MAX_TEXT_CHARS', 'KOREAN_MIN_RATIO', 'DELIVERY_DELAY_SIM_S',
           'MESSAGE_ID_MAX', 'is_message_id', 'transport_parameter_names']
