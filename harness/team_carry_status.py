"""Executor-level team-carry STATUS channel (CANDIDATE, pending user approval, 2026-09-26).

Purpose: the pair own-camera study (cohort 31d16b0) showed that the approved
readiness barrier (``harness.pair_carry_sync.PairCarrySync``) cannot tell a
partner that is still aligning from a dead one: the robot that gripped first
gave up after its 20 s barrier limit while the partner was alive and aligning
(4/6 runs). This channel carries ONLY each robot's own executor state, so a
waiting robot can extend its wait while the partner is alive, and abort at once
when the partner aborts or goes silent.

Contract (fixed; identical in every study condition, including no_comm):
* It is NOT the LLM dialogue channel and never carries natural language. The
  dialogue channel (if a condition has one) is separate and unchanged.
* A message has exactly: ``robot_id``, ``task_id``, ``seq`` (strictly
  increasing per sender), ``state`` (one of ``STATES``), ``sent_at_s`` (the
  sender's own sim clock). No free text, no coordinates, no images, no ground
  truth, no perception numbers.
* States: ``aligning`` (approaching/aligning its handle), ``ready`` (own grasp
  closed and seen; waiting to lift), ``lift``, ``carry``, ``put_down`` (lowering,
  opening or backing off), ``abort`` (own executor failed/stopped; terminal).
* Heartbeat: a sender publishes on every state change and at least every
  ``HEARTBEAT_S``. A partner is ``alive`` while its latest message is at most
  ``STALE_S`` old.
* Wait rule for a robot blocked at a barrier (see ``wait_verdict``):
  - partner ``abort`` -> ABORT now;
  - partner silent (> ``STALE_S``) -> ABORT;
  - partner alive and ``aligning`` -> WAIT, up to ``MAX_PARTNER_ALIGN_WAIT_S``
    measured from this robot's own barrier entry;
  - otherwise the barrier's own limit applies (unchanged).
The channel is a flag in the pair study (``--status-channel on|off``) so both
can be measured; ``off`` is the v1 barrier-only behaviour.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

PROFILE = 'team_carry_status_v1_candidate'
STATES = ('aligning', 'ready', 'lift', 'carry', 'put_down', 'abort')
FIELDS = frozenset({'robot_id', 'task_id', 'seq', 'state', 'sent_at_s'})
HEARTBEAT_S = .4
STALE_S = 2.0
MAX_PARTNER_ALIGN_WAIT_S = 75.       # partner's own align limit (60 s) + margin


class StatusViolation(ValueError):
    """A message outside the fixed status contract."""


@dataclass(frozen=True)
class StatusMessage:
    robot_id: str
    task_id: str
    seq: int
    state: str
    sent_at_s: float

    @classmethod
    def parse(cls, raw: Mapping[str, Any]) -> 'StatusMessage':
        if not isinstance(raw, Mapping) or set(raw) != FIELDS:
            raise StatusViolation(f'status message fields must be exactly {sorted(FIELDS)}')
        if raw['state'] not in STATES:
            raise StatusViolation(f'unknown state {raw["state"]!r}')
        if not isinstance(raw['robot_id'], str) or not isinstance(raw['task_id'], str):
            raise StatusViolation('robot_id/task_id must be strings')
        if isinstance(raw['seq'], bool) or not isinstance(raw['seq'], int) or raw['seq'] < 1:
            raise StatusViolation('seq must be a positive int')
        t = raw['sent_at_s']
        if isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(float(t)):
            raise StatusViolation('sent_at_s must be finite')
        return cls(raw['robot_id'], raw['task_id'], raw['seq'], raw['state'], float(t))


class StatusChannel:
    """In-process bus: one latest message per participant plus a full audit log."""

    def __init__(self, task_id: str, participants: tuple[str, ...]):
        self.task_id = task_id
        self.participants = tuple(participants)
        self.latest: dict[str, StatusMessage] = {}
        self.log: list[dict[str, Any]] = []
        self.rejected: list[dict[str, Any]] = []

    def publish(self, raw: Mapping[str, Any], received_at_s: float) -> bool:
        try:
            msg = StatusMessage.parse(raw)
            if msg.task_id != self.task_id or msg.robot_id not in self.participants:
                raise StatusViolation('wrong task or sender')
            prev = self.latest.get(msg.robot_id)
            if prev is not None and (msg.seq <= prev.seq or prev.state == 'abort'):
                raise StatusViolation('non-increasing seq or message after abort')
        except StatusViolation as exc:
            self.rejected.append({'raw': dict(raw) if isinstance(raw, Mapping) else repr(raw),
                                  'reason': str(exc), 'received_at_s': received_at_s})
            return False
        self.latest[msg.robot_id] = msg
        self.log.append({**msg.__dict__, 'received_at_s': round(received_at_s, 4)})
        return True

    def partner_view(self, me: str, now: float) -> dict[str, Any]:
        others = [p for p in self.participants if p != me]
        out = {}
        for p in others:
            msg = self.latest.get(p)
            out[p] = {'state': None, 'age_s': None, 'alive': False} if msg is None else {
                'state': msg.state, 'age_s': round(now - msg.sent_at_s, 3), 'alive': now - msg.sent_at_s <= STALE_S}
        return out


class StatusPublisher:
    """A robot's sender: state changes + heartbeat, from its own executor state only."""

    def __init__(self, channel: StatusChannel, robot_id: str):
        self.channel, self.robot_id = channel, robot_id
        self.seq = 0
        self.state: str | None = None
        self.sent_at = -math.inf

    def tick(self, state: str, now: float) -> None:
        if self.state == 'abort':
            return
        if state != self.state or now - self.sent_at >= HEARTBEAT_S:
            self.seq += 1
            self.channel.publish({'robot_id': self.robot_id, 'task_id': self.channel.task_id, 'seq': self.seq,
                                  'state': state, 'sent_at_s': round(now, 4)}, now)
            self.state, self.sent_at = state, now


def wait_verdict(view: Mapping[str, Mapping[str, Any]], waited_s: float, barrier_limit_s: float) -> dict[str, str]:
    """Decision for a robot blocked at a barrier, from the partner status view only."""
    for partner, v in view.items():
        if v['state'] == 'abort':
            return {'verdict': 'ABORT', 'why': f'{partner}_abort'}
        if not v['alive'] and waited_s > STALE_S:
            return {'verdict': 'ABORT', 'why': f'{partner}_silent'}
    if any(v['state'] == 'aligning' and v['alive'] for v in view.values()):
        if waited_s <= MAX_PARTNER_ALIGN_WAIT_S:
            return {'verdict': 'WAIT', 'why': 'partner_aligning'}
        return {'verdict': 'ABORT', 'why': 'partner_align_wait_exceeded'}
    if waited_s > barrier_limit_s:
        return {'verdict': 'ABORT', 'why': 'barrier_limit'}
    return {'verdict': 'WAIT', 'why': 'barrier'}
