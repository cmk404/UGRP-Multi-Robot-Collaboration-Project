"""Read-only receipts for messages actually delivered between robot planners.

These files are for an operator. They are never supplied to a robot request,
camera renderer, action selector, or referee.
"""
from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import tempfile
import threading
from datetime import datetime, timezone
import unicodedata


SCHEMA = 'ugrp.peer-communication.v1'
LATEST_LIMIT = 8192


def _json(value):
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'))


def _terminal_json(value):
    # json.dumps already quotes ASCII controls. Escape remaining Unicode
    # format/control characters without hiding normal Korean text.
    return ''.join(char if unicodedata.category(char)[0] != 'C'
                   else f'\\u{ord(char):04x}' for char in _json(value))


class CommunicationObserver:
    """Append an audited receipt after each successful inbox delivery."""

    def __init__(self, directory: Path, *, run_id: str, mode: str):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.log_path = self.directory / 'conversation.jsonl'
        self.latest_path = self.directory / 'latest-dialogue.json'
        # A reused run directory must not silently mix two different sessions.
        self.log_path.open('x', encoding='utf-8').close()
        self.run_id = run_id
        self.mode = mode
        self.seq = 0
        self.fresh_count = 0
        self.recent = deque(maxlen=3)
        self.lock = threading.Lock()
        self._publish()

    def _publish(self):
        state = {'schema': SCHEMA, 'run_id': self.run_id, 'seq': self.seq,
                 'fresh_peer_messages': self.fresh_count,
                 'status': ('저장 계획 재생: 새 자연어 메시지 0개' if self.mode == 'fixture' and not self.fresh_count
                            else '실제 모델의 동료 전달 메시지 대기 중' if not self.fresh_count
                            else f'실제 전달 메시지 {self.fresh_count}건'),
                 'recent_messages': list(self.recent),
                 'full_log': 'team/conversation.jsonl'}
        data = (_json(state) + '\n').encode('utf-8')
        if len(data) > LATEST_LIMIT:
            raise ValueError('communication latest sidecar exceeded size limit')
        fd, name = tempfile.mkstemp(prefix='.latest-dialogue-', dir=self.directory)
        try:
            with os.fdopen(fd, 'wb') as file:
                file.write(data)
                file.flush()
                os.fsync(file.fileno())
            os.replace(name, self.latest_path)
        finally:
            if os.path.exists(name):
                os.unlink(name)

    def _append(self, kind, *, source, phase, sender, turn, sim_time_s,
                request_id, recipients=None, text=None, details=None):
        self.seq += 1
        event = {'schema': SCHEMA, 'seq': self.seq, 'run_id': self.run_id,
                 'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
                 'kind': kind, 'source': source, 'phase': phase, 'sender': sender,
                 'recipients': list(recipients or []), 'turn': turn,
                 'sim_time_s': sim_time_s, 'request_id': request_id}
        if text is not None:
            event['text'] = text
        if details is not None:
            event['details'] = details
        with self.log_path.open('a', encoding='utf-8') as file:
            file.write(_json(event) + '\n')
            file.flush()
            os.fsync(file.fileno())
        return event

    def record_reply(self, sender, reply, records, *, phase, turn, sim_time_s,
                     delivered_to=()):
        """Call only after validated reply.message has reached the listed inboxes."""
        if reply is None:
            return
        source = ('fixture' if (records and records[-1].get('model') == 'scripted-fixture-not-llm')
                  or (not records and self.mode == 'fixture')
                  else 'llm')
        request_id = reply.get('request_id')
        with self.lock:
            mode_changed = source == 'llm' and self.mode != 'llm'
            if source == 'llm':
                self.mode = 'llm'
            if reply.get('reason'):
                self._append('decision_explanation', source=source, phase=phase,
                             sender=sender, turn=turn, sim_time_s=sim_time_s,
                             request_id=request_id, text=reply['reason'])
            if phase == 'planning':
                self._append('proposal_vote', source=source, phase=phase,
                             sender=sender, turn=turn, sim_time_s=sim_time_s,
                             request_id=request_id,
                             details={key: reply.get(key) for key in
                                      ('proposal_id', 'plan_hash', 'accept')})
            recipients = tuple(delivered_to)
            displayed_message = False
            if reply.get('message') and recipients:
                kind = 'fixture_peer_message' if source == 'fixture' else 'peer_message'
                event = self._append(kind, source=source, phase=phase, sender=sender,
                                     turn=turn, sim_time_s=sim_time_s,
                                     request_id=request_id, recipients=recipients,
                                     text=reply['message'])
                if source == 'llm':
                    self.fresh_count += 1
                    displayed_message = True
                    self.recent.append({key: event[key] for key in
                                        ('seq', 'phase', 'sender', 'recipients', 'turn', 'source')}
                                       | {'text': event['text'][:320]})
                # JSON quoting escapes terminal control bytes while preserving
                # the unmodified UTF-8 text in the append-only event.
                print('peer_delivery ' + _terminal_json(event), flush=True)
            if mode_changed or displayed_message:
                self._publish()


def read_latest(path: Path, *, max_bytes=LATEST_LIMIT):
    """Reject partial/oversized sidecars; atomic replacement prevents partial reads."""
    try:
        with Path(path).open('rb') as file:
            data = file.read(max_bytes + 1)
    except FileNotFoundError:
        return None
    if len(data) > max_bytes:
        return None
    try:
        state = json.loads(data)
    except (ValueError, UnicodeDecodeError):
        return None
    return state if isinstance(state, dict) and state.get('schema') == SCHEMA else None
