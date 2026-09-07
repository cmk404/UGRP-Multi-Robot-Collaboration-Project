"""Bounded delivery of agent-authored messages, with no access to world state."""
from copy import deepcopy


class CameraMessageBus:
    def __init__(self, robots, mode='none', ttl=30.0, limit=6):
        if mode not in ('none', 'status', 'natural'):
            raise ValueError('invalid communication mode')
        self.robots = tuple(robots)
        self.mode, self.ttl, self.limit = mode, float(ttl), int(limit)
        self.sent = []

    def publish(self, sender, payload, *, now, observed_at, decision_id):
        if self.mode == 'none' or payload is None:
            return None
        if sender not in self.robots or not decision_id:
            raise ValueError('invalid message provenance')
        if self.mode == 'natural':
            if not isinstance(payload, str) or not 1 <= len(payload.strip()) <= 400:
                raise ValueError('invalid natural message')
        else:
            if not isinstance(payload, dict) or set(payload) != {'observed', 'intent', 'request'}:
                raise ValueError('invalid status message')
            if not isinstance(payload['observed'], str) or not 1 <= len(payload['observed']) <= 240:
                raise ValueError('invalid observation text')
            if payload['intent'] not in ('approach','align','grasp','lift','carry','place','recover','wait','done','unknown'):
                raise ValueError('invalid intent')
            if payload['request'] not in ('none','help','clear_path','inspect','handoff'):
                raise ValueError('invalid request')
        row = {'message_id': f'msg-{len(self.sent)+1:06d}', 'sender': sender,
               'sent_at': float(now), 'observed_at': float(observed_at),
               'decision_id': decision_id, 'mode': self.mode, 'content': deepcopy(payload)}
        self.sent.append(row)
        return deepcopy(row)

    def inbox(self, recipient, now):
        if recipient not in self.robots:
            raise ValueError('invalid recipient')
        if self.mode == 'none':
            return []
        return [dict(deepcopy(row), age_s=float(now)-row['observed_at'])
                for row in self.sent if row['sender'] != recipient
                and 0 <= float(now)-row['observed_at'] <= self.ttl][-self.limit:]
