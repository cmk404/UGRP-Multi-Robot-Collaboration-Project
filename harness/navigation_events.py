"""Owned-RGB stop events and per-robot input accounting; never select steering."""
from __future__ import annotations

import base64
import math
from collections.abc import Mapping

import numpy as np

from harness.visual_drive_guard import validate_visual_drive
from harness.visual_progress import _small_gray


def _drive_family(action):
    if action.get('kind') != 'drive':
        return None
    forward = float(action.get('fwd', 0)) > 0
    turn = float(action.get('turn', 0))
    return (forward, 1 if turn > 0 else -1 if turn < 0 else 0)


class NavigationEvents:
    """Compare RGB over elapsed motor-command time, across short decisions.

    The live runner supplies cumulative, actually elapsed drive leases. This is
    command execution evidence, NOT odometry or proof of task progress. ``now``
    remains a compatibility clock for callers without an executor.
    """
    def __init__(self):
        self.reset()

    def reset(self, known_message_ids=()):
        """Full episode/observation reset; do not call for each short drive."""
        self._reset_progress()
        self.known_messages = set(known_message_ids)
        self._family = None

    def _reset_progress(self):
        self.anchor = None
        self.anchor_time = None

    def begin_command(self, action, known_message_ids=()):
        """Refresh message acknowledgments without erasing a continuing drive."""
        family = _drive_family(action)
        if family is None or family != self._family:
            self._reset_progress()
        self._family = family
        self.known_messages = set(known_message_ids)

    def inspect(self, nav, action, now, messages=(), *, executed_drive_s=None):
        guard = validate_visual_drive(nav, action)
        if not guard['allowed']:
            return guard
        new_messages = [m['message_id'] for m in messages
                        if m['message_id'] not in self.known_messages]
        if new_messages:
            return {'allowed': False, 'reason': 'NEW_PEER_MESSAGE',
                    'evidence': {'message_ids': new_messages}}
        clock = float(now if executed_drive_s is None else executed_drive_s)
        if not math.isfinite(clock) or clock < 0:
            raise ValueError('INVALID_DRIVE_PROGRESS_CLOCK')
        gray = _small_gray(base64.b64decode(nav['image']))
        # Flat/featureless imagery cannot establish whether the chassis moved.
        # Safety vetoes above still apply. This threshold is a development value.
        if float(np.std(gray)) < 2.0 or _drive_family(action) in (None, (False, 0)):
            self._reset_progress()
            return {**guard, 'progress': {'status': 'PROGRESS_UNOBSERVABLE'}}
        if self.anchor is None or clock < self.anchor_time:
            self.anchor, self.anchor_time = gray, clock
        elif clock - self.anchor_time >= 2.0 - 1e-9:
            window = clock - self.anchor_time
            change = float(np.mean(np.abs(gray.astype(float) - self.anchor.astype(float)))) / 255
            self.anchor, self.anchor_time = gray, clock
            if change <= .004:
                return {'allowed': False, 'reason': 'OWN_RGB_STAGNATION',
                        'evidence': {'nav_mean_absolute_change': change,
                                     'window_s': window, 'threshold': .004,
                                     'clock': 'drive_control' if executed_drive_s is not None else 'legacy_now'}}
        return guard


class InferenceInputBudget:
    """Conservative estimated preflight, never a claimed hard provider cap.

    All submitted requests reserve capacity. Missing usage consumes its reserved
    estimate and invalidates a verified-within-budget claim. Late/rejected replies
    count exactly once. The provider can exceed an estimate; actual overrun is
    retained and verification fails, rather than being hidden.
    """
    def __init__(self, limit, *, initial_request_estimate=6000):
        if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0:
            raise ValueError('input token budget must be a positive integer')
        if (isinstance(initial_request_estimate, bool)
                or not isinstance(initial_request_estimate, int) or initial_request_estimate <= 0):
            raise ValueError('request estimate must be a positive integer')
        self.limit = limit
        self.initial_request_estimate = min(limit, initial_request_estimate)
        self.tokens = 0
        self.estimated_unreported_tokens = 0
        self.calls_without_usage = 0
        self.seen = set()
        self.reservations = {}
        self._max_observed = 0

    @property
    def next_request_estimate(self):
        return max(self.initial_request_estimate, math.ceil(self._max_observed * 1.25))

    @property
    def accounted_tokens(self):
        return self.tokens + self.estimated_unreported_tokens + sum(self.reservations.values())

    @property
    def remaining(self):
        return max(0, self.limit - self.accounted_tokens)

    @property
    def can_reserve(self):
        return self.next_request_estimate <= self.remaining

    def reserve(self, call_id):
        if not isinstance(call_id, str) or not call_id:
            raise ValueError('INVALID_CALL_ID')
        if call_id in self.seen or call_id in self.reservations:
            raise ValueError('DUPLICATE_BUDGET_RESERVATION')
        if not self.can_reserve:
            return False
        self.reservations[call_id] = self.next_request_estimate
        return True

    def cancel_unstarted(self, call_id):
        """Only for a Future successfully cancelled before invocation."""
        self.reservations.pop(call_id, None)

    def record(self, call_id, usage):
        if call_id in self.seen:
            return
        estimate = self.reservations.pop(call_id, self.next_request_estimate)
        self.seen.add(call_id)
        tokens = usage.get('prompt_tokens') if isinstance(usage, Mapping) else None
        if isinstance(tokens, int) and not isinstance(tokens, bool) and tokens >= 0:
            self.tokens += tokens
            self._max_observed = max(self._max_observed, tokens)
        else:
            self.calls_without_usage += 1
            self.estimated_unreported_tokens += estimate

    @property
    def exhausted(self):
        return self.accounted_tokens >= self.limit

    @property
    def verified_within_limit(self):
        return (not self.reservations and self.calls_without_usage == 0
                and self.tokens <= self.limit)

    def snapshot(self, *, calls_used, max_calls, remaining_sim_seconds):
        return {'budget_mode': 'estimated_preflight',
                'calls_remaining': max(0, max_calls - calls_used),
                'input_tokens_remaining': self.remaining,
                'reported_prompt_tokens': self.tokens,
                'reserved_input_tokens': sum(self.reservations.values()),
                'estimated_unreported_tokens': self.estimated_unreported_tokens,
                'calls_without_usage': self.calls_without_usage,
                'next_request_estimate': self.next_request_estimate,
                'terminal_input_reserve': min(12000, self.limit),
                'remaining_sim_seconds': max(0.0, float(remaining_sim_seconds))}
