"""Stop events from owned RGB and delivered messages; never select steering."""
import base64
import numpy as np

from harness.visual_drive_guard import validate_visual_drive
from harness.visual_progress import _small_gray


class NavigationEvents:
    def __init__(self):
        self.reset()

    def reset(self, known_message_ids=()):
        self.anchor = None
        self.anchor_time = None
        self.known_messages = set(known_message_ids)

    def inspect(self, nav, action, now, messages=()):
        guard = validate_visual_drive(nav, action)
        if not guard['allowed']:
            return guard
        # Every new received message warrants re-evaluation. The runtime does
        # not interpret a peer statement as a fact or choose a response.
        new_messages = [m['message_id'] for m in messages
                        if m['message_id'] not in self.known_messages]
        if new_messages:
            return {'allowed': False, 'reason': 'NEW_PEER_MESSAGE',
                    'evidence': {'message_ids': new_messages}}
        gray = _small_gray(base64.b64decode(nav['image']))
        if self.anchor is None:
            self.anchor, self.anchor_time = gray, float(now)
        elif float(now) - self.anchor_time >= 2.0:
            change = float(np.mean(np.abs(gray.astype(float) - self.anchor.astype(float)))) / 255
            self.anchor, self.anchor_time = gray, float(now)
            if change <= .004:
                return {'allowed': False, 'reason': 'OWN_RGB_STAGNATION',
                        'evidence': {'nav_mean_absolute_change': change,
                                     'window_s': 2.0, 'threshold': .004}}
        return guard


class InferenceInputBudget:
    """Count reported input usage once, including rejected model decisions.

    The call cap remains necessary when a provider omits token accounting.
    An in-flight response can exceed the remaining input budget by one call.
    """
    def __init__(self, limit):
        self.limit = int(limit)
        if self.limit <= 0:
            raise ValueError('input token budget must be positive')
        self.tokens = 0
        self.calls_without_usage = 0
        self.seen = set()

    def record(self, call_id, usage):
        if call_id in self.seen:
            return
        self.seen.add(call_id)
        tokens = usage.get('prompt_tokens') if isinstance(usage, dict) else None
        if isinstance(tokens, int) and not isinstance(tokens, bool) and tokens >= 0:
            self.tokens += tokens
        else:
            self.calls_without_usage += 1

    @property
    def exhausted(self):
        return self.tokens >= self.limit
