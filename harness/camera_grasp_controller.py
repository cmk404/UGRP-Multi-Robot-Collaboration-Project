"""Image-landmark feedback and explicit grasp transitions; no simulator imports."""
from __future__ import annotations

import copy
import math

from harness.camera_local_servo import LocalVisualServo


class CameraGraspController:
    """Own-issued command memory is not measured joint state or physical success."""

    def __init__(self):
        self.model = LocalVisualServo()
        self.issued_pulses = {}
        self.history = []
        self.stage = 'acquire'
        self.previous = None
        self.pending = None
        self.aligned_count = 0
        self.verify_count = 0
        self.retries = 0
        self.unseen_count = 0
        self.probe_index = 0
        self.events = []
        self.last_decision = {}

    @staticmethod
    def error(obs):
        jaws, target = obs.get('jaws'), obs.get('target')
        if jaws is None or target is None or obs.get('confidence', 0) < .7:
            return None
        if obs.get('view') == 'overhead' and obs.get('identity_confidence', 0) < .8:
            return None
        return math.hypot(*(target[k] - (jaws[0][k] + jaws[1][k]) / 2 for k in (0, 1)))

    def _bounded(self, action):
        action = dict(action)
        if action['kind'] in ('arm', 'look'):
            channel = 6 if action['kind'] == 'look' else action['servo_id']
            field = 'pan_pulse' if channel == 6 else 'pulse'
            if channel != 1:
                # Unknown initial physical pose: first command is bounded around
                # a documented neutral command, never read from an encoder.
                last = self.issued_pulses.get(channel, 1500)
                action[field] = max(500, min(2500, max(last - 100, min(last + 100, action[field]))))
        elif action['kind'] == 'drive':
            action['forward'] = max(-.05, min(.05, action['forward']))
            action['turn'] = max(-.1, min(.1, action['turn']))
            action['duration_s'] = max(0, min(.4, action['duration_s']))
        return action

    def _issue(self, action, obs, reason):
        action = self._bounded(action)
        pending = dict(action)
        if action['kind'] in ('arm', 'look'):
            channel = 6 if action['kind'] == 'look' else action['servo_id']
            value = action['pan_pulse'] if channel == 6 else action['pulse']
            if channel in self.issued_pulses:
                pending['_delta'] = value - self.issued_pulses[channel]
            self.issued_pulses[channel] = value
        self.pending = pending
        self.previous = copy.deepcopy(obs)
        self.history.append(copy.deepcopy(action))
        self.history = self.history[-8:]
        self.last_decision = {'stage': self.stage, 'reason': reason,
                              'visual_error': self.error(obs), 'retries': self.retries,
                              'learning': self.model.summary()}
        return action

    def step(self, obs, *, active=True):
        if self.previous is not None and self.pending is not None:
            self.model.observe(self.previous, self.pending, obs, isolated=True)
        self.pending = None
        if not active:
            return self._issue({'kind': 'wait'}, obs, 'scheduled peer turn; observe own previous action only')
        error = self.error(obs)
        suggestion = obs.get('suggested_action', {'kind': 'wait'})
        if self.stage == 'blocked':
            return self._issue({'kind': 'wait'}, obs, 'bounded recovery exhausted; no success claim')
        if self.stage == 'hold':
            if obs.get('capture_visible') and obs.get('lift_visible') and obs.get('confidence', 0) >= .7:
                return self._issue({'kind': 'wait'}, obs, 'visual hold evidence; physical success is evaluator-only')
            self.stage = 'recover'
        if self.stage == 'verify_close':
            self.verify_count += 1
            if obs.get('capture_visible') and obs.get('confidence', 0) >= .7:
                self.stage = 'test_lift'
            elif self.verify_count >= 2:
                self.stage = 'recover'
            else:
                return self._issue({'kind': 'wait'}, obs, 'await visible enclosure; command acknowledgement is insufficient')
        if self.stage == 'test_lift':
            if obs.get('capture_visible') and obs.get('lift_visible') and obs.get('confidence', 0) >= .7:
                self.stage = 'hold'
                return self._issue({'kind': 'wait'}, obs, 'visual lift observed; await independent physical evaluation')
            self.verify_count += 1
            if self.verify_count > 5:
                self.stage = 'recover'
            else:
                # Small test motion in documented shoulder-lift command direction.
                return self._issue({'kind': 'arm', 'servo_id': 5,
                                    'pulse': self.issued_pulses.get(5, 1500) - 50}, obs, 'small test lift; reobserve before continuing')
        if self.stage == 'recover':
            self.retries += 1
            self.aligned_count = self.verify_count = 0
            self.stage = 'blocked' if self.retries >= 3 else 'acquire'
            return self._issue({'kind': 'arm', 'servo_id': 1, 'pulse': 2000}, obs, 'open after visually unconfirmed capture')
        if self.issued_pulses.get(1) != 2000:
            return self._issue({'kind': 'arm', 'servo_id': 1, 'pulse': 2000}, obs, 'establish open command, not assumed aperture')
        if error is None:
            self.unseen_count += 1
            self.aligned_count = 0
            self.stage = 'acquire'
            if self.unseen_count >= 12:
                self.stage = 'blocked'
                return self._issue({'kind': 'wait'}, obs, 'jaws/target or identity not observable after bounded search')
            # A suggested gripper close cannot bypass the visual alignment gate.
            if suggestion['kind'] == 'arm' and suggestion['servo_id'] == 1:
                suggestion = {'kind': 'wait'}
            return self._issue(suggestion, obs, 'bounded visual search; landmarks unavailable or uncertain')
        self.unseen_count = 0
        self.stage = 'align'
        jaws, target = obs['jaws'], obs['target']
        axis = [jaws[1][k] - jaws[0][k] for k in (0, 1)]
        span2 = sum(x*x for x in axis)
        projection = sum((target[k] - jaws[0][k]) * axis[k] for k in (0, 1)) / max(span2, 1e-12)
        aligned = error <= min(.035, math.sqrt(span2) * .25) and span2 >= .01**2 and .1 <= projection <= .9
        self.aligned_count = self.aligned_count + 1 if aligned else 0
        if self.aligned_count >= 2:
            self.stage = 'verify_close'
            self.verify_count = 0
            return self._issue({'kind': 'arm', 'servo_id': 1, 'pulse': 1500}, obs, 'two visible midpoint alignment observations; test closure, not success')
        proposed = self.model.propose(obs, self.issued_pulses)
        if proposed is not None:
            return self._issue(proposed, obs, 'local current-image model predicts reduced alignment error')
        # Explicit bounded exploration supplies previously missing action data.
        channels = (3, 4, 5, 6)
        channel = channels[(self.probe_index // 4) % len(channels)]
        offset = (0, 50, -100, 50)[self.probe_index % 4]
        self.probe_index += 1
        target = self.issued_pulses.get(channel, 1500) + offset
        action = ({'kind': 'look', 'pan_pulse': target} if channel == 6 else
                  {'kind': 'arm', 'servo_id': channel, 'pulse': target})
        return self._issue(action, obs, 'isolated small joint probe for a state-local motion model')
