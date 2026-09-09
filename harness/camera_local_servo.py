"""Small state-conditioned visual servo learned only from issued commands.

The learner deliberately has no robot-state input.  It relates changes in image
landmarks to the private ``_delta`` attached to an issued command, and removes
that private field from every command it proposes for execution.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any


_CHANNELS = (3, 4, 5, 6)
_MAX_TRANSITIONS = 128
_MAX_VISUAL_CHANGE = 0.40
_MAX_NEIGHBOR_DISTANCE = 0.45


def _point(value: Any) -> tuple[float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    try:
        point = (float(value[0]), float(value[1]))
    except (TypeError, ValueError):
        return None
    if not all(0.0 <= item <= 1.0 for item in point):
        return None
    return point


def goal_features(observation: dict[str, Any]) -> tuple[float, float] | None:
    """Return target minus jaw midpoint when the landmarks are trustworthy."""
    try:
        if observation.get("view") not in {"own", "overhead"}:
            return None
        if float(observation.get("confidence", 0.0)) < 0.7:
            return None
        if observation.get("view") == "overhead" and float(
            observation.get("identity_confidence", 0.0)
        ) < 0.8:
            return None
        jaws = observation.get("jaws")
        if not isinstance(jaws, (list, tuple)) or len(jaws) != 2:
            return None
        left, right, target = _point(jaws[0]), _point(jaws[1]), _point(observation.get("target"))
        if left is None or right is None or target is None:
            return None
    except (TypeError, ValueError):
        return None
    midpoint = ((left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0)
    return (target[0] - midpoint[0], target[1] - midpoint[1])


def _visual_state(observation: dict[str, Any]) -> tuple[float, ...] | None:
    error = goal_features(observation)
    jaws = observation.get("jaws")
    target = _point(observation.get("target"))
    if error is None or not isinstance(jaws, (list, tuple)) or len(jaws) != 2 or target is None:
        return None
    left, right = _point(jaws[0]), _point(jaws[1])
    if left is None or right is None:
        return None
    dx, dy = right[0] - left[0], right[1] - left[1]
    separation = hypot(dx, dy)
    if separation <= 1e-6:
        return None
    midpoint = ((left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0)
    # Direction is unit length, so orientation and apparent jaw size remain
    # distinct conditioning variables.
    return (*midpoint, *target, dx / separation, dy / separation, separation)


def _channel(action: dict[str, Any]) -> int | None:
    if action.get("kind") == "arm" and action.get("servo_id") in (3, 4, 5):
        return int(action["servo_id"])
    if action.get("kind") == "look":
        return 6
    return None


@dataclass(frozen=True)
class _Transition:
    channel: int
    view: str
    state: tuple[float, ...]
    derivative: tuple[float, float]


class LocalVisualServo:
    """Bounded, local visual-Jacobian goal scorer."""

    def __init__(self, max_transitions: int = _MAX_TRANSITIONS) -> None:
        if not 1 <= max_transitions <= _MAX_TRANSITIONS:
            raise ValueError(f"max_transitions must be between 1 and {_MAX_TRANSITIONS}")
        self.max_transitions = max_transitions
        self.transitions: list[_Transition] = []

    def observe(
        self,
        before: dict[str, Any],
        action: dict[str, Any],
        after: dict[str, Any],
        isolated: bool = True,
    ) -> None:
        """Record one isolated image response to an issued pulse delta."""
        channel = _channel(action)
        before_state = _visual_state(before)
        after_state = _visual_state(after)
        before_goal, after_goal = goal_features(before), goal_features(after)
        delta = action.get("_delta")
        if (
            not isolated
            or channel is None
            or before.get("view") != after.get("view")
            or before_state is None
            or after_state is None
            or before_goal is None
            or after_goal is None
            or isinstance(delta, bool)
        ):
            return
        try:
            command_delta = float(delta)
        except (TypeError, ValueError):
            return
        change = (after_goal[0] - before_goal[0], after_goal[1] - before_goal[1])
        # Zero/tiny commands are numerically useless.  A very large landmark
        # jump is more likely a changed detection or unisolated scene event.
        if (
            abs(command_delta) < 1.0
            or hypot(*change) > _MAX_VISUAL_CHANGE
            or self._distance(before_state, after_state) > _MAX_NEIGHBOR_DISTANCE
        ):
            return
        derivative = (change[0] / command_delta, change[1] / command_delta)
        if hypot(*derivative) > 0.02:
            return
        self.transitions.append(_Transition(channel, str(before["view"]), before_state, derivative))
        del self.transitions[:-self.max_transitions]

    @staticmethod
    def _distance(a: tuple[float, ...], b: tuple[float, ...]) -> float:
        # Orientation occupies two coordinates and would otherwise dominate a
        # small positional neighborhood, so give it half weight.
        weights = (1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 1.0)
        return sum(w * (x - y) ** 2 for x, y, w in zip(a, b, weights)) ** 0.5

    def _estimate(
        self, channel: int, view: str, state: tuple[float, ...]
    ) -> tuple[float, float] | None:
        nearby = []
        for item in self.transitions:
            if item.channel != channel or item.view != view:
                continue
            distance = self._distance(state, item.state)
            if distance <= _MAX_NEIGHBOR_DISTANCE:
                nearby.append((distance, item.derivative))
        if len(nearby) < 2:
            return None
        weights = [1.0 / (distance + 0.03) for distance, _ in nearby]
        total = sum(weights)
        mean = tuple(
            sum(weight * derivative[axis] for weight, (_, derivative) in zip(weights, nearby)) / total
            for axis in (0, 1)
        )
        magnitude = hypot(*mean)
        residual = (
            sum(
                weight * ((derivative[0] - mean[0]) ** 2 + (derivative[1] - mean[1]) ** 2)
                for weight, (_, derivative) in zip(weights, nearby)
            )
            / total
        ) ** 0.5
        # Conflicting samples have no usable local Jacobian.  The small floor
        # still permits stable channels whose true image response is subtle.
        if magnitude < 1e-6 or residual > max(0.0005, magnitude):
            return None
        return mean

    def propose(
        self, observation: dict[str, Any], issued_pulses: dict[int, int]
    ) -> dict[str, Any] | None:
        state, goal = _visual_state(observation), goal_features(observation)
        if state is None or goal is None:
            return None
        current_error = goal[0] ** 2 + goal[1] ** 2
        if current_error <= 1e-10:
            return None
        view = str(observation["view"])
        best: tuple[float, int, int, int] | None = None
        for channel in _CHANNELS:
            derivative = self._estimate(channel, view, state)
            if derivative is None or channel not in issued_pulses:
                continue
            try:
                current_pulse = int(issued_pulses[channel])
            except (TypeError, ValueError):
                continue
            for requested_delta in (-50, 50):
                pulse = max(500, min(2500, current_pulse + requested_delta))
                actual_delta = pulse - current_pulse
                if actual_delta == 0:
                    continue
                predicted = (
                    goal[0] + derivative[0] * actual_delta,
                    goal[1] + derivative[1] * actual_delta,
                )
                score = predicted[0] ** 2 + predicted[1] ** 2
                candidate = (score, channel, pulse, actual_delta)
                if best is None or candidate < best:
                    best = candidate
        if best is None or best[0] >= current_error * 0.95:
            return None
        _, channel, pulse, _ = best
        if channel == 6:
            return {"kind": "look", "pan_pulse": pulse}
        return {"kind": "arm", "servo_id": channel, "pulse": pulse}

    def summary(self) -> dict[str, Any]:
        counts = {channel: 0 for channel in _CHANNELS}
        for item in self.transitions:
            counts[item.channel] += 1
        return {
            "sample_count": len(self.transitions),
            "max_transitions": self.max_transitions,
            "minimum_samples_per_channel": 2,
            "channel_sample_counts": counts,
            "sampled_channels": [channel for channel, count in counts.items() if count >= 1],
            "support_note": "A sampled channel still requires two nearby, same-view, consistent samples at proposal time.",
            "conditioning": "jaw midpoint, target, jaw direction, jaw separation",
        }
