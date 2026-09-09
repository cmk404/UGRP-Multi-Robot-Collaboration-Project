"""Bounded, local image Jacobian learned only from fresh primitive probes.

The class is deliberately independent of cameras and command execution. Its
caller owns observation freshness and issues returned commands one at a time.
"""
from __future__ import annotations

from collections import deque
from itertools import product
import math

import numpy as np


_PULSE_CHANNELS = {"wrist": 3, "elbow": 4, "shoulder": 5, "look": 6}
_CHANNEL_ORDER = ("forward", "turn", "look", "elbow", "wrist", "shoulder")
AXIS_DEADBAND_RAD = .16


def _axis_delta(after, before):
    return (after - before + math.pi / 2) % math.pi - math.pi / 2


def axis_residual(angle):
    """Signed angular excess outside the alignment-safe inner cone."""
    return math.copysign(max(0.0, abs(angle) - AXIS_DEADBAND_RAD), angle)


def _error(alignment):
    if not isinstance(alignment, dict):
        return None
    offset = alignment.get("offset_px")
    width = alignment.get("width_px")
    angle = alignment.get("axis_error_rad")
    if (not isinstance(offset, (list, tuple)) or len(offset) != 2 or
            not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in offset) or
            not isinstance(width, (int, float)) or isinstance(width, bool) or
            not isinstance(angle, (int, float)) or isinstance(angle, bool)):
        return None
    if not math.isfinite(width) or width <= 0 or not math.isfinite(angle):
        return None
    value = np.asarray((offset[0], offset[1],
                        max(8.0, 10.0 * width) * axis_residual(angle)), dtype=float)
    return value if np.all(np.isfinite(value)) else None


def _response(before, after):
    """Pixel response using the before-frame scale and an unoriented axis delta."""
    before_error, after_error = _error(before), _error(after)
    if before_error is None or after_error is None:
        return None
    before_angle = before["axis_error_rad"]
    angle_delta = _axis_delta(after["axis_error_rad"], before_angle)
    residual_delta = (axis_residual(before_angle + angle_delta)
                      - axis_residual(before_angle))
    value = np.asarray((
        after["offset_px"][0] - before["offset_px"][0],
        after["offset_px"][1] - before["offset_px"][1],
        max(8.0, 10.0 * before["width_px"]) * residual_delta,
    ), dtype=float)
    return value if np.all(np.isfinite(value)) else None


def _endpoint(alignment):
    value = alignment.get("endpoint") if isinstance(alignment, dict) else None
    if (not isinstance(value, (list, tuple)) or len(value) != 2 or
            not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                    for x in value)):
        return None
    return tuple(float(x) for x in value)


def _pulse_map(value):
    if not isinstance(value, dict):
        return None
    if any(isinstance(k, bool) or isinstance(v, bool) for k, v in value.items()):
        return None
    try:
        result = {int(k): int(v) for k, v in value.items()}
    except (TypeError, ValueError):
        return None
    if any(isinstance(v, bool) or not 500 <= v <= 2500 for v in result.values()):
        return None
    return result


class LocalPixelJacobian:
    """Remember up to 128 local primitive effects and propose a bounded DLS step."""

    def __init__(self, max_samples=128, *, min_consistent=2, state_radius=10.0,
                 pulse_radius=150, damping=1.0, min_improvement=0.15):
        if not 1 <= max_samples <= 128:
            raise ValueError("max_samples must be in 1..128")
        if min_consistent < 2:
            raise ValueError("min_consistent must be at least two")
        self.samples = deque(maxlen=max_samples)
        self.min_consistent = int(min_consistent)
        self.state_radius = float(state_radius)
        self.pulse_radius = int(pulse_radius)
        self.damping = float(damping)
        self.min_improvement = float(min_improvement)
        self.last_proposal = {"sample_count": 0, "usable_channels": [],
                              "command_count": 0, "reason": "not_proposed"}

    def diagnostics(self):
        """Return compact planner state suitable for a controller decision record."""
        result = dict(self.last_proposal)
        result["sample_count"] = len(self.samples)
        return result

    @staticmethod
    def _amount(action, before, after, primitive):
        if not isinstance(action, dict):
            return None
        if primitive in _PULSE_CHANNELS:
            channel = _PULSE_CHANNELS[primitive]
            if action.get("kind") not in ("arm", "look"):
                return None
            if primitive == "look":
                target = action.get("pan_pulse")
                valid_action = action.get("kind") == "look"
            else:
                target = action.get("pulse")
                valid_action = action.get("kind") == "arm" and action.get("servo_id") == channel
            delta = after.get(channel, before.get(channel, 1500)) - before.get(channel, 1500)
            if not valid_action or isinstance(target, bool) or not isinstance(target, int):
                return None
            if target != after.get(channel) or not 0 < abs(delta) <= 100:
                return None
            # No other known arm/look pulse may change during an isolated probe.
            if any(before.get(ch) != after.get(ch) for ch in _PULSE_CHANNELS.values() if ch != channel):
                return None
            return delta / 50.0
        if action.get("kind") != "drive":
            return None
        if before != after:
            return None
        f, t, duration = action.get("forward"), action.get("turn"), action.get("duration_s")
        if not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                   for x in (f, t, duration)) or not 0 < duration <= 1:
            return None
        if primitive == "forward" and t == 0 and -.05 <= f <= .15 and f != 0:
            return f * duration / .05
        if primitive == "turn" and f == 0 and -.1 <= t <= .1 and t != 0:
            return t * duration / .1
        return None

    def add_sample(self, before_alignment, after_alignment, action,
                   pulses_before, pulses_after, *, fresh, same_endpoint, primitive):
        """Accept one isolated, fresh before/action/after measurement."""
        if fresh is not True or same_endpoint is not True or primitive not in _CHANNEL_ORDER:
            return False
        before_error, after_error = _error(before_alignment), _error(after_alignment)
        response = _response(before_alignment, after_alignment)
        before_endpoint, after_endpoint = _endpoint(before_alignment), _endpoint(after_alignment)
        before_pulses, after_pulses = _pulse_map(pulses_before), _pulse_map(pulses_after)
        if (before_error is None or after_error is None or response is None or
                before_endpoint is None or after_endpoint is None or
                before_pulses is None or after_pulses is None):
            return False
        if math.dist(before_endpoint, after_endpoint) >= .04:
            return False
        amount = self._amount(action, before_pulses, after_pulses, primitive)
        if amount is None or not math.isfinite(amount) or abs(amount) < 1e-9:
            return False
        self.samples.append({
            "primitive": primitive,
            "amount": amount,
            "before_error": tuple(before_error),
            "after_error": tuple(after_error),
            "response": tuple(response),
            "axis_error_rad": float(before_alignment["axis_error_rad"]),
            "width_px": float(before_alignment["width_px"]),
            "endpoint": before_endpoint,
            "pulses": tuple(sorted(before_pulses.items())),
        })
        return True

    def _column(self, primitive, current_alignment, current_error, current_pulses):
        slopes = []
        current_endpoint = _endpoint(current_alignment)
        current_width = float(current_alignment["width_px"])
        for sample in self.samples:
            if sample["primitive"] != primitive:
                continue
            before = np.asarray(sample["before_error"])
            if (np.linalg.norm(before[:2] - current_error[:2]) > min(self.state_radius, 10.0) or
                    abs(_axis_delta(sample["axis_error_rad"], current_alignment["axis_error_rad"])) > .35):
                continue
            if current_endpoint is None or math.dist(sample["endpoint"], current_endpoint) >= .04:
                continue
            if not .7 <= sample["width_px"] / current_width <= 1.4:
                continue
            pose = dict(sample["pulses"])
            if any(abs(pose.get(ch, current_pulses.get(ch, 1500)) - current_pulses.get(ch, 1500)) > self.pulse_radius
                   for ch in _PULSE_CHANNELS.values()):
                continue
            response=np.asarray(sample["response"],dtype=float).copy()
            sample_scale=max(8.0,10.0*sample["width_px"])
            current_scale=max(8.0,10.0*current_width)
            response[2] *= current_scale/sample_scale
            slopes.append(response/sample["amount"])
        if len(slopes) < self.min_consistent:
            return None
        # Find the largest deterministic group agreeing in direction and scale.
        best = []
        for anchor in slopes:
            an = np.linalg.norm(anchor)
            group = []
            for slope in slopes:
                sn = np.linalg.norm(slope)
                if an < 1e-9 or sn < 1e-9:
                    agree = an < 1e-9 and sn < 1e-9
                else:
                    agree = np.dot(anchor, slope) / (an * sn) >= .7 and 1 / 3 <= sn / an <= 3
                if agree:
                    group.append(slope)
            if len(group) > len(best):
                best = group
        if len(best) < self.min_consistent:
            return None
        if len(best) * 2 <= len(slopes):
            return None
        array = np.stack(best)
        center = np.median(array, axis=0)
        residual = np.linalg.norm(array - center, axis=1)
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        keep = residual <= median + 3.0 * max(mad, .25)
        inliers = array[keep]
        if len(inliers) < self.min_consistent:
            return None
        return np.mean(inliers, axis=0)

    def _action(self, primitive, amount, pulses):
        if primitive in _PULSE_CHANNELS:
            channel = _PULSE_CHANNELS[primitive]
            old = pulses.get(channel, 1500)
            delta = int(round(max(-1.0, min(1.0, amount)) * 50))
            target = max(500, min(2500, old + delta))
            if target == old:
                return None
            return ({"kind": "look", "pan_pulse": target} if channel == 6 else
                    {"kind": "arm", "servo_id": channel, "pulse": target})
        if primitive == "forward":
            speed = max(-.05, min(.05, amount * .05))
            return None if abs(speed) < 1e-6 else {"kind": "drive", "forward": speed, "turn": 0.0, "duration_s": 1.0}
        speed = max(-.1, min(.1, amount * .1))
        return None if abs(speed) < 1e-6 else {"kind": "drive", "forward": 0.0, "turn": speed, "duration_s": 1.0}

    def propose(self, alignment, pulses):
        """Return zero to three bounded actions, or ``None`` without support."""
        error, pulse_state = _error(alignment), _pulse_map(pulses)
        if error is None or pulse_state is None:
            self.last_proposal = {"sample_count": len(self.samples), "usable_channels": [],
                                  "command_count": 0, "reason": "invalid_current_state"}
            return None
        names, columns = [], []
        for name in _CHANNEL_ORDER:
            column = self._column(name, alignment, error, pulse_state)
            if column is not None and np.all(np.isfinite(column)):
                names.append(name); columns.append(column)
        if not columns:
            self.last_proposal = {"sample_count": len(self.samples), "usable_channels": [],
                                  "command_count": 0, "reason": "insufficient_local_samples"}
            return None
        matrix = np.column_stack(columns)
        try:
            amounts = -matrix.T @ np.linalg.solve(
                matrix @ matrix.T + self.damping ** 2 * np.eye(3), error)
        except np.linalg.LinAlgError:
            self.last_proposal = {"sample_count": len(self.samples), "usable_channels": names,
                                  "command_count": 0, "reason": "dls_failed"}
            return None
        chosen = sorted(range(len(names)), key=lambda i: (-abs(amounts[i]), i))[:3]
        if not chosen:
            return None
        reduced = matrix[:, chosen]
        bounds = []
        for original_i in chosen:
            name = names[original_i]
            if name in _PULSE_CHANNELS:
                current = pulse_state.get(_PULSE_CHANNELS[name], 1500)
                bounds.append((max(-1.0, (500 - current) / 50.0),
                               min(1.0, (2500 - current) / 50.0)))
            else:
                bounds.append((-1.0, 1.0))
        candidates = []
        # For at most three variables, exhaustively choose whether each is at
        # its lower bound, free, or at its upper bound.  Solving the remaining
        # free variables finds the bounded least-squares optimum without scipy.
        for status in product((-1, 0, 1), repeat=len(chosen)):
            candidate = np.zeros(len(chosen), dtype=float)
            fixed = [i for i, value in enumerate(status) if value]
            free = [i for i, value in enumerate(status) if not value]
            for i in fixed:
                candidate[i] = bounds[i][status[i] > 0]
            if free:
                residual = error + reduced[:, fixed] @ candidate[fixed] if fixed else error
                free_matrix = reduced[:, free]
                try:
                    candidate[free] = np.linalg.solve(
                        free_matrix.T @ free_matrix
                        + self.damping ** 2 * np.eye(len(free)),
                        -free_matrix.T @ residual,
                    )
                except np.linalg.LinAlgError:
                    continue
                if any(candidate[i] < bounds[i][0] - 1e-9 or
                       candidate[i] > bounds[i][1] + 1e-9 for i in free):
                    continue

            actions, applied = [], []
            for local_i, original_i in enumerate(chosen):
                name = names[original_i]
                action = self._action(name, float(candidate[local_i]), pulse_state)
                if action is None:
                    continue
                after = dict(pulse_state)
                if name in _PULSE_CHANNELS:
                    after[_PULSE_CHANNELS[name]] = action.get("pan_pulse", action.get("pulse"))
                amount = self._amount(action, pulse_state, after, name)
                if amount is not None:
                    actions.append(action)
                    applied.append((original_i, amount))
            if not actions:
                continue
            prediction = error.copy()
            for original_i, amount in applied:
                prediction += matrix[:, original_i] * amount
            candidates.append((float(np.linalg.norm(prediction)), tuple(candidate), actions))

        if not candidates:
            self.last_proposal = {"sample_count": len(self.samples), "usable_channels": names,
                                  "command_count": 0, "reason": "bounded_step_is_zero"}
            return None
        predicted_norm, _, actions = min(candidates, key=lambda item: (item[0], item[1]))
        observed_norm = float(np.linalg.norm(error))
        if observed_norm - predicted_norm < self.min_improvement:
            self.last_proposal = {"sample_count": len(self.samples), "usable_channels": names,
                                  "command_count": 0, "reason": "no_predicted_improvement",
                                  "observed_error": observed_norm, "predicted_error": predicted_norm}
            return None
        self.last_proposal = {"sample_count": len(self.samples), "usable_channels": names,
                              "command_count": len(actions[:3]), "reason": "dls_proposal",
                              "observed_error": observed_norm, "predicted_error": predicted_norm}
        return actions[:3]
