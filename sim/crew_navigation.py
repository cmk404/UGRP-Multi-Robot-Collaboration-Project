"""Small, non-blocking path followers for shared-world crew scheduling.

The controller only computes a wheel command.  It deliberately knows nothing
about MuJoCo, clocks, locks, or other robots; a caller may update several
instances and then advance one shared physics tick.
"""
from __future__ import annotations

import math
from typing import Sequence

import numpy as np

try:  # Keep this utility importable in lightweight planner/test environments.
    from sim.masterpi_dynamics_v2 import FORWARD_PATTERN, YAW_LEFT_PATTERN
except ModuleNotFoundError as exc:  # pragma: no cover - exercised without MuJoCo
    if exc.name != "mujoco":
        raise
    FORWARD_PATTERN = np.ones(4, dtype=float)
    YAW_LEFT_PATTERN = np.array([-1.0, 1.0, -1.0, 1.0], dtype=float)


def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class ForwardPathController:
    """Follow cardinal or arbitrary XY waypoints by turning, then driving forward.

    ``update`` is intentionally nonblocking and side-effect free outside this
    instance.  A caller can return its zero command while yielding a path and
    resume it later without losing the waypoint index.
    """

    def __init__(
        self,
        path: Sequence[Sequence[float]],
        final_yaw: float | None = None,
        tolerance_m: float = 0.018,
        max_speed: float = 0.55,
        intermediate_tolerance_m: float = 0.065,
        final_anchor_tolerance_m: float | None = None,
    ) -> None:
        points = tuple((float(p[0]), float(p[1])) for p in path)
        if not points:
            raise ValueError("path must contain at least one waypoint")
        if any(not math.isfinite(v) for point in points for v in point):
            raise ValueError("path coordinates must be finite")
        if tolerance_m <= 0.0 or not math.isfinite(tolerance_m):
            raise ValueError("tolerance_m must be positive and finite")
        if max_speed <= 0.0 or not math.isfinite(max_speed):
            raise ValueError("max_speed must be positive and finite")
        if intermediate_tolerance_m <= 0.0 or not math.isfinite(intermediate_tolerance_m):
            raise ValueError("intermediate_tolerance_m must be positive and finite")
        self.path = points
        self.final_yaw = None if final_yaw is None else float(final_yaw)
        if self.final_yaw is not None and not math.isfinite(self.final_yaw):
            raise ValueError("final_yaw must be finite")
        self.tolerance_m = float(tolerance_m)
        self.max_speed = float(max_speed)
        self.intermediate_tolerance_m = float(intermediate_tolerance_m)
        self.final_anchor_tolerance_m = (
            self.tolerance_m if final_anchor_tolerance_m is None
            else float(final_anchor_tolerance_m)
        )
        if self.final_anchor_tolerance_m <= 0.0 or not math.isfinite(self.final_anchor_tolerance_m):
            raise ValueError("final_anchor_tolerance_m must be positive and finite")
        self.waypoint_index = 0
        self._state = "ROTATE"
        self._done = False
        self._target: tuple[float, float] | None = self.path[0]
        self._final_anchor: tuple[float, float] | None = None
        self._final_previous_position: np.ndarray | None = None
        self._final_heading_aligned = False
        self._previous_position: np.ndarray | None = None
        self._brake_elapsed = 0.0
        self._brake_then_final = False

    @property
    def state(self) -> str:
        return self._state

    @property
    def done(self) -> bool:
        return self._done

    @property
    def target(self) -> tuple[float, float] | None:
        return self._target

    def _finish_or_final_yaw(self) -> None:
        if self.final_yaw is None:
            self._state = "DONE"
            self._done = True
        else:
            self._state = "BRAKE"
            self._target = None
            self._final_anchor = self.path[-1]
            self._final_previous_position = None
            self._brake_elapsed = 0.0
            self._brake_then_final = True

    def _waypoint_tolerance(self) -> float:
        return (
            self.tolerance_m
            if self.waypoint_index == len(self.path) - 1
            else self.intermediate_tolerance_m
        )

    def update(self, position_xy: Sequence[float], yaw: float, dt: float) -> np.ndarray:
        """Return this tick's normalized four-wheel command."""
        if len(position_xy) < 2:
            raise ValueError("position_xy must contain x and y")
        if dt <= 0.0 or not math.isfinite(float(dt)):
            raise ValueError("dt must be positive and finite")
        position = np.asarray(position_xy[:2], dtype=float)
        if not np.all(np.isfinite(position)) or not math.isfinite(float(yaw)):
            raise ValueError("position and yaw must be finite")
        if self._done:
            return np.zeros(4, dtype=float)
        if self._previous_position is None:
            velocity = np.zeros(2, dtype=float)
        else:
            velocity = (position - self._previous_position) / float(dt)
        self._previous_position = position.copy()

        if self._state == "BRAKE":
            self._brake_elapsed += float(dt)
            speed = float(np.linalg.norm(velocity))
            if ((self._brake_elapsed >= 0.20 and speed <= 0.015)
                    or self._brake_elapsed >= 0.60):
                if self._brake_then_final:
                    self._state = "FINAL_YAW"
                else:
                    self._state = "ROTATE"
            else:
                return np.zeros(4, dtype=float)

        # Consume all waypoints already inside the acceptance disk.  This
        # prevents a tiny segment from causing a rotate/drive/rotate flip.
        advanced = False
        while self.waypoint_index < len(self.path):
            target = np.asarray(self.path[self.waypoint_index], dtype=float)
            if float(np.linalg.norm(target - position)) > self._waypoint_tolerance():
                break
            self.waypoint_index += 1
            advanced = True
            if self.waypoint_index >= len(self.path):
                self._finish_or_final_yaw()
                if self._done:
                    return np.zeros(4, dtype=float)
                if self._state == "BRAKE":
                    return np.zeros(4, dtype=float)
        if self.waypoint_index < len(self.path):
            self._target = self.path[self.waypoint_index]
            if advanced:
                # Every new segment gets a fresh heading decision.  Without
                # this reset an L-turn (or a reverse segment) could inherit
                # FORWARD state and drive along the previous heading.
                self._state = "BRAKE"
                self._brake_elapsed = 0.0
                self._brake_then_final = False
                return np.zeros(4, dtype=float)

        if self._state == "FINAL_YAW":
            error = _wrap(self.final_yaw - float(yaw))  # type: ignore[operator]
            anchor = np.asarray(self._final_anchor, dtype=float)
            anchor_error = anchor - position
            anchor_distance = float(np.linalg.norm(anchor_error))
            if (abs(error) <= math.radians(1.0)
                    and anchor_distance <= self.final_anchor_tolerance_m):
                self._state = "DONE"
                self._done = True
                return np.zeros(4, dtype=float)
            # Finish the heading before correcting the final anchor. Mixing
            # translation into a large yaw correction makes the offset chassis
            # orbit away from the anchor and can consume the task timeout.
            turn = min(0.22, max(0.08, abs(error) * 0.9))
            if not self._final_heading_aligned and abs(error) > math.radians(1.0):
                if anchor_distance <= 0.10:
                    return YAW_LEFT_PATTERN * (turn if error > 0.0 else -turn)
            else:
                self._final_heading_aligned = True

            # Once aligned, compensate the measured world XY error in the
            # current local frame. This bounded final-pose adjustment may be
            # slightly bidirectional so an anchor directly behind the chassis
            # cannot deadlock; route segments remain forward-only.
            hold_vector = 1.2 * anchor_error - 1.0 * velocity
            hold_norm = float(np.linalg.norm(hold_vector))
            if hold_norm > 1e-9:
                hold_vector *= min(0.80, hold_norm) / hold_norm
            cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
            local_forward = cy * float(hold_vector[0]) + sy * float(hold_vector[1])
            local_left = -sy * float(hold_vector[0]) + cy * float(hold_vector[1])
            correction_scale = float(np.hypot(local_forward, local_left))
            if (anchor_distance <= self.final_anchor_tolerance_m
                    and float(np.linalg.norm(velocity)) < 0.005):
                correction_scale = 0.0
            elif anchor_distance > self.final_anchor_tolerance_m:
                # The reduced chassis has static friction: an asymptotically
                # small hold command can stop just outside the anchor disk.
                # Keep correcting physically without widening the tolerance.
                correction_scale = max(0.18, correction_scale)
            correction_norm = math.hypot(local_forward, local_left)
            if correction_norm > 1e-9:
                local_forward = local_forward / correction_norm * correction_scale
                local_left = local_left / correction_norm * correction_scale
            else:
                local_forward = local_left = 0.0
            command = (
                FORWARD_PATTERN * local_forward
                + np.array([-1.0, 1.0, 1.0, -1.0], dtype=float) * local_left
                + YAW_LEFT_PATTERN * (
                    (turn if error > 0.0 else -turn)
                    if abs(error) > math.radians(1.0) else 0.0
                )
            )
            peak = float(np.max(np.abs(command)))
            return command / max(1.0, peak)

        target = np.asarray(self.path[self.waypoint_index], dtype=float)
        delta = target - position
        distance = float(np.linalg.norm(delta))
        desired = math.atan2(float(delta[1]), float(delta[0]))
        heading_error = _wrap(desired - float(yaw))
        # Hysteresis avoids issuing tiny alternating rotate commands near the
        # forward boundary.  The position acceptance disk handles completion.
        if self._state == "FORWARD" and abs(heading_error) > math.radians(15.0):
            self._state = "ROTATE"
        if self._state == "ROTATE" and abs(heading_error) > math.radians(4.0):
            magnitude = min(self.max_speed, max(0.12, abs(heading_error) * 1.8))
            return YAW_LEFT_PATTERN * (magnitude if heading_error > 0.0 else -magnitude)

        self._state = "FORWARD"
        # The reduced chassis has measurable coast after a command is removed;
        # begin tapering well before the acceptance disk to avoid overshooting
        # a short waypoint and then chasing it with reverse-like corrections.
        taper_distance = max(0.30, self.tolerance_m * 4.0)
        speed = self.max_speed * min(1.0, distance / taper_distance)
        speed = max(0.08, speed) if distance > self._waypoint_tolerance() else 0.0
        # Forward projection stays positive; yaw correction is bounded and
        # cannot turn this into reverse motion.
        # Heading is already within the rotate hysteresis band here.  Keep a
        # small trim only; a large simultaneous yaw command carries through
        # the chassis inertia and can arc past a short waypoint.
        correction = max(-0.12, min(0.12, heading_error * 0.25))
        command = FORWARD_PATTERN * speed + YAW_LEFT_PATTERN * correction
        peak = float(np.max(np.abs(command)))
        return command / max(1.0, peak)


class DynamicPeerYield:
    """Robot-local collision forecast with deterministic right of way.

    The caller supplies actuator state for the other active controllers.  This
    keeps traffic handling below task assignment and perception: it neither
    changes a job nor exposes simulator peer poses to a planner/LLM.
    """

    def __init__(self, robot_id: str, started: float) -> None:
        self.robot_id = str(robot_id)
        self.priority = (float(started), self.robot_id)
        self.blocked_since: float | None = None
        self.last_replan = -math.inf

    @staticmethod
    def _forward_speed(command: Sequence[float]) -> float:
        return max(0.0, float(np.dot(np.asarray(command, dtype=float), FORWARD_PATTERN) / 4.0))

    def should_yield(
        self,
        position_xy: Sequence[float],
        yaw: float,
        command: Sequence[float],
        peers: Sequence[tuple[str, Sequence[float], float, Sequence[float], tuple[float, str]]],
    ) -> bool:
        position = np.asarray(position_xy[:2], dtype=float)
        speed = self._forward_speed(command)
        velocity = np.array((math.cos(yaw), math.sin(yaw))) * speed * 0.34
        for peer_id, peer_xy, peer_yaw, peer_command, peer_priority in peers:
            separation = np.asarray(peer_xy[:2], dtype=float) - position
            distance = float(np.linalg.norm(separation))
            peer_speed = self._forward_speed(peer_command)
            peer_velocity = np.array((math.cos(peer_yaw), math.sin(peer_yaw))) * peer_speed * 0.34
            relative = peer_velocity - velocity
            relative_norm2 = float(np.dot(relative, relative))
            horizon = 1.1
            closest_t = 0.0 if relative_norm2 < 1e-8 else max(
                0.0, min(horizon, -float(np.dot(separation, relative)) / relative_norm2)
            )
            predicted = float(np.linalg.norm(separation + closest_t * relative))
            # The hard-radius clause covers footprint motion while turning.
            # Outside it, require a peer to be ahead on a closing trajectory;
            # a parked robot beside the start or final anchor must not create
            # an artificial traffic deadlock.
            command_active = float(np.max(np.abs(np.asarray(command, dtype=float)))) > 0.01
            conflict = (distance < 0.28 and command_active) or (
                speed > 0.01
                and float(np.dot(separation, velocity)) > 0.0
                and predicted < 0.40
            )
            if conflict and self.priority > peer_priority:
                return True
        return False

    def note(self, yielding: bool, now: float) -> bool:
        """Record yielding and report when a bounded dynamic replan is due."""
        if not yielding:
            self.blocked_since = None
            return False
        if self.blocked_since is None:
            self.blocked_since = float(now)
        if now - self.blocked_since >= 0.45 and now - self.last_replan >= 0.9:
            self.last_replan = float(now)
            return True
        return False
