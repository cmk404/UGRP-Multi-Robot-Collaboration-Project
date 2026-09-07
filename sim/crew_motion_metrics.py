"""Measured trajectory metrics for concurrent simulator crews.

The recorder intentionally consumes only timestamped poses.  It does not
consume action names, declared phases, or success flags, so a controller cannot
claim concurrent motion without the trajectories showing it.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Callable, IO, Mapping


_TAU = 2.0 * math.pi


def _angle_delta(old: float, new: float) -> float:
    return (new - old + math.pi) % _TAU - math.pi


@dataclass(frozen=True)
class _Pose:
    time: float
    position: tuple[float, float, float]
    yaw: float


@dataclass(frozen=True)
class _Segment:
    robot_id: str
    start: float
    end: float
    distance: float
    forward: float
    reverse: float
    lateral: float
    yaw_delta: float
    yaw_rate: float
    moving: bool


class CrewMotionRecorder:
    """Record and summarize measured per-robot trajectories.

    ``sample_sink`` may be a text stream or a callable accepting one JSON
    string.  Every call, including rejected timestamps, is emitted as JSONL so
    malformed timing and zero-time pose changes remain auditable.
    """

    def __init__(
        self,
        sample_sink: IO[str] | Callable[[str], Any] | None = None,
        *,
        movement_epsilon_m: float = 1e-4,
        movement_speed_threshold_mps: float = 0.02,
        rotation_rate_threshold_rad_s: float = 0.15,
    ) -> None:
        self.sample_sink = sample_sink
        self.movement_epsilon_m = float(movement_epsilon_m)
        self.movement_speed_threshold_mps = float(movement_speed_threshold_mps)
        self.rotation_rate_threshold_rad_s = float(rotation_rate_threshold_rad_s)
        self._last: dict[str, _Pose] = {}
        self._segments: dict[str, list[_Segment]] = {}
        self._first_time: float | None = None
        self._last_time: float | None = None
        self._invalid_samples: list[dict[str, Any]] = []
        self._zero_time_pose_jumps: list[dict[str, Any]] = []

    def _emit(self, item: dict[str, Any]) -> None:
        if self.sample_sink is None:
            return
        line = json.dumps(item, sort_keys=True, separators=(",", ":")) + "\n"
        if callable(self.sample_sink):
            self.sample_sink(line)
        else:
            self.sample_sink.write(line)

    @staticmethod
    def _parse_pose(value: Mapping[str, Any], timestamp: float) -> _Pose:
        raw_position = value.get("position")
        if not isinstance(raw_position, (list, tuple)) or len(raw_position) < 2:
            raise ValueError("position must contain at least x and y")
        xyz = tuple(float(raw_position[i]) if i < len(raw_position) else 0.0 for i in range(3))
        yaw = float(value.get("yaw", 0.0))
        if not all(math.isfinite(v) for v in (*xyz, yaw, timestamp)):
            raise ValueError("pose values must be finite")
        return _Pose(timestamp, xyz, yaw)

    def observe(self, simtime: float, poses: Mapping[str, Mapping[str, Any]]) -> None:
        """Observe all available robot poses at one simulator timestamp."""
        try:
            timestamp = float(simtime)
        except (TypeError, ValueError) as exc:
            raise ValueError("simtime must be finite") from exc
        if not math.isfinite(timestamp):
            raise ValueError("simtime must be finite")

        normalized: dict[str, Any] = {}
        for robot_id, raw in poses.items():
            rid = str(robot_id)
            if not isinstance(raw, Mapping):
                self._record_invalid(timestamp, rid, "pose must be a mapping", raw)
                continue
            try:
                pose = self._parse_pose(raw, timestamp)
            except (TypeError, ValueError) as exc:
                self._record_invalid(timestamp, rid, str(exc), raw)
                continue
            normalized_pose: dict[str, Any] = {"position": list(pose.position), "yaw": pose.yaw}
            for optional_key in ("role", "state"):
                if optional_key in raw:
                    normalized_pose[optional_key] = raw[optional_key]
            normalized[rid] = normalized_pose
            previous = self._last.get(rid)
            if previous is not None and timestamp <= previous.time:
                reason = "duplicate timestamp" if timestamp == previous.time else "timestamp moved backwards"
                displacement = math.dist(previous.position, pose.position)
                item = {"robot_id": rid, "time": timestamp, "reason": reason, "displacement_m": displacement}
                self._record_invalid(timestamp, rid, reason, raw, extra=item)
                if timestamp == previous.time and displacement > self.movement_epsilon_m:
                    self._zero_time_pose_jumps.append(item)
                continue

            if previous is not None:
                self._add_segment(rid, previous, pose)
            self._last[rid] = pose
            self._first_time = timestamp if self._first_time is None else min(self._first_time, timestamp)
            self._last_time = timestamp if self._last_time is None else max(self._last_time, timestamp)

        self._emit({"simtime": timestamp, "poses": normalized, "valid": True})

    def _record_invalid(
        self,
        timestamp: float,
        rid: str,
        reason: str,
        raw: Any,
        *,
        extra: dict[str, Any] | None = None,
    ) -> None:
        item: dict[str, Any] = {"robot_id": rid, "time": timestamp, "reason": reason}
        if extra:
            item.update(extra)
        self._invalid_samples.append(item)
        self._emit({"simtime": timestamp, "robot_id": rid, "pose": raw, "valid": False, "reason": reason})

    def _add_segment(self, rid: str, old: _Pose, new: _Pose) -> None:
        dx = new.position[0] - old.position[0]
        dy = new.position[1] - old.position[1]
        distance = math.hypot(dx, dy)
        dt = new.time - old.time
        yaw_delta = _angle_delta(old.yaw, new.yaw)
        forward_component = dx * math.cos(old.yaw) + dy * math.sin(old.yaw)
        lateral_component = -dx * math.sin(old.yaw) + dy * math.cos(old.yaw)
        moving = (
            distance > self.movement_epsilon_m
            and distance / dt >= self.movement_speed_threshold_mps
        )
        segment = _Segment(
            robot_id=rid,
            start=old.time,
            end=new.time,
            distance=distance,
            forward=max(0.0, forward_component),
            reverse=max(0.0, -forward_component),
            lateral=abs(lateral_component),
            yaw_delta=yaw_delta,
            yaw_rate=abs(yaw_delta) / dt,
            moving=moving,
        )
        self._segments.setdefault(rid, []).append(segment)

    def _segments_for(self, rid: str) -> list[_Segment]:
        return self._segments.get(rid, [])

    def summary(self) -> dict[str, Any]:
        robot_ids = sorted(set(self._last) | set(self._segments))
        robots: dict[str, dict[str, Any]] = {}
        for rid in robot_ids:
            segments = self._segments_for(rid)
            path = sum(s.distance for s in segments)
            moving_seconds = sum(s.end - s.start for s in segments if s.moving)
            rotating_seconds = sum(s.end - s.start for s in segments if s.yaw_rate >= self.rotation_rate_threshold_rad_s)
            signs = [1 if s.forward > self.movement_epsilon_m else -1 if s.reverse > self.movement_epsilon_m else 0 for s in segments if s.moving]
            reversals = sum(1 for a, b in zip(signs, signs[1:]) if a and b and a != b)
            nonrotating = [s for s in segments if s.yaw_rate < self.rotation_rate_threshold_rad_s]
            reverse_runs: list[tuple[float, float]] = []
            run_distance = run_seconds = 0.0
            for segment in segments:
                is_reverse = segment.moving and segment.yaw_rate < self.rotation_rate_threshold_rad_s and segment.reverse > self.movement_epsilon_m
                if is_reverse:
                    run_distance += segment.reverse
                    run_seconds += segment.end - segment.start
                elif run_distance:
                    reverse_runs.append((run_distance, run_seconds))
                    run_distance = run_seconds = 0.0
            if run_distance:
                reverse_runs.append((run_distance, run_seconds))
            robots[rid] = {
                "distance_m": path,
                "path_length_m": path,
                "moving_seconds": moving_seconds,
                "forward_travel_m": sum(s.forward for s in segments),
                "reverse_travel_m": sum(s.reverse for s in segments),
                "lateral_travel_m": sum(s.lateral for s in segments),
                "nonrotating_forward_travel_m": sum(s.forward for s in nonrotating),
                "nonrotating_reverse_travel_m": sum(s.reverse for s in nonrotating),
                "rotation_seconds": rotating_seconds,
                "yaw_rotation_rad": sum(abs(s.yaw_delta) for s in segments),
                "small_rotational_translation_m": sum(
                    s.distance for s in segments
                    if s.yaw_rate >= self.rotation_rate_threshold_rad_s and s.distance > self.movement_epsilon_m
                ),
                "max_continuous_nonrotating_reverse_distance_m": max((distance for distance, _ in reverse_runs), default=0.0),
                "max_continuous_nonrotating_reverse_seconds": max((seconds for _, seconds in reverse_runs), default=0.0),
                "reversal_segments": reversals,
                "sample_count": len(segments) + (1 if rid in self._last else 0),
                "moving_intervals": [[s.start, s.end] for s in segments if s.moving],
            }

        overlap_windows = self._overlap_windows(robot_ids)
        for rid in robot_ids:
            robots[rid]["longest_idle_while_peers_moving_seconds"] = self._longest_idle(rid, robot_ids, overlap_windows)
        concurrency = {
            "overlap_2way_seconds": sum(e - s for s, e, count in overlap_windows if count >= 2),
            "overlap_3way_seconds": sum(e - s for s, e, count in overlap_windows if count >= 3),
            "overlap_windows": [{"start": s, "end": e, "moving_count": count} for s, e, count in overlap_windows if count >= 2],
        }
        diagnostics = {
            "invalid_samples": list(self._invalid_samples),
            "zero_time_pose_jumps": list(self._zero_time_pose_jumps),
        }
        result: dict[str, Any] = {
            "robots": robots,
            "per_robot": robots,
            "concurrency": concurrency,
            "sim_time_span_seconds": 0.0 if self._first_time is None or self._last_time is None else self._last_time - self._first_time,
            "diagnostics": diagnostics,
        }
        return result

    def _overlap_windows(self, robot_ids: list[str]) -> list[tuple[float, float, int]]:
        intervals = {rid: [(s.start, s.end) for s in self._segments_for(rid) if s.moving] for rid in robot_ids}
        points = sorted({point for values in intervals.values() for interval in values for point in interval})
        windows: list[tuple[float, float, int]] = []
        for start, end in zip(points, points[1:]):
            if end <= start:
                continue
            count = sum(any(a <= start and end <= b for a, b in values) for values in intervals.values())
            if count >= 2:
                if windows and windows[-1][1] == start and windows[-1][2] == count:
                    windows[-1] = (windows[-1][0], end, count)
                else:
                    windows.append((start, end, count))
        return windows

    def _longest_idle(self, rid: str, robot_ids: list[str], _overlap_windows: list[tuple[float, float, int]]) -> float:
        own = [(s.start, s.end) for s in self._segments_for(rid) if s.moving]
        peer_intervals = [
            (s.start, s.end)
            for other in robot_ids
            if other != rid
            for s in self._segments_for(other)
            if s.moving
        ]
        if not peer_intervals:
            return 0.0
        peer = []
        for start, end in sorted(peer_intervals):
            if not peer or start > peer[-1][1]:
                peer.append((start, end))
            else:
                peer[-1] = (peer[-1][0], max(peer[-1][1], end))
        longest = 0.0
        # A peer interval is sufficient evidence that at least one other robot
        # moved; subtract this robot's active portions to measure its idle gap.
        for start, end in peer:
            cuts = [start, end]
            for a, b in own:
                if a < end and b > start:
                    cuts.extend((max(a, start), min(b, end)))
            cuts = sorted(set(cuts))
            longest = max(longest, max((b - a for a, b in zip(cuts, cuts[1:]) if not any(x <= a and b <= y for x, y in own)), default=0.0))
        return longest


MotionMetricsRecorder = CrewMotionRecorder

__all__ = ["CrewMotionRecorder", "MotionMetricsRecorder"]
