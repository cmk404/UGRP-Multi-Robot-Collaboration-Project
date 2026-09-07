#!/usr/bin/env python3
"""Run one complete MasterPi grasp goal in one fresh Pi process.

The public low-level skills remain available for diagnostics and recovery, but a
normal grasp goal should not pay four local Python launches, four SSH command
boundaries, four Robot initializations, and repeated camera-stream setup. This
runner owns one Robot/actuator lease and one fresh MJPEG stream, then calls the
existing shared skill functions in order. It does not duplicate manipulation
policy.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from typing import Sequence

import approach as approach_mod
import pick as pick_mod
import search as search_mod
import track as track_mod
from deploy import DEFAULT_HOST, deploy_and_run
from robot import Robot


DEFAULT_SEARCH_SECONDS = 30.0
DEFAULT_TRACK_SECONDS = 3.0
STAGES = ("search", "track", "approach", "pick")


class SharedVideoCamera:
    """Expose LiveVideo through the snapshot-style camera callable API."""

    def __init__(self, video) -> None:
        self.video = video
        self.source = getattr(video, "source", "shared-mjpeg")

    def read(self, *, quiet: bool = False):
        del quiet
        return self.video.read()

    def close(self) -> None:
        # Ownership belongs to run_grasp_task(), not an individual stage.
        return None


def _event(
    event: str,
    *,
    target_color: str,
    stage: str | None = None,
    detail: str | None = None,
) -> None:
    payload = {"task_event": event, "target_color": target_color}
    if stage is not None:
        payload["stage"] = stage
    if detail:
        payload["detail"] = detail
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), flush=True)


def _run_stage(name: str, target_color: str, fn) -> None:
    _event("stage_start", stage=name, target_color=target_color)
    started = time.monotonic()
    try:
        code = fn()
    except Exception as exc:
        _event("stage_failed", stage=name, target_color=target_color, detail=str(exc))
        raise RuntimeError(f"task stage={name}: {exc}") from exc
    if code not in (None, 0):
        message = f"returned non-zero status {code}"
        _event("stage_failed", stage=name, target_color=target_color, detail=message)
        raise RuntimeError(f"task stage={name}: {message}")
    elapsed = time.monotonic() - started
    _event(
        "stage_complete",
        stage=name,
        target_color=target_color,
        detail=f"elapsed_s={elapsed:.3f}",
    )


def _target_detector(target_color: str):
    return lambda frame, **kwargs: search_mod.detect_target_blob(
        frame, target_color, **kwargs
    )


def run_grasp_task(
    robot: Robot,
    *,
    target_color: str = "red",
    search_seconds: float = DEFAULT_SEARCH_SECONDS,
    track_seconds: float = DEFAULT_TRACK_SECONDS,
    start_stage: str = "search",
    flip_x: bool = False,
    flip_y: bool = False,
    precision=None,
    video=None,
    clock=time,
) -> int:
    """Execute search -> track -> approach -> pick with shared process state."""
    target_color = str(target_color).strip().lower()
    if target_color not in {"red", "blue", "yellow"}:
        raise ValueError("target_color must be red, blue, or yellow")
    if search_seconds <= 0 or track_seconds <= 0:
        raise ValueError("search_seconds and track_seconds must be > 0")
    if start_stage not in STAGES:
        raise ValueError(f"start_stage must be one of {STAGES}, got {start_stage!r}")
    start_index = STAGES.index(start_stage)

    precision = precision or approach_mod.load_precision_controller()
    owns_video = video is None
    video = video or precision.LiveVideo(precision.STREAM_URL)
    shared_camera = SharedVideoCamera(video)
    detector = _target_detector(target_color)

    _event("task_start", target_color=target_color)
    try:
        if start_index <= STAGES.index("search"):
            _run_stage(
                "search",
                target_color,
                lambda: search_mod.run_search(
                    robot,
                    seconds=search_seconds,
                    detector=detector,
                    target_color=target_color,
                    camera=shared_camera,
                ),
            )
        if start_index <= STAGES.index("track"):
            _run_stage(
                "track",
                target_color,
                lambda: track_mod.run_track(
                    robot,
                    capture=shared_camera.read,
                    seconds=track_seconds,
                    flip_x=flip_x,
                    flip_y=flip_y,
                    clock=clock,
                    target_color=target_color,
                ),
            )
        if start_index <= STAGES.index("approach"):
            _run_stage(
                "approach",
                target_color,
                lambda: approach_mod.run_approach(
                    robot,
                    flip_x=flip_x,
                    flip_y=flip_y,
                    precision=precision,
                    target_color=target_color,
                    video=video,
                ),
            )
        _run_stage(
            "pick",
            target_color,
            lambda: pick_mod.run_precision_pick(
                robot,
                flip_x=flip_x,
                flip_y=flip_y,
                precision=precision,
                target_color=target_color,
                video=video,
            ),
        )
        _event("task_complete", target_color=target_color)
        return 0
    finally:
        try:
            robot.stop()
        finally:
            if owns_video:
                video.close()


def run_on_robot(args: argparse.Namespace) -> int:
    robot = Robot(dry_run=args.dry_run)

    def handle_signal(signum: int, _frame: object) -> None:
        try:
            pick_mod.invalidate_pick_plan()
            robot.stop()
        finally:
            raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    robot.stop()
    print(json.dumps({"probe": robot.probe()}, sort_keys=True), flush=True)
    return run_grasp_task(
        robot,
        target_color=args.target_color,
        search_seconds=args.search_seconds,
        track_seconds=args.track_seconds,
        start_stage=args.start_stage,
        flip_x=args.flip_x,
        flip_y=args.flip_y,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="search/track/approach/pick을 하나의 fresh MasterPi task process에서 실행합니다."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--target-color", choices=("red", "blue", "yellow"), default="red")
    parser.add_argument("--search-seconds", type=float, default=DEFAULT_SEARCH_SECONDS, help=argparse.SUPPRESS)
    parser.add_argument("--track-seconds", type=float, default=DEFAULT_TRACK_SECONDS, help=argparse.SUPPRESS)
    parser.add_argument("--start-stage", choices=STAGES, default="search", help=argparse.SUPPRESS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.search_seconds <= 0 or args.track_seconds <= 0:
        raise ValueError("search-seconds and track-seconds must be > 0")
    if args.on_robot or search_mod.is_robot():
        return run_on_robot(args)
    forwarded = [
        "--target-color", args.target_color,
        "--search-seconds", str(args.search_seconds),
        "--track-seconds", str(args.track_seconds),
        "--start-stage", args.start_stage,
    ]
    if args.dry_run:
        forwarded.append("--dry-run")
    if args.flip_x:
        forwarded.append("--flip-x")
    if args.flip_y:
        forwarded.append("--flip-y")
    return deploy_and_run("task_runner.py", forwarded, host=args.host)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
