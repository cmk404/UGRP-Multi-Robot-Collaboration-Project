#!/usr/bin/env python3
"""Keep the red block in the camera center using only the arm.

Mac에서 실행하면 이 폴더를 로봇에 복사한 뒤 Pi에서 돌립니다. OpenCV는
로봇에만 있으면 됩니다. SSH 별칭이 없어도 됩니다:

    python3 scripts/track_red_block.py --host <로봇IP>
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from camera import capture_bgr, detect_red_blob, detect_target_blob
from deploy import DEFAULT_HOST, deploy_and_run
from poses import (
    BASE_CENTER,
    POSE_SEARCH,
    clamp_pulse,
)
from robot import Robot, is_robot


PAN_SERVO = 6
TILT_SERVO = 3
PAN_MIN = 1050
PAN_MAX = 1950
TILT_MIN = 500
TILT_MAX = 1200
PAN_START = BASE_CENTER
TILT_START = POSE_SEARCH[3]
DEADBAND = 0.05
MAX_STEP = 40
TRACK_MIN_AREA = 500
KP = 0.28
KI = 0.03
KD = 0.03
CENTER_CONFIRMATIONS = 2
MAX_CONSECUTIVE_MISSES = 4
DEFAULT_TRACK_SECONDS = 8.0


@dataclass
class PidState:
    integral: float = 0.0
    previous: float = 0.0


@dataclass
class Gaze:
    pan: int = PAN_START
    tilt: int = TILT_START
    pan_pid: PidState | None = None
    tilt_pid: PidState | None = None

    def __post_init__(self) -> None:
        if self.pan_pid is None:
            self.pan_pid = PidState()
        if self.tilt_pid is None:
            self.tilt_pid = PidState()


def pid_step(
    state: PidState,
    error: float,
    dt: float,
    *,
    kp: float = KP,
    ki: float = KI,
    kd: float = KD,
    limit: float = MAX_STEP,
) -> float:
    if dt <= 0:
        dt = 0.05
    state.integral = max(-300.0, min(300.0, state.integral + error * dt))
    derivative = (error - state.previous) / dt
    state.previous = error
    output = kp * error + ki * state.integral + kd * derivative
    return max(-limit, min(limit, output))


def centered(nx: float, ny: float, deadband: float = DEADBAND) -> bool:
    return abs(nx - 0.5) <= deadband and abs(ny - 0.5) <= deadband


def centered_at_gaze(nx: float, ny: float, gaze: Gaze, deadband: float = DEADBAND) -> bool:
    """Accept image centering or a safe servo-limit lock.

    Near a floor target the eye-in-hand camera can reach TILT_MIN while the
    block is still below image centre. That is not a tracking failure: the
    next approach stage can use the locked target and chassis geometry. The
    same rule applies at pan limits, where approach aligns the chassis under
    the gaze.
    """
    error_x = 0.5 - nx
    error_y = 0.5 - ny
    pan_saturated = (
        (gaze.pan >= PAN_MAX and error_x > 0.0)
        or (gaze.pan <= PAN_MIN and error_x < 0.0)
    )
    tilt_saturated = (
        (gaze.tilt >= TILT_MAX and error_y > 0.0)
        or (gaze.tilt <= TILT_MIN and error_y < 0.0)
    )
    return (abs(error_x) <= deadband or pan_saturated) and (
        abs(error_y) <= deadband or tilt_saturated
    )


def tracking_step(
    blob,
    gaze: Gaze,
    *,
    dt: float,
    flip_x: bool = False,
    flip_y: bool = False,
    deadband: float = DEADBAND,
) -> Gaze:
    if blob is None:
        return gaze

    nx = 1.0 - blob.nx if flip_x else blob.nx
    ny = 1.0 - blob.ny if flip_y else blob.ny
    error_x = 0.5 - nx
    error_y = 0.5 - ny
    if abs(error_x) < deadband:
        error_x = 0.0
        assert gaze.pan_pid is not None
        gaze.pan_pid.integral *= 0.8
    if abs(error_y) < deadband:
        error_y = 0.0
        assert gaze.tilt_pid is not None
        gaze.tilt_pid.integral *= 0.8

    pan_delta = pid_step(gaze.pan_pid, error_x * 640.0, dt)
    tilt_delta = pid_step(gaze.tilt_pid, error_y * 480.0, dt)
    gaze.pan = clamp_pulse(int(round(gaze.pan + pan_delta)), PAN_MIN, PAN_MAX)
    gaze.tilt = clamp_pulse(int(round(gaze.tilt + tilt_delta)), TILT_MIN, TILT_MAX)
    return gaze


def run_track(
    robot: Robot,
    *,
    capture=capture_bgr,
    seconds: float | None = None,
    flip_x: bool = False,
    flip_y: bool = False,
    clock=time,
    target_color: str = "red",
) -> int:
    """Run the physical PID tracker against an injected camera/clock.

    REAL uses capture_bgr + time. SIM injects MuJoCo RGB and virtual time, so
    controller constants, target-loss rules, PID state, and servo commands are
    literally shared rather than reimplemented.
    """
    detector = detect_red_blob if target_color == "red" else (lambda frame, **kwargs: detect_target_blob(frame, target_color, **kwargs))
    current_pan = robot.pose.get(PAN_SERVO, PAN_START)
    current_tilt = robot.pose.get(TILT_SERVO, TILT_START)
    try:
        initial = detector(capture(quiet=True), min_area=TRACK_MIN_AREA, crop_left=100)
    except RuntimeError:
        initial = None
    gaze = Gaze(pan=current_pan, tilt=current_tilt)
    if initial is None:
        print(
            f"red not visible at current gaze pan={current_pan} tilt={current_tilt}; "
            "holding gaze for fresh frames",
            flush=True,
        )
    else:
        print(
            f"red already visible -> keep gaze pan={current_pan} tilt={current_tilt} "
            f"nx={initial.nx:.3f} ny={initial.ny:.3f}",
            flush=True,
        )

    last = clock.monotonic()
    started = last
    misses = 0
    frames = 0
    announced = False
    centered_streak = 0

    while True:
        if seconds is not None and clock.monotonic() - started >= seconds:
            robot.stop()
            raise RuntimeError(
                f"track window expired after {seconds:.2f}s without "
                f"{CENTER_CONFIRMATIONS} centered camera confirmations"
            )
        frame = capture(quiet=announced)
        announced = True
        blob = detector(frame, min_area=TRACK_MIN_AREA, crop_left=100)
        now = clock.monotonic()
        dt = min(0.4, max(0.05, now - last))
        last = now
        frames += 1

        if blob is None:
            centered_streak = 0
            misses += 1
            if misses == 1:
                print(
                    f"red temporarily missing; hold pan={gaze.pan} tilt={gaze.tilt}",
                    flush=True,
                )
            if misses >= MAX_CONSECUTIVE_MISSES:
                robot.stop()
                raise RuntimeError(
                    f"track lost target for {misses} consecutive fresh frames; "
                    "gaze preserved for search recovery"
                )
            clock.sleep(0.05)
            continue

        misses = 0
        centered_streak = centered_streak + 1 if centered_at_gaze(
            1.0 - blob.nx if flip_x else blob.nx,
            1.0 - blob.ny if flip_y else blob.ny,
            gaze,
        ) else 0
        if centered_streak >= CENTER_CONFIRMATIONS:
            robot.stop()
            print(
                f"track verified centered nx={blob.nx:.3f} ny={blob.ny:.3f} "
                f"area={blob.area} confirmations={centered_streak}",
                flush=True,
            )
            return 0
        previous = (gaze.pan, gaze.tilt)
        gaze = tracking_step(blob, gaze, dt=dt, flip_x=flip_x, flip_y=flip_y)
        robot.nudge_servos({PAN_SERVO: gaze.pan, TILT_SERVO: gaze.tilt}, duration=0.1)
        if frames == 1 or frames % 8 == 0 or (gaze.pan, gaze.tilt) != previous:
            print(
                f"track nx={blob.nx:.3f} ny={blob.ny:.3f} "
                f"pan={gaze.pan} tilt={gaze.tilt} area={blob.area}",
                flush=True,
            )


def run_on_robot(args: argparse.Namespace) -> int:
    robot = Robot(dry_run=args.dry_run)

    def handle_signal(signum: int, _frame: object) -> None:
        try:
            robot.stop()
        except Exception:
            pass
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    robot.stop()
    print(json.dumps({"probe": robot.probe()}, sort_keys=True), flush=True)
    return run_track(
        robot,
        seconds=args.seconds,
        flip_x=args.flip_x,
        flip_y=args.flip_y,
        target_color=getattr(args, "target_color", "red"),
    )

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="빨간 블록을 화면 중앙에 유지합니다. 바퀴는 쓰지 않고 팔만 움직입니다."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--on-robot", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument(
        "--seconds",
        type=float,
        default=DEFAULT_TRACK_SECONDS,
        help="중앙 확인 제한 시간(기본 8초). 에이전트 호출은 더 짧은 3.0초를 사용합니다",
    )
    parser.add_argument("--target-color", choices=("red", "blue", "yellow"), default="red")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.on_robot or is_robot():
        return run_on_robot(args)
    extra: list[str] = []
    if args.dry_run:
        extra.append("--dry-run")
    if args.flip_x:
        extra.append("--flip-x")
    if args.flip_y:
        extra.append("--flip-y")
    if args.seconds <= 0:
        raise ValueError("seconds must be > 0")
    extra.extend(["--seconds", str(args.seconds), "--target-color", args.target_color])
    return deploy_and_run("track.py", extra, host=args.host, tty=False)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
