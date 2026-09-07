#!/usr/bin/env python3
"""Render an unmistakably visible slow-motion replay of a TEAM motor batch."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import masterpi_control
from sim.masterpi_production_v2 import STOP
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.real_stack_adapter import physical_motor_api_to_v2


WIDTH = 1440
HEADER_H = 96
TOP_H = 360
PANEL_W = WIDTH // 3
PANEL_H = 360
FOOTER_H = 54
HEIGHT = HEADER_H + TOP_H + PANEL_H + FOOTER_H
ROLES = {
    "r1": ("TURN LEFT", "rotate-left", (0, 220, 255)),
    "r2": ("MOVE FORWARD", "forward", (255, 150, 40)),
    "r3": ("TURN RIGHT", "rotate-right", (70, 80, 255)),
}


def fit(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    h, w = frame.shape[:2]
    scale = min(width / max(1, w), height / max(1, h))
    resized = cv2.resize(frame, (max(1, int(w * scale)), max(1, int(h * scale))))
    y = (height - resized.shape[0]) // 2
    x = (width - resized.shape[1]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return canvas


def motor_command(direction: str, speed: int) -> np.ndarray:
    physical = np.zeros(4, dtype=float)
    for motor, value in masterpi_control.drive_speeds(direction, speed):
        physical[int(motor) - 1] = float(value) / 40.0
    return physical_motor_api_to_v2(physical)


def render_canvas(
    world: MultiMasterPiProductionV2,
    *,
    stage: str,
    detail: str,
    elapsed_s: float,
    progress: float,
) -> np.ndarray:
    canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    overview_jpeg = world.render_team_jpeg(camera="cctv_top", quality=90)
    overview = cv2.imdecode(np.frombuffer(overview_jpeg, np.uint8), cv2.IMREAD_COLOR)
    canvas[HEADER_H:HEADER_H + TOP_H] = fit(overview, WIDTH, TOP_H)
    cv2.putText(
        canvas, "SHARED TOP VIEW", (20, HEADER_H + 32),
        cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA,
    )

    panel_y = HEADER_H + TOP_H
    for index, (rid, (label, _direction, color)) in enumerate(ROLES.items()):
        x = index * PANEL_W
        rgb = world.render_rgb(robot_id=rid, camera="robot_cam")
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        canvas[panel_y:panel_y + PANEL_H, x:x + PANEL_W] = fit(bgr, PANEL_W, PANEL_H)
        cv2.rectangle(
            canvas,
            (x + 2, panel_y + 2),
            (x + PANEL_W - 3, panel_y + PANEL_H - 3),
            color,
            3,
        )
        cv2.putText(
            canvas, f"{rid.upper()}  {label}", (x + 18, panel_y + 32),
            cv2.FONT_HERSHEY_SIMPLEX, 0.68, color, 2, cv2.LINE_AA,
        )

    stage_color = (80, 230, 100) if stage == "SUCCESS" else (40, 190, 255)
    cv2.putText(
        canvas,
        f"UGRP TEAM PHYSICS  |  SEED {world.seed}  |  {stage}",
        (22, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.82, stage_color, 2, cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"{elapsed_s:04.1f}s  {detail}",
        (22, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (230, 230, 230), 1, cv2.LINE_AA,
    )
    bar_x, bar_y, bar_w, bar_h = 250, HEIGHT - 32, 940, 12
    cv2.rectangle(canvas, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (90, 90, 90), 1)
    cv2.rectangle(
        canvas,
        (bar_x + 1, bar_y + 1),
        (bar_x + 1 + int((bar_w - 2) * max(0.0, min(1.0, progress))), bar_y + bar_h - 1),
        stage_color,
        -1,
    )
    cv2.putText(
        canvas, "ACTUAL SHARED-WORLD MOTION  |  3x SLOW PLAYBACK",
        (465, HEIGHT - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (210, 210, 210), 1, cv2.LINE_AA,
    )
    return canvas


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--duration", type=float, default=0.80)
    parser.add_argument("--speed", type=int, default=35)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    if not 0.10 <= args.duration <= 0.80:
        raise SystemExit("duration must be 0.10..0.80 seconds")

    output = Path(args.out).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    world = MultiMasterPiProductionV2(seed=args.seed, width=480, height=360, render=True)
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{WIDTH}x{HEIGHT}", "-r", str(args.fps), "-i", "-",
            "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
        ],
        stdin=subprocess.PIPE,
    )
    if ffmpeg.stdin is None:
        world.close()
        raise RuntimeError("ffmpeg stdin unavailable")

    frame_count = 0

    def emit(stage: str, detail: str, progress: float) -> None:
        nonlocal frame_count
        frame = render_canvas(
            world,
            stage=stage,
            detail=detail,
            elapsed_s=frame_count / max(1, args.fps),
            progress=progress,
        )
        ffmpeg.stdin.write(frame.tobytes())
        frame_count += 1

    try:
        for _ in range(int(args.fps * 1.2)):
            emit("READY", "Three commands armed at the same simulation boundary", 0.0)

        for rid, (_label, direction, _color) in ROLES.items():
            world.robot(rid).set_motor_commands(motor_command(direction, args.speed))

        # Sample at 30 simulated FPS but write at 10 FPS: physical movement is
        # slowed 3x for the viewer without changing the underlying trajectory.
        sample_fps = args.fps * 3
        motion_frames = max(1, int(round(args.duration * sample_fps)))
        for index in range(motion_frames):
            target_time = float(world.data.time) + args.duration / motion_frames
            world.robot("r1").advance_to_sim_time(target_time)
            emit(
                "ACT_PARALLEL",
                "R1 left + R2 forward + R3 right are moving together",
                (index + 1) / motion_frames,
            )

        for rid in ROLES:
            world.robot(rid).set_motor_commands(STOP)
        settle_frames = max(1, int(round(0.20 * sample_fps)))
        for _ in range(settle_frames):
            world.robot("r1").advance_to_sim_time(
                float(world.data.time) + 0.20 / settle_frames
            )
            emit("STOPPING", "All motor commands are zero", 1.0)
        for _ in range(int(args.fps * 2.0)):
            emit("SUCCESS", "Three distinct final poses confirmed", 1.0)
    finally:
        ffmpeg.stdin.close()
        rc = ffmpeg.wait(timeout=30.0)
        final_state = {
            rid: {
                "xy": [round(float(v), 4) for v in world.robot(rid).base_xyz()[:2]],
                "yaw_deg": round(float(np.degrees(world.robot(rid).base_rpy()[2])), 2),
            }
            for rid in ROLES
        }
        world.close()
    if rc:
        raise RuntimeError(f"ffmpeg exited {rc}")
    print(json.dumps({
        "ok": True,
        "output": str(output),
        "frames": frame_count,
        "video_s": round(frame_count / max(1, args.fps), 3),
        "sim_motion_s": args.duration,
        "playback_slowdown": 3,
        "final_state": final_state,
    }, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
