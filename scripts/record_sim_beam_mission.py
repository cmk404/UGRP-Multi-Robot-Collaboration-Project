#!/usr/bin/env python3
"""Record the physical Cooperative Beam Transport mission as a visible MP4."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.cooperative_payload import BEAM_GOAL, BEAM_START, beam_pose
from sim.multi_masterpi_production import MultiMasterPiProductionV2


WIDTH = 1440
HEADER_H = 96
TOP_H = 360
PANEL_W = WIDTH // 3
PANEL_H = 360
FOOTER_H = 54
HEIGHT = HEADER_H + TOP_H + PANEL_H + FOOTER_H
ROLE_LABELS = {
    "r1": ("CARRIER LEFT", (0, 220, 255)),
    "r2": ("SCOUT", (255, 150, 40)),
    "r3": ("CARRIER RIGHT", (70, 80, 255)),
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
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
    last_capture_sim = -1e9

    def phase_label() -> str:
        raw = str(
            (world.beam_transport_trace[-1].get("phase") if world.beam_transport_trace else "ready")
            or "ready"
        )
        return raw.replace("_", " ").upper()

    def emit(stage: str | None = None) -> None:
        nonlocal frame_count
        payload = beam_pose(world.data, world.model)
        x = float(payload["position"][0])
        z = float(payload["position"][2])
        progress = max(0.0, min(1.0, (x - BEAM_START[0]) / (BEAM_GOAL[0] - BEAM_START[0])))
        current = stage or phase_label()
        canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        overview_jpeg = world.render_team_jpeg(camera="cctv_top", quality=90)
        overview = cv2.imdecode(np.frombuffer(overview_jpeg, np.uint8), cv2.IMREAD_COLOR)
        canvas[HEADER_H:HEADER_H + TOP_H] = fit(overview, WIDTH, TOP_H)
        cv2.putText(
            canvas, "SHARED TOP VIEW  |  START A -> DESTINATION B", (20, HEADER_H + 32),
            cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2, cv2.LINE_AA,
        )

        panel_y = HEADER_H + TOP_H
        for index, (rid, (role, colour)) in enumerate(ROLE_LABELS.items()):
            rgb = world.render_rgb(robot_id=rid, camera="robot_cam")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            x0 = index * PANEL_W
            canvas[panel_y:panel_y + PANEL_H, x0:x0 + PANEL_W] = fit(
                bgr, PANEL_W, PANEL_H
            )
            cv2.rectangle(
                canvas,
                (x0 + 2, panel_y + 2),
                (x0 + PANEL_W - 3, panel_y + PANEL_H - 3),
                colour,
                3,
            )
            cv2.putText(
                canvas, f"{rid.upper()}  {role}", (x0 + 18, panel_y + 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.64, colour, 2, cv2.LINE_AA,
            )

        success = current == "SUCCESS"
        colour = (80, 230, 100) if success else (40, 190, 255)
        cv2.putText(
            canvas,
            f"UGRP COOPERATIVE BEAM  |  SEED {args.seed}  |  {current}",
            (22, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.80, colour, 2, cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"video {frame_count / max(1, args.fps):04.1f}s  beam x={x:.3f}m z={z:.3f}m",
            (22, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 1, cv2.LINE_AA,
        )
        bar_x, bar_y, bar_w, bar_h = 250, HEIGHT - 32, 940, 12
        cv2.rectangle(canvas, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (90, 90, 90), 1)
        cv2.rectangle(
            canvas,
            (bar_x + 1, bar_y + 1),
            (bar_x + 1 + int((bar_w - 2) * progress), bar_y + bar_h - 1),
            colour,
            -1,
        )
        cv2.putText(
            canvas,
            "TWO ENDPOINT CONTACTS + SCOUT + SHARED PHYSICS + DESTINATION REFEREE",
            (325, HEIGHT - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.50,
            (210, 210, 210), 1, cv2.LINE_AA,
        )
        ffmpeg.stdin.write(canvas.tobytes())
        frame_count += 1

    def capture() -> None:
        nonlocal last_capture_sim
        sim_time = float(world.data.time)
        payload = beam_pose(world.data, world.model)
        moving_payload = (
            float(payload["position"][2]) > BEAM_START[2] + 0.015
            or float(payload["position"][0]) > BEAM_START[0] + 0.03
        )
        interval = 0.05 if moving_payload else 2.0
        if sim_time - last_capture_sim + 1e-9 < interval:
            return
        last_capture_sim = sim_time
        emit()

    try:
        for _ in range(args.fps):
            emit("READY")
        world.frame_callback = capture
        results = world.act_parallel([
            {
                "robot_id": "r1", "action": "team_beam_transport",
                "mission_id": "beam_transport_v1", "role": "carrier_left",
            },
            {
                "robot_id": "r2", "action": "team_beam_transport",
                "mission_id": "beam_transport_v1", "role": "scout",
            },
            {
                "robot_id": "r3", "action": "team_beam_transport",
                "mission_id": "beam_transport_v1", "role": "carrier_right",
            },
        ])
        world.frame_callback = None
        if not all(result.ok for result in results.values()):
            raise RuntimeError(str({rid: result.reason for rid, result in results.items()}))
        for _ in range(args.fps * 2):
            emit("SUCCESS")
        final = world.beam_state()
        report = {
            "ok": True,
            "output": str(output),
            "frames": frame_count,
            "video_s": round(frame_count / max(1, args.fps), 3),
            "worker_wall_s": world.last_parallel_timing.get("wall_s"),
            "position": final["position"],
            "evaluation": final["evaluation"],
            "trace": final["trace"],
        }
    finally:
        world.frame_callback = None
        ffmpeg.stdin.close()
        rc = ffmpeg.wait(timeout=30.0)
        world.close()
    if rc:
        raise RuntimeError(f"ffmpeg exited {rc}")
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
