#!/usr/bin/env python3
"""Record the multi-zone all-cargo warehouse mission with visible motion."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.multi_masterpi_production import MultiMasterPiProductionV2


WIDTH = 1280
HEADER_H = 80
TOP_W = 800
TOP_H = 600
PANEL_W = 480
PANEL_H = 200
FOOTER_H = 40
HEIGHT = HEADER_H + TOP_H + FOOTER_H
ROBOT_COLORS = {"r1": (0, 220, 255), "r2": (255, 150, 40), "r3": (70, 80, 255)}


def fit(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    h, w = frame.shape[:2]
    scale = min(width / max(1, w), height / max(1, h))
    image = cv2.resize(frame, (max(1, int(w * scale)), max(1, int(h * scale))))
    y = (height - image.shape[0]) // 2
    x = (width - image.shape[1]) // 2
    canvas[y:y + image.shape[0], x:x + image.shape[1]] = image
    return canvas


def fill_crop(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    h, w = frame.shape[:2]
    scale = max(width / max(1, w), height / max(1, h))
    image = cv2.resize(frame, (max(1, int(round(w * scale))), max(1, int(round(h * scale)))))
    y = max(0, (image.shape[0] - height) // 2)
    x = max(0, (image.shape[1] - width) // 2)
    return image[y:y + height, x:x + width]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--source-zone", default="A", choices=("A", "B", "C"))
    parser.add_argument("--destination-zone", default="B", choices=("A", "B", "C"))
    parser.add_argument("--retarget-at-phase")
    parser.add_argument("--retarget-destination", choices=("A", "B", "C"))
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    output = Path(args.out).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    world = MultiMasterPiProductionV2(seed=args.seed, width=480, height=360, render=True)
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
            "-pix_fmt", "bgr24", "-s", f"{WIDTH}x{HEIGHT}", "-r", str(args.fps),
            "-i", "-", "-an", "-c:v", "libx264", "-preset", "veryfast",
            "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(output),
        ],
        stdin=subprocess.PIPE,
    )
    if ffmpeg.stdin is None:
        world.close()
        raise RuntimeError("ffmpeg stdin unavailable")

    frame_count = 0
    callback_count = 0
    retargeted = False

    def current_cargo_id() -> str | None:
        for event in reversed(world.warehouse_trace):
            cargo_id = event.get("cargo_id")
            if cargo_id:
                return str(cargo_id)
        return None

    def emit(stage: str | None = None) -> None:
        nonlocal frame_count
        state = world.warehouse_state()
        motion = state.get("motion_metrics") or {}
        current_goal = (state.get("current_goal") or {}).get("destination_zone") or args.destination_zone
        cargo_id = current_cargo_id()
        cargo = state["cargo"].get(cargo_id, {}) if cargo_id else {}
        phase = stage or str(
            world.warehouse_trace[-1].get("phase") if world.warehouse_trace else "ready"
        ).replace("_", " ").upper()
        canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        top_jpeg = world.render_team_jpeg(camera="cctv_warehouse", quality=91)
        top = cv2.imdecode(np.frombuffer(top_jpeg, np.uint8), cv2.IMREAD_COLOR)
        canvas[HEADER_H:HEADER_H + TOP_H, :TOP_W] = fit(top, TOP_W, TOP_H)
        cv2.putText(
            canvas, "WAREHOUSE CHECKPOINTS  |  A BLUE  |  B GREEN  |  C YELLOW",
            (18, HEADER_H + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.50,
            (255, 255, 255), 2, cv2.LINE_AA,
        )

        for index, rid in enumerate(("r1", "r2", "r3")):
            y0 = HEADER_H + index * PANEL_H
            rgb = world.render_rgb(robot_id=rid, camera="robot_cam")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            canvas[y0:y0 + PANEL_H, TOP_W:TOP_W + PANEL_W] = fill_crop(
                bgr, PANEL_W, PANEL_H,
            )
            cv2.rectangle(
                canvas, (TOP_W + 2, y0 + 2),
                (WIDTH - 3, y0 + PANEL_H - 3), ROBOT_COLORS[rid], 3,
            )
            cv2.putText(
                canvas, f"{rid.upper()}  EYE-IN-HAND", (TOP_W + 16, y0 + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, ROBOT_COLORS[rid], 2, cv2.LINE_AA,
            )

        success = phase == "SUCCESS"
        colour = (80, 230, 100) if success else (40, 190, 255)
        cargo_text = "ALL CARGO" if not cargo_id else (
            f"{cargo_id}  pair={'+'.join(cargo.get('carriers') or [])}"
        )
        cv2.putText(
            canvas,
            f"UGRP ADAPTIVE WAREHOUSE  |  SEED {args.seed}  |  GOAL {current_goal}  |  {phase}  |  moved {state['moved_count']}/{state['total_count']}",
            (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.67, colour, 2, cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"{frame_count / max(1, args.fps):04.1f}s  {cargo_text}",
            (18, 61), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 230, 230), 1, cv2.LINE_AA,
        )
        progress = float(state["moved_count"]) / max(1, float(state["total_count"]))
        if cargo_id and cargo:
            spec = world.warehouse_spec_by_id[cargo_id]
            x = float(cargo["position"][0])
            progress = min(1.0, max(0.0, (
                state["moved_count"] + (x - spec.start_xyz[0]) / (spec.goal_xyz[0] - spec.start_xyz[0])
            ) / state["total_count"]))
        bar_x, bar_y, bar_w, bar_h = 80, HEIGHT - 14, 1120, 8
        cv2.rectangle(canvas, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (90, 90, 90), 1)
        cv2.rectangle(
            canvas, (bar_x + 1, bar_y + 1),
            (bar_x + 1 + int((bar_w - 2) * progress), bar_y + bar_h - 1),
            colour, -1,
        )
        cv2.putText(
            canvas,
            f"EMPTY STRAFE {motion.get('empty_lateral_segments', 0)}  |  LOADED LATERAL {100.0 * float(motion.get('loaded_lateral_ratio') or 0.0):.0f}%  |  ROTATE + FORWARD",
            (190, HEIGHT - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
            (210, 210, 210), 1, cv2.LINE_AA,
        )
        ffmpeg.stdin.write(canvas.tobytes())
        frame_count += 1

    def capture() -> None:
        nonlocal callback_count, retargeted
        callback_count += 1
        if (
            not retargeted
            and args.retarget_at_phase
            and args.retarget_destination
            and any(
                event.get("phase") == args.retarget_at_phase
                for event in world.warehouse_trace
            )
        ):
            world.request_warehouse_goal_update(
                args.retarget_destination,
                reason="video_injected_moving_goal",
            )
            retargeted = True
        # The controller exposes dense trajectory samples. Keep a compact
        # review video while retaining sub-10 cm motion between frames.
        if callback_count % 18 == 0:
            emit()

    try:
        for _ in range(args.fps):
            emit("READY")
        world.frame_callback = capture
        results = world.act_parallel([
            {
                "robot_id": rid, "action": "team_zone_transfer",
                "mission_id": f"warehouse_{args.source_zone.lower()}_to_{args.destination_zone.lower()}_video",
                "source_zone": args.source_zone,
                "destination_zone": args.destination_zone, "selector": "all",
            }
            for rid in ("r1", "r2", "r3")
        ])
        world.frame_callback = None
        if not all(result.ok for result in results.values()):
            raise RuntimeError(str({rid: result.reason for rid, result in results.items()}))
        for _ in range(args.fps * 2):
            emit("SUCCESS")
        state = world.warehouse_state()
        report = {
            "ok": True, "output": str(output), "frames": frame_count,
            "video_s": round(frame_count / max(1, args.fps), 3),
            "moved_count": state["moved_count"],
            "remaining_ids": state["remaining_ids"],
            "pairs": [
                event["carriers"] for event in world.warehouse_trace
                if event["phase"] == "cargo_delivered"
            ],
            "transport_mode": "DYNAMIC_INWARD_BODY_GRASP_MECANUM",
            "route": list((state.get("mission") or {}).get("route") or ()),
            "final_goal": (state.get("current_goal") or {}).get("destination_zone"),
            "retargeted": retargeted,
            "role_auctions": sum(
                event.get("phase") == "role_auction_completed"
                for event in world.warehouse_trace
            ),
            "terrain_reports": sum(
                event.get("phase") == "scout_terrain_report"
                for event in world.warehouse_trace
            ),
            "terrain": [
                {
                    "id": item.terrain_id,
                    "kind": item.kind,
                    "center_xy": item.center_xy,
                    "height_m": item.height_m,
                }
                for item in world.warehouse_terrain_specs
            ],
            "local_paths": sum(
                event.get("phase") == "warehouse_local_path_planned"
                for event in world.warehouse_trace
            ),
            "motion_metrics": state.get("motion_metrics"),
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
