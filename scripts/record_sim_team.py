#!/usr/bin/env python3
"""Record one real TEAM-room run from three cached first-person cameras."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.record_sim_pipeline import LatestSnapshot, post_command


PANEL_W = 480
PANEL_H = 360
HEADER_H = 96
FOOTER_H = 50
ROLES = {"r1": "TURN LEFT", "r2": "MOVE FORWARD", "r3": "TURN RIGHT"}
EXPECTED_TOOLS = {"r1": "turn_left", "r2": "move_forward", "r3": "turn_right"}
ROLE_COLORS = {
    "r1": (0, 220, 255),
    "r2": (255, 150, 40),
    "r3": (70, 80, 255),
}


def fit_panel(frame: np.ndarray | None) -> np.ndarray:
    canvas = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    if frame is None:
        cv2.putText(
            canvas, "WAITING FOR CAMERA", (105, 190),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 180), 2, cv2.LINE_AA,
        )
        return canvas
    h, w = frame.shape[:2]
    scale = min(PANEL_W / max(1, w), PANEL_H / max(1, h))
    resized = cv2.resize(frame, (max(1, int(w * scale)), max(1, int(h * scale))))
    y = (PANEL_H - resized.shape[0]) // 2
    x = (PANEL_W - resized.shape[1]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return canvas


def json_request(method: str, url: str, payload: dict | None = None, timeout: float = 10.0) -> dict:
    data = None if payload is None else json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method=method,
    )
    with urlopen(req, timeout=timeout) as response:
        value = json.load(response)
    if not isinstance(value, dict):
        raise RuntimeError(f"non-object response from {url}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--out", required=True)
    parser.add_argument(
        "--bridge",
        default=os.environ.get("UGRP_SIM_BRIDGE_LOCAL", "http://127.0.0.1:8091"),
    )
    parser.add_argument("--team", default="http://127.0.0.1:8082")
    args = parser.parse_args()

    bridge = args.bridge.rstrip("/")
    team = args.team.rstrip("/")
    output = Path(args.out).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    token = (ROOT / ".sim_bridge_token").read_text().strip()

    cameras = {
        rid: LatestSnapshot(f"{bridge}/robot/{rid}/snapshot", interval_s=0.08)
        for rid in ("r1", "r2", "r3")
    }
    for camera in cameras.values():
        camera.start()

    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if all(camera.latest() is not None for camera in cameras.values()):
            break
        errors = [camera.error for camera in cameras.values() if camera.error]
        if errors:
            raise RuntimeError(errors[0])
        time.sleep(0.02)
    else:
        raise RuntimeError("TEAM cameras did not become ready")

    width = PANEL_W * 3
    height = HEADER_H + PANEL_H + FOOTER_H
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}", "-r", str(args.fps), "-i", "-",
            "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "21",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
        ],
        stdin=subprocess.PIPE,
    )
    if ffmpeg.stdin is None:
        raise RuntimeError("ffmpeg stdin unavailable")

    shared = {
        "stage": "CONNECTING",
        "detail": "Waiting for three independent agents",
        "robots": {rid: "WAIT" for rid in ROLES},
        "done": False,
        "done_at": None,
        "error": None,
        "report": {},
    }
    lock = threading.Lock()

    def run_team() -> None:
        started = time.time()
        try:
            post_command(
                bridge, token,
                {"action": "reset", "robot_id": "r1", "seed": args.seed},
            )
            json_request("POST", f"{team}/api/team/reset", {"mode": "sim"})
            message = (
                "세 로봇은 자기에게 지정된 public tool을 반드시 딱 한 번 실행해. "
                "R1은 turn_left(speed=35,duration=0.30), "
                "R2는 move_forward(speed=35,duration=0.30), "
                "R3는 turn_right(speed=35,duration=0.30)를 맡아. "
                "이미 화면을 보고 답만 하지 말고 세 동작을 동시에 시작한 뒤 모두 멈춰."
            )
            posted = json_request(
                "POST", f"{team}/api/team/chat",
                {"mode": "sim", "message": message},
            )
            with lock:
                shared["stage"] = "PARALLEL PLAN"
                shared["detail"] = "R1 + R2 + R3 are proposing independently"

            snap: dict = {}
            while time.time() - started < 60.0:
                snap = json_request("GET", f"{team}/api/team?mode=sim")
                wakes = snap.get("wakeups") or []
                proposals = {
                    w.get("robot_id"): w
                    for w in wakes if w.get("reason") == "consensus_proposal"
                }
                executions = {
                    w.get("robot_id"): w
                    for w in wakes if w.get("reason") == "consensus_execute"
                }
                tool_events = {
                    e.get("robot_id"): e
                    for e in (snap.get("events") or []) if e.get("kind") == "tool"
                }
                robot_status = {}
                for rid in ROLES:
                    proposal = proposals.get(rid) or {}
                    execution = executions.get(rid) or {}
                    if rid in tool_events:
                        robot_status[rid] = "OK" if tool_events[rid].get("ok") else "FAILED"
                    elif execution.get("status") in {"done", "error"}:
                        robot_status[rid] = "DONE" if execution.get("status") == "done" else "ERROR"
                    elif execution:
                        robot_status[rid] = "ACT"
                    elif proposal.get("status") == "done":
                        robot_status[rid] = "READY"
                    elif proposal.get("status") == "claimed":
                        robot_status[rid] = "PLAN"
                    else:
                        robot_status[rid] = "WAIT"
                with lock:
                    shared["robots"] = robot_status
                    if executions:
                        shared["stage"] = "ACT_PARALLEL"
                        shared["detail"] = "First commands synchronized into one worker batch"

                if len(executions) == 3 and all(
                    w.get("status") in {"done", "error"} for w in executions.values()
                ):
                    break
                time.sleep(0.10)
            else:
                raise TimeoutError("TEAM run did not finish within 60 seconds")

            tool_events = [
                e for e in (snap.get("events") or []) if e.get("kind") == "tool"
            ]
            latest_tools = {
                rid: next((e for e in reversed(tool_events) if e.get("robot_id") == rid), {})
                for rid in ROLES
            }
            if not all(
                latest_tools[rid].get("ok") is True
                and latest_tools[rid].get("tool") == EXPECTED_TOOLS[rid]
                for rid in ROLES
            ):
                raise RuntimeError("one or more assigned TEAM motions were not executed")
            executions = [
                w for w in (snap.get("wakeups") or [])
                if w.get("reason") == "consensus_execute"
            ]
            report = {
                "posted_targets": posted.get("targets"),
                "team_elapsed_s": round(time.time() - started, 3),
                "execution_wakes": [
                    {
                        "robot_id": w.get("robot_id"),
                        "claimed_offset_s": round(float(w.get("claimed_at") or started) - started, 3),
                        "completed_offset_s": round(float(w.get("completed_at") or started) - started, 3),
                        "status": w.get("status"),
                    }
                    for w in executions
                ],
                "tools": latest_tools,
            }
            with lock:
                shared["stage"] = "SUCCESS"
                shared["detail"] = "All three assigned motions completed"
                shared["robots"] = {rid: "OK" for rid in ROLES}
                shared["report"] = report
        except Exception as exc:
            with lock:
                shared["stage"] = "FAILED"
                shared["detail"] = str(exc)[:110]
                shared["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            with lock:
                shared["done"] = True
                shared["done_at"] = time.monotonic()

    worker = threading.Thread(target=run_team, daemon=True)
    worker.start()
    video_started = time.monotonic()
    next_frame = video_started
    try:
        while True:
            now = time.monotonic()
            with lock:
                stage = str(shared["stage"])
                detail = str(shared["detail"])
                robot_status = dict(shared["robots"])
                done = bool(shared["done"])
                done_at = shared["done_at"]
            if done and done_at is not None and now - float(done_at) >= 2.5:
                break
            if now < next_frame:
                time.sleep(min(0.02, next_frame - now))
                continue

            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            for index, rid in enumerate(("r1", "r2", "r3")):
                x = index * PANEL_W
                panel = fit_panel(cameras[rid].latest())
                canvas[HEADER_H:HEADER_H + PANEL_H, x:x + PANEL_W] = panel
                color = ROLE_COLORS[rid]
                cv2.rectangle(
                    canvas,
                    (x + 2, HEADER_H + 2),
                    (x + PANEL_W - 3, HEADER_H + PANEL_H - 3),
                    color,
                    3,
                )
                cv2.putText(
                    canvas,
                    f"{rid.upper()}  {ROLES[rid]}",
                    (x + 18, HEADER_H + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.68, color, 2, cv2.LINE_AA,
                )
                cv2.putText(
                    canvas,
                    robot_status.get(rid, "WAIT"),
                    (x + 18, HEADER_H + 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (80, 230, 100) if robot_status.get(rid) == "OK" else (245, 245, 245),
                    2, cv2.LINE_AA,
                )

            stage_color = (80, 230, 100) if stage == "SUCCESS" else (40, 190, 255)
            if stage == "FAILED":
                stage_color = (70, 70, 255)
            elapsed = now - video_started
            cv2.putText(
                canvas,
                f"UGRP TEAM PARALLEL  |  SEED {args.seed}  |  {stage}",
                (22, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.82, stage_color, 2, cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                f"{elapsed:05.1f}s  {detail}",
                (22, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (230, 230, 230), 1, cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                "INDEPENDENT LLMs  ->  CONSENSUS  ->  ONE SYNCHRONIZED FIRST-ACTION BATCH",
                (250, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1, cv2.LINE_AA,
            )
            ffmpeg.stdin.write(canvas.tobytes())
            next_frame += 1.0 / max(1, args.fps)
    finally:
        worker.join(timeout=70.0)
        for camera in cameras.values():
            camera.close()
        ffmpeg.stdin.close()
        rc = ffmpeg.wait(timeout=30.0)
        # Leave the live demo deterministic and idle for the next command.
        post_command(
            bridge, token,
            {"action": "reset", "robot_id": "r1", "seed": args.seed},
        )

    if rc:
        raise RuntimeError(f"ffmpeg exited {rc}")
    with urlopen(f"{bridge}/health", timeout=3.0) as response:
        health = json.load(response)
    with lock:
        result = {
            "ok": shared["error"] is None,
            "output": str(output),
            "video_wall_s": round(time.monotonic() - video_started, 3),
            "team": dict(shared["report"]),
            "error": shared["error"],
            "active_streams_after": health.get("active_streams"),
            "queue_depth_after": health.get("queue_depth"),
            "inflight_after": health.get("inflight"),
        }
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
