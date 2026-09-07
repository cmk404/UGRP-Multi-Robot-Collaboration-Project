#!/usr/bin/env python3
"""Record the live shared-world pickup pipeline as a side-by-side MP4."""
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

from harness.pi_camera import open_mjpeg_stream, pop_jpegs


ROBOT_W = 640
ROBOT_H = 480
HEADER_H = 76


class LatestMjpeg:
    def __init__(self, url: str):
        self.url = url
        self.frame: np.ndarray | None = None
        self.error: str | None = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.stream = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        buf = bytearray()
        try:
            self.stream = open_mjpeg_stream(url=self.url)
            while not self.stop.is_set():
                chunk = self.stream.read(8192)
                if not chunk:
                    raise RuntimeError("MJPEG stream ended")
                buf.extend(chunk)
                frames = pop_jpegs(buf)
                if not frames:
                    continue
                image = cv2.imdecode(
                    np.frombuffer(frames[-1], dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if image is not None:
                    with self.lock:
                        self.frame = image
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            if self.stream is not None:
                self.stream.close()

    def latest(self) -> np.ndarray | None:
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self) -> None:
        self.stop.set()
        if self.stream is not None:
            try:
                self.stream.close()
            except Exception:
                pass
        self.thread.join(timeout=2.0)


class LatestSnapshot:
    """Poll the Bridge cache without creating a presentation subscription."""

    def __init__(self, url: str, interval_s: float = 0.08):
        self.url = url
        self.interval_s = max(0.04, float(interval_s))
        self.frame: np.ndarray | None = None
        self.error: str | None = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        while not self.stop.is_set():
            try:
                with urlopen(self.url, timeout=1.0) as response:
                    data = response.read()
                image = cv2.imdecode(
                    np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if image is not None:
                    with self.lock:
                        self.frame = image
                self.error = None
            except Exception as exc:
                self.error = f"{type(exc).__name__}: {exc}"
            self.stop.wait(self.interval_s)

    def latest(self) -> np.ndarray | None:
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=2.0)


def fit(frame: np.ndarray | None) -> np.ndarray:
    if frame is None:
        canvas = np.zeros((ROBOT_H, ROBOT_W, 3), dtype=np.uint8)
        cv2.putText(
            canvas, "WAITING FOR STREAM", (155, 250),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (190, 190, 190), 2, cv2.LINE_AA,
        )
        return canvas
    h, w = frame.shape[:2]
    scale = min(ROBOT_W / max(1, w), ROBOT_H / max(1, h))
    resized = cv2.resize(frame, (max(1, int(w * scale)), max(1, int(h * scale))))
    canvas = np.zeros((ROBOT_H, ROBOT_W, 3), dtype=np.uint8)
    y = (ROBOT_H - resized.shape[0]) // 2
    x = (ROBOT_W - resized.shape[1]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return canvas


def post_command(base: str, token: str, payload: dict, timeout: float = 120.0) -> dict:
    req = Request(
        base + "/command",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "X-UGRP-Sim-Token": token},
        method="POST",
    )
    with urlopen(req, timeout=timeout) as response:
        return json.load(response)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--robot-id", default="r1", choices=("r1", "r2", "r3"))
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument(
        "--first-person-only", action="store_true",
        help="poll cached robot snapshots without enabling presentation rendering",
    )
    ap.add_argument("--out", required=True)
    ap.add_argument("--bridge", default=os.environ.get("UGRP_SIM_BRIDGE_LOCAL", "http://127.0.0.1:8091"))
    args = ap.parse_args()

    output = Path(args.out).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    token = (ROOT / ".sim_bridge_token").read_text().strip()
    bridge = args.bridge.rstrip("/")
    robot = (
        LatestSnapshot(f"{bridge}/robot/{args.robot_id}/snapshot")
        if args.first_person_only
        else LatestMjpeg(f"{bridge}/robot/{args.robot_id}/stream")
    )
    observer = None if args.first_person_only else LatestMjpeg(
        f"{bridge}/observer/{args.robot_id}_cctv_front_left/stream"
    )
    robot.start()
    if observer is not None:
        observer.start()

    width = ROBOT_W if args.first_person_only else ROBOT_W * 2
    height = ROBOT_H + HEADER_H
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}", "-r", str(args.fps), "-i", "-",
            "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "22",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
        ],
        stdin=subprocess.PIPE,
    )
    if ffmpeg.stdin is None:
        raise RuntimeError("ffmpeg stdin unavailable")

    shared = {
        "stage": "CONNECTING",
        "detail": "Waiting for live cameras",
        "done": False,
        "done_at": None,
        "results": [],
        "error": None,
    }
    state_lock = threading.Lock()

    def pipeline() -> None:
        try:
            post_command(bridge, token, {"action": "reset", "robot_id": args.robot_id, "seed": args.seed})
            for action in ("search", "track", "approach", "pick"):
                with state_lock:
                    shared["stage"] = action.upper()
                    shared["detail"] = f"Executing public skill: {action}"
                started = time.perf_counter()
                result = post_command(
                    bridge, token,
                    {"action": action, "robot_id": args.robot_id, "target_color": "red"},
                )
                row = {
                    "action": action,
                    "ok": bool(result.get("ok")),
                    "wall_s": round(time.perf_counter() - started, 3),
                    "worker_action_s": result.get("worker_action_s"),
                    "observer_status": result.get("sim_observer_status"),
                    "reason": result.get("reason"),
                }
                with state_lock:
                    shared["results"].append(row)
                    shared["detail"] = (
                        f"{action}: {'OK' if row['ok'] else 'FAILED'} "
                        f"({row['wall_s']:.2f}s)"
                    )
                if not row["ok"]:
                    raise RuntimeError(str(row["reason"] or f"{action} failed"))
            with state_lock:
                shared["stage"] = "SUCCESS"
                shared["detail"] = "Stable grasp confirmed by MuJoCo evaluator"
        except Exception as exc:
            with state_lock:
                shared["stage"] = "FAILED"
                shared["detail"] = str(exc)[:110]
                shared["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            with state_lock:
                shared["done"] = True
                shared["done_at"] = time.monotonic()

    # Wait for both subscription edges and their first frame without an
    # arbitrary long startup sleep.
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if robot.latest() is not None and (
            observer is None or observer.latest() is not None
        ):
            break
        if robot.error or (observer is not None and observer.error):
            raise RuntimeError(robot.error or (observer.error if observer else None))
        time.sleep(0.02)

    worker = threading.Thread(target=pipeline, daemon=True)
    worker.start()
    video_started = time.monotonic()
    next_frame = video_started
    try:
        while True:
            now = time.monotonic()
            with state_lock:
                stage = str(shared["stage"])
                detail = str(shared["detail"])
                done = bool(shared["done"])
                done_at = shared["done_at"]
            if done and done_at is not None and now - float(done_at) >= 2.5:
                break
            if now < next_frame:
                time.sleep(min(0.02, next_frame - now))
                continue
            left = fit(robot.latest())
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            canvas[HEADER_H:, :ROBOT_W] = left
            if observer is not None:
                right = fit(observer.latest())
                canvas[HEADER_H:, ROBOT_W:] = right
            color = (70, 220, 90) if stage == "SUCCESS" else (40, 190, 255)
            if stage == "FAILED":
                color = (80, 80, 255)
            elapsed = now - video_started
            cv2.putText(canvas, f"UGRP SIM  |  SEED {args.seed}  |  {stage}", (22, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.78, color, 2, cv2.LINE_AA)
            cv2.putText(canvas, f"{elapsed:05.1f}s  {detail}", (22, 61), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (230, 230, 230), 1, cv2.LINE_AA)
            cv2.putText(canvas, "R1 FIRST-PERSON", (18, HEADER_H + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
            if observer is not None:
                cv2.putText(canvas, "R1 THIRD-PERSON OBSERVER", (ROBOT_W + 18, HEADER_H + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
            ffmpeg.stdin.write(canvas.tobytes())
            next_frame += 1.0 / max(1, args.fps)
    finally:
        worker.join(timeout=140.0)
        robot.close()
        if observer is not None:
            observer.close()
        ffmpeg.stdin.close()
        rc = ffmpeg.wait(timeout=30.0)
        # Leave the shared demo in its deterministic initial state.
        post_command(bridge, token, {"action": "reset", "robot_id": args.robot_id, "seed": args.seed})
    if rc:
        raise RuntimeError(f"ffmpeg exited {rc}")
    with state_lock:
        report = {
            "ok": shared["error"] is None,
            "output": str(output),
            "video_wall_s": round(time.monotonic() - video_started, 3),
            "results": list(shared["results"]),
            "error": shared["error"],
        }
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
