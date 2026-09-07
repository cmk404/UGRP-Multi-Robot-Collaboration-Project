#!/usr/bin/env python3
"""Extract one MasterPi chassis calibration trial from a fixed overhead video.

Attach two ArUco markers rigidly to the robot, one toward the rear and one toward
the front. Their measured centre-to-centre separation is the metric scale at the
robot's own height, avoiding a guessed camera-height/floor-plane scale.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def _angle_wrap(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def _marker_centers(frame, dictionary, detector) -> dict[int, np.ndarray]:
    corners, ids, _rejected = detector.detectMarkers(frame)
    if ids is None:
        return {}
    out: dict[int, np.ndarray] = {}
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        pts = np.asarray(marker_corners, dtype=float).reshape(-1, 2)
        out[int(marker_id)] = pts.mean(axis=0)
    return out


def smooth(values: np.ndarray, radius: int = 2) -> np.ndarray:
    if len(values) < 3 or radius <= 0:
        return values.copy()
    out = np.empty_like(values)
    for i in range(len(values)):
        lo = max(0, i - radius)
        hi = min(len(values), i + radius + 1)
        out[i] = np.median(values[lo:hi], axis=0)
    return out


def analyze(
    video: Path,
    *,
    rear_id: int,
    front_id: int,
    marker_separation_m: float,
    drive_s: float,
    coast_s: float,
    motion_threshold_mps: float = 0.015,
) -> dict:
    if marker_separation_m <= 0:
        raise ValueError("marker_separation_m must be positive")
    if drive_s <= 0 or coast_s < 0:
        raise ValueError("drive_s must be >0 and coast_s >=0")
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"could not open {video}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
    if fps <= 1.0:
        fps = 30.0
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    times: list[float] = []
    centers: list[np.ndarray] = []
    headings: list[np.ndarray] = []
    ppm: list[float] = []
    frame_idx = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            markers = _marker_centers(frame, dictionary, detector)
            if rear_id in markers and front_id in markers:
                rear = markers[rear_id]
                front = markers[front_id]
                vec_image = front - rear
                sep_px = float(np.linalg.norm(vec_image))
                if sep_px >= 5.0:
                    # Cartesian screen coordinates: x right, y up.
                    rear_xy = np.asarray([rear[0], -rear[1]], dtype=float)
                    front_xy = np.asarray([front[0], -front[1]], dtype=float)
                    vec = front_xy - rear_xy
                    headings.append(vec / np.linalg.norm(vec))
                    centers.append((rear_xy + front_xy) / 2.0)
                    ppm.append(sep_px / marker_separation_m)
                    times.append(frame_idx / fps)
            frame_idx += 1
    finally:
        cap.release()
    if len(times) < max(12, int(round(fps * 0.5))):
        raise RuntimeError("too few frames contained both ArUco markers")

    t = np.asarray(times, dtype=float)
    c_px = smooth(np.asarray(centers, dtype=float), radius=2)
    h = smooth(np.asarray(headings, dtype=float), radius=2)
    h /= np.maximum(1e-9, np.linalg.norm(h, axis=1, keepdims=True))
    ppm_arr = smooth(np.asarray(ppm, dtype=float).reshape(-1, 1), radius=3).reshape(-1)

    # Integrate pixel-to-metre motion using the local two-marker scale at each
    # time step. This is more robust than assuming one global pixel scale.
    pos_m = np.zeros_like(c_px)
    for i in range(1, len(t)):
        scale = max(1e-6, 0.5 * (ppm_arr[i - 1] + ppm_arr[i]))
        pos_m[i] = pos_m[i - 1] + (c_px[i] - c_px[i - 1]) / scale
    dt = np.diff(t)
    step_dist = np.linalg.norm(np.diff(pos_m, axis=0), axis=1)
    speed = np.zeros(len(t), dtype=float)
    good_dt = np.maximum(dt, 1e-6)
    speed[1:] = step_dist / good_dt
    speed = smooth(speed.reshape(-1, 1), radius=2).reshape(-1)

    # Require a short run of motion, not one noisy frame.
    moving = speed >= float(motion_threshold_mps)
    onset_idx = None
    run = max(2, int(round(fps * 0.10)))
    for i in range(0, max(0, len(moving) - run + 1)):
        if bool(np.all(moving[i:i + run])):
            onset_idx = i
            break
    if onset_idx is None:
        raise RuntimeError("could not detect chassis motion onset")
    onset_t = float(t[onset_idx])
    command_end_t = onset_t + float(drive_s)
    final_t = min(float(t[-1]), command_end_t + float(coast_s))
    end_idx = int(np.argmin(np.abs(t - final_t)))
    command_end_idx = int(np.argmin(np.abs(t - command_end_t)))

    h0 = h[onset_idx]
    left0 = np.asarray([-h0[1], h0[0]], dtype=float)
    delta = pos_m[end_idx] - pos_m[onset_idx]
    dx = float(np.dot(delta, h0))
    dy = float(np.dot(delta, left0))
    theta0 = math.atan2(float(h0[1]), float(h0[0]))
    theta1 = math.atan2(float(h[end_idx, 1]), float(h[end_idx, 0]))
    dyaw_deg = math.degrees(_angle_wrap(theta1 - theta0))
    stop_distance = float(np.linalg.norm(pos_m[end_idx] - pos_m[command_end_idx]))
    segment = speed[onset_idx:end_idx + 1]
    peak_speed = float(np.percentile(segment, 95)) if segment.size else 0.0

    return {
        "dx_m": dx,
        "dy_m": dy,
        "dyaw_deg": dyaw_deg,
        "peak_speed_mps": peak_speed,
        "stop_distance_m": stop_distance,
        "measurement_source": "fixed_overhead_video_two_aruco_scale",
        "video": str(video),
        "video_fps": fps,
        "tracked_frames": len(t),
        "motion_onset_s": onset_t,
        "command_end_s": command_end_t,
        "analysis_end_s": float(t[end_idx]),
        "mean_marker_scale_px_per_m": float(np.mean(ppm_arr[onset_idx:end_idx + 1])),
        "rear_marker_id": rear_id,
        "front_marker_id": front_id,
        "marker_separation_m": marker_separation_m,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("video", type=Path)
    ap.add_argument("--rear-id", type=int, default=10)
    ap.add_argument("--front-id", type=int, default=11)
    ap.add_argument("--marker-separation-m", type=float, required=True)
    ap.add_argument("--drive-s", type=float, required=True)
    ap.add_argument("--coast-s", type=float, default=0.50)
    ap.add_argument("--motion-threshold-mps", type=float, default=0.015)
    args = ap.parse_args()
    out = analyze(
        args.video,
        rear_id=args.rear_id,
        front_id=args.front_id,
        marker_separation_m=args.marker_separation_m,
        drive_s=args.drive_s,
        coast_s=args.coast_s,
        motion_threshold_mps=args.motion_threshold_mps,
    )
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
