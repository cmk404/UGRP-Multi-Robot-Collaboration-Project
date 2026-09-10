#!/usr/bin/env python3
"""Reproduce the v12 orange-beam minAreaRect contamination audit."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROUNDS = (0, 141, 216, 333, 374)
ORANGE_LOW = np.array((3, 105, 45), np.uint8)
ORANGE_HIGH = np.array((24, 255, 255), np.uint8)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nearest_evaluation(rows: list[dict], target: float) -> dict:
    return min(rows, key=lambda row: abs(float(row["sim_time_s"]) - target))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=Path("/Users/changmin/projects/ugrp/outputs/pixel-grasp-20260910-r1-v12"),
    )
    parser.add_argument("--json-out", type=Path, default=Path("/tmp/v12_beam_rect_audit.json"))
    parser.add_argument("--image-out", type=Path, default=Path("/tmp/v12_beam_mask_rect_audit.jpg"))
    args = parser.parse_args()
    root = args.run_dir.resolve()
    metadata_path = root / "result.json"
    if not metadata_path.exists():
        metadata_path = root / "progress.json"
    report = json.loads(metadata_path.read_text())
    calls = {int(call["round"]): call for call in report["calls"]}
    evaluation_path = root / "evaluation-only.jsonl"
    evaluations = [json.loads(line) for line in evaluation_path.read_text().splitlines() if line]
    records = []
    panels = []

    for round_index in ROUNDS:
        path = root / "r1" / f"{round_index:03d}-top.jpg"
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f"cannot decode {path}")
        height, width = image.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), ORANGE_LOW, ORANGE_HIGH)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        reported = calls[round_index]["observation"].get("beam")
        reference_center = np.asarray(reported["center"] if reported else [.509, .508]) * (width, height)
        contour = min(
            contours,
            key=lambda item: np.linalg.norm(np.asarray(cv2.minAreaRect(item)[0]) - reference_center),
        )
        rect = cv2.minAreaRect(contour)
        (cx, cy), (rw, rh), angle = rect
        rect_length, rect_width = sorted((float(rw), float(rh)), reverse=True)
        x, y, box_width, box_height = cv2.boundingRect(contour)
        spans = []
        for row in range(y, y + box_height):
            columns = np.where(mask[row, x:x + box_width] > 0)[0] + x
            if len(columns):
                spans.append({
                    "y": row,
                    "left": int(columns.min()),
                    "right": int(columns.max()),
                    "count": int(len(columns)),
                    "center_x": float((columns.min() + columns.max()) / 2),
                })
        central = [span for span in spans if y + .15 * box_height <= span["y"] <= y + .65 * box_height]
        median_width = float(np.median([span["count"] for span in central]))
        median_center = float(np.median([span["center_x"] for span in central]))
        accepted = [
            span for span in spans
            if .75 * median_width <= span["count"] <= 1.25 * median_width
            and abs(span["center_x"] - median_center) <= .2 * median_width
        ]
        runs: list[list[dict]] = []
        for span in accepted:
            if not runs or span["y"] != runs[-1][-1]["y"] + 1:
                runs.append([])
            runs[-1].append(span)
        core = max(runs, key=len)
        robust_endpoints = [
            [median_center / width, float(core[0]["y"]) / height],
            [median_center / width, float(core[-1]["y"]) / height],
        ]
        samples_per_round = len(evaluations) // int(report["rounds_completed"])
        assert samples_per_round * int(report["rounds_completed"]) == len(evaluations)
        evaluation_index = max(0, round_index * samples_per_round - 1)
        evaluation = evaluations[evaluation_index]
        target_time = evaluation["sim_time_s"]
        record = {
            "round": round_index,
            "image": {
                "path": str(path), "sha256": sha256(path), "image_size": [width, height],
            },
            "reported": reported,
            "recomputed_component": {
                "area_px": float(cv2.contourArea(contour)),
                "bbox_xywh": [x, y, box_width, box_height],
                "min_area_rect": {
                    "center_px": [float(cx), float(cy)], "length_px": rect_length,
                    "width_px": rect_width, "opencv_angle_deg": float(angle),
                    "corners_px": cv2.boxPoints(rect).astype(float).tolist(),
                },
                "central_shaft": {
                    "slice_range_fraction": [.15, .65],
                    "median_width_px": median_width,
                    "median_center_x_px": median_center,
                    "longest_consistent_run_y_px": [core[0]["y"], core[-1]["y"]],
                    "robust_endpoints_normalized": robust_endpoints,
                    "widest_row": max(spans, key=lambda span: span["count"]),
                },
            },
            "output_only_reference": {
                "sample_index": evaluation_index,
                "relation_to_rgb": (
                    "previous round final physics sample, before current action" if round_index else
                    "first post-action sample; no pre-startup evaluation sample exists"
                ),
                "matched_sim_time_s": evaluation["sim_time_s"],
                "beam_position_m": evaluation["position_m"],
                "beam_tilt_deg": evaluation["tilt_deg"],
                "r1_bilateral_contact": evaluation["contacts"]["r1"]["bilateral"],
                "r1_base": evaluation["bases"]["r1"],
                "source": str(evaluation_path),
                "source_sha256": sha256(evaluation_path),
                "usage": "post-run discrimination only; never controller input",
            },
        }
        records.append(record)

        overlay = image.copy()
        cv2.drawContours(overlay, [contour], -1, (0, 255, 0), 2)
        cv2.drawContours(overlay, [cv2.boxPoints(rect).astype(np.int32)], -1, (255, 0, 255), 2)
        crop = overlay[370:650, 570:740]
        mask_crop = np.zeros_like(crop)
        mask_crop[:, :, 0] = mask[370:650, 570:740]
        panel = cv2.resize(np.hstack((crop, mask_crop)), (680, 560), interpolation=cv2.INTER_NEAREST)
        cv2.putText(
            panel, f"r{round_index} rectW={rect_width:.1f} shaft={median_width:.1f}",
            (8, 30), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2,
        )
        panels.append(panel)

    output = {
        "audit": "v12 beam component minAreaRect versus robust central shaft",
        "scope": "vertical scanline diagnostic for selected v12 frames; not a runtime estimator",
        "source_sha": report["source_sha"],
        "metadata": {"path": str(metadata_path), "sha256": sha256(metadata_path)},
        "threshold": {"hsv_low": ORANGE_LOW.tolist(), "hsv_high": ORANGE_HIGH.tolist(),
                      "morphology": "3x3 open then close"},
        "rounds": records,
        "all_evaluated_beam_poses_constant": all(
            row["position_m"] == evaluations[0]["position_m"]
            and row["tilt_deg"] == evaluations[0]["tilt_deg"]
            for row in evaluations
        ),
        "conclusion": (
            "Central shaft width and x remain stable while small endpoint-connected orange tails "
            "rotate and widen the whole-component minAreaRect and displace its endpoints."
        ),
    }
    args.json_out.write_text(json.dumps(output, indent=2) + "\n")
    cv2.imwrite(str(args.image_out), np.vstack(panels))
    print(json.dumps({"json": str(args.json_out), "image": str(args.image_out),
                      "rounds": list(ROUNDS), "source_sha": report["source_sha"]}))


if __name__ == "__main__":
    main()
