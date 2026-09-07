#!/usr/bin/env python3
"""Fit the v2 reduced MasterPi chassis dynamics to measured physical trials.

Input is JSONL.  Each row is one physical trial measured in a fixed world frame:

  {"split":"fit","direction":"forward","command":35,"drive_s":0.5,
   "coast_s":0.5,"dx_m":0.12,"dy_m":0.00,"dyaw_deg":0.4}

Live chassis commands now use 31..40 because <=30 is the measured hardware dead
zone. This offline fitter still accepts historical 1..40 trials so dead-zone data
can remain in calibration datasets. It never drives hardware; it only consumes already-measured trials and simulates the
v2 model.  It also never marks the calibration manifest validated: servo,
gripper, geometry and held-out acceptance must be completed separately.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Iterable

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2
from scripts.benchmarks.masterpi_calibration_common import real_measurement_source

MANIFEST = ROOT / "sim/masterpi_dynamics_calibration.json"

# Normalized simulator wheel patterns. Physical MotorTransport signs are
# hardware-specific and are handled by scripts/masterpi_control.py.
PATTERNS = {
    "forward": np.array([1, 1, 1, 1], dtype=float),
    "backward": np.array([-1, -1, -1, -1], dtype=float),
    "left": np.array([-1, 1, 1, -1], dtype=float),
    "right": np.array([1, -1, -1, 1], dtype=float),
    "rotate-left": np.array([-1, 1, -1, 1], dtype=float),
    "rotate-right": np.array([1, -1, 1, -1], dtype=float),
}

PARAMS = (
    "motor_time_constant_s",
    "max_forward_force_n",
    "max_lateral_force_n",
    "max_yaw_torque_nm",
    "linear_damping_n_per_mps",
    "yaw_damping_nm_per_radps",
    "stop_linear_damping_n_per_mps",
    "stop_yaw_damping_nm_per_radps",
)

BOUNDS = {
    "motor_time_constant_s": (0.02, 0.40),
    "max_forward_force_n": (0.20, 8.0),
    "max_lateral_force_n": (0.10, 8.0),
    "max_yaw_torque_nm": (0.01, 0.60),
    "linear_damping_n_per_mps": (0.10, 8.0),
    "yaw_damping_nm_per_radps": (0.005, 0.80),
    "stop_linear_damping_n_per_mps": (1.0, 80.0),
    "stop_yaw_damping_nm_per_radps": (0.05, 5.0),
}


def load_trials(path: Path) -> list[dict]:
    rows = []
    for lineno, raw in enumerate(path.read_text().splitlines(), 1):
        raw = raw.strip()
        if not raw or raw.startswith("#"):
            continue
        try:
            row = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"{path}:{lineno}: invalid JSON: {exc}") from exc
        direction = row.get("direction")
        if direction not in PATTERNS:
            raise SystemExit(f"{path}:{lineno}: bad direction {direction!r}")
        command = float(row.get("command", 0))
        if not 1 <= command <= 40:
            raise SystemExit(f"{path}:{lineno}: command must be 1..40")
        drive_s = float(row.get("drive_s", 0))
        coast_s = float(row.get("coast_s", 0.5))
        if drive_s <= 0 or coast_s < 0:
            raise SystemExit(f"{path}:{lineno}: drive_s must be >0 and coast_s >=0")
        for key in ("dx_m", "dy_m", "dyaw_deg"):
            if key not in row or row[key] is None or not math.isfinite(float(row[key])):
                raise SystemExit(f"{path}:{lineno}: missing/non-finite {key}")
        if not real_measurement_source(row.get("measurement_source")):
            raise SystemExit(f"{path}:{lineno}: measurement_source must identify physical REAL evidence")
        row = dict(row)
        row["command"] = command
        row["drive_s"] = drive_s
        row["coast_s"] = coast_s
        row["split"] = str(row.get("split", "fit")).lower()
        rows.append(row)
    if not rows:
        raise SystemExit("no calibration trials")
    return rows


def simulate(row: dict, params: dict[str, float]) -> dict[str, float]:
    w = MasterPiDynamicsV2(seed=17, dynamics=params, use_calibration_manifest=False)
    p0 = w.base_xyz().copy()
    yaw0 = w.base_rpy()[2]
    scale = row["command"] / 40.0
    pattern = PATTERNS[row["direction"]] * scale
    peak_linear = 0.0
    peak_yaw = 0.0
    steps = max(1, int(round(row["drive_s"] / float(w.model.opt.timestep))))
    w.set_motor_commands(pattern)
    for _ in range(steps):
        w._physics_step()
        qv = w.data.qvel[w.base_dadr:w.base_dadr + 6]
        peak_linear = max(peak_linear, float(math.hypot(qv[0], qv[1])))
        peak_yaw = max(peak_yaw, abs(float(qv[5])))
    stop_start = w.base_xyz().copy()
    if row["coast_s"]:
        w.stop(settle_s=row["coast_s"])
    p = w.base_xyz()
    dyaw = math.degrees((w.base_rpy()[2] - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    return {
        "dx_m": float(p[0] - p0[0]),
        "dy_m": float(p[1] - p0[1]),
        "dyaw_deg": float(dyaw),
        "peak_speed_mps": peak_linear,
        "peak_yaw_rate_radps": peak_yaw,
        "stop_distance_m": float(np.linalg.norm(p[:2] - stop_start[:2])),
    }


def residual_vector(rows: Iterable[dict], params: dict[str, float]) -> np.ndarray:
    residuals = []
    for row in rows:
        pred = simulate(row, params)
        # Normalize by the actual acceptance tolerances so translation and yaw
        # have comparable influence on the fit.
        residuals.extend([
            (pred["dx_m"] - float(row["dx_m"])) / 0.025,
            (pred["dy_m"] - float(row["dy_m"])) / 0.025,
            (pred["dyaw_deg"] - float(row["dyaw_deg"])) / 5.0,
        ])
        if row.get("peak_speed_mps") is not None:
            denom = max(0.05, abs(float(row["peak_speed_mps"])))
            residuals.append((pred["peak_speed_mps"] - float(row["peak_speed_mps"])) / denom)
        if row.get("stop_distance_m") is not None:
            residuals.append((pred["stop_distance_m"] - float(row["stop_distance_m"])) / 0.02)
    return np.asarray(residuals, dtype=float)


def loss(rows: list[dict], params: dict[str, float]) -> float:
    r = residual_vector(rows, params)
    return float(np.mean(np.square(r))) if r.size else float("inf")


def clipped(params: dict[str, float]) -> dict[str, float]:
    return {k: max(BOUNDS[k][0], min(BOUNDS[k][1], float(params[k]))) for k in PARAMS}


def fit(rows: list[dict], initial: dict[str, float], passes: int = 8) -> tuple[dict[str, float], float]:
    """Dependency-free bounded coordinate pattern search.

    Calibration datasets are small and simulation is deterministic, so a robust
    pattern search is adequate here and avoids adding SciPy to the runtime image.
    """
    current = clipped(initial)
    best = loss(rows, current)
    # Multiplicative step works across parameters whose scales differ by orders.
    factors = [1.70, 1.40, 1.22, 1.12, 1.06, 1.03, 1.015, 1.008]
    for factor in factors[:max(1, passes)]:
        improved = True
        while improved:
            improved = False
            for key in PARAMS:
                base = current[key]
                candidates = []
                for candidate_value in (base / factor, base * factor):
                    trial = dict(current)
                    trial[key] = candidate_value
                    trial = clipped(trial)
                    candidates.append((loss(rows, trial), trial))
                cand_loss, cand = min(candidates, key=lambda x: x[0])
                if cand_loss + 1e-12 < best:
                    current, best = cand, cand_loss
                    improved = True
    return current, best


def metrics(rows: list[dict], params: dict[str, float]) -> dict:
    trans = []
    yaw = []
    peak_rel = []
    stop = []
    predictions = []
    for row in rows:
        pred = simulate(row, params)
        predictions.append({"trial": row, "prediction": pred})
        trans.append(math.hypot(pred["dx_m"] - float(row["dx_m"]), pred["dy_m"] - float(row["dy_m"])))
        yaw.append(abs(pred["dyaw_deg"] - float(row["dyaw_deg"])))
        if row.get("peak_speed_mps") is not None:
            actual = abs(float(row["peak_speed_mps"]))
            peak_rel.append(abs(pred["peak_speed_mps"] - actual) / max(actual, 0.05))
        if row.get("stop_distance_m") is not None:
            stop.append(abs(pred["stop_distance_m"] - float(row["stop_distance_m"])))
    return {
        "trials": len(rows),
        "translation_endpoint_mae_m": float(np.mean(trans)) if trans else None,
        "yaw_endpoint_mae_deg": float(np.mean(yaw)) if yaw else None,
        "peak_speed_relative_error": float(np.mean(peak_rel)) if peak_rel else None,
        "stop_distance_mae_m": float(np.mean(stop)) if stop else None,
        "predictions": predictions,
    }


def manifest_initial() -> dict[str, float]:
    defaults = MasterPiDynamicsV2(seed=1, use_calibration_manifest=False).dynamics
    try:
        manifest = json.loads(MANIFEST.read_text())
        saved = manifest.get("parameters", {})
    except Exception:
        saved = {}
    out = dict(defaults)
    for key in PARAMS:
        value = saved.get(key)
        if isinstance(value, (int, float)):
            out[key] = float(value)
    return clipped(out)


def update_manifest(params: dict[str, float], fit_metrics: dict, hold_metrics: dict) -> None:
    manifest = json.loads(MANIFEST.read_text())
    manifest["validated"] = False
    manifest["validated_at"] = None
    manifest.setdefault("parameters", {}).update({k: float(v) for k, v in params.items()})
    results = manifest.setdefault("results", {})
    results["fit_trials"] = int(fit_metrics["trials"])
    results["held_out_trials"] = int(hold_metrics["trials"])
    for key in ("translation_endpoint_mae_m", "yaw_endpoint_mae_deg", "peak_speed_relative_error", "stop_distance_mae_m"):
        results[key] = hold_metrics.get(key)
    MANIFEST.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trials", type=Path)
    ap.add_argument("--passes", type=int, default=8)
    ap.add_argument("--write-manifest", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    rows = load_trials(args.trials)
    fit_rows = [r for r in rows if r["split"] != "holdout"]
    hold_rows = [r for r in rows if r["split"] == "holdout"]
    if len(fit_rows) < 6:
        raise SystemExit("need at least 6 fit trials spanning multiple directions/commands")
    initial = manifest_initial()
    params, fit_loss = fit(fit_rows, initial, passes=args.passes)
    fit_metrics = metrics(fit_rows, params)
    hold_metrics = metrics(hold_rows, params)
    out = {
        "classification": "platform_calibration_not_research_result",
        "parameters": params,
        "normalized_fit_mse": fit_loss,
        "fit": {k: v for k, v in fit_metrics.items() if k != "predictions"},
        "holdout": {k: v for k, v in hold_metrics.items() if k != "predictions"},
        "manifest_was_validated": False,
    }
    if args.write_manifest:
        update_manifest(params, fit_metrics, hold_metrics)
        out["manifest_updated"] = str(MANIFEST)
        out["manifest_validated"] = False
    if args.json:
        print(json.dumps(out, indent=2, ensure_ascii=False))
    else:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        if not hold_rows:
            print("WARNING: no holdout trials; calibration cannot be accepted.")
        print("NOTE: this fitter never sets validated=true. Servo/gripper and held-out acceptance remain required.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
