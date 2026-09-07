#!/usr/bin/env python3
"""Render fixed MasterPi geometry-review views from the production V2 model."""
from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
from PIL import Image

from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2
from harness.real_geometry import SERVO_DEVIATION


# Manufacturer deviation-adjustment reference: arm brackets in one straight
# line.  This is a geometry inspection pose, not a control/training action.
REFERENCE_POSE = {
    1: 2000,
    3: 1500 + SERVO_DEVIATION[3],
    4: 1500 + SERVO_DEVIATION[4],
    5: 1500 + SERVO_DEVIATION[5],
    6: 1500,
}

VIEWS = {
    "three_quarter": dict(azimuth=135.0, elevation=-18.0, distance=0.62),
    "side": dict(azimuth=90.0, elevation=-8.0, distance=0.58),
    "front": dict(azimuth=180.0, elevation=-8.0, distance=0.58),
    "top_three_quarter": dict(azimuth=140.0, elevation=-32.0, distance=0.64),
}


def render(output: Path, *, width: int = 640, height: int = 480) -> list[Path]:
    output.mkdir(parents=True, exist_ok=True)
    world = MasterPiDynamicsV2(seed=1, render=False, width=width, height=height, use_calibration_manifest=False)
    renderer = mujoco.Renderer(world.model, height=height, width=width)
    paths: list[Path] = []
    try:
        world.set_base_pose_for_test()
        world.set_servo_pulses(REFERENCE_POSE, forward_only=True)
        # Move task objects away so the review frames isolate the robot geometry.
        for idx, name in enumerate(("red_block", "blue_block", "yellow_block")):
            world.set_free_body_pose_for_reset(name, (1.25, 0.75 + idx * 0.12, 0.015))
        mujoco.mj_forward(world.model, world.data)

        for name, params in VIEWS.items():
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:] = [0.0, 0.0, 0.155]
            cam.azimuth = params["azimuth"]
            cam.elevation = params["elevation"]
            cam.distance = params["distance"]
            renderer.update_scene(world.data, camera=cam)
            image = renderer.render().copy()
            path = output / f"masterpi_{name}.jpg"
            Image.fromarray(image).save(path, quality=94)
            paths.append(path)
    finally:
        renderer.close()
        world.close()
    return paths


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, default=Path("outputs/masterpi_geometry_review"))
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()
    for path in render(args.output, width=args.width, height=args.height):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
