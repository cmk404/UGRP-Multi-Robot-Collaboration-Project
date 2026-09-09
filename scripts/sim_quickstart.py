"""Small deterministic, offline proof that the camera-team simulator runs.

This intentionally demonstrates only physics motion and rendering.  It does
not run a planner, contact the network, or validate autonomous transport.
"""

from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw


DEFAULT_SIM_SECONDS = 1.5
DEFAULT_FRAME_COUNT = 12
MIN_DISPLACEMENT_M = 0.005


def _position(world: Any, robot_id: str) -> tuple[float, float]:
    xyz = world.robot(robot_id).base_xyz()
    return float(xyz[0]), float(xyz[1])


def _has_visual_detail(image: Image.Image) -> bool:
    colors = image.convert("RGB").getcolors(maxcolors=image.width * image.height)
    return colors is None or len(colors) >= 8


def _frame(world: Any, robot_id: str, sim_time: float) -> Image.Image:
    overview = Image.open(io.BytesIO(world.render_team_jpeg(camera="cctv_warehouse", quality=84))).convert("RGB")
    camera = Image.fromarray(world.render_rgb(robot_id=robot_id, camera="robot_cam")).convert("RGB")
    if not _has_visual_detail(overview) or not _has_visual_detail(camera):
        raise RuntimeError("overview or robot camera frame is blank or nearly blank")
    size = (320, 240)
    overview.thumbnail(size, Image.Resampling.LANCZOS)
    camera.thumbnail(size, Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (640, 268), "black")
    canvas.paste(overview, ((320 - overview.width) // 2, 28 + (240 - overview.height) // 2))
    canvas.paste(camera, (320 + (320 - camera.width) // 2, 28 + (240 - camera.height) // 2))
    draw = ImageDraw.Draw(canvas)
    draw.text((8, 7), f"Overview | simulated time {sim_time:.2f} s", fill="white")
    draw.text((328, 7), f"{robot_id.upper()} camera | physical wheel drive", fill="white")
    return canvas


def run(output: Path, *, seed: int = 41, seconds: float = DEFAULT_SIM_SECONDS,
        frame_count: int = DEFAULT_FRAME_COUNT) -> dict[str, Any]:
    """Run the bounded demo and return the summary written to ``output``."""
    if not math.isfinite(seconds) or not 0.2 <= seconds <= 5.0:
        raise ValueError("seconds must be finite and between 0.2 and 5.0")
    if not 3 <= frame_count <= 30:
        raise ValueError("frame_count must be between 3 and 30")
    output = output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)

    # Imports stay here so --help and argument validation do not initialize
    # MuJoCo or any rendering resources.
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2

    world = None
    port = None
    frames: list[Image.Image] = []
    robot_id = "r1"
    try:
        world = MultiMasterPiProductionV2(
            warehouse_layout="camera_team", seed=seed, width=320, height=240,
            render=True, warehouse_cargo_ids=("small_box_01",),
        )
        port = CameraRobotPort(world, robot_id)
        started_at = float(world.data.time)
        initial_xy = _position(world, robot_id)
        lease_end = started_at
        for index in range(frame_count):
            target = started_at + seconds * index / (frame_count - 1)
            while float(world.data.time) + 1e-9 < target:
                now = float(world.data.time)
                if now + 1e-9 >= lease_end:
                    lease_duration = min(1.0, started_at + seconds - now)
                    port.apply(
                        {"kind": "drive", "forward": 0.12, "turn": 0.0,
                         "duration_s": lease_duration},
                        now,
                    )
                    lease_end = now + lease_duration
                world.robot(robot_id).advance_to_sim_time(min(target, lease_end))
                port.tick(float(world.data.time))
            frames.append(_frame(world, robot_id, float(world.data.time)))
        port.stop()
        final_xy = _position(world, robot_id)
        elapsed = float(world.data.time) - started_at
    finally:
        try:
            if port is not None:
                port.stop()
        finally:
            if world is not None:
                world.close()

    displacement = math.dist(initial_xy, final_xy)
    if displacement < MIN_DISPLACEMENT_M:
        raise RuntimeError(f"robot displacement too small: {displacement:.6f} m")
    if not frames:
        raise RuntimeError("no rendered frames")

    gif_path = output / "quickstart.gif"
    frames[0].save(
        gif_path, save_all=True, append_images=frames[1:], duration=140,
        loop=0, optimize=False,
    )
    summary = {
        "ok": True,
        "mode": "offline_deterministic_physics_demo",
        "seed": seed,
        "simulated_time_s": round(elapsed, 4),
        "robot_id": robot_id,
        "initial_xy_m": [round(value, 6) for value in initial_xy],
        "final_xy_m": [round(value, 6) for value in final_xy],
        "robot_displacement_m": round(displacement, 6),
        "frame_count": len(frames),
        "outputs": {"animation": gif_path.name, "summary": "summary.json"},
        "scope": "Physics motion and camera rendering only; no LLM, network, hardware, autonomous transport, or task-success validation.",
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="new output directory")
    parser.add_argument("--seed", type=int, default=41)
    parser.add_argument("--seconds", type=float, default=DEFAULT_SIM_SECONDS)
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAME_COUNT)
    args = parser.parse_args()
    try:
        summary = run(args.output, seed=args.seed, seconds=args.seconds, frame_count=args.frames)
    except (ValueError, FileExistsError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
