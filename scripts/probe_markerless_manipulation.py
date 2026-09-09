"""No-LLM markerless pick-and-release probe in the camera-team fixture.

The scripted actor uses only its own RGB/PWM observations. It acquires one
small box, verifies that the held box is inside the specified visible source zone, and releases it at the
same location. This is a manipulation probe and never claims transport success.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path

import mujoco

from harness.llm_transport_skill import LLMTransportSkill
from harness.visual_drive_guard import validate_visual_drive
from harness.visual_macro_runtime import VisualMacroExecutor
from harness.visual_placement import inspect_placement
from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--initial-delay", type=float, default=1.54)
    parser.add_argument("--seconds", type=float, default=180.0)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--release-zone", choices=("A", "B", "C"), default="B")
    args = parser.parse_args()
    if not 0 <= args.initial_delay <= 10 or not 0 < args.seconds <= 300:
        parser.error("invalid bounded delay or duration")

    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    (out / "inputs").mkdir()
    world = MultiMasterPiProductionV2(
        warehouse_layout="camera_team",
        seed=args.seed,
        render=True,
        warehouse_cargo_ids=("small_box_01",),
    )
    world.model.opt.impratio = 10
    world.model.opt.noslip_iterations = 3
    for zone in "abc":
        geom_id = mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_GEOM, "warehouse_zone_" + zone,
        )
        world.model.geom_group[geom_id] = 0

    port = CameraRobotPort(world, "r1")
    # The release area is explicitly configured from the visible source zone.
    # No route or loaded chassis motion is requested after acquisition.
    actor = LLMTransportSkill("r1", "small_box_01", args.release_zone)
    control = (out / "control.jsonl").open("w")
    commands = (out / "commands.jsonl").open("w")

    def emit(row):
        commands.write(json.dumps(row) + "\n")
        commands.flush()

    executor = VisualMacroExecutor(
        port,
        log_callback=emit,
        drive_guard=lambda action, now: validate_visual_drive(
            port.capture(camera="nav_cam"), action,
        ),
    )
    source = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for base in ("harness", "sim", "calibration")
        for path in sorted(Path(base).rglob("*"))
        if path.is_file() and path.suffix in (".py", ".json", ".xml", ".png")
    }
    (out / "source-manifest.json").write_text(json.dumps(source, indent=2))
    (out / "run-config.json").write_text(json.dumps({
        "output": str(out), "seed": args.seed, "initial_delay": args.initial_delay,
        "seconds": args.seconds, "record": args.record,
        "controller": "scripted_own_rgb_pwm_markerless_manipulation",
        "destination_zone": args.release_zone, "llm_calls": 0,
    }, indent=2))

    video = None
    error = None
    step = 0
    pick_requested = False
    release_requested = False
    released_placement = None
    latest_placement = None
    loaded_drive_requested = False
    start = float(world.data.time)
    try:
        if args.record:
            from scripts.record_visual_box import SingleBoxVideo
            video = SingleBoxVideo(world, out / "motion-1x.mp4", actor.box)
            video.capture(force=True)

        while float(world.data.time) - start <= args.seconds:
            now = float(world.data.time)
            executor.tick(now)
            if executor.idle and now - start >= args.initial_delay:
                if actor.state == "idle":
                    actor.request({"kind": "approach"})

                wrist = port.capture()
                nav = port.capture(camera="nav_cam")
                phase = actor.phase
                action = actor.advance(wrist)

                if actor.state == "ready_to_pick" and not pick_requested:
                    action = actor.request({"kind": "pick"})
                    pick_requested = True
                elif actor.state == "carrying" and not release_requested:
                    latest_placement = inspect_placement(
                        wrist, nav, cargo_id=actor.cargo_id,
                        destination_zone=actor.destination_zone,
                        stage="before_release",
                        held_identity_confirmed=actor.held,
                    )
                    actor.observe_placement(latest_placement)
                    if latest_placement.get("status") == "inside":
                        action = actor.request(
                            {"kind": "release"}, placement_evidence=latest_placement,
                        )
                        release_requested = True
                    else:
                        error = "PRE_RELEASE_PLACEMENT_NOT_CONFIRMED:" + str(latest_placement.get("reason"))
                elif actor.state == "released":
                    release_confirmed = actor.box.reason == "VISUAL_RELEASE_CONFIRMED"
                    released_placement = inspect_placement(
                        wrist, nav, cargo_id=actor.cargo_id,
                        destination_zone=actor.destination_zone,
                        stage="released",
                        held_identity_confirmed=release_confirmed,
                        release_commanded=release_confirmed,
                    )
                    latest_placement = released_placement
                    actor.observe_placement(released_placement)
                    if released_placement.get("status") == "inside":
                        action = actor.request(
                            {"kind": "finish"}, placement_evidence=released_placement,
                        )

                if actor.state == "carrying" and action and action.get("kind") == "drive":
                    loaded_drive_requested = True
                wrist_path = f"inputs/{step:04d}-wrist.jpg"
                nav_path = f"inputs/{step:04d}-nav.jpg"
                (out / wrist_path).write_bytes(base64.b64decode(wrist["image"]))
                (out / nav_path).write_bytes(base64.b64decode(nav["image"]))
                control.write(json.dumps({
                    "step": step, "time": now, "phase": phase,
                    "state": actor.state,
                    "wrist_path": wrist_path, "wrist_sha256": wrist["sha256"],
                    "nav_path": nav_path, "nav_sha256": nav["sha256"],
                    "own_pose_commands": wrist["actuator_state"]["servo_pulses"],
                    "target": actor.box.last_target, "box": actor.box.last_box,
                    "face_alignment": actor.box.last_face_alignment,
                    "attachment": actor.box.last_attachment,
                    "placement": latest_placement, "action": action,
                }) + "\n")
                control.flush()
                step += 1

                if error or actor.state in ("finished", "failure", "grip_uncertain"):
                    break
                if action:
                    if action.get("kind") == "finish":
                        break
                    executor.submit(action, wrist, phase, now)

            world._physics_step_for(world.robot("r1"))
            if video and world.data.time >= video.next_frame:
                video.capture()
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        executor.cancel(float(world.data.time), "markerless_manipulation_probe_complete")
        port.stop()
        manipulation_passed = bool(
            actor.state == "finished"
            and actor.reason == "VISUAL_RELEASE_CONFIRMED"
            and released_placement is not None
            and released_placement.get("status") == "inside"
            and release_requested
            and not loaded_drive_requested
            and error is None
        )
        result = {
            "fixture": "camera_team_markerless_same_location_pick_release",
            "probe_scope": "manipulation_only",
            "controller_inputs": "own RGB + own PWM only",
            "llm_calls": 0,
            "transport_attempted": False,
            "transport_success_claimed": False,
            "seed": args.seed,
            "initial_delay_s": args.initial_delay,
            "state": actor.state,
            "reason": actor.reason,
            "held": actor.held,
            "release_requested": release_requested,
            "released_placement": released_placement,
            "loaded_drive_requested": loaded_drive_requested,
            "manipulation_passed": manipulation_passed,
            "sim_seconds": float(world.data.time) - start,
            "steps": step,
            "error": error,
            "evaluation_only_final": world.warehouse_state(),
        }
        (out / "result.json").write_text(json.dumps(result, indent=2))
        if video:
            video.capture(force=True)
            video.close()
        control.close()
        commands.close()
        world.close()

    print(json.dumps({
        key: value for key, value in result.items() if key != "evaluation_only_final"
    }), flush=True)
    return 0 if result["manipulation_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
