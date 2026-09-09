"""Replay N3 through gripper-open, then diagnose current release verification.

This bounded no-LLM diagnostic replays the exact macro payloads and submission
simulation times from the frozen seed-45 N3 run through control step 240.  That
step is the actual gripper-open command.  The current VisualBoxSkill then owns
retraction, verify_release, and its fresh left/right/home ground-stationarity
sweep.  Actor decisions receive only own RGB/PWM observations; world state is
written solely to separate evaluation-only outputs.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any, Mapping

import mujoco
import numpy as np

import harness.visual_box_skill as visual_box_skill_module
from harness.llm_transport_skill import LLMTransportSkill
from harness.visual_drive_guard import validate_visual_drive
from harness.visual_macro_runtime import VisualMacroExecutor
from harness.visual_placement import inspect_placement
from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2


DEFAULT_FROZEN_RUN = Path("outputs/markerless-improvement-20260909/n3/solo-45")
DEFAULT_CUTOFF_CONTROL_STEP = 240


def _jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def load_prefix_markerless_observer(path: Path) -> tuple[Any, dict[str, Any]]:
    """Load one frozen observer under an isolated, non-package module name."""
    resolved = path.resolve(strict=True)
    if not resolved.is_file():
        raise ValueError(f"prefix markerless source is not a file: {resolved}")
    digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
    spec = importlib.util.spec_from_file_location(
        f"_release_recovery_frozen_markerless_box_{digest[:16]}", resolved,
    )
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot load prefix markerless source: {resolved}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    observer = getattr(module, "observe_ground_box", None)
    if not callable(observer):
        raise ValueError("prefix markerless source has no callable observe_ground_box")
    return observer, {"path": str(resolved), "sha256": digest}


def frozen_release_prefix(
    run: Path = DEFAULT_FROZEN_RUN,
    cutoff_control_step: int = DEFAULT_CUTOFF_CONTROL_STEP,
) -> dict[str, Any]:
    """Return source metadata and macros through a verified open command."""
    source_result = json.loads((run / "result.json").read_text())
    seed = source_result.get("seed")
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError(f"invalid frozen seed: {seed!r}")
    outcomes = source_result.get("outcomes", {})
    destination = outcomes.get("r1", {}).get("destination_zone")
    if destination not in {"A", "B", "C"}:
        raise ValueError(f"invalid frozen destination: {destination!r}")
    control = _jsonl(run / "control.jsonl")
    matches = [row for row in control if row.get("step") == cutoff_control_step]
    if len(matches) != 1:
        raise ValueError(
            f"expected one frozen cutoff step {cutoff_control_step}, found {len(matches)}"
        )
    cutoff = matches[0]
    if (cutoff.get("phase") != "open" or cutoff.get("skill_state") != "releasing"
            or cutoff.get("action") != {
        "kind": "pose", "pulses": {"1": 2000},
    }):
        raise ValueError("frozen cutoff is not the expected actual open command")
    cutoff_time = float(cutoff["time"])
    phases_by_time = {float(row["time"]): row.get("phase") for row in control}
    macros = [
        {
            "time": float(row["time"]),
            "macro": row["macro"],
            "source_frame_sha256": row.get("source_frame_sha256"),
            "source_phase": phases_by_time.get(float(row["time"])),
        }
        for row in _jsonl(run / "commands.jsonl")
        if row.get("event") == "macro_submitted" and float(row["time"]) <= cutoff_time + 1e-9
    ]
    if not macros or macros[-1]["time"] != cutoff_time or macros[-1]["macro"] != cutoff["action"]:
        raise ValueError("commands/control cutoff mismatch")
    # These are camera decisions that intentionally produced no executable
    # macro, but did advance the visual state machine. Omitting them changes
    # the image used to compute pending_hover/carry state on the next macro.
    state_checkpoints = [
        {
            "kind": "state_checkpoint", "time": float(row["time"]),
            "source_step": int(row["step"]), "source_phase": row["phase"],
            "source_skill_state": row["skill_state"],
            "source_frame_sha256": row.get("wrist_sha256"),
        }
        for row in control
        if float(row["time"]) <= cutoff_time + 1e-9
        and row.get("action") == {"kind": "llm_request"}
        and row.get("skill_state") != "idle"
    ]
    # check_grip is the one recorded model action that changes actor state but
    # returns no macro. Pick and release remain coupled to their exact macro
    # events below because those requests lead directly to executable poses.
    model_checkpoints = []
    decisions_path = run / "llm-decisions.jsonl"
    if decisions_path.is_file():
        for row in _jsonl(decisions_path):
            decision = row.get("decision") or {}
            action = decision.get("action") or {}
            if (row.get("event") == "llm_result" and row.get("disposition") == "accepted"
                    and action.get("kind") == "check_grip"
                    and float(row["time"]) <= cutoff_time + 1e-9):
                audit_frames = (row.get("audit") or {}).get("input_frames") or []
                wrist = next((item for item in audit_frames if item.get("camera") == "robot_cam"), {})
                model_checkpoints.append({
                    "kind": "model_request", "time": float(row["time"]),
                    "action": {"kind": "check_grip"},
                    "source_frame_sha256": wrist.get("sha256"),
                })
    events = (
        [{"kind": "macro", **row} for row in macros]
        + state_checkpoints + model_checkpoints
    )
    priority = {"state_checkpoint": 0, "model_request": 1, "macro": 2}
    events.sort(key=lambda row: (row["time"], priority[row["kind"]]))
    return {
        "macros": macros, "cutoff": cutoff, "seed": seed,
        "destination_zone": destination, "events": events,
        "state_checkpoints": state_checkpoints,
        "model_checkpoints": model_checkpoints,
    }


def _same_action(actual: Mapping[str, Any], expected: Mapping[str, Any]) -> bool:
    return json.dumps(actual, sort_keys=True, separators=(",", ":")) == json.dumps(
        expected, sort_keys=True, separators=(",", ":")
    )


def _relative_pose(world: Any, cargo_id: str, robot_id: str = "r1") -> dict[str, Any]:
    """Evaluation-only cargo pose relative to the gripper body."""
    robot = world.robot(robot_id)
    spec = world.warehouse_spec_by_id[cargo_id]
    cargo_bid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, spec.body_name)
    inverse_pos, inverse_quat = np.zeros(3), np.zeros(4)
    relative_pos, relative_quat = np.zeros(3), np.zeros(4)
    mujoco.mju_negPose(
        inverse_pos, inverse_quat,
        world.data.xpos[robot.gripper_bid], world.data.xquat[robot.gripper_bid],
    )
    mujoco.mju_mulPose(
        relative_pos, relative_quat, inverse_pos, inverse_quat,
        world.data.xpos[cargo_bid], world.data.xquat[cargo_bid],
    )
    angle = 2.0 * math.acos(min(1.0, abs(float(relative_quat[0]))))
    return {
        "translation_m": relative_pos.tolist(),
        "quaternion_wxyz": relative_quat.tolist(),
        "orientation_angle_deg": math.degrees(angle),
        "contact": robot.finger_cargo_contact(cargo_id),
    }


def _source_manifest() -> dict[str, str]:
    files = [Path(__file__)]
    for base in ("harness", "sim", "calibration"):
        files.extend(
            path for path in sorted(Path(base).rglob("*"))
            if path.is_file() and path.suffix in (".py", ".json", ".xml", ".png")
        )
    return {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frozen-run", type=Path, default=DEFAULT_FROZEN_RUN)
    parser.add_argument(
        "--cutoff-control-step", type=int, default=DEFAULT_CUTOFF_CONTROL_STEP,
    )
    parser.add_argument(
        "--prefix-markerless-source", type=Path,
        help="frozen markerless_box.py used only around prefix actor.advance calls",
    )
    parser.add_argument("--seconds", type=float, default=240.0)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    if args.cutoff_control_step < 0:
        parser.error("cutoff-control-step must be nonnegative")
    if not 208.0 <= args.seconds <= 300.0:
        parser.error("seconds must be between 208 and 300")

    current_observer = visual_box_skill_module.observe_ground_box
    current_observer_path = Path(current_observer.__code__.co_filename).resolve(strict=True)
    current_observer_meta = {
        "path": str(current_observer_path),
        "sha256": hashlib.sha256(current_observer_path.read_bytes()).hexdigest(),
    }
    prefix_observer = None
    prefix_observer_meta = None
    if args.prefix_markerless_source is not None:
        prefix_observer, prefix_observer_meta = load_prefix_markerless_observer(
            args.prefix_markerless_source,
        )

    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    (out / "inputs").mkdir()
    (out / "guard-inputs").mkdir()
    frozen = frozen_release_prefix(args.frozen_run, args.cutoff_control_step)
    prefix = frozen["macros"]
    prefix_events = frozen["events"]
    source_commands = args.frozen_run / "commands.jsonl"
    (out / "source-manifest.json").write_text(json.dumps(_source_manifest(), indent=2))
    (out / "run-config.json").write_text(json.dumps({
        "seed": frozen["seed"],
        "controller": "scripted_own_rgb_pwm_release_recovery_diagnostic",
        "llm_calls": 0,
        "max_sim_seconds": args.seconds,
        "frozen_prefix_source": str(source_commands),
        "frozen_prefix_sha256": hashlib.sha256(source_commands.read_bytes()).hexdigest(),
        "frozen_prefix_macro_count": len(prefix),
        "frozen_nonmacro_checkpoint_count": len(frozen["state_checkpoints"]),
        "frozen_model_checkpoint_count": len(frozen["model_checkpoints"]),
        "frozen_destination_zone": frozen["destination_zone"],
        "cutoff_control_step": args.cutoff_control_step,
        "cutoff_phase": frozen["cutoff"]["phase"],
        "cutoff_sim_time": frozen["cutoff"]["time"],
        "prefix_markerless_observer": prefix_observer_meta,
        "current_continuation_markerless_observer": current_observer_meta,
        "observer_boundary": (
            "frozen observer only around prefix actor.advance; current observer "
            "restored before every non-prefix operation and continuation"
        ),
        "continuation": "current VisualBoxSkill retract+verify_release+ground sweep",
        "transport_attempted": False,
        "transport_success_claimed": False,
    }, indent=2))

    world = MultiMasterPiProductionV2(
        warehouse_layout="camera_team", seed=frozen["seed"], render=True,
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
    actor = LLMTransportSkill(
        "r1", "small_box_01", frozen["destination_zone"],
    )

    def advance_prefix(observation: Mapping[str, Any]) -> Mapping[str, Any] | None:
        """Advance without changing actor state/actions outside VisualBoxSkill."""
        if prefix_observer is None:
            return actor.advance(observation)
        visual_box_skill_module.observe_ground_box = prefix_observer
        try:
            return actor.advance(observation)
        finally:
            visual_box_skill_module.observe_ground_box = current_observer
    command_log = (out / "commands.jsonl").open("w")
    control_log = (out / "control.jsonl").open("w")
    truth_log = (out / "evaluation-only.jsonl").open("w")
    guard_log = (out / "drive-guard.jsonl").open("w")

    def emit(row: Mapping[str, Any]) -> None:
        command_log.write(json.dumps(row) + "\n")
        command_log.flush()

    guard_index = 0

    def drive_guard(action: Mapping[str, Any], now: float) -> Mapping[str, Any]:
        nonlocal guard_index
        nav = port.capture(camera="nav_cam")
        verdict = validate_visual_drive(nav, action)
        nav_path = f"guard-inputs/{guard_index:04d}-nav.jpg"
        (out / nav_path).write_bytes(base64.b64decode(nav["image"]))
        guard_log.write(json.dumps({
            "step": guard_index, "time": now, "nav_path": nav_path,
            "nav_sha256": nav["sha256"],
            "own_pose_commands": nav["actuator_state"]["servo_pulses"],
            "action": action, "verdict": verdict,
        }) + "\n")
        guard_log.flush()
        guard_index += 1
        return verdict

    executor = VisualMacroExecutor(port, log_callback=emit, drive_guard=drive_guard)
    video = None
    error = None
    prefix_index = 0
    prefix_event_index = 0
    observation_index = 0
    prefix_exact = True
    continuation_started = False
    release_evidence = None
    checkpoints: list[dict[str, Any]] = []
    start = float(world.data.time)
    next_truth = start

    try:
        if args.record:
            from scripts.record_visual_box import SingleBoxVideo
            video = SingleBoxVideo(world, out / "motion-1x.mp4", actor.box)
            video.capture(force=True)

        while float(world.data.time) - start <= args.seconds:
            now = float(world.data.time)
            executor.tick(now)
            observation = None
            action = None
            runner_phase = "prefix" if prefix_event_index < len(prefix_events) else "current_release_verification"

            if executor.idle and prefix_event_index < len(prefix_events):
                expected = prefix_events[prefix_event_index]
                if now + 1e-9 >= expected["time"]:
                    observation = port.capture()
                    before_phase = actor.phase
                    if expected["kind"] == "state_checkpoint":
                        action = advance_prefix(observation)
                        if (action is not None
                                or actor.state != expected["source_skill_state"]
                                or actor.phase != expected["source_phase"]):
                            prefix_exact = False
                            raise RuntimeError(
                                f"FROZEN_STATE_CHECKPOINT_DIVERGED_AT_{prefix_event_index}: "
                                f"expected=({expected['source_skill_state']!r},"
                                f"{expected['source_phase']!r},None) actual="
                                f"({actor.state!r},{actor.phase!r},{action!r})"
                            )
                    elif expected["kind"] == "model_request":
                        action = actor.request(expected["action"])
                        if action is not None:
                            prefix_exact = False
                            raise RuntimeError(
                                f"FROZEN_MODEL_REQUEST_RETURNED_MACRO_AT_{prefix_event_index}: {action!r}"
                            )
                    else:
                        macro = expected["macro"]
                        if actor.state == "idle":
                            actor.request({"kind": "approach"})
                        if actor.state == "carrying" and macro.get("kind") in {"drive", "wait"}:
                            action = actor.request(macro)
                        elif expected["source_phase"] == "release" and actor.state == "carrying":
                            nav = port.capture(camera="nav_cam")
                            release_evidence = inspect_placement(
                                observation, nav, cargo_id=actor.cargo_id,
                                destination_zone=actor.destination_zone,
                                stage="before_release", held_identity_confirmed=actor.held,
                            )
                            actor.request({"kind": "release"}, placement_evidence=release_evidence)
                            action = advance_prefix(observation)
                        else:
                            action = advance_prefix(observation)
                            if actor.state == "ready_to_pick" and actor.pending_hover is not None:
                                if _same_action(macro, actor.pending_hover):
                                    action = actor.request({"kind": "pick"})
                            if action is None and actor.state == "carrying" and macro.get("kind") in {"drive", "wait"}:
                                action = actor.request(macro)
                        if action is None or not _same_action(action, macro):
                            prefix_exact = False
                            raise RuntimeError(
                                f"FROZEN_PREFIX_DIVERGED_AT_{prefix_index}: "
                                f"expected={macro!r} actual={action!r}"
                            )
                        executor.submit(action, observation, expected["source_phase"] or actor.phase, now)
                        prefix_index += 1
                    timing_error = now - expected["time"]
                    checkpoints.append({
                        "event_index": prefix_event_index, "kind": expected["kind"],
                        "source_time": expected["time"], "replay_time": now,
                        "timing_error_s": timing_error, "source_phase": expected.get("source_phase"),
                        "replay_before_phase": before_phase,
                        "source_frame_sha256": expected.get("source_frame_sha256"),
                        "replay_frame_sha256": observation["sha256"],
                        "frame_sha256_match": observation["sha256"] == expected.get("source_frame_sha256"),
                    })
                    if abs(timing_error) > float(world.model.opt.timestep) + 1e-9:
                        prefix_exact = False
                        raise RuntimeError(
                            f"FROZEN_PREFIX_TIMING_DIVERGED_AT_EVENT_{prefix_event_index}: error_s={timing_error}"
                        )
                    prefix_event_index += 1
            elif executor.idle and prefix_event_index == len(prefix_events):
                continuation_started = True
                observation = port.capture()
                action = actor.advance(observation)
                if actor.state in ("released", "failure"):
                    runner_phase = "terminal"
                elif action is not None and action.get("kind") != "finish":
                    executor.submit(action, observation, actor.phase, now)

            if observation is not None:
                wrist_path = f"inputs/{observation_index:04d}-wrist.jpg"
                (out / wrist_path).write_bytes(base64.b64decode(observation["image"]))
                control_log.write(json.dumps({
                    "step": observation_index, "time": now, "runner_phase": runner_phase,
                    "skill_state": actor.state, "skill_phase": actor.phase,
                    "wrist_path": wrist_path, "wrist_sha256": observation["sha256"],
                    "own_pose_commands": observation["actuator_state"]["servo_pulses"],
                    "target": actor.box.last_target,
                    "target_provenance": actor.box.last_target_provenance,
                    "action": action,
                }) + "\n")
                control_log.flush()
                observation_index += 1

            if actor.state in ("released", "failure"):
                break
            if now >= next_truth:
                truth_log.write(json.dumps({
                    "time": now,
                    "relative_box_to_grip": _relative_pose(world, actor.cargo_id),
                }) + "\n")
                truth_log.flush()
                next_truth = now + 0.25

            world._physics_step_for(world.robot("r1"))
            if video and world.data.time >= video.next_frame:
                video.capture()
        else:
            error = "SIM_TIME_LIMIT"
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        # Defensive restoration also covers exceptions outside advance_prefix.
        visual_box_skill_module.observe_ground_box = current_observer
        executor.cancel(float(world.data.time), "release_recovery_probe_complete")
        port.stop()
        control_rows = [
            json.loads(line) for line in (out / "control.jsonl").read_text().splitlines()
            if line.strip()
        ]
        ground_sweep_phases = {row["skill_phase"] for row in control_rows}
        ground_estimates = [list(point) for point in actor.box._release_ground_probe]
        pairwise_xy_spreads = [
            float(np.linalg.norm(np.asarray(right[:2]) - np.asarray(left[:2])))
            for index, left in enumerate(ground_estimates)
            for right in ground_estimates[index + 1:]
        ]
        fresh_ground_sweep = bool(
            len(ground_estimates) == 4
            and all(phase in ground_sweep_phases for phase in (
                "release_ground_left", "release_ground_right", "release_ground_home",
            ))
        )
        terminal_clean = bool(actor.state in ("released", "failure") and error is None)
        diagnostic_success = bool(
            actor.state == "released"
            and actor.box.reason == "VISUAL_RELEASE_CONFIRMED"
            and prefix_exact and prefix_index == len(prefix)
            and prefix_event_index == len(prefix_events)
            and fresh_ground_sweep
            and error is None
        )
        result = {
            "fixture": "camera_team_markerless_release_recovery_diagnostic",
            "probe_scope": "controlled_release_diagnostic_only",
            "controller_inputs": "own RGB + own PWM only",
            "llm_calls": 0,
            "seed": frozen["seed"],
            "prefix_macros_replayed": prefix_index,
            "prefix_macros_expected": len(prefix),
            "prefix_events_replayed": prefix_event_index,
            "prefix_events_expected": len(prefix_events),
            "prefix_exact": bool(prefix_exact and prefix_index == len(prefix)
                                 and prefix_event_index == len(prefix_events)),
            "prefix_claim": "macro payloads and submission simulation times only",
            "prefix_markerless_observer": prefix_observer_meta,
            "current_continuation_markerless_observer": current_observer_meta,
            "observer_restored_for_continuation": bool(
                visual_box_skill_module.observe_ground_box is current_observer
            ),
            "prefix_checkpoints": checkpoints,
            "prefix_max_abs_timing_error_s": max(
                (abs(row["timing_error_s"]) for row in checkpoints), default=None,
            ),
            "release_evidence": release_evidence,
            "continuation_started": continuation_started,
            "release_ground_sweep": {
                "observations_base_m": ground_estimates,
                "observation_count": len(ground_estimates),
                "maximum_pairwise_xy_spread_m": max(pairwise_xy_spreads, default=None),
                "threshold_m": 0.010,
                "phases_observed": sorted(
                    phase for phase in ground_sweep_phases if phase.startswith("release_ground_")
                ),
                "fresh_full_sweep_observed": fresh_ground_sweep,
            },
            "skill_state": actor.state,
            "skill_reason": actor.reason,
            "visual_box_reason": actor.box.reason,
            "transport_attempted": False,
            "transport_success_claimed": False,
            "diagnostic_success": diagnostic_success,
            "diagnostic_completed": terminal_clean,
            "diagnostic_failure_cleanly_observed": bool(
                terminal_clean and actor.state == "failure" and not diagnostic_success
            ),
            "sim_seconds": float(world.data.time) - start,
            "error": error,
            "evaluation_only": {
                "final_relative_box_to_grip": _relative_pose(world, actor.cargo_id),
                "final_warehouse_state": world.warehouse_state(),
            },
        }
        (out / "result.json").write_text(json.dumps(result, indent=2))
        if video:
            video.capture(force=True)
            video.close()
        command_log.close()
        control_log.close()
        truth_log.close()
        guard_log.close()
        world.close()

    print(json.dumps({
        key: value for key, value in result.items()
        if key not in {"evaluation_only", "prefix_checkpoints", "release_evidence"}
    }))
    return 0 if result["diagnostic_completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
