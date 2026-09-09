"""Bounded no-LLM diagnostic for the seed-46 loaded-grip ambiguity.

The runner replays the frozen M4 manipulation prefix through the unchanged
VisualBoxSkill/LLMTransportSkill, applies exactly one scripted loaded-motion
case, then runs the existing strict own-RGB/PWM attachment probe.  It is a
diagnostic and never claims navigation or transport success.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

import mujoco
import numpy as np

from harness.llm_transport_skill import LLMTransportSkill
from harness.visual_drive_guard import validate_visual_drive
from harness.visual_macro_runtime import VisualMacroExecutor
from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2


FROZEN_RUN = Path("outputs/markerless-blocks-20260909/m4/solo-46")
CASES = {
    "stop": {"kind": "wait", "duration": 0.6},
    "straight": {"kind": "drive", "fwd": 0.13, "turn": 0.0, "duration": 0.6},
    "turn": {"kind": "drive", "fwd": 0.0, "turn": 0.04, "duration": 0.6},
    "originalcombined": {"kind": "drive", "fwd": 0.13, "turn": 0.04, "duration": 2.5},
}


def _jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def _frozen_plan(run: Path) -> dict[str, Any]:
    """Extract the original prefix phases, first loaded drive, and monitors."""
    rows = _jsonl(run / "commands.jsonl")
    control = _jsonl(run / "control.jsonl")
    monitors = _jsonl(run / "visual-guard.jsonl")
    macros = [row for row in rows if row.get("event") == "macro_submitted"]
    end = next(
        index for index, row in enumerate(macros)
        if row.get("macro") == CASES["originalcombined"]
    )
    first_drive = macros[end]
    control_by_time = {float(row["time"]): row for row in control}
    prefix = []
    for row in macros[:end]:
        source_time = float(row["time"])
        control_row = control_by_time.get(source_time)
        prefix.append({
            "time": source_time,
            "macro": row["macro"],
            "source_frame_sha256": row.get("source_frame_sha256"),
            # LLM-returned macros have no same-time control row and use the
            # actor phase after request(), exactly as the original runner did.
            "before_phase": control_row.get("phase") if control_row else None,
        })
    monitor_times = [float(row["time"]) for row in monitors]
    if not monitor_times or monitor_times[0] != 54.544000000020695:
        raise ValueError("unexpected frozen carry-monitor origin")
    return {
        "prefix": prefix,
        "first_loaded_drive_time": float(first_drive["time"]),
        "first_loaded_drive": first_drive["macro"],
        "monitor_times": monitor_times,
    }


def _same_action(actual: Mapping[str, Any], expected: Mapping[str, Any]) -> bool:
    return json.dumps(actual, sort_keys=True, separators=(",", ":")) == json.dumps(
        expected, sort_keys=True, separators=(",", ":")
    )


def _relative_pose(world: Any, cargo_id: str, robot_id: str = "r1") -> dict[str, Any]:
    """Evaluation-only cargo pose in the gripper-body frame."""
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
    contact = robot.finger_cargo_contact(cargo_id)
    return {
        "translation_m": relative_pos.tolist(),
        "quaternion_wxyz": relative_quat.tolist(),
        "orientation_angle_deg": math.degrees(angle),
        "contact": contact,
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
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--case", choices=tuple(CASES), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--probe-settle", type=float, default=0.5)
    args = parser.parse_args()

    if args.seed < 0:
        parser.error("seed must be nonnegative")
    if not 0.0 <= args.probe_settle <= 5.0:
        parser.error("probe-settle must be between 0 and 5 sim seconds")
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    (out / "inputs").mkdir()
    (out / "guard-inputs").mkdir()
    frozen = _frozen_plan(FROZEN_RUN)
    prefix = frozen["prefix"]
    (out / "source-manifest.json").write_text(json.dumps(_source_manifest(), indent=2))
    (out / "run-config.json").write_text(json.dumps({
        "seed": args.seed,
        "case": args.case,
        "stimulus": CASES[args.case],
        "controller": "scripted_own_rgb_pwm_loaded_grip_diagnostic",
        "llm_calls": 0,
        "max_sim_seconds": 180.0,
        "probe_settle_s": args.probe_settle,
        "frozen_prefix_source": str(FROZEN_RUN / "commands.jsonl"),
        "frozen_prefix_sha256": hashlib.sha256(
            (FROZEN_RUN / "commands.jsonl").read_bytes()
        ).hexdigest(),
        "frozen_prefix_macro_count": len(prefix),
        "transport_attempted": False,
        "transport_success_claimed": False,
    }, indent=2))

    world = MultiMasterPiProductionV2(
        warehouse_layout="camera_team", seed=args.seed, render=True,
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
    actor = LLMTransportSkill("r1", "small_box_01", "B")
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
    observation_index = 0
    phase = "prefix"
    stimulus_submitted = False
    settle_submitted = False
    probe_started = False
    probe_finished = False
    probe_passed = False
    prefix_exact = True
    prefix_checkpoints: list[dict[str, Any]] = []
    stimulus_execution = None
    baseline_truth = None
    final_truth = None
    start = float(world.data.time)
    next_truth = start
    monitor_index = 0
    next_monitor = frozen["monitor_times"][0]
    stop_after_log = False

    try:
        if args.record:
            from scripts.record_visual_box import SingleBoxVideo
            video = SingleBoxVideo(world, out / "motion-1x.mp4", actor.box)
            video.capture(force=True)

        while float(world.data.time) - start <= 180.0:
            now = float(world.data.time)
            executor.tick(now)
            action = None
            observation = None

            if phase == "prefix" and executor.idle and prefix_index < len(prefix):
                expected = prefix[prefix_index]
                if now + 1e-9 >= expected["time"]:
                    observation = port.capture()
                    before_phase = actor.phase
                    if actor.state == "idle":
                        actor.request({"kind": "approach"})
                        before_phase = actor.phase
                    action = actor.advance(observation)
                    if actor.state == "ready_to_pick" and actor.pending_hover is not None and _same_action(expected["macro"], actor.pending_hover):
                        action = actor.request({"kind": "pick"})
                    if action is None or not _same_action(action, expected["macro"]):
                        prefix_exact = False
                        raise RuntimeError(
                            f"FROZEN_PREFIX_DIVERGED_AT_{prefix_index}: "
                            f"expected={expected['macro']!r} actual={action!r}"
                        )
                    expected_phase = expected["before_phase"]
                    submit_phase = expected_phase if expected_phase is not None else actor.phase
                    if expected_phase is not None and before_phase != expected_phase:
                        prefix_exact = False
                        raise RuntimeError(
                            f"FROZEN_PREFIX_PHASE_DIVERGED_AT_{prefix_index}: "
                            f"expected={expected_phase!r} actual={before_phase!r}"
                        )
                    timing_error = now - expected["time"]
                    prefix_checkpoints.append({
                        "index": prefix_index,
                        "source_time": expected["time"],
                        "replay_time": now,
                        "timing_error_s": timing_error,
                        "action_exact": True,
                        "source_frame_sha256": expected["source_frame_sha256"],
                        "replay_frame_sha256": observation["sha256"],
                        "frame_sha256_match": (
                            observation["sha256"] == expected["source_frame_sha256"]
                        ),
                        "submit_phase": submit_phase,
                        "source_before_phase": expected_phase,
                    })
                    # Shared physics ticks are 2 ms, so a replay later than one
                    # tick is not described as exact even if the macro matches.
                    if abs(timing_error) > float(world.model.opt.timestep) + 1e-9:
                        prefix_exact = False
                        raise RuntimeError(
                            f"FROZEN_PREFIX_TIMING_DIVERGED_AT_{prefix_index}: "
                            f"error_s={timing_error}"
                        )
                    executor.submit(action, observation, submit_phase, now)
                    prefix_index += 1
            elif phase == "prefix" and executor.idle and prefix_index == len(prefix):
                observation = port.capture()
                action = actor.advance(observation)
                if action is not None or actor.state != "carrying":
                    raise RuntimeError(
                        f"PREFIX_DID_NOT_REACH_CONFIRMED_CARRY: state={actor.state} action={action!r}"
                    )
                baseline_truth = _relative_pose(world, actor.cargo_id)
                phase = "pre_stimulus_monitors"
            elif (phase == "pre_stimulus_monitors" and executor.idle
                  and now + 1e-9 >= frozen["first_loaded_drive_time"]):
                observation = port.capture()
                action = actor.request(CASES[args.case])
                executor.submit(action, observation, "loaded_grip_stimulus", now)
                stimulus_submitted = True
                phase = "stimulus"
            elif phase == "stimulus" and executor.idle and stimulus_submitted:
                stimulus_execution = executor.last_execution
                phase = "settle"
            elif phase == "settle" and executor.idle and not settle_submitted:
                observation = port.capture()
                action = actor.request({"kind": "wait", "duration": args.probe_settle})
                executor.submit(action, observation, "loaded_grip_settle", now)
                settle_submitted = True
            elif phase == "settle" and executor.idle and settle_submitted:
                # A check_grip is an explicit diagnostic intervention here;
                # it does not weaken or bypass the existing attachment gate.
                actor.state = "grip_uncertain"
                actor.request({"kind": "check_grip"})
                probe_started = True
                phase = "probe"
            elif phase == "probe" and executor.idle:
                observation = port.capture()
                action = actor.advance(observation)
                if actor.state == "carrying":
                    probe_finished = True
                    probe_passed = True
                    final_truth = _relative_pose(world, actor.cargo_id)
                    stop_after_log = True
                if actor.state == "failure":
                    probe_finished = True
                    final_truth = _relative_pose(world, actor.cargo_id)
                    stop_after_log = True
                if action and not stop_after_log:
                    if action.get("kind") == "finish":
                        probe_finished = True
                        final_truth = _relative_pose(world, actor.cargo_id)
                        stop_after_log = True
                    else:
                        executor.submit(action, observation, actor.phase, now)

            # Preserve the source carry-anchor update schedule before the
            # stimulus and continue the same 0.5 s cadence during motion.
            if (phase in {"pre_stimulus_monitors", "stimulus"}
                    and actor.state == "carrying" and now + 1e-9 >= next_monitor):
                monitor = port.capture()
                monitor_action = actor.advance(monitor)
                monitor_path = f"inputs/monitor-{monitor['frame_id']:06d}.jpg"
                (out / monitor_path).write_bytes(base64.b64decode(monitor["image"]))
                control_log.write(json.dumps({
                    "event": "carry_monitor", "time": now,
                    "source_monitor_time": (
                        frozen["monitor_times"][monitor_index]
                        if monitor_index < len(frozen["monitor_times"]) else None
                    ),
                    "wrist_path": monitor_path, "wrist_sha256": monitor["sha256"],
                    "own_pose_commands": monitor["actuator_state"]["servo_pulses"],
                    "skill_state": actor.state, "attachment": actor.box.last_attachment,
                }) + "\n")
                control_log.flush()
                monitor_index += 1
                next_monitor = (
                    frozen["monitor_times"][monitor_index]
                    if monitor_index < len(frozen["monitor_times"])
                    else now + 0.5
                )
                if actor.state == "grip_uncertain":
                    executor.cancel(now, "carry_monitor_grip_uncertain")
                    stimulus_execution = executor.last_execution
                    phase = "settle"
                elif monitor_action is not None:
                    raise RuntimeError(f"UNEXPECTED_CARRY_MONITOR_ACTION:{monitor_action!r}")

            if observation is not None:
                wrist_path = f"inputs/{observation_index:04d}-wrist.jpg"
                (out / wrist_path).write_bytes(base64.b64decode(observation["image"]))
                control_log.write(json.dumps({
                    "step": observation_index, "time": now, "runner_phase": phase,
                    "skill_state": actor.state, "skill_phase": actor.phase,
                    "wrist_path": wrist_path, "wrist_sha256": observation["sha256"],
                    "own_pose_commands": observation["actuator_state"]["servo_pulses"],
                    "action": action, "attachment": actor.box.last_attachment,
                }) + "\n")
                control_log.flush()
                observation_index += 1

            if stop_after_log:
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
        executor.cancel(float(world.data.time), "loaded_grip_probe_complete")
        port.stop()
        if final_truth is None:
            final_truth = _relative_pose(world, actor.cargo_id)
        result = {
            "fixture": "camera_team_markerless_loaded_grip_diagnostic",
            "probe_scope": "scripted_diagnostic_only",
            "controller_inputs": "own RGB + own PWM only",
            "llm_calls": 0,
            "seed": args.seed,
            "case": args.case,
            "stimulus": CASES[args.case],
            "prefix_macros_replayed": prefix_index,
            "prefix_macros_expected": len(prefix),
            "prefix_exact": prefix_exact and prefix_index == len(prefix),
            "prefix_claim": "macro payloads and submission times only",
            "initial_strict_attachment_confirmed": baseline_truth is not None,
            "prefix_checkpoints": prefix_checkpoints,
            "prefix_max_abs_timing_error_s": max(
                (abs(row["timing_error_s"]) for row in prefix_checkpoints), default=None,
            ),
            "stimulus_submitted": stimulus_submitted,
            "frozen_first_loaded_drive_time": frozen["first_loaded_drive_time"],
            "carry_monitors_replayed": monitor_index,
            "stimulus_execution": stimulus_execution,
            "probe_started": probe_started,
            "probe_finished": probe_finished,
            "strict_attachment_probe_passed": probe_passed,
            "skill_state": actor.state,
            "skill_reason": actor.reason,
            "attachment": actor.box.last_attachment,
            "transport_attempted": False,
            "transport_success_claimed": False,
            "diagnostic_completed": bool(probe_finished and error is None),
            "sim_seconds": float(world.data.time) - start,
            "error": error,
            "evaluation_only": {
                "baseline_relative_box_to_grip": baseline_truth,
                "final_relative_box_to_grip": final_truth,
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

    print(json.dumps({key: value for key, value in result.items() if key not in {"evaluation_only", "prefix_checkpoints", "attachment"}}))
    return 0 if result["diagnostic_completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
