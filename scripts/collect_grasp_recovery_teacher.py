#!/usr/bin/env python3
"""Collect privileged expert recovery demonstrations from local RGB perturbations.

Each case starts in a fresh plain-beam world.  Coordinates and saved goal
commands are teacher-only.  Actor rows contain only one robot's RGB, the fixed
top RGB, and that robot's issued-command snapshot.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback
from typing import Any, Mapping
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

ROBOT_IDS = ("r1", "r3")
CHANNELS = (3, 4, 5)
MAX_PERTURB = 150
MAX_EXPERT_STEP = 25
MAX_CORRECTION_STEPS = 8


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def _pulses(value: Mapping[Any, Any]) -> dict[int, int]:
    return {int(k): int(v) for k, v in value.items()}


def _json_pulses(value: Mapping[int, int]) -> dict[str, int]:
    return {str(k): int(v) for k, v in sorted(value.items())}


def load_cases(path: Path) -> list[dict[str, Any]]:
    raw = json.loads(path.read_text())
    if not isinstance(raw, dict) or not isinstance(raw.get("cases"), list) or not raw["cases"]:
        raise ValueError("cases JSON must contain a non-empty cases list")
    seen: set[str] = set()
    result = []
    for item in raw["cases"]:
        if not isinstance(item, dict) or not isinstance(item.get("id"), str) or not item["id"]:
            raise ValueError("every case requires a non-empty string id")
        case_id = item["id"]
        if case_id in seen or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for ch in case_id):
            raise ValueError(f"case id must be unique and path-safe: {case_id!r}")
        perturb = item.get("perturb")
        if not isinstance(perturb, dict) or set(perturb) != set(ROBOT_IDS):
            raise ValueError(f"{case_id}: perturb must contain exactly r1 and r3")
        normalized: dict[str, list[int]] = {}
        for rid in ROBOT_IDS:
            values = perturb[rid]
            if (not isinstance(values, list) or len(values) != 3
                    or any(isinstance(v, bool) or not isinstance(v, int) for v in values)
                    or max(map(abs, values)) > MAX_PERTURB):
                raise ValueError(f"{case_id}/{rid}: expected three integer perturbations within +/-{MAX_PERTURB}")
            normalized[rid] = list(values)
        seen.add(case_id)
        result.append({"id": case_id, "perturb": normalized})
    return result


def bounded_expert_action(remaining: Mapping[str, Mapping[int, int]]) -> dict[str, dict[int, int]]:
    return {
        rid: {
            int(channel): max(-MAX_EXPERT_STEP, min(MAX_EXPERT_STEP, int(delta)))
            for channel, delta in values.items()
        }
        for rid, values in remaining.items()
    }


class Records:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.rgb_dir = out_dir / "rgb"
        self.rgb_dir.mkdir()
        self.video_dir = out_dir / "videos"
        self.video_dir.mkdir()
        self.actor: list[dict[str, Any]] = []
        self.labels: list[dict[str, Any]] = []
        self.commands: list[dict[str, Any]] = []

    def jpeg(self, relative: str, content: bytes) -> dict[str, Any]:
        if not content.startswith(b"\xff\xd8") or not content.endswith(b"\xff\xd9"):
            raise RuntimeError(f"invalid JPEG: {relative}")
        path = self.out_dir / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(content)
        return {"path": relative, "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}

    def observe(
        self, world: Any, *, case_id: str, observation_id: str, phase: str,
        commands: Mapping[str, Mapping[int, int]], full_remaining: Mapping[str, Mapping[int, int]],
        next_action: Mapping[str, Mapping[int, int]],
    ) -> None:
        stem = f"{case_id}/{observation_id}"
        top = self.jpeg(f"rgb/{stem}__shared_top.jpg", world.render_team_jpeg(camera="cctv_top", quality=95))
        for rid in ROBOT_IDS:
            own = self.jpeg(
                f"rgb/{stem}__{rid}_own.jpg",
                world.render_jpeg(robot_id=rid, camera="robot_cam", quality=95),
            )
            sample_id = f"{case_id}:{observation_id}:{rid}"
            self.actor.append({
                "sample_id": sample_id, "case_id": case_id, "robot_id": rid, "phase": phase,
                "observations": {"own_rgb": own, "shared_top_rgb": top},
                "own_issued_command_snapshot": _json_pulses(commands[rid]),
            })
            self.labels.append({
                "sample_id": sample_id, "case_id": case_id, "robot_id": rid,
                "channels": list(CHANNELS),
                "correction_pulses": [int(full_remaining[rid][channel]) for channel in CHANNELS],
                "bounded_next_pulses": [int(next_action[rid][channel]) for channel in CHANNELS],
                "teacher_basis": "saved issued-command goal after trusted initialization replay",
            })

    def move(
        self, world: Any, commands: dict[str, dict[int, int]], targets: Mapping[str, Mapping[int, int]],
        duration_s: float, settle_s: float, *, case_id: str, phase: str,
    ) -> None:
        normalized = {rid: _pulses(values) for rid, values in targets.items()}
        before = {rid: _json_pulses(commands[rid]) for rid in normalized}
        world._team_joint_move_servos(normalized, duration_s, settle_s=settle_s)
        for rid, values in normalized.items():
            commands[rid].update(values)
        self.commands.append({
            "sequence": len(self.commands), "case_id": case_id, "phase": phase,
            "targets": {rid: _json_pulses(values) for rid, values in normalized.items()},
            "issued_before": before,
            "issued_after": {rid: _json_pulses(commands[rid]) for rid in normalized},
            "duration_s": float(duration_s), "settle_s": float(settle_s),
        })


def _configure_world(world: Any, fixture: Mapping[str, Any]) -> None:
    import mujoco
    from scripts.probe_dual_grasp_sync import _camera_look_at

    for rid, pose in fixture["base_poses"].items():
        world.controllers[rid].set_base_pose_for_test(tuple(float(v) for v in pose), 0.0)
    camera = fixture["top_camera"]
    cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_top")
    if cid < 0:
        raise RuntimeError("cctv_top missing")
    world.model.cam_pos[cid] = camera["position_m"]
    world.model.cam_quat[cid] = camera["quaternion_wxyz"]
    world.model.cam_fovy[cid] = camera["fov_y_deg"]
    mujoco.mj_forward(world.model, world.data)
    _camera_look_at(world)  # presentation-only cctv_warehouse


def run_case(
    case: Mapping[str, Any], skill: Mapping[str, Any], fixture: Mapping[str, Any], records: Records,
) -> dict[str, Any]:
    case_id = str(case["id"])
    result: dict[str, Any] = {"id": case_id, "ok": False, "excluded_from_training": True}
    world = None
    video = None
    commands = {rid: {} for rid in ROBOT_IDS}
    started = time.monotonic()
    try:
        import mujoco
        import sim.multi_masterpi_production as production
        from scripts.probe_dual_grasp_sync import (
            Video, _plain_beam_contact, _plain_beam_xml, _pose_metrics,
        )
        from scripts.run_camera_pair_transport import evaluate_grasp_samples

        result["environment"] = {
            "python": sys.version, "platform": platform.platform(),
            "mujoco": getattr(mujoco, "__version__", "unknown"),
        }
        original = production.build_multi_robot_xml
        with patch.object(production, "build_multi_robot_xml", _plain_beam_xml(original)):
            world = production.MultiMasterPiProductionV2(
                seed=int(fixture["seed"]), width=960, height=720, render=True,
            )
        _configure_world(world, fixture)
        video = Video(world, records.video_dir / f"{case_id}.mp4", 4)
        video.stage = f"recovery teacher {case_id}"
        world.frame_callback = video.capture
        for command in skill["initialization_replay"]:
            records.move(
                world, commands,
                {rid: _pulses(values) for rid, values in command["targets"].items()},
                float(command["duration_s"]), float(command.get("settle_s", 0.0)),
                case_id=case_id, phase=f"initialization:{command['phase']}",
            )
        goal = {rid: dict(commands[rid]) for rid in ROBOT_IDS}
        disturbed = {
            rid: {
                channel: max(500, min(2500, goal[rid][channel] + int(case["perturb"][rid][index])))
                for index, channel in enumerate(CHANNELS)
            }
            for rid in ROBOT_IDS
        }
        records.move(world, commands, disturbed, .35, .10, case_id=case_id, phase="disturb")
        result["requested_perturbation"] = case["perturb"]
        result["applied_perturbation"] = {
            rid: [commands[rid][channel] - goal[rid][channel] for channel in CHANNELS]
            for rid in ROBOT_IDS
        }

        correction_steps = 0
        for step in range(MAX_CORRECTION_STEPS):
            remaining = {
                rid: {channel: goal[rid][channel] - commands[rid][channel] for channel in CHANNELS}
                for rid in ROBOT_IDS
            }
            if all(delta == 0 for values in remaining.values() for delta in values.values()):
                break
            action = bounded_expert_action(remaining)
            records.observe(
                world, case_id=case_id, observation_id=f"correction-{step:02d}-before",
                phase="recovery_before_action", commands=commands,
                full_remaining=remaining, next_action=action,
            )
            targets = {
                rid: {channel: commands[rid][channel] + action[rid][channel] for channel in CHANNELS}
                for rid in ROBOT_IDS
            }
            records.move(world, commands, targets, .35, .10, case_id=case_id, phase="recovery")
            correction_steps += 1
        remaining = {
            rid: {channel: goal[rid][channel] - commands[rid][channel] for channel in CHANNELS}
            for rid in ROBOT_IDS
        }
        if any(delta != 0 for values in remaining.values() for delta in values.values()):
            raise RuntimeError(f"teacher did not restore goal within {MAX_CORRECTION_STEPS} steps: {remaining}")
        zero = {rid: {channel: 0 for channel in CHANNELS} for rid in ROBOT_IDS}
        records.observe(
            world, case_id=case_id, observation_id="goal-anchor", phase="recovery_goal_anchor",
            commands=commands, full_remaining=zero, next_action=zero,
        )

        close = {rid: {1: int(skill["close_pulses"][rid])} for rid in ROBOT_IDS}
        records.move(
            world, commands, close, float(skill["close_duration_s"]), float(skill["close_settle_s"]),
            case_id=case_id, phase="demonstrated_close",
        )
        close_contacts = {rid: _plain_beam_contact(world, rid) for rid in ROBOT_IDS}
        result["close_contacts"] = close_contacts
        if not all(bool(value.get("bilateral")) for value in close_contacts.values()):
            raise RuntimeError(f"bilateral close failed: {close_contacts}")
        lift = {
            rid: {
                int(channel): max(500, min(2500, commands[rid][int(channel)] + int(delta)))
                for channel, delta in skill["lift_delta_pulses"][rid].items()
            }
            for rid in ROBOT_IDS
        }
        records.move(
            world, commands, lift, float(skill["lift_duration_s"]), float(skill["lift_settle_s"]),
            case_id=case_id, phase="demonstrated_lift",
        )

        samples = []
        result["hold_evaluation_samples"] = samples
        duration = float(skill.get("hold_s", 2.2))
        next_sample = float(world.data.time)
        end = next_sample + duration
        dt = float(world.model.opt.timestep)
        while float(world.data.time) + 1e-9 < end:
            world._physics_step_for(world.controllers["r1"])
            video.capture()
            now = float(world.data.time)
            if now + 1e-9 >= next_sample:
                samples.append({
                    **_pose_metrics(world),
                    "contacts": {rid: _plain_beam_contact(world, rid) for rid in ROBOT_IDS},
                })
                next_sample += 0.1
        evaluation = evaluate_grasp_samples(samples)
        result["evaluation"] = evaluation
        if not evaluation["grasp_success"]:
            raise RuntimeError(f"physical recovery verification failed: {evaluation}")
        result.update({
            "ok": True, "excluded_from_training": False, "correction_steps": correction_steps,
        })
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["traceback"] = traceback.format_exc()
    finally:
        if world is not None:
            world.frame_callback = None
        if video is not None:
            try:
                video.close()
                video_path = records.video_dir / f"{case_id}.mp4"
                result["video"] = {
                    "path": str(video_path.relative_to(records.out_dir)),
                    "sha256": _sha256(video_path), "bytes": video_path.stat().st_size,
                }
            except Exception as exc:
                result["video_error"] = f"{type(exc).__name__}: {exc}"
                result["ok"] = False
                result["excluded_from_training"] = True
        if world is not None:
            try:
                world.close()
            except Exception as exc:
                result["cleanup_error"] = f"{type(exc).__name__}: {exc}"
                result["ok"] = False
                result["excluded_from_training"] = True
        result["wall_elapsed_s"] = round(time.monotonic() - started, 6)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--cases-json", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    model_dir = args.model_dir.expanduser().resolve()
    cases_path = args.cases_json.expanduser().resolve()
    out_dir = args.out_dir.expanduser().resolve()
    if out_dir.exists():
        parser.error(f"--out-dir must be new: {out_dir}")
    skill_path = model_dir / "student-skill.json"
    fixture_path = model_dir / "evaluation-fixture.json"
    skill = json.loads(skill_path.read_text())
    fixture = json.loads(fixture_path.read_text())
    if skill.get("schema") != "ugrp.local_rgb_grasp_skill.v1":
        raise ValueError("unsupported student skill")
    if fixture.get("schema") != "ugrp.local_grasp_evaluation_fixture.v1" or fixture.get("actor_access") is not False:
        raise ValueError("invalid privileged evaluation fixture")
    cases = load_cases(cases_path)
    out_dir.mkdir(parents=True)
    records = Records(out_dir)
    source_sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    case_results = []
    for index, case in enumerate(cases, start=1):
        result = run_case(case, skill, fixture, records)
        case_results.append(result)
        _write(out_dir / "actor_samples.json", records.actor)
        _write(out_dir / "privileged_labels.json", records.labels)
        _write(out_dir / "teacher_commands.json", records.commands)
        _write(out_dir / "report.json", {
            "ok": False, "complete": False, "source_sha": source_sha, "cases": case_results,
            "successful_cases": sum(bool(item["ok"]) for item in case_results),
            "failed_cases": sum(not bool(item["ok"]) for item in case_results),
        })
        print(json.dumps({
            "case": result["id"], "ok": result["ok"], "completed": index,
            "total": len(cases), "wall_elapsed_s": result["wall_elapsed_s"],
        }), flush=True)
    manifest = {
        "schema": "ugrp.grasp_recovery_teacher.v1", "source_sha": source_sha,
        "environment": {
            "python": sys.version, "platform": platform.platform(),
            "mujoco": next((item.get("environment", {}).get("mujoco") for item in case_results if item.get("environment")), "unavailable"),
        },
        "config": {
            "fresh_world_per_case": True, "plain_beam": True, "weld": False,
            "channels": list(CHANNELS), "max_perturb_pwm": MAX_PERTURB,
            "max_expert_step_pwm": MAX_EXPERT_STEP, "max_correction_steps": MAX_CORRECTION_STEPS,
            "actor_contract": ["own_rgb", "fixed_shared_top_rgb", "own_issued_command_snapshot"],
        },
        "inputs": {
            "student_skill": {"path": str(skill_path), "sha256": _sha256(skill_path)},
            "evaluation_fixture": {"path": str(fixture_path), "sha256": _sha256(fixture_path)},
            "cases": {"path": str(cases_path), "sha256": _sha256(cases_path)},
        },
        "files": {
            "actor_samples": "actor_samples.json", "privileged_labels": "privileged_labels.json",
            "teacher_commands": "teacher_commands.json", "report": "report.json", "rgb_root": "rgb/",
        },
        "cases": [
            {key: value for key, value in result.items() if key != "traceback"}
            for result in case_results
        ],
        "counts": {"cases": len(cases), "actor_samples": len(records.actor), "privileged_labels": len(records.labels)},
    }
    _write(out_dir / "manifest.json", manifest)
    report = {
        "ok": all(result["ok"] for result in case_results),
        "complete": True,
        "source_sha": source_sha,
        "all_cases_ok": all(result["ok"] for result in case_results),
        "successful_cases": sum(bool(result["ok"]) for result in case_results),
        "failed_cases": sum(not bool(result["ok"]) for result in case_results),
        "cases": case_results,
    }
    _write(out_dir / "report.json", report)
    print(json.dumps({"out_dir": str(out_dir), **{k: report[k] for k in ("all_cases_ok", "successful_cases", "failed_cases")}}))
    return 0 if report["all_cases_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
