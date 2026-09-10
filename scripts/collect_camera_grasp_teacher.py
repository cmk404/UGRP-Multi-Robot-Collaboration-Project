#!/usr/bin/env python3
"""Collect a bounded privileged-teacher dataset for local dual side grasping.

The teacher may read fixture coordinates, solve IK, and inspect contacts.  Those
signals are written to ``privileged_labels.json`` and are never included in the
actor records.  Each actor record contains only that robot's wrist RGB, the
fixed shared top RGB, and a snapshot of commands issued by that robot.

This collector is deliberately local to the pre-grasp fixture.  It performs no
navigation, changes no collision geometry, activates no weld, and writes body
poses only while initializing the fixture.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import traceback
from typing import Any, Mapping, TYPE_CHECKING
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.cooperative_payload import (  # noqa: E402
    BEAM_CARRIER_IDS,
    BEAM_ENDPOINT_OFFSETS_M,
    BEAM_START,
    beam_pose,
)
if TYPE_CHECKING:
    from sim.multi_masterpi_production import MultiMasterPiProductionV2

TEACHER_STAGES = ("hover", "preclose", "closed", "lifted", "held")
PROBE_SERVOS = (3, 4, 5)
PROBE_DELTAS = (-50, -25, 25, 50)
TOP_CAMERA_POSITION = (0.55, -2.0, 2.5)
TOP_CAMERA_QUATERNION = (1.0, 0.0, 0.0, 0.0)
TOP_CAMERA_FOV_DEG = 55.0


def _git(args: list[str]) -> str:
    result = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, capture_output=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else f"unavailable: {result.stderr.strip()}"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _file_sha256(path: Path) -> str:
    return _sha256(path.read_bytes())


def _json_write(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _string_pulses(pulses: Mapping[int, int]) -> dict[str, int]:
    return {str(k): int(v) for k, v in sorted(pulses.items())}


def _set_fixed_top_camera(world: MultiMasterPiProductionV2) -> dict[str, Any]:
    import mujoco
    camera_id = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_top",
    )
    if camera_id < 0:
        raise RuntimeError("cctv_top camera missing")
    world.model.cam_pos[camera_id] = TOP_CAMERA_POSITION
    world.model.cam_quat[camera_id] = TOP_CAMERA_QUATERNION
    world.model.cam_fovy[camera_id] = TOP_CAMERA_FOV_DEG
    mujoco.mj_forward(world.model, world.data)
    return {
        "name": "cctv_top",
        "position_m": [float(v) for v in world.model.cam_pos[camera_id]],
        "quaternion_wxyz": [float(v) for v in world.model.cam_quat[camera_id]],
        "fov_y_deg": float(world.model.cam_fovy[camera_id]),
    }


class DatasetWriter:
    """Write image blobs once and keep actor and privileged records separate."""

    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.images_dir = out_dir / "rgb"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.actor_samples: list[dict[str, Any]] = []
        self.privileged_labels: list[dict[str, Any]] = []
        self.image_index: dict[str, dict[str, Any]] = {}

    def _save_jpeg(self, relative: Path, jpeg: bytes) -> dict[str, Any]:
        if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            raise RuntimeError(f"renderer returned invalid JPEG for {relative}")
        path = self.out_dir / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(jpeg)
        record = {
            "path": relative.as_posix(),
            "sha256": _sha256(jpeg),
            "bytes": len(jpeg),
        }
        self.image_index[relative.as_posix()] = record
        return record

    def capture(
        self,
        world: MultiMasterPiProductionV2,
        *,
        sample_id: str,
        phase: str,
        teacher_action: Mapping[str, Any],
        probe_delta: Mapping[str, Any] | None = None,
        physics_evaluation: Mapping[str, Any] | None = None,
    ) -> None:
        top = self._save_jpeg(
            Path("rgb") / f"{sample_id}__shared_top.jpg",
            world.render_team_jpeg(camera="cctv_top", quality=95),
        )
        for rid in BEAM_CARRIER_IDS:
            own = self._save_jpeg(
                Path("rgb") / f"{sample_id}__{rid}_own.jpg",
                world.render_jpeg(robot_id=rid, camera="robot_cam", quality=95),
            )
            issued = _string_pulses(world.controllers[rid].servo_command_pulses)
            actor = {
                "sample_id": f"{sample_id}:{rid}",
                "robot_id": rid,
                "phase": phase,
                "observations": {"own_rgb": own, "shared_top_rgb": top},
                "own_issued_command_snapshot": issued,
            }
            self.actor_samples.append(actor)
            self.privileged_labels.append({
                "sample_id": actor["sample_id"],
                "teacher_action": dict(teacher_action),
                "probe_delta_label": dict(probe_delta or {}),
                "physics_evaluation": dict(physics_evaluation or {}),
            })

    def write(self) -> None:
        _json_write(self.out_dir / "actor_samples.json", self.actor_samples)
        _json_write(self.out_dir / "privileged_labels.json", self.privileged_labels)


def _physics_snapshot(world: MultiMasterPiProductionV2) -> dict[str, Any]:
    pose = beam_pose(world.data, world.model)
    return {
        "sim_time_s": round(float(world.data.time), 6),
        "beam_position_m": [round(float(v), 6) for v in pose["position"]],
        "height_above_start_m": round(float(pose["position"][2]) - BEAM_START[2], 6),
        "contacts": {
            rid: world.controllers[rid].finger_payload_contact(rid)
            for rid in BEAM_CARRIER_IDS
        },
        "weld_active": {
            rid: bool(world._beam_constraint_active(rid)) for rid in BEAM_CARRIER_IDS
        },
    }


def _move(
    world: MultiMasterPiProductionV2,
    issued: dict[str, dict[int, int]],
    command_trace: list[dict[str, Any]],
    poses: Mapping[str, Mapping[int, int]],
    duration_s: float,
    *,
    settle_s: float = 0.0,
    phase: str,
) -> None:
    normalized = {
        rid: {int(servo): int(pulse) for servo, pulse in pose.items()}
        for rid, pose in poses.items()
    }
    before = {rid: _string_pulses(issued[rid]) for rid in normalized}
    world._team_joint_move_servos(normalized, duration_s, settle_s=settle_s)
    for rid, pose in normalized.items():
        issued[rid].update(pose)
    command_trace.append({
        "sequence": len(command_trace),
        "phase": phase,
        "targets": {rid: _string_pulses(pose) for rid, pose in normalized.items()},
        "issued_before": before,
        "issued_after": {rid: _string_pulses(issued[rid]) for rid in normalized},
        "duration_s": float(duration_s),
        "settle_s": float(settle_s),
        "execution": "MultiMasterPiProductionV2._team_joint_move_servos",
    })


def _teacher_action(
    name: str,
    targets: Mapping[str, Mapping[int, int]],
    *,
    privileged_basis: str,
) -> dict[str, Any]:
    return {
        "name": name,
        "target_pulses": {
            rid: _string_pulses(pose) for rid, pose in targets.items()
        },
        "privileged_basis": privileged_basis,
    }


def collect(out_dir: Path, *, seed: int, collect_probes: bool) -> dict[str, Any]:
    # Dataset and command-record tests run without the simulation dependency.
    import mujoco
    import sim.multi_masterpi_production as multi_production
    from sim.multi_masterpi_production import TEAM_APPROACH_STANDOFF_M, MultiMasterPiProductionV2
    from scripts.probe_dual_grasp_sync import _plain_beam_model_record, _plain_beam_xml

    out_dir.mkdir(parents=True, exist_ok=False)
    writer = DatasetWriter(out_dir)
    video = None
    report: dict[str, Any] = {"ok": False, "error": None}
    world: MultiMasterPiProductionV2 | None = None
    issued: dict[str, dict[int, int]] = {rid: {} for rid in BEAM_CARRIER_IDS}
    command_trace: list[dict[str, Any]] = []
    try:
        # Reuse the already validated uniform fixture exactly; do not modify its
        # compiled geometry after construction or add any new physical feature.
        original_builder = multi_production.build_multi_robot_xml
        with patch.object(
            multi_production,
            "build_multi_robot_xml",
            _plain_beam_xml(original_builder),
        ):
            world = MultiMasterPiProductionV2(seed=seed, width=960, height=720, render=True)
        fixture = _plain_beam_model_record(world)
        camera = _set_fixed_top_camera(world)
        from scripts.probe_dual_grasp_sync import Video
        video = Video(world, out_dir / "teacher.mp4", 4)
        video.stage = "privileged teacher training"
        world.frame_callback = video.capture
        precision = world._precision_module()
        hover = {int(k): int(v) for k, v in world._hover_pose().items()}
        grasp = {
            int(k): int(v) for k, v in precision.solve_ik(
                precision.CAPTURE_FINGERTIP_RADIUS_CM,
                precision.CAPTURE_GRASP_HEIGHT_CM,
                precision.CAPTURE_GRASP_PITCH_DEG,
            ).items()
        }
        yaw = {"r1": 2500, "r3": 500}

        # The only direct pose operation: known fixture initialization.
        setup_poses: dict[str, list[float]] = {}
        for rid in BEAM_CARRIER_IDS:
            sign = 1.0 if rid == "r1" else -1.0
            robot = world.controllers[rid]
            pose = (
                BEAM_START[0],
                BEAM_START[1] - sign * (0.16 + TEAM_APPROACH_STANDOFF_M),
                float(robot.base_xyz()[2]),
            )
            robot.set_base_pose_for_test(pose, 0.0)
            setup_poses[rid] = [float(v) for v in pose]

        from sim.masterpi_production_v2 import SEARCH_POSE

        folded = {
            rid: {**SEARCH_POSE, 1: precision.GRIPPER_OPEN, 6: 1500}
            for rid in BEAM_CARRIER_IDS
        }
        _move(world, issued, command_trace, folded, 0.6, phase="folded_setup")
        rotate = {rid: {6: yaw[rid]} for rid in BEAM_CARRIER_IDS}
        _move(world, issued, command_trace, rotate, 1.0, settle_s=0.15, phase="side_rotation")
        hover_targets = {
            rid: {1: precision.GRIPPER_OPEN, 6: yaw[rid], **hover}
            for rid in BEAM_CARRIER_IDS
        }
        _move(world, issued, command_trace, hover_targets, 0.70, settle_s=0.08, phase="hover")
        writer.capture(
            world, sample_id="stage_hover", phase="hover",
            teacher_action=_teacher_action(
                "move_to_hover", hover_targets,
                privileged_basis="known fixture coordinates plus existing IK recipe",
            ),
            physics_evaluation=_physics_snapshot(world),
        )

        for height in precision.GRASP_DESCENT_HEIGHTS_CM:
            if height <= precision.CAPTURE_GRASP_HEIGHT_CM + 0.15:
                continue
            descent = {
                int(k): int(v) for k, v in precision.solve_ik(
                    precision.CAPTURE_FINGERTIP_RADIUS_CM,
                    height,
                    precision.CAPTURE_GRASP_PITCH_DEG,
                ).items()
            }
            _move(
                world, issued, command_trace,
                {rid: descent for rid in BEAM_CARRIER_IDS}, 0.32,
                phase=f"descent_{height:.3f}cm",
            )
        grasp_targets = {rid: grasp for rid in BEAM_CARRIER_IDS}
        _move(world, issued, command_trace, grasp_targets, 0.42, settle_s=0.08, phase="preclose")
        writer.capture(
            world, sample_id="stage_preclose", phase="preclose_open",
            teacher_action=_teacher_action(
                "open_preclose_extraction_goal", grasp_targets,
                privileged_basis="known beam endpoint coordinates plus existing capture IK",
            ),
            physics_evaluation=_physics_snapshot(world),
        )

        if collect_probes:
            for rid in BEAM_CARRIER_IDS:
                for servo in PROBE_SERVOS:
                    nominal = int(issued[rid][servo])
                    for delta in PROBE_DELTAS:
                        target = max(500, min(2500, nominal + delta))
                        actual_delta = target - nominal
                        pair_id = f"{rid}_s{servo}_requested{delta:+d}_applied{actual_delta:+d}"
                        probe = {
                            "pair_id": pair_id, "acted_robot_id": rid, "servo": servo,
                            "requested_delta_pwm": delta, "delta_pwm": actual_delta, "from_phase": "preclose_open",
                        }
                        writer.capture(
                            world,
                            sample_id=f"probe_{pair_id}_before",
                            phase="preclose_probe_before",
                            teacher_action={
                                "name": "observe_before_local_servo_probe",
                                "target_pulses": {},
                                "privileged_basis": "isolated finite-difference pair baseline",
                            },
                            probe_delta={**probe, "pair_role": "before"},
                            physics_evaluation=_physics_snapshot(world),
                        )
                        target_pose = {rid: {servo: target}}
                        _move(
                            world, issued, command_trace, target_pose, 0.18,
                            settle_s=0.04, phase="preclose_probe",
                        )
                        writer.capture(
                            world,
                            sample_id=f"probe_{pair_id}_after",
                            phase="preclose_probe_after",
                            teacher_action=_teacher_action(
                                "local_servo_probe", target_pose,
                                privileged_basis="bounded teacher perturbation around open preclose",
                            ),
                            probe_delta={**probe, "pair_role": "after"},
                            physics_evaluation=_physics_snapshot(world),
                        )
                        restore = {rid: {servo: nominal}}
                        _move(
                            world, issued, command_trace, restore, 0.18,
                            settle_s=0.04, phase="preclose_restore",
                        )

        close_targets = {
            rid: {1: precision.GRIPPER_CLOSE} for rid in BEAM_CARRIER_IDS
        }
        _move(world, issued, command_trace, close_targets, 0.65, settle_s=0.18, phase="closed")
        closed_physics = _physics_snapshot(world)
        writer.capture(
            world, sample_id="stage_closed", phase="closed",
            teacher_action=_teacher_action(
                "simultaneous_close", close_targets,
                privileged_basis="known gripper close pulse",
            ),
            physics_evaluation=closed_physics,
        )
        if not all(bool(v.get("bilateral")) for v in closed_physics["contacts"].values()):
            raise RuntimeError(f"CARRIER_CONTACT_MISSING: {closed_physics['contacts']}")

        lift_targets = {rid: hover for rid in BEAM_CARRIER_IDS}
        _move(world, issued, command_trace, lift_targets, 0.70, settle_s=0.25, phase="lifted")
        lifted_physics = _physics_snapshot(world)
        writer.capture(
            world, sample_id="stage_lifted", phase="lifted",
            teacher_action=_teacher_action(
                "lift_to_hover", lift_targets,
                privileged_basis="existing hover IK recipe after verified bilateral close",
            ),
            physics_evaluation=lifted_physics,
        )
        if lifted_physics["height_above_start_m"] < 0.035:
            raise RuntimeError(f"COOPERATIVE_LIFT_FAILED: {lifted_physics}")

        dt = float(world.model.opt.timestep)
        report["hold_evaluation_samples"] = [lifted_physics]
        for _ in range(max(1, int(round(2.0 / dt)))):
            world._physics_step_for(world.controllers["r1"])
            snapshot = _physics_snapshot(world)
            report["hold_evaluation_samples"].append(snapshot)
            if snapshot["height_above_start_m"] < 0.03:
                raise RuntimeError("UNASSISTED_HOLD_LIFT_LOST")
            if not all(bool(v.get("bilateral")) for v in snapshot["contacts"].values()):
                raise RuntimeError("UNASSISTED_HOLD_BILATERAL_CONTACT_LOST")
            if any(snapshot["weld_active"].values()):
                raise RuntimeError("WELD_ACTIVE_DURING_TEACHER_COLLECTION")
        held_physics = _physics_snapshot(world)
        writer.capture(
            world, sample_id="stage_held", phase="held_2s",
            teacher_action={
                "name": "hold_without_new_command",
                "target_pulses": {},
                "privileged_basis": "2 second no-weld reference evaluation",
            },
            physics_evaluation=held_physics,
        )
        report.update({
            "ok": True,
            "success_criteria": {
                "bilateral_both_at_close": True,
                "lift_height_min_m": 0.035,
                "bilateral_and_height_min_m_held_for_s": 2.0,
                "weld_inactive": True,
            },
            "final_privileged_evaluation": held_physics,
            "setup_base_poses_m": setup_poses,
            "camera": camera,
            "fixture": fixture,
        })
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
        if world is not None:
            try:
                report["failure_privileged_evaluation"] = _physics_snapshot(world)
            except Exception as snapshot_exc:
                report["snapshot_error"] = f"{type(snapshot_exc).__name__}: {snapshot_exc}"
    finally:
        writer.write()
        _json_write(out_dir / "teacher_commands.json", command_trace)
        report["actor_sample_count"] = len(writer.actor_samples)
        report["privileged_label_count"] = len(writer.privileged_labels)
        if world is not None:
            world.frame_callback = None
        if video is not None:
            try:
                video.close()
            except Exception as video_exc:
                report["video_error"] = f"{type(video_exc).__name__}: {video_exc}"
        if world is not None:
            world.close()
    return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Collect local RGB grasp demonstrations from a privileged training teacher."
    )
    parser.add_argument("--out-dir", type=Path, required=True, help="New output directory; existing directories are rejected.")
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--collect-probes", action="store_true", help="Collect isolated servo 3/4/5 +/-25/50 PWM image effects.")
    args = parser.parse_args()
    out_dir = args.out_dir.expanduser().resolve()
    script_path = Path(__file__).resolve()
    source_inputs = [
        script_path,
        ROOT / "scripts" / "probe_dual_grasp_sync.py",
        ROOT / "sim" / "multi_masterpi_production.py",
        ROOT / "sim" / "masterpi_production_v2.py",
        ROOT / "sim" / "cooperative_payload.py",
    ]
    manifest: dict[str, Any] = {
        "format": "camera_grasp_privileged_teacher_v1",
        "scope": "local_pregrasp_teaching_only_no_navigation",
        "repository": str(ROOT),
        "source_sha": _git(["rev-parse", "HEAD"]),
        "git_status_porcelain": _git(["status", "--porcelain"]),
        "input_hashes": {str(path.relative_to(ROOT)): _file_sha256(path) for path in source_inputs},
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "mujoco": getattr(mujoco, "__version__", "unknown"),
            "executable": sys.executable,
            "UGRP_BEAM_DYNAMIC": os.environ.get("UGRP_BEAM_DYNAMIC"),
        },
        "config": {
            "seed": args.seed,
            "collect_probes": args.collect_probes,
            "side_grasp": True,
            "approach_distance_m": 0.0,
            "transport_distance_m": 0.0,
            "weld_assistance": False,
            "fixture_geometry": "validated_plain_beam_from_probe_dual_grasp_sync",
            "runtime_geometry_modification": False,
            "body_pose_writes": "initialization_only",
            "top_camera": {
                "name": "cctv_top",
                "position_m": list(TOP_CAMERA_POSITION),
                "quaternion_wxyz": list(TOP_CAMERA_QUATERNION),
                "fov_y_deg": TOP_CAMERA_FOV_DEG,
            },
            "teacher_stage_boundaries": list(TEACHER_STAGES),
            "probe_servos": list(PROBE_SERVOS),
            "probe_delta_pwm": list(PROBE_DELTAS),
            "actor_observation_contract": [
                "own_robot_rgb", "fixed_shared_top_rgb", "own_issued_command_history",
            ],
            "privileged_teacher_only": [
                "fixture_coordinates", "IK_targets", "contact_state", "payload_pose", "success_evaluation",
            ],
            "student_pilot_protocol": {
                "scope": "teacher_initialized_local_pregrasp_with_unseen_arm_perturbations",
                "navigation_claim": False,
                "control_episode_forbidden_inputs": ["IK", "coordinates", "contacts", "payload_pose"],
                "preclose_control": "own/top RGB finite-difference correction for servos 3/4/5",
                "close_lift": "teacher demonstration action-delta playback",
                "required_comparison": [
                    "local_RGB_recovery_plus_teacher_lift_playback",
                    "open_loop_teacher_action_playback_ablation",
                ],
            },
        },
        "files": {
            "actor_samples": "actor_samples.json",
            "privileged_labels": "privileged_labels.json",
            "teacher_commands": "teacher_commands.json",
            "run_report": "run_report.json",
            "rgb_root": "rgb/",
        },
    }
    if out_dir.exists():
        parser.error(f"--out-dir must be new: {out_dir}")
    run_report = collect(out_dir, seed=args.seed, collect_probes=args.collect_probes)
    _json_write(out_dir / "run_report.json", run_report)
    actor_samples = json.loads((out_dir / "actor_samples.json").read_text(encoding="utf-8"))
    privileged_labels = json.loads((out_dir / "privileged_labels.json").read_text(encoding="utf-8"))
    command_trace = json.loads((out_dir / "teacher_commands.json").read_text(encoding="utf-8"))
    manifest["samples"] = [
        {
            "sample_id": sample["sample_id"],
            "robot_id": sample["robot_id"],
            "phase": sample["phase"],
            "observations": sample["observations"],
            "own_issued_command_snapshot": sample["own_issued_command_snapshot"],
        }
        for sample in actor_samples
    ]
    manifest["probe_delta_labels"] = [
        {"sample_id": label["sample_id"], **label["probe_delta_label"]}
        for label in privileged_labels if label.get("probe_delta_label")
    ]
    manifest["teacher_command_trace"] = command_trace
    manifest["result"] = {
        "ok": bool(run_report["ok"]),
        "actor_sample_count": run_report["actor_sample_count"],
        "privileged_label_count": run_report["privileged_label_count"],
        "error": run_report.get("error"),
    }
    _json_write(out_dir / "manifest.json", manifest)
    print(json.dumps({"out_dir": str(out_dir), "source_sha": manifest["source_sha"], **manifest["result"]}, ensure_ascii=False, sort_keys=True))
    return 0 if run_report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
