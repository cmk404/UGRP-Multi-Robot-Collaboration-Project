"""Evaluation-only deterministic replay of a saved pick-match raw command stream.

This diagnostic never invokes a planner. It reconstructs the saved fixture and
reissues only raw actuator commands at their recorded MuJoCo times. The external
camera and all contact/trajectory fields are evaluation-only evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

import numpy as np


def load_json(path: Path) -> Any:
    return json.loads(path.read_text())


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def load_raw_actions(path: Path) -> list[dict[str, Any]]:
    actions = []
    previous = -math.inf
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        row = json.loads(line)
        if row.get("event") != "raw_action":
            continue
        when = float(row["time"])
        if not math.isfinite(when) or when < previous:
            raise ValueError(f"nonmonotonic raw command time at line {line_no}")
        raw = row.get("raw_action")
        if not isinstance(raw, dict):
            raise ValueError(f"missing raw_action at line {line_no}")
        actions.append({"time": when, "raw_action": raw, "source_line": line_no})
        previous = when
    if not actions:
        raise ValueError("episode contains no raw actions")
    return actions


def load_reference(path: Path) -> list[dict[str, Any]]:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError("episode contains no referee samples")
    previous = -math.inf
    for index, row in enumerate(rows):
        now = float(row["elapsed_sim_s"])
        if now <= previous:
            raise ValueError(f"nonmonotonic referee time at row {index}")
        previous = now
    return rows


def compare_trajectories(reference: list[dict[str, Any]], replay: list[dict[str, Any]],
                         position_tolerance_m: float, lift_tolerance_m: float) -> dict[str, Any]:
    if len(reference) != len(replay):
        return {"accepted": False, "reason": "SAMPLE_COUNT_MISMATCH",
                "reference_samples": len(reference), "replay_samples": len(replay)}
    position_errors, lift_errors, time_errors = [], [], []
    for expected, actual in zip(reference, replay):
        position_errors.append(float(np.linalg.norm(
            np.asarray(actual["position"], dtype=float) - np.asarray(expected["position"], dtype=float))))
        lift_errors.append(abs(float(actual["lift_m"]) - float(expected["lift_m"])))
        time_errors.append(abs(float(actual["elapsed_sim_s"]) - float(expected["elapsed_sim_s"])))
    max_position = max(position_errors, default=math.inf)
    max_lift = max(lift_errors, default=math.inf)
    max_time = max(time_errors, default=math.inf)
    accepted = max_position <= position_tolerance_m and max_lift <= lift_tolerance_m and max_time <= 0.003
    return {"accepted": accepted,
            "reason": "CLOSE_TRAJECTORY_MATCH" if accepted else "TRAJECTORY_MISMATCH",
            "reference_samples": len(reference), "replay_samples": len(replay),
            "max_position_error_m": max_position, "max_lift_error_m": max_lift,
            "max_time_error_s": max_time,
            "position_rmse_m": float(np.sqrt(np.mean(np.square(position_errors)))),
            "lift_rmse_m": float(np.sqrt(np.mean(np.square(lift_errors)))),
            "position_tolerance_m": position_tolerance_m,
            "lift_tolerance_m": lift_tolerance_m}


def cargo_contacts(world: Any, cargo_body_id: int) -> list[dict[str, Any]]:
    import mujoco

    found = []
    for index in range(int(world.data.ncon)):
        contact = world.data.contact[index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        body1, body2 = int(world.model.geom_bodyid[geom1]), int(world.model.geom_bodyid[geom2])
        if cargo_body_id not in {body1, body2}:
            continue
        wrench = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(world.model, world.data, index, wrench)
        found.append({
            "contact_index": index,
            "geom1": mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, geom1),
            "geom2": mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, geom2),
            "body1": mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY, body1),
            "body2": mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_BODY, body2),
            "distance_m": float(contact.dist), "normal_force_n": float(wrench[0]),
        })
    return found


class ObserverVideo:
    def __init__(self, world: Any, path: Path, condition: str, seed: int, fps: int = 10):
        import mujoco

        self.world = world
        self.fps = fps
        self.next_elapsed = 0.0
        self.renderer = mujoco.Renderer(world.model, height=720, width=1280)
        cargo = np.asarray(world.warehouse_state()["cargo"]["small_box_01"]["position"], dtype=float)
        robot = np.asarray(world.data.xpos[world.robot("r1").robot_bid], dtype=float)
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(world.model, self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = (cargo + robot) / 2.0
        self.camera.lookat[2] = max(0.12, float(self.camera.lookat[2]))
        self.camera.distance = max(1.15, float(np.linalg.norm(cargo[:2] - robot[:2])) * 1.8)
        self.camera.azimuth = 145.0
        self.camera.elevation = -24.0
        self.spec = {"type": "fixed_free_evaluation_camera", "lookat": self.camera.lookat.tolist(),
                     "distance": float(self.camera.distance), "azimuth": float(self.camera.azimuth),
                     "elevation": float(self.camera.elevation), "actor_input": False}
        self.label = f"{condition} seed={seed} COMMAND REPLAY - EVALUATOR ONLY"
        self.process = subprocess.Popen([
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
            "-pix_fmt", "rgb24", "-s", "1280x720", "-r", str(fps), "-i", "-",
            "-an", "-c:v", "libx264", "-preset", "ultrafast", "-crf", "23",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)], stdin=subprocess.PIPE)

    def capture(self, elapsed: float) -> None:
        import cv2
        import mujoco

        self.renderer.update_scene(self.world.data, camera=self.camera)
        frame = self.renderer.render().copy()
        cv2.rectangle(frame, (0, 0), (1280, 42), (0, 0, 0), -1)
        cv2.putText(frame, f"{self.label}  sim={elapsed:.1f}s", (14, 29),
                    cv2.FONT_HERSHEY_SIMPLEX, .72, (255, 255, 255), 2, cv2.LINE_AA)
        self.process.stdin.write(frame.tobytes())
        self.next_elapsed += 1.0 / self.fps

    def close(self) -> None:
        errors = []
        try:
            self.renderer.close()
        except Exception as exc:
            errors.append(f"renderer: {exc}")
        try:
            if self.process.stdin:
                self.process.stdin.close()
            status = self.process.wait(timeout=30)
            if status:
                errors.append(f"ffmpeg exited {status}")
        except Exception as exc:
            errors.append(f"ffmpeg close: {exc}")
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if errors:
            raise RuntimeError("; ".join(errors))


def verify_source_tree(repo: Path, source_sha: str) -> dict[str, Any]:
    replay_sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    status = subprocess.run(["git", "diff", "--quiet", source_sha, "--", "sim", "harness"], cwd=repo)
    if status.returncode not in {0, 1}:
        raise RuntimeError("git diff source verification failed")
    files = ("sim/camera_robot_port.py", "sim/multi_masterpi_production.py")
    hashes = {}
    for relative in files:
        current = (repo / relative).read_bytes()
        original = subprocess.check_output(["git", "show", f"{source_sha}:{relative}"], cwd=repo)
        hashes[relative] = {"working_tree_sha256": sha(current), "source_git_sha256": sha(original),
                            "matches": current == original}
    if status.returncode != 0 or not all(item["matches"] for item in hashes.values()):
        raise RuntimeError("sim/harness dependencies differ from source episode SHA")
    return {"replay_git_sha": replay_sha, "source_git_sha": source_sha,
            "sim_harness_tree_matches_source": True, "critical_file_hashes": hashes}


def run(args: argparse.Namespace) -> dict[str, Any]:
    import mujoco

    episode = args.episode.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = load_json(episode / "config.json")
    fixture = load_json(episode / "fixture-evaluation-only.json")
    reference = load_reference(episode / "evaluation-only.jsonl")
    actions = load_raw_actions(episode / "raw-commands.jsonl")
    if config.get("git_sha") != args.expected_sha:
        raise ValueError(f"source episode SHA is {config.get('git_sha')}, expected {args.expected_sha}")

    repo = args.repo.resolve()
    provenance = verify_source_tree(repo, str(config["git_sha"]))
    sys.path.insert(0, str(repo))
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2

    world = None
    video = None
    replay_rows = []
    cleanup_errors = []
    try:
        world = MultiMasterPiProductionV2(warehouse_layout="camera_team", seed=int(config["seed"]),
                    render=not args.no_video, width=640, height=480,
                    warehouse_cargo_ids=("small_box_01",))
        world.model.opt.impratio = 10
        world.model.opt.noslip_iterations = 0
        for eid in world.warehouse_weld_ids.values():
            world.data.eq_active[int(eid)] = 0
        port = CameraRobotPort(world, "r1")
        qpos_initial = sha(world.data.qpos.tobytes())
        if qpos_initial != fixture["initial_qpos_sha256"]:
            raise RuntimeError("reconstructed initial qpos does not match source fixture")
        camera_hash = sha(b"".join(v.tobytes() for v in
            (world.model.cam_pos, world.model.cam_quat, world.model.cam_fovy)))
        geometry_hash = sha(b"".join(v.tobytes() for v in
            (world.model.geom_size, world.model.geom_rgba, world.model.geom_friction)))
        if camera_hash != fixture["camera_parameters_sha256"] or geometry_hash != fixture["geometry_sha256"]:
            raise RuntimeError("reconstructed camera/geometry fixture hash mismatch")
        initial_height = float(world.warehouse_state()["cargo"]["small_box_01"]["position"][2])
        cargo_spec = next(spec for spec in world.warehouse_specs if spec.cargo_id == "small_box_01")
        cargo_body_id = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, cargo_spec.body_name)
        start = float(world.data.time)
        if actions[0]["time"] < start - 1e-9:
            raise RuntimeError("raw command stream begins before reconstructed fixture time")
        if not args.no_video:
            video = ObserverVideo(world, output / "evaluation-command-replay.mp4",
                                  str(config["condition"]), int(config["seed"]))
            (output / "observer-camera-evaluation-only.json").write_text(json.dumps(video.spec, indent=2) + "\n")
        action_index = 0
        for expected in reference:
            target = start + float(expected["elapsed_sim_s"])
            while float(world.data.time) + 1e-9 < target:
                now = float(world.data.time)
                port.tick(now)
                while action_index < len(actions) and actions[action_index]["time"] <= now + 1e-9:
                    port.apply(actions[action_index]["raw_action"], now)
                    action_index += 1
                world._physics_step_for(world.robot("r1"))
            now = float(world.data.time)
            port.tick(now)
            while action_index < len(actions) and actions[action_index]["time"] <= now + 1e-9:
                port.apply(actions[action_index]["raw_action"], now)
                action_index += 1
            cargo = world.warehouse_state()["cargo"]["small_box_01"]
            elapsed = now - start
            row = {"elapsed_sim_s": elapsed, "position": [float(v) for v in cargo["position"]],
                   "lift_m": float(cargo["position"][2]) - initial_height,
                   "qpos_sha256": sha(world.data.qpos.tobytes()),
                   "cargo_contacts": cargo_contacts(world, cargo_body_id),
                   "constraints_active": {rid: bool(world.data.eq_active[int(eid)])
                       for (cargo_id, rid), eid in world.warehouse_weld_ids.items()
                       if cargo_id == "small_box_01"}}
            replay_rows.append(row)
            if video is not None and elapsed + 1e-9 >= video.next_elapsed:
                video.capture(elapsed)
        port.stop()
        comparison = compare_trajectories(reference, replay_rows,
                                           args.position_tolerance_m, args.lift_tolerance_m)
        all_constraints_off = all(not any(row["constraints_active"].values()) for row in replay_rows)
        comparison["all_target_welds_off"] = all_constraints_off
        comparison["all_raw_actions_reissued"] = action_index == len(actions)
        final_camera_hash = sha(b"".join(v.tobytes() for v in
            (world.model.cam_pos, world.model.cam_quat, world.model.cam_fovy)))
        final_geometry_hash = sha(b"".join(v.tobytes() for v in
            (world.model.geom_size, world.model.geom_rgba, world.model.geom_friction)))
        fixture_unchanged = final_camera_hash == camera_hash and final_geometry_hash == geometry_hash
        comparison["fixture_unchanged"] = fixture_unchanged
        comparison["diagnostic_accepted"] = bool(comparison["accepted"] and all_constraints_off
                                                  and action_index == len(actions) and fixture_unchanged)
        with (output / "replay-evaluation-only.jsonl").open("w") as handle:
            for row in replay_rows:
                handle.write(json.dumps(row) + "\n")
        result = {"schema": "ugrp.pick_match_raw_replay.v1", "source_episode": str(episode),
                  "source_git_sha": config["git_sha"], "seed": config["seed"],
                  "scope": "evaluation_only_command_replay_not_actor_input_not_new_model_success",
                  "model_calls": 0, "computer_vision_policy_calls": 0,
                  "truth_guided_corrections": 0, "raw_actions": len(actions),
                  "provenance": provenance,
                  "source_files": {
                      "config.json": sha((episode / "config.json").read_bytes()),
                      "raw-commands.jsonl": sha((episode / "raw-commands.jsonl").read_bytes()),
                      "evaluation-only.jsonl": sha((episode / "evaluation-only.jsonl").read_bytes())},
                  "comparison": comparison,
                  "fixture": {"initial_qpos_sha256": qpos_initial,
                              "camera_parameters_sha256": camera_hash,
                              "geometry_sha256": geometry_hash,
                              "final_camera_parameters_sha256": final_camera_hash,
                              "final_geometry_sha256": final_geometry_hash},
                  "cleanup_errors": cleanup_errors,
                  "observer_video": video.spec if video is not None else None}
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        if video is not None:
            try:
                video.close()
            except Exception as exc:
                cleanup_errors.append(f"video.close: {exc}")
        if world is not None:
            try:
                world.close()
            except Exception as exc:
                cleanup_errors.append(f"world.close: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    parser.add_argument("--expected-sha", default="c692d5c45138e3e77157dd2854b80449a3f0f5b1")
    parser.add_argument("--position-tolerance-m", type=float, default=2e-4)
    parser.add_argument("--lift-tolerance-m", type=float, default=2e-4)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    try:
        result = run(args)
        status = 0 if result["comparison"]["diagnostic_accepted"] and not result["cleanup_errors"] else 1
        (args.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    except Exception as exc:
        result = {"schema": "ugrp.pick_match_raw_replay.v1", "source_episode": str(args.episode.resolve()),
                  "scope": "evaluation_only_command_replay_not_actor_input_not_new_model_success",
                  "error": f"{type(exc).__name__}: {exc}", "diagnostic_accepted": False}
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "failure-result.json").write_text(json.dumps(result, indent=2) + "\n")
        status = 1
    print(json.dumps(result, indent=2))
    return status


if __name__ == "__main__":
    raise SystemExit(main())
