"""Regression tests for continuous, planar TEAM warehouse motion.

The visible warehouse demo used to update MuJoCo free-body/base poses directly
between rendered frames.  That can look like motion in a video while still
being a teleport, and it also hides whether the chassis ever performed a real
lateral translation.  These tests observe the public shared-world callback so
the checks remain independent of a particular renderer or video encoder.
"""

from __future__ import annotations

from contextlib import ExitStack
import math
import unittest
from unittest.mock import patch

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.warehouse_mission import CARGO_SPECS


MISSION_COMMANDS = [
    {
        "robot_id": rid,
        "action": "team_zone_transfer",
        "mission_id": "warehouse_motion_continuity",
        "source_zone": "A",
        "destination_zone": "B",
        "selector": "all",
    }
    for rid in ("r1", "r2", "r3")
]

# The callback is deliberately treated as a trajectory stream, not as a
# render-frame stream.  Implementations may call it more often than a video
# encoder; they must not call it only after a final pose has been installed.
MAX_ROBOT_STEP_M = 0.12
MAX_CARGO_STEP_M = 0.08
MIN_LATERAL_TRAVEL_M = 0.08
MAX_LATERAL_YAW_SWING_RAD = math.radians(8.0)
MAX_FILTERED_TEAM_PATH_M = 28.0
# Runtime auctions may legitimately give one robot two long return legs while
# keeping the team path inside the stricter aggregate envelope. Guard against
# the old 49 m detour at team level without re-imposing a fixed role rotation.
MAX_FILTERED_ROBOT_PATH_M = 14.0
# The procedural barrier and gate intentionally require a measurable detour.
# Keep it well below the former outer-aisle path while allowing valid A* paths.
MAX_PAYLOAD_DETOUR_RATIO = 2.20


def _angle_delta(first: float, second: float) -> float:
    return abs((float(second) - float(first) + math.pi) % (2.0 * math.pi) - math.pi)


def _snapshot(world: MultiMasterPiProductionV2) -> dict[str, object]:
    robots = {
        rid: {
            "position": tuple(float(v) for v in world.robot(rid).base_xyz()),
            "yaw": float(world.robot(rid).base_rpy()[2]),
        }
        for rid in ("r1", "r2", "r3")
    }
    warehouse = world.warehouse_state()
    cargo_state = warehouse["cargo"]
    cargo = {
        cargo_id: tuple(float(v) for v in item["position"])
        for cargo_id, item in cargo_state.items()
    }
    return {
        "sim_time": float(world.data.time),
        "robots": robots,
        "cargo": cargo,
    }


def _run_with_samples() -> tuple[MultiMasterPiProductionV2, list[dict[str, object]], dict[str, object]]:
    world = MultiMasterPiProductionV2(seed=11, render=False)
    samples: list[dict[str, object]] = [_snapshot(world)]

    def capture() -> None:
        samples.append(_snapshot(world))

    world.frame_callback = capture
    results = world.act_parallel(MISSION_COMMANDS)
    world.frame_callback = None
    return world, samples, results


class WarehouseMotionContinuityTests(unittest.TestCase):
    def test_robot_trajectory_has_no_pose_jump_or_zero_time_motion(self):
        world, samples, results = _run_with_samples()
        try:
            self.assertTrue(
                all(result.ok for result in results.values()),
                {rid: result.reason for rid, result in results.items()},
            )
            self.assertGreaterEqual(len(samples), 8, "warehouse action did not expose a trajectory")
            violations: list[str] = []
            for index, (previous, current) in enumerate(zip(samples, samples[1:]), start=1):
                sim_dt = float(current["sim_time"]) - float(previous["sim_time"])
                for rid in ("r1", "r2", "r3"):
                    old = previous["robots"][rid]["position"]
                    new = current["robots"][rid]["position"]
                    distance = math.dist(old, new)
                    if distance > MAX_ROBOT_STEP_M:
                        violations.append(
                            f"sample {index} {rid} jumped {distance:.3f}m "
                            f"(dt={sim_dt:.6f}s)"
                        )
                    # A pose change without advancing the shared simulation is
                    # exactly the qpos/base-pose teleport this regression guards.
                    if sim_dt <= 1e-9 and distance > 0.002:
                        violations.append(
                            f"sample {index} {rid} moved {distance:.3f}m without sim time"
                        )
            self.assertFalse(violations, "non-continuous robot trajectory: " + "; ".join(violations[:8]))
        finally:
            world.close()

    def test_team_performs_meaningful_lateral_translation_with_heading_held(self):
        world, samples, results = _run_with_samples()
        try:
            self.assertTrue(
                all(result.ok for result in results.values()),
                {rid: result.reason for rid, result in results.items()},
            )
            found: tuple[str, int, int, float] | None = None
            # This is a chassis requirement: at least one carrier/scout must
            # translate along world Y while keeping its heading essentially
            # unchanged.  A turn-then-drive detour is not a lateral segment.
            for rid in ("r1", "r2", "r3"):
                ys = [float(sample["robots"][rid]["position"][1]) for sample in samples]
                for start in range(len(samples)):
                    for end in range(start + 2, len(samples)):
                        lateral = max(ys[start:end + 1]) - min(ys[start:end + 1])
                        if lateral < MIN_LATERAL_TRAVEL_M:
                            continue
                        yaw_swing = _angle_delta(
                            samples[start]["robots"][rid]["yaw"],
                            samples[end]["robots"][rid]["yaw"],
                        )
                        if yaw_swing <= MAX_LATERAL_YAW_SWING_RAD:
                            found = (rid, start, end, lateral)
                            break
                    if found:
                        break
                if found:
                    break
            self.assertIsNotNone(
                found,
                "no robot trajectory segment translated laterally by at least "
                f"{MIN_LATERAL_TRAVEL_M:.2f}m while its heading stayed fixed",
            )
        finally:
            world.close()

    def test_cargo_trajectory_is_bounded_and_does_not_use_pose_reset_api(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        samples: list[dict[str, object]] = [_snapshot(world)]

        def capture() -> None:
            samples.append(_snapshot(world))

        world.frame_callback = capture
        try:
            # Reset helpers are valid during world construction/reset, but are
            # not a transport primitive.  Catching these calls makes the test
            # fail even if an implementation subdivides a teleport into many
            # visually smooth pose assignments.
            with ExitStack() as stack:
                base_pose_setters = []
                free_body_pose_setters = []
                for rid in ("r1", "r2", "r3"):
                    controller = world.robot(rid)
                    base_pose_setters.append(stack.enter_context(
                        patch.object(
                            controller,
                            "set_base_pose_for_test",
                            wraps=controller.set_base_pose_for_test,
                        )
                    ))
                    free_body_pose_setters.append(stack.enter_context(
                        patch.object(
                            controller,
                            "set_free_body_pose_for_reset",
                            wraps=controller.set_free_body_pose_for_reset,
                        )
                    ))
                results = world.act_parallel(MISSION_COMMANDS)
            self.assertTrue(
                all(result.ok for result in results.values()),
                {rid: result.reason for rid, result in results.items()},
            )
            self.assertEqual(
                sum(spy.call_count for spy in base_pose_setters),
                0,
                "transport must not install robot base poses directly",
            )
            self.assertEqual(
                sum(spy.call_count for spy in free_body_pose_setters),
                0,
                "transport must not install cargo free-body poses directly",
            )
            violations: list[str] = []
            for index, (previous, current) in enumerate(zip(samples, samples[1:]), start=1):
                sim_dt = float(current["sim_time"]) - float(previous["sim_time"])
                for cargo_id in previous["cargo"]:
                    distance = math.dist(previous["cargo"][cargo_id], current["cargo"][cargo_id])
                    if distance > MAX_CARGO_STEP_M:
                        violations.append(f"sample {index} {cargo_id} moved {distance:.3f}m")
                    if sim_dt <= 1e-9 and distance > 0.002:
                        violations.append(f"sample {index} {cargo_id} moved without sim time")
            self.assertFalse(violations, "non-continuous cargo trajectory: " + "; ".join(violations[:8]))
        finally:
            world.frame_callback = None
            world.close()

    def test_continuous_motion_path_still_delivers_every_cargo(self):
        world, samples, results = _run_with_samples()
        try:
            self.assertTrue(
                all(result.ok for result in results.values()),
                {rid: result.reason for rid, result in results.items()},
            )
            state = world.warehouse_state()
            self.assertEqual(state["status"], "SUCCESS")
            self.assertEqual(state["moved_count"], len(CARGO_SPECS))
            self.assertEqual(state["remaining_ids"], [])
            self.assertTrue(state["success"])
            self.assertTrue(all(item["zone"] == "B" for item in state["cargo"].values()))

            # Physical continuity alone is not enough: the former route sent
            # every robot through distant outer aisles and accumulated about
            # 49 m of team travel for a 0.74 m cargo transfer. The new physical
            # gate/barrier needs a real detour, but must remain inside a compact
            # obstacle-course envelope after ignoring sub-2 mm settling noise.
            robot_paths = {}
            for rid in ("r1", "r2", "r3"):
                distances = [
                    math.dist(previous["robots"][rid]["position"][:2], current["robots"][rid]["position"][:2])
                    for previous, current in zip(samples, samples[1:])
                ]
                robot_paths[rid] = sum(distance for distance in distances if distance > 0.002)
            self.assertLessEqual(sum(robot_paths.values()), MAX_FILTERED_TEAM_PATH_M, robot_paths)
            self.assertTrue(
                all(distance <= MAX_FILTERED_ROBOT_PATH_M for distance in robot_paths.values()),
                robot_paths,
            )

            for spec in world.warehouse_specs:
                distances = [
                    math.dist(previous["cargo"][spec.cargo_id][:2], current["cargo"][spec.cargo_id][:2])
                    for previous, current in zip(samples, samples[1:])
                ]
                path = sum(distance for distance in distances if distance > 0.001)
                direct = math.dist(spec.start_xyz[:2], spec.goal_xyz[:2])
                self.assertLessEqual(path / direct, MAX_PAYLOAD_DETOUR_RATIO, spec.cargo_id)

            phases = [event["phase"] for event in world.warehouse_trace]
            self.assertNotIn("warehouse_outer_aisle_strafe", phases)
            self.assertNotIn("warehouse_aisle_return", phases)
            self.assertFalse(
                any(phase.startswith("warehouse_payload_x_correction_") for phase in phases),
                phases,
            )
            self.assertFalse(
                any(phase.startswith("warehouse_payload_y_correction_") for phase in phases),
                phases,
            )
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
