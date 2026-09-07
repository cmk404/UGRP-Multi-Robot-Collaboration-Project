from __future__ import annotations

import unittest

import mujoco

from sim.adaptive_warehouse import (
    CargoObservation,
    RobotObservation,
    TerrainObservation,
    auction_roles,
    plan_local_path,
    shortest_zone_route,
)
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.warehouse_mission import CARGO_SPECS, cargo_specs_for_seed


def mission_commands(destination: str = "B") -> list[dict[str, object]]:
    return [
        {
            "robot_id": rid,
            "action": "team_zone_transfer",
            "mission_id": "adaptive_unseen_goal",
            "source_zone": "A",
            "destination_zone": destination,
            "selector": "all",
        }
        for rid in ("r1", "r2", "r3")
    ]


class AdaptivePlannerTests(unittest.TestCase):
    def test_role_auction_uses_pose_not_cargo_identity(self):
        cargo = CargoObservation(
            cargo_id="never_seen_before",
            position_xy=(0.8, 0.0),
            yaw_rad=0.0,
            dimensions_m=(0.05, 0.46, 0.04),
            mass_kg=0.29,
        )
        first = auction_roles(
            (
                RobotObservation("r1", (0.7, -0.3)),
                RobotObservation("r2", (0.7, 0.3)),
                RobotObservation("r3", (-0.2, 0.0)),
            ),
            cargo,
        )
        second = auction_roles(
            (
                RobotObservation("r1", (-0.2, 0.0)),
                RobotObservation("r2", (0.7, -0.3)),
                RobotObservation("r3", (0.7, 0.3)),
            ),
            cargo,
        )
        self.assertEqual(first.carriers, ("r1", "r2"))
        self.assertEqual(second.carriers, ("r2", "r3"))
        self.assertNotEqual(first.carriers, second.carriers)

    def test_route_changes_when_observed_graph_changes(self):
        normal = {
            "A": {"B": 1.0, "C": 3.0},
            "B": {"A": 1.0, "C": 1.0},
            "C": {"A": 3.0, "B": 1.0},
        }
        blocked_b = {
            "A": {"B": float("inf"), "C": 3.0},
            "B": {"A": float("inf"), "C": 1.0},
            "C": {"A": 3.0, "B": 1.0},
        }
        self.assertEqual(shortest_zone_route("A", "C", normal), ("A", "B", "C"))
        self.assertEqual(shortest_zone_route("A", "C", blocked_b), ("A", "C"))

    def test_inflated_path_routes_around_a_hard_obstacle(self):
        wall = TerrainObservation(
            "novel_wall", "barrier", (1.4, 0.0), (0.08, 0.30),
            0.12, False, float("inf"),
        )
        path = plan_local_path(
            (0.7, 0.0), (2.1, 0.0), (wall,),
            footprint_xy=(0.12, 0.18),
            bounds=(0.5, 2.3, -1.0, 1.0),
        )
        self.assertGreater(len(path), 2)
        self.assertTrue(any(abs(point[1]) >= 0.48 for point in path), path)

    def test_each_seed_builds_a_reproducible_unfamiliar_object_profile(self):
        first = cargo_specs_for_seed(41)
        replay = cargo_specs_for_seed(41)
        other = cargo_specs_for_seed(42)
        self.assertEqual(first, replay)
        self.assertNotEqual(
            [(s.dimensions_m, s.mass_kg, s.start_yaw_rad) for s in first],
            [(s.dimensions_m, s.mass_kg, s.start_yaw_rad) for s in other],
        )
        self.assertTrue(all(not spec.carriers and not spec.scout for spec in first))
        self.assertTrue(any(spec.dimensions_m != base.dimensions_m for spec, base in zip(first, CARGO_SPECS)))


class AdaptiveWarehouseRuntimeTests(unittest.TestCase):
    def test_scene_observation_drives_roles_and_terrain_speed(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            before = world.observe_warehouse_scene()
            self.assertEqual(len(before["terrain"]), 4)
            self.assertEqual(
                {item["kind"] for item in before["terrain"]},
                {"mound", "barrier", "gate_post"},
            )
            self.assertTrue(all(
                item["source"] == "sim_geometry_observation"
                for item in before["cargo"].values()
            ))
            results = world.act_parallel(mission_commands("B"))
            self.assertTrue(all(result.ok for result in results.values()), results)
            phases = [event["phase"] for event in world.warehouse_trace]
            self.assertEqual(phases.count("role_auction_completed"), len(CARGO_SPECS))
            self.assertIn("warehouse_local_path_planned", phases)
            self.assertIn("controller_route_query", phases)
            self.assertNotIn("scout_camera_observed", phases)  # render=False is not a camera measurement
            self.assertIn("scout_camera_unavailable", phases)
            reports = [
                event for event in world.warehouse_trace
                if event["phase"] == "controller_route_query"
            ]
            self.assertTrue(all(len(event["hard_obstacles"]) == 3 for event in reports))
            self.assertTrue(all(event["actor_evidence"] is False for event in reports))
            state = world.warehouse_state()
            self.assertEqual({item["zone"] for item in state["cargo"].values()}, {"B"})
            metrics = state["motion_metrics"]
            self.assertEqual(metrics["empty_lateral_segments"], 0)
            self.assertGreater(metrics["empty_rotate_forward_segments"], 0)
            self.assertLessEqual(metrics["loaded_lateral_ratio"], 0.35)
            lateral_moves = [
                event for event in world.warehouse_trace
                if event.get("movement_mode") == "mecanum_lateral"
                and "payload_start" in event
                and not event["phase"].endswith("_complete")
            ]
            self.assertTrue(all(
                abs(float(event["payload_target"]) - float(event["payload_start"]))
                <= 0.081
                for event in lateral_moves
            ))
        finally:
            world.close()

    def test_complex_terrain_exists_as_collision_geometry(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            kinds = {item.terrain_id: item.kind for item in world.warehouse_terrain_specs}
            self.assertEqual(len(kinds), 4)
            for terrain_id, kind in kinds.items():
                geom_id = mujoco.mj_name2id(
                    world.model, mujoco.mjtObj.mjOBJ_GEOM, terrain_id,
                )
                self.assertGreaterEqual(geom_id, 0)
                self.assertNotEqual(int(world.model.geom_contype[geom_id]), 0)
                expected = mujoco.mjtGeom.mjGEOM_ELLIPSOID if kind == "mound" else mujoco.mjtGeom.mjGEOM_BOX
                self.assertEqual(int(world.model.geom_type[geom_id]), int(expected))
        finally:
            world.close()

    def test_goal_change_during_motion_replans_before_release(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        changed = False

        def retarget_after_leg_starts() -> None:
            nonlocal changed
            if not changed and any(
                event["phase"] == "warehouse_route_leg_started"
                for event in world.warehouse_trace
            ):
                result = world.request_warehouse_goal_update(
                    "C", revision=2, reason="moving_goal_test",
                )
                self.assertTrue(result["accepted"])
                changed = True

        world.frame_callback = retarget_after_leg_starts
        try:
            results = world.act_parallel(mission_commands("B"))
            self.assertTrue(all(result.ok for result in results.values()), results)
            state = world.warehouse_state()
            self.assertEqual(state["current_goal"]["destination_zone"], "C")
            self.assertEqual({item["zone"] for item in state["cargo"].values()}, {"C"})
            self.assertTrue(any(event["phase"] == "route_replanned" for event in world.warehouse_trace))
            delivered = [event for event in world.warehouse_trace if event["phase"] == "cargo_delivered"]
            self.assertTrue(delivered)
            self.assertTrue(all(event["destination_zone"] == "C" for event in delivered))
            metrics = state["motion_metrics"]
            self.assertEqual(metrics["empty_lateral_segments"], 0)
            self.assertLessEqual(metrics["loaded_lateral_ratio"], 0.35)
        finally:
            world.frame_callback = None
            world.close()

    def test_invalid_and_stale_goal_updates_are_rejected(self):
        world = MultiMasterPiProductionV2(seed=3, render=False)
        try:
            with self.assertRaisesRegex(ValueError, "RETARGET_REJECTED"):
                world.request_warehouse_goal_update("Z", revision=1)
            accepted = world.request_warehouse_goal_update("B", revision=2)
            stale = world.request_warehouse_goal_update("C", revision=2)
            self.assertTrue(accepted["accepted"])
            self.assertFalse(stale["accepted"])
            self.assertEqual(stale["reason"], "STALE_GOAL_REVISION")
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
