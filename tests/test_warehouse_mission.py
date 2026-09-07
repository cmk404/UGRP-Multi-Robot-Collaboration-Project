from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import mujoco

from harness import team_bus
from harness.mission_language import parse_transfer_command
from scripts import sim_actions
from sim.multi_masterpi_production import MultiMasterPiProductionV2, build_multi_robot_xml
from sim.warehouse_mission import CARGO_SPECS, WAREHOUSE_ZONES, cargo_specs_for_seed


class WarehouseLanguageTests(unittest.TestCase):
    def test_a_to_b_and_zone_colors_compile_to_same_all_cargo_route(self):
        cases = (
            "A구역의 모든 짐을 B구역으로 옮겨",
            "파란 구역 짐 전부 초록 구역으로 운반해",
            "loading zone A의 all cargo를 delivery zone B로 move",
        )
        parsed = [parse_transfer_command(text) for text in cases]
        self.assertTrue(all(item.kind == "WAREHOUSE_TRANSFER" for item in parsed))
        self.assertTrue(all(item.source.id == "A" for item in parsed))
        self.assertTrue(all(item.destination.id == "B" for item in parsed))
        self.assertTrue(all(item.selector.mode == "all" for item in parsed))
        self.assertEqual({item.mission_id for item in parsed}, {parsed[0].mission_id})

    def test_type_selector_does_not_confuse_zone_color(self):
        mission = parse_transfer_command("파란 구역의 파이프 전부를 초록 구역으로 옮겨")
        self.assertEqual(mission.source.id, "A")
        self.assertEqual(mission.destination.id, "B")
        self.assertEqual(mission.selector.mode, "type")
        self.assertEqual(mission.selector.object_type, "pipe")

    def test_observe_word_does_not_turn_all_cargo_into_pipe_selector(self):
        mission = parse_transfer_command(
            "A구역의 모든 짐을 C구역으로 옮겨줘. 물체와 지형을 관찰하고 협업해."
        )
        self.assertEqual(mission.selector.mode, "all")
        self.assertIsNone(mission.selector.object_type)

    def test_a_to_c_expands_through_b_checkpoint(self):
        mission = parse_transfer_command("A구역의 모든 짐을 C구역으로 옮겨")
        self.assertEqual(mission.source.id, "A")
        self.assertEqual(mission.destination.id, "C")
        self.assertEqual(tuple(zone.id for zone in mission.route), ("A", "B", "C"))
        explicit = parse_transfer_command("A구역의 짐을 B구역을 거쳐 C구역으로 옮겨")
        self.assertEqual(explicit.mission_id, mission.mission_id)

    def test_missing_or_identical_zone_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "AMBIGUOUS_ZONE"):
            parse_transfer_command("A구역의 모든 짐을 옮겨")
        with self.assertRaisesRegex(ValueError, "INVALID_ROUTE"):
            parse_transfer_command("A구역의 모든 짐을 A구역으로 옮겨")

    def test_team_bus_preserves_one_canonical_intent_across_all_wakes(self):
        intent = parse_transfer_command("A구역의 모든 짐을 B구역으로 옮겨").as_dict()
        with tempfile.TemporaryDirectory() as td, patch.dict("os.environ", {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            posted = team_bus.post_operator_chat(
                "A구역의 모든 짐을 B구역으로 옮겨",
                namespace="sim", intent_override=intent,
            )
            self.assertEqual(posted["chat"]["intent"], intent)
            self.assertTrue(all(w["intent"] == intent for w in posted["wakeups"]))
            round_id = posted["consensus_round"]["id"]
            committed = None
            for rid in team_bus.ROBOT_IDS:
                committed = team_bus.submit_consensus_proposal(
                    round_id, rid, {"ready": True}, namespace="sim",
                )
            self.assertTrue(committed["committed"])
            self.assertTrue(
                all(w["intent"] == intent for w in committed["execution_wakeups"])
            )


class WarehouseWorldTests(unittest.TestCase):
    def test_seed_randomizes_metric_layout_inside_fixed_safety_lanes(self):
        layouts = []
        for seed in range(8):
            specs = cargo_specs_for_seed(seed)
            layouts.append(tuple(
                (spec.cargo_id, spec.start_xyz[:2], spec.goal_xyz[:2])
                for spec in specs
            ))
            for spec, nominal in zip(specs, CARGO_SPECS):
                self.assertLessEqual(
                    abs(spec.start_xyz[0] - WAREHOUSE_ZONES["A"].center_xy[0]), 0.036,
                )
                self.assertLessEqual(abs(spec.start_xyz[1] - nominal.start_xyz[1]), 0.046)
                self.assertLessEqual(
                    abs(spec.goal_xyz[0] - WAREHOUSE_ZONES["B"].center_xy[0]), 0.036,
                )
                self.assertLessEqual(abs(spec.goal_xyz[1] - nominal.goal_xyz[1]), 0.046)
        self.assertEqual(len(set(layouts)), len(layouts))

    def test_scene_contains_three_colored_zones_and_three_actual_cargo_bodies(self):
        model = mujoco.MjModel.from_xml_string(build_multi_robot_xml())
        self.assertEqual(set(WAREHOUSE_ZONES), {"A", "B", "C"})
        self.assertEqual(
            {zone.color for zone in WAREHOUSE_ZONES.values()},
            {"blue", "green", "yellow"},
        )
        for zone_id in WAREHOUSE_ZONES:
            self.assertGreaterEqual(
                mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_GEOM,
                    f"warehouse_zone_{zone_id.lower()}",
                ),
                0,
            )
        for spec in CARGO_SPECS:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, spec.body_name)
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spec.joint_name)
            self.assertGreaterEqual(bid, 0)
            self.assertGreaterEqual(jid, 0)
            self.assertGreater(spec.mass_kg, 0.0)
            self.assertGreater(max(spec.dimensions_m) / min(spec.dimensions_m), 2.0)

    def test_cargo_has_no_artificial_handle_and_welds_target_real_body(self):
        model = mujoco.MjModel.from_xml_string(build_multi_robot_xml())
        names = []
        for object_type, count in (
            (mujoco.mjtObj.mjOBJ_BODY, model.nbody),
            (mujoco.mjtObj.mjOBJ_GEOM, model.ngeom),
        ):
            names.extend(
                mujoco.mj_id2name(model, object_type, index) or ""
                for index in range(count)
            )
        self.assertFalse(
            [name for name in names if "warehouse_" in name and "handle" in name],
            names,
        )
        for spec in CARGO_SPECS:
            body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, spec.body_name,
            )
            main_geom = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, f"{spec.body_name}_geom",
            )
            self.assertGreaterEqual(main_geom, 0)
            self.assertEqual(spec.carriers, ())
            self.assertEqual(spec.scout, "")
            for rid in ("r1", "r2", "r3"):
                weld_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_EQUALITY,
                    f"{rid}__{spec.cargo_id}_grasp",
                )
                self.assertEqual(int(model.eq_obj2id[weld_id]), body_id)

    def test_single_robot_cannot_start_all_cargo_transfer(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            before = {
                key: tuple(value["position"])
                for key, value in world.warehouse_state()["cargo"].items()
            }
            result = world.act(
                "r1", "team_zone_transfer", mission_id="warehouse_a_to_b_test",
                source_zone="A", destination_zone="B", selector="all",
            )
            after = {
                key: tuple(value["position"])
                for key, value in world.warehouse_state()["cargo"].items()
            }
            self.assertFalse(result.ok)
            self.assertIn("COOPERATIVE_QUORUM_REQUIRED", result.reason)
            self.assertEqual(before, after)
        finally:
            world.close()

    def test_all_cargo_mission_moves_every_shape_with_independent_crew(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            results = world.act_parallel([
                {
                    "robot_id": rid, "action": "team_zone_transfer",
                    "mission_id": "warehouse_a_to_b_test", "source_zone": "A",
                    "destination_zone": "B", "selector": "all",
                }
                for rid in ("r1", "r2", "r3")
            ])
            self.assertTrue(all(result.ok for result in results.values()), results)
            state = world.warehouse_state()
            self.assertEqual(state["status"], "SUCCESS")
            self.assertEqual(state["moved_count"], len(CARGO_SPECS))
            self.assertEqual(state["remaining_ids"], [])
            self.assertTrue(state["success"])
            for cargo in state["cargo"].values():
                self.assertEqual(cargo["zone"], "B")
                self.assertTrue(cargo["stable"])
                self.assertTrue(cargo["evaluation"]["success"])
            delivered = [
                event for event in state["trace"] if event["phase"] == "cargo_delivered"
            ]
            self.assertEqual(len(delivered), len(CARGO_SPECS))
            pairs = {tuple(event["carriers"]) for event in delivered}
            self.assertTrue(all(len(set(pair)) == 2 and set(pair) <= {"r1", "r2", "r3"} for pair in pairs))
            self.assertTrue(all(set(event["carriers"]) | {event["scout"]} == {"r1", "r2", "r3"} for event in delivered))
            self.assertGreater(state["crew_motion"]["concurrency"]["overlap_3way_seconds"], .5)
            self.assertTrue(all(row["distance_m"] > .3 for row in state["crew_motion"]["robots"].values()))
            auctions = [
                event for event in world.warehouse_trace
                if event["phase"] == "role_auction_completed"
            ]
            self.assertEqual(len(auctions), len(CARGO_SPECS))
            self.assertTrue(all(set(event["bids"]) == {"r1", "r2", "r3"} for event in auctions))
            self.assertTrue(all(
                event["transport_mode"] == "DYNAMIC_INWARD_BODY_GRASP_MECANUM"
                for event in delivered
            ))
            facing = [
                event for event in world.warehouse_trace
                if event["phase"] == "carriers_facing_each_other"
            ]
            self.assertEqual(len(facing), len(CARGO_SPECS))
            self.assertTrue(all(float(event["facing_dot"]) <= -0.95 for event in facing))
            staged = [
                event for event in world.warehouse_trace
                if event["phase"] == "robots_staged"
            ]
            self.assertEqual(len(staged), len(CARGO_SPECS))
            self.assertTrue(all(
                all(float(pitch) <= -10.0 for pitch in event["camera_pitch_deg"].values())
                for event in staged
            ))
            self.assertTrue(world.last_parallel_timing["joint_physics"])
        finally:
            world.close()

    def test_seed_25_keeps_block_layout_clear_and_completes_warehouse(self):
        world = MultiMasterPiProductionV2(seed=25, render=False)
        try:
            pipe = world.warehouse_state()["cargo"]["steel_pipe_01"]
            self.assertLess(float(pipe["position"][2]), 0.025)
            results = world.act_parallel([
                {
                    "robot_id": rid,
                    "action": "team_zone_transfer",
                    "mission_id": "warehouse_seed_25_regression",
                    "source_zone": "A",
                    "destination_zone": "B",
                    "selector": "all",
                }
                for rid in ("r1", "r2", "r3")
            ])
            self.assertTrue(
                all(result.ok for result in results.values()),
                {rid: result.reason for rid, result in results.items()},
            )
            self.assertEqual(world.warehouse_state()["remaining_ids"], [])
        finally:
            world.close()

    def test_a_to_c_uses_b_checkpoint_with_direct_body_grasp(self):
        world = MultiMasterPiProductionV2(seed=25, render=False)
        try:
            results = world.act_parallel([
                {
                    "robot_id": rid,
                    "action": "team_zone_transfer",
                    "mission_id": "warehouse_seed_25_a_to_c",
                    "source_zone": "A",
                    "destination_zone": "C",
                    "selector": "all",
                }
                for rid in ("r1", "r2", "r3")
            ])
            self.assertTrue(
                all(result.ok for result in results.values()),
                {rid: result.reason for rid, result in results.items()},
            )
            checkpoints = [
                event for event in world.warehouse_trace
                if event["phase"] == "warehouse_waypoint_reached"
            ]
            self.assertEqual(len(checkpoints), len(CARGO_SPECS) * 2)
            self.assertEqual(
                [(event["zone_id"], event["final"]) for event in checkpoints],
                [("B", False), ("C", True)] * len(CARGO_SPECS),
            )
            self.assertTrue(all(
                item["zone"] == "C"
                for item in world.warehouse_state()["cargo"].values()
            ))
        finally:
            world.close()

    def test_stateful_a_to_b_then_b_to_c_without_reset(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            for source, destination in (("A", "B"), ("B", "C")):
                results = world.act_parallel([
                    {
                        "robot_id": rid,
                        "action": "team_zone_transfer",
                        "mission_id": f"warehouse_stateful_{source}_{destination}",
                        "source_zone": source,
                        "destination_zone": destination,
                        "selector": "all",
                    }
                    for rid in ("r1", "r2", "r3")
                ])
                self.assertTrue(
                    all(result.ok for result in results.values()),
                    {rid: result.reason for rid, result in results.items()},
                )
                self.assertTrue(all(
                    item["zone"] == destination
                    for item in world.warehouse_state()["cargo"].values()
                ))
        finally:
            world.close()

    def test_sim_action_schema_is_one_shared_mission_not_per_cargo(self):
        self.assertIn("team_zone_transfer", sim_actions.ACTIONS)
        self.assertEqual(
            set(sim_actions.ACTION_PARAMETERS["team_zone_transfer"]),
            {"mission_id", "source_zone", "destination_zone", "selector"},
        )
        clean = sim_actions._validate_params(
            "team_zone_transfer",
            {
                "mission_id": "warehouse_a_to_b_test",
                "source_zone": "A", "destination_zone": "B", "selector": "all",
            },
        )
        self.assertEqual(clean["selector"], "all")


if __name__ == "__main__":
    unittest.main()
