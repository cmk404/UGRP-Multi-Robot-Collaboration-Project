import unittest
from types import SimpleNamespace
from unittest.mock import patch

import sim.warehouse_research as research


class FakeWorld:
    def __init__(self):
        self.seed = 7
        self.data = object()
        self.model = object()
        self.controllers = {}
        self.warehouse_spec_by_id = {"crate-1": SimpleNamespace(cargo_id="crate-1")}
        self.warehouse_zone_positions = {"crate-1": {"A": (0, 0, 0), "B": (1, 0, 0)}}
        self.replaced = []
        self.transports = []
        self.warehouse_status = "NEGOTIATING"
        self.warehouse_trace = []
        self.delivered = False

    def warehouse_state(self):
        return {"cargo": {"crate-1": {"evaluation": {"success": self.delivered}, "stable": self.delivered}}}

    def _warehouse_zone_for_position(self, cid, position):
        return "A"

    def _replace_warehouse_spec(self, spec):
        self.replaced.append(spec)

    def _transport_warehouse_cargo(self, spec, route):
        self.transports.append((spec, route))
        self.delivered = True

    def _record_warehouse_phase(self, *args, **kwargs):
        self.warehouse_trace.append((args, kwargs))

    def __getattr__(self, name):
        if "auction" in name:
            raise AssertionError("central auction must never be called")
        raise AttributeError(name)


def episode(revision=1):
    return {"id": "ep", "condition": "llm_peer_comm", "destination": "B",
            "revision": revision, "cargo_ids": ["crate-1"], "attempts": 0,
            "used_decisions": set(), "disabled_carrier": None,
            "observations": {rid: {"cargo": [{"cargo_id": "crate-1"}],
                                    "observation_id": f"obs-{rid}", "revision": revision}
                             for rid in research.ROBOT_IDS}}


def proposals(revision=1, roster=None):
    roster = roster or {"carrier_0": "r1", "carrier_1": "r2", "scout": "r3"}
    return {rid: {"robot_id": rid, "role": next(role for role, who in roster.items() if who == rid),
                  "cargo_id": "crate-1", "destination": "B", "revision": revision,
                  "observation_id": f"obs-{rid}", "assignments": roster,
                  "message": f"{rid} proposes its local role"}
                for rid in research.ROBOT_IDS}


class WarehouseResearchTests(unittest.TestCase):
    def test_local_identity_memory_survives_occlusion_without_stale_geometry_or_peer_leak(self):
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            status = research.dispatch(world, "r1", {"operation": "begin", "source_zone": "A", "destination_zone": "B"})
            episode_id = status["episode_id"]
            seen = {"cargo": [{"cargo_id": "oak_plank_01", "label_color": "red", "range_m": 1.2, "bearing_rad": .2}]}
            with patch("sim.warehouse_observation.observe_robot_scan", return_value=seen):
                research.dispatch(world, "r1", {"operation": "observe", "episode_id": episode_id, "sequence": 0})
            with patch("sim.warehouse_observation.observe_robot_scan", return_value={"cargo": []}):
                remembered = research.dispatch(world, "r1", {"operation": "observe", "episode_id": episode_id, "sequence": 1})
                other = research.dispatch(world, "r2", {"operation": "observe", "episode_id": episode_id, "sequence": 1})
            self.assertEqual(remembered["cargo"][0]["cargo_id"], "oak_plank_01")
            self.assertFalse(remembered["cargo"][0]["current_view"])
            self.assertIsNone(remembered["cargo"][0]["range_m"])
            self.assertIsNone(remembered["cargo"][0]["bearing_rad"])
            self.assertEqual(other["cargo"], [])
            world.reset(11)
            with self.assertRaisesRegex(ValueError, "STALE_EPISODE"):
                research.dispatch(world, "r1", {"operation": "observe", "episode_id": episode_id, "sequence": 2})
        finally:
            world.close()

    def test_mixed_observation_sequences_fail_before_transport(self):
        world = FakeWorld()
        ep = episode()
        for index, obs in enumerate(ep["observations"].values()):
            obs["sequence"] = index
        with self.assertRaisesRegex(ValueError, "MIXED_OBSERVATION_SEQUENCES"):
            research._execute(world, ep, {"proposals": proposals()})
        self.assertEqual(world.transports, [])

    def test_referee_executes_authored_assignments_without_auction(self):
        world = FakeWorld()
        ep = episode()
        roster = {"carrier_0": "r3", "carrier_1": "r1", "scout": "r2"}
        with patch.object(research, "cargo_pose", return_value={"position": (0, 0, 0)}), \
             patch.object(research, "warehouse_route_zones", return_value=("A", "B")), \
             patch.object(research, "replace", side_effect=lambda spec, **kw: SimpleNamespace(**kw)):
            result = research._execute(world, ep, {"proposals": proposals(roster=roster)})
        self.assertTrue(result["ok"])
        self.assertEqual(result["assignments"], roster)
        self.assertEqual(world.replaced[0].carriers, ("r3", "r1"))
        self.assertEqual(world.replaced[0].scout, "r2")
        self.assertEqual(len(world.transports), 1)

    def test_stale_proposals_are_rejected_before_transport(self):
        world = FakeWorld()
        ep = episode(revision=2)
        result = research._execute(world, ep, {"proposals": proposals(revision=1)})
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "AGREEMENT_REJECTED")
        self.assertEqual(world.transports, [])

    def test_replayed_plan_is_rejected_before_second_transport(self):
        world = FakeWorld()
        ep = episode()
        req = {"proposals": proposals()}
        with patch.object(research, "cargo_pose", return_value={"position": (0, 0, 0)}), \
             patch.object(research, "warehouse_route_zones", return_value=("A", "B")), \
             patch.object(research, "replace", side_effect=lambda spec, **kw: SimpleNamespace(**kw)):
            first = research._execute(world, ep, req)
            ep["observations"] = episode()["observations"]
            with self.assertRaisesRegex(ValueError, "REPLAYED_DECISIONS"):
                research._execute(world, ep, req)
        self.assertTrue(first["ok"])
        self.assertEqual(len(world.transports), 1)


if __name__ == "__main__":
    unittest.main()
