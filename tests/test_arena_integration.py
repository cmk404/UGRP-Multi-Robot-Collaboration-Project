from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.warehouse_arena import arena_for_seed
from sim.warehouse_mission import add_warehouse_mission_xml


class ArenaIntegrationTests(unittest.TestCase):
    def test_initial_cargo_is_inside_the_declared_seeded_source(self):
        for seed in (41, 58, 73):
            arena = arena_for_seed(seed)
            source = arena.zones[arena.source_zone]
            for cargo in arena.cargo_specs:
                self.assertEqual(
                    cargo.start_xyz,
                    arena.zone_positions[cargo.cargo_id][arena.source_zone],
                )
                self.assertLessEqual(
                    abs(cargo.start_xyz[0] - source.center_xy[0])
                    + cargo.dimensions_m[0] / 2.0,
                    source.half_extents_xy[0],
                )
                self.assertLessEqual(
                    abs(cargo.start_xyz[1] - source.center_xy[1])
                    + cargo.dimensions_m[1] / 2.0,
                    source.half_extents_xy[1],
                )

    def test_rejected_new_seed_reset_does_not_mutate_or_close_episode(self):
        class Engine:
            closed = False

            def close(self):
                self.closed = True

        world = MultiMasterPiProductionV2.__new__(MultiMasterPiProductionV2)
        world.warehouse_layout = "arena"
        world._warehouse_geometry_seed = 41
        world.seed = 41
        world._mixed_engine = Engine()

        with self.assertRaisesRegex(ValueError, "ARENA_NEW_SEED_REQUIRES_NEW_WORLD"):
            world.reset(58)

        self.assertEqual(world.seed, 41)
        self.assertFalse(world._mixed_engine.closed)

    def test_every_arena_barrier_is_compiled_as_static_collidable_geometry(self):
        arena = arena_for_seed(41)
        root = ET.Element("mujoco")
        ET.SubElement(root, "asset")
        world = ET.SubElement(root, "worldbody")
        ET.SubElement(root, "equality")

        add_warehouse_mission_xml(
            root,
            specs=arena.cargo_specs,
            terrain=arena.terrain,
            zones=arena.zones,
        )

        for barrier in arena.terrain:
            geom = world.find(f"geom[@name='{barrier.terrain_id}']")
            self.assertIsNotNone(geom)
            self.assertEqual(geom.get("type"), "box")
            self.assertEqual(geom.get("mass"), "0")
            self.assertEqual(geom.get("contype"), "1")
            self.assertEqual(geom.get("conaffinity"), "3")
            position = tuple(float(v) for v in geom.get("pos").split())
            size = tuple(float(v) for v in geom.get("size").split())
            self.assertAlmostEqual(position[0], barrier.center_xy[0], places=6)
            self.assertAlmostEqual(position[1], barrier.center_xy[1], places=6)
            self.assertAlmostEqual(position[2], barrier.height_m / 2.0, places=6)
            self.assertAlmostEqual(size[0], barrier.half_extents_xy[0], places=6)
            self.assertAlmostEqual(size[1], barrier.half_extents_xy[1], places=6)
            self.assertAlmostEqual(size[2], barrier.height_m / 2.0, places=6)


if __name__ == "__main__":
    unittest.main()


class ArenaOrientationRegressionTests(unittest.TestCase):
    def test_diagonal_goal_requires_diagonal_payload_orientation(self):
        from sim.warehouse_mission import evaluate_cargo_delivery
        spec = arena_for_seed(58).cargo_specs[0]
        self.assertTrue(evaluate_cargo_delivery({"position": spec.goal_xyz, "yaw": spec.goal_yaw_rad}, spec)["success"])
        self.assertFalse(evaluate_cargo_delivery({"position": spec.goal_xyz, "yaw": 0.}, spec)["success"])

    def test_contact_recovery_uses_robot_local_wheels(self):
        import math
        import numpy as np
        from sim.masterpi_dynamics_v2 import FORWARD_PATTERN
        pattern = MultiMasterPiProductionV2._warehouse_world_velocity_pattern(math.pi/2., (0., 1.))
        np.testing.assert_allclose(pattern, FORWARD_PATTERN, atol=1e-12)
