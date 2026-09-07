from __future__ import annotations

import unittest

import mujoco

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.warehouse_arena import arena_for_seed


class WarehouseCargoFixtureTests(unittest.TestCase):
    def test_default_arena_still_compiles_all_three_cargos(self):
        world = MultiMasterPiProductionV2(warehouse_layout="arena", seed=41, render=False)
        try:
            expected = arena_for_seed(41).cargo_specs
            self.assertEqual(tuple(world.warehouse_spec_by_id), tuple(s.cargo_id for s in expected))
            self.assertEqual(len(world.warehouse_body_ids), 3)
            self.assertTrue(all(body_id >= 0 for body_id in world.warehouse_body_ids.values()))
        finally:
            world.close()

    def test_one_box_fixture_compiles_and_reset_retains_only_selected_body(self):
        original = next(s for s in arena_for_seed(41).cargo_specs if s.cargo_id == "small_box_01")
        world = MultiMasterPiProductionV2(
            warehouse_layout="arena",
            warehouse_cargo_ids=("small_box_01",),
            seed=41,
            render=False,
        )
        try:
            self.assertEqual(tuple(world.warehouse_spec_by_id), ("small_box_01",))
            self.assertEqual(world.warehouse_specs[0].dimensions_m, original.dimensions_m)
            self.assertEqual(tuple(world.warehouse_arena.zone_positions), ("small_box_01",))
            self.assertGreaterEqual(
                mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, original.body_name), 0
            )
            omitted = [s for s in arena_for_seed(41).cargo_specs if s.cargo_id != "small_box_01"]
            self.assertTrue(all(
                mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, spec.body_name) == -1
                for spec in omitted
            ))

            world.reset()

            self.assertEqual(tuple(world.warehouse_spec_by_id), ("small_box_01",))
            self.assertEqual(tuple(world.warehouse_zone_positions), ("small_box_01",))
            self.assertEqual(world.warehouse_specs[0].dimensions_m, original.dimensions_m)
        finally:
            world.close()

    def test_invalid_selected_cargo_ids_are_rejected(self):
        invalid = (
            ((), "must not be empty"),
            (("small_box_01", "small_box_01"), "must be unique"),
            (("missing_box",), "unknown warehouse cargo IDs"),
        )
        for cargo_ids, message in invalid:
            with self.subTest(cargo_ids=cargo_ids):
                with self.assertRaisesRegex(ValueError, message):
                    MultiMasterPiProductionV2(
                        warehouse_layout="arena",
                        warehouse_cargo_ids=cargo_ids,
                        seed=41,
                        render=False,
                    )


if __name__ == "__main__":
    unittest.main()
