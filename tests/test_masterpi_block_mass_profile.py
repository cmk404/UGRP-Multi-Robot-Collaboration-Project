import unittest

from sim.masterpi_dynamics_v2 import (
    MasterPiDynamicsV2, PROVISIONAL_TARGET_BLOCK_MASS_KG, TARGET_BLOCK_SIDE_M,
)


class MasterPiBlockMassProfileTests(unittest.TestCase):
    def test_default_target_is_lightweight_foam_not_old_50g_placeholder(self):
        self.assertEqual(PROVISIONAL_TARGET_BLOCK_MASS_KG, 0.005)
        density = PROVISIONAL_TARGET_BLOCK_MASS_KG / (TARGET_BLOCK_SIDE_M ** 3)
        self.assertLess(density, 500.0)

    def test_default_world_uses_provisional_mass_without_claiming_validation(self):
        world = MasterPiDynamicsV2(seed=1, render=False, use_calibration_manifest=False)
        try:
            self.assertEqual(world.physical_params["block_mass_kg"], PROVISIONAL_TARGET_BLOCK_MASS_KG)
            self.assertIn("UNCALIBRATED", world.calibration_status)
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
