import unittest
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.warehouse_mission import CARGO_SPECS, warehouse_manifest

class MixedFixtureTests(unittest.TestCase):
    def test_opt_in_scene_has_one_joint_two_solo_real_bodies(self):
        w=MultiMasterPiProductionV2(warehouse_layout="mixed",render=False)
        try:
            self.assertEqual(sorted(s.required_carriers for s in w.warehouse_specs),[1,1,2])
            for s in w.warehouse_specs:
                bid=w.warehouse_body_ids[s.cargo_id]
                self.assertGreater(bid,0)
                self.assertAlmostEqual(float(w.model.body_mass[bid]),s.mass_kg,places=5)
                self.assertEqual(len([k for k in w.warehouse_weld_ids if k[0]==s.cargo_id]),3)
            self.assertEqual(sorted(c['required_carriers'] for c in warehouse_manifest(w.warehouse_specs)['cargo']),[1,1,2])
            self.assertEqual([s.required_carriers for s in CARGO_SPECS],[2,2,2])
        finally:w.close()

if __name__=="__main__":unittest.main()
