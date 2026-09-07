import unittest
from dataclasses import replace
import mujoco
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.solo_cargo_task import SoloCargoTask

class LoadSensorTests(unittest.TestCase):
    def run_load(self, multiplier):
        w=MultiMasterPiProductionV2(warehouse_layout='mixed',render=False)
        s=replace(w.warehouse_spec_by_id['small_box_01'],carriers=('r3',));w._replace_warehouse_spec(s)
        # Physical environment perturbation; sensor code never receives mass.
        b=w.warehouse_body_ids[s.cargo_id]
        w.model.body_mass[b]*=multiplier;w.model.body_inertia[b]*=multiplier
        mujoco.mj_setConst(w.model,w.data)
        t=SoloCargoTask(w,s,'r3',('A','B'))
        while not t.done and not t.help_required and w.data.time<90:
            t.before_step();w._physics_step_for(w.robot('r1'))
        estimate=dict(t.load_estimate);result=t.ok;help_required=t.help_required
        w.close();return estimate,result,help_required

    def test_measured_light_load_can_continue(self):
        estimate,ok,help_required=self.run_load(1)
        self.assertTrue(ok);self.assertFalse(help_required)
        self.assertAlmostEqual(estimate['mass_kg'],.03,delta=.01)

    def test_same_visual_object_with_heavy_load_requests_help(self):
        estimate,ok,help_required=self.run_load(6)
        self.assertFalse(ok);self.assertTrue(help_required)
        self.assertGreater(estimate['mass_kg']-2*estimate['mass_std_kg'],.10)
        self.assertFalse(estimate['real_sensor_calibrated'])

if __name__=='__main__':unittest.main()
