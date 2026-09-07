import unittest
from dataclasses import replace
from harness.task_recovery import RecoveryEvent,measurement_confidence,handling_decision
from sim.solo_cargo_task import SoloCargoTask
from sim.multi_masterpi_production import MultiMasterPiProductionV2

class RecoveryTests(unittest.TestCase):
    def test_joint_requires_only_its_two_participants(self):
        e=RecoveryEvent(("r1","r2"),"obstacle",{})
        self.assertIsNone(e.submit("r1",e.event_id,"replan"))
        with self.assertRaises(ValueError):e.submit("r3",e.event_id,"replan")
        self.assertEqual(e.submit("r2",e.event_id,"replan"),"replan")
        with self.assertRaises(ValueError):e.submit("r1",e.event_id,"resume")

    def test_uncertain_or_old_measurement_cannot_authorize_motion(self):
        self.assertFalse(measurement_confidence({"current_view":False,"range_m":1.})["usable_for_motion"])
        self.assertFalse(measurement_confidence({"current_view":True,"range_m":1.,"confidence":.95},age_s=3.)["usable_for_motion"])
        self.assertEqual(handling_decision()["action"],"observe")
        self.assertEqual(handling_decision(width_m=.03,mass_kg=.09,mass_std_kg=.02)["action"],"request_help")
        self.assertEqual(handling_decision(solo_failure="SOLO_LIFT_FAILED")["action"],"request_help")

    def test_pause_while_held_preserves_grasp_and_peer_can_move(self):
        import numpy as np
        w=MultiMasterPiProductionV2(warehouse_layout="mixed",render=False)
        try:
            s=replace(w.warehouse_spec_by_id['small_box_01'],carriers=('r3',));w._replace_warehouse_spec(s)
            task=SoloCargoTask(w,s,'r3',('A','B'))
            while task.state!="CARRY" and not task.done:
                task.before_step();w._physics_step_for(w.robot('r1'))
            self.assertFalse(task.done,task.reason)
            task.pause('event1');initial=w.robot('r1').base_xyz().copy();phase=task.state
            for _ in range(500):
                task.before_step();w.robot('r1').set_motor_commands(np.ones(4)*.4);w._physics_step_for(w.robot('r1'))
            self.assertGreater(w.robot('r1').base_xyz()[0]-initial[0],.08)
            self.assertTrue(w._cargo_constraint_active(s.cargo_id,'r3'))
            self.assertEqual(task.state,phase)
            with self.assertRaises(ValueError):task.recover('old','resume')
            task.recover('event1','resume');w.robot('r1').set_motor_commands(np.zeros(4))
            self.assertFalse(task.paused)
        finally:w.close()

if __name__=='__main__':unittest.main()
