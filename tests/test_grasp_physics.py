import unittest
from sim.grasp_physics import PhysicsGraspWorld, scripted_contact_grasp
from sim.rl_skill import grasp_completion

class GraspPhysicsTests(unittest.TestCase):
    def test_contact_world_has_no_pin_attach_helper(self):
        w=PhysicsGraspWorld(seed=1, render=False)
        try:
            self.assertFalse(hasattr(w, '_pin_block_to_gripper'))
        finally:
            w.close()

    def test_scripted_grasp_lifts_by_bilateral_contact(self):
        w,trace=scripted_contact_grasp(seed=4)
        try:
            s=w.state()
            self.assertTrue(s['left_contact'])
            self.assertTrue(s['right_contact'])
            self.assertTrue(s['lifted'])
            self.assertTrue(s['stable'])
            self.assertGreater(s['red_xyz'][2], .30)
        finally:
            w.close()

    def test_strict_completion_requires_stable_bilateral_lift(self):
        self.assertTrue(grasp_completion({
            "stable": True, "bilateral_contact": True, "lifted": True,
            "red_xyz": [0, 0, .3],
        }))
        self.assertFalse(grasp_completion({
            "stable": True, "bilateral_contact": False, "lifted": True,
            "red_xyz": [0, 0, .3],
        }))

    def test_completed_grasp_rejects_disruptive_repick(self):
        w, _ = scripted_contact_grasp(seed=4)
        try:
            before = w.state()
            r = w.act("pick")
            after = w.state()
            self.assertFalse(r.ok)
            self.assertIn("already stably grasped", r.reason)
            self.assertTrue(after["stable"])
            self.assertTrue(after["bilateral_contact"])
            self.assertAlmostEqual(before["red_xyz"][2], after["red_xyz"][2], places=3)
        finally:
            w.close()

if __name__ == '__main__': unittest.main()
