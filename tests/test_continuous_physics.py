import unittest, numpy as np, mujoco
from sim.continuous_physics import ContinuousPhysicsWorld, XML

class ContinuousPhysicsTests(unittest.TestCase):
    def setUp(self): self.w=ContinuousPhysicsWorld(seed=123,render=False)
    def tearDown(self): self.w.close()
    def test_no_mocap_or_attach_cheat(self):
        self.assertNotIn('mocap=',XML); self.assertFalse(hasattr(self.w,'_pin_block_to_gripper')); self.assertNotIn('<weld',XML)
    def test_onboard_camera_is_fixed_to_mast(self):
        cid=mujoco.mj_name2id(self.w.model,mujoco.mjtObj.mjOBJ_CAMERA,'robot_cam'); bid=int(self.w.model.cam_bodyid[cid]); self.assertEqual(mujoco.mj_id2name(self.w.model,mujoco.mjtObj.mjOBJ_BODY,bid),'camera_mast')
    def test_gripper_force_is_bounded(self):
        for name in ('a_left_finger','a_right_finger'):
            aid=mujoco.mj_name2id(self.w.model,mujoco.mjtObj.mjOBJ_ACTUATOR,name); self.assertLessEqual(float(self.w.model.actuator_forcerange[aid,1]),7.5)
    def test_base_motion_is_continuous_and_rate_limited(self):
        xs=[]
        for _ in range(250): self.w.command_base(.25,0,0); self.w.step_physics(1); xs.append(self.w.base_pose()[0])
        dx=np.diff(xs); self.assertGreater(xs[-1]-xs[0],.01); self.assertLess(float(np.max(np.abs(dx))),.004); self.assertLessEqual(float(np.max(np.abs(self.w._cmd_v))),.28)
    def test_stable_requires_real_hold_window(self):
        self.assertFalse(self.w.state()['stable']); self.assertEqual(self.w._stable_steps,0)

if __name__=='__main__': unittest.main()
