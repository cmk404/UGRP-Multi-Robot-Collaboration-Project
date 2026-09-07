"""A simulated wrist force sensor; never reads the cargo body's mass/pose."""
from collections import deque
import math
import numpy as np


class WristLoadSensor:
    def __init__(self, max_samples=200, minimum_samples=30):
        self.samples=deque(maxlen=max_samples)
        self.minimum_samples=minimum_samples

    def sample(self, world, robot_id):
        import mujoco
        mujoco.mj_rnePostConstraint(world.model,world.data)
        # Equivalent virtual six-axis sensor at this robot's gripper. During
        # a stopped hold, external downward wrist force estimates supported load.
        robot=world.robot(robot_id)
        force_z=float(world.data.cfrc_ext[robot.gripper_bid,5])
        if math.isfinite(force_z):self.samples.append(max(0.,-force_z))

    def estimate(self):
        if len(self.samples)<self.minimum_samples:
            return {"available":False,"reason":"INSUFFICIENT_STATIONARY_LOAD_SAMPLES",
                    "source":"sim_virtual_wrist_wrench"}
        values=np.asarray(self.samples)
        median=float(np.median(values))
        sigma=max(.02,float(1.4826*np.median(np.abs(values-median))))
        return {"available":True,"mass_kg":median/9.81,"mass_std_kg":sigma/9.81,
                "samples":len(values),"source":"sim_virtual_wrist_wrench",
                "real_sensor_calibrated":False}
