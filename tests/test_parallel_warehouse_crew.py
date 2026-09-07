from dataclasses import replace
import math
import unittest
from unittest.mock import patch

import numpy as np

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.warehouse_crew import WarehouseCrew


class ParallelWarehouseCrewTests(unittest.TestCase):
    def test_one_delayed_robot_does_not_stop_other_two(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        spec = replace(world.warehouse_spec_by_id["steel_pipe_01"], carriers=("r1", "r2"), scout="r3")
        crew = WarehouseCrew(world, spec, ("A", "B"))
        world._warehouse_crew = crew
        try:
            starts = {rid: np.asarray(world.robot(rid).base_xyz()[:2]).copy() for rid in world.robot_ids}
            for rid in world.robot_ids:
                crew.navigate(rid, (starts[rid][0]+.35, starts[rid][1]), "independent_test")
            update = crew.tasks["r1"].update
            deadline = float(world.data.time)+1.2
            with patch.object(crew.tasks["r1"], "update", side_effect=lambda *args:
                              np.zeros(4) if world.data.time < deadline else update(*args)):
                while world.data.time < deadline:
                    world._physics_step_for(world.robot("r1"))
            self.assertLess(np.linalg.norm(world.robot("r1").base_xyz()[:2]-starts["r1"]), .01)
            for rid in ("r2", "r3"):
                self.assertGreater(world.robot(rid).base_xyz()[0]-starts[rid][0], .10)
        finally:
            crew.finish()
            world._warehouse_crew = None
            world.close()

    def test_three_real_chassis_have_overlapping_forward_travel(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        spec = replace(world.warehouse_spec_by_id["steel_pipe_01"], carriers=("r1", "r2"), scout="r3")
        crew = WarehouseCrew(world, spec, ("A", "B"))
        world._warehouse_crew = crew
        try:
            for rid in world.robot_ids:
                start = world.robot(rid).base_xyz()
                crew.navigate(rid, (start[0]+.35, start[1]), "independent_test")
            crew.wait_for(world.robot_ids, max_sim_s=10)
            crew.finish()
            metrics = world._warehouse_crew_metrics
            self.assertGreater(metrics["concurrency"]["overlap_3way_seconds"], .5)
            for row in metrics["robots"].values():
                self.assertGreater(row["forward_travel_m"], .25)
                self.assertLess(row["nonrotating_reverse_travel_m"], .01)
        finally:
            world._warehouse_crew = None
            world.close()

    def test_empty_robot_turns_to_face_a_destination_behind_it(self):
        world = MultiMasterPiProductionV2(seed=11, render=False)
        robot = world.robot("r1")
        start = tuple(robot.base_xyz()[:2])
        try:
            world._warehouse_drive_robot_forward_to("r1", (start[0]-.35, start[1]),
                                                     phase="return_test", max_sim_s=20)
            self.assertLess(math.dist(robot.base_xyz()[:2], (start[0]-.35, start[1])), .05)
            self.assertLess(abs(world._wrap_angle(robot.base_rpy()[2]-math.pi)), .25)
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
