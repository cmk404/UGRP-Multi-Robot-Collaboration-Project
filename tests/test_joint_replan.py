from dataclasses import replace
import unittest

import mujoco
import numpy as np

from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.masterpi_production_v2 import SEARCH_POSE
from sim.solo_cargo_task import SoloCargoTask
from sim.warehouse_crew import WarehouseCrew
from sim.warehouse_mission import TerrainSpec, cargo_pose
from sim.warehouse_observation import observe_robots_scan


class _MixedHooks:
    _pause_tick = False

    def service_joint_pause(self):
        pass

    def before_step(self):
        pass

    def after_step(self):
        pass


class JointReplanTests(unittest.TestCase):
    def test_seed23_late_joint_stages_after_two_continuous_solo_deliveries(self):
        world = MultiMasterPiProductionV2(seed=23, warehouse_layout="mixed", render=True)
        schedule = ((7.71, "r3", "small_box_01"), (12.736, "r1", "small_box_02"))
        tasks = {}
        next_assignment = 0
        crew = None
        try:
            pending = {s.cargo_id for s in world.warehouse_specs}
            world._sync_warehouse_goal_specs(sorted(pending), "B")
            for rid in world.robot_ids:
                observe_robots_scan(world, (rid,), pending_ids=pending, sensor_seed=23)
            while float(world.data.time) < 57.88:
                now = float(world.data.time)
                while next_assignment < len(schedule) and now + 1e-9 >= schedule[next_assignment][0]:
                    _, rid, cargo_id = schedule[next_assignment]
                    original = world.warehouse_spec_by_id[cargo_id]
                    pose = cargo_pose(world.data, world.model, original)
                    spec = replace(original, carriers=(rid,), scout="",
                        start_xyz=tuple(pose["position"]),
                        goal_xyz=world.warehouse_zone_positions[cargo_id]["B"])
                    world._replace_warehouse_spec(spec)
                    tasks[rid] = SoloCargoTask(world, spec, rid, ("A", "B"))
                    next_assignment += 1
                for task in tasks.values():
                    task.before_step()
                world._physics_step_for(world.robot("r1"))
            self.assertTrue(all(task.ok for task in tasks.values()),
                            {rid: task.reason for rid, task in tasks.items()})

            original = world.warehouse_spec_by_id["oak_plank_01"]
            pose = cargo_pose(world.data, world.model, original)
            spec = replace(original, carriers=("r1", "r2"), scout="",
                start_xyz=tuple(pose["position"]),
                goal_xyz=world.warehouse_zone_positions[original.cargo_id]["B"])
            world._replace_warehouse_spec(spec)
            world._mixed_engine = _MixedHooks()
            crew = WarehouseCrew(world, spec, ("A", "B"))
            world._warehouse_crew = crew
            precision = world._precision_module()
            transit = {**SEARCH_POSE, 1: precision.GRIPPER_OPEN, 6: 1500}
            world._team_joint_move_servos({rid: transit for rid in spec.carriers}, .55)
            formation = world._warehouse_inward_formation(spec, .16)
            for rid in spec.carriers:
                crew.navigate(rid, (formation[rid]["x"], formation[rid]["y"]),
                              "approach_grip", final_yaw=0.)
            started = float(world.data.time)
            while (not all(crew.tasks[r].done for r in spec.carriers)
                   and world.data.time < started + 90.):
                world._physics_step_for(world.robot("r1"))
            self.assertTrue(all(crew.tasks[r].done for r in spec.carriers), {
                rid: {"xy": list(world.robot(rid).base_xyz()[:2]),
                      "state": crew.tasks[rid].state,
                      "path": crew.tasks[rid].path}
                for rid in spec.carriers})
            self.assertLess(float(world.data.time) - started, 90.)
            self.assertFalse(crew.blocked_since)
        finally:
            if crew is not None:
                crew.finish()
            world._warehouse_crew = None
            world._mixed_engine = None
            world.close()

    def test_planner_peer_clearance_matches_runtime_forecast(self):
        world = MultiMasterPiProductionV2(seed=23, warehouse_layout="mixed", render=False)
        spec = world.warehouse_spec_by_id["oak_plank_01"]
        spec = replace(spec, carriers=("r1", "r2"), scout="",
            start_xyz=tuple(cargo_pose(world.data, world.model, spec)["position"]),
            goal_xyz=world.warehouse_zone_positions[spec.cargo_id]["B"])
        crew = WarehouseCrew(world, spec, ("A", "B"))
        try:
            obstacles = world._warehouse_navigation_observations(
                exclude_robots=(), exclude_robot="r1")
            raw_peer = next(o for o in obstacles if o.terrain_id == "peer:r2")
            self.assertEqual(raw_peer.half_extents_xy, (.12, .12))
            # The actual planned route must remain outside the controller's
            # .34 m forecast radius around the completed carrier anchor.
            r2_xy = world.robot("r2").base_xyz()[:2]
            path = crew.path("r1", (1.05, -1.38), include_peers=True)
            segment_samples = []
            for first, last in zip(path, path[1:]):
                first, last = np.asarray(first), np.asarray(last)
                count = max(2, int(np.linalg.norm(last-first)/.01)+1)
                segment_samples.extend(first+(last-first)*u for u in np.linspace(0., 1., count))
            self.assertTrue(all(
                np.linalg.norm(point-np.asarray(r2_xy)) >= .34-1e-9
                for point in segment_samples
            ), path)
        finally:
            crew.finish()
            world.close()

    def test_seed37_barrier_replan_avoids_legacy_beam_and_finishes_staging(self):
        world = MultiMasterPiProductionV2(seed=37, warehouse_layout="mixed", render=False)
        world._mixed_engine = _MixedHooks()
        spec = world.warehouse_spec_by_id["oak_plank_01"]
        spec = replace(spec, carriers=("r1", "r2"), scout="",
            start_xyz=tuple(cargo_pose(world.data, world.model, spec)["position"]),
            goal_xyz=world.warehouse_zone_positions[spec.cargo_id]["B"])
        world._replace_warehouse_spec(spec)
        crew = WarehouseCrew(world, spec, ("A", "B"))
        world._warehouse_crew = crew
        try:
            formation = world._warehouse_inward_formation(spec, .21)
            for rid in spec.carriers:
                crew.navigate(rid, (formation[rid]["x"], formation[rid]["y"]),
                              "approach_grip", final_yaw=0.)
            while world.data.time < 6.:
                world._physics_step_for(world.robot("r1"))

            gid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_GEOM,
                                    "mixed_event_barrier")
            world.model.geom_pos[gid] = [.72, -.64, .06]
            world.warehouse_terrain_specs = tuple(
                t for t in world.warehouse_terrain_specs
                if t.terrain_id != "mixed_event_barrier") + (TerrainSpec(
                    "mixed_event_barrier", "barrier", (.72, -.64),
                    (.06, .09), .12, False, float("inf")),)
            mujoco.mj_forward(world.model, world.data)
            for rid in spec.carriers:
                old = crew.tasks[rid]
                crew.navigate(rid, old.path[-1], crew.activities[rid],
                              final_yaw=old.final_yaw)

            deadline = float(world.data.time) + 45.
            while (not all(crew.tasks[rid].done for rid in spec.carriers)
                    and world.data.time < deadline):
                world._physics_step_for(world.robot("r1"))

            self.assertTrue(all(crew.tasks[rid].done for rid in spec.carriers), {
                rid: {"state": crew.tasks[rid].state,
                      "target": crew.tasks[rid].target,
                      "xy": list(world.robot(rid).base_xyz()[:2]),
                      "path": crew.tasks[rid].path}
                for rid in spec.carriers})
            replan_paths = [e for e in world.warehouse_trace
                            if e["phase"] == "crew_yield_replanned"]
            self.assertTrue(replan_paths)
            self.assertFalse(crew.blocked_since)
        finally:
            crew.finish()
            world._warehouse_crew = None
            world._mixed_engine = None
            world.close()


if __name__ == "__main__":
    unittest.main()
