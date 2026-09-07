from __future__ import annotations

import math
import unittest
from dataclasses import replace

import mujoco
import numpy as np

from sim.crew_navigation import DynamicPeerYield, ForwardPathController
from sim.masterpi_dynamics_v2 import FORWARD_PATTERN, YAW_LEFT_PATTERN
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.solo_cargo_task import SoloCargoTask
from sim.warehouse_mission import cargo_pose
from sim.warehouse_observation import observe_robots_scan


class DynamicPeerYieldTests(unittest.TestCase):
    def test_later_task_yields_at_crossing_while_earlier_task_keeps_right_of_way(self):
        earlier = DynamicPeerYield("r2", 7.71)
        later = DynamicPeerYield("r1", 10.01)
        forward = np.ones(4) * 0.5
        earlier_peer = [("r2", (0.20, 0.0), np.pi / 2, forward, earlier.priority)]
        later_peer = [("r1", (0.0, -0.20), 0.0, forward, later.priority)]
        self.assertTrue(later.should_yield((0.0, -0.20), 0.0, forward, earlier_peer))
        self.assertFalse(earlier.should_yield((0.20, 0.0), np.pi / 2, forward, later_peer))

    def test_stationary_peer_off_route_does_not_stop_forward_motion(self):
        traffic = DynamicPeerYield("r2", 5.0)
        peer = [("r1", (0.0, 0.7), 0.0, np.zeros(4), (1.0, "r1"))]
        self.assertFalse(traffic.should_yield((0.0, 0.0), 0.0, np.ones(4) * 0.5, peer))

    def test_stationary_or_completed_peer_ahead_has_right_of_way(self):
        traffic = DynamicPeerYield("r2", 5.0)
        stationary_priority = (-np.inf, "r1")
        peer = [("r1", (0.34, 0.0), 0.0, np.zeros(4), stationary_priority)]
        self.assertTrue(traffic.should_yield(
            (0.0, 0.0), 0.0, np.ones(4) * 0.5, peer
        ))

    def test_stationary_peer_beside_goal_anchor_does_not_block_completion(self):
        traffic = DynamicPeerYield("r2", 5.0)
        stationary_priority = (-np.inf, "r1")
        peer = [("r1", (0.0, 0.30), 0.0, np.zeros(4), stationary_priority)]
        # A zero chassis command at the anchor cannot collide with the peer.
        self.assertFalse(traffic.should_yield(
            (0.0, 0.0), 0.0, np.zeros(4), peer
        ))

    def test_replan_is_delayed_and_rate_limited(self):
        traffic = DynamicPeerYield("r2", 5.0)
        self.assertFalse(traffic.note(True, 1.0))
        self.assertFalse(traffic.note(True, 1.44))
        self.assertTrue(traffic.note(True, 1.46))
        self.assertFalse(traffic.note(True, 2.0))
        self.assertTrue(traffic.note(True, 2.37))
        self.assertFalse(traffic.note(False, 2.38))
        self.assertIsNone(traffic.blocked_since)

    def test_final_yaw_turn_stays_above_static_friction_without_anchor_orbit(self):
        controller = ForwardPathController([(0.0, 0.0)], final_yaw=0.0,
                                           tolerance_m=.035)
        controller.update((0.0, 0.0), 0.0, .02)
        for _ in range(31):
            controller.update((-.0318, .0157), math.radians(2.5), .02)
        command = controller.update((-.0318, .0157), math.radians(2.5), .02)
        self.assertEqual(controller.state, "FINAL_YAW")
        self.assertGreaterEqual(abs(float(np.dot(command, YAW_LEFT_PATTERN) / 4)), .08-1e-9)
        self.assertAlmostEqual(float(np.dot(command, FORWARD_PATTERN) / 4), 0.0)

    def test_final_anchor_correction_stays_forward_and_above_static_friction(self):
        controller = ForwardPathController([(0.0, 0.0)], final_yaw=0.0,
                                           tolerance_m=.035)
        controller.update((0.0, 0.0), 0.0, .02)
        for _ in range(31):
            controller.update((-.036, .0), 0.0, .02)
        command = controller.update((-.036, .0), 0.0, .02)
        self.assertEqual(controller.state, "FINAL_YAW")
        self.assertGreaterEqual(float(np.dot(command, FORWARD_PATTERN) / 4), .18-1e-9)
        self.assertAlmostEqual(float(np.dot(command, YAW_LEFT_PATTERN) / 4), 0.0)

    def test_final_anchor_behind_uses_bounded_pose_correction(self):
        controller = ForwardPathController([(0.0, 0.0)], final_yaw=0.0,
                                           tolerance_m=.035)
        controller.update((0.0, 0.0), 0.0, .02)
        position = .04
        for _ in range(31):
            controller.update((position, 0.0), 0.0, .02)
        for _ in range(20):
            command = controller.update((position, 0.0), 0.0, .02)
            position += float(np.dot(command, FORWARD_PATTERN) / 4) * .02
            if controller.done:
                break
        self.assertTrue(controller.done)
        self.assertLessEqual(abs(position), .035)


class SoloTrafficReplayTests(unittest.TestCase):
    def assert_replay_succeeds(self, seed, schedule):
        world = MultiMasterPiProductionV2(seed=seed, warehouse_layout="mixed", render=True)
        tasks = {}
        next_assignment = 0
        peer_contact_ticks = 0
        assigned_ids = tuple(item[1] for item in schedule)
        try:
            pending = {s.cargo_id for s in world.warehouse_specs}
            world._sync_warehouse_goal_specs(sorted(pending), "B")
            for rid in world.robot_ids:
                observe_robots_scan(world, (rid,), pending_ids=pending, sensor_seed=seed)
            last_start=max(item[0] for item in schedule)
            while float(world.data.time) < last_start+90.02:
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
                    self.assertEqual(tasks[rid].navigator.tolerance_m, .035)
                    self.assertLessEqual(tasks[rid].navigator.final_anchor_tolerance_m, .01)
                    next_assignment += 1
                for task in tasks.values():
                    task.before_step()
                world._physics_step_for(world.robot("r1"))
                for contact in world.data.contact:
                    names = [mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or ""
                             for g in (contact.geom1, contact.geom2)]
                    if all(any(n.startswith(rid+"__") for n in names) for rid in assigned_ids):
                        peer_contact_ticks += 1
                        break
                if len(tasks) == len(schedule) and all(task.done for task in tasks.values()):
                    break
            self.assertEqual(peer_contact_ticks, 0)
            self.assertEqual(set(tasks), {item[1] for item in schedule})
            for task in tasks.values():
                self.assertTrue(task.ok, task.reason)
                self.assertLess(float(world.data.time) - task.started, 90.0)
        finally:
            world.close()

    def test_seed37_preflight_concurrent_approaches_finish_without_peer_contact(self):
        self.assert_replay_succeeds(37, (
            (7.71, "r2", "small_box_01"), (10.01, "r1", "small_box_02"),
        ))

    def test_seed23_stationary_unassigned_peer_does_not_stall_replanning(self):
        self.assert_replay_succeeds(23, (
            (7.71, "r3", "small_box_01"), (15.55, "r1", "small_box_02"),
        ))


if __name__ == "__main__":
    unittest.main()
