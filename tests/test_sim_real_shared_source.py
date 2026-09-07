from __future__ import annotations

import inspect
from pathlib import Path
import threading
import unittest
from unittest import mock

from scripts import robot_actions, sim_actions
from sim.masterpi_production_v2 import ActionResult, MasterPiProductionV2
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim import multi_masterpi_production as multi_world
from sim import real_stack_adapter as adapter
from sim.team_batch import first_command_metadata, use_team_batch


class SimRealSharedSourceTests(unittest.TestCase):
    def test_team_batch_metadata_is_attached_to_first_command_only(self):
        self.assertEqual(first_command_metadata(), {})
        with use_team_batch("sim:cr7", 3):
            self.assertEqual(first_command_metadata(), {
                "team_batch_id": "sim:cr7",
                "team_batch_expected": 3,
            })
            self.assertEqual(first_command_metadata(), {})
        self.assertEqual(first_command_metadata(), {})

    def test_parallel_camera_rendering_uses_one_gl_owner_thread(self):
        """Renderer construction, use and close never cross a CGL thread."""
        calls: list[tuple[str, int, int]] = []

        class FakeRenderer:
            def __init__(self, _model, *, height, width):
                self.height = int(height)
                self.width = int(width)
                self.owner = threading.get_ident()
                calls.append(("create", self.owner, self.owner))

            def update_scene(self, *_args, **_kwargs):
                calls.append(("update", self.owner, threading.get_ident()))

            def render(self):
                calls.append(("render", self.owner, threading.get_ident()))
                return __import__("numpy").zeros(
                    (self.height, self.width, 3), dtype="uint8"
                )

            def close(self):
                calls.append(("close", self.owner, threading.get_ident()))

        with mock.patch.object(multi_world.mujoco, "Renderer", FakeRenderer):
            world = MultiMasterPiProductionV2(seed=2, width=64, height=48, render=True)
            images: dict[str, object] = {}
            errors: list[BaseException] = []
            start = threading.Barrier(3)

            def render(rid: str) -> None:
                try:
                    start.wait(timeout=2.0)
                    images[rid] = world.render_rgb(robot_id=rid)
                except BaseException as exc:
                    errors.append(exc)

            workers = [
                threading.Thread(target=render, args=(rid,))
                for rid in ("r1", "r2", "r3")
            ]
            try:
                for worker in workers:
                    worker.start()
                for worker in workers:
                    worker.join(timeout=5.0)
                self.assertFalse(any(worker.is_alive() for worker in workers))
                self.assertFalse(errors, errors)
                self.assertEqual(set(images), {"r1", "r2", "r3"})
            finally:
                world.close()

        owner_threads = {owner for _kind, owner, _called_on in calls}
        self.assertEqual(len(owner_threads), 1)
        self.assertTrue(calls)
        self.assertTrue(all(owner == called_on for _kind, owner, called_on in calls))

    def test_act_parallel_reports_real_wall_clock_overlap(self):
        world = MultiMasterPiProductionV2(seed=2, render=False)
        entered: set[str] = set()
        lock = threading.Lock()
        both_entered = threading.Event()
        delivered: list[tuple[str, float]] = []

        def fake_act(rid, action, **params):
            with lock:
                entered.add(rid)
                if len(entered) == 2:
                    both_entered.set()
            if not both_entered.wait(timeout=2.0):
                raise AssertionError("parallel peer never entered")
            both_entered.wait(0.02)
            return ActionResult(True, action, "ok", {"robot_id": rid})

        try:
            with mock.patch.object(world, "act", side_effect=fake_act):
                results = world.act_parallel(
                    [
                        {"robot_id": "r1", "action": "track"},
                        {"robot_id": "r2", "action": "track"},
                    ],
                    on_result=lambda rid, _result, timing: delivered.append(
                        (rid, timing["duration_s"])
                    ),
                )
            self.assertEqual(set(results), {"r1", "r2"})
            self.assertEqual({rid for rid, _duration in delivered}, {"r1", "r2"})
            self.assertGreater(world.last_parallel_timing["all_overlap_s"], 0.0)
        finally:
            world.close()

    def test_public_actions_overlap_with_isolated_contextual_handoffs(self):
        """Public multi-robot actions overlap without detector/path cross-talk."""
        world = MultiMasterPiProductionV2(seed=2, render=False)
        barrier = threading.Barrier(2)
        observed: dict[str, tuple[Path, Path, object]] = {}
        errors: list[BaseException] = []

        def fake_approach(robot, *, precision, target_color, **_kwargs):
            # Exercise the same target-detector binding as the real approach
            # entrypoint before both workers are released together.
            adapter.approach_mod.configure_target_detector(precision, target_color)
            barrier.wait(timeout=3.0)
            observed[robot.world.robot_id] = (
                adapter.precision_handoff._plan_path(),
                adapter.carry_handoff_mod._carry_path(),
                adapter.precision_mod._TARGET_DETECTOR.get(),
            )
            return "overlapped"

        def run(rid: str, color: str) -> None:
            try:
                world.act(rid, "approach", target_color=color)
            except BaseException as exc:  # surface barrier failures in the test
                errors.append(exc)

        original_defaults = (
            adapter.precision_mod.CAPTURE_CREEP_SECONDS,
            adapter.precision_mod.CAPTURE_TARGET_NY,
        )
        with mock.patch.object(adapter.approach_mod, "run_approach", side_effect=fake_approach):
            threads = [
                threading.Thread(target=run, args=("r1", "red")),
                threading.Thread(target=run, args=("r2", "blue")),
            ]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(timeout=5.0)
        try:
            self.assertFalse(errors, errors)
            self.assertEqual(set(observed), {"r1", "r2"})
            self.assertEqual(
                observed["r1"][:2],
                (Path("/tmp/ugrp-sim-r1-pick-plan.json"), Path("/tmp/ugrp-sim-r1-carry-handoff.json")),
            )
            self.assertEqual(
                observed["r2"][:2],
                (Path("/tmp/ugrp-sim-r2-pick-plan.json"), Path("/tmp/ugrp-sim-r2-carry-handoff.json")),
            )
            # Red uses the physical default detector; blue gets a contextual
            # alternate detector.  The important property is that blue's
            # binding does not leak into red's worker context.
            self.assertIsNone(observed["r1"][2])
            self.assertIsNotNone(observed["r2"][2])
            self.assertNotEqual(observed["r1"][2], observed["r2"][2])
            self.assertEqual(
                (
                    adapter.precision_mod.CAPTURE_CREEP_SECONDS,
                    adapter.precision_mod.CAPTURE_TARGET_NY,
                ),
                original_defaults,
            )
            self.assertIsNone(adapter.precision_mod._TARGET_DETECTOR.get())
        finally:
            world.close()

    def test_multi_robot_public_approach_and_pick_use_shared_real_controller(self):
        world = MultiMasterPiProductionV2(seed=2, render=False)
        try:
            robot = world.controllers["r1"]
            approach_result = ActionResult(
                True, "approach", "REAL approach sentinel", robot.state()
            )
            pick_result = ActionResult(
                True, "pick", "REAL pick sentinel", robot.state()
            )
            with mock.patch.object(robot, "act", side_effect=[approach_result, pick_result]) as real_act, \
                 mock.patch.object(world, "_geometric_approach") as geometric_approach, \
                 mock.patch.object(world, "_geometric_pick") as geometric_pick:
                got_approach = world.act("r1", "approach", target_color="red")
                robot.pregrasp_color = "red"
                got_pick = world.act("r1", "pick", target_color="red")
            self.assertIs(got_approach, approach_result)
            self.assertIs(got_pick, pick_result)
            self.assertEqual(
                [call.args[0] for call in real_act.call_args_list],
                ["approach", "pick"],
            )
            geometric_approach.assert_not_called()
            geometric_pick.assert_not_called()
        finally:
            world.close()

    def test_carry_search_turns_are_feedback_bounded_not_pulse_counted(self):
        self.assertEqual(
            adapter.carry_search_turn_for_heading_error(-1.0),
            ("rotate-right", adapter.CARRY_SEARCH_COARSE_TURN_SECONDS),
        )
        self.assertEqual(
            adapter.carry_search_turn_for_heading_error(0.20),
            ("rotate-left", adapter.CARRY_SEARCH_FINE_TURN_SECONDS),
        )
        self.assertIsNone(adapter.carry_search_turn_for_heading_error(0.01))

    def test_sim_delivery_uses_shared_carry_approach_policy(self):
        self.assertEqual(adapter.place_mod.CARRY_APPROACH_SPEED, 35)
        self.assertEqual(adapter.place_mod.CARRY_BODY_CENTER_DEADBAND, 0.060)
        self.assertEqual(
            adapter.place_mod.carry_approach_motion(adapter.place_mod.TARGET_RADIUS_MAX_CM + 19.0),
            ("forward", 35, 0.60),
        )

    def test_agent_catalog_keeps_real_contracts_and_sim_team_extensions(self):
        sim_only = {
            "stage_base", "stack_on", "team_tower", "team_beam_transport",
            "team_zone_transfer",
        }
        sim_actions_common = {k: v for k, v in sim_actions.ACTIONS.items() if k not in sim_only}
        sim_params_common = {k: v for k, v in sim_actions.ACTION_PARAMETERS.items() if k not in sim_only}
        sim_contracts_common = {k: v for k, v in sim_actions.CONTRACTS.items() if k not in sim_only}
        self.assertEqual(sim_actions_common, robot_actions.ACTIONS)
        self.assertEqual(sim_params_common, robot_actions.ACTION_PARAMETERS)
        self.assertEqual(sim_contracts_common, robot_actions.CONTRACTS)
        self.assertIn("stage_base", sim_actions.ACTIONS)
        self.assertIn("stack_on", sim_actions.ACTIONS)
        self.assertIn("team_tower", sim_actions.ACTIONS)
        self.assertIn("team_beam_transport", sim_actions.ACTIONS)
        self.assertIn("team_zone_transfer", sim_actions.ACTIONS)
        self.assertEqual(set(sim_actions.ACTION_PARAMETERS["stage_base"]), {"target_color"})
        self.assertEqual(set(sim_actions.ACTION_PARAMETERS["stack_on"]), {"target_color", "destination_color"})
        self.assertEqual(
            set(sim_actions.ACTION_PARAMETERS["team_beam_transport"]),
            {"mission_id", "role"},
        )
        self.assertNotIn("carry", sim_actions.ACTIONS)
        self.assertFalse(any(name.startswith("map_") for name in sim_actions.ACTIONS))
        self.assertFalse(any(
            name != "search_destination" and name.startswith(("search_", "track_", "approach_", "pick_"))
            for name in sim_actions.ACTIONS
        ))

    def test_search_track_and_approach_delegate_to_real_functions(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            with mock.patch.object(adapter.search_mod, "run_search", return_value=0) as real_search:
                stack.search(target_color="red", seconds=1.25)
            self.assertEqual(real_search.call_count, 1)
            self.assertIs(real_search.call_args.args[0], stack.robot)
            self.assertEqual(real_search.call_args.kwargs["target_color"], "red")
            self.assertEqual(real_search.call_args.kwargs["seconds"], 1.25)

            with mock.patch.object(adapter.track_mod, "run_track", return_value=0) as real_track:
                stack.track(target_color="blue", seconds=1.5)
            real_track.assert_called_once_with(
                stack.robot, capture=stack.camera.read, seconds=1.5,
                clock=stack.clock, target_color="blue",
            )

            with mock.patch.object(adapter.approach_mod, "run_approach", return_value=0) as real_approach:
                stack.approach(target_color="yellow")
            real_approach.assert_called_once_with(
                stack.robot,
                precision=adapter.precision_mod,
                target_color="yellow",
                fast_camera_control=adapter.SIM_FAST_CONTINUOUS_APPROACH,
            )
        finally:
            world.close()

    def test_sim_pick_delegates_to_real_pick_function_then_evaluator_only(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            with mock.patch.object(adapter.pick_mod, "run_precision_pick", return_value=0) as real_pick, \
                 mock.patch.object(stack, "_verify_sim_grasp_postcondition") as evaluator:
                stack.pick(target_color="blue")
            real_pick.assert_called_once_with(
                stack.robot, precision=adapter.precision_mod, target_color="blue"
            )
            evaluator.assert_called_once_with("blue")
        finally:
            world.close()

    def test_sim_put_down_delegates_to_real_inverse_pick_path_then_evaluator_only(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            world.grasp_color = "red"
            with mock.patch.object(adapter.put_down_mod, "execute_put_down", return_value="red") as real_put_down, \
                 mock.patch.object(stack.clock, "sleep"):
                stack.put_down()
            real_put_down.assert_called_once_with(stack.robot)
            self.assertIsNone(world.grasp_color)
        finally:
            world.close()

    def test_carry_watchdog_tolerates_one_transient_contact_gap_but_rejects_sustained_loss(self):
        import numpy as np
        from types import SimpleNamespace

        class FakeWorld:
            def __init__(self):
                self.data = SimpleNamespace(time=1.0)
                self.servo_command_pulses = {1: 1500}
                self.physical_params = {}
                self.grasp_color = "red"
                self.contact = False
                self.block = np.asarray([0.0, 0.0, 0.23])
                self.grip = np.asarray([0.0, 0.0, 0.205])
            def _reconcile_grasp_state(self): pass
            def finger_block_contact(self, color): return {"bilateral": self.contact}
            def body_xyz(self, name): return self.block
            def site_xyz(self, name): return self.grip
            def set_motor_commands(self, values): pass

        world = FakeWorld()
        robot = adapter.SimRobot(world, adapter.SimClock(world))
        self.assertTrue(robot.carry_intact("red"))  # first no-contact sample gets grace
        world.data.time += 0.10
        self.assertTrue(robot.carry_intact("red"))
        world.contact = True
        world.data.time += 0.02
        self.assertTrue(robot.carry_intact("red"))  # recovery clears miss timer
        world.contact = False
        world.data.time += 0.01
        self.assertTrue(robot.carry_intact("red"))
        world.data.time += 0.23
        self.assertFalse(robot.carry_intact("red"))

    def test_carry_watchdog_rejects_clear_separation_immediately(self):
        import numpy as np
        from types import SimpleNamespace

        class FakeWorld:
            data = SimpleNamespace(time=1.0)
            servo_command_pulses = {1: 1500}
            physical_params = {}
            grasp_color = "red"
            def _reconcile_grasp_state(self): pass
            def finger_block_contact(self, color): return {"bilateral": False}
            def body_xyz(self, name): return np.asarray([0.0, 0.0, 0.23])
            def site_xyz(self, name): return np.asarray([0.0, 0.0, 0.17])
            def set_motor_commands(self, values): pass

        robot = adapter.SimRobot(FakeWorld(), adapter.SimClock(FakeWorld()))
        # Rebind both robot and clock to one world instance.
        world = FakeWorld(); robot = adapter.SimRobot(world, adapter.SimClock(world))
        self.assertFalse(robot.carry_intact("red"))

    def test_sim_place_evaluator_uses_current_30mm_cube_geometry(self):
        import numpy as np
        from types import SimpleNamespace

        class FakeWorld:
            def __init__(self, red, blue):
                self.points = {"red_block": np.asarray(red, dtype=float), "blue_block": np.asarray(blue, dtype=float)}
                self.grasp_color = "red"
                self.spatial_memory = {"red": {"relation": "HELD"}}
            def body_xyz(self, name): return self.points[name]

        stack = adapter.MigratedRealStack.__new__(adapter.MigratedRealStack)
        stack.world = FakeWorld([0.004, 0.0, 0.045], [0.0, 0.0, 0.015])
        stack._verify_sim_place_postcondition("red", "blue")
        self.assertIsNone(stack.world.grasp_color)
        self.assertEqual(stack.world.spatial_memory["red"]["relation"], "ON_BLUE")

    def test_sim_place_evaluator_rejects_floor_miss_and_large_overhang(self):
        import numpy as np
        class FakeWorld:
            def __init__(self, red):
                self.points = {"red_block": np.asarray(red, dtype=float), "blue_block": np.asarray([0.0, 0.0, 0.015])}
                self.grasp_color = None
                self.spatial_memory = {}
            def body_xyz(self, name): return self.points[name]
        for red in ([0.004, 0.0, 0.015], [0.020, 0.0, 0.045]):
            stack = adapter.MigratedRealStack.__new__(adapter.MigratedRealStack)
            stack.world = FakeWorld(red)
            with self.assertRaises(RuntimeError):
                stack._verify_sim_place_postcondition("red", "blue")

    def test_sim_place_delegates_to_real_place_function_then_evaluator_only(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            with mock.patch.object(adapter.place_mod, "execute_place") as real_place, \
                 mock.patch.object(stack, "_verify_sim_place_postcondition") as evaluator:
                stack.place(target_color="red", destination_color="blue")
            real_place.assert_called_once_with(stack.robot, "red", "blue")
            evaluator.assert_called_once_with("red", "blue")
        finally:
            world.close()

    def test_sim_close_creep_uses_camera_replay_calibration_without_changing_real_default(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            real_default = adapter.precision_mod.CAPTURE_CREEP_SECONDS
            self.assertAlmostEqual(real_default, 0.06, places=6)
            with adapter._patched_modules(stack):
                self.assertAlmostEqual(
                    adapter.precision_mod.CAPTURE_CREEP_SECONDS,
                    adapter.SIM_CAPTURE_CREEP_SECONDS,
                    places=6,
                )
                self.assertLess(adapter.precision_mod.CAPTURE_CREEP_SECONDS, real_default)
            self.assertAlmostEqual(adapter.precision_mod.CAPTURE_CREEP_SECONDS, real_default, places=6)
        finally:
            world.close()

    def test_fast_sim_confirmation_counts_restore_real_defaults(self):
        world = MasterPiProductionV2(seed=2, render=False)
        names = (
            "ACQUIRE_CONFIRMATIONS", "CENTER_CONFIRMATIONS",
            "FINAL_X_CONFIRMATIONS", "FINAL_SAMPLE_COUNT",
            "APPROACH_RANGE_SAMPLE_COUNT", "CAPTURE_SAMPLE_COUNT",
            "VERIFY_FRAME_COUNT",
        )
        defaults = {name: getattr(adapter.precision_mod, name) for name in names}
        try:
            stack = adapter.MigratedRealStack(world)
            with adapter._patched_modules(stack):
                if adapter.SIM_FAST_CAMERA_CONTROL:
                    self.assertEqual(adapter.precision_mod.APPROACH_RANGE_SAMPLE_COUNT, 1)
                    self.assertEqual(adapter.precision_mod.CAPTURE_SAMPLE_COUNT, 1)
                    self.assertEqual(adapter.precision_mod.VERIFY_FRAME_COUNT, 3)
                    self.assertEqual(adapter.precision_mod.FACE_ROUTE_TARGET_ERROR_DEG, 10.0)
                    self.assertEqual(adapter.approach_mod.COARSE_FACE_ACCEPT_DEG, 18.0)
            self.assertEqual(
                {name: getattr(adapter.precision_mod, name) for name in names},
                defaults,
            )
        finally:
            world.close()

    def test_sim_close_creep_progress_threshold_matches_shorter_pulse_without_changing_real(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            real_default = adapter.precision_mod.CAPTURE_CREEP_MIN_PROGRESS_NY
            real_low_progress = adapter.precision_mod.CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES
            real_max_creeps = adapter.precision_mod.CAPTURE_MAX_CREEP_PULSES
            self.assertAlmostEqual(real_default, adapter.precision_mod.CAPTURE_MIN_PROGRESS_NY, places=6)
            self.assertEqual(real_low_progress, 0)
            self.assertEqual(real_max_creeps, 7)
            with adapter._patched_modules(stack):
                self.assertAlmostEqual(
                    adapter.precision_mod.CAPTURE_CREEP_MIN_PROGRESS_NY,
                    adapter.SIM_CAPTURE_CREEP_MIN_PROGRESS_NY,
                    places=6,
                )
                self.assertLess(
                    adapter.precision_mod.CAPTURE_CREEP_MIN_PROGRESS_NY,
                    adapter.precision_mod.CAPTURE_MIN_PROGRESS_NY,
                )
                self.assertEqual(
                    adapter.precision_mod.CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES,
                    adapter.SIM_CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES,
                )
                self.assertEqual(
                    adapter.precision_mod.CAPTURE_MAX_CREEP_PULSES,
                    adapter.SIM_CAPTURE_MAX_CREEP_PULSES,
                )
            self.assertAlmostEqual(adapter.precision_mod.CAPTURE_CREEP_MIN_PROGRESS_NY, real_default, places=6)
            self.assertEqual(adapter.precision_mod.CAPTURE_CREEP_MAX_LOW_PROGRESS_PULSES, real_low_progress)
            self.assertEqual(adapter.precision_mod.CAPTURE_MAX_CREEP_PULSES, real_max_creeps)
        finally:
            world.close()

    def test_sim_dogleg_restore_compensates_twin_yaw_response_without_changing_real_default(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            real_default = adapter.precision_mod.FACE_REPOSITION_RESTORE_TURN_SECONDS
            self.assertAlmostEqual(real_default, adapter.precision_mod.FACE_REPOSITION_TURN_SECONDS, places=6)
            with adapter._patched_modules(stack):
                self.assertAlmostEqual(
                    adapter.precision_mod.FACE_REPOSITION_RESTORE_TURN_SECONDS,
                    adapter.SIM_FACE_REPOSITION_RESTORE_TURN_SECONDS,
                    places=6,
                )
                self.assertGreater(
                    adapter.precision_mod.FACE_REPOSITION_RESTORE_TURN_SECONDS,
                    adapter.precision_mod.FACE_REPOSITION_TURN_SECONDS,
                )
            self.assertAlmostEqual(
                adapter.precision_mod.FACE_REPOSITION_RESTORE_TURN_SECONDS, real_default, places=6
            )
        finally:
            world.close()

    def test_persistent_sim_restores_real_detector_after_color_action(self):
        world = MasterPiProductionV2(seed=2, render=False)
        try:
            stack = adapter.MigratedRealStack(world)
            original = adapter.precision_mod.detect_red_blob
            replacement = lambda frame, **kwargs: None
            with adapter._patched_modules(stack):
                adapter.precision_mod._ugrp_original_detect_red_blob = original
                adapter.precision_mod.detect_red_blob = replacement
                self.assertIs(adapter.precision_mod.detect_red_blob, replacement)
            self.assertIs(adapter.precision_mod.detect_red_blob, original)
            self.assertFalse(hasattr(adapter.precision_mod, "_ugrp_original_detect_red_blob"))
        finally:
            world.close()

    def test_active_v2_contains_no_sim_only_manipulation_policy(self):
        source = inspect.getsource(MasterPiProductionV2)
        for obsolete in (
            "def _track_color", "def _approach_color", "def _pick_color",
            "def _carry", "def _navigate_to_memory", "def _place_on",
        ):
            self.assertNotIn(obsolete, source)


if __name__ == "__main__":
    unittest.main()
