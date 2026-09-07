from __future__ import annotations
import math

import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock

from harness.catalog import default_registry
from harness.executive import TaskExecutive
from harness.goals import infer_goal
from harness.loop import _planner_state_public, _planner_tool_result, run_loop
from harness.state import StateEstimator


class NoCallCompleter:
    def complete(self, messages, image=None):
        raise AssertionError("LLM must not be called for deterministic current-red grasp")


class RealFailureGateTests(unittest.TestCase):
    def test_colorless_followup_grasp_compiles_current_red_pipeline(self):
        goal = infer_goal("집어보라고")
        self.assertEqual(goal.kind, "GRASP")
        self.assertEqual(goal.subject, "UNKNOWN")
        plan = TaskExecutive().plan(
            goal,
            ["search", "track", "approach", "pick", "move_forward"],
        )
        self.assertEqual(plan, ["search", "track", "approach", "pick"])

    @mock.patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
    def test_failed_approach_never_falls_through_to_pick_or_llm(self):
        seen = []

        def runner(name, **kwargs):
            seen.append(name)
            if name == "search":
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "camera_red": {"visible": True},
                }
            if name == "track":
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "camera_red": {"visible": True},
                    "center_verified": True,
                }
            if name == "approach":
                return {
                    "ok": False, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "FAILED",
                    "outcome_status": "NOT_ACHIEVED", "failure_code": "SKILL_FAILED",
                    "reason": "could not hold the red block at image centre",
                }
            raise AssertionError(f"destructive tool must not run after approach failure: {name}")

        result = run_loop(
            NoCallCompleter(),
            default_registry(runner=runner, actions_path="scripts/robot_actions.py"),
            "집어보라고",
            execute=True,
            auto_observe=True,
            executive=TaskExecutive(strict_pick_preconditions=True),
            state_estimator=StateEstimator(),
        )
        self.assertEqual(seen, ["search", "track", "approach"])
        self.assertEqual(result.stopped, "final")
        self.assertIn("완료하지 못했", result.final or "")

    @mock.patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
    def test_target_lost_approach_runs_bounded_search_recovery_then_continues(self):
        seen = []
        approach_calls = 0

        def runner(name, **kwargs):
            nonlocal approach_calls
            seen.append(name)
            if name == "search":
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "camera_red": {"visible": True},
                }
            if name == "track":
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "camera_red": {"visible": True},
                    "center_verified": True,
                }
            if name == "approach":
                approach_calls += 1
                if approach_calls == 1:
                    return {
                        "ok": False, "skill": name,
                        "command_status": "ACCEPTED", "execution_status": "FAILED",
                        "outcome_status": "NOT_ACHIEVED",
                        "failure_code": "TARGET_NOT_VISIBLE",
                        "required_state": "target.visible=true",
                        "recommended_recovery": "search",
                        "reason": "target disappeared during final horizontal alignment",
                    }
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "camera_red": {"visible": True},
                    "center_verified": True, "range_verified": True,
                }
            if name == "pick":
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "UNKNOWN",
                    "visual_hold": {"probable": True, "confidence": .65},
                }
            raise AssertionError(name)

        result = run_loop(
            NoCallCompleter(),
            default_registry(runner=runner, actions_path="scripts/robot_actions.py"),
            "빨간 블럭 잡아봐",
            execute=True, auto_observe=True,
            executive=TaskExecutive(strict_pick_preconditions=True),
            state_estimator=StateEstimator(),
        )
        self.assertEqual(seen, ["search", "track", "approach", "search", "track", "approach", "pick"])
        self.assertIn(result.stopped, {"final", "goal_achieved"})
        self.assertIn("완료", result.final or "")

    @mock.patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
    def test_strict_real_grasp_miss_stops_without_second_pipeline(self):
        seen = []

        def runner(name, **kwargs):
            seen.append(name)
            base = {
                "skill": name,
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
            }
            if name == "search":
                return {"ok": True, **base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True}}
            if name == "track":
                return {
                    "ok": True, **base, "outcome_status": "ACHIEVED",
                    "camera_red": {"visible": True}, "center_verified": True,
                }
            if name == "approach":
                return {
                    "ok": True, **base, "outcome_status": "ACHIEVED",
                    "camera_red": {"visible": True}, "center_verified": True,
                    "range_verified": True,
                }
            if name == "pick":
                return {
                    "ok": False, **base, "execution_status": "FAILED",
                    "outcome_status": "NOT_ACHIEVED",
                    "failure_code": "GRASP_NOT_ACQUIRED",
                    "required_state": "grasp.state=PROBABLE_HELD|HELD",
                    "recommended_recovery": None,
                    "reason": "red block is still visible after the grasp",
                }
            raise AssertionError(f"strict REAL must stop after first destructive miss: {name}")

        result = run_loop(
            NoCallCompleter(),
            default_registry(runner=runner, actions_path="scripts/robot_actions.py"),
            "빨간 블럭 잡아봐",
            execute=True, auto_observe=True,
            executive=TaskExecutive(strict_pick_preconditions=True),
            state_estimator=StateEstimator(),
        )
        self.assertEqual(seen, ["search", "track", "approach", "pick"])
        self.assertEqual(result.stopped, "final")
        self.assertIn("완료하지 못했", result.final or "")

    @mock.patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
    def test_sim_grasp_miss_also_stops_without_second_pipeline(self):
        seen = []

        def runner(name, **kwargs):
            seen.append(name)
            base = {
                "skill": name,
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
            }
            if name == "search":
                return {"ok": True, **base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True}}
            if name == "track":
                return {
                    "ok": True, **base, "outcome_status": "ACHIEVED",
                    "camera_red": {"visible": True}, "center_verified": True,
                }
            if name == "approach":
                return {
                    "ok": True, **base, "outcome_status": "ACHIEVED",
                    "camera_red": {"visible": True}, "center_verified": True,
                    "range_verified": True,
                }
            if name == "pick":
                return {
                    "ok": False, **base, "execution_status": "FAILED",
                    "outcome_status": "NOT_ACHIEVED",
                    "failure_code": "GRASP_NOT_ACQUIRED",
                    "required_state": "grasp.state=PROBABLE_HELD|HELD",
                    "recommended_recovery": None,
                    "reason": "red block is still visible after the grasp",
                }
            raise AssertionError(f"SIM parity must stop after first destructive miss: {name}")

        result = run_loop(
            NoCallCompleter(),
            default_registry(runner=runner, actions_path="scripts/robot_actions.py"),
            "빨간 블럭 잡아봐",
            execute=True, auto_observe=True,
            executive=TaskExecutive(strict_pick_preconditions=False),
            state_estimator=StateEstimator(),
        )
        self.assertEqual(seen, ["search", "track", "approach", "pick"])
        self.assertEqual(result.stopped, "final")
        self.assertIn("완료하지 못했", result.final or "")

    def test_strict_real_pick_requires_successful_approach_handoff(self):
        est = StateEstimator()
        est.update_vision({"visible": True, "cx": .50, "cy": .72, "area_ratio": .08})
        self.assertEqual(est.state.target.range_class, "PREGRASP")
        self.assertTrue(est.state.target.centered)
        decision = TaskExecutive(strict_pick_preconditions=True).check("pick", est.state)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.failure_code, "PREGRASP_PLAN_MISSING")
        self.assertEqual(decision.recovery, "approach")

        est.update_tool_result({"tool": "approach", "result": {
            "skill": "approach", "command_status": "ACCEPTED",
            "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
            "camera_red": {"visible": True}, "center_verified": True,
        }})
        self.assertEqual(est.state.task.phase, "PREGRASP_READY")
        self.assertTrue(TaskExecutive(strict_pick_preconditions=True).check("pick", est.state).allowed)

    def test_probable_real_pick_never_promotes_to_held(self):
        est = StateEstimator()
        est.update_tool_result({"tool":"pick","result":{
            "skill":"pick","command_status":"ACCEPTED",
            "execution_status":"COMPLETED","outcome_status":"UNKNOWN",
            "visual_hold":{"probable":True,"confidence":.65,
                           "source":"whole_view_floor_clear_without_positive_gripper_sensor"},
        }})
        self.assertEqual(est.state.grasp.state, "PROBABLE_HELD")
        self.assertNotEqual(est.state.grasp.state, "HELD")

    def test_chassis_motion_invalidates_pregrasp_ready_phase(self):
        est = StateEstimator()
        est.update_tool_result({"tool": "approach", "result": {
            "skill": "approach", "command_status": "ACCEPTED",
            "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
        }})
        self.assertEqual(est.state.task.phase, "PREGRASP_READY")
        est.update_tool_result({"tool": "move_forward", "result": {
            "skill": "move_forward", "chassis_motion": True,
            "command_status": "ACCEPTED", "execution_status": "COMPLETED",
            "outcome_status": "UNKNOWN",
        }})
        self.assertEqual(est.state.task.phase, "UNKNOWN")

    def test_planner_feedback_strips_bulk_debug_state(self):
        est = StateEstimator()
        est.state.semantic_map = {"huge": "x" * 50000}
        est.state.spatial_memory = {
            "red": {"visible": True, "image_xy": [.5, .7], "debug": "y" * 50000},
        }
        compact_state = _planner_state_public(est.state)
        self.assertNotIn("semantic_map", compact_state)
        self.assertNotIn("debug", compact_state["landmarks"]["red"])
        compact_tool = _planner_tool_result({
            "ok": False, "tool": "approach", "observed": "/tmp/frame.jpg",
            "world_state": {"huge": "z" * 50000},
            "result": {
                "skill": "approach", "failure_code": "SKILL_FAILED",
                "reason": "centre failed", "argv": ["lots"], "debug": "q" * 50000,
            },
        })
        text = repr(compact_tool)
        self.assertIn("SKILL_FAILED", text)
        self.assertNotIn("argv", text)
        self.assertNotIn("debug", text)
        self.assertNotIn("observed", text)


ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"


def load_reference():
    if str(PACKAGE) not in sys.path:
        sys.path.insert(0, str(PACKAGE))
    path = PACKAGE / "physical_state_machine_reference.py"
    spec = importlib.util.spec_from_file_location("precision_reference_failure_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class NearFieldCenteringTests(unittest.TestCase):

    def test_target_bearing_combines_pan_and_residual_image_error(self):
        ref = load_reference()
        gaze = ref.Gaze(1510, 600)
        blob = types.SimpleNamespace(nx=.42, ny=.50)
        bearing = ref.target_bearing_left_deg(gaze, blob, flip_x=False)
        pan_only = (gaze.pan - ref.BASE_CENTER) / ref.PULSE_PER_DEGREE
        self.assertGreater(bearing, ref.BODY_BEARING_DEADBAND_DEG)
        self.assertGreater(bearing, pan_only + 4.0)

    def test_body_realign_holds_pan_and_never_reverses_after_small_crossing(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        video = object(); lock = object()
        start = ref.Gaze(1510, 600)
        # First bearing is left of chassis. One fixed-pan turn crosses centre by
        # a few degrees; the controller must accept it instead of rotate-right.
        off_axis = types.SimpleNamespace(nx=.42, ny=.50)
        crossed = types.SimpleNamespace(nx=.56, ny=.50)
        with mock.patch.object(
            ref, "read_locked",
            side_effect=[(object(), off_axis), (object(), crossed)],
        ), mock.patch.object(ref, "start_motion") as motion, \
             mock.patch.object(ref.time, "sleep", return_value=None), \
             mock.patch.object(ref, "centre_gaze") as centre:
            result = ref.align_body_to_gaze(
                robot, video, lock, start, flip_x=False, flip_y=False
            )
        self.assertEqual(result, start)
        motion.assert_called_once_with(robot, "rotate-left", ref.TURN_SPEED)
        centre.assert_not_called()

    def test_body_realign_uses_only_one_fine_pulse_then_proceeds(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        gaze = ref.Gaze(ref.BASE_CENTER, 600)
        blob = types.SimpleNamespace(nx=.50, ny=.50)
        with mock.patch.object(ref, "read_locked", return_value=(object(), blob)), \
             mock.patch.object(ref, "target_bearing_left_deg", side_effect=[2.2, 1.4]), \
             mock.patch.object(ref, "start_motion") as motion, \
             mock.patch.object(ref.time, "sleep", return_value=None), \
             mock.patch.object(ref, "centre_gaze") as centre:
            result = ref.align_body_to_gaze(
                robot, object(), object(), gaze, flip_x=False, flip_y=False
            )
        self.assertEqual(result, gaze)
        motion.assert_called_once_with(robot, "rotate-left", ref.TURN_SPEED)
        centre.assert_not_called()
        self.assertEqual(ref.BODY_FINE_ACCEPT_DEG, 3.0)

    def test_body_realign_recovery_keeps_post_recovery_centering_budget(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        video = object(); lock = object(); start = ref.Gaze(1510, 740)
        off_axis = types.SimpleNamespace(nx=.42, ny=.46)
        centred = types.SimpleNamespace(nx=.50, ny=.46)
        recovered = ref.Gaze(ref.BASE_CENTER, 740)
        with mock.patch.object(
            ref, "read_locked",
            side_effect=[(object(), off_axis), (object(), None), (object(), centred)],
        ), mock.patch.object(ref, "start_motion"), \
             mock.patch.object(ref.time, "sleep", return_value=None), \
             mock.patch.object(ref, "centre_gaze", return_value=recovered) as centre:
            result = ref.align_body_to_gaze(
                robot, video, lock, start, flip_x=False, flip_y=False
            )
        self.assertIs(result, recovered)
        self.assertEqual(
            centre.call_args.kwargs["max_frames"], ref.LOCK_HOLD_FRAMES + 20
        )

    def test_body_realign_budget_covers_measured_slow_fine_convergence(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        video = object(); lock = object(); start = ref.Gaze(1755, 740)
        blob = types.SimpleNamespace(nx=.50, ny=.46)
        # Seed-12 authoritative replay: coarse pulses remove several degrees,
        # then minimum-speed fine pulses remove only ~0.9-1.0 deg each.  The old
        # 12-pulse budget failed while still monotonically approaching zero.
        bearings = [
            25.34, 22.58, 19.33, 16.01, 12.88, 9.63, 8.58, 7.73,
            6.74, 5.76, 4.84, 3.98, 3.10, 2.20, 1.30,
        ]
        with mock.patch.object(ref, "read_locked", return_value=(object(), blob)), \
             mock.patch.object(ref, "target_bearing_left_deg", side_effect=bearings), \
             mock.patch.object(ref, "start_motion") as motion, \
             mock.patch.object(ref.time, "sleep", return_value=None), \
             mock.patch.object(ref, "centre_gaze") as centre:
            result = ref.align_body_to_gaze(
                robot, video, lock, start, flip_x=False, flip_y=False
            )
        self.assertEqual(result, start)
        self.assertEqual(ref.BODY_ALIGN_MAX_PULSES, 15)
        self.assertEqual(motion.call_count, 14)
        self.assertTrue(all(call.args[1] == "rotate-left" for call in motion.call_args_list))
        centre.assert_not_called()

    def test_too_close_target_retreats_before_any_body_yaw(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        gaze = ref.Gaze(ref.BASE_CENTER, 500)
        blobs = [
            types.SimpleNamespace(nx=.50, ny=.87),
            types.SimpleNamespace(nx=.50, ny=.78),
            types.SimpleNamespace(nx=.50, ny=.66),
        ]
        with mock.patch.object(
            ref, "read_locked", side_effect=[(object(), b) for b in blobs]
        ), mock.patch.object(ref, "start_motion") as motion,              mock.patch.object(ref.time, "sleep", return_value=None):
            result = ref.retreat_for_body_alignment(
                robot, object(), object(), gaze, flip_y=False
            )
        self.assertEqual(result, gaze)
        self.assertEqual(motion.call_count, 2)
        self.assertEqual(
            [call.args[1] for call in motion.call_args_list],
            ["backward", "backward"],
        )
        self.assertNotIn("rotate-left", [call.args[1] for call in motion.call_args_list])
        self.assertNotIn("rotate-right", [call.args[1] for call in motion.call_args_list])

    def test_approach_does_not_realign_for_ordinary_pan_offset(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        gaze = ref.Gaze(ref.BASE_CENTER + ref.BODY_REALIGN_LIMIT + 10, 740)
        sample = ref.BlockEstimate(
            radius_cm=25.5, lateral_left_cm=0.0, forward_cm=25.5,
            yaw_left_deg=0.0, camera_radius_cm=0.0, camera_height_cm=10.0,
            ray_pitch_deg=-45.0, block_height_cm=3.0, nx=.5, ny=.6,
        )
        calls = []
        def retreat(*args, **kwargs):
            calls.append("retreat")
            return gaze
        def align(*args, **kwargs):
            calls.append("align")
            return ref.Gaze(ref.BASE_CENTER, gaze.tilt)
        with mock.patch.object(ref, "retreat_for_body_alignment", side_effect=retreat), \
             mock.patch.object(ref, "align_body_to_gaze", side_effect=align), \
             mock.patch.object(
                 ref, "stationary_range_samples",
                 return_value=(ref.Gaze(ref.BASE_CENTER, gaze.tilt), [sample]*3, types.SimpleNamespace(nx=.5, ny=.6, width=.10, height=.10, area=10000)),
             ), \
             mock.patch.object(ref, "motion_pulse_with_gaze") as motion:
            result = ref.approach_with_locked_gaze(
                robot, object(), object(), gaze,
                flip_x=False, flip_y=False, target_radius_cm=25.5,
                allow_blind_arrival=False,
            )
        self.assertEqual(calls, [])
        motion.assert_not_called()
        self.assertAlmostEqual(result.block.radius_cm, 25.5)

    def test_coarse_staging_does_not_drive_forward_when_only_half_cm_outside_target(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        gaze = ref.Gaze(ref.BASE_CENTER, 740)
        sample = ref.BlockEstimate(
            radius_cm=22.47, lateral_left_cm=0.0, forward_cm=22.47,
            yaw_left_deg=0.0, camera_radius_cm=0.0, camera_height_cm=10.0,
            ray_pitch_deg=-45.0, block_height_cm=3.0, nx=.5, ny=.6,
        )
        with mock.patch.object(
            ref, "stationary_range_samples",
            return_value=(gaze, [sample, sample, sample], types.SimpleNamespace(nx=.5, ny=.6, width=.10, height=.10, area=10000)),
        ), mock.patch.object(ref, "motion_pulse_with_gaze") as motion:
            result = ref.approach_with_locked_gaze(
                robot, object(), object(), gaze,
                flip_x=False, flip_y=False, target_radius_cm=22.0,
                allow_blind_arrival=False,
            )
        motion.assert_not_called()
        self.assertAlmostEqual(result.block.radius_cm, 22.47)

    def test_coarse_staging_backs_out_when_already_inside_requested_radius(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        gaze = ref.Gaze(ref.BASE_CENTER, 740)
        def sample(radius):
            return ref.BlockEstimate(
                radius_cm=radius, lateral_left_cm=0.0, forward_cm=radius,
                yaw_left_deg=0.0, camera_radius_cm=0.0, camera_height_cm=10.0,
                ray_pitch_deg=-45.0, block_height_cm=3.0, nx=.5, ny=.6,
            )
        first, second = sample(18.5), sample(21.6)
        observations = [
            (gaze, [first, first, first], types.SimpleNamespace(nx=.5, ny=.8, width=.10, height=.10, area=10000)),
            (gaze, [second, second, second], types.SimpleNamespace(nx=.5, ny=.65, width=.10, height=.10, area=10000)),
        ]
        motions=[]
        def pulse(*args, **kwargs):
            motions.append((kwargs["kind"], kwargs["duration"]))
            return gaze
        with mock.patch.object(ref, "stationary_range_samples", side_effect=observations),              mock.patch.object(ref, "motion_pulse_with_gaze", side_effect=pulse):
            result = ref.approach_with_locked_gaze(
                robot, object(), object(), gaze,
                flip_x=False, flip_y=False, target_radius_cm=22.0,
                allow_blind_arrival=False,
            )
        self.assertEqual(motions[0][0], "backward")
        self.assertAlmostEqual(result.block.radius_cm, 21.6)

    def test_approach_verifies_final_state_after_last_allowed_motion_pulse(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        gaze = ref.Gaze(ref.BASE_CENTER, 740)

        def sample(radius):
            return ref.BlockEstimate(
                radius_cm=radius, lateral_left_cm=0.0, forward_cm=radius,
                yaw_left_deg=0.0, camera_radius_cm=0.0,
                camera_height_cm=10.0, ray_pitch_deg=-45.0,
                block_height_cm=3.0, nx=.5, ny=.6,
            )

        observations = []
        for radius in (33.0, 28.2, 23.2, 23.8, 24.3, 24.9, 25.4):
            item = sample(radius)
            observations.append((
                gaze, [item, item, item],
                types.SimpleNamespace(nx=.5, ny=.6, width=.1, height=.1, area=10000),
            ))
        motions = []
        with mock.patch.object(
            ref, "stationary_range_samples", side_effect=observations,
        ), mock.patch.object(
            ref, "motion_pulse_with_gaze",
            side_effect=lambda *args, **kwargs: motions.append(kwargs["kind"]) or gaze,
        ):
            result = ref.approach_with_locked_gaze(
                robot, object(), object(), gaze,
                flip_x=False, flip_y=False, target_radius_cm=26.0,
                allow_blind_arrival=False, allow_continuous_far=False,
                max_motion_steps=6,
            )
        self.assertEqual(len(motions), 6)
        self.assertAlmostEqual(result.block.radius_cm, 25.4)

    def test_range_jump_accepts_stable_forward_progress_above_hard_floor(self):
        ref = load_reference()
        samples = [
            types.SimpleNamespace(radius_cm=66.2),
            types.SimpleNamespace(radius_cm=66.4),
            types.SimpleNamespace(radius_cm=66.5),
        ]
        with mock.patch.object(
            ref, "stationary_range_samples",
            return_value=(ref.Gaze(ref.BASE_CENTER, 740), samples, object()),
        ):
            actual = ref._confirm_range_jump(
                object(), object(), object(), ref.Gaze(ref.BASE_CENTER, 740),
                False, False, 87.28, 16.73, stable_forward_floor_cm=15.0,
            )
        self.assertAlmostEqual(actual, 66.4)

    def test_range_jump_rejects_forward_progress_inside_hard_floor(self):
        ref = load_reference()
        samples = [
            types.SimpleNamespace(radius_cm=13.0),
            types.SimpleNamespace(radius_cm=13.2),
            types.SimpleNamespace(radius_cm=13.3),
        ]
        with mock.patch.object(
            ref, "stationary_range_samples",
            return_value=(ref.Gaze(ref.BASE_CENTER, 740), samples, object()),
        ):
            actual = ref._confirm_range_jump(
                object(), object(), object(), ref.Gaze(ref.BASE_CENTER, 740),
                False, False, 87.28, 16.73, stable_forward_floor_cm=15.0,
            )
        self.assertIsNone(actual)

    def test_target_bearing_flip_x_mirrors_pixel_component(self):
        ref = load_reference()
        gaze = ref.Gaze(ref.BASE_CENTER, 600)
        blob = types.SimpleNamespace(nx=.40, ny=.50)
        normal = ref.target_bearing_left_deg(gaze, blob, flip_x=False)
        flipped = ref.target_bearing_left_deg(gaze, blob, flip_x=True)
        self.assertAlmostEqual(normal, -flipped, places=6)

    def test_arm_face_error_is_relative_to_servo6_approach_ray(self):
        ref = load_reference()
        # A -68.7deg visible edge has its nearest normal at +21.3deg. Chassis
        # forward therefore starts 21.3deg off, but an arm ray yawed +21.3deg is
        # square without rotating the chassis.
        self.assertAlmostEqual(ref.arm_face_signed_error_deg(-68.7, 0.0), 21.3, delta=.05)
        self.assertAlmostEqual(ref.arm_face_signed_error_deg(-68.7, 21.3), 0.0, delta=.05)
        self.assertAlmostEqual(ref.arm_face_signed_error_deg(+68.7, -21.3), 0.0, delta=.05)

    def test_face_route_temporary_turn_has_wider_tolerance_than_final_restore(self):
        ref = load_reference()
        self.assertGreater(
            ref.FACE_ROUTE_TRAVEL_BEARING_TOLERANCE_DEG,
            ref.FACE_ROUTE_BEARING_TOLERANCE_DEG,
        )
        # At the observed 7.8 cm route length, 4 deg heading error contributes
        # under 0.6 cm endpoint error, below the route range tolerance.
        lateral_error = 7.8 * math.sin(math.radians(ref.FACE_ROUTE_TRAVEL_BEARING_TOLERANCE_DEG))
        self.assertLess(lateral_error, ref.FACE_ROUTE_RANGE_TOLERANCE_CM)

    def test_face_route_requires_bearing_as_well_as_radius_for_2d_endpoint(self):
        ref = load_reference()
        block = ref.BlockEstimate(
            radius_cm=24.14, lateral_left_cm=0.0, forward_cm=24.14,
            yaw_left_deg=0.1, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=-30.0, block_height_cm=3.0, nx=.5, ny=.4,
        )
        face = ref.FaceAlignmentEstimate(-57.6, 32.4, 3.0)
        plan = ref.plan_face_normal_route(block, face, reach_correction_cm=0.0)
        endpoint = ref.face_route_travel_endpoint_bearing_deg(plan, 45.8)
        self.assertAlmostEqual(endpoint, 72.1, delta=.3)
        # Exact failed SIM postcondition: radial range looked ready, but the
        # target was still ~8 deg short of the planned side-coordinate endpoint.
        current = ref.BlockEstimate(
            radius_cm=20.8, lateral_left_cm=0.0, forward_cm=20.8,
            yaw_left_deg=55.0, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=-30.0, block_height_cm=3.0, nx=.5, ny=.4,
        )
        remaining, bearing_error, _ = ref.face_route_translation_errors(
            plan, current, achieved_travel_target_bearing_deg=45.8
        )
        self.assertLessEqual(remaining, ref.FACE_ROUTE_RANGE_TOLERANCE_CM)
        self.assertGreater(abs(bearing_error), ref.FACE_ROUTE_ENDPOINT_BEARING_TOLERANCE_DEG)

    def test_face_normal_route_plans_real_lateral_translation_before_close_approach(self):
        ref = load_reference()
        block = ref.BlockEstimate(
            radius_cm=21.7, lateral_left_cm=0.0, forward_cm=21.7,
            yaw_left_deg=0.1, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=-30.0, block_height_cm=3.0, nx=.5, ny=.4,
        )
        face = ref.FaceAlignmentEstimate(-57.9, 32.1, 3.0)
        plan = ref.plan_face_normal_route(block, face)
        self.assertAlmostEqual(plan.current_radius_cm, 26.7, delta=.05)
        self.assertAlmostEqual(plan.face_normal_bearing_deg, 32.1, delta=.1)
        self.assertAlmostEqual(plan.target_bearing_deg, 26.1, delta=.1)
        self.assertAlmostEqual(plan.target_radius_cm, 20.0, delta=.01)
        self.assertLess(plan.travel_bearing_deg, -30.0)  # drive to robot-right, not strafe
        self.assertGreater(plan.travel_distance_cm, 8.0)
        self.assertLess(plan.travel_distance_cm, 13.0)

        mirrored = ref.BlockEstimate(
            radius_cm=21.7, lateral_left_cm=0.0, forward_cm=21.7,
            yaw_left_deg=-0.1, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=-30.0, block_height_cm=3.0, nx=.5, ny=.4,
        )
        mirror_plan = ref.plan_face_normal_route(
            mirrored, ref.FaceAlignmentEstimate(+57.9, 32.1, 3.0)
        )
        self.assertGreater(mirror_plan.travel_bearing_deg, 30.0)
        self.assertAlmostEqual(
            abs(mirror_plan.travel_distance_cm), plan.travel_distance_cm, delta=.15
        )

    def test_face_route_projects_real_out_of_corridor_target_when_residual_is_safe(self):
        ref = load_reference()
        # Exact geometry from REAL run 20260901T065618Z-real-a822ae0f:
        # bearing=+1.5deg, floor-edge=+42.2deg -> nearest face normal=-47.8deg.
        # A 6deg ideal residual asks for -41.8deg, outside the +/-38deg hard
        # corridor. -36deg is still only 11.8deg from the face normal, so the
        # route should use that safe reachable endpoint rather than abort.
        block = ref.BlockEstimate(
            radius_cm=26.47, lateral_left_cm=0.0, forward_cm=26.47,
            yaw_left_deg=1.5, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=-30.0, block_height_cm=3.0, nx=.5, ny=.4,
        )
        face = ref.FaceAlignmentEstimate(+42.2, 49.3, 3.0)
        plan = ref.plan_face_normal_route(block, face, reach_correction_cm=0.0)
        self.assertAlmostEqual(plan.face_normal_bearing_deg, -47.8, delta=.1)
        expected = -(ref.ARM_FACE_MAX_BEARING_DEG - ref.FACE_ROUTE_ARM_BEARING_MARGIN_DEG)
        self.assertAlmostEqual(plan.target_bearing_deg, expected, delta=.05)
        residual = abs(ref.arm_face_signed_error_deg(face.edge_angle_deg, plan.target_bearing_deg))
        self.assertAlmostEqual(residual, 11.8, delta=.1)
        self.assertLessEqual(residual, ref.FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG)
        self.assertLess(abs(plan.target_bearing_deg), ref.ARM_FACE_MAX_BEARING_DEG)

    def test_face_route_still_rejects_when_corridor_projection_cannot_make_face_safe(self):
        ref = load_reference()
        block = ref.BlockEstimate(
            radius_cm=26.0, lateral_left_cm=0.0, forward_cm=26.0,
            yaw_left_deg=0.0, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=-30.0, block_height_cm=3.0, nx=.5, ny=.4,
        )
        # +30deg edge -> nearest normal=-60deg. Even a -36deg projected arm ray
        # leaves 24deg error, beyond the 14deg staging acceptance gate.
        face = ref.FaceAlignmentEstimate(+30.0, 60.0, 3.0)
        with self.assertRaisesRegex(RuntimeError, "usable servo-6 corridor"):
            ref.plan_face_normal_route(block, face, reach_correction_cm=0.0)

    def test_face_route_does_not_accept_a_15deg_staging_error_before_close_approach(self):
        ref = load_reference()
        block = ref.BlockEstimate(
            radius_cm=23.76, lateral_left_cm=0.0, forward_cm=23.76,
            yaw_left_deg=0.6, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=-30.0, block_height_cm=3.0, nx=.5, ny=.4,
        )
        face = ref.FaceAlignmentEstimate(-73.7, 16.3, 3.0)
        self.assertAlmostEqual(
            abs(ref.arm_face_signed_error_deg(face.edge_angle_deg, block.yaw_left_deg)),
            15.7, delta=.05,
        )
        plan = ref.plan_face_normal_route(block, face, reach_correction_cm=0.0)
        # The old policy treated 15.7deg as already safe because it was under
        # the final 20deg abort threshold. It must now make a bounded side
        # correction with a 6deg movement target, while the independent
        # acceptance gate remains 14deg.
        self.assertAlmostEqual(plan.target_bearing_deg, 10.3, delta=.1)
        self.assertGreater(plan.travel_distance_cm, 5.0)
        self.assertLessEqual(
            ref.FACE_ROUTE_STAGING_ACCEPT_ERROR_DEG,
            ref.FACE_ALIGNMENT_ABORT_DEG - 6.0,
        )

    def test_face_reposition_dogleg_never_uses_pure_strafe(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        lock = types.SimpleNamespace(note_ego_motion=mock.Mock())
        with mock.patch.object(ref, "start_motion") as start, \
             mock.patch.object(ref.time, "sleep", return_value=None):
            ref.face_reposition_dogleg(robot, lock, side="right")
        kinds = [call.args[1] for call in start.call_args_list]
        self.assertEqual(kinds, ["rotate-left", "backward", "rotate-right"])
        self.assertNotIn("left", kinds)
        self.assertNotIn("right", kinds)
        self.assertEqual(robot.stop.call_count, 3)

    def test_final_arm_face_alignment_uses_dogleg_and_straight_depth_only(self):
        ref = load_reference()
        gaze0 = ref.Gaze(1500,600)
        gaze1 = ref.Gaze(1700,600)
        blob0 = types.SimpleNamespace(nx=.50, ny=.30, area=9000)
        blob1 = types.SimpleNamespace(nx=.50, ny=.30, area=9000)
        measurements = [
            (ref.FaceAlignmentEstimate(-60.0,30.0,3.0), 0.0, +30.0, blob0),
            (ref.FaceAlignmentEstimate(-60.0,10.0,3.0), 20.0, +10.0, blob1),
        ]
        robot = types.SimpleNamespace(stop=mock.Mock())
        with mock.patch.object(ref, "measure_arm_face_alignment", side_effect=measurements), \
             mock.patch.object(ref, "face_reposition_dogleg") as dogleg, \
             mock.patch.object(ref, "fine_align_horizontal", return_value=gaze1) as pan_only, \
             mock.patch.object(ref, "visual_capture_approach", return_value=(gaze1, blob1)) as depth, \
             mock.patch.object(ref, "orbit_strafe_measure_bearing") as strafe, \
             mock.patch.object(ref, "align_body_to_gaze") as chassis_reface:
            gaze, blob, bearing = ref.align_block_face_with_arm_yaw(
                robot, object(), object(), gaze0,
                flip_x=False, flip_y=False, target_nx=.50,
            )
        self.assertEqual(gaze, gaze1)
        self.assertIs(blob, blob1)
        self.assertAlmostEqual(bearing, 20.0)
        dogleg.assert_called_once_with(robot, mock.ANY, side="right")
        pan_only.assert_called_once()
        self.assertEqual(depth.call_args.kwargs["target_ny"], ref.ARM_FACE_CAPTURE_TARGET_NY)
        strafe.assert_not_called()
        chassis_reface.assert_not_called()

    def test_final_arm_face_alignment_stops_when_dogleg_does_not_improve(self):
        ref = load_reference()
        gaze = ref.Gaze(1500,600)
        blob = types.SimpleNamespace(nx=.50, ny=.30, area=9000)
        measurement = (ref.FaceAlignmentEstimate(-60.0,30.0,3.0), 0.0, +30.0, blob)
        robot = types.SimpleNamespace(stop=mock.Mock())
        with mock.patch.object(ref, "measure_arm_face_alignment", return_value=measurement), \
             mock.patch.object(ref, "face_reposition_dogleg") as dogleg, \
             mock.patch.object(ref, "fine_align_horizontal", return_value=gaze), \
             mock.patch.object(ref, "visual_capture_approach", return_value=(gaze, blob)), \
             mock.patch.object(ref, "orbit_strafe_measure_bearing") as strafe:
            with self.assertRaisesRegex(RuntimeError, "did not improve"):
                ref.align_block_face_with_arm_yaw(
                    robot, object(), object(), gaze,
                    flip_x=False, flip_y=False, target_nx=.50,
                )
        self.assertEqual(dogleg.call_count, ref.ARM_FACE_MAX_STALLED_PULSES)
        strafe.assert_not_called()

    def test_face_alignment_estimator_accepts_face_normal_view(self):
        ref = load_reference()
        blob = types.SimpleNamespace(
            box_points=((.40,.60),(.60,.60),(.60,.40),(.40,.40)),
            rectangularity=.90,
        )
        # The two lower corners project to a lateral floor edge. A lateral edge
        # means its face normal is on the chassis centreline: ideal grasp view.
        with mock.patch.object(ref, "project_floor_pixel", side_effect=[(0.0,10.0),(3.0,10.0)]):
            estimate = ref.estimate_face_alignment({3:500,4:2320,5:1320,6:1500}, blob, flip_x=False)
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.error_deg, 0.0, delta=.1)

    def test_face_alignment_estimator_rejects_diagonal_view_as_large_error(self):
        ref = load_reference()
        blob = types.SimpleNamespace(
            box_points=((.40,.60),(.60,.60),(.60,.40),(.40,.40)),
            rectangularity=.90,
        )
        with mock.patch.object(ref, "project_floor_pixel", side_effect=[(0.0,10.0),(2.0,12.0)]):
            estimate = ref.estimate_face_alignment({3:500,4:2320,5:1320,6:1500}, blob, flip_x=False)
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.error_deg, 45.0, delta=.1)

    def test_face_alignment_repositions_without_pure_strafe_then_refaces(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        video = object(); lock = object(); gaze = ref.Gaze(1500,600)
        measurements = [
            ref.FaceAlignmentEstimate(edge_angle_deg=45.0,error_deg=45.0,edge_length_cm=3.0),
            ref.FaceAlignmentEstimate(edge_angle_deg=80.0,error_deg=10.0,edge_length_cm=3.0),
        ]
        staged = types.SimpleNamespace(gaze=gaze)
        with mock.patch.object(ref, "measure_face_alignment", side_effect=measurements), \
             mock.patch.object(ref, "face_reposition_dogleg") as dogleg, \
             mock.patch.object(ref, "align_body_to_gaze", return_value=gaze) as reface, \
             mock.patch.object(ref, "centre_gaze", return_value=gaze), \
             mock.patch.object(ref, "approach_with_locked_gaze", return_value=staged) as depth, \
             mock.patch.object(ref, "orbit_strafe_measure_bearing") as strafe:
            result = ref.align_block_face(robot, video, lock, gaze, flip_x=False, flip_y=False)
        self.assertEqual(result, gaze)
        dogleg.assert_called_once_with(robot, lock, side="left")
        reface.assert_called_once()
        self.assertEqual(depth.call_args.kwargs["target_radius_cm"], ref.FACE_ALIGN_START_RADIUS_CM)
        strafe.assert_not_called()

    def test_face_alignment_no_improvement_fails_without_strafe(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        video = object(); lock = object(); gaze = ref.Gaze(1500,600)
        diagonal = ref.FaceAlignmentEstimate(edge_angle_deg=-45.0,error_deg=45.0,edge_length_cm=3.0)
        staged = types.SimpleNamespace(gaze=gaze)
        with mock.patch.object(ref, "measure_face_alignment", return_value=diagonal), \
             mock.patch.object(ref, "face_reposition_dogleg") as dogleg, \
             mock.patch.object(ref, "align_body_to_gaze", return_value=gaze), \
             mock.patch.object(ref, "centre_gaze", return_value=gaze), \
             mock.patch.object(ref, "approach_with_locked_gaze", return_value=staged), \
             mock.patch.object(ref, "orbit_strafe_measure_bearing") as strafe:
            with self.assertRaisesRegex(RuntimeError, "did not improve"):
                ref.align_block_face(robot, video, lock, gaze, flip_x=False, flip_y=False)
        self.assertEqual(dogleg.call_count, ref.FACE_REPOSITION_MAX_STALLED_PULSES)
        strafe.assert_not_called()

    def test_orbit_strafe_measures_target_bearing_before_reface(self):
        ref = load_reference()
        robot = types.SimpleNamespace(stop=mock.Mock())
        gaze = ref.Gaze(1500,600)
        before = types.SimpleNamespace(nx=.50)
        after = types.SimpleNamespace(nx=.40)
        with mock.patch.object(ref, "read_locked", side_effect=[(object(), before), (object(), after)]), \
             mock.patch.object(ref, "start_motion") as start, \
             mock.patch.object(ref.time, "sleep", return_value=None):
            _gaze, shift = ref.orbit_strafe_measure_bearing(
                robot, object(), object(), gaze, direction="right", flip_x=False
            )
        start.assert_called_once_with(robot, "right", ref.ORBIT_SPEED)
        self.assertEqual(ref.ORBIT_SPEED, 40)
        self.assertAlmostEqual(shift, ref.CAMERA_HFOV_DEG * .10, places=4)
        robot.stop.assert_called()

    def test_grasp_verification_uses_fresh_snapshots_not_stream_buffer(self):
        ref = load_reference()
        frame = object()
        blob = types.SimpleNamespace(nx=.50, ny=.60, area=1800)
        video = mock.Mock()
        with mock.patch.object(ref, "capture_bgr", return_value=frame) as snap, \
             mock.patch.object(ref, "detect_red_blob", return_value=blob), \
             mock.patch.object(ref.time, "sleep", return_value=None):
            summary = ref.capture_red_summary(video)
        self.assertEqual(summary.hits, ref.VERIFY_FRAME_COUNT)
        self.assertEqual(snap.call_count, ref.VERIFY_FRAME_COUNT)
        video.read.assert_not_called()

    def test_live_video_reconnects_after_one_http_eof(self):
        ref = load_reference()
        ok, encoded = ref.cv2.imencode(".jpg", ref.np.zeros((12, 16, 3), dtype=ref.np.uint8))
        self.assertTrue(ok)
        jpeg = encoded.tobytes()

        class EOFResponse:
            def read(self, _size): return b""
            def close(self): pass

        class HoldingResponse:
            def __init__(self):
                self.sent = False
                self.closed = False
            def read(self, _size):
                if not self.sent:
                    self.sent = True
                    return jpeg
                while not self.closed:
                    ref.time.sleep(.005)
                return b""
            def close(self): self.closed = True

        responses = [EOFResponse(), HoldingResponse()]
        opens = []
        def opener(_url, timeout=0):
            opens.append(timeout)
            return responses.pop(0)

        video = ref.LiveVideo("fake://stream", opener=opener, reconnect_delay=0)
        try:
            frame = video.read(timeout=1.0)
            self.assertEqual(frame.shape[:2], (12,16))
            self.assertGreaterEqual(len(opens), 2)
        finally:
            video.close()

    def test_live_video_initial_open_tolerates_short_refusal_window(self):
        ref = load_reference()
        ok, encoded = ref.cv2.imencode(".jpg", ref.np.zeros((12, 16, 3), dtype=ref.np.uint8))
        self.assertTrue(ok)
        jpeg = encoded.tobytes()

        class HoldingResponse:
            def __init__(self):
                self.sent = False
                self.closed = False
            def read(self, _size):
                if not self.sent:
                    self.sent = True
                    return jpeg
                while not self.closed:
                    ref.time.sleep(.005)
                return b""
            def close(self): self.closed = True

        attempts = []
        def opener(_url, timeout=0):
            attempts.append(timeout)
            if len(attempts) <= 4:
                raise ConnectionRefusedError(111, "connection refused")
            return HoldingResponse()

        video = ref.LiveVideo(
            "fake://stream", opener=opener, max_reconnects=5, reconnect_delay=0
        )
        try:
            frame = video.read(timeout=1.0)
            self.assertEqual(frame.shape[:2], (12,16))
            self.assertEqual(len(attempts), 5)
        finally:
            video.close()

    def test_live_video_reader_tolerates_multi_attempt_restart_window(self):
        ref = load_reference()
        ok, encoded = ref.cv2.imencode(".jpg", ref.np.zeros((12, 16, 3), dtype=ref.np.uint8))
        self.assertTrue(ok)
        jpeg = encoded.tobytes()

        class EOFResponse:
            def read(self, _size): return b""
            def close(self): pass

        class HoldingResponse:
            def __init__(self):
                self.sent = False
                self.closed = False
            def read(self, _size):
                if not self.sent:
                    self.sent = True
                    return jpeg
                while not self.closed:
                    ref.time.sleep(.005)
                return b""
            def close(self): self.closed = True

        opens = []
        def opener(_url, timeout=0):
            opens.append(timeout)
            if len(opens) == 1:
                return EOFResponse()
            if len(opens) <= 5:
                raise ConnectionRefusedError(111, "connection refused")
            return HoldingResponse()

        video = ref.LiveVideo(
            "fake://stream", opener=opener, max_reconnects=6, reconnect_delay=0
        )
        try:
            frame = video.read(timeout=1.0)
            self.assertEqual(frame.shape[:2], (12,16))
            self.assertEqual(len(opens), 6)
        finally:
            video.close()

    def test_known_cube_grasp_is_lower_than_bad_near_field_height_estimate(self):
        ref = load_reference()
        robot = types.SimpleNamespace(pose={3:500,4:2320,5:1320,6:1500})
        block = ref.BlockEstimate(
            radius_cm=15.47, lateral_left_cm=0.0, forward_cm=15.47,
            yaw_left_deg=0.0, camera_radius_cm=0.0, camera_height_cm=0.0,
            ray_pitch_deg=0.0, block_height_cm=6.04, nx=.5, ny=.7,
        )
        plan = ref.calculate_pick_plan(robot, ref.Gaze(1500,500), block)
        fk = ref.forward_kinematics(plan.grasp_pose, ref.GRIPPER_LINK)
        self.assertAlmostEqual(ref.GRASP_REFERENCE_BLOCK_HEIGHT_CM, 3.0)
        self.assertLessEqual(fk.height_cm, 1.0)
        self.assertGreaterEqual(fk.height_cm, 0.6)

    def test_pick_reference_is_captured_before_camera_leaves_measurement_pose(self):
        ref = load_reference()
        calls = []
        class Robot:
            dry_run = True
            def __init__(self): self.pose={1:2000,3:500,4:2320,5:1320,6:1500}
            def stop(self): calls.append("stop")
            def move_servo(self, servo, pulse, duration):
                calls.append(("servo",servo,pulse)); self.pose[servo]=pulse
            def nudge_servos(self, updates, duration):
                calls.append(("arm",dict(updates))); self.pose.update(updates)
        plan = ref.PickPlan(
            block=types.SimpleNamespace(), base_pulse=1500,
            hover_pose={3:650,4:1757,5:2030},
            grasp_pose={3:1017,4:2002,5:2336}, fingertip_radius_cm=15.97,
        )
        robot=Robot()
        reference=object()
        with mock.patch.object(ref, "make_verification_reference", side_effect=lambda r,v: (calls.append("reference") or reference)),              mock.patch.object(ref.time, "sleep", return_value=None):
            returned=ref.execute_pick_to_hover(robot, object(), plan)
        self.assertIs(returned, reference)
        self.assertLess(calls.index("reference"), next(i for i,x in enumerate(calls) if isinstance(x, tuple) and x[0]=="arm"))

    def test_arm_face_stage_precedes_final_hand_eye_depth(self):
        ref = load_reference()
        # REAL face projection was valid near ny~=0.31 but unavailable in
        # archived frames at 0.408/0.412. Keep face observation early, then use
        # the existing calibrated hand-eye target for final straight reach.
        self.assertAlmostEqual(ref.ARM_FACE_CAPTURE_TARGET_NY, 0.30, places=6)
        self.assertGreater(ref.CAPTURE_TARGET_NY, ref.ARM_FACE_CAPTURE_TARGET_NY + 0.10)
        self.assertLess(ref.CAPTURE_TARGET_NY, ref.CAPTURE_TOO_CLOSE_NY)
        self.assertLessEqual(
            abs(ref.arm_face_signed_error_deg(-75.0, +13.0)),
            ref.FACE_ALIGNMENT_TOLERANCE_DEG,
        )

    def test_metric_range_stops_before_fixed_visual_capture(self):
        ref = load_reference()
        self.assertGreater(ref.CAPTURE_STAGING_RADIUS_CM, ref.CAPTURE_FINGERTIP_RADIUS_CM + 4.0)
        self.assertLess(ref.CAPTURE_TARGET_NY, 0.50)
        self.assertGreater(ref.CAPTURE_TARGET_NY, 0.40)
        self.assertAlmostEqual(
            ref.CAPTURE_TARGET_NY,
            ref.capture_target_ny_from_hand_eye(),
            delta=.003,
        )
        self.assertLess(ref.CAPTURE_TARGET_NY, ref.CAPTURE_TOO_CLOSE_NY)
        self.assertLess(ref.CAPTURE_CREEP_SECONDS, ref.NEAR_FORWARD_SECONDS)

    def test_capture_grasp_matches_masterpi_official_ik_fixture(self):
        ref = load_reference()
        # Physical Pi ArmIK fixture for (x=0,y=12.5,z=0), alpha=-90 is
        # {3:629,4:1768,5:2449}; the local closed-form solver rounds 3/5 by one.
        pose = ref.solve_ik(12.5, 0.0, preferred_pitch_deg=ref.CAPTURE_GRASP_PITCH_DEG)
        official = {3:629,4:1768,5:2449}
        self.assertTrue(all(abs(pose[k]-official[k]) <= 1 for k in official))
        point = ref.forward_kinematics(pose, ref.GRIPPER_LINK)
        self.assertAlmostEqual(point.pitch_deg, -90.0, delta=.15)

    def test_visual_capture_stop_is_decoupled_from_provisional_forward_jaw_reach(self):
        ref = load_reference()
        # Keep the already-observed close-camera stop at the same pixel while
        # extending only the final jaw/TCP reach after two clean 12.5 cm misses and one clean 15.0 cm miss.
        self.assertAlmostEqual(ref.CAPTURE_VISUAL_BLOCK_RADIUS_CM, 12.0, places=6)
        self.assertAlmostEqual(ref.CAPTURE_LONGITUDINAL_REACH_CORRECTION_CM, 5.0, places=6)
        self.assertAlmostEqual(ref.CAPTURE_BLOCK_RADIUS_CM, 17.0, places=6)
        self.assertAlmostEqual(ref.CAPTURE_FINGERTIP_RADIUS_CM, 17.5, places=6)
        self.assertAlmostEqual(ref.capture_target_ny_from_hand_eye(), .442, delta=.004)

    def test_capture_plan_uses_fixed_masterpi_tcp_not_bad_metric_radius(self):
        ref = load_reference()
        robot = types.SimpleNamespace(pose={3:600,4:2200,5:1900,6:1500})
        blob = types.SimpleNamespace(nx=.50,ny=.442,area=9000)
        plan = ref.make_capture_pick_plan(robot, ref.Gaze(1500,600), blob)
        point = ref.forward_kinematics(plan.grasp_pose, ref.GRIPPER_LINK)
        self.assertAlmostEqual(plan.fingertip_radius_cm, ref.CAPTURE_FINGERTIP_RADIUS_CM, places=6)
        self.assertAlmostEqual(point.radius_cm, ref.CAPTURE_FINGERTIP_RADIUS_CM, delta=.08)
        self.assertAlmostEqual(point.height_cm, ref.CAPTURE_GRASP_HEIGHT_CM, delta=.08)

    def test_capture_grasp_is_lower_but_keeps_wrist_servo_margin(self):
        ref = load_reference()
        robot = types.SimpleNamespace(pose={3:600,4:2200,5:1900,6:1500})
        blob = types.SimpleNamespace(nx=.50,ny=.442,area=9000)
        plan = ref.make_capture_pick_plan(robot, ref.Gaze(1500,600), blob)
        point = ref.forward_kinematics(plan.grasp_pose, ref.GRIPPER_LINK)
        self.assertLess(point.height_cm, 0.0)
        self.assertAlmostEqual(point.height_cm, -0.60, delta=.08)
        self.assertLessEqual(plan.grasp_pose[5], 2485)

    def test_capture_coordinate_uses_explicit_tcp_target_x(self):
        ref = load_reference()
        robot = types.SimpleNamespace(pose={3:600,4:2200,5:1900,6:1500})
        blob = types.SimpleNamespace(nx=.42,ny=.442,area=9000)
        with mock.patch.object(ref, "CAPTURE_TARGET_NX", .42):
            plan = ref.make_capture_pick_plan(robot, ref.Gaze(1500,600), blob)
        self.assertEqual(plan.base_pulse, ref.BASE_CENTER)
        self.assertAlmostEqual(plan.block.yaw_left_deg, 0.0, places=6)

    def test_blind_descent_keeps_tcp_radius_and_monotonically_lowers_z(self):
        ref = load_reference()
        robot = types.SimpleNamespace(pose={3:600,4:2200,5:1900,6:1500})
        blob = types.SimpleNamespace(nx=.50,ny=.442,area=9000)
        plan = ref.make_capture_pick_plan(robot, ref.Gaze(1500,600), blob)
        points=[ref.forward_kinematics(p,ref.GRIPPER_LINK) for p in ref.grasp_descent_waypoints(plan)]
        self.assertGreaterEqual(len(points),4)
        self.assertTrue(all(abs(p.radius_cm-ref.CAPTURE_FINGERTIP_RADIUS_CM)<.10 for p in points))
        self.assertTrue(all(a.height_cm>b.height_cm for a,b in zip(points,points[1:])))
        self.assertAlmostEqual(points[-1].height_cm,ref.CAPTURE_GRASP_HEIGHT_CM,delta=.08)

    def test_precapture_real_failure_trajectory_switches_before_old_timeout(self):
        ref=load_reference()
        values=[.527,.552,.579,.608,.635,.665,.696,.729]
        blobs=[types.SimpleNamespace(nx=.50,ny=v,area=8000+i*50) for i,v in enumerate(values)]
        samples=iter((b,b.ny) for b in blobs)
        class Robot:
            dry_run=True
            pose={3:500,4:2320,5:1320,6:1500}
            def __init__(self): self.drives=0
            def stop(self): pass
            def drive(self,kind,speed,duration): self.drives+=1
        robot=Robot(); gaze=ref.Gaze(1500,500)
        with mock.patch.object(ref,"fine_align_horizontal",return_value=gaze), \
             mock.patch.object(ref,"_capture_sample",side_effect=lambda *a: next(samples)):
            _g,_l,b=ref.visual_precapture_approach(robot,object(),object(),gaze)
        self.assertGreaterEqual(
            b.ny,
            ref.PRECAPTURE_TARGET_NY - ref.PRECAPTURE_TARGET_TOLERANCE_NY,
        )
        self.assertLessEqual(robot.drives, 7)
        self.assertAlmostEqual(ref.PRECAPTURE_TARGET_NY, .72, places=2)
        self.assertAlmostEqual(ref.PRECAPTURE_TARGET_TOLERANCE_NY, .01, places=3)
        self.assertGreaterEqual(ref.PRECAPTURE_MAX_CREEP_PULSES, 10)

    def test_precapture_overlap_ready_does_not_drive(self):
        ref=load_reference()
        # Reproduces the observed 0.717/0.720 boundary miss. Being within the
        # overlap tolerance must switch views without another forward pulse.
        ready_ny = ref.PRECAPTURE_TARGET_NY - 0.003
        blob=types.SimpleNamespace(nx=.50,ny=ready_ny,area=9000)
        class Robot:
            dry_run=True
            pose={3:500,4:2320,5:1320,6:1500}
            def stop(self): pass
            def drive(self,*a,**k): raise AssertionError("overlap-ready target must not drive")
        gaze=ref.Gaze(1500,500)
        with mock.patch.object(ref,"fine_align_horizontal",return_value=gaze), \
             mock.patch.object(ref,"_capture_sample",return_value=(blob,ready_ny)):
            g,l,b=ref.visual_precapture_approach(Robot(),object(),object(),gaze)
        self.assertIs(g,gaze)
        self.assertIs(b,blob)

    def test_precapture_far_target_creeps_before_close_camera_switch(self):
        ref=load_reference()
        far=types.SimpleNamespace(nx=.50,ny=.60,area=8000)
        closer=types.SimpleNamespace(nx=.50,ny=.81,area=9000)
        class Robot:
            dry_run=True
            pose={3:500,4:2320,5:1320,6:1500}
            def __init__(self): self.drives=0
            def stop(self): pass
            def drive(self,kind,speed,duration): self.drives+=1
        robot=Robot(); gaze=ref.Gaze(1500,500)
        samples=iter([(far,.60),(closer,.81),(closer,.81)])
        with mock.patch.object(ref,"fine_align_horizontal",return_value=gaze), \
             mock.patch.object(ref,"_capture_sample",side_effect=lambda *a: next(samples)):
            _g,_l,b=ref.visual_precapture_approach(robot,object(),object(),gaze)
        self.assertEqual(robot.drives,1)
        self.assertIs(b,closer)

    def test_precapture_uses_coarse_to_fine_durations_and_fewer_pulses(self):
        ref = load_reference()
        state = {"ny": .423}
        durations = []
        class Robot:
            dry_run = True
            pose = dict(ref.PRECAPTURE_ARM_POSE)
            def stop(self): pass
            def drive(self, kind, speed, duration):
                self_kind = kind
                durations.append(duration)
                # Approximate the measured 60 ms -> ~0.04 ny physical progress.
                state["ny"] += .65 * duration
        def sample(*_args):
            b = types.SimpleNamespace(nx=.50, ny=state["ny"], area=8000)
            return b, b.ny
        gaze = ref.Gaze(1500, ref.PRECAPTURE_ARM_POSE[3])
        with mock.patch.object(ref, "_capture_sample", side_effect=sample), \
             mock.patch.object(ref, "fine_align_horizontal", side_effect=AssertionError("centered x must not pay pan loop")):
            _g, _l, blob = ref.visual_precapture_approach(
                Robot(), object(), object(), gaze
            )
        self.assertGreaterEqual(blob.ny, ref.PRECAPTURE_TARGET_NY-ref.PRECAPTURE_TARGET_TOLERANCE_NY)
        self.assertLessEqual(len(durations), 6)
        self.assertEqual(durations[:2], [ref.PRECAPTURE_FAST_CREEP_SECONDS]*2)
        self.assertIn(ref.PRECAPTURE_MID_CREEP_SECONDS, durations)
        self.assertEqual(durations[-1], ref.PRECAPTURE_NEAR_CREEP_SECONDS)

    def test_capture_sample_uses_freshest_close_depth_when_median_lags_motion(self):
        ref = load_reference()
        # Reproduces the physical close-view sequence from
        # 20260831T082105Z-approach-be2ce487.  The median (.390) was still below
        # the ready threshold (.417), while the freshest frame (.421) was already
        # safely inside it.  Control must not command another forward pulse from
        # the stale median.
        blobs = [
            types.SimpleNamespace(nx=.487, ny=.352, area=28676),
            types.SimpleNamespace(nx=.489, ny=.390, area=29300),
            types.SimpleNamespace(nx=.478, ny=.421, area=27767),
        ]

        class Video:
            def read(self):
                return object()

        class Lock:
            def __init__(self):
                self.index = 0
            def select(self, blob):
                return blob

        with mock.patch.object(ref, "detect_red_blob", side_effect=blobs), \
             mock.patch.object(ref.time, "sleep", return_value=None):
            last, control_ny = ref._capture_sample(Video(), Lock())

        self.assertIs(last, blobs[-1])
        self.assertAlmostEqual(control_ny, .421, places=6)
        self.assertGreaterEqual(
            control_ny, ref.CAPTURE_TARGET_NY - ref.CAPTURE_TARGET_TOLERANCE_NY
        )

    def test_capture_sample_skips_pan_loop_when_existing_sample_is_centered(self):
        ref = load_reference()
        gaze = ref.Gaze(1500, 600)
        blob = types.SimpleNamespace(nx=.51, ny=.40, area=10000)
        with mock.patch.object(ref, "fine_align_horizontal") as align, \
             mock.patch.object(ref, "_capture_sample", side_effect=AssertionError("provided sample must be reused")):
            returned, out, ny, nx = ref._sample_capture_with_optional_x_align(
                types.SimpleNamespace(), object(), object(), gaze,
                flip_x=False, target_nx=.50, sample=(blob,.40),
            )
        self.assertIs(returned, gaze)
        self.assertIs(out, blob)
        self.assertAlmostEqual(ny,.40)
        self.assertAlmostEqual(nx,.51)
        align.assert_not_called()

    def test_capture_retry_uses_fixed_view_closer_target_not_metric_range(self):
        ref=load_reference()
        blob=types.SimpleNamespace(nx=.50,ny=.55,area=11000)
        remaining=ref.RedSummary(hits=9,total=9,nx=.50,ny=.55,area=11000,last_blob=blob,last_frame=None)
        class Robot:
            dry_run=True
            pose={3:600,4:2200,5:1900,6:1490,1:1500}
            def move_servo(self,servo,pulse,duration): self.pose[servo]=pulse
        robot=Robot(); gaze=ref.Gaze(1490,600); plan=types.SimpleNamespace()
        with mock.patch.object(ref,"visual_capture_approach",return_value=(gaze,blob)) as visual,              mock.patch.object(ref,"make_capture_pick_plan",return_value=plan):
            _g,_l,returned=ref.recalculate_capture_retry_plan(robot,object(),remaining,flip_x=False)
        self.assertIs(returned,plan)
        self.assertEqual(visual.call_args.kwargs["target_ny"],ref.CAPTURE_RETRY_TARGET_NY)

    def test_precapture_fourteenth_bounded_creep_can_reach_overlap(self):
        ref = load_reference()
        # Post-drive samples are now carried into the next loop head instead of
        # re-reading the same stopped state a second time.
        values = [.392, .412, .433, .454, .475, .498, .521, .544,
                  .569, .596, .621, .650, .677, .704, .733]
        blobs = [types.SimpleNamespace(nx=.50, ny=v, area=6000+i*100) for i, v in enumerate(values)]
        samples = iter((b, b.ny) for b in blobs)

        class Robot:
            dry_run = True
            pose = dict(ref.PRECAPTURE_ARM_POSE)
            def __init__(self): self.drives = 0
            def stop(self): pass
            def drive(self, kind, speed, duration): self.drives += 1

        robot = Robot(); gaze = ref.Gaze(1500, ref.PRECAPTURE_ARM_POSE[3]); lock = object()
        with mock.patch.object(ref, "fine_align_horizontal", return_value=gaze), \
             mock.patch.object(ref, "_capture_sample", side_effect=lambda *a: next(samples)):
            _gaze, _lock, blob = ref.visual_precapture_approach(
                robot, object(), lock, gaze, flip_x=False
            )
        self.assertEqual(ref.PRECAPTURE_MAX_CREEP_PULSES, 14)
        self.assertEqual(robot.drives, 14)
        self.assertGreaterEqual(blob.ny, ref.PRECAPTURE_TARGET_NY - ref.PRECAPTURE_TARGET_TOLERANCE_NY)
        self.assertLess(blob.ny, ref.PRECAPTURE_TOO_CLOSE_NY)

    def test_visual_capture_seventh_bounded_creep_can_finish_measured_camera_window(self):
        ref = load_reference()
        # The verified post-drive sample is reused as the next loop-head sample.
        values = [.135, .160, .192, .244, .298, .354, .406, .458]
        blobs = [types.SimpleNamespace(nx=.50, ny=v, area=15000+i*500) for i, v in enumerate(values)]
        samples = iter((b, b.ny) for b in blobs)

        class Robot:
            dry_run = True
            def __init__(self): self.drives = 0
            def stop(self): pass
            def drive(self, kind, speed, duration):
                self.drives += 1
                self.assertions = (kind, speed, duration)

        robot = Robot()
        gaze = ref.Gaze(1500, 600)
        with mock.patch.object(ref, "fine_align_horizontal", return_value=gaze), \
             mock.patch.object(ref, "_capture_sample", side_effect=lambda *a: next(samples)):
            returned_gaze, returned_blob = ref.visual_capture_approach(
                robot, object(), object(), gaze
            )
        self.assertIs(returned_gaze, gaze)
        self.assertIs(returned_blob, blobs[-1])
        self.assertEqual(robot.drives, 7)
        self.assertEqual(ref.CAPTURE_MAX_CREEP_PULSES, 7)
        self.assertGreaterEqual(returned_blob.ny, ref.CAPTURE_TARGET_NY - ref.CAPTURE_TARGET_TOLERANCE_NY)
        self.assertLess(returned_blob.ny, ref.CAPTURE_TOO_CLOSE_NY)

    def test_visual_capture_too_close_retreats_into_final_hand_eye_window(self):
        ref = load_reference()
        values = [.512, .482, .454]
        blobs = [types.SimpleNamespace(nx=.50, ny=v, area=13000) for v in values]
        samples = iter((b, b.ny) for b in blobs)

        class Lock:
            def __init__(self): self.ego = 0
            def note_ego_motion(self): self.ego += 1

        class Robot:
            dry_run = True
            def __init__(self): self.drives = []
            def stop(self): pass
            def drive(self, kind, speed, duration): self.drives.append((kind, speed, duration))

        robot = Robot()
        lock = Lock()
        gaze = ref.Gaze(1500, 600)
        with mock.patch.object(ref, "_capture_sample", side_effect=lambda *a: next(samples)):
            returned_gaze, returned_blob = ref.visual_capture_approach(
                robot, object(), lock, gaze, target_ny=ref.CAPTURE_TARGET_NY
            )
        self.assertIs(returned_gaze, gaze)
        self.assertIs(returned_blob, blobs[-1])
        self.assertEqual(len(robot.drives), 2)
        self.assertTrue(all(d[0] == "backward" for d in robot.drives))
        self.assertTrue(all(d[2] == ref.CAPTURE_RETREAT_SECONDS for d in robot.drives))
        self.assertEqual(lock.ego, 2)
        self.assertLessEqual(returned_blob.ny, ref.CAPTURE_TARGET_NY + ref.CAPTURE_TARGET_TOLERANCE_NY)

    def test_visual_capture_too_close_retreats_to_early_face_staging_window(self):
        ref = load_reference()
        values = [.515, .440, .360, .320]
        blobs = [types.SimpleNamespace(nx=.50, ny=v, area=13000) for v in values]
        samples = iter((b, b.ny) for b in blobs)

        class Robot:
            dry_run = True
            def __init__(self): self.drives = []
            def stop(self): pass
            def drive(self, kind, speed, duration): self.drives.append((kind, speed, duration))

        robot = Robot()
        gaze = ref.Gaze(1500, 600)
        with mock.patch.object(ref, "_capture_sample", side_effect=lambda *a: next(samples)):
            _, returned_blob = ref.visual_capture_approach(
                robot, object(), object(), gaze, target_ny=ref.ARM_FACE_CAPTURE_TARGET_NY
            )
        self.assertEqual(len(robot.drives), 3)
        self.assertLessEqual(
            returned_blob.ny,
            ref.ARM_FACE_CAPTURE_TARGET_NY + ref.CAPTURE_TARGET_TOLERANCE_NY,
        )

    def test_visual_capture_requested_window_overshoot_retreats_before_face_measurement(self):
        ref = load_reference()
        # Exact SIM anomaly: a 60 ms creep jumped from .258 to .333 while the
        # early face window is .300 +/- .025.  .333 is below the global
        # CAPTURE_TOO_CLOSE_NY guard but must not be accepted as capture-ready.
        values = [.333, .308]
        blobs = [types.SimpleNamespace(nx=.45, ny=v, area=18000) for v in values]
        samples = iter((b, b.ny) for b in blobs)

        class Robot:
            dry_run = True
            def __init__(self): self.drives = []
            def stop(self): pass
            def drive(self, kind, speed, duration): self.drives.append((kind, speed, duration))

        robot = Robot()
        gaze = ref.Gaze(1500, 600)
        with mock.patch.object(ref, "_capture_sample", side_effect=lambda *a: next(samples)):
            _, returned_blob = ref.visual_capture_approach(
                robot, object(), object(), gaze, target_nx=.45,
                target_ny=ref.ARM_FACE_CAPTURE_TARGET_NY,
            )
        self.assertEqual(robot.drives, [("backward", ref.MOTOR_SPEED, ref.CAPTURE_RETREAT_SECONDS)])
        self.assertIs(returned_blob, blobs[-1])
        self.assertLessEqual(
            returned_blob.ny,
            ref.ARM_FACE_CAPTURE_TARGET_NY + ref.CAPTURE_TARGET_TOLERANCE_NY,
        )

    def test_visual_capture_ready_does_not_move_chassis(self):
        ref=load_reference()
        blob=types.SimpleNamespace(nx=.50,ny=.445,area=11000)
        class Robot:
            dry_run=True
            def stop(self): pass
            def drive(self,*a,**k): raise AssertionError("capture-ready target must not drive")
        gaze=ref.Gaze(1500,600)
        with mock.patch.object(ref,"fine_align_horizontal",return_value=gaze),              mock.patch.object(ref,"_capture_sample",return_value=(blob,.445)):
            returned_gaze,returned_blob=ref.visual_capture_approach(Robot(),object(),object(),gaze)
        self.assertIs(returned_gaze,gaze)
        self.assertIs(returned_blob,blob)

    def test_hand_eye_capture_target_is_above_camera_center(self):
        ref = load_reference()
        target = ref.capture_target_ny_from_hand_eye()
        self.assertAlmostEqual(target, .442, delta=.004)
        self.assertLess(target, .5)
        # The old camera-centre policy (.52+) is outside the calibrated TCP window.
        self.assertGreater(.52, ref.CAPTURE_TOO_CLOSE_NY)

    def test_hand_eye_capture_exposes_explicit_tcp_pixel_not_implicit_camera_center(self):
        ref = load_reference()
        nx, ny = ref.capture_target_pixel_from_hand_eye()
        self.assertAlmostEqual(nx, ref.CAPTURE_TARGET_NX, places=6)
        self.assertAlmostEqual(ny, ref.CAPTURE_TARGET_NY, delta=.004)
        self.assertLess(ny, .5)

    def test_target_lock_accepts_same_shape_larger_jump_immediately_after_ego_motion(self):
        ref = load_reference()
        seed = types.SimpleNamespace(nx=.547, ny=.494, area=29741, width=186, height=178)
        moved = types.SimpleNamespace(nx=.619, ny=.244, area=26390, width=168, height=179)
        lock = ref.TargetLock(seed)
        self.assertIsNone(lock.select(moved))
        lock = ref.TargetLock(seed)
        lock.note_ego_motion()
        self.assertIs(lock.select(moved), moved)

    def test_final_horizontal_alignment_waits_for_delayed_pan_response(self):
        ref = load_reference()
        blobs = [
            types.SimpleNamespace(nx=.348, ny=.44, area=8000),
            types.SimpleNamespace(nx=.348, ny=.44, area=8000),
            types.SimpleNamespace(nx=.49, ny=.44, area=8000),
            types.SimpleNamespace(nx=.50, ny=.44, area=8000),
            types.SimpleNamespace(nx=.50, ny=.44, area=8000),
        ]
        gaze = ref.Gaze(1500, 600)
        moves = []
        class Robot:
            def stop(self): pass
            def nudge_servos(self, updates, duration): moves.append(dict(updates))
        with mock.patch.object(ref, "read_locked", side_effect=[(object(), b) for b in blobs]), \
             mock.patch.object(ref.time, "sleep"):
            ref.fine_align_horizontal(Robot(), object(), object(), gaze, flip_x=False)
        # The duplicate stale nx frame must not cause a second large pan command.
        self.assertLessEqual(len(moves), 2)

    def test_final_horizontal_alignment_uses_coarse_pan_step_for_large_error(self):
        ref = load_reference()
        blobs = [
            types.SimpleNamespace(nx=.40, ny=.44, area=8000),
            types.SimpleNamespace(nx=.50, ny=.44, area=8000),
            types.SimpleNamespace(nx=.50, ny=.44, area=8000),
            types.SimpleNamespace(nx=.50, ny=.44, area=8000),
        ]
        gaze = ref.Gaze(1500, 600)
        moves = []
        class Robot:
            def stop(self): pass
            def nudge_servos(self, updates, duration): moves.append(dict(updates))
        with mock.patch.object(
            ref, "read_locked", side_effect=[(object(),b) for b in blobs]
        ), mock.patch.object(ref.time,"sleep"):
            ref.fine_align_horizontal(Robot(),object(),object(),gaze,flip_x=False)
        self.assertEqual(moves[0][ref.PAN_SERVO], 1500 + ref.FINAL_X_COARSE_MAX_STEP_PULSE)
        self.assertGreater(ref.FINAL_X_COARSE_MAX_STEP_PULSE, ref.FINAL_X_MAX_STEP_PULSE)

    def test_capture_pose_preserves_safe_precapture_pan(self):
        ref = load_reference()
        start = ref.Gaze(1536, ref.PRECAPTURE_ARM_POSE[3])
        calls=[]
        blob=types.SimpleNamespace(nx=.50,ny=.20,area=9000)
        class Robot:
            dry_run=True
            pose={**ref.PRECAPTURE_ARM_POSE,6:1536}
            def stop(self): pass
            def nudge_servos(self,updates,duration):
                calls.append(dict(updates)); self.pose.update(updates)
        video=types.SimpleNamespace(read=mock.Mock(return_value=object()))
        capture_lock=object()
        with mock.patch.object(ref,"detect_red_blob",return_value=blob), \
             mock.patch.object(ref,"confirmed_candidate",return_value=blob), \
             mock.patch.object(ref,"TargetLock",return_value=capture_lock), \
             mock.patch.object(ref,"fine_align_horizontal",side_effect=lambda r,v,l,g,**k:g):
            returned, lock=ref.establish_capture_pose(Robot(),video,object(),start)
        self.assertEqual(calls[0][6],1536)
        self.assertEqual(returned.pan,1536)
        self.assertIs(lock,capture_lock)

    def test_capture_pose_preserves_valid_hard_limit_precapture_pan(self):
        ref = load_reference()
        start = ref.Gaze(ref.PAN_MIN, ref.PRECAPTURE_ARM_POSE[3])
        calls=[]
        blob=types.SimpleNamespace(nx=.50,ny=.20,area=9000)
        class Robot:
            dry_run=True
            pose={**ref.PRECAPTURE_ARM_POSE,6:ref.PAN_MIN}
            def stop(self): pass
            def nudge_servos(self,updates,duration):
                calls.append(dict(updates)); self.pose.update(updates)
        video=types.SimpleNamespace(read=mock.Mock(return_value=object()))
        capture_lock=object()
        with mock.patch.object(ref,"detect_red_blob",return_value=blob), \
             mock.patch.object(ref,"confirmed_candidate",return_value=blob), \
             mock.patch.object(ref,"TargetLock",return_value=capture_lock), \
             mock.patch.object(ref,"fine_align_horizontal",side_effect=lambda r,v,l,g,**k:g):
            returned, lock=ref.establish_capture_pose(Robot(),video,object(),start)
        self.assertEqual(calls[0][6],ref.PAN_MIN)
        self.assertEqual(returned.pan,ref.PAN_MIN)
        self.assertIs(lock,capture_lock)

    def test_final_horizontal_alignment_honors_noncenter_tcp_target_x(self):
        ref = load_reference()
        blob = types.SimpleNamespace(nx=.42, ny=.44, area=8000)
        gaze = ref.Gaze(1500, 600)
        class Robot:
            def stop(self): pass
            def nudge_servos(self, *args, **kwargs):
                raise AssertionError("already at TCP target x; camera-centre correction is wrong")
        with mock.patch.object(ref, "read_locked", return_value=(object(), blob)), \
             mock.patch.object(ref.time, "sleep"):
            returned = ref.fine_align_horizontal(
                Robot(), object(), object(), gaze,
                flip_x=False, target_nx=.42,
            )
        self.assertIs(returned, gaze)

    def test_grasp_verifier_does_not_call_moved_red_a_success(self):
        ref = load_reference()
        moved = types.SimpleNamespace(nx=.80, ny=.70, area=3500)
        summary = ref.RedSummary(
            hits=7, total=9, nx=.80, ny=.70, area=3500,
            last_blob=moved, last_frame=None,
        )
        reference = ref.VerificationReference(
            pose={3:600,4:2200,5:1900,6:1500},
            summary=ref.RedSummary(
                hits=9,total=9,nx=.50,ny=.44,area=5000,
                last_blob=types.SimpleNamespace(nx=.50,ny=.44,area=5000),
                last_frame=None,
            ),
        )
        class Robot:
            dry_run=True
            pose={3:600,4:2200,5:1900,6:1500}
            def move_servo(self,*args): pass
            def nudge_servos(self,updates,duration): self.pose.update(updates)
            def stop(self): pass
        with mock.patch.object(ref,"capture_red_summary",return_value=summary) as capture:
            clear, remaining = ref.verify_grasp(Robot(), object(), reference)
        self.assertFalse(clear)
        self.assertIs(remaining, summary)
        # Verification inspects the whole view but only counts a detection if
        # it is geometrically compatible with the floor plane at the saved pose.
        self.assertIsNone(capture.call_args.kwargs["reference"])
        self.assertEqual(capture.call_args.kwargs["floor_pose"], reference.pose)

    def test_grasp_verification_floor_filter_ignores_visible_held_cube(self):
        ref = load_reference()
        frame = object()
        held = types.SimpleNamespace(nx=.66, ny=.84, area=24000, height=150)
        floor = types.SimpleNamespace(nx=.49, ny=.39, area=22500, height=150)
        pose = {3:600, 4:2200, 5:1900, 6:1500}
        # Held-looking red cannot intersect the floor from this camera pose;
        # a missed/pushed cube still can.
        with mock.patch.object(ref, "capture_bgr", return_value=frame), \
             mock.patch.object(ref, "detect_red_blob", side_effect=[held] * ref.VERIFY_FRAME_COUNT), \
             mock.patch.object(ref, "estimate_block", return_value=None), \
             mock.patch.object(ref.time, "sleep", return_value=None):
            summary = ref.capture_red_summary(object(), floor_pose=pose)
        self.assertEqual(summary.hits, 0)

        floor_est = types.SimpleNamespace(radius_cm=12.5, source="floor_ray")
        with mock.patch.object(ref, "capture_bgr", return_value=frame), \
             mock.patch.object(ref, "detect_red_blob", side_effect=[floor] * ref.VERIFY_FRAME_COUNT), \
             mock.patch.object(ref, "estimate_block", return_value=floor_est), \
             mock.patch.object(ref.time, "sleep", return_value=None):
            summary = ref.capture_red_summary(object(), floor_pose=pose)
        self.assertEqual(summary.hits, ref.VERIFY_FRAME_COUNT)

    def test_grasp_verification_rejects_size_fallback_as_floor_evidence(self):
        ref = load_reference()
        frame = object()
        held = types.SimpleNamespace(nx=.45, ny=.85, area=40000, height=180)
        pose = {3:600, 4:2200, 5:1900, 6:1437}
        # Reproduce the live false-negative shape: the held cube is visible, but
        # the floor ray is impossible and estimate_block degrades to size range.
        size_only = types.SimpleNamespace(radius_cm=6.6, source="size_apparent")
        with mock.patch.object(ref, "capture_bgr", return_value=frame), \
             mock.patch.object(ref, "detect_red_blob", side_effect=[held] * ref.VERIFY_FRAME_COUNT), \
             mock.patch.object(ref, "estimate_block", return_value=size_only), \
             mock.patch.object(ref.time, "sleep", return_value=None):
            summary = ref.capture_red_summary(
                object(), floor_pose=pose, decision="floor_clear"
            )
        self.assertEqual(summary.hits, 0)

    def test_final_creep_is_half_or_less_of_old_near_pulse(self):
        ref = load_reference()
        self.assertLessEqual(ref.FINAL_CREEP_SECONDS, ref.NEAR_FORWARD_SECONDS / 2.0)
        self.assertGreaterEqual(ref.CHASSIS_HARD_LIMIT_CM, 14.0)
        self.assertGreater(ref.STOP_RADIUS_CM, ref.CHASSIS_HARD_LIMIT_CM)

    def test_near_floor_limit_accepts_tracker_deadband_instead_of_waiting_forever(self):
        ref = load_reference()
        blob = types.SimpleNamespace(nx=.450, ny=.696, area=1000)
        gaze = ref.Gaze(pan=1520, tilt=ref.TILT_MIN)

        class Robot:
            def stop(self): pass

        with mock.patch.object(ref, "read_locked", return_value=(object(), blob)), \
             mock.patch.object(ref, "tracking_update", side_effect=lambda robot, blob, gaze, **kw: (gaze, 0.0, False)), \
             mock.patch.object(ref.time, "sleep", return_value=None):
            returned = ref.centre_gaze(
                Robot(), object(), object(), gaze,
                flip_x=False, flip_y=False, confirmations=3, max_frames=3,
            )
        self.assertIs(returned, gaze)
        self.assertGreaterEqual(ref.IMAGE_X_DEADBAND, ref.TRACK_DEADBAND)


    def test_capture_window_exhaustion_is_progress_stall_with_search_recovery(self):
        from scripts.robot_actions import classify_failure
        code, required, recovery = classify_failure(
            "approach", "could not bring red block into calibrated visual capture window"
        )
        self.assertEqual(code, "APPROACH_PROGRESS_STALLED")
        self.assertEqual(required, "arm.pose=SEARCH_TRACK")
        self.assertEqual(recovery, "search")

if __name__ == "__main__":
    unittest.main(verbosity=2)
