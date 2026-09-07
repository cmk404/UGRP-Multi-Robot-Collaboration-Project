from __future__ import annotations

import tempfile
import unittest
from unittest import mock
from pathlib import Path

from harness.catalog import default_registry
from harness.executive import TaskExecutive
from harness.loop import ReplayCompleter, run_loop
from harness.perception import detect_red_target
from harness.state import StateEstimator


class StateEstimatorTests(unittest.TestCase):
    def test_temporal_target_state(self):
        est = StateEstimator()
        est.update_vision({"visible": True, "cx": .54, "cy": .44, "area_ratio": .02})
        est.update_vision({"visible": True, "cx": .51, "cy": .64, "area_ratio": .05})
        state = est.state
        self.assertTrue(state.target.visible)
        self.assertGreaterEqual(state.target.visible_streak, 2)
        self.assertTrue(state.target.centered)
        self.assertEqual(state.target.range_class, "NEAR")  # filtered range does not jump on one frame
        est.update_vision({"visible": True, "cx": .50, "cy": .70, "area_ratio": .07})
        self.assertEqual(est.state.target.range_class, "PREGRASP")

    def test_pick_execution_does_not_claim_grasp_success(self):
        est = StateEstimator()
        est.update_tool_result({
            "result": {
                "skill": "pick",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN",
            }
        })
        self.assertEqual(est.state.grasp.state, "UNKNOWN")
        self.assertEqual(est.state.last_action.execution_status, "COMPLETED")
        self.assertEqual(est.state.last_action.outcome_status, "UNKNOWN")

    def test_verified_put_down_clears_carried_state(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        est.update_tool_result({
            "result": {
                "skill": "put_down",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "release_verified": True,
            }
        })
        self.assertEqual(est.state.grasp.state, "EMPTY")
        self.assertIsNone(est.state.grasp.held_object_color)
        self.assertEqual(est.state.task.phase, "RELEASED")

    def test_operator_can_clear_probable_but_not_verified_hold(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        self.assertTrue(est.accept_operator_empty_gripper())
        self.assertEqual(est.state.grasp.state, "EMPTY")
        est.state.grasp.state = "HELD"
        est.state.grasp.held_object_color = "blue"
        self.assertFalse(est.accept_operator_empty_gripper())
        self.assertEqual(est.state.grasp.held_object_color, "blue")

    def test_state_keeps_selected_held_and_destination_identities_separate(self):
        est = StateEstimator()
        est.set_goal_context(target_color="blue", destination_color="yellow")
        est.update_tool_result({
            "result": {
                "skill": "pick", "target_color": "blue",
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN", "visual_hold": {"probable": True, "confidence": .65},
            }
        })
        state = est.state
        self.assertEqual(state.target.selected_color, "blue")
        self.assertEqual(state.grasp.held_object_color, "blue")
        self.assertEqual(state.task.destination_color, "yellow")


class ExecutivePlanTests(unittest.TestCase):
    def test_sim_grasp_includes_verified_carry_lift(self):
        goal = __import__("harness.goals", fromlist=["infer_goal"]).infer_goal("빨간 블록 집어")
        plan = TaskExecutive().plan(goal, ["search", "track", "approach", "pick", "carry"])
        self.assertEqual(plan, ["search", "track", "approach", "pick", "carry"])

    def test_real_grasp_without_carry_stays_adapter_safe(self):
        goal = __import__("harness.goals", fromlist=["infer_goal"]).infer_goal("빨간 블록 집어")
        plan = TaskExecutive().plan(goal, ["search", "track", "approach", "pick"])
        self.assertEqual(plan, ["search", "track", "approach", "pick"])


    def test_track_postcondition_survives_off_center_passive_refresh(self):
        est = StateEstimator()
        est.set_goal_context(target_color="red")
        est.update_vision({"visible": True, "cx": .38, "cy": .47, "area_ratio": .01})
        self.assertFalse(est.state.target.centered)
        est.update_tool_result({
            "ok": True, "tool": "track",
            "result": {
                "skill": "track", "target_color": "red",
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED", "center_verified": True,
            },
        })
        self.assertTrue(est.state.target.centered)
        # Servo-limit locks are valid TRACK outcomes even with off-centre cx.
        est.update_vision({"visible": True, "cx": .377, "cy": .466, "area_ratio": .0075})
        self.assertTrue(est.state.target.centered)

    def test_track_alignment_latch_clears_on_target_loss_or_new_search(self):
        est = StateEstimator()
        est.update_tool_result({
            "ok": True, "tool": "track",
            "result": {"skill": "track", "outcome_status": "ACHIEVED"},
        })
        self.assertTrue(est.state.target.centered)
        est.update_vision({"visible": False})
        self.assertFalse(est.state.target.centered)
        est.update_tool_result({
            "ok": True, "tool": "track",
            "result": {"skill": "track", "outcome_status": "ACHIEVED"},
        })
        est.update_tool_result({
            "ok": True, "tool": "search",
            "result": {"skill": "search", "outcome_status": "ACHIEVED"},
        })
        est.update_vision({"visible": True, "cx": .38, "cy": .47, "area_ratio": .01})
        self.assertFalse(est.state.target.centered)

    def test_real_grasp_skips_proven_search_and_track_but_keeps_approach(self):
        goal = __import__("harness.goals", fromlist=["infer_goal"]).infer_goal("빨간 블록 집어")
        est = StateEstimator()
        est.set_goal_context(target_color="red")
        est.state.target.visible = True
        est.state.target.centered = True
        est.state.spatial_memory["red"] = {"visible": True, "image_xy": [0.50, 0.50]}
        plan = TaskExecutive(strict_pick_preconditions=True).plan(
            goal, ["search", "track", "approach", "pick", "place"], est.state
        )
        self.assertEqual(plan, ["approach", "pick"])

    def test_real_grasp_does_not_skip_from_stale_other_colour_target_state(self):
        goal = __import__("harness.goals", fromlist=["infer_goal"]).infer_goal("파란 블록 집어")
        est = StateEstimator()
        # Simulate red being the previously selected visible/centered target,
        # then goal-context switching to blue before a fresh camera frame arrives.
        est.state.target.visible = True
        est.state.target.centered = True
        est.state.spatial_memory["red"] = {"visible": True, "image_xy": [0.50, 0.50]}
        est.set_goal_context(target_color="blue")
        plan = TaskExecutive(strict_pick_preconditions=True).plan(
            goal, ["search", "track", "approach", "pick", "place"], est.state
        )
        self.assertEqual(plan, ["search", "track", "approach", "pick"])

    def test_real_grasp_keeps_track_when_visible_but_not_centered(self):
        goal = __import__("harness.goals", fromlist=["infer_goal"]).infer_goal("빨간 블록 집어")
        est = StateEstimator()
        est.set_goal_context(target_color="red")
        est.state.target.visible = True
        est.state.target.centered = False
        est.state.spatial_memory["red"] = {"visible": True, "image_xy": [0.50, 0.50]}
        plan = TaskExecutive(strict_pick_preconditions=True).plan(
            goal, ["search", "track", "approach", "pick", "place"], est.state
        )
        self.assertEqual(plan, ["track", "approach", "pick"])


class ExecutiveTests(unittest.TestCase):
    @mock.patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
    def test_rule_baseline_can_still_execute_deterministic_recovery(self):
        seen = []

        def runner(name, **kwargs):
            seen.append(name)
            if name == "search":
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED",
                    "camera_red": {"visible": True, "cx": .50, "cy": .30, "area_ratio": .01},
                }
            if name == "approach":
                return {
                    "ok": True, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED",
                    "camera_red": {"visible": True, "cx": .50, "cy": .72, "area_ratio": .08},
                }
            return {
                "ok": True, "skill": name,
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN",
                "camera_red": {"visible": False, "pixels": 0},
            }

        est = StateEstimator()
        est.update_vision({"visible": False, "pixels": 0})
        result = run_loop(
            ReplayCompleter([
                {"tool": "pick"},
                {"final": "집기 동작을 수행했어"},
                {"final": "집기 성공 여부를 센서로 확인할 수 없어."},
            ]),
            default_registry(runner=runner, actions_path="scripts/sim_actions.py"),
            "빨간 블록을 집어",
            execute=True,
            auto_observe=True,
            observe=lambda: None,
            state_estimator=est,
            executive=TaskExecutive(),
        )
        self.assertEqual(seen, ["search", "track", "search", "approach", "pick"])
        self.assertEqual(result.stopped, "final")
        recovery_names = [
            step.action.name for step in result.steps
            if step.raw.startswith("executive_recovery:")
        ]
        self.assertEqual(recovery_names, ["search"])
        self.assertEqual(result.world_state["last_action"]["name"], "pick")
        self.assertEqual(result.world_state["last_action"]["outcome_status"], "UNKNOWN")

    def test_default_vlm_mode_rejects_without_hidden_recovery(self):
        seen = []
        planner_inputs = []

        class CapturingCompleter:
            def __init__(self):
                self.calls = 0

            def complete(self, messages, image=None):
                del image
                self.calls += 1
                planner_inputs.append(list(messages))
                if self.calls == 1:
                    return '{"tool":"pick","args":{"target_color":"red"}}'
                if self.calls == 2:
                    # The planner, not Executive, chooses the recovery action.
                    return '{"tool":"search","args":{"target_color":"red"}}'
                return '{"final":"아직 집지는 못했어."}'

        def runner(name, **kwargs):
            seen.append((name, kwargs))
            return {
                "ok": True, "skill": name, **kwargs,
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                # Deliberately prescriptive debug text: this must stay out of
                # the next VLM prompt in autonomous mode.
                "reason": "run approach next",
                "camera_red": {"visible": True, "cx": .5, "cy": .3, "area_ratio": .01},
            }

        est = StateEstimator()
        est.set_goal_context(target_color="red")
        est.update_vision({"visible": False, "pixels": 0})
        completer = CapturingCompleter()
        result = run_loop(
            completer,
            default_registry(runner=runner, actions_path="scripts/robot_actions.py"),
            "빨간 블록을 집어",
            execute=True, auto_observe=True, observe=lambda: None,
            state_estimator=est, executive=TaskExecutive(strict_pick_preconditions=True),
        )

        self.assertEqual(seen, [("search", {"target_color": "red"})])
        self.assertFalse(any(step.raw.startswith("executive_recovery:") for step in result.steps))
        first = next(step for step in result.steps if getattr(step.action, "name", None) == "pick")
        self.assertEqual(first.result["result"]["failure_code"], "TARGET_NOT_VISIBLE")
        self.assertNotIn("recommended_recovery", first.result["result"])
        second_prompt = "\n".join(str(m.get("content", "")) for m in planner_inputs[1])
        self.assertIn("TARGET_NOT_VISIBLE", second_prompt)
        self.assertIn("target.visible=true", second_prompt)
        self.assertNotIn('"recommended_recovery": "search"', second_prompt)
        self.assertNotIn("target is not visible", second_prompt)
        third_prompt = "\n".join(str(m.get("content", "")) for m in planner_inputs[2])
        self.assertNotIn("run approach next", third_prompt)


    def test_repeated_rejection_reports_stagnation_without_recovery_policy(self):
        planner_inputs = []

        class RepeatingCompleter:
            def __init__(self):
                self.calls = 0
            def complete(self, messages, image=None):
                del image
                self.calls += 1
                planner_inputs.append(list(messages))
                if self.calls <= 2:
                    return '{"tool":"track","args":{"target_color":"red"}}'
                return '{"final":"다른 판단이 필요해."}'

        est = StateEstimator()
        est.set_goal_context(target_color="red")
        est.update_vision({"visible": False})
        result = run_loop(
            RepeatingCompleter(),
            default_registry(runner=lambda name, **kwargs: {"ok": True}, actions_path="scripts/robot_actions.py"),
            "빨간 블록을 집어",
            execute=True, auto_observe=True, observe=lambda: None,
            state_estimator=est, executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(result.stopped, "final")
        second_rejection_prompt = "\n".join(str(m.get("content", "")) for m in planner_inputs[2])
        self.assertGreaterEqual(second_rejection_prompt.count('"tool": "track"'), 2)
        self.assertIn('"status": "BLOCKED"', second_rejection_prompt)
        self.assertNotIn('"recommended_recovery": "search"', second_rejection_prompt)
        # The loop reports infeasibility but does not secretly execute another skill.
        self.assertFalse(any(step.raw.startswith("executive_recovery:") for step in result.steps))

    def test_contract_is_exposed_by_registry(self):
        registry = default_registry(actions_path="scripts/sim_actions.py", runner=lambda name, **kwargs: {})
        contract = registry.get("pick").contract
        self.assertIn("target.range_class=PREGRASP", contract["preconditions"])
        self.assertEqual(contract["recoveries"]["TARGET_TOO_FAR"], "approach")


class PerceptionTests(unittest.TestCase):
    def test_red_detector_from_rgb_image(self):
        try:
            from PIL import Image, ImageDraw
        except Exception:
            self.skipTest("Pillow unavailable")
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "red.png"
            im = Image.new("RGB", (100, 100), (30, 30, 30))
            draw = ImageDraw.Draw(im)
            draw.rectangle((45, 60, 65, 80), fill=(255, 0, 0))
            im.save(path)
            det = detect_red_target(path)
        self.assertTrue(det["visible"])
        self.assertGreater(det["cy"], .6)
        self.assertGreater(det["area_ratio"], .03)


if __name__ == "__main__":
    unittest.main()

class GoalGateTests(unittest.TestCase):
    def test_success_final_blocked_when_grasp_unknown(self):
        result = run_loop(
            ReplayCompleter([
                {"tool": "pick"},
                {"final": "집었어"},
                {"final": "집기 성공 여부를 센서로 확인할 수 없어."},
            ]),
            default_registry(runner=lambda name, **kwargs: {
                "ok": True,
                "skill": name,
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN",
            }, actions_path="scripts/sim_actions.py"),
            "빨간 블록을 집어",
            execute=True,
            state_estimator=StateEstimator(),
            executive=TaskExecutive(),
        )
        self.assertEqual(result.final, "집기 성공 여부를 센서로 확인할 수 없어.")
        self.assertTrue(any("goal predicate not satisfied" in (s.error or "") for s in result.steps))

    def test_failure_report_can_stop_when_goal_unknown(self):
        result = run_loop(
            ReplayCompleter([{"tool": "pick"}, {"final": "성공 여부를 확인할 수 없어 중단했어."}]),
            default_registry(runner=lambda name, **kwargs: {
                "ok": True, "skill": name,
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN",
            }, actions_path="scripts/sim_actions.py"),
            "빨간 블록을 집어",
            execute=True,
        )
        self.assertEqual(result.stopped, "final")

@mock.patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
class StructuredGoalNoLlmTests(unittest.TestCase):
    def test_explicit_red_lift_uses_executive_without_completer(self):
        from harness.loop import run_loop
        from harness.catalog import default_registry

        class NoCallCompleter:
            def complete(self, messages, image=None):
                raise AssertionError("LLM must not be called for structured red-block lift goal")

        seen = []
        def runner(name, **kwargs):
            seen.append(name)
            vision = {"visible": True, "cx": 0.5, "cy": 0.7, "area_ratio": 0.05}
            return {
                "ok": True,
                "skill": name,
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED" if name in {"search", "track", "approach"} else "UNKNOWN",
                "camera_red": vision,
            }

        registry = default_registry(actions_path="scripts/sim_actions.py", runner=runner)
        import os
        old_flag = os.environ.get("UGRP_STRUCTURED_RED_FASTPATH")
        os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = "1"
        try:
            result = run_loop(
                NoCallCompleter(), registry,
                "빨간 블록을 집어서 들어줘",
                execute=True, auto_observe=True,
                observe=lambda: None,
                max_steps=16,
            )
        finally:
            if old_flag is None:
                os.environ.pop("UGRP_STRUCTURED_RED_FASTPATH", None)
            else:
                os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = old_flag
        self.assertEqual(result.stopped, "final")
        self.assertIn("확정할 수 없어", result.final or "")
        self.assertIn("pick", seen)
        # REAL pick already ends in the carry pose; standalone carry was removed
        # from the public manipulation pipeline.
        self.assertNotIn("carry", seen)

    def test_wrong_held_color_is_released_then_requested_color_pipeline_runs_without_llm(self):
        class NoCallCompleter:
            def complete(self, messages, image=None):
                raise AssertionError("wrong-object recovery must be deterministic")

        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        seen = []

        def runner(name, **kwargs):
            seen.append((name, kwargs))
            base = {
                "ok": True, "skill": name, **kwargs,
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
            }
            if name == "put_down":
                return {**base, "outcome_status": "ACHIEVED", "release_verified": True}
            vision = {"visible": True, "cx": .50, "cy": .70, "area_ratio": .08}
            if name == "search":
                return {**base, "outcome_status": "ACHIEVED", "target_vision": vision}
            if name == "track":
                return {**base, "outcome_status": "ACHIEVED", "target_vision": vision, "center_verified": True}
            if name == "approach":
                return {
                    **base, "outcome_status": "ACHIEVED", "target_vision": vision,
                    "center_verified": True, "range_verified": True,
                }
            if name == "pick":
                return {
                    **base, "outcome_status": "UNKNOWN",
                    "visual_hold": {"probable": True, "confidence": .65},
                }
            raise AssertionError(name)

        result = run_loop(
            NoCallCompleter(),
            default_registry(actions_path="scripts/robot_actions.py", runner=runner),
            "파란 블럭 집어봐",
            execute=True, auto_observe=True, observe=lambda: None, max_steps=10,
            state_estimator=est, executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(
            seen,
            [
                ("put_down", {}),
                ("search", {"target_color": "blue"}),
                ("track", {"target_color": "blue"}),
                ("approach", {"target_color": "blue"}),
                ("pick", {"target_color": "blue"}),
            ],
        )
        self.assertEqual(result.world_state["grasp"]["held_object_color"], "blue")
        self.assertEqual(result.world_state["grasp"]["state"], "PROBABLE_HELD")
        self.assertFalse(any("OBJECT_ALREADY_CARRIED" in (step.error or "") for step in result.steps))
        self.assertFalse(any("plan expects" in (step.error or "") for step in result.steps))

    def test_exact_destination_only_followup_never_falls_back_to_search(self):
        from harness.catalog import default_registry
        from harness.loop import run_loop
        from harness.state import StateEstimator

        class NoCallCompleter:
            def complete(self, messages, image=None):
                raise AssertionError("destination-only held placement must be deterministic")

        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        seen = []

        def runner(name, **kwargs):
            seen.append((name, kwargs))
            if name not in {"search_destination", "place"}:
                raise AssertionError(f"must not restart pickup acquisition while held: {name}")
            return {
                "ok": True, "skill": name, **kwargs,
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "destination_visible": name == "search_destination",
                "place_verified": name == "place",
            }

        result = run_loop(
            NoCallCompleter(),
            default_registry(actions_path="scripts/robot_actions.py", runner=runner),
            "파란색 블럭 위에 올려봐",
            execute=True, auto_observe=True, observe=lambda: None,
            state_estimator=est, executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(
            seen,
            [
                ("search_destination", {"target_color": "red", "destination_color": "blue"}),
                ("place", {"target_color": "red", "destination_color": "blue"}),
            ],
        )
        self.assertEqual(result.world_state["task"]["phase"], "PLACED")

    def test_destination_search_is_allowed_while_held_but_regular_search_is_not(self):
        from harness.state import StateEstimator
        est = StateEstimator()
        est.state.grasp.state = "HELD"
        est.state.grasp.held_object_color = "red"
        ex = TaskExecutive(strict_pick_preconditions=True)
        ordinary = ex.check("search", est.state, {"target_color": "blue"})
        self.assertFalse(ordinary.allowed)
        self.assertEqual(ordinary.failure_code, "OBJECT_ALREADY_CARRIED")
        delivery = ex.check(
            "search_destination", est.state,
            {"target_color": "red", "destination_color": "blue"},
        )
        self.assertTrue(delivery.allowed)

    def test_known_delivery_loss_stops_destination_and_manual_motion_recovery(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        est.update_tool_result({"result": {
            "ok": False, "skill": "place_on_yellow",
            "command_status": "ACCEPTED", "execution_status": "FAILED",
            "outcome_status": "NOT_ACHIEVED", "failure_code": "SKILL_FAILED",
            "reason": "carried red block was lost during delivery; aborted immediately",
        }})
        self.assertEqual(est.state.grasp.state, "EMPTY")
        self.assertEqual(est.state.task.phase, "CARRY_LOST")
        executive = TaskExecutive(strict_pick_preconditions=True)
        for skill, args in (
            ("search_destination", {"target_color": "red", "destination_color": "yellow"}),
            ("turn_right", {}),
            ("move_forward", {}),
            ("search", {"target_color": "red"}),
        ):
            decision = executive.check(skill, est.state, args)
            self.assertFalse(decision.allowed)
            self.assertEqual(decision.failure_code, "CARRY_LOST_DURING_DELIVERY")
            self.assertIsNone(decision.recovery)

    def test_release_completed_but_unverified_clears_stale_carry_state(self):
        est = StateEstimator()
        est.state.grasp.state = "HELD"
        est.state.grasp.held_object_color = "red"
        est.update_tool_result({"result": {
            "ok": False, "skill": "place_on_yellow",
            "command_status": "ACCEPTED", "execution_status": "FAILED",
            "outcome_status": "NOT_ACHIEVED", "failure_code": "SKILL_FAILED",
            "reason": "release completed but red-on-yellow stack postcondition was not visually verified",
        }})
        self.assertEqual(est.state.grasp.state, "EMPTY")
        self.assertIsNone(est.state.grasp.held_object_color)
        self.assertEqual(est.state.task.phase, "CARRY_LOST")
        blocked = TaskExecutive(strict_pick_preconditions=True).check(
            "search_destination", est.state,
            {"target_color": "red", "destination_color": "yellow"},
        )
        self.assertFalse(blocked.allowed)
        self.assertIsNone(blocked.recovery)

    def test_loop_stops_after_alias_delivery_loss_before_next_destination_search(self):
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        seen = []

        def runner(name, **kwargs):
            seen.append((name, kwargs))
            if name != "place_on_yellow":
                raise AssertionError(f"must stop before stale recovery action: {name}")
            return {
                "ok": False, "skill": name,
                "command_status": "ACCEPTED", "execution_status": "FAILED",
                "outcome_status": "NOT_ACHIEVED", "failure_code": "SKILL_FAILED",
                "reason": "carried red block was lost during delivery; aborted immediately",
            }

        result = run_loop(
            ReplayCompleter([
                {"tool": "place_on_yellow"},
                {"tool": "search_destination", "args": {"target_color": "red", "destination_color": "yellow"}},
            ]),
            default_registry(actions_path="scripts/sim_actions.py", runner=runner),
            "다음 작업",
            execute=True, state_estimator=est,
            executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(seen, [("place_on_yellow", {})])
        self.assertEqual(result.stopped, "carry_lost")
        self.assertEqual(result.world_state["grasp"]["state"], "EMPTY")

    def test_followup_place_while_held_never_restarts_pick_pipeline(self):
        from harness.catalog import default_registry
        from harness.loop import run_loop
        from harness.state import StateEstimator

        class NoCallCompleter:
            def complete(self, messages, image=None):
                raise AssertionError("follow-up place is deterministic")

        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        seen = []
        def runner(name, **kwargs):
            seen.append((name, kwargs))
            self.assertIn(name, {"search_destination", "place"})
            self.assertEqual(kwargs, {"target_color": "red", "destination_color": "blue"})
            return {
                "ok": True, "skill": name, **kwargs,
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "destination_visible": name == "search_destination",
                "place_verified": name == "place",
            }

        result = run_loop(
            NoCallCompleter(),
            default_registry(actions_path="scripts/robot_actions.py", runner=runner),
            "그거 파란 블록 위에 올려",
            execute=True, auto_observe=True, observe=lambda: None,
            state_estimator=est, executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(seen, [
            ("search_destination", {"target_color": "red", "destination_color": "blue"}),
            ("place", {"target_color": "red", "destination_color": "blue"}),
        ])
        self.assertEqual(result.world_state["task"]["phase"], "PLACED")
