#!/usr/bin/env python3
import json
import pathlib
import sys
import unittest
from unittest.mock import patch


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.catalog import SKILLS, default_registry
from harness.cli import main
from harness.loop import ReplayCompleter, dispatch, run_loop
from harness.protocol import FinalAnswer, Look, Plan, ProtocolError, ToolCall, Wait, parse_action
from harness.registry import Registry, ToolError, validate_args


class ProtocolTests(unittest.TestCase):
    def test_parses_tool_call(self):
        action = parse_action('{"tool": "approach", "args": {}}')
        self.assertIsInstance(action, ToolCall)
        self.assertEqual(action.name, "approach")
        self.assertEqual(action.args, {})

    def test_parses_final_inside_prose(self):
        action = parse_action('알겠습니다.\n{"final": "도착"}\n')
        self.assertEqual(action, FinalAnswer("도착"))

    def test_strips_markdown_fence(self):
        action = parse_action("```json\n{\"tool\": \"pick\"}\n```")
        self.assertEqual(action.name, "pick")

    def test_rejects_python(self):
        with self.assertRaises(ProtocolError):
            parse_action("print('hi')")

    def test_parses_tool_call_with_say(self):
        action = parse_action(
            '{"say": "앞에 빨간 블록 보여. 다가갈게.", "tool": "approach"}'
        )
        self.assertEqual(action.name, "approach")
        self.assertEqual(action.say, "앞에 빨간 블록 보여. 다가갈게.")

    def test_say_only_is_final(self):
        action = parse_action('{"say": "어느 빨간 블록이야?"}')
        self.assertEqual(action, FinalAnswer("어느 빨간 블록이야?"))

    def test_parses_plan_wait_look(self):
        plan = parse_action('{"say": "다가간 뒤 집을게.", "plan": ["다가감", "집기"]}')
        self.assertIsInstance(plan, Plan)
        self.assertEqual(plan.steps, ("다가감", "집기"))
        wait = parse_action('{"wait": 1.5}')
        self.assertEqual(wait, Wait(1.5))
        look = parse_action('{"look": true, "say": "지금 장면"}')
        self.assertEqual(look.say, "지금 장면")
        self.assertIsInstance(look, Look)

    def test_talk_only_prompt_forbids_tools(self):
        from harness.protocol import system_prompt

        text = system_prompt("- pick", talk_only=True)
        self.assertIn("talk-only", text)
        self.assertNotIn("Available tools", text)
        self.assertNotIn("talk-only", system_prompt("- pick"))
        auto = system_prompt("- search\n- pick", auto_observe=True)
        self.assertIn('{"look": true}', auto)
        self.assertIn('never {"tool": "look"}', auto)
        self.assertIn("No situation-to-action policy is supplied", auto)
        self.assertIn("recent_action_evidence", auto)
        self.assertIn("observations only", auto)
        self.assertIn("executable=false", auto)
        self.assertIn("pregrasp_reach.available must be true", auto)
        self.assertNotIn("search instead", auto)
        self.assertNotIn("deterministic recovery", auto)
        self.assertNotIn("do not repeat", auto.lower())

    def test_action_schema_describes_effects_without_situation_policy(self):
        schema = default_registry(actions_path="scripts/sim_actions.py").prompt_schema()
        for prescriptive in (
            "다음 행동을 결정",
            "장애물 회피",
            "시야 변경이나 방향 정렬",
            "새 장면을 확인",
        ):
            self.assertNotIn(prescriptive, schema)
        self.assertIn("move_forward: 차체를 짧게 앞으로 이동합니다.", schema)
        self.assertIn("duration:float=0.3(0.10..0.80초)", schema)


class RealResumeGraspTests(unittest.TestCase):
    def test_one_shot_real_grasp_restore_is_conservative_and_consumed(self):
        import tempfile
        from harness.state import StateEstimator
        from harness.web import restore_one_shot_real_grasp

        with tempfile.TemporaryDirectory() as td:
            path = pathlib.Path(td) / "resume.json"
            path.write_text(json.dumps({
                "state": "PROBABLE_HELD",
                "held_object_color": "red",
                "confidence": 0.65,
                "visual_support": True,
                "held_streak": 1,
            }))
            est = StateEstimator()
            self.assertTrue(restore_one_shot_real_grasp(est, path))
            self.assertEqual(est.state.grasp.state, "PROBABLE_HELD")
            self.assertEqual(est.state.grasp.held_object_color, "red")
            self.assertFalse(path.exists())
            self.assertFalse(restore_one_shot_real_grasp(est, path))

    def test_one_shot_restore_rejects_empty_or_invalid_claims(self):
        import tempfile
        from harness.state import StateEstimator
        from harness.web import restore_one_shot_real_grasp

        with tempfile.TemporaryDirectory() as td:
            path = pathlib.Path(td) / "resume.json"
            path.write_text(json.dumps({"state": "EMPTY", "held_object_color": "red"}))
            est = StateEstimator()
            self.assertFalse(restore_one_shot_real_grasp(est, path))
            self.assertEqual(est.state.grasp.state, "UNKNOWN")
            self.assertFalse(path.exists())


class RegistryTests(unittest.TestCase):
    def test_decorator_and_unknown_tool(self):
        registry = Registry()

        @registry.tool(description="더한다")
        def add(left: int, right: int) -> int:
            return left + right

        self.assertEqual(registry.names(), ["add"])
        with self.assertRaises(ToolError):
            registry.get("delete")
        self.assertEqual(validate_args(registry.get("add"), {"left": 1, "right": 2}), {"left": 1, "right": 2})

    def test_rejects_extra_and_wrong_type(self):
        registry = Registry()

        @registry.tool(description="켜기")
        def look(flip_x: bool = False) -> str:
            return "ok"

        with self.assertRaises(ToolError):
            validate_args(registry.get("look"), {"unknown": True})
        with self.assertRaises(ToolError):
            validate_args(registry.get("look"), {"flip_x": "yes"})


class LoopTests(unittest.TestCase):
    def test_action_queue_plan_epoch_and_stale_pending_ttl(self):
        from harness.action_queue import RobotActionQueue

        queue = RobotActionQueue()
        first = queue.enqueue("approach", {"target_color": "red"}, source="old")
        old_epoch = queue.snapshot()["plan_epoch"]
        first.created_at = 1.0
        replaced = queue.replace_pending(
            [("search", {"target_color": "red"})], source="new_plan"
        )
        self.assertGreater(replaced[0].plan_epoch, old_epoch)
        self.assertEqual(queue.snapshot()["history"][-1]["reason"], "replanned")
        replaced[0].created_at = 1.0
        self.assertEqual(queue.invalidate_stale(10.0, now=20.0), 1)
        snapshot = queue.snapshot()
        self.assertTrue(snapshot["paused"])
        self.assertEqual(snapshot["history"][-1]["reason"], "stale_pending_ttl")

    def test_queued_recovery_stops_when_same_failure_has_no_progress(self):
        from harness.executive import TaskExecutive
        from harness.state import StateEstimator

        calls = []
        def runner(name, **kwargs):
            calls.append(name)
            common = {
                "skill": name, "target_color": "red",
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
            }
            if name == "approach":
                return {
                    **common, "ok": False, "execution_status": "FAILED",
                    "outcome_status": "NOT_ACHIEVED",
                    "failure_code": "APPROACH_PROGRESS_STALLED",
                    "required_state": "arm.pose=SEARCH_TRACK",
                    "recommended_recovery": "search", "reason": "stall",
                }
            if name == "search":
                return {**common, "ok": True, "target_vision": {"visible": True, "cx": .5, "cy": .5, "area_ratio": .01}}
            if name == "track":
                return {**common, "ok": True, "center_verified": True, "target_vision": {"visible": True, "cx": .5, "cy": .5, "area_ratio": .03}}
            return {**common, "ok": True}

        estimator = StateEstimator()
        estimator.set_goal_context(target_color="red")
        estimator.update_vision({"visible": True, "cx": .5, "cy": .5, "area_ratio": .01})
        result = run_loop(
            ReplayCompleter([{"plan": ["approach", "pick"]}, {"final": "진전 없어 중단"}]),
            default_registry(runner=runner), "빨간 블록 집어", execute=True,
            auto_observe=True, observe=None, max_steps=10,
            state_estimator=estimator,
            executive=TaskExecutive(strict_pick_preconditions=True),
            auto_recommended_recovery=True,
        )
        # The second identical approach failure is reported to the planner;
        # the same recovery chain is not launched a third time.
        self.assertEqual(calls, ["approach", "search", "track", "approach"])
        self.assertEqual(result.stopped, "final")

    def test_dry_run_does_not_call_handler(self):
        called = []
        registry = Registry()

        @registry.tool(description="다가갑니다")
        def approach() -> dict:
            called.append("approach")
            return {"moved": True}

        result = run_loop(
            ReplayCompleter([{"tool": "approach"}, {"final": "앞에 도착"}]),
            registry,
            "빨간 블록 앞으로 가",
            execute=False,
        )
        self.assertEqual(called, [])
        self.assertEqual(result.final, "앞에 도착")
        self.assertEqual(result.steps[0].result["dry_run"], True)
        self.assertEqual(result.steps[0].result["tool"], "approach")

    def test_team_rules_enter_system_prompt_only_when_peer_context_names_self(self):
        seen_prompts = []

        class Capture:
            def complete(self, messages, image=None):
                seen_prompts.append(messages[0]["content"])
                return json.dumps({"final": "ok"}, ensure_ascii=False)

        run_loop(Capture(), Registry(), "빨간 블럭 찾아", execute=False,
                 planner_context={"self_id": "r2", "team_goal": "각자 하나씩"})
        run_loop(Capture(), Registry(), "빨간 블럭 찾아", execute=False)
        run_loop(Capture(), Registry(), "빨간 블럭 찾아", execute=False,
                 planner_context_provider=lambda: {"self_id": "r3", "peers": {}})
        self.assertIn("TEAM: you are robot r2", seen_prompts[0])
        self.assertIn("r1, r3", seen_prompts[0])
        self.assertIn("PEER_TOO_CLOSE", seen_prompts[0])
        self.assertIn("send_peer_message", seen_prompts[0])
        self.assertNotIn("TEAM:", seen_prompts[1])
        self.assertIn("TEAM: you are robot r3", seen_prompts[2])

    def test_repeated_protocol_errors_end_the_turn_instead_of_looping(self):
        completer = ReplayCompleter(["garbage one", "garbage two", "garbage three", {"final": "never reached"}])
        result = run_loop(completer, Registry(), "빨간 블록 앞으로 가", execute=False)
        self.assertEqual(result.stopped, "protocol_error")
        self.assertEqual(completer.index, 3)
        self.assertEqual(sum(1 for step in result.steps if step.error), 3)
        self.assertIn("형식 오류", result.final)

    def test_one_protocol_error_is_still_recoverable(self):
        completer = ReplayCompleter(["garbage", {"final": "이제 됐어"}])
        result = run_loop(completer, Registry(), "빨간 블록 앞으로 가", execute=False)
        self.assertEqual(result.final, "이제 됐어")
        self.assertEqual(result.stopped, "final")

    def test_planner_input_callback_receives_exact_image_and_world_state(self):
        seen = []
        result = run_loop(
            ReplayCompleter([{"final": "봤어"}]),
            Registry(),
            "장면 봐",
            image="/tmp/exact-planner-input.jpg",
            execute=False,
            conversation_only=True,
            on_planner_input=lambda image, state: seen.append((image, state)),
        )
        self.assertEqual(result.final, "봤어")
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0][0], "/tmp/exact-planner-input.jpg")
        self.assertIn("target", seen[0][1])
        self.assertIn("grasp", seen[0][1])

    def test_live_planner_context_refreshes_before_each_model_decision(self):
        class ContextCompleter:
            def __init__(self):
                self.seen = []
                self.replies = [
                    'not-json-yet',
                    '{"final":"done"}',
                ]

            def complete(self, messages, image=None):
                del image
                contexts = [
                    json.loads(m["content"].split("multi_robot_peer_context: ", 1)[1])
                    for m in messages
                    if m.get("content", "").startswith("multi_robot_peer_context: ")
                ]
                self.assert_context_count = len(contexts)
                self.seen.append(contexts[-1])
                return self.replies.pop(0)

        registry = Registry()
        calls = []

        def provider():
            seq = len(calls) + 1
            calls.append(seq)
            return {"self_id": "r2", "peer_messages": [{"seq": seq, "message": f"m{seq}"}]}

        completer = ContextCompleter()
        result = run_loop(
            completer, registry, "협업 상태 확인", execute=True, max_steps=2,
            auto_observe=True, planner_context_provider=provider,
        )
        self.assertEqual(result.final, "done")
        self.assertEqual(calls, [1, 2])
        self.assertEqual([ctx["peer_messages"][0]["seq"] for ctx in completer.seen], [1, 2])
        self.assertEqual(completer.assert_context_count, 1)

    def test_planner_sees_action_contract_truth_after_successful_approach(self):
        from harness.executive import TaskExecutive
        from harness.state import StateEstimator

        captured = []

        class Recorder:
            def __init__(self):
                self.calls = 0
            def complete(self, messages, image=None):
                self.calls += 1
                captured.append(messages)
                if self.calls == 1:
                    return '{"tool":"approach","args":{"target_color":"red"}}'
                return '{"final":"검증을 위해 중단했어"}'

        estimator = StateEstimator()
        estimator.state.target.selected_color = "red"
        estimator.state.target.visible = True
        estimator.state.target.centered = True
        estimator.state.target.range_class = "NEAR"

        def runner(name, **kwargs):
            self.assertEqual(name, "approach")
            return {
                "ok": True,
                "skill": "approach",
                "target_color": "red",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "range_verified": True,
                "center_verified": True,
                "target_vision": {"visible": True, "cx": 0.49, "cy": 0.62, "area_ratio": 0.12},
            }

        registry = default_registry(runner=runner, actions_path="scripts/sim_actions.py")
        result = run_loop(
            Recorder(), registry, "빨간 블록을 집어줘",
            execute=True,
            state_estimator=estimator,
            executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(result.final, "검증을 위해 중단했어")
        second = captured[1]
        state_text = next(m["content"] for m in reversed(second) if str(m.get("content", "")).startswith("world_state: "))
        planner_state = json.loads(state_text.split(": ", 1)[1])
        self.assertTrue(planner_state["causal_handoffs"]["pregrasp_reach"]["available"])
        actions = planner_state["action_contract_status"]
        self.assertTrue(actions["approach"]["expected_postconditions_all_satisfied"])
        self.assertTrue(actions["pick"]["executable"])
        self.assertFalse(actions["pick"]["expected_postconditions_all_satisfied"])
        self.assertEqual(planner_state["goal_evidence"]["sensor_status"], "UNKNOWN")

    def test_execute_calls_allowlisted_function(self):
        registry = Registry()

        @registry.tool(description="더한다")
        def add(left: int, right: int) -> int:
            return left + right

        payload = dispatch(registry, ToolCall("add", {"left": 2, "right": 3}), execute=True)
        self.assertEqual(payload["result"]["value"], 5)

    def test_handler_exception_becomes_structured_failure_instead_of_escaping(self):
        registry = Registry()

        @registry.tool(description="실패")
        def broken() -> str:
            raise ImportError("missing fresh camera symbol")

        payload = dispatch(registry, ToolCall("broken", {}), execute=True)
        self.assertFalse(payload["ok"])
        self.assertEqual(payload["result"]["execution_status"], "FAILED")
        self.assertEqual(payload["result"]["failure_code"], "TOOL_EXCEPTION")
        self.assertIn("ImportError", payload["result"]["reason"])

    def test_unknown_tool_is_returned_as_error_and_loop_continues(self):
        registry = Registry()

        @registry.tool(description="정지")
        def stop() -> str:
            return "stopped"

        result = run_loop(
            ReplayCompleter([{"tool": "explode"}, {"final": "호출하지 않음"}]),
            registry,
            "폭파",
        )
        self.assertIn("unknown tool", result.steps[0].error)
        self.assertEqual(result.final, "호출하지 않음")


class CatalogTests(unittest.TestCase):
    def test_default_tools_are_red_skills(self):
        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        self.assertEqual(registry.names(), sorted(SKILLS))

    def test_execute_uses_injected_runner(self):
        seen = []
        registry = default_registry(runner=lambda name, **kwargs: seen.append(name) or {"ran": name})
        result = run_loop(
            ReplayCompleter([{"tool": "pick"}, {"final": "듦"}]),
            registry,
            "집어",
            execute=True,
        )
        self.assertEqual(seen, ["pick"])
        self.assertEqual(result.steps[0].result["result"]["ran"], "pick")

    def test_loads_swapped_actions_file(self):
        import tempfile
        from pathlib import Path

        path = Path(tempfile.mkdtemp()) / "other_actions.py"
        path.write_text(
            "ACTIONS = {'wave': '손을 흔듭니다'}\n"
            "def run(name, extra=None):\n"
            "    return {'skill': name}\n",
            encoding="utf-8",
        )
        registry = default_registry(actions_path=path)
        self.assertEqual(registry.names(), ["wave"])

    def test_plan_step_aliases(self):
        from harness.catalog import resolve_plan_step

        names = ["approach", "pick"]
        self.assertEqual(resolve_plan_step("다가감", names), "approach")
        self.assertEqual(resolve_plan_step("집기", names), "pick")
        self.assertIsNone(resolve_plan_step("확인", names))


class RobotActionContractTests(unittest.TestCase):
    def test_agent_actions_are_bounded_and_pick_includes_carry_pose(self):
        from harness.catalog import load_actions_module

        mod = load_actions_module()
        self.assertNotIn("carry", mod.ACTIONS)
        self.assertNotIn("fetch", mod.ACTIONS)
        calls = []
        original = mod.main
        try:
            mod.main = lambda argv: calls.append(list(argv)) or 0
            track = mod.run("track")
            pick = mod.run("pick")
        finally:
            mod.main = original
        self.assertEqual(track["argv"], ["track", "--seconds", "3.0", "--target-color", "red"])
        self.assertEqual(pick["argv"], ["pick", "--preserve-gaze", "--target-color", "red"])
        self.assertTrue(track["ok"] and pick["ok"])

    def test_nonzero_agent_action_is_reported_as_not_ok(self):
        from harness.catalog import load_actions_module

        mod = load_actions_module()
        original = mod.main
        try:
            mod.main = lambda argv: 1
            result = mod.run("pick")
        finally:
            mod.main = original
        self.assertFalse(result["ok"])
        self.assertEqual(result["exit_code"], 1)


class SequenceTests(unittest.TestCase):
    def test_approach_then_pick_then_final_dry_run(self):
        seen = []
        registry = default_registry(runner=lambda name, **kwargs: seen.append(name) or {"ran": name})
        result = run_loop(
            ReplayCompleter(
                [{"tool": "approach"}, {"tool": "pick"}, {"final": "집었음"}]
            ),
            registry,
            "빨간 블록을 집어",
            execute=False,
        )
        self.assertEqual(seen, [])
        tools = [
            step.action.name
            for step in result.steps
            if isinstance(step.action, ToolCall)
        ]
        self.assertEqual(tools, ["approach", "pick"])
        self.assertTrue(all(step.result["dry_run"] for step in result.steps if step.result))
        self.assertEqual(result.final, "집었음")
        self.assertEqual(result.stopped, "final")

    def test_approach_then_pick_execute_calls_runner(self):
        seen = []
        registry = default_registry(runner=lambda name, **kwargs: seen.append(name) or {"ran": name})
        result = run_loop(
            ReplayCompleter(
                [{"tool": "approach"}, {"tool": "pick"}, {"final": "집었음"}]
            ),
            registry,
            "빨간 블록을 집어",
            execute=True,
        )
        self.assertEqual(seen, ["approach", "pick"])
        self.assertEqual(result.final, "집었음")

    def test_plan_wait_look_then_observe(self):
        frames = []
        waited = []
        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        result = run_loop(
            ReplayCompleter(
                [
                    {"plan": ["다가감"]},
                    {"wait": 0.5},
                    {"look": True},
                    {"final": "도착"},
                ]
            ),
            registry,
            "앞으로 가",
            observe=lambda: frames.append("scene") or f"frame-{len(frames)}",
            sleeper=waited.append,
        )
        kinds = [type(step.action).__name__ for step in result.steps]
        self.assertEqual(kinds, ["Plan", "ToolCall", "Wait", "Look", "FinalAnswer"])
        self.assertEqual(waited, [0.5])
        self.assertEqual(frames, ["scene", "scene"])
        self.assertNotIn("observed", result.steps[1].result)
        self.assertEqual(result.steps[2].result["observed"], "frame-1")
        self.assertEqual(result.steps[3].result["observed"], "frame-2")
        self.assertEqual(result.final, "도착")
        self.assertEqual(result.action_queue["pending"], [])
        self.assertEqual(result.action_queue["history"][-1]["tool"], "approach")
        self.assertEqual(result.action_queue["history"][-1]["status"], "COMPLETED")

    def test_auto_observe_after_tool_allows_final(self):
        frames = []
        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        result = run_loop(
            ReplayCompleter([{"tool": "approach"}, {"final": "도착"}]),
            registry,
            "앞으로 가",
            image="/tmp/before.jpg",
            observe=lambda: frames.append("scene") or "/tmp/after.jpg",
            auto_observe=True,
        )
        self.assertEqual(result.stopped, "final")
        self.assertEqual(result.final, "도착")
        self.assertEqual(frames, ["scene"])
        self.assertEqual(result.steps[0].result["observed"], "/tmp/after.jpg")

    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "0"})
    def test_authoritative_completion_uses_facts_without_second_image_or_verifier(self):
        class RecordingCompleter:
            def __init__(self):
                self.images = []
                self.replies = [
                    '{"tool":"pick"}',
                    '{"final":"안정적으로 집었어"}',
                ]
            def complete(self, messages, image=None):
                self.images.append(image)
                return self.replies.pop(0)

        checks = []
        completer = RecordingCompleter()
        registry = default_registry(runner=lambda name, **kwargs: {
            "skill": name,
            "task_complete": True,
            "authoritative_completion": True,
            "progress": {"stable": True, "bilateral_contact": True, "block_height_m": .5},
        })
        result = run_loop(
            completer, registry, "빨간 블록을 집어",
            image="/tmp/before.jpg", execute=True,
            observe=lambda: "/tmp/after.jpg", auto_observe=True,
            verify_final=lambda *args: checks.append(args) or (True, "visual ok"),
        )
        self.assertEqual(result.final, "안정적으로 집었어")
        self.assertEqual(completer.images, ["/tmp/before.jpg", None])
        self.assertEqual(checks, [])
        self.assertTrue(result.steps[0].result["result"]["authoritative_completion"])

    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "0"})
    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "0"})
    def test_failed_queued_skill_pauses_later_work_and_allows_replan(self):
        class QueueAwareCompleter:
            def __init__(self):
                self.contexts = []
                self.replies = [
                    '{"plan":["approach","pick"]}',
                    '{"plan":["track"]}',
                    '{"final":"실패한 접근 뒤 track만 실행하고 중단했어"}',
                ]
            def complete(self, messages, image=None):
                del image
                states = [m["content"] for m in messages if str(m.get("content", "")).startswith("world_state: ")]
                if states:
                    self.contexts.append(json.loads(states[-1].split("world_state: ", 1)[1]))
                return self.replies.pop(0)

        calls = []
        def runner(name, **kwargs):
            calls.append(name)
            if name == "approach":
                return {
                    "ok": False, "skill": name,
                    "command_status": "ACCEPTED", "execution_status": "FAILED",
                    "outcome_status": "NOT_ACHIEVED", "failure_code": "APPROACH_FAILED",
                    "reason": "synthetic approach failure",
                }
            return {
                "ok": True, "skill": name,
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
            }

        completer = QueueAwareCompleter()
        result = run_loop(
            completer,
            default_registry(runner=runner),
            "가",
            execute=True,
        )
        self.assertEqual(calls, ["approach", "track"])
        blocked = completer.contexts[1]["action_queue"]
        self.assertTrue(blocked["paused"])
        self.assertEqual([item["tool"] for item in blocked["pending"]], ["pick"])
        self.assertEqual(blocked["history"][-1]["tool"], "approach")
        self.assertEqual(blocked["history"][-1]["status"], "FAILED")
        self.assertEqual(blocked["history"][-1]["failure_code"], "APPROACH_FAILED")
        # Replanning replaced the still-unstarted pick; it was never dispatched.
        self.assertNotIn("pick", calls)
        self.assertEqual(result.final, "실패한 접근 뒤 track만 실행하고 중단했어")
        self.assertEqual(result.action_queue["pending"], [])
        self.assertEqual(result.action_queue["history"][-1]["tool"], "track")

    def test_action_synchronous_sensor_evidence_still_gets_fresh_post_action_image(self):
        class RecordingCompleter:
            def __init__(self):
                self.images = []
                self.states = []
                self.replies = ['{"tool":"search"}', '{"final":"검증을 위해 중단했어"}']
            def complete(self, messages, image=None):
                self.images.append(image)
                for message in reversed(messages):
                    content = str(message.get("content", ""))
                    if content.startswith("world_state: "):
                        self.states.append(json.loads(content.split(": ", 1)[1]))
                        break
                return self.replies.pop(0)

        frames = []
        completer = RecordingCompleter()
        registry = default_registry(
            runner=lambda name, **kwargs: {
                "ok": True,
                "skill": name,
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "target_color": "red",
                "target_vision": {"visible": True, "cx": 0.31, "cy": 0.24, "area_ratio": 0.0032},
                "camera_red": {"visible": True, "cx": 0.31, "cy": 0.24, "area_ratio": 0.0032},
            },
            actions_path="scripts/sim_actions.py",
        )
        result = run_loop(
            completer, registry, "빨간 블록 찾아줘",
            image="/tmp/before.jpg", execute=True,
            observe=lambda: frames.append("fresh") or "/tmp/after.jpg",
            auto_observe=True,
        )
        # The fresh post-action frame is still captured even though the skill
        # returned synchronous sensor evidence.
        self.assertEqual(frames, ["fresh"])
        # That refreshed world_state satisfies the SEARCH goal, so the executor
        # finishes deterministically instead of spending another planner call
        # (and never resends raw pixels).
        self.assertEqual(result.stopped, "goal_achieved")
        self.assertEqual(result.final, "빨간 블록을 찾았어.")
        self.assertEqual(completer.images, ["/tmp/before.jpg"])
        self.assertEqual(completer.replies, ['{"final":"검증을 위해 중단했어"}'])
        self.assertTrue(result.world_state["target"]["visible"])
        search_step = next(
            step for step in result.steps
            if isinstance(step.action, ToolCall) and step.action.name == "search"
        )
        self.assertEqual(search_step.result["observed"], "/tmp/after.jpg")
        self.assertTrue(search_step.result["world_state"]["target"]["visible"])

    def test_auto_observe_updates_state_without_resending_post_action_pixels(self):
        class RecordingCompleter:
            def __init__(self):
                self.images = []
                self.replies = [
                    '{"plan":["approach"]}',
                    '{"final":"도착"}',
                ]
            def complete(self, messages, image=None):
                self.images.append(image)
                return self.replies.pop(0)

        completer = RecordingCompleter()
        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        result = run_loop(
            completer,
            registry,
            "앞으로 가",
            image="/tmp/before.jpg",
            observe=lambda: "/tmp/after.jpg",
            auto_observe=True,
        )
        self.assertEqual(result.stopped, "final")
        # Initial pixels are sent once. The fresh post-action frame is still
        # captured for deterministic state, but the next LLM call is text-only.
        self.assertEqual(completer.images, ["/tmp/before.jpg", None])

    def test_explicit_look_attaches_one_fresh_image_to_next_planner_call(self):
        class RecordingCompleter:
            def __init__(self):
                self.images = []
                self.replies = [
                    '{"look":true}',
                    '{"final":"봤어"}',
                ]
            def complete(self, messages, image=None):
                self.images.append(image)
                return self.replies.pop(0)

        completer = RecordingCompleter()
        result = run_loop(
            completer,
            default_registry(runner=lambda name, **kwargs: {"ran": name}),
            "장면 확인해",
            image="/tmp/before.jpg",
            observe=lambda: "/tmp/after.jpg",
            auto_observe=True,
        )
        self.assertEqual(result.final, "봤어")
        self.assertEqual(completer.images, ["/tmp/before.jpg", "/tmp/after.jpg"])

    def test_auto_observe_compacts_long_context(self):
        class LongCompleter:
            def __init__(self):
                self.sizes = []
                self.i = 0
            def complete(self, messages, image=None):
                self.sizes.append(len(messages))
                self.i += 1
                if self.i <= 8:
                    return '{"tool":"approach"}'
                return '{"final":"끝"}'

        c = LongCompleter()
        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        result = run_loop(
            c, registry, "계속 접근해", image="/tmp/start.jpg",
            max_steps=8, observe=lambda: "/tmp/frame.jpg", auto_observe=True,
        )
        self.assertEqual(result.stopped, "final")
        self.assertLessEqual(max(c.sizes), 8)

    def test_recent_action_evidence_reports_progress_without_blocking_repeat(self):
        class EvidenceCompleter:
            def __init__(self):
                self.contexts = []
                self.replies = [
                    '{"tool":"search","args":{"target_color":"red"}}',
                    '{"tool":"search","args":{"target_color":"red"}}',
                    '{"final":"근거를 보고 종료"}',
                ]

            def complete(self, messages, image=None):
                del image
                state_messages = [m["content"] for m in messages if m.get("content", "").startswith("world_state: ")]
                if state_messages:
                    self.contexts.append(json.loads(state_messages[-1].split("world_state: ", 1)[1]))
                return self.replies.pop(0)

        calls = []
        completer = EvidenceCompleter()
        registry = default_registry(runner=lambda name, **kwargs: calls.append(name) or {
            "skill": name,
            "command_status": "ACCEPTED",
            "execution_status": "COMPLETED",
            "outcome_status": "ACHIEVED",
            "target_color": kwargs.get("target_color", "red"),
            "target_vision": {"visible": True, "cx": 0.52, "cy": 0.31, "area_ratio": 0.012},
        })
        # A sensor-satisfied SEARCH goal now ends the turn deterministically after
        # the first successful search, so use a request without a terminal goal
        # predicate to observe how the harness reports a repeated action.
        result = run_loop(
            completer,
            registry,
            "빨간 블록 어디 있는지 알려줘",
            execute=True,
            max_steps=3,
        )
        self.assertEqual(calls, ["search", "search"])
        self.assertEqual(result.final, "근거를 보고 종료")
        self.assertEqual(completer.contexts[0]["recent_action_evidence"], [])
        first = completer.contexts[1]["recent_action_evidence"][-1]
        self.assertEqual(first["action"], "search")
        self.assertTrue(first["executed"])
        self.assertEqual(first["same_action_streak"], 1)
        self.assertEqual(first["outcome_status"], "ACHIEVED")
        self.assertTrue(any(change["field"] == "target.visible" for change in first["changed_fields"]))
        second = completer.contexts[2]["recent_action_evidence"][-1]
        self.assertEqual(second["same_action_streak"], 2)
        self.assertEqual(second["changed_fields"], [])
        self.assertEqual(completer.contexts[2]["goal_evidence"]["kind"], "GENERIC")
        self.assertTrue(completer.contexts[2]["target"]["visible"])

    def test_plan_queue_executes_in_declared_order_without_tool_reemission(self):
        seen = []
        registry = default_registry(runner=lambda name, **kwargs: seen.append(name) or {"ran": name})
        result = run_loop(
            ReplayCompleter(
                [
                    {"plan": ["다가감", "집기"]},
                    {"final": "집었음"},
                ]
            ),
            registry,
            "집어",
            execute=True,
            observe=lambda: "/tmp/frame.jpg",
            auto_observe=True,
        )
        self.assertEqual(seen, ["approach", "pick"])
        self.assertEqual(result.final, "집었음")
        completed = [item for item in result.action_queue["history"] if item["status"] == "COMPLETED"]
        self.assertEqual([item["tool"] for item in completed[-2:]], ["approach", "pick"])
        self.assertEqual(result.action_queue["pending"], [])

    def test_strict_grasp_plan_queue_rechecks_each_head_before_execution(self):
        from harness.executive import TaskExecutive
        from harness.state import StateEstimator

        calls = []
        def runner(name, **kwargs):
            calls.append(name)
            base = {
                "ok": True,
                "skill": name,
                "target_color": kwargs.get("target_color", "red"),
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
            }
            if name == "search":
                base["target_vision"] = {"visible": True, "cx": .50, "cy": .25, "area_ratio": .01}
            elif name == "track":
                base["center_verified"] = True
                base["target_vision"] = {"visible": True, "cx": .50, "cy": .30, "area_ratio": .02}
            elif name == "approach":
                base["range_verified"] = True
                base["center_verified"] = True
                base["target_vision"] = {"visible": True, "cx": .50, "cy": .62, "area_ratio": .12}
            elif name == "pick":
                base["grasp_verified"] = True
            return base

        result = run_loop(
            ReplayCompleter([
                {"plan": ["search", "track", "approach", "pick"]},
                {"final": "집었어"},
            ]),
            default_registry(runner=runner),
            "빨간 블록 잡아",
            execute=True,
            state_estimator=StateEstimator(),
            executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(calls, ["search", "track", "approach", "pick"])
        # Once the drained queue leaves the GRASP goal sensor-satisfied, the
        # executor finishes without another planner call.
        self.assertEqual(result.stopped, "goal_achieved")
        self.assertEqual(result.final, "빨간 블록 집기를 완료했어.")
        self.assertEqual(result.world_state["grasp"]["state"], "HELD")
        history = [item for item in result.action_queue["history"] if item["status"] == "COMPLETED"]
        self.assertEqual([item["tool"] for item in history[-4:]], ["search", "track", "approach", "pick"])

    def test_blocked_queue_head_keeps_later_skills_waiting_and_is_visible_to_llm(self):
        from harness.action_queue import RobotActionQueue
        from harness.executive import TaskExecutive
        from harness.state import StateEstimator

        class CapturingCompleter:
            def __init__(self):
                self.contexts = []
                self.replies = [
                    '{"plan":["pick","approach"]}',
                    '{"final":"집기 계획이 준비되지 않아 중단했어"}',
                ]
            def complete(self, messages, image=None):
                del image
                states = [m["content"] for m in messages if str(m.get("content", "")).startswith("world_state: ")]
                if states:
                    self.contexts.append(json.loads(states[-1].split("world_state: ", 1)[1]))
                return self.replies.pop(0)

        estimator = StateEstimator()
        estimator.state.target.selected_color = "red"
        estimator.state.target.visible = True
        estimator.state.target.centered = True
        estimator.state.target.range_class = "PREGRASP"
        estimator.state.task.phase = "TARGET_ACQUISITION"
        dispatched = []
        queue = RobotActionQueue()
        completer = CapturingCompleter()
        result = run_loop(
            completer,
            default_registry(runner=lambda name, **kwargs: dispatched.append(name) or {"ran": name}),
            "빨간 블록 잡아",
            execute=True,
            state_estimator=estimator,
            executive=TaskExecutive(strict_pick_preconditions=True),
            action_queue=queue,
        )
        self.assertEqual(dispatched, [])
        self.assertEqual(len(completer.contexts), 2)
        visible = completer.contexts[1]["action_queue"]
        self.assertTrue(visible["paused"])
        self.assertEqual([item["tool"] for item in visible["pending"]], ["approach"])
        self.assertFalse(visible["pending"][0]["executable_now"] is None)
        blocked = visible["history"][-1]
        self.assertEqual(blocked["tool"], "pick")
        self.assertEqual(blocked["status"], "BLOCKED")
        self.assertEqual(blocked["failure_code"], "PREGRASP_PLAN_MISSING")
        self.assertEqual(result.action_queue["pending"][0]["tool"], "approach")

    def test_grasp_probable_held_finishes_without_extra_planner_call(self):
        from harness.executive import TaskExecutive
        from harness.state import StateEstimator

        class Capture:
            def __init__(self):
                self.calls = 0
            def complete(self, messages, image=None):
                self.calls += 1
                return '{"tool":"pick","args":{"target_color":"red"}}'

        estimator = StateEstimator()
        estimator.set_goal_context(target_color="red", destination_color=None)
        estimator.state.target.visible = True
        estimator.state.target.centered = True
        estimator.state.target.range_class = "PREGRASP"
        estimator.state.task.phase = "PREGRASP_READY"
        c = Capture()
        result = run_loop(
            c,
            default_registry(runner=lambda name, **kwargs: {
                "ok": True, "skill": "pick", "target_color": "red",
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN",
                "visual_hold": {"probable": True, "confidence": 0.65},
            }),
            "빨간 블록 집어", execute=True, auto_observe=True, observe=None,
            state_estimator=estimator, executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(c.calls, 1)
        self.assertEqual(result.stopped, "goal_achieved")
        self.assertEqual(result.world_state["grasp"]["state"], "PROBABLE_HELD")
        self.assertIn("완료", result.final)

    def test_executed_skill_recovery_advisory_is_visible_to_next_planner_call(self):
        from harness.state import StateEstimator
        from harness.executive import TaskExecutive

        class Capture:
            def __init__(self):
                self.calls = []
                self.replies = [
                    '{"tool":"track","args":{"target_color":"red"}}',
                    '{"final":"작업 실패"}',
                ]
            def complete(self, messages, image=None):
                self.calls.append(list(messages))
                return self.replies.pop(0)

        def runner(name, **kwargs):
            return {
                "ok": False, "skill": name, "target_color": "red",
                "command_status": "ACCEPTED", "execution_status": "FAILED",
                "outcome_status": "NOT_ACHIEVED", "failure_code": "TARGET_NOT_VISIBLE",
                "required_state": "target.visible=true", "recommended_recovery": "search",
                "reason": "track lost target for 4 consecutive fresh frames",
            }

        estimator = StateEstimator()
        estimator.set_goal_context(target_color="red", destination_color=None)
        estimator.update_vision({"visible": True, "cx": 0.5, "cy": 0.4, "area_ratio": 0.01})
        c = Capture()
        run_loop(
            c, default_registry(runner=runner), "빨간 블록 집어",
            execute=True, auto_observe=True, observe=None,
            state_estimator=estimator, executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(len(c.calls), 2)
        second = "\n".join(str(m.get("content", "")) for m in c.calls[1])
        self.assertIn('"recommended_recovery": "search"', second)
        self.assertIn("advisory", c.calls[1][0]["content"])

    def test_driver_confirmed_track_loss_beats_one_passive_visible_frame_until_search(self):
        from harness.state import StateEstimator

        estimator = StateEstimator()
        estimator.set_goal_context(target_color="red", destination_color=None)
        estimator.update_tool_result({
            "ok": False, "tool": "track", "result": {
                "skill": "track", "target_color": "red",
                "command_status": "ACCEPTED", "execution_status": "FAILED",
                "outcome_status": "NOT_ACHIEVED", "failure_code": "TARGET_NOT_VISIBLE",
                "required_state": "target.visible=true", "recommended_recovery": "search",
            }
        })
        self.assertFalse(estimator.state.target.visible)
        estimator.update_scene_memory({
            "red": {"visible": True, "cx": 0.1, "cy": 0.62, "area_ratio": 0.02},
            "blue": {"visible": False}, "yellow": {"visible": False},
        })
        self.assertFalse(estimator.state.target.visible)
        self.assertEqual(estimator.state.target.range_class, "UNKNOWN")
        estimator.update_tool_result({
            "ok": True, "tool": "search", "result": {
                "skill": "search", "target_color": "red",
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED", "target_vision": {"visible": True},
            }
        })
        self.assertTrue(estimator.state.target.visible)

    def test_passive_rgb_does_not_erase_successful_approach_precision_range(self):
        from harness.executive import TaskExecutive
        from harness.state import StateEstimator

        estimator = StateEstimator()
        estimator.set_goal_context(target_color="red", destination_color=None)
        estimator.update_tool_result({
            "ok": True,
            "tool": "approach",
            "result": {
                "skill": "approach",
                "target_color": "red",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "center_verified": True,
                "range_verified": True,
                "target_vision": {"visible": True, "cx": 0.5, "cy": 0.62, "area_ratio": 0.1},
            },
        })
        self.assertEqual(estimator.state.task.phase, "PREGRASP_READY")
        self.assertEqual(estimator.state.target.range_class, "PREGRASP")
        # A close passive frame can report a coarser NEAR cy; it must not erase
        # the stronger approach/FK-IK handoff before immediate pick.
        estimator.update_scene_memory({
            "red": {"visible": True, "cx": 0.51, "cy": 0.47, "area_ratio": 0.04},
            "blue": {"visible": False}, "yellow": {"visible": False},
        })
        self.assertEqual(estimator.state.task.phase, "PREGRASP_READY")
        self.assertEqual(estimator.state.target.range_class, "PREGRASP")
        decision = TaskExecutive(strict_pick_preconditions=True).check(
            "pick", estimator.state, {"target_color": "red"}
        )
        self.assertTrue(decision.allowed)

    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "0"})
    def test_direct_recovery_replaces_stale_failed_queue_tail(self):
        calls = []
        class RecoveryCompleter:
            def __init__(self):
                self.replies = [
                    '{"plan":["approach","pick"]}',
                    '{"tool":"track","args":{"target_color":"red"}}',
                    '{"final":"작업 실패"}',
                ]
            def complete(self, messages, image=None):
                return self.replies.pop(0)

        def runner(name, **kwargs):
            calls.append(name)
            if name == "approach":
                return {
                    "ok": False, "skill": name, "command_status": "ACCEPTED",
                    "execution_status": "FAILED", "outcome_status": "NOT_ACHIEVED",
                    "failure_code": "APPROACH_PROGRESS_STALLED", "reason": "synthetic",
                }
            return {
                "ok": True, "skill": name, "command_status": "ACCEPTED",
                "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
                "target_color": "red", "center_verified": True,
                "target_vision": {"visible": True, "cx": 0.5, "cy": 0.4, "area_ratio": 0.02},
            }

        result = run_loop(
            RecoveryCompleter(), default_registry(runner=runner), "빨간 블록 집어",
            execute=True,
        )
        self.assertEqual(calls, ["approach", "track"])
        self.assertNotIn("pick", calls)
        cancelled = [x for x in result.action_queue["history"] if x["tool"] == "pick"]
        self.assertTrue(cancelled)
        self.assertEqual(cancelled[-1]["status"], "CANCELLED")
        self.assertEqual(result.action_queue["pending"], [])

    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "0"})
    def test_team_planner_error_falls_back_to_executive_queue_and_finishes_search(self):
        from harness.vlm import VlmError

        calls = []
        class Down:
            def complete(self, messages, image=None):
                raise VlmError("Groq 한도에 걸렸습니다. retry in 17.500s")

        def runner(name, **kwargs):
            calls.append((name, kwargs))
            self.assertEqual(name, "search")
            return {
                "ok": True, "skill": name, "target_color": kwargs.get("target_color"),
                "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED", "target_vision": {"visible": True},
            }

        result = run_loop(
            Down(), default_registry(runner=runner), "노란 블록 찾아봐",
            execute=True, auto_observe=True, observe=None,
            executive_fallback_on_planner_error=True,
        )
        self.assertEqual([name for name, _args in calls], ["search"])
        self.assertEqual(result.stopped, "goal_achieved")
        self.assertIn("노란", result.final)
        self.assertEqual(result.action_queue["history"][-1]["source"], "team_planner_fallback")

    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "0"})
    def test_team_queue_uses_recommended_recovery_without_second_planner_call(self):
        from harness.executive import TaskExecutive
        from harness.state import StateEstimator

        calls = []
        class OnePlan:
            def __init__(self): self.calls = 0
            def complete(self, messages, image=None):
                self.calls += 1
                if self.calls > 1:
                    raise AssertionError("recommended recovery should avoid a second planner call")
                return '{"plan":["approach","pick"]}'

        approach_attempts = 0
        def runner(name, **kwargs):
            nonlocal approach_attempts
            calls.append(name)
            color = kwargs.get("target_color", "red")
            if name == "approach":
                approach_attempts += 1
                if approach_attempts == 1:
                    return {
                        "ok": False, "skill": name, "target_color": color,
                        "command_status": "ACCEPTED", "execution_status": "FAILED",
                        "outcome_status": "NOT_ACHIEVED",
                        "failure_code": "APPROACH_PROGRESS_STALLED",
                        "required_state": "arm.pose=SEARCH_TRACK",
                        "recommended_recovery": "search", "reason": "synthetic stall",
                    }
                return {
                    "ok": True, "skill": name, "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "center_verified": True,
                    "range_verified": True, "target_vision": {"visible": True},
                }
            if name == "search":
                return {
                    "ok": True, "skill": name, "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "target_vision": {"visible": True},
                }
            if name == "track":
                return {
                    "ok": True, "skill": name, "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "center_verified": True,
                    "target_vision": {"visible": True},
                }
            if name == "pick":
                return {
                    "ok": True, "skill": name, "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "UNKNOWN",
                    "visual_hold": {"probable": True, "confidence": 0.65},
                }
            raise AssertionError(name)

        estimator = StateEstimator()
        estimator.set_goal_context(target_color="red", destination_color=None)
        estimator.update_vision({"visible": True, "cx": 0.5, "cy": 0.4, "area_ratio": 0.01})
        completer = OnePlan()
        result = run_loop(
            completer, default_registry(runner=runner), "빨간 블록 집어",
            execute=True, auto_observe=True, observe=None,
            state_estimator=estimator, executive=TaskExecutive(strict_pick_preconditions=True),
            auto_recommended_recovery=True,
        )
        self.assertEqual(completer.calls, 1)
        self.assertEqual(calls, ["approach", "search", "track", "approach", "pick"])
        self.assertEqual(result.stopped, "goal_achieved")
        self.assertFalse(result.action_queue["paused"])

    def test_persistent_queue_is_paused_for_review_on_next_turn_and_shown_to_llm(self):
        from harness.action_queue import RobotActionQueue

        queue = RobotActionQueue()
        queue.enqueue("approach", {"target_color": "red"}, source="previous_turn")
        captured = []
        class Completer:
            def complete(self, messages, image=None):
                del image
                state = next(
                    json.loads(m["content"].split("world_state: ", 1)[1])
                    for m in reversed(messages)
                    if str(m.get("content", "")).startswith("world_state: ")
                )
                captured.append(state["action_queue"])
                return '{"final":"기존 queue를 확인했어"}'

        calls = []
        result = run_loop(
            Completer(),
            default_registry(runner=lambda name, **kwargs: calls.append(name) or {"ran": name}),
            "현재 상태 알려줘",
            action_queue=queue,
        )
        self.assertEqual(calls, [])
        self.assertTrue(captured[0]["paused"])
        self.assertEqual(captured[0]["pause_reason"], "new_turn_review")
        self.assertEqual(captured[0]["pending"][0]["tool"], "approach")
        self.assertEqual(result.action_queue["pending"][0]["tool"], "approach")

    def test_max_steps_counts_tools_not_wait_look(self):
        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        completer = ReplayCompleter(
            [
                {"plan": ["approach"]},
                {"wait": 0.2},
                {"look": True},
                {"tool": "approach"},
                {"look": True},
                {"final": "도착"},
            ]
        )
        result = run_loop(
            completer,
            registry,
            "가",
            max_steps=1,
            observe=lambda: "frame",
            sleeper=lambda _seconds: None,
        )
        # wait/look after the single allowed tool are accepted without error;
        # only the second tool proposal hits the budget, which now ends the turn
        # deterministically instead of asking the planner for another final.
        accepted = [step for step in result.steps if isinstance(step.action, (Wait, Look))]
        self.assertEqual(len(accepted), 2)
        self.assertTrue(all(step.error is None for step in accepted))
        self.assertEqual(result.stopped, "tool_budget")
        tool_steps = [step for step in result.steps if isinstance(step.action, ToolCall)]
        self.assertEqual([step.error is None for step in tool_steps], [True, False])
        self.assertIn("tool limit reached", tool_steps[1].error)
        self.assertEqual(completer.index, 4)
        self.assertTrue(result.final)

    @patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "0"})
    def test_independent_final_verifier_can_reject_false_success(self):
        seen = []
        checks = []
        registry = default_registry(runner=lambda name, **kwargs: seen.append(name) or {"ran": name})
        decisions = iter([(False, "블록이 아직 바닥에 있음"), (True, "블록이 들려 있음")])

        def verifier(goal, final_text, image):
            checks.append((goal, final_text, image))
            return next(decisions)

        result = run_loop(
            ReplayCompleter(
                [
                    {"tool": "pick"},
                    {"final": "집었어"},
                    {"tool": "track"},
                    {"tool": "pick"},
                    {"final": "집었어"},
                ]
            ),
            registry,
            "빨간 블록을 집어",
            image="/tmp/before.jpg",
            execute=True,
            observe=lambda: "/tmp/current.jpg",
            auto_observe=True,
            verify_final=verifier,
        )
        self.assertEqual(result.stopped, "final")
        self.assertEqual(result.final, "집었어")
        self.assertEqual(seen, ["pick", "track", "pick"])
        self.assertIn("final verification failed", result.steps[1].error)
        self.assertEqual(len(checks), 2)

    def test_cancel_stops_before_next_step(self):
        stop = {"yes": False}
        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        result = run_loop(
            ReplayCompleter(
                [{"tool": "approach"}, {"tool": "pick"}, {"final": "끝"}]
            ),
            registry,
            "집어",
            should_cancel=lambda: stop["yes"],
            on_step=lambda _step: stop.__setitem__("yes", True),
        )
        self.assertEqual(result.stopped, "cancel")
        tools = [
            step.action.name
            for step in result.steps
            if isinstance(step.action, ToolCall) and not step.error
        ]
        self.assertEqual(tools, ["approach"])

    def test_talk_only_returns_once_without_running_or_claiming_tools(self):
        seen = []
        completer = ReplayCompleter([{"tool": "approach"}])
        registry = default_registry(runner=lambda name, **kwargs: seen.append(name) or {"ran": name})
        result = run_loop(
            completer,
            registry,
            "빨간 블록 집어",
            talk_only=True,
            execute=True,
        )
        self.assertEqual(seen, [])
        self.assertEqual(completer.index, 0)
        self.assertEqual(len(result.steps), 1)
        self.assertIsNone(result.steps[0].error)
        self.assertIn("실행하지 않았어", result.final)
        self.assertEqual(result.stopped, "robot_offline")


class ChatTests(unittest.TestCase):
    def test_handle_turn_formats_skill_sequence(self):
        from harness.chat import format_skill_log, handle_turn

        seen = []
        registry = default_registry(runner=lambda name, **kwargs: seen.append(name) or {"ran": name})
        log, result = handle_turn(
            "빨간 블록을 집어",
            completer=ReplayCompleter(
                [{"tool": "approach"}, {"tool": "pick"}, {"final": "집었음"}]
            ),
            registry=registry,
            execute=False,
        )
        self.assertEqual(seen, [])
        self.assertIn("[dry-run] tool: approach", log)
        self.assertIn("[dry-run] tool: pick", log)
        self.assertIn("final: 집었음", log)
        self.assertEqual(format_skill_log(result), log)
        from harness.chat import result_payload

        events = result_payload(result)["events"]
        self.assertEqual([event["type"] for event in events], ["tool", "tool", "final"])

    def test_spoken_text_uses_say_then_final(self):
        from harness.chat import spoken_text

        registry = default_registry(runner=lambda name, **kwargs: {"ran": name})
        result = run_loop(
            ReplayCompleter(
                [
                    {"say": "앞에 보여. 다가갈게.", "tool": "approach"},
                    {"final": "앞에 도착했어."},
                ]
            ),
            registry,
            "빨간 거 집어",
        )
        self.assertEqual(spoken_text(result), "앞에 보여. 다가갈게.\n앞에 도착했어.")

    def test_handle_turn_rejects_empty(self):
        from harness.chat import handle_turn

        log, result = handle_turn(
            "  ",
            completer=ReplayCompleter([]),
        )
        self.assertIn("empty", log)
        self.assertEqual(result.stopped, "empty")

    def test_handle_turn_forwards_image_into_loop(self):
        from harness.chat import handle_turn

        seen = []

        class Recorder:
            def complete(self, messages, image=None):
                seen.append((messages[1]["content"], image))
                return '{"final": "ok"}'

        log, result = handle_turn(
            "이게 뭐야",
            image="/tmp/scene.jpg",
            completer=Recorder(),
        )
        self.assertEqual(result.final, "ok")
        # The image travels only through the provider's multimodal field; the
        # local filesystem path is not duplicated into the text prompt.
        self.assertEqual(seen[0][0], "이게 뭐야")
        self.assertEqual(seen[0][1], "/tmp/scene.jpg")
        self.assertIn("final: ok", log)


class WebTests(unittest.TestCase):
    def setUp(self):
        import base64
        import json
        import threading
        import urllib.error
        import urllib.request

        from harness.loop import ReplayCompleter
        from harness.web import ChatState, make_server

        self.json = json
        self.urllib_error = urllib.error
        self.urllib_request = urllib.request
        self.state = ChatState(
            completer=ReplayCompleter(
                [
                    {
                        "say": "앞에 빨간 블록 보여. 다가갈게.",
                        "tool": "approach",
                    },
                    {"final": "앞에 도착했어. 집을까?"},
                ]
                * 8
            ),
            password="secret",
        )
        self.httpd = make_server(self.state, host="127.0.0.1", port=0)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        token = base64.b64encode(b"ugrp:secret").decode("ascii")
        self.auth = {"Authorization": f"Basic {token}"}
        self.state.grab_camera = lambda: b"\xff\xd8\xff\xd9"

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()

    def _open(self, path, data=None, headers=None):
        req = self.urllib_request.Request(
            f"http://127.0.0.1:{self.port}{path}",
            data=data,
            headers=headers or {},
            method="POST" if data is not None else "GET",
        )
        return self.urllib_request.urlopen(req, timeout=3)

    def test_rejects_missing_password(self):
        with self.assertRaises(self.urllib_error.HTTPError) as ctx:
            self._open("/")
        self.assertEqual(ctx.exception.code, 401)

    def test_serves_chat_html(self):
        with self._open("/", headers=self.auth) as res:
            body = res.read().decode("utf-8")
        self.assertIn("/api/turn", body)
        self.assertIn("/api/team/chat", body)
        self.assertIn('id="team-chat-form"', body)
        self.assertIn("@R1", body)
        self.assertIn("TEAM chat", body)
        self.assertIn('id="team-cam-overview"', body)
        self.assertIn("/api/camera", body)
        self.assertIn("/api/camera", body)
        self.assertIn("/api/observer/cctv_front_left/snapshot", body)
        self.assertIn("/sim-scene.xml", (pathlib.Path(__file__).resolve().parents[1] / "harness" / "static" / "sim3d.js").read_text())
        self.assertIn("MuJoCo 실제 RGB", body)
        self.assertIn("/api/tools", body)
        self.assertNotIn("/api/robot", body)
        self.assertIn("MasterPi", body)
        self.assertIn("<h1>UGRP</h1>", body)
        self.assertIn("background: var(--send)", body)
        self.assertIn("isComposing", body)
        self.assertIn("stream: true", body)
        self.assertIn("계획", body)
        self.assertIn("중지", body)
        self.assertIn("/api/status", body)
        self.assertIn("/api/real-traces", body)
        self.assertIn("최근 REAL 실행 분석", body)
        self.assertIn("/api/cancel", body)
        self.assertIn("/api/sim/reset", body)
        self.assertIn("/api/sim/wake", body)
        self.assertIn("id=\"sim-wake\"", body)
        self.assertIn("SIM 켜기", body)
        self.assertIn("id=\"sim-seed\"", body)
        self.assertIn("랜덤", body)
        self.assertIn("async function loadTools", body)
        self.assertIn("지금은 대화만", body)
        self.assertNotIn("getUserMedia", body)
        self.assertNotIn("맥 웹캠은 쓰지 않습니다", body)
        self.assertNotIn("장면을 보고, 맞는지 말한 다음", body)

    def test_real_trace_api_lists_details_and_serves_only_run_images(self):
        import os
        import tempfile
        from harness.execution_trace import activate_run, create_run, finish_run
        from harness.loop import LoopResult

        with tempfile.TemporaryDirectory() as td:
            old = os.environ.get("UGRP_REAL_TRACE_ROOT")
            os.environ["UGRP_REAL_TRACE_ROOT"] = td
            try:
                source = pathlib.Path(td) / "source.jpg"
                source.write_bytes(b"trace-image")
                run_id, run_dir = create_run(message="빨간 블록 테스트", execute=True, initial_image=source)
                with activate_run(run_id, run_dir):
                    finish_run(LoopResult(final="ok", stopped="final"))
                with self._open("/api/real-traces", headers=self.auth) as res:
                    index = self.json.loads(res.read().decode("utf-8"))
                self.assertEqual(index["runs"][0]["run_id"], run_id)
                with self._open(f"/api/real-traces/{run_id}", headers=self.auth) as res:
                    detail = self.json.loads(res.read().decode("utf-8"))
                self.assertEqual(detail["analysis"]["status"], "COMPLETED")
                rel = next((run_dir / "harness_frames").iterdir()).relative_to(run_dir)
                with self._open(f"/api/real-traces/{run_id}/asset/{rel.as_posix()}", headers=self.auth) as res:
                    self.assertEqual(res.read(), b"trace-image")
                    self.assertEqual(res.headers.get_content_type(), "image/jpeg")
                with self.assertRaises(self.urllib_error.HTTPError) as ctx:
                    self._open(f"/api/real-traces/{run_id}/asset/%2e%2e/run.json", headers=self.auth)
                self.assertEqual(ctx.exception.code, 404)
            finally:
                if old is None:
                    os.environ.pop("UGRP_REAL_TRACE_ROOT", None)
                else:
                    os.environ["UGRP_REAL_TRACE_ROOT"] = old

    def test_team_wake_does_not_pollute_private_robot_history(self):
        from harness.loop import ReplayCompleter
        from harness import web as harness_web

        self.state.history[:] = [
            {"role": "user", "content": "R1 개인 대화"},
            {"role": "assistant", "content": "개인 응답"},
        ]
        before = list(self.state.history)
        self.state.completer = ReplayCompleter([{"final": "TEAM 전용 응답"}])
        payload = self.json.dumps({
            "message": "@R1 TEAM에서만 보일 메시지",
            "team_wake": True,
            "chat_only": True,
            "execute": False,
        }).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with patch.object(harness_web.team_bus, "context_for", return_value={
            "goal": None, "chat": [], "messages": [], "events": [],
        }):
            with self._open("/api/turn", data=payload, headers=headers) as res:
                body = self.json.loads(res.read().decode("utf-8"))

        self.assertEqual(body["final"], "TEAM 전용 응답")
        self.assertEqual(self.state.history, before)

    def test_turn_lock_rejects_overlapping_robot_turn(self):
        payload = self.json.dumps({"message": "안녕"}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        self.assertTrue(self.state._turn_lock.acquire(blocking=False))
        try:
            with self.assertRaises(self.urllib_error.HTTPError) as ctx:
                self._open("/api/turn", data=payload, headers=headers)
            self.assertEqual(ctx.exception.code, 409)
            body = self.json.loads(ctx.exception.read().decode("utf-8"))
            self.assertEqual(body["error"], "robot_busy")
            self.assertEqual(body["robot_id"], "r1")
        finally:
            self.state._turn_lock.release()

    def test_turn_speaks_then_acts(self):
        payload = self.json.dumps({"message": "빨간 거 집어"}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertIn("다가갈게", data["text"])
        self.assertIn("집을까?", data["text"])
        self.assertEqual(data["tools"][0]["name"], "approach")
        self.assertTrue(data["tools"][0]["dry_run"])
        self.assertFalse(data.get("error"))
        self.assertEqual(data["events"][0]["type"], "tool")

    def test_turn_streams_agent_events(self):
        payload = self.json.dumps({"message": "빨간 거 집어", "stream": True}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            self.assertIn("event-stream", res.headers.get_content_type())
            raw = res.read().decode("utf-8")
        events = []
        for block in raw.split("\n\n"):
            line = next((row for row in block.split("\n") if row.startswith("data: ")), None)
            if line:
                events.append(self.json.loads(line[6:]))
        types = [event["type"] for event in events]
        self.assertIn("tool", types)
        self.assertEqual(types[-1], "done")
        self.assertTrue(events[-1]["tools"])

    def test_turn_stream_emits_keepalive_during_slow_completion(self):
        import os
        import time

        class SlowCompleter:
            def complete(self, messages, image=None):
                time.sleep(1.15)
                return '{"final":"ok"}'

        self.state.completer = SlowCompleter()
        payload = self.json.dumps({"message": "안녕", "stream": True}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with patch.dict(os.environ, {"UGRP_SSE_HEARTBEAT_S": "1"}):
            with self._open("/api/turn", data=payload, headers=headers) as res:
                raw = res.read().decode("utf-8")
        self.assertIn(": keepalive\n\n", raw)
        self.assertIn('"type": "done"', raw)

    def test_stream_surfaces_internal_error_as_sse_instead_of_network_drop(self):
        class BrokenCompleter:
            def complete(self, messages, image=None):
                raise RuntimeError("synthetic stream failure")

        self.state.completer = BrokenCompleter()
        payload = self.json.dumps({"message": "안녕", "stream": True}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            raw = res.read().decode("utf-8")
        self.assertIn('"type": "error"', raw)
        self.assertIn("RuntimeError: synthetic stream failure", raw)
        self.assertIn('"type": "done"', raw)

    def test_final_after_tool_requires_post_tool_observation(self):
        from harness.loop import ReplayCompleter, run_loop
        from harness.catalog import default_registry

        replies = [
            {"tool": "approach"},
            {"final": "도착"},
            {"look": True},
            {"final": "도착"},
        ]
        observed = []

        def observe():
            observed.append(True)
            return "/tmp/after.jpg"

        result = run_loop(
            ReplayCompleter(replies),
            default_registry(),
            "빨간 블록 앞으로 가",
            image="/tmp/before.jpg",
            max_steps=3,
            execute=False,
            observe=observe,
            sleeper=lambda _seconds: None,
        )
        self.assertEqual(result.stopped, "final")
        self.assertEqual(result.final, "도착")
        self.assertEqual(len(observed), 1)
        self.assertIn("must wait/look", result.steps[1].error)
        self.assertEqual(type(result.steps[2].action).__name__, "Look")

    def test_status_exposes_per_robot_action_queue(self):
        self.state.action_queue.enqueue(
            "approach", {"target_color": "red"}, source="test_status"
        )
        with self._open("/api/status", headers=self.auth) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        queue = data["action_queue"]
        self.assertEqual(queue["pending"][0]["tool"], "approach")
        self.assertEqual(queue["pending"][0]["args"], {"target_color": "red"})
        self.assertIsNone(queue["running"])

    def test_status_and_execute_gate(self):
        with self._open("/api/status", headers=self.auth) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertFalse(data["execute_allowed"])
        payload = self.json.dumps({"message": "빨간 거 집어", "execute": True}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            body = self.json.loads(res.read().decode("utf-8"))
        self.assertTrue(body["tools"][0]["dry_run"])
        self.assertNotIn("tool:", self.state.history[1]["content"])
        self.assertIn("다가갈게", self.state.history[1]["content"])

    def test_real_execute_reconciles_stale_probable_hold_before_any_motion(self):
        from harness.real_carry import CarryEvidence

        self.state.execute = True
        self.state.grab_camera = None  # hard gate after the read-only reconciliation
        self.state.state_estimator.state.grasp.state = "PROBABLE_HELD"
        self.state.state_estimator.state.grasp.held_object_color = "red"
        with patch(
            "harness.web.read_carry_evidence",
            return_value=CarryEvidence(True, False, "stale", "red", 900.0, 1500),
        ):
            payload = self.json.dumps({"message": "빨간색 블럭 집어봐", "execute": True}).encode()
            headers = {**self.auth, "Content-Type": "application/json"}
            with self._open("/api/turn", data=payload, headers=headers) as res:
                data = self.json.loads(res.read().decode("utf-8"))
        self.assertEqual(data["stopped"], "camera_unavailable")
        self.assertEqual(self.state.state_estimator.state.grasp.state, "UNKNOWN")
        self.assertIsNone(self.state.state_estimator.state.grasp.held_object_color)
        self.assertEqual(data["tools"], [])

    def test_real_execute_keeps_probable_hold_when_causal_handoff_is_fresh(self):
        from harness.real_carry import CarryEvidence

        self.state.execute = True
        self.state.grab_camera = None
        self.state.state_estimator.state.grasp.state = "PROBABLE_HELD"
        self.state.state_estimator.state.grasp.held_object_color = "red"
        with patch(
            "harness.web.read_carry_evidence",
            return_value=CarryEvidence(True, True, "fresh_precision_pick", "red", 12.0, 1500),
        ):
            payload = self.json.dumps({"message": "빨간색 블럭 집어봐", "execute": True}).encode()
            headers = {**self.auth, "Content-Type": "application/json"}
            with self._open("/api/turn", data=payload, headers=headers) as res:
                data = self.json.loads(res.read().decode("utf-8"))
        self.assertEqual(data["stopped"], "camera_unavailable")
        self.assertEqual(self.state.state_estimator.state.grasp.state, "PROBABLE_HELD")
        self.assertEqual(self.state.state_estimator.state.grasp.held_object_color, "red")

    def test_sse_disconnect_requests_current_action_cancel(self):
        import threading
        from harness.web import ChatHandler

        cancelled = []
        self.state.registry.cancel_current = lambda: cancelled.append(True) or True

        class BrokenWriter:
            def write(self, _data):
                raise BrokenPipeError("client disconnected")

            def flush(self):
                pass

        handler = object.__new__(ChatHandler)
        handler.state = self.state
        handler.wfile = BrokenWriter()
        handler._sse_write_lock = threading.Lock()
        self.assertFalse(handler._sse_write(b": keepalive\n\n"))
        self.assertTrue(self.state.cancel.is_set())
        self.assertEqual(cancelled, [True])

    def test_cancel_endpoint_sets_flag(self):
        cancelled = []
        self.state.registry.cancel_current = lambda: cancelled.append(True) or True
        with self._open("/api/cancel", data=b"{}", headers={**self.auth, "Content-Type": "application/json"}) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertTrue(data["ok"])
        self.assertTrue(data["action_terminated"])
        self.assertTrue(self.state.cancel.is_set())
        self.assertEqual(cancelled, [True])

    def test_sim_wake_without_systemctl_returns_graceful_offline(self):
        from harness import web as web_module

        self.state.ui_mode = "sim"
        with patch.object(web_module.shutil, "which", return_value=None), patch.object(
            web_module, "urlopen", side_effect=OSError("bridge down")
        ):
            with self._open("/api/sim/wake", data=b"{}", headers={**self.auth, "Content-Type": "application/json"}) as res:
                data = self.json.loads(res.read().decode("utf-8"))
        self.assertTrue(data["ok"])
        self.assertFalse(data["sim_online"])
        self.assertFalse(data["recovery_started"])
        self.assertIn("systemctl", data["recovery_detail"])

    def test_sim_activity_without_systemctl_stays_200(self):
        import io

        from harness import web as web_module

        self.state.ui_mode = "sim"

        def fake_urlopen(req, timeout=None):
            url = req.full_url if hasattr(req, "full_url") else str(req)
            if url.endswith("/sim/user-activity"):
                return io.BytesIO(b"{}")
            raise OSError("bridge health down")

        with patch.object(web_module.shutil, "which", return_value=None), patch.object(
            web_module, "urlopen", side_effect=fake_urlopen
        ):
            try:
                with self._open("/api/sim/activity", data=b"{}", headers={**self.auth, "Content-Type": "application/json"}) as res:
                    data = self.json.loads(res.read().decode("utf-8"))
            except self.urllib_error.HTTPError as exc:
                self.fail(f"/api/sim/activity raised HTTP {exc.code}")
        self.assertTrue(data["ok"])
        self.assertFalse(data["sim_online"])

    def test_turn_without_scene_is_talk_only(self):
        from harness.loop import ReplayCompleter

        self.state.grab_camera = None
        self.state.completer = ReplayCompleter(
            [{"final": "지금은 로봇이 없어서 대화만 할 수 있어"}]
        )
        payload = self.json.dumps({"message": "안녕"}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertEqual(data["text"], "지금은 로봇이 없어서 대화만 할 수 있어")
        self.assertFalse(data.get("tools"))
        self.assertEqual(data["events"][0]["type"], "final")

    def test_real_execute_without_camera_never_claims_motion(self):
        self.state.execute = True
        self.state.grab_camera = None
        payload = self.json.dumps({"message": "빨간 블록 찾아줘", "execute": True}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertEqual(data["stopped"], "camera_unavailable")
        self.assertEqual(data["tools"], [])
        self.assertIn("움직이지 않았어", data["text"])


    def test_sim_execute_without_camera_never_claims_motion(self):
        self.state.execute = True
        self.state.ui_mode = "sim"
        self.state.grab_camera = None
        payload = self.json.dumps({"message": "빨간 블록 집어줘", "execute": True}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertEqual(data["stopped"], "gpu_offline")
        self.assertEqual(data["tools"], [])
        self.assertIn("실행하지 않았어", data["text"])
        self.assertNotIn("집기 동작까지 수행", data["text"])

    def test_status_talk_only_until_frame(self):
        with self._open("/api/status", headers=self.auth) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertTrue(data["talk_only"])
        self.state.remember_frame(b"\xff\xd8\xff\xd9")
        with self._open("/api/status", headers=self.auth) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertFalse(data["talk_only"])

    def test_sim_status_keeps_quiet_authoritative_camera_available(self):
        import time

        self.state.ui_mode = "sim"
        self.state.robot_id = "r1"
        self.state.sim_bridge_health = lambda force=False: {
            "remote_ws_connected": True,
            "remote_authoritative": True,
            "remote_episode_id": 4,
            "robot_frame_meta": {
                "r1": {"generated_wall_s": time.time() - 60.0}
            },
        }
        with self._open("/api/status", headers=self.auth) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertFalse(data["talk_only"])
        self.assertTrue(data["camera_available"])
        self.assertFalse(data["camera_frame_fresh"])

    def test_turn_accepts_image_data_url(self):
        png = (
            "data:image/png;base64,"
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+ip1sAAAAASUVORK5CYII="
        )
        payload = self.json.dumps({"message": "이거 맞아?", "image": png}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertTrue(data["text"])
        self.assertFalse(data.get("error"))

    def test_camera_returns_jpeg(self):
        jpeg = b"\xff\xd8\xff\xd9"
        self.state.grab_camera = lambda: jpeg
        with self._open("/api/camera", headers=self.auth) as res:
            self.assertEqual(res.headers.get_content_type(), "image/jpeg")
            self.assertEqual(res.read(), jpeg)

    def test_snapshot_ui_frame_can_update_cache_without_mutating_planner_state(self):
        from unittest.mock import patch

        self.state.ui_mode = "sim"
        self.state.state_estimator.state.target.visible = True
        fake = {"visible": False, "cx": None, "cy": None, "area_ratio": None}
        with patch("harness.web.detect_red_target_bytes", return_value=fake):
            self.state.remember_frame(b"ui-frame", perceive=False)
            self.assertEqual(self.state.cached_frame(), b"ui-frame")
            self.assertTrue(self.state.state_estimator.state.target.visible)
            self.state._last_perception_at = 0.0
            self.state.remember_frame(b"planner-frame", perceive=True)
        self.assertFalse(self.state.state_estimator.state.target.visible)

    def test_snapshot_backed_camera_bypasses_stale_startup_cache(self):
        old = b"\xff\xd8old\xff\xd9"
        fresh = b"\xff\xd8fresh\xff\xd9"
        calls = []
        self.state.remember_frame(old)
        self.state.prefer_fresh_camera_snapshot = True
        self.state.grab_camera = lambda: (calls.append("grab") or fresh)
        with self._open("/api/camera", headers=self.auth) as res:
            self.assertEqual(res.headers.get_content_type(), "image/jpeg")
            self.assertEqual(res.read(), fresh)
        self.assertEqual(calls, ["grab"])

    def test_physical_camera_keeps_cache_first_path(self):
        old = b"\xff\xd8old\xff\xd9"
        fresh = b"\xff\xd8fresh\xff\xd9"
        calls = []
        self.state.remember_frame(old)
        self.state.prefer_fresh_camera_snapshot = False
        self.state.grab_camera = lambda: (calls.append("grab") or fresh)
        with self._open("/api/camera", headers=self.auth) as res:
            self.assertEqual(res.read(), old)
        self.assertEqual(calls, [])

    def test_turn_grabs_masterpi_camera_when_no_upload(self):
        seen = []

        def grab():
            seen.append("grab")
            return b"\xff\xd8\xff\xd9"

        self.state.grab_camera = grab
        payload = self.json.dumps({"message": "지금 장면 맞아?"}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/turn", data=payload, headers=headers) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertGreaterEqual(len(seen), 1)
        self.assertTrue(data["text"])
        self.assertFalse(data.get("error"))
        self.assertTrue(self.state.last_image)
        self.assertTrue(data["events"])

    def test_lists_python_tools(self):
        with self._open("/api/tools", headers=self.auth) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        names = [tool["name"] for tool in data["tools"]]
        self.assertEqual(names, sorted(SKILLS))
        self.assertIn("robot_actions.py", data["file"])

    def test_camera_stream_proxies_bytes(self):
        class FakeStream:
            content_type = "multipart/x-mixed-replace; boundary=frame"

            def __init__(self):
                self._data = b"--frame\r\n\xff\xd8\xff\xd9"
                self._sent = False

            def read(self, size=8192):
                if self._sent:
                    return b""
                self._sent = True
                return self._data

            def close(self):
                pass

        self.state.open_stream = FakeStream
        with self._open("/api/camera/stream", headers=self.auth) as res:
            self.assertIn("multipart", res.headers.get_content_type())
            self.assertEqual(res.read(), b"--frame\r\n\xff\xd8\xff\xd9")
        self.assertEqual(self.state.cached_frame(), b"\xff\xd8\xff\xd9")

    def test_robot_status_and_arm_command(self):
        class FakeRobot:
            def __init__(self):
                self.calls = []
                self.body = {
                    "connected": True,
                    "motors": [{"id": 1, "speed": 0}],
                    "arm": [{"id": 1, "name": "집게", "pulse": 2000}],
                }

            def snapshot(self):
                return self.body

            def refresh(self):
                return self.body

            def drive(self, payload):
                self.calls.append(("drive", payload))
                return self.body

            def arm(self, payload):
                self.calls.append(("arm", payload))
                self.body = dict(self.body)
                self.body["arm"] = [{"id": payload["servo"], "name": "집게", "pulse": payload["pulse"]}]
                return self.body

            def stop(self):
                self.calls.append(("stop", {}))
                return self.body

        robot = FakeRobot()
        self.state.robot = robot
        with self._open("/api/robot", headers=self.auth) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertTrue(data["connected"])
        payload = self.json.dumps({"servo": 1, "pulse": 1800, "duration": 0.4}).encode()
        headers = {**self.auth, "Content-Type": "application/json"}
        with self._open("/api/robot/arm", data=payload, headers=headers) as res:
            data = self.json.loads(res.read().decode("utf-8"))
        self.assertEqual(robot.calls[0][0], "arm")
        self.assertEqual(data["arm"][0]["pulse"], 1800)


class PiCameraTests(unittest.TestCase):
    def test_rejects_non_jpeg(self):
        from harness.pi_camera import CameraError, grab_snapshot

        def ssh_run(*args, **kwargs):
            class Result:
                returncode = 0
                stdout = b"not-a-jpeg"
                stderr = b""

            return Result()

        with self.assertRaises(CameraError):
            grab_snapshot(host="ugrp1", ssh_run=ssh_run, ssh_args=("ugrp1", []))

    def test_reads_jpeg_from_ssh(self):
        from harness.pi_camera import grab_snapshot

        jpeg = b"\xff\xd8\xff\xd9"

        def ssh_run(cmd, **kwargs):
            class Result:
                returncode = 0
                stdout = jpeg
                stderr = b""

            self.assertEqual(cmd[0], "ssh")
            self.assertIn("curl -sf", cmd[-1])
            return Result()

        data = grab_snapshot(host="ugrp1", ssh_run=ssh_run, ssh_args=("ugrp1", []))
        self.assertEqual(data, jpeg)

    def test_camera_self_heal_only_starts_existing_service_and_never_kills(self):
        from harness.pi_camera import ensure_camera_service

        seen = []
        def ssh_run(cmd, **kwargs):
            seen.append(cmd)
            class Result:
                returncode = 0
                stdout = b""
                stderr = b""
            return Result()

        self.assertTrue(ensure_camera_service(
            host="ugrp1", ssh_run=ssh_run, ssh_args=("ugrp1", [])
        ))
        self.assertEqual(len(seen), 1)
        command = seen[0][-1]
        self.assertIn("curl -sf", command)
        self.assertIn("systemctl --user start ugrp-camera.service", command)
        lowered = command.lower()
        for forbidden in ("pkill", "killall", "systemctl --user stop", "systemctl stop"):
            self.assertNotIn(forbidden, lowered)

    def test_camera_self_heal_reports_unavailable_without_destructive_recovery(self):
        from harness.pi_camera import ensure_camera_service

        def ssh_run(cmd, **kwargs):
            class Result:
                returncode = 1
                stdout = b""
                stderr = b"camera still unavailable"
            return Result()

        self.assertFalse(ensure_camera_service(
            host="ugrp1", ssh_run=ssh_run, ssh_args=("ugrp1", [])
        ))

    def test_pops_jpegs_from_mjpeg_buffer(self):
        from harness.pi_camera import pop_jpegs

        first = b"\xff\xd8\x01\xff\xd9"
        second = b"\xff\xd8\x02\xff\xd9"
        buf = bytearray(b"hdr" + first + b"--x\r\n" + second[:3])
        frames = pop_jpegs(buf)
        self.assertEqual(frames, [first])
        self.assertEqual(bytes(buf), second[:3])
        buf.extend(second[3:])
        self.assertEqual(pop_jpegs(buf), [second])
        self.assertEqual(bytes(buf), b"")


class RobotPanelTests(unittest.TestCase):
    def test_snapshot_does_not_run_ssh(self):
        from harness.robot import RobotPanel

        class Runner:
            def run(self, argv, timeout):
                raise AssertionError(f"unexpected ssh {argv}")

        panel = RobotPanel(host="ugrp1", runner=Runner(), ssh_args=("ugrp1", []))
        body = panel.snapshot()
        self.assertFalse(body["connected"])
        self.assertEqual(body["arm"][0]["pulse"], 2000)

    def test_drive_keeps_last_command(self):
        from dashboard.server import CommandResult
        from harness.robot import RobotPanel

        calls = []

        class Runner:
            def run(self, argv, timeout):
                calls.append((list(argv), timeout))
                return CommandResult(0, "{}", "")

        panel = RobotPanel(host="ugrp1", runner=Runner(), ssh_args=("ugrp1", []))
        body = panel.drive({"direction": "forward", "speed": 35, "duration": 0.3})
        self.assertTrue(body["connected"])
        self.assertEqual(body["motion"]["direction"], "forward")
        self.assertEqual(calls[0][0][0], "ssh")
        self.assertIn("drive", calls[0][0])
        self.assertIn("forward", calls[0][0])


class TalkTests(unittest.TestCase):
    def test_turn_keeps_history(self):
        from harness.talk import ReplayTalker, turn

        history, answer = turn([], "안녕", ReplayTalker(["반가워"]))
        self.assertEqual(answer, "반가워")
        self.assertEqual(
            history,
            [
                {"role": "user", "content": "안녕"},
                {"role": "assistant", "content": "반가워"},
            ],
        )


class CliTests(unittest.TestCase):
    def test_list_tools(self):
        code = main(["--list-tools"])
        self.assertEqual(code, 0)

    def test_replay_dry_run(self):
        replay = json.dumps([{"tool": "approach"}, {"final": "ok"}], ensure_ascii=False)
        code = main(["빨간 블록 앞으로", "--replay", replay])
        self.assertEqual(code, 0)


class VisualVerifierTests(unittest.TestCase):
    def test_parses_fenced_verifier_json(self):
        from harness.verify import verify_final

        class Fake:
            def complete(self, messages, image=None):
                self.image = image
                return '```json\n{"accept": false, "success_claim": true, "evidence": "바닥에 있음"}\n```'

        fake = Fake()
        accepted, evidence = verify_final(fake, "집어", "집었어", "/tmp/x.jpg")
        self.assertFalse(accepted)
        self.assertEqual(evidence, "바닥에 있음")
        self.assertEqual(fake.image, "/tmp/x.jpg")


class GroqTests(unittest.TestCase):
    def test_loads_keys_from_env_and_file(self):
        import tempfile
        from pathlib import Path

        from harness.groq import load_groq_keys

        path = Path(tempfile.mkdtemp()) / "keys"
        path.write_text("gsk_filekeyxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n", encoding="utf-8")
        keys = load_groq_keys(
            env={"GROQ_API_KEY": "gsk_envkeyxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"},
            path=path,
        )
        self.assertEqual(
            keys,
            [
                "gsk_envkeyxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
                "gsk_filekeyxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx",
            ],
        )

    def test_rotates_on_rate_limit_and_sends_image(self):
        import json
        import tempfile
        from io import BytesIO
        from pathlib import Path
        from urllib.error import HTTPError

        from harness.groq import GroqCompleter

        image = Path(tempfile.mkdtemp()) / "scene.jpg"
        image.write_bytes(b"\xff\xd8\xff\xd9")
        seen = []

        class Ok:
            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def read(self):
                return json.dumps(
                    {"choices": [{"message": {"content": '{"final": "ok"}'}}]}
                ).encode()

        def http_open(request, timeout=0):
            seen.append(request.get_header("Authorization"))
            if len(seen) == 1:
                raise HTTPError(
                    request.full_url, 429, "Too Many Requests", hdrs=None, fp=BytesIO(b"{}")
                )
            body = json.loads(request.data.decode())
            self.assertEqual(body["model"], "qwen/qwen3.8-27b")
            self.assertEqual(body["reasoning_effort"], "none")
            self.assertEqual(body["reasoning_format"], "hidden")
            self.assertNotIn("response_format", body)
            self.assertEqual(request.get_header("User-agent"), "ugrp-harness/1.0")
            content = body["messages"][-1]["content"]
            self.assertEqual(content[0]["type"], "text")
            self.assertTrue(content[1]["image_url"]["url"].startswith("data:image/jpeg;base64,"))
            return Ok()

        completer = GroqCompleter(
            keys=["gsk_one" + ("x" * 40), "gsk_two" + ("y" * 40)],
            http_open=http_open,
        )
        text = completer.complete(
            [{"role": "user", "content": "빨간 블록 보여?"}],
            image=str(image),
        )
        self.assertEqual(text, '{"final": "ok"}')
        self.assertEqual(len(seen), 2)

    def test_retries_shared_rate_limit_after_retry_after(self):
        import json
        from io import BytesIO
        from urllib.error import HTTPError

        from harness.groq import GroqCompleter

        calls = []
        sleeps = []

        class Ok:
            def __enter__(self):
                return self
            def __exit__(self, *args):
                return False
            def read(self):
                return json.dumps({"choices": [{"message": {"content": '{"final": "ok"}'}}]}).encode()

        def http_open(request, timeout=0):
            calls.append(request.get_header("Authorization"))
            if len(calls) <= 2:
                raise HTTPError(
                    request.full_url, 429, "Too Many Requests",
                    hdrs={"Retry-After": "0.2"}, fp=BytesIO(b"{}")
                )
            return Ok()

        completer = GroqCompleter(
            keys=["gsk_one" + ("x" * 40), "gsk_two" + ("y" * 40)],
            http_open=http_open, sleeper=sleeps.append, rate_limit_retries=2,
        )
        self.assertEqual(
            completer.complete([{"role": "user", "content": "test"}]),
            '{"final": "ok"}',
        )
        self.assertEqual(len(calls), 3)
        self.assertEqual(sleeps, [0.2])

    def test_default_completion_budget_is_small_for_agent_json(self):
        from harness.groq import GroqCompleter
        completer = GroqCompleter(keys=["gsk_one" + ("x" * 40)], http_open=lambda *a, **k: None)
        self.assertEqual(completer.max_tokens, 256)

    def test_strips_think_tags(self):
        from harness.groq import _strip_think

        self.assertEqual(
            _strip_think('<think>plan</think>\n{"final": "ok"}'),
            '{"final": "ok"}',
        )


if __name__ == "__main__":
    unittest.main()


class CleanHistoryTests(unittest.TestCase):
    def test_history_does_not_reinject_internal_tool_log(self):
        from harness.chat import history_text
        from harness.loop import LoopResult, Step
        from harness.protocol import ToolCall, FinalAnswer
        result = LoopResult(final="됐어", stopped="final")
        result.steps = [
            Step(raw="", action=ToolCall(name="turn_right", args={}, say="오른쪽 볼게"), result={"result": {"debug_marker": "internal-only"}}),
            Step(raw="", action=FinalAnswer(text="됐어")),
        ]
        text = history_text(result)
        self.assertIn("오른쪽 볼게", text)
        self.assertIn("됐어", text)
        self.assertNotIn("tool:", text)
        self.assertNotIn("internal-only", text)
