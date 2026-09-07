from __future__ import annotations

import unittest
from unittest import mock

from harness.catalog import default_registry
from harness.loop import ReplayCompleter, run_loop, _fusable_acquisition_prefix, _acquisition_recovery_chain
from harness.executive import TaskExecutive
from scripts import robot_actions


PROGRAM = [
    ("search", {"target_color": "blue"}),
    ("track", {"target_color": "blue"}),
    ("approach", {"target_color": "blue"}),
    ("pick", {"target_color": "blue"}),
]


class HiddenProgramSurfaceTests(unittest.TestCase):
    def tearDown(self):
        for path in (robot_actions.LAST_REMOTE_ERROR_PATH, robot_actions.LAST_REAL_TRACE_PATH):
            try:
                path.unlink()
            except FileNotFoundError:
                pass

    def test_program_runner_is_hidden_from_public_tool_surface(self):
        registry = default_registry(actions_path="scripts/robot_actions.py")
        self.assertIsNotNone(registry.program_runner)
        # Public action count may grow; the invariant is that the hidden fused
        # executor itself never becomes a model-selectable action.
        self.assertNotIn("grasp", registry.names())
        self.assertNotIn("run_program", registry.names())
        self.assertNotIn("run_program", registry.prompt_schema())

    def test_custom_runner_disables_hidden_production_fastpath(self):
        registry = default_registry(
            actions_path="scripts/robot_actions.py", runner=lambda name, **kwargs: {}
        )
        self.assertIsNone(registry.program_runner)

    def test_run_program_launches_task_runner_once_and_returns_public_stage_results(self):
        with mock.patch.object(robot_actions, "_run_skill_process", return_value=0) as launch, \
             mock.patch.object(robot_actions, "_trace_fields", return_value={}):
            results = robot_actions.run_program(PROGRAM)
        launch.assert_called_once()
        self.assertEqual(launch.call_args.args[0], "task_runner")
        self.assertEqual([r["skill"] for r in results], ["search", "track", "approach", "pick"])
        self.assertTrue(all(r["target_color"] == "blue" for r in results))
        self.assertTrue(results[1]["center_verified"])
        self.assertTrue(results[2]["range_verified"])
        self.assertEqual(results[3]["outcome_status"], "UNKNOWN")
        self.assertTrue(results[3]["visual_hold"]["probable"])

    def test_run_program_attributes_failure_to_internal_public_stage(self):
        def launch(*args, **kwargs):
            robot_actions.LAST_REMOTE_ERROR_PATH.write_text(
                "task stage=approach: target disappeared during visual approach",
                encoding="utf-8",
            )
            return 1

        with mock.patch.object(robot_actions, "_run_skill_process", side_effect=launch), \
             mock.patch.object(robot_actions, "_trace_fields", return_value={}):
            results = robot_actions.run_program(PROGRAM)
        self.assertEqual([r["skill"] for r in results], ["search", "track", "approach"])
        self.assertFalse(results[-1]["ok"])
        self.assertEqual(results[-1]["failure_code"], "TARGET_NOT_VISIBLE")
        self.assertEqual(results[-1]["recommended_recovery"], "search")



    def test_suffix_approach_pick_is_fusable(self):
        suffix = [("approach", {"target_color": "red"}), ("pick", {"target_color": "red"})]
        self.assertEqual(_fusable_acquisition_prefix(suffix), suffix)

    def test_search_recovery_rebuilds_track_before_approach(self):
        chain = _acquisition_recovery_chain("search", "approach", {"target_color": "red"})
        self.assertEqual(chain, [
            ("search", {"target_color": "red"}),
            ("track", {"target_color": "red"}),
            ("approach", {"target_color": "red"}),
        ])

    def test_run_program_approach_pick_starts_task_runner_at_approach(self):
        suffix = [("approach", {"target_color": "red"}), ("pick", {"target_color": "red"})]
        with mock.patch.object(robot_actions, "_run_skill_process", return_value=0) as launch, \
             mock.patch.object(robot_actions, "_trace_fields", return_value={}):
            results = robot_actions.run_program(suffix)
        self.assertEqual([r["skill"] for r in results], ["approach", "pick"])
        argv = launch.call_args.args[1]
        self.assertEqual(argv[argv.index("--start-stage") + 1], "approach")

class HiddenProgramLoopTests(unittest.TestCase):
    @mock.patch.dict("os.environ", {"UGRP_STRUCTURED_RED_FASTPATH": "1"})
    def test_structured_goal_fuses_process_but_emits_same_four_public_stages(self):
        public_calls = []
        program_calls = []

        def public_runner(name, **kwargs):
            public_calls.append((name, kwargs))
            raise AssertionError("fused acquisition must not launch standalone public processes")

        registry = default_registry(
            actions_path="scripts/robot_actions.py", runner=public_runner
        )

        def program_runner(program):
            program_calls.append(program)
            color = program[0][1]["target_color"]
            return [
                {
                    "ok": True, "skill": "search", "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "target_vision": {"visible": True},
                },
                {
                    "ok": True, "skill": "track", "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "target_vision": {"visible": True},
                    "center_verified": True,
                },
                {
                    "ok": True, "skill": "approach", "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "ACHIEVED", "target_vision": {"visible": True},
                    "center_verified": True, "range_verified": True,
                },
                {
                    "ok": True, "skill": "pick", "target_color": color,
                    "command_status": "ACCEPTED", "execution_status": "COMPLETED",
                    "outcome_status": "UNKNOWN",
                    "visual_hold": {"probable": True, "confidence": 0.65},
                },
            ]

        registry.program_runner = program_runner
        result = run_loop(
            ReplayCompleter([]), registry, "파란 블럭 집어봐",
            execute=True, auto_observe=True, observe=lambda: None,
            max_steps=8, executive=TaskExecutive(strict_pick_preconditions=True),
        )
        self.assertEqual(len(program_calls), 1)
        self.assertEqual(program_calls[0], PROGRAM)
        self.assertEqual(public_calls, [])
        stage_steps = [step.action.name for step in result.steps if getattr(step.action, "name", None)]
        self.assertEqual(stage_steps, ["search", "track", "approach", "pick"])
        self.assertTrue(all(step.result.get("fused_program") for step in result.steps[:4]))
        self.assertEqual(result.world_state["grasp"]["state"], "PROBABLE_HELD")


if __name__ == "__main__":
    unittest.main()
