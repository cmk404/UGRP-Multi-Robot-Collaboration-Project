from __future__ import annotations

import unittest

from harness.catalog import default_registry
from harness.executive import TaskExecutive
from harness.goals import infer_goal
from harness.state import StateEstimator
from scripts import robot_actions


class ColorGoalRoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.names = default_registry(actions_path="scripts/sim_actions.py", runner=lambda _: {}).names()

    def plan(self, text: str):
        goal = infer_goal(text)
        return goal, TaskExecutive().plan(goal, self.names)

    def test_blue_pick_uses_shared_generic_pipeline(self):
        goal, plan = self.plan("파란 블럭 집어봐")
        self.assertEqual((goal.kind, goal.subject), ("GRASP", "BLUE_BLOCK"))
        self.assertEqual(plan, ["search", "track", "approach", "pick"])

    def test_yellow_pick_preserves_subject(self):
        goal, plan = self.plan("노란 블럭 집어봐")
        self.assertEqual((goal.kind, goal.subject), ("GRASP", "YELLOW_BLOCK"))
        self.assertEqual(plan, ["search", "track", "approach", "pick"])

    def test_red_on_blue_preserves_source_and_destination(self):
        goal, plan = self.plan("빨간 블럭 잡아서 파란 블럭 위에 올려줘")
        self.assertEqual((goal.kind, goal.subject, goal.target), ("PLACE", "RED_BLOCK", "BLUE_BLOCK"))
        self.assertEqual(plan, ["search", "track", "approach", "pick", "search_destination", "place"])

    def test_blue_on_yellow_preserves_source_and_destination(self):
        goal, plan = self.plan("파란 블럭을 노란 블럭 위에 올려줘")
        self.assertEqual((goal.kind, goal.subject, goal.target), ("PLACE", "BLUE_BLOCK", "YELLOW_BLOCK"))
        self.assertEqual(plan, ["search", "track", "approach", "pick", "search_destination", "place"])

    def test_tower_handoff_status_sentence_does_not_hide_the_source_colour(self):
        # R3's router handoff: a status sentence ("...바닥층에 놓였어.") precedes the
        # command. The "에 놓" of the status sentence used to split the text, so
        # the source became UNKNOWN and the plan defaulted to the red block.
        goal, plan = self.plan(
            "[TEAM · R3 → R1] [TOWER_STEP_2] 협업 2/3: 노란 블럭이 바닥층에 놓였어. "
            "이제 파란 블럭을 노란 블럭 위에 올려 줘."
        )
        self.assertEqual((goal.kind, goal.subject, goal.target), ("PLACE", "BLUE_BLOCK", "YELLOW_BLOCK"))
        self.assertEqual(plan, ["search", "track", "approach", "pick", "search_destination", "place"])
        goal, _ = self.plan("협업 3/3: 파란 블럭이 노란 블럭 위에 올라갔어. 이제 빨간 블럭을 파란 블럭 위에 올려 줘.")
        self.assertEqual((goal.subject, goal.target), ("RED_BLOCK", "BLUE_BLOCK"))

    def test_different_held_color_is_put_down_before_new_pick(self):
        goal = infer_goal("파란 블럭 집어봐")
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        calls = TaskExecutive(strict_pick_preconditions=True).plan_calls(
            goal, list(robot_actions.ACTIONS), est.state
        )
        self.assertEqual(
            calls,
            [
                ("put_down", {}),
                ("search", {"target_color": "blue"}),
                ("track", {"target_color": "blue"}),
                ("approach", {"target_color": "blue"}),
                ("pick", {"target_color": "blue"}),
            ],
        )

    def test_real_blue_pick_uses_generic_tools_with_runtime_color(self):
        goal = infer_goal("파란 블록 집어")
        calls = TaskExecutive().plan_calls(goal, list(robot_actions.ACTIONS))
        self.assertEqual(
            calls,
            [
                ("search", {"target_color": "blue"}),
                ("track", {"target_color": "blue"}),
                ("approach", {"target_color": "blue"}),
                ("pick", {"target_color": "blue"}),
            ],
        )

    def test_destination_only_followup_exact_operator_phrase_places_held_red_on_blue(self):
        goal = infer_goal("파란색 블럭 위에 올려봐")
        self.assertEqual(
            (goal.kind, goal.subject, goal.target),
            ("PLACE", "UNKNOWN", "BLUE_BLOCK"),
        )
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        calls = TaskExecutive(strict_pick_preconditions=True).plan_calls(
            goal, list(robot_actions.ACTIONS), est.state
        )
        self.assertEqual(
            calls,
            [
                ("search_destination", {"target_color": "red", "destination_color": "blue"}),
                ("place", {"target_color": "red", "destination_color": "blue"}),
            ],
        )

    def test_exact_user_followup_phrase_treats_named_color_as_destination(self):
        goal = infer_goal("잡았는데.. 이제 파란색 블럭위에 올려볼래? 그거")
        self.assertEqual((goal.kind, goal.subject, goal.target), ("PLACE", "UNKNOWN", "BLUE_BLOCK"))
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        calls = TaskExecutive().plan_calls(goal, list(robot_actions.ACTIONS), est.state)
        self.assertEqual(calls, [
            ("search_destination", {"target_color": "red", "destination_color": "blue"}),
            ("place", {"target_color": "red", "destination_color": "blue"}),
        ])

    def test_selected_blue_scene_updates_blue_target_not_visible_red(self):
        est = StateEstimator()
        est.set_goal_context(target_color="blue")
        est.update_scene_memory({
            "red": {"visible": True, "cx": 0.10, "cy": 0.10, "area_ratio": 0.01},
            "blue": {"visible": True, "cx": 0.70, "cy": 0.60, "area_ratio": 0.05},
        })
        self.assertEqual(est.state.target.selected_color, "blue")
        self.assertAlmostEqual(est.state.target.cx, 0.70)
        self.assertAlmostEqual(est.state.target.cy, 0.60)

    def test_pronoun_followup_resolves_held_object_and_places_directly(self):
        goal = infer_goal("그거 파란 블록 위에 올려")
        self.assertEqual((goal.kind, goal.subject, goal.target), ("PLACE", "UNKNOWN", "BLUE_BLOCK"))
        est = StateEstimator()
        est.state.grasp.state = "PROBABLE_HELD"
        est.state.grasp.held_object_color = "red"
        calls = TaskExecutive().plan_calls(goal, list(robot_actions.ACTIONS), est.state)
        self.assertEqual(calls, [
            ("search_destination", {"target_color": "red", "destination_color": "blue"}),
            ("place", {"target_color": "red", "destination_color": "blue"}),
        ])


if __name__ == "__main__":
    unittest.main()
