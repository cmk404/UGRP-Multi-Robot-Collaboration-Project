import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class PlannerInputParityTests(unittest.TestCase):
    def test_sim_preserves_real_tool_schema_plus_calibrated_team_actions(self):
        from harness.catalog import default_registry
        real = default_registry(actions_path="scripts/robot_actions.py")
        sim = default_registry(actions_path="scripts/sim_actions.py")
        self.assertEqual(
            set(sim.names()) - set(real.names()),
            {
                "stage_base", "stack_on", "team_tower", "team_beam_transport",
                "team_zone_transfer",
            },
        )
        self.assertEqual(set(real.names()) - set(sim.names()), set())
        schema = sim.prompt_schema()
        self.assertIn("stage_base", schema)
        self.assertIn("stack_on", schema)
        self.assertIn("target_color", schema)
        self.assertIn("destination_color", schema)

    def test_planner_tool_feedback_removes_environment_specific_geometry(self):
        from harness.loop import _planner_tool_result
        common = {
            "ok": True, "tool": "track",
            "result": {
                "skill": "track", "command_status": "ACCEPTED",
                "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
                "target_color": "red", "center_verified": True,
            },
        }
        real = {**common, "result": {**common["result"],
            "target_vision": {"visible": True},
            "verification_source": "pi_multiframe_centered_track",
        }}
        sim = {**common, "result": {**common["result"],
            "target_vision": {"visible": True, "cx": .12, "cy": .81, "area_ratio": .09, "bbox": [0,0,1,1]},
            "camera_red": {"visible": True, "cx": .12},
            "verification_source": "sim_internal_detail",
            "spatial_memory": {"red": {"position_xy": [1.2, 3.4]}},
        }}
        self.assertEqual(_planner_tool_result(real), _planner_tool_result(sim))
        text = repr(_planner_tool_result(sim))
        self.assertNotIn("position_xy", text)
        self.assertNotIn("verification_source", text)
        self.assertNotIn("bbox", text)
        self.assertNotIn("area_ratio", text)

    def test_planner_world_state_hides_metric_simulator_geometry(self):
        from harness.loop import _planner_state_public
        from harness.state import StateEstimator
        est = StateEstimator()
        est.state.spatial_memory = {
            "red": {
                "visible": True, "image_xy": [.4, .6],
                "metric_position_available": True, "position_xy": [1.2, 3.4],
                "relation": "GROUND", "distance_m": .12, "bearing_deg": 9.0,
            }
        }
        landmark = _planner_state_public(est.state)["landmarks"]["red"]
        self.assertEqual(landmark, {"visible": True, "image_xy": [.4, .6]})

    def test_sim_action_result_does_not_surface_mujoco_geometry(self):
        from unittest.mock import patch
        import scripts.sim_actions as sim_actions
        raw = {
            "ok": True, "reason": "done",
            "vision": {"visible": True, "cx": .2, "cy": .7, "area_ratio": .1},
            "blue_vision": {"visible": True, "cx": .8},
            "spatial_memory": {"red": {"position_xy": [9.0, 9.0], "distance_m": .1}},
        }
        with patch.object(sim_actions, "_bridge_health", return_value={}), \
             patch.object(sim_actions, "_post_bridge_action", return_value=raw):
            result = sim_actions.run("search", target_color="red")
        self.assertEqual(result["target_vision"], {"visible": True})
        self.assertEqual(result["camera_red"], {"visible": True})
        self.assertNotIn("camera_blue", result)
        self.assertNotIn("spatial_memory", result)



class SimPickContractParityTests(unittest.TestCase):
    def test_sim_chat_state_uses_strict_precision_pick_preconditions(self):
        from unittest.mock import patch
        from harness.loop import ReplayCompleter
        from harness.web import ChatState
        with patch.dict("os.environ", {"UGRP_UI_MODE": "sim"}, clear=False):
            state = ChatState(
                completer=ReplayCompleter([]),
                actions_path="scripts/sim_actions.py",
            )
        self.assertTrue(state.executive.strict_pick_preconditions)
