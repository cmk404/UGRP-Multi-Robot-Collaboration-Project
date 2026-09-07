#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest
from unittest import mock

from harness.catalog import default_registry, resolve_plan_step, tools_payload
from harness.loop import dispatch
from harness.protocol import ToolCall
from harness.registry import ToolError
from harness.state import StateEstimator

ROOT = pathlib.Path(__file__).resolve().parents[1]
PKG = ROOT / "scripts" / "red_block"


def load(name):
    path = PKG / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"test_{name}_primitive_module", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.path.insert(0, str(PKG))
    spec.loader.exec_module(module)
    return module


PRIMITIVE = load("primitive")


class PrimitiveCatalogTests(unittest.TestCase):
    def test_real_registry_exposes_composable_chassis_primitives(self):
        registry = default_registry(actions_path="scripts/robot_actions.py")
        for name in ("move_forward", "move_backward", "move_left", "move_right", "turn_left", "turn_right", "stop_motion"):
            self.assertIn(name, registry.names())
        self.assertNotIn("strafe_left", registry.names())
        self.assertNotIn("strafe_right", registry.names())
        params = registry.get("move_forward").parameters
        self.assertEqual([p.name for p in params], ["speed", "duration"])
        self.assertEqual(params[0].default, 35)
        self.assertAlmostEqual(params[1].default, 0.30)
        lateral = registry.get("move_left").parameters
        self.assertEqual(lateral[0].default, 65)
        self.assertAlmostEqual(lateral[1].default, 0.65)

    def test_tool_args_reach_runner(self):
        seen = []
        def runner(name, **kwargs):
            seen.append((name, kwargs))
            return {"ok": True}
        registry = default_registry(actions_path="scripts/robot_actions.py", runner=runner)
        out = dispatch(
            registry,
            ToolCall(name="turn_right", args={"speed": 35, "duration": 0.2}),
            execute=True,
        )
        self.assertTrue(out["ok"])
        self.assertEqual(seen, [("turn_right", {"speed": 35, "duration": 0.2})])

    def test_wrong_primitive_arg_type_is_rejected_before_execution(self):
        registry = default_registry(actions_path="scripts/robot_actions.py", runner=lambda *a, **k: {})
        with self.assertRaises(ToolError):
            dispatch(registry, ToolCall(name="move_forward", args={"duration": "long"}), execute=True)

    def test_api_payload_exposes_parameter_metadata(self):
        registry = default_registry(actions_path="scripts/robot_actions.py")
        payload = tools_payload(registry, source="scripts/robot_actions.py")
        item = next(x for x in payload["tools"] if x["name"] == "turn_left")
        self.assertEqual([p["name"] for p in item["parameters"]], ["speed", "duration"])

    def test_korean_plan_aliases_resolve_to_primitives(self):
        names = default_registry(actions_path="scripts/robot_actions.py").names()
        self.assertEqual(resolve_plan_step("우회전", names), "turn_right")
        self.assertEqual(resolve_plan_step("전진", names), "move_forward")
        self.assertEqual(resolve_plan_step("왼쪽 이동", names), "move_left")
        self.assertEqual(resolve_plan_step("오른쪽 이동", names), "move_right")


class PrimitiveStateTests(unittest.TestCase):
    def test_chassis_motion_invalidates_robot_relative_state(self):
        estimator = StateEstimator()
        t = estimator.state.target
        t.visible = True
        t.confidence = .9
        t.cx = .5
        t.cy = .7
        t.centered = True
        t.range_class = "PREGRASP"
        estimator.state.spatial_memory = {"red": {"xyz": [0.1, 0.2, 0.0]}}
        estimator.state.semantic_map = {"reference_frame": "robot_base_at_observation"}
        estimator.update_tool_result({
            "result": {
                "skill": "turn_left",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN",
                "motion_executed": True,
                "chassis_motion": True,
            }
        })
        self.assertIsNone(estimator.state.target.visible)
        self.assertIsNone(estimator.state.target.centered)
        self.assertEqual(estimator.state.target.range_class, "UNKNOWN")
        self.assertEqual(estimator.state.spatial_memory, {})
        self.assertEqual(estimator.state.semantic_map, {})

    def test_stop_does_not_invalidate_stationary_state(self):
        estimator = StateEstimator()
        estimator.state.target.visible = True
        estimator.state.target.centered = True
        estimator.update_tool_result({"result": {
            "skill": "stop_motion",
            "outcome_status": "UNKNOWN",
            "motion_executed": True,
            "chassis_motion": False,
        }})
        self.assertIs(estimator.state.target.visible, True)
        self.assertIs(estimator.state.target.centered, True)


class PrimitiveDriverTests(unittest.TestCase):
    def test_all_motion_names_map_to_existing_masterpi_directions(self):
        self.assertEqual(set(PRIMITIVE.MOTIONS), {"forward", "backward", "left", "right", "rotate-left", "rotate-right", "stop"})

    def test_bounds_are_stricter_than_raw_masterpi_driver(self):
        PRIMITIVE.validate_motion("forward", 35, 0.10)
        PRIMITIVE.validate_motion("rotate-right", 40, 0.80)
        PRIMITIVE.validate_motion("left", 70, 0.65)
        PRIMITIVE.validate_motion("right", 70, 0.65)
        with self.assertRaises(ValueError):
            PRIMITIVE.validate_motion("forward", 30, 0.30)
        with self.assertRaises(ValueError):
            PRIMITIVE.validate_motion("forward", 41, 0.30)
        with self.assertRaises(ValueError):
            PRIMITIVE.validate_motion("forward", 35, 0.81)

    def test_driver_executes_exactly_one_bounded_motion(self):
        robot = mock.Mock()
        rc = PRIMITIVE.run_primitive(robot, "right", 35, 0.2)
        self.assertEqual(rc, 0)
        robot.drive.assert_called_once_with("right", 65, 0.65)

    def test_stop_never_calls_drive(self):
        robot = mock.Mock()
        rc = PRIMITIVE.run_primitive(robot, "stop", 0, 0.3)
        self.assertEqual(rc, 0)
        robot.stop.assert_called_once_with()
        robot.drive.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
