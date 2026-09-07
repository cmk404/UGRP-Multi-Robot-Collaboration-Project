#!/usr/bin/env python3
import dataclasses
import importlib.util
import pathlib
import sys
import tempfile
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"


def load(name: str):
    path = PACKAGE / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    sys.path.insert(0, str(PACKAGE))
    spec.loader.exec_module(module)
    return module


HANDOFF = load("precision_handoff")


@dataclasses.dataclass(frozen=True)
class Block:
    radius_cm: float
    lateral_left_cm: float
    forward_cm: float
    yaw_left_deg: float
    camera_radius_cm: float
    camera_height_cm: float
    ray_pitch_deg: float
    block_height_cm: float
    nx: float
    ny: float


@dataclasses.dataclass(frozen=True)
class Plan:
    block: Block
    base_pulse: int
    hover_pose: dict[int, int]
    grasp_pose: dict[int, int]
    fingertip_radius_cm: float


class PrecisionHandoffTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = pathlib.Path(self.tmp.name) / "plan.json"
        self.patch = mock.patch.object(HANDOFF, "PLAN_PATH", self.path)
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        self.tmp.cleanup()

    def plan(self):
        return Plan(
            block=Block(16.2, .3, 16.1, 1.0, 10, 8, -30, 3.0, .5, .5),
            base_pulse=1510,
            hover_pose={3: 700, 4: 2200, 5: 1300},
            grasp_pose={3: 800, 4: 2300, 5: 1800},
            fingertip_radius_cm=16.7,
        )

    def test_save_load_and_rebuild_preserves_metric_plan(self):
        plan = self.plan()
        HANDOFF.save_pick_plan(plan, robot_pose={3: 900,4:2200,5:1300,6:1500}, target_visible=True)
        payload_time = __import__("json").loads(self.path.read_text())["created_at"]
        payload = HANDOFF.load_pick_plan_payload(now=payload_time)
        precision = types.SimpleNamespace(BlockEstimate=Block, PickPlan=Plan)
        rebuilt = HANDOFF.rebuild_pick_plan(precision, payload)
        self.assertEqual(rebuilt, plan)
        HANDOFF.validate_measurement_pose({3:900,4:2200,5:1300,6:1500}, payload)

    def test_stale_plan_is_rejected_and_deleted(self):
        plan = self.plan()
        HANDOFF.save_pick_plan(plan, robot_pose={3:900,4:2200,5:1300,6:1500}, target_visible=True)
        created = __import__('json').loads(self.path.read_text())["created_at"]
        with self.assertRaises(RuntimeError):
            HANDOFF.load_pick_plan_payload(now=created + HANDOFF.MAX_PLAN_AGE_SECONDS + 1)
        self.assertFalse(self.path.exists())

    def test_changed_camera_pose_invalidates_plan(self):
        plan = self.plan()
        HANDOFF.save_pick_plan(plan, robot_pose={3:900,4:2200,5:1300,6:1500}, target_visible=True)
        payload = HANDOFF.load_pick_plan_payload()
        with self.assertRaises(RuntimeError):
            HANDOFF.validate_measurement_pose({3:900,4:2200,5:1300,6:1550}, payload)
        self.assertFalse(self.path.exists())

    def test_v2_coarse_handoff_roundtrip_and_pose_validation(self):
        import json
        pose = {3: 980, 4: 2320, 5: 1320, 6: 1500}
        HANDOFF.save_coarse_handoff(
            robot_pose=pose,
            target_visible=True,
            target_color="red",
            gaze_pan=1500,
            gaze_tilt=980,
            coarse_radius_cm=26.0,
        )
        created = json.loads(self.path.read_text())["created_at"]
        payload = HANDOFF.load_coarse_handoff_payload(now=created)
        self.assertEqual(payload["version"], HANDOFF.COARSE_HANDOFF_VERSION)
        self.assertEqual(payload["kind"], HANDOFF.COARSE_HANDOFF_KIND)
        self.assertEqual(payload["target_color"], "red")
        self.assertAlmostEqual(payload["coarse_radius_cm"], 26.0)
        HANDOFF.require_coarse_handoff_color(payload, "red")
        HANDOFF.validate_coarse_handoff_pose(pose, payload)

    def test_v2_coarse_handoff_stale_is_rejected_and_deleted(self):
        import json
        pose = {3: 980, 4: 2320, 5: 1320, 6: 1500}
        HANDOFF.save_coarse_handoff(
            robot_pose=pose,
            target_visible=True,
            target_color="red",
            gaze_pan=1500,
            gaze_tilt=980,
            coarse_radius_cm=26.0,
        )
        created = json.loads(self.path.read_text())["created_at"]
        with self.assertRaises(RuntimeError):
            HANDOFF.load_coarse_handoff_payload(
                now=created + HANDOFF.MAX_PLAN_AGE_SECONDS + 1
            )
        self.assertFalse(self.path.exists())

    def test_v2_coarse_handoff_pose_change_invalidates(self):
        pose = {3: 980, 4: 2320, 5: 1320, 6: 1500}
        HANDOFF.save_coarse_handoff(
            robot_pose=pose,
            target_visible=True,
            target_color="red",
            gaze_pan=1500,
            gaze_tilt=980,
            coarse_radius_cm=26.0,
        )
        payload = HANDOFF.load_coarse_handoff_payload()
        changed = dict(pose)
        changed[6] = 1540
        with self.assertRaises(RuntimeError):
            HANDOFF.validate_coarse_handoff_pose(changed, payload)
        self.assertFalse(self.path.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
