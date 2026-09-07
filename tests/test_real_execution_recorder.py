from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RED_DIR = ROOT / "scripts" / "red_block"
if str(RED_DIR) not in sys.path:
    sys.path.insert(0, str(RED_DIR))


class RealExecutionRecorderTests(unittest.TestCase):
    def setUp(self):
        self.old_env = {k: os.environ.get(k) for k in (
            "UGRP_REAL_TRACE_ENABLE",
            "UGRP_REAL_TRACE_DIR",
            "UGRP_REAL_TRACE_SKILL",
            "UGRP_REAL_TRACE_FRAME_INTERVAL_S",
            "UGRP_REAL_TRACE_ALL_FRAMES",
            "UGRP_REAL_RUN_ID",
            "UGRP_REAL_TRACE_SPAN_ID",
            "UGRP_ROBOT_POSE_STATE",
        )}

    def tearDown(self):
        for key, value in self.old_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def _reload_recorder(self):
        import recorder
        return importlib.reload(recorder)

    def test_disabled_recorder_is_noop(self):
        with tempfile.TemporaryDirectory() as td:
            os.environ.pop("UGRP_REAL_TRACE_ENABLE", None)
            os.environ["UGRP_REAL_TRACE_DIR"] = td
            rec = self._reload_recorder()
            rec.event("should_not_exist", value=1)
            self.assertFalse((Path(td) / "events.jsonl").exists())

    def test_records_frame_detection_pose_and_commands_as_jsonl(self):
        with tempfile.TemporaryDirectory() as td:
            os.environ["UGRP_REAL_TRACE_ENABLE"] = "1"
            os.environ["UGRP_REAL_TRACE_DIR"] = td
            os.environ["UGRP_REAL_TRACE_SKILL"] = "search"
            os.environ["UGRP_REAL_TRACE_FRAME_INTERVAL_S"] = "999"
            os.environ["UGRP_REAL_TRACE_ALL_FRAMES"] = "1"
            os.environ["UGRP_REAL_RUN_ID"] = "run-unit"
            os.environ["UGRP_REAL_TRACE_SPAN_ID"] = "span-unit"
            pose_path = Path(td) / "pose.json"
            pose_path.write_text(json.dumps({"pose": {"3": 700, "6": 1500}, "updated_at": __import__("time").time()}))
            os.environ["UGRP_ROBOT_POSE_STATE"] = str(pose_path)
            rec = self._reload_recorder()

            frame = np.zeros((48, 64, 3), dtype=np.uint8)
            seq = rec.record_camera_frame(frame, source="unit-test")
            blob = SimpleNamespace(
                cx=32, cy=24, area=1200, width=20, height=20,
                nx=0.5, ny=0.5, box_points=((0.4, 0.4),), rectangularity=0.9,
            )
            rec.record_detection(frame, color="red", blob=blob, detector="unit", min_area=500, crop_left=0)
            rec.record_pose({3: 700, 6: 1500}, reason="unit")
            rec.event("chassis_command", direction="forward", speed=35, duration_s=0.1, wheel_speeds=[(1, -35), (2, 35)])

            events = [json.loads(line) for line in (Path(td) / "events.jsonl").read_text().splitlines()]
            kinds = [item["kind"] for item in events]
            self.assertEqual(seq, 1)
            self.assertIn("camera_frame", kinds)
            self.assertIn("detection", kinds)
            self.assertIn("commanded_pose", kinds)
            self.assertIn("chassis_command", kinds)
            det = next(item for item in events if item["kind"] == "detection")
            self.assertEqual(det["frame_seq"], 1)
            self.assertTrue(det["visible"])
            self.assertEqual(det["blob"]["area"], 1200)
            self.assertEqual(det["skill"], "search")
            camera_event = next(item for item in events if item["kind"] == "camera_frame")
            sampled = camera_event["sampled_jpeg"]
            self.assertTrue(sampled)
            self.assertTrue((Path(td) / sampled).is_file())
            self.assertEqual(camera_event["run_id"], "run-unit")
            self.assertEqual(camera_event["span_id"], "span-unit")
            self.assertEqual(camera_event["commanded_pose"], {"3": 700, "6": 1500})
            self.assertIsNotNone(camera_event["pose_state_age_s"])
            self.assertEqual(sorted(item["event_seq"] for item in events), list(range(1, len(events) + 1)))

    def test_all_frames_mode_preserves_each_consumed_frame(self):
        with tempfile.TemporaryDirectory() as td:
            os.environ["UGRP_REAL_TRACE_ENABLE"] = "1"
            os.environ["UGRP_REAL_TRACE_DIR"] = td
            os.environ["UGRP_REAL_TRACE_ALL_FRAMES"] = "1"
            rec = self._reload_recorder()
            frame = np.zeros((24, 32, 3), dtype=np.uint8)
            rec.record_camera_frame(frame.copy(), source="a")
            rec.record_camera_frame(frame.copy(), source="b")
            events = [json.loads(line) for line in (Path(td) / "events.jsonl").read_text().splitlines()]
            camera_events = [e for e in events if e["kind"] == "camera_frame"]
            self.assertEqual(len(camera_events), 2)
            self.assertTrue(all(e["sampled_jpeg"] for e in camera_events))
            self.assertTrue(all(e["jpeg_save_ms"] >= 0 for e in camera_events))
            self.assertTrue(all(e["pose_read_ms"] >= 0 for e in camera_events))
            rec._write_recorder_summary()
            events = [json.loads(line) for line in (Path(td) / "events.jsonl").read_text().splitlines()]
            summary = next(e for e in events if e["kind"] == "recorder_summary")
            self.assertEqual(summary["frames_seen"], 2)
            self.assertEqual(summary["frames_saved"], 2)
            self.assertGreaterEqual(summary["frame_record_avg_ms"], 0)

    def test_package_contains_recorder(self):
        import deploy
        names = {p.name for p in deploy.package_files()}
        self.assertIn("recorder.py", names)

    def test_robot_action_result_exposes_trace_reference(self):
        scripts_dir = ROOT / "scripts"
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        from scripts import robot_actions

        fake_dir = str(ROOT / "outputs" / "real_traces" / "fake-trace")
        def fake_main(_argv):
            robot_actions.LAST_REAL_TRACE_PATH.write_text(
                json.dumps({"trace_id": "fake-trace", "trace_dir": fake_dir}),
                encoding="utf-8",
            )
            return 0

        with mock.patch.object(robot_actions, "main", side_effect=fake_main):
            result = robot_actions.run("search")
        self.assertEqual(result["real_trace_id"], "fake-trace")
        self.assertEqual(result["real_trace_dir"], fake_dir)
        try:
            robot_actions.LAST_REAL_TRACE_PATH.unlink()
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    unittest.main()
