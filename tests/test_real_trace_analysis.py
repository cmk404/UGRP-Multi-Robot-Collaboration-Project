from __future__ import annotations

import json
import tempfile
import os
import unittest
from pathlib import Path

from harness.trace_analysis import analyze_run
from harness.execution_trace import activate_run, append_event, create_run, finish_run, trace_asset, trace_detail, trace_index
from harness.loop import LoopResult


class RealExecutionTraceTests(unittest.TestCase):
    def test_run_context_persists_initial_image_pose_and_finish_analysis(self):
        with tempfile.TemporaryDirectory() as td:
            old = os.environ.get("UGRP_REAL_TRACE_ROOT")
            os.environ["UGRP_REAL_TRACE_ROOT"] = td
            try:
                image = Path(td) / "input.jpg"
                image.write_bytes(b"fake-jpeg")
                run_id, run_dir = create_run(
                    message="빨간 블럭 찾아봐", execute=True, initial_world_state={"target": {"visible": True}},
                    initial_image=image, initial_commanded_pose={3: 500, 6: 1500},
                    initial_pose_age_s=.2, initial_pose_stable=True,
                )
                with activate_run(run_id, run_dir):
                    append_event("unit_event", value=7)
                    summary = finish_run(LoopResult(final="찾았어", stopped="final"))
                self.assertEqual(summary["status"], "COMPLETED")
                meta = json.loads((run_dir / "run.json").read_text())
                self.assertEqual(meta["initial_commanded_pose"], {"3": 500, "6": 1500})
                events = [json.loads(line) for line in (run_dir / "events.jsonl").read_text().splitlines()]
                frame = next(e for e in events if e["kind"] == "harness_frame")
                self.assertEqual(frame["commanded_pose"], {"3": 500, "6": 1500})
                self.assertTrue((run_dir / frame["image_path"]).is_file())
                self.assertTrue((run_dir / "analysis.json").is_file())
            finally:
                if old is None:
                    os.environ.pop("UGRP_REAL_TRACE_ROOT", None)
                else:
                    os.environ["UGRP_REAL_TRACE_ROOT"] = old

    def test_trace_index_detail_and_asset_are_read_only_and_path_safe(self):
        with tempfile.TemporaryDirectory() as td:
            old = os.environ.get("UGRP_REAL_TRACE_ROOT")
            os.environ["UGRP_REAL_TRACE_ROOT"] = td
            try:
                image = Path(td) / "source.jpg"
                image.write_bytes(b"jpeg-evidence")
                run_id, run_dir = create_run(message="test command", execute=True, initial_image=image)
                with activate_run(run_id, run_dir):
                    finish_run(LoopResult(final="ok", stopped="final"))
                index = trace_index()
                self.assertEqual(index["run_count"], 1)
                self.assertEqual(index["runs"][0]["run_id"], run_id)
                self.assertEqual(index["runs"][0]["status"], "COMPLETED")
                detail = trace_detail(run_id)
                self.assertIsNotNone(detail)
                self.assertEqual(detail["analysis"]["status"], "COMPLETED")
                rel = next((run_dir / "harness_frames").iterdir()).relative_to(run_dir)
                self.assertEqual(trace_asset(run_id, str(rel)), run_dir / rel)
                self.assertIsNone(trace_asset(run_id, "../outside.jpg"))
                self.assertIsNone(trace_detail("../escape"))
            finally:
                if old is None:
                    os.environ.pop("UGRP_REAL_TRACE_ROOT", None)
                else:
                    os.environ["UGRP_REAL_TRACE_ROOT"] = old


class RealTraceAnalysisTests(unittest.TestCase):
    def test_failure_report_links_pose_detection_command_and_frames(self):
        with tempfile.TemporaryDirectory() as td:
            run = Path(td) / "run-1"
            span = run / "skills" / "span-1"
            frames = span / "frames"
            frames.mkdir(parents=True)
            (run / "run.json").write_text(json.dumps({"run_id": "run-1", "message": "빨간 블럭 집어봐"}))
            (frames / "frame-000001.jpg").write_bytes(b"jpeg1")
            (frames / "frame-000002.jpg").write_bytes(b"jpeg2")
            events = [
                {"kind": "camera_frame", "wall_time_s": 1.0, "frame_seq": 1, "sampled_jpeg": "frames/frame-000001.jpg", "commanded_pose": {"3": 500}},
                {"kind": "detection", "wall_time_s": 1.1, "frame_seq": 1, "color": "red", "visible": True, "detector": "red_lab+hsv", "blob": {"nx": .5, "ny": .7, "area": 9000}},
                {"kind": "servo_batch_command", "wall_time_s": 1.2, "updates": {"3": 600, "4": 2200, "5": 1900}, "duration_s": .65},
                {"kind": "commanded_pose", "wall_time_s": 1.3, "pose": {"3": 600, "4": 2200, "5": 1900}, "reason": "nudge_servos"},
                {"kind": "camera_frame", "wall_time_s": 1.4, "frame_seq": 2, "sampled_jpeg": "frames/frame-000002.jpg", "commanded_pose": {"3": 600, "4": 2200, "5": 1900}},
                {"kind": "detection", "wall_time_s": 1.5, "frame_seq": 2, "color": "red", "visible": False, "detector": "red_lab+hsv", "blob": None},
                {"kind": "recorder_summary", "wall_time_s": 1.6, "frames_seen": 2, "frames_saved": 2, "frame_record_avg_ms": 4.2, "frame_record_max_ms": 5.1, "jpeg_save_avg_ms": 3.8, "pose_read_avg_ms": 0.1, "event_write_avg_ms": 0.2},
            ]
            (span / "events.jsonl").write_text("\n".join(json.dumps(e) for e in events) + "\n")
            (span / "stderr.log").write_text("error: red block not visible after entering fixed capture pose\n")
            (span / "stdout.log").write_text("precision approach: choose face-normal base placement before grasp\n")
            (span / "result.json").write_text(json.dumps({
                "run_id": "run-1", "span_id": "span-1", "script": "approach.py", "skill": "approach",
                "started_wall_s": .5, "ended_wall_s": 2.0, "duration_s": 1.5,
                "execution_started_wall_s": .7, "execution_ended_wall_s": 1.8, "execution_duration_s": 1.1,
                "trace_copy_duration_s": .12, "exit_code": 1,
            }))
            summary = analyze_run(run, write=True)
            self.assertEqual(summary["status"], "FAILED")
            self.assertEqual(summary["failure"]["skill"], "approach")
            self.assertIn("fixed capture pose", summary["failure"]["reason"])
            self.assertEqual(summary["evidence"]["last_commanded_pose"]["pose"]["3"], 600)
            self.assertFalse(summary["evidence"]["last_detection"]["visible"])
            self.assertTrue(summary["evidence"]["last_input_frame"].endswith("frame-000002.jpg"))
            self.assertEqual(summary["root_cause"]["category"], "TARGET_LOST_AFTER_ARM_POSE_CHANGE")
            transition = summary["evidence"]["visibility_transition"]
            self.assertTrue(any(e.get("kind") == "servo_batch_command" for e in transition["commands_between"]))
            self.assertEqual(summary["failure_boundary"]["first_failed_skill"], "approach")
            self.assertEqual(summary["observability"]["recorder"][0]["frame_record_avg_ms"], 4.2)
            self.assertEqual(summary["observability"]["trace_copy_total_s"], 0.12)
            self.assertEqual(summary["observability"]["skill_execution_total_s"], 1.1)
            self.assertTrue((run / "analysis.json").is_file())
            self.assertTrue((run / "analysis.md").is_file())


if __name__ == "__main__":
    unittest.main()
