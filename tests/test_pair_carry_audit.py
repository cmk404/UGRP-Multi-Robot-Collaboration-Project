import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from harness.pair_carry_policy import PairCarryPolicy, ROBOTS
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_short_transport_student import choose_carry_actions
from scripts.audit_pair_carry_sync import audit


def sha(data):
    return hashlib.sha256(data).hexdigest()


class PairCarryAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.run, self.transport, self.grasp = root / "run", root / "transport", root / "grasp"
        (self.run / "rgb").mkdir(parents=True)
        self.transport.mkdir()
        self.grasp.mkdir()
        records = {}
        for rid in ROBOTS:
            raw = json.dumps({"schema": "ugrp.camera_short_transport_model.v1", "robot_id": rid}).encode()
            (self.transport / f"{rid}.json").write_bytes(raw)
            records[rid] = {"path": f"{rid}.json", "sha256": sha(raw)}
        manifest = json.dumps({"schema": "ugrp.camera_short_transport_skill.v1", "models": records}).encode()
        (self.transport / "short-transport-skill.json").write_bytes(manifest)
        self.grasp_skill = b'{"schema":"test-grasp-skill"}'
        (self.grasp / "student-skill.json").write_bytes(self.grasp_skill)
        (self.run / "grasp-result.json").write_text("{}")
        self.ready = {"ok": True, "held_estimate": True, "ready": True,
                      "forward": 0.0, "diagnostics": {"progress_m": .2}}
        self.report = self._make_report(manifest, records)
        self._save()
        self.predict_patch = patch("scripts.audit_pair_carry_sync.predict_transport", return_value=self.ready)
        self.skew_patch = patch("scripts.audit_pair_carry_sync.payload_skew", return_value=0.0)
        self.grasp_patch = patch("scripts.audit_pair_carry_sync.audit_grasp",
                                 return_value={"ok": True, "calls": 4})
        self.predict_patch.start()
        self.skew_patch.start()
        self.grasp_patch.start()

    def tearDown(self):
        patch.stopall()
        self.temp.cleanup()

    def _image(self, name, data):
        path = self.run / "rgb" / name
        path.write_bytes(data)
        return {"path": f"rgb/{name}", "sha256": sha(data)}

    def _make_report(self, manifest, records, condition="sync"):
        anchor_top = self._image("anchor-top.jpg", b"top-anchor")
        anchors = {rid: {"own": self._image(f"anchor-{rid}.jpg", f"anchor-{rid}".encode()),
                         "top": anchor_top} for rid in ROBOTS}
        histories = {rid: [] for rid in ROBOTS}
        policy = PairCarryPolicy("case-1")
        confirming, consecutive = False, 0
        steps = []
        for index in range(3):
            now = .1 * index
            images = json.loads(json.dumps(anchors)) if index == 0 else {
                rid: {"own": self._image(f"{index}-{rid}.jpg", f"own-{index}-{rid}".encode()),
                      "top": self._image(f"{index}-top.jpg", f"top-{index}".encode())}
                for rid in ROBOTS}
            decisions = {rid: self.ready for rid in ROBOTS}
            frame_ids = {rid: f"{rid}-frame-{index}" for rid in ROBOTS}
            if condition == "sync":
                control = policy.step(decisions, 0.0, frame_ids, now, list(ROBOTS))
            else:
                base = choose_carry_actions(decisions, confirming)
                control = {**base, "mode": "CONFIRM" if confirming else "CRUISE",
                           "abort": not base["valid"], "done": False, "permission": None,
                           "skew_error_px": None, "recovery_count": 0}
                if confirming:
                    consecutive = consecutive + 1 if base["ready"] else 0
                    control["done"] = consecutive >= 2
                    if not base["ready"]:
                        confirming = False
                elif base["ready"]:
                    confirming = True
            actions = {rid: {"kind": "drive", "forward": control["forwards"][rid], "turn": 0.0,
                             "duration_s": control["duration_s"]} for rid in ROBOTS}
            steps.append({"index": index, "observed_at_s": now, "request_wall_s": now,
                          "decision_wall_s": now + .01, "frame_ids": frame_ids, "images": images,
                          "own_command_histories": {rid: list(histories[rid]) for rid in ROBOTS},
                          "decisions": decisions, "skew_px": 0.0, "delivered_reports": list(ROBOTS),
                          "control": control, "actions": actions, "command_issued_wall_s": now + .02,
                          "execution_end_s": now + .10, "execution_end_wall_s": now + .03})
            for rid in ROBOTS:
                histories[rid].append(actions[rid])
        sample = {"sim_time_s": 0.0, "phase": "carry", "position_m": [0, 0, 0],
                  "height_above_start_m": 0.0, "constraints_active": {rid: False for rid in ROBOTS}}
        (self.run / "evaluation-only.jsonl").write_text(json.dumps(sample) + "\n")
        geometry = {key: sha(key.encode()) for key in
                    ("body_inertia", "body_mass", "geom_friction", "geom_pos",
                     "geom_quat", "geom_rgba", "geom_size")}
        camera = {"fov_y_deg": 50.0, "position": [0.0, 0.0, 1.0],
                  "quaternion": [1.0, 0.0, 0.0, 0.0]}
        invariants = {"geometry_sha256": geometry,
                      "policy_cameras": {name: dict(camera) for name in
                                         ("cctv_top", "r1__robot_cam", "r3__robot_cam")}}
        return {"schema": "ugrp.pair_carry_sync.v1", "case_id": "case-1", "condition": condition,
                "grasp_skill_sha256": sha(self.grasp_skill),
                "transport_skill_sha256": sha(manifest),
                "transport_model_sha256": {rid: records[rid]["sha256"] for rid in ROBOTS},
                "anchor_images": anchors, "steps": steps, "carry_ready": True,
                "sync_events": policy.sync.events if condition == "sync" else [],
                "policy_events": policy.events if condition == "sync" else [],
                "evaluation": evaluate_transport_samples([sample]), "weld_active_ticks": 0,
                "invariants_initial": invariants,
                "invariants_final": json.loads(json.dumps(invariants)),
                "error": None, "success": False}

    def _save(self):
        (self.run / "result.json").write_text(json.dumps(self.report))

    def test_replays_saved_inputs_and_separates_audit_from_physical_success(self):
        result = audit(self.run, self.transport, self.grasp)
        self.assertTrue(result["success"])
        self.assertFalse(result["physical_success"])
        self.assertFalse(result["evaluation_success"])
        self.assertEqual({"steps": 3, "actions": 6, "evaluation_samples": 1, "grasp_calls": 4},
                         result["counts"])

    def test_replays_independent_baseline_branch(self):
        manifest = (self.transport / "short-transport-skill.json").read_bytes()
        records = json.loads(manifest)["models"]
        self.report = self._make_report(manifest, records, condition="baseline")
        self._save()
        result = audit(self.run, self.transport, self.grasp)
        self.assertTrue(result["success"])
        self.assertEqual("baseline", result["actor"]["condition"])

    def test_rejects_valid_looking_control_and_action_tamper(self):
        self.report["steps"][0]["control"]["forwards"]["r1"] = .01
        self.report["steps"][0]["actions"]["r1"]["forward"] = .01
        self._save()
        result = audit(self.run, self.transport, self.grasp)
        self.assertFalse(result["success"])
        self.assertIn("control replay mismatch", result["errors"][0])

    def test_rejects_hidden_input_field_and_mutated_rgb(self):
        self.report["steps"][0]["images"]["r1"]["own"]["world_pose"] = [0, 0, 0]
        self._save()
        self.assertIn("hidden input fields", audit(self.run, self.transport, self.grasp)["errors"][0])
        del self.report["steps"][0]["images"]["r1"]["own"]["world_pose"]
        path = self.run / self.report["steps"][1]["images"]["r1"]["own"]["path"]
        path.write_bytes(b"mutated-jpeg")
        self._save()
        self.assertIn("RGB hash mismatch", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_history_injection(self):
        self.report["steps"][1]["own_command_histories"]["r1"] = []
        self._save()
        self.assertIn("own command history mismatch", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_skipped_row_duplicate_frame_and_nonmonotonic_time(self):
        self.report["steps"].pop(1)
        self._save()
        self.assertIn("time/index", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report = self._make_report((self.transport / "short-transport-skill.json").read_bytes(),
            json.loads((self.transport / "short-transport-skill.json").read_text())["models"])
        self.report["steps"][1]["frame_ids"]["r1"] = self.report["steps"][0]["frame_ids"]["r1"]
        self._save()
        self.assertIn("frame is duplicate", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report = self._make_report((self.transport / "short-transport-skill.json").read_bytes(),
            json.loads((self.transport / "short-transport-skill.json").read_text())["models"])
        self.report["steps"][1]["observed_at_s"] = self.report["steps"][0]["observed_at_s"]
        self._save()
        self.assertIn("strictly monotonic", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_model_and_evaluation_tamper(self):
        self.report["transport_model_sha256"]["r1"] = "0" * 64
        self._save()
        self.assertIn("model hash mismatch", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report["transport_model_sha256"]["r1"] = json.loads(
            (self.transport / "short-transport-skill.json").read_text())["models"]["r1"]["sha256"]
        self.report["evaluation"]["success"] = True
        self._save()
        self.assertIn("physical evaluation differs", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_failed_grasp_and_grasp_skill_tamper(self):
        with patch("scripts.audit_pair_carry_sync.audit_grasp", return_value={"ok": False, "calls": 4}):
            self.assertIn("grasp audit did not pass", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report["grasp_skill_sha256"] = "0" * 64
        self._save()
        self.assertIn("grasp skill hash mismatch", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_weld_invariant_and_saved_success_tamper(self):
        self.report["evaluation"] = {"success": True}
        self.report["success"] = True
        self.report["weld_active_ticks"] = 1
        self._save()
        with patch("scripts.audit_pair_carry_sync.evaluate_transport_samples", return_value={"success": True}):
            self.assertIn("saved physical success differs", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report["weld_active_ticks"] = 0
        self.report["invariants_final"]["geometry_sha256"]["geom_pos"] = "f" * 64
        self._save()
        with patch("scripts.audit_pair_carry_sync.evaluate_transport_samples", return_value={"success": True}):
            self.assertIn("saved physical success differs", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report["invariants_final"] = json.loads(json.dumps(self.report["invariants_initial"]))
        self.report["success"] = False
        self._save()
        with patch("scripts.audit_pair_carry_sync.evaluate_transport_samples", return_value={"success": True}):
            self.assertIn("saved physical success differs", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_missing_or_malformed_invariant_metadata(self):
        del self.report["invariants_initial"]["policy_cameras"]["r3__robot_cam"]
        self._save()
        self.assertIn("camera coverage mismatch", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_anchor_and_execution_timeline_tamper(self):
        self.report["steps"][0]["images"]["r1"] = self.report["steps"][1]["images"]["r1"]
        self._save()
        self.assertIn("first step images differ", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report = self._make_report((self.transport / "short-transport-skill.json").read_bytes(),
            json.loads((self.transport / "short-transport-skill.json").read_text())["models"])
        self.report["steps"][0]["execution_end_s"] += .01
        self._save()
        self.assertIn("previous execution end", audit(self.run, self.transport, self.grasp)["errors"][0])

    def test_rejects_delivered_report_and_sync_event_tamper(self):
        self.report["steps"][1]["delivered_reports"] = ["r1"]
        self._save()
        self.assertIn("replay mismatch", audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report = self._make_report((self.transport / "short-transport-skill.json").read_bytes(),
            json.loads((self.transport / "short-transport-skill.json").read_text())["models"])
        self.report["sync_events"][0]["reason"] = "tampered"
        self._save()
        self.assertIn("synchronization event replay mismatch",
                      audit(self.run, self.transport, self.grasp)["errors"][0])
        self.report = self._make_report((self.transport / "short-transport-skill.json").read_bytes(),
            json.loads((self.transport / "short-transport-skill.json").read_text())["models"])
        self.report["policy_events"].append({"event": "SKEW_HOLD", "time_s": .2,
                                              "skew_error_px": 4.0})
        self._save()
        self.assertIn("policy event replay mismatch",
                      audit(self.run, self.transport, self.grasp)["errors"][0])


if __name__ == "__main__":
    unittest.main()
