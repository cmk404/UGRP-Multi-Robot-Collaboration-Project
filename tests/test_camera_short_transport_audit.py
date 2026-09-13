import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.audit_camera_short_transport_student import audit
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_short_transport_student import choose_carry_actions

ROBOTS = ("r1", "r3")


def digest(data):
    return hashlib.sha256(data).hexdigest()


class ShortTransportAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.run, self.models = root / "run", root / "models"
        (self.run / "rgb").mkdir(parents=True)
        self.models.mkdir()
        records = {}
        for rid in ROBOTS:
            data = json.dumps({"schema": "ugrp.camera_short_transport_model.v1",
                               "robot_id": rid}).encode()
            path = self.models / f"{rid}.json"
            path.write_bytes(data)
            records[rid] = {"path": path.name, "sha256": digest(data)}
        manifest = json.dumps({"schema": "ugrp.camera_short_transport_skill.v1",
                               "models": records}).encode()
        (self.models / "short-transport-skill.json").write_bytes(manifest)
        self.ready = {"ok": True, "held_estimate": True, "ready": True,
                      "forward": 0.0, "diagnostics": {"progress_m": .2}}
        self.report = self.make_report(manifest, records)

    def tearDown(self):
        self.temp.cleanup()

    def image(self, name, data):
        path = self.run / "rgb" / name
        path.write_bytes(data)
        return {"path": f"rgb/{name}", "sha256": digest(data)}

    def make_report(self, manifest, records):
        anchors = {}
        for rid in ROBOTS:
            anchors[rid] = {"own": self.image(f"anchor-{rid}.jpg", f"own-{rid}".encode()),
                            "top": self.image("anchor-top.jpg", b"shared-anchor-top")}
        histories, calls = {r: [] for r in ROBOTS}, []
        confirming = False
        for index in range(3):
            decisions = {r: self.ready for r in ROBOTS}
            control = choose_carry_actions(decisions, confirming)
            for rid in ROBOTS:
                images = anchors[rid] if index == 0 else {
                    "own": self.image(f"{index}-{rid}.jpg", f"own-{index}-{rid}".encode()),
                    "top": self.image(f"{index}-top.jpg", f"top-{index}".encode())}
                action = {"kind": "drive", "forward": control["forwards"][rid],
                          "turn": 0.0, "duration_s": control["duration_s"]}
                calls.append({"index": index, "robot_id": rid, "frame_id": index + 1,
                    "stationary": confirming, "images": images,
                    "own_command_history": list(histories[rid]),
                    "decision": self.ready, "action": action})
                histories[rid].append(action)
            confirming = True
        sample = {"sim_time_s": 0.0, "phase": "carry", "position_m": [0, 0, 0],
                  "height_above_start_m": 0.0,
                  "constraints_active": {r: False for r in ROBOTS}}
        (self.run / "evaluation-only.jsonl").write_text(json.dumps(sample) + "\n")
        evaluation = evaluate_transport_samples([sample])
        report = {"schema": "ugrp.short_transport_student.v1",
            "config": {"condition": "visual", "maximum_carry_slices": 100,
                       "stationary_confirmations": 2, "weld": False},
            "transport_skill_sha256": digest(manifest),
            "transport_model_sha256": {r: records[r]["sha256"] for r in ROBOTS},
            "actor_initial_images": anchors,
            "actor_initial_issued_arm_commands": {r: {"1": 1500} for r in ROBOTS},
            "carry_calls": calls, "carry_ready": True, "approach_ok": True,
            "evaluation": evaluation, "weld_active_ticks": 0,
            "approach_payload_contact_steps": 0,
            "invariants_initial": {"geometry_sha256": {"x": "same"}},
            "invariants_final": {"geometry_sha256": {"x": "same"}},
            "error": None, "success": False}
        (self.run / "result.json").write_text(json.dumps(report))
        return report

    def save(self):
        (self.run / "result.json").write_text(json.dumps(self.report))

    @patch("scripts.audit_camera_short_transport_student._predict")
    def test_rgb_audit_can_pass_when_physics_experiment_fails(self, predict):
        predict.return_value = self.ready
        result = audit(self.run, self.models)
        self.assertTrue(result["success"])
        self.assertTrue(result["rgb_action_audit_passed"])
        self.assertFalse(result["experiment_success"])
        self.assertEqual(result["counts"]["carry_calls"], 6)

    @patch("scripts.audit_camera_short_transport_student._predict")
    def test_rejects_rgb_hash_tamper(self, predict):
        predict.return_value = self.ready
        (self.run / self.report["carry_calls"][2]["images"]["own"]["path"]).write_bytes(b"tampered")
        result = audit(self.run, self.models)
        self.assertFalse(result["rgb_action_audit_passed"])
        self.assertIn("RGB hash mismatch", result["errors"][0])

    @patch("scripts.audit_camera_short_transport_student._predict")
    def test_rejects_history_injection_and_action_change(self, predict):
        predict.return_value = self.ready
        self.report["carry_calls"][2]["own_command_history"] = []
        self.save()
        self.assertIn("history mismatch", audit(self.run, self.models)["errors"][0])
        self.report = self.make_report(
            (self.models / "short-transport-skill.json").read_bytes(),
            json.loads((self.models / "short-transport-skill.json").read_text())["models"])
        self.report["carry_calls"][0]["action"]["forward"] = .01
        self.save()
        self.assertIn("action replay mismatch", audit(self.run, self.models)["errors"][0])

    @patch("scripts.audit_camera_short_transport_student._predict")
    def test_rejects_path_traversal_and_symlink(self, predict):
        predict.return_value = self.ready
        self.report["carry_calls"][0]["images"]["own"]["path"] = "../outside.jpg"
        self.save()
        self.assertIn("traversal", audit(self.run, self.models)["errors"][0])
        outside = Path(self.temp.name) / "outside.jpg"
        outside.write_bytes(b"own-r1")
        link = self.run / "rgb" / "link.jpg"
        link.symlink_to(outside)
        self.report["carry_calls"][0]["images"]["own"] = {
            "path": "rgb/link.jpg", "sha256": digest(outside.read_bytes())}
        self.save()
        self.assertIn("symlink", audit(self.run, self.models)["errors"][0])


if __name__ == "__main__":
    unittest.main()
