import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scripts.audit_camera_short_transport_student import audit
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from scripts.run_camera_short_transport_student import choose_carry_actions
from scripts.run_camera_varied_start_student import AXES, PHASES, choose_stage_actions

ROBOTS = ("r1", "r3")


def digest(data):
    return hashlib.sha256(data).hexdigest()


class ShortTransportAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.run, self.models, self.stages = root / "run", root / "models", root / "stages"
        (self.run / "rgb").mkdir(parents=True)
        self.models.mkdir()
        self.stages.mkdir()
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
        stage_records = {r: {} for r in ROBOTS}
        for rid in ROBOTS:
            for stage in AXES:
                data = json.dumps({"robot_id": rid, "stage": stage}).encode()
                path = self.stages / f"{rid}-{stage}.json"
                path.write_bytes(data)
                stage_records[rid][stage] = {"path": path.name, "sha256": digest(data)}
        (self.stages / "varied-start-skill.json").write_text(json.dumps({"models": stage_records}))

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

    def add_varied_approach(self):
        manifest_path = self.stages / "varied-start-skill.json"
        records = json.loads(manifest_path.read_text())["models"]
        self.report["approach_stage_skill_sha256"] = digest(manifest_path.read_bytes())
        self.report["approach_stage_model_sha256"] = {
            r: {s: records[r][s]["sha256"] for s in AXES} for r in ROBOTS}
        ready = {"ok": True, "ready": True, "stationary_ready": True,
                 "command": 0.0, "precision": "fine", "diagnostics": {"score": .5}}
        histories, calls, results = {r: [] for r in ROBOTS}, [], []
        frame = 10
        for phase_index, stage in enumerate(PHASES):
            confirmations = []
            for index in range(3):
                frame += 1
                top = self.image(f"approach-{phase_index}-{index}-top.jpg", f"atop-{frame}".encode())
                control = choose_stage_actions({r: ready for r in ROBOTS}, stage, index > 0)
                for rid in ROBOTS:
                    own = self.image(f"approach-{phase_index}-{index}-{rid}.jpg", f"aown-{rid}-{frame}".encode())
                    action = {"kind": "mecanum", **control["commands"][rid],
                              "duration_s": control["duration_s"]}
                    calls.append({"phase_index": phase_index, "stage": stage, "index": index,
                        "robot_id": rid, "frame_id": frame, "stationary": index > 0,
                        "images": {"own": own, "top": top}, "decision": ready,
                        "action": action, "own_command_history": list(histories[rid])})
                    histories[rid].append(action)
                if index > 0:
                    confirmations.append({"frame_ids": {r: frame for r in ROBOTS},
                        "ready": {r: True for r in ROBOTS}, "stationary": True})
            results.append({"phase_index": phase_index, "stage": stage, "ok": True,
                "confirmations": confirmations,
                "reason": "two consecutive fresh stationary RGB confirmations",
                "movement_slices": 0})
        final = []
        for index in range(2):
            frame += 1
            top = self.image(f"final-{index}-top.jpg", f"ftop-{index}".encode())
            images, decisions = {}, {}
            for rid in ROBOTS:
                images[rid] = {"own": self.image(f"final-{index}-{rid}.jpg", f"fown-{index}-{rid}".encode()),
                               "top": top}
                decisions[rid] = {stage: ready for stage in AXES}
            final.append({"frame_ids": {r: frame for r in ROBOTS},
                          "images": images, "decisions": decisions})
        self.report.update({"approach_calls": calls, "stage_results": results,
                            "final_alignment_checks": final, "approach_ok": True})
        self.save()
        return ready

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

    @patch("scripts.audit_camera_short_transport_student._predict")
    @patch("scripts.audit_camera_short_transport_student._predict_stage")
    def test_varied_approach_replays_actions_and_final_checks(self, predict_stage, predict):
        ready = self.add_varied_approach()
        predict.return_value = self.ready
        predict_stage.return_value = ready
        result = audit(self.run, self.models, self.stages)
        self.assertTrue(result["success"])
        self.assertEqual(result["approach"]["stage_count"], len(PHASES))
        self.assertEqual(result["approach"]["final_alignment_check_count"], 2)
        self.report["approach_calls"][0]["action"]["turn"] = .01
        self.save()
        self.assertIn("approach action replay mismatch",
                      audit(self.run, self.models, self.stages)["errors"][0])

    @patch("scripts.audit_camera_short_transport_student._predict")
    @patch("scripts.audit_camera_short_transport_student._predict_stage")
    def test_varied_approach_rejects_injected_history_and_final_decision(self, predict_stage, predict):
        ready = self.add_varied_approach()
        predict.return_value = self.ready
        predict_stage.return_value = ready
        self.report["approach_calls"][2]["own_command_history"] = []
        self.save()
        self.assertIn("command history mismatch",
                      audit(self.run, self.models, self.stages)["errors"][0])
        self.add_varied_approach()
        self.report["final_alignment_checks"][0]["decisions"]["r1"]["yaw"] = {
            **ready, "diagnostics": {"score": .6}}
        self.save()
        self.assertIn("final alignment decision replay mismatch",
                      audit(self.run, self.models, self.stages)["errors"][0])

    @patch("scripts.audit_camera_short_transport_student._predict_stage")
    def test_failed_approach_is_complete_audit_without_carry_evidence(self, predict_stage):
        self.add_varied_approach()
        unsupported = {"ok": False, "ready": False, "stationary_ready": False,
                       "command": 0.0, "precision": "fine",
                       "diagnostics": {"reason": "outside"}}
        predict_stage.return_value = unsupported
        self.report["approach_calls"] = self.report["approach_calls"][:2]
        for row in self.report["approach_calls"]:
            row["decision"] = unsupported
            row["action"] = {"kind": "mecanum", "forward": 0.0,
                             "left": 0.0, "turn": 0.0, "duration_s": .25}
        self.report["stage_results"] = [{"phase_index": 0, "stage": PHASES[0],
            "ok": False, "confirmations": [],
            "reason": "RGB outside learned stage support", "movement_slices": 0}]
        self.report.pop("final_alignment_checks")
        self.report["approach_ok"] = False
        self.report["carry_calls"] = []
        self.report["carry_ready"] = False
        self.report.pop("actor_initial_images")
        self.report.pop("actor_initial_issued_arm_commands")
        self.report["error"] = "RuntimeError: RGB approach did not qualify"
        self.save()
        result = audit(self.run, self.models, self.stages)
        self.assertTrue(result["success"])
        self.assertFalse(result["approach"]["approach_ok"])
        self.assertEqual(result["counts"]["carry_calls"], 0)


if __name__ == "__main__":
    unittest.main()
