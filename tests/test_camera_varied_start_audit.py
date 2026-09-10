import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from scripts.audit_camera_varied_start_student import audit
from scripts.run_camera_varied_start_cohort import load_cases
from scripts.run_camera_pair_transport import evaluate_grasp_samples
from scripts.run_camera_varied_start_student import AXES, LIMITS, PHASES, choose_stage_actions

ROBOTS = ("r1", "r3")


def _sha(data):
    return hashlib.sha256(data).hexdigest()


class VariedStartAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.run, self.stage, self.straight, self.grasp = [root / name for name in
            ("run", "stage", "straight", "grasp")]
        for path in (self.run / "rgb", self.stage, self.straight, self.grasp):
            path.mkdir(parents=True)
        nested = {r: {} for r in ROBOTS}
        for rid in ROBOTS:
            for stage in AXES:
                data = json.dumps({"robot_id": rid, "stage": stage}).encode()
                name = f"{rid}-{stage}.json"; (self.stage / name).write_bytes(data)
                nested[rid][stage] = {"path": name, "sha256": _sha(data)}
        (self.stage / "varied-start-skill.json").write_text(json.dumps({"models": nested}))
        records = {}
        for rid in ROBOTS:
            data = json.dumps({"robot_id": rid}).encode(); name = f"{rid}.json"
            (self.grasp / name).write_bytes(data)
            records[rid] = {"path": name, "sha256": _sha(data)}
        (self.grasp / "student-skill.json").write_text(json.dumps({"models": records}))
        straight_records = {}
        for rid in ROBOTS:
            data = json.dumps({"straight": rid}).encode(); name = f"straight-{rid}.json"
            (self.straight / name).write_bytes(data)
            straight_records[rid] = {"path": name, "sha256": _sha(data)}
        (self.straight / "approach-skill.json").write_text(json.dumps({"models": straight_records}))

    def tearDown(self):
        self.temp.cleanup()

    def image(self, name, data):
        path = self.run / "rgb" / name; path.write_bytes(data)
        return {"path": f"rgb/{name}", "sha256": _sha(data)}

    def make_run(self, *, final_ready=True):
        ready = {"ok": True, "command": 0.0, "ready_score": 1.0, "ready": True,
                 "reason": "ready", "diagnostics": {}}
        not_ready = {**ready, "command": .01, "ready_score": .2,
                     "ready": False, "reason": "move"}
        history, calls, trace, results = {r: [] for r in ROBOTS}, [], [], []
        frame = 0
        for phase_index, stage in enumerate(PHASES):
            confirmations = []
            for index in range(3):
                frame += 1
                top = self.image(f"p{phase_index}-{index}-top", f"top-{frame}".encode())
                control = choose_stage_actions({r: ready for r in ROBOTS}, stage, index > 0)
                actions = {}
                for rid in ROBOTS:
                    own = self.image(f"p{phase_index}-{index}-{rid}", f"own-{rid}-{frame}".encode())
                    action = {"kind": "mecanum", **control["commands"][rid],
                              "duration_s": control["duration_s"]}
                    calls.append({"phase_index": phase_index, "stage": stage, "index": index,
                        "robot_id": rid, "frame_id": frame, "stationary": index > 0,
                        "images": {"own": own, "top": top}, "decision": ready,
                        "action": action, "own_command_history": list(history[rid])})
                    history[rid].append(action); actions[rid] = action
                trace.append({"stage": "approach_stop_dwell", "actions": actions})
                if index > 0:
                    confirmations.append({"frame_ids": {r: frame for r in ROBOTS},
                                          "ready": {r: True for r in ROBOTS}, "stationary": True})
            results.append({"phase_index": phase_index, "stage": stage, "ok": True,
                "confirmations": confirmations,
                "reason": "two consecutive fresh stationary RGB confirmations", "movement_slices": 0})
        final = []
        for index in range(2):
            frame += 1
            top = self.image(f"final-{index}-top", f"final-top-{index}".encode())
            images, decisions = {}, {}
            for rid in ROBOTS:
                own = self.image(f"final-{index}-{rid}", f"final-own-{rid}-{index}".encode())
                images[rid] = {"own": own, "top": top}
                decisions[rid] = {stage: (ready if final_ready else not_ready) for stage in AXES}
            final.append({"frame_ids": {r: frame for r in ROBOTS}, "images": images,
                          "decisions": decisions})
            trace.append({"stage": "approach_stop_dwell", "actions":
                {r: {"kind": "drive", "forward": 0.0, "turn": 0.0, "duration_s": .25} for r in ROBOTS}})
        sample = {"sim_time_s": 0.0, "phase": "approach", "height_above_start_m": 0.0,
                  "contacts": {}, "constraints_active": {}}
        (self.run / "evaluation-only.jsonl").write_text(json.dumps(sample) + "\n")
        evaluation = evaluate_grasp_samples([sample])
        (self.run / "execution-trace.json").write_text(json.dumps(
            [{"stage": "folded_setup", "command": {}}] + trace))
        source = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
        stage_skill = json.loads((self.stage / "varied-start-skill.json").read_text())
        grasp_skill = json.loads((self.grasp / "student-skill.json").read_text())
        report = {"source_sha": source, "error": None,
            "config": {"condition": "visual", "start_poses_setup_only": {},
                "phases": list(PHASES), "limits": LIMITS, "slice_s": .2,
                "stop_dwell_s": .25, "max_confirmation_frames": 4,
                "required_consecutive_stationary_ready_frames": 2, "weld": False},
            "stage_skill_sha256": _sha((self.stage / "varied-start-skill.json").read_bytes()),
            "stage_model_sha256": {r: {s: stage_skill["models"][r][s]["sha256"] for s in AXES} for r in ROBOTS},
            "grasp_skill_sha256": _sha((self.grasp / "student-skill.json").read_bytes()),
            "grasp_model_sha256": {r: grasp_skill["models"][r]["sha256"] for r in ROBOTS},
            "approach_calls": calls, "stage_results": results,
            "final_alignment_checks": final, "approach_ok": final_ready,
            "calls": [], "evaluation": evaluation, "grasp_success": False,
            "approach_physics_steps": 100, "approach_payload_contact_steps": 0,
            "approach_collision_events": [], "success": False}
        (self.run / "result.json").write_text(json.dumps(report))
        if final_ready:
            (self.run / "grasp-result.json").write_text(json.dumps(
                {"source_sha": source, "calls": [], "evaluation": evaluation}))
        return report, ready, not_ready

    def test_exact_phase_replay_and_final_checks(self):
        _report, ready, _ = self.make_run()
        with patch("scripts.audit_camera_varied_start_student._predict_stage", return_value=ready), \
             patch("scripts.audit_camera_varied_start_student._audit_grasp", return_value={"ok": True}):
            result = audit(self.run, self.stage, self.straight, self.grasp)
        self.assertTrue(result["approach_ok"])
        self.assertEqual([row["stage"] for row in result["stage_results"]], list(PHASES))

    def test_final_alignment_failure_is_audited_normal_abort(self):
        _report, ready, not_ready = self.make_run(final_ready=False)
        calls = {0: 0}
        def prediction(_model, own, _top):
            calls[0] += 1
            return not_ready if b"final-own" in own else ready
        with patch("scripts.audit_camera_varied_start_student._predict_stage", side_effect=prediction):
            result = audit(self.run, self.stage, self.straight, self.grasp)
        self.assertFalse(result["approach_ok"])

    def test_coarse_heading_cannot_approve_final_grasp(self):
        report, ready, _ = self.make_run(final_ready=False)
        coarse = {**ready, "precision": "coarse"}
        for check in report["final_alignment_checks"]:
            for rid in ("r1", "r3"):
                for axis in AXES:
                    check["decisions"][rid][axis] = coarse
        (self.run / "result.json").write_text(json.dumps(report))
        def prediction(_model, own, _top):
            return coarse if b"final-own" in own else ready
        with patch("scripts.audit_camera_varied_start_student._predict_stage", side_effect=prediction):
            result = audit(self.run, self.stage, self.straight, self.grasp)
        self.assertFalse(result["approach_ok"])

    def test_straight_condition_replays_only_forward_phase(self):
        report, ready, _ = self.make_run()
        raw = {"ok": True, "forward": 0.0, "ready": True, "reason": "ready",
               "diagnostics": {}, "stop_score": 1.0}
        saved = {**raw, "command": 0.0}
        report["config"]["condition"] = "straight"
        report["approach_calls"] = report["approach_calls"][:6]
        for call in report["approach_calls"]:
            call["stage"] = "forward"; call["decision"] = saved
        record = report["stage_results"][0]
        record["stage"] = "forward"
        report["stage_results"] = [record]
        report.pop("final_alignment_checks")
        skill = json.loads((self.straight / "approach-skill.json").read_text())
        report["straight_skill_sha256"] = _sha((self.straight / "approach-skill.json").read_bytes())
        report["straight_model_sha256"] = {r: skill["models"][r]["sha256"] for r in ROBOTS}
        (self.run / "result.json").write_text(json.dumps(report))
        trace = json.loads((self.run / "execution-trace.json").read_text())
        (self.run / "execution-trace.json").write_text(json.dumps(trace[:4]))
        with patch("scripts.audit_camera_varied_start_student._predict_straight", return_value=raw), \
             patch("scripts.audit_camera_varied_start_student._audit_grasp", return_value={"ok": True}):
            result = audit(self.run, self.stage, self.straight, self.grasp)
        self.assertEqual(result["condition"], "straight")
        self.assertEqual([row["stage"] for row in result["stage_results"]], ["forward"])

    def test_rejects_history_and_rgb_corruption(self):
        report, ready, _ = self.make_run()
        report["approach_calls"][2]["own_command_history"] = []
        (self.run / "result.json").write_text(json.dumps(report))
        with patch("scripts.audit_camera_varied_start_student._predict_stage", return_value=ready):
            with self.assertRaisesRegex(ValueError, "history mismatch"):
                audit(self.run, self.stage, self.straight, self.grasp)

        report, ready, _ = self.make_run()
        path = self.run / report["approach_calls"][0]["images"]["own"]["path"]
        path.write_bytes(b"tampered")
        with patch("scripts.audit_camera_varied_start_student._predict_stage", return_value=ready):
            with self.assertRaisesRegex(ValueError, "RGB hash mismatch"):
                audit(self.run, self.stage, self.straight, self.grasp)

    def test_cohort_case_scope_is_strict_and_independent(self):
        path = Path(self.temp.name) / "cases.json"
        valid = {"cases": [{"case_id": "corner-1", "start_poses": {
            "r1": {"distance_m": .15, "lateral_m": -.06, "yaw_deg": -10},
            "r3": {"distance_m": .40, "lateral_m": .06, "yaw_deg": 10}}}]}
        path.write_text(json.dumps(valid))
        self.assertEqual(load_cases(path), valid["cases"])
        valid["cases"][0]["start_poses"]["r3"]["yaw_deg"] = 10.01
        path.write_text(json.dumps(valid))
        with self.assertRaisesRegex(ValueError, "outside protocol"):
            load_cases(path)

if __name__ == "__main__":
    unittest.main()
