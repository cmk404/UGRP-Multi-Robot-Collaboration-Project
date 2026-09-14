import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from scripts.audit_camera_approach_student import audit
from scripts.run_camera_approach_student import choose_actions
from scripts.run_camera_pair_transport import evaluate_grasp_samples


ROBOTS = ("r1", "r3")


def sha(data):
    return hashlib.sha256(data).hexdigest()


class ApproachAuditTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.run, self.approach, self.grasp = root / "run", root / "approach", root / "grasp"
        for path in (self.run / "rgb", self.approach, self.grasp):
            path.mkdir(parents=True)
        self.models = {"r1": {"robot": "r1"}, "r3": {"robot": "r3"}}
        self._manifest(self.approach, "approach-skill.json")
        self._manifest(self.grasp, "student-skill.json")

    def tearDown(self):
        self.tmp.cleanup()

    def _manifest(self, root, name):
        records = {}
        for rid in ROBOTS:
            data = json.dumps(self.models[rid]).encode()
            (root / f"{rid}.json").write_bytes(data)
            records[rid] = {"path": f"{rid}.json", "sha256": sha(data)}
        (root / name).write_text(json.dumps({"models": records}))

    def _rgb(self, name, data):
        path = self.run / "rgb" / name
        path.write_bytes(data)
        return {"path": f"rgb/{name}", "sha256": sha(data)}

    def make_run(self, decisions):
        """Decisions is one per round and is returned for both independent models."""
        calls, history = [], {r: [] for r in ROBOTS}
        goals = {r: [] for r in ROBOTS}
        trace = []
        phase, cruise_index = "cruise", 0
        for index, decision in enumerate(decisions):
            top = self._rgb(f"{index}-top.jpg", f"top-{index}".encode())
            by_robot = {r: self._rgb(f"{index}-{r}.jpg", f"{r}-{index}".encode()) for r in ROBOTS}
            ds = {r: decision for r in ROBOTS}
            control = choose_actions(ds, "visual", phase, cruise_index, 1.0)
            for rid in ROBOTS:
                calls.append({"index": index, "cruise_index": cruise_index, "phase": phase,
                    "robot_id": rid, "frame_id": index + 1, "stationary": phase != "cruise",
                    "images": {"own": by_robot[rid], "top": top}, "decision": decision,
                    "action": control["actions"][rid], "own_command_history": list(history[rid])})
                history[rid].append(control["actions"][rid])
                if phase != "cruise":
                    goals[rid].append({"frame_id": index + 1, "stationary": True,
                        "ok": not control["blocked"], "ready": control["ready"][rid], "index": index})
            trace.append({"stage": "approach" if any(a["forward"] for a in control["actions"].values()) else "approach_stop_dwell",
                          "actions": control["actions"]})
            if control["blocked"] or phase == "confirmation-2":
                break
            if phase == "confirmation-1":
                phase = "confirmation-2"
            elif control["enter_confirmation"]:
                phase = "confirmation-1"
            else:
                cruise_index += 1
        completed = phase == "confirmation-2" and not control["blocked"]
        sample = {"sim_time_s": 0.0, "phase": "approach", "height_above_start_m": 0.0,
                  "contacts": {}, "constraints_active": {}}
        (self.run / "evaluation-only.jsonl").write_text(json.dumps(sample) + "\n")
        evaluation = evaluate_grasp_samples([sample])
        (self.run / "execution-trace.json").write_text(json.dumps(trace))
        head = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
        report = {"source_sha": head, "error": None,
            "config": {"condition": "visual", "playback_seconds": 1.0, "max_rounds": 100,
                       "slice_s": .2, "confirmation_dwell_s": .25},
            "approach_skill_sha256": sha((self.approach / "approach-skill.json").read_bytes()),
            "approach_model_sha256": {r: sha((self.approach / f"{r}.json").read_bytes()) for r in ROBOTS},
            "grasp_skill_sha256": sha((self.grasp / "student-skill.json").read_bytes()),
            "grasp_model_sha256": {r: sha((self.grasp / f"{r}.json").read_bytes()) for r in ROBOTS},
            "approach_calls": calls, "goal_confirmation": goals, "approach_ok": completed,
            "calls": [], "evaluation": evaluation, "grasp_success": False,
            "approach_physics_steps": 10, "approach_payload_contact_steps": 0,
            "approach_collision_events": [], "success": False}
        (self.run / "result.json").write_text(json.dumps(report))
        if completed:
            (self.run / "grasp-result.json").write_text(json.dumps(
                {"source_sha": head, "calls": [], "evaluation": evaluation}))
        return report

    def test_replays_threshold_confirmation_and_ood_stop(self):
        ready = {"ok": True, "ready": True, "forward": 0.0}
        self.make_run([ready, ready, ready])
        with patch("scripts.audit_camera_approach_student._predict_approach", return_value=ready), \
             patch("scripts.audit_camera_approach_student._audit_grasp", return_value={"ok": True}):
            result = audit(self.run, self.approach, self.grasp)
        self.assertTrue(result["approach_ok"])
        self.assertEqual([r["phase"] for r in result["replay"]],
                         ["cruise", "confirmation-1", "confirmation-2"])

        # An unsupported/OOD observation must stop immediately and never enter grasp.
        for child in self.run.iterdir():
            if child.is_file():
                child.unlink()
            else:
                for item in child.iterdir():
                    item.unlink()
        ood = {"ok": False, "ready": False, "forward": 0.15}
        self.make_run([ood])
        with patch("scripts.audit_camera_approach_student._predict_approach", return_value=ood):
            result = audit(self.run, self.approach, self.grasp)
        self.assertFalse(result["approach_ok"])
        self.assertEqual(result["replay"][0]["actions"]["r1"]["forward"], 0.0)

    def test_rejects_rgb_hash_and_action_history_corruption(self):
        ood = {"ok": False, "ready": False, "forward": 0.0}
        report = self.make_run([ood])
        image_path = self.run / report["approach_calls"][0]["images"]["own"]["path"]
        image_path.write_bytes(b"corrupt")
        with patch("scripts.audit_camera_approach_student._predict_approach", return_value=ood):
            with self.assertRaisesRegex(ValueError, "RGB hash mismatch"):
                audit(self.run, self.approach, self.grasp)

        report = self.make_run([ood])
        report["approach_calls"][0]["own_command_history"] = [{"kind": "drive"}]
        (self.run / "result.json").write_text(json.dumps(report))
        with patch("scripts.audit_camera_approach_student._predict_approach", return_value=ood):
            with self.assertRaisesRegex(ValueError, "own command history mismatch"):
                audit(self.run, self.approach, self.grasp)


if __name__ == "__main__":
    unittest.main()
