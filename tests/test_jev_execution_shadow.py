"""No live API calls. Exercise disclosure boundary and invalid response handling."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image
from scripts.jev_execution_shadow import make_case, request_body, validate_response, OPTIONS, rule


class ShadowTests(unittest.TestCase):
    def test_per_robot_whitelist_excludes_private_and_evaluation_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            episode = root / "seed" / "episode"
            episode.mkdir(parents=True)
            image = episode / "rgb.jpg"
            Image.new("RGB", (8, 8), "red").save(image)
            ref = {"path": "rgb.jpg", "sha256": hashlib.sha256(image.read_bytes()).hexdigest()}
            rows = [{"kind": "act_carry", "index": i, "evaluation": "DO_NOT_SEND",
                     "inputs": {"r1": {"images": {"own": ref, "top": ref}, "context": [0]*8,
                                         "physical_robot_id": "r3"}, "r3": "DO_NOT_SEND"},
                     "decisions": {"r1": {"done": False, "stop_score": 0.1,
                         "own_attachment": {"held_estimate": True, "anchor_area_ratio": 1,
                                            "unexpected_private": "DO_NOT_SEND"}}, "r3": "DO_NOT_SEND"},
                     "permission": "DO_NOT_SEND"} for i in range(5)]
            (episode / "pair-decisions.json").write_text(json.dumps(rows))
            case = make_case(root, "seed", "episode", 4, "r1")
            wire = json.dumps(request_body(case["state"], "jev-latest"))
            self.assertNotIn("DO_NOT_SEND", wire)
            self.assertNotIn("source_sha256", wire)
            self.assertEqual(case["provenance"]["physical_robot_id"], "r3")
            self.assertEqual(rule(case["state"]), "reobserve")
            image.write_bytes(b"tampered")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                make_case(root, "seed", "episode", 4, "r1")

    def test_rejects_invalid_provider_decisions(self):
        a = {"type": "choice", "choice": "continue", "confidence": 1,
             "probabilities": {k: float(k == "continue") for k in OPTIONS}}
        response = {"answers": {"action": a}}
        self.assertEqual(validate_response(response)["choice"], "continue")
        for field, value in [("choice", "release"), ("confidence", float("nan"))]:
            bad = json.loads(json.dumps(response))
            bad["answers"]["action"][field] = value
            with self.assertRaises(ValueError):
                validate_response(bad)
        a["probabilities"]["continue"] = 0.2
        with self.assertRaises(ValueError):
            validate_response(response)


if __name__ == "__main__":
    unittest.main()
