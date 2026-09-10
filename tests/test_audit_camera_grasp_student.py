import hashlib
import json

import cv2
import numpy as np
import pytest

from harness.camera_teacher_student import fit_visual_jacobian, predict_correction
from scripts.audit_camera_grasp_student import audit


def _jpeg(dx=0, dy=0, top_dx=0):
    own = np.zeros((144, 192, 3), np.uint8)
    top = np.zeros((192, 256, 3), np.uint8)
    cv2.rectangle(own, (40 + dx, 45), (55 + dx, 60), (220, 220, 220), -1)
    cv2.rectangle(own, (120, 85 + dy), (136, 101 + dy), (150, 150, 150), -1)
    cv2.rectangle(top, (110 + top_dx, 75), (121 + top_dx, 90), (255, 255, 255), -1)
    return tuple(cv2.imencode(".jpg", image)[1].tobytes() for image in (own, top))


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fixture(tmp_path, tamper_action=False):
    run, models = tmp_path / "run", tmp_path / "models"
    (run / "rgb").mkdir(parents=True);models.mkdir()
    ref_own, ref_top = _jpeg()
    samples = []
    for channel, shift in enumerate(((4, 0, 0), (0, 4, 0), (0, 0, 4))):
        for sign in (-1, 1):
            own, top = _jpeg(*(sign * value for value in shift))
            delta = [0, 0, 0];delta[channel] = sign * 50
            samples.append({"own_jpeg": own, "top_jpeg": top, "delta_pulses": delta})
    model = fit_visual_jacobian(ref_own, ref_top, samples)
    model_path = models / "r1-model.json"
    model_path.write_text(json.dumps(model, sort_keys=True, allow_nan=False))
    skill = {"models": {"r1": {"path": "r1-model.json", "sha256": _sha(model_path)}}}
    skill_path = models / "student-skill.json";skill_path.write_text(json.dumps(skill, sort_keys=True))
    own, top = _jpeg(2, -2, 2)
    records = {}
    for name, value in (("own", own), ("top", top)):
        path = run / "rgb" / f"000-{name}.jpg";path.write_bytes(value)
        records[name] = {"path": f"rgb/000-{name}.jpg", "sha256": _sha(path)}
    decision = predict_correction(model, own, top, max_step=25)
    before = {1: 2000, 3: 1500, 4: 1500, 5: 1500}
    targets = {ch: before[ch] + delta for ch, delta in zip((3, 4, 5), decision["delta_pulses"]) if delta}
    actions = [{"kind": "arm", "servo_id": ch, "pulse": pulse} for ch, pulse in targets.items()]
    if tamper_action:
        actions[0]["pulse"] += 1
    call = {"round": 0, "robot_id": "r1", "images": records,
            "own_commands_before": before, "decision": decision,
            "actions": actions, "condition": "visual"}
    final = {**before, **targets}
    report = {"config": {"condition": "visual", "max_step": 25, "rounds": 1},
              "skill_sha256": _sha(skill_path), "model_sha256": {"r1": _sha(model_path)},
              "actor_initial_issued_commands": {"r1": before},
              "preclose_issued_commands": {"r1": final},
              "post_recovery_images": {"r1": records},
              "final_visual_errors": {"r1": decision}, "calls": [call]}
    (run / "result.json").write_text(json.dumps(report, sort_keys=True))
    return run, models


def test_exact_rgb_student_replay_succeeds(tmp_path):
    run, models = _fixture(tmp_path)
    result = audit(run, models)
    assert result["ok"] is True
    assert result["calls_source"] == "result.json:calls"
    assert result["calls"] == 1


def test_tampered_rgb_is_rejected(tmp_path):
    run, models = _fixture(tmp_path)
    (run / "rgb" / "000-own.jpg").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="RGB hash mismatch"):
        audit(run, models)


def test_tampered_raw_action_is_rejected(tmp_path):
    run, models = _fixture(tmp_path, tamper_action=True)
    with pytest.raises(ValueError, match="final raw action replay mismatch"):
        audit(run, models)
