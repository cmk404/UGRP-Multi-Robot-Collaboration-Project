import hashlib
import json

import cv2
import numpy as np
import pytest

from harness.known_map_navigation import KnownMapNavigator
from scripts.audit_known_map_navigation import audit_run
from sim.authored_navigation_map import map_sha256


def _dump(path, value):
    path.write_text(json.dumps(value, sort_keys=True) + "\n")


def _jpeg(point, data):
    h, w = 480, 640
    camera = data["top_camera"]
    distance = camera["position_m"][2] - .09
    visible_h = 2 * distance * np.tan(np.deg2rad(camera["fov_y_deg"]) / 2)
    visible_w = visible_h * w / h
    u = round((point[0] - camera["position_m"][0]) * w / visible_w + (w - 1) / 2)
    v = round((camera["position_m"][1] - point[1]) * h / visible_h + (h - 1) / 2)
    image = np.zeros((h, w, 3), np.uint8)
    cv2.circle(image, (u, v), 10, (0, 255, 255), -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


def _map():
    return {"schema": "ugrp.authored_navigation_map.v1", "map_id": "audit_test", "version": 1,
            "frame": "warehouse_xy_m", "bounds_m": [-.85, 1.95, -3.1, -.9],
            "grid_resolution_m": .025,
            "top_camera": {"name": "cctv_top", "position_m": [.55, -2, 2.5],
                           "quaternion_wxyz": [1, 0, 0, 0], "fov_y_deg": 55},
            "footprint": {"unloaded_radius_m": .18, "safety_margin_m": .04},
            "zones": {"start": {"center_m": [-.5, -2.55], "radius_m": .2},
                      "goal": {"center_m": [1.55, -2.55], "radius_m": .18}},
            "obstacles": []}


def _rehash(root):
    files = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
             for path in sorted(root.rglob("*")) if path.is_file() and path.name != "manifest.json"}
    _dump(root / "manifest.json", files)


def make_run(root):
    (root / "rgb").mkdir(parents=True)
    data = _map(); digest = map_sha256(data)
    own = _jpeg((-.5, -2.55), data)
    actor = KnownMapNavigator(data, "r1", "map")
    history = []
    rows = []
    for frame in range(2):
        for kind in ("own", "top"):
            (root / "rgb" / f"{frame:04d}-{kind}.jpg").write_bytes(own)
        decision = actor.decide(own, own, frame)
        images = {kind: {"path": f"rgb/{frame:04d}-{kind}.jpg", "sha256": hashlib.sha256(own).hexdigest()}
                  for kind in ("own", "top")}
        rows.append({"frame_id": frame, "map_sha256": digest, "images": images,
                     "issued_history": list(history), "decision": decision})
        history.append(decision["action"])
    (root / "actor-decisions.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    _dump(root / "actor-map.json", data)
    _dump(root / "run.json", {"schema": "ugrp.known_map_run.v1", "map_sha256": digest,
                              "condition": "map", "robot_id": "r1"})
    invariant = {"map_sha256": digest, "actor_cameras": {}, "added_navigation_camera": False}
    _dump(root / "invariants-before.json", invariant); _dump(root / "invariants-after.json", invariant)
    _dump(root / "result.json", {"map_sha256": digest, "condition": "map", "decisions": 2,
                                 "actor_status": rows[-1]["decision"]["status"],
                                 "evaluation": {"success": False}, "error": None})
    _rehash(root)
    return rows


def test_audit_replays_inputs_and_keeps_navigation_result_separate(tmp_path):
    make_run(tmp_path)
    result = audit_run(tmp_path)
    assert result["audit_pass"] is True
    assert result["navigation_success"] is False
    assert result["frames"] == 2
    assert "does not cryptographically prove" in result["claim_limit"]


def test_manifest_detects_rgb_byte_tamper(tmp_path):
    make_run(tmp_path)
    (tmp_path / "rgb/0001-top.jpg").write_bytes(b"tampered")
    with pytest.raises(ValueError, match="manifest hash mismatch"):
        audit_run(tmp_path)


def test_rejects_extra_actor_input_field_even_with_rehashed_manifest(tmp_path):
    rows = make_run(tmp_path)
    rows[0]["world_state"] = {"robot_xy": [0, 0]}
    (tmp_path / "actor-decisions.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    _rehash(tmp_path)
    with pytest.raises(ValueError, match="unexpected input fields"):
        audit_run(tmp_path)


def test_rejects_issued_history_mutation_even_with_rehashed_manifest(tmp_path):
    rows = make_run(tmp_path)
    rows[1]["issued_history"] = []
    (tmp_path / "actor-decisions.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    _rehash(tmp_path)
    with pytest.raises(ValueError, match="issued_history chain"):
        audit_run(tmp_path)


def test_rejects_unexpected_image_record_field(tmp_path):
    rows = make_run(tmp_path)
    rows[0]["images"]["own"]["robot_pose"] = [0, 0]
    (tmp_path / "actor-decisions.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    _rehash(tmp_path)
    with pytest.raises(ValueError, match="unexpected fields"):
        audit_run(tmp_path)
