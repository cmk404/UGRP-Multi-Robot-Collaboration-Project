"""Focused offline checks for E7 transmitted-context compaction."""
import base64
import hashlib
import json
import cv2
import numpy as np

from harness import gemini_transport_policy as policy


def observation(camera, frame_id, value):
    frame = np.full((48, 64, 3), value, np.uint8)
    ok, encoded = cv2.imencode(".jpg", frame)
    assert ok
    jpeg = encoded.tobytes()
    return {"robot_id": "r1", "frame_id": frame_id, "sim_time": float(frame_id),
            "image": base64.b64encode(jpeg).decode(),
            "sha256": hashlib.sha256(jpeg).hexdigest(), "camera": camera,
            "actuator_state": {"servo_pulses": {}}}


class Completer:
    model_name = "offline"
    last_model = "offline"
    last_usage = None
    def __init__(self): self.calls = []
    def complete(self, messages, *, images):
        self.calls.append((messages, images))
        return json.dumps({"action": {"kind": "drive", "fwd": 0, "turn": .1, "duration": 1},
            "reason": "현재 화면을 보고 좌측 우회를 유지함",
            "navigation_note": {"observed": "오른쪽 장애물이 가까이 보임", "maneuver": "left",
                "resume_when": "전방 통로가 열릴 때", "bypass_side": "left",
                "bypass_until": "장애물이 화면 오른쪽 뒤로 벗어날 때"}}, ensure_ascii=False)


def test_compacts_only_redundant_text_and_preserves_images_audit_and_control_context(monkeypatch):
    def nav_evidence(obs, zone, previous):
        return {"source": "validated own nav_cam RGB only", "image_sha256": obs["sha256"],
                "orange_obstacles": [{"bbox_norm": [.7, .5, .2, .3], "area_fraction": .06}],
                "magenta_peers": [], "target_floor": {"zone": zone, "visible": False},
                "forward_stop_check": {"vetoed": True, "reason": "OWN_RGB_NEAR_ORANGE",
                    "meaning": "same-RGB positive-forward stop check"}}
    monkeypatch.setattr(policy, "navigation_evidence", nav_evidence)
    completer = Completer()
    planner = policy.GeminiTransportPlanner("r1", completer)
    memory = [{"action": {"kind": "drive", "fwd": .1, "turn": 0, "duration": 1},
               "feedback": "STALE_LLM_FRAME", "skill_state": "carrying"},
              {"placement_evidence": {"status": "outside", "reason": "margin",
                                       "stage": "before_release", "source": "own_rgb"}}]
    execution = {"last_decision": {"call_id": "c1", "action": {"kind": "drive", "fwd": 0,
                 "turn": .1, "duration": 1}, "disposition": "accepted"},
                 "last_macro": {"decision_id": "d1", "status": "completed",
                 "macro": {"kind": "drive", "fwd": 0, "turn": .1, "duration": 1},
                 "elapsed_drive_control_s": 1.0}}
    first_wrist = observation("robot_cam", 1, 10)
    first_nav = observation("nav_cam", 2, 20)
    planner.decide(first_wrist, first_nav, memory, cargo_id="small_box_01",
                   destination_zone="B", skill_state="carrying", execution_feedback=execution)
    current_wrist = observation("robot_cam", 3, 30)
    current_nav = observation("nav_cam", 4, 40)
    planner.decide(current_wrist, current_nav, memory, cargo_id="small_box_01",
                   destination_zone="B", skill_state="carrying", execution_feedback=execution)

    messages, images = completer.calls[-1]
    context = json.loads(messages[-1]["content"])
    assert "recent_visual_history" not in context
    assert "last_actions" not in context["own_local_memory"]
    assert context["own_local_memory"]["critical_feedback"]
    assert context["own_local_memory"]["latest_placement"]["status"] == "outside"
    assert context["execution_feedback"] == execution
    assert context["previous_model_action"] == {"kind": "drive", "fwd": 0, "turn": .1, "duration": 1}
    assert context["previous_navigation_note"] == {"observed": "오른쪽 장애물이 가까이 보임",
        "provenance": "fallible_historical_model_note_not_current_rgb"}
    assert context["navigation_commitment"] == {"bypass_side": "left",
        "bypass_until": "장애물이 화면 오른쪽 뒤로 벗어날 때"}
    assert "image_sha256" not in context["nav_evidence"]
    assert context["nav_evidence"]["orange_obstacles"][0]["area_fraction"] == .06
    assert context["nav_evidence"]["forward_stop_check"]["vetoed"] is True
    assert all("sha256" not in frame for frame in context["camera_frames"])
    assert [(x["label"], x["frame_id"], x["sim_time"]) for x in context["camera_frames"]] == [
        ("WRIST_CAM", 3, 3.0), ("NAV_CAM", 4, 4.0)]

    assert len(images) <= 3
    for item, original in zip(images[:2], (current_wrist, current_nav)):
        transmitted = base64.b64decode(item["image"].split(",", 1)[1])
        assert hashlib.sha256(transmitted).hexdigest() == original["sha256"]
    audit = planner.last_audit
    assert audit["input_frames"][0]["sha256"] == current_wrist["sha256"]
    assert audit["input_frames"][0]["transmitted_sha256"] == current_wrist["sha256"]
    assert audit["input_frames"][1]["sha256"] == current_nav["sha256"]
    assert audit["input_frames"][1]["transmitted_sha256"] == current_nav["sha256"]
