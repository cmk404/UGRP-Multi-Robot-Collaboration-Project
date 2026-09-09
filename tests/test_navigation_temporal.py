"""Offline tests for RGB-only navigation temporal evidence."""
import base64
import hashlib
import json

import cv2
import numpy as np
import pytest


from harness import navigation_temporal as MODULE
from harness.navigation_temporal import NavigationTemporalEvidence
from harness.gemini_transport_policy import GeminiTransportPlanner


def observation(value=0, *, robot_id="r1", frame_id=1):
    image = np.full((48, 64, 3), value, np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    jpeg = encoded.tobytes()
    return {"robot_id": robot_id, "frame_id": frame_id, "sim_time": float(frame_id),
            "image": base64.b64encode(jpeg).decode(),
            "sha256": hashlib.sha256(jpeg).hexdigest(), "camera": "nav_cam",
            "actuator_state": {"servo_pulses": {}}}


def evidence(*, veto=False, orange=None, peers=None):
    return {"orange_obstacles": orange or [], "magenta_peers": peers or [],
            "forward_stop_check": {"vetoed": veto, "reason": "test"}}


def macro(macro_id, *, fwd=0.0, turn=0.0, elapsed=1.0, status="completed"):
    return {"last_macro": {"decision_id": macro_id, "status": status,
                           "macro": {"kind": "drive", "fwd": fwd, "turn": turn},
                           "elapsed_drive_control_s": elapsed}}


def test_counts_actual_terminal_macros_not_proposals_and_deduplicates_stale_feedback():
    history = NavigationTemporalEvidence("r1")
    proposed = {"last_decision": {"action": {"kind": "drive", "fwd": .1, "turn": 0},
                                  "disposition": "accepted"}}
    first = history.observe(observation(), evidence(), proposed)
    assert first["forward_control_s_total"] == 0
    turn = macro("m1", turn=.2, elapsed=2.0)
    second = history.observe(observation(frame_id=2), evidence(), turn)
    third = history.observe(observation(frame_id=3), evidence(), turn)
    assert second["executed_since_forward"]["turn_only_macros"] == 1
    assert third["executed_since_forward"]["turn_only_macros"] == 1


def test_stopped_partial_forward_counts_applied_control_and_resets_turn_cycle():
    history = NavigationTemporalEvidence("r1")
    history.observe(observation(frame_id=1), evidence(), macro("left", turn=.2))
    history.observe(observation(frame_id=2), evidence(), macro("right", turn=-.2))
    cycled = history.observe(observation(frame_id=3), evidence(), macro("left2", turn=.2))
    assert cycled["executed_since_forward"] == {"turn_only_macros": 3,
                                                 "turn_direction_changes": 2}
    moved = history.observe(observation(frame_id=4), evidence(),
                            macro("partial", fwd=.1, elapsed=.25, status="stopped"))
    assert moved["forward_control_s_total"] == .25
    assert moved["executed_since_forward"] == {"turn_only_macros": 0,
                                                "turn_direction_changes": 0}

    interrupted = history.observe(observation(frame_id=5), evidence(),
                                  macro("guard-stop", fwd=.1, elapsed=.1, status="interrupted"))
    assert interrupted["forward_control_s_total"] == .35


def test_retains_recent_blocker_and_flags_only_separated_no_forward_similar_view():
    history = NavigationTemporalEvidence("r1")
    blocker = [{"bbox_norm": [0.0, .69, .15, .31], "area_fraction": .032}]
    first = history.observe(observation(30, frame_id=1), evidence(veto=True, orange=blocker), None)
    assert first["most_recent_near_blocker"] == {
        "label": "orange", "side": "left", "bbox_norm": [0.0, .69, .15, .31],
        "observations_ago": 0, "forward_control_s_since_seen": 0.0}
    history.observe(observation(180, frame_id=2), evidence(), macro("turn", turn=.2))
    repeated = history.observe(observation(31, frame_id=3), evidence(), macro("turn2", turn=-.2))
    revisit = repeated["potential_revisited_blocked_view"]
    assert revisit["matched_observations_ago"] == 2
    assert "proof" in revisit["meaning"]
    assert repeated["most_recent_near_blocker"]["observations_ago"] == 2


def test_visible_near_peer_is_remembered_without_claiming_identity():
    history = NavigationTemporalEvidence("r1")
    peer = [{"bbox_norm": [.65, .6, .2, .3], "area_fraction": .018}]
    current = history.observe(observation(), evidence(peers=peer), None)
    assert current["most_recent_near_blocker"] == {
        "label": "peer", "side": "right", "bbox_norm": [.65, .6, .2, .3],
        "observations_ago": 0, "forward_control_s_since_seen": 0.0}


def test_positive_forward_prevents_old_blocked_view_match_and_updates_since_seen():
    history = NavigationTemporalEvidence("r1")
    blocker = [{"bbox_norm": [.7, .7, .25, .3], "area_fraction": .04}]
    history.observe(observation(20, frame_id=1), evidence(veto=True, orange=blocker), None)
    history.observe(observation(100, frame_id=2), evidence(),
                    macro("forward", fwd=.1, elapsed=.4, status="stopped"))
    current = history.observe(observation(20, frame_id=3), evidence(), None)
    assert "potential_revisited_blocked_view" not in current
    assert current["most_recent_near_blocker"]["forward_control_s_since_seen"] == .4


def test_original_blocked_observation_eligibility_is_bounded_and_forward_expires_it():
    history = NavigationTemporalEvidence("r1")
    blocked = observation(25, frame_id=1)
    history.observe(blocked, evidence(veto=True), None)
    history.observe(observation(100, frame_id=2), evidence(), macro("turn-1", turn=.2))
    assert history.eligible_recent_blocked_observation() is None
    history.observe(observation(120, frame_id=3), evidence(), macro("turn-2", turn=-.2))
    reference = history.eligible_recent_blocked_observation()
    assert reference is not blocked
    assert reference["sha256"] == blocked["sha256"]
    reference["sha256"] = "mutated"
    assert history.eligible_recent_blocked_observation()["sha256"] == blocked["sha256"]

    history.observe(observation(140, frame_id=4), evidence(),
                    macro("forward", fwd=.1, elapsed=.1, status="interrupted"))
    assert history.eligible_recent_blocked_observation() is None


def test_original_blocked_observation_expires_after_twelve_observations():
    history = NavigationTemporalEvidence("r1")
    history.observe(observation(frame_id=1), evidence(veto=True), None)
    for index in range(2, 14):
        history.observe(observation(index, frame_id=index), evidence(),
                        macro(f"turn-{index}", turn=.2))
    assert history.recent_blocked_observations_ago == 12
    assert history.eligible_recent_blocked_observation() is not None
    history.observe(observation(14, frame_id=14), evidence(), macro("turn-14", turn=.2))
    assert history.eligible_recent_blocked_observation() is None


def test_output_is_bounded_and_ownership_is_enforced():
    history = NavigationTemporalEvidence("r1")
    for index in range(30):
        result = history.observe(observation(index, frame_id=index + 1), evidence(),
                                 macro(f"turn-{index}", turn=.2 if index % 2 else -.2))
    assert len(history._frames) == 12
    assert len(json.dumps(result, separators=(",", ":"))) < 600
    with pytest.raises(ValueError, match="OWNERSHIP"):
        history.observe(observation(robot_id="r2"), evidence(), None)




class Completer:
    model_name = "offline"
    last_model = "offline"
    last_usage = None

    def __init__(self):
        self.calls = []

    def complete(self, messages, *, images):
        self.calls.append((messages, images))
        return json.dumps({
            "action": {"kind": "drive", "fwd": 0, "turn": .2, "duration": 1},
            "reason": "현재 두 화면을 비교해 다음 행동을 선택함",
            "navigation_note": {"observed": "현재 화면을 확인함", "maneuver": "left",
                                "resume_when": "통로가 열릴 때"},
        }, ensure_ascii=False)


def planner_observation(camera, frame, frame_id):
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    jpeg = encoded.tobytes()
    return {"robot_id": "r1", "frame_id": frame_id, "sim_time": float(frame_id),
            "image": base64.b64encode(jpeg).decode(),
            "sha256": hashlib.sha256(jpeg).hexdigest(), "camera": camera,
            "actuator_state": {"servo_pulses": {}}}


def test_planner_sends_original_blocked_rgb_as_preferred_third_image_and_audits_it():
    completer = Completer()
    planner = GeminiTransportPlanner("r1", completer)
    wrist_frame = np.full((480, 640, 3), 40, np.uint8)
    blocked_frame = np.full((480, 640, 3), 40, np.uint8)
    cv2.rectangle(blocked_frame, (250, 375), (390, 479), (0, 150, 255), -1)
    blocked = planner_observation("nav_cam", blocked_frame, 1)

    def decide(nav, feedback=None):
        wrist = planner_observation("robot_cam", wrist_frame, nav["frame_id"])
        result = planner.decide(wrist, nav, [], cargo_id="small_box_01",
                                destination_zone="B", skill_state="carrying",
                                execution_feedback=feedback)
        assert result["action"]["kind"] == "drive"
        assert len(completer.calls[-1][1]) <= 3

    decide(blocked)
    clear1 = planner_observation("nav_cam", np.full((480, 640, 3), 90, np.uint8), 2)
    decide(clear1, macro("turn-1", turn=.2))
    # Same immediate previous view also qualifies PREVIOUS_NAV; blocked history wins the one slot.
    clear2 = planner_observation("nav_cam", np.full((480, 640, 3), 90, np.uint8), 3)
    decide(clear2, macro("turn-2", turn=-.2))

    messages, images = completer.calls[-1]
    assert [item["label"].split()[0] for item in images] == [
        "CURRENT_WRIST", "CURRENT_NAV", "RECENT_BLOCKED_NAV"]
    transmitted = base64.b64decode(images[2]["image"].split(",", 1)[1])
    assert hashlib.sha256(transmitted).hexdigest() == blocked["sha256"]
    context = json.loads(messages[-1]["content"])
    assert context["navigation_reference"] == {
        "kind": "recent_forward_veto_own_nav_rgb", "frame_id": 1, "sim_time": 1.0,
        "sha256": blocked["sha256"], "observations_ago": 2,
        "no_forward_control_since_reference": True,
        "reason_shown": "compare a prior blocked view after turn-only motion"}
    assert planner.last_audit["input_frames"][2] == {
        "camera": "nav_cam", "label": "RECENT_BLOCKED_NAV", "frame_id": 1,
        "sim_time": 1.0, "sha256": blocked["sha256"],
        "transmitted_sha256": blocked["sha256"]}

    clear3 = planner_observation("nav_cam", np.full((480, 640, 3), 170, np.uint8), 4)
    decide(clear3, macro("forward", fwd=.1, elapsed=.2, status="completed"))
    labels = [item["label"].split()[0] for item in completer.calls[-1][1]]
    assert "RECENT_BLOCKED_NAV" not in labels
    assert "navigation_reference" not in json.loads(completer.calls[-1][0][-1]["content"])
