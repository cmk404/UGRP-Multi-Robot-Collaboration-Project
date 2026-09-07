import json

from harness.transport_context import compact_own_memory, memory_needs_nav_comparison


def test_compaction_keeps_latest_semantics_and_three_actions_only():
    memory = [
        {"action": {"kind": "approach"}, "placement": {"status": "outside", "stage": "released",
                                                              "reason": "경계 밖", "source": "own_rgb"}},
        {"action": {"kind": "pick"}},
        {"action": {"kind": "drive", "fwd": .04, "turn": 0, "duration": 2}},
        {"action": {"kind": "release"}, "placement": {"status": "inside", "stage": "released",
                                                                "reason": "세 변 확인", "source": "own_rgb"},
         "geometry": {"pixels": list(range(1000))}, "nav_sha256": "a" * 64},
    ]
    result = compact_own_memory(memory)
    assert result["latest_placement"]["status"] == "inside"
    assert [item["kind"] for item in result["last_actions"]] == ["pick", "drive", "release"]
    serialized = json.dumps(result)
    assert "pixels" not in serialized
    assert "sha256" not in serialized


def test_only_comparison_relevant_critical_feedback_requests_previous_nav():
    blocked = compact_own_memory([{"feedback": "OWN_RGB_DRIVE_BLOCKED"}])
    grip = compact_own_memory([{"feedback": "GRIP_UNCERTAIN"}])
    assert memory_needs_nav_comparison(blocked)
    assert not memory_needs_nav_comparison(grip)


def test_saved_request_shape_keeps_actual_placement_identity_without_geometry():
    # Shape copied from repaired02/team-41 request memory, with bulky values shortened.
    memory = [{"action": {"kind": "pick"}, "feedback": "accepted"},
              {"placement_evidence": {
                  "status": "uncertain", "reason": "DESTINATION_ZONE_NOT_VISIBLE",
                  "stage": "before_release", "cargo_id": "small_box_02",
                  "destination_zone": "B", "wrist_sha256": "a" * 64,
                  "nav_sha256": "b" * 64,
                  "identity": {"confirmed": True, "marker_id": 15, "visible": False,
                               "provenance": "caller_confirmed_tracked_id_grasp"},
                  "predicted_footprint_pixels": [[319.5, 605.8], [442.8, 775.0]],
                  "calibrated_lowering_center_m": [0.165, 0.0]}},
              {"feedback": "NAVIGATION_INTERRUPTED:OWN_RGB_STAGNATION",
               "reason": "NEW_PEER_MESSAGE"},
              {"action": {"kind": "drive", "fwd": 0.08, "turn": -0.2, "duration": 0.8},
               "feedback": "accepted"}]
    result = compact_own_memory(memory)
    assert result["latest_placement"] == {
        "status": "uncertain", "reason": "DESTINATION_ZONE_NOT_VISIBLE",
        "stage": "before_release", "cargo_id": "small_box_02",
        "destination_zone": "B", "identity": {"confirmed": True}}
    feedback = json.dumps(result["critical_feedback"], ensure_ascii=False)
    assert "OWN_RGB_STAGNATION" in feedback
    assert "NEW_PEER_MESSAGE" in feedback
    assert "accepted" not in feedback
    assert "pixels" not in json.dumps(result)
