import unittest

from sim.self_observer import PICK_MAX_BASE_SHIFT_M, diagnose_action


def base_state(**overrides):
    state = {
        "base_xyz": [0.0, 0.0, 0.04],
        "base_rpy": [0.0, 0.0, 0.0],
        "base_yaw": 0.0,
        "red_xyz": [0.25, 0.0, 0.015],
        "blue_xyz": [0.45, 0.1, 0.015],
        "yellow_xyz": [0.45, -0.1, 0.015],
        "grasp_color": None,
        "stable": False,
        "lifted": False,
        "bilateral_contact": False,
        "left_contact": False,
        "right_contact": False,
    }
    state.update(overrides)
    return state


def codes(report):
    return {x["code"] for x in report["issues"]}


def test_pick_rejects_chassis_motion_beyond_near_field_contract():
    after = base_state(
        base_xyz=[PICK_MAX_BASE_SHIFT_M + 0.03, 0.0, 0.04],
        red_xyz=[0.19, 0.0, 0.09], grasp_color="red", stable=True,
        lifted=True, bilateral_contact=True, left_contact=True, right_contact=True,
    )
    report = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(red_xyz=[0.19, 0.0, 0.015]),
        after=after, result_ok=True,
    )
    assert "PICK_BASE_MOTION_CONTRACT_EXCEEDED" in codes(report)
    assert report["metrics"]["pick_base_shift_limit_m"] == PICK_MAX_BASE_SHIFT_M


def test_pick_false_success_is_critical():
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(), after=base_state(), result_ok=True,
    )
    assert r["status"] == "BROKEN"
    assert "PICK_FALSE_SUCCESS" in codes(r)


def test_stable_pick_has_no_false_success():
    after = base_state(
        red_xyz=[0.19, 0.0, 0.09], grasp_color="red", stable=True,
        lifted=True, bilateral_contact=True, left_contact=True, right_contact=True,
    )
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(red_xyz=[0.19, 0.0, 0.015]), after=after, result_ok=True,
    )
    assert "PICK_FALSE_SUCCESS" not in codes(r)


def test_failed_pick_that_pushes_block_is_flagged():
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(red_xyz=[0.20, 0.0, 0.015]),
        after=base_state(red_xyz=[0.28, 0.0, 0.015]), result_ok=False,
    )
    assert "DESTRUCTIVE_GRASP_MISS" in codes(r)


def test_failed_pick_that_nudges_block_without_retained_lift_is_classified():
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(red_xyz=[0.5103, 0.0698, 0.0149]),
        after=base_state(red_xyz=[0.5205, 0.0531, 0.0149]), result_ok=False,
        result_reason="shared REAL pick completed but MuJoCo postcondition failed",
    )
    assert "GRASP_MISS_DISPLACED_TARGET" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_pick_nearfield_motion_budget_failure_is_classified():
    report = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(), after=base_state(), result_ok=False,
        result_reason="approach exceeded 6 measured motion pulses",
    )
    assert "PICK_NEARFIELD_MOTION_BUDGET_EXHAUSTED" in codes(report)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(report)


def test_body_alignment_budget_failure_is_classified():
    r = diagnose_action(
        action="approach", params={"target_color": "red"},
        before=base_state(base_yaw=0.0),
        after=base_state(base_yaw=0.35), result_ok=False,
        result_reason="body alignment exceeded 12 one-direction turn pulses",
    )
    assert "BODY_ALIGNMENT_BUDGET_EXHAUSTED" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_gaze_recentering_budget_failure_is_classified():
    r = diagnose_action(
        action="approach", params={"target_color": "red"},
        before=base_state(base_yaw=0.0),
        after=base_state(base_yaw=0.38), result_ok=False,
        result_reason="could not hold the red block at image centre",
    )
    assert "GAZE_RECENTER_BUDGET_EXHAUSTED" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_close_face_geometry_failure_is_classified():
    for reason in (
        "only 0 valid block-face samples; refusing a diagonal blind approach",
        "only 0 valid arm-face range samples; refusing blind arm-only grasp",
    ):
        r = diagnose_action(
            action="pick", params={"target_color": "red"},
            before=base_state(), after=base_state(), result_ok=False, result_reason=reason,
        )
        assert "FACE_GEOMETRY_UNAVAILABLE" in codes(r)
        assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_arm_face_reposition_stall_is_classified():
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(), after=base_state(), result_ok=False,
        result_reason="arm-relative face error did not improve for 2 consecutive base repositions; current=34.9deg best=34.6deg",
    )
    assert "ARM_FACE_REPOSITION_STALLED" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_capture_window_budget_failure_is_classified():
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(), after=base_state(), result_ok=False,
        result_reason="could not bring red block into calibrated visual capture window",
    )
    assert "CAPTURE_WINDOW_BUDGET_EXHAUSTED" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_pick_target_loss_near_field_is_classified():
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(), after=base_state(), result_ok=False,
        result_reason="target disappeared during final horizontal alignment",
    )
    assert "PICK_TARGET_LOST_NEAR_FIELD" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_missing_precision_pick_handoff_is_classified():
    r = diagnose_action(
        action="pick", params={"target_color": "red"},
        before=base_state(), after=base_state(), result_ok=False,
        result_reason="precision pick plan is missing; run approach immediately before pick",
    )
    assert "PRECISION_PICK_HANDOFF_MISSING" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_track_chassis_motion_is_visible_to_observer():
    r = diagnose_action(
        action="track", params={"target_color": "red"},
        before=base_state(),
        after=base_state(base_xyz=[0.06, 0.0, 0.04], base_yaw=0.5), result_ok=True,
    )
    assert "PERCEPTION_ACTION_MOVED_CHASSIS" in codes(r)
    assert "LOCAL_OBSERVATION_ROTATED_CHASSIS" in codes(r)


def test_place_transient_contact_false_loss_is_classified():
    before = base_state(
        red_xyz=[0.20, 0.0, 0.23], grasp_color="red", stable=True, lifted=True,
        bilateral_contact=True, left_contact=True, right_contact=True, grip_error_m=0.020,
    )
    after = base_state(
        red_xyz=[0.21, 0.0, 0.229], grasp_color="red", stable=False, lifted=True,
        bilateral_contact=False, left_contact=False, right_contact=False, grip_error_m=0.025,
    )
    r = diagnose_action(
        action="place", params={"target_color": "red", "destination_color": "blue"},
        before=before, after=after, result_ok=False,
        result_reason="carried red block was lost during delivery; aborted immediately",
    )
    assert "CARRY_CONTACT_TRANSIENT_AT_PLACE_ABORT" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_place_destination_visibility_loss_during_alignment_is_classified():
    before = base_state(
        red_xyz=[0.20, 0.0, 0.15], blue_xyz=[0.36, -0.11, 0.015],
        grasp_color="red", stable=True, lifted=True, bilateral_contact=True,
        left_contact=True, right_contact=True, grip_error_m=0.021,
    )
    after = base_state(
        red_xyz=[0.38, -0.11, 0.15], blue_xyz=[0.36, -0.11, 0.015],
        grasp_color="red", stable=True, lifted=True, bilateral_contact=True,
        left_contact=True, right_contact=True, grip_error_m=0.021,
    )
    r = diagnose_action(
        action="place", params={"target_color": "red", "destination_color": "blue"},
        before=before, after=after, result_ok=False,
        result_reason="blue target lost while body-centering",
        before_vision={"blue": {"visible": True}},
        after_vision={"blue": {"visible": False}},
    )
    assert "PLACE_DESTINATION_VISIBILITY_LOST_DURING_ALIGNMENT" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)
    assert "CARRIED_OBJECT_DROPPED_DURING_PLACE" not in codes(r)


def test_destination_search_true_drop_is_classified():
    before = base_state(
        red_xyz=[0.20, 0.0, 0.23], grasp_color="red", stable=True, lifted=True,
        bilateral_contact=True, left_contact=True, right_contact=True, grip_error_m=0.020,
    )
    # Reproduce the real failed trace: logical grasp identity can remain stale
    # for one observer snapshot even though the cube is >4 cm from the grip.
    after = base_state(
        red_xyz=[0.21, 0.0, 0.13], grasp_color="red", stable=False, lifted=True,
        bilateral_contact=False, left_contact=False, right_contact=False, grip_error_m=0.116,
    )
    r = diagnose_action(
        action="search_destination",
        params={"target_color": "red", "destination_color": "blue"},
        before=before, after=after, result_ok=False,
        result_reason="carried red block was lost during delivery; aborted immediately",
    )
    assert "CARRIED_OBJECT_DROPPED_DURING_DESTINATION_SEARCH" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)
    assert r["status"] == "BROKEN"


def test_destination_search_after_known_loss_is_classified():
    before = base_state(grasp_color=None, stable=False, lifted=False, bilateral_contact=False)
    after = base_state(grasp_color=None, stable=False, lifted=False, bilateral_contact=False)
    r = diagnose_action(
        action="search_destination",
        params={"target_color": "red", "destination_color": "yellow"},
        before=before, after=after, result_ok=False,
        result_reason="carried red block is not physically confirmed before destination search",
    )
    assert "DESTINATION_SEARCH_AFTER_CARRIED_OBJECT_LOSS" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)


def test_remote_worker_missing_runtime_module_is_classified():
    before = base_state()
    after = base_state()
    r = diagnose_action(
        action="move_forward", params={}, before=before, after=after,
        result_ok=False, result_reason="No module named 'scripts.robot_actions'",
    )
    assert "SIM_WORKER_RUNTIME_DEPENDENCY_MISSING" in codes(r)
    assert "ACTION_FAILED_UNCLASSIFIED" not in codes(r)
    assert r["status"] == "BROKEN"


# Keep these lightweight assertion-style checks runnable by both pytest and the
# repository's canonical `python -m unittest discover` command. No pytest-only
# feature is required here, so missing pytest must not silently drop 12 tests.
def load_tests(loader, tests, pattern):
    del loader, tests, pattern
    suite = unittest.TestSuite()
    for name in sorted(
        key for key, value in globals().items()
        if key.startswith("test_") and callable(value)
    ):
        suite.addTest(unittest.FunctionTestCase(globals()[name]))
    return suite
