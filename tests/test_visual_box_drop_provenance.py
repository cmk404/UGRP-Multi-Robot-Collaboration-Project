import base64
import hashlib
from unittest import mock

import cv2
import numpy as np

from harness.visual_box_skill import VisualBoxSkill
from harness.llm_transport_skill import LLMTransportSkill


POSE = {"1": 1500, "3": 694, "4": 1717, "5": 2220, "6": 1500}


def _jpeg():
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(frame, (180, 100), (610, 450), (200, 200, 0), -1)
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    return encoded.tobytes()


JPEG = _jpeg()


def _observation(frame, pan=1500):
    image = base64.b64encode(JPEG).decode()
    return {
        "robot_id": "r1", "frame_id": frame, "sim_time": frame / 10,
        "image": image, "sha256": hashlib.sha256(JPEG).hexdigest(),
        "camera": "robot_cam",
        "actuator_state": {"servo_pulses": {**POSE, "6": pan}},
    }


class _Tracker:
    def __init__(self, result):
        self.result = result

    def observe(self, _image):
        return dict(self.result)


def _surface(height=.02):
    return {
        "visible": True,
        "confidence": .55,
        "estimated_surface_patch_base_m": [.20, -.04, height + .016],
        "estimated_box_center_height_m": height,
        "provenance": "own_rgb_cyan_top_quad+test",
    }


def _attachment(attached=True):
    return {"attached": attached, "mask_iou": .99, "centroid_delta_px": 1.,
            "area_ratio": 1., "reason": "TEST"}


def _carrying_skill(tracker=None, task="external_navigation"):
    skill = VisualBoxSkill(perception_mode="fiducial", task=task)
    skill.phase = "carry"
    skill.held = True
    skill._grasp = {3: 900, 4: 1800, 5: 2300, 6: 1500}
    skill.tracker = tracker or _Tracker({"visible": False})
    skill._attachment_image = _observation(1)["image"]
    skill._carry_previous_image = skill._attachment_image
    return skill


def test_surface_low_height_requires_strict_pan_probe_and_pass_resumes_carry():
    skill = _carrying_skill(task="short_transfer")
    with (mock.patch("harness.visual_box_skill.observe_known_box_top", return_value=_surface()),
          mock.patch("harness.visual_box_skill.compare_box_comotion",
                     return_value=_attachment(True))):
        left = skill.decide(_observation(2))
        right = skill.decide(_observation(3, 1560))
        home = skill.decide(_observation(4, 1440))
        resumed = skill.decide(_observation(5))
        carrying = skill.decide(_observation(6))

    assert skill.last_target_provenance.startswith("surface_height:")
    assert left == {"kind": "pose", "pulses": {6: 1560, 1: 1500}}
    assert right == {"kind": "pose", "pulses": {6: 1440, 1: 1500}}
    assert home == {"kind": "pose", "pulses": {6: 1500, 1: 1500}}
    assert resumed == {"kind": "wait", "duration": .1}
    assert carrying == {"kind": "drive", "fwd": .12, "turn": 0., "duration": 1.}
    assert skill.phase == "carry"


def test_surface_low_height_probe_failure_keeps_existing_uncertain_drop_path():
    skill = _carrying_skill(task="short_transfer")
    results = [_attachment(True), _attachment(True), _attachment(True),
               _attachment(True), _attachment(False), _attachment(True)]
    with (mock.patch("harness.visual_box_skill.observe_known_box_top", return_value=_surface()),
          mock.patch("harness.visual_box_skill.compare_box_comotion", side_effect=results)):
        skill.decide(_observation(2))
        skill.decide(_observation(3, 1560))
        skill.decide(_observation(4, 1440))
        rejected = skill.decide(_observation(5))

    assert rejected == {"kind": "finish", "reason": "VISUAL_LOAD_DROPPED_OR_OCCLUDED"}


def test_external_navigation_surfaces_uncertainty_until_model_selects_check_grip():
    actor = LLMTransportSkill("r1", "small_box_01", "B")
    actor.state = "carrying"
    actor.box.phase = "carry"
    actor.box.held = True
    actor.box._grasp = {3: 900, 4: 1800, 5: 2300, 6: 1500}
    actor.box._attachment_image = _observation(1)["image"]
    actor.box._carry_previous_image = actor.box._attachment_image
    actor.box.tracker = _Tracker({"visible": False})

    with (mock.patch("harness.visual_box_skill.observe_known_box_top", return_value=_surface()),
          mock.patch("harness.visual_box_skill.compare_box_comotion",
                     return_value=_attachment(True))):
        # Monitoring reports uncertainty; it cannot autonomously move the arm.
        assert actor.advance(_observation(2)) is None
        assert actor.state == "grip_uncertain"
        assert actor.operation is None
        assert actor.last_guard_reason == "TOP_GEOMETRY_AMBIGUOUS_FOR_DROP"

        actor.request({"kind": "check_grip"})
        assert actor.state == "picking" and actor.operation == "pick"
        left = actor.advance(_observation(3))
        right = actor.advance(_observation(4, 1560))
        home = actor.advance(_observation(5, 1440))
        done = actor.advance(_observation(6))

    assert left == {"kind": "pose", "pulses": {6: 1560, 1: 1500}}
    assert right == {"kind": "pose", "pulses": {6: 1440, 1: 1500}}
    assert home == {"kind": "pose", "pulses": {6: 1500, 1: 1500}}
    assert done is None
    assert actor.state == "carrying" and actor.operation is None
    assert actor.box.phase == "carry"
    assert actor.box._surface_drop_probe_validated is True


def test_model_selected_surface_probe_failure_remains_strict():
    actor = LLMTransportSkill("r1", "small_box_01", "B")
    actor.state = "carrying"
    actor.box.phase = "carry"
    actor.box.held = True
    actor.box._grasp = {3: 900, 4: 1800, 5: 2300, 6: 1500}
    actor.box._attachment_image = _observation(1)["image"]
    actor.box._carry_previous_image = actor.box._attachment_image
    actor.box.tracker = _Tracker({"visible": False})

    with (mock.patch("harness.visual_box_skill.observe_known_box_top", return_value=_surface()),
          mock.patch("harness.visual_box_skill.compare_box_comotion",
                     return_value=_attachment(True))):
        actor.advance(_observation(2))
    actor.request({"kind": "check_grip"})
    # verify_lift, left endpoint, right endpoint plus a failing side-pair,
    # then home. The original home+side-pair conjunction rejects the probe.
    results = [_attachment(True), _attachment(True), _attachment(True),
               _attachment(False), _attachment(True)]
    with (mock.patch("harness.visual_box_skill.observe_known_box_top", return_value=_surface()),
          mock.patch("harness.visual_box_skill.compare_box_comotion", side_effect=results)):
        actor.advance(_observation(3))
        actor.advance(_observation(4, 1560))
        actor.advance(_observation(5, 1440))
        rejected = actor.advance(_observation(6))

    assert rejected == {"kind": "finish", "reason": "VISUAL_ATTACHMENT_UNCONFIRMED"}
    assert actor.state == "failure"


def test_marker_derived_low_height_still_aborts_immediately():
    tracker = _Tracker({
        "visible": True,
        "reprojection_rmse_px": 1.,
        "box_face_inset_camera_m": [0., 0., .2],
        "provenance": "raw_rgb+aruco_id_confirmed",
    })
    skill = _carrying_skill(tracker)
    with (mock.patch("harness.visual_box_skill.camera_to_base", return_value=np.asarray([.2, 0., .08])),
          mock.patch("harness.visual_box_skill.observe_known_box_top",
                     return_value={"visible": False}),
          mock.patch("harness.visual_box_skill.compare_box_comotion",
                     return_value=_attachment(True))):
        rejected = skill.decide(_observation(2))

    assert skill.last_target_provenance.startswith("marker_pose:")
    assert rejected == {"kind": "finish", "reason": "VISUAL_LOAD_DROPPED"}


def test_absent_cargo_attachment_failure_still_aborts():
    skill = _carrying_skill()
    with (mock.patch("harness.visual_box_skill.observe_known_box_top",
                     return_value={"visible": False}),
          mock.patch("harness.visual_box_skill.compare_box_comotion",
                     return_value=_attachment(False))):
        rejected = skill.decide(_observation(2))

    assert rejected == {"kind": "finish", "reason": "VISUAL_LOAD_DROPPED_OR_OCCLUDED"}
