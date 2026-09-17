"""Pure RGB/own-actuator state machine for the single-box skill demo."""

from __future__ import annotations

import base64
import hashlib
import math
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np

from harness.monocular_box import CameraBoxTracker
from harness.markerless_box import observe_ground_box
from harness.markerless_face import MarkerlessFaceAligner
from harness.approach_geometry import assess_face_standoff
from harness.visual_arm import camera_extrinsics, camera_to_base, forward_grip, solve_grip_ik, tool_pose
from harness.visual_floor import observe_zone
from harness.visual_box_surface import observe_known_box_top
from harness.visual_attachment import compare_box_comotion


SEARCH = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}


class VisualBoxSkill:
    """Choose bounded macros from camera-port observations only.

    The runner executes returned macros and owns physics.  This object retains
    only its tracker, own PWM/history, and derived RGB measurements; it has no
    world, robot, model, data, renderer, contact, constraint, or object-pose
    reference.
    """

    def __init__(self, task="short_transfer", destination_zone="B", robot_id="r1", cargo_id="small_box_01",
                 near_field_reacquisition=False, perception_mode="markerless",
                 attachment_home_reference="anchor", attachment_min_saturation=65, release_refine_ground_fit=False):
        if task not in {"short_transfer", "destination_zone", "external_navigation"}:
            raise ValueError("unsupported visual box task")
        if destination_zone not in {"A", "B", "C"}:
            raise ValueError("destination_zone must be A, B, or C")
        self.release_refine_ground_fit=bool(release_refine_ground_fit)
        self.task = task
        self.destination_zone = destination_zone
        self.robot_id = str(robot_id)
        self.cargo_id = cargo_id
        if not isinstance(near_field_reacquisition, bool):
            raise ValueError("near_field_reacquisition must be a bool")
        self.near_field_reacquisition = near_field_reacquisition
        if perception_mode not in {"markerless", "fiducial"}:
            raise ValueError("unsupported perception_mode")
        self.perception_mode = perception_mode
        if attachment_home_reference not in {'anchor', 'previous_endpoint'}:
            raise ValueError('unsupported attachment home reference')
        if attachment_min_saturation not in (65,150):raise ValueError('unsupported attachment calibration')
        self.attachment_min_saturation=attachment_min_saturation
        self.attachment_home_reference = attachment_home_reference
        self._probe_last_image = None
        self._probe_last_pan = None
        self.tracker = CameraBoxTracker(target_id=cargo_id) if perception_mode == "fiducial" else None
        self._face_aligner = MarkerlessFaceAligner() if perception_mode == "markerless" else None
        self.last_face_alignment = None
        self._face_alignment_waits = 0
        self._face_inspection_reached = False
        self.phase = "approach"
        self.reason = "RUNNING"
        self.held = False
        self.last_box: dict[str, Any] | None = None
        self.last_target: tuple[float, float, float] | None = None
        self.last_target_provenance: str | None = None
        self._last_seen_pose: dict[str, int] | None = None
        self._missing = 0
        self._face_approach = False
        self._carry_steps = 0
        self._lift_confirmations = 0
        self._carry_missing = 0
        self.last_surface = None
        self._release_path = []
        self._release_scan_attempts = 0
        self._release_ground_probe = []
        self._release_ground_origin_pan = None
        self._inspection_pose = None
        self._attachment_image = None
        self._attachment_pan = None
        self._carry_previous_image = None
        self._probe_results = []
        self._probe_side_image = None
        self._probe_side_pair = None
        self._probe_origin_phase = None
        self._surface_drop_probe_validated = False
        self.last_attachment = None
        self._grasp: dict[int, int] | None = None
        self._hover: dict[int, int] | None = None
        self._lower_path: list[dict[int, int]] = []
        self._lift_path: list[dict[int, int]] = []
        self._last_frame_id = 0
        self._last_sim_time = -math.inf
        self._hashes: list[str] = []
        self._history: list[dict[str, Any]] = []

    def _compare_attachment(self,*args,**kwargs):
        if self.attachment_min_saturation!=65:kwargs['min_saturation']=self.attachment_min_saturation
        return compare_box_comotion(*args,**kwargs)

    @property
    def history(self) -> tuple[dict[str, Any], ...]:
        """Bounded metadata history; images and privileged state are not kept."""
        return tuple(dict(item) for item in self._history)

    def decide(self, observation: Mapping[str, Any]) -> dict[str, Any]:
        obs, pose = self._validate_observation(observation)
        ground_phase = self.phase in {"approach", "verify_release", "release_ground_left", "release_ground_right", "release_ground_home"}
        if self.perception_mode == "markerless":
            # A floor hypothesis is only appropriate before pickup or after
            # opening/retracting. Never manufacture a ground-height estimate
            # while the object is carried.
            ground_options={"refine_position":True} if self.release_refine_ground_fit and self.phase!="approach" else {}
            # The dispatch cargo binding uses a stronger cyan cut than the
            # legacy room: blue-green floor pixels otherwise merge the released
            # box into a clipped background component. Projection, identity and
            # multi-view ground-stationarity gates remain unchanged.
            if self.phase!="approach" and self.attachment_min_saturation>65:
                ground_options["min_saturation"]=self.attachment_min_saturation
            box = (observe_ground_box(obs["image"], pose, self.cargo_id,**ground_options)
                   if ground_phase else {"visible": False, "reason": "GROUND_ESTIMATE_NOT_APPLICABLE_WHILE_HELD"})
        else:
            box = self.tracker.observe(obs["image"])
        target = None
        target_provenance = None
        if self.perception_mode == "markerless" and box.get("visible") is True:
            point = box.get("estimated_box_center_base_m")
            if isinstance(point, (list, tuple)) and len(point) == 3 and np.all(np.isfinite(point)):
                target = np.asarray(point, dtype=float)
                target_provenance = "ground_shape_fit:" + str(box.get("provenance", "unknown"))
        elif self.perception_mode == "fiducial" and box.get("visible") is True and _finite_or_inf(box.get("reprojection_rmse_px")) < 3.0:
            camera_point = box.get("box_face_inset_camera_m")
            if isinstance(camera_point, (list, tuple)) and len(camera_point) == 3:
                estimate = np.asarray(camera_to_base(camera_point, pose), dtype=float)
                estimate[2] -= 0.06
                if np.all(np.isfinite(estimate)):
                    target = estimate
                    target_provenance = "marker_pose:" + str(box.get("provenance", "unknown"))
        self.last_surface = None
        if self._grasp is not None and self.phase != "approach" and not (self.perception_mode == "markerless" and ground_phase):
            surface = observe_known_box_top(obs["image"], pose, target_id=self.cargo_id)
            self.last_surface = surface
            if target is None and surface.get("visible") and surface.get("confidence", 0) >= .5:
                target = np.asarray(surface["estimated_surface_patch_base_m"], dtype=float)
                target[2] = surface["estimated_box_center_height_m"]
                target_provenance = "surface_height:" + str(surface.get("provenance", "unknown"))
        self.last_box = dict(box)
        self.last_target = tuple(float(v) for v in target) if target is not None else None
        self.last_target_provenance = target_provenance

        if self.phase == "approach":
            if self._face_aligner is not None:
                # The 4 cm top is too small for reliable face orientation at
                # the initial half-metre range. First obtain a closer RGB
                # view while retaining the observed centre and drive guard.
                if target is not None and np.linalg.norm(target[:2]) <= .405:
                    self._face_inspection_reached = True
                if self._face_inspection_reached:
                    self.last_face_alignment = self._face_aligner.observe(
                        box, tuple(target[:2]) if target is not None else ())
                else:
                    self.last_face_alignment = {"ready": False,
                        "reason": "CLOSER_FACE_INSPECTION_REQUIRED", "normal_xy": None}
            return self._approach(box, target, pose)
        if self.phase == "lower":
            if self._lower_path:
                return _pose(self._lower_path.pop(0))
            self.phase = "close"
            return _pose({1: 1500})
        if self.phase == "close":
            self.phase = "lift"
            if self._lift_path:
                return _pose(self._lift_path.pop(0))
            return self._finish("MISSING_LIFT_PATH")
        if self.phase == "lift":
            if self._lift_path:
                return _pose(self._lift_path.pop(0))
            self.phase = "verify_lift"
            return _wait(0.05)
        if self.phase == "verify_lift":
            self.last_attachment = self._compare_attachment(obs["image"], obs["image"])
            if not self.last_attachment["attached"]:
                return self._finish("VISUAL_LIFT_UNCONFIRMED")
            self._attachment_image = obs["image"]
            self._attachment_pan = int(pose["6"])
            self._probe_results = []
            if not 560 <= self._attachment_pan <= 2440:
                return self._finish("ATTACHMENT_PROBE_PAN_LIMIT")
            self.phase = "attachment_left"
            return _pose({6: self._attachment_pan + 60, 1: 1500})
        if self.phase in {"attachment_left", "attachment_right", "attachment_home",
                          "carry_probe_left", "carry_probe_right", "carry_probe_home"}:
            pan_delta = int(pose["6"]) - int(self._attachment_pan)
            self.last_attachment = self._compare_attachment(
                self._attachment_image, obs["image"], camera_pan_delta_pwm=pan_delta)
            if (self.attachment_home_reference == 'previous_endpoint'
                    and self.phase in {'attachment_home','carry_probe_home'}):
                # All three fresh interventions must show comotion. Compare
                # home to the immediately preceding endpoint so slow grip
                # compliance does not accumulate across the entire sweep.
                # Opposite endpoints must still pass the independent 120-PWM
                # test below; a stationary floor box cannot pass that test.
                anchor_metrics = self.last_attachment
                self.last_attachment = self._compare_attachment(self._probe_last_image,
                    obs['image'], camera_pan_delta_pwm=int(pose['6'])-self._probe_last_pan)
                self.last_attachment['initial_anchor_metrics'] = anchor_metrics
            self._probe_results.append(bool(self.last_attachment["attached"]))
            is_carry_probe = self.phase.startswith("carry_probe")
            if self.phase == "attachment_left":
                self._probe_side_image = obs["image"]
                self.phase = "attachment_right"
                return _pose({6: self._attachment_pan - 60, 1: 1500})
            if self.phase == "carry_probe_left":
                self._probe_side_image = obs["image"]
                self.phase = "carry_probe_right"
                return _pose({6: self._attachment_pan - 60, 1: 1500})
            if self.phase == "attachment_right":
                self._probe_side_pair = self._compare_attachment(
                    self._probe_side_image, obs["image"], camera_pan_delta_pwm=-120)
                self.phase = "attachment_home"
                self._probe_last_image, self._probe_last_pan = obs['image'], int(pose['6'])
                return _pose({6: self._attachment_pan, 1: 1500})
            if self.phase == "carry_probe_right":
                self._probe_side_pair = self._compare_attachment(
                    self._probe_side_image, obs["image"], camera_pan_delta_pwm=-120)
                self.phase = "carry_probe_home"
                self._probe_last_image, self._probe_last_pan = obs['image'], int(pose['6'])
                return _pose({6: self._attachment_pan, 1: 1500})
            # Require the box to remain camera-relative across the complete
            # left-to-right sweep and to recover at home.  Either endpoint can
            # move more than its one-sided bound while a compliant held box
            # yaws, but a detached box traverses far across the two endpoints.
            self.last_attachment["side_pair_metrics"] = self._probe_side_pair
            sequential_failed = (self.attachment_home_reference == 'previous_endpoint'
                                 and not all(self._probe_results))
            if sequential_failed or not self.last_attachment["attached"] or not self._probe_side_pair["attached"]:
                return self._finish("VISUAL_LOAD_DROPPED_OR_OCCLUDED" if is_carry_probe
                                    else "VISUAL_ATTACHMENT_UNCONFIRMED")
            if self._probe_origin_phase == "surface_low_height":
                self._surface_drop_probe_validated = True
            self.held = True
            self._attachment_image = obs["image"]
            self._carry_previous_image = obs["image"]
            self.phase = "carry"
            self._probe_results = []
            self._probe_origin_phase = None
            return _wait(.1)
        if self.phase == "carry":
            return self._carry(obs, pose)
        if self.phase == "release":
            if self._release_path:
                return _pose(self._release_path.pop(0))
            self.phase = "open"
            return _pose(self._required_plan(self._grasp, "grasp"))
        if self.phase == "open":
            self.phase = "retract"
            return _pose({1: 2000})
        if self.phase == "retract":
            self.phase = "verify_release"
            return _pose({**self._inspection_pose, 1: 2000})
        if self.phase == "verify_release":
            if self.perception_mode == "markerless" and target is not None:
                # Ground-fitting one image is conditional on the floor model.
                # Also require the observed box to stay fixed in the base
                # frame across a controlled camera sweep after opening.
                expected = forward_grip(self._required_plan(self._grasp, "grasp"))
                if np.linalg.norm(np.asarray(target[:2])-np.asarray(expected[:2])) <= .045:
                    pan = int(pose["6"])
                    if not 560 <= pan <= 2440:
                        return self._finish("RELEASE_PROBE_PAN_LIMIT")
                    self._release_ground_origin_pan = pan
                    self._release_ground_probe = [tuple(target)]
                    self.phase = "release_ground_left"
                    return _pose({6: pan + 60})
                target = None
            if target is not None and -.01 <= target[2] <= .04:
                return self._finish("VISUAL_RELEASE_CONFIRMED")
            self._release_scan_attempts += 1
            if self._release_scan_attempts > 24:
                return self._finish("VISUAL_RELEASE_UNCONFIRMED")
            elbow = min(2500, int(pose["4"]) + 8)
            if elbow != int(pose["4"]) and self._release_scan_attempts <= 13:
                return _pose({4: elbow})
            offset = ((self._release_scan_attempts - 13) // 2 + 1) * 30
            offset *= -1 if self._release_scan_attempts % 2 else 1
            return _pose({6: _clip_int(self._inspection_pose[6] + offset, 500, 2500)})
        if self.phase in {"release_ground_left", "release_ground_right", "release_ground_home"}:
            expected_pan = self._release_ground_origin_pan + {
                "release_ground_left": 60, "release_ground_right": -60,
                "release_ground_home": 0}[self.phase]
            if abs(int(pose["6"]) - expected_pan) > 2:
                return self._finish("RELEASE_PROBE_POSE_UNCONFIRMED")
            if target is None:
                return self._finish("RELEASE_GROUND_TARGET_UNOBSERVABLE")
            self._release_ground_probe.append(tuple(target))
            # Check every pair, especially opposite sweep endpoints: a
            # camera-following object can otherwise sit within each origin
            # tolerance while traversing twice that amount left to right.
            if any(np.linalg.norm(np.asarray(target[:2])-np.asarray(prior[:2])) > .010
                   for prior in self._release_ground_probe[:-1]):
                return self._finish("RELEASE_OBJECT_NOT_GROUND_STATIONARY")
            if self.phase == "release_ground_left":
                self.phase = "release_ground_right"
                return _pose({6: self._release_ground_origin_pan - 60})
            if self.phase == "release_ground_right":
                self.phase = "release_ground_home"
                return _pose({6: self._release_ground_origin_pan})
            return self._finish("VISUAL_RELEASE_CONFIRMED")
        return {"kind": "finish", "reason": self.reason}

    def _approach(self, box, target, pose):
        if target is None:
            self._missing += 1
            if self._missing > 20:
                return self._finish("TARGET_NOT_VISIBLE")
            if self.last_box is not None and self._last_seen_pose is not None:
                delta = (1 if self._missing % 2 else -1) * 12 * ((self._missing + 1) // 2)
                wrist = int(pose["3"])
                new = _clip_int(int(self._last_seen_pose["3"]) + delta, 500, 2200)
                if new != wrist:
                    return _pose({3: new})
                return _pose({4: _clip_int(int(pose["4"]) - delta, 500, 2500)})
            return _drive(0.0, 0.12, 0.4)

        self._missing = 0
        self._last_seen_pose = dict(pose)
        x, y, _z = (float(v) for v in target)
        bearing = math.atan2(y, x)
        desired_pan = _clip_int(round(1500 + math.degrees(bearing) * 2000 / 180), 500, 2500)
        if abs(int(pose["6"]) - desired_pan) > 20:
            return _pose({6: desired_pan})
        centroid = box.get("pixel_centroid")
        if not isinstance(centroid, (list, tuple)) or len(centroid) != 2:
            return self._finish("INVALID_BOX_CENTROID")
        vertical_error = float(centroid[1]) - 218.7
        if abs(vertical_error) > (15 if x < 0.35 else 35):
            limit = 8 if x < 0.35 else 65
            delta = _clip_int(round(-vertical_error * 0.65), -limit, limit)
            wrist = int(pose["3"])
            adjusted = _clip_int(wrist + delta, 500, 2200)
            if adjusted != wrist:
                return _pose({3: adjusted})
            return _pose({4: _clip_int(int(pose["4"]) - delta, 500, 2500)})
        if self.perception_mode == "markerless" and not self._face_inspection_reached:
            if abs(bearing) > .05:
                return _drive(0.0, float(np.clip(bearing * .6, -.18, .18)), .4)
            return _drive(.10, 0.0, .6)
        if not self._face_approach:
            if self.perception_mode == "markerless":
                alignment = self.last_face_alignment or {}
                if not alignment.get("ready"):
                    self._face_alignment_waits += 1
                    if self._face_alignment_waits > 20:
                        return self._finish("BOX_FACE_ALIGNMENT_UNOBSERVABLE")
                    return _wait(.1)
                self._face_alignment_waits = 0
                normal = np.asarray([*alignment["normal_xy"], 0.0], dtype=float)
            else:
                marker_pose = box.get("marker_pose_camera")
                rvec = marker_pose.get("rotation_rvec_rad") if isinstance(marker_pose, Mapping) else None
                if not isinstance(rvec, (list, tuple)) or len(rvec) != 3:
                    return self._finish("INVALID_MARKER_ROTATION")
                rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
                normal = np.asarray(camera_extrinsics(pose)[1], dtype=float).T @ rotation[:, 2]
            norm = float(np.linalg.norm(normal[:2]))
            if norm <= 1e-9 or not math.isfinite(norm):
                return self._finish("INVALID_MARKER_NORMAL")
            # An explicitly requested post-release recovery may start with the
            # box already inside the calibrated grasp envelope.  In that one
            # case, a front-facing marker is direct RGB evidence that the
            # usual 35 cm face standoff lies behind the chassis.  Keep the
            # normal route for every ordinary acquisition and for oblique,
            # off-axis, or out-of-range reacquisitions.
            radial = float(np.linalg.norm(target[:2]))
            toward_chassis = -target[:2] / max(radial, 1e-9)
            face_alignment = float(np.dot(normal[:2] / norm, toward_chassis))
            if (self.near_field_reacquisition and 0.145 <= radial <= 0.20
                    and abs(bearing) <= 0.10
                    and face_alignment >= math.cos(math.radians(15.0))):
                self._face_approach = True
                return _wait(0.05)
            # A usable face standoff can be reached before the exact derived
            # waypoint crosses the chassis origin. Continuing tangent travel
            # there can push the marker beyond the wrist pan range. Finish
            # only this subgoal; the normal target-bearing and IK checks below
            # still govern the final approach and grasp.
            if assess_face_standoff(target[:2], normal[:2]).reached:
                self._face_approach = True
                return _wait(0.05)
            waypoint = target[:2] + normal[:2] / norm * 0.35
            if float(np.linalg.norm(waypoint)) < 0.055:
                self._face_approach = True
                return _wait(0.05)
            heading = math.atan2(float(waypoint[1]), float(waypoint[0]))
            if abs(heading) > 0.06:
                return _drive(0.0, float(np.clip(heading * 0.6, -0.18, 0.18)), 0.4)
            return _drive(0.12, 0.0, 0.6 if np.linalg.norm(waypoint) > 0.2 else 0.3)
        if abs(bearing) > (0.10 if x < 0.32 else 0.05):
            return _drive(0.0, float(np.clip(bearing * 0.6, -0.18, 0.18)), 0.4)
        if float(np.linalg.norm(target[:2])) > 0.168:
            return _drive(0.15 if x > 0.3 else 0.08, 0.0, 1.0 if x > 0.35 else 0.3)
        if x < 0.145:
            return self._finish("APPROACH_OVERSHOT")
        try:
            self._inspection_pose = {int(key): value for key, value in pose.items()}
            self._grasp = solve_grip_ik(x, y, 0.024, -90)
            grasp_pitch = tool_pose(self._grasp).pitch_deg
            self._hover = solve_grip_ik(x, y, 0.095, grasp_pitch)
            down_heights = np.linspace(0.095, 0.024, 16)[1:]
            up_heights = np.linspace(0.024, 0.095, 16)[1:]
            self._lower_path = [solve_grip_ik(x, y, float(height), grasp_pitch) for height in down_heights]
            self._release_path = [dict(item) for item in self._lower_path]
            self._lift_path = [
                {**solve_grip_ik(x, y, float(height), grasp_pitch), 1: 1500}
                for height in up_heights
            ]
        except (ValueError, RuntimeError) as exc:
            return self._finish(f"IK_UNAVAILABLE:{exc}")
        self.phase = "lower"
        return _pose({**self._hover, 1: 2000})

    def _carry(self, obs, pose):
        # During travel, track between fresh frames while retaining an anchor.
        # Small contact compliance or illumination drift is not itself a drop.
        self.last_attachment = self._compare_attachment(self._carry_previous_image, obs["image"])
        anchor = self._compare_attachment(self._attachment_image, obs["image"])
        self.last_attachment["carry_anchor_metrics"] = anchor
        if not self.last_attachment["attached"]:
            if self.task == "external_navigation":
                return self._finish("VISUAL_LOAD_DROPPED_OR_OCCLUDED")
            # Base motion and contact compliance can create one ambiguous
            # frame.  Stop and run the same own-PWM camera probe used after
            # lifting instead of declaring a drop from that single sample.
            self._attachment_image = obs["image"]
            self._attachment_pan = int(pose["6"])
            if not 560 <= self._attachment_pan <= 2440:
                return self._finish("ATTACHMENT_PROBE_PAN_LIMIT")
            self._probe_results = []
            self._probe_origin_phase = "attachment_change"
            self.phase = "carry_probe_left"
            return _pose({6: self._attachment_pan + 60, 1: 1500})
        if (anchor.get("mask_iou", 0) < .80
                or anchor.get("centroid_delta_px", math.inf) > 25.6
                or not .8 <= anchor.get("area_ratio", 0) <= 1.25):
            return self._finish("VISUAL_GRASP_DRIFT")
        low_target = self.last_target is not None and self.last_target[2] < .035
        marker_height = bool(self.last_target_provenance
                             and self.last_target_provenance.startswith("marker_pose:"))
        surface_height = bool(self.last_target_provenance
                              and self.last_target_provenance.startswith("surface_height:"))
        if low_target and marker_height:
            return self._finish("VISUAL_LOAD_DROPPED")
        if low_target and surface_height and not self._surface_drop_probe_validated:
            # A cyan surface is not fresh identity evidence and a partial face
            # can satisfy the upright-top fit at extreme close range.  Stop
            # chassis motion and use the existing strict left/right/home
            # camera-relative attachment intervention before declaring loss.
            self._attachment_pan = int(pose["6"])
            if not 560 <= self._attachment_pan <= 2440:
                return self._finish("ATTACHMENT_PROBE_PAN_LIMIT")
            self._probe_results = []
            self._probe_origin_phase = "surface_low_height"
            if self.task == "external_navigation":
                # The external planner must explicitly select check_grip;
                # never execute an arm intervention as a navigation guard.
                return self._finish("TOP_GEOMETRY_AMBIGUOUS_FOR_DROP")
            self.phase = "carry_probe_left"
            return _pose({6: self._attachment_pan + 60, 1: 1500})
        if not low_target:
            self._surface_drop_probe_validated = False
        self._carry_previous_image = obs["image"]
        if self.task == "external_navigation":
            return _wait(0.05)
        if self.task == "short_transfer":
            if self._carry_steps >= 10:
                self.phase = "release"
                return _wait(0.05)
            self._carry_steps += 1
            return _drive(0.12, 0.0, 1.0)
        zone = observe_zone(obs["image"], self.destination_zone, pose, camera_to_base)
        if zone.get("visible") is not True:
            return _drive(0.0, 0.15, 0.6)
        goal = zone.get("estimated_base_m")
        if not isinstance(goal, (list, tuple)) or len(goal) < 2:
            return self._finish("INVALID_ZONE_ESTIMATE")
        gx, gy = float(goal[0]), float(goal[1])
        if not math.isfinite(gx) or not math.isfinite(gy):
            return self._finish("INVALID_ZONE_ESTIMATE")
        angle = math.atan2(gy, gx)
        if abs(angle) > 0.12:
            return _drive(0.0, float(np.clip(angle * 0.6, -0.18, 0.18)), 0.5)
        if gx > 0.18:
            return _drive(0.12, 0.0, 0.8)
        self.phase = "release"
        return _wait(0.05)

    def _validate_observation(self, observation):
        if not isinstance(observation, Mapping):
            raise ValueError("observation must be an object")
        allowed = {"robot_id", "frame_id", "sim_time", "image", "sha256", "camera", "actuator_state"}
        if set(observation) != allowed:
            raise ValueError("observation fields do not match camera-port schema")
        if observation["robot_id"] != self.robot_id or observation["camera"] != "robot_cam":
            raise ValueError("observation belongs to another sensor")
        frame = observation["frame_id"]
        now = observation["sim_time"]
        if isinstance(frame, bool) or not isinstance(frame, int) or frame <= self._last_frame_id:
            raise ValueError("stale or invalid frame_id")
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(float(now)) or float(now) < self._last_sim_time:
            raise ValueError("stale or invalid sim_time")
        image = observation["image"]
        digest = observation["sha256"]
        if not isinstance(image, str) or not isinstance(digest, str):
            raise ValueError("invalid image/hash")
        try:
            payload = base64.b64decode(image, validate=True)
        except Exception as exc:
            raise ValueError("invalid base64 JPEG") from exc
        if not payload or hashlib.sha256(payload).hexdigest() != digest:
            raise ValueError("image hash mismatch")
        state = observation["actuator_state"]
        pose = state.get("servo_pulses") if isinstance(state, Mapping) else None
        if not isinstance(pose, Mapping):
            raise ValueError("missing own servo PWM state")
        clean = {}
        for servo in (1, 3, 4, 5, 6):
            value = pose.get(str(servo))
            if isinstance(value, bool) or not isinstance(value, int) or not 500 <= value <= 2500:
                raise ValueError(f"invalid own servo {servo} PWM")
            clean[str(servo)] = value
        self._last_frame_id = frame
        self._last_sim_time = float(now)
        self._hashes.append(digest)
        self._hashes = self._hashes[-40:]
        self._history.append(
            {"frame_id": frame, "sim_time": float(now), "sha256": digest, "phase": self.phase}
        )
        self._history = self._history[-40:]
        return observation, clean

    def _finish(self, reason):
        if reason == "VISUAL_RELEASE_CONFIRMED":
            self.held = False
        self.phase = "finished"
        self.reason = str(reason)
        return {"kind": "finish", "reason": self.reason}

    @staticmethod
    def _required_plan(plan, name):
        if plan is None:
            raise RuntimeError(f"missing {name} plan")
        return plan


def _pose(pulses):
    clean = {int(servo): int(pulse) for servo, pulse in pulses.items()}
    if any(servo not in {1, 3, 4, 5, 6} or not 500 <= pulse <= 2500 for servo, pulse in clean.items()):
        raise ValueError("unsafe pose macro")
    return {"kind": "pose", "pulses": clean}


def _drive(fwd, turn, duration):
    values = (float(fwd), float(turn), float(duration))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite drive macro")
    if not 0 <= values[0] <= 0.15 or not -0.2 <= values[1] <= 0.2 or not 0 <= values[2] <= 1:
        raise ValueError("unsafe drive macro")
    return {"kind": "drive", "fwd": values[0], "turn": values[1], "duration": values[2]}


def _wait(duration):
    value = float(duration)
    if not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError("unsafe wait macro")
    return {"kind": "wait", "duration": value}


def _clip_int(value, lower, upper):
    return int(max(lower, min(upper, int(value))))


def _finite_or_inf(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return math.inf
    return float(value) if math.isfinite(float(value)) else math.inf
