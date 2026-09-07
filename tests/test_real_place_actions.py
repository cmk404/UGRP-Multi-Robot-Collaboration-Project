from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
RED_BLOCK = ROOT / "scripts" / "red_block"
if str(RED_BLOCK) not in sys.path:
    sys.path.insert(0, str(RED_BLOCK))

from harness.executive import TaskExecutive
from harness.goals import GoalSpec
from harness.state import StateEstimator
from scripts import robot_actions
from geometry import BlockEstimate, GRIPPER_LINK, calculate_place_plan, forward_kinematics
from pick import _matches_pickup_site
import place
from place import DELIVERY_SCAN_POSE, DELIVERY_TILT_MAX, DELIVERY_TILT_MIN, _delivery_pose


class RealExecutivePlanTests(unittest.TestCase):
    def test_real_place_does_not_require_fake_map_or_legacy_carry(self):
        goal = GoalSpec("PLACE", "빨간 블록을 파란 블록 위에", "RED_BLOCK", "BLUE_BLOCK")
        plan = TaskExecutive().plan(goal, list(robot_actions.ACTIONS))
        self.assertEqual(plan, ["search", "track", "approach", "pick", "search_destination", "place"])

    def test_real_lift_ends_at_pick_because_pick_already_lifts(self):
        goal = GoalSpec("LIFT", "빨간 블록 들어줘", "RED_BLOCK", "UNKNOWN")
        plan = TaskExecutive().plan(goal, list(robot_actions.ACTIONS))
        self.assertEqual(plan, ["search", "track", "approach", "pick"])

    def test_sim_plan_keeps_map_and_carry_when_adapter_supports_them(self):
        tools = [
            "map_yellow", "search", "track", "approach", "pick", "carry", "place_on_yellow"
        ]
        goal = GoalSpec("PLACE", "빨간 블록을 노란 블록 위에", "RED_BLOCK", "YELLOW_BLOCK")
        plan = TaskExecutive().plan(goal, tools)
        self.assertEqual(
            plan,
            ["map_yellow", "search", "track", "approach", "pick", "carry", "place_on_yellow"],
        )

    def test_pick_rejects_unknown_visibility(self):
        state = StateEstimator().state
        decision = TaskExecutive(strict_pick_preconditions=True).check("pick", state)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.failure_code, "TARGET_NOT_VISIBLE")
        self.assertEqual(decision.recovery, "search")

    def test_pick_requires_successful_handoff_then_range_and_center_confirmation(self):
        state = StateEstimator().state
        state.target.visible = True
        state.target.centered = True
        state.target.range_class = "UNKNOWN"
        decision = TaskExecutive(strict_pick_preconditions=True).check("pick", state)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.failure_code, "PREGRASP_PLAN_MISSING")
        self.assertEqual(decision.recovery, "approach")

        # Once a successful approach has produced the metric/FK-IK handoff,
        # range and centering remain independent destructive-action gates.
        state.task.phase = "PREGRASP_READY"
        decision = TaskExecutive(strict_pick_preconditions=True).check("pick", state)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.failure_code, "TARGET_TOO_FAR")
        self.assertEqual(decision.recovery, "approach")
        state.target.range_class = "PREGRASP"
        state.target.centered = None
        decision = TaskExecutive(strict_pick_preconditions=True).check("pick", state)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.failure_code, "TARGET_NOT_CENTERED")
        self.assertEqual(decision.recovery, "track")


class RealGraspEvidenceTests(unittest.TestCase):
    def test_camera_verified_pick_becomes_held(self):
        est = StateEstimator()
        est.update_tool_result({
            "result": {
                "skill": "pick",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "grasp_verified": True,
                "verification_source": "camera_pickup_site_disappearance",
            }
        })
        self.assertEqual(est.state.grasp.state, "HELD")
        self.assertGreaterEqual(est.state.grasp.confidence, 0.95)

    def test_failed_pick_verification_becomes_empty(self):
        est = StateEstimator()
        est.update_tool_result({
            "result": {
                "skill": "pick",
                "command_status": "ACCEPTED",
                "execution_status": "FAILED",
                "outcome_status": "NOT_ACHIEVED",
                "grasp_verified": False,
            }
        })
        self.assertEqual(est.state.grasp.state, "EMPTY")

    def test_motor_only_pick_still_does_not_claim_grasp(self):
        est = StateEstimator()
        est.update_tool_result({
            "result": {
                "skill": "pick",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "UNKNOWN",
            }
        })
        self.assertEqual(est.state.grasp.state, "UNKNOWN")

    def test_bare_achieved_place_does_not_create_placed_truth(self):
        est = StateEstimator()
        est.state.grasp.state = "HELD"
        est.update_tool_result({
            "result": {
                "skill": "place_on_blue",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
            }
        })
        self.assertEqual(est.state.grasp.state, "HELD")
        self.assertNotEqual(est.state.task.phase, "PLACED")

    def test_verified_place_creates_placed_truth(self):
        est = StateEstimator()
        est.state.grasp.state = "HELD"
        est.update_tool_result({
            "result": {
                "skill": "place_on_blue",
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
                "outcome_status": "ACHIEVED",
                "place_verified": True,
            }
        })
        self.assertEqual(est.state.grasp.state, "EMPTY")
        self.assertEqual(est.state.task.phase, "PLACED")


class RobotActionBoundaryTests(unittest.TestCase):
    def test_generic_runtime_colors_are_forwarded_without_alias_tools(self):
        with patch.object(robot_actions, "main", return_value=0) as main:
            result = robot_actions.run(
                "place", target_color="blue", destination_color="yellow",
            )
        self.assertEqual(main.call_args.args[0], [
            "place", "--target-color", "blue", "--destination-color", "yellow",
        ])
        self.assertEqual(result["target_color"], "blue")
        self.assertEqual(result["destination_color"], "yellow")
        self.assertTrue(result["place_verified"])

    def test_destination_search_routes_through_place_locate_only(self):
        with patch.object(robot_actions, "main", return_value=0) as main:
            result = robot_actions.run(
                "search_destination", target_color="red", destination_color="blue",
            )
        self.assertEqual(main.call_args.args[0], [
            "search_destination", "--target-color", "red", "--destination-color", "blue",
        ])
        self.assertTrue(result["destination_visible"])
        self.assertEqual(result["destination_color"], "blue")

    def test_real_pick_exit_zero_is_probable_not_verified(self):
        with patch.object(robot_actions, "main", return_value=0):
            result = robot_actions.run("pick")
        self.assertEqual(result["outcome_status"], "UNKNOWN")
        self.assertNotIn("grasp_verified", result)
        self.assertTrue(result["visual_hold"]["probable"])
        self.assertLess(result["visual_hold"]["confidence"], .85)
        self.assertEqual(result["verification_source"], "camera_floor_clear_probable_only")

    def test_verified_track_exit_zero_reports_centered_camera_evidence(self):
        with patch.object(robot_actions, "main", return_value=0):
            result = robot_actions.run("track")
        self.assertEqual(result["outcome_status"], "ACHIEVED")
        self.assertIs(result["center_verified"], True)
        self.assertEqual(result["camera_red"], {"visible": True})
        self.assertEqual(result["verification_source"], "pi_multiframe_centered_track")

    def test_approach_exit_zero_reports_camera_verified_pregrasp(self):
        with patch.object(robot_actions, "main", return_value=0):
            result = robot_actions.run("approach")
        self.assertEqual(result["outcome_status"], "ACHIEVED")
        self.assertTrue(result["range_verified"])
        self.assertEqual(result["verification_source"], "camera_metric_arm_reach_controller")
        self.assertIs(result["center_verified"], True)
        self.assertEqual(result["camera_red"], {"visible": True})

    def test_place_exit_zero_is_structured_achieved(self):
        with patch.object(robot_actions, "main", return_value=0):
            result = robot_actions.run("place_on_yellow")
        self.assertEqual(result["outcome_status"], "ACHIEVED")
        self.assertTrue(result["place_verified"])

    def test_dry_run_never_claims_verified_physical_outcome(self):
        with patch.object(robot_actions, "main", return_value=0):
            pick = robot_actions.run("pick", extra=["--dry-run"])
            place = robot_actions.run("place_on_blue", extra=["--dry-run"])
        self.assertEqual(pick["outcome_status"], "UNKNOWN")
        self.assertNotIn("grasp_verified", pick)
        self.assertEqual(place["outcome_status"], "UNKNOWN")
        self.assertNotIn("place_verified", place)

    def test_pick_detect_only_never_claims_grasp(self):
        with patch.object(robot_actions, "main", return_value=0):
            result = robot_actions.run("pick", extra=["--detect-only"])
        self.assertEqual(result["outcome_status"], "UNKNOWN")
        self.assertNotIn("grasp_verified", result)

    def test_failed_remote_skill_surfaces_actual_reason(self):
        def fail_with_reason(argv):
            robot_actions.LAST_REMOTE_ERROR_PATH.write_text(
                "could not bring red block into far/close camera overlap\n",
                encoding="utf-8",
            )
            return 1
        with patch.object(robot_actions, "main", side_effect=fail_with_reason):
            result = robot_actions.run("approach")
        self.assertEqual(result["outcome_status"], "NOT_ACHIEVED")
        self.assertIn("far/close camera overlap", result["reason"])

    def test_lost_pick_target_surfaces_search_recovery(self):
        def fail_with_reason(argv):
            robot_actions.LAST_REMOTE_ERROR_PATH.write_text(
                "target disappeared during final horizontal alignment\n", encoding="utf-8"
            )
            return 1
        with patch.object(robot_actions, "main", side_effect=fail_with_reason):
            result = robot_actions.run("pick")
        self.assertEqual(result["failure_code"], "TARGET_NOT_VISIBLE")
        self.assertEqual(result["recommended_recovery"], "search")
        self.assertEqual(result["required_state"], "target.visible=true")

    def test_visually_proven_pick_miss_surfaces_regrasp_code(self):
        def fail_with_reason(argv):
            robot_actions.LAST_REMOTE_ERROR_PATH.write_text(
                "red block is still visible after the grasp; refusing an automatic closer retry\n",
                encoding="utf-8",
            )
            return 1
        with patch.object(robot_actions, "main", side_effect=fail_with_reason):
            result = robot_actions.run("pick")
        self.assertEqual(result["failure_code"], "GRASP_NOT_ACQUIRED")
        self.assertIsNone(result["recommended_recovery"])
        self.assertNotIn("grasp_verified", result)

    def test_stale_pick_handoff_surfaces_approach_recovery(self):
        def fail_with_reason(argv):
            robot_actions.LAST_REMOTE_ERROR_PATH.write_text(
                "precision pick plan is stale (61.0s); run approach immediately before pick\n",
                encoding="utf-8",
            )
            return 1
        with patch.object(robot_actions, "main", side_effect=fail_with_reason):
            result = robot_actions.run("pick")
        self.assertEqual(result["failure_code"], "PREGRASP_PLAN_MISSING")
        self.assertEqual(result["recommended_recovery"], "approach")

    def test_failed_pick_does_not_invent_empty_gripper_evidence(self):
        with patch.object(robot_actions, "main", return_value=1):
            result = robot_actions.run("pick")
        self.assertEqual(result["outcome_status"], "NOT_ACHIEVED")
        self.assertNotIn("grasp_verified", result)

    def test_child_skill_runs_in_fresh_python_process(self):
        proc = SimpleNamespace(wait=Mock(return_value=0), poll=Mock(return_value=None), pid=43210)
        with patch.object(robot_actions.subprocess, "Popen", return_value=proc) as popen:
            code = robot_actions.main(["pick"])
        self.assertEqual(code, 0)
        argv = popen.call_args.args[0]
        expected_python = robot_actions.SKILL_PYTHON if robot_actions.SKILL_PYTHON.is_file() else Path(sys.executable)
        self.assertEqual(argv[0], str(expected_python))
        self.assertTrue(argv[1].endswith("scripts/red_block/pick.py"))
        self.assertTrue(popen.call_args.kwargs["start_new_session"])

    def test_child_process_failure_is_converted_to_return_code(self):
        proc = SimpleNamespace(wait=Mock(return_value=7), poll=Mock(return_value=None), pid=43210)
        with patch.object(robot_actions.subprocess, "Popen", return_value=proc):
            code = robot_actions.main(["pick"])
        self.assertEqual(code, 7)

    def test_cancel_current_terminates_only_private_action_process_group(self):
        proc = SimpleNamespace(wait=Mock(return_value=-15), poll=Mock(return_value=None), pid=43210)
        with robot_actions._CURRENT_PROCESS_LOCK:
            robot_actions._CURRENT_PROCESS = proc
        try:
            with patch.object(robot_actions.os, "killpg") as killpg:
                self.assertTrue(robot_actions.cancel_current())
            killpg.assert_called_once_with(43210, robot_actions.signal.SIGTERM)
            proc.wait.assert_called_once_with(timeout=robot_actions.CANCEL_GRACE_SECONDS)
        finally:
            with robot_actions._CURRENT_PROCESS_LOCK:
                robot_actions._CURRENT_PROCESS = None


class CarriedArmTransitionTests(unittest.TestCase):
    def test_place_arm_transition_batches_345_together(self):
        class R:
            dry_run = True
            pose = {1: 1500, 3: 600, 4: 2200, 5: 1650, 6: 1450}
            def __init__(self): self.calls=[]
            def nudge_servos(self, updates, duration=0.1):
                self.calls.append((dict(updates), duration)); self.pose.update(updates)
        r=R()
        place.move_closed_grasp_arm_together(r, {3:771,4:1758,5:2080,6:1432,1:1500}, duration=1.1)
        self.assertEqual(r.calls, [({3:771,4:1758,5:2080}, 1.1)])
        self.assertEqual(r.pose[1], 1500)
        self.assertEqual(r.pose[6], 1450)

    def test_place_arm_transition_rejects_open_gripper(self):
        class R:
            dry_run = True
            pose = {1: 2000}
            def nudge_servos(self,*a,**k): raise AssertionError('must not move')
        with self.assertRaises(place.PlaceError):
            place.move_closed_grasp_arm_together(R(), {3:771,4:1758,5:2080}, duration=1.1)


class CarryNearViewTests(unittest.TestCase):
    def test_near_view_rejects_destination_that_collapses_to_self_blob(self):
        class R:
            dry_run = True
            pose = {1: 1500, 3: 740, 4: 2200, 5: place.CARRY_TRANSPORT_WRIST_PULSE, 6: 1500}
            def move_servo(self, servo, pulse, duration):
                self.pose[servo] = pulse

        # Equivalent to the failing actor-camera transition: a normal square
        # yellow target is followed after the near-view move by a thin,
        # bottom-right self-colour component.  It must abort before any
        # placement-arm move or gripper release can be reached.
        square_target = SimpleNamespace(nx=.50, ny=.70, area=11649, width=118, height=120)
        self_blob = SimpleNamespace(nx=.85, ny=.908, area=1040, width=67, height=25)
        frame = object()
        with patch.object(place, "_detect", side_effect=[(frame, square_target), (frame, self_blob)]):
            with self.assertRaisesRegex(place.PlaceError, "observation became inconsistent"):
                place.center_target(R(), "yellow", confirmations=1)

    def test_near_view_accepts_a_larger_consistent_destination(self):
        before = SimpleNamespace(area=11649, width=118, height=120)
        after = SimpleNamespace(area=26346, width=260, height=145)
        self.assertTrue(place.destination_observation_consistent(before, after))

    def test_near_delivery_view_moves_only_as_far_as_needed(self):
        class R:
            dry_run = True
            pose = {1: 1500, 3: 600, 4: 2200, 5: 1400, 6: 1468}
            def __init__(self): self.calls=[]
            def move_servo(self, servo, pulse, duration):
                self.calls.append((servo, pulse, duration)); self.pose[servo] = pulse
        r=R(); blob=SimpleNamespace(ny=0.70, width=150, height=100)
        self.assertTrue(place.maybe_enter_near_delivery_view(r, blob))
        self.assertEqual(r.calls, [(5, place.CARRY_NEAR_VIEW_WRIST_PULSE, place.CARRY_NEAR_VIEW_DURATION)])
        self.assertEqual(r.pose[3], 600); self.assertEqual(r.pose[4], 2200); self.assertEqual(r.pose[6], 1468)

    def test_clipped_near_view_steps_to_close_view(self):
        class R:
            dry_run = True
            pose = {1: 1500, 3: 600, 4: 2200, 5: place.CARRY_NEAR_VIEW_WRIST_PULSE, 6: 1468}
            def __init__(self): self.calls=[]
            def move_servo(self, servo, pulse, duration):
                self.calls.append((servo, pulse, duration)); self.pose[servo] = pulse
        r=R(); blob=SimpleNamespace(ny=0.84, width=160, height=110)
        self.assertTrue(place.maybe_enter_near_delivery_view(r, blob))
        self.assertEqual(r.calls, [(5, place.CARRY_CLOSE_VIEW_WRIST_PULSE, place.CARRY_CLOSE_VIEW_DURATION)])

    def test_low_view_measurement_uses_single_sample(self):
        class R:
            dry_run = True
            pose = {1:1500,3:600,4:2200,5:place.CARRY_NEAR_VIEW_WRIST_PULSE,6:1500}
            def stop(self): pass
        blob = SimpleNamespace(cx=320,cy=240,area=10000,width=100,height=100,nx=0.5,ny=0.5)
        estimate = BlockEstimate(radius_cm=17.0,lateral_left_cm=0.0,forward_cm=17.0,yaw_left_deg=0.0,camera_radius_cm=8.0,camera_height_cm=16.0,ray_pitch_deg=-60.0,block_height_cm=3.0,nx=0.5,ny=0.5)
        calls=[]
        with patch.object(place, '_detect', side_effect=lambda color: (object(), blob)) as detect, \
             patch.object(place, 'estimate_block', side_effect=lambda pose, b: (calls.append(1) or estimate)), \
             patch.object(place, 'save_debug_frame'):
            out=place.stationary_measurement(R(),'blue')
        self.assertEqual(out.radius_cm,17.0)
        self.assertEqual(len(calls),1)
        self.assertEqual(detect.call_count,1)

    def test_transport_wrist_restores_before_chassis_motion(self):
        class R:
            dry_run = True
            pose = {1:1500,5:place.CARRY_CLOSE_VIEW_WRIST_PULSE}
            def __init__(self): self.calls=[]
            def move_servo(self, servo, pulse, duration): self.calls.append((servo,pulse,duration)); self.pose[servo]=pulse
        r=R(); self.assertTrue(place.prepare_carry_chassis_motion(r))
        self.assertEqual(r.pose[5], place.CARRY_TRANSPORT_WRIST_PULSE)
        self.assertEqual(r.calls[0][0:2], (5, place.CARRY_TRANSPORT_WRIST_PULSE))

    def test_post_motion_view_restores_prior_near_or_close_class_only(self):
        class R:
            dry_run = True
            pose = {1:1500,5:place.CARRY_TRANSPORT_WRIST_PULSE}
            def __init__(self): self.calls=[]
            def move_servo(self, servo, pulse, duration): self.calls.append((servo,pulse,duration)); self.pose[servo]=pulse
        r=R()
        self.assertTrue(place.restore_post_motion_delivery_view(r, place.CARRY_NEAR_VIEW_WRIST_PULSE))
        self.assertEqual(r.pose[5], place.CARRY_NEAR_VIEW_WRIST_PULSE)
        r.pose[5] = place.CARRY_TRANSPORT_WRIST_PULSE
        self.assertTrue(place.restore_post_motion_delivery_view(r, place.CARRY_CLOSE_VIEW_WRIST_PULSE))
        self.assertEqual(r.pose[5], place.CARRY_CLOSE_VIEW_WRIST_PULSE)
        r.pose[5] = place.CARRY_TRANSPORT_WRIST_PULSE
        self.assertFalse(place.restore_post_motion_delivery_view(r, place.CARRY_TRANSPORT_WRIST_PULSE + 15))
        self.assertEqual(r.pose[5], place.CARRY_TRANSPORT_WRIST_PULSE)

    def test_body_turn_restores_near_view_before_visual_reacquisition(self):
        class R:
            dry_run = True
            pose = {1:1500,3:600,4:2200,5:place.CARRY_NEAR_VIEW_WRIST_PULSE,6:1450}
            def __init__(self): self.events=[]
            def stop(self): self.events.append(("stop", self.pose[5]))
            def move_servo(self, servo, pulse, duration):
                self.events.append(("servo", servo, pulse)); self.pose[servo]=pulse
            def drive(self, direction, speed, duration):
                self.events.append(("drive", direction, self.pose[5]))
                self.assert_transport = self.pose[5] == place.CARRY_TRANSPORT_WRIST_PULSE
        r=R()
        calls=[]
        blob=SimpleNamespace(nx=0.45, ny=0.70, width=100, height=80)
        def fake_center(robot, color):
            calls.append(robot.pose[5])
            if len(calls) == 2:
                self.assertEqual(robot.pose[5], place.CARRY_NEAR_VIEW_WRIST_PULSE)
                robot.pose[6] = place.BASE_CENTER
            return blob
        with patch.object(place, 'center_target', side_effect=fake_center):
            out=place.align_body_to_target(r, 'blue')
        self.assertIs(out, blob)
        self.assertTrue(r.assert_transport)
        self.assertEqual(calls, [place.CARRY_NEAR_VIEW_WRIST_PULSE, place.CARRY_NEAR_VIEW_WRIST_PULSE])
        self.assertIn(("servo", 5, place.CARRY_TRANSPORT_WRIST_PULSE), r.events)
        self.assertIn(("servo", 5, place.CARRY_NEAR_VIEW_WRIST_PULSE), r.events)

    def test_near_delivery_view_does_not_trigger_for_mid_frame_target(self):
        class R:
            dry_run = True
            pose = {1:1500,5:1400}
            def move_servo(self,*a,**k): raise AssertionError('must not move')
        self.assertFalse(place.maybe_enter_near_delivery_view(R(), SimpleNamespace(ny=0.55, width=100, height=100)))

    def test_carried_approach_uses_minimum_effective_speed_and_bounded_segments(self):
        self.assertEqual(place.carry_approach_motion(place.TARGET_RADIUS_MAX_CM + 19.0), ("forward", 35, 0.60))
        self.assertEqual(place.carry_approach_motion(place.TARGET_RADIUS_MAX_CM + 13.0), ("forward", 35, 0.42))
        self.assertEqual(place.carry_approach_motion(place.TARGET_RADIUS_MAX_CM + 6.0), ("forward", 35, 0.18))
        self.assertEqual(place.carry_approach_motion(place.TARGET_RADIUS_MAX_CM + 2.0), ("forward", 35, 0.10))
        self.assertEqual(place.carry_approach_motion(place.TARGET_RADIUS_MIN_CM - 1.0), ("backward", 35, 0.12))
        self.assertIsNone(place.carry_approach_motion((place.TARGET_RADIUS_MIN_CM + place.TARGET_RADIUS_MAX_CM) / 2.0))

    def test_terminal_stage_keeps_visual_bearing_inside_real_place_envelope(self):
        measured = BlockEstimate(radius_cm=21.9,lateral_left_cm=-1.3,forward_cm=21.86,yaw_left_deg=-3.4,camera_radius_cm=10.0,camera_height_cm=18.3,ray_pitch_deg=-60.0,block_height_cm=7.0,nx=0.46,ny=0.71)
        staged = place.terminal_staged_estimate(measured)
        self.assertEqual(staged.radius_cm, place.TERMINAL_STAGED_TARGET_RADIUS_CM)
        self.assertAlmostEqual(staged.yaw_left_deg, measured.yaw_left_deg)
        self.assertLessEqual(staged.radius_cm + 0.5, place.CALIBRATED_FINGERTIP_RADIUS_MAX_CM)
        self.assertAlmostEqual(staged.block_height_cm, place.DEFAULT_BLOCK_HEIGHT_CM)

    def test_carried_body_alignment_uses_bearing_tiers(self):
        self.assertEqual(place.carry_body_turn_for_bearing_deg(-16.0), ("rotate-right", 35, 0.34))
        self.assertEqual(place.carry_body_turn_for_bearing_deg(7.0), ("rotate-left", 35, 0.16))
        self.assertEqual(place.carry_body_turn_for_bearing_deg(1.0), ("rotate-left", 35, 0.16))
        self.assertEqual(place.carry_body_turn_for_bearing_deg(-1.0), ("rotate-right", 35, 0.16))
        self.assertIsNone(place.carry_body_turn_for_bearing_deg(0.5))


class ReleaseSupportVerificationTests(unittest.TestCase):
    def test_supported_release_silhouette_passes(self):
        blob = SimpleNamespace(nx=0.450, ny=0.831, width=400, height=131)
        self.assertTrue(place._release_support_candidate(blob))

    def test_floor_miss_silhouette_is_rejected(self):
        # Reproduced miss: full floor face is tall and bottom-clipped.
        blob = SimpleNamespace(nx=0.450, ny=0.738, width=313, height=250)
        self.assertFalse(place._release_support_candidate(blob))

    def test_transient_falling_frame_is_rejected(self):
        blob = SimpleNamespace(nx=0.342, ny=0.796, width=361, height=189)
        self.assertFalse(place._release_support_candidate(blob))


class PhysicalPlacementGeometryTests(unittest.TestCase):
    def test_place_plan_is_inside_calibrated_geometry(self):
        target = BlockEstimate(
            radius_cm=16.0,
            lateral_left_cm=0.0,
            forward_cm=16.0,
            yaw_left_deg=0.0,
            camera_radius_cm=8.0,
            camera_height_cm=18.0,
            ray_pitch_deg=-35.0,
            block_height_cm=3.0,
            nx=0.5,
            ny=0.5,
        )
        plan = calculate_place_plan(target, carried_height_cm=3.0)
        self.assertAlmostEqual(plan.fingertip_radius_cm, 16.0, places=5)
        self.assertGreater(plan.release_height_cm, 3.0)
        self.assertGreater(plan.hover_height_cm, plan.release_height_cm)
        release_tip = forward_kinematics(plan.release_pose, GRIPPER_LINK)
        self.assertAlmostEqual(release_tip.radius_cm, plan.fingertip_radius_cm, delta=0.25)
        self.assertAlmostEqual(release_tip.height_cm, plan.release_height_cm, delta=0.25)
        # Stacking uses destination-centre radius directly; pickup's +0.5 cm
        # fingertip correction must not leak into place geometry.
        self.assertAlmostEqual(plan.fingertip_radius_cm, target.radius_cm, places=6)

    def test_delivery_scan_keeps_carried_block_high_across_tilt_envelope(self):
        minimum_tip_height = 999.0
        for tilt in range(DELIVERY_TILT_MIN, DELIVERY_TILT_MAX + 1, 20):
            pose = _delivery_pose(1500, tilt)
            tip = forward_kinematics(pose, GRIPPER_LINK)
            minimum_tip_height = min(minimum_tip_height, tip.height_cm)
        self.assertGreaterEqual(minimum_tip_height, 12.0)
        self.assertEqual(DELIVERY_SCAN_POSE[4], 2320)
        self.assertEqual(DELIVERY_SCAN_POSE[5], 1320)

    def test_pickup_site_match_requires_same_location_and_scale(self):
        ref = SimpleNamespace(nx=0.50, ny=0.66, area=4000)
        same = SimpleNamespace(nx=0.53, ny=0.64, area=4300)
        far = SimpleNamespace(nx=0.84, ny=0.20, area=4300)
        self.assertTrue(_matches_pickup_site(ref, same))
        self.assertFalse(_matches_pickup_site(ref, far))


class RealStructuredPlaceIntegrationTests(unittest.TestCase):
    class NoCallCompleter:
        def complete(self, messages, image=None):
            raise AssertionError("LLM must not be called for deterministic REAL placement goal")

    def test_real_place_fastpath_runs_verified_pick_then_place_without_map_or_carry(self):
        import os
        from harness.catalog import default_registry
        from harness.loop import run_loop

        seen = []

        def runner(name, **kwargs):
            seen.append(name)
            base = {
                "ok": True,
                "skill": name,
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
            }
            if name == "search":
                return {**base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True, "cx": .5, "cy": .30, "area_ratio": .01}}
            if name == "track":
                return {**base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True, "cx": .5, "cy": .32, "area_ratio": .01}}
            if name == "approach":
                return {**base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True, "cx": .5, "cy": .70, "area_ratio": .07}}
            if name == "pick":
                return {**base, "outcome_status": "ACHIEVED", "grasp_verified": True, "verification_source": "camera_pickup_site_disappearance"}
            if name == "search_destination":
                self.assertEqual(kwargs, {"target_color": "red", "destination_color": "blue"})
                return {**base, "outcome_status": "ACHIEVED", "destination_visible": True}
            if name == "place":
                self.assertEqual(kwargs, {"target_color": "red", "destination_color": "blue"})
                return {**base, "outcome_status": "ACHIEVED", "place_verified": True, "verification_source": "camera_red_over_target_stack_geometry"}
            raise AssertionError(f"unexpected REAL action {name}")

        registry = default_registry(actions_path="scripts/robot_actions.py", runner=runner)
        old_flag = os.environ.get("UGRP_STRUCTURED_RED_FASTPATH")
        os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = "1"
        try:
            result = run_loop(
                self.NoCallCompleter(),
                registry,
                "빨간 블록을 파란 블록 위에 올려줘",
                execute=True,
                auto_observe=True,
                observe=lambda: None,
                max_steps=16,
            )
        finally:
            if old_flag is None:
                os.environ.pop("UGRP_STRUCTURED_RED_FASTPATH", None)
            else:
                os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = old_flag

        self.assertEqual(seen, ["search", "track", "approach", "pick", "search_destination", "place"])
        self.assertEqual(result.stopped, "final")
        self.assertEqual(result.final, "요청한 작업을 완료했어.")
        self.assertEqual(result.world_state["task"]["phase"], "PLACED")
        self.assertEqual(result.world_state["grasp"]["state"], "EMPTY")

    def test_motion_only_real_skills_refresh_camera_before_next_precondition(self):
        import os
        import tempfile
        from harness.catalog import default_registry
        from harness.loop import run_loop

        try:
            from PIL import Image, ImageDraw
        except Exception:
            self.skipTest("Pillow unavailable")

        seen = []
        with tempfile.TemporaryDirectory() as td:
            paths = []
            for index, (top, bottom) in enumerate(((18, 42), (34, 58), (60, 84))):
                path = Path(td) / f"frame-{index}.png"
                im = Image.new("RGB", (100, 100), (25, 25, 25))
                draw = ImageDraw.Draw(im)
                draw.rectangle((40, top, 60, bottom), fill=(255, 0, 0))
                im.save(path)
                paths.append(str(path))
            frames = iter(paths)

            def runner(name, **kwargs):
                seen.append(name)
                base = {
                    "ok": True,
                    "skill": name,
                    "command_status": "ACCEPTED",
                    "execution_status": "COMPLETED",
                }
                if name in {"search", "track"}:
                    # These bounded camera motions do not guarantee their
                    # postcondition and therefore need a fresh frame.
                    return {**base, "outcome_status": "UNKNOWN"}
                if name == "approach":
                    # The REAL approach driver exits 0 only after its live
                    # camera reports the grasp zone, so no extra lagging frame
                    # should overwrite PREGRASP.
                    return {**base, "outcome_status": "ACHIEVED", "range_verified": True}
                if name == "pick":
                    return {**base, "outcome_status": "ACHIEVED", "grasp_verified": True}
                if name == "search_destination":
                    self.assertEqual(kwargs, {"target_color": "red", "destination_color": "blue"})
                    return {**base, "outcome_status": "ACHIEVED", "destination_visible": True}
                if name == "place":
                    self.assertEqual(kwargs, {"target_color": "red", "destination_color": "blue"})
                    return {**base, "outcome_status": "ACHIEVED", "place_verified": True}
                raise AssertionError(f"unexpected REAL action {name}")

            registry = default_registry(actions_path="scripts/robot_actions.py", runner=runner)
            old_flag = os.environ.get("UGRP_STRUCTURED_RED_FASTPATH")
            os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = "1"
            try:
                result = run_loop(
                    self.NoCallCompleter(),
                    registry,
                    "빨간 블록을 파란 블록 위에 올려줘",
                    execute=True,
                    auto_observe=True,
                    observe=lambda: next(frames, None),
                    max_steps=16,
                )
            finally:
                if old_flag is None:
                    os.environ.pop("UGRP_STRUCTURED_RED_FASTPATH", None)
                else:
                    os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = old_flag

        self.assertEqual(seen, ["search", "track", "approach", "pick", "search_destination", "place"])
        self.assertEqual(result.stopped, "final")
        self.assertEqual(result.world_state["task"]["phase"], "PLACED")

    def test_failed_verified_pick_never_reaches_place(self):
        import os
        from harness.catalog import default_registry
        from harness.loop import run_loop

        seen = []

        def runner(name, **kwargs):
            seen.append(name)
            base = {
                "skill": name,
                "command_status": "ACCEPTED",
                "execution_status": "COMPLETED",
            }
            if name == "search":
                return {"ok": True, **base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True, "cx": .5, "cy": .30, "area_ratio": .01}}
            if name == "track":
                return {"ok": True, **base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True, "cx": .5, "cy": .32, "area_ratio": .01}}
            if name == "approach":
                return {"ok": True, **base, "outcome_status": "ACHIEVED", "camera_red": {"visible": True, "cx": .5, "cy": .70, "area_ratio": .07}}
            if name == "pick":
                return {
                    "ok": False,
                    **base,
                    "execution_status": "FAILED",
                    "outcome_status": "NOT_ACHIEVED",
                    "grasp_verified": False,
                    "failure_code": "GRASP_NOT_ACQUIRED",
                    "reason": "grasp postcondition failed: red block remains at pickup site",
                }
            raise AssertionError(f"place must not run after failed pick: {name}")

        registry = default_registry(actions_path="scripts/robot_actions.py", runner=runner)
        old_flag = os.environ.get("UGRP_STRUCTURED_RED_FASTPATH")
        os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = "1"
        try:
            result = run_loop(
                self.NoCallCompleter(),
                registry,
                "빨간 블록을 파란 블록 위에 올려줘",
                execute=True,
                auto_observe=True,
                observe=lambda: None,
                max_steps=8,
            )
        finally:
            if old_flag is None:
                os.environ.pop("UGRP_STRUCTURED_RED_FASTPATH", None)
            else:
                os.environ["UGRP_STRUCTURED_RED_FASTPATH"] = old_flag

        self.assertNotIn("place", seen)
        self.assertIn("pick", seen)
        self.assertEqual(result.world_state["grasp"]["state"], "EMPTY")
        self.assertNotEqual(result.world_state["task"]["phase"], "PLACED")


class ColourDetectorTests(unittest.TestCase):
    def test_blue_and_yellow_detectors_on_synthetic_bgr(self):
        try:
            import cv2  # noqa: F401
            import numpy as np
        except Exception:
            self.skipTest("OpenCV unavailable on Oracle host; detector is exercised on Pi runtime")
        from camera import detect_color_blob

        blue = np.zeros((120, 160, 3), dtype=np.uint8)
        blue[45:90, 60:110] = (255, 20, 20)
        yellow = np.zeros((120, 160, 3), dtype=np.uint8)
        yellow[45:90, 60:110] = (20, 220, 240)
        self.assertIsNotNone(detect_color_blob(blue, "blue", min_area=100))
        self.assertIsNotNone(detect_color_blob(yellow, "yellow", min_area=100))
        self.assertIsNone(detect_color_blob(blue, "yellow", min_area=100))

    def test_generic_detector_rejects_thin_self_coloured_component(self):
        try:
            import cv2
            import numpy as np
        except Exception:
            self.skipTest("OpenCV unavailable on Oracle host; detector is exercised on Pi runtime")
        from camera import detect_color_blob

        for color, bgr in (("blue", (255, 20, 20)), ("yellow", (20, 220, 240))):
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            # A 67x25 dense strip is above the normal area floor, but its
            # shape matches the reproduced eye-in-hand self artefact rather
            # than a bounded delivery block.
            cv2.rectangle(frame, (514, 424), (580, 448), bgr, -1)
            self.assertIsNone(detect_color_blob(frame, color, min_area=800), color)

            cv2.rectangle(frame, (250, 280), (367, 399), bgr, -1)
            detected = detect_color_blob(frame, color, min_area=800)
            self.assertIsNotNone(detected, color)
            self.assertLess(abs(detected.nx - .482), .03)


if __name__ == "__main__":
    unittest.main()
