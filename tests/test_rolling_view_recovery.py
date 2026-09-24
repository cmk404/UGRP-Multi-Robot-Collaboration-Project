"""The saved v40 weak view is a development fixture, not a physical result."""
from __future__ import annotations

import base64
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from concurrent.futures import Future

import pytest

from harness.markerless_box import observe_ground_box
from harness.markerless_box import _PROVENANCE as GROUND_BOX_PROVENANCE
from harness.rolling_visual_servo import (
    ActiveViewRecovery, VIEW_RECOVERY_MAX_ELAPSED_S,
    VIEW_RECOVERY_MAX_EPISODES, VIEW_RECOVERY_MAX_POSES)
from harness.visual_box_skill import VisualBoxSkill
from scripts.run_dispatch_skills import SkillScene


FIXTURE = Path(__file__).parent / "fixtures" / "rolling_view_recovery"


def _saved_samples():
    manifest = json.loads((FIXTURE / "manifest.json").read_text())
    samples = []
    for item in manifest["items"]:
        raw = (FIXTURE / item["fixture"]).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == item["source_sha256"]
        box = observe_ground_box(base64.b64encode(raw).decode(),
                                 item["servo_pulses"], "small_box_01")
        samples.append({"frame_id": item["frame_id"],
                        "capture_started_at_s": item["capture_started_at_s"],
                        "observed_at_s": item["observed_at_s"],
                        "decision_at_s": item["decision_at_s"],
                        "own_sha256": item["source_sha256"],
                        "top_sha256": item["top_sha256"],
                        "paired_top_sha256": item["top_sha256"],
                        "pose": item["servo_pulses"], "box": box,
                        "target": box["estimated_box_center_base_m"],
                        "cargo_id": "small_box_01", "plan_hash": "saved-plan",
                        "phase": "approach"})
    return samples


def _anchor_receipts():
    # Saved macro_finished issued pan 1558 from a prior own RGB command;
    # motion_confirmed is false and no measured qpos is consulted.
    manifest = json.loads((FIXTURE / "manifest.json").read_text())
    receipts = {}
    for servo, key in [(3, "anchor_completed_wrist_receipt"),
                       (6, "anchor_completed_pose_receipt")]:
        raw = manifest[key]
        assert raw["event"] == "macro_finished"
        assert raw["execution"]["motion_confirmed"] is False
        receipts[str(servo)] = {"status": raw["execution"]["status"],
                                "macro": raw["macro"],
                                "ended_at": raw["execution"]["ended_at"]}
    return receipts


def _started():
    strong, weak = _saved_samples()
    assert strong["box"]["confidence"] >= .75
    assert .50 <= weak["box"]["confidence"] < .75
    recovery = ActiveViewRecovery()
    assert recovery.remember(strong, _anchor_receipts(), 13.0955)
    event = recovery.start(weak)
    assert event["kind"] == "pose_issued"
    assert event["issued_pose"] == {"6": 1558}
    assert event["anchor_frame_id"] == strong["frame_id"]
    assert event["own_sha256"] == weak["own_sha256"]
    return recovery, strong, weak


def _post_pose(strong, weak):
    # A hypothetical *new* post-pose capture with the saved strong pixels.
    # This checks the evidence contract, not that a v43 robot actually moved.
    sample = copy.deepcopy(strong)
    sample.update(frame_id=weak["frame_id"] + 8,
                  capture_started_at_s=15.25, observed_at_s=15.30,
                  decision_at_s=15.45, top_sha256="c" * 64,
                  paired_top_sha256="c" * 64,
                  completed_pose={"status": "completed",
                                  "macro": {"kind": "pose", "pulses": {"6": 1558}},
                                  "ended_at": 15.10})
    return sample


def test_saved_weak_rgb_restores_only_last_completed_pan_and_needs_new_strong_view():
    recovery, strong, weak = _started()
    event = recovery.advance(_post_pose(strong, weak))
    assert event["kind"] == "recovered"
    assert event["frame_id"] > weak["frame_id"]
    assert event["locked_pan_pwm"] == 1558
    assert recovery.episode is None


def test_recovery_rejects_same_frame_missing_receipt_and_plan_change():
    recovery, strong, weak = _started()
    for change, reason in [
        ({"frame_id": weak["frame_id"]}, "no_distinct_post_pose_rgb"),
        ({"own_sha256": weak["own_sha256"]}, "no_distinct_post_pose_rgb"),
        ({"completed_pose": None}, "active_view_pose_receipt_missing"),
        ({"plan_hash": "changed"}, "plan_changed_during_active_view"),
    ]:
        trial = copy.deepcopy(recovery)
        sample = _post_pose(strong, weak)
        sample.update(change)
        assert trial.advance(sample)["reason"] == reason


def test_active_view_aborts_when_candidate_exits_approach():
    recovery, strong, weak = _started()
    sample = _post_pose(strong, weak)
    sample["phase"] = "lower"
    event = recovery.advance(sample)
    assert event["kind"] == "failed"
    assert event["reason"] == "phase_exit_during_active_view"
    assert event["wheels_held"] is True
    assert recovery.episode is None


def test_recovery_rejects_stale_detached_ambiguous_or_post_anchor_wheels():
    strong, weak = _saved_samples()
    for mutate, expected in [
        (lambda s: s.update(decision_at_s=s["observed_at_s"] + .61),
         "stale_or_unpaired_rgb"),
        (lambda s: s["target"].__setitem__(0, s["target"][0] + .12),
         "weak_view_identity_unresolved"),
        (lambda s: s["box"].update(ambiguity_reason="SECOND_CYAN"),
         "weak_view_identity_unresolved"),
        (lambda s: s["box"].update(area_px=s["box"]["area_px"] * .2),
         "weak_view_identity_unresolved"),
    ]:
        recovery = ActiveViewRecovery()
        assert recovery.remember(strong, _anchor_receipts(), 13.0955)
        current = copy.deepcopy(weak)
        mutate(current)
        assert recovery.start(current)["reason"] == expected
    recovery = ActiveViewRecovery()
    assert recovery.remember(strong, _anchor_receipts(), 13.0955)
    recovery.note_wheel_issue()
    assert recovery.start(weak)["reason"] == "anchor_stale_or_wheel_issued"
    recovery = ActiveViewRecovery()
    assert recovery.remember(strong, _anchor_receipts(), 13.0955)
    changed_plan = copy.deepcopy(weak)
    changed_plan["plan_hash"] = "different-plan"
    assert recovery.start(changed_plan)["reason"] == "anchor_plan_changed"


def test_no_new_rgb_or_no_confidence_progress_fails_in_fixed_budget():
    recovery, strong, weak = _started()
    same = copy.deepcopy(weak)
    same.update(frame_id=weak["frame_id"] + 8,
                observed_at_s=15.30, capture_started_at_s=15.25,
                decision_at_s=15.45,
                completed_pose={"status": "completed",
                                "macro": {"kind": "pose", "pulses": {"6": 1558}},
                                "ended_at": 15.10},
                pose=copy.deepcopy(strong["pose"]))
    assert recovery.advance(same)["reason"] == "no_distinct_post_pose_rgb"
    assert recovery.episode is None
    recovery, strong, weak = _started()
    post = _post_pose(strong, weak)
    post["box"]["confidence"] = .69
    first = recovery.advance(post)
    assert first["kind"] == "pose_issued" and first["issued_pose"] == {"3": 716}
    second_view = copy.deepcopy(post)
    second_view.update(frame_id=post["frame_id"] + 5,
                       capture_started_at_s=15.85, observed_at_s=15.90,
                       decision_at_s=16.0, own_sha256="d" * 64,
                       completed_pose={"status": "completed",
                                       "macro": {"kind": "pose", "pulses": {"3": 716}},
                                       "ended_at": 15.70})
    second_view["pose"]["3"] = 716
    second = recovery.advance(second_view)
    assert second["kind"] == "pose_issued" and second["issued_pose"] == {"3": 764}
    third_view = copy.deepcopy(second_view)
    third_view.update(frame_id=second_view["frame_id"] + 5,
                      capture_started_at_s=16.45, observed_at_s=16.50,
                      decision_at_s=16.60, own_sha256="e" * 64,
                      completed_pose={"status": "completed",
                                      "macro": {"kind": "pose", "pulses": {"3": 764}},
                                      "ended_at": 16.30})
    third_view["pose"]["3"] = 764
    assert recovery.advance(third_view)["reason"] == "active_view_budget_exhausted"
    assert recovery.episode is None
    recovery, strong, weak = _started()
    late = _post_pose(strong, weak)
    late["box"]["confidence"] = .69
    late.update(capture_started_at_s=17.90, observed_at_s=17.95,
                decision_at_s=18.05, completed_pose={"status": "completed",
                    "macro": {"kind": "pose", "pulses": {"6": 1558}},
                    "ended_at": 17.80})
    assert recovery.advance(late)["reason"] == "pose_cannot_settle_within_budget"
    assert VIEW_RECOVERY_MAX_POSES == 3
    assert VIEW_RECOVERY_MAX_ELAPSED_S == 4.0
    assert VIEW_RECOVERY_MAX_EPISODES == 3


def test_view_lock_skips_only_bounded_pan_correction_and_releases_on_progress():
    skill = VisualBoxSkill()
    skill._last_sim_time = 14.55
    skill.lock_rolling_approach_view(1558, .60)
    box = {"visible": True, "confidence": .82, "target_id": skill.cargo_id,
           "ambiguity_reason": None,
           "identity_source": "task_catalog_reference_only_not_visually_decoded",
           "reason": "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
           "provenance": GROUND_BOX_PROVENANCE}
    skill._last_sim_time = 15.30
    assert skill._retain_rolling_view(box, .60, {"6": 1558}, 1528)
    assert not skill._retain_rolling_view(box, .53, {"6": 1558}, 1528)
    assert skill._rolling_view_lock is None


@pytest.mark.parametrize("bad_receipt", [
    None,
    {"6": {"status": "completed", "macro": {"kind": "pose", "pulses": {"6": 1558}},
           "ended_at": 11.8}},
    {"3": {"status": "interrupted", "macro": {"kind": "pose", "pulses": {"3": 740}},
           "ended_at": 6.2},
     "6": {"status": "completed", "macro": {"kind": "pose", "pulses": {"6": 1558}},
           "ended_at": 11.8}},
])
def test_anchor_must_match_a_completed_issued_pose(bad_receipt):
    strong, _ = _saved_samples()
    assert not ActiveViewRecovery().remember(strong, bad_receipt, 13.0955)


def test_owner_holds_zero_wheels_before_interpolated_pose_and_logs_join():
    class Port:
        _motor_commands = (.1, .1, .1, .1)

        def hold(self, _now):
            self._motor_commands = (0., 0., 0., 0.)

    _, _, weak = _started()
    scene = SkillScene.__new__(SkillScene)
    scene.bindings = SimpleNamespace(solo="r2")
    scene.ports = {"r2": Port()}
    scene.solo_raw = []
    scene.solo_rows = []
    box = VisualBoxSkill()
    box._last_sim_time = weak["observed_at_s"]
    scene.solo = SimpleNamespace(box=box, phase="approach")
    scene.solo_executor = SimpleNamespace(submit=Mock())
    event = {"kind": "pose_issued", "reason": "weak_visible_direct_cyan",
             "issued_pose": {"6": 1558}}
    obs = {"sha256": weak["own_sha256"]}
    row = {"action": {"kind": "drive", "fwd": .1, "turn": 0., "duration": .6}}
    scene._issue_rolling_view_pose(event, weak, obs, row, 14.55)
    assert scene.ports["r2"]._motor_commands == (0.,) * 4
    assert scene.solo_raw[-1]["event"] == "rolling_view_recovery_hold"
    assert scene.solo_raw[-1]["motor_commands"] == [0.] * 4
    assert scene.solo_rows[-1]["view_recovery"]["macro_source_own_sha256"] == weak["own_sha256"]
    assert scene.solo_rows[-1]["issued_action"] == {"kind": "pose", "pulses": {"6": 1558}}
    scene.solo_executor.submit.assert_called_once()


def test_realtime_owner_replaces_weak_drive_with_pose_without_issuing_wheels():
    strong, weak = _saved_samples()
    recovery = ActiveViewRecovery()
    assert recovery.remember(strong, _anchor_receipts(), 13.0955)

    class Port:
        _motor_commands = (0., 0., 0., 0.)
        apply_bounded = Mock()

        def hold(self, _now):
            self._motor_commands = (0., 0., 0., 0.)

    scene = SkillScene.__new__(SkillScene)
    scene.authorize = Mock()
    scene.time = lambda: weak["decision_at_s"]
    scene.bindings = SimpleNamespace(
        solo="r2", tasks={"box": {"id": "box-job"}}, finished=set(),
        committed={"plan_hash": "saved-plan"}, permission=lambda *_: True)
    scene.ports = {"r2": Port()}
    scene.solo_executor = SimpleNamespace(
        tick=Mock(), submit=Mock(), idle=True,
        last_execution={"status": "completed",
                        "macro": {"kind": "pose", "pulses": {"6": 1528}},
                        "ended_at": 14.22199999998953})
    scene.rolling_visual_servo = True
    scene._view_recovery = recovery
    scene._solo_pose_log_cursor = 0
    scene._solo_completed_pose_receipts = {}
    scene._solo_approach_lease = None
    scene._solo_approach_last_lease = None
    scene._solo_approach_last_source = None
    scene._solo_pending_lease = None
    scene._solo_motion_accept_after_s = 0.
    scene._solo_budget_tick = Mock()
    scene._clear_solo_pending_lease = Mock()
    scene._clear_solo_approach_lease = Mock()
    scene._solo_set_resource_wait = Mock()
    scene._commit_permissions = Mock(return_value=True)
    scene.realtime_stats = {"solo_decisions": 0, "max_decision_age_s": 0.,
                            "solo_stale_rgb": 0}
    scene.solo_rows = []
    scene.solo_raw = []
    old_skill = VisualBoxSkill()
    old_skill.last_box = weak["box"]
    old_skill.last_target = tuple(weak["target"])
    old_skill._last_sim_time = weak["observed_at_s"]
    scene.solo = SimpleNamespace(phase="approach", box=old_skill,
                                 done=False, navigator=None)
    candidate_skill = copy.deepcopy(old_skill)
    candidate = SimpleNamespace(phase="approach", box=candidate_skill,
                                done=False, navigator=None)
    observation = {"frame_id": weak["frame_id"],
                   "sim_time": weak["observed_at_s"],
                   "sha256": weak["own_sha256"], "image": "encoded",
                   "actuator_state": {"motor_commands": [0., 0., 0., 0.],
                                      "servo_pulses": weak["pose"]}}
    result = {"candidate": candidate,
              "action": {"kind": "drive", "fwd": .1, "turn": 0., "duration": .6},
              "evidence": {}, "before": "approach", "permission_requests": (),
              "observation": observation, "images": {"top": {"sha256": weak["top_sha256"]}},
              "top_sha256": weak["top_sha256"], "input_transform": {},
              "observed_at_s": weak["observed_at_s"], "frame_id": weak["frame_id"]}
    future = Future()
    future.set_result(result)
    scene._solo_pending = {"future": future, "index": 13, "stage": "APPROACH",
                           "capture_started_at_s": weak["capture_started_at_s"]}
    scene._solo_tick_realtime_owned()
    scene._clear_solo_approach_lease.assert_called_once_with(
        weak["decision_at_s"], "fresh_rgb_decision")
    assert scene.ports["r2"]._motor_commands == (0.,) * 4
    scene.ports["r2"].apply_bounded.assert_not_called()
    assert scene.solo_rows[-1]["rolling_support"]["reason"] == "weak_direct_cyan_rgb"
    assert scene.solo_rows[-1]["view_recovery"]["kind"] == "pose_issued"
    assert scene.solo_rows[-1]["issued_action"] == {"kind": "pose", "pulses": {"6": 1558}}
    scene.solo_executor.submit.assert_called_once()


def test_owner_tick_cancels_active_pose_at_elapsed_budget_without_another_capture():
    recovery, _, weak = _started()

    class Port:
        _motor_commands = (.1, .1, .1, .1)

        def hold(self, _now):
            self._motor_commands = (0., 0., 0., 0.)

    scene = SkillScene.__new__(SkillScene)
    scene.solo = SimpleNamespace(phase="approach")
    scene.authorize = Mock()
    scene.time = lambda: weak["decision_at_s"] + VIEW_RECOVERY_MAX_ELAPSED_S
    scene._view_recovery = recovery
    scene.bindings = SimpleNamespace(solo="r2", committed={"plan_hash": "saved-plan"},
                                     permission=lambda *_: True)
    scene.ports = {"r2": Port()}
    scene.solo_raw = []
    scene.solo_executor = SimpleNamespace(cancel=Mock(), tick=Mock())
    with pytest.raises(RuntimeError, match="elapsed budget exhausted"):
        scene._solo_tick_realtime_owned()
    scene.solo_executor.cancel.assert_called_once()
    scene.solo_executor.tick.assert_not_called()
    assert scene.ports["r2"]._motor_commands == (0.,) * 4
    assert scene.solo_raw[-1]["reason"] == "active_view_elapsed_budget"


def test_all_actual_solo_wheel_issues_invalidate_a_prior_anchor():
    strong, weak = _saved_samples()
    recovery = ActiveViewRecovery()
    assert recovery.remember(strong, _anchor_receipts(), 13.0955)
    scene = SkillScene.__new__(SkillScene)
    scene._view_recovery = recovery
    scene._solo_pose_log_cursor = 0
    scene._solo_completed_pose_receipts = {}
    scene.bindings = SimpleNamespace(solo="r2")
    scene.solo_raw = [
        {"event": "raw_action", "robot_id": "r2",
         "raw_action": {"kind": "drive", "forward": .08}},
        {"event": "rolling_raw_action", "robot_id": "r2",
         "raw_action": {"kind": "drive", "forward": .08}},
    ]
    scene._refresh_completed_issued_pose_receipts()
    assert recovery.wheel_generation == 2
    assert recovery.start(weak)["reason"] == "anchor_stale_or_wheel_issued"


def test_opt_in_flag_requires_rolling_and_is_default_off(tmp_path, capsys):
    from scripts.run_dispatch_e2e import main

    with pytest.raises(SystemExit):
        main(["--output", str(tmp_path / "out"), "--rolling-view-recovery"])
    assert "requires --rolling-visual-servo" in capsys.readouterr().err
