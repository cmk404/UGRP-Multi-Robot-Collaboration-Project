"""Opt-in rolling approach: RGB causality, bounded leases, and owner aborts."""
from concurrent.futures import CancelledError, Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from harness.rolling_visual_servo import (RollingApproachLease,
    approach_drive_support, guard_near_entry_from_issued_commands)
from harness.markerless_box import _PROVENANCE as GROUND_BOX_PROVENANCE
from scripts.research_dispatch_scene import DispatchScene
from scripts.run_dispatch_skills import SkillScene


OWN = "a" * 64
TOP = "b" * 64


def _support(**changes):
    args = dict(
        action={"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        box={"visible": True, "confidence": .86,
             "target_id": "box", "ambiguity_reason": None,
             "identity_source": "task_catalog_reference_only_not_visually_decoded",
             "reason": "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
             "provenance": GROUND_BOX_PROVENANCE},
        target=(.28, 0., .024), cargo_id="box", frame_id=5, observed_at_s=1.0,
        decision_at_s=1.1, capture_started_at_s=.98,
        own_sha256=OWN, top_sha256=TOP, paired_top_sha256=TOP,
    )
    args.update(changes)
    return approach_drive_support(**args)


def test_direct_paired_fresh_rgb_allows_one_original_motion_horizon():
    evidence = _support()
    assert evidence["allowed"] and evidence["original_duration_s"] == .3
    lease = RollingApproachLease(
        action={"kind": "drive", "forward": .08, "turn": 0., "duration_s": .3},
        evidence=evidence, decision_at_s=1.1, first_issued_at_s=1.1,
        plan_hash="p", valid_until_s=1.35, motor_commands=(.08,) * 4,
        permission_requests=(("box", "APPROACH"),),
        issued_windows=[(1.1, 1.35)],
    )
    assert lease.next_duration_s(1.1) == pytest.approx(.25)
    assert lease.next_duration_s(1.33) == pytest.approx(.07)
    assert lease.next_duration_s(1.4) == 0
    assert lease.frame_within_issued_command_window(1.2)
    assert not lease.frame_within_issued_command_window(1.4)
    long_evidence=_support(action={"kind": "drive", "fwd": .15,
                                   "turn": 0., "duration": 1.0})
    long_lease=RollingApproachLease(
        action={"kind": "drive", "forward": .15, "turn": 0., "duration_s": 1.},
        evidence=long_evidence, decision_at_s=1.1, first_issued_at_s=1.1,
        plan_hash="p", valid_until_s=1.35, motor_commands=(.15,) * 4,
        permission_requests=(("box", "APPROACH"),))
    assert long_lease.next_duration_s(1.58) == pytest.approx(.02)
    assert long_lease.next_duration_s(1.6) == 0


def test_near_moving_rgb_requires_new_settled_view_before_legacy_handoff():
    lease=RollingApproachLease(
        action={"kind": "drive", "forward": .08, "turn": 0., "duration_s": .3},
        evidence=_support(), decision_at_s=1.1, first_issued_at_s=1.1,
        plan_hash="p", valid_until_s=1.35, motor_commands=(.08,) * 4,
        permission_requests=(("box", "APPROACH"),),
        issued_windows=[(1.1,1.35)], hold_at_s=1.3)
    # The issued integral is a reobserve trigger, never a measured distance.
    moving=guard_near_entry_from_issued_commands(
        _support(target=(.235,0.,.024)),lease,1.2,1.3)
    assert moving["reason"] == "moving_rgb_near_entry_requires_settled_reobservation"
    assert moving["post_capture_issued_forward_integral_m"] == pytest.approx(.008)
    assert moving["issued_integral_is_not_measured_travel"]
    assert moving["previous_command_settle_until_s"] == pytest.approx(1.5)
    fresh=guard_near_entry_from_issued_commands(
        _support(target=(.235,0.,.024),observed_at_s=1.52,
                 decision_at_s=1.55,capture_started_at_s=1.51),
        lease,1.52,1.55)
    assert fresh["reason"] == "final_entry_handoff"
    assert fresh["post_capture_issued_forward_integral_m"] == 0
    far=guard_near_entry_from_issued_commands(
        _support(target=(.35,0.,.024)),lease,1.2,1.3)
    assert far["allowed"]


@pytest.mark.parametrize("change,reason", [
    ({"decision_at_s": 1.61}, "stale_or_noncausal_rgb"),
    ({"observed_at_s": .9}, "stale_or_noncausal_rgb"),
    ({"frame_id": 4, "prior_frame_id": 4,
      "prior_observed_at_s": .8, "prior_own_sha256": "c" * 64}, "reused_rgb_evidence"),
    ({"prior_frame_id": 4, "prior_observed_at_s": .8,
      "prior_own_sha256": OWN}, "reused_rgb_evidence"),
    ({"paired_top_sha256": "c" * 64}, "unpaired_rgb_identity"),
    ({"box": {"visible": False}}, "no_direct_cyan_rgb"),
    ({"box": {"visible": True, "confidence": .74,
              "reason": "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
              "provenance": GROUND_BOX_PROVENANCE}}, "weak_direct_cyan_rgb"),
    ({"target": (.21, 0., .024)}, "final_entry_handoff"),
])
def test_rolling_support_fails_closed_without_fresh_direct_visual_basis(change, reason):
    assert _support(**change)["reason"] == reason


def _scene(now=1.1):
    clock = [now]
    port = SimpleNamespace(_motor_commands=(0.,) * 4,
                           _command_expires_at=None, _drive_expires_at=None,
                           validate_bounded=Mock(), apply_bounded=Mock(), hold=Mock())

    def apply(action, issued_at, duration):
        port._motor_commands=(float(action["forward"]),) * 4
        port._command_expires_at=port._drive_expires_at=issued_at+duration

    def hold(_now):
        port._motor_commands=(0.,) * 4
        port._command_expires_at=port._drive_expires_at=None

    port.apply_bounded.side_effect=apply
    port.hold.side_effect=hold
    scene=SkillScene.__new__(SkillScene)
    scene.time=lambda: clock[0]
    scene.authorize=Mock()
    scene.bindings=SimpleNamespace(
        solo="r2", tasks={"box": {"id": "box-job"}}, finished=set(),
        committed={"plan_hash": "plan"})
    permission=Mock(return_value=True)
    scene._permission_snapshot=Mock(
        return_value=SimpleNamespace(permission=permission))
    scene._commit_permissions=Mock(return_value=True)
    scene.ports={"r2": port}
    scene.command_history={"r2": []}
    scene.solo_renewals=[]
    scene.solo_raw=[]
    scene.solo=SimpleNamespace(phase="approach", done=False, navigator=None)
    scene._solo_pending=None
    scene._solo_approach_lease=None
    return scene, port, clock, permission


def test_owner_renews_only_residual_of_original_decision_while_rgb_pending():
    scene, port, clock, _ = _scene()
    support=_support()
    first=scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        support, clock[0], [("box", "APPROACH")])
    assert first == pytest.approx(.25)
    assert scene.command_history["r2"][-1]["source_rgb_evidence"]["source_own_sha256"] == OWN
    assert scene.command_history["r2"][-1]["valid_until_s"] == pytest.approx(1.35)
    assert scene.solo_raw[-1]["raw_action"]["duration_s"] == pytest.approx(.25)
    scene._solo_pending={"future": Future()}
    clock[0]=1.33
    scene._tick_solo_approach_lease(clock[0])
    assert port.apply_bounded.call_count == 2
    assert scene.command_history["r2"][-1]["action"]["duration_s"] == pytest.approx(.07)
    assert scene.command_history["r2"][-1]["source_rgb_evidence"]["renewed_with_same_rgb"]
    assert scene._solo_approach_lease.valid_until_s == pytest.approx(1.4)
    clock[0]=1.4
    scene._tick_solo_approach_lease(clock[0])
    assert scene._solo_approach_lease is None
    assert port.apply_bounded.call_count == 2
    assert port.hold.call_count == 1
    assert scene.solo_renewals[-1]["reason"] == "original_horizon_or_rgb_ttl"


def test_autonomous_port_expiry_is_a_normal_hold_not_a_foreign_command():
    scene, port, clock, _ = _scene()
    scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        _support(), clock[0], [("box", "APPROACH")])
    # VisualMacroExecutor.tick calls port.tick before the owner lease check.
    port.hold(1.35)
    clock[0]=1.35
    scene._tick_solo_approach_lease(clock[0])
    assert scene._solo_approach_lease is None
    assert scene.solo_renewals[-1]["reason"] == "original_horizon_or_rgb_ttl"
    assert port.apply_bounded.call_count == 1


def test_early_hold_truncates_effective_command_window_for_rgb_audit():
    scene, _, clock, _ = _scene()
    scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        _support(), clock[0], [("box", "APPROACH")])
    lease=scene._solo_approach_lease
    clock[0]=1.2
    scene._clear_solo_approach_lease(clock[0],"new_pose")
    stop=scene.solo_renewals[-1]
    assert stop["scheduled_lease_windows"] == [(1.1,1.35)]
    assert stop["effective_command_windows"] == [(1.1,1.2)]
    assert not lease.frame_within_issued_command_window(1.25)
    assert lease.issued_forward_command_integral_m(1.2,1.3) == 0


@pytest.mark.parametrize("invalid", ["plan", "permission", "port", "phase"])
def test_owner_stops_immediately_on_authority_or_lifecycle_change(invalid):
    scene, port, clock, permission = _scene()
    scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        _support(), clock[0], [("box", "APPROACH")])
    if invalid == "plan":
        scene.bindings.committed["plan_hash"]="other"
    elif invalid == "permission":
        permission.return_value=False
    elif invalid == "port":
        port._motor_commands=(.01,) * 4
    else:
        scene.solo.phase="lower"
    clock[0]=1.15
    if invalid in {"plan", "port"}:
        with pytest.raises(RuntimeError, match=("plan changed" if invalid == "plan"
                                                else "motor command changed")):
            scene._tick_solo_approach_lease(clock[0])
    else:
        scene._tick_solo_approach_lease(clock[0])
    assert scene._solo_approach_lease is None
    port.hold.assert_called_once_with(1.15)
    assert port.apply_bounded.call_count == 1


def test_owner_exception_revokes_active_motion_before_reraising():
    scene, port, clock, _ = _scene()
    scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        _support(), clock[0], [("box", "APPROACH")])
    scene._solo_tick_realtime_owned=Mock(side_effect=RuntimeError("worker failed"))
    with pytest.raises(RuntimeError, match="worker failed"):
        scene._solo_tick_realtime()
    assert scene._solo_approach_lease is None
    port.hold.assert_called_once_with(1.1)
    assert scene.solo_renewals[-1]["reason"] == "owner_exception"


def test_cancelled_rgb_worker_stops_active_lease():
    scene, port, clock, _ = _scene()
    scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        _support(), clock[0], [("box", "APPROACH")])
    cancelled=Future()
    cancelled.cancel()
    scene._solo_pending={"future": cancelled, "index": 1,
                         "stage": "APPROACH"}
    scene.rolling_visual_servo=True
    scene.solo_executor=SimpleNamespace(tick=Mock(), idle=True)
    scene._solo_budget_tick=Mock()
    clock[0]=1.15
    with pytest.raises(CancelledError):
        scene._solo_tick_realtime()
    assert scene._solo_approach_lease is None
    port.hold.assert_called_once_with(1.15)
    assert scene.solo_renewals[-1]["reason"] == "owner_exception"


def test_scene_close_stops_rolling_before_world_teardown(monkeypatch):
    scene, port, clock, _ = _scene()
    scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        _support(), clock[0], [("box", "APPROACH")])
    scene.native_view=None
    scene.world=SimpleNamespace(_physics_step_for=Mock())
    scene.original_step=Mock()
    scene.solo_executor=None
    scene._decision_workers=None
    scene._solo_pending_lease=None

    def close_world(_scene):
        assert _scene._solo_approach_lease is None
        assert _scene.solo_renewals[-1]["reason"] == "trial_end"
        assert _scene.solo_raw[-1]["event"] == "rolling_motor_hold"
        port.hold.assert_called_once_with(1.15)

    monkeypatch.setattr(DispatchScene,"close",close_world)
    clock[0]=1.15
    scene.close()


def test_owner_requests_next_paired_rgb_while_first_lease_is_still_active():
    from sim.snapshot_contract import SnapshotBackpressure

    scene, port, clock, _ = _scene()
    scene._start_solo_approach_lease(
        {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        _support(), clock[0], [("box", "APPROACH")])
    scene.rolling_visual_servo=True
    scene.solo_executor=SimpleNamespace(tick=Mock(), idle=True)
    scene._solo_budget_tick=Mock()
    scene._solo_set_resource_wait=Mock()
    scene.realtime_stats={"solo_backpressure": 0}
    scene.solo_lease=1.12
    scene._solo_retry_at=0.
    scene.solo_started=1.1
    scene.solo_rows=[]
    port._actuator_state=Mock(return_value={"servo_pulses": {}})
    scene.capture_async=Mock(side_effect=SnapshotBackpressure("busy"))
    clock[0]=1.12
    scene._solo_tick_realtime()
    scene.capture_async.assert_called_once_with(
        "solo-0", own_robots=("r2",), overview=False)
    assert scene._solo_approach_lease is not None
    assert port._motor_commands == (.08,) * 4
    assert scene.realtime_stats["solo_backpressure"] == 1


@pytest.mark.parametrize("direct,expected", [
    (True, "issued"), (False, "held")])
def test_owner_commits_only_a_direct_current_rgb_drive(direct, expected):
    scene, port, clock, _ = _scene()
    scene.rolling_visual_servo=True
    scene.solo_executor=SimpleNamespace(tick=Mock(), idle=True, submit=Mock())
    scene._solo_budget_tick=Mock()
    scene._solo_set_resource_wait=Mock()
    scene._solo_pending_lease=None
    scene._solo_motion_accept_after_s=0.
    scene._solo_approach_last_source=None
    scene._solo_retry_at=0.
    scene.solo_rows=[]
    scene.realtime_stats={"solo_decisions": 0, "solo_stale_rgb": 0,
                          "max_decision_age_s": 0.}
    box={"visible": direct, "confidence": .86,
         "target_id": "box", "ambiguity_reason": None,
         "identity_source": "task_catalog_reference_only_not_visually_decoded",
         "reason": "FLOOR_CUBOID_HYPOTHESIS_VALIDATED",
         "provenance": GROUND_BOX_PROVENANCE}
    candidate=SimpleNamespace(
        phase="approach", done=False, reason="RUNNING", navigator=None,
        box=SimpleNamespace(last_box=box, last_target=(.28, 0., .024),
            cargo_id="box", last_attachment=None, last_approach_adjustment=None))
    future=Future()
    future.set_result({
        "candidate": candidate,
        "action": {"kind": "drive", "fwd": .08, "turn": 0., "duration": .3},
        "evidence": {}, "before": "approach", "permission_requests": (),
        "observation": {"frame_id": 5, "sim_time": 1., "sha256": OWN,
                        "image": "encoded"},
        "images": {"top": {"sha256": TOP}},
        "top_sha256": TOP, "input_transform": {},
        "observed_at_s": 1., "frame_id": 5,
    })
    scene._solo_pending={"future": future, "index": 0,
                         "stage": "APPROACH", "capture_started_at_s": .98}
    scene._solo_tick_realtime()
    assert scene.solo is candidate
    assert scene.solo_rows[-1]["rolling_capture_evidence"]["top_sha256"] == TOP
    scene.solo_executor.submit.assert_not_called()
    if expected == "issued":
        assert scene.solo_rows[-1]["rolling_support"]["allowed"]
        assert scene._solo_approach_lease is not None
        assert scene.command_history["r2"][-1]["source_rgb_evidence"]["source_frame_id"] == 5
        assert port.apply_bounded.call_count == 1
    else:
        assert scene.solo_rows[-1]["rolling_dropped"] == "no_direct_cyan_rgb"
        assert scene._solo_approach_lease is None
        assert port.apply_bounded.call_count == 0
        port.hold.assert_called()


def test_opt_in_requires_realtime_open_map_and_is_not_a_default(tmp_path,capsys):
    from scripts.run_dispatch_e2e import main

    with pytest.raises(SystemExit):
        main(["--output", str(tmp_path/"out"),
              "--rolling-visual-servo"])
    assert "requires skills --realtime-control" in capsys.readouterr().err
    scene=SkillScene.__new__(SkillScene)
    scene.rolling_visual_servo=True
    scene.realtime_control=True
    scene.bindings=SimpleNamespace(static_map={
        "map_id": "dispatch_open", "terrain": [],
        "obstacles": [{"id": "service_island"}]})
    with pytest.raises(ValueError, match="dispatch_open without internal obstacles"):
        scene.start_solo()
