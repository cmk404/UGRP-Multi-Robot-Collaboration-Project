#!/usr/bin/env python3
"""Run the portable UGRP regression suite used by GitHub Actions."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
if __name__ == "__main__":
    sys.path.insert(0, str(ROOT))

from scripts import agent_lock

TEST_PATTERNS = (
    "tests/test_model_artifacts.py",
    "tests/test_simulation_session.py",
    "tests/test_agent_lock.py",
    "tests/test_ci_host_lock.py",
    "tests/test_simulation_console.py",
    "tests/test_simulation_dispatch.py",
    "tests/test_dispatch_replay.py",
    "tests/test_dispatch_plan_guidance.py",
    "tests/test_dynamic_coordination.py",
    "tests/test_dispatch_regrasp_binding.py",
    "tests/test_dispatch_regrasp_coarse.py",
    "tests/test_simulation_extensions.py",
    "tests/test_simulation_scenes.py",
    "tests/test_simulation_workflow_manager.py",
    "tests/test_simulation_input_provenance.py",
    "tests/test_simulation_launch_options.py",
    "tests/test_simulation_start_menu.py",
    "tests/test_simulation_interrupt_cleanup.py",
    "tests/test_dispatch_scene_parity.py",
    "tests/test_cloud_preflight.py",
    "tests/test_carry_failure_measurement.py",
    "tests/test_repeated_skill_summary.py",
    "tests/test_colab_simulation.py",
    "tests/test_cpu_model_diagnostic.py",
    "tests/test_mac_skill_cohort.py",
    "tests/test_model_mailbox.py",
    "tests/test_kaggle_simulation.py",
    "tests/test_kaggle_cli_auth.py",
    "tests/test_kaggle_gpu_recovery.py",
    "tests/test_kaggle_egl.py",
    "tests/test_cloud_collection.py",
    "tests/test_colab_recovery.py",
    "tests/test_colab_relay_pipeline.py",
    "tests/test_colab_egl.py",
    "tests/test_multi_object_plan.py",
    "tests/test_multi_object_scene.py",
    "tests/test_multi_object_execution.py",
    "tests/test_zone_dispatch.py", "tests/test_zone_comm_boundary.py",
    "tests/test_zone_cargo.py", "tests/test_zone_team_jobs.py", "tests/test_zone_rgb_outcome.py",
    "tests/test_zone_cargo_perception.py",
    "tests/test_zone_dialogue_ko.py",
    "tests/test_zone_hard_routes.py",
    "tests/test_zone_cargo_perception_v2.py",
    "tests/test_zone_study_contract.py", "tests/test_zone_study_inputs.py",
    "tests/test_zone_study_scenarios.py", "tests/test_zone_study_protocol.py",
    "tests/test_zone_sim_cost.py", "tests/test_zone_event_scheduler.py",
    "tests/test_zone_study_eval.py", "tests/test_zone_study_offline.py",
    "tests/test_zone_study_review_fixes.py", "tests/test_zone_study_review_r5.py",
    "tests/test_owncam_localizer.py", "tests/test_zone_landmarks_sim.py",
    "tests/test_rgb_execution*.py",
    "tests/test_rgb_communication*.py",
    "tests/test_act_map_suite.py",
    "tests/test_jev_execution_shadow.py",
    "tests/test_jev_motion.py",
    "tests/test_tensorboard_export.py",
    "tests/test_tensorboard_launcher.py",
    "tests/test_colab_carry_bundle.py",
    "tests/test_carry_training_checkpoint_optional.py",
    "tests/test_act_route_teacher*.py",
    "tests/test_jev_skill_motion.py",
    "tests/test_act_speed_generalization.py",
    "tests/test_act_recovery.py", "tests/test_recovery_commands.py",
    "tests/test_reference_approach_data.py",
    "tests/test_camera_pair*.py",
    "tests/test_pair_carry*.py",
    "tests/test_task_stage_sync.py",
    "tests/test_task_stage_execution.py",
    "tests/test_research_camera_e2e.py",
    "tests/test_research_entry_stop.py",
    "tests/test_research_cohort_admission.py",
    "tests/test_saved_act_inputs.py",
    "tests/test_carry_termination_audit.py",
    "tests/test_matched_carry_cohort.py",
    "tests/test_matched_carry_shards.py",
    "tests/test_carry_failure_measurement.py",
    "tests/test_research_execution_recovery.py",
    "tests/test_research_visual_evidence.py",
    "tests/test_camera_goal_transport.py",
    "tests/test_three_robot_plan.py",
    "tests/test_three_robot_mission.py",
    "tests/test_three_robot_physical_controls.py",
    "tests/test_dispatch_research.py",
    "tests/test_dispatch_execution.py",
    "tests/test_dispatch_skill_binding.py",
    "tests/test_dispatch_box_identity.py",
    "tests/test_dispatch_beam_scan_cache.py",
    "tests/test_pair_coarse_pixel_cache.py",
    "tests/test_pair_coarse_concurrency.py",
    "tests/test_pair_navigation*.py",
    "tests/test_pair_transport*.py",
    "tests/test_dispatch_adaptive.py",
    "tests/test_dispatch_pair_navigation.py",
    "tests/test_dispatch_evaluation.py",
    "tests/test_dispatch_feasibility.py",
    "tests/test_camera_action_learning.py",
    "tests/test_camera_local_servo.py",
    "tests/test_camera_visual_observer.py",
    "tests/test_camera_motion_identity.py",
    "tests/test_camera_landmark_tracker.py",
    "tests/test_camera_grasp_controller.py",
    "tests/test_camera_robot_port.py",
    "tests/test_camera_beam_features.py",
    "tests/test_camera_beam_shaft.py",
    "tests/test_camera_beam_fast_exact.py",
    "tests/test_dispatch_translation_skew_fallback.py",
    "tests/test_dispatch_pair_prefetch.py",
    "tests/test_communication_observer.py",
    "tests/test_dispatch_solo_cadence.py",
    "tests/test_fine_gain_schedule.py",
    "tests/test_dispatch_coarse_handoff.py",
    "tests/test_camera_varied_start_runner.py",
    "tests/test_dispatch_pair_pending_renewal.py",
    "tests/test_dispatch_solo_budget.py",
    "tests/test_rolling_view_recovery.py",
    "tests/test_settled_view_recovery.py",
    "tests/test_camera_gripper_motion.py",
    "tests/test_camera_pixel_grasp.py",
    "tests/test_camera_sweep_trial.py",
    "tests/test_camera_pixel_resume.py",
    "tests/test_camera_pixel_jacobian.py",
    "tests/test_camera_teacher_student.py",
    "tests/test_camera_recovery_student.py",
    "tests/test_audit_camera_grasp_student.py",
    "tests/test_camera_grasp_teacher.py",
    "tests/test_grasp_recovery_teacher.py",
    "tests/test_grasp_recovery_cohort.py",
    "tests/test_camera_approach_student.py",
    "tests/test_camera_approach_scene.py",
    "tests/test_audit_camera_approach_student.py",
    "tests/test_camera_varied_start*.py",
    "tests/test_camera_short_transport*.py",
    "tests/test_known_map*.py",
    "tests/test_rgb_traffic*.py",
    "tests/test_markerless*.py",
    "tests/test_visual_attachment*.py",
    "tests/test_placement_guidance.py",
    "tests/test_visual_placement*.py",
    "tests/test_gemini_transport_policy.py",
    "tests/test_transport_context.py",
    "tests/test_budget_repair_regressions.py",
    "tests/test_navigation_evidence.py",
    "tests/test_navigation_temporal.py",
    "tests/test_ugrp_session.py",
    "tests/test_seed_validation_model.py",
    "tests/test_semantic_pick_policy.py",
    "tests/test_pick_match*.py",
    "tests/test_replay_pick_match.py",
)


def local_lock_root(primary: Path | None = None) -> Path | None:
    """Share the experiment lock only with worktrees of the local primary repo.

    A GitHub runner or unrelated clone must not create a developer's Mac path.
    Do not use CI=true as an escape hatch: local offline tests also set it.
    """
    primary = primary or agent_lock.DEFAULT_ROOT.parent.parent
    if not (primary / ".git").is_dir():
        return None
    common = subprocess.check_output(
        ["git", "rev-parse", "--git-common-dir"], cwd=ROOT, text=True,
    ).strip()
    if (ROOT / common).resolve() != (primary / ".git").resolve():
        return None
    return primary / "outputs" / "agent-locks"


def run_locked(command: list[str], env: dict, lock_root: Path) -> int:
    """Fail before spawn when busy; retain the lock until our group is gone."""
    from scripts import ugrp_session

    branch = subprocess.check_output(
        ["git", "branch", "--show-current"], cwd=ROOT, text=True,
    ).strip() or "detached"
    owner = branch.split("/", 1)[0] if branch.startswith(("codex/", "claude/")) else "local-tests"
    try:
        acquired = agent_lock.acquire(
            lock_root, owner=owner, branch=branch, purpose="local offline regression tests",
            pid=os.getpid(), expected_minutes=10,
        )
    except RuntimeError as error:
        print(f"Tests not started: {error}", file=sys.stderr)
        return 3

    child = None
    requested_signal = None
    previous = {}
    cleanup_verified = False

    def request_stop(signum, _frame):
        nonlocal requested_signal
        if requested_signal is None:
            requested_signal = signum

    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[sig] = signal.signal(sig, request_stop)
        child = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True)
        deadline = None
        while True:
            if requested_signal is not None and deadline is None:
                try:
                    os.killpg(child.pid, requested_signal)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + 5
            try:
                code = child.wait(timeout=0.1)
                return 128 + (-code) if code < 0 else code
            except subprocess.TimeoutExpired:
                if deadline is not None and time.monotonic() >= deadline:
                    ugrp_session.stop_group(child.pid, grace=1)
    finally:
        try:
            if child is not None:
                try:
                    ugrp_session.stop_group(child.pid, grace=1)
                except ProcessLookupError:
                    pass
                child.wait(timeout=5)
            cleanup_verified = child is None or not ugrp_session.process_group_alive(child.pid)
        finally:
            held = agent_lock.status(lock_root)
            ours = held and all(held.get(key) == acquired[key] for key in ("owner", "pid", "acquired_unix"))
            if cleanup_verified and ours:
                agent_lock.release(lock_root, owner=owner)
            elif ours:
                print("Test process cleanup unconfirmed; host lock retained for inspection", file=sys.stderr)
            for sig, handler in previous.items():
                signal.signal(sig, handler)
            if not cleanup_verified:
                raise RuntimeError("Owned test group cleanup unconfirmed; inspect the retained lock")


def main() -> int:
    tests = sorted(
        {
            str(path.relative_to(ROOT))
            for pattern in TEST_PATTERNS
            for path in ROOT.glob(pattern)
        }
    )
    if not tests:
        print("No CI tests matched", file=sys.stderr)
        return 2

    env = os.environ.copy()
    for name in tuple(env):
        if name.endswith("_API_KEY") or name in {"GOOGLE_APPLICATION_CREDENTIALS"}:
            env.pop(name)
    env.update({"CI": "true", "PYTHONDONTWRITEBYTECODE": "1"})
    command = [sys.executable, "-m", "pytest", "-q", *tests]
    print(f"Running {len(tests)} offline test modules", flush=True)
    lock_root = local_lock_root()
    if lock_root is not None:
        return run_locked(command, env, lock_root)
    return subprocess.call(command, cwd=ROOT, env=env)


if __name__ == "__main__":
    raise SystemExit(main())
