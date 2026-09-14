#!/usr/bin/env python3
"""Run the portable UGRP regression suite used by GitHub Actions."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
TEST_PATTERNS = (
    "tests/test_camera_pair*.py",
    "tests/test_pair_carry*.py",
    "tests/test_pair_navigation*.py",
    "tests/test_camera_action_learning.py",
    "tests/test_camera_local_servo.py",
    "tests/test_camera_visual_observer.py",
    "tests/test_camera_motion_identity.py",
    "tests/test_camera_landmark_tracker.py",
    "tests/test_camera_grasp_controller.py",
    "tests/test_camera_robot_port.py",
    "tests/test_camera_beam_features.py",
    "tests/test_camera_beam_shaft.py",
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
)


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
    return subprocess.call(command, cwd=ROOT, env=env)


if __name__ == "__main__":
    raise SystemExit(main())
