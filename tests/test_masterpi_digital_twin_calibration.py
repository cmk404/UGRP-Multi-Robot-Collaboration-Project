from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from unittest import mock

from sim import masterpi_dynamics_v2 as dyn
from scripts.benchmarks import masterpi_calibration_plan as plan
from scripts.benchmarks.analyze_masterpi_overhead_video import analyze


class MasterPiDigitalTwinCalibrationTests(unittest.TestCase):
    def test_calibration_plan_has_live_safe_commands_and_heldout_gate(self):
        rows = plan.make_plan()
        self.assertEqual(len(rows), 108)
        self.assertEqual(sum(r["split"] == "fit" for r in rows), 72)
        self.assertEqual(sum(r["split"] == "holdout" for r in rows), 36)
        self.assertEqual({r["command"] for r in rows}, {31, 35, 40})
        self.assertEqual({r["drive_s"] for r in rows}, {0.20, 0.55})
        self.assertTrue(all(31 <= r["command"] <= 40 for r in rows))

    def test_load_fitted_dynamics_keeps_unvalidated_status_explicit(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "manifest.json"
            p.write_text(json.dumps({
                "validated": False,
                "parameters": {
                    "motor_time_constant_s": 0.12,
                    "max_forward_force_n": 1.7,
                    "max_lateral_force_n": None,
                },
            }))
            values, status = dyn.load_fitted_dynamics(p)
            self.assertEqual(status, "REAL_FITTED_UNVALIDATED")
            self.assertAlmostEqual(values["motor_time_constant_s"], 0.12)
            self.assertAlmostEqual(values["max_forward_force_n"], 1.7)
            self.assertNotIn("max_lateral_force_n", values)

    def test_world_uses_manifest_fit_but_does_not_call_it_validated(self):
        fitted = {
            "motor_time_constant_s": 0.13,
            "max_forward_force_n": 1.9,
            "max_lateral_force_n": 1.3,
            "max_yaw_torque_nm": 0.10,
            "linear_damping_n_per_mps": 1.8,
            "yaw_damping_nm_per_radps": 0.07,
        }
        with mock.patch.object(dyn, "load_fitted_dynamics", return_value=(fitted, "REAL_FITTED_UNVALIDATED")):
            world = dyn.MasterPiDynamicsV2(seed=3)
        try:
            self.assertEqual(world.calibration_status, "REAL_FITTED_UNVALIDATED")
            for key, value in fitted.items():
                self.assertAlmostEqual(world.dynamics[key], value)
            state = world.state()
            self.assertEqual(state["calibration_status"], "REAL_FITTED_UNVALIDATED")
        finally:
            world.close()

    def test_two_marker_video_recovers_metric_forward_motion(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "trial.avi"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30.0, (800, 600))
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            rear = cv2.aruco.generateImageMarker(dictionary, 10, 90)
            front = cv2.aruco.generateImageMarker(dictionary, 11, 90)
            for i in range(75):
                frame = np.full((600, 800, 3), 255, np.uint8)
                if i < 15:
                    shift = 0.0
                elif i < 32:
                    shift = (i - 15) / 17.0 * 200.0
                else:
                    shift = 200.0
                rear_cx = int(round(250 + shift))
                for marker, cx in ((rear, rear_cx), (front, rear_cx + 100)):
                    frame[255:345, cx - 45:cx + 45] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
                writer.write(frame)
            writer.release()
            result = analyze(
                path, rear_id=10, front_id=11, marker_separation_m=0.10,
                drive_s=0.55, coast_s=0.50, motion_threshold_mps=0.01,
            )
            self.assertAlmostEqual(result["dx_m"], 0.20, delta=0.02)
            self.assertAlmostEqual(result["dy_m"], 0.0, delta=0.01)
            self.assertAlmostEqual(result["dyaw_deg"], 0.0, delta=1.0)
            self.assertLess(result["stop_distance_m"], 0.01)

    def test_explicit_fitter_dynamics_override_manifest(self):
        explicit = {
            "motor_time_constant_s": 0.09,
            "max_forward_force_n": 2.0,
            "max_lateral_force_n": 1.5,
            "max_yaw_torque_nm": 0.11,
            "linear_damping_n_per_mps": 1.6,
            "yaw_damping_nm_per_radps": 0.08,
        }
        with mock.patch.object(dyn, "load_fitted_dynamics", side_effect=AssertionError("manifest should not be read")):
            world = dyn.MasterPiDynamicsV2(seed=4, dynamics=explicit, use_calibration_manifest=False)
        try:
            self.assertEqual(world.calibration_status, "EXPLICIT_DYNAMICS_OVERRIDE")
        finally:
            world.close()


if __name__ == "__main__":
    unittest.main()
