import math
import unittest

from harness.approach_geometry import assess_face_standoff


class FaceStandoffTest(unittest.TestCase):
    def test_recorded_seed42_rgb_replay_reaches_standoff_before_camera_edge(self):
        # Recomputed from stored wrist JPEGs, recorded PWM, CameraBoxTracker,
        # and camera_extrinsics. Both qualify before E4 loses the marker.
        replayed = (
            ((-0.02141108, 0.32807394), (0.31389, -0.94946)),  # E3 step 74
            ((-0.02605170, 0.32868857), (0.30878, -0.95114)),  # E4 step 75
        )
        for target, normal in replayed:
            with self.subTest(target=target):
                result = assess_face_standoff(target, normal)
                self.assertTrue(result.reached)
                self.assertLess(abs(result.radial_distance_m - 0.35), 0.055)
                self.assertGreaterEqual(result.face_alignment, math.cos(math.radians(15)))

    def test_earlier_oblique_replay_does_not_finish_face_positioning(self):
        # E4 step 64 has the right range but its face is still about 24 degrees
        # oblique, so the controller must continue face positioning.
        result = assess_face_standoff((0.04137717, 0.32818826), (0.28660, -0.95805))
        self.assertFalse(result.reached)
        self.assertLess(result.face_alignment, math.cos(math.radians(15)))

    def test_small_replay_perturbations_remain_inside_existing_tolerances(self):
        target = (-0.02605170, 0.32868857)
        base_angle = math.atan2(-0.95114, 0.30878)
        for range_delta in (-0.004, 0.004):
            scale = (math.hypot(*target) + range_delta) / math.hypot(*target)
            perturbed_target = (target[0] * scale, target[1] * scale)
            for angle_delta_deg in (-1.0, 1.0):
                angle = base_angle + math.radians(angle_delta_deg)
                result = assess_face_standoff(
                    perturbed_target, (math.cos(angle), math.sin(angle))
                )
                self.assertTrue(result.reached)

    def test_aligned_face_outside_standoff_band_is_not_reached(self):
        result = assess_face_standoff((0.44, 0.0), (-1.0, 0.0))
        self.assertFalse(result.reached)


if __name__ == "__main__":
    unittest.main()
