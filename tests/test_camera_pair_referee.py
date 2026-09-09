import unittest

from scripts.run_camera_pair_transport import evaluate_grasp_samples


def sample(t, *, lift=0.03, r1=True, r3=True, weld=False):
    return {
        'sim_time_s': t,
        'height_above_start_m': lift,
        'contacts': {
            'r1': {'bilateral': r1},
            'r3': {'bilateral': r3},
        },
        'constraints_active': {'r1': weld, 'r3': False},
    }


class CameraPairRefereeTests(unittest.TestCase):
    def test_accepts_threshold_for_twenty_one_contiguous_samples(self):
        result = evaluate_grasp_samples([sample(i / 10) for i in range(21)])
        self.assertTrue(result['grasp_success'])
        self.assertEqual(result['longest_qualifying_duration_s'], 2.0)
        self.assertEqual(result['max_lift_m'], 0.03)

    def test_drop_breaks_contiguous_hold(self):
        samples = [sample(i / 10) for i in range(25)]
        samples[10] = sample(1.0, lift=0.029)
        result = evaluate_grasp_samples(samples)
        self.assertFalse(result['grasp_success'])
        self.assertEqual(result['longest_qualifying_duration_s'], 1.3)

    def test_requires_bilateral_contact_from_both_robots(self):
        samples = [sample(i / 10, r3=False) for i in range(20)]
        self.assertFalse(evaluate_grasp_samples(samples)['grasp_success'])

    def test_rejects_weld_assistance(self):
        samples = [sample(i / 10, weld=True) for i in range(20)]
        self.assertFalse(evaluate_grasp_samples(samples)['grasp_success'])


if __name__ == '__main__':
    unittest.main()
