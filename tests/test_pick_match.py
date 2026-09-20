import unittest

from scripts.run_pick_match import score_grasp


def rows(lift=.05, contact=True, weld=False, count=25):
    return [{'elapsed_sim_s': i / 10, 'lift_m': lift, 'bilateral_contact': contact,
             'constraints_active': {'r1': weld, 'r2': False, 'r3': False}}
            for i in range(count)]


class RefereeTests(unittest.TestCase):
    def test_continuous_unassisted_hold(self):
        self.assertTrue(score_grasp(rows())['physics_grasp_success'])

    def test_no_grasp_from_height_or_contact_alone(self):
        self.assertFalse(score_grasp(rows(contact=False))['physics_grasp_success'])
        self.assertFalse(score_grasp(rows(lift=.02))['physics_grasp_success'])

    def test_assistance_invalidates_episode_even_before_hold(self):
        data = rows(count=40)
        data[0]['constraints_active']['r1'] = True
        self.assertFalse(score_grasp(data)['physics_grasp_success'])

    def test_gaps_cannot_count_towards_hold(self):
        data = rows(count=20)
        data[10:] = [{**r, 'elapsed_sim_s': r['elapsed_sim_s'] + 10} for r in data[10:]]
        self.assertFalse(score_grasp(data)['physics_grasp_success'])

    def test_missing_constraint_evidence_is_rejected(self):
        data = rows()
        data[0]['constraints_active'] = {}
        with self.assertRaises(ValueError):
            score_grasp(data)


if __name__ == '__main__':
    unittest.main()
