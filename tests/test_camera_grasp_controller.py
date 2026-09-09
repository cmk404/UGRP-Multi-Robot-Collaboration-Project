import unittest
from harness.camera_grasp_controller import CameraGraspController


def observation(**updates):
    obs = dict(view='own', jaws=[[.4, .5], [.6, .5]], target=[.5, .5],
               confidence=.9, identity_confidence=.9, capture_visible=False,
               lift_visible=False, suggested_action={'kind': 'wait'})
    obs.update(updates)
    return obs


class ControllerTests(unittest.TestCase):
    def test_close_requires_visible_alignment_and_does_not_claim_capture(self):
        c = CameraGraspController()
        self.assertEqual(c.step(observation())['pulse'], 2000)
        c.step(observation())
        self.assertEqual(c.step(observation())['pulse'], 1500)
        self.assertEqual(c.stage, 'verify_close')
        c.step(observation())
        self.assertEqual(c.step(observation())['pulse'], 2000)
        self.assertNotEqual(c.stage, 'hold')

    def test_hallucinated_gripper_proposal_cannot_bypass_missing_landmarks(self):
        c = CameraGraspController()
        obs = observation(jaws=None, suggested_action={'kind': 'arm', 'servo_id': 1, 'pulse': 1500})
        c.step(obs)
        for _ in range(14):
            action = c.step(obs)
            self.assertFalse(action.get('servo_id') == 1 and action.get('pulse') == 1500)
        self.assertEqual(c.stage, 'blocked')

    def test_own_command_delta_is_bounded_and_reverse_available(self):
        c = CameraGraspController()
        c.step(observation())
        a = c.step(observation(jaws=None, suggested_action={'kind': 'arm', 'servo_id': 4, 'pulse': 2500}))
        self.assertEqual(a['pulse'], 1600)
        a = c.step(observation(jaws=None, suggested_action={'kind': 'drive', 'forward': -.05, 'turn': 0, 'duration_s': .4}))
        self.assertEqual(a['forward'], -.05)

    def test_peer_turn_never_moves_and_uncertain_identity_cannot_close(self):
        c = CameraGraspController()
        obs = observation(view='overhead', identity_confidence=.1)
        self.assertEqual(c.step(obs, active=False), {'kind': 'wait'})
        c.step(obs)
        for _ in range(3):
            self.assertNotEqual(c.step(obs).get('pulse'), 1500)

    def test_view_switch_is_not_two_consecutive_alignment_observations(self):
        c = CameraGraspController()
        c.step(observation())
        c.step(observation())
        action = c.step(observation(view='overhead'))
        self.assertFalse(action.get('servo_id') == 1 and action.get('pulse') == 1500)

    def test_low_identity_cannot_verify_capture_or_lift(self):
        c = CameraGraspController()
        c.stage = 'verify_close'
        obs = observation(view='overhead', identity_confidence=.2, capture_visible=True, lift_visible=True)
        self.assertEqual(c.step(obs), {'kind': 'wait'})
        self.assertEqual(c.stage, 'verify_close')


if __name__ == '__main__':
    unittest.main()
