import unittest
from unittest.mock import patch
from harness.camera_grasp_controller import CameraGraspController, STARTUP_COMMANDS


def observation(**updates):
    obs = dict(view='own', jaws=[[.4, .5], [.6, .5]], target=[.5, .5],
               confidence=.9, identity_confidence=.9, capture_visible=False,
               lift_visible=False, suggested_action={'kind': 'wait'})
    obs.update(updates)
    return obs


class ControllerTests(unittest.TestCase):
    def test_close_requires_visible_alignment_and_does_not_claim_capture(self):
        c = CameraGraspController()
        c.calibration_index = 8
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
            self.assertEqual(c.step(obs)['kind'], 'look')

    def test_view_switch_is_not_two_consecutive_alignment_observations(self):
        c = CameraGraspController()
        c.calibration_index = 8
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

    def test_explicit_startup_commands_prevent_unknown_neutral_jump(self):
        c = CameraGraspController(STARTUP_COMMANDS)
        action = c.step(observation(jaws=None, suggested_action={'kind': 'arm', 'servo_id': 3, 'pulse': 800}))
        self.assertEqual(action['pulse'], 800)
        self.assertEqual(c.pending['_delta'], 60)

    def test_repeated_drive_remains_available_for_coarse_approach(self):
        c = CameraGraspController(STARTUP_COMMANDS)
        c.calibration_index = 8
        drive = {'kind': 'drive', 'forward': .05, 'turn': 0, 'duration_s': .4}
        obs = observation(target=[.8, .5], suggested_action=drive)
        expected = {**drive, 'forward': .15, 'duration_s': .8}
        self.assertEqual(c.step(obs), expected)
        self.assertEqual(c.step(obs), expected)
        self.assertEqual(c.stage, 'approach')

    def test_bidirectional_calibration_does_not_close_or_change_startup_pose(self):
        c = CameraGraspController(STARTUP_COMMANDS)
        initial = dict(c.issued_pulses)
        actions = [c.step(observation(target=[.8, .5])) for _ in range(8)]
        self.assertEqual(c.issued_pulses, initial)
        self.assertEqual(c.calibration_index, 8)
        self.assertTrue(all(a.get('servo_id') != 1 for a in actions))

    def test_local_step_is_reverted_when_observed_error_worsens(self):
        c = CameraGraspController(STARTUP_COMMANDS)
        c.calibration_index = 8
        proposal = {'kind': 'arm', 'servo_id': 3, 'pulse': 790}
        with patch.object(c.model, 'propose', return_value=proposal):
            self.assertEqual(c.step(observation(target=[.59, .5])), proposal)
            self.assertEqual(c.step(observation(target=[.61, .5])),
                             {'kind': 'arm', 'servo_id': 3, 'pulse': 740})
        self.assertEqual(len(c.rejected_actions), 1)

    def test_stalled_alignment_can_reposition_base(self):
        c = CameraGraspController(STARTUP_COMMANDS)
        c.calibration_index = 8
        obs = observation(target=[.59, .5], suggested_action={'kind': 'drive', 'forward': .04, 'turn': 0, 'duration_s': .2})
        with patch.object(c.model, 'propose', return_value=None):
            actions = [c.step(obs) for _ in range(7)]
        self.assertEqual(actions[-1]['kind'], 'drive')


if __name__ == '__main__':
    unittest.main()
