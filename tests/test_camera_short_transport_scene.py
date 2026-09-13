import unittest
from types import SimpleNamespace

from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.run_camera_short_transport_student import choose_carry_actions


class ShortSceneTests(unittest.TestCase):
    def fake_scene(self):
        scene = ShortTransportScene.__new__(ShortTransportScene)
        scene.trace = []
        scene.phase = 'grasp_hold'
        scene.time = lambda: 10.
        scene.tick = lambda duration: None
        scene.applied = []
        scene.ports = {r: SimpleNamespace(apply=lambda action, stamp, r=r:
                        scene.applied.append((r, action, stamp))) for r in ('r1', 'r3')}
        return scene

    def test_invalid_peer_command_rejects_both_before_motion(self):
        for bad in (float('nan'), -.01, .11, True):
            scene = self.fake_scene()
            with self.assertRaises(ValueError):
                scene.carry_drive({'r1': .05, 'r3': bad})
            self.assertEqual(scene.applied, [])

    def test_carry_preserves_loaded_phase_and_shared_dispatch_time(self):
        scene = self.fake_scene()
        scene.carry_drive({'r1': .05, 'r3': .06})
        self.assertEqual(scene.phase, 'carry')
        self.assertEqual([x[2] for x in scene.applied], [10., 10.])
        scene.carry_drive({'r1': 0., 'r3': 0.}, stage='carry_stop')
        self.assertEqual(scene.phase, 'carry_stop')

    def test_place_uses_saved_issued_targets_without_port_arm_state(self):
        scene = self.fake_scene()
        scene.commands = {r: {3: 600, 4: 700, 5: 800} for r in ('r1', 'r3')}
        preclose = {r: {3: 1000, 4: 900, 5: 1100} for r in ('r1', 'r3')}
        scene.grasp_report = {'preclose_issued_commands': preclose}
        replays = []
        scene.replay = lambda commands, stage: replays.append((commands, stage))
        scene.capture = lambda tag: {'tag': tag}
        scene.place()
        self.assertEqual(replays[0][0][0]['targets'], preclose)
        self.assertEqual(replays[1][0][0]['targets'], {'r1': {1: 2000}, 'r3': {1: 2000}})
        self.assertEqual(replays[2][0][0]['targets'], scene.commands)
        self.assertEqual(scene.applied, [])
        self.assertEqual(scene.phase, 'release_hold')

    def test_rgb_invalid_peer_stops_both_and_confirmation_never_moves(self):
        good = {'ok': True, 'held_estimate': True, 'ready': False, 'forward': .1}
        rows = {'r1': good, 'r3': {**good, 'held_estimate': False}}
        self.assertEqual(choose_carry_actions(rows)['forwards'], {'r1': 0., 'r3': 0.})
        rows['r3'] = good
        self.assertEqual(choose_carry_actions(rows, True)['forwards'], {'r1': 0., 'r3': 0.})


if __name__ == '__main__':
    unittest.main()
