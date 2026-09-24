"""Check that realtime renewal commands cannot silently change ACT labels."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from scripts.build_carry_act_data import aligned_actions, merge_base, sha


def command(issued_at, forward, duration=.25):
    return {'stage': 'TRANSIT', 'issued_at_s': issued_at,
            'action': {'kind': 'mecanum', 'forward': forward, 'left': 0.,
                       'turn': 0., 'duration_s': duration}}


class RealtimeAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.rows = [
            {'sim_time_s': 1., 'frame_ids': {'r1': 10}},
            {'sim_time_s': 1.5, 'frame_ids': {'r1': 15}},
            {'sim_time_s': 2., 'frame_ids': {'r1': 20}},
        ]
        self.renewals = [
            {'kind': 'carry_pending_renewal', 'reason': 'same_accepted_cruise_while_next_rgb_pending',
             'issued_at_s': 1.25, 'duration_s': .1, 'source_frame_ids': {'r1': 10}},
        ]
        self.commands = [
            command(1., .1), command(1.25, .1, .1),
            command(1.5, -.1), command(2., 0., .2),
        ]

    def test_realtime_renewal_is_verified_but_not_sampled(self):
        matched, audit = aligned_actions(self.rows, self.renewals, self.commands)
        self.assertEqual([v['issued_at_s'] for v in matched], [1., 1.5, 2.])
        self.assertEqual(audit, {'decision_commands': 3, 'renewal_commands': 1,
                                 'transit_commands': 4})

    def test_reject_changed_renewal_action(self):
        commands = copy.deepcopy(self.commands)
        commands[1]['action']['forward'] = -.1
        with self.assertRaisesRegex(ValueError, 'changed accepted action'):
            aligned_actions(self.rows, self.renewals, commands)

    def test_reject_unaccounted_command(self):
        with self.assertRaisesRegex(ValueError, 'count mismatch'):
            aligned_actions(self.rows, self.renewals,
                            self.commands + [command(1.75, .1)])

    def test_legacy_terminal_without_command(self):
        matched, audit = aligned_actions(self.rows, [], [command(1., .1),
                                                          command(1.5, -.1)])
        self.assertIsNone(matched[-1])
        self.assertEqual(audit['renewal_commands'], 0)


class DatasetSplitTest(unittest.TestCase):
    def test_shared_images_must_stay_in_one_full_episode_split(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = []
            for name in ('a', 'b'):
                episode_root = root / name
                episode_root.mkdir()
                (episode_root / 'image.jpg').write_bytes(b'shared source image')
                episode = {'root': str(episode_root), 'files': {},
                           'image_hashes': {'image.jpg': sha(episode_root / 'image.jpg')},
                           'sequences': {'r1': []}}
                episodes.append(episode)
            base = root / 'base.json'
            base.write_text(json.dumps({'train': episodes, 'development': []}))
            result = {'train': [], 'development': []}
            with self.assertRaisesRegex(ValueError, 'identical source images'):
                merge_base(result, base, sha(base), [root / 'a'])
            merged = merge_base(result, base, sha(base), [root / 'a', root / 'b'])
            self.assertEqual(len(merged['development']), 2)
            self.assertEqual(merged['train'], [])


if __name__ == '__main__':
    unittest.main()
