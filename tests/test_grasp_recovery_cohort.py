import hashlib
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image
from scripts.run_grasp_recovery_cohort import compare_initial_rgb


class PairingTests(unittest.TestCase):
    def test_small_raster_noise_allowed_but_state_image_change_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            a = np.full((100, 100, 3), 100, dtype=np.uint8)
            Image.fromarray(a).save(root / 'a.png')
            def record(name):
                item = {'path': name, 'sha256': hashlib.sha256((root / name).read_bytes()).hexdigest()}
                return {'r1': {'own': item, 'top': item}}
            first = record('a.png')
            a[0, 0, 0] += 2
            Image.fromarray(a).save(root / 'b.png')
            metrics = compare_initial_rgb(root, first, root, record('b.png'))
            self.assertFalse(metrics['r1/own']['byte_exact'])
            self.assertEqual(metrics['r1/own']['max_abs'], 2)
            a[0, 0, 0] += 2
            Image.fromarray(a).save(root / 'c.png')
            with self.assertRaisesRegex(ValueError, 'initial RGB differs'):
                compare_initial_rgb(root, first, root, record('c.png'))
            a[:] = 101
            Image.fromarray(a).save(root / 'd.png')
            with self.assertRaisesRegex(ValueError, 'initial RGB differs'):
                compare_initial_rgb(root, first, root, record('d.png'))


if __name__ == '__main__':
    unittest.main()
