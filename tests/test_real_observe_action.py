import unittest
from unittest import mock
import scripts.robot_actions as ra
class RealObserveActionTests(unittest.TestCase):
    def test_catalog_has_read_only_observe(self):
        self.assertIn('observe_scene',ra.ACTIONS)
        self.assertEqual(ra.CONTRACTS['observe_scene']['preconditions'],[])
    def test_dispatch_does_not_use_red_block_script(self):
        with mock.patch('scripts.real_observe_scene.main',return_value=0) as f:
            self.assertEqual(ra.main(['observe_scene']),0); f.assert_called_once()
if __name__=='__main__':unittest.main()

class RealObserveStructuredResultTests(unittest.TestCase):
    def test_run_returns_structured_observation(self):
        fake={'ok':True,'skill':'observe_scene','detections':{'red':{'visible':False}},'metric_estimates':{},'outcome_status':'ACHIEVED'}
        with mock.patch('scripts.real_observe_scene.run',return_value=dict(fake)) as f:
            out=ra.run('observe_scene',['--host','fakepi'])
        self.assertEqual(out['detections'],fake['detections'])
        self.assertEqual(out['argv'][0],'observe_scene')
        f.assert_called_once_with('fakepi')
