import json, unittest
from harness.real_pose import parse_commanded_pose, PoseStabilityTracker, POSE_STREAM_CMD

class RealPoseTests(unittest.TestCase):
    def test_parse(self):
        raw=json.dumps({'pose':{'3':740,'4':2320,'5':1320,'6':1500},'updated_at':100.0,'last_writer':'x','last_servo':6,'last_pulse':1500})
        p=parse_commanded_pose(raw,now=100.5)
        self.assertEqual(p.pose[6],1500); self.assertAlmostEqual(p.age_s,.5); self.assertEqual(p.last_writer,'x')
    def test_pose_stream_frames_each_json_with_newline(self):
        self.assertIn("printf", POSE_STREAM_CMD)
        self.assertIn(chr(92) + "n", POSE_STREAM_CMD)

    def test_stability_requires_repeat(self):
        raw=json.dumps({'pose':{'3':740,'4':2320,'5':1320,'6':1500},'updated_at':100.0})
        p=parse_commanded_pose(raw,now=100.5)
        t=PoseStabilityTracker(min_settle_s=.18)
        self.assertFalse(t.observe(p)); self.assertTrue(t.observe(p))

if __name__=='__main__': unittest.main()
