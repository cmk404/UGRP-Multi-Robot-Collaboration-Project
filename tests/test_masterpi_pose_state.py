import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('masterpi_pose_state', ROOT/'scripts'/'masterpi_control.py')
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)

class MasterPiPoseStateTests(unittest.TestCase):
    def test_record_round_trip_and_metadata(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'pose.json'
            mod.record_servo_command(3,740,path=p,writer='test-a')
            mod.record_servo_command(6,1231,path=p,writer='test-b')
            self.assertEqual(mod.load_pose_state(p),{3:740,6:1231})
            raw=json.loads(p.read_text())
            self.assertEqual(raw['last_writer'],'test-b')
            self.assertEqual(raw['last_servo'],6)
    def test_dry_run_does_not_record(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'pose.json'
            with mock.patch.object(mod,'POSE_STATE_PATH',p):
                t=mod.ServoTransport(dry_run=True)
                t.write_servo(3,740,.2)
            self.assertFalse(p.exists())
    def test_live_transport_records_only_after_success(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'pose.json'
            with mock.patch.object(mod,'POSE_STATE_PATH',p), \
                 mock.patch.object(mod,'record_servo_command') as rec, \
                 mock.patch.object(mod.subprocess,'run') as run:
                t=mod.ServoTransport(dry_run=False)
                t.write_servo(5,1320,.2)
                run.assert_called_once()
                rec.assert_called_once()
                self.assertEqual(rec.call_args.args[:2],(5,1320))

if __name__=='__main__': unittest.main()
