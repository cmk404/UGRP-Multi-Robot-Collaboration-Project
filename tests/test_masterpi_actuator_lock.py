import importlib.util
import tempfile
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('masterpi_lock', ROOT/'scripts'/'masterpi_control.py')
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)

class MasterPiActuatorLockTests(unittest.TestCase):
    def test_second_writer_is_rejected_until_first_releases(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'actuator.lock'
            first=mod.acquire_actuator_lease(owner='first',path=p)
            try:
                with self.assertRaises(mod.ControlError) as cm:
                    mod.acquire_actuator_lease(owner='second',path=p)
                self.assertIn('first',str(cm.exception))
            finally:
                first.close()
            second=mod.acquire_actuator_lease(owner='second',path=p)
            second.close()

if __name__=='__main__': unittest.main()
