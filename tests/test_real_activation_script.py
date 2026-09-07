import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class RealActivationScriptTests(unittest.TestCase):
    def test_activation_never_kills_or_stops_robot(self):
        s=(ROOT/'scripts/activate_real_spatial_stack.sh').read_text()
        self.assertNotIn('kill ',s)
        self.assertNotIn('masterpi_control.py stop',s)
        self.assertIn('LOCK_EX|fcntl.LOCK_NB',s)
        self.assertIn('systemctl --user restart ugrp-real.service',s)
        self.assertNotIn('restart ugrp-camera.service',s)
        self.assertIn('-o User="$ROBOT_USER"',s)
        self.assertIn('-o HostName="$ROBOT_HOST"',s)
        self.assertIn('-o HostKeyAlias="$ROBOT_HOSTKEY_ALIAS"',s)
        self.assertIn('ROBOT_HOST="${UGRP_ROBOT_HOST:-100.119.44.65}"',s)
        self.assertIn('for _ in $(seq 1 20)',s)
        self.assertIn('/api/status',s)
        self.assertNotIn('sleep 2',s)
        self.assertIn('UGRP_REAL_RESUME_GRASP_PATH',s)
        self.assertIn('PROBABLE_HELD',s)
        self.assertIn('held_object_color',s)
        self.assertIn('harness.real_carry',s)
        self.assertIn('carry.get("valid") is True',s)
if __name__=='__main__': unittest.main()
