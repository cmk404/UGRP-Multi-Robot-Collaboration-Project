import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class RealRadarUiTests(unittest.TestCase):
    def test_real_mode_keeps_radar_and_uses_real_status(self):
        html=(ROOT/'harness/static/index.html').read_text()
        js=(ROOT/'harness/static/sim3d.js').read_text()
        self.assertIn('semanticPanel.style.display = "block"',html)
        # REAL mode polls the robot-scoped status through the per-robot API
        # prefix (window.UGRP_API_PREFIX), never a hard-coded /real path.
        self.assertIn("if(currentMode==='real'){",js)
        self.assertIn("fetch(apiPath('/api/status')",js)
    def test_real_mode_has_no_sim_fallback_zones(self):
        js=(ROOT/'harness/static/sim3d.js').read_text()
        self.assertIn("currentMode==='sim'?fallbackZones:[]",js)
        self.assertIn("DEPTH 준비 전",js)
        self.assertIn("setMode",js)
if __name__=='__main__': unittest.main()
