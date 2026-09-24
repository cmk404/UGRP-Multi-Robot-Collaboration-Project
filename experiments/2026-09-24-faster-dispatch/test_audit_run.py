"""Small actual-schema regression for the independent v25 fine bridge audit."""

import importlib.util
import json
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("faster_dispatch_audit", HERE / "audit_run.py")
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)
FIXTURE = json.loads((HERE / "audit-v25-fine-fixture.json").read_text())


class FineBridgeAuditTest(unittest.TestCase):
    def audit(self, pair_decisions):
        return AUDIT.audit_pair_fine_bridges(
            pair_decisions, FIXTURE["issued_commands"], FIXTURE["result"])

    def test_real_receipt_schema_passes(self):
        result = self.audit(FIXTURE["pair_decisions"])
        self.assertEqual(result["status"], "pass", result["errors"])
        self.assertEqual(result["issued_by_slot"], {"r1": 1, "r3": 1})

    def test_near_ready_or_excess_extension_fails(self):
        rows = json.loads(json.dumps(FIXTURE["pair_decisions"]))
        receipt = rows[1]
        receipt["far_error_m"]["r1"] = .001
        receipt["total_extension_s"] = .06
        result = self.audit(rows)
        self.assertEqual(result["status"], "fail")
        self.assertTrue(any("near ready" in error for error in result["errors"]))
        self.assertTrue(any("exceeds .05" in error for error in result["errors"]))


if __name__ == "__main__":
    unittest.main()
