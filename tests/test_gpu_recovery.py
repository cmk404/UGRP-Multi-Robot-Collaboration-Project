import os
import unittest
from pathlib import Path
from unittest import mock

from scripts import gpu_worker_recover as recovery
from scripts.sim_worker_failover import (
    RECOVERY_COOLDOWN,
    RECOVERY_MISS_LIMIT,
    REMOTE_WORKER_CONTRACT,
    provider_is_allowed,
    should_start_recovery,
    worker_contract_is_allowed,
)


class MacWorkerRecoveryPolicyTests(unittest.TestCase):
    def test_mac_is_the_only_supported_worker_location(self):
        with mock.patch.dict(os.environ, {"UGRP_GPU_PROVIDERS": "lightning,azure,colab"}, clear=True):
            self.assertEqual(recovery.provider_order(), ["mac"])
            self.assertEqual(recovery.configured_providers(), [])

    def test_mac_recovery_requires_explicit_enablement(self):
        with mock.patch.dict(os.environ, {}, clear=True), mock.patch.object(recovery, "MAC", Path("/bin/sh")):
            self.assertFalse(recovery.provider_configured("mac"))
        with mock.patch.dict(os.environ, {"UGRP_ALLOW_MAC_WORKER": "1"}, clear=True), mock.patch.object(
            recovery, "MAC", Path("/bin/sh")
        ):
            self.assertTrue(recovery.provider_configured("mac"))
            self.assertEqual(recovery.command_for("mac")[-1], "/bin/sh")

    def test_retired_cloud_names_are_never_configured(self):
        with mock.patch.dict(
            os.environ,
            {"UGRP_ALLOW_AZURE_WORKER": "1", "UGRP_ALLOW_COLAB_WORKER": "1"},
            clear=True,
        ):
            for name in ("lightning", "azure", "colab"):
                self.assertFalse(recovery.provider_configured(name))
                with self.assertRaises(ValueError):
                    recovery.command_for(name)

    def test_remote_ready_requires_mac_authority_and_current_contract(self):
        healthy = {
            "remote_ws_connected": True,
            "remote_authoritative": True,
            "remote_provider": "mac",
            "remote_worker_contract": recovery.REMOTE_WORKER_CONTRACT,
        }
        with mock.patch.object(recovery, "remote_health", return_value=healthy):
            self.assertTrue(recovery.remote_ready())
            self.assertTrue(recovery.remote_ready("mac"))
            self.assertFalse(recovery.remote_ready("azure"))
        with mock.patch.object(recovery, "remote_health", return_value={**healthy, "remote_authoritative": False}):
            self.assertFalse(recovery.remote_ready())
        with mock.patch.object(recovery, "remote_health", return_value={**healthy, "remote_worker_contract": "legacy"}):
            self.assertFalse(recovery.remote_ready())

    def test_watchdog_waits_for_threshold_and_recent_activity(self):
        self.assertFalse(
            should_start_recovery(
                RECOVERY_MISS_LIMIT - 1,
                now=100,
                last_attempt=0,
                active=False,
            )
        )
        self.assertFalse(
            should_start_recovery(
                RECOVERY_MISS_LIMIT,
                now=100,
                last_attempt=0,
                active=False,
                recent_activity=False,
            )
        )
        self.assertTrue(
            should_start_recovery(
                RECOVERY_MISS_LIMIT,
                now=100,
                last_attempt=0,
                active=False,
            )
        )

    def test_watchdog_respects_active_recovery_and_cooldown(self):
        self.assertFalse(
            should_start_recovery(999, now=100, last_attempt=0, active=True)
        )
        self.assertFalse(
            should_start_recovery(999, now=100, last_attempt=99, active=False)
        )
        self.assertTrue(
            should_start_recovery(
                999,
                now=100 + RECOVERY_COOLDOWN,
                last_attempt=100,
                active=False,
            )
        )

    def test_watchdog_accepts_only_configured_mac_and_current_contract(self):
        self.assertTrue(provider_is_allowed("mac", {"mac"}))
        self.assertFalse(provider_is_allowed("azure", {"mac"}))
        self.assertFalse(provider_is_allowed(None, {"mac"}))
        self.assertTrue(worker_contract_is_allowed(REMOTE_WORKER_CONTRACT))
        self.assertFalse(worker_contract_is_allowed("legacy"))
        self.assertFalse(worker_contract_is_allowed(None))


if __name__ == "__main__":
    unittest.main()
