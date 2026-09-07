import os
import unittest
from unittest import mock
from pathlib import Path

from scripts import gpu_worker_recover as gpu


class GPURecoveryPolicyTests(unittest.TestCase):
    def test_default_provider_is_lightning_only(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(gpu.provider_order(), ["lightning"])

    def test_azure_requires_explicit_opt_in(self):
        with mock.patch.dict(os.environ, {"UGRP_GPU_PROVIDERS": "azure"}, clear=True), \
             mock.patch.object(gpu, "AZURE", Path("/bin/sh")), \
             mock.patch.object(gpu, "AZURE_KEY", Path("/bin/sh")):
            self.assertFalse(gpu.provider_configured("azure"))
        with mock.patch.dict(
            os.environ,
            {"UGRP_GPU_PROVIDERS": "azure", "UGRP_ALLOW_AZURE_WORKER": "1"},
            clear=True,
        ), mock.patch.object(gpu, "AZURE", Path("/bin/sh")), mock.patch.object(gpu, "AZURE_KEY", Path("/bin/sh")):
            self.assertTrue(gpu.provider_configured("azure"))
            self.assertEqual(gpu.command_for("azure")[-1], "/bin/sh")

    def test_colab_requires_explicit_opt_in(self):
        with mock.patch.dict(os.environ, {"UGRP_GPU_PROVIDERS": "colab"}, clear=True), mock.patch.object(gpu, "COLAB", Path("/bin/sh")):
            self.assertFalse(gpu.provider_configured("colab"))
        with mock.patch.dict(
            os.environ,
            {"UGRP_GPU_PROVIDERS": "colab", "UGRP_ALLOW_COLAB_WORKER": "1"},
            clear=True,
        ), mock.patch.object(gpu, "COLAB", Path("/bin/sh")):
            self.assertTrue(gpu.provider_configured("colab"))

    def test_colab_can_precede_lightning_when_explicitly_configured(self):
        with mock.patch.dict(
            os.environ, {"UGRP_GPU_PROVIDERS": "colab,lightning", "UGRP_ALLOW_COLAB_WORKER": "1"}, clear=True
        ), mock.patch.object(gpu, "COLAB", Path("/bin/sh")):
            self.assertEqual(gpu.configured_providers(), ["colab"])

    def test_remote_ready_requires_authority_and_provider_match(self):
        with mock.patch.object(gpu, "remote_health", return_value={
            "remote_ws_connected": True, "remote_authoritative": True, "remote_provider": "colab",
            "remote_worker_contract": gpu.REMOTE_WORKER_CONTRACT,
        }):
            self.assertTrue(gpu.remote_ready())
            self.assertTrue(gpu.remote_ready("colab"))
            self.assertFalse(gpu.remote_ready("azure"))
        with mock.patch.object(gpu, "remote_health", return_value={
            "remote_ws_connected": True, "remote_authoritative": False, "remote_provider": "colab",
            "remote_worker_contract": gpu.REMOTE_WORKER_CONTRACT,
        }):
            self.assertFalse(gpu.remote_ready("colab"))

    def test_provider_order_deduplicates(self):
        with mock.patch.dict(
            os.environ, {"UGRP_GPU_PROVIDERS": "lightning, colab,lightning"}, clear=True
        ):
            self.assertEqual(gpu.provider_order(), ["lightning", "colab"])


if __name__ == "__main__":
    unittest.main()
