from __future__ import annotations

import tarfile
import tempfile
from pathlib import Path
import unittest

from scripts.colab_worker_recover import BUNDLE_FILES, make_bundle, parse_sessions
from scripts.gpu_worker_bundle import REQUIREMENTS
from scripts.sim_worker_failover import RECOVERY_COOLDOWN, RECOVERY_MISS_LIMIT, POLL, REMOTE_WORKER_CONTRACT, provider_is_allowed, should_start_recovery, worker_contract_is_allowed


class ColabRecoveryTests(unittest.TestCase):
    def test_bundle_covers_every_red_block_python_module(self):
        root = Path(__file__).resolve().parents[1]
        red_block = root / "scripts" / "red_block"
        expected = {str(path.relative_to(root)) for path in red_block.glob("*.py")}
        bundled = {rel for rel in BUNDLE_FILES if rel.startswith("scripts/red_block/")}
        self.assertEqual(bundled, expected)

    def test_gpu_watchdog_is_fast_and_has_no_15s_cpu_fallback(self):
        self.assertLessEqual(POLL * RECOVERY_MISS_LIMIT, 3.0)


    def test_watchdog_reloads_gpu_env_on_each_probe(self):
        src=(Path(__file__).resolve().parents[1]/"scripts/sim_worker_failover.py").read_text()
        self.assertIn(". ./.env.gpu", src)
        self.assertIn("gpu_worker_recover.py --preflight", src)

    def test_lightning_config_helper_uses_hidden_input(self):
        src=(Path(__file__).resolve().parents[1]/"scripts/configure_lightning_gpu.sh").read_text()
        self.assertIn("read -r -s KEY", src)
        self.assertNotIn("echo $KEY", src)


    def test_lightning_recovery_has_running_studio_fast_path_and_no_detached_sdk_supervisor(self):
        src=(Path(__file__).resolve().parents[1]/"scripts/lightning_worker_recover.py").read_text()
        self.assertIn("fast_restart_existing", src)
        self.assertIn("run_and_detach(", src)
        self.assertNotIn("nohup bash supervise_worker.sh", src)

    def test_watchdog_does_not_restart_coworker_on_gpu_reconnect(self):
        src=(Path(__file__).resolve().parents[1]/"scripts/sim_worker_failover.py").read_text()
        self.assertNotIn("restart_coworker()", src)
        self.assertNotIn("systemctl('restart',COWORKER)", src)

    def test_parse_sessions_extracts_named_colab_sessions(self):
        text = """[ugrp-gpu] gpu-t4-abc | Hardware: T4 | Variant: GPU\n[other] cpu-xyz | Hardware: CPU\n"""
        self.assertEqual(parse_sessions(text), {"ugrp-gpu", "other"})

    def test_bundle_contains_only_runtime_code_not_secrets(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "bundle.tgz"
            make_bundle(path)
            with tarfile.open(path, "r:gz") as tf:
                names = set(tf.getnames())
        self.assertEqual(names, set(BUNDLE_FILES))
        self.assertIn("sim/masterpi_dynamics_v2.py", names)
        self.assertIn("sim/worker_contract.py", names)
        self.assertIn("sim/masterpi_dynamics_calibration.json", names)
        self.assertIn("sim/real_stack_adapter.py", names)
        self.assertIn("scripts/red_block/recorder.py", names)
        self.assertTrue(any(req.startswith("opencv-python-headless==") for req in REQUIREMENTS))
        self.assertTrue(any(req.startswith("gymnasium==") for req in REQUIREMENTS))
        self.assertTrue(all("==" in req for req in REQUIREMENTS))
        self.assertFalse(any("token" in name.lower() or "groq" in name.lower() for name in names))

    def test_recovery_waits_until_miss_threshold(self):
        self.assertFalse(should_start_recovery(RECOVERY_MISS_LIMIT - 1, now=100, last_attempt=0, active=False))
        self.assertTrue(should_start_recovery(RECOVERY_MISS_LIMIT, now=100, last_attempt=0, active=False))

    def test_unconfigured_remote_provider_cannot_receive_authority(self):
        self.assertTrue(provider_is_allowed("colab", {"colab"}))
        self.assertFalse(provider_is_allowed("azure", {"colab"}))
        self.assertFalse(provider_is_allowed(None, {"colab"}))

    def test_legacy_worker_contract_cannot_receive_authority(self):
        self.assertTrue(worker_contract_is_allowed(REMOTE_WORKER_CONTRACT))
        self.assertFalse(worker_contract_is_allowed("legacy"))
        self.assertFalse(worker_contract_is_allowed(None))

    def test_recovery_requires_recent_user_activity(self):
        self.assertFalse(should_start_recovery(999, now=100, last_attempt=0, active=False, recent_activity=False))
        self.assertTrue(should_start_recovery(999, now=100, last_attempt=0, active=False, recent_activity=True))

    def test_recovery_respects_active_service_and_cooldown(self):
        self.assertFalse(should_start_recovery(999, now=100, last_attempt=0, active=True))
        self.assertFalse(should_start_recovery(999, now=100, last_attempt=99, active=False))
        self.assertTrue(
            should_start_recovery(
                999,
                now=100 + RECOVERY_COOLDOWN,
                last_attempt=100,
                active=False,
            )
        )


if __name__ == "__main__":
    unittest.main()
