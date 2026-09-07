#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "scripts" / "red_block"


def load(name: str):
    path = PACKAGE / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    sys.path.insert(0, str(PACKAGE))
    spec.loader.exec_module(module)
    return module


DEPLOY = load("deploy")


class RedBlockDeployTests(unittest.TestCase):
    def test_explicit_ipv4_overrides_hostname(self):
        destination, opts = DEPLOY.ssh_connection_args("100.119.44.65")
        self.assertEqual(destination, "ugrp1")
        self.assertIn("HostName=100.119.44.65", opts)
        self.assertIn("HostKeyAlias=ugrp1.local", opts)
        self.assertIn("User=ugrp1", opts)

    def test_prefers_tailscale_over_mdns(self):
        with mock.patch.object(DEPLOY, "tailscale_ipv4", return_value="100.119.44.65"), mock.patch.object(
            DEPLOY, "host_resolves", return_value=True
        ):
            destination, opts = DEPLOY.ssh_connection_args("ugrp1")
        self.assertEqual(destination, "ugrp1")
        self.assertIn("HostName=100.119.44.65", opts)
        self.assertIn("HostKeyAlias=ugrp1.local", opts)

    def test_mdns_alias_is_used_when_tailscale_is_missing(self):
        with mock.patch.object(DEPLOY, "tailscale_ipv4", return_value=None), mock.patch.object(
            DEPLOY, "host_resolves", return_value=True
        ):
            destination, opts = DEPLOY.ssh_connection_args("ugrp1")
        self.assertEqual(destination, "ugrp1")
        self.assertNotIn("HostName=100.119.44.65", " ".join(opts))

    def test_raises_when_neither_mdns_nor_tailscale_works(self):
        with mock.patch.object(DEPLOY, "host_resolves", return_value=False), mock.patch.object(
            DEPLOY, "tailscale_ipv4", return_value=None
        ):
            with self.assertRaises(RuntimeError):
                DEPLOY.ssh_connection_args("ugrp1")


if __name__ == "__main__":
    unittest.main(verbosity=2)
