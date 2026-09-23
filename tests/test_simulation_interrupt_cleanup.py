"""A second terminal stop must not interrupt native run artifact finalization."""
import json
import signal

import pytest


pytest.importorskip("mujoco")

from scripts import sim_cli
from sim.session import Simulation


def test_repeated_stop_during_cleanup_keeps_result_and_exit_code(tmp_path, monkeypatch):
    class Viewer:
        def __init__(self, callback):
            self.callback = callback
            self.polls = 0

        def is_running(self):
            self.polls += 1
            if self.polls == 1:
                self.callback(32)  # Resume the initially paused preview.
            return True

        def set_texts(self, _texts):
            pass

        def close(self):
            pass

        def _sim(self):
            return None

    class InterruptedSimulation(Simulation):
        def launch_viewer(self, *, camera, key_callback):
            self._viewer = Viewer(key_callback)
            return self._viewer

        def sync_viewer(self):
            pass

        def step(self, steps=1):
            raise KeyboardInterrupt  # The first Ctrl-C during active work.

    real_signal = signal.signal
    original_term = signal.getsignal(signal.SIGTERM)
    repeated_stop = []

    def signal_with_repeated_stop(sig, handler):
        if sig == signal.SIGTERM and handler is original_term and not repeated_stop:
            repeated_stop.append(True)
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        return real_signal(sig, handler)

    monkeypatch.setattr("sim.session.Simulation", InterruptedSimulation)
    monkeypatch.setattr(sim_cli.signal, "signal", signal_with_repeated_stop)
    monkeypatch.setenv("UGRP_SIM_MANAGED_CHILD", "1")
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"version": 1, "scene": {"layout": "dispatch/shared_crossing", "seed": 11}}))
    output = tmp_path / "preview"

    assert sim_cli.main(["run", str(config), "--output", str(output), "--paused"]) == 130
    assert repeated_stop
    result = json.loads((output / "result.json").read_text())
    assert result["stop_reason"] == "interrupted"
    assert result["protocol_complete"] is False
    assert "commands.jsonl" in result["artifacts_sha256"]
