"""Exercise native pause/reset recovery with deliberate clock and NaN faults.

Uses the real viewer and CLI key callback; does not claim a mouse click test.
Run with mjpython on macOS or Python under Xvfb on Linux.
"""
import argparse
import json
import time
from pathlib import Path
from unittest.mock import patch

from scripts.sim_cli import main as cli
from sim.session import Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = args.output / "config.json"
    config.write_text(json.dumps({"version": 1, "scene": {"layout": "dispatch/shared_crossing", "seed": 11},
                                  "run": {"sim_seconds": 2, "wall_seconds": 60, "realtime_factor": 2}}))

    for fault in ("external_reset", "physics_instability"):
        class RecoverySimulation(Simulation):
            injected = False
            resume_sent = False
            fault_since = None
            reset_sent = False

            def launch_viewer(self, *, camera, key_callback):
                self.callback = key_callback
                return super().launch_viewer(camera=camera, key_callback=key_callback)

            def step(self, steps=1):
                if not self.injected and self.time >= .5:
                    self.injected = True
                    if fault == "external_reset":
                        self._world.data.time = 0.
                    else:
                        self._world.data.qvel[0] = float("nan")
                return super().step(steps)

            def sync_viewer(self):
                try:
                    super().sync_viewer()
                finally:
                    if self._state_error is not None:
                        self.fault_since = self.fault_since or time.monotonic()
                        if not self.reset_sent and time.monotonic() - self.fault_since >= 1:
                            assert self._viewer.is_running()
                            self.callback(ord("R"))
                            self.reset_sent = True
                    elif self.reset_sent and not self.resume_sent:
                        self.callback(32)
                        self.resume_sent = True

        with patch("sim.session.Simulation", RecoverySimulation):
            assert cli(["run", str(config), "--output", str(args.output / fault)]) == 2
        result = json.loads((args.output / fault / "result.json").read_text())
        assert len(result["runtime_events"]) == 1 and result["runtime_events"][0]["kind"] == fault
        assert result["episodes"] == 2 and result["stop_reason"] == "sim_limit"
        assert result["sim_s"] >= 2 - 1e-8 and result["protocol_complete"] is False
        print(f"Verified {fault}: window stayed open, R reset, Space resumed; failure retained", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
