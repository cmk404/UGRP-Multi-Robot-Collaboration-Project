"""Real MuJoCo reset/warning handling; viewer events use a deterministic handle."""
import json
from contextlib import nullcontext

import pytest

pytest.importorskip("mujoco")

from scripts.sim_cli import main
from sim.session import Simulation


@pytest.mark.parametrize("fault", ["reset", "nan_velocity"])
@pytest.mark.parametrize("headless", [False, True])
def test_cli_preserves_failure_and_native_viewer_waits_for_reset(tmp_path, monkeypatch, fault, headless):
    handles = []

    class Viewer:
        def __init__(self, callback):
            self.callback = callback
            self.polls = 0
            self.texts = []
            self.closed = False
        def lock(self):
            return nullcontext()
        def sync(self):
            pass
        def is_running(self):
            self.polls += 1
            if self.polls == 3:
                self.callback(32)  # Cannot resume an invalid state.
            if self.polls == 4:
                self.callback(ord("R"))
            return self.polls < 7
        def set_texts(self, value):
            self.texts.append(value[2])
        def close(self):
            self.closed = True
        def _sim(self):
            return None

    class FaultSimulation(Simulation):
        calls = 0
        def launch_viewer(self, *, camera, key_callback):
            self._viewer = Viewer(key_callback)
            handles.append(self._viewer)
            return self._viewer
        def step(self, steps=1):
            self.calls += 1
            if self.calls == 2:
                if fault == "reset":
                    self._world.data.time = 0.
                else:
                    self._world.data.qvel[0] = float("nan")
            return super().step(steps)

    monkeypatch.setattr("sim.session.Simulation", FaultSimulation)
    monkeypatch.setenv("DISPLAY", ":fixture")
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"version": 1, "scene": {"layout": "dispatch/shared_crossing", "seed": 11}}))
    output = tmp_path / "run"
    args = ["run", str(config), "--output", str(output)] + (["--headless"] if headless else [])
    if headless:
        with pytest.raises(SystemExit) as stopped:
            main(args)
        assert stopped.value.code == 2
    else:
        assert main(args) == 2
        assert handles[0].closed and handles[0].polls == 7
        assert any("unstable" in text for text in handles[0].texts)
        assert handles[0].texts[-1].startswith("Paused")
    result = json.loads((output / "result.json").read_text())
    assert result["protocol_complete"] is False
    assert result["sim_s"] >= 0
    assert result["episodes"] == (1 if headless else 2)
    assert len(result["runtime_events"]) == 1
    event = result["runtime_events"][0]
    assert event["kind"] == ("external_reset" if fault == "reset" else "physics_instability")
    assert json.loads((output / "runtime-events.json").read_text()) == [event]
    assert "must not move backwards" not in result.get("error", "")
