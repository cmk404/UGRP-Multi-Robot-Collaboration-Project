import io
import json
from types import SimpleNamespace

import pytest

from sim.session import Simulation
from sim.session_console import Console, manual_command, parse_reply, request_for
from tests.test_simulation_session import World


class Process:
    def __init__(self, done=False):
        self.done, self.terminated = done, False

    def poll(self):
        return 0 if self.done else None

    def terminate(self):
        self.terminated = self.done = True

    def wait(self, **_):
        return 0


def arguments(**overrides):
    return SimpleNamespace(**dict(dict(mode="manual", robot="r1", model="fixture", paused=False,
        max_calls=9, max_rounds=2, model_timeout=1, task=None, exit_after_task=True), **overrides))


def reply(*, done=False, message=None, action=None):
    return json.dumps(dict(action=action or {"kind": "wait"}, say="확인", message=message, done=done))


@pytest.mark.parametrize("text", ["r1 앞으로 9초", "drive nan 0 .5", "arm 2 1500", "알아서 운반해", "r4 앞으로"])
def test_manual_rejects_unbounded_or_ambiguous_actions(text):
    with pytest.raises(ValueError):
        manual_command(text)


def test_manual_korean_and_raw_reply_contract():
    rid, action = manual_command("r2 오른쪽 0.3초")
    assert rid == "r2" and action == {"kind": "drive", "forward": 0, "turn": -.15, "duration_s": .3}
    for raw in (None, '[]', reply(message="peer"), reply(done=True, action={"kind": "look", "pan_pulse": 1500})):
        with pytest.raises(ValueError):
            parse_reply(raw, "llm-independent", {})
    assert parse_reply(reply(message="peer"), "llm-peer", {})["message"] == "peer"


def test_actor_request_uses_only_owned_rgb_and_commands():
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        for robot in ("r1", "r2"):
            sim.apply(robot, {"kind": "wait"})
        observe = sim.observe
        sim.observe = lambda robot: dict(observe(robot), contacts="forbidden", poses="forbidden", success=True)
        request = request_for(sim, "r1", "task", "llm-independent", [], [{"from": "r2", "content": "private"}], model="fixture", timeout_s=1)
        data = json.loads(request["messages"][1]["content"])
        assert data["inbox"] == []
        assert "forbidden" not in request["messages"][1]["content"]
        assert {row["robot"] for row in data["observation"]["command_history"]} == {"r1"}
        assert len(request["images"]) == 2
        assert [image["label"] for image in request["images"]] == ["OWN_RGB", "SHARED_TOP_RGB"]


def test_stop_revokes_motion_and_switch_resets_episode(tmp_path):
    config = {"version": 1, "actions": [{"at_s": 0, "robot": "r2", "command": {"kind": "drive", "forward": .1, "turn": 0, "duration_s": 1}}]}
    with Simulation(config, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments())
        console.command("r1 앞으로 0.5초")
        sim.step(2)
        assert sim._world.robot("r1").motors != [0.] * 4
        assert sim._world.robot("r2").motors == [0.] * 4  # script was disabled
        console.command("정지")
        assert sim._world.robot("r1").motors == [0.] * 4 and console.paused
        console.command("/mode script")
        assert sim.episode == 1 and console.paused
        console.command("/run")
        sim.step()
        assert sim._world.robot("r2").motors != [0.] * 4
        console.close()


def test_mode_switch_cancels_pending_model_and_discards_stale_reply(tmp_path):
    process = Process()
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments(mode="llm-single"), spawn=lambda *a, **k: process)
        console.command("앞으로 조금 이동해")
        console.poll()
        assert console.waiting
        console.command("/mode manual")
        assert process.terminated and not console.waiting and console.goal is None
        console.poll()
        assert not [row for row in sim.command_history if row["event"] == "command"]
        console.close()


@pytest.mark.parametrize("mode", ["llm-independent", "llm-peer"])
def test_private_decisions_and_peer_mail_delivery_between_rounds(tmp_path, mode):
    requests = []
    def spawn(argv, **_):
        from pathlib import Path
        path = Path(argv[-1])
        requests.append(json.loads(path.read_text()))
        path.with_suffix(".response.json").write_text(json.dumps({"raw": reply(message="hello" if mode == "llm-peer" else None), "wall_s": .1}))
        return Process(done=True)
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments(mode=mode), spawn=spawn)
        console.command("협력해")
        console.poll(); console.poll()
        assert len(sim.command_history) == 4  # reset plus three actions
        sim.step(10)
        console.poll()
        for index, request in enumerate(requests[3:]):
            content = json.loads(request["messages"][1]["content"])
            rid = content["robot_id"]
            assert len(content["own_decisions"]) == 1
            assert all(row["robot"] == rid for row in content["observation"]["command_history"])
            assert len(content["inbox"]) == (2 if mode == "llm-peer" else 0)
            assert all(row["from"] != rid for row in content["inbox"])
        assert len(requests) == 6
        console.close()


def test_budget_is_preflighted_for_whole_round(tmp_path):
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments(mode="llm-peer", max_calls=2), spawn=lambda *a, **k: pytest.fail("no partial round"))
        console.command("협력해"); console.poll()
        assert console.calls == 0 and console.quit
        console.close()


def test_poll_is_nonblocking_and_timeout_stops_owned_request(tmp_path, monkeypatch):
    process = Process()
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments(mode="llm-single"), spawn=lambda *a, **k: process)
        console.command("둘러봐"); console.poll(); console.poll()
        assert console.waiting and sim.time == 0
        started = console.jobs["r1"]["started"]
        monkeypatch.setattr("sim.session_console.time.monotonic", lambda: started + 7)
        console.poll()
        assert process.terminated and console.quit and console.failures == 1
        console.close()


def test_spawn_failure_cancels_already_started_peers(tmp_path):
    process = Process()
    def spawn(*args, **kwargs):
        if console.calls == 2:
            raise OSError("fixture")
        return process
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments(mode="llm-peer"), spawn=spawn)
        console.command("둘러봐"); console.poll()
        assert process.terminated and console.failures == 1 and not console.waiting
        console.close()


def test_invalid_model_output_does_not_move_any_robot(tmp_path):
    def spawn(argv, **_):
        from pathlib import Path
        path = Path(argv[-1])
        raw = None if "r2" in path.name else reply(action={"kind": "drive", "forward": .1, "turn": 0, "duration_s": .5})
        path.with_suffix(".response.json").write_text(json.dumps({"raw": raw, "wall_s": .1}))
        return Process(done=True)
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments(mode="llm-peer"), spawn=spawn)
        console.command("이동해"); console.poll(); console.poll()
        assert console.failures == 1 and console.quit
        assert all(sim._world.robot(r).motors == [0.] * 4 for r in ("r1", "r2", "r3"))
        console.close()


def test_eof_quits_and_invalid_input_is_recorded(tmp_path):
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments(), stream=io.StringIO("/unknown\n"))
        console._read()
        console.poll()
        assert console.failures == 1
        console.quit = False
        console.poll()
        assert console.quit
        console.close()


def test_model_worker_saves_exact_wire_request_and_response(tmp_path, monkeypatch):
    from scripts.sim_console_model import complete
    calls = []
    def open_fixture(request, **kwargs):
        calls.append(json.loads(request.data))
        return io.BytesIO(json.dumps({"choices": [{"message": {"content": reply(done=True)}}], "usage": {"prompt_tokens": 15, "completion_tokens": 10}, "model": "fixture"}).encode())
    monkeypatch.setattr("scripts.sim_console_model.urlopen", open_fixture)
    path = tmp_path / "request.json"
    path.write_text(json.dumps({"model": "fixture", "max_tokens": 512, "timeout_s": 1,
        "messages": [{"role": "user", "content": "hello"}],
        "images": [{"label": "OWN_RGB", "image": "data:image/jpeg;base64,YQ=="}]}))
    assert complete(path) == 0
    assert json.loads(path.with_suffix(".wire.json").read_text()) == calls[0]
    response = json.loads(path.with_suffix(".response.json").read_text())
    assert response["usage"]["prompt_tokens"] == 15
    assert response["raw"] == reply(done=True) and response["error"] is None


def test_script_cli_accepts_controller_override_before_empty_check(tmp_path, monkeypatch):
    monkeypatch.setattr('sim.workflow_manager.RECORDS', tmp_path / 'fixture-records')
    from scripts.sim_cli import main
    target = tmp_path / "scene.json"
    target.write_text('{"version":1}')
    (tmp_path / "policy.py").write_text("def create(*, robot_id, seed, params):\n    return None\n")
    seen = []
    monkeypatch.setattr("scripts.sim_cli.run", lambda config, args: seen.append(config) or 0)
    assert main(["console", str(target), "--mode", "script", "--controller", "r1=policy.py:create"]) == 0
    assert seen[0]["controllers"]["r1"]["factory"] == "policy.py:create"


def test_partial_model_receipts_do_not_claim_complete_usage_or_latency():
    from scripts.sim_cli import model_totals
    responses = [{"wall_s": 1, "usage": {"prompt_tokens": 10, "completion_tokens": 5}}]
    assert model_totals(responses, complete=False) == {"model_latency_s": None}
    assert model_totals(responses, complete=True) == {"model_latency_s": 1, "input_tokens": 10, "output_tokens": 5}
    assert model_totals([], complete=True) == {"model_latency_s": 0}


def test_pause_run_preserves_pending_motion_but_stop_revokes_it(tmp_path):
    with Simulation({"version": 1}, render=True, world_factory=World) as sim:
        console = Console(sim, tmp_path, arguments())
        console.command("r1 앞으로 1초")
        sim.step(10)
        console.command("/pause")
        console.command("/run")
        sim.step(10)
        assert sim._world.robot("r1").motors == [.1] * 4
        console.command("/stop")
        assert sim._world.robot("r1").motors == [0.] * 4
        console.close()
