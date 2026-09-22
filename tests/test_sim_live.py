import json
import threading
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from scripts.sim_live import LiveState, Playground, make_server, validate_command


class Robot:
    servo_command_pulses = {1: 1500, 3: 1500, 4: 1500, 5: 1500, 6: 1500}

    def __init__(self, world):
        self.world = world
        self.motors = [0.] * 4

    def set_motor_commands(self, values):
        self.motors = values

    def advance_to_sim_time(self, target):
        self.world.data.time = target


class World:
    def __init__(self, **kwargs):
        self.data = SimpleNamespace(time=0.)
        self.robots = {rid: Robot(self) for rid in ("r1", "r2", "r3")}
        self.closed = False

    def robot(self, rid):
        return self.robots[rid]

    def render_jpeg(self, **kwargs):
        return b"jpeg"

    render_team_jpeg = render_jpeg

    def close(self):
        self.closed = True


@pytest.fixture
def playground(tmp_path):
    state = LiveState(41)
    app = Playground(state, tmp_path, world_factory=World)
    app.reset(41)
    return app


def test_drive_expires_without_another_browser_request(playground):
    app = playground
    app.apply({"action": "drive", "robot": "r2", "direction": "forward", "id": "move"})
    assert app.world.robot("r2").motors == [.12] * 4
    assert app.world.robot("r1").motors == [0.] * 4
    app.advance(.55)
    assert app.world.robot("r2").motors == [0.] * 4


def test_pause_and_reset_cancel_commands(playground):
    app = playground
    app.apply({"action": "demo", "id": "demo"})
    app.apply({"action": "pause", "id": "pause"})
    assert app.paused and app.world.robot("r1").motors == [0.] * 4
    old = app.world
    app.apply({"action": "reset", "seed": 42, "id": "reset"})
    assert old.closed and app.world is not old
    assert app.seed == 42 and app.episode == 2 and app.paused
    assert app.world.robot("r1").motors == [0.] * 4


def test_finite_run_closes_world_and_keeps_evidence(tmp_path):
    state = LiveState(41)
    app = Playground(state, tmp_path, duration=.03, world_factory=World)
    assert app.run() == 0
    assert app.world.closed and state.stop.is_set()
    result = json.loads((tmp_path / "result.json").read_text())
    assert result["exit_reason"] == "duration_limit"
    assert "success" not in result and result["model_calls"] == 0
    assert (tmp_path / "overview.jpg").read_bytes() == b"jpeg"


def test_renderer_failure_preserves_error_and_closes_world(tmp_path):
    class BrokenWorld(World):
        def render_jpeg(self, **kwargs):
            raise RuntimeError("renderer failed")
    app = Playground(LiveState(41), tmp_path, world_factory=BrokenWorld)
    assert app.run() == 1
    assert app.world.closed
    assert "renderer failed" in json.loads((tmp_path / "result.json").read_text())["error"]


@pytest.mark.parametrize("command", [[], {"action": "reset", "seed": True},
    {"action": "reset", "seed": -1}, {"action": "drive", "robot": "r4", "direction": "forward"},
    {"action": "drive", "robot": "r1", "direction": []},
    {"action": "play", "extra": 1}, {"action": "execute", "code": "arbitrary"}])
def test_invalid_commands(command):
    with pytest.raises(ValueError):
        validate_command(command)


@pytest.fixture
def http_server():
    state = LiveState(41)
    server = make_server(state, 0)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    yield state, f"http://127.0.0.1:{server.server_port}"
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def post(url, token, command, **headers):
    request = Request(url + "/api/control", data=json.dumps(command).encode(),
        headers={"Content-Type": "application/json", "X-UGRP-Control": token, **headers})
    return urlopen(request, timeout=2)


def test_http_camera_readiness_and_control_is_queued(http_server):
    state, url = http_server
    with pytest.raises(HTTPError) as exc:
        urlopen(url + "/frame/r1")
    assert exc.value.code == 503
    with pytest.raises(HTTPError) as exc:
        post(url, state.token, {"action": "demo"})
    assert exc.value.code == 409
    state.update(phase="paused")
    state.publish({"r1": b"jpeg"})
    assert urlopen(url + "/frame/r1").read() == b"jpeg"
    response = post(url, state.token, {"action": "demo"})
    assert response.status == 202
    assert state.commands.get_nowait()["id"] == json.load(response)["accepted"]
    assert state.status["commands"] == 0  # accepted does not mean executed


def test_http_rejects_cross_origin_and_rebinding_and_bounds_queue(http_server):
    state, url = http_server
    state.update(phase="paused")
    for headers, token in [({}, "wrong"), ({"Origin": "https://example.com"}, state.token),
                            ({"Host": "attacker.example"}, state.token)]:
        with pytest.raises(HTTPError) as exc:
            post(url, token, {"action": "demo"}, **headers)
        assert exc.value.code == 403
    assert state.commands.empty()
    for _ in range(16):
        post(url, state.token, {"action": "demo"}).close()
    with pytest.raises(HTTPError) as exc:
        post(url, state.token, {"action": "demo"})
    assert exc.value.code == 429
    # Shutdown remains available even if other controls have filled the queue.
    assert post(url, state.token, {"action": "shutdown"}).status == 202
    assert state.stop.is_set()
