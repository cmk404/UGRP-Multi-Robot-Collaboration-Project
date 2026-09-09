import json

import pytest

from harness.camera_visual_observer import CameraVisualObserver, parse_observation


def jpeg(payload=b"x"):
    return b"\xff\xd8" + payload + b"\xff\xd9"


def valid(**changes):
    value = {
        "view": "own", "jaws": [[0.2, 0.3], [0.8, 0.3]],
        "target": [0.5, 0.4], "confidence": 0.8,
        "identity_confidence": 0.6, "self_center": [0.4, 0.7],
        "capture_visible": False, "lift_visible": False, "reason": "visible pixels",
        "suggested_action": {"kind": "wait"},
    }
    value.update(changes)
    return value


class Completer:
    def __init__(self, response=None):
        self.response = json.dumps(response or valid())
        self.calls = []

    def complete(self, messages, *, images):
        self.calls.append({"messages": messages, "images": images})
        return self.response


def test_request_has_current_then_same_previous_pair_and_bounded_raw_actions():
    observer = CameraVisualObserver("r1", Completer())
    first = observer.prepare_request(jpeg(b"own1"), jpeg(b"top1"), [])
    assert [x["label"] for x in first["images"]] == ["CURRENT_OWN_VIEW", "CURRENT_OVERHEAD_VIEW"]
    actions = [{"kind": "look", "pan_pulse": 1500 + i} for i in range(10)]
    second = observer.prepare_request(jpeg(b"own2"), jpeg(b"top2"), actions)
    assert [x["label"] for x in second["images"]] == [
        "CURRENT_OWN_VIEW", "CURRENT_OVERHEAD_VIEW", "PREVIOUS_OWN_VIEW", "PREVIOUS_OVERHEAD_VIEW"
    ]
    assert '"pan_pulse":1500' not in second["messages"][1]["content"]
    assert '"pan_pulse":1502' in second["messages"][1]["content"]
    assert "world state" in second["messages"][0]["content"]


@pytest.mark.parametrize("own,top,actions", [(b"bad", jpeg(), []), (jpeg(), b"bad", []), (jpeg(), jpeg(), {} ), (jpeg(), jpeg(), [{"world_state": [1, 2]}])])
def test_input_boundaries(own, top, actions):
    with pytest.raises(ValueError):
        CameraVisualObserver("r1", Completer()).prepare_request(own, top, actions)


@pytest.mark.parametrize("changes", [
    {"jaws": [[-0.1, 0.2], [0.3, 0.4]]}, {"jaws": [[0.1, 0.2]]},
    {"target": [float("nan"), 0.2]}, {"self_center": [0.2, 1.1]},
    {"confidence": True}, {"identity_confidence": float("inf")},
    {"capture_visible": 1}, {"reason": ""}, {"view": "side"},
    {"suggested_action": {"kind": "drive", "forward": -0.06, "turn": 0, "duration_s": 0.2}},
    {"jaws": None, "target": None, "capture_visible": True},
])
def test_rejects_invalid_landmarks_and_schema(changes):
    completer = Completer(valid(**changes))
    with pytest.raises(ValueError):
        CameraVisualObserver("r1", completer).observe(jpeg(), jpeg(), [])


def test_fenced_json_and_null_hidden_landmarks_are_accepted():
    response = valid(jaws=None, target=None, self_center=None)
    completer = Completer()
    completer.response = "```json\n" + json.dumps(response) + "\n```"
    observer = CameraVisualObserver("r3", completer)
    assert observer.observe(jpeg(), jpeg(), [{"kind": "wait"}])["jaws"] is None
    assert observer.last_response == completer.response
    assert observer.last_request == completer.calls[0]


def test_history_does_not_cross_robot_instances():
    r1 = CameraVisualObserver("r1", Completer())
    r3 = CameraVisualObserver("r3", Completer())
    r1.prepare_request(jpeg(b"r1"), jpeg(b"top"), [])
    request = r3.prepare_request(jpeg(b"r3"), jpeg(b"top"), [])
    assert len(request["images"]) == 2
    assert "r3" in request["messages"][0]["content"]


def test_suggested_reverse_drive_and_first_absolute_servo_are_allowed():
    reverse = Completer(valid(suggested_action={"kind": "drive", "forward": -0.05, "turn": 0.1, "duration_s": 0.4}))
    assert CameraVisualObserver("r1", reverse).observe(jpeg(), jpeg(), [])["suggested_action"]["forward"] == -0.05
    arm = Completer(valid(suggested_action={"kind": "arm", "servo_id": 4, "pulse": 2100}))
    assert CameraVisualObserver("r1", arm).observe(jpeg(), jpeg(), [])["suggested_action"]["pulse"] == 2100


def test_suggested_servo_delta_is_limited_against_last_same_channel_command():
    issued = [
        {"kind": "arm", "servo_id": 4, "pulse": 1500},
        {"kind": "arm", "servo_id": 3, "pulse": 2100},
        {"kind": "arm", "servo_id": 4, "pulse": 1600},
    ]
    okay = Completer(valid(suggested_action={"kind": "arm", "servo_id": 4, "pulse": 1700}))
    CameraVisualObserver("r1", okay).observe(jpeg(), jpeg(), issued)
    bad = Completer(valid(suggested_action={"kind": "arm", "servo_id": 4, "pulse": 1701}))
    with pytest.raises(ValueError):
        CameraVisualObserver("r1", bad).observe(jpeg(), jpeg(), issued)


def test_parse_observation_is_public_and_strict():
    assert parse_observation(json.dumps(valid()))["view"] == "own"
    with pytest.raises(ValueError):
        parse_observation(json.dumps({**valid(), "extra": 1}))
