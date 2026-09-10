import pytest

from harness.camera_local_servo import LocalVisualServo, goal_features


def obs(target=(0.60, 0.50), jaws=((0.45, 0.48), (0.55, 0.52)), **extra):
    value = {
        "view": "own",
        "jaws": [list(jaws[0]), list(jaws[1])] if jaws is not None else None,
        "target": list(target) if target is not None else None,
        "confidence": 0.95,
        "identity_confidence": 0.95,
    }
    value.update(extra)
    return value


def shifted(value, error_dx):
    result = dict(value)
    result["target"] = [value["target"][0] + error_dx, value["target"][1]]
    return result


def test_goal_features_rejects_missing_and_untrusted_landmarks():
    assert goal_features(obs()) == pytest.approx((0.10, 0.0))
    assert goal_features(obs(target=None)) is None
    assert goal_features(obs(jaws=None)) is None
    assert goal_features(obs(confidence=0.69)) is None
    assert goal_features(obs(view="overhead", identity_confidence=0.79)) is None


def test_proposal_uses_only_nearby_visual_state_samples():
    servo = LocalVisualServo()
    near = obs(target=(0.60, 0.50))
    far = obs(target=(0.90, 0.15), jaws=((0.05, 0.80), (0.15, 0.80)))
    for _ in range(2):
        servo.observe(near, {"kind": "arm", "servo_id": 3, "pulse": 1550, "_delta": 50}, shifted(near, -0.04))
        servo.observe(far, {"kind": "arm", "servo_id": 3, "pulse": 1550, "_delta": 50}, shifted(far, 0.04))
    assert servo.propose(near, {3: 1500}) == {"kind": "arm", "servo_id": 3, "pulse": 1550}
    assert servo.propose(far, {3: 1500}) == {"kind": "arm", "servo_id": 3, "pulse": 1450}


def test_peer_confounded_and_inconsistent_samples_are_not_used():
    servo = LocalVisualServo()
    before = obs()
    for _ in range(3):
        servo.observe(before, {"kind": "arm", "servo_id": 4, "pulse": 1550, "_delta": 50}, shifted(before, -0.04), isolated=False)
    assert servo.summary()["sample_count"] == 0
    servo.observe(before, {"kind": "arm", "servo_id": 4, "pulse": 1550, "_delta": 50}, shifted(before, -0.04))
    servo.observe(before, {"kind": "arm", "servo_id": 4, "pulse": 1550, "_delta": 50}, shifted(before, 0.04))
    assert servo.propose(before, {4: 1500}) is None


def test_cross_view_transition_is_rejected():
    servo = LocalVisualServo()
    own = obs()
    overhead = obs(view="overhead")
    servo.observe(
        own,
        {"kind": "arm", "servo_id": 3, "pulse": 1550, "_delta": 50},
        shifted(overhead, -0.04),
    )
    assert servo.summary()["sample_count"] == 0


def test_same_coordinates_from_other_view_do_not_train_proposal():
    servo = LocalVisualServo()
    overhead = obs(view="overhead")
    for _ in range(2):
        servo.observe(
            overhead,
            {"kind": "arm", "servo_id": 3, "pulse": 1550, "_delta": 50},
            shifted(overhead, -0.04),
        )
    assert servo.propose(obs(), {3: 1500}) is None
    assert servo.propose(overhead, {3: 1500}) == {
        "kind": "arm", "servo_id": 3, "pulse": 1550
    }


def test_predicted_error_descends_and_pulse_is_bounded():
    servo = LocalVisualServo()
    before = obs(target=(0.70, 0.50))
    for channel in (5, 6):
        action = ({"kind": "look", "pan_pulse": 2500, "_delta": 50} if channel == 6 else
                  {"kind": "arm", "servo_id": channel, "pulse": 2500, "_delta": 50})
        servo.observe(before, action, shifted(before, -0.05))
        servo.observe(before, action, shifted(before, -0.05))
    proposal = servo.propose(before, {5: 2490, 6: 2500})
    assert proposal == {"kind": "arm", "servo_id": 5, "pulse": 2500}
    assert "_delta" not in proposal
    assert servo.summary()["sampled_channels"] == [5, 6]


def test_history_is_bounded_to_128_transitions():
    servo = LocalVisualServo()
    before = obs()
    for _ in range(140):
        servo.observe(before, {"kind": "arm", "servo_id": 3, "pulse": 1550, "_delta": 50}, shifted(before, -0.01))
    assert servo.summary()["sample_count"] == 128
