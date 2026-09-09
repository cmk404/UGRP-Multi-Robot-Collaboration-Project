import numpy as np

from harness.camera_pixel_jacobian import LocalPixelJacobian


def alignment(x, y=0, angle=0, width=10, endpoint=(.5, .7)):
    return {"offset_px": [x, y], "axis_error_rad": angle, "width_px": width,
            "endpoint": list(endpoint)}


def add(j, primitive, before, delta, effect, pulses=None):
    pulses = dict(pulses or {3: 1500, 4: 1500, 5: 1500, 6: 1500})
    after_pulses = dict(pulses)
    if primitive in {"wrist", "elbow", "shoulder", "look"}:
        channel = {"wrist": 3, "elbow": 4, "shoulder": 5, "look": 6}[primitive]
        after_pulses[channel] += delta
        action = ({"kind": "look", "pan_pulse": after_pulses[channel]} if channel == 6 else
                  {"kind": "arm", "servo_id": channel, "pulse": after_pulses[channel]})
        amount = delta / 50
    elif primitive == "forward":
        action = {"kind": "drive", "forward": delta, "turn": 0.0, "duration_s": 1.0}
        amount = delta / .05
    else:
        action = {"kind": "drive", "forward": 0.0, "turn": delta, "duration_s": 1.0}
        amount = delta / .1
    end = np.asarray(before, dtype=float) + np.asarray(effect) * amount
    return j.add_sample(alignment(*before), alignment(*end), action, pulses, after_pulses,
                        fresh=True, same_endpoint=True, primitive=primitive)


def test_rejects_nonfresh_mismatched_and_nonprimitive_samples():
    j = LocalPixelJacobian()
    p = {3: 1500, 4: 1500, 5: 1500, 6: 1500}
    action = {"kind": "arm", "servo_id": 3, "pulse": 1550}
    q = {**p, 3: 1550}
    assert not j.add_sample(alignment(1), alignment(2), action, p, q,
                            fresh=False, same_endpoint=True, primitive="wrist")
    assert not j.add_sample(alignment(1), alignment(2), action, p, q,
                            fresh=True, same_endpoint=False, primitive="wrist")
    assert not j.add_sample(alignment(1), alignment(2), action, p, q,
                            fresh=True, same_endpoint=True, primitive="bogus")
    assert j.propose(alignment(1, width=float("nan")), p) is None
    assert len(j.samples) == 0


def test_drive_probe_rejects_a_simultaneous_pose_change():
    j = LocalPixelJacobian()
    p = {3: 1500, 4: 1500, 5: 1500, 6: 1500}
    q = {**p, 4: 1550}
    action = {"kind": "drive", "forward": .05, "turn": 0.0, "duration_s": .4}
    assert not j.add_sample(alignment(1), alignment(2), action, p, q,
                            fresh=True, same_endpoint=True, primitive="forward")


def test_history_is_hard_bounded_at_128_fresh_samples():
    j = LocalPixelJacobian()
    for i in range(129):
        assert add(j, "wrist", (i / 10, 0, 0), 50, (1, 0, 0))
    assert len(j.samples) == 128
    assert j.samples[0]["before_error"][0] == .1
    assert j.diagnostics()["sample_count"] == 128


def test_two_consistent_nearby_samples_are_required_per_channel():
    j = LocalPixelJacobian()
    assert add(j, "wrist", (20, 0, 0), 50, (2, 0, 0))
    assert j.propose(alignment(20), {3: 1500, 4: 1500, 5: 1500, 6: 1500}) is None
    assert add(j, "wrist", (21, 0, 0), -50, (2.1, 0, 0))
    proposed = j.propose(alignment(20), {3: 1500, 4: 1500, 5: 1500, 6: 1500})
    assert proposed and proposed[0]["servo_id"] == 3 and proposed[0]["pulse"] < 1500


def test_opposite_or_remote_samples_do_not_supply_a_column():
    j = LocalPixelJacobian(state_radius=30)
    add(j, "elbow", (10, 0, 0), 50, (1, 0, 0))
    add(j, "elbow", (11, 0, 0), -50, (-1, 0, 0))
    assert j.propose(alignment(10), {3: 1500, 4: 1500, 5: 1500, 6: 1500}) is None


def test_tied_two_by_two_contradictory_groups_are_rejected():
    j = LocalPixelJacobian()
    for x, effect in ((10, (1, 0, 0)), (11, (1.1, 0, 0)),
                      (12, (-1, 0, 0)), (13, (-1.1, 0, 0))):
        add(j, "elbow", (x, 0, 0), 50, effect)
    assert j.propose(alignment(10), {3: 1500, 4: 1500, 5: 1500, 6: 1500}) is None


def test_same_error_from_stale_target_endpoint_is_not_local_support():
    j = LocalPixelJacobian()
    p = {3: 1500, 4: 1500, 5: 1500, 6: 1500}
    q = {**p, 4: 1550}
    old = (.2, .3)
    for x in (10, 11):
        assert j.add_sample(alignment(x, endpoint=old), alignment(x + 1, endpoint=old),
                            {"kind": "arm", "servo_id": 4, "pulse": 1550}, p, q,
                            fresh=True, same_endpoint=True, primitive="elbow")
    assert j.propose(alignment(10, endpoint=(.8, .7)), p) is None


def test_incompatible_beam_width_and_axis_neighborhood_are_not_reused():
    j = LocalPixelJacobian()
    p = {3: 1500, 4: 1500, 5: 1500, 6: 1500}
    q = {**p, 4: 1550}
    action = {"kind": "arm", "servo_id": 4, "pulse": 1550}
    for angle in (.5, .51):
        assert j.add_sample(alignment(10, angle=angle, width=10),
                            alignment(11, angle=angle, width=10), action, p, q,
                            fresh=True, same_endpoint=True, primitive="elbow")
    assert j.propose(alignment(10, angle=0, width=10), p) is None
    assert j.propose(alignment(10, angle=.5, width=20), p) is None
    j = LocalPixelJacobian(state_radius=5)
    add(j, "elbow", (100, 0, 0), 50, (1, 0, 0))
    add(j, "elbow", (101, 0, 0), -50, (1, 0, 0))
    assert j.propose(alignment(10), {3: 1500, 4: 1500, 5: 1500, 6: 1500}) is None


def test_angular_response_is_rescaled_to_current_compatible_beam_width():
    j = LocalPixelJacobian()
    p = {3: 1500, 4: 1500, 5: 1500, 6: 1500};q={**p,3:1550}
    action={"kind":"arm","servo_id":3,"pulse":1550}
    for x in (5,6):
        assert j.add_sample(
            alignment(x,angle=0,width=10),alignment(x,angle=.1,width=10),
            action,p,q,fresh=True,same_endpoint=True,primitive="wrist",
        )
    current=alignment(5,angle=0,width=12)
    column=j._column("wrist",current,np.asarray((5,0,0),dtype=float),p)
    assert column is not None
    assert abs(column[2]-12.0)<1e-9


def test_residual_outlier_is_rejected_and_plan_is_bounded():
    j = LocalPixelJacobian()
    for before, effect in [((20, 10, 0), (2, 0, 0)), ((21, 9, 0), (2.1, .1, 0)),
                           ((19, 11, 0), (1.9, -.1, 0)), ((20, 10, 0), (5, 4, 0))]:
        add(j, "wrist", before, 50, effect)
    plan = j.propose(alignment(20, 10), {3: 1500, 4: 1500, 5: 1500, 6: 1500})
    assert plan and len(plan) == 1
    assert 1400 <= plan[0]["pulse"] < 1500


def test_dls_selects_at_most_three_unique_bounded_commands():
    j = LocalPixelJacobian()
    effects = {"wrist": (2, 0, 0), "elbow": (0, 2, 0),
               "look": (0, 0, 2), "shoulder": (1, 1, 1)}
    pose = {3: 500, 4: 1500, 5: 1500, 6: 1500}
    for name, effect in effects.items():
        add(j, name, (20, 20, .2), 50, effect, pose)
        add(j, name, (21, 19, .19), 50, effect, pose)
    plan = j.propose(alignment(20, 20, .2), pose)
    assert plan and len(plan) <= 3
    keys = [(a["kind"], a.get("servo_id", 6)) for a in plan]
    assert len(keys) == len(set(keys))
    assert all(500 <= a.get("pulse", a.get("pan_pulse", 1500)) <= 2500 for a in plan)
    diagnostics = j.diagnostics()
    assert diagnostics["sample_count"] == 8
    assert diagnostics["command_count"] == len(plan)
    assert diagnostics["predicted_error"] < diagnostics["observed_error"]


def test_singular_columns_remain_finite_under_damping():
    j = LocalPixelJacobian()
    for name in ("wrist", "elbow"):
        add(j, name, (30, 0, 0), 50, (1, 0, 0))
        add(j, name, (31, 0, 0), -50, (1, 0, 0))
    plan = j.propose(alignment(30), {3: 1500, 4: 1500, 5: 1500, 6: 1500})
    assert plan and 1 <= len(plan) <= 2
    assert all(isinstance(a.get("pulse"), int) for a in plan)


def test_axis_response_wraps_and_uses_before_width_scale():
    j = LocalPixelJacobian()
    p = {3: 1500, 4: 1500, 5: 1500, 6: 1500}
    q = {**p, 3: 1550}
    action = {"kind": "arm", "servo_id": 3, "pulse": 1550}
    before = alignment(5, angle=1.56, width=10)
    after = alignment(5, angle=-1.56, width=30)
    assert j.add_sample(before, after, action, p, q, fresh=True,
                        same_endpoint=True, primitive="wrist")
    # The unoriented-axis change crosses the wrap by about +.022 rad and uses
    # the before width's 100 px/rad scale, rather than a 300 px/rad subtraction.
    assert 2.0 < j.samples[0]["response"][2] < 2.3


def test_returned_arm_and_forward_commands_use_tight_step_bounds():
    p = {3: 1500, 4: 1500, 5: 1500, 6: 1500}
    arm = LocalPixelJacobian()
    add(arm, "wrist", (100, 0, 0), 50, (1, 0, 0), p)
    add(arm, "wrist", (101, 0, 0), 50, (1, 0, 0), p)
    arm_plan = arm.propose(alignment(100), p)
    assert arm_plan and abs(arm_plan[0]["pulse"] - 1500) <= 50

    drive = LocalPixelJacobian()
    add(drive, "forward", (100, 0, 0), .05, (1, 0, 0), p)
    add(drive, "forward", (101, 0, 0), .05, (1, 0, 0), p)
    drive_plan = drive.propose(alignment(100), p)
    assert drive_plan and abs(drive_plan[0]["forward"]) <= .05


def test_coupled_smaller_correction_improves_when_each_full_step_worsens():
    j = LocalPixelJacobian()
    # From error [5,5], either full negative column alone increases the norm:
    # [5,5]-[10,-8]=[-5,13], and symmetrically for the other column.
    for x in (5, 5.5):
        add(j, "wrist", (x, 5, 0), 50, (10, -8, 0))
        add(j, "elbow", (x, 5, 0), 50, (-8, 10, 0))
    plan = j.propose(alignment(5, 5), {3: 1500, 4: 1500, 5: 1500, 6: 1500})
    assert plan and {a["servo_id"] for a in plan} == {3, 4}
    assert all(1450 <= a["pulse"] < 1500 for a in plan)
    diagnostics = j.diagnostics()
    assert diagnostics["predicted_error"] < diagnostics["observed_error"]
