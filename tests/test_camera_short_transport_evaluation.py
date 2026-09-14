import unittest

from scripts.evaluate_camera_short_transport import evaluate_transport_samples


def contact(held=True):
    return {rid: {"left": held, "right": held, "bilateral": held}
            for rid in ("r1", "r3")}


def sample(t, phase, x, *, y=0.0, z=.10, lift=.04, held=True,
           weld=False, floor=False, robot_contact=False):
    return {"sim_time_s": t, "phase": phase, "position_m": [x, y, z],
            "height_above_start_m": lift, "constraints_active": {"r1": weld, "r3": False},
            "contacts": contact(held), "payload_floor_contact": floor,
            "payload_robot_contact": robot_contact}


def valid_samples(*, final_x=.20, final_y=0.0, release_drift=0.0):
    rows = []
    for i in range(21):
        rows.append(sample(i * .1, "grasp_hold", 0.0))
    rows += [sample(2.1, "carry", .00), sample(2.2, "carry", .05),
             sample(2.3, "carry", .10), sample(2.4, "carry", .15),
             sample(2.5, "carry_stop", .20)]
    rows += [sample(2.6, "place_lower", final_x, y=final_y),
             sample(2.7, "place_open", final_x, y=final_y, lift=0, held=False, floor=True),
             sample(2.8, "place_retract", final_x, y=final_y, lift=0, held=False, floor=True)]
    for i in range(11):
        rows.append(sample(2.9 + i * .1, "release_hold",
                           final_x + release_drift * i / 10, y=final_y,
                           lift=0, held=False, floor=True))
    return rows


class CameraShortTransportEvaluationTests(unittest.TestCase):
    def test_valid_exact_duration_and_boundaries(self):
        result = evaluate_transport_samples(valid_samples(final_x=.23, final_y=.03))
        self.assertTrue(result["success"], result)
        self.assertAlmostEqual(result["metrics"]["grasp_hold_duration_s"], 2.0)
        self.assertAlmostEqual(result["metrics"]["release_hold_duration_s"], 1.0)
        self.assertAlmostEqual(result["metrics"]["release_position_span_3d_m"], 0.0)

    def test_required_phase_and_contact_fail_closed(self):
        rows = [row for row in valid_samples() if row["phase"] != "place_retract"]
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["required_phases"])
        rows = valid_samples(); rows[22]["contacts"]["r1"]["bilateral"] = False
        result = evaluate_transport_samples(rows)
        self.assertFalse(result["gates"]["carry_continuity"])
        self.assertFalse(result["success"])

    def test_missing_floor_and_weld_fail(self):
        rows = valid_samples(); rows[-1].pop("payload_floor_contact")
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["release_grounded_clear"])
        rows = valid_samples(); rows[5]["constraints_active"]["r1"] = True
        result = evaluate_transport_samples(rows)
        self.assertFalse(result["gates"]["weld_off"])
        self.assertFalse(result["success"])

    def test_teleport_without_carry_progress_fails(self):
        rows = valid_samples()
        for row in rows:
            if row["phase"] in {"carry", "carry_stop"}:
                row["position_m"][0] = .20
        result = evaluate_transport_samples(rows)
        self.assertFalse(result["gates"]["carry_progress"])
        self.assertTrue(result["gates"]["transport_endpoint"])

    def test_carry_loss_fails_even_if_recovered(self):
        rows = valid_samples(); rows[23]["height_above_start_m"] = .029
        rows[23]["contacts"]["r3"]["bilateral"] = False
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["carry_continuity"])

    def test_sample_gap_fails_continuity(self):
        rows = valid_samples()
        for row in rows[23:]:
            row["sim_time_s"] += .2
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["carry_continuity"])

    def test_interleaved_phase_is_not_a_contiguous_block(self):
        rows = valid_samples()
        rows.insert(23, sample(2.25, "place_lower", .075))
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["carry_continuity"])

    def test_release_stable_vs_drifting_and_robot_contact(self):
        self.assertTrue(evaluate_transport_samples(valid_samples(release_drift=.005))["gates"]["release_stable"])
        self.assertFalse(evaluate_transport_samples(valid_samples(release_drift=.005001))["gates"]["release_stable"])
        rows = valid_samples(); rows[-2]["payload_robot_contact"] = True
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["release_grounded_clear"])

    def test_missing_and_nonfinite_data_fail_closed(self):
        rows = valid_samples(); rows[0]["position_m"][0] = float("nan")
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["valid_samples"])
        rows = valid_samples(); del rows[-1]["contacts"]["r1"]["left"]
        self.assertFalse(evaluate_transport_samples(rows)["gates"]["release_grounded_clear"])


if __name__ == "__main__":
    unittest.main()
