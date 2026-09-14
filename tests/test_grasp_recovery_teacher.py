import json
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from scripts.collect_grasp_recovery_teacher import Records, bounded_expert_action, load_cases


JPEG = b"\xff\xd8recovery-teacher-test\xff\xd9"


class FakeWorld:
    def render_team_jpeg(self, **kwargs):
        return JPEG

    def render_jpeg(self, **kwargs):
        return JPEG


def test_load_cases_validates_per_robot_shape_bounds_and_safe_unique_ids():
    with TemporaryDirectory() as temporary:
        path = Path(temporary) / "cases.json"
        path.write_text(json.dumps({"cases": [
            {"id": "case-01", "perturb": {"r1": [150, -1, 0], "r3": [0, 2, -150]}},
        ]}))
        assert load_cases(path)[0]["perturb"]["r1"] == [150, -1, 0]
        path.write_text(json.dumps({"cases": [
            {"id": "bad/path", "perturb": {"r1": [0, 0, 0], "r3": [0, 0, 0]}},
        ]}))
        with pytest.raises(ValueError):
            load_cases(path)


def test_bounded_action_clips_each_teacher_delta_to_25():
    action = bounded_expert_action({
        "r1": {3: -100, 4: 7, 5: 25}, "r3": {3: 80, 4: -26, 5: 0},
    })
    assert action == {
        "r1": {3: -25, 4: 7, 5: 25}, "r3": {3: 25, 4: -25, 5: 0},
    }


def test_actor_observation_excludes_teacher_labels_and_paths_are_case_unique():
    with TemporaryDirectory() as temporary:
        out = Path(temporary)
        records = Records(out)
        commands = {rid: {3: 1000, 4: 1500, 5: 2000} for rid in ("r1", "r3")}
        remaining = {rid: {3: -30, 4: 0, 5: 8} for rid in ("r1", "r3")}
        bounded = bounded_expert_action(remaining)
        records.observe(
            FakeWorld(), case_id="case-01", observation_id="correction-00-before",
            phase="recovery_before_action", commands=commands,
            full_remaining=remaining, next_action=bounded,
        )
        actor = records.actor[0]
        assert set(actor) == {
            "sample_id", "case_id", "robot_id", "phase", "observations",
            "own_issued_command_snapshot",
        }
        assert actor["observations"]["own_rgb"]["path"].startswith("rgb/case-01/")
        label = records.labels[0]
        assert label["correction_pulses"] == [-30, 0, 8]
        assert label["bounded_next_pulses"] == [-25, 0, 8]
        assert label["sample_id"] == actor["sample_id"]
