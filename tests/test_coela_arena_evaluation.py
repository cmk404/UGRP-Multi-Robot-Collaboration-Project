import unittest
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import patch

from scripts.evaluate_coela_arena import (
    layout_record,
    score_arena_result,
    static_geometry_snapshot,
)


@dataclass(frozen=True)
class Terrain:
    terrain_id: str
    kind: str
    center_xy: tuple[float, float]
    half_extents_xy: tuple[float, float]
    height_m: float
    traversable: bool = False


@dataclass(frozen=True)
class Arena:
    source_zone: str
    destination_zone: str
    zones: dict
    terrain: tuple[Terrain, ...]
    robot_start_poses: dict


class World:
    warehouse_arena = Arena(
        "S", "D", {"S": {"center": (-1, 0)}, "D": {"center": (1, 0)}},
        (Terrain("wall-1", "barrier", (0, 0), (.2, .8), .25),),
        {"r1": (-1.4, 0, 0)},
    )
    warehouse_zone_positions = {"cargo": {"S": (-1, 0, .02), "D": (1, 0, .02)}}

    def __init__(self):
        self.model = SimpleNamespace(
            geom_pos=[[0.0, 0.0, 0.125]],
            geom_size=[[0.2, 0.8, 0.125]],
            geom_quat=[[1.0, 0.0, 0.0, 0.0]],
            geom_type=[6], geom_bodyid=[0], geom_contype=[1], geom_conaffinity=[3],
        )
        self.data = SimpleNamespace(geom_xpos=[[0.0, 0.0, 0.125]])


class ArenaEvaluationTests(unittest.TestCase):
    def test_layout_record_contains_seeded_fixture_and_positions(self):
        record = layout_record(World())
        self.assertEqual(record["arena"]["source_zone"], "S")
        self.assertEqual(record["arena"]["destination_zone"], "D")
        self.assertEqual(record["warehouse_zone_positions"]["cargo"]["D"], [1, 0, .02])

    def test_static_geometry_snapshot_has_positions_and_dimensions(self):
        with patch("scripts.evaluate_coela_arena.mujoco.mj_name2id", return_value=0):
            snapshot = static_geometry_snapshot(World())
        self.assertEqual(snapshot[0]["terrain_id"], "wall-1")
        self.assertEqual(snapshot[0]["world_position"], [0.0, 0.0, 0.125])
        self.assertEqual(snapshot[0]["size"], [0.2, 0.8, 0.125])
        self.assertEqual(snapshot[0]["contype"], 1)

    def test_missing_scene_geom_is_an_evaluation_error(self):
        with patch("scripts.evaluate_coela_arena.mujoco.mj_name2id", return_value=-1):
            with self.assertRaisesRegex(RuntimeError, "ARENA_STATIC_GEOM_MISSING:wall-1"):
                static_geometry_snapshot(World())

    def test_success_requires_delivery_and_unchanged_static_geometry(self):
        with patch("scripts.evaluate_coela_arena.mujoco.mj_name2id", return_value=0):
            before = static_geometry_snapshot(World())
        successful = score_arena_result(
            {"success": True, "reason": "SUCCESS", "status": {"history": []}},
            before, list(before),
        )
        self.assertTrue(successful["overall_task_success"])
        self.assertTrue(successful["static_geometry_present"])
        self.assertTrue(successful["static_geometry_unchanged"])

        moved = [dict(before[0], world_position=[0.1, 0.0, 0.125])]
        invalid = score_arena_result(
            {"success": True, "reason": "SUCCESS", "status": {"history": []}},
            before, moved,
        )
        self.assertFalse(invalid["overall_task_success"])
        self.assertEqual(invalid["reason"], "STATIC_GEOMETRY_CHANGED")

    def test_physical_failure_remains_primary_evidence(self):
        failure = {"event": "failed", "reason": "JOINT_CONTACT_LOST"}
        cleanup = {"event": "failed", "reason": "MIXED_EXECUTION_CANCELLED"}
        scored = score_arena_result(
            {"success": False, "reason": "INCOMPLETE", "status": {"history": [failure, cleanup]}},
            [], [],
        )
        self.assertEqual(scored["first_execution_failure"], failure)
        self.assertEqual(scored["primary_failure"], "JOINT_CONTACT_LOST")
        self.assertEqual(scored["cleanup_failures"], [cleanup])


if __name__ == "__main__":
    unittest.main()
