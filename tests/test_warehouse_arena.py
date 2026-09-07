from __future__ import annotations

import math
import unittest

from sim.warehouse_arena import arena_for_seed


def _rects_overlap(ac, ah, bc, bh, clearance=0.0):
    return (
        abs(ac[0] - bc[0]) < ah[0] + bh[0] + clearance
        and abs(ac[1] - bc[1]) < ah[1] + bh[1] + clearance
    )


class WarehouseArenaTests(unittest.TestCase):
    def test_seed_is_deterministic_and_layouts_vary(self):
        for seed in (41, 58, 73):
            self.assertEqual(arena_for_seed(seed), arena_for_seed(seed))
        fixtures = [arena_for_seed(seed) for seed in (41, 58, 73)]
        self.assertGreater(len({(a.source_zone, a.destination_zone) for a in fixtures}), 1)
        signatures = {
            tuple((t.terrain_id, t.half_extents_xy, t.height_m) for t in a.terrain)
            for a in fixtures
        }
        self.assertEqual(len(signatures), 3)
        self.assertEqual(
            [(a.source_zone, a.destination_zone) for a in fixtures],
            [("A", "B"), ("A", "C"), ("B", "C")],
        )

    def test_zones_are_square_separated_and_compact(self):
        arena = arena_for_seed(41)
        self.assertEqual(set(arena.zones), {"A", "B", "C"})
        zones = list(arena.zones.values())
        for zone in zones:
            self.assertEqual(zone.half_extents_xy[0], zone.half_extents_xy[1])
            self.assertAlmostEqual(2 * zone.half_extents_xy[0], 0.82)
        for index, left in enumerate(zones):
            for right in zones[index + 1:]:
                self.assertFalse(_rects_overlap(
                    left.center_xy, left.half_extents_xy,
                    right.center_xy, right.half_extents_xy,
                ))
                self.assertLessEqual(math.dist(left.center_xy, right.center_xy), 4.0)

    def test_static_obstacles_are_outside_zones_and_block_route_centre(self):
        observed_centres = set()
        for seed in range(20):
            arena = arena_for_seed(seed)
            self.assertGreaterEqual(len(arena.terrain), 2)
            self.assertTrue(all(not item.traversable for item in arena.terrain))
            for obstacle in arena.terrain:
                observed_centres.add(obstacle.center_xy)
                for zone in arena.zones.values():
                    self.assertFalse(_rects_overlap(
                        obstacle.center_xy, obstacle.half_extents_xy,
                        zone.center_xy, zone.half_extents_xy,
                    ))
            source = arena.zones[arena.source_zone].center_xy
            destination = arena.zones[arena.destination_zone].center_xy
            midpoint = tuple((a + b) / 2 for a, b in zip(source, destination))
            self.assertTrue(any(
                abs(midpoint[0] - item.center_xy[0]) <= item.half_extents_xy[0] + 0.045
                and abs(midpoint[1] - item.center_xy[1]) <= item.half_extents_xy[1] + 0.035
                for item in arena.terrain
            ))
        # Jitter makes the physical geometry genuinely seeded instead of
        # selecting among only the three exact edge midpoints.
        self.assertGreater(len(observed_centres), 20)

    def test_cargo_goals_and_starts_are_inside_zones_and_collision_free(self):
        for seed in range(20):
            arena = arena_for_seed(seed)
            self.assertEqual([c.required_carriers for c in arena.cargo_specs], [2, 1, 1])
            source = arena.zones[arena.source_zone]
            destination = arena.zones[arena.destination_zone]
            points = []
            for cargo in arena.cargo_specs:
                points.append(cargo.start_xyz[:2])
                for point, zone in ((cargo.start_xyz, source), (cargo.goal_xyz, destination)):
                    self.assertLess(abs(point[0] - zone.center_xy[0]), zone.half_extents_xy[0])
                    self.assertLess(abs(point[1] - zone.center_xy[1]), zone.half_extents_xy[1])
                for obstacle in arena.terrain:
                    self.assertFalse(_rects_overlap(
                        cargo.start_xyz[:2], (cargo.dimensions_m[0] / 2, cargo.dimensions_m[1] / 2),
                        obstacle.center_xy, obstacle.half_extents_xy,
                    ))
            for index, point in enumerate(points):
                for other in points[index + 1:]:
                    self.assertGreater(math.dist(point, other), 0.25)
            for pose in arena.robot_start_poses.values():
                # A 0.165 m chassis plus the plank's 0.215 m long half-axis
                # need 0.38 m in the conservative radial approximation.
                self.assertGreater(min(math.dist(pose[:2], p) for p in points), 0.45)
                # Full-size robots stage outside the compact painted square;
                # the fixture does not scale their physical geometry down.
                self.assertLessEqual(
                    abs(pose[0] - source.center_xy[0]) - source.half_extents_xy[0],
                    0.191,
                )
                self.assertLessEqual(
                    abs(pose[1] - source.center_xy[1]) - source.half_extents_xy[1],
                    0.441,
                )

    def test_completed_solo_docks_leave_distinct_approach_bases(self):
        for seed in (41, 58, 73):
            arena = arena_for_seed(seed)
            source = arena.zones[arena.source_zone].center_xy
            destination = arena.zones[arena.destination_zone].center_xy
            heading = math.atan2(destination[1] - source[1], destination[0] - source[0])
            backward = (-0.16 * math.cos(heading), -0.16 * math.sin(heading))
            solos = [cargo for cargo in arena.cargo_specs if cargo.required_carriers == 1]
            bases = [
                (cargo.goal_xyz[0] + backward[0], cargo.goal_xyz[1] + backward[1])
                for cargo in solos
            ]
            self.assertGreater(math.dist(*bases), 0.40)

    def test_joint_release_parking_clears_every_solo_final_approach(self):
        for seed in range(20):
            arena = arena_for_seed(seed)
            plank = next(c for c in arena.cargo_specs if c.required_carriers == 2)
            long_axis = (
                -math.sin(plank.goal_yaw_rad),
                math.cos(plank.goal_yaw_rad),
            )
            carrier_parking = [
                (
                    plank.goal_xyz[0] + sign * 0.515 * long_axis[0],
                    plank.goal_xyz[1] + sign * 0.515 * long_axis[1],
                )
                for sign in (-1.0, 1.0)
            ]
            solo_bases = []
            for cargo in (c for c in arena.cargo_specs if c.required_carriers == 1):
                dx = cargo.goal_xyz[0] - cargo.start_xyz[0]
                dy = cargo.goal_xyz[1] - cargo.start_xyz[1]
                length = math.hypot(dx, dy)
                solo_bases.append((
                    cargo.goal_xyz[0] - 0.165 * dx / length,
                    cargo.goal_xyz[1] - 0.165 * dy / length,
                ))
            for carrier in carrier_parking:
                for solo in solo_bases:
                    dx = abs(carrier[0] - solo[0])
                    dy = abs(carrier[1] - solo[1])
                    self.assertGreater(math.hypot(dx, dy), 0.40)
                    self.assertTrue(
                        dx >= 0.40 or dy >= 0.40,
                        (seed, carrier, solo, dx, dy),
                    )

    def test_joint_plank_long_axis_is_perpendicular_to_seeded_transfer(self):
        for seed in (41, 58, 73):
            arena = arena_for_seed(seed)
            plank = next(c for c in arena.cargo_specs if c.required_carriers == 2)
            dx = plank.goal_xyz[0] - plank.start_xyz[0]
            dy = plank.goal_xyz[1] - plank.start_xyz[1]
            distance = math.hypot(dx, dy)
            route_unit = (dx / distance, dy / distance)
            long_axis = (-math.sin(plank.start_yaw_rad), math.cos(plank.start_yaw_rad))
            self.assertAlmostEqual(
                route_unit[0] * long_axis[0] + route_unit[1] * long_axis[1],
                0.0,
                places=12,
            )
            self.assertAlmostEqual(plank.goal_yaw_rad, plank.start_yaw_rad, places=12)

    def test_rotated_plank_aabb_fits_in_source_and_destination_squares(self):
        for seed in range(20):
            arena = arena_for_seed(seed)
            plank = next(c for c in arena.cargo_specs if c.required_carriers == 2)
            half_width = plank.dimensions_m[0] / 2.0
            half_length = plank.dimensions_m[1] / 2.0
            cosine = abs(math.cos(plank.start_yaw_rad))
            sine = abs(math.sin(plank.start_yaw_rad))
            rotated_half = (
                cosine * half_width + sine * half_length,
                sine * half_width + cosine * half_length,
            )
            for point, zone_id in (
                (plank.start_xyz, arena.source_zone),
                (plank.goal_xyz, arena.destination_zone),
            ):
                zone = arena.zones[zone_id]
                self.assertLessEqual(
                    abs(point[0] - zone.center_xy[0]) + rotated_half[0],
                    zone.half_extents_xy[0],
                )
                self.assertLessEqual(
                    abs(point[1] - zone.center_xy[1]) + rotated_half[1],
                    zone.half_extents_xy[1],
                )
            for solo in (c for c in arena.cargo_specs if c.required_carriers == 1):
                self.assertEqual(solo.start_yaw_rad, 0.0)
                self.assertEqual(solo.goal_yaw_rad, 0.0)


if __name__ == "__main__":
    unittest.main()
