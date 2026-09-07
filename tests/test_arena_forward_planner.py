import math
import unittest
from sim.warehouse_arena import arena_for_seed
from sim import adaptive_warehouse as candidate


def unit(start, goal):
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    length = math.hypot(dx, dy)
    return dx/length, dy/length


def assert_clear(test, path, obstacles, footprint):
    for a, b in zip(path, path[1:]):
        for step in range(101):
            u = step/100
            x, y = a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1])
            for item in obstacles:
                if item.traversable:
                    continue
                test.assertFalse(
                    abs(x-item.center_xy[0]) <= item.half_extents_xy[0]+footprint[0]
                    and abs(y-item.center_xy[1]) <= item.half_extents_xy[1]+footprint[1],
                    (a, b, item.terrain_id),
                )


class ArenaForwardPlannerCandidateTests(unittest.TestCase):
    def test_zero_progress_axis_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "progress_axis must be nonzero"):
            candidate.plan_monotone_visibility_path(
                (0,0),(1,1),(),footprint_xy=(.1,.1),
                bounds=(-1,2,-1,2),progress_axis=(0,0),
            )

    def test_start_and_goal_must_be_inside_bounds(self):
        cases = (
            ((-1.01, 0.0), (0.5, 0.0), "path start is outside"),
            ((0.0, 0.0), (1.01, 0.0), "path goal is outside"),
        )
        for start, goal, message in cases:
            with self.subTest(start=start, goal=goal):
                with self.assertRaisesRegex(ValueError, message):
                    candidate.plan_monotone_visibility_path(
                        start, goal, (), footprint_xy=(.1,.1),
                        bounds=(-1,1,-1,1), progress_axis=(1,0),
                    )

    def test_goal_behind_progress_axis_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "goal is behind monotone progress axis"):
            candidate.plan_monotone_visibility_path(
                (0,0),(-.5,.2),(),footprint_xy=(.1,.1),
                bounds=(-1,1,-1,1),progress_axis=(1,0),
            )

    def test_twenty_generated_arenas_have_clear_monotone_plank_corridor(self):
        for seed in range(20):
            arena = arena_for_seed(seed)
            spec = next(item for item in arena.cargo_specs if item.required_carriers == 2)
            start, goal = spec.start_xyz[:2], spec.goal_xyz[:2]
            axis = unit(start, goal)
            c, s = abs(math.cos(spec.start_yaw_rad)), abs(math.sin(spec.start_yaw_rad))
            hx, hy = spec.dimensions_m[0]/2+.13, spec.half_length_m+.32
            footprint = (c*hx+s*hy, s*hx+c*hy)
            obstacles = tuple(candidate.TerrainObservation(
                item.terrain_id,item.kind,item.center_xy,item.half_extents_xy,
                item.height_m,item.traversable,item.cost_multiplier,
            ) for item in arena.terrain)
            bounds = (
                min(z.center_xy[0]-z.half_extents_xy[0] for z in arena.zones.values())-1.,
                max(z.center_xy[0]+z.half_extents_xy[0] for z in arena.zones.values())+1.,
                min(z.center_xy[1]-z.half_extents_xy[1] for z in arena.zones.values())-1.,
                max(z.center_xy[1]+z.half_extents_xy[1] for z in arena.zones.values())+1.,
            )
            path = candidate.plan_monotone_visibility_path(
                start,goal,obstacles,footprint_xy=footprint,
                bounds=bounds,progress_axis=axis,
            )
            for a,b in zip(path,path[1:]):
                self.assertGreaterEqual((b[0]-a[0])*axis[0]+(b[1]-a[1])*axis[1],-.020,seed)
            assert_clear(self,path,obstacles,footprint)

    def test_legacy_default_keeps_four_connected_axis_path(self):
        path = candidate.plan_local_path((0,0),(1,1),(),footprint_xy=(.1,.1),
                                         resolution_m=.1,bounds=(-.2,1.2,-.2,1.2))
        self.assertTrue(all(abs(a[0]-b[0])<1e-9 or abs(a[1]-b[1])<1e-9
                            for a,b in zip(path,path[1:])))


if __name__ == "__main__": unittest.main()
