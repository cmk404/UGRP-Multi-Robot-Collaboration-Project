"""Environment v3: wall profile walls_v3 (0.40 m) and the sparse tag rule ugrp.zone_tag_rule.v3.

Pins every earlier zone map file byte for byte, checks the v3 maps against the
written rule (experiments/2026-09-26-zone-env-v3/tag_rule_v3.md) and, with
MuJoCo, that the compiled scene has 0.40 m walls, visual-only tags, the approved
cargo contact profile, and that tags do not change the physics.
"""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import math
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAPS = ROOT/'maps'/'zones'
# Every zone map file published before v3 (unchanged by this work) and the three v3 files.
PUBLISHED_SHA256 = {
    'zone_open': '2057e40536f0f71077ff362ee127b36c6a4a486fca409d92cfb348a3a8c1c1e0',
    'zone_wide': '6b7b9eb671e5114c78b52e1ee0a4cc02288038aa04edfe37ab4f908bbac6f852',
    'zone_wide_corridor': 'f7b62732a6db80b97f51cb792082b702dfbbade20de7c81a7795611794919c32',
    'zone_wide_corridor_tags_v1': 'a349d42d60f3fcefd1efa29016aeb39e15e40c0092268c0485814e7f9f3916db',
    'zone_wide_door': 'a4d2c03de5d0085c2d0ea04d631ec4e9dd95de6dffbb8d44c09870e2ec921903',
    'zone_wide_door_tags_v1': 'f86fc314ed3c4c2866f9b5c8448b9b1919462f0c1a958bedf46442513604ea14',
    'zone_wide_door_tags_v2': 'f9b0ef0d9f35457b34711c0ec5d11688af3130238560195c982660137ad3e73b',
    'zone_wide_two_doors': '04b6baf6e457e65e490f921ad2f82aa2778726bf6e7d547fdb5158e55b5b84d2',
    'zone_wide_two_doors_tags_v1': '2562d2f09940e9ba864cf1fe71740592e5305835a6c1af24b3493298a583c4ed',
    'zone_wide_two_doors_tags_v2': '10f9b2f854a161843d9dc53bb518431cd9a35317b4252491f8e4186a25e3b206',
    'zone_wide_door_tags_v3': '74a3a95925a9531878b43a9c63d26f34ce391ede852d90b4f592e54c793fee0e',
    'zone_wide_two_doors_tags_v3': '9a1391762661c0303105878caa7854a031896bed68b5c2651031d29dc9a8d290',
    'zone_wide_corridor_tags_v3': '1414781ac689012dc5d679266da625c36ab9d5362ebeddbbd342a3ad81ba5243',
}
# Robot prompt text of the walled base maps (sim.zone_arena.static_map_text) before v3.
BASE_TEXT_SHA256 = {
    'zone_wide_door': '37413609a00f921800c8f7355640c992cb5c8648078c4350b0f377d386bb6868',
    'zone_wide_two_doors': 'c2823ec8de94245a26f363dd3847693d3d7d4615a3c9b96d0c70103249ec9aa5',
    'zone_wide_corridor': '2df40afbd13c1bdd09baa89178d53f94ae3f80d289ed98a5ce2fb426a0bb1a9d',
}
V3 = ('zone_wide_door_tags_v3', 'zone_wide_two_doors_tags_v3', 'zone_wide_corridor_tags_v3')
V3_COUNTS = {'zone_wide_door_tags_v3': (33, 11), 'zone_wide_two_doors_tags_v3': (36, 12),
             'zone_wide_corridor_tags_v3': (33, 11)}
# zone_wide_door_tags_v3 sites of tag_rule_v3.md: (kind, wall, normal, centre).
DOOR_SITES = [
    ('door_frame', 'wall_divider_1', [-1, 0], [2.175, -.25]), ('door_frame', 'wall_divider_1', [1, 0], [2.225, -.25]),
    ('door_frame', 'wall_divider_2', [-1, 0], [2.175, .35]), ('door_frame', 'wall_divider_2', [1, 0], [2.225, .35]),
    ('pickup_bay', 'wall_south', [0, 1], [.125, -3.125]), ('pickup_bay', 'wall_divider_1', [-1, 0], [2.175, -.85]),
    ('pickup_bay', 'wall_north', [0, -1], [.125, 1.425]), ('pickup_bay', 'wall_divider_1', [-1, 0], [2.175, -2.15]),
    ('zone', 'wall_east', [-1, 0], [5.375, .40]), ('zone', 'wall_east', [-1, 0], [5.375, -2.10]),
    ('zone', 'wall_east', [-1, 0], [5.375, -.85])]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PublishedMapTests(unittest.TestCase):
    def test_every_published_zone_map_file_is_pinned_and_unchanged(self):
        on_disk = {p.stem for p in MAPS.glob('*.json')}
        self.assertLessEqual(set(PUBLISHED_SHA256), on_disk)
        for name, digest in PUBLISHED_SHA256.items():
            self.assertEqual(sha(MAPS/f'{name}.json'), digest, name)

    def test_base_map_prompt_text_is_unchanged(self):
        from sim.zone_arena import authored_map, static_map_text
        for name, digest in BASE_TEXT_SHA256.items():
            self.assertEqual(hashlib.sha256(static_map_text(authored_map(name)).encode()).hexdigest(), digest)

    def test_v1_v2_registry_is_unchanged(self):
        from sim.zone_landmarks import ALL_TAGGED_MAPS, ENV_V3_TAGGED_MAPS, TAGGED_MAPS
        self.assertEqual(set(TAGGED_MAPS), {'zone_wide_door_tags_v1', 'zone_wide_two_doors_tags_v1',
                                            'zone_wide_corridor_tags_v1', 'zone_wide_door_tags_v2',
                                            'zone_wide_two_doors_tags_v2'})
        self.assertEqual(set(ENV_V3_TAGGED_MAPS), set(V3))
        self.assertEqual(set(ALL_TAGGED_MAPS), set(TAGGED_MAPS) | set(V3))


class WallProfileTests(unittest.TestCase):
    def test_profiles(self):
        from sim.zone_arena import WALL_PROFILES, apply_wall_profile, authored_map, wall_profile_record
        self.assertEqual(WALL_PROFILES['walls_v1']['height_m'], .10)
        self.assertEqual(WALL_PROFILES['walls_v3']['height_m'], .40)
        base = authored_map('zone_wide_door')
        before = copy.deepcopy(base)
        tall = apply_wall_profile(base, 'walls_v3')
        self.assertEqual(base, before)                       # the input map is not modified
        self.assertEqual(tall['wall_profile'], wall_profile_record('walls_v3'))
        self.assertEqual({o['height_m'] for o in base['obstacles']}, {.10})
        self.assertEqual({o['height_m'] for o in tall['obstacles']}, {.40})
        strip = lambda m: [{k: v for k, v in o.items() if k != 'height_m'} for o in m['obstacles']]
        self.assertEqual(strip(base), strip(tall))            # footprints unchanged
        with self.assertRaises(ValueError):
            wall_profile_record('walls_v2')

    def test_wall_height_rule_matches_the_recorded_analysis(self):
        import json
        from sim.zone_arena import WALL_PROFILES
        data = json.loads((ROOT/'experiments'/'2026-09-26-zone-env-v3'/'camera_height.json').read_text())
        cam, top = data['kinematic_max']['camera_z_m'], data['robot_top_max']['z_m']
        spec = WALL_PROFILES['walls_v3']
        self.assertAlmostEqual(spec['kinematic_max_camera_z_m'], cam, places=3)
        self.assertAlmostEqual(spec['robot_top_max_z_m'], top, places=3)
        self.assertAlmostEqual(spec['max_work_posture_camera_z_m'], data['max_named_camera_z_m'], places=3)
        self.assertAlmostEqual(spec['height_m'], math.ceil(max(cam + .05, top + .03)/.05 - 1e-9)*.05, places=9)

    def test_prompt_text_reports_the_map_wall_height(self):
        from sim.zone_arena import static_map_text
        from sim.zone_landmarks import tagged_map
        self.assertIn('(0.40 m high;', static_map_text(tagged_map('zone_wide_door_tags_v3')))


class TagRuleV3Tests(unittest.TestCase):
    def test_v3_maps_match_definition_base_and_profile(self):
        from sim.research_dispatch_arena import digest
        from sim.zone_arena import apply_wall_profile, authored_map
        from sim.zone_landmarks import ENV_V3_TAGGED_MAPS, tagged_map
        for name in V3:
            static = tagged_map(name)                          # refuses drift from the definition
            spec = ENV_V3_TAGGED_MAPS[name]
            base = authored_map(spec['base'])
            self.assertEqual(static['base_map'], {'map_id': base['map_id'], 'version': base['version'],
                                                  'static_map_sha256': digest(base)})
            self.assertEqual((static['map_id'], static['version']), (name, 3))
            walled = apply_wall_profile(base, 'walls_v3')
            self.assertEqual({k: v for k, v in static.items() if k not in ('map_id', 'version', 'base_map', 'landmarks')},
                             {k: v for k, v in walled.items() if k not in ('map_id', 'version')})
            self.assertEqual(static['landmarks']['placement']['rule'], 'ugrp.zone_tag_rule.v3')

    def test_counts_sites_and_columns(self):
        from sim.zone_landmarks import tagged_map
        for name, (n_tags, n_sites) in V3_COUNTS.items():
            marks = tagged_map(name)['landmarks']
            self.assertEqual((len(marks['tags']), len(marks['sites'])), (n_tags, n_sites), name)
            self.assertEqual([t['id'] for t in marks['tags']], list(range(n_tags)))
            for site in marks['sites']:
                column = [t for t in marks['tags'] if t['site'] == site['id']]
                self.assertEqual([t['center_m'][2] for t in column], [.05, .15, .25])
                self.assertEqual({tuple(t['center_m'][:2]) for t in column}, {tuple(site['center_m'])})
                self.assertEqual(site['tag_ids'], [t['id'] for t in column])
        sites = tagged_map('zone_wide_door_tags_v3')['landmarks']['sites']
        self.assertEqual([(s['kind'], s['wall'], s['normal_xy'], s['center_m']) for s in sites], DOOR_SITES)

    def test_fewer_tags_than_v1_and_v2(self):
        from sim.zone_landmarks import tagged_map
        for base in ('zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor'):
            v3 = len(tagged_map(base + '_tags_v3')['landmarks']['tags'])
            self.assertLess(v3, .6*len(tagged_map(base + '_tags_v1')['landmarks']['tags']))

    def test_tags_sit_on_wall_faces_below_the_wall_top_and_clear_of_other_walls(self):
        from sim.zone_landmarks import tagged_map
        for name in V3:
            static = tagged_map(name)
            walls = {o['id']: o for o in static['obstacles']}
            plate = static['landmarks']['placement']['plate_m']
            for tag in static['landmarks']['tags']:
                wall = walls[tag['wall']]
                (cx, cy), (hx, hy) = wall['center_m'], wall['half_extents_m']
                nx, ny = tag['normal_xy']
                x, y, z = tag['center_m']
                if nx:
                    self.assertAlmostEqual(x, cx + nx*hx, places=4)
                else:
                    self.assertAlmostEqual(y, cy + ny*hy, places=4)
                self.assertLessEqual(z + plate/2, wall['height_m'])
                self.assertGreaterEqual(z - plate/2, 0.)
                along = abs(x - cx) if ny else abs(y - cy)
                self.assertLessEqual(along + plate/2, (hx if ny else hy) + 1e-9)
                for other in static['obstacles']:
                    if other['id'] == wall['id']:
                        continue
                    (ox, oy), (ohx, ohy) = other['center_m'], other['half_extents_m']
                    for side in (-plate/2, plate/2):      # both plate edges stand in free space
                        px = x + nx*.005 + (-ny)*side
                        py = y + ny*.005 + nx*side
                        self.assertFalse(abs(px - ox) < ohx and abs(py - oy) < ohy, (name, tag['id']))

    def test_door_frames_are_the_v2_post_tag_positions(self):
        from sim.zone_landmarks import tagged_map
        v2 = tagged_map('zone_wide_door_tags_v2')['landmarks']['tags']
        v3 = tagged_map('zone_wide_door_tags_v3')['landmarks']['tags']
        posts = {(tuple(t['center_m']), tuple(t['normal_xy'])) for t in v2 if t.get('mount') == 'door_post'}
        frames = {(tuple(t['center_m']), tuple(t['normal_xy'])) for t in v3 if t['site'] in
                  ('site_00', 'site_01', 'site_02', 'site_03') and t['center_m'][2] > .1}
        self.assertEqual(frames, posts)

    def test_pickup_bays_match_the_study_projection_split(self):
        from sim.zone_landmarks import tagged_map
        bays = tagged_map('zone_wide_door_tags_v3')['landmarks']['pickup_bays']
        # harness/zone_map_schematic.pickup_bays (PR #190): 2 columns x 3 rows over regions.pickup
        self.assertEqual([(b['bay_id'], b['center_m'], b['half_extents_m']) for b in bays],
                         [('P1', [.125, -.85], [.575, 1.95]), ('P2', [1.275, -.85], [.575, 1.95])])
        self.assertEqual([s['center_m'] for s in bays[0]['slots']], [[.125, -2.15], [.125, -.85], [.125, .45]])

    def test_rule_is_deterministic_and_map_only(self):
        from sim.zone_arena import apply_wall_profile, authored_map
        from sim.zone_tag_rule_v3 import PLACEMENT_V3, place_tags_v3
        static = apply_wall_profile(authored_map('zone_wide_corridor'), 'walls_v3')
        self.assertEqual(place_tags_v3(static, PLACEMENT_V3), place_tags_v3(copy.deepcopy(static), PLACEMENT_V3))
        low = authored_map('zone_wide_corridor')               # 0.10 m walls cannot hold the 0.25 m tag
        with self.assertRaises(ValueError):
            place_tags_v3(low, PLACEMENT_V3)
        source = (ROOT/'sim'/'zone_tag_rule_v3.py').read_text()
        for word in ('mujoco', 'xpos', 'qpos', 'eval_only'):
            self.assertNotIn(word, source)

    def test_corridor_sites(self):
        from sim.zone_landmarks import tagged_map
        sites = tagged_map('zone_wide_corridor_tags_v3')['landmarks']['sites']
        got = {(s['kind'], s['wall'], tuple(s['normal_xy']), tuple(s['center_m'])) for s in sites
               if s['kind'].startswith(('corridor', 'passing'))}
        self.assertEqual(got, {('corridor_entrance', 'wall_divider_1', (-1, 0), (2.175, .875)),
                               ('corridor_axis', 'wall_east', (-1, 0), (5.375, 1.175)),
                               ('passing_bay', 'wall_bay_south', (0, 1), (3.1, .375))})


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'mujoco is not installed')
class V3SceneTests(unittest.TestCase):
    GOAL, EXTRA = {'A': {'cyan': 1}}, {'red': 1}

    def world(self, name, profile, tags=True, transform=None):
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_landmarks import TaggedZoneScene
        scene = TaggedZoneScene.from_tagged(name, 11, self.GOAL, self.EXTRA, contact_profile=profile)
        if not tags:
            from sim.zone_scene import ZoneScene
            fn = lambda xml: ZoneScene.transform(scene, xml)
        else:
            fn = transform or scene.transform
        world = MultiMasterPiProductionV2(seed=11, width=640, height=480, render=False,
                                          warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                          xml_transform=fn)
        scene.setup(world)
        return scene, world

    def test_compiled_walls_are_0p40_m_with_visual_only_tags_and_noslip(self):
        import mujoco
        scene, world = self.world('zone_wide_door_tags_v3', 'cargo_noslip_v1')
        try:
            model = world.model
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(model.ngeom)]
            walls = [g for g, n in enumerate(names) if n.startswith('zone_wall_')]
            self.assertEqual(len(walls), len(scene.config['static_map']['obstacles']))
            for g in walls:
                self.assertAlmostEqual(float(model.geom_size[g][2]), .20, places=6)
                self.assertAlmostEqual(float(model.geom_pos[g][2]), .20, places=6)
                self.assertEqual((int(model.geom_contype[g]), int(model.geom_conaffinity[g])), (1, 3))
            tags = [g for g, n in enumerate(names) if n.startswith('tag_')]
            self.assertEqual(len(tags), scene.manifest['tag_geoms'])
            for g in tags:
                self.assertEqual((int(model.geom_contype[g]), int(model.geom_conaffinity[g])), (0, 0))
            self.assertEqual(int(model.opt.noslip_iterations), 10)
            self.assertEqual(scene.manifest['contact_solver_profile'], 'cargo_noslip_v1')
            self.assertEqual(scene.manifest['wall_profile']['id'], 'walls_v3')
            self.assertEqual(scene.manifest['tag_count'], 33)
        finally:
            world.close()

    def test_zone_scene_cargo_profile_equals_the_m1_runner_construction(self):
        from sim.zone_cargo_contact import apply
        scene_a, a = self.world('zone_wide_door_tags_v2', 'cargo_noslip_v1')
        try:
            xml_a = a.scene_xml
        finally:
            a.close()
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_landmarks import TaggedZoneScene
        scene_b = TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 11, self.GOAL, self.EXTRA,
                                              contact_profile='local_contact_fine')
        b = MultiMasterPiProductionV2(seed=11, width=640, height=480, render=False,
                                      warehouse_layout=scene_b.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=lambda xml: apply(scene_b.transform(xml), 'cargo_noslip_v1'))
        try:
            self.assertEqual(xml_a, b.scene_xml)
        finally:
            b.close()

    def test_v3_tags_do_not_change_physics(self):
        import numpy as np
        from sim.camera_robot_port import CameraRobotPort
        runs = []
        for tags in (True, False):
            scene, world = self.world('zone_wide_door_tags_v3', 'cargo_noslip_v1', tags=tags)
            try:
                ports = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in ('r1', 'r2')}
                t0 = float(world.data.time)
                ports['r1'].apply({'kind': 'mecanum', 'forward': .12, 'left': .03, 'turn': .05, 'duration_s': 1.}, t0)
                ports['r2'].apply({'kind': 'arm', 'servo_id': 3, 'pulse': 900}, t0)
                trace = []
                for _ in range(int(1.2/float(world.model.opt.timestep))):
                    now = float(world.data.time)
                    for port in ports.values():
                        port.tick(now)
                    world._physics_step_for(world.controllers['r1'])
                    trace.append(world.data.qpos.copy())
                runs.append(np.array(trace))
            finally:
                world.close()
        self.assertTrue(np.array_equal(runs[0], runs[1]), float(np.abs(runs[0] - runs[1]).max()))
        self.assertGreater(float(np.abs(runs[0][-1] - runs[0][0]).max()), .05)


if __name__ == '__main__':
    unittest.main()
