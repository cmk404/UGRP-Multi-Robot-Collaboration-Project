"""TaggedCargoZoneScene: tagged static map + catalogue cargo (M2 pair beam carry)."""
from __future__ import annotations

import importlib.util
import unittest

GOAL = {'A': {'cyan': 1}}
BEAM = {'item_id': 'beam', 'kind': 'long_beam', 'pose': [1.0, -1.2, 0.2]}


def build(cls_tagged_cargo, cargo, profile='local_contact_fine'):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_landmarks import TaggedZoneScene
    from sim.zone_tagged_cargo_scene import TaggedCargoZoneScene
    if cls_tagged_cargo:
        scene = TaggedCargoZoneScene.from_tagged_cargo('zone_wide_door_tags_v2', 11, cargo=cargo, goal=GOAL,
                                                       contact_profile=profile)
    else:
        scene = TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 11, GOAL, None, contact_profile=profile)
    world = MultiMasterPiProductionV2(seed=11, width=640, height=480, render=False,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    return scene, world


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'mujoco is not installed')
class TaggedCargoSceneTests(unittest.TestCase):
    def test_no_cargo_equals_tagged_scene(self):
        _, a = build(False, None)
        _, b = build(True, [])
        self.assertEqual(a.scene_xml, b.scene_xml)

    def test_beam_is_placed_and_profile_applied(self):
        import mujoco
        scene, world = build(True, [BEAM], profile='cargo_noslip_v1')
        self.assertEqual([i['item_id'] for i in scene.config['cargo_set']['items']], ['beam'])
        self.assertIn('cargo_contact_profile', scene.manifest)
        mujoco.mj_forward(world.model, world.data)
        from sim.zone_cargo import instances
        body = world.data.body(instances([BEAM])[0].body)
        self.assertAlmostEqual(float(body.xpos[0]), 1.0, places=2)
        self.assertAlmostEqual(float(body.xpos[1]), -1.2, places=2)
        # static map tags are still present (visual only)
        self.assertTrue(scene.config['static_map'].get('tags') or scene.config['static_map'].get('landmarks'))


if __name__ == '__main__':
    unittest.main()
