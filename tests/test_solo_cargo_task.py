from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import sim.solo_cargo_task as solo_module
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.solo_cargo_task import SoloCargoTask
from sim.warehouse_arena import arena_for_seed

class SoloPhysicsTests(unittest.TestCase):
    def test_real_solo_cargo_contact_lift_delivery_without_pose_reset(self):
        w=MultiMasterPiProductionV2(warehouse_layout="mixed",render=False)
        s=replace(w.warehouse_spec_by_id['small_box_01'],carriers=('r3',))
        w._replace_warehouse_spec(s)
        try:
            t=SoloCargoTask(w,s,'r3',('A','B'))
            with patch.object(w.robot('r3'),'set_base_pose_for_test',side_effect=AssertionError('pose reset')),patch.object(w.robot('r3'),'set_free_body_pose_for_reset',side_effect=AssertionError('body reset')):
                while not t.done:
                    t.before_step();w._physics_step_for(w.robot('r1'))
            self.assertTrue(t.ok,t.reason)
            phases=[e['phase'] for e in w.warehouse_trace]
            self.assertIn('solo_contact_verified',phases)
            self.assertIn('solo_delivered',phases)
            self.assertLess(phases.index('solo_contact_verified'),phases.index('solo_delivered'))
            self.assertFalse(w._cargo_constraint_active(s.cargo_id,'r3'))
            self.assertTrue(w.warehouse_state()['cargo'][s.cargo_id]['evaluation']['success'])
        finally:w.close()

    def test_large_cargo_cannot_use_solo_controller(self):
        w=MultiMasterPiProductionV2(warehouse_layout="mixed",render=False)
        try:
            s=replace(w.warehouse_spec_by_id['oak_plank_01'],carriers=('r1',))
            with self.assertRaisesRegex(ValueError,'SOLO_CARGO_AWARD_REQUIRED'):
                SoloCargoTask(w,s,'r1',('A','B'))
        finally:w.close()


class SoloPlannerFootprintTests(unittest.TestCase):
    def _planned_footprint(self, layout, carrying):
        task=SoloCargoTask.__new__(SoloCargoTask)
        task.rid="r1"
        task.spec=SimpleNamespace(cargo_id="small_box_01")
        task.robot=SimpleNamespace(base_xyz=lambda:np.asarray((0.,0.,.0325)))
        task.world=SimpleNamespace(
            warehouse_layout=layout,
            warehouse_specs=(),
            _cargo_constraint_active=lambda cargo_id,rid:carrying,
            _warehouse_navigation_observations=lambda **kwargs:(),
        )
        captured={}
        def plan(start,target,obstacles,**kwargs):
            captured.update(kwargs)
            return (tuple(start),tuple(target))
        with patch.object(solo_module,"plan_local_path",side_effect=plan), \
             patch.object(solo_module,"_navigation_bounds",return_value=(-2.,2.,-2.,2.)):
            task._path((1.,0.))
        return captured["footprint_xy"]

    def test_arena_carry_path_covers_deployed_arm_and_payload(self):
        self.assertEqual(self._planned_footprint("arena",True),(.30,.30))

    def test_contact_approach_and_legacy_carry_keep_tight_footprint(self):
        self.assertEqual(self._planned_footprint("arena",False),(.13,.13))
        self.assertEqual(self._planned_footprint("mixed",True),(.13,.13))

    def test_future_joint_parking_reservations_cover_retreat_without_blocking_solo_goals(self):
        for seed in range(20):
            arena=arena_for_seed(seed)
            world=SimpleNamespace(warehouse_layout="arena",warehouse_specs=arena.cargo_specs)
            reserved=solo_module._future_joint_parking_obstacles(world)
            self.assertEqual(len(reserved),2)
            plank=next(c for c in arena.cargo_specs if c.required_carriers==2)
            axis=np.asarray((-np.sin(plank.goal_yaw_rad),np.cos(plank.goal_yaw_rad)))
            goal=np.asarray(plank.goal_xyz[:2])
            for sign,item in zip((-1.,1.),reserved):
                self.assertAlmostEqual(
                    item.half_extents_xy[0],max(.10,.155*abs(axis[0])),places=12,
                )
                self.assertAlmostEqual(
                    item.half_extents_xy[1],max(.10,.155*abs(axis[1])),places=12,
                )
                for distance in (.36,.515):
                    point=goal+sign*distance*axis
                    self.assertLessEqual(abs(point[0]-item.center_xy[0]),item.half_extents_xy[0]+1e-12)
                    self.assertLessEqual(abs(point[1]-item.center_xy[1]),item.half_extents_xy[1]+1e-12)
            for cargo in (c for c in arena.cargo_specs if c.required_carriers==1):
                delta=np.asarray(cargo.goal_xyz[:2])-np.asarray(cargo.start_xyz[:2])
                anchor=np.asarray(cargo.goal_xyz[:2])-.165*delta/np.linalg.norm(delta)
                self.assertTrue(all(
                    abs(anchor[0]-item.center_xy[0])>item.half_extents_xy[0]+.30
                    or abs(anchor[1]-item.center_xy[1])>item.half_extents_xy[1]+.30
                    for item in reserved
                ),(seed,cargo.cargo_id))

    def test_future_joint_parking_reservations_are_arena_only(self):
        world=SimpleNamespace(warehouse_layout="mixed",warehouse_specs=())
        self.assertEqual(solo_module._future_joint_parking_obstacles(world),())

if __name__=="__main__":unittest.main()
