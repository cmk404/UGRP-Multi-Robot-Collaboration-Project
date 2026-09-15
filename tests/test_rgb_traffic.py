from __future__ import annotations

import copy
import unittest

from harness.traffic_reservations import TrafficCoordinator, TrafficCommandGate, paths_conflict
from scripts.traffic_scenarios import SCENARIOS, scenario


def report(unit, sequence=0, now=0., *, position=None, route=None, arrived=False):
    path = route or ([[-1., 0.], [1., 0.]] if unit == 'r1' else [[0., -1.], [0., 1.]])
    return {'unit_id': unit, 'participants': [unit], 'accepted_by': [unit], 'task_id': 'test:'+unit,
        'plan_version': 1, 'route_version': 1, 'sequence': sequence, 'clock_s': now,
        'clock_id': 'issued_command_clock_v1', 'source': 'top_rgb', 'top_sha256': 'a'*64,
        'position_m': position or path[0], 'heading_rad': None, 'uncertainty_m': .035,
        'footprint': {'kind': 'unloaded_circle', 'radius_m': .18}, 'route': path,
        'arrived': arrived, 'visual_ok': True}


def reports(sequence=0, now=0.):
    return {u: report(u, sequence, now) for u in ('r1', 'r3')}


class TrafficTests(unittest.TestCase):
    def test_continuous_conflict_geometry(self):
        self.assertTrue(paths_conflict([[-1,0],[1,0]], [[0,-1],[0,1]], .1))
        self.assertTrue(paths_conflict([[-1,0],[1,0]], [[1,0],[-1,0]], .1))
        self.assertTrue(paths_conflict([[0,0],[2,0]], [[1,0],[3,0]], .1))
        self.assertFalse(paths_conflict([[0,0],[1,0]], [[0,1],[1,1]], .9))
        self.assertFalse(paths_conflict([[0,0],[1,0]], [[2,0],[3,0]], .9))
        self.assertTrue(paths_conflict([[0,0]], [[-.1,0],[.1,0]], .1))

    def test_wait_until_fresh_stationary_clear_then_resume(self):
        coordinator = TrafficCoordinator(('r1','r3'))
        permission = coordinator.step(reports(), 0.)
        self.assertEqual([permission[u]['phase'] for u in ('r1','r3')], ['GO','HOLD'])
        permission = coordinator.step(reports(1, 50.), 50.)
        self.assertEqual(permission['r3']['phase'], 'HOLD', 'ETA/GO expiry must not transfer occupancy')
        for sequence in (2, 3, 4):
            snapshot = reports(sequence, 50.+sequence)
            snapshot['r1'] = report('r1', sequence, 50.+sequence, position=[1.,0.], route=[[1.,0.]], arrived=True)
            permission = coordinator.step(snapshot, 50.+sequence)
        self.assertEqual(permission['r3']['phase'], 'GO')
        events = [e['event'] for e in coordinator.events]
        self.assertLess(events.index('CLEAR'), events.index('RELEASE'))
        self.assertNotIn('r1', coordinator.reservations)

    def test_missing_duplicate_reordered_invalid_reports_hold_both_without_release(self):
        for corrupt in ('missing', 'duplicate', 'old', 'truth', 'nan', 'wrong_unit', 'empty'):
            with self.subTest(corrupt=corrupt):
                coordinator = TrafficCoordinator(('r1','r3'))
                coordinator.step(reports(), 0.)
                data = reports(1, .5)
                if corrupt == 'missing': del data['r1']
                if corrupt == 'duplicate': data['r1']['sequence'] = 0
                if corrupt == 'old': data['r1']['clock_s'] = 0.
                if corrupt == 'truth': data['r1']['source'] = 'simulator_pose'
                if corrupt == 'nan': data['r1']['position_m'][0] = float('nan')
                if corrupt == 'wrong_unit': data['r1']['unit_id'] = 'r3'
                if corrupt == 'empty': data['r1'] = None
                decision = coordinator.step(data, .5)
                self.assertTrue(all(p['phase'] == 'HOLD' for p in decision.values()))
                self.assertIn('r1', coordinator.reservations)
                self.assertEqual(coordinator.reports['r1']['sequence'], 0)

    def test_restart_preserves_possible_occupancy_and_invalidates_old_go(self):
        original = TrafficCoordinator(('r1','r3'))
        data = reports(); permission = original.step(data, 0.)
        restored = TrafficCoordinator.restore(original.snapshot())
        self.assertEqual(original.reservations, restored.reservations)
        self.assertGreater(restored.epoch, original.epoch)
        gate = TrafficCommandGate('r1')
        self.assertFalse(gate.accept(permission['r1'], data['r1'], now=.1,
            epoch=restored.epoch, current_generation=1, duration_s=.25))
        new = restored.step(reports(1, 100.), 100.)
        self.assertEqual(new['r3']['phase'], 'HOLD')

    def test_plan_change_keeps_occupied_route_and_stops_owner(self):
        coordinator = TrafficCoordinator(('r1','r3'))
        coordinator.step(reports(), 0.)
        data = reports(1, .5); data['r1']['plan_version'] = 2
        permission = coordinator.step(data, .5)
        self.assertTrue(all(p['phase'] == 'HOLD' for p in permission.values()))
        self.assertTrue(coordinator.reservations['r1']['replan_blocked'])

    def test_parked_body_on_exit_prevents_grant(self):
        coordinator = TrafficCoordinator(('r1','r3'))
        data = reports(); data['r3'] = report('r3', position=[.8,0.], route=[[.8,0.]], arrived=True)
        permission = coordinator.step(data, 0.)
        self.assertEqual(permission['r1']['phase'], 'HOLD')
        self.assertIn('peer_occupies_route', permission['r1']['reason'])

    def test_route_extension_retains_old_occupancy_and_invalidates_generation(self):
        coordinator = TrafficCoordinator(('r1','r3'))
        old = coordinator.step(reports(), 0.)
        data = reports(1, .5)
        data['r1']['route_version'] = 2
        data['r1']['route'] = [[-1.,0.], [-.3,.12], [1.,0.]]
        new = coordinator.step(data, .5)
        self.assertEqual(new['r1']['phase'], 'GO')
        self.assertGreater(new['r1']['generation'], old['r1']['generation'])
        self.assertEqual(len(coordinator.reservations['r1']['routes']), 2)
        self.assertEqual(new['r3']['phase'], 'HOLD')
        self.assertFalse(TrafficCommandGate('r1').accept(old['r1'], data['r1'], now=.5,
            epoch=1, current_generation=new['r1']['generation'], duration_s=.05))

    def test_gate_rejects_wrong_identity_generation_expiry_and_replay(self):
        coordinator = TrafficCoordinator(('r1','r3'))
        data = reports(); permission = coordinator.step(data, 0.)['r1']
        for field, value in [('generation', 99), ('identity', ['wrong',1,1]),
                             ('unit_id', 'r3'), ('sequence', -1), ('expires_at_s', .1)]:
            candidate = copy.deepcopy(permission); candidate[field] = value
            self.assertFalse(TrafficCommandGate('r1').accept(candidate, data['r1'], now=.1,
                epoch=1, current_generation=1, duration_s=.25))
        gate = TrafficCommandGate('r1')
        self.assertTrue(gate.accept(permission, data['r1'], now=0., epoch=1, current_generation=1, duration_s=.6))
        self.assertFalse(gate.accept(permission, data['r1'], now=0., epoch=1, current_generation=1, duration_s=.6))

    def test_authored_scenarios_keep_camera_and_body_bounds(self):
        for name in SCENARIOS:
            maps, setup = scenario(name)
            self.assertEqual(maps['r1']['obstacles'], maps['r3']['obstacles'])
            for unit in maps:
                self.assertEqual(maps[unit]['top_camera']['fov_y_deg'], 55)
                self.assertEqual(maps[unit]['zones']['start']['center_m'], setup[unit]['start_xy_m'])


if __name__ == '__main__':
    unittest.main()
