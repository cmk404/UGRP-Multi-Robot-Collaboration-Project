"""Own-camera observation memory ("look once, remember"): boundary, reuse adapters, memory and policy."""
from __future__ import annotations

import ast
import json
import math
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

RUNTIME = (ROOT/'harness'/'owncam_memory.py', ROOT/'harness'/'owncam_memory_kf.py',
           ROOT/'harness'/'owncam_drive_mem.py', ROOT/'harness'/'m1_owncam_memory.py')
ALLOWED = {'__future__', 'math', 'copy', 'collections.abc', 'numpy', 'cv2', 'harness.owncam_drive',
           'harness.owncam_memory_kf', 'harness.wall_tags', 'sim.masterpi_camera_profile', 'harness.zone_color_boxes',
           'harness.owncam_memory', 'harness.owncam_drive_mem', 'harness.m1_owncam_delivery',
           'harness.owncam_pose_source'}
FORBIDDEN = ('mujoco', 'xpos', 'xquat', 'xmat', 'qpos', 'qvel', 'base_xyz', 'base_rpy', 'eval_only', 'gt_trajectory',
             'frames_eval', 'MjData', 'setup_only', 'position_m', 'GtStubPoseSource', 'cctv_top', 'nav_cam')
CAL = ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'calibration_m1_dev.json'
SEARCH = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}
CARRY = {1: 1500, 3: 777, 4: 2053, 5: 1646, 6: 1500}


def tagged(name):
    return json.loads((ROOT/'maps'/'zones'/f'{name}.json').read_text())


def params():
    return json.loads(CAL.read_text())['params']


def report(x, y, yaw, sxy=.03, syaw=.01, t=0., load='unloaded'):
    from harness.owncam_pose_source import PoseReport
    cov = np.diag([sxy**2/2, sxy**2/2, syaw**2])
    return PoseReport(t_est=t, initialized=True, x_m=x, y_m=y, yaw_rad=yaw, cov=tuple(map(tuple, cov)),
                      std_xy_m=sxy, std_yaw_rad=syaw, since_tag_s=0., load_state=load, source='owncam_pf_v2:abcd1234')


class BoundaryTests(unittest.TestCase):
    def test_imports_are_allowed(self):
        for path in RUNTIME:
            for node in ast.walk(ast.parse(path.read_text())):
                names = ([a.name for a in node.names] if isinstance(node, ast.Import) else
                         [node.module] if isinstance(node, ast.ImportFrom) else [])
                for n in names:
                    with self.subTest(path=path.name, module=n):
                        self.assertIn(n, ALLOWED)

    def test_no_simulator_state_names(self):
        for path in RUNTIME:
            tree = ast.parse(path.read_text())
            names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | \
                    {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
            strings = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
            for token in FORBIDDEN:
                with self.subTest(path=path.name, token=token):
                    self.assertNotIn(token, names)
                    self.assertFalse(any(token in s for s in strings if not s.startswith(('"""', 'Own', 'M1'))
                                         and '\n' not in s))

    def test_memory_modules_import_with_mujoco_poisoned(self):
        code = ("import sys; sys.modules['mujoco'] = None\n"
                "import harness.owncam_memory, harness.owncam_memory_kf, harness.owncam_drive_mem, harness.m1_owncam_memory\n"
                "bad = [m for m in sys.modules if m.startswith(('sim.multi_masterpi', 'sim.zone_scene', 'sim.zone_arena', "
                "'scripts.zone_teacher', 'sim.session'))]\n"
                "assert not bad, bad\n")
        out = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(out.returncode, 0, out.stderr[-2000:])

    def test_frozen_m1_files_are_unchanged(self):
        frozen = json.loads((ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'frozen_source.json').read_text())
        import hashlib
        for f, want in frozen['sha256'].items():
            with self.subTest(file=f):
                self.assertEqual(hashlib.sha256((ROOT/f).read_bytes()).hexdigest(), want)


class KalmanAdapterTests(unittest.TestCase):
    def test_update_matches_the_textbook_filter_and_joseph_form(self):
        from harness.owncam_memory_kf import kf_predict, kf_update
        x, P = np.array([1., 2.]), np.diag([.04, .09])
        x, P = kf_predict(x, P, .01*np.eye(2))
        np.testing.assert_allclose(P, np.diag([.05, .10]))
        z, R = np.array([1.2, 1.7]), np.diag([.01, .02])
        x1, P1, info = kf_update(x, P, z, R)
        K = P @ np.linalg.inv(P + R)
        np.testing.assert_allclose(x1, x + K @ (z - x))
        np.testing.assert_allclose(P1, (np.eye(2) - K) @ P, atol=1e-12)      # optimal gain: Joseph == simple form
        y, S = z - x, P + R
        self.assertAlmostEqual(info['mahalanobis2'], float(y @ np.linalg.solve(S, y)))
        self.assertAlmostEqual(info['loglik'], float(-.5*(2*math.log(2*math.pi) + math.log(np.linalg.det(S))
                                                          + y @ np.linalg.solve(S, y))))

    def test_association_new_update_and_ambiguous(self):
        from harness.owncam_memory_kf import associate
        I = np.eye(2)
        tracks = [(np.array([0., 0.]), .0004*I), (np.array([1., 0.]), .0004*I)]
        rows = associate(tracks, [(np.array([.01, 0.]), .0004*I), (np.array([3., 0.]), .0004*I)])
        self.assertEqual([r['decision'] for r in rows], ['update', 'new'])
        self.assertEqual(rows[0]['track'], 0)
        wide = [(np.array([0., 0.]), .25*I), (np.array([.1, 0.]), .25*I)]
        self.assertEqual(associate(wide, [(np.array([.05, 0.]), .01*I)])[0]['decision'], 'ambiguous')
        # one-to-one per frame: two detections at one track -> the second is not a new object either
        rows = associate(tracks[:1], [(np.array([0., 0.]), .0004*I), (np.array([.005, 0.]), .0004*I)])
        self.assertEqual(sorted(r['decision'] for r in rows), ['ambiguous', 'update'])

    def test_observation_covariance_propagates_the_pose(self):
        from harness.owncam_memory_kf import observation_to_map
        cov = np.diag([0., 0., .01**2])
        z, R = observation_to_map((1., 2., math.pi/2), cov, (1., 0.), .0)
        np.testing.assert_allclose(z, [1., 3.], atol=1e-12)
        self.assertAlmostEqual(R[0, 0], 1e-4)                         # yaw error moves the point sideways
        self.assertAlmostEqual(R[1, 1], 0., places=12)


class ViewModelTests(unittest.TestCase):
    def setUp(self):
        from harness.owncam_memory import ViewModel
        self.static = tagged('zone_wide_door_tags_v1')
        self.view = ViewModel(self.static, params())

    def test_predicted_tags_match_a_synthetic_render(self):
        from tests.test_owncam_localizer import synthetic_detections
        rng = np.random.default_rng(0)
        for pose, servo in (((1.2, .05, 0.), SEARCH), ((-.5, -.85, 0.), SEARCH), ((2.6, .05, 0.), {**SEARCH, 6: 1770})):
            seen = {int(d['id']) for d in synthetic_detections(self.static, pose, servo, rng) if d['side_px'] >= 14}
            # the synthetic generator has no black fisheye border: keep tags inside the pinhole render
            # (sim.masterpi_camera_profile.raw_fisheye_remap), as the simulated frames have content there
            index = {int(t): i for i, t in enumerate(self.view.tag_ids)}
            for tid in list(seen):
                corners = self.view.to_camera(self.view.tag_corners[index[tid]], np.asarray(pose), servo, False)[0]
                _, ideal, _ = self.view.project(corners)
                if np.any(ideal < 0) or np.any(ideal[:, 0] > 639) or np.any(ideal[:, 1] > 479):
                    seen.discard(tid)
            pred = {v['id'] for v in self.view.visible_tags(pose, servo, False)}
            with self.subTest(pose=pose):
                self.assertTrue(seen, 'synthetic view has tags')
                self.assertLessEqual(len(seen - pred), max(1, len(seen)//5))   # large synthetic tags are predicted

    def test_walls_occlude_and_back_faces_are_invisible(self):
        cam = np.array([1.0, -1.0, .21])
        # the floor just behind the 0.10 m divider (x = 2.2) is hidden; a 0.05 m tag 0.8 m beyond it
        # is seen over the wall from a 0.21 m camera; the floor in front of the wall is seen
        pts = np.array([[2.4, -1.0, 0.], [3.0, -1.0, .05], [2.0, -1.0, 0.]])
        self.assertEqual(self.view.occluded(cam, pts).tolist(), [True, False, False])
        west = self.view.visible_tags((0., -.85, math.pi), SEARCH, False)
        east = self.view.visible_tags((0., -.85, 0.), SEARCH, False)
        self.assertTrue(west and east)
        self.assertTrue(all(self.static['landmarks']['tags'][v['index']]['normal_xy'][0] > 0 for v in west))

    def test_held_box_band_limits_loaded_views(self):
        free = self.view.visible_tags((1.4, .05, 0.), CARRY, False)
        held = self.view.visible_tags((1.4, .05, 0.), CARRY, True)
        self.assertTrue(all(max(v['px'][1] for v in [h]) <= 172 for h in held))
        self.assertLessEqual(len(held), len(free))

    def test_fisher_information_is_informative_for_visible_tags(self):
        vis = self.view.visible_tags((-.5, -.85, 0.), SEARCH, False)
        info = self.view.fisher((-.5, -.85, 0.), SEARCH, False, vis)
        self.assertEqual(info.shape, (3, 3))
        self.assertTrue(np.all(np.linalg.eigvalsh(info) > -1e-6))
        self.assertGreater(info[2, 2], 1e3)                           # bearings pin yaw
        np.testing.assert_allclose(self.view.fisher((0., 0., 0.), SEARCH, False, []), 0.)


class _Det:
    """Injected own-RGB detector: returns the listed base-frame detections."""

    def __init__(self):
        self.rows = []

    def __call__(self, image, servo):
        return [{'kind': k, 'range_class': rc, 'estimated_box_center_base_m': [bx, by, .016]}
                for k, rc, bx, by in self.rows]


class MemoryTests(unittest.TestCase):
    def setUp(self):
        from harness.owncam_memory import OwnCamMemory
        self.det = _Det()
        self.events = []
        self.mem = OwnCamMemory(tagged('zone_wide_door_tags_v1'), params(), robot_id='r1', detect=self.det,
                                on_event=lambda k, row: self.events.append(k))

    def feed(self, t, rep, servo=SEARCH, settled=1., loaded=False, tags=()):
        return self.mem.observe_frame(t, frame_id=int(t*10), image=None, servo=servo, tag_detections=[
            {'id': i} for i in tags], report=rep, arm_settled_s=settled, loaded=loaded)

    def test_box_track_confirms_ages_and_goes_absent(self):
        rep = report(-.47, -.85, 0.)
        self.det.rows = [('cyan', 'near', .87, 0.)]
        self.feed(1., rep)
        tr = self.mem.tracks[0]
        self.assertEqual((tr.state, tr.kind), ('tentative', 'cyan'))
        self.feed(1.2, rep)
        self.assertEqual(tr.state, 'confirmed')
        self.assertIn('track_confirmed', self.events)
        np.testing.assert_allclose(tr.x, [.40, -.85], atol=.02)
        self.assertIs(self.mem.best_target('cyan', 2.), tr)
        self.assertEqual(self.mem.reverify(tr.track_id, 2.)['status'], 'fresh')
        self.assertEqual(self.mem.reverify(tr.track_id, 200.)['status'], 'stale')      # remembered, but old
        self.assertIsNone(self.mem.best_target('cyan', 200.))
        self.det.rows = []
        for k in range(3):                                          # in view, not detected: absent
            self.feed(3. + .2*k, rep)
        self.assertEqual(tr.state, 'absent')
        self.assertEqual(self.mem.reverify(tr.track_id, 4.)['status'], 'absent')

    def test_unsettled_loaded_or_unknown_posture_frames_do_not_touch_boxes(self):
        rep = report(-.47, -.85, 0.)
        self.det.rows = [('cyan', 'near', .87, 0.)]
        self.feed(1., rep, settled=.1)
        self.feed(1.2, rep, loaded=True, servo=CARRY)
        self.feed(1.4, rep, servo={1: 2000, 3: 900, 4: 2000, 5: 1500, 6: 1500})
        self.assertEqual(self.mem.tracks, [])

    def test_free_floor_is_remembered_and_decays(self):
        rep = report(-.47, -.85, 0.)
        self.feed(1., rep)
        cov = self.mem.view_coverage((-.47, -.85, 0.), SEARCH)
        self.assertGreater(cov['near_cells'], 50)
        before = cov['near_unknown_frac']
        self.feed(1.2, rep)
        self.assertLess(self.mem.view_coverage((-.47, -.85, 0.), SEARCH)['near_unknown_frac'], before)
        self.mem._decay_to(1.2 + 600.)                             # long unobserved: back to unknown
        self.assertGreater(self.mem.view_coverage((-.47, -.85, 0.), SEARCH)['near_unknown_frac'], .9)

    def test_search_pans_skip_known_floor(self):
        rep = report(-.47, -.85, 0.)
        for k in range(4):
            self.feed(1. + .2*k, rep, servo={**SEARCH, 6: 1500})
        plan = self.mem.plan_search_pans((-.47, -.85, 0.), (1500, 1230, 970, 1770, 2030))
        self.assertNotIn(1500, plan['pans'])
        self.assertIn(970, plan['pans'])

    def test_keepouts_and_slot_occupancy_use_confirmed_tracks(self):
        rep = report(-.47, -.85, 0.)
        self.det.rows = [('red', 'near', .87, 0.), ('cyan', 'far_coarse', 1.9, .3)]
        self.feed(1., rep)
        self.feed(1.2, rep)
        keep = self.mem.keepouts()
        self.assertEqual(len(keep), 1)                               # the tentative far cyan is not a keep-out
        self.assertGreaterEqual(keep[0]['half_extents_m'][0], .03)
        red = next(t for t in self.mem.tracks if t.kind == 'red')
        self.assertEqual(self.mem.slot_state(2., red.x, (.06, .06))['state'], 'occupied')
        self.assertEqual(self.mem.slot_state(2., (4.6, 0.), (.06, .06))['state'], 'unknown')
        far = self.mem.best_far('cyan', 2.)
        self.assertIsNotNone(far)
        self.assertEqual(far.state, 'tentative')

    def test_missing_expected_view_needs_three_settled_frames(self):
        rep = report(1.25, -.55, 0., load='loaded')
        strict = self.mem.view.visible_tags((1.25, -.55, 0.), CARRY, True, strict=True, sigma=(.03, .01))
        self.assertTrue(strict, 'the carry view ~1 m before the divider expects its tags')
        for k in range(2):
            self.feed(1. + .2*k, rep, servo=CARRY, loaded=True)
        self.assertFalse(self.mem.view_missing())
        self.feed(1.4, rep, servo=CARRY, loaded=True)
        self.assertTrue(self.mem.view_missing())
        self.feed(1.6, rep, servo=CARRY, loaded=True, tags=[strict[0]['id']])
        self.assertFalse(self.mem.view_missing())

    def test_look_plan_is_short_with_tags_and_full_without(self):
        est = {'initialized': True, 'x': 1.4, 'y': .05, 'yaw': 0., 'cov': np.diag([.001, .001, .003]).tolist()}
        plan = self.mem.plan_look(est, loaded=True, now=1., reason='uncertain')
        self.assertEqual(plan['mode'], 'short')
        self.assertTrue(1 <= len(plan['pans']) <= 3)
        self.assertLess(plan['predicted_std_yaw_rad'], plan['prior_std_yaw_rad'])
        blind = dict(tagged('zone_wide_door_tags_v1'))
        blind['landmarks'] = {**blind['landmarks'], 'tags': blind['landmarks']['tags'][:1]}
        from harness.owncam_memory import OwnCamMemory
        empty = OwnCamMemory(blind, params(), robot_id='r1', detect=_Det())
        self.assertEqual(empty.plan_look(est, loaded=True, now=1., reason='uncertain')['mode'], 'full')

    def test_remembered_pan_failures_lower_the_detection_prior(self):
        pose = (1.4, .05, 0.)
        vis = self.mem.view.visible_tags(pose, {**CARRY, 3: 1072, 4: 2400, 5: 1482, 6: 2030}, True)
        p0 = self.mem._pan_detect_prob(pose, 'look', True, 2030, vis)
        self.mem.pan_stats[self.mem._stat_key(pose, 'look', True, 2030)] = [10, 0]
        self.assertLess(self.mem._pan_detect_prob(pose, 'look', True, 2030, vis), p0/3)

    def test_snapshot_is_json(self):
        rep = report(-.47, -.85, 0.)
        self.det.rows = [('cyan', 'near', .87, 0.)]
        self.feed(1., rep, tags=[3])
        snap = self.mem.snapshot(1.)
        json.dumps(snap)
        json.dumps(self.mem.grid_record())
        self.assertEqual(snap['schema'], 'ugrp.owncam_memory.v1')
        self.assertEqual(snap['kf_sources']['filterpy']['version'], '1.4.5')


class DriverPolicyTests(unittest.TestCase):
    def _leg(self, loaded):
        from harness.m1_owncam_memory import _LegDriverMem
        from harness.owncam_localizer import OwnCamLocalizer
        from harness.owncam_memory import OwnCamMemory
        static = tagged('zone_wide_door_tags_v1')
        mem = OwnCamMemory(static, params(), robot_id='r1', detect=_Det())
        loc = OwnCamLocalizer(static, params(), seed=0)
        leg = _LegDriverMem(mem, loc, static, params(), loaded=loaded, goal_xy=(2.65, .05), door_xy=(2.2, .05),
                            initial_servo=CARRY if loaded else SEARCH)
        return leg, mem

    @staticmethod
    def _est(x, sxy, syaw):
        return {'initialized': True, 'x': x, 'y': .05, 'yaw': 0., 'std_xy_m': sxy, 'std_yaw_rad': syaw,
                'since_tag_s': 9., 'cov': np.diag([sxy**2/2, sxy**2/2, syaw**2]).tolist()}

    def test_no_travel_look_and_confident_door_checkpoints_are_skipped(self):
        leg, mem = self._leg(True)
        self.assertIsNone(leg._needs_look(self._est(0., .03, .01), 0.))
        self.assertIsNone(leg._needs_look(self._est(.9, .03, .01), 1.))       # 0.9 m travelled: no look
        self.assertIsNone(leg._needs_look(self._est(1.0, .03, .01), 2.))      # door 1.5 m checkpoint skipped
        self.assertEqual(leg.look_counts['skipped'], 1)
        self.assertEqual(leg._needs_look(self._est(1.7, .045, .01), 3.), 'door_checkpoint_0.6')
        self.assertEqual(leg._needs_look(self._est(1.8, .08, .01), 4.), 'uncertain')   # same v2 thresholds

    def test_missing_expected_view_forces_a_full_look(self):
        leg, mem = self._leg(True)
        mem.missing_run = 3
        self.assertEqual(leg._needs_look(self._est(.5, .03, .01), 0.), 'expected_view_missing')
        leg._start_look(0., 'expected_view_missing')
        from harness.owncam_drive import WIDE_LOOK_PANS
        self.assertEqual(leg.look_mode, 'full')
        self.assertEqual(leg.look_queue, list(WIDE_LOOK_PANS))
        self.assertFalse(mem.view_missing())                                    # reset when the look starts

    def test_short_look_stops_early_and_escalates_when_still_uncertain(self):
        leg, mem = self._leg(True)
        leg.loc.estimate = lambda: {**self._est(1.4, .03, .06), 't': 0.}
        leg.loc.predict_to = lambda t: None
        with mock.patch.object(mem, 'plan_look', return_value={'pans': [2030, 970], 'mode': 'short'}):
            leg._start_look(0., 'uncertain')
        self.assertEqual((leg.look_mode, leg.look_queue), ('short', [2030, 970]))
        self.assertTrue(leg._should_refix(True))                                # yaw 3.4 deg: escalate
        self.assertEqual(leg.look_counts['escalated'], 1)
        leg.loc.estimate = lambda: {**self._est(1.4, .03, .01), 't': 0.}
        self.assertFalse(leg._should_refix(True))
        leg.state, leg.state_since, leg.look_queue = 'look_pan', 0., [970]
        leg.arm_target = {6: 2030}
        leg.servo[6] = 2030
        leg.tick(1.)
        self.assertEqual(leg.look_counts['early_stop'], 1)

    def test_confident_arrival_check_is_skipped(self):
        leg, mem = self._leg(False)
        leg.loc.estimate = lambda: {**self._est(2.65, .02, .01), 't': 0.}
        leg.loc.predict_to = lambda t: None
        leg.state = 'drive'
        self.assertEqual(leg._start_look(0., 'arrival_check'), [{'kind': 'hold'}])
        self.assertEqual(leg.state, 'drive')
        self.assertEqual(leg.look_counts['skipped'], 1)


class ControllerTests(unittest.TestCase):
    def _ctl(self):
        from harness.m1_owncam_memory import M1OwnCamDeliveryMem
        from harness.wrist_zone_skill import PoseEstimate
        from harness.wrist_zone_skill_v6 import StaticKeepout
        from harness.wrist_zone_skill_v9 import WristZoneDeliveryV9
        static = tagged('zone_wide_door_tags_v1')
        keep = tuple(StaticKeepout(f'spawn_row_{i}', (-.85, y), .17, 'static_layout_idle_spawn')
                     for i, y in enumerate((-2.25, -.85, .55)))
        ctl = M1OwnCamDeliveryMem(static, params(), box_kind='cyan', slot_id='A1', slot_xy=(4.6, 0.),
                                  skill_factory=lambda o: WristZoneDeliveryV9(o, mode='m1', static_keepouts=keep,
                                                                              static_bounds_m=static['bounds_m']),
                                  pose_estimate_cls=PoseEstimate, search_rows_y=(-2.45, -1.65, -.85, -.05, .75))
        ctl.on_command({'t': 0., 'kind': 'initial_servo_command', 'pulses': {str(k): v for k, v in SEARCH.items()}})
        ctl.memory._detect = _Det()
        return ctl

    def test_without_frames_the_memory_controller_fails_like_the_baseline(self):
        from harness.m1_owncam_delivery import MAX_GATE_LOOKS
        ctl = self._ctl()
        t = 0.
        for _ in range(2000):
            d = ctl.decide(t)
            if d['mode'] == 'done':
                break
            for c in d.get('commands', []):
                ctl.on_command({'t': t, **c})
            t += .1
        self.assertEqual(ctl.outcome, 'NOT_INITIALIZED')
        self.assertEqual(ctl.init_looks, MAX_GATE_LOOKS)
        json.dumps(ctl.summary(), default=str)

    def test_gate_looks_short_first_then_full_and_manipulation_full(self):
        from harness.m1_owncam_delivery import WIDE_LOOK_PANS
        from harness.owncam_pose_source import PoseReport
        ctl = self._ctl()
        ob = {'actuator_state': {'servo_pulses': {str(k): v for k, v in SEARCH.items()}}}
        good = report(4.3, 0., 0., sxy=.06, syaw=.05)
        ctl.pose.report = lambda now: PoseReport(**{**good.__dict__, 't_est': now})
        ctl.pose.loc.estimate = lambda: {'initialized': True, 'x': 4.3, 'y': 0., 'yaw': 0.,
                                         'cov': np.diag([.0018, .0018, .0025]).tolist()}
        ctl.gate_looks = 1
        ctl._gate_look(1., 'gate:release:std_xy', ob, loaded=False)
        self.assertEqual(ctl.sweep['mode'], 'short')
        self.assertLess(len(ctl.sweep['queue']), len(WIDE_LOOK_PANS))
        ctl.sweep, ctl.gate_looks = None, 2
        ctl._gate_look(2., 'gate:release:std_xy', ob, loaded=False)
        self.assertEqual((ctl.sweep['mode'], ctl.sweep['queue']), ('full', list(WIDE_LOOK_PANS)))
        ctl.sweep = None
        ctl._gate_look(3., 'post_manipulation', ob, loaded=False)
        self.assertEqual(ctl.sweep['mode'], 'full')

    def test_preplace_look_is_skipped_when_the_release_gate_holds(self):
        from harness.owncam_pose_source import PoseReport
        ctl = self._ctl()
        ob = {'actuator_state': {'servo_pulses': {str(k): v for k, v in CARRY.items()}}}
        good = report(4.3, 0., 0., sxy=.02, syaw=.01, load='loaded')
        ctl.pose.report = lambda now: PoseReport(**{**good.__dict__, 't_est': now})
        ctl.last_look_t = 9.
        self.assertEqual(ctl._gate_look(10., 'preplace', ob, loaded=True)['mode'], 'tick')
        self.assertIsNone(ctl.sweep)
        self.assertEqual(ctl.gate_modes[-1]['mode'], 'skipped')

    def test_initial_look_can_already_find_the_target(self):
        ctl = self._ctl()
        rep = report(-.85, -.85, 0.)
        ctl.pose.report = lambda now: rep
        ctl.memory._detect.rows = [('cyan', 'near', .65, 0.)]
        for k in range(2):
            ctl.memory.observe_frame(1. + .2*k, frame_id=k + 1, image=None, servo=SEARCH, tag_detections=[],
                                     report=rep, arm_settled_s=1., loaded=False)
        d = ctl._init(2.)
        self.assertEqual(d['mode'], 'tick')
        self.assertEqual(ctl.phase, 'approach_leg')
        self.assertEqual(ctl.pickup_source, 'own_rgb_search')
        self.assertAlmostEqual(ctl.target_xy[0], -.2, delta=.03)

    def test_viewpoints_with_known_floor_are_skipped_then_revisited(self):
        ctl = self._ctl()
        ctl.viewpoints = [(-.47, -.85), (-.47, -.85)]
        ctl.view_index = 0
        ctl.memory.log_odds[:] = -2.
        ctl.memory.far_seen[:] = 10
        ctl.phase = 'search_sweep'
        d = ctl._search_decide(5.)
        self.assertEqual(d['mode'], 'tick')
        self.assertTrue(ctl.revisit)
        self.assertEqual(ctl.phase, 'search_leg')


class RunnerTests(unittest.TestCase):
    def test_conditions_select_the_controller_and_guards_refuse(self):
        import scripts.run_m1_owncam_memory as R
        from harness.m1_owncam_delivery import M1OwnCamDelivery
        from harness.m1_owncam_memory import M1OwnCamDeliveryMem
        self.assertIs(R.controller_class('off'), M1OwnCamDelivery)
        self.assertIs(R.controller_class('memory_v1'), M1OwnCamDeliveryMem)
        with self.assertRaises(SystemExit):
            R.controller_class('memory_v0')
        with self.assertRaises(SystemExit):
            R.check_threads({'OMP_NUM_THREADS': '1'})
        self.assertEqual(set(R.check_threads({k: '1' for k in R.THREAD_VARS})), set(R.THREAD_VARS))

    def test_runner_patches_the_class_only_for_the_episode(self):
        import harness.m1_owncam_delivery as base
        import scripts.run_m1_owncam as runner
        import scripts.run_m1_owncam_memory as R
        seen = []

        def fake_run(spec, out, student):
            seen.append((base.M1OwnCamDelivery.__name__, student['condition']))
            out.mkdir(parents=True)
            for n in ('result.json', 'manifest.json'):
                (out/n).write_text('{}')
            return {'controller': {'schema': 'x'}}, {}
        original = base.M1OwnCamDelivery
        with mock.patch.object(runner, 'run', side_effect=fake_run), \
             mock.patch.dict('os.environ', {k: '1' for k in R.THREAD_VARS}), \
             mock.patch.object(R, 'free_gib', return_value=100.):
            import tempfile
            with tempfile.TemporaryDirectory() as tmp:
                _, _, rec = R.run_episode({'episode_id': 'e'}, Path(tmp)/'on'/'e', {}, 'memory_v1', prereg_sha256='0')
                self.assertEqual(rec['controller_class'], 'harness.m1_owncam_memory.M1OwnCamDeliveryMem')
                R.run_episode({'episode_id': 'e'}, Path(tmp)/'off'/'e', {}, 'off', prereg_sha256='0')
        self.assertEqual(seen, [('M1OwnCamDeliveryMem', 'memory_v1'), ('M1OwnCamDelivery', 'off')])
        self.assertIs(base.M1OwnCamDelivery, original)
        with mock.patch.object(R, 'free_gib', return_value=5.), \
             mock.patch.dict('os.environ', {k: '1' for k in R.THREAD_VARS}), self.assertRaises(SystemExit):
            R.run_episode({'episode_id': 'e'}, Path('/nonexistent-never')/'x', {}, 'off', prereg_sha256='0')

    def test_test_split_needs_a_frozen_source(self):
        import scripts.run_m1_owncam_memory as R
        prereg = ROOT/'experiments'/'2026-09-26-zone-owncam-memory'/'prereg.json'
        with self.assertRaises(SystemExit):
            R.main(['--prereg', str(prereg), '--condition', 'off', '--output', '/x', '--split', 'test'])
        with self.assertRaises(SystemExit):
            R.main(['--prereg', str(prereg), '--condition', 'off', '--output', '/x', '--only', 'm1mem-s161'])


if __name__ == '__main__':
    unittest.main()
