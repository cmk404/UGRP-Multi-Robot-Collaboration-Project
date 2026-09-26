"""Own-camera observation memory v2 ("look once, remember", landmark-agnostic): boundary, catalogue,
interim tag provider, reuse adapters, memory and policy."""
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
           ROOT/'harness'/'owncam_drive_mem.py', ROOT/'harness'/'m1_owncam_memory.py',
           ROOT/'harness'/'owncam_landmarks.py', ROOT/'harness'/'owncam_landmark_tags.py')
ALLOWED = {'__future__', 'math', 'copy', 'collections.abc', 'numpy', 'cv2', 'harness.owncam_drive',
           'harness.owncam_memory_kf', 'harness.visual_arm', 'sim.masterpi_camera_profile', 'harness.zone_color_boxes',
           'harness.owncam_memory', 'harness.owncam_drive_mem', 'harness.m1_owncam_delivery',
           'harness.owncam_pose_source', 'harness.owncam_landmarks', 'harness.owncam_landmark_tags', 'hashlib', 'json',
           'dataclasses', 'harness.wall_tags'}
# Only the INTERIM tag provider and the M1 wiring may know about tags (user decision 2026-09-26).
TAG_FREE = (ROOT/'harness'/'owncam_memory.py', ROOT/'harness'/'owncam_landmarks.py',
            ROOT/'harness'/'owncam_drive_mem.py', ROOT/'harness'/'owncam_memory_kf.py')
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

    def test_memory_core_is_tag_free(self):
        for path in TAG_FREE:
            tree = ast.parse(path.read_text())
            mods = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
            names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)} | \
                {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
            keys = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)}
            with self.subTest(path=path.name):
                self.assertNotIn('harness.wall_tags', mods)
                self.assertNotIn('harness.owncam_landmark_tags', mods)
                self.assertFalse({'TagDetector', 'tag_world_frame', 'visible_tags', 'tag_ids'} & names)
                self.assertNotIn('tags', keys)

    def test_memory_modules_import_with_mujoco_poisoned(self):
        code = ("import sys; sys.modules['mujoco'] = None\n"
                "import harness.owncam_memory, harness.owncam_memory_kf, harness.owncam_drive_mem, harness.m1_owncam_memory\n"
                "import harness.owncam_landmarks, harness.owncam_landmark_tags\n"
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


def tag_det(tag_id, t_ct=(0., 0., 1.)):
    """A pose-source tag detection row (only the fields the interim provider reads)."""
    return {'id': int(tag_id), 'solutions': [{'t_ct': list(t_ct), 'R_ct': np.eye(3).tolist(), 'reproj_px': .1}]}


class CatalogueTests(unittest.TestCase):
    def test_v1_catalogue_is_derived_from_walls_and_doors(self):
        from harness.owncam_landmarks import LandmarkCatalogue
        cat = LandmarkCatalogue(tagged('zone_wide_door_tags_v1'))
        d = cat.describe()
        self.assertEqual(d['counts'], {'wall_corner': 8, 'wall_end': 0, 'door_post': 4, 'door_gap': 1, 'wall_face': 10})
        posts = sorted(lm.xy for lm in cat.landmarks if lm.type == 'door_post')
        self.assertEqual(posts, [(2.175, -0.2), (2.175, 0.3), (2.225, -0.2), (2.225, 0.3)])
        self.assertIn('door_1:post_lo:-x', cat.by_id)
        corner = cat.by_id['corner:wall_north+wall_west:+x-y']
        self.assertEqual((corner.xy, corner.free_quadrants), ((-1.025, 1.425), ((1, -1),)))
        self.assertEqual(cat.by_id['door_1:gap'].xy, (2.2, .05))
        json.dumps([lm.as_dict() for lm in cat.landmarks])

    def test_tags_are_never_read_and_never_required(self):
        from harness.owncam_landmarks import LandmarkCatalogue
        full = tagged('zone_wide_door_tags_v1')
        bare = json.loads(json.dumps(full))
        bare['landmarks'] = {'tags': []}
        self.assertEqual(LandmarkCatalogue(full).sha256, LandmarkCatalogue(bare).sha256)
        no_landmarks = {k: v for k, v in full.items() if k != 'landmarks'}
        self.assertEqual(LandmarkCatalogue(no_landmarks).sha256, LandmarkCatalogue(full).sha256)

    def test_door_posts_of_v2_and_two_doors(self):
        from harness.owncam_landmarks import LandmarkCatalogue
        v2 = LandmarkCatalogue(tagged('zone_wide_door_tags_v2'))
        self.assertEqual(v2.describe()['counts']['door_post'], 4)
        two = LandmarkCatalogue(tagged('zone_wide_two_doors_tags_v1'))
        self.assertEqual(two.describe()['counts']['door_gap'], 2)
        self.assertTrue(any(i.startswith('door_wide:post_hi') for i in two.by_id))

    def test_seen_from_respects_the_free_side(self):
        from harness.owncam_landmarks import LandmarkCatalogue
        cat = LandmarkCatalogue(tagged('zone_wide_door_tags_v1'))
        plan = cat.plan_points()
        i = plan['ids'].index('corner:wall_north+wall_west:+x-y')
        self.assertTrue(cat.seen_from(np.array([i]), (0., 0.))[0])
        self.assertFalse(cat.seen_from(np.array([i]), (-2., 0.))[0])
        f = plan['ids'].index('face:wall_divider_1:-x:0')
        self.assertTrue(cat.seen_from(np.array([f]), (1., -1.))[0])
        self.assertFalse(cat.seen_from(np.array([f]), (3., -1.))[0])


class ViewModelTests(unittest.TestCase):
    def setUp(self):
        from harness.owncam_landmark_tags import TagLandmarkProvider
        from harness.owncam_memory import OwnCamMemory
        self.static = tagged('zone_wide_door_tags_v1')
        self.mem = OwnCamMemory(self.static, params(), robot_id='r1', provider=TagLandmarkProvider(self.static, params()),
                                detect=lambda image, servo: [])
        self.view, self.cat, self.prov = self.mem.view, self.mem.catalogue, self.mem.provider

    def test_predicted_tags_match_a_synthetic_render(self):
        from tests.test_owncam_localizer import synthetic_detections
        rng = np.random.default_rng(0)
        for pose, servo in (((1.2, .05, 0.), SEARCH), ((-.5, -.85, 0.), SEARCH), ((2.6, .05, 0.), {**SEARCH, 6: 1770})):
            seen = {int(d['id']) for d in synthetic_detections(self.static, pose, servo, rng) if d['side_px'] >= 14}
            # the synthetic generator has no black fisheye border: keep tags inside the pinhole render
            # (sim.masterpi_camera_profile.raw_fisheye_remap), as the simulated frames have content there
            index = {int(t): i for i, t in enumerate(self.prov.tag_ids)}
            for tid in list(seen):
                corners = self.view.to_camera(self.prov.tag_corners[index[tid]], np.asarray(pose), servo, False)[0]
                _, ideal, _ = self.view.project(corners)
                if np.any(ideal < 0) or np.any(ideal[:, 0] > 639) or np.any(ideal[:, 1] > 479):
                    seen.discard(tid)
            pred = {v['id'] for v in self.prov.visible_tags(pose, servo, False)}
            with self.subTest(pose=pose):
                self.assertTrue(seen, 'synthetic view has tags')
                self.assertLessEqual(len(seen - pred), max(1, len(seen)//5))   # large synthetic tags are predicted

    def test_walls_occlude_and_back_faces_are_invisible(self):
        cam = np.array([1.0, -1.0, .21])
        # the floor just behind the 0.10 m divider (x = 2.2) is hidden; a 0.05 m point 0.8 m beyond it
        # is seen over the wall from a 0.21 m camera; the floor in front of the wall is seen
        pts = np.array([[2.4, -1.0, 0.], [3.0, -1.0, .05], [2.0, -1.0, 0.]])
        self.assertEqual(self.view.occluded(cam, pts).tolist(), [True, False, False])
        west = self.view.visible_landmarks(self.cat, (0., -.85, math.pi), SEARCH, False)
        east = self.view.visible_landmarks(self.cat, (0., -.85, 0.), SEARCH, False)
        self.assertTrue(west and east)
        self.assertTrue(all(r['id'].startswith(('face:wall_west', 'corner:wall_north+wall_west',
                                                'corner:wall_south+wall_west')) for r in west))
        self.assertTrue(any(r['id'].startswith('door_1:post') or r['id'].startswith('face:wall_divider') for r in east))
        self.assertFalse(any(':+x' in r['id'] and 'divider' in r['id'] for r in east))   # far faces of the divider

    def test_held_box_band_limits_loaded_views(self):
        free = self.view.visible_landmarks(self.cat, (1.4, .05, 0.), CARRY, False)
        held = self.view.visible_landmarks(self.cat, (1.4, .05, 0.), CARRY, True)
        self.assertTrue(all(r['px'][1] <= 168 for r in held))
        self.assertLessEqual(len(held), len(free))

    def test_fisher_information_is_informative_for_visible_landmarks(self):
        pose = (-.5, -.85, 0.)
        rows = self.view.visible_landmarks(self.cat, pose, SEARCH, False)
        sup = self.prov.support(rows, pose, SEARCH, False)
        info = self.view.landmark_fisher(pose, SEARCH, False, rows, sup['w'], self.prov.noise(False),
                                         identified_face_points=True)
        self.assertEqual(info.shape, (3, 3))
        self.assertTrue(np.all(np.linalg.eigvalsh(info) > -1e-6))
        self.assertGreater(info[2, 2], 1e3)                           # bearings pin yaw
        np.testing.assert_allclose(self.view.landmark_fisher(pose, SEARCH, False, [], np.zeros(0),
                                                             self.prov.noise(False), identified_face_points=True), 0.)
        # a wall-face line without identified points carries no information along the face
        face = [r for r in rows if r['type'] == 'wall_face']
        line = self.view.landmark_fisher(pose, SEARCH, False, face, np.ones(len(face)),
                                         {'azimuth_std_rad': .01, 'elevation_std_rad': .02},
                                         identified_face_points=False)
        u = np.array([face[0]['face_dir'][0], face[0]['face_dir'][1], 0.])
        self.assertLess(float(u @ line @ u), 1e-6*max(1., float(np.trace(line))))

    def test_box_detection_is_reprojected_with_the_pf_camera_model(self):
        from harness.owncam_memory import BOX_CENTRE_Z_M, camera_in_base, correct_box_detection
        p = params()
        o, r = camera_in_base(SEARCH)
        truth = np.array([1.1, -.5, BOX_CENTRE_Z_M])
        seen = self.view._correct((truth - o) @ r, False)          # how the real camera sees the true point
        ray = r @ seen                                              # read with the nominal camera (detect_own)
        nominal = o + (BOX_CENTRE_Z_M - o[2])/ray[2]*ray
        self.assertGreater(math.dist(nominal[:2], truth[:2]), .05)     # the uncorrected range bias
        fixed = correct_box_detection(nominal[:2], SEARCH, p, loaded=False)
        np.testing.assert_allclose(fixed, truth[:2], atol=1e-6)
        seen_l = self.view._correct((truth - o) @ r, True)
        ray_l = r @ seen_l
        nominal_l = o + (BOX_CENTRE_Z_M - o[2])/ray_l[2]*ray_l
        np.testing.assert_allclose(correct_box_detection(nominal_l[:2], SEARCH, p, loaded=True), truth[:2], atol=2e-4)


class TagProviderTests(unittest.TestCase):
    def setUp(self):
        from harness.owncam_landmark_tags import TagLandmarkProvider
        from harness.owncam_memory import OwnCamMemory
        self.static = tagged('zone_wide_door_tags_v1')
        self.mem = OwnCamMemory(self.static, params(), robot_id='r1', provider=TagLandmarkProvider(self.static, params()),
                                detect=lambda image, servo: [])

    def test_every_tag_is_anchored_to_a_static_map_landmark(self):
        prov = self.mem.provider
        self.assertTrue(prov.interim)
        self.assertEqual(prov.describe()['label'], 'interim, tag provider')
        for (lid, ltype), sup in zip(prov.anchor, prov.supports):
            self.assertIn(lid, self.mem.catalogue.by_id)
            self.assertIn(ltype, ('wall_corner', 'door_post', 'wall_face'))
            self.assertIn(lid, sup)

    def test_observations_are_generic_landmark_records(self):
        prov = self.mem.provider
        tid = int(prov.tag_ids[0])
        out = self.mem.observe_frame(1., frame_id=10, image=None, servo=SEARCH, report=report(0., -.85, 0.),
                                     arm_settled_s=1., loaded=False, provider_inputs={'tag_detections': [tag_det(tid)]})
        self.assertEqual(len(self.mem.observations), 1)
        o = self.mem.observations[0]
        self.assertEqual(o['feature_id'], f'tag:{tid}')
        self.assertIn(o['landmark_id'], self.mem.catalogue.by_id)
        self.assertEqual((o['provider'], o['interim'], o['frame_id'], o['posture']), ('tags_interim', True, 10, 'search'))
        self.assertEqual(o['pose_xyyaw'], [0., -.85, 0.])
        self.assertAlmostEqual(o['range_m'], 1.)
        self.assertEqual(out['observed'], [o['landmark_id']])
        self.assertEqual(self.mem.last_fix['landmarks'], [o['landmark_id']])
        self.assertIsNone(self.mem.last_look_fix)                  # search posture: not a look fix

    def test_support_follows_the_tags_next_to_each_landmark(self):
        pose = (0., -.85, 0.)
        rows = self.mem.view.visible_landmarks(self.mem.catalogue, pose, SEARCH, False)
        sup = self.mem.provider.support(rows, pose, SEARCH, False)
        self.assertEqual(len(sup['w']), len(rows))
        self.assertGreater(sup['p_any'], .9)
        self.assertTrue(all(f.startswith('tag:') for f in sup['features']))
        from harness.owncam_landmark_tags import TagLandmarkProvider
        bare = json.loads(json.dumps(self.static))
        bare['landmarks']['tags'] = []
        empty = TagLandmarkProvider(bare, params())
        empty.bind(self.mem.view, self.mem.catalogue)
        self.assertEqual(float(np.sum(empty.support(rows, pose, SEARCH, False)['w'])), 0.)


class _Det:
    """Injected own-RGB detector: returns the listed base-frame detections."""

    def __init__(self):
        self.rows = []

    def __call__(self, image, servo):
        return [{'kind': k, 'range_class': rc, 'estimated_box_center_base_m': [bx, by, .016]}
                for k, rc, bx, by in self.rows]


class MemoryTests(unittest.TestCase):
    def setUp(self):
        from harness.owncam_landmark_tags import TagLandmarkProvider
        from harness.owncam_memory import OwnCamMemory
        self.det = _Det()
        self.events = []
        static = tagged('zone_wide_door_tags_v1')
        self.mem = OwnCamMemory(static, params(), robot_id='r1', detect=self.det,
                                provider=TagLandmarkProvider(static, params()),
                                on_event=lambda k, row: self.events.append(k))

    def feed(self, t, rep, servo=SEARCH, settled=1., loaded=False, tags=()):
        return self.mem.observe_frame(t, frame_id=int(t*10), image=None, servo=servo, report=rep,
                                      arm_settled_s=settled, loaded=loaded,
                                      provider_inputs={'tag_detections': [tag_det(i) for i in tags]})

    def test_box_track_confirms_ages_and_goes_absent(self):
        from harness.owncam_memory import correct_box_detection
        rep = report(-.47, -.85, 0.)
        self.det.rows = [('cyan', 'near', .87, 0.)]
        self.feed(1., rep)
        tr = self.mem.tracks[0]
        self.assertEqual((tr.state, tr.kind), ('tentative', 'cyan'))
        self.feed(1.2, rep)
        self.assertEqual(tr.state, 'confirmed')
        self.assertIn('track_confirmed', self.events)
        cx, cy = correct_box_detection((.87, 0.), SEARCH, params())
        self.assertLess(cx, .87)                                    # the A2 re-projection shortens the range
        np.testing.assert_allclose(tr.x, [-.47 + cx, -.85 + cy], atol=.01)
        self.assertIs(self.mem.best_target('cyan', 2.), tr)
        self.assertEqual(self.mem.reverify(tr.track_id, 2.)['status'], 'fresh')
        self.assertEqual(self.mem.reverify(tr.track_id, 200.)['status'], 'stale')      # remembered, but old
        self.assertIsNone(self.mem.best_target('cyan', 200.))
        self.det.rows = []
        for k in range(3):                                          # in view, not detected: absent
            self.feed(3. + .2*k, rep)
        self.assertEqual(tr.state, 'absent')
        self.assertEqual(self.mem.reverify(tr.track_id, 4.)['status'], 'absent')

    def test_track_sigma_keeps_the_pose_floor(self):
        from harness.owncam_memory import TRACK_FLOOR_M
        rep = report(-.47, -.85, 0., sxy=.06, syaw=.001)
        self.det.rows = [('cyan', 'near', .6, 0.)]
        for k in range(40):
            self.feed(1. + .2*k, rep)
        tr = self.mem.tracks[0]
        self.assertGreaterEqual(tr.sigma_m(), max(TRACK_FLOOR_M, .06/math.sqrt(2)) - 1e-6)

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
        pose = (1.25, -.55, 0.)
        rep = report(*pose, load='loaded')
        strict = self.mem.view.visible_landmarks(self.mem.catalogue, pose, CARRY, True, strict=True, sigma=(.03, .01))
        sup = self.mem.provider.support(strict, pose, CARRY, True, strict=True, sigma=(.03, .01))
        expected = [r['id'] for r, w in zip(strict, sup['w']) if w >= .9]
        self.assertTrue(expected, 'the carry view ~1 m before the divider expects its landmarks')
        for k in range(2):
            self.feed(1. + .2*k, rep, servo=CARRY, loaded=True)
        self.assertFalse(self.mem.view_missing())
        self.feed(1.4, rep, servo=CARRY, loaded=True)
        self.assertTrue(self.mem.view_missing())
        prov = self.mem.provider
        tid = next(int(t) for t, sup_ids in zip(prov.tag_ids, prov.supports) if set(sup_ids) & set(expected))
        self.feed(1.6, rep, servo=CARRY, loaded=True, tags=[tid])
        self.assertFalse(self.mem.view_missing())

    def test_look_plan_is_short_with_landmarks_and_full_without_a_detector(self):
        from harness.owncam_memory import OwnCamMemory
        est = {'initialized': True, 'x': 1.4, 'y': .05, 'yaw': 0., 'cov': np.diag([.001, .001, .003]).tolist()}
        plan = self.mem.plan_look(est, loaded=True, now=1., reason='uncertain')
        self.assertEqual(plan['mode'], 'short')
        self.assertTrue(1 <= len(plan['pans']) <= 3)
        self.assertLess(plan['predicted_std_yaw_rad'], plan['prior_std_yaw_rad'])
        # tag-free provider without a detector (vision pending): nothing supported -> full look
        tag_free = OwnCamMemory(tagged('zone_wide_door_tags_v1'), params(), robot_id='r1', detect=_Det())
        self.assertEqual(tag_free.provider.name, 'geometric')
        self.assertEqual(tag_free.plan_look(est, loaded=True, now=1., reason='uncertain')['mode'], 'full')
        # a geometric detector hook makes the same planner choose pans from door posts / corners
        from harness.owncam_landmarks import GeometricLandmarkProvider
        geo = OwnCamMemory(tagged('zone_wide_door_tags_v1'), params(), robot_id='r1', detect=_Det(),
                           provider=GeometricLandmarkProvider(detector=lambda *a: []))
        p2 = geo.plan_look({**est, 'x': .5, 'y': -.5}, loaded=False, now=1., reason='uncertain')
        self.assertEqual(p2['mode'], 'short')

    def test_the_memory_runs_on_a_map_without_tags(self):
        from harness.owncam_memory import OwnCamMemory
        bare = tagged('zone_wide_door_tags_v1')
        bare = {k: v for k, v in bare.items() if k != 'landmarks'}
        mem = OwnCamMemory(bare, params(), robot_id='r1', detect=self.det)
        self.det.rows = [('cyan', 'near', .87, 0.)]
        for k in range(2):
            mem.observe_frame(1. + .2*k, frame_id=k, image=None, servo=SEARCH, report=report(-.47, -.85, 0.),
                              arm_settled_s=1., loaded=False)
        self.assertEqual(mem.tracks[0].state, 'confirmed')
        json.dumps(mem.snapshot(2.))

    def test_remembered_pan_failures_lower_the_detection_prior(self):
        pose = (1.4, .05, 0.)
        p0 = self.mem._pan_detect_prob(pose, 'look', True, 2030, .9)
        self.mem.pan_stats[self.mem._stat_key(pose, 'look', True, 2030)] = [10, 0]
        self.assertLess(self.mem._pan_detect_prob(pose, 'look', True, 2030, .9), p0/3)

    def test_stale_look_fix(self):
        from harness.owncam_memory import LOOK_P20
        rep = report(-.47, -.85, 0.)
        self.assertFalse(self.mem.look_fix_fresh(1., (-.47, -.85)))
        tid = int(self.mem.provider.tag_ids[0])
        self.feed(1., rep, servo={**LOOK_P20, 1: 2000, 6: 1500}, tags=[tid])
        self.assertTrue(self.mem.look_fix_fresh(2., (-.47, -.85)))
        self.assertTrue(self.mem.look_fix_since(1.))
        self.assertFalse(self.mem.look_fix_since(1.5))
        self.assertFalse(self.mem.look_fix_fresh(2., (.2, -.85)))          # 0.67 m of own travel
        self.assertFalse(self.mem.look_fix_fresh(100., (-.47, -.85)))       # too old

    def test_snapshot_is_json(self):
        rep = report(-.47, -.85, 0.)
        self.det.rows = [('cyan', 'near', .87, 0.)]
        self.feed(1., rep, tags=[int(self.mem.provider.tag_ids[0])])
        snap = self.mem.snapshot(1.)
        json.dumps(snap)
        json.dumps(self.mem.grid_record())
        self.assertEqual(snap['schema'], 'ugrp.owncam_memory.v2')
        self.assertEqual(snap['provider']['label'], 'interim, tag provider')
        self.assertEqual(snap['catalogue']['counts']['door_post'], 4)
        self.assertEqual(snap['kf_sources']['filterpy']['version'], '1.4.5')


class DriverPolicyTests(unittest.TestCase):
    def _leg(self, loaded):
        from harness.m1_owncam_memory import _LegDriverMem
        from harness.owncam_landmark_tags import TagLandmarkProvider
        from harness.owncam_localizer import OwnCamLocalizer
        from harness.owncam_memory import OwnCamMemory
        static = tagged('zone_wide_door_tags_v1')
        mem = OwnCamMemory(static, params(), robot_id='r1', detect=_Det(), provider=TagLandmarkProvider(static, params()))
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
        self.assertEqual(leg.look_counts['early_stop'], 0)                      # A3: no look fix in this dwell yet
        mem.last_look_fix = {'t': .5, 'xy': [1.4, .05]}
        leg.state, leg.state_since, leg.look_queue = 'look_pan', 0., [970]
        leg.arm_target = {6: 2030}
        leg.tick(1.)
        self.assertEqual(leg.look_counts['early_stop'], 1)

    def test_confident_arrival_check_is_skipped_only_with_a_fresh_look_fix(self):
        leg, mem = self._leg(False)
        leg.loc.estimate = lambda: {**self._est(2.65, .02, .01), 't': 0.}
        leg.loc.predict_to = lambda t: None
        leg.state = 'drive'
        leg._start_look(0., 'arrival_check')                                   # A3: no look fix yet -> look
        self.assertEqual(leg.state, 'look_arm')
        self.assertEqual((leg.look_counts['skipped'], leg.look_counts['stale_fix']), (0, 1))
        leg, mem = self._leg(False)
        leg.loc.estimate = lambda: {**self._est(2.65, .02, .01), 't': 0.}
        leg.loc.predict_to = lambda t: None
        leg.state = 'drive'
        mem.last_look_fix = {'t': 0., 'xy': [2.4, .05]}
        self.assertEqual(leg._start_look(1., 'arrival_check'), [{'kind': 'hold'}])
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
            ctl.memory.observe_frame(1. + .2*k, frame_id=k + 1, image=None, servo=SEARCH, report=rep,
                                     arm_settled_s=1., loaded=False, provider_inputs={'tag_detections': []})
        d = ctl._init(2.)
        self.assertEqual(d['mode'], 'tick')
        self.assertEqual(ctl.phase, 'approach_leg')
        self.assertEqual(ctl.pickup_source, 'own_rgb_search')
        from harness.owncam_memory import correct_box_detection
        self.assertAlmostEqual(ctl.target_xy[0], -.85 + correct_box_detection((.65, 0.), SEARCH, params())[0], delta=.01)
        self.assertEqual(ctl.summary()['landmark_provider']['result_label'], 'interim, tag provider')

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
        self.assertIs(R.controller_class('memory_v2'), M1OwnCamDeliveryMem)
        for gone in ('memory_v0', 'memory_v1'):                                # v1 = ad78ef2, dev-a1 only
            with self.assertRaises(SystemExit):
                R.controller_class(gone)
        self.assertEqual(R.RESULT_LABELS['memory_v2'], 'interim, tag provider')
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
                _, _, rec = R.run_episode({'episode_id': 'e'}, Path(tmp)/'on'/'e', {}, 'memory_v2', prereg_sha256='0')
                self.assertEqual(rec['result_label'], 'interim, tag provider')
                self.assertEqual(rec['controller_class'], 'harness.m1_owncam_memory.M1OwnCamDeliveryMem')
                R.run_episode({'episode_id': 'e'}, Path(tmp)/'off'/'e', {}, 'off', prereg_sha256='0')
        self.assertEqual(seen, [('M1OwnCamDeliveryMem', 'memory_v2'), ('M1OwnCamDelivery', 'off')])
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
