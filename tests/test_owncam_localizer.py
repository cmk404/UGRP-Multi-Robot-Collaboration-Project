"""Own-camera localization: input boundary, geometry, tagged maps (no SIM)."""
from __future__ import annotations

import ast
import builtins
import hashlib
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RUNTIME_MODULES = (ROOT/'harness'/'owncam_localizer.py', ROOT/'harness'/'wall_tags.py')
ALLOWED_IMPORTS = {'__future__', 'math', 'copy', 'collections.abc', 'numpy', 'cv2', 'harness.wall_tags',
                   'harness.visual_arm', 'sim.masterpi_camera_profile'}
# Simulator state accessors that must never appear in the run-time path.
FORBIDDEN = ('mujoco', 'xpos', 'xquat', 'xmat', 'qpos', 'qvel', 'cam_xpos', 'cam_xmat', 'base_xyz', 'base_rpy',
             'site_xyz', 'eval_only', 'frames_gt', 'gt_trajectory', 'MjData', 'world.data', '.data.body')
# Base maps as merged in PR #173 (head 4789d93): must stay byte-identical.
BASE_MAP_SHA256 = {
    'zone_wide_door': 'a4d2c03de5d0085c2d0ea04d631ec4e9dd95de6dffbb8d44c09870e2ec921903',
    'zone_wide_two_doors': '04b6baf6e457e65e490f921ad2f82aa2778726bf6e7d547fdb5158e55b5b84d2',
    'zone_wide_corridor': 'f7b62732a6db80b97f51cb792082b702dfbbade20de7c81a7795611794919c32',
}


class RuntimeBoundaryTests(unittest.TestCase):
    def test_runtime_modules_import_only_allowed_modules(self):
        for path in RUNTIME_MODULES:
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names = [a.name for a in node.names]
                elif isinstance(node, ast.ImportFrom):
                    names = [node.module]
                else:
                    continue
                for name in names:
                    with self.subTest(path=path.name, module=name):
                        self.assertIn(name, ALLOWED_IMPORTS)

    def test_runtime_modules_never_name_simulator_state(self):
        for path in RUNTIME_MODULES:
            tree = ast.parse(path.read_text())
            names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
            attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
            strings = {n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
                       and not n.value.startswith(('Own-camera', 'Wall AprilTag'))}
            code = [line for line in path.read_text().splitlines() if not line.lstrip().startswith('#')]
            body = '\n'.join(code)
            for token in FORBIDDEN:
                bare = token.strip('.').split('.')[-1]
                with self.subTest(path=path.name, token=token):
                    self.assertNotIn(bare, names | attrs)
                    self.assertFalse(any(token in s for s in strings))
                    if '.' in token:
                        self.assertNotIn(token, body)

    def test_localizer_runs_with_mujoco_import_poisoned(self):
        code = (
            "import sys; sys.modules['mujoco'] = None\n"
            "import harness.owncam_localizer as L, harness.wall_tags as W\n"
            "from tests.test_owncam_localizer import synthetic_run\n"
            "err = synthetic_run(frames=40)\n"
            "bad = [m for m in sys.modules if m.startswith(('sim.multi_masterpi', 'sim.masterpi_production', "
            "'scripts.zone_teacher', 'sim.zone_scene', 'sim.session'))]\n"
            "assert not bad, bad\n"
            "print(err)\n")
        out = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(out.returncode, 0, out.stderr[-2000:])
        self.assertLess(float(out.stdout.strip().splitlines()[-1]), .05)

    def test_offline_localize_never_opens_eval_only(self):
        import scripts.eval_owncam_localization as E
        with tempfile.TemporaryDirectory() as tmp:
            ep = Path(tmp)/'ep'
            write_synthetic_episode(ep)
            real_open = builtins.open
            opened = []

            def guarded(path, *a, **k):
                opened.append(str(path))
                if 'eval_only' in str(path):
                    raise AssertionError(f'localize opened {path}')
                return real_open(path, *a, **k)
            builtins.open = guarded
            try:
                inputs = E.load_inputs(ep)
            finally:
                builtins.open = real_open
            est, _ = E.localize(tagged('zone_wide_door_tags_v1'), inputs, synthetic_params(), seed=0)
            self.assertTrue(any('inputs' in p for p in opened))
            self.assertTrue(est[-1]['initialized'])


class GeometryTests(unittest.TestCase):
    def test_tag_frame_is_right_handed_and_faces_the_room(self):
        from harness.wall_tags import tag_world_frame
        static = tagged('zone_wide_door_tags_v1')
        cx, cy = np.mean(static['bounds_m'][:2]), np.mean(static['bounds_m'][2:])
        for tag in static['landmarks']['tags']:
            c, rot = tag_world_frame(tag)
            self.assertAlmostEqual(float(np.linalg.det(rot)), 1., places=9)
            np.testing.assert_allclose(np.cross(rot[:, 0], rot[:, 1]), rot[:, 2], atol=1e-12)
            if tag['wall'] in ('wall_north', 'wall_south', 'wall_west', 'wall_east'):
                self.assertGreater(float(np.dot(rot[:2, 2], [cx - c[0], cy - c[1]])), 0)

    def test_pnp_recovers_synthetic_tag_translation(self):
        from harness.wall_tags import TagDetector, observed_tag_in_camera
        det = TagDetector()
        h = .036
        for t in ((0., .05, 1.), (.3, .1, 2.2), (-.4, .08, 1.5)):
            # a tag facing the camera, rotated 20 deg about the vertical
            a = math.radians(20)
            ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
            rot = ry @ np.diag([1., -1., -1.])   # tag x right, y up, z towards the camera
            corners = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]) @ rot.T + np.array(t)
            norm = corners[:, :2]/corners[:, 2:]
            sols = det.solve_pnp(norm, .072)
            obs, _ = observed_tag_in_camera({'solutions': sols})
            # IPPE is analytic but not exact under perspective: <0.3% of range
            self.assertLess(float(np.linalg.norm(obs - np.array(t))), .003*float(np.linalg.norm(t)))

    def test_undistort_inverts_fisheye_distortion(self):
        import cv2
        from harness.wall_tags import TagDetector
        det = TagDetector()
        norm = np.array([[0., 0.], [.3, -.2], [-.4, .25], [.1, .3]])
        px = cv2.fisheye.distortPoints(norm.reshape(-1, 1, 2), det.K, det.D).reshape(-1, 2)
        np.testing.assert_allclose(det.undistort(px), norm, atol=1e-6)

    def test_synthetic_localization_tracks_a_drive_through_the_door(self):
        self.assertLess(synthetic_run(frames=120), .03)


class TaggedMapTests(unittest.TestCase):
    def test_base_maps_are_unchanged(self):
        for name, sha in BASE_MAP_SHA256.items():
            data = (ROOT/'maps'/'zones'/f'{name}.json').read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), sha, name)

    def test_tagged_maps_match_definition_and_link_base_hash(self):
        from sim.research_dispatch_arena import digest
        from sim.zone_arena import authored_map
        from sim.zone_landmarks import TAGGED_MAPS
        from sim.zone_landmarks import tagged_map
        for name, spec in TAGGED_MAPS.items():
            static = tagged_map(name)
            base = authored_map(spec['base'])
            self.assertEqual(static['base_map']['static_map_sha256'], digest(base))
            self.assertEqual({k: v for k, v in static.items() if k not in ('map_id', 'version', 'base_map', 'landmarks')},
                             {k: v for k, v in base.items() if k not in ('map_id', 'version')})

    def test_tags_sit_on_wall_faces_inside_the_wall_height(self):
        from sim.zone_landmarks import TAGGED_MAPS
        for name in TAGGED_MAPS:
            static = tagged(name)
            walls = {o['id']: o for o in static['obstacles']}
            tags = static['landmarks']['tags']
            placement = static['landmarks']['placement']
            self.assertEqual([t['id'] for t in tags], list(range(len(tags))))
            self.assertLess(len(tags), 587)  # tag36h11 dictionary size
            for tag in tags:
                wall = walls[tag['wall']]
                (cx, cy), (hx, hy) = wall['center_m'], wall['half_extents_m']
                nx, ny = tag['normal_xy']
                x, y, z = tag['center_m']
                self.assertAlmostEqual(x, cx + nx*hx, places=4) if nx else self.assertAlmostEqual(y, cy + ny*hy, places=4)
                self.assertLessEqual(z + placement['plate_m']/2, wall['height_m'] + 1e-9)
                self.assertGreaterEqual(z - placement['plate_m']/2, 0.)
                # the plate lies within the face and clear of every other wall
                along = abs(x - cx) if ny else abs(y - cy)
                self.assertLessEqual(along + placement['plate_m']/2, (hx if ny else hy) + 1e-9)
                for other in static['obstacles']:
                    if other['id'] == wall['id']:
                        continue
                    (ox, oy), (ohx, ohy) = other['center_m'], other['half_extents_m']
                    front = (x + nx*.005, y + ny*.005)
                    self.assertFalse(abs(front[0] - ox) < ohx and abs(front[1] - oy) < ohy, (name, tag['id']))

    def test_every_door_has_post_tags_on_both_faces(self):
        for name in ('zone_wide_door_tags_v1', 'zone_wide_two_doors_tags_v1'):
            static = tagged(name)
            for door in (p for p in static['passages'] if p['kind'] == 'door'):
                dx, dy = door['center_m']
                half = door['width_m']/2
                y0, y1 = static['bounds_m'][2:]
                for edge in (dy - half, dy + half):
                    if min(abs(edge - y0), abs(edge - y1)) < .05:
                        continue  # the door opening reaches a perimeter wall: no post there
                    for normal in ([-1, 0], [1, 0]):
                        near = [t for t in static['landmarks']['tags'] if t['normal_xy'] == normal
                                and abs(t['center_m'][0] - dx) < .03 and abs(abs(t['center_m'][1] - edge) - .075) < .01]
                        self.assertTrue(near, (name, door['id'], edge, normal))


# ---------------------------------------------------------------- helpers
def tagged(name):
    """The committed map JSON as the robot loads it (verified in TaggedMapTests)."""
    return json.loads((ROOT/'maps'/'zones'/f'{name}.json').read_text())


LOOK = {1: 1500, 3: 740, 4: 2320, 5: 1320, 6: 1500}   # the search pose (camera -21 deg)


def synthetic_params():
    from harness.owncam_localizer import DEFAULT_PARAMS
    import copy
    p = copy.deepcopy(DEFAULT_PARAMS)
    p['particles'] = 1500
    p['motion'].update(gain=np.eye(3).tolist(), tau_s=1e-3)
    p['measurement'].update(azimuth_std_rad=math.radians(1.), elevation_std_rad=math.radians(2.),
                            range_log_std=.08, tag_temper=1.)
    return p


def synthetic_detections(static, pose_xyyaw, servo, rng, noise_px=.5):
    """Detections a pinhole-in-fisheye camera would give for the map tags."""
    import cv2
    from harness.wall_tags import TagDetector, camera_in_base, tag_world_frame
    det = TagDetector()
    o_bc, r_bc = camera_in_base(servo)
    x, y, yaw = pose_xyyaw
    c, s = math.cos(yaw), math.sin(yaw)
    r_wb = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    o_wc = np.array([x, y, 0.]) + r_wb @ o_bc
    r_wc = r_wb @ r_bc
    out = []
    for tag in static['landmarks']['tags']:
        cw, rot = tag_world_frame(tag)
        h = tag['size_m']/2
        corners_w = cw + np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]]) @ rot.T
        pc = (corners_w - o_wc) @ r_wc
        if np.any(pc[:, 2] < .1) or np.dot(r_wc.T @ rot[:, 2], pc.mean(0)) >= 0:
            continue
        norm = pc[:, :2]/pc[:, 2:]
        px = cv2.fisheye.distortPoints(norm.reshape(-1, 1, 2), det.K, det.D).reshape(-1, 2)
        if np.any(px < 0) or np.any(px[:, 0] > 639) or np.any(px[:, 1] > 479) or np.linalg.norm(pc.mean(0)) > 3.:
            continue
        px = px + rng.normal(size=px.shape)*noise_px
        side = float(np.mean(np.linalg.norm(px - np.roll(px, 1, axis=0), axis=1)))
        if side < 8:
            continue
        sols = det.solve_pnp(det.undistort(px), tag['size_m'])
        if sols:
            out.append({'id': tag['id'], 'side_px': side, 'solutions': sols})
    return out


def synthetic_drive(frames):
    """Truth and issued commands: drive east through door_1 at 0.1 m/s."""
    commands = [{'t': 0., 'kind': 'initial_servo_command', 'pulses': {str(k): v for k, v in LOOK.items()}}]
    truth, x, y, yaw, t = [], 1.2, .05, 0., 0.
    for i in range(frames):
        commands.append({'t': round(t, 4), 'kind': 'mecanum', 'forward': .1, 'left': 0., 'turn': 0., 'duration_s': .3})
        t += .25
        x += .1*.25
        truth.append((round(t, 4), x, y, yaw))
    return commands, truth


def synthetic_run(frames=80):
    from harness.owncam_localizer import OwnCamLocalizer
    static = tagged('zone_wide_door_tags_v1')
    rng = np.random.default_rng(3)
    commands, truth = synthetic_drive(frames)
    loc = OwnCamLocalizer(static, synthetic_params(), seed=1)
    ci, errs = 0, []
    for t, x, y, yaw in truth:
        while ci < len(commands) and commands[ci]['t'] < t - 1e-6:
            loc.command(commands[ci]); ci += 1
        est = loc.update(t, synthetic_detections(static, (x, y, yaw), LOOK, rng), None)
        if est['initialized']:
            errs.append(math.hypot(est['x'] - x, est['y'] - y))
    return float(np.median(errs[len(errs)//4:]))


def write_synthetic_episode(ep):
    static = tagged('zone_wide_door_tags_v1')
    rng = np.random.default_rng(5)
    commands, truth = synthetic_drive(20)
    (ep/'inputs').mkdir(parents=True)
    (ep/'derived').mkdir()
    (ep/'eval_only').mkdir()
    frames, dets = [], []
    for i, (t, x, y, yaw) in enumerate(truth):
        frames.append({'frame': i, 't': t, 'file': f'frames/{i:05d}.jpg', 'sha256': '',
                       'commanded_servo': {str(k): v for k, v in LOOK.items()}, 'posture': 'search_drive'})
        dets.append({'frame': i, 't': t, 'detections': synthetic_detections(static, (x, y, yaw), LOOK, rng)})
    for name, rows in (('inputs/commands.jsonl', commands), ('inputs/frames.jsonl', frames),
                       ('derived/detections.jsonl', dets)):
        (ep/name).write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (ep/'eval_only'/'frames_gt.jsonl').write_text('{"trap": true}\n')


if __name__ == '__main__':
    unittest.main()
