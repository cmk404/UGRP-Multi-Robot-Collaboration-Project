"""M1 own-camera delivery: runtime boundary, M1 contract, pose limits, controller mechanics (no simulator)."""
from __future__ import annotations

import ast
import base64
import hashlib
import json
import math
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

RUNTIME = (ROOT/'harness'/'m1_owncam_delivery.py', ROOT/'harness'/'m1_owncam_contract.py',
           ROOT/'harness'/'owncam_pose_source.py')
ALLOWED = {'__future__', 'math', 'hashlib', 'json', 'base64', 'collections.abc', 'dataclasses', 'numpy',
           'harness', 'harness.m1_owncam_contract', 'harness.owncam_pose_source', 'harness.owncam_localizer',
           'harness.wall_tags', 'harness.owncam_drive', 'harness.owncam_drive_v2', 'harness.zone_color_boxes',
           'harness.wrist_zone_skill', 'harness.wrist_zone_skill_v5', 'harness.wrist_zone_skill_v6'}
FORBIDDEN = ('mujoco', 'xpos', 'xquat', 'qpos', 'qvel', 'base_xyz', 'base_rpy', 'eval_only', 'gt_trajectory',
             'frames_eval', 'MjData', 'setup_only', 'position_m', 'GtStubPoseSource')


def obs(frame_id=5, camera='robot_cam', robot='r1', t=10., payload=b'jpeg-bytes', sha=None):
    return {'camera': camera, 'robot_id': robot, 'frame_id': frame_id, 'sim_time': t,
            'image': base64.b64encode(payload).decode(), 'sha256': sha or hashlib.sha256(payload).hexdigest(),
            'actuator_state': {'servo_pulses': {'1': 2000, '3': 740, '4': 2320, '5': 1320, '6': 1500}}}


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
            for token in FORBIDDEN:
                with self.subTest(path=path.name, token=token):
                    self.assertNotIn(token, names)

    def test_controller_imports_with_mujoco_poisoned(self):
        code = ("import sys; sys.modules['mujoco'] = None\n"
                "import harness.m1_owncam_delivery, harness.m1_owncam_contract, harness.owncam_pose_source\n"
                "import harness.wrist_zone_skill_v4, harness.wrist_zone_skill_v5\n"
                "bad = [m for m in sys.modules if m.startswith(('sim.multi_masterpi', 'sim.zone_scene', "
                "'scripts.zone_teacher', 'sim.session'))]\n"
                "assert not bad, bad\n")
        out = subprocess.run([sys.executable, '-c', code], cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(out.returncode, 0, out.stderr[-2000:])


class ContractTests(unittest.TestCase):
    def test_sources(self):
        from harness.m1_owncam_contract import M1ContractError, require_m1_source
        self.assertEqual(require_m1_source('owncam_pf_v2:abcd1234'), 'owncam_pf_v2:abcd1234')
        for bad in ('gt_stub_eval_only', 'gt', '', None, 'truth:owncam_pf', 'owncam'):
            with self.subTest(bad=bad), self.assertRaises(M1ContractError):
                require_m1_source(bad)

    def test_observation_validation(self):
        from harness.m1_owncam_contract import M1ContractError, validate_observation
        validate_observation(obs(), robot_id='r1', previous_frame_id=4, now=10.1)
        cases = {'nav_cam': obs(camera='nav_cam'), 'top': obs(camera='cctv_top'), 'robot': obs(robot='r2'),
                 'old_frame': obs(frame_id=4), 'stale': obs(t=9.), 'hash': obs(sha='0'*64)}
        for name, o in cases.items():
            with self.subTest(case=name), self.assertRaises(M1ContractError):
                validate_observation(o, robot_id='r1', previous_frame_id=4, now=10.1)

    def _judge(self, **kw):
        from harness.m1_owncam_contract import judge
        base = dict(pose_sources=['owncam_pf_v2:abcd1234'], skill_reason='SKILL_OWN_RGB_PLACEMENT_IN_SLOT',
                    skill_claim_in_slot=True, gt_box_in_slot=True, wall_contacts=0, weld_used=False,
                    face_fallback_used=False, pickup_source='own_rgb_search', within_limit=True,
                    extra_checks={'look_back_pose_gate_ok': True})
        return judge(**{**base, **kw})

    def test_judge_separates_diagnostic_and_m1(self):
        ok = self._judge()
        self.assertTrue(ok['m1_success'] and ok['diagnostic_success'] and ok['counts_as_m1'])
        gt = self._judge(pose_sources=['gt_stub_eval_only'])
        self.assertTrue(gt['diagnostic_success'])
        self.assertFalse(gt['m1_success'] or gt['counts_as_m1'])
        self.assertIn('all_pose_sources_owncam', gt['m1_failed_checks'])
        for kw, check in ((dict(pickup_source='scenario'), 'pickup_from_own_rgb'),
                          (dict(face_fallback_used=True), 'no_face_normal_map_fallback'),
                          (dict(wall_contacts=1), 'no_wall_contact'), (dict(weld_used=True), 'weld_off'),
                          (dict(extra_checks={'look_back_pose_gate_ok': False}), 'look_back_pose_gate_ok')):
            with self.subTest(check=check):
                r = self._judge(**kw)
                self.assertFalse(r['m1_success'])
                self.assertIn(check, r['m1_failed_checks'])
        false = self._judge(gt_box_in_slot=False)
        self.assertTrue(false['false_success'])
        self.assertFalse(false['m1_success'] or false['diagnostic_success'])

    def test_exporter_guard(self):
        from harness.m1_owncam_contract import M1ContractError, assert_exportable
        ok = self._judge()
        assert_exportable({**ok, 'input_contract': 'x'})
        with self.assertRaises(M1ContractError):
            assert_exportable({k: v for k, v in ok.items() if k != 'counts_as_m1'} | {'input_contract': 'x'})
        with self.assertRaises(M1ContractError):
            assert_exportable({**ok, 'input_contract': 'x', 'counts_as_m1': False})
        with self.assertRaises(M1ContractError):
            assert_exportable({**ok, 'input_contract': 'x', 'pose_sources_seen': ['gt_stub_eval_only']})


class PoseLimitTests(unittest.TestCase):
    def test_limits(self):
        from harness.owncam_pose_source import PoseLimits, PoseReport, check_limits
        lim = PoseLimits(.05, .035, max_since_tag_s=3., max_since_look_s=3.)
        good = PoseReport(t_est=10., initialized=True, x_m=1., y_m=0., yaw_rad=0., std_xy_m=.04,
                          std_yaw_rad=.01, since_tag_s=1.)
        self.assertEqual(check_limits(good, 10.1, lim, since_look_s=1.), [])
        self.assertEqual(check_limits(PoseReport(t_est=10., initialized=False), 10., lim), ['not_initialized'])
        bad = PoseReport(t_est=9., initialized=True, x_m=1., y_m=0., yaw_rad=0., std_xy_m=.06,
                         std_yaw_rad=.05, since_tag_s=5.)
        self.assertEqual(set(check_limits(bad, 10., lim, since_look_s=4.)),
                         {'stale', 'std_xy', 'std_yaw', 'since_tag', 'since_look'})

    def test_source_label_is_owncam_and_stable(self):
        from harness.m1_owncam_contract import require_m1_source
        from harness.owncam_pose_source import calibration_label
        a = calibration_label({'b': 1, 'a': [1, 2]})
        self.assertEqual(a, calibration_label({'a': [1, 2], 'b': 1}))
        require_m1_source(a)


class M1LocalizerTests(unittest.TestCase):
    """M1 calibration additions are optional: absent keys and the default profile keep loop v2 exactly."""

    @staticmethod
    def _params(name):
        path = {'v2': ROOT/'experiments'/'2026-09-26-zone-owncam-loop-v2'/'calibration_loop_v2.json',
                'm1': ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'calibration_m1_dev.json'}[name]
        return json.loads(path.read_text())['params']

    def _loc(self, name):
        import numpy as np
        from harness.owncam_localizer import OwnCamLocalizer
        from sim.zone_landmarks import tagged_map
        loc = OwnCamLocalizer(tagged_map('zone_wide_door_tags_v2'), self._params(name), seed=3)
        loc.initialized = True
        loc.px[:] = [0., -1., 0.]
        loc.scale[:] = np.array([1.3, 1., 1.])
        return loc

    def _drive(self, loc):
        loc.command({'t': 0., 'kind': 'drive', 'forward': .1, 'turn': .05, 'duration_s': .5})
        loc.command({'t': .6, 'kind': 'hold'})
        loc.predict_to(1.5)

    def test_default_profile_is_loop_v2(self):
        import numpy as np
        a, b = self._loc('v2'), self._loc('m1')
        for loc in (a, b):
            self._drive(loc)
        np.testing.assert_array_equal(a.px, b.px)

    def test_fine_profile_lags_per_axis_and_ignores_slip_scale(self):
        import numpy as np
        loc = self._loc('m1')
        loc.set_motion_profile(0., 'fine')
        mp = loc.params['motion_profiles']['fine']
        mp['noise_rel'], mp['noise_abs'] = [0., 0., 0.], [0., 0., 0.]
        self._drive(loc)
        g, tau = mp['gain'][0][0], mp['tau_axis_s'][0]
        lag = lambda D: D - tau*(1 - math.exp(-D/tau)) + mp['tau_stop_s']*(1 - math.exp(-D/tau))
        self.assertAlmostEqual(float(np.mean(loc.px[:, 0])), g*.1*lag(.5), delta=.003)   # scale 1.3 not applied
        with self.assertRaises(KeyError):
            loc.set_motion_profile(2., 'nope')

    def test_kidnap_reset_needs_settled_consecutive_frames(self):
        import numpy as np
        loc = self._loc('m1')
        loc.params['measurement']['max_range_m'] = None
        tag = next(iter(loc.tags))
        loc._loglik = lambda px, dets, pose: np.full(len(px), -9.)
        loc._reset_from = lambda dets, pose, k: np.zeros((k, 3))
        det = [{'id': tag, 'solutions': [1]}]
        loc.command({'t': 1., 'kind': 'look', 'pan_pulse': 1500})
        loc.update(1.0, det)                        # 0 s after an own servo command: not counted
        loc.update(1.2, det)                        # 0.2 s: not settled
        self.assertEqual(loc.stats['resets'], 0)
        loc.update(1.35, det)                       # settled, 1st
        self.assertEqual(loc.stats['resets'], 0)
        loc.update(1.5, det)                        # settled, 2nd in a row -> reset
        self.assertEqual(loc.stats.get('kidnap_resets'), 1)
        v2 = self._loc('v2')
        v2.params['measurement']['max_range_m'] = None
        v2._loglik, v2._reset_from = loc._loglik, loc._reset_from
        for t in (1.35, 1.5, 1.7, 1.9):
            v2.update(t, det)
        self.assertEqual(v2.stats['resets'], 0)     # loop v2 never resets on a single-tag floor


class ControllerTests(unittest.TestCase):
    def _ctl(self, order_kind='own_rgb_bay'):
        from harness.m1_owncam_delivery import M1OwnCamDelivery
        from harness.wrist_zone_skill import PoseEstimate
        from harness.wrist_zone_skill_v4 import WristZoneDeliveryV4
        from harness.wrist_zone_skill_v5 import WristZoneDeliveryV5
        from sim.zone_landmarks import tagged_map
        cal = json.loads((ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'calibration_m1_dev.json').read_text())
        factory = ((lambda o: WristZoneDeliveryV5(o, mode='m1')) if order_kind == 'own_rgb_bay'
                   else (lambda o: WristZoneDeliveryV4(o)))
        ctl = M1OwnCamDelivery(tagged_map('zone_wide_door_tags_v2'), cal['params'], box_kind='cyan', slot_id='A1',
                               slot_xy=(4.6, 0.), skill_factory=factory, order_kind=order_kind,
                               pose_estimate_cls=PoseEstimate, search_rows_y=(-2.45, -1.65, -.85, -.05, .75))
        ctl.on_command({'t': 0., 'kind': 'initial_servo_command',
                        'pulses': {'1': 2000, '3': 740, '4': 2320, '5': 1320, '6': 1500}})
        return ctl

    def test_init_looks_until_localized_then_gives_up(self):
        from harness.m1_owncam_delivery import MAX_GATE_LOOKS
        ctl = self._ctl()
        t, pans = 0., set()
        for _ in range(2000):
            d = ctl.decide(t)
            if d['mode'] == 'done':
                break
            for c in d.get('commands', []):
                ctl.on_command({'t': t, **c})
                if c['kind'] == 'look':
                    pans.add(c['pan_pulse'])
            t += .1
        self.assertEqual(ctl.outcome, 'NOT_INITIALIZED')          # no frames: never localizes, never moves
        self.assertEqual(ctl.init_looks, MAX_GATE_LOOKS)
        self.assertTrue({970, 2030} <= pans)                      # the wide sweep ran
        self.assertIsNone(ctl.skill)

    def test_v5_order_is_a_coarse_bay_from_own_search(self):
        from harness.wrist_zone_skill_v5 import BAY_APPROACH_CLEARANCE_M, CoarseOrderSheet
        ctl = self._ctl()
        ctl.target_xy, ctl.pickup_source = (-.19, -2.48), 'own_rgb_search'
        order = ctl._make_order()
        self.assertIsInstance(order, CoarseOrderSheet)
        self.assertEqual(order.pickup_bay_half_m, (.25, .25))
        self.assertFalse(hasattr(order, 'pickup_xy_m'))
        gx, gy = ctl._approach_goal(ctl.target_xy)
        self.assertAlmostEqual(gx, -.19 - .25 - BAY_APPROACH_CLEARANCE_M)
        self.assertEqual(ctl.skill_factory(order).mode, 'm1')
        v4 = self._ctl('own_rgb_point')
        v4.target_xy = (-.19, -2.48)
        self.assertEqual(tuple(v4._make_order().pickup_xy_m), (-.19, -2.48))

    def test_fine_profile_follows_skill_phase_and_forces_a_look(self):
        ctl = self._ctl()
        ctl._set_motion_profile(1., 'nav_pregrasp')
        self.assertIsNone(ctl.pose.loc.motion_profile)
        ctl._set_motion_profile(2., 'grasp')
        self.assertEqual(ctl.pose.loc.motion_profile, 'fine')
        self.assertTrue(ctl.manipulated)
        ctl._set_motion_profile(3., 'reseat_release')
        self.assertEqual(ctl.pose.loc.motion_profile, 'fine')
        ctl._set_motion_profile(4., 'nav_preplace')
        self.assertIsNone(ctl.pose.loc.motion_profile)
        self.assertTrue(ctl.manipulated)                          # cleared only by the post-manipulation look

    def test_order_sheet_has_no_scenario_position(self):
        import inspect

        from harness.m1_owncam_delivery import M1OwnCamDelivery
        params = inspect.signature(M1OwnCamDelivery.__init__).parameters
        self.assertNotIn('pickup_xy', params)
        self.assertNotIn('spawn_y_hint', params)


class PreReviewTests(unittest.TestCase):
    """Codex M1 pre-review fixes: test launch guard, per-frame look-back gate, v6 approach, adoption."""

    PREREG = ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'prereg.json'

    def test_test_split_needs_explicit_flag_and_freeze(self):
        from unittest import mock

        import scripts.run_m1_owncam as runner
        seen = []
        with mock.patch.object(runner, 'run', side_effect=lambda spec, out, student: seen.append(spec) or
                               ({'outcome': 'x', 'm1_success': False, 'm1_failed_checks': [], 'diagnostic_success': False,
                                 'false_success': False, 'sim_s': 0, 'looks': 0}, {'wall_s': 0, 'load_average': {}})):
            runner.main(['--prereg', str(self.PREREG), '--output', '/nonexistent-never-written'])
            self.assertTrue(seen and all(sp['split'] == 'dev' for sp in seen))       # default: dev only
            self.assertTrue(all(sp['contact_profile'] for sp in seen))
            with self.assertRaises(SystemExit):
                runner.main(['--prereg', str(self.PREREG), '--output', '/x', '--split', 'test'])
            with self.assertRaises(SystemExit):
                runner.main(['--prereg', str(self.PREREG), '--output', '/x', '--only', 'm1test-s101'])
            n = len(seen)
            with self.assertRaises(SystemExit):
                runner.main(['--prereg', str(self.PREREG), '--output', '/x', '--split', 'test',
                             '--frozen', str(ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'no_such_frozen.json')])
            self.assertEqual(len(seen), n)                                              # nothing ran

    def _ctl_v6(self):
        from harness.m1_owncam_delivery import M1OwnCamDelivery
        from harness.wrist_zone_skill import PoseEstimate
        from harness.wrist_zone_skill_v6 import StaticKeepout, WristZoneDeliveryV6
        from sim.zone_landmarks import tagged_map
        static = tagged_map('zone_wide_door_tags_v2')
        cal = json.loads((ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'calibration_m1_dev.json').read_text())
        keep = tuple(StaticKeepout(f'spawn_row_{i}', (-.85, y), .17, 'static_layout_idle_spawn')
                     for i, y in enumerate((-2.25, -.85, .55)))
        ctl = M1OwnCamDelivery(static, cal['params'], box_kind='cyan', slot_id='A1', slot_xy=(4.6, 0.),
                               skill_factory=lambda o: WristZoneDeliveryV6(o, mode='m1', static_keepouts=keep,
                                                                           static_bounds_m=static['bounds_m']),
                               pose_estimate_cls=PoseEstimate, search_rows_y=(-2.45, -1.65, -.85, -.05, .75))
        return ctl, keep

    def test_v6_approach_point_clears_spawn_keepouts_with_a_half_metre_bay(self):
        ctl, keep = self._ctl_v6()
        for target in ((-.19, -2.48), (-.2, -.85), (.4, -.05), (1.6, .75)):
            ctl.skill, ctl.target_xy = None, target
            goal = ctl._approach_goal(target)
            self.assertEqual(ctl.skill.order.pickup_bay_half_m, (.25, .25))
            self.assertIsNotNone(goal)
            for k in keep:
                self.assertGreater(math.hypot(goal[0] - k.xy_m[0], goal[1] - k.xy_m[1]), .17 + .17)

    def test_look_back_gate_is_checked_on_every_confirmation_frame(self):
        from harness.m1_owncam_delivery import MAX_GATE_LOOKS
        from harness.owncam_pose_source import PoseReport
        ctl, _ = self._ctl_v6()

        class Skill:                                    # stand-in: always at a confirmation step
            phase, look_back_steps, decided = 'look_back', 0, []
            box = type('B', (), {'held': False})()

            def decide(self, obs, est):
                self.decided.append(obs['frame_id'])
                return {'kind': 'wait', 'duration': .1}
        ctl.skill, ctl.phase, ctl.servo = Skill(), 'skill', {1: 1500, 3: 740, 4: 2320, 5: 1320, 6: 1500}
        good = PoseReport(t_est=0., initialized=True, x_m=4.4, y_m=0., yaw_rad=0., std_xy_m=.02, std_yaw_rad=.01,
                          since_tag_s=.5, source='owncam_pf_v2:abcd1234')
        bad = PoseReport(**{**good.__dict__, 'std_xy_m': .2, 'std_yaw_rad': .2})
        fid = [10]

        def step(now, report):
            fid[0] += 1
            ctl.last_obs = obs(frame_id=fid[0], t=now)
            ctl.pose.report = lambda _now: PoseReport(**{**report.__dict__, 't_est': _now})
            return ctl._skill(now)
        ctl.last_look_t = 99.
        self.assertEqual(step(100., good)['mode'], 'macro')                         # fresh look, gate passes
        self.assertEqual([g['frame_id'] for g in ctl.lookback_gates], Skill.decided)
        d = step(101., bad)                                                           # retry frame, now uncertain
        self.assertEqual(d['mode'], 'tick')                                           # re-look, no confirmation
        self.assertEqual(len(Skill.decided), 1)
        ctl.sweep = None
        ctl.gate_looks = MAX_GATE_LOOKS
        self.assertEqual(step(102., bad)['outcome'], 'POSE_UNCERTAIN')
        self.assertEqual(len(Skill.decided), 1)

    def test_runner_keepouts_build_every_static_keepout_skill(self):
        import importlib

        import scripts.run_m1_owncam as runner
        from harness.wrist_zone_skill_v5 import CoarseOrderSheet
        from sim.zone_landmarks import tagged_map
        static = tagged_map('zone_wide_door_tags_v2')
        keep = runner.static_layout_keepouts(static)
        self.assertEqual([k.xy_m for k in keep], [(-.85, -2.25), (-.85, -.85), (-.85, .55)])
        order = CoarseOrderSheet('cyan', 'b', (.4, -.05), (.25, .25), 'A1', (4.6, 0.))
        for name in runner.STATIC_KEEPOUT_SKILLS:                 # the exact runner path (dev-a7 crash)
            module, cls, _, kwargs = runner.SKILLS[name]
            sk = getattr(importlib.import_module(module), cls)(order, robot_id='r3', static_keepouts=keep,
                                                               static_bounds_m=static['bounds_m'], **kwargs)
            self.assertFalse(sk.approach_point()['blocked'])

    def test_test_adoption_rules(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('m1_build', ROOT/'experiments'/'2026-09-26-zone-m1-owncam'/'build_results.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ids = [f'm1test-s{i}' for i in range(101, 107)]
        f = {'frozen_source_sha': 'abc'}
        ok = [{'attempt': 'test', 'episode': e, 'infrastructure_failure': False, 'freeze': f} for e in ids]
        adopted, missing, problems = mod.adopt_test(ok, ids)
        self.assertEqual((len(adopted), missing, problems), (6, [], []))
        infra = ok[:5] + [{'attempt': 'test', 'episode': ids[5], 'infrastructure_failure': True, 'freeze': f}]
        self.assertEqual(mod.adopt_test(infra, ids)[1], [ids[5]])                   # missing until a rerun
        rerun = infra + [{'attempt': 'test-rerun', 'episode': ids[5], 'infrastructure_failure': False, 'freeze': f}]
        self.assertEqual(mod.adopt_test(rerun, ids)[1:], ([], []))
        dup = ok + [{'attempt': 'test-rerun', 'episode': ids[0], 'infrastructure_failure': False, 'freeze': f}]
        self.assertTrue(mod.adopt_test(dup, ids)[2])                                 # rerun of a finished episode
        mixed = ok[:5] + [{**ok[5], 'freeze': {'frozen_source_sha': 'zzz'}}]
        self.assertTrue(mod.adopt_test(mixed, ids)[2])


if __name__ == '__main__':
    unittest.main()
