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
           'harness.wrist_zone_skill'}
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
                "import harness.wrist_zone_skill_v4\n"
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


class ControllerTests(unittest.TestCase):
    def _ctl(self):
        from harness.m1_owncam_delivery import M1OwnCamDelivery
        from harness.wrist_zone_skill import PoseEstimate
        from harness.wrist_zone_skill_v4 import WristZoneDeliveryV4
        from sim.zone_landmarks import tagged_map
        cal = json.loads((ROOT/'experiments'/'2026-09-26-zone-owncam-loop-v2'/'calibration_loop_v2.json').read_text())
        ctl = M1OwnCamDelivery(tagged_map('zone_wide_door_tags_v2'), cal['params'], box_kind='cyan', slot_id='A1',
                               slot_xy=(4.6, 0.), skill_factory=lambda o: WristZoneDeliveryV4(o),
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

    def test_order_sheet_has_no_scenario_position(self):
        import inspect

        from harness.m1_owncam_delivery import M1OwnCamDelivery
        params = inspect.signature(M1OwnCamDelivery.__init__).parameters
        self.assertNotIn('pickup_xy', params)
        self.assertNotIn('spawn_y_hint', params)


if __name__ == '__main__':
    unittest.main()
