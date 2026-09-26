"""M2 pair approach driver: order sheet, geometry, input boundary and final-heading control."""
from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness import pair_owncam_approach as pa  # noqa: E402

CAL = ROOT / 'experiments' / '2026-09-26-zone-owncam-loop-v2' / 'calibration_loop_v2.json'
MAP = ROOT / 'maps' / 'zones' / 'zone_wide_door_tags_v2.json'


def static_map():
    from sim.zone_landmarks import TaggedZoneScene
    return TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 11, {'A': {'cyan': 1}}, None).config['static_map']


def test_order_sheet_is_the_setup_pose_on_the_grid_and_bounded():
    for beam in ((0.62, -1.23, 0.13), (0.47, -0.78, -0.31), (0.78, -1.62, 0.38)):
        sheet = pa.coarse_order_sheet(beam)['beam_xyyaw']
        assert abs(sheet[0] - beam[0]) <= .05 + 1e-9 and abs(sheet[1] - beam[1]) <= .05 + 1e-9
        assert abs(sheet[2] - beam[2]) <= math.radians(5.) + 1e-9
        assert abs(sheet[0] / .10 - round(sheet[0] / .10)) < 1e-6


def test_prestation_is_behind_the_station_along_its_heading():
    x, y, yaw = pa.prestation((1., 2., math.pi / 2), .3)
    assert math.isclose(x, 1., abs_tol=1e-9) and math.isclose(y, 1.7) and yaw == math.pi / 2


def test_beam_keepout_holds_rotated_footprint():
    ko = pa.beam_keepout((0., 0., math.pi / 2), .6, .04, .06)
    assert math.isclose(ko['half_extents_m'][1], .3 + .06, abs_tol=1e-6)
    assert math.isclose(ko['half_extents_m'][0], .02 + .06, abs_tol=1e-6)


def test_student_modules_do_not_import_the_simulator():
    # Same rule as the M1 boundary test (tests/test_owncam_localizer.py): no MuJoCo, no simulator-state
    # modules; static calibrations such as sim.masterpi_camera_profile are allowed inputs.
    program = ("import sys; sys.modules['mujoco'] = None\n"
               "import harness.pair_owncam_approach\n"
               "bad = [m for m in sys.modules if m.startswith(('sim.multi_masterpi', 'sim.masterpi_production', "
               "'scripts.zone_teacher', 'sim.zone_scene', 'sim.session', 'sim.zone_cargo', 'sim.camera_robot_port'))]\n"
               "assert not bad, bad\n")
    subprocess.run([sys.executable, '-c', program], cwd=ROOT, check=True)


def _driver(goal, pose):
    params = json.loads(CAL.read_text())['params']
    drv = pa.PairApproachDriver(static_map(), params, goal_xyyaw=goal, initial_servo={1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500})
    drv.loc.initialized = True
    drv.loc.px[:] = np.array(pose, float)
    drv.loc.px[:, :2] += np.random.default_rng(0).normal(scale=.002, size=(drv.loc.n, 2))
    drv.state, drv.state_since = 'drive', 0.
    drv.checkpoints_done.update((1.5, .6))
    return drv


def test_turns_in_place_toward_final_heading_at_goal_position():
    drv = _driver((0., -1., 2.8), (0., -1., 2.3))
    drv.loc.last_tag_t = 0.
    cmds = drv.tick(0.)
    assert cmds and cmds[0]['kind'] == 'mecanum', cmds
    assert cmds[0]['forward'] == 0. and cmds[0]['left'] == 0. and cmds[0]['turn'] > 0


def test_translates_in_body_frame_toward_goal_with_any_heading():
    drv = _driver((.5, -1., 0.), (0., -1., math.pi / 2))   # goal is east, robot faces north
    drv.loc.last_tag_t = 0.
    cmds = drv.tick(0.)
    assert cmds[0]['kind'] == 'mecanum'
    assert cmds[0]['left'] < 0          # east is the robot's right
    assert abs(cmds[0]['forward']) < abs(cmds[0]['left'])


def test_door_schedule_moves_own_base_onto_the_axis_and_keeps_axial_identical():
    from scripts import run_m2_pair as rm
    scheds = {}
    for rid, est in (('r1', (0.3, 0.13, 0.05)), ('r2', (1.75, 0.10, math.pi - 0.05))):
        st = rm.M2DoorStudent.__new__(rm.M2DoorStudent)
        st.rid, st.claims = rid, {}
        st.door_plan, st.axial_m, st.grasp_estimate = rm.DOOR_PLAN, 2.2, list(est)
        st.segments, st.seg = [.55, .85, .80], 0
        scheds[rid] = st.door_schedule(10.)
        align = scheds[rid][0][2]
        # r1 (facing +x) is north of the axis -> moves right (negative left); r2 (facing -x) north -> moves left
        assert (align['left'] < 0) if rid == 'r1' else (align['left'] > 0)
        assert (align['turn'] < 0) if rid == 'r1' else (align['turn'] > 0)
    (a0, a1, ca), (b0, b1, cb) = scheds['r1'][1], scheds['r2'][1]
    assert (a0, a1) == (b0, b1) and ca['forward'] == -cb['forward'] > 0
    assert math.isclose(a1 - a0, .55 / (rm.study.SPEED_M_S * rm.study.CARRY_ODOM_SCALE['axial']))


def _driver_v2(goal, pose):
    params = json.loads(CAL.read_text())['params']
    drv = pa.PairApproachDriverV2(static_map(), params, goal_xyyaw=goal,
                                  initial_servo={1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500})
    drv.loc.initialized = True
    drv.loc.px[:] = np.array(pose, float)
    drv.loc.px[:, :2] += np.random.default_rng(0).normal(scale=.002, size=(drv.loc.n, 2))
    drv.loc.last_tag_t = 0.
    drv.state, drv.state_since = 'drive', 0.
    return drv


def test_v2_translates_with_heading_held_far_from_the_goal():
    drv = _driver_v2((1.5, -1.0, math.pi - .1), (0., -1., 0.))
    cmds = drv.tick(0.)
    assert cmds[0]['kind'] == 'mecanum' and cmds[0]['forward'] > 0
    assert abs(cmds[0]['turn']) < 1e-3                    # holds the start heading, no turn toward pi
    assert drv.hold_yaw is not None and not drv.rotated


def test_v2_rotates_in_place_near_the_goal():
    drv = _driver_v2((.2, -1.0, math.pi - .1), (0., -1., 0.))
    cmds = drv.tick(0.)
    assert cmds[0]['kind'] == 'mecanum' and cmds[0]['forward'] == 0. and cmds[0]['left'] == 0.
    assert abs(cmds[0]['turn']) == pa.TURN_IN_PLACE and drv.rotated


def test_v2_looks_after_each_rotation_step():
    drv = _driver_v2((.2, -1.0, math.pi - .1), (0., -1., 0.))
    drv.tick(0.)                                  # sets the step reference
    drv.loc.px[:, 2] = pa.TURN_STEP_RAD + .01     # estimated rotation reached one step
    cmds = drv.tick(.1)
    assert cmds == [{'kind': 'hold'}] and drv.state == 'look_arm' and drv.look_reason == 'turn_step'


def test_v2_relocalizes_after_unfixed_looks_then_gives_up():
    from harness.owncam_drive import MAX_LOOKS_WITHOUT_FIX
    drv = _driver_v2((1.0, -1.0, 0.), (0., -1., 0.))
    old = drv.loc
    drv.looks_without_fix = MAX_LOOKS_WITHOUT_FIX
    drv.tick(0.)
    assert drv.loc is not old and not drv.loc.initialized and drv.look_reason == 'relocalize'
    drv.state, drv.relocalizations, drv.looks_without_fix = 'drive', pa.MAX_RELOCALIZE, MAX_LOOKS_WITHOUT_FIX
    drv.tick(1.)
    assert drv.outcome == 'lost'


def test_door_v2_pose_from_arrival_estimate_and_own_beam_views():
    from scripts import run_m2_pair as rm
    st = rm.M2DoorStudent.__new__(rm.M2DoorStudent)
    x0, y0, t0 = .1, -.2, .3                         # arrival estimate
    G = (x0 + math.cos(t0) * .46 - math.sin(t0) * .02, y0 + math.sin(t0) * .46 + math.cos(t0) * .02)
    phi = t0 + .05                                    # beam axis heading in the world
    xf, yf, tf = .35, -.12, .33                       # true pose after the align
    dx, dy = G[0] - xf, G[1] - yf
    gf = (math.cos(tf) * dx + math.sin(tf) * dy, -math.sin(tf) * dx + math.cos(tf) * dy)
    st.claims = {'at_prestation': {'estimate': [x0, y0, t0]}}
    st.vo_obs = [{'g': [.46, .02], 'h': .05, 'moved_before': False},
                 {'g': list(gf), 'h': phi - tf, 'moved_before': True}]
    x, y, t = st._vo_pose()
    assert math.isclose(x, xf, abs_tol=1e-9) and math.isclose(y, yf, abs_tol=1e-9) and math.isclose(t, tf, abs_tol=1e-9)
