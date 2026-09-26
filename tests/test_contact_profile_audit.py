"""Tests for the cargo_noslip_v1 side-effect audit (scripts/audit_contact_profiles.py).

The audit itself must not change contact profiles or defaults, so these tests
also pin the invariants it relies on: the profile stays opt-in, its only solver
change is ``noslip_iterations``, and the scenario variant used by the audit
(``zone_wide_door``, which PR #167 did not pin) differs by nothing else.

The pre-registered comparison (``compare``) is a pure function, so the gate
arithmetic is tested without a simulator.
"""
import copy
import math

import pytest

from scripts import audit_contact_profiles as audit

BASE_SEGMENT = {'label': 'straight_1', 'action': {'kind': 'mecanum', 'forward': .10, 'left': 0., 'turn': 0.,
                                                 'duration_s': 1.},
                'displacement_mm': 100., 'yaw_change_deg': 0., 'grip_xyz_m': [1., 2., .18],
                'joints_deg': {'arm_yaw': 0., 'shoulder': 114., 'elbow': -69., 'wrist_pitch': -73.},
                'pose_after': [1., 2., 0.], 'wall_contacts_now': 0}


def _drive_metrics(**over):
    metrics = {'segments': [copy.deepcopy(BASE_SEGMENT)], 'final_xy_m': [1., 2.], 'final_yaw_deg': 0.}
    metrics.update(over)
    return metrics


def _wall_metrics(**over):
    metrics = {'max_penetration_mm': .26, 'along_wall_slide_mm': 300., 'escape_mm': 120.}
    metrics.update(over)
    return metrics


def _carry_metrics(**over):
    metrics = {'hold_finger_total_n': {'r1': 10.38}, 'placement_err_mm': 5.5,
               'max_cargo_tilt_deg': .01, 'carrier_spread_change_mm': 1.}
    metrics.update(over)
    return metrics


def _gates(rows):
    return {row['gate']: row['ok'] for row in rows}


# --- registry and contract --------------------------------------------------

def test_every_scenario_is_pre_registered_with_a_tolerance_and_a_question():
    assert set(audit.SCENARIOS) == set(audit.TOLERANCES)
    for name, spec in audit.SCENARIOS.items():
        assert spec['kind'] in ('tape', 'carry')
        assert spec['question'] and spec['metric'], name
        if spec['kind'] == 'carry':
            assert spec['hold_s'] >= 120., 'the pre-registered hold is 120 SIM s'
            assert 'legs' in spec and 'cargo_kind' in spec and 'roles' in spec


def test_pre_registration_covers_the_four_requested_questions():
    questions = ' '.join(spec['question'] for spec in audit.SCENARIOS.values())
    for topic in ('mecanum drive', 'solo box grasp-lift-hold', 'pair beam hold', 'wall/obstacle contact'):
        assert topic in questions


def test_sim_step_cost_is_not_a_metric():
    forbidden = ('wall_s', 'wall_time', 'steps_per_s', 'sim_speed', 'rate_hz', 'realtime')
    for name, tol in audit.TOLERANCES.items():
        for key in tol:
            assert not any(key.startswith(word) or key.endswith(word) for word in forbidden), (name, key)
    assert not any(word in gate for gate in audit.HARD_GATES for word in forbidden)


def test_tapes_are_deterministic_and_respect_the_command_contract():
    from sim.camera_robot_port import validate_raw_action
    for name, spec in audit.SCENARIOS.items():
        if spec['kind'] != 'tape':
            continue
        first, second = spec['tape'](), spec['tape']()
        assert first == second, f'{name}: the A/B tape must be identical for both profiles'
        labels = [label for label, _, _ in first]
        assert len(labels) == len(set(labels)), name
        for _, action, dwell in first:
            validate_raw_action(action, allow_reverse=True, allow_mecanum=True)
            assert dwell >= 0.


def test_wall_tape_presses_then_slides_then_escapes():
    labels = [label for label, _, _ in audit.SCENARIOS['wall_push']['tape']()]
    assert [l for l in labels if l.startswith('press_')] and [l for l in labels if l.startswith('slide_')]
    slide = [a for label, a, _ in audit.SCENARIOS['wall_push']['tape']() if label.startswith('slide_')]
    assert all(a['forward'] > 0 and a['left'] != 0 for a in slide), 'the slide must press and move along the wall'
    escape = [a for label, a, _ in audit.SCENARIOS['wall_push']['tape']() if label.startswith('escape_')]
    assert all(a['forward'] < 0 for a in escape)


def test_unknown_scenario_or_profile_is_refused():
    with pytest.raises(ValueError):
        audit.compare('not_a_scenario', {}, {})
    assert audit.PROFILES == ('local_contact_fine', 'cargo_noslip_v1')


# --- pre-registered gate arithmetic -----------------------------------------

def test_drive_gate_uses_the_larger_of_absolute_and_relative_tolerance():
    base = _drive_metrics()
    same = _gates(audit.compare('drive_commands', base, _drive_metrics()))
    assert all(same.values())
    # 1.9 mm on a 100 mm command: inside the 2.0 mm floor.
    seg = copy.deepcopy(BASE_SEGMENT); seg['displacement_mm'] = 101.9
    assert _gates(audit.compare('drive_commands', base, _drive_metrics(segments=[seg])))['disp::straight_1']
    seg = copy.deepcopy(BASE_SEGMENT); seg['displacement_mm'] = 102.1
    assert not _gates(audit.compare('drive_commands', base, _drive_metrics(segments=[seg])))['disp::straight_1']
    # 1% dominates for a long command.
    big = copy.deepcopy(BASE_SEGMENT); big['displacement_mm'] = 1000.
    grown = copy.deepcopy(big); grown['displacement_mm'] = 1009.
    assert _gates(audit.compare('drive_commands', _drive_metrics(segments=[big]),
                                _drive_metrics(segments=[grown])))['disp::straight_1']


def test_drive_gate_catches_heading_and_cumulative_pose_drift():
    base = _drive_metrics()
    seg = copy.deepcopy(BASE_SEGMENT); seg['yaw_change_deg'] = .6
    assert not _gates(audit.compare('drive_commands', base, _drive_metrics(segments=[seg])))['yaw::straight_1']
    far = _drive_metrics(final_xy_m=[1.011, 2.])
    assert not _gates(audit.compare('drive_commands', base, far))['final_xy']
    near = _drive_metrics(final_xy_m=[1.009, 2.])
    assert _gates(audit.compare('drive_commands', base, near))['final_xy']
    assert not _gates(audit.compare('drive_commands', base, _drive_metrics(final_yaw_deg=.6)))['final_yaw']


def test_a_b_comparison_refuses_mismatched_command_tapes():
    other = copy.deepcopy(BASE_SEGMENT); other['label'] = 'straight_2'
    with pytest.raises(ValueError):
        audit.compare('drive_commands', _drive_metrics(), _drive_metrics(segments=[other]))
    arm = {'segments': [copy.deepcopy(BASE_SEGMENT)]}
    arm_other = {'segments': [other]}
    with pytest.raises(ValueError):
        audit.compare('arm_sweep', arm, arm_other)


def test_wall_gates_cover_penetration_slide_and_escape():
    base = _wall_metrics()
    assert all(_gates(audit.compare('wall_push', base, _wall_metrics())).values())
    # 10% of a 300 mm slide = 30 mm.
    assert _gates(audit.compare('wall_push', base, _wall_metrics(along_wall_slide_mm=271.)))['along_wall_slide']
    assert not _gates(audit.compare('wall_push', base, _wall_metrics(along_wall_slide_mm=269.)))['along_wall_slide']
    assert not _gates(audit.compare('wall_push', base, _wall_metrics(max_penetration_mm=1.)))['max_penetration']
    # Escape is both a difference gate and an absolute floor: sticking to the wall fails.
    stuck = _gates(audit.compare('wall_push', _wall_metrics(escape_mm=10.), _wall_metrics(escape_mm=10.)))
    assert not stuck['escape_floor_baseline'] and not stuck['escape_floor_candidate']


def test_idle_gate_requires_both_profiles_to_keep_resting_bodies_still():
    base = {'body_drift_mm': {'red': .1, 'team_beam': .2}}
    assert all(_gates(audit.compare('idle_settle', base, copy.deepcopy(base))).values())
    moved = {'body_drift_mm': {'red': .9, 'team_beam': .2}}
    rows = _gates(audit.compare('idle_settle', base, moved))
    assert not rows['drift_abs::red'] and not rows['drift_diff::red']
    assert rows['drift_abs::team_beam'] and rows['drift_diff::team_beam']


def test_arm_gate_compares_grip_point_and_joint_angles():
    base = {'segments': [copy.deepcopy(BASE_SEGMENT)]}
    assert all(_gates(audit.compare('arm_sweep', base, copy.deepcopy(base))).values())
    moved = copy.deepcopy(BASE_SEGMENT); moved['grip_xyz_m'] = [1.0011, 2., .18]
    assert not _gates(audit.compare('arm_sweep', base, {'segments': [moved]}))['grip::straight_1']
    turned = copy.deepcopy(BASE_SEGMENT); turned['joints_deg']['elbow'] = -68.7
    rows = _gates(audit.compare('arm_sweep', base, {'segments': [turned]}))
    assert not rows['joint::straight_1::elbow'] and rows['joint::straight_1::shoulder']


def test_carry_gates_compare_grip_force_placement_tilt_and_formation():
    base = _carry_metrics()
    assert all(_gates(audit.compare('solo_carry', base, _carry_metrics())).values())
    assert not _gates(audit.compare('solo_carry', base,
                                    _carry_metrics(hold_finger_total_n={'r1': 11.5})))['finger_force::r1']
    assert not _gates(audit.compare('solo_carry', base, _carry_metrics(placement_err_mm=11.)))['placement']
    pair = audit.compare('pair_beam_hold', _carry_metrics(hold_finger_total_n={'r1': 10.7, 'r2': 10.7}),
                         _carry_metrics(hold_finger_total_n={'r1': 10.7, 'r2': 10.7},
                                        max_cargo_tilt_deg=1.5, carrier_spread_change_mm=7.))
    rows = _gates(pair)
    assert not rows['cargo_tilt'] and not rows['formation_spread']
    # Slip itself is the intended effect of the profile, never a side-effect gate.
    assert not any('slip' in row['gate'] for row in pair)


def test_solo_hold_load_has_no_placement_gate():
    rows = _gates(audit.compare('solo_hold_load', _carry_metrics(placement_err_mm=None),
                                _carry_metrics(placement_err_mm=None)))
    assert 'placement' not in rows and 'finger_force::r1' in rows


# --- helpers ----------------------------------------------------------------

def test_hard_gates_flag_weld_divergence_and_nan():
    ok = audit.hard_gates({'eq_active_max': 0, 'max_qvel_abs': 3.2, 'nan_samples': 0})
    assert all(row['ok'] for row in ok) and len(ok) == 3
    bad = {row['gate']: row['ok'] for row in
           audit.hard_gates({'eq_active_max': 1, 'max_qvel_abs': 500., 'nan_samples': 2})}
    assert bad == {'eq_active_max': False, 'max_qvel_abs': False, 'nan_samples': False}
    missing = {row['gate']: row['ok'] for row in audit.hard_gates({})}
    assert not any(missing.values())


def test_creep_fit_returns_mm_per_minute_and_r2():
    times = [i*.5 for i in range(40)]
    slope, r2 = audit.creep_fit(times, [2.*t for t in times])     # 2 mm/s = 120 mm/min
    assert math.isclose(slope, 120., rel_tol=1e-6) and math.isclose(r2, 1., abs_tol=1e-9)
    flat, flat_r2 = audit.creep_fit(times, [1.] * len(times))
    assert math.isclose(flat, 0., abs_tol=1e-9) and flat_r2 is None
    assert audit.creep_fit([0., 1.], [0., 1.]) == (None, None)


# --- invariants the audit must not break ------------------------------------

def test_audit_adds_no_contact_profile_and_changes_no_default():
    from sim.dispatch_contact_profile import PROFILES
    from sim.zone_cargo_contact import CARGO_PROFILES
    assert PROFILES == ('legacy', 'global_noslip', 'local_contact', 'local_contact_fine')
    assert set(CARGO_PROFILES) == {'cargo_noslip_v1'}
    assert CARGO_PROFILES['cargo_noslip_v1']['option'] == {'noslip_iterations': '10'}
    assert CARGO_PROFILES['cargo_noslip_v1']['base'] == 'local_contact_fine'


def test_profile_selection_is_explicit_in_the_audit_cli():
    import argparse
    with pytest.raises(SystemExit):
        audit.main(['--scenario', 'idle_settle', '--output', '/tmp/should-not-exist'])
    parser = argparse.ArgumentParser()
    assert isinstance(parser, argparse.ArgumentParser)


def test_audit_scene_variant_differs_only_by_the_noslip_option():
    """zone_wide_door (the audit variant) was not pinned by PR #167."""
    pytest.importorskip('mujoco')
    import xml.etree.ElementTree as ET
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo_contact import profile_record
    from sim.zone_cargo_scene import CargoZoneScene
    worlds = {}
    for profile in audit.PROFILES:
        scene = CargoZoneScene.from_cargo_config(audit.VARIANT, 11, cargo=[], goal=audit.GOAL,
                                                 contact_profile=profile)
        worlds[profile] = (scene, MultiMasterPiProductionV2(
            seed=11, width=64, height=48, render=False, warehouse_layout=scene.engine_layout,
            warehouse_cargo_ids=None, xml_transform=scene.transform))
    base_scene, base = worlds['local_contact_fine']
    noslip_scene, noslip = worlds['cargo_noslip_v1']
    assert base.model.opt.noslip_iterations == 0 and noslip.model.opt.noslip_iterations == 10
    assert base.model.opt.timestep == noslip.model.opt.timestep == pytest.approx(.00025)
    assert base.model.opt.impratio == noslip.model.opt.impratio
    assert base.model.opt.cone == noslip.model.opt.cone
    a, b = ET.fromstring(base.scene_xml), ET.fromstring(noslip.scene_xml)
    b.find('option').attrib.pop('noslip_iterations')
    assert ET.tostring(a) == ET.tostring(b)
    assert noslip_scene.manifest['cargo_contact_profile']['sha256'] == profile_record('cargo_noslip_v1')['sha256']
    assert 'cargo_contact_profile' not in base_scene.manifest
    assert not noslip.data.eq_active.any() and not base.data.eq_active.any()
