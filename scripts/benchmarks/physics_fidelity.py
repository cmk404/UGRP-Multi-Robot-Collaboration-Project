#!/usr/bin/env python3
"""Audit whether the current MasterPi simulation is suitable for transfer learning.

This is a platform fidelity gate, not a research metric.  A green software test
suite only proves code consistency; this audit checks structural properties that
must be true before a policy trained in SIM can plausibly transfer to MasterPi.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco

from harness.real_geometry import DEFAULT_BLOCK_HEIGHT_M, LINK_2_CM, LINK_3_CM
from sim.masterpi_camera_profile import CAMERA_FX_PX, CAMERA_FY_PX, CAMERA_CX_PX, CAMERA_CY_PX
from sim.masterpi_physics import XML as PRODUCTION_XML
from sim.masterpi_dynamics_v2 import XML as V2_XML, OFFICIAL_TOTAL_MASS_KG
from sim.grasp_physics import XML as LEGACY_XML

CALIBRATION_FILE = ROOT / 'sim/masterpi_dynamics_calibration.json'


def _id(model, kind, name: str) -> int:
    return mujoco.mj_name2id(model, kind, name)


def _geom_contact(model, name: str) -> tuple[int, int]:
    gid = _id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if gid < 0:
        return (-1, -1)
    return int(model.geom_contype[gid]), int(model.geom_conaffinity[gid])


def _joint_names(model) -> list[str]:
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or f'joint#{i}' for i in range(model.njnt)]


def _robot_body_ids(model) -> list[int]:
    robot = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    out = []
    for bid in range(model.nbody):
        cur = bid
        while cur > 0:
            if cur == robot:
                out.append(bid)
                break
            cur = int(model.body_parentid[cur])
    return out


def _robot_geom_contact_stats(model) -> dict:
    robot = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    under_robot = []
    for gid in range(model.ngeom):
        bid = int(model.geom_bodyid[gid])
        cur = bid
        is_robot = False
        while cur > 0:
            if cur == robot:
                is_robot = True
                break
            cur = int(model.body_parentid[cur])
        if is_robot:
            under_robot.append(gid)
    enabled = [g for g in under_robot if int(model.geom_contype[g]) != 0 and int(model.geom_conaffinity[g]) != 0]
    return {'total': len(under_robot), 'contact_enabled': len(enabled), 'fraction': len(enabled) / max(1, len(under_robot))}


V2_REQUIRED_COLLISION_GEOMS = (
    'base_lower_collision', 'base_top', 'front_plate', 'rear_cage_collision',
    'wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr',
    'arm_pedestal_collision', 'upper_arm_collision',
    'forearm_collision', 'wrist_collision', 'camera_bracket',
    'left_finger', 'right_finger',
)


def _required_geom_contact_stats(model, names) -> dict:
    missing = []
    disabled = []
    enabled = []
    for name in names:
        gid = _id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid < 0:
            missing.append(name)
            continue
        if int(model.geom_contype[gid]) != 0 and int(model.geom_conaffinity[gid]) != 0:
            enabled.append(name)
        else:
            disabled.append(name)
    total = len(names)
    return {
        'required': total,
        'contact_enabled': len(enabled),
        'fraction': len(enabled) / max(1, total),
        'missing': missing,
        'disabled': disabled,
        'enabled': enabled,
    }


def _block_side(model, geom_name: str) -> float | None:
    gid = _id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if gid < 0:
        return None
    # For box geoms, geom_size stores half extents.
    return float(2.0 * model.geom_size[gid][0])


def _check(checks: list[dict], key: str, ok: bool, severity: str, measured, required, note: str) -> None:
    checks.append({
        'id': key,
        'ok': bool(ok),
        'severity': severity,
        'measured': measured,
        'required': required,
        'note': note,
    })


def production_audit() -> dict:
    model = mujoco.MjModel.from_xml_string(PRODUCTION_XML)
    checks: list[dict] = []
    joints = _joint_names(model)

    planar = all(n in joints for n in ('base_x', 'base_y', 'base_yaw'))
    base_bid = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    free_base = False
    for jid in range(model.njnt):
        if int(model.jnt_bodyid[jid]) == base_bid and int(model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_FREE):
            free_base = True
    _check(
        checks, 'base_6dof_dynamics', free_base and not planar, 'critical',
        {'free_joint': free_base, 'direct_planar_joints': planar},
        'free 6-DoF chassis supported by floor contacts',
        'Current base is driven directly in generalized x/y/yaw coordinates; a policy can learn motion unavailable to the real wheeled robot.',
    )

    wheel_names = ('wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr')
    wheel_contact = {name: _geom_contact(model, name) for name in wheel_names}
    wheel_joints = [n for n in joints if 'wheel' in n.lower()]
    wheels_physical = bool(wheel_joints) and all(c[0] != 0 and c[1] != 0 for c in wheel_contact.values())
    _check(
        checks, 'mecanum_wheel_dynamics', wheels_physical, 'critical',
        {'wheel_joints': wheel_joints, 'wheel_contacts': wheel_contact},
        'four driven wheel joints with ground interaction / calibrated reduced mecanum dynamics',
        'The visible mecanum wheels are currently decoration: no wheel joints and their collisions are disabled.',
    )

    robot_collisions = _robot_geom_contact_stats(model)
    _check(
        checks, 'robot_collision_model', robot_collisions['fraction'] >= 0.55, 'critical',
        robot_collisions, '>=55% of simplified robot collision geoms enabled',
        'Most chassis/arm geometry cannot collide, so walls, blocks and self-contact do not constrain motion realistically.',
    )

    finger_contacts = {
        'left': _geom_contact(model, 'left_finger'),
        'right': _geom_contact(model, 'right_finger'),
        'red_block': _geom_contact(model, 'red_block_geom'),
    }
    finger_ok = all(v[0] != 0 and v[1] != 0 for v in finger_contacts.values())
    _check(
        checks, 'gripper_block_contact', finger_ok, 'required', finger_contacts,
        'finger/block contact enabled',
        'This is one of the physically meaningful parts of the current simulator: the red cube moves through MuJoCo contact/friction, not an attach teleport.',
    )

    block_side = _block_side(model, 'red_block_geom')
    _check(
        checks, 'block_scale', block_side is not None and abs(block_side - 0.05) < 1e-6, 'required',
        block_side, 0.05, 'Production scene uses the intended 50 mm cube scale.',
    )

    robot_mass = sum(float(model.body_mass[bid]) for bid in _robot_body_ids(model))
    _check(
        checks, 'robot_mass_parity', abs(robot_mass - 1.10) <= 0.12, 'critical',
        round(robot_mass, 4), '1.10 kg ±0.12 kg',
        'Manufacturer mass is 1.10 kg; visual geoms currently contribute duplicate inferred mass and distort inertia/dynamics.',
    )

    elbow_bid = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'elbow_link')
    wrist_bid = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'wrist_link')
    sim_link2 = float(sum(float(x) ** 2 for x in model.body_pos[elbow_bid]) ** 0.5) if elbow_bid >= 0 else None
    sim_link3 = float(sum(float(x) ** 2 for x in model.body_pos[wrist_bid]) ** 0.5) if wrist_bid >= 0 else None
    arm_geometry_ok = (
        sim_link2 is not None and sim_link3 is not None
        and abs(sim_link2 - LINK_2_CM / 100.0) <= .005
        and abs(sim_link3 - LINK_3_CM / 100.0) <= .005
    )
    _check(
        checks, 'arm_geometry_parity', arm_geometry_ok, 'critical',
        {'sim_link2_m': sim_link2, 'sim_link3_m': sim_link3,
         'real_link2_m': LINK_2_CM / 100.0, 'real_link3_m': LINK_3_CM / 100.0},
        '<=5 mm link-length mismatch',
        'The SIM kinematic chain must reproduce the physical pickup-controller link calibration before arm policies are transferable.',
    )

    cam = _id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'robot_cam')
    intr = list(map(float, model.cam_intrinsic[cam])) if cam >= 0 else None
    expected_intr = [CAMERA_FX_PX, CAMERA_FY_PX, 320.0 - CAMERA_CX_PX, 240.0 - CAMERA_CY_PX]
    camera_ok = intr is not None and all(abs(a - b) <= 1.0 for a, b in zip(intr, expected_intr))
    _check(
        checks, 'camera_intrinsics_parity', camera_ok, 'critical',
        {'sim_intrinsic': intr, 'measured_ugrp1_intrinsic': expected_intr},
        '<=1 px intrinsic mismatch',
        'Camera transfer requires the measured ugrp1 raw-camera focal/principal pixel geometry.',
    )

    direct_base_actuators = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or '').startswith(('p_base_', 'a_base_'))
    ]
    _check(
        checks, 'hardware_command_parity', len(direct_base_actuators) == 0, 'critical',
        direct_base_actuators,
        'wheel/motor commands, not direct world-frame x/y/yaw actuators',
        'Training actions must share the physical command interface; direct Cartesian chassis actuators violate that boundary.',
    )

    critical_failures = [c['id'] for c in checks if c['severity'] == 'critical' and not c['ok']]
    return {
        'name': 'production_masterpi_physics',
        'training_ready': not critical_failures,
        'critical_failures': critical_failures,
        'checks': checks,
    }


def candidate_v2_audit() -> dict:
    model = mujoco.MjModel.from_xml_string(V2_XML)
    checks: list[dict] = []
    joints = _joint_names(model)
    base_bid = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    free_base = any(
        int(model.jnt_bodyid[jid]) == base_bid
        and int(model.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_FREE)
        for jid in range(model.njnt)
    )
    planar = all(n in joints for n in ('base_x', 'base_y', 'base_yaw'))
    _check(checks, 'base_6dof_dynamics', free_base and not planar, 'critical',
           {'free_joint': free_base, 'direct_planar_joints': planar},
           'free 6-DoF chassis supported by floor contacts',
           'v2 uses a free chassis, so z/roll/pitch and contact reactions are physical state rather than deleted coordinates.')

    wheel_names = ('wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr')
    wheel_joint_names = {f'{name}_joint' for name in wheel_names}
    wheel_contacts = {name: _geom_contact(model, name) for name in wheel_names}
    wheel_ok = wheel_joint_names.issubset(set(joints)) and all(a != 0 and b != 0 for a, b in wheel_contacts.values())
    _check(checks, 'mecanum_wheel_structure', wheel_ok, 'critical',
           {'wheel_joints': sorted(wheel_joint_names.intersection(joints)), 'wheel_contacts': wheel_contacts},
           'four wheel joints + enabled wheel/floor contact',
           'v2 has explicit rotating wheels; the unresolved 45-degree passive rollers are represented by a reduced ABAB traction model.')

    # The v2 scene intentionally contains many massless, contact-disabled visual
    # geoms (mecanum roller meshes, boards, servo shells). Counting those in the
    # denominator makes visual detail look like missing collision physics. Audit
    # the canonical simplified collision envelope instead.
    collisions = _required_geom_contact_stats(model, V2_REQUIRED_COLLISION_GEOMS)
    _check(checks, 'robot_collision_model', collisions['fraction'] == 1.0, 'critical', collisions,
           'all canonical simplified chassis/wheel/arm/finger collision geoms enabled',
           'v2 keeps decorative detail non-colliding while requiring the complete simplified physical envelope.')

    robot_mass = sum(float(model.body_mass[bid]) for bid in _robot_body_ids(model))
    _check(checks, 'robot_mass_parity', abs(robot_mass - OFFICIAL_TOTAL_MASS_KG) <= .001, 'critical',
           round(robot_mass, 6), OFFICIAL_TOTAL_MASS_KG,
           'v2 uses an explicit mass budget totaling the manufacturer 1.10 kg; inertia distribution is still provisional until calibrated.')

    elbow_bid = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'elbow_link')
    wrist_bid = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'wrist_link')
    link2 = float(sum(float(x) ** 2 for x in model.body_pos[elbow_bid]) ** .5)
    link3 = float(sum(float(x) ** 2 for x in model.body_pos[wrist_bid]) ** .5)
    arm_ok = abs(link2 - LINK_2_CM / 100.0) <= 1e-6 and abs(link3 - LINK_3_CM / 100.0) <= 1e-6
    _check(checks, 'arm_geometry_parity', arm_ok, 'critical',
           {'sim_link2_m': link2, 'sim_link3_m': link3,
            'real_link2_m': LINK_2_CM / 100.0, 'real_link3_m': LINK_3_CM / 100.0},
           'physical calibrated link lengths',
           'v2 is built from the same link constants used by the current physical pickup FK.')

    cam = _id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'robot_cam')
    intr = list(map(float, model.cam_intrinsic[cam])) if cam >= 0 else None
    expected_intr = [CAMERA_FX_PX, CAMERA_FY_PX, 320.0 - CAMERA_CX_PX, 240.0 - CAMERA_CY_PX]
    camera_ok = intr is not None and all(abs(a - b) <= 1e-3 for a, b in zip(intr, expected_intr))
    _check(checks, 'camera_intrinsics_parity', camera_ok,
           'critical', {'sim_intrinsic': intr, 'measured_ugrp1_intrinsic': expected_intr},
           'measured ugrp1 raw-camera pixel intrinsics',
           'v2 uses the measured physical focal/principal pixel geometry; raw fisheye parity is regression-tested separately.')

    direct_base_actuators = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or '').startswith(('p_base_', 'a_base_'))
    ]
    _check(checks, 'hardware_command_parity', not direct_base_actuators, 'critical', direct_base_actuators,
           'four physical motor commands + PWM-derived arm targets',
           'v2 does not expose direct world-frame chassis x/y/yaw actuators.')

    block_side = _block_side(model, 'red_block_geom')
    _check(checks, 'block_scale_parity',
           block_side is not None and abs(block_side - DEFAULT_BLOCK_HEIGHT_M) <= 1e-6,
           'critical', block_side, DEFAULT_BLOCK_HEIGHT_M,
           'v2 uses the 30 mm cube encoded by the current REAL pickup geometry.')

    try:
        calibration = json.loads(CALIBRATION_FILE.read_text())
    except (OSError, ValueError, TypeError):
        calibration = {}
    validated = calibration.get('validated') is True
    results = calibration.get('results') if isinstance(calibration.get('results'), dict) else {}
    acceptance = calibration.get('acceptance') if isinstance(calibration.get('acceptance'), dict) else {}
    held_out = int(results.get('held_out_trials') or 0)
    metrics = {
        'held_out_trials': held_out,
        'translation_endpoint_mae_m': results.get('translation_endpoint_mae_m'),
        'yaw_endpoint_mae_deg': results.get('yaw_endpoint_mae_deg'),
        'peak_speed_relative_error': results.get('peak_speed_relative_error'),
        'stop_distance_mae_m': results.get('stop_distance_mae_m'),
        'servo_endpoint_mae_deg': results.get('servo_endpoint_mae_deg'),
        'servo_settle_time_relative_error': results.get('servo_settle_time_relative_error'),
        'grasp_success_rate_gap': results.get('grasp_success_rate_gap'),
    }
    metric_pairs = (
        ('translation_endpoint_mae_m', 'held_out_translation_endpoint_mae_m_max'),
        ('yaw_endpoint_mae_deg', 'held_out_yaw_endpoint_mae_deg_max'),
        ('peak_speed_relative_error', 'held_out_peak_speed_relative_error_max'),
        ('stop_distance_mae_m', 'held_out_stop_distance_mae_m_max'),
        ('servo_endpoint_mae_deg', 'held_out_servo_endpoint_mae_deg_max'),
        ('servo_settle_time_relative_error', 'held_out_servo_settle_time_relative_error_max'),
        ('grasp_success_rate_gap', 'grasp_success_rate_gap_max'),
    )
    metric_ok = held_out >= 20
    for result_key, limit_key in metric_pairs:
        value = results.get(result_key)
        limit = acceptance.get(limit_key)
        metric_ok = metric_ok and isinstance(value, (int, float)) and isinstance(limit, (int, float)) and float(value) <= float(limit)
    calibration_ok = validated and metric_ok
    _check(checks, 'dynamics_calibrated_against_real', calibration_ok, 'critical',
           {'validated_flag': validated, **metrics},
           {'held_out_trials_min': 20, 'acceptance': acceptance},
           'This is intentionally the remaining blocker: v2 force/slip/damping/servo/gripper parameters must be fitted and pass held-out physical trials before learning is allowed.')

    critical_failures = [c['id'] for c in checks if c['severity'] == 'critical' and not c['ok']]
    return {
        'name': 'candidate_masterpi_dynamics_v2',
        'training_ready': not critical_failures,
        'critical_failures': critical_failures,
        'checks': checks,
    }


def legacy_audit() -> dict:
    model = mujoco.MjModel.from_xml_string(LEGACY_XML)
    checks: list[dict] = []
    robot = _id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    mocap = bool(robot >= 0 and int(model.body_mocapid[robot]) >= 0)
    _check(checks, 'legacy_base_not_mocap', not mocap, 'critical', mocap, False,
           'Legacy PPO base is mocap/kinematic and is directly repositioned during reset.')
    side = _block_side(model, 'red_block_geom')
    _check(checks, 'legacy_block_scale', side is not None and abs(side - .05) < 1e-6, 'critical', side, .05,
           'Historical PPO uses a 150 mm cube, triple the current 50 mm target.')
    arm_names = [n for n in _joint_names(model) if n in {'arm_yaw','shoulder','elbow','wrist_pitch','wrist_roll','gripper_pitch'}]
    _check(checks, 'legacy_arm_topology', len(arm_names) == 4, 'critical', arm_names,
           'four arm joints matching current MasterPi model',
           'Historical PPO controls a different six-axis arm topology.')
    env_text = (ROOT / 'sim/grasp_physics.py').read_text() + (ROOT / 'sim/grasp_env.py').read_text()
    hidden_truth = all(token in env_text for token in ('block_vel', '(block-grip)', 'left_force', 'right_force'))
    _check(checks, 'legacy_observation_transferability', not hidden_truth, 'critical', hidden_truth, False,
           'Historical policy observation contains simulator-only block pose/velocity/contact force truth not available on physical MasterPi.')
    critical_failures = [c['id'] for c in checks if c['severity'] == 'critical' and not c['ok']]
    return {
        'name': 'legacy_ppo_physics',
        'training_ready': not critical_failures,
        'critical_failures': critical_failures,
        'checks': checks,
    }


def audit() -> dict:
    prod = production_audit()
    v2 = candidate_v2_audit()
    legacy = legacy_audit()
    return {
        'classification': 'platform_fidelity_gate_not_research_result',
        # The currently deployed production simulator remains the authoritative
        # training gate. Candidate v2 cannot silently turn this green.
        'training_ready': bool(prod['training_ready']),
        'production': prod,
        'candidate_v2': v2,
        'legacy_ppo': legacy,
        'policy': 'Do not start sim-to-real training until the calibrated v2 model replaces production and every production critical failure is empty.',
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--allow-fail', action='store_true', help='report failure but exit 0')
    args = ap.parse_args()
    result = audit()
    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        print('TRAINING READY:', 'YES' if result['training_ready'] else 'NO')
        for section in ('production', 'candidate_v2', 'legacy_ppo'):
            group = result[section]
            print(f"\n[{group['name']}] training_ready={group['training_ready']}")
            for c in group['checks']:
                mark = 'PASS' if c['ok'] else 'FAIL'
                print(f"  {mark:4s} {c['id']}: {c['note']}")
        if not result['training_ready']:
            print('\nBLOCK sim-to-real training until production critical failures are fixed and physically calibrated.')
    return 0 if result['training_ready'] or args.allow_fail else 2


if __name__ == '__main__':
    raise SystemExit(main())
