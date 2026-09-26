"""M1 run: one robot delivers the cyan box from the pickup area through the door to a zone slot on its own camera.

The runner is the physics owner. The controller (``harness.m1_owncam_delivery``)
receives only the robot's own ``robot_cam`` observations and its own issued
commands (plus the static tagged map, fixed calibrations and the order sheet:
colour, pickup area rows, destination slot). No pose stub of any kind: the
M1 contract rejects every non own-camera pose source in the controller, the
judge and the export (Codex review #1). Simulator truth is written to
``eval_only/`` for scoring only. Sync SIM only; weld OFF.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCHEMA = 'ugrp.m1_owncam_run.v3'
FRAME_S = .2
TICK_S = .1
GT_S = .05
SIM_LIMIT_S = 720.
SLOT_HALF_M = .06
ON_FLOOR_MAX_Z_M = .05
# skill -> (module, class, order kind, extra kwargs); v5 runs in its own M1 mode (it rejects non-owncam sources)
SKILLS = {'v4': ('harness.wrist_zone_skill_v4', 'WristZoneDeliveryV4', 'own_rgb_point', {}),
          'v5': ('harness.wrist_zone_skill_v5', 'WristZoneDeliveryV5', 'own_rgb_bay', {'mode': 'm1'}),
          'v6': ('harness.wrist_zone_skill_v6', 'WristZoneDeliveryV6', 'own_rgb_bay', {'mode': 'm1'}),
          'v9': ('harness.wrist_zone_skill_v9', 'WristZoneDeliveryV9', 'own_rgb_bay', {'mode': 'm1'})}
STATIC_KEEPOUT_SKILLS = ('v6', 'v9')     # skills that take the static layout keep-outs (#181 v6+ API)
CONTACT_PROFILES = ('local_contact_fine', 'cargo_noslip_v1')
SPAWN_KEEPOUT_RADIUS_M = .17             # an idle MasterPi footprint (as the #181 v6 runner)
# Evaluation-only retention rule (amendment A5, fixed before the test): skill phases in which the box
# must be held (intended set-downs - reseat_*, release and later - are excluded).
CARRY_PHASES = ('to_carry_posture', 'nav_preplace', 'grip_check', 'pre_release')
RETAIN_MIN_BOX_Z_M = .04                 # on the floor the box centre is at 0.016 m
GRASP_MIN_BOTH_FINGER_FRACTION = .95
# Everything a test run depends on must be unchanged since the frozen source commit (Codex pre-review #1).
FROZEN_PATHS = ('harness', 'scripts', 'sim', 'maps', 'configs', 'experiments/2026-09-26-zone-m1-owncam',
                'experiments/2026-09-26-zone-owncam-loop-v2/calibration_loop_v2.json')
RUNTIME_FILES = ('harness/m1_owncam_delivery.py', 'harness/m1_owncam_contract.py', 'harness/owncam_pose_source.py',
                 'harness/owncam_localizer.py', 'harness/owncam_drive.py', 'harness/owncam_drive_v2.py',
                 'harness/wall_tags.py', 'harness/map_goto.py', 'harness/zone_color_boxes.py',
                 'harness/wrist_zone_skill.py', 'harness/wrist_zone_skill_v2.py', 'harness/wrist_zone_skill_v3.py',
                 'harness/wrist_zone_skill_v4.py', 'harness/wrist_zone_skill_v5.py', 'harness/m1_contract.py',
                 'harness/wrist_zone_skill_v6.py', 'harness/wrist_zone_skill_v7.py', 'harness/wrist_zone_skill_v8.py',
                 'harness/wrist_zone_skill_v9.py', 'harness/owncam_view.py', 'harness/markerless_box.py',
                 'harness/visual_box_skill.py', 'harness/visual_attachment.py',
                 'sim/zone_cargo_contact.py', 'sim/zone_landmarks.py', 'sim/zone_scene.py', 'sim/camera_robot_port.py',
                 'sim/exact_speedups.py', 'sim/physics_drive_kernel.py', 'scripts/run_m1_owncam.py')


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in rows))


def run(spec, out, student, speedups=None):
    """``speedups``: a ``sim.exact_speedups`` set name ('none' = original path, 'exact-v1' = bit-exact CPU cuts)."""
    import importlib

    import cv2
    import mujoco
    import numpy as np
    from harness import m1_contract, m1_owncam_contract
    from harness.m1_owncam_delivery import M1OwnCamDelivery
    from harness.map_goto import plan_path
    from harness.owncam_drive import LOADED_ENVELOPE
    from harness.map_goto import UNLOADED_ENVELOPE
    from harness.wrist_zone_skill import PoseEstimate
    from scripts.record_owncam_localization import LoggingPort
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.exact_speedups import ContactPrefilter, install_drive_kernel
    from sim.exact_speedups import resolve as resolve_speedups
    from sim.research_dispatch_arena import digest
    from sim.zone_arena import LAYOUTS
    from sim.zone_landmarks import TaggedZoneScene

    from sim.zone_cargo_contact import CARGO_PROFILES, base_profile, profile_record
    from sim.zone_cargo_contact import apply as apply_cargo_profile

    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    started, load_start = time.time(), os.getloadavg()
    code = {'sha': git('rev-parse', 'HEAD'), 'dirty': bool(git('status', '--porcelain', '--', *FROZEN_PATHS)),
            'stamped': 'at launch', 'runtime_files_sha256': {f: sha_bytes((ROOT/f).read_bytes()) for f in RUNTIME_FILES}}
    (out/'attempt_started.json').write_text(json.dumps(
        {'episode': spec['episode_id'], 'split': spec['split'], 'code': code, 'student': student,
         'started_unix': round(started, 3), 'note': 'written before physics; a run dir without result.json is an '
         'infrastructure failure (killed / host error)'}, indent=2) + '\n')
    profile = spec['contact_profile']
    if profile not in CONTACT_PROFILES:
        raise ValueError(f'contact profile must be one of {CONTACT_PROFILES}')
    scene = TaggedZoneScene.from_tagged(spec['map'], spec['seed'], spec['goal'], spec.get('extra_boxes'),
                                        contact_profile=base_profile(profile))
    xml_transform = ((lambda xml: apply_cargo_profile(scene.transform(xml), profile)) if profile in CARGO_PROFILES
                     else scene.transform)
    world = MultiMasterPiProductionV2(seed=spec['seed'], width=640, height=480, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=xml_transform)
    scene.setup(world)
    contact_record = {'profile': profile, 'base_profile': base_profile(profile),
                      'cargo_profile': profile_record(profile) if profile in CARGO_PROFILES else None,
                      'noslip_iterations': int(world.model.opt.noslip_iterations),
                      'timestep_s': float(world.model.opt.timestep),
                      'final_scene_xml_sha256': sha_bytes(world.scene_xml.encode()),
                      'user_decision': 'pending user approval (PR #181/#189); cargo_noslip_v1 is the primary condition'}
    if profile == 'cargo_noslip_v1' and contact_record['noslip_iterations'] <= 0:
        raise RuntimeError('cargo_noslip_v1 requested but noslip_iterations is 0 in the built model')
    setup_diag = {}
    if spec.get('box_yaw_deg') is not None:
        # Dev diagnostic only (amendment A2): rotate the cyan box in place (setup-only, never an input).
        if spec['split'] != 'dev':
            raise ValueError('box_yaw_deg is a dev diagnostic setting')
        cyan = next(o for _, o in sorted(scene.config['setup_only']['objects'].items()) if o['kind'] == 'cyan')
        jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, cyan['joint_name'])
        q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
        half = math.radians(float(spec['box_yaw_deg']))/2
        world.data.qpos[q + 3:q + 7] = [math.cos(half), 0, 0, math.sin(half)]
        world.data.qvel[v:v + 6] = 0
        mujoco.mj_forward(world.model, world.data)
        setup_diag = {'box_yaw_deg': float(spec['box_yaw_deg'])}
    static = scene.config['static_map']
    objects = scene.config['setup_only']['objects']
    spawns = scene.config['setup_only']['spawns']
    rid = min(spawns, key=lambda r: abs(spawns[r][1] - spec['spawn_y']))
    slot_xy = next(s['center_m'] for slots in static['zone_slots'].values() for s in slots if s['slot_id'] == spec['slot_id'])
    calibration_path = ROOT/student['calibration']
    calibration = json.loads(calibration_path.read_text())
    module, name, order_kind, skill_kwargs = SKILLS[student['skill']]
    skill_mod = importlib.import_module(module)
    skill_cls = getattr(skill_mod, name)
    keepout_records = []
    if student['skill'] in STATIC_KEEPOUT_SKILLS:
        keepouts = static_layout_keepouts(static)
        skill_kwargs = {**skill_kwargs, 'static_keepouts': keepouts, 'static_bounds_m': static['bounds_m']}
        keepout_records = [k.record() for k in keepouts]
    rows_y = LAYOUTS['zone_wide']['pickup_rows_y']

    ctl_ref = {}

    def planner(start, goal, carrying):
        ctl = ctl_ref['ctl']
        result = plan_path(static, start, goal, LOADED_ENVELOPE if carrying else UNLOADED_ENVELOPE,
                           obstacles=ctl._keepouts(), escape_start_m=.25)
        return None if result is None else [tuple(p) for p in result['waypoints_m'][1:]]

    ctl = M1OwnCamDelivery(static, calibration['params'], box_kind='cyan', slot_id=spec['slot_id'], slot_xy=slot_xy,
                           skill_factory=lambda order: skill_cls(order, planner=planner, robot_id=rid, **skill_kwargs),
                           pose_estimate_cls=PoseEstimate, search_rows_y=rows_y, robot_id=rid, seed=spec['seed'],
                           order_kind=order_kind)
    ctl_ref['ctl'] = ctl
    commands = []

    def sink(row):
        commands.append(row)
        ctl.on_command(row)
    initial = {int(k): int(v) for k, v in world.robot(rid).servo_command_pulses.items()}
    sink({'t': 0.0, 'kind': 'initial_servo_command', 'pulses': dict(initial)})
    raw = {r: CameraRobotPort(world, r, allow_reverse=True, allow_mecanum=True) for r in ('r1', 'r2', 'r3')}
    port = LoggingPort(raw[rid], sink)
    box = next(oid for oid, o in sorted(objects.items()) if o['kind'] == 'cyan')
    box_body = objects[box]['body_name']
    model, data = world.model, world.data
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(model.ngeom)]
    wall_geoms = {g for g, n in enumerate(names) if n.startswith('zone_wall_')}
    own_geoms = {g for g, n in enumerate(names) if n.startswith(rid + '__')}
    peer_geoms = {g for g, n in enumerate(names) if n.startswith(('r1__', 'r2__', 'r3__')) and g not in own_geoms}
    box_geom = {g for g, n in enumerate(names) if n == box_body + '_geom'}
    other_boxes = {g for g, n in enumerate(names) if n.startswith('cargo_box_') and g not in box_geom}
    left_finger = {g for g, n in enumerate(names) if n == rid + '__left_finger'}
    right_finger = {g for g, n in enumerate(names) if n == rid + '__right_finger'}
    speedup_set, speedup_items = resolve_speedups(speedups)
    speedup_record = {'set': speedup_set, 'items': list(speedup_items),
                      'note': 'execution infrastructure only; same trajectory/commands/frames (sim.exact_speedups)'}
    if 'drive_kernel' in speedup_items:
        try:
            speedup_record['drive_kernel'] = install_drive_kernel(world)
        except BaseException:
            world.close()          # a foreign kernel in the hook: refuse the run, release the renderer
            raise
    prefilter = ContactPrefilter(model.ngeom, own_geoms | box_geom) if 'contact_prefilter' in speedup_items else None
    force6 = np.zeros(6)
    retention = {'carry_steps': 0, 'both_finger_steps': 0, 'min_box_z_m': None, 'low_box_steps': 0,
                 'max_box_penetration_m': 0., 'max_box_normal_force_n': 0., 'kind_steps': {}}
    retention_log, window = [], {}
    frames_dir = out/'frames'
    frames_dir.mkdir()
    frames, frame_eval, gt, contacts, decisions = [], [], [], [], []
    state = {'next_frame': 0., 'next_gt': 0., 'max_eq_active': 0, 'phase_times': {}}

    def truth():
        r = world.robot(rid)
        xyz, rpy = r.base_xyz(), r.base_rpy()
        return float(xyz[0]), float(xyz[1]), float(rpy[2])

    def capture(now):
        obs = port.capture()
        jpeg = base64.b64decode(obs['image'])
        rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        index = len(frames)
        (frames_dir/f'{index:05d}.jpg').write_bytes(jpeg)
        report = ctl.on_frame(now, obs, rgb)
        frames.append({'frame': index, 't': round(now, 4), 'file': f'frames/{index:05d}.jpg', 'frame_id': obs['frame_id'],
                       'sha256': obs['sha256'], 'camera': obs['camera'], 'phase': ctl.phase,
                       'skill_phase': getattr(ctl.skill, 'phase', None),
                       'commanded_servo': obs['actuator_state']['servo_pulses'], 'report': report.as_dict()})
        x, y, yaw = truth()
        row = {'frame': index, 't': round(now, 4), 'gt': [round(x, 5), round(y, 5), round(yaw, 6)],
               'box_xyz': [round(float(v), 4) for v in data.body(box_body).xpos], 'phase': ctl.phase,
               'skill_phase': getattr(ctl.skill, 'phase', None)}
        if report.initialized:
            row.update(pos_err_m=round(math.hypot(report.x_m - x, report.y_m - y), 5),
                       yaw_err_deg=round(abs(math.degrees((report.yaw_rad - yaw + math.pi) % (2*math.pi) - math.pi)), 4),
                       std_xy_m=round(report.std_xy_m, 5))
        frame_eval.append(row)
        state['next_frame'] = now + FRAME_S
        return obs

    def physics(seconds):
        n = max(1, round(seconds/float(model.opt.timestep)))
        for _ in range(n):
            now = float(data.time)
            for p in raw.values():
                p.tick(now)
            world._physics_step_for(world.controllers[rid])
            now = float(data.time)
            kinds_now, lf, rf = set(), False, False
            for i in (range(data.ncon) if prefilter is None else prefilter.indices(data)):
                c = data.contact[i]
                pair = {int(c.geom1), int(c.geom2)}
                mine = pair & (own_geoms | box_geom)
                if not mine:
                    continue
                if pair & box_geom:
                    lf, rf = lf or bool(pair & left_finger), rf or bool(pair & right_finger)
                    retention['max_box_penetration_m'] = max(retention['max_box_penetration_m'], -float(c.dist))
                    mujoco.mj_contactForce(model, data, i, force6)
                    retention['max_box_normal_force_n'] = max(retention['max_box_normal_force_n'], float(force6[0]))
                other = next(iter(pair - mine), None)
                kind = ('wall' if other in wall_geoms else 'peer_robot' if other in peer_geoms and mine & own_geoms else
                        'other_box' if other in other_boxes else None)
                if kind:
                    kinds_now.add(kind)
                if kind and (not contacts or contacts[-1]['t'] < now - .1 or contacts[-1]['kind'] != kind):
                    contacts.append({'t': round(now, 4), 'kind': kind, 'dist_m': round(float(c.dist), 5),
                                     'geoms': sorted(names[g] for g in pair), 'phase': ctl.phase})
            for kind in kinds_now:              # per physics step, not the 0.1 s de-duplicated log
                retention['kind_steps'][kind] = retention['kind_steps'].get(kind, 0) + 1
            skill_phase = getattr(ctl.skill, 'phase', None)
            if skill_phase in CARRY_PHASES:
                bz = float(data.body(box_body).xpos[2])
                retention['carry_steps'] += 1
                retention['both_finger_steps'] += int(lf and rf)
                retention['low_box_steps'] += int(bz < RETAIN_MIN_BOX_Z_M)
                retention['min_box_z_m'] = bz if retention['min_box_z_m'] is None else min(retention['min_box_z_m'], bz)
                w = window.setdefault('w', {'t0': now, 'phase': skill_phase, 'steps': 0, 'both': 0, 'zmin': bz})
                w['steps'] += 1; w['both'] += int(lf and rf); w['zmin'] = min(w['zmin'], bz)
                if now - w['t0'] >= GT_S or skill_phase != w['phase']:
                    retention_log.append({'t': round(w['t0'], 4), 'phase': w['phase'], 'steps': w['steps'],
                                          'both_finger_steps': w['both'], 'min_box_z_m': round(w['zmin'], 5)})
                    window.pop('w')
            if len(data.eq_active):
                state['max_eq_active'] = max(state['max_eq_active'], int(data.eq_active.max()))
            if now + 1e-9 >= state['next_gt']:
                x, y, yaw = truth()
                gt.append({'t': round(now, 4), 'x': round(x, 5), 'y': round(y, 5), 'yaw': round(yaw, 6),
                           'box_xyz': [round(float(v), 4) for v in data.body(box_body).xpos]})
                state['next_gt'] = now + GT_S
            if now + 1e-9 >= state['next_frame']:
                capture(now)

    def apply(action):
        port.apply(action, float(data.time))

    def execute_macro(action, obs):
        """Skill macros (as scripts/run_zone_owncam_skill.py executes them), frames at 5 Hz throughout."""
        kind = action['kind']
        if kind == 'drive':
            apply({'kind': 'drive', 'forward': action['fwd'], 'turn': action['turn'], 'duration_s': action['duration']})
            physics(action['duration'] + .2)
            port.hold(float(data.time))
        elif kind == 'mecanum':
            apply({'kind': 'mecanum', 'forward': action['forward'], 'left': action['left'], 'turn': action['turn'],
                   'duration_s': action['duration']})
            physics(action['duration'] + .1)
            port.hold(float(data.time))
        elif kind == 'pose':
            targets = action['pulses']
            start = obs['actuator_state']['servo_pulses']
            delta = max(abs(p - start[str(s)]) for s, p in targets.items())
            duration = max(.25, delta/600.)
            count = max(5, math.ceil(duration/.05))
            for sample in range(1, count + 1):
                u = sample/count
                ease = u*u*(3 - 2*u)
                for servo, end in targets.items():
                    pulse = round(start[str(servo)] + ease*(end - start[str(servo)]))
                    apply({'kind': 'look', 'pan_pulse': pulse} if int(servo) == 6 else
                          {'kind': 'arm', 'servo_id': int(servo), 'pulse': pulse})
                physics(duration/count)
            physics(.3 if ctl.skill is not None and ctl.skill.phase == 'grasp' and ctl.skill.box.phase == 'approach' else .15)
        elif kind == 'wait':
            apply({'kind': 'wait'})
            physics(max(.05, action['duration']))
        else:
            raise ValueError('UNKNOWN_MACRO')

    physics(.5)
    outcome, exception = None, None
    try:
        while True:
            now = float(data.time)
            if now > SIM_LIMIT_S:
                outcome = 'SIM_LIMIT'
                break
            state['phase_times'].setdefault(f'{ctl.phase}:{getattr(ctl.skill, "phase", "")}', round(now, 2))
            decision = ctl.decide(now)
            mode = decision['mode']
            if mode == 'done':
                outcome = decision['outcome']
                break
            if mode == 'capture':
                capture(now)
                continue
            if mode == 'tick':
                for cmd in decision['commands']:
                    if cmd['kind'] == 'hold':
                        port.hold(now)
                    else:
                        apply(cmd)
                physics(TICK_S)
            elif mode == 'macro':
                decisions.append({'t': round(now, 3), 'skill_phase': ctl.skill.phase, 'action': decision['action']})
                execute_macro(decision['action'], ctl.last_obs)
                capture(float(data.time))
    except Exception as exc:                 # noqa: BLE001 - a controller/skill exception is a FAILED episode
        import traceback
        exception = {'type': type(exc).__name__, 'message': str(exc)[:2000], 'traceback': traceback.format_exc()[-6000:]}
        outcome = f'EXCEPTION:{type(exc).__name__}'
    port.hold(float(data.time))
    physics(.5)                       # settle before the final truth sample
    summary = ctl.summary()
    bx, by, bz = (float(v) for v in data.body(box_body).xpos)
    gt_in_slot = abs(bx - slot_xy[0]) <= SLOT_HALF_M and abs(by - slot_xy[1]) <= SLOT_HALF_M and bz < ON_FLOOR_MAX_Z_M
    placement = summary['skill_placement'] or {}
    claim = placement.get('reason') == 'IN_SLOT'
    gates = summary.get('lookback_gates') or []
    gate = next((g for g in gates if g.get('frame_id') == placement.get('frame_id')), {})
    wall_contacts = retention['kind_steps'].get('wall', 0)
    carry_steps = retention['carry_steps']
    grasp_physical = bool(carry_steps) and retention['both_finger_steps'] >= GRASP_MIN_BOTH_FINGER_FRACTION*carry_steps
    box_retained = bool(carry_steps) and retention['low_box_steps'] == 0
    judged = m1_owncam_contract.judge(
        pose_sources=summary['pose_sources'], skill_reason=outcome, skill_claim_in_slot=claim,
        gt_box_in_slot=gt_in_slot, wall_contacts=wall_contacts, weld_used=state['max_eq_active'] > 0,
        face_fallback_used=summary['face_fallback_used'], pickup_source=summary['pickup_source'] or 'none',
        within_limit=outcome != 'SIM_LIMIT',
        extra_checks={'look_back_pose_gate_ok': bool(gate) and not gate.get('violations'),
                      'no_exception': exception is None,
                      'grasp_physical_both_fingers': grasp_physical, 'box_retained_in_carry': box_retained})
    target = summary['target_xy']
    box0 = objects[box]['position_m']
    input_contract = ('own robot_cam JPEG + own issued commands + static tagged map v2 + fixed calibrations + '
                      'order sheet (cyan, pickup area rows, destination slot); pickup point/bay from own RGB search; '
                      'no pose stub; GT only in eval_only/')
    # PR #181 outcome block (their contract) next to this runner's stricter M1 checks.
    skill_summary = summary.get('skill_summary') or {}
    outcome_block = m1_contract.outcome_fields(
        mode='m1', pose_sources_seen=summary['pose_sources'], diagnostic_success=judged['diagnostic_success'],
        input_contract={'text': input_contract}, cameras_seen=skill_summary.get('cameras_seen') or ['robot_cam'])
    strict_m1 = bool(judged['m1_success'] and outcome_block['m1_success'])
    result = {'schema': SCHEMA, 'episode': spec['episode_id'], 'split': spec['split'], 'robot_id': rid,
              'outcome': outcome, **outcome_block, **judged, 'm1_success': strict_m1, 'success': strict_m1,
              'counts_as_m1': bool(judged['counts_as_m1'] and outcome_block['counts_as_m1']),
              'input_contract': input_contract,
              'evaluation_only': {'gt_box_final_xyz': [round(bx, 4), round(by, 4), round(bz, 4)], 'setup_diagnostic': setup_diag,
                                  'slot_xy': slot_xy, 'gt_box_in_slot': gt_in_slot,
                                  'search_target_error_m': None if target is None else
                                  round(math.hypot(target[0] - box0[0], target[1] - box0[1]), 4),
                                  'wall_contacts': wall_contacts,
                                  'contacts': {k: sum(1 for c in contacts if c['kind'] == k)
                                               for k in ('wall', 'peer_robot', 'other_box')},
                                  'max_eq_active': state['max_eq_active'],
                                  'contact_steps_per_physics_step': retention['kind_steps'],
                                  'retention': {**retention, 'rule': {'carry_phases': CARRY_PHASES,
                                                'min_box_z_m': RETAIN_MIN_BOX_Z_M,
                                                'min_both_finger_fraction': GRASP_MIN_BOTH_FINGER_FRACTION}},
                                  'contact_profile': contact_record},
              'sim_s': round(float(data.time), 2), 'looks': summary['looks'], 'placement': placement,
              'lookback_gate': gate, 'lookback_gates': gates, 'exception': exception,
              'static_keepouts': keepout_records, 'controller': summary, 'phase_times': state['phase_times'],
              'commands': len(commands), 'frames': len(frames)}
    m1_owncam_contract.assert_exportable(result)
    m1_contract.validate_outcome(result)
    jsonl(out/'inputs'/'commands.jsonl', commands)
    jsonl(out/'inputs'/'frames.jsonl', frames)
    jsonl(out/'controller_events.jsonl', ctl.events)
    jsonl(out/'skill_events.jsonl', getattr(ctl.skill, 'events', []) or [])
    jsonl(out/'macros.jsonl', decisions)
    jsonl(out/'eval_only'/'frames_eval.jsonl', frame_eval)
    jsonl(out/'eval_only'/'gt_trajectory.jsonl', gt)
    jsonl(out/'eval_only'/'contacts.jsonl', contacts)
    jsonl(out/'eval_only'/'retention.jsonl', retention_log)
    (out/'scene.xml').write_text(world.scene_xml)
    manifest = {'schema': SCHEMA, 'spec': spec, 'student': student, 'code': code, 'map_id': spec['map'],
                'static_map_sha256': digest(static), 'landmarks_sha256': scene.manifest['landmarks_sha256'],
                'scene_xml_sha256': scene.manifest['scene_xml_sha256'],
                'calibration_sha256': sha_bytes(calibration_path.read_bytes()), 'pose_source': ctl.pose.source,
                'weld': scene.manifest['weld'], 'contact_profile': contact_record,
                'timestep_s': float(model.opt.timestep), 'frame_period_s': FRAME_S, 'tick_s': TICK_S, 'sync_sim': True,
                'speedups': speedup_record,
                'env': {'python': platform.python_version(), 'platform': platform.platform(),
                        'mujoco': mujoco.__version__, 'opencv': cv2.__version__, 'numpy': np.__version__,
                        'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                                   'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
                'load_average': {'start': [round(v, 2) for v in load_start], 'end': [round(v, 2) for v in os.getloadavg()]},
                'wall_s': round(time.time() - started, 1)}
    manifest['files'] = {str(p.relative_to(out)): sha_bytes(p.read_bytes())
                         for p in sorted(out.rglob('*')) if p.is_file() and p.suffix in ('.jsonl', '.xml')}
    (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, default=str) + '\n')
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    world.close()
    return result, manifest


def static_layout_keepouts(static):
    """Every idle-spawn spot of the tagged map's base layout as #181 StaticKeepout discs (never a live pose).

    The class is v6's: v7-v9 inherit v6's isinstance check and v9 does not re-export it
    (dev-a7 crashed at setup on ``skill_mod.StaticKeepout``).
    """
    from harness.wrist_zone_skill_v6 import StaticKeepout
    from sim.zone_arena import layout
    arena = layout(static['base_map']['map_id'])       # zone_wide_door
    return tuple(StaticKeepout(f'spawn_row_{i}', (float(arena['spawn_x']), float(y)), SPAWN_KEEPOUT_RADIUS_M,
                               'static_layout_idle_spawn') for i, y in enumerate(arena['spawn_rows_y']))


def effective_student(prereg_path: Path, prereg: dict) -> dict:
    """The registered student block with the recorded amendments applied in order (never silently)."""
    return effective_prereg(prereg_path, prereg)[0]


def effective_prereg(prereg_path: Path, prereg: dict) -> tuple[dict, list]:
    """(student, episodes): amendments patch the student and may ADD dev episodes only."""
    student, episodes = dict(prereg['student']), list(prereg['episodes'])
    amend_path = prereg_path.parent/'prereg_amendments.json'
    applied = []
    if amend_path.exists():
        for a in json.loads(amend_path.read_text())['amendments']:
            if a.get('student_patch'):
                student.update(a['student_patch'])
            for e in a.get('add_episodes', []):
                if e['split'] != 'dev' or any(x['episode_id'] == e['episode_id'] for x in episodes):
                    raise ValueError(f"amendment {a['id']} may only add new dev episodes")
                episodes.append(e)
            applied.append(a['id'])
        student['amendments_applied'] = applied
        student['amendments_sha256'] = sha_bytes(amend_path.read_bytes())
    return student, episodes


def check_frozen(frozen_path: Path, prereg_path: Path, student: dict) -> dict:
    """Refuse a test launch unless the tree is clean and every run input equals the frozen source."""
    if not frozen_path.is_file():
        raise SystemExit(f'test refused: no frozen source file {frozen_path}')
    frozen = json.loads(frozen_path.read_text())
    dirty = git('status', '--porcelain', '--', *FROZEN_PATHS)
    if dirty:
        raise SystemExit(f'test refused: uncommitted changes under {FROZEN_PATHS}:\n{dirty}')
    sha = frozen['source_sha']
    changed = git('diff', '--name-only', sha, 'HEAD', '--', *FROZEN_PATHS).split()
    allowed = set(frozen.get('records_only_paths', []))
    bad = [f for f in changed if f not in allowed]
    if bad:
        raise SystemExit(f'test refused: files changed since frozen source {sha[:9]}: {bad}')
    want = frozen['sha256']
    have = {f: sha_bytes((ROOT/f).read_bytes()) for f in want}
    diff = [f for f in want if have[f] != want[f]]
    if diff:
        raise SystemExit(f'test refused: hash mismatch vs frozen_source.json: {diff}')
    for key in ('skill', 'calibration', 'contact_profile'):
        if frozen['student'].get(key) != student.get(key):
            raise SystemExit(f'test refused: student {key} {student.get(key)!r} != frozen {frozen["student"].get(key)!r}')
    return {'frozen_source_sha': sha, 'frozen_file': str(frozen_path.relative_to(ROOT)),
            'frozen_file_sha256': sha_bytes(frozen_path.read_bytes()), 'head': git('rev-parse', 'HEAD')}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--prereg', required=True)
    p.add_argument('--only', default='')
    p.add_argument('--output', required=True)
    p.add_argument('--split', choices=('dev', 'test'), default='dev',
                   help='dev by default; the one-shot test split needs --split test and --frozen')
    p.add_argument('--frozen', default='', help='frozen_source.json (required for --split test)')
    p.add_argument('--speedups', default='none', choices=('none', 'exact-v1'),
                   help='bit-exact CPU speedups (sim.exact_speedups); recorded in manifest.json')
    args = p.parse_args(argv)
    prereg_path = Path(args.prereg).resolve()
    prereg = json.loads(prereg_path.read_text())
    student, episodes = effective_prereg(prereg_path, prereg)
    freeze = None
    if args.split == 'test':
        if not args.frozen:
            raise SystemExit('test refused: --frozen experiments/.../frozen_source.json is required')
        freeze = check_frozen(Path(args.frozen).resolve(), prereg_path, student)
        student = {**student, 'freeze': freeze}
    only = {s for s in args.only.split(',') if s}
    selected = [e for e in episodes if e['split'] == args.split and (not only or e['episode_id'] in only)]
    unknown = only - {e['episode_id'] for e in episodes if e['split'] == args.split}
    if unknown:
        raise SystemExit(f'--only names episodes outside the {args.split} split: {sorted(unknown)}')
    for spec in selected:
        spec = {**spec, 'contact_profile': student.get('contact_profile', spec.get('contact_profile'))}
        extra = {} if args.speedups == 'none' else {'speedups': args.speedups}     # 'none' = the unchanged call
        result, manifest = run(spec, Path(args.output)/spec['episode_id'], student, **extra)
        print(json.dumps({'episode': spec['episode_id'], 'outcome': result['outcome'], 'm1_success': result['m1_success'],
                          'failed': result['m1_failed_checks'], 'diagnostic_success': result['diagnostic_success'],
                          'false_success': result['false_success'], 'sim_s': result['sim_s'], 'looks': result['looks'],
                          'wall_s': manifest['wall_s'], 'load': manifest['load_average']}), flush=True)


if __name__ == '__main__':
    main()
