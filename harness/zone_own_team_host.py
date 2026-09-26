"""Package F multi-robot physics owner: one MuJoCo world, three robots, one own-camera executor each.

Split from ``harness/zone_own_executor.py`` (issue #221). The host owns physics and rendering. Each
executor receives exactly its own port's ``robot_cam`` observations and its own issued-command rows.
Everything read from the simulator (poses, box positions, contacts, equality constraints) goes to
``self.eval_only`` and is written under ``eval_only/`` by the caller.

One cancel path (Codex review 2 of PR #206, P1-4/P1-5): an accepted ``abort``, a job's local SIM
deadline (checked by the host even while a macro runs, and scheduled as a wake-up time), the
episode end and a controller exception all (1) drop the robot's scheduled macro commands, (2) hold
the robot at once and (3) end the job with exactly one terminal event. A robot whose controller
raised is stopped for good: its executor refuses every later job.
"""
from __future__ import annotations

import copy
import math
from collections.abc import Callable, Mapping

import numpy as np

from harness.zone_own_executor import DEFAULT_JOB_SIM_LIMIT_S, TICK_S, ZoneOwnExecutor
from harness.zone_own_status import CARRY_PHASES
from harness.zone_study_contract import ROBOTS

CONTACT_DEDUPE_S = .1
HOST_APIS = ('deliver', 'goto', 'look_around', 'hold', 'wait', 'abort')
CONTACT_PROFILE_DECISION = {'cargo_noslip_v1': 'approved by the user 2026-09-26 as the study-wide contact profile'}


class _RobotSlot:
    """Host-side bookkeeping of one robot (never visible to any executor)."""

    def __init__(self, rid, port, executor):
        self.rid, self.port, self.executor = rid, port, executor
        self.commands: list[dict] = []
        self.frames: list[dict] = []
        self.timeline: list[tuple[float, list]] = []
        self.next_decide = 0.
        self.next_frame = 0.
        self.capture_after = False
        self.decisions: list[dict] = []
        self.cancellations: list[dict] = []
        self.exception: dict | None = None
        self.dead = False


class OwnCamTeamHost:
    """One MuJoCo world, three robots, one own-camera executor each (sync SIM, weld OFF)."""

    FRAME_S = .2
    GT_S = .05

    def __init__(self, spec: Mapping, student: Mapping, *, root, study_layer: Callable, frames_dir=None):
        import importlib
        import json
        from pathlib import Path

        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_arena import LAYOUTS, layout
        from sim.zone_cargo_contact import CARGO_PROFILES, apply as apply_cargo_profile, base_profile, profile_record
        from sim.zone_landmarks import TaggedZoneScene

        self.spec, self.student, self.root = dict(spec), dict(student), Path(root)
        profile = spec['contact_profile']
        self.scene = TaggedZoneScene.from_tagged(spec['map'], spec['seed'], spec['goal'], spec.get('extra_boxes'),
                                                 contact_profile=base_profile(profile))
        xml_transform = ((lambda xml: apply_cargo_profile(self.scene.transform(xml), profile))
                         if profile in CARGO_PROFILES else self.scene.transform)
        self.world = MultiMasterPiProductionV2(seed=spec['seed'], width=640, height=480, render=True,
                                               warehouse_layout=self.scene.engine_layout, warehouse_cargo_ids=None,
                                               xml_transform=xml_transform)
        self.scene.setup(self.world)
        self.contact_record = {'profile': profile, 'base_profile': base_profile(profile),
                               'cargo_profile': profile_record(profile) if profile in CARGO_PROFILES else None,
                               'noslip_iterations': int(self.world.model.opt.noslip_iterations),
                               'timestep_s': float(self.world.model.opt.timestep),
                               'user_decision': CONTACT_PROFILE_DECISION.get(profile, 'none recorded')}
        if profile == 'cargo_noslip_v1' and self.contact_record['noslip_iterations'] <= 0:
            raise RuntimeError('cargo_noslip_v1 requested but noslip_iterations is 0')
        self.static = self.scene.config['static_map']
        self.objects = self.scene.config['setup_only']['objects']
        self.spawns = self.scene.config['setup_only']['spawns']
        calibration = json.loads((self.root / student['calibration']).read_text())
        skill_cls = getattr(importlib.import_module(student['skill_module']), student['skill_class'])
        arena = layout(self.static['base_map']['map_id'])
        # Static layout only (idle-spawn discs, #181 v6 contract; v9 inherits v6's class), never a live pose.
        from harness.wrist_zone_skill_v6 import StaticKeepout
        keepouts = tuple(StaticKeepout(f'spawn_row_{i}', (float(arena['spawn_x']), float(y)), .17,
                                       'static_layout_idle_spawn')
                         for i, y in enumerate(arena['spawn_rows_y']))
        self.keepout_records = [k.record() for k in keepouts]
        discs = [{'id': f'spawn_row_{i}', 'center_m': [float(arena['spawn_x']), float(y)], 'radius_m': .17,
                  'source': 'static_layout_idle_spawn'} for i, y in enumerate(arena['spawn_rows_y'])]
        from harness.map_goto import UNLOADED_ENVELOPE, plan_path
        from harness.owncam_drive import LOADED_ENVELOPE
        from harness.wrist_zone_skill import PoseEstimate
        rows_y = LAYOUTS['zone_wide']['pickup_rows_y']
        self.order_sheet = spec['order_sheet']
        self.robots: dict[str, _RobotSlot] = {}
        for rid in ROBOTS:
            port = CameraRobotPort(self.world, rid, allow_reverse=True, allow_mecanum=True)
            ex_ref: dict = {}
            own_static = copy.deepcopy(self.static)          # per-robot copies: no shared mutable state

            def planner(start, goal, carrying, _ref=ex_ref, _static=own_static):
                job = _ref['ex'].job
                obstacles = job.ctl._keepouts() if job is not None and job.ctl is not None else []
                result = plan_path(_static, start, goal, LOADED_ENVELOPE if carrying else UNLOADED_ENVELOPE,
                                   obstacles=obstacles, escape_start_m=.25)
                return None if result is None else [tuple(p) for p in result['waypoints_m'][1:]]

            def factory(order, *, robot_id, _planner=planner, _keepouts=keepouts,
                        _bounds=tuple(own_static['bounds_m'])):
                return skill_cls(order, planner=_planner, robot_id=robot_id, mode='m1', static_keepouts=_keepouts,
                                 static_bounds_m=list(_bounds))

            ex = ZoneOwnExecutor(rid, own_static, calibration['params'], self.order_sheet, skill_factory=factory,
                                 pose_estimate_cls=PoseEstimate, search_rows_y=rows_y, mode=student.get('mode', 'm1'),
                                 seed=spec['seed'], job_sim_limit_s=spec.get('job_sim_limit_s', DEFAULT_JOB_SIM_LIMIT_S),
                                 static_keepouts=copy.deepcopy(discs))
            ex_ref['ex'] = ex
            self.robots[rid] = _RobotSlot(rid, port, ex)
        self.study_layer = study_layer
        self.frames_dir = Path(frames_dir) if frames_dir else None
        self.eval_only = {'gt': [], 'frames_eval': [], 'contacts': [], 'kind_steps': {r: {} for r in ROBOTS},
                          'retention': {r: {'carry_steps': 0, 'both_finger_steps': 0, 'low_box_steps': 0,
                                            'min_box_z_m': None} for r in ROBOTS},
                          'max_eq_active': 0}
        self._last_contact: dict[tuple[str, str], float] = {}
        self.api_calls: list[dict] = []
        self.event_log: list[dict] = []
        self.closed = False
        self._geoms()
        self._next_gt = 0.
        for rid in self.robots:
            pulses = {int(k): int(v) for k, v in self.world.robot(rid).servo_command_pulses.items()}
            self._sink(rid, {'t': 0.0, 'kind': 'initial_servo_command', 'pulses': pulses})

    # ------------------------------------------------------------ eval-only geometry
    def _geoms(self):
        import mujoco
        m = self.world.model
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
        self._names = names
        self._wall = {g for g, n in enumerate(names) if n.startswith('zone_wall_')}
        self._own = {r: {g for g, n in enumerate(names) if n.startswith(r + '__')} for r in ROBOTS}
        self._fingers = {r: ({g for g, n in enumerate(names) if n == r + '__left_finger'},
                             {g for g, n in enumerate(names) if n == r + '__right_finger'}) for r in ROBOTS}
        self._box_geom = {oid: {g for g, n in enumerate(names) if n == o['body_name'] + '_geom'}
                          for oid, o in self.objects.items()}
        self._all_box = set().union(*self._box_geom.values())
        self.assigned_box = {}                  # eval-only: robot -> box id of its scripted order line

    # ------------------------------------------------------------ own-input plumbing
    def _sink(self, rid, row):
        slot = self.robots[rid]
        slot.commands.append(row)
        slot.executor.on_command(row)

    def _apply(self, rid, action, now):
        row = {'t': round(float(now), 4), **action}
        self._sink(rid, row)
        self.robots[rid].port.apply(action, now)

    def _hold(self, rid, now):
        self._sink(rid, {'t': round(float(now), 4), 'kind': 'hold'})
        self.robots[rid].port.hold(now)

    def _drop_scheduled(self, rid, now, why):
        """Discard this robot's scheduled macro commands and hold it now (the one cancel path)."""
        slot = self.robots[rid]
        dropped = sum(len(cmds) for _, cmds in slot.timeline)
        slot.timeline, slot.capture_after = [], False
        self._hold(rid, now)
        slot.next_decide = float(now)
        slot.cancellations.append({'t': round(float(now), 4), 'why': why, 'dropped_macro_commands': dropped})

    def _guard(self, rid, now, fn, *args):
        """A controller/skill exception fails THAT robot's job and stops the robot for good; the others go on."""
        slot = self.robots[rid]
        if slot.dead:
            return None
        try:
            return fn(rid, now, *args)
        except OSError:
            raise                                  # host I/O (disk full, render device): infrastructure, not the robot
        except Exception as exc:                  # noqa: BLE001 - recorded as a failed job, never swallowed
            import traceback
            slot.exception = {'t': round(now, 3), 'type': type(exc).__name__, 'message': str(exc)[:2000],
                              'traceback': traceback.format_exc()[-6000:]}
            slot.dead = True
            slot.executor.stop(now, f'EXCEPTION:{type(exc).__name__}')
            self._drop_scheduled(rid, now, 'exception')
            return None

    def _capture(self, rid, now):
        return self._guard(rid, now, self._capture_raw)

    def _capture_raw(self, rid, now):
        import base64

        import cv2
        slot = self.robots[rid]
        obs = slot.port.capture()                         # this robot's own robot_cam only
        jpeg = base64.b64decode(obs['image'])
        rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        index = len(slot.frames)
        if self.frames_dir is not None:
            d = self.frames_dir / rid
            d.mkdir(parents=True, exist_ok=True)
            (d / f'{index:05d}.jpg').write_bytes(jpeg)
        report = slot.executor.on_frame(now, obs, rgb)
        slot.frames.append({'frame': index, 't': round(now, 4), 'frame_id': obs['frame_id'], 'sha256': obs['sha256'],
                            'robot_id': obs['robot_id'], 'camera': obs['camera'],
                            'commanded_servo': obs['actuator_state']['servo_pulses'], 'report': report.as_dict()})
        x, y, yaw = self._truth(rid)
        row = {'robot_id': rid, 'frame': index, 't': round(now, 4), 'gt': [round(x, 5), round(y, 5), round(yaw, 6)]}
        if report.initialized:
            row.update(pos_err_m=round(math.hypot(report.x_m - x, report.y_m - y), 5),
                       yaw_err_deg=round(abs(math.degrees((report.yaw_rad - yaw + math.pi) % (2 * math.pi) - math.pi)), 4),
                       std_xy_m=round(report.std_xy_m, 5))
        self.eval_only['frames_eval'].append(row)
        slot.next_frame = now + self.FRAME_S

    def _truth(self, rid):
        r = self.world.robot(rid)
        xyz, rpy = r.base_xyz(), r.base_rpy()
        return float(xyz[0]), float(xyz[1]), float(rpy[2])

    # ------------------------------------------------------------ physics
    def _contact_kinds(self, data):
        kinds = {r: set() for r in ROBOTS}
        fingers = {r: {} for r in ROBOTS}
        for i in range(data.ncon):
            c = data.contact[i]
            pair = {int(c.geom1), int(c.geom2)}
            for r in ROBOTS:
                mine = pair & self._own[r]
                if not mine:
                    continue
                other = next(iter(pair - mine), None)
                kind = ('wall' if other in self._wall else
                        'peer_robot' if any(other in self._own[q] for q in ROBOTS if q != r) else
                        'box' if other in self._all_box else None)
                if kind:
                    kinds[r].add(kind)
                if other in self._all_box:
                    lf, rf = self._fingers[r]
                    box = next(b for b, gs in self._box_geom.items() if other in gs)
                    f = fingers[r].setdefault(box, [False, False])
                    f[0] |= bool(mine & lf)
                    f[1] |= bool(mine & rf)
        return kinds, fingers

    def _record_contacts(self, now, kinds):
        """Eval-only contact log, de-duplicated per (robot, kind) within CONTACT_DEDUPE_S (not the global last row)."""
        for r in ROBOTS:
            for k in kinds[r]:
                self.eval_only['kind_steps'][r][k] = self.eval_only['kind_steps'][r].get(k, 0) + 1
                last = self._last_contact.get((r, k))
                if last is None or last < now - CONTACT_DEDUPE_S:
                    self.eval_only['contacts'].append({'t': round(now, 4), 'robot_id': r, 'kind': k})
                self._last_contact[(r, k)] = now

    def _physics_until(self, t_end):
        world, data = self.world, self.world.data
        while float(data.time) < t_end - 1e-9:
            now = float(data.time)
            for s in self.robots.values():
                s.port.tick(now)
            world._physics_step_for(world.controllers['r1'])
            now = float(data.time)
            kinds, fingers = self._contact_kinds(data)
            self._record_contacts(now, kinds)
            for r in ROBOTS:
                ex = self.robots[r].executor
                sk = ex.job.ctl.skill if ex.job is not None and ex.job.ctl is not None else None
                box = self.assigned_box.get(r)
                if sk is not None and sk.phase in CARRY_PHASES and box is not None:
                    ret = self.eval_only['retention'][r]
                    bz = float(data.body(self.objects[box]['body_name']).xpos[2])
                    ret['carry_steps'] += 1
                    ret['both_finger_steps'] += int(all(fingers[r].get(box, (False, False))))
                    ret['low_box_steps'] += int(bz < .04)
                    ret['min_box_z_m'] = bz if ret['min_box_z_m'] is None else min(ret['min_box_z_m'], bz)
            if len(data.eq_active):
                self.eval_only['max_eq_active'] = max(self.eval_only['max_eq_active'], int(data.eq_active.max()))
            if now + 1e-9 >= self._next_gt:
                self.eval_only['gt'].append({'t': round(now, 4), 'robots': {r: [round(v, 5) for v in self._truth(r)]
                                                                             for r in ROBOTS},
                                             'boxes': {b: [round(float(v), 4) for v in
                                                           data.body(o['body_name']).xpos]
                                                       for b, o in self.objects.items() if o['kind'] == 'cyan'}})
                self._next_gt = now + self.GT_S
            for rid, s in self.robots.items():
                if not s.dead and now + 1e-9 >= s.next_frame:
                    self._capture(rid, now)

    # ------------------------------------------------------------ macros (as scripts/run_m1_owncam.execute_macro)
    def _macro_timeline(self, rid, action, now):
        ex = self.robots[rid].executor
        kind = action['kind']
        if kind == 'drive':
            cmd = {'kind': 'drive', 'forward': action['fwd'], 'turn': action['turn'], 'duration_s': action['duration']}
            return [(now, [cmd]), (now + action['duration'] + .2, ['hold'])]
        if kind == 'mecanum':
            cmd = {'kind': 'mecanum', 'forward': action['forward'], 'left': action['left'], 'turn': action['turn'],
                   'duration_s': action['duration']}
            return [(now, [cmd]), (now + action['duration'] + .1, ['hold'])]
        if kind == 'pose':
            start = ex.last_obs['actuator_state']['servo_pulses']
            targets = action['pulses']
            delta = max(abs(p - start[str(s)]) for s, p in targets.items())
            duration = max(.25, delta / 600.)
            count = max(5, math.ceil(duration / .05))
            out = []
            for sample in range(1, count + 1):
                u = sample / count
                ease = u * u * (3 - 2 * u)
                cmds = []
                for servo, end in targets.items():
                    pulse = round(start[str(servo)] + ease * (end - start[str(servo)]))
                    cmds.append({'kind': 'look', 'pan_pulse': pulse} if int(servo) == 6 else
                                {'kind': 'arm', 'servo_id': int(servo), 'pulse': pulse})
                out.append((now + (sample - 1) * duration / count, cmds))
            sk = ex.job.ctl.skill if ex.job is not None and ex.job.ctl is not None else None
            settle = .3 if sk is not None and sk.phase == 'grasp' and sk.box.phase == 'approach' else .15
            out.append((now + duration + settle, []))
            return out
        if kind == 'wait':
            return [(now, [{'kind': 'wait'}]), (now + max(.05, action['duration']), [])]
        raise ValueError('UNKNOWN_MACRO')

    def _decide(self, rid, now):
        return self._guard(rid, now, self._decide_raw)

    def _decide_raw(self, rid, now):
        slot = self.robots[rid]
        ex = slot.executor
        for _ in range(12):
            decision = ex.step(now)
            mode = decision['mode']
            if mode == 'capture':
                self._capture(rid, now)
                if slot.dead:
                    return
                continue
            if mode == 'tick':
                for cmd in decision['commands']:
                    if cmd['kind'] == 'hold':
                        self._hold(rid, now)
                    else:
                        self._apply(rid, cmd, now)
                slot.next_decide = now + TICK_S
                return
            if mode == 'macro':
                slot.decisions.append({'t': round(now, 3), 'action': decision['action']})
                slot.timeline = self._macro_timeline(rid, decision['action'], now)
                slot.capture_after = True
                return
            raise ValueError(f'unknown decision mode {mode!r}')
        slot.next_decide = now + TICK_S

    def _run_timeline(self, rid, now):
        return self._guard(rid, now, self._run_timeline_raw)

    def _run_timeline_raw(self, rid, now):
        slot = self.robots[rid]
        while slot.timeline and slot.timeline[0][0] <= now + 1e-9:
            _, cmds = slot.timeline.pop(0)
            for cmd in cmds:
                if cmd == 'hold':
                    self._hold(rid, now)
                else:
                    self._apply(rid, cmd, now)
        if not slot.timeline and slot.capture_after:
            slot.capture_after = False
            self._capture(rid, now)
            slot.next_decide = now

    def _expire(self, rid, now):
        """Local SIM deadline, also while a macro runs: drop the schedule, hold, one terminal event."""
        if self.robots[rid].executor.expire_if_due(now):
            self._drop_scheduled(rid, now, 'local_timeout')

    def call(self, rid, api, *args):
        """The study layer's only door into an executor: the job API of THAT robot."""
        if api not in HOST_APIS:
            raise ValueError(f'unknown executor API {api!r}; known: {HOST_APIS}')
        slot = self.robots[rid]
        ex = slot.executor
        now = float(self.world.data.time)
        ex.now = now
        if self.closed:
            ack = ex.refuse(api, 'EPISODE_ENDED')
        else:
            ack = getattr(ex, api)(*args)
            if api == 'abort' and ack['accepted']:
                self._drop_scheduled(rid, now, 'abort')
        self.api_calls.append(ack)
        return ack

    def _next_wake(self, live):
        times = []
        for s in live:
            times.append(s.timeline[0][0] if s.timeline else s.next_decide)
            deadline = s.executor.deadline()
            if deadline is not None:
                times.append(deadline + 1e-6)
        return min(times)

    def _deliver_events(self, now):
        for rid in ROBOTS:
            for ev in self.robots[rid].executor.drain_events():
                self.event_log.append(ev)
                self.study_layer(self, 'event', ev, now)

    def run(self, sim_limit_s: float, done: Callable[[], bool] | None = None) -> dict:
        data = self.world.data
        self._physics_until(.5)
        for s in self.robots.values():
            s.next_decide = float(data.time)
        self.study_layer(self, 'start', None, float(data.time))
        outcome = None
        while True:
            now = float(data.time)
            if now > sim_limit_s:
                outcome = 'SIM_LIMIT'
                break
            for rid in ROBOTS:
                slot = self.robots[rid]
                if slot.dead:
                    continue
                self._expire(rid, now)
                if slot.timeline or slot.capture_after:
                    self._run_timeline(rid, now)
                if not slot.dead and not slot.timeline and not slot.capture_after and now + 1e-9 >= slot.next_decide:
                    self._decide(rid, now)
            self._deliver_events(now)
            if done is not None and done():
                outcome = 'STUDY_LAYER_DONE'
                break
            live = [s for s in self.robots.values() if not s.dead]
            if not live:
                outcome = 'ALL_ROBOTS_STOPPED'
                break
            self._physics_until(max(self._next_wake(live), now + float(self.world.model.opt.timestep)))
        self.close_episode(outcome)
        self._physics_until(float(data.time) + .5)
        return {'outcome': outcome, 'sim_s': round(float(data.time), 3)}

    def close_episode(self, outcome):
        """Episode end: every active job ends once ('EPISODE_END:<outcome>'), schedules drop, robots hold,
        and later calls are refused."""
        now = float(self.world.data.time)
        self.closed = True
        for rid, slot in self.robots.items():
            if slot.executor.cancel(now, f'EPISODE_END:{outcome}') or slot.timeline or slot.capture_after:
                self._drop_scheduled(rid, now, f'episode_end:{outcome}')
            else:
                self._hold(rid, now)
        self._deliver_events(now)

    def close(self):
        self.world.close()
