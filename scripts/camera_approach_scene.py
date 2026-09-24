#!/usr/bin/env python3
"""Bounded physical executor; privileged referee state never enters predictors."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

ROBOTS = ('r1', 'r3')
DRIVE_SLICE_S = .20
STOP_DWELL_S = .25
MAX_APPROACH_ROUNDS = 100


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def image_record(path: Path, root: Path, data: bytes) -> dict[str, Any]:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(path)
    path.write_bytes(data)
    return {'path': path.relative_to(root).as_posix(), 'sha256': sha256_bytes(data)}


def normalize_replay(command: Mapping[str, Any]) -> dict[str, dict[int, int]]:
    return {rid: {int(k): int(v) for k, v in pose.items()}
            for rid, pose in command['targets'].items()}


def ready_after_two_fresh_stationary(history: list[dict[str, Any]]) -> bool:
    if len(history) < 2:
        return False
    a, b = history[-2:]
    return (isinstance(a.get('frame_id'), int) and isinstance(b.get('frame_id'), int)
            and b['frame_id'] > a['frame_id']
            and all(bool(item.get(key)) for item in (a, b)
                    for key in ('stationary', 'ok', 'ready')))


def payload_contact_other_geoms(contacts, beam_geom: int, robot_geoms: set[int]) -> list[int]:
    """Count body/arm/finger beam collisions, without truthy metadata fields."""
    return [b if a == beam_geom else a for a, b in contacts
            if (a == beam_geom and b in robot_geoms) or
               (b == beam_geom and a in robot_geoms)]


def validate_start_poses(start_poses: Mapping[str, Mapping[str, Any]], *,
                         max_distance_m: float = .45) -> dict[str, dict[str, float]]:
    if not isinstance(start_poses, Mapping) or set(start_poses) != set(ROBOTS):
        raise ValueError('start_poses must contain exactly r1 and r3')
    result = {}
    bounds = {'distance_m': (-.02, max_distance_m), 'lateral_m': (-.1, .1), 'yaw_deg': (-20., 20.)}
    for rid in ROBOTS:
        pose = start_poses[rid]
        if not isinstance(pose, Mapping) or set(pose) != set(bounds):
            raise ValueError(f'{rid} start pose requires distance_m, lateral_m, yaw_deg')
        result[rid] = {}
        for key, (lower, upper) in bounds.items():
            value = pose[key]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    or lower is not None and not lower <= value <= upper):
                bound = f' in {lower}..{upper}' if lower is not None else ''
                raise ValueError(f'{rid}.{key} must be finite{bound}')
            result[rid][key] = float(value)
    return result


class ApproachScene:
    def __init__(self, out_dir: Path, grasp_model_dir: Path, *, fps: int = 4):
        self.out, self.models_root = out_dir.resolve(), grasp_model_dir.resolve()
        self.fps = fps
        self.world = self.video = self.referee = None
        self._original_step = None
        self.ports = {}
        self.commands = {r: {} for r in ROBOTS}
        self.trace = []
        self.frame_ids = {r: 0 for r in ROBOTS}
        self.evaluation_samples = []
        self.approach_payload_contact_steps = 0
        self.approach_physics_steps = 0
        self.approach_collision_events = []
        self.phase = 'setup'
        self.skill = json.loads((self.models_root / 'student-skill.json').read_text())
        self.fixture = json.loads((self.models_root / 'evaluation-fixture.json').read_text())
        self.grasp_report = None

    def open(self, base_offsets: Mapping[str, float] | None = None, *,
             start_poses: Mapping[str, Mapping[str, Any]] | None = None,
             max_start_distance_m: float = .45):
        starts = (validate_start_poses(start_poses, max_distance_m=max_start_distance_m)
                  if start_poses is not None else None)
        import mujoco
        from unittest.mock import patch
        import sim.multi_masterpi_production as production
        from sim.camera_robot_port import CameraRobotPort
        from sim.cooperative_payload import BEAM_GEOM_NAME
        from scripts.probe_dual_grasp_sync import Video, _camera_look_at, _plain_beam_xml
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out / 'rgb').mkdir()
        with patch.object(production, 'build_multi_robot_xml',
                          _plain_beam_xml(production.build_multi_robot_xml)):
            self.world = production.MultiMasterPiProductionV2(
                seed=int(self.fixture['seed']), width=960, height=720, render=True)
        self.configure_world_setup()
        offsets = ({r: starts[r]['distance_m'] for r in ROBOTS} if starts is not None
                   else base_offsets or {r: 0.0 for r in ROBOTS})
        for rid in ROBOTS:
            pose = list(self.fixture['base_poses'][rid])
            pose[0] -= float(offsets[rid])
            if starts is not None:
                pose[1] += starts[rid]['lateral_m']
            yaw = math.radians(starts[rid]['yaw_deg']) if starts is not None else 0.0
            self.world.controllers[rid].set_base_pose_for_test(tuple(pose), yaw)
        c = self.fixture['top_camera']
        cid = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_top')
        self.world.model.cam_pos[cid] = c['position_m']
        self.world.model.cam_quat[cid] = c['quaternion_wxyz']
        self.world.model.cam_fovy[cid] = c['fov_y_deg']
        mujoco.mj_forward(self.world.model, self.world.data)
        # Observer video only; policy own/top cameras remain unchanged.
        _camera_look_at(self.world, approach_distance=max(map(float, offsets.values())))
        self.configure_observer_camera()
        self.replay([self.skill['initialization_replay'][0]], 'folded_setup')
        self.ports = {r: CameraRobotPort(self.world, r, allow_reverse=starts is not None,
                                         allow_mecanum=starts is not None) for r in ROBOTS}
        self.beam_geom = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, BEAM_GEOM_NAME)
        self.robot_geoms = {i for i in range(self.world.model.ngeom)
                            if (mujoco.mj_id2name(self.world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith(('r1__', 'r3__'))}
        self.video = Video(self.world, self.out / 'motion.mp4', self.fps)
        self.referee = (self.out / 'evaluation-only.jsonl').open('w')
        self.world.frame_callback = self._frame
        self._original_step = self.world._physics_step_for
        self.world._physics_step_for = self._step_with_referee
        self.phase = 'setup'
        self._referee_tick()
        self.phase = 'approach'
        self.video.stage = self.phase
        self.video.capture(force=True)
        return self

    def configure_world_setup(self):
        """Optional authored scene setup, before any task motion or observation."""
        pass

    def configure_observer_camera(self):
        """Optional presentation-only camera setup before recording begins."""
        pass

    def time(self):
        return float(self.world.data.time)

    def _step_with_referee(self, active, commands=None):
        self._original_step(active, commands)
        self._referee_tick()

    def _referee_tick(self):
        if self.phase.startswith('approach'):
            self.approach_physics_steps += 1
            touched = payload_contact_other_geoms(
                [(int(c.geom1), int(c.geom2)) for c in self.world.data.contact[:self.world.data.ncon]],
                self.beam_geom, self.robot_geoms)
            if touched:
                self.approach_payload_contact_steps += 1
                self.approach_collision_events.append({'sim_time_s': self.time(), 'robot_geom_ids': touched})
        if not self.evaluation_samples or self.time() - self.evaluation_samples[-1]['sim_time_s'] >= .099:
            sample = self.evaluation_snapshot(full_state=False)
            self.evaluation_samples.append(sample)
            self.referee.write(json.dumps(sample, sort_keys=True) + '\n')

    def _frame(self):
        if self.video:
            self.video.stage = self.phase
            self.video.capture()

    def replay(self, commands, stage):
        self.phase = stage
        for c in commands:
            targets = normalize_replay(c)
            self.world._team_joint_move_servos(targets, float(c['duration_s']), settle_s=float(c.get('settle_s', 0)))
            for rid, pose in targets.items():
                self.commands[rid].update(pose)
            self.trace.append({'stage': stage, 'command': c, 'end_sim_time_s': self.time()})

    def tick(self, duration_s):
        for _ in range(round(duration_s / float(self.world.model.opt.timestep))):
            for port in self.ports.values():
                port.tick(self.time())
            self.world._physics_step_for(self.world.controllers['r1'])
            self._frame()
        for port in self.ports.values():
            port.tick(self.time())

    def drive(self, forwards, duration_s=DRIVE_SLICE_S):
        self.phase = 'approach' if any(forwards.values()) else 'approach_stop_dwell'
        start = self.time()
        actions = {r: {'kind': 'drive', 'forward': float(forwards[r]), 'turn': 0.0,
                       'duration_s': float(duration_s)} for r in ROBOTS}
        for rid, action in actions.items():
            self.ports[rid].apply(action, start)
        self.tick(duration_s)
        self.trace.append({'stage': self.phase, 'actions': actions,
                           'start_sim_time_s': start, 'end_sim_time_s': self.time()})

    def drive_mecanum(self, commands, duration_s=DRIVE_SLICE_S):
        if not isinstance(commands, Mapping) or set(commands) != set(ROBOTS):
            raise ValueError('commands must contain exactly r1 and r3')
        for rid, command in commands.items():
            if not isinstance(command, Mapping) or set(command) != {'forward','left','turn'}:
                raise ValueError(f'{rid} command requires exactly forward, left, and turn')
            for axis, (lower, upper) in {'forward':(-.05,.15),'left':(-.10,.10),'turn':(-.15,.15)}.items():
                value=command[axis]
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lower<=value<=upper:
                    raise ValueError(f'{rid}.{axis} must be finite in {lower}..{upper}')
        if isinstance(duration_s,bool) or not isinstance(duration_s,(int,float)) or not math.isfinite(duration_s) or not 0<=duration_s<=1:
            raise ValueError('duration_s must be finite in 0..1')
        moving = any(any(command[axis] != 0.0 for axis in ('forward','left','turn'))
                     for command in commands.values())
        self.phase = 'approach' if moving else 'approach_stop_dwell'
        start = self.time()
        actions = {rid: {'kind':'mecanum','forward':commands[rid]['forward'],
                         'left':commands[rid]['left'],'turn':commands[rid]['turn'],
                         'duration_s':duration_s} for rid in ROBOTS}
        for rid, action in actions.items():
            self.ports[rid].apply(action, start)
        self.tick(duration_s)
        self.trace.append({'stage':self.phase,'actions':actions,
                           'start_sim_time_s':start,'end_sim_time_s':self.time()})

    def stop_dwell(self):
        self.drive({r: 0.0 for r in ROBOTS}, STOP_DWELL_S)

    def capture(self, tag):
        top = self.world.render_team_jpeg(camera='cctv_top', quality=95)
        top_record = image_record(self.out / 'rgb' / f'{tag}-top.jpg', self.out, top)
        result = {}
        for rid in ROBOTS:
            own = self.world.render_jpeg(robot_id=rid, camera='robot_cam', quality=95)
            own_record = image_record(self.out / 'rgb' / f'{tag}-{rid}-own.jpg', self.out, own)
            self.frame_ids[rid] += 1
            result[rid] = {'own_bytes': own, 'top_bytes': top, 'own_rgb': own_record,
                           'shared_top_rgb': top_record, 'frame_id': self.frame_ids[rid]}
        return result

    def finish_grasp(self, predict_correction, models, *, rounds=16, after_close=None, hold=None, close_pulse=None):
        from scripts.run_camera_pair_transport import evaluate_grasp_samples
        def observe_and_predict(tag):
            def predict(frames):
                return {rid:predict_correction(models[rid],frames[rid]['own_bytes'],
                                                frames[rid]['top_bytes'],max_step=25)
                        for rid in ROBOTS}
            hook=getattr(self,'observe_and_compute',None)
            if hook is not None:return hook(tag,predict)
            frames=self.capture(tag)
            return frames,predict(frames)
        self.replay(self.skill['initialization_replay'][1:], 'grasp_initialization')
        # Match the original zero-perturbation runner's pre-recovery settling step.
        targets = {r: {c: self.commands[r][c] for c in (3, 4, 5)} for r in ROBOTS}
        self.replay([{'targets': targets, 'duration_s': .35, 'settle_s': .10}], 'grasp_pre_recovery_settle')
        calls = []
        rec = {'config': {'condition': 'visual', 'rounds': rounds, 'max_step': 25},
               'skill_sha256': sha256_bytes((self.models_root / 'student-skill.json').read_bytes()),
               'model_sha256': {r: self.skill['models'][r]['sha256'] for r in ROBOTS},
               'actor_initial_issued_commands': {r: dict(self.commands[r]) for r in ROBOTS},
               'calls': calls, 'error': None}
        # Preserve RGB correction evidence even if a later readiness gate stops.
        self.grasp_report = rec
        for i in range(rounds):
            frames,predictions=observe_and_predict(f'grasp-{i:03d}')
            targets={}
            for rid in ROBOTS:
                before = dict(self.commands[rid])
                d = predictions[rid]
                pose = {int(ch): max(500, min(2500, before[int(ch)] + int(delta)))
                        for ch, delta in zip(models[rid]['channels'], d['delta_pulses']) if delta}
                if pose:
                    targets[rid] = pose
                calls.append({'round': i, 'robot_id': rid, 'condition': 'visual',
                              'images': {'own': frames[rid]['own_rgb'], 'top': frames[rid]['shared_top_rgb']},
                              'decision': d, 'own_commands_before': before,
                              'actions': [{'kind': 'arm', 'servo_id': ch, 'pulse': pulse} for ch, pulse in pose.items()]})
            if targets:
                self.replay([{'targets': targets, 'duration_s': .35, 'settle_s': .10}], 'grasp_rgb_recovery')
            else:
                self.phase = 'grasp_rgb_recovery'
                self.tick(.45)
        frames,final_visual_errors=observe_and_predict('grasp-post-recovery')
        rec['preclose_issued_commands'] = {r: dict(self.commands[r]) for r in ROBOTS}
        rec['post_recovery_images'] = {r: {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']} for r in ROBOTS}
        rec['final_visual_errors'] = final_visual_errors
        if close_pulse is not None and close_pulse not in (1500,1600,1700,1800):
            raise ValueError('explicit bounded grasp command comparison required')
        rec['close_command_override'] = close_pulse
        self.replay([{'targets': {r: {1: int(self.skill['close_pulses'][r] if close_pulse is None else close_pulse)} for r in ROBOTS},
                      'duration_s': self.skill['close_duration_s'], 'settle_s': self.skill['close_settle_s']}], 'grasp_close')
        if after_close is not None:
            after_close()
        lift = {r: {int(ch): max(500, min(2500, self.commands[r][int(ch)] + int(delta)))
                    for ch, delta in self.skill['lift_delta_pulses'][r].items()} for r in ROBOTS}
        self.replay([{'targets': lift, 'duration_s': self.skill['lift_duration_s'],
                      'settle_s': self.skill['lift_settle_s']}], 'grasp_lift')
        self.phase = 'grasp_hold'
        (hold or self.tick)(float(self.skill['hold_s']))
        rec['evaluation'] = evaluate_grasp_samples(self.evaluation_samples)
        self.grasp_report = rec
        return calls

    def evaluation_snapshot(self, *, full_state=True):
        from scripts.probe_dual_grasp_sync import _plain_beam_contact, _pose_metrics
        result = {**_pose_metrics(self.world), 'phase': self.phase,
                  'bases': {r: list(map(float, self.world.controllers[r].base_xyz())) for r in ROBOTS},
                  'base_rpy': {r: list(map(float, self.world.controllers[r].base_rpy())) for r in ROBOTS},
                  'base_yaw_rad': {r: float(self.world.controllers[r].base_rpy()[2]) for r in ROBOTS},
                  'contacts': {r: _plain_beam_contact(self.world, r) for r in ROBOTS}}
        if full_state:
            result.update(qpos=self.world.data.qpos.tolist(), qvel=self.world.data.qvel.tolist())
        return result

    def close(self):
        if self.world:
            for port in self.ports.values():
                port.stop()
            self.world.frame_callback = None
            if self._original_step:
                self.world._physics_step_for = self._original_step
        if self.referee:
            self.referee.close()
        if self.video:
            self.video.close()
        if self.world:
            self.world.close()
        if self.out.exists():
            (self.out / 'execution-trace.json').write_text(json.dumps(self.trace, indent=2) + '\n')
