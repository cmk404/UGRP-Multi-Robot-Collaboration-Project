#!/usr/bin/env python3
"""Bounded physical executor; privileged referee state never enters predictors."""
from __future__ import annotations
import hashlib
import json
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

    def open(self, base_offsets: Mapping[str, float] | None = None):
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
        offsets = base_offsets or {r: 0.0 for r in ROBOTS}
        for rid in ROBOTS:
            pose = list(self.fixture['base_poses'][rid])
            pose[0] -= float(offsets[rid])
            self.world.controllers[rid].set_base_pose_for_test(tuple(pose), 0.0)
        c = self.fixture['top_camera']
        cid = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_top')
        self.world.model.cam_pos[cid] = c['position_m']
        self.world.model.cam_quat[cid] = c['quaternion_wxyz']
        self.world.model.cam_fovy[cid] = c['fov_y_deg']
        mujoco.mj_forward(self.world.model, self.world.data)
        # Observer video only; policy own/top cameras remain unchanged.
        _camera_look_at(self.world, approach_distance=max(map(float, offsets.values())))
        self.replay([self.skill['initialization_replay'][0]], 'folded_setup')
        self.ports = {r: CameraRobotPort(self.world, r) for r in ROBOTS}
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

    def finish_grasp(self, predict_correction, models, *, rounds=16):
        from scripts.run_camera_pair_transport import evaluate_grasp_samples
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
        for i in range(rounds):
            frames, targets = self.capture(f'grasp-{i:03d}'), {}
            for rid in ROBOTS:
                before = dict(self.commands[rid])
                d = predict_correction(models[rid], frames[rid]['own_bytes'], frames[rid]['top_bytes'], max_step=25)
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
        frames = self.capture('grasp-post-recovery')
        rec['preclose_issued_commands'] = {r: dict(self.commands[r]) for r in ROBOTS}
        rec['post_recovery_images'] = {r: {'own': frames[r]['own_rgb'], 'top': frames[r]['shared_top_rgb']} for r in ROBOTS}
        rec['final_visual_errors'] = {r: predict_correction(models[r], frames[r]['own_bytes'], frames[r]['top_bytes'], max_step=25) for r in ROBOTS}
        self.replay([{'targets': {r: {1: int(self.skill['close_pulses'][r])} for r in ROBOTS},
                      'duration_s': self.skill['close_duration_s'], 'settle_s': self.skill['close_settle_s']}], 'grasp_close')
        lift = {r: {int(ch): max(500, min(2500, self.commands[r][int(ch)] + int(delta)))
                    for ch, delta in self.skill['lift_delta_pulses'][r].items()} for r in ROBOTS}
        self.replay([{'targets': lift, 'duration_s': self.skill['lift_duration_s'],
                      'settle_s': self.skill['lift_settle_s']}], 'grasp_lift')
        self.phase = 'grasp_hold'
        self.tick(float(self.skill['hold_s']))
        rec['evaluation'] = evaluate_grasp_samples(self.evaluation_samples)
        self.grasp_report = rec
        return calls

    def evaluation_snapshot(self, *, full_state=True):
        from scripts.probe_dual_grasp_sync import _plain_beam_contact, _pose_metrics
        result = {**_pose_metrics(self.world), 'phase': self.phase,
                  'bases': {r: list(map(float, self.world.controllers[r].base_xyz())) for r in ROBOTS},
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
