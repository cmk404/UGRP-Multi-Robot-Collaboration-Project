"""Physics/recording owner for unloaded authored-map navigation.

This module is never passed to the navigator. Only map JSON and raw JPEG bytes
leave this boundary. Referee measurements are output-only.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess
from unittest.mock import patch
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

from sim.authored_navigation_map import augment_map_xml, map_sha256
from sim.camera_robot_port import CameraRobotPort
import sim.multi_masterpi_production as production

FIXED_TOP = {'name': 'cctv_top', 'position_m': [.55, -2., 2.5],
             'quaternion_wxyz': [1., 0., 0., 0.], 'fov_y_deg': 55.}
FOLDED_COMMANDS = {1: 2000, 3: 740, 4: 2320, 5: 1320, 6: 1500}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def course_xml(xml: str, authored_map: dict):
    """Keep the exact robot bodies; replace legacy task props with this course."""
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    robots = {node.get('name'): ET.tostring(node) for node in world.findall('body')
              if node.get('name', '').endswith('__robot')}
    if len(robots) != 3:
        raise ValueError('expected the three unchanged robot bodies')
    for node in list(world):
        if node.tag == 'geom' and node.get('name') != 'floor':
            world.remove(node)
        elif node.tag == 'body' and node.get('name') not in robots:
            # Schema placeholders remain for existing reset/controller APIs.
            # These legacy task props are not part of the unloaded map course.
            for body in node.iter('body'):
                body.set('gravcomp', '1')
            for geom in node.iter('geom'):
                geom.attrib.update(contype='0', conaffinity='0', rgba='0 0 0 0', group='5')
    xml = augment_map_xml(ET.tostring(root, encoding='unicode'), authored_map)
    final = ET.fromstring(xml).find('worldbody')
    after = {node.get('name'): ET.tostring(node) for node in final.findall('body')
             if node.get('name') in robots}
    if robots != after:
        raise ValueError('map construction changed robot body or camera geometry')
    return xml, {name: sha(data) for name, data in robots.items()}


class KnownMapScene:
    def __init__(self, out_dir: Path, authored_map: dict, robot_id: str, case: dict):
        self.out, self.map, self.rid, self.case = out_dir, authored_map, robot_id, case
        self.world = self.port = self.video = self.referee = None
        self.frames = 0
        self.samples = []
        self.collision_steps = self.physics_steps = self.weld_steps = 0
        self.max_tilt_rad = 0.
        self.next_video_s = 0.
        self.robot_xml_sha = {}
        self.scene_xml_sha = None
        self.last_status = 'setup'

    def open(self):
        if self.map['top_camera'] != FIXED_TOP:
            raise ValueError('course must preserve the previously approved fixed top camera')
        if self.rid not in ('r1', 'r3'):
            raise ValueError('this isolated navigation cohort supports r1 or r3')
        original = production.build_multi_robot_xml
        def builder(*args, **kwargs):
            kwargs['navigation_camera'] = False
            xml, self.robot_xml_sha = course_xml(original(*args, **kwargs), self.map)
            self.scene_xml_sha = sha(xml.encode('utf-8'))
            return xml
        with patch.object(production, 'build_multi_robot_xml', builder):
            self.world = production.MultiMasterPiProductionV2(seed=11, width=960, height=720, render=True)
        world = self.world
        for i, rid in enumerate(world.robot_ids):
            world.controllers[rid].set_base_pose_for_test((-1.2 + 1.2 * i, .6, .0325), 0.)
        x, y = self.case['start_xy_m']
        world.controllers[self.rid].set_base_pose_for_test((x, y, .0325), math.radians(self.case['start_yaw_deg']))
        world._team_joint_move_servos({self.rid: FOLDED_COMMANDS}, .6, settle_s=.5)
        cid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_top')
        world.model.cam_pos[cid] = FIXED_TOP['position_m']
        world.model.cam_quat[cid] = FIXED_TOP['quaternion_wxyz']
        world.model.cam_fovy[cid] = FIXED_TOP['fov_y_deg']
        # Only the observer camera moves. Neither actor camera is repositioned.
        observer = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_warehouse')
        position, target = np.array([.55, -4.3, 3.0]), np.array([.55, -2., 0.])
        forward = target - position; forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        quat = np.empty(4); mujoco.mju_mat2Quat(quat, np.column_stack((right, up, -forward)).ravel())
        world.model.cam_pos[observer] = position; world.model.cam_quat[observer] = quat
        world.model.cam_fovy[observer] = 55.
        mujoco.mj_forward(world.model, world.data)
        self.port = CameraRobotPort(world, self.rid, allow_reverse=True, allow_mecanum=True)
        self.robot_geoms = {i for i in range(world.model.ngeom)
            if (mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith(self.rid + '__')}
        self.obstacle_geoms = {i for i in range(world.model.ngeom)
            if (mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('known_map_')
            and int(world.model.geom_contype[i]) != 0}
        self.referee = (self.out / 'evaluation-only.jsonl').open('w')
        self.video = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', '960x720', '-r', '4', '-i', '-', '-an', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
            str(self.out / 'motion.mp4')], stdin=subprocess.PIPE)
        self.initial_invariants = self.invariants()
        self._referee_tick(force=True)
        self._video_frame(force=True)
        return self

    def invariants(self):
        m = self.world.model
        cameras = {}
        for name in (self.rid + '__robot_cam', 'cctv_top'):
            cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name)
            cameras[name] = {'position': m.cam_pos[cid].tolist(), 'quaternion': m.cam_quat[cid].tolist(),
                             'fovy': float(m.cam_fovy[cid])}
        ids = sorted(self.robot_geoms)
        digest = hashlib.sha256()
        for array in (m.geom_size, m.geom_pos, m.geom_quat, m.geom_rgba, m.geom_friction):
            digest.update(array[ids].tobytes())
        return {'actor_cameras': cameras, 'robot_geometry_sha256': digest.hexdigest(),
                'scene_xml_sha256': self.scene_xml_sha,
                'robot_xml_sha256': self.robot_xml_sha, 'map_sha256': map_sha256(self.map),
                'added_navigation_camera': any((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i) or '').endswith('nav_cam') for i in range(m.ncam))}

    def capture(self, index):
        own = self.world.render_jpeg(robot_id=self.rid, camera='robot_cam', quality=95)
        top = self.world.render_team_jpeg(camera='cctv_top', quality=95)
        records = {}
        for kind, data in [('own', own), ('top', top)]:
            p = self.out / 'rgb' / f'{index:04d}-{kind}.jpg'
            p.write_bytes(data); records[kind] = {'path': str(p.relative_to(self.out)), 'sha256': sha(data)}
        return own, top, records

    def execute(self, action):
        now = float(self.world.data.time)
        self.port.apply(action, now)
        duration = float(action['duration_s'])
        for _ in range(round(duration / self.world.model.opt.timestep)):
            self.port.tick(float(self.world.data.time))
            self.world._physics_step_for(self.world.controllers[self.rid])
            self._referee_tick()
            self._video_frame()
        self.port.tick(float(self.world.data.time))

    def _referee_tick(self, force=False):
        world, robot = self.world, self.world.controllers[self.rid]
        self.physics_steps += 1
        collisions = []
        for contact in world.data.contact[:world.data.ncon]:
            a, b = int(contact.geom1), int(contact.geom2)
            if ((a in self.robot_geoms and b in self.obstacle_geoms) or
                    (b in self.robot_geoms and a in self.obstacle_geoms)) and contact.dist <= 0:
                collisions.append([a, b])
        self.collision_steps += bool(collisions)
        self.weld_steps += bool(np.any(world.data.eq_active))
        xyz, rpy = robot.base_xyz(), robot.base_rpy()
        self.max_tilt_rad = max(self.max_tilt_rad, abs(float(rpy[0])), abs(float(rpy[1])))
        now = float(world.data.time)
        if force or not self.samples or now - self.samples[-1]['sim_time_s'] >= .099:
            sample = {'sim_time_s': now, 'robot_xyz_m': list(map(float, xyz)),
                'robot_rpy_rad': list(map(float, rpy)), 'map_contacts': collisions,
                'collision_steps_total': self.collision_steps, 'weld_steps_total': self.weld_steps}
            self.samples.append(sample); self.referee.write(json.dumps(sample) + '\n')

    def _video_frame(self, force=False):
        now = float(self.world.data.time)
        if not force and now < self.next_video_s:
            return
        data = self.world.render_team_jpeg(camera='cctv_warehouse', quality=92)
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        frame = cv2.resize(frame, (960, 720))
        cv2.rectangle(frame, (0, 0), (960, 42), (18, 18, 18), -1)
        cv2.putText(frame, f'{self.map["map_id"]} | {self.rid} | {self.last_status} | {now:.1f}s',
                    (15, 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (240, 240, 240), 1, cv2.LINE_AA)
        self.video.stdin.write(frame.tobytes()); self.frames += 1; self.next_video_s = now + .25

    def evaluate(self, actor_status):
        self._referee_tick(force=True)
        samples = self.samples
        xy = np.asarray(samples[-1]['robot_xyz_m'][:2])
        goal = self.map['zones']['goal']
        distance = float(np.linalg.norm(xy - np.asarray(goal['center_m'])))
        length = sum(float(np.linalg.norm(np.asarray(b['robot_xyz_m'][:2]) - a['robot_xyz_m'][:2])) for a,b in zip(samples, samples[1:]))
        xmin, xmax, ymin, ymax = self.map['bounds_m']
        radius = self.map['footprint']['unloaded_radius_m']
        inside = all(xmin + radius <= s['robot_xyz_m'][0] <= xmax - radius
                     and ymin + radius <= s['robot_xyz_m'][1] <= ymax - radius for s in samples)
        invariant = self.invariants() == self.initial_invariants and not self.initial_invariants['added_navigation_camera']
        gates = {'actor_arrived': actor_status == 'arrived', 'goal_distance': distance <= goal['radius_m'],
            'zero_map_collisions': self.collision_steps == 0, 'weld_off': self.weld_steps == 0,
            'footprint_inside_map': inside, 'upright': self.max_tilt_rad < math.radians(10), 'invariants': invariant}
        return {'success': all(gates.values()), 'gates': gates, 'metrics': {'goal_distance_m': distance,
            'path_length_m': length, 'collision_steps': self.collision_steps, 'physics_steps': self.physics_steps,
            'weld_steps': self.weld_steps, 'max_tilt_rad': self.max_tilt_rad,
            'sim_elapsed_s': samples[-1]['sim_time_s']-samples[0]['sim_time_s']},
            'scope': 'unloaded single active robot; all-step map contact/weld checks; sampled pose/path; output-only referee'}

    def close(self):
        try:
            if self.port:
                self.port.stop()
            if self.referee:
                self.referee.close()
            if self.video:
                try:
                    self.video.stdin.close()
                    code = self.video.wait(timeout=20)
                    if code:
                        raise RuntimeError(f'video encoding failed: {code}')
                finally:
                    if self.video.poll() is None:
                        self.video.terminate(); self.video.wait(timeout=5)
        finally:
            if self.world:
                self.world.close()
