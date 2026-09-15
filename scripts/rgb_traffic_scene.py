"""Two simultaneous unloaded robots; setup/referee are private to the runner."""
from __future__ import annotations

import math
import subprocess
from unittest.mock import patch

import cv2
import mujoco
import numpy as np

from scripts.known_map_scene import KnownMapScene, FIXED_TOP, FOLDED_COMMANDS, course_xml, sha
from sim.camera_robot_port import CameraRobotPort
from sim.authored_navigation_map import map_sha256
import sim.multi_masterpi_production as production


class RGBTrafficScene(KnownMapScene):
    def __init__(self, out, maps, setup):
        super().__init__(out, maps['r1'], 'r1', setup['r1'])
        self.maps, self.setup = maps, setup
        self.units = tuple(sorted(maps))
        self.ports = {}
        self.peer_collision_steps = 0
        self.contact_events = []

    def open(self):
        if self.units != ('r1', 'r3') or any(m['top_camera'] != FIXED_TOP for m in self.maps.values()):
            raise ValueError('two unloaded r1/r3 and preserved cameras required')
        if any(m['obstacles'] != self.map['obstacles'] for m in self.maps.values()):
            raise ValueError('one shared physical map required')
        original = production.build_multi_robot_xml
        def builder(*args, **kwargs):
            kwargs['navigation_camera'] = False
            xml, self.robot_xml_sha = course_xml(original(*args, **kwargs), self.map)
            self.scene_xml_sha = sha(xml.encode())
            return xml
        with patch.object(production, 'build_multi_robot_xml', builder):
            self.world = production.MultiMasterPiProductionV2(seed=11, width=960, height=720, render=True)
        world = self.world
        # Ground truth only initializes the private fixture, before execution.
        for i, unit in enumerate(world.robot_ids):
            world.controllers[unit].set_base_pose_for_test((-1.2 + 1.2*i, .6, .0325), 0.)
        for unit, case in self.setup.items():
            x, y = case['start_xy_m']
            world.controllers[unit].set_base_pose_for_test((x, y, .0325), math.radians(case['start_yaw_deg']))
        world._team_joint_move_servos({u: FOLDED_COMMANDS for u in self.units}, .6, settle_s=.5)
        m = world.model
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_top')
        m.cam_pos[cid] = FIXED_TOP['position_m']; m.cam_quat[cid] = FIXED_TOP['quaternion_wxyz']
        m.cam_fovy[cid] = FIXED_TOP['fov_y_deg']
        # Same presentation camera as the established unloaded-map runner.
        observer = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, 'cctv_warehouse')
        position, target = np.array([.55, -4.3, 3.0]), np.array([.55, -2., 0.])
        forward = target-position; forward /= np.linalg.norm(forward)
        right = np.cross(forward, [0., 0., 1.]); right /= np.linalg.norm(right)
        up = np.cross(right, forward); quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, np.column_stack((right, up, -forward)).ravel())
        m.cam_pos[observer] = position; m.cam_quat[observer] = quat; m.cam_fovy[observer] = 55.
        mujoco.mj_forward(m, world.data)
        mujoco.mj_saveLastXML(str(self.out/'scene.xml'), m)
        self.ports = {u: CameraRobotPort(world, u, allow_reverse=True, allow_mecanum=True) for u in self.units}
        self.port = self.ports['r1']
        self.geoms = {u: {i for i in range(m.ngeom) if
            (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith(u+'__')} for u in self.units}
        self.robot_geoms = set.union(*self.geoms.values())
        self.obstacle_geoms = {i for i in range(m.ngeom) if
            (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('known_map_') and m.geom_contype[i]}
        self.referee = (self.out/'evaluation-only.jsonl').open('w')
        self.video = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', '960x720', '-r', '4', '-i', '-', '-an', '-c:v', 'libx264',
            '-preset', 'veryfast', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
            str(self.out/'motion.mp4')], stdin=subprocess.PIPE)
        self.initial_invariants = self.invariants()
        self._referee_tick(force=True); self._video_frame(force=True)
        return self

    def invariants(self):
        record = super().invariants()
        m = self.world.model
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, 'r3__robot_cam')
        record['actor_cameras']['r3__robot_cam'] = {'position': m.cam_pos[cid].tolist(),
            'quaternion': m.cam_quat[cid].tolist(), 'fovy': float(m.cam_fovy[cid])}
        record['actor_maps_sha256'] = {u: map_sha256(d) for u, d in self.maps.items()}
        return record

    def capture(self, index):
        top = self.world.render_team_jpeg(camera='cctv_top', quality=95)
        top_path = self.out/'rgb'/f'{index:04d}-top.jpg'; top_path.write_bytes(top)
        frames, records = {}, {}
        for unit in self.units:
            own = self.world.render_jpeg(robot_id=unit, camera='robot_cam', quality=95)
            own_path = self.out/'rgb'/f'{index:04d}-{unit}-own.jpg'; own_path.write_bytes(own)
            frames[unit] = (own, top)
            records[unit] = {k: {'path': str(p.relative_to(self.out)), 'sha256': sha(data)}
                for k, p, data in [('own', own_path, own), ('top', top_path, top)]}
        return frames, records

    def execute(self, actions, *, drive_blocked=()):
        now = float(self.world.data.time)
        for unit, action in actions.items():
            self.ports[unit].apply(action, now)
            if unit in drive_blocked:
                self.ports[unit].stop()
        duration = max(a['duration_s'] for a in actions.values())
        for _ in range(round(duration/self.world.model.opt.timestep)):
            for port in self.ports.values(): port.tick(float(self.world.data.time))
            self.world._physics_step_for(self.world.controllers['r1'])
            self._referee_tick(); self._video_frame()
        for port in self.ports.values(): port.tick(float(self.world.data.time))
        return duration

    def _referee_tick(self, force=False):
        import json
        world = self.world
        self.physics_steps += 1
        hits, peer = [], False
        for c in world.data.contact[:world.data.ncon]:
            if c.dist > 0: continue
            a, b = int(c.geom1), int(c.geom2)
            wall = ((a in self.robot_geoms and b in self.obstacle_geoms) or
                    (b in self.robot_geoms and a in self.obstacle_geoms))
            between = any(a in self.geoms[u] and b in self.geoms[v]
                          for u in self.units for v in self.units if u != v)
            if wall or between: hits.append([a, b])
            peer |= between
        self.collision_steps += bool(hits); self.peer_collision_steps += peer
        self.weld_steps += bool(np.any(world.data.eq_active))
        now = float(world.data.time)
        if hits: self.contact_events.append({'sim_time_s': now, 'geom_pairs': hits})
        if force or not self.samples or now-self.samples[-1]['sim_time_s'] >= .099:
            poses = {u: {'xyz_m': list(map(float, world.controllers[u].base_xyz())),
                         'rpy_rad': list(map(float, world.controllers[u].base_rpy()))} for u in self.units}
            self.max_tilt_rad = max(self.max_tilt_rad, *(abs(v) for p in poses.values() for v in p['rpy_rad'][:2]))
            sample = {'sim_time_s': now, 'robots': poses, 'collision_steps_total': self.collision_steps,
                'peer_collision_steps_total': self.peer_collision_steps, 'weld_steps_total': self.weld_steps}
            self.samples.append(sample); self.referee.write(json.dumps(sample)+'\n')

    def evaluate(self, statuses):
        self._referee_tick(force=True)
        goals = {u: math.dist(self.samples[-1]['robots'][u]['xyz_m'][:2], d['zones']['goal']['center_m'])
                 for u, d in self.maps.items()}
        separation = min(math.dist(s['robots']['r1']['xyz_m'][:2], s['robots']['r3']['xyz_m'][:2]) for s in self.samples)
        xmin, xmax, ymin, ymax = self.map['bounds_m']; radius = self.map['footprint']['unloaded_radius_m']
        inside = all(xmin+radius <= s['robots'][u]['xyz_m'][0] <= xmax-radius and
                     ymin+radius <= s['robots'][u]['xyz_m'][1] <= ymax-radius for s in self.samples for u in self.units)
        gates = {'both_rgb_arrived': all(s == 'arrived' for s in statuses.values()),
            'both_truth_goals': all(goals[u] <= self.maps[u]['zones']['goal']['radius_m'] for u in self.units),
            'zero_collisions': self.collision_steps == 0, 'weld_off': self.weld_steps == 0,
            'inside_map': inside, 'upright': self.max_tilt_rad < math.radians(10),
            'invariants': self.invariants() == self.initial_invariants and not self.initial_invariants['added_navigation_camera']}
        return {'success': all(gates.values()), 'gates': gates, 'goal_distances_m': goals,
            'sampled_min_center_separation_m': separation,
            'sampled_min_circle_clearance_m': separation-2*radius,
            'collision_steps': self.collision_steps, 'peer_collision_steps': self.peer_collision_steps,
            'weld_steps': self.weld_steps, 'max_tilt_rad': self.max_tilt_rad,
            'sim_elapsed_s': self.samples[-1]['sim_time_s']-self.samples[0]['sim_time_s']}

    def close(self):
        try:
            for port in self.ports.values(): port.stop()
        finally:
            super().close()
