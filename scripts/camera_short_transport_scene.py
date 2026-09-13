"""Physical execution and output-only referee for the short carry curriculum."""
from __future__ import annotations

import hashlib
import math

from scripts.camera_approach_scene import ApproachScene, ROBOTS


class ShortTransportScene(ApproachScene):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.weld_active_ticks = 0
        self.referee_ticks = 0

    def _referee_tick(self):
        self.referee_ticks += 1
        if any(self.world._beam_constraint_active(r) for r in ROBOTS):
            self.weld_active_ticks += 1
        super()._referee_tick()

    def evaluation_snapshot(self, *, full_state=True):
        result = super().evaluation_snapshot(full_state=full_state)
        others = []
        for contact in self.world.data.contact[:self.world.data.ncon]:
            a, b = int(contact.geom1), int(contact.geom2)
            if self.beam_geom in (a, b):
                others.append(b if a == self.beam_geom else a)
        # Only the static world body is ground/support in this unchanged fixture.
        result['payload_floor_contact'] = any(
            int(self.world.model.geom_bodyid[g]) == 0 for g in others)
        result['payload_robot_contact'] = any(g in self.robot_geoms for g in others)
        return result

    def invariant_record(self):
        """Geometry/camera fingerprints are experiment metadata, never actor inputs."""
        import mujoco
        m = self.world.model
        cameras = {}
        for name in ('cctv_top', 'r1__robot_cam', 'r3__robot_cam'):
            index = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name)
            if index < 0:
                raise ValueError(f'missing policy camera {name}')
            cameras[name] = {'position': m.cam_pos[index].tolist(),
                             'quaternion': m.cam_quat[index].tolist(),
                             'fov_y_deg': float(m.cam_fovy[index])}
        fields = ('geom_size', 'geom_pos', 'geom_quat', 'geom_rgba',
                  'geom_friction', 'body_mass', 'body_inertia')
        return {'policy_cameras': cameras,
                'geometry_sha256': {k: hashlib.sha256(getattr(m, k).tobytes()).hexdigest()
                                    for k in fields}}

    def carry_drive(self, forwards, duration_s=.20, *, stage='carry'):
        if set(forwards) != set(ROBOTS):
            raise ValueError('exactly r1 and r3 commands required')
        if stage not in ('carry', 'carry_stop'):
            raise ValueError('invalid carry phase')
        for value in (*forwards.values(), duration_s):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('finite numeric commands required')
        if any(not 0 <= v <= .10 for v in forwards.values()) or not 0 < duration_s <= .25:
            raise ValueError('carry forward must be 0..0.10 and duration 0..0.25')
        self.phase = stage
        start = self.time()
        actions = {r: {'kind': 'drive', 'forward': float(forwards[r]), 'turn': 0.,
                       'duration_s': float(duration_s)} for r in ROBOTS}
        for rid in ROBOTS:
            self.ports[rid].apply(actions[rid], start)
        self.tick(duration_s)
        self.trace.append({'stage': stage, 'actions': actions,
                           'start_sim_time_s': start, 'end_sim_time_s': self.time()})
        return actions

    def place(self):
        """Demonstrated command targets only; no joint/contact feedback or port arm cache."""
        lifted = {r: {ch: self.commands[r][ch] for ch in (3, 4, 5)} for r in ROBOTS}
        preclose = {r: {ch: self.grasp_report['preclose_issued_commands'][r][ch]
                        for ch in (3, 4, 5)} for r in ROBOTS}
        self.replay([{'targets': preclose, 'duration_s': .70, 'settle_s': .25}], 'place_lower')
        self.capture('place-lowered')
        self.replay([{'targets': {r: {1: 2000} for r in ROBOTS},
                      'duration_s': .65, 'settle_s': .50}], 'place_open')
        self.capture('place-opened')
        self.replay([{'targets': lifted, 'duration_s': .70, 'settle_s': .25}], 'place_retract')
        self.phase = 'release_hold'
        self.tick(2.2)
        self.capture('release-final')
