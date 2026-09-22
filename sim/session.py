"""A synchronous, single-owner API over the existing three-MasterPi physics world.

This owns a local simulator, not hardware or an LLM runtime. Controllers receive
copies from observe(); privileged state is exposed separately for evaluation.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import math
from contextlib import nullcontext

from sim.camera_robot_port import CameraRobotPort
from sim.session_config import ROBOTS, validate_config


class Simulation:
    def __init__(self, config, *, render=False, world_factory=None):
        self.config = validate_config(config)
        self.render = bool(render)
        self._viewer = None
        self._closed = False
        self.command_history = []
        self.episode = -1
        if world_factory is None:
            from sim.multi_masterpi_production import MultiMasterPiProductionV2
            world_factory = MultiMasterPiProductionV2
        scene = self.config["scene"]
        self._world = world_factory(seed=scene["seed"], warehouse_layout=scene["layout"],
                                    warehouse_cargo_ids=scene["cargo_ids"],
                                    **self.config["camera"], render=self.render)
        try:
            self.reset()
        except BaseException:
            self._world.close()
            self._closed = True
            raise

    def _check_open(self):
        if self._closed:
            raise RuntimeError("simulation is closed")

    def _lock(self):
        return self._viewer.lock() if self._viewer is not None else nullcontext()

    @property
    def timestep(self):
        return float(self._world.model.opt.timestep)

    @property
    def time(self):
        """Episode-relative physics time; excludes the world's reset settling."""
        return float(self._world.data.time) - self._start_time

    def reset(self):
        """Reset the same configured scene and command schedule, retaining viewer handles.

        A different layout/seed requires a new Simulation so compiled geometry
        cannot silently diverge from the requested configuration.
        """
        self._check_open()
        with self._lock():
            self._world.reset(seed=self.config["scene"]["seed"])
            for rid, pose in self.config["scene"]["robots"].items():
                self._world.robot(rid).set_base_pose_for_test(pose["xyz_m"], math.radians(pose["yaw_deg"]))
            # All assistance remains off; reset/setup poses are never actor observations.
            for names in ("grasp_weld_ids", "beam_weld_ids", "warehouse_weld_ids"):
                for eid in getattr(self._world, names, {}).values():
                    if eid >= 0:
                        self._world.data.eq_active[eid] = 0
            self._ports = {rid: CameraRobotPort(self._world, rid, **self.config["control"]) for rid in ROBOTS}
            self._start_time = float(self._world.data.time)
            self._next_action = 0
            self.episode += 1
            self.command_history.append({"event": "reset", "episode": self.episode, "at_s": 0.0})
        return {"episode": self.episode, "time_s": self.time, "timestep_s": self.timestep}

    def apply(self, robot, command):
        """Issue one raw actuator command without advancing physics; return an ACK."""
        self._check_open()
        if robot not in self._ports:
            raise ValueError(f"unknown robot: {robot}")
        with self._lock():
            ack = self._ports[robot].apply(command, float(self._world.data.time))
            self.command_history.append({"event": "command", "episode": self.episode,
                                         "at_s": self.time, "robot": robot,
                                         "command": copy.deepcopy(command), "ack": copy.deepcopy(ack)})
        return ack

    def step(self, steps=1):
        """Advance exactly N engine ticks, enforcing command leases on every tick.

        Configured actions execute immediately before the first tick at/after
        their episode-relative at_s. Same-time entries retain file order.
        """
        self._check_open()
        if type(steps) is not int or not 1 <= steps <= 1_000_000:
            raise ValueError("steps: integer in [1, 1000000] required")
        for _ in range(steps):
            actions = self.config["actions"]
            while self._next_action < len(actions) and actions[self._next_action]["at_s"] <= self.time + 1e-10:
                event = actions[self._next_action]
                self.apply(event["robot"], event["command"])
                self._next_action += 1
            with self._lock():
                for port in self._ports.values():
                    port.tick(float(self._world.data.time))
                self._world._physics_step_for(self._world.robot("r1"))
                for port in self._ports.values():
                    port.tick(float(self._world.data.time))
        return self.time

    def observe(self, robot, *, include_top=True):
        """Copy own calibrated RGB, shared top RGB and own issued command state.

        No measured poses, joints, contact, task success or peer commands.
        This is a synchronous API: call from the same owner thread as step().
        """
        self._check_open()
        if not self.render:
            raise RuntimeError("observe() requires Simulation(config, render=True)")
        if robot not in self._ports:
            raise ValueError(f"unknown robot: {robot}")
        result = self._ports[robot].capture()
        if include_top:
            jpeg = self._world.render_team_jpeg(camera="cctv_top")
            result["top_rgb"] = {"camera": "cctv_top", "image": base64.b64encode(jpeg).decode("ascii"),
                                 "sha256": hashlib.sha256(jpeg).hexdigest()}
        result["episode"] = self.episode
        result["episode_time_s"] = self.time
        return result

    def evaluation_state(self):
        """Privileged diagnostic output. Never pass this result to a controller."""
        self._check_open()
        return {"time_s": self.time,
                "robots_xyz_m": {rid: [float(x) for x in self._world.robot(rid).base_xyz()] for rid in ROBOTS},
                "active_equalities": int(sum(self._world.data.eq_active))}

    def launch_viewer(self, *, camera="free", key_callback=None):
        """Open MuJoCo's native viewer (use mjpython on macOS)."""
        self._check_open()
        if self._viewer is not None:
            raise RuntimeError("viewer already opened")
        import mujoco
        import mujoco.viewer
        camera_id = -1
        if camera != "free":
            camera_id = mujoco.mj_name2id(self._world.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
            if camera_id < 0:
                raise ValueError(f"unknown camera: {camera}")
        self._viewer = mujoco.viewer.launch_passive(self._world.model, self._world.data, key_callback=key_callback)
        with self._viewer.lock():
            if camera_id >= 0:
                self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                self._viewer.cam.fixedcamid = camera_id
            else:
                xmin, xmax, ymin, ymax = self._world.warehouse_navigation_bounds
                self._viewer.cam.lookat[:] = [(xmin + xmax) / 2, (ymin + ymax) / 2, 0.0]
                self._viewer.cam.distance = max(xmax - xmin, ymax - ymin) * 1.2
                self._viewer.cam.azimuth = 90
                self._viewer.cam.elevation = -55
        self.sync_viewer()
        return self._viewer

    def sync_viewer(self):
        self._check_open()
        if self._viewer is not None:
            # Never hold viewer.lock(): MuJoCo sync acquires it internally.
            self._viewer.sync()

    def close(self):
        if self._closed:
            return
        try:
            if self._viewer is not None:
                self._viewer.close()
            for port in self._ports.values():
                port.hold(float(self._world.data.time))
        finally:
            self._world.close()
            self._closed = True

    def __enter__(self):
        self._check_open()
        return self

    def __exit__(self, *_):
        self.close()
