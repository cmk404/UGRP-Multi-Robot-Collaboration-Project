"""Versioned, dependency-light configuration for local simulation sessions."""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path

from sim.scene_objects import validate_objects
from sim.session_extensions import RAW_KINDS, validate_command, validate_reference

LAYOUTS = ("standard", "mixed", "arena", "camera_team")
ROBOTS = ("r1", "r2", "r3")
DEFAULT_CONFIG = {
    "version": 1,
    "scene": {"layout": "camera_team", "seed": 41, "cargo_ids": None, "robots": {},
              "objects": [], "builder": None, "params": {}},
    "camera": {"width": 384, "height": 288},
    "control": {"allow_reverse": True, "allow_mecanum": True},
    "run": {"sim_seconds": 30.0, "wall_seconds": 1800.0, "realtime_factor": 1.0},
    "actions": [],
    "action_plugins": {},
    "controllers": {},
}


def number(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name}: finite number in [{low}, {high}] required")
    return value


def _object(value, allowed, name):
    if not isinstance(value, dict) or set(value) - set(allowed):
        raise ValueError(f"{name}: object with fields {sorted(allowed)} required")


def validate_config(value):
    """Return an independent, fully resolved config; reject typos and unsafe commands."""
    _object(value, DEFAULT_CONFIG, "config")
    if type(value.get("version")) is not int or value["version"] != 1:
        raise ValueError("version: supported version is 1")
    config = copy.deepcopy(DEFAULT_CONFIG)
    for group in ("scene", "camera", "control", "run"):
        supplied = value.get(group, {})
        _object(supplied, config[group], group)
        config[group].update(copy.deepcopy(supplied))
    scene = config["scene"]
    if scene["layout"] not in LAYOUTS:
        raise ValueError(f"scene.layout: choose from {LAYOUTS}")
    if type(scene["seed"]) is not int or not 0 <= scene["seed"] <= 1_000_000:
        raise ValueError("scene.seed: integer in [0, 1000000] required")
    ids = scene["cargo_ids"]
    if ids is not None and (not isinstance(ids, list) or not ids or
                           any(not isinstance(x, str) or not x for x in ids) or len(set(ids)) != len(ids)):
        raise ValueError("scene.cargo_ids: null or nonempty unique string list required")
    _object(scene["robots"], ROBOTS, "scene.robots")
    for rid, pose in scene["robots"].items():
        _object(pose, ("xyz_m", "yaw_deg"), f"scene.robots.{rid}")
        if set(pose) != {"xyz_m", "yaw_deg"} or not isinstance(pose["xyz_m"], list) or len(pose["xyz_m"]) != 3:
            raise ValueError(f"scene.robots.{rid}: xyz_m [x,y,z] and yaw_deg required")
        for xyz in pose["xyz_m"]:
            number(xyz, f"{rid}.xyz_m", -100, 100)
        number(pose["yaw_deg"], f"{rid}.yaw_deg", -360, 360)
    scene["objects"] = validate_objects(scene["objects"])
    if scene["builder"] is not None:
        validate_reference(scene["builder"])
    if not isinstance(scene["params"], dict):
        raise ValueError("scene.params: object required")
    for name, size in config["camera"].items():
        if type(size) is not int or not 32 <= size <= 2048:
            raise ValueError(f"camera.{name}: integer in [32, 2048] required")
    for name, flag in config["control"].items():
        if type(flag) is not bool:
            raise ValueError(f"control.{name}: boolean required")
    number(config["run"]["sim_seconds"], "run.sim_seconds", .001, 86400)
    number(config["run"]["wall_seconds"], "run.wall_seconds", .1, 86400)
    number(config["run"]["realtime_factor"], "run.realtime_factor", .01, 100)
    plugins = copy.deepcopy(value.get("action_plugins", {}))
    if not isinstance(plugins, dict):
        raise ValueError("action_plugins: name -> file.py:callable object required")
    for name, ref in plugins.items():
        if not isinstance(name, str) or not name.isidentifier() or name in RAW_KINDS:
            raise ValueError("action_plugins: unique custom name; built-in names cannot be replaced")
        validate_reference(ref)
    config["action_plugins"] = plugins
    controllers = copy.deepcopy(value.get("controllers", {}))
    _object(controllers, ROBOTS, "controllers")
    for rid, supplied in controllers.items():
        _object(supplied, ("factory", "params", "period_s"), f"controllers.{rid}")
        spec = {"params": {}, "period_s": .2, **supplied}
        validate_reference(spec.get("factory"))
        if not isinstance(spec["params"], dict):
            raise ValueError(f"controllers.{rid}.params: object required")
        number(spec["period_s"], f"controllers.{rid}.period_s", .01, 60)
        controllers[rid] = spec
    config["controllers"] = controllers
    actions = copy.deepcopy(value.get("actions", []))
    if not isinstance(actions, list):
        raise ValueError("actions: list required")
    for event in actions:
        _object(event, ("at_s", "robot", "command"), "action event")
        if set(event) != {"at_s", "robot", "command"} or event["robot"] not in ROBOTS:
            raise ValueError("action event requires at_s, robot (r1/r2/r3), command")
        number(event["at_s"], "action.at_s", 0, config["run"]["sim_seconds"])
        if event["at_s"] >= config["run"]["sim_seconds"]:
            raise ValueError("action.at_s must be before run.sim_seconds")
        if event["robot"] in controllers:
            raise ValueError("scheduled actions and controller cannot own the same robot")
        validate_command(event["command"], plugins, config["control"])
    config["actions"] = sorted(actions, key=lambda event: event["at_s"])
    return config


def load_config(path: str | Path):
    return validate_config(json.loads(Path(path).read_text(encoding="utf-8")))
