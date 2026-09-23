"""Trusted local Python extensions; public interfaces contain no simulator truth.

Plugins run ordinary Python, not a security sandbox. Each reference is a .py
file relative to the config directory, followed by :callable_name.
"""
from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path
import types

from sim.camera_robot_port import validate_raw_action
from sim.scene_objects import validate_objects

RAW_KINDS = frozenset(("drive", "mecanum", "look", "arm", "wait"))


def validate_reference(ref):
    if not isinstance(ref, str) or ":" not in ref:
        raise ValueError("extension reference must be path/to/file.py:callable")
    path, name = ref.rsplit(":", 1)
    if not path.endswith(".py") or not name.isidentifier() or name.startswith("_"):
        raise ValueError("extension reference must be path/to/file.py:public_callable")
    return path, name


def validate_command(command, plugins, control):
    if not isinstance(command, dict):
        raise ValueError("command: object required")
    kind = command.get("kind")
    if isinstance(kind, str) and kind in plugins:
        if set(command) - {"kind", "params"} or not isinstance(command.get("params", {}), dict):
            raise ValueError("custom action requires kind and optional params object")
    else:
        validate_raw_action(command, **control)


class Extensions:
    def __init__(self, config, base_dir):
        self.config = config
        self.base_dir = Path(base_dir).resolve()
        self.sources = {}
        self._modules = {}
        self.actions = {name: self.load(ref) for name, ref in config["action_plugins"].items()}
        self.factories = {rid: self.load(spec["factory"]) for rid, spec in config["controllers"].items()}
        scene = config["scene"]
        extra = []
        if scene["builder"] is not None:
            extra = self.load(scene["builder"])(seed=scene["seed"], params=copy.deepcopy(scene["params"]))
        self.objects = validate_objects(scene["objects"] + validate_objects(extra))

    def load(self, ref):
        filename, name = validate_reference(ref)
        path = (self.base_dir / filename).resolve()
        if path not in self._modules:
            # Compile the recorded bytes directly: no stale .pyc when editing a
            # copied experiment rapidly, and one module instance per session.
            data = path.read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            module = types.ModuleType("ugrp_extension_" + digest)
            module.__file__ = str(path)
            sys.modules[module.__name__] = module
            try:
                exec(compile(data, str(path), "exec"), module.__dict__)
            finally:
                sys.modules.pop(module.__name__, None)
            self._modules[path] = module
            self.sources[str(path)] = {"sha256": digest, "bytes": data}
        function = getattr(self._modules[path], name, None)
        if not callable(function):
            raise ValueError(f"{ref}: callable not found")
        return function

    def lower(self, command):
        validate_command(command, self.actions, self.config["control"])
        kind = command["kind"]
        raw = (self.actions[kind](copy.deepcopy(command.get("params", {})))
               if kind in self.actions else command)
        # One call produces exactly one raw command. Stateful or multi-step
        # skills belong in controllers, where timing is explicit.
        json.dumps(raw, allow_nan=False)
        validate_raw_action(raw, **self.config["control"])
        return copy.deepcopy(raw)

    def controllers(self):
        instances = {}
        for rid, factory in self.factories.items():
            controller = factory(robot_id=rid, seed=self.config["scene"]["seed"],
                                 params=copy.deepcopy(self.config["controllers"][rid]["params"]))
            if not callable(getattr(controller, "act", None)):
                raise ValueError(f"controllers.{rid}: factory must return an object with act(observation)")
            instances[rid] = controller
        return instances
