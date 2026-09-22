"""Declarative extra geometry. No imports of MuJoCo or the robot runtime."""
from __future__ import annotations

import copy
import math
import re
import xml.etree.ElementTree as ET


def validate_objects(objects):
    if not isinstance(objects, list) or len(objects) > 256:
        raise ValueError("scene.objects: list of at most 256 objects required")
    resolved, names = [], set()
    defaults = {"shape": "box", "xyz_m": [0, 0, .1], "size_m": [.1, .1, .1],
                "euler_deg": [0, 0, 0], "dynamic": False, "mass_kg": 1.0,
                "rgba": [.6, .3, .1, 1], "friction": [1, .005, .0001]}
    for item in objects:
        if not isinstance(item, dict) or set(item) - (set(defaults) | {"name"}):
            raise ValueError("scene object: unknown fields or not an object")
        obj = {**copy.deepcopy(defaults), **copy.deepcopy(item)}
        name = obj.get("name")
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,63}", name) or name in names:
            raise ValueError("scene object name: unique identifier required")
        names.add(name)
        if obj["shape"] not in ("box", "sphere", "cylinder"):
            raise ValueError(f"{name}: shape must be box, sphere or cylinder")
        if type(obj["dynamic"]) is not bool:
            raise ValueError(f"{name}.dynamic: boolean required")
        sizes = {"box": 3, "sphere": 1, "cylinder": 2}[obj["shape"]]
        for key, length, lo, hi in (("xyz_m", 3, -100, 100), ("euler_deg", 3, -360, 360),
                                    ("size_m", sizes, .0001, 10), ("rgba", 4, 0, 1),
                                    ("friction", 3, 0, 10), ("mass_kg", None, .0001, 1000)):
            vals = [obj[key]] if length is None else obj[key]
            if not isinstance(vals, list) or len(vals) != (length or 1) or any(
                type(v) not in (int, float) or not math.isfinite(v) or not lo <= v <= hi for v in vals
            ):
                raise ValueError(f"{name}.{key}: {length or 1} finite values in [{lo}, {hi}] required")
        resolved.append(obj)
    return resolved


def append_objects(worldbody, objects):
    """Append namespaced bodies before compilation; sizes use MuJoCo half extents."""
    def vector(values):
        return " ".join(str(v) for v in values)

    for obj in validate_objects(objects):
        # Quaternion avoids dependence on the base MJCF compiler's angle unit.
        roll, pitch, yaw = (math.radians(v) / 2 for v in obj["euler_deg"])
        cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
        sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
        quat = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]
        name = "user__" + obj["name"]
        body = ET.SubElement(worldbody, "body", name=name, pos=vector(obj["xyz_m"]), quat=vector(quat))
        if obj["dynamic"]:
            ET.SubElement(body, "freejoint", name=name + "__joint")
        ET.SubElement(body, "geom", name=name + "__geom", type=obj["shape"],
                      size=vector(obj["size_m"]), mass=str(obj["mass_kg"]),
                      rgba=vector(obj["rgba"]), friction=vector(obj["friction"]),
                      contype="1", conaffinity="1", group="0")
