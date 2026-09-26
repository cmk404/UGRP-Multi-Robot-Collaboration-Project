"""Evaluation-only TOP camera profiles for the zone maps (versioned).

The zone maps author four fixed TOP cameras (``static_map['top_cameras']``, all
straight down, fovy 55 deg, z 2.5 m; ``sim/zone_arena.LAYOUTS``). Since the
2026-09-25/26 study decisions the TOP cameras are **evaluation-only**: they
feed the referee, videos and report stills, never a robot input (robots get
their own wrist RGB, the static map, the order sheet, their own command history
and their condition's dialogue channel). Environment v3 (walls 0.40 m, PR #208)
left the single-lane corridor of ``zone_wide_corridor`` partly hidden from the
TOP views (lane 0.869, passing bay 0.818 of the floor points; ST5 in
``experiments/2026-09-26-zone-env-v3``). Issue #218 (coordinator decision,
delegated by the user) moves the evaluation TOP for that map instead of the
walls.

This module never changes a map file, the scene XML or the authored camera
list. A profile is an overlay applied to a built world *after* ``scene.setup``
(which places the authored TOPs) and to a copy of the static map for the
evaluation tools that project TOP pixels onto the floor
(``harness/zone_perception.pixel_to_floor``). Physics and the robots' cameras
do not depend on these camera parameters.

Profiles (explicit id everywhere; there is no implicit default):

* ``zone_eval_top_v1`` -- the authored TOPs of every map, unchanged. Every
  record before this module used exactly these cameras.
* ``zone_eval_top_v2`` -- v1 plus one moved camera on the corridor base map
  (see ``PROFILES``); every other map is identical to v1.

Evidence and selection rule: ``experiments/2026-09-26-zone-eval-topcam``.
"""
from __future__ import annotations

import copy
import math
from typing import Any, Mapping

from sim.research_dispatch_arena import digest

SCHEMA = 'ugrp.zone_eval_top.v1'
STRAIGHT_DOWN = [1., 0., 0., 0.]
# SHA-256 (sim.research_dispatch_arena.digest) of the authored base map a camera
# override was measured on. An override is refused for any other geometry.
CORRIDOR_BASE_SHA256 = '262b8d7c3e73bcc6b8ff21d5ab64204ae869670c82141312593e99e590e96a20'
PROFILES: dict[str, dict[str, Any]] = {
    'zone_eval_top_v1': {
        'version': 1, 'overrides': {},
        'scope': 'authored TOP cameras of every zone map, unchanged (all records before 2026-09-26 #218)'},
}
OVERRIDE_KEYS = ('position_m', 'fov_y_deg')


class EvalTopError(ValueError):
    """A profile, map or world does not match the evaluation TOP contract."""


def _profile(profile_id):
    if not isinstance(profile_id, str) or profile_id not in PROFILES:
        raise EvalTopError(f'unknown evaluation TOP profile {profile_id!r}; choose one of {sorted(PROFILES)}')
    return PROFILES[profile_id]


def profile_record(profile_id: str) -> dict[str, Any]:
    """The profile as written into an evaluation record (id, parameters, parameter hash)."""
    value = {'schema': SCHEMA, 'id': profile_id, **copy.deepcopy(_profile(profile_id))}
    value['sha256'] = digest(value)
    return value


def base_map_of(static: Mapping[str, Any]) -> tuple[str, str]:
    """(base map id, base map SHA-256) of an authored or tagged/wall-profiled zone map."""
    if not isinstance(static, Mapping) or not isinstance(static.get('map_id'), str):
        raise EvalTopError('static map with a map_id is required')
    base = static.get('base_map')
    if base is not None:
        if not isinstance(base, Mapping) or not isinstance(base.get('map_id'), str) \
                or not isinstance(base.get('static_map_sha256'), str):
            raise EvalTopError('base_map needs map_id and static_map_sha256')
        return base['map_id'], base['static_map_sha256']
    return static['map_id'], digest(dict(static))


def _finite(value, what, n=None):
    items = value if n is not None else [value]
    if n is not None and (not isinstance(value, (list, tuple)) or len(value) != n):
        raise EvalTopError(f'{what}: {n} numbers required')
    for v in items:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
            raise EvalTopError(f'{what}: finite numbers required, got {v!r}')


def _check_camera(camera, what):
    if not isinstance(camera, Mapping) or not isinstance(camera.get('name'), str) or not camera['name']:
        raise EvalTopError(f'{what}: camera with a name required')
    _finite(camera.get('position_m'), f'{what}.position_m', 3)
    _finite(camera.get('fov_y_deg'), f'{what}.fov_y_deg')
    if not 0. < float(camera['fov_y_deg']) < 180.:
        raise EvalTopError(f'{what}.fov_y_deg must be in (0, 180)')
    # harness/zone_perception.pixel_to_floor supports only the authored straight-down TOP.
    if list(map(float, camera.get('quaternion_wxyz', ()))) != STRAIGHT_DOWN:
        raise EvalTopError(f'{what}: evaluation TOPs stay straight down (quaternion 1 0 0 0)')


def eval_top_cameras(static: Mapping[str, Any], profile_id: str) -> list[dict[str, Any]]:
    """Evaluation TOP cameras of ``static`` under ``profile_id`` (a new list; ``static`` unchanged)."""
    profile = _profile(profile_id)
    authored = static.get('top_cameras') if isinstance(static, Mapping) else None
    if not isinstance(authored, list) or not authored:
        raise EvalTopError('static map has no top_cameras')
    cameras = copy.deepcopy(authored)
    for i, camera in enumerate(cameras):
        _check_camera(camera, f'top_cameras[{i}]')
    names = [c['name'] for c in cameras]
    if len(set(names)) != len(names):
        raise EvalTopError('duplicate TOP camera names')
    base_id, base_sha = base_map_of(static)
    override = profile['overrides'].get(base_id)
    if override is None:
        return cameras
    if base_sha != override['base_static_map_sha256']:
        raise EvalTopError(f'{profile_id}: base map {base_id} geometry differs from the measured one; '
                           're-measure before using this profile')
    for name, change in override['cameras'].items():
        if name not in names:
            raise EvalTopError(f'{profile_id}: map {static["map_id"]} has no TOP {name}')
        camera = cameras[names.index(name)]
        camera.update({k: copy.deepcopy(change[k]) for k in OVERRIDE_KEYS if k in change})
        _check_camera(camera, f'{profile_id}.{name}')
    return cameras


def eval_static_map(static: Mapping[str, Any], profile_id: str) -> dict[str, Any]:
    """Copy of ``static`` for EVALUATION tools only: TOP cameras under the profile, profile recorded.

    Never pass the result to a robot, a prompt or a controller.
    """
    cameras = eval_top_cameras(static, profile_id)
    value = copy.deepcopy(dict(static))
    value['top_cameras'] = cameras
    if 'top_camera' in value:
        first = value['top_camera'].get('name') if isinstance(value['top_camera'], Mapping) else None
        match = [c for c in cameras if c['name'] == first]
        if not match:
            raise EvalTopError('top_camera is not one of top_cameras')
        value['top_camera'] = copy.deepcopy(match[0])
    value['eval_top_profile'] = profile_record(profile_id)
    return value


def _model_camera(world, name):
    try:
        return world.model.camera(name)
    except (KeyError, ValueError) as exc:
        raise EvalTopError(f'world has no camera {name}') from exc


def applied_cameras(world, cameras) -> list[dict[str, Any]]:
    """TOP parameters as they are in the compiled model now (read back, not assumed)."""
    rows = []
    for spec in cameras:
        cam = _model_camera(world, spec['name'])
        rows.append({'name': spec['name'], 'position_m': [round(float(v), 6) for v in cam.pos],
                     'quaternion_wxyz': [round(float(v), 6) for v in cam.quat],
                     'fov_y_deg': round(float(cam.fovy[0]), 6)})
    return rows


def _matches(row, spec):
    return (all(abs(a - float(b)) <= 1e-6 for a, b in zip(row['position_m'], spec['position_m']))
            and all(abs(a - float(b)) <= 1e-6 for a, b in zip(row['quaternion_wxyz'], spec['quaternion_wxyz']))
            and abs(row['fov_y_deg'] - float(spec['fov_y_deg'])) <= 1e-6)


def apply_to_world(world, static: Mapping[str, Any], profile_id: str) -> dict[str, Any]:
    """Place the evaluation TOPs of ``profile_id`` in a built world; call after every ``scene.setup``.

    Only ``model.cam_pos/cam_quat/cam_fovy`` of the TOP cameras change; physics
    state is only re-derived with ``mj_forward``. Returns the record to store
    next to the evaluation frames (profile, requested and read-back values).
    """
    import mujoco
    cameras = eval_top_cameras(static, profile_id)
    for spec in cameras:
        cam = _model_camera(world, spec['name'])
        cam.pos[:] = spec['position_m']
        cam.quat[:] = spec['quaternion_wxyz']
        cam.fovy[:] = spec['fov_y_deg']
    mujoco.mj_forward(world.model, world.data)
    return verify_world(world, static, profile_id)


def verify_world(world, static: Mapping[str, Any], profile_id: str) -> dict[str, Any]:
    """Raise unless the world's TOPs equal the profile (e.g. after a scene reset put v1 back)."""
    cameras = eval_top_cameras(static, profile_id)
    applied = applied_cameras(world, cameras)
    wrong = [s['name'] for s, row in zip(cameras, applied) if not _matches(row, s)]
    if wrong:
        raise EvalTopError(f'world TOPs {wrong} differ from {profile_id}; apply the profile after scene.setup')
    base_id, base_sha = base_map_of(static)
    return {'profile': profile_record(profile_id), 'map_id': static['map_id'], 'base_map_id': base_id,
            'base_static_map_sha256': base_sha, 'cameras': cameras, 'applied': applied,
            'cameras_sha256': digest(cameras), 'note': 'evaluation-only TOP cameras; never a robot input'}


__all__ = ['CORRIDOR_BASE_SHA256', 'EvalTopError', 'PROFILES', 'SCHEMA', 'applied_cameras', 'apply_to_world',
           'base_map_of', 'eval_static_map', 'eval_top_cameras', 'profile_record', 'verify_world']
