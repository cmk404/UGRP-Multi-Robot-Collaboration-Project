#!/usr/bin/env python3
"""Teacher renders of the tag-free environment v3 (training labels + evaluation episodes).

What it runs: the M1 own-camera delivery runner (``scripts/run_m1_owncam.py``,
pinned source ``kiro/zone-map-v3`` at ``tagfree_scene.V3_SOURCE_SHA``) byte for
byte -- the same 5 Hz own-frame loop, macro execution, command logging, GT
trajectory and outcome records -- with these documented teacher substitutions
(AGENTS.md teacher exception: demonstrations and training targets may use
ground truth; nothing here is a student result):

1. scene: ``TagFreeZoneScene`` (base map + walls_v3 0.40 m walls + zero tags)
   instead of ``TaggedZoneScene``;
2. pose: the controller's pose source is the simulator truth
   (``teacher_gt_eval_only``) instead of the tag particle filter, so the robot is
   driven by the teacher. The controller logic (search viewpoints, pregrasp
   approach, carry leg with its distance/door-checkpoint looks, pre-place and
   look-back looks, re-anchoring) is the M1 controller's own; covariance-triggered
   looks never fire because the teacher is certain;
3. skill v9 runs in ``mode='diagnostic'`` (it accepts the teacher pose);
4. labels: at every own ``robot_cam`` capture the same camera is also rendered
   as a MuJoCo segmentation image (ideal pinhole, before the fisheye remap) and
   the true camera pose is stored. Both go to ``eval_only/`` (teacher/training
   data only).

After the run the output is split into:

* ``frames/`` + ``inputs/commands.jsonl`` + ``inputs/frames.jsonl`` (student view:
  no pose report) + ``inputs/motion_profile.jsonl`` (the controller's own
  manipulation-phase switches, as the M1 PF used them) -- the only files a
  student localizer may read;
* ``teacher/`` -- controller/skill events and the frames rows with the teacher's
  (GT) pose reports;
* ``eval_only/`` -- GT trajectory, per-frame GT, segmentation labels, camera poses.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
V3_ROOT = Path(os.environ.get('UGRP_V3_SOURCE', '/Users/changmin/projects/ugrp-wt/kiro-vision-loc-v3src'))
sys.path.insert(0, str(V3_ROOT))
sys.path.insert(0, str(HERE))

import tagfree_scene  # noqa: E402

SCHEMA = 'ugrp.vision_loc_teacher_render.v1'
TEACHER_SOURCE = 'teacher_gt_eval_only'
TEACHER_STD_XY_M, TEACHER_STD_YAW_RAD = .01, .005
# Segmentation label classes (ideal pinhole robot_cam view).
CLASSES = {'floor': 0, 'wall': 1, 'self': 2, 'object': 3, 'background': 4}
IGNORE = 255
OWN_FILES = ('run_vl_teacher_render.py', 'tagfree_scene.py', 'episodes.json',
             'maps/zone_wide_door_walls_v3_notags.json')


def sha_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def git(root: Path, *args) -> str:
    return subprocess.run(['git', *args], cwd=root, capture_output=True, text=True).stdout.strip()


def check_sources() -> dict:
    head = git(V3_ROOT, 'rev-parse', 'HEAD')
    if head != tagfree_scene.V3_SOURCE_SHA:
        raise SystemExit(f'v3 source tree {V3_ROOT} is at {head}, expected {tagfree_scene.V3_SOURCE_SHA}')
    dirty = git(V3_ROOT, 'status', '--porcelain', '--untracked-files=no')
    if dirty:
        raise SystemExit(f'v3 source tree is dirty:\n{dirty}')
    own_root = Path(git(HERE, 'rev-parse', '--show-toplevel'))
    rel = HERE.relative_to(own_root)
    own_dirty = git(own_root, 'status', '--porcelain', '--', *[str(rel/f) for f in OWN_FILES])
    return {'v3_source': {'branch': tagfree_scene.V3_SOURCE_BRANCH, 'sha': head, 'path': str(V3_ROOT), 'dirty': False},
            'render_source': {'repo': str(own_root), 'sha': git(own_root, 'rev-parse', 'HEAD'),
                              'dirty_own_files': bool(own_dirty),
                              'files_sha256': {f: sha_bytes((HERE/f).read_bytes()) for f in OWN_FILES}}}


# ----------------------------------------------------------------------------- teacher pose
REGISTRY: dict = {}


def make_teacher_classes():
    import numpy as np
    from harness import m1_owncam_delivery as delivery
    from harness.owncam_localizer import OwnCamLocalizer
    from harness.owncam_pose_source import PoseReport
    from sim import multi_masterpi_production as mmp

    class RegisteringWorld(mmp.MultiMasterPiProductionV2):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            REGISTRY['world'] = self

    class TeacherLocalizer(OwnCamLocalizer):
        """The M1 localizer's command/load bookkeeping; the estimate is the simulator truth."""

        def __init__(self, static_map, params, robot_id, seed=0):
            super().__init__(static_map, params, seed=seed)
            self.robot_id = robot_id
            self.initialized = True

        def truth(self):
            r = REGISTRY['world'].robot(self.robot_id)
            xyz, rpy = r.base_xyz(), r.base_rpy()
            return float(xyz[0]), float(xyz[1]), float(rpy[2])

        def predict_to(self, t):
            self.t = max(self.t, float(t))

        def update(self, t, detections, commanded_pose=None):
            self.t = max(self.t, float(t))
            return self.estimate()

        def estimate(self):
            x, y, yaw = self.truth()
            sx, sy = TEACHER_STD_XY_M/np.sqrt(2), TEACHER_STD_XY_M/np.sqrt(2)
            return {'t': round(self.t, 4), 'initialized': True, 'x': x, 'y': y, 'yaw': yaw,
                    'cov': [[sx*sx, 0., 0.], [0., sy*sy, 0.], [0., 0., TEACHER_STD_YAW_RAD**2]],
                    'std_xy_m': TEACHER_STD_XY_M, 'std_yaw_rad': TEACHER_STD_YAW_RAD, 'n_eff': float(self.n),
                    'since_tag_s': 0.0}

    class TeacherPoseSource:
        """Duck-typed ``OwnCamPoseSource`` on the teacher localizer (no image is used)."""

        def __init__(self, static_map, params, robot_id, seed=0):
            self.loc = TeacherLocalizer(static_map, params, robot_id, seed=seed)
            self.source = TEACHER_SOURCE
            self.servo, self.last_obs, self.frames = {}, None, 0

        def on_command(self, row):
            self.loc.command(row)

        def set_motion_profile(self, now, name):
            if name != self.loc.motion_profile:
                self.loc.set_motion_profile(now, name)

        def on_frame(self, now, rgb):
            self.frames += 1
            return self.report(now)

        def report(self, now):
            self.loc.predict_to(now)
            est = self.loc.estimate()
            return PoseReport(t_est=float(now), initialized=True, x_m=est['x'], y_m=est['y'], yaw_rad=est['yaw'],
                              cov=tuple(tuple(r) for r in est['cov']), std_xy_m=est['std_xy_m'],
                              std_yaw_rad=est['std_yaw_rad'], since_tag_s=0.0, last_valid_obs=None,
                              n_eff=float(self.loc.n), load_state='loaded' if self.loc.load.loaded else 'unloaded',
                              source=self.source)

    class TeacherDelivery(delivery.M1OwnCamDelivery):
        """M1 controller driven by the teacher pose (GT)."""

        def __init__(self, static_map, params, **kwargs):
            super().__init__(static_map, params, **kwargs)
            self.pose = TeacherPoseSource(static_map, params, kwargs.get('robot_id', 'r1'), seed=kwargs.get('seed', 0))

        def _estimate(self, report):
            est = self.pose_estimate_cls(report.x_m, report.y_m, report.yaw_rad, report.source)
            self.pose_sources.add(est.source)
            return est

    return RegisteringWorld, TeacherDelivery


# ----------------------------------------------------------------------------- labels
def make_labeling_port(state: dict):
    import cv2
    import mujoco
    import numpy as np
    from sim import camera_robot_port

    class LabelingPort(camera_robot_port.CameraRobotPort):
        def capture(self, camera='robot_cam'):
            obs = super().capture(camera)
            if camera == 'robot_cam' and self.robot_id == state.get('rid'):
                label, cam = render_label(self._world, self.robot_id, state)
                out = state['out']/'eval_only'/'labels'
                out.mkdir(parents=True, exist_ok=True)
                name = f"{int(obs['frame_id']):05d}.png"
                ok, png = cv2.imencode('.png', label)
                (out/name).write_bytes(png.tobytes())
                state['rows'].append({'frame_id': int(obs['frame_id']), 'sim_time': round(float(obs['sim_time']), 4),
                                      'frame_sha256': obs['sha256'], 'label': f'eval_only/labels/{name}',
                                      'label_sha256': sha_bytes(png.tobytes()), **cam})
            return obs

    def class_table(model, rid):
        table = np.full(model.ngeom, CLASSES['object'], np.uint8)
        for g in range(model.ngeom):
            n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
            if n == 'floor' or (n.startswith('zone_') and not n.startswith('zone_wall_')):
                table[g] = CLASSES['floor']
            elif n.startswith('zone_wall_'):
                table[g] = CLASSES['wall']
            elif n.startswith(rid + '__'):
                table[g] = CLASSES['self']
        return table

    def render_label(world, rid, state):
        robot = world.robot(rid)
        if 'table' not in state:
            state['table'] = class_table(world.model, rid)
            state['class_geoms'] = {c: [mujoco.mj_id2name(world.model, mujoco.mjtObj.mjOBJ_GEOM, g)
                                        for g in np.flatnonzero(state['table'] == v)][:400]
                                    for c, v in CLASSES.items() if c != 'background'}

        def fn():
            with world.physics_lock, world.render_lock:
                robot._sync_real_camera_mount()
                r = world.renderer
                r.enable_segmentation_rendering()
                try:
                    r.update_scene(world.data, camera=robot._n('robot_cam'),
                                   scene_option=robot._robot_sensor_scene_option)
                    seg = r.render().copy()
                finally:
                    r.disable_segmentation_rendering()
                cam = world.data.camera(robot._n('robot_cam'))
                pose = {'cam_pos_m': [round(float(v), 6) for v in cam.xpos],
                        'cam_xmat': [round(float(v), 7) for v in cam.xmat]}
                xyz, rpy = robot.base_xyz(), robot.base_rpy()
                pose['base_gt'] = [round(float(xyz[0]), 5), round(float(xyz[1]), 5), round(float(rpy[2]), 6)]
            return seg, pose
        if threading.get_ident() == world._render_thread_id:
            seg, pose = fn()
        else:
            seg, pose = world._render_executor.submit(fn).result(timeout=30.0)
        objid, objtype = seg[..., 0], seg[..., 1]
        label = np.full(objid.shape, CLASSES['background'], np.uint8)
        geom = (objtype == int(mujoco.mjtObj.mjOBJ_GEOM)) & (objid >= 0)
        label[geom] = state['table'][objid[geom]]
        other = (objid >= 0) & ~geom
        label[other] = CLASSES['object']
        return label, pose

    return LabelingPort


# ----------------------------------------------------------------------------- split outputs
def split_outputs(out: Path, label_rows: list, moved: dict) -> None:
    """Student view in inputs/, teacher raw in teacher/, GT in eval_only/ (see module docstring)."""
    teacher = out/'teacher'
    teacher.mkdir(exist_ok=True)
    frames_path = out/'inputs'/'frames.jsonl'
    rows = [json.loads(line) for line in frames_path.read_text().splitlines() if line.strip()]
    shutil.move(str(frames_path), teacher/'frames_with_teacher_report.jsonl')
    moved['inputs/frames.jsonl'] = 'teacher/frames_with_teacher_report.jsonl'
    student = [{k: v for k, v in r.items() if k != 'report'} for r in rows]
    frames_path.write_text(''.join(json.dumps(r) + '\n' for r in student))
    for name in ('controller_events.jsonl', 'skill_events.jsonl', 'macros.jsonl'):
        if (out/name).exists():
            shutil.move(str(out/name), teacher/name)
            moved[name] = f'teacher/{name}'
    events = [json.loads(line) for line in (teacher/'controller_events.jsonl').read_text().splitlines() if line.strip()]
    prof = [{'t': e['t'], 'profile': e.get('profile')} for e in events if e.get('event') == 'motion_profile']
    (out/'inputs'/'motion_profile.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in prof))
    (out/'eval_only'/'labels.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in label_rows))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--episodes', default=str(HERE/'episodes.json'))
    p.add_argument('--only', required=True, help='comma-separated episode ids')
    p.add_argument('--output', required=True)
    p.add_argument('--smoke-sim-limit', type=float, default=None,
                   help='smoke test only: stop at this SIM time (recorded; never a dataset episode)')
    p.add_argument('--allow-dirty', action='store_true', help='smoke only: uncommitted render files')
    args = p.parse_args(argv)
    sources = check_sources()
    if sources['render_source']['dirty_own_files'] and not (args.allow_dirty and args.smoke_sim_limit):
        raise SystemExit('commit the render files before a dataset render (--allow-dirty only with --smoke-sim-limit)')
    table = json.loads(Path(args.episodes).read_text())
    student = dict(table['controller'])
    wanted = [s for s in args.only.split(',') if s]
    by_id = {e['episode_id']: e for e in table['episodes']}
    unknown = [e for e in wanted if e not in by_id]
    if unknown:
        raise SystemExit(f'unknown episodes {unknown}')

    import sim.camera_robot_port as crp
    import sim.multi_masterpi_production as mmp
    import sim.zone_landmarks as zl
    from harness import m1_owncam_delivery as delivery
    from scripts import run_m1_owncam as m1run
    scene_cls = tagfree_scene.make_scene_class()
    world_cls, teacher_cls = make_teacher_classes()
    state = {'rows': []}
    port_cls = make_labeling_port(state)

    class TagFreeShim:
        @classmethod
        def from_tagged(cls, name, seed, goal, extra_boxes=None, contact_profile=None):
            base = table['base_map']
            if name != tagfree_scene.map_id(base):
                raise ValueError(f'teacher render expects map {tagfree_scene.map_id(base)}, got {name}')
            return scene_cls.from_tagfree(base, seed, goal, extra_boxes, contact_profile=contact_profile)

    patches = {'sim.zone_landmarks.TaggedZoneScene': 'TagFreeZoneScene (tagfree_scene.py)',
               'sim.multi_masterpi_production.MultiMasterPiProductionV2': 'subclass registering the world (truth access)',
               'harness.m1_owncam_delivery.M1OwnCamDelivery': 'TeacherDelivery (pose = simulator truth)',
               'sim.camera_robot_port.CameraRobotPort': 'LabelingPort (segmentation label + camera pose per own frame)',
               "scripts.run_m1_owncam.SKILLS['v9']": "mode 'diagnostic' (accepts the teacher pose)"}
    zl.TaggedZoneScene = TagFreeShim
    mmp.MultiMasterPiProductionV2 = world_cls
    delivery.M1OwnCamDelivery = teacher_cls
    crp.CameraRobotPort = port_cls
    module, name, kind, kwargs = m1run.SKILLS['v9']
    m1run.SKILLS['v9'] = (module, name, kind, {**kwargs, 'mode': 'diagnostic'})
    if args.smoke_sim_limit:
        m1run.SIM_LIMIT_S = float(args.smoke_sim_limit)
        patches['scripts.run_m1_owncam.SIM_LIMIT_S'] = f'{args.smoke_sim_limit} (smoke test)'

    for ep in wanted:
        spec = dict(by_id[ep])
        spec['map'] = tagfree_scene.map_id(table['base_map'])
        spec['contact_profile'] = student['contact_profile']
        out = Path(args.output)/ep
        state.update(rows=[], out=out)
        state.pop('table', None)

        # The recorded robot id is chosen inside run() (spawn row nearest spawn_y); resolve it the same way.
        base = table['base_map']
        from sim.zone_arena import episode as zone_episode
        cfg = zone_episode(base, spec['seed'], goal=spec['goal'], extra_boxes=spec.get('extra_boxes'))
        spawns = cfg['setup_only']['spawns']
        state['rid'] = min(spawns, key=lambda r: abs(spawns[r][1] - spec['spawn_y']))
        started, load0 = time.time(), os.getloadavg()
        result, manifest = m1run.run(spec, out, student)
        moved = {}
        split_outputs(out, state['rows'], moved)
        teacher_manifest = {
            'schema': SCHEMA, 'episode': ep, 'split': spec['split'], 'role': spec['role'], 'robot_id': state['rid'],
            'sources': sources, 'patches': patches, 'teacher_pose_source': TEACHER_SOURCE,
            'teacher_pose_std': {'xy_m': TEACHER_STD_XY_M, 'yaw_rad': TEACHER_STD_YAW_RAD},
            'static_map': {'map_id': spec['map'], 'file': 'maps/zone_wide_door_walls_v3_notags.json',
                           'file_sha256': sources['render_source']['files_sha256'][
                               'maps/zone_wide_door_walls_v3_notags.json'],
                           'static_map_sha256': manifest['static_map_sha256'],
                           'landmarks_sha256': manifest['landmarks_sha256'], 'tags': 0},
            'label_classes': CLASSES, 'label_ignore': IGNORE, 'label_space': 'ideal pinhole robot_cam (K of '
            'sim.masterpi_camera_profile.scaled_camera_matrix(640, 480)), before the raw fisheye remap',
            'label_frames': len(state['rows']), 'class_geoms_sample': state.get('class_geoms'),
            'moved_files': moved, 'student_view': ['frames/', 'inputs/commands.jsonl', 'inputs/frames.jsonl',
                                                   'inputs/motion_profile.jsonl'],
            'smoke_sim_limit_s': args.smoke_sim_limit,
            'outcome': result['outcome'], 'diagnostic_success': result['diagnostic_success'],
            'm1_success': result['m1_success'], 'sim_s': result['sim_s'], 'frames': result['frames'],
            'wall_s': round(time.time() - started, 1),
            'load_average': {'start': [round(v, 2) for v in load0], 'end': [round(v, 2) for v in os.getloadavg()]},
            'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                       'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}}
        teacher_manifest['files_sha256'] = {str(q.relative_to(out)): sha_bytes(q.read_bytes())
                                            for q in sorted(out.rglob('*.jsonl'))}
        (out/'teacher_manifest.json').write_text(json.dumps(teacher_manifest, indent=2) + '\n')
        print(json.dumps({'episode': ep, 'outcome': result['outcome'], 'diag': result['diagnostic_success'],
                          'sim_s': result['sim_s'], 'frames': result['frames'], 'labels': len(state['rows']),
                          'wall_s': teacher_manifest['wall_s'], 'load': teacher_manifest['load_average']}), flush=True)


if __name__ == '__main__':
    main()
