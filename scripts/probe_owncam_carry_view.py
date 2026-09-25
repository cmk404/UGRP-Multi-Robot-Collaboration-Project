"""Carry/look posture probe for the wrist camera on zone_wide_door (DIAGNOSTIC).

Question: with the cyan zone box held (weld OFF, zone contact profile), which
ISSUED arm poses let the arm-tip fisheye ``robot_cam`` see (a) floor/obstacles
ahead, (b) walls and door posts at several heights, (c) zone paint, and which
of them keep the box in the gripper?

This is a TEACHER-assisted probe, not a skill result: ground truth places the
robot next to the box, drives the chassis between stops and scores whether the
box stays in the gripper. Only the rendered robot_cam images and the issued
PWM feed the view metrics. Test AprilTag panels (tag36h11) are VISUAL-ONLY
planes added to this probe scene to measure detectability; they are not part of
the map (the localisation branch owns real tags) and the file never changes
maps/zones/*.json.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.owncam_view import posture, tag_pixel_side, view_metrics  # noqa: E402

SCHEMA = 'ugrp.owncam_carry_view_probe.v1'
DOOR_Y = .05
WALL_FACE_X = 2.175          # west face of the divider (x = 2.20 - 0.025)
SEED = 11
# Pre-registered posture set: (name, grip radius m, grip height m, tool pitch deg).
POSTURES = (
    ('teacher_hover', None, None, None),       # the zone teacher's carry pose (hover above grasp)
    ('carry_p45', .155, .14, -45.),
    ('carry_p30', .14, .18, -30.),
    ('look_p20', .14, .18, -20.),
    ('look_p10', .14, .22, -10.),
)
PANS = (1500, 1230, 1770)    # straight, 24 deg right, 24 deg left (issued servo-6 PWM)
LENS_DISTANCES_M = (1.5, 1.0, .6, .3)
# Test tags: (lateral offset from the door centre line m, centre height m, size m).
TEST_TAGS = tuple((lat, h, .08) for lat in (-.30, .30) for h in (.05, .15, .25)) + \
    tuple((lat, h, .05) for lat in (-.60, .60) for h in (.05, .15, .25))


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_tag_textures(folder):
    import cv2
    folder.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    files = {}
    for tid in range(len(TEST_TAGS)):
        inner = cv2.aruco.generateImageMarker(dictionary, tid, 200)
        # one white cell of quiet zone on each side (tag36h11 = 8 black-bordered cells + 2 white)
        image = cv2.copyMakeBorder(inner, 25, 25, 25, 25, cv2.BORDER_CONSTANT, value=255)
        path = folder / f'tag36h11_{tid:02d}.png'
        cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
        files[tid] = path
    return files


def add_test_tags(xml, files):
    """Visual-only west-facing planes on the divider around door_1 (probe scene only)."""
    root = ET.fromstring(xml)
    asset = root.find('asset')
    if asset is None:
        asset = ET.SubElement(root, 'asset')
    world = root.find('worldbody')
    for tid, (lat, height, size) in enumerate(TEST_TAGS):
        ET.SubElement(asset, 'texture', name=f'probe_tag_{tid}', type='2d', file=str(files[tid]))
        ET.SubElement(asset, 'material', name=f'probe_tag_{tid}', texture=f'probe_tag_{tid}',
                      texrepeat='1 1', emission='.25', specular='0', shininess='0')
        # Plane normal is local +z; xyaxes (0,-1,0),(0,0,1) gives normal -x (west) with the
        # texture upright for a robot facing east. The quiet zone makes the panel 10/8 of the tag side.
        half = size * 10 / 8 / 2
        ET.SubElement(world, 'geom', name=f'probe_tag_{tid}', type='plane', material=f'probe_tag_{tid}',
                      pos=f'{WALL_FACE_X - .002} {DOOR_Y + lat} {height}', size=f'{half} {half} .01',
                      xyaxes='0 -1 0 0 0 1', contype='0', conaffinity='0', group='0')
    return ET.tostring(root, encoding='unicode')


def detect_tags(bgr):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    out = []
    for c, i in zip(corners, ids.reshape(-1) if ids is not None else []):
        c = c.reshape(4, 2)
        side = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))
        out.append({'id': int(i), 'side_px': round(side, 1), 'centre_px': c.mean(0).round(1).tolist()})
    return out


class Probe:
    def __init__(self, out):
        from sim.camera_robot_port import CameraRobotPort
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.zone_arena import episode
        from sim.zone_scene import ZoneScene
        from scripts.zone_teacher import ArmSequence, FOLDED
        self.out = out
        cfg = episode('zone_wide_door', SEED, goal={'B': {'cyan': 1}})
        cfg['contact_solver_profile'] = 'local_contact_fine'
        self.config = cfg
        self.definition = ZoneScene.from_zone_config(cfg)
        files = write_tag_textures(out / 'tag-textures')
        transform = self.definition.transform
        self.world = MultiMasterPiProductionV2(seed=SEED, width=640, height=480, render=True,
                                              warehouse_layout=self.definition.engine_layout,
                                              warehouse_cargo_ids=None,
                                              xml_transform=lambda xml: add_test_tags(transform(xml), files))
        self.definition.setup(self.world)
        self.port = CameraRobotPort(self.world, 'r1', allow_reverse=True, allow_mecanum=True)
        self.arm = ArmSequence(self.port, FOLDED)
        self.box = next(iter(cfg['setup_only']['objects'].values()))['body_name']
        self.max_offset_m = 0.
        self.reference = None

    # ---------- teacher/GT helpers (diagnostic only) ----------
    def gt_pose(self):
        r = self.world.robot('r1')
        xyz = r.base_xyz()
        return float(xyz[0]), float(xyz[1]), float(r.base_rpy()[2])

    def box_in_gripper(self):
        """Box centre in the r1 gripper body frame (GT, evaluation only)."""
        d = self.world.data
        g = d.body('r1__gripper')
        return np.asarray(g.xmat).reshape(3, 3).T @ (np.asarray(d.body(self.box).xpos) - np.asarray(g.xpos))

    def box_z(self):
        return float(self.world.data.body(self.box).xpos[2])

    def step(self, seconds):
        dt = float(self.world.model.opt.timestep)
        for _ in range(max(1, round(seconds / dt))):
            now = float(self.world.data.time)
            self.arm.tick(now)
            self.port.tick(now)
            self.world._physics_step_for(self.world.controllers['r1'])
        if self.reference is not None:
            self.max_offset_m = max(self.max_offset_m, float(np.linalg.norm(self.box_in_gripper() - self.reference)))

    def arm_to(self, targets, settle=.6):
        now = float(self.world.data.time)
        self.arm.queue(targets, now)
        while not self.arm.tick(float(self.world.data.time)):
            self.step(.05)
        self.step(settle)

    def place_robot_and_box(self, box_xy):
        import mujoco
        world = self.world
        jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, self.box + '_free')
        q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
        world.data.qpos[q:q + 7] = [box_xy[0], box_xy[1], .016, 1, 0, 0, 0]
        world.data.qvel[v:v + 6] = 0
        world.robot('r1').set_base_pose_for_test((box_xy[0] - .155, box_xy[1], .032355118817659255), 0.)
        mujoco.mj_forward(world.model, world.data)
        self.step(.5)

    def grasp(self):
        from scripts.zone_teacher import CLOSED, OPEN, GRASP_Z_M, HOVER_Z_M
        from harness.visual_arm import solve_grip_ik, tool_pose
        x, y, yaw = self.gt_pose()
        bx, by, _ = self.world.data.body(self.box).xpos
        dx, dy = bx - x, by - y
        fx, fy = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
        grasp = solve_grip_ik(fx, fy, GRASP_Z_M, -90)
        pitch = tool_pose(grasp).pitch_deg
        hover = solve_grip_ik(fx, fy, HOVER_Z_M, pitch)
        self.hover = hover
        self.arm_to({**hover, 1: OPEN}, .3)
        for z in np.linspace(HOVER_Z_M, GRASP_Z_M, 8)[1:]:
            self.arm_to({**solve_grip_ik(fx, fy, float(z), pitch), 1: OPEN}, 0.)
        self.arm_to({1: CLOSED}, .4)
        self.arm_to({**hover, 1: CLOSED}, .6)
        self.reference = self.box_in_gripper()
        return self.box_z()

    def drive_to(self, goal, heading=0., tol=.015, limit_s=60.):
        """GT P-controller (teacher) for chassis moves between probe stops."""
        start = float(self.world.data.time)
        while float(self.world.data.time) - start < limit_s:
            x, y, yaw = self.gt_pose()
            ex, ey = goal[0] - x, goal[1] - y
            eyaw = (heading - yaw + math.pi) % (2 * math.pi) - math.pi
            if math.hypot(ex, ey) < tol and abs(eyaw) < .02:
                break
            fwd = math.cos(yaw) * ex + math.sin(yaw) * ey
            left = -math.sin(yaw) * ex + math.cos(yaw) * ey
            self.port.apply({'kind': 'mecanum', 'forward': float(np.clip(1.2 * fwd, -.05, .12)),
                             'left': float(np.clip(1.2 * left, -.08, .08)),
                             'turn': float(np.clip(.8 * eyaw, -.12, .12)), 'duration_s': .15},
                            float(self.world.data.time))
            self.step(.1)
        self.port.hold(float(self.world.data.time))
        self.step(.4)
        return self.gt_pose()

    def render(self, name):
        import cv2
        jpeg = self.world.render_jpeg(robot_id='r1', camera='robot_cam', quality=90)
        path = self.out / 'frames' / f'{name}.jpg'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(jpeg)
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        return path, bgr


def held_box_mask(bgr):
    """Own-image occlusion by the held cyan box (hue band of the zone cyan paint)."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((80, 90, 30)), np.array((100, 255, 255))) > 0
    return mask


def pose_of(probe, name, radius, height, pitch, pan):
    if name == 'teacher_hover':
        return {**probe.hover, 1: 1500, 6: pan}
    return posture(radius, height, pitch, pan=pan, grip=1500)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-dirty', action='store_true', help='development only; records dirty=true')
    args = parser.parse_args()
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip())
    if dirty and not args.allow_dirty:
        raise SystemExit('commit and freeze the source before a recorded probe (or --allow-dirty)')
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    probe = Probe(out)
    log = {'schema': SCHEMA, 'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
           'dirty_source': dirty, 'variant': 'zone_wide_door', 'seed': SEED,
           'contact_solver_profile': 'local_contact_fine', 'weld': 'off',
           'scene_xml_sha256': hashlib.sha256(probe.world.scene_xml.encode()).hexdigest(),
           'claim_scope': ('diagnostic probe: GT teacher places, grasps and drives; view metrics come only from '
                           'robot_cam renders and issued PWM; test tags are visual-only probe panels'),
           'postures': [dict(zip(('name', 'radius_m', 'height_m', 'pitch_deg'), p)) for p in POSTURES],
           'test_tags': [{'id': i, 'lateral_from_door_m': t[0], 'centre_height_m': t[1], 'size_m': t[2],
                          'wall_face_x_m': WALL_FACE_X} for i, t in enumerate(TEST_TAGS)],
           'stages': []}
    # --- grasp west of the door, on the door centre line ---
    probe.place_robot_and_box((1.10, DOOR_Y))
    lift_z = probe.grasp()
    log['grasp'] = {'box_z_after_lift_m': round(lift_z, 4), 'held': lift_z > .045}
    load = __import__('os').getloadavg()
    log['load_average_start'] = [round(v, 2) for v in load]

    def hold_record(stage, extra):
        offset = float(np.linalg.norm(probe.box_in_gripper() - probe.reference))
        rec = {'stage': stage, 'sim_time_s': round(float(probe.world.data.time), 3), 'box_z_m': round(probe.box_z(), 4),
               'box_offset_in_gripper_m': round(offset, 4), 'held': probe.box_z() > .045 and offset < .02, **extra}
        log['stages'].append(rec)
        return rec

    # --- 1. postures at rest: stability + view metrics + renders ---
    for name, radius, height, pitch in POSTURES:
        for pan in PANS:
            pose = pose_of(probe, name, radius, height, pitch, pan)
            probe.arm_to(pose)
            path, bgr = probe.render(f'rest-{name}-pan{pan}')
            occ = held_box_mask(bgr)
            metrics = view_metrics(pose, occlusion=occ)
            hold_record('rest', {'posture': name, 'pan_pwm': pan, 'issued_pwm': {str(k): v for k, v in pose.items()},
                                 'frame': str(path.relative_to(out)), 'frame_sha256': _sha(path),
                                 'held_box_pixel_fraction': round(float(occ.mean()), 4), 'view': metrics})
    # --- 2. drive stability in each carry posture ---
    for name, radius, height, pitch in POSTURES:
        pose = pose_of(probe, name, radius, height, pitch, 1500)
        probe.arm_to(pose)
        probe.max_offset_m = 0.
        x, y, _ = probe.gt_pose()
        probe.drive_to((x - .40, y), 0.)                 # reverse 0.4 m (slow)
        probe.drive_to((x - .40, y), math.pi / 2)        # turn 90 deg left in place
        probe.drive_to((x - .40, y), 0.)                 # back
        probe.drive_to((x, y), 0.)                       # forward 0.4 m
        hold_record('drive', {'posture': name, 'max_box_offset_during_drive_m': round(probe.max_offset_m, 4)})
    # --- 3. door approach: tag detection vs lens distance, posture and pan ---
    from harness.visual_arm import camera_extrinsics
    for distance in LENS_DISTANCES_M:
        for name, radius, height, pitch in POSTURES[1:]:
            for pan in PANS:
                pose = pose_of(probe, name, radius, height, pitch, pan)
                lens_x = camera_extrinsics({**pose, 6: 1500})[0][0]
                target = (WALL_FACE_X - distance - lens_x, DOOR_Y)
                if abs(probe.gt_pose()[0] - target[0]) > .01:
                    probe.arm_to(pose_of(probe, 'carry_p30', .14, .18, -30., 1500), .2)
                    probe.drive_to(target, 0.)
                probe.arm_to(pose, .4)
                path, bgr = probe.render(f'door-d{distance:.1f}-{name}-pan{pan}')
                detections = detect_tags(bgr)
                x, y, yaw = probe.gt_pose()
                predicted = []
                for tid, (lat, h, size) in enumerate(TEST_TAGS):
                    # GT-relative geometry for the evaluation table only.
                    dx, dy = WALL_FACE_X - x, DOOR_Y + lat - y
                    fx, fy = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
                    origin = camera_extrinsics(pose)[0]
                    pred = tag_pixel_side(pose, distance_m=fx - origin[0], height_m=h, size_m=size, lateral_m=fy)
                    predicted.append({'id': tid, 'in_view': pred['in_view'], 'side_px': pred['side_px']})
                hold_record('door', {'lens_distance_m': distance, 'posture': name, 'pan_pwm': pan,
                                     'frame': str(path.relative_to(out)), 'frame_sha256': _sha(path),
                                     'gt_base_xy_eval_only': [round(x, 3), round(y, 3)],
                                     'detected': detections, 'predicted': predicted})
    # --- 4. zone C paint through the door (teacher drive), look postures ---
    probe.arm_to(pose_of(probe, 'carry_p30', .14, .18, -30., 1500), .2)
    probe.drive_to((1.95, DOOR_Y), 0.)
    probe.drive_to((2.45, DOOR_Y), 0.)
    probe.drive_to((2.30, -.45), 0.)
    for name, radius, height, pitch in POSTURES:
        pose = pose_of(probe, name, radius, height, pitch, 1500)
        probe.arm_to(pose, .4)
        path, bgr = probe.render(f'zoneC-{name}')
        import cv2
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        purple = cv2.inRange(hsv, np.array((125, 40, 20)), np.array((165, 255, 255))) > 0
        hold_record('zone_c', {'posture': name, 'frame': str(path.relative_to(out)), 'frame_sha256': _sha(path),
                               'zone_c_purple_pixel_fraction': round(float(purple.mean()), 4),
                               'gt_base_xy_eval_only': [round(v, 3) for v in probe.gt_pose()[:2]]})
    log['final'] = {'box_z_m': round(probe.box_z(), 4), 'still_held': probe.box_z() > .045}
    log['sim_seconds'] = round(float(probe.world.data.time), 3)
    log['wall_seconds'] = round(time.monotonic() - started, 1)
    log['load_average_end'] = [round(v, 2) for v in __import__('os').getloadavg()]
    (out / 'probe.json').write_text(json.dumps(log, indent=1) + '\n')
    print(json.dumps({'grasp': log['grasp'], 'final': log['final'], 'sim_s': log['sim_seconds']}))


if __name__ == '__main__':
    main()
