"""Observer-only analysis of the hard-route fixture runs (never robot input).

- per-run metrics from result.json and teacher-events.json;
- robot-robot, robot-wall and carried-box-wall contacts recomputed from the
  recorded replay states (mj_forward per sampled frame);
- mp4 renders of chosen replays, top-down PNGs of each map (rendered and
  schematic) and one robot-camera frame at the door.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

ROBOTS = ('r1', 'r2', 'r3')
CONTACT_HZ = 10


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _owner(name):
    for rid in ROBOTS:
        if name.startswith(rid + '__'):
            return rid
    if name.startswith('zone_wall_'):
        return 'wall'
    if name.startswith('cargo_box_'):
        return 'box'
    return None


def contacts(run_dir):
    """Episodes and seconds of penetrating contact per category, at CONTACT_HZ."""
    import mujoco
    from scripts.dispatch_replay import load
    model, states, _, _ = load(run_dir)
    data = mujoco.MjData(model)
    owner = [_owner(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or '') for i in range(model.ngeom)]
    times = states['time']
    step = max(1, round((len(times)/max(times[-1]-times[0], 1e-9))/CONTACT_HZ))
    cats = {'robot_robot': [], 'robot_wall': [], 'box_wall': []}
    for index in range(0, len(times), step):
        data.qpos[:] = states['qpos'][index]
        mujoco.mj_forward(model, data)
        hit = set()
        for c in data.contact[:data.ncon]:
            if c.dist >= 0:
                continue
            a, b = sorted((owner[c.geom1] or '', owner[c.geom2] or ''))
            if a in ROBOTS and b in ROBOTS and a != b:
                hit.add('robot_robot')
            elif (a in ROBOTS and b == 'wall') or (b in ROBOTS and a == 'wall'):
                hit.add('robot_wall')
            elif {a, b} == {'box', 'wall'}:
                hit.add('box_wall')
        for cat in cats:
            cats[cat].append(cat in hit)
    dt = step*(times[-1]-times[0])/max(len(times)-1, 1)
    out = {}
    for cat, flags in cats.items():
        episodes = sum(1 for i, f in enumerate(flags) if f and (i == 0 or not flags[i-1]))
        out[cat] = {'episodes': episodes, 'seconds': round(sum(flags)*dt, 2)}
    return out


def drive_waits(run_dir, static):
    """SIM seconds robots stood still (< 5 mm per 0.1 s) in a drive phase
    (to_box, carry), overall and within 1.0 m of a single-lane passage."""
    import mujoco
    from harness.static_keepouts import passage_zones, rect_distance
    from scripts.dispatch_replay import label_at, load
    model, states, labels, _ = load(run_dir)
    data = mujoco.MjData(model)
    cores = [core for _, core, _ in passage_zones(static)]
    times = states['time']
    step = max(1, round((len(times)/max(times[-1]-times[0], 1e-9))/CONTACT_HZ))
    prev, total, near = {}, 0., 0.
    for index in range(0, len(times), step):
        data.qpos[:] = states['qpos'][index]
        mujoco.mj_forward(model, data)
        phases = dict(part.split(':') for part in label_at(labels, index).split(' | ') if ':' in part)
        for rid in ROBOTS:
            xy = data.body(rid + '__robot').xpos[:2].copy()
            if rid in prev and phases.get(rid) in ('to_box', 'carry') and math.dist(xy, prev[rid][0]) < .005:
                dt = float(times[index]-prev[rid][1])
                total += dt
                if any(rect_distance(xy, c) <= 1. for c in cores):
                    near += dt
            prev[rid] = (xy, times[index])
    return round(total, 1), round(near, 1)


def metrics(run_dir, row):
    result = json.loads((run_dir/'result.json').read_text())
    events = json.loads((run_dir/'teacher-events.json').read_text())
    outcomes = Counter(e.get('outcome') for e in events if e['event'] == 'job_end')
    waits = {}
    for e in events:
        if e['event'] == 'passage_wait':
            waits[e['passage']] = round(waits.get(e['passage'], 0.) + e['wait_s'], 2)
    blocked = [e for e in events if e['event'] == 'phase' and e.get('outcome') == 'teacher_path_blocked']
    stats = result['coordination_stats']
    return {**{k: row[k] for k in ('run', 'variant', 'goal', 'seed', 'coordination', 'wall_s',
                                   'load_1min_start', 'load_1min_end')},
            'phase': result['phase'], 'error': result['error'],
            'goal_met_referee': result['physical_success_teacher_condition'], 'goal_met_rgb': result['goal_met_rgb'],
            'makespan_sim_s': result.get('makespan_sim_s'), 'control_end_sim_s': result['sim_end_s'],
            'llm_calls': result['llm_calls'], 'jobs_placed': outcomes.get('placed_by_teacher', 0),
            'teacher_path_blocked': outcomes.get('teacher_path_blocked', 0),
            'goal_occupied': sum(1 for e in blocked if e.get('goal_occupied')),
            'grasp_failed': outcomes.get('grasp_failed_by_teacher', 0),
            'box_taken_by_peer': outcomes.get('box_taken_by_peer', 0),
            'dropped_in_transit': outcomes.get('dropped_in_transit', 0),
            'generic_yields': sum(1 for e in events if e['event'] == 'yield'),
            'passage_standoffs': sum(1 for e in events if e['event'] == 'passage_standoff'),
            'passage_fallbacks': sum(1 for e in events if e['event'] == 'passage_standoff' and e.get('fallback')),
            'passage_no_spot': sum(1 for e in events if e['event'] == 'passage_yield_end' and e['reason'] == 'no_spot'),
            'door_wait_s': round(sum(waits.values()), 2), 'door_wait_by_passage_s': waits,
            'claim_collisions': stats.get('collisions'), 'invalid_claims': stats.get('invalid_claims'),
            'sha256': {name: sha(run_dir/name) for name in ('result.json', 'teacher-events.json', 'scene.xml')}
            | {'replay_manifest': sha(run_dir/'replay'/'replay.json')}}


def render_mp4(run_dir, target, *, speed=4., width=960, height=540, fps=30):
    import cv2
    import mujoco
    from scripts.dispatch_replay import frame_at, label_at, load
    model, states, labels, _ = load(run_dir)
    view = json.loads((Path(run_dir)/'replay'/'view.json').read_text())
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height, width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = view['lookat']
    cam.distance, cam.azimuth, cam.elevation = view['distance'], view['azimuth'], -70
    times = states['time'].tolist()
    proc = subprocess.Popen(['ffmpeg', '-y', '-v', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{width}x{height}',
                             '-r', str(fps), '-i', '-', '-pix_fmt', 'yuv420p', '-vcodec', 'libx264', '-crf', '26', str(target)],
                            stdin=subprocess.PIPE)
    t = times[0]
    while t <= times[-1]:
        index = frame_at(times, t)
        data.qpos[:] = states['qpos'][index]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, cam)
        frame = renderer.render().copy()
        text = f'SIM {times[index]:6.1f}s  x{speed:g}  {label_at(labels, index)}'
        cv2.putText(frame, text, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2, cv2.LINE_AA)
        proc.stdin.write(frame.tobytes())
        t += speed/fps
    proc.stdin.close()
    if proc.wait():
        raise RuntimeError('ffmpeg failed')
    renderer.close()


def render_top_down(run_dir, target):
    import cv2
    import mujoco
    from scripts.dispatch_replay import load
    model, states, _, _ = load(run_dir)
    data = mujoco.MjData(model)
    data.qpos[:] = states['qpos'][0]
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 720, 960)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [2.175, -.85, 0.]
    cam.distance, cam.azimuth, cam.elevation = 6.3, 90, -90
    renderer.update_scene(data, cam)
    cv2.imwrite(str(target), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
    renderer.close()


def schematic(static, target, scale=150):
    """Annotated plan view: walls, passages with widths, zones, pickup, TOP footprints."""
    import cv2
    import numpy as np
    x0, x1, y0, y1 = static['bounds_m']
    pad = 40
    w, h = int((x1-x0)*scale)+2*pad, int((y1-y0)*scale)+2*pad+30
    img = np.full((h, w, 3), 255, np.uint8)
    def px(x, y):
        return int(pad+(x-x0)*scale), int(pad+30+(y1-y)*scale)
    def rect(c, he, color, thick=-1):
        cv2.rectangle(img, px(c[0]-he[0], c[1]+he[1]), px(c[0]+he[0], c[1]-he[1]), color, thick)
    for cam in static['top_cameras']:
        cx, cy, cz = cam['position_m']
        hy = cz*math.tan(math.radians(cam['fov_y_deg'])/2)
        rect((cx, cy), (hy*960/720, hy), (225, 225, 225), 1)
    colors = {'pickup': (200, 140, 60), 'zone_A': (40, 140, 245), 'zone_B': (230, 110, 50), 'zone_C': (200, 60, 170)}
    for rid, region in static['regions'].items():
        overlay = img.copy()
        rect(region['center_m'], region['half_extents_m'], colors[rid])
        img[:] = cv2.addWeighted(overlay, .25, img, .75, 0)
        cv2.putText(img, rid, px(region['center_m'][0]-.25, region['center_m'][1]), cv2.FONT_HERSHEY_SIMPLEX, .5,
                    (60, 60, 60), 1, cv2.LINE_AA)
    for passage in static.get('passages', []):
        overlay = img.copy()
        rect(passage['center_m'], passage['half_extents_m'], (80, 200, 80) if passage.get('lanes') != 1 else (60, 180, 250))
        img[:] = cv2.addWeighted(overlay, .45, img, .55, 0)
        label = passage['id'] + (f" {passage['width_m']:.2f} m" if 'width_m' in passage else '')
        cx, cy = passage['center_m']
        cv2.putText(img, label, px(cx+.06, cy+.12 if passage['axis'] == 'x' else cy), cv2.FONT_HERSHEY_SIMPLEX,
                    .45, (0, 0, 0), 1, cv2.LINE_AA)
    for wall in static['obstacles']:
        rect(wall['center_m'], wall['half_extents_m'], (70, 60, 50))
    cv2.putText(img, f"{static['map_id']} v{static['version']}  (x east, y north; grid 0.5 m)", (pad, 22),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 0), 1, cv2.LINE_AA)
    for gx in np.arange(math.ceil(x0*2)/2, x1, .5):
        cv2.line(img, px(gx, y0), px(gx, y0-.04), (0, 0, 0), 1)
    cv2.imwrite(str(target), img)


def robot_view_at_door(run_dir, target, door_xy):
    """Render the own camera of the robot nearest the door at the frame it is closest."""
    import cv2
    import mujoco
    import numpy as np
    from scripts.dispatch_replay import load
    model, states, _, _ = load(run_dir)
    data = mujoco.MjData(model)
    best = None
    for index in range(0, len(states['time']), 3):
        data.qpos[:] = states['qpos'][index]
        mujoco.mj_forward(model, data)
        for rid in ROBOTS:
            cam = model.camera(rid + '__robot_cam').id
            x, y = (float(v) for v in data.cam_xpos[cam][:2])
            heading = float(-data.cam_xmat[cam].reshape(3, 3)[:, 2][0])
            # West of the door, on its axis, facing east (towards the opening).
            if not (door_xy[0]-.8 <= x <= door_xy[0]-.2 and abs(y-door_xy[1]) <= .25 and heading > .9):
                continue
            d = abs(x-(door_xy[0]-.45)) + abs(y-door_xy[1])
            if best is None or d < best[0]:
                best = (d, index, rid)
    _, index, rid = best
    data.qpos[:] = states['qpos'][index]
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 480, 640)
    renderer.update_scene(data, rid + '__robot_cam')
    cv2.imwrite(str(target), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
    renderer.close()
    cam = model.camera(rid + '__robot_cam').id
    return {'robot': rid, 'sim_time_s': round(float(states['time'][index]), 2),
            'camera_xy_m': [round(float(v), 3) for v in data.cam_xpos[cam][:2]]}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True, help='matrix output folder')
    p.add_argument('--record', type=Path, required=True, help='experiment folder for PNG/JSON')
    p.add_argument('--media', type=Path, required=True, help='folder for mp4 renders (local)')
    p.add_argument('--videos', nargs='*', default=[])
    args = p.parse_args()
    from sim import zone_arena as za
    rows = []
    for path in sorted((args.output/'runs').glob('*.json')):
        row = json.loads(path.read_text())
        run_dir = args.output/row['run']
        if not (run_dir/'result.json').is_file():
            rows.append({**row, 'error': 'no result.json'})
            continue
        m = metrics(run_dir, row)
        m['contacts'] = contacts(run_dir)
        m['drive_wait_s'], m['drive_wait_near_passage_s'] = drive_waits(run_dir, za.authored_map(row['variant']))
        rows.append(m)
        print(json.dumps({k: m[k] for k in ('run', 'goal_met_referee', 'teacher_path_blocked', 'passage_standoffs',
                                           'door_wait_s', 'makespan_sim_s')} | {'contacts': m['contacts']}), flush=True)
    (args.output/'analysis.json').write_text(json.dumps(rows, indent=2) + '\n')
    args.media.mkdir(parents=True, exist_ok=True)
    maps = {}
    for variant in ('zone_wide_door', 'zone_wide_two_doors', 'zone_wide_corridor'):
        static = za.authored_map(variant)
        png = args.record/f'map-{variant}.png'
        schematic(static, png)
        sample = next((args.output/r['run'] for r in rows if r.get('variant') == variant
                       and (args.output/r['run']/'replay').is_dir()), None)
        render = args.record/f'render-{variant}.png'
        if sample:
            render_top_down(sample, render)
        maps[variant] = {'schematic_png': png.name, 'render_png': render.name if sample else None}
    door = za.authored_map('zone_wide_door')['passages'][0]['center_m']
    door_run = next(args.output/r['run'] for r in rows if r.get('variant') == 'zone_wide_door')
    maps['robot_view_at_door'] = robot_view_at_door(door_run, args.record/'robot-rgb-at-door.png', door)
    maps['robot_view_at_door']['png'] = 'robot-rgb-at-door.png'
    videos = {}
    for run in args.videos:
        target = args.media/f'{run}.mp4'
        render_mp4(args.output/run, target)
        videos[run] = {'path': str(target), 'sha256': sha(target), 'bytes': target.stat().st_size}
    (args.output/'media.json').write_text(json.dumps({'maps': maps, 'videos': videos}, indent=2) + '\n')


if __name__ == '__main__':
    main()
