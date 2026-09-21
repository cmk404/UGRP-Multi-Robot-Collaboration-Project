#!/usr/bin/env python3
"""Prepare maps, splits and optional real camera previews; no policy execution."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.act_map_suite import SPEC, load_suite, manifest, geometry_check, scene_config, student_task, camera_coverage
from sim.research_dispatch_arena import digest, FIXED_TOP


def write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def file_sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def diagram(case, check, path):
    from PIL import Image, ImageDraw
    from scripts.build_pair_terrain_gallery import font
    data = case['map']
    image = Image.new('RGB', (640, 540), 'white')
    draw = ImageDraw.Draw(image)
    draw.text((18, 12), case['id'], fill='#18344b', font=font(23))
    draw.text((18, 43), case['topology_id'], fill='#62727c', font=font(16))
    xmin, xmax, ymin, ymax = data['bounds_m']
    scale = min(600/(xmax-xmin), 370/(ymax-ymin))
    def xy(point):
        return (20+(point[0]-xmin)*scale, 78+(ymax-point[1])*scale)
    draw.rectangle((*xy((xmin, ymax)), *xy((xmax, ymin))), fill='#eef3f7', outline='#6f8292')
    for box in data['obstacles']:
        x, y = box['center_m']; hx, hy = box['half_extents_m']
        draw.rectangle((*xy((x-hx, y+hy)), *xy((x+hx, y-hy))), fill='#384958')
    def footprint(pose, color):
        x, y, angle = pose; f = data['footprint']
        hx, hy = f['half_forward_m']+f['margin_m'], f['half_lateral_m']+f['margin_m']
        corners = [xy((x+math.cos(angle)*u-math.sin(angle)*v,
                       y+math.sin(angle)*u+math.cos(angle)*v)) for u, v in ((-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy))]
        draw.line(corners+[corners[0]], fill=color, width=2)
    route = check['route_m_rad']
    if route:
        draw.line([xy(p) for p in route], fill='#2583bf', width=3)
        for pose in route:
            footprint(pose, '#85b0c9')
    start = case['evaluation_only']['nominal_start_m_rad']
    goal = [*data['goal']['center_m'], math.radians(data['goal']['relative_yaw_deg'])]
    for pose, label, color in ((start, 'S', '#17659a'), (goal, 'G', '#168364')):
        footprint(pose, color)
        x, y = xy(pose)
        draw.ellipse((x-12,y-12,x+12,y+12), fill=color)
        draw.text((x-6,y-11), label, font=font(19), fill='white')
    status = {'geometric_candidate': '기하 경로 후보 · 실제 운반 미검증',
              'geometric_impossible_control': '기하 통과 불가능 대조 · 전체 하중 폭 기준',
              'no_lattice_path_unresolved': '격자 경로 없음 · 물리 불가능 판정 아님'}
    draw.text((18, 470), status[check['classification']], fill='#18344b', font=font(20))
    draw.text((18, 505), 'S = 평가용 명목 시작   G = 작성 목표   선 = 기하 검사', fill='#62727c', font=font(17))
    image.save(path)


def render_case(case, config, out):
    import mujoco
    import numpy as np
    from scripts.research_dispatch_scene import DispatchScene
    scene = DispatchScene(config, out)
    try:
        scene.open()
        frames = scene.capture('preview')
        model, state = scene.world.model, scene.world.data
        compiled = []
        for box in config['static_map']['obstacles']:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'dispatch_'+box['id'])
            if gid < 0 or not model.geom_contype[gid] or not model.geom_conaffinity[gid]:
                raise ValueError('missing collidable map obstacle '+box['id'])
            if not np.allclose(model.geom_pos[gid], [*box['center_m'], box['height_m']/2], atol=1e-10, rtol=0):
                raise ValueError('compiled map position mismatch')
            if not np.allclose(model.geom_size[gid], [*box['half_extents_m'], box['height_m']/2], atol=1e-10, rtol=0):
                raise ValueError('compiled map size mismatch')
            compiled.append(box['id'])
        goal = config['static_map']['docks']['local_goal']['slots']['beam']
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'dispatch_local_goal_beam')
        if (gid < 0 or model.geom_contype[gid] or model.geom_conaffinity[gid]
                or not np.allclose(model.geom_pos[gid][:2], goal['center_m'])
                or not np.allclose(model.geom_size[gid][:2], goal['half_extents_m'])):
            raise ValueError('goal paint mismatch')
        invariants = scene.invariants()
        camera = invariants['policy_cameras']['cctv_top']
        if (camera['position'] != FIXED_TOP['position_m'] or camera['quaternion'] != FIXED_TOP['quaternion_wxyz']
                or camera['fovy'] != FIXED_TOP['fov_y_deg'] or invariants['weld_active']):
            raise ValueError('camera or weld invariant failed')
        # Floor and obstacle tops must fit the unchanged TOP projection at 960x720.
        fov_checks = camera_coverage(config)
        fully_visible = all(x['inside_top_frustum'] for x in fov_checks)
        # Do not change historical assets or claim that their clipped wall tops
        # are fully visible. Only NEW instances must meet the expansion gate.
        if not fully_visible and case['split'] != 'regression':
            raise ValueError('geometry extends outside fixed TOP frustum')
        beam_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'team_beam_geom')
        moving = scene.robot_ids | {beam_gid}
        contacts = []
        for c in state.contact[:state.ncon]:
            pair = {int(c.geom1), int(c.geom2)}
            if c.dist < 0 and pair & scene.obstacle_ids and pair & moving:
                contacts.append({'geoms': [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in sorted(pair)],
                                 'distance_m': float(c.dist)})
        return {'scope': 'static folded-arm initialization and real cameras; no grasp/carry/teacher/policy trial',
                'source_pair_map_sha256': digest(case['map']),
                'dispatch_map_sha256': config['static_map_sha256'],
                'scene_xml_sha256': file_sha(out/'scene.xml'),
                'invariants': invariants, 'compiled_obstacles': compiled, 'fov_checks': fov_checks,
                'fixed_top_full_geometry_visible': fully_visible,
                'legacy_fov_limitation': not fully_visible and case['split'] == 'regression',
                'initial_obstacle_contacts': contacts,
                'preview_setup_clear': not contacts,
                'rgb': {r: {k: v for k, v in f.items() if not k.endswith('_bytes')} for r, f in frames.items()},
                'sim_seconds': float(state.time), 'transport_success': None}
    finally:
        scene.close()


def build(args):
    if args.output.exists():
        raise ValueError('output exists; never overwrite a previous run')
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip():
        raise ValueError('commit source and configuration before generating experiment evidence')
    source_sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    spec, cases = load_suite(args.spec)
    args.output.mkdir(parents=True)
    started = time.monotonic()
    record = manifest(spec, cases)
    record.update(source_sha=source_sha, render=args.render,
                  scope='map preparation and static visual QA; physical transport untested')
    write(args.output/'manifest.json', record)
    environment = {'python': sys.version, 'platform': platform.platform(),
                   'packages': {name: importlib.metadata.version(name) for name in ('numpy','Pillow','mujoco')}}
    write(args.output/'environment.json', environment)
    results = []
    for case in cases:
        tick = time.monotonic()
        out = args.output/case['id']; out.mkdir()
        write(out/'case-evaluation-only.json', {k: v for k, v in case.items() if k != 'map'})
        write(out/'map.json', case['map'])
        write(out/'student-task.json', student_task(case['map']))
        config = scene_config(case, physics_seed=spec['physics_initialization_seeds'][0])
        write(out/'scene-setup-only.json', config)
        check = geometry_check(case)
        write(out/'geometry-evaluation-only.json', check)
        diagram(case, check, out/'diagram.png')
        row = {'id': case['id'], 'split': case['split'], 'geometry': check, 'render': None, 'error': None}
        try:
            if args.render:
                row['render'] = render_case(case, config, out/'preview')
        except Exception as exc:
            row['error'] = f'{type(exc).__name__}: {exc}'
        row['wall_seconds'] = time.monotonic()-tick
        write(out/'result.json', row)
        results.append(row)
        print(json.dumps({'id': case['id'], 'geometry': check['classification'], 'error': row['error'],
                          'wall_seconds': round(row['wall_seconds'], 2)}), flush=True)
    from PIL import Image, ImageDraw
    from scripts.build_pair_terrain_gallery import font
    for kind in ('diagram', 'top', 'own') if args.render else ('diagram',):
        cell_w, cell_h = 400, 345
        gallery = Image.new('RGB', (cell_w*4, cell_h*math.ceil(len(cases)/4)), 'white')
        draw = ImageDraw.Draw(gallery)
        for i, case in enumerate(cases):
            folder = args.output/case['id']
            path = folder/'diagram.png' if kind == 'diagram' else folder/'preview/rgb'/('preview-top.jpg' if kind == 'top' else 'preview-r1.jpg')
            x, y = (i%4)*cell_w, (i//4)*cell_h
            draw.text((x+5, y+5), case['id'], font=font(17), fill='#18344b')
            if path.exists():
                with Image.open(path) as img:
                    img.thumbnail((390, 310))
                    gallery.paste(img, (x+5, y+30))
        gallery.save(args.output/f'{kind}-overview.jpg', quality=90)
    summary = {'source_sha': source_sha, 'manifest_sha256': file_sha(args.output/'manifest.json'),
               'cases': len(cases), 'geometric_candidates': sum(r['geometry']['route_found'] for r in results),
               'render_errors': sum(r['error'] is not None for r in results),
               'preview_setup_collisions': sum(bool(r['render'] and not r['render']['preview_setup_clear']) for r in results),
               'legacy_fov_limitations': sum(bool(r['render'] and r['render']['legacy_fov_limitation']) for r in results),
               'wall_seconds': time.monotonic()-started, 'policy_calls': 0, 'api_cost_usd': 0,
               'transport_attempts': 0, 'transport_success_rate': None,
               'storage': 'raw evidence local only; not remotely backed up'}
    write(args.output/'summary.json', summary)
    write(args.output/'artifact-hashes.json', {str(p.relative_to(args.output)): file_sha(p)
          for p in sorted(args.output.rglob('*')) if p.is_file()})
    print(json.dumps(summary), flush=True)
    return 1 if summary['render_errors'] or summary['preview_setup_collisions'] else 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--spec', type=Path, default=SPEC)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--render', action='store_true')
    return build(p.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
