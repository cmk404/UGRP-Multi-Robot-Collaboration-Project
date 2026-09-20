#!/usr/bin/env python3
"""Check registered terrain maps and optionally render unloaded setup scenes."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.pair_navigation import validate_map as validate_pair_map
from scripts.build_pair_terrain_gallery import CATALOG, font, sha, write
from sim.adaptive_warehouse import TerrainObservation, plan_local_path
from sim.authored_navigation_map import augment_map_xml, load_map, map_sha256


def check_entry(entry, source_entry, source_root):
    pair_path = CATALOG.parent / entry['map']
    solo_path = CATALOG.parent / entry['unloaded_map']
    source_path = source_root / source_entry['map']
    pair = validate_pair_map(json.loads(pair_path.read_text()))
    # Pair maps are byte-identical copies of the reviewed example geometry.
    assert pair_path.read_bytes() == source_path.read_bytes()
    solo = load_map(solo_path)
    assert solo['bounds_m'] == pair['bounds_m']
    assert solo['top_camera'] == pair['top_camera']
    assert solo['zones']['start'] == pair['start_zone']
    assert solo['zones']['goal']['center_m'] == pair['goal']['center_m']
    assert solo['zones']['goal']['radius_m'] == .18
    assert solo['footprint'] == {'unloaded_radius_m': .18, 'safety_margin_m': .04}
    assert solo['grid_resolution_m'] == .025
    assert len(solo['obstacles']) == len(pair['obstacles'])
    xml = ET.fromstring(augment_map_xml('<mujoco><worldbody/></mujoco>', solo))
    for a, b in zip(pair['obstacles'], solo['obstacles']):
        assert all(a[key] == b[key] for key in a)
        geom = xml.find(f".//geom[@name='known_map_{b['id']}']")
        assert geom is not None
        assert list(map(float, geom.get('pos').split())) == [*b['center_m'], b['height_m']/2]
        assert list(map(float, geom.get('size').split())) == [*b['half_extents_m'], b['height_m']/2]
    terrain = tuple(TerrainObservation(b['id'], b['kind'], tuple(b['center_m']),
        tuple(b['half_extents_m']), b['height_m'], b['traversable'], b['cost_multiplier'])
        for b in solo['obstacles'])
    try:
        route = plan_local_path(solo['zones']['start']['center_m'], solo['zones']['goal']['center_m'],
            terrain, footprint_xy=(.22, .22), resolution_m=solo['grid_resolution_m'],
            bounds=tuple(solo['bounds_m']))
    except ValueError as error:
        if 'no collision-free' not in str(error):
            raise
        route = None
    assert (route is not None) == entry['expected_geometric_route']
    return solo, {'id': entry['id'], 'title_ko': entry['title_ko'],
        'pair_file': str(pair_path.relative_to(ROOT)), 'pair_sha256': sha(pair_path),
        'source_file': str(source_path.resolve().relative_to(ROOT)),
        'unloaded_file': str(solo_path.resolve().relative_to(ROOT)), 'unloaded_sha256': sha(solo_path),
        'unloaded_map_sha256': map_sha256(solo), 'source_and_registered_geometry_equal': True,
        'unloaded_route_m': route, 'unloaded_route_found': route is not None}


def render_unloaded(data, out):
    import mujoco
    import numpy as np
    from scripts.known_map_scene import KnownMapScene
    from scripts.run_known_map_navigation import validate_case

    case = validate_case({'case_id': data['map_id'] + '_preview', 'robot_id': 'r1',
        'start_xy_m': data['zones']['start']['center_m'], 'start_yaw_deg': 0}, data)
    out.mkdir(); (out/'rgb').mkdir()
    scene = KnownMapScene(out, data, 'r1', case)
    try:
        scene.open()
        before = scene.invariants()
        scene.capture(0)
        (out/'observer.jpg').write_bytes(scene.world.render_team_jpeg(camera='cctv_warehouse', quality=95))
        m, d = scene.world.model, scene.world.data
        mujoco.mj_saveLastXML(str(out/'scene.xml'), m)
        for wall in data['obstacles']:
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'known_map_' + wall['id'])
            assert gid >= 0
            assert np.allclose(m.geom_pos[gid], [*wall['center_m'], wall['height_m']/2], atol=1e-10)
            assert np.allclose(m.geom_size[gid], [*wall['half_extents_m'], wall['height_m']/2], atol=1e-10)
        assert scene.collision_steps == 0 and not np.any(d.eq_active)
        assert before == scene.invariants() and not before['added_navigation_camera']
        return {'invariants': before, 'setup_sim_seconds': float(d.time),
            'initial_map_collisions': 0, 'weld_off': True, 'compiled_walls_match': True}
    finally:
        scene.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    if args.render and subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip():
        raise RuntimeError('commit maps and verification source before rendering')
    out = args.out_dir.resolve(); out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    catalog = json.loads(CATALOG.read_text())
    source_path = (CATALOG.parent/catalog['source_catalog']).resolve()
    source = json.loads(source_path.read_text())
    assert [e['id'] for e in catalog['entries']] == [e['id'] for e in source['entries']]
    record = {'schema': 'ugrp.terrain_map_registration.v1', 'catalog_sha256': sha(CATALOG),
        'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'environment': {'python': platform.python_version(), 'platform': platform.platform()},
        'scope': 'Registered map parity and static setup only; no physical passage or actor decisions',
        'rendered': args.render, 'navigation_actions': 0, 'model_calls': 0, 'model_cost_usd': 0, 'entries': []}
    for entry, original in zip(catalog['entries'], source['entries']):
        data, check = check_entry(entry, original, source_path.parent)
        if args.render:
            check['scene'] = render_unloaded(data, out/entry['id'])
        record['entries'].append(check)
        print(json.dumps({'id': entry['id'], 'route_found': check['unloaded_route_found'],
                          'rendered': args.render}), flush=True)
    if args.render:
        import mujoco
        from PIL import Image, ImageDraw
        record['environment']['mujoco'] = mujoco.__version__
        scenes = [e['scene']['invariants'] for e in record['entries']]
        for field in ('actor_cameras', 'robot_geometry_sha256', 'robot_xml_sha256'):
            assert all(s[field] == scenes[0][field] for s in scenes)
        canvas = Image.new('RGB', (1920, 1130), '#e6edf4'); draw = ImageDraw.Draw(canvas)
        draw.text((28,18), 'maps/navigation에서 불러온 예시 지형', font=font(36), fill='#182b42')
        draw.text((28,69), '단독 주행용 장면의 초기 정지 화면 · 실제 통과 시험은 아직 수행하지 않음', font=font(22), fill='#435e75')
        for i, e in enumerate(record['entries']):
            x,y = 20+(i%3)*640, 120+(i//3)*505
            draw.text((x,y), e['title_ko'], font=font(23), fill='#182b42')
            with Image.open(out/e['id']/'rgb/0000-top.jpg') as pic:
                canvas.paste(pic.resize((620,465)), (x,y+33))
        canvas.save(out/'navigation-overview.jpg', quality=95)
    record['elapsed_wall_seconds'] = time.monotonic() - started
    record['files'] = [{'path': str(p.relative_to(out)), 'bytes': p.stat().st_size, 'sha256': sha(p)}
                       for p in sorted(out.rglob('*')) if p.is_file()]
    write(out/'registration-record.json', record)


if __name__ == '__main__':
    main()
