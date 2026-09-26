"""ST6 (amendment design, POST HOC to the v3 loop test): where on the walls could a tag be read at all?

Diagnostic only (simulator state, mj_forward, no physics step, no robot input, no seed). It
answers a map-level question raised by the failure analysis (loop/failure_analysis.json):
from the stations where the v3 test episodes lost their fix, is there ANY wall position whose
tag the wrist camera detects? If not, adding tags cannot address that failure.

Candidate map: the v3 door map (0.40 m walls) with its tags replaced by a dense grid of
candidate tags (same 0.072 m tag36h11 / 0.09 m plate) on every wall face except the west
perimeter wall, every 0.15 m along the face, at z = 0.05 / 0.15 / 0.25 m (the v3 column).

Stations (robot facing east, the approach convention):
  strip_loaded   base x 1.85/1.95, y -1.2..-0.4 (the path along the divider west face that the
                 loaded planner uses to reach door_1 from the south; s726/s727/s728 lost the fix here)
  door_unloaded  base x 1.95..2.55, y -0.03/0.05/0.13 (door_1 passage and exit; s711/s721 R3,
                 s722/s723 no_tag looks)
  door_loaded    base x 1.95..2.55, y 0.05
Views: the drive posture (SEARCH unloaded, CARRY_POSTURE loaded) and LOOK_P20 at every pan of
WIDE_LOOK_PANS, the arm states and held-box pose of the committed posture library
(outputs/zone-env-v3-20260926/static/posture_library.json, static checks f843d392).

  python static_candidates.py --library <posture_library.json> --output <raw dir> --record st6_candidates.json
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path

X = Path(__file__).resolve().parent
ROOT = X.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SPACING_M = .15
HEIGHTS_M = (.05, .15, .25)
EXCLUDED_WALLS = ('wall_west',)
GROUPS = {
    'strip_loaded': {'load': 'loaded', 'bases': [(x, y) for x in (1.85, 1.95) for y in (-1.2, -1.0, -.8, -.6, -.4)]},
    'door_unloaded': {'load': 'unloaded', 'bases': [(x, y) for x in (1.95, 2.10, 2.25, 2.40, 2.55)
                                                    for y in (-.03, .05, .13)]},
    'door_loaded': {'load': 'loaded', 'bases': [(x, .05) for x in (1.95, 2.10, 2.25, 2.40, 2.55)]},
}


def candidate_tags(static, heights=HEIGHTS_M):
    from sim.zone_tag_rule_v3 import faces
    tags = []
    for face in faces(static):
        if face['wall'] in EXCLUDED_WALLS:
            continue
        for lo, hi in face['free']:
            s = lo + .075
            while s <= hi - .075 + 1e-9:
                x, y = (s, face['coord']) if face['axis'] == 'x' else (face['coord'], s)
                nx, ny = face['normal']
                for z in heights:
                    tags.append({'id': len(tags), 'wall': face['wall'], 'normal_xy': [nx, ny],
                                 'center_m': [round(x, 4), round(y, 4), z], 'yaw_rad': round(math.atan2(ny, nx), 6),
                                 'size_m': .072})
                s += SPACING_M
    if len(tags) > 587:
        raise ValueError(f'{len(tags)} candidates exceed the tag36h11 dictionary')
    return tags


def make_candidate_world(sc, z):
    """One batch per tag height: the renderer holds at most 10000 visual geoms (mujoco.Renderer default)."""
    import mujoco
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_landmarks import TaggedZoneScene
    scene = TaggedZoneScene.from_tagged(sc.V3, sc.SEED, sc.GOAL, sc.EXTRA, contact_profile=sc.PROFILE)
    static = copy.deepcopy(scene.config['static_map'])
    static['landmarks']['tags'] = candidate_tags(static, (z,))
    static['landmarks'].pop('sites', None)
    scene.config['static_map'] = static           # diagnostic scene only (never a robot map)
    world = MultiMasterPiProductionV2(seed=sc.SEED, width=640, height=480, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    if world.model.ngeom > 9000:
        world.close()
        raise RuntimeError(f'{world.model.ngeom} geoms: the render buffer (10000) could drop tags')
    return scene, world, static


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--record', type=Path, default=None)
    a = p.parse_args(argv)
    from importlib import util
    spec = util.spec_from_file_location('static_checks', X/'static_checks.py')
    sc = util.module_from_spec(spec)
    spec.loader.exec_module(sc)
    from harness.owncam_drive import WIDE_LOOK_PANS
    from harness.wall_tags import TagDetector
    lib = json.loads(a.library.read_text())
    started, load_start = time.time(), os.getloadavg()
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output/'frames').mkdir(exist_ok=True)
    v3 = json.loads((ROOT/'maps'/'zones'/f'{sc.V3}.json').read_text())
    v3_sites = [(s['wall'], tuple(s['normal_xy']), s['center_m']) for s in v3['landmarks']['sites']]
    pans = tuple(dict.fromkeys(WIDE_LOOK_PANS))
    rows, tags, batches = [], {}, []
    for z in HEIGHTS_M:
        scene, world, static = make_candidate_world(sc, z)
        batch = {t['id']: t for t in static['landmarks']['tags']}
        batches.append({'z_m': z, 'tags': len(batch), 'ngeom': int(world.model.ngeom),
                        'sha256': hashlib.sha256(json.dumps(static['landmarks']['tags']).encode()).hexdigest()})
        detector = TagDetector.for_map(static)
        try:
            rid, box_body = lib['robot_id'], lib['box_body']
            sc.clear_scene(world, scene, rid, keep_box=box_body)
            for group, spec_ in GROUPS.items():
                load = spec_['load']
                drive = ('SEARCH_unloaded' if load == 'unloaded' else 'CARRY_loaded')
                look = 'LOOK_P20_' + load
                for bx, by in spec_['bases']:
                    for view, key, pan in [('drive', drive, None)] + [(f'look_{q}', look, q) for q in pans]:
                        state = lib[key]
                        box = (box_body, state['box_rel']) if load == 'loaded' else None
                        if box is None:
                            sc.set_free(world, box_body, (60., 60., .016), (1., 0., 0., 0.))
                        sc.place(world, rid, (bx, by), 0., state, pan=pan, box=box)
                        jpeg, seen = sc.wrist_jpeg(world, rid)
                        origin, _ = sc.camera_pose(world, rid)
                        name = f'z{z:.2f}_{group}_x{bx:.2f}_y{by:+.2f}_{view}'
                        if view in ('drive', 'look_1500', 'look_970', 'look_2030') and by in (-.8, .05):
                            (a.output/'frames'/f'{name}.jpg').write_bytes(jpeg)
                        dets = sc.detect_frame(detector, seen, batch)
                        for d in dets:
                            t = batch[d['id']]
                            d['true_range_m'] = round(float(math.dist(origin, t['center_m'])), 4)
                            d['key'] = f"{t['wall']}|{t['normal_xy'][0]},{t['normal_xy'][1]}|{t['center_m'][0]:.3f},{t['center_m'][1]:.3f},{z:.2f}"
                            tags[d['key']] = t
                        rows.append({'z_m': z, 'group': group, 'base': [bx, by], 'view': view,
                                     'camera_z_m': round(float(origin[2]), 4), 'detections': dets,
                                     'frame_sha256': hashlib.sha256(jpeg).hexdigest()})
        finally:
            world.close()

    def near_v3_site(t):
        return next((f'{w}{n}@{c}' for w, n, c in v3_sites if w == t['wall'] and n == tuple(t['normal_xy'])
                     and math.dist(c, t['center_m'][:2]) <= .10), None)

    summary = {}
    for group, spec_ in GROUPS.items():
        g = [r for r in rows if r['group'] == group]
        stations = {tuple(r['base']) for r in g}
        per_pos, per_station = {}, {}
        for r in g:
            for d in r['detections']:
                t = tags[d['key']]
                key = f"{t['wall']} n{tuple(t['normal_xy'])} xy({t['center_m'][0]:.3f},{t['center_m'][1]:.3f})"
                e = per_pos.setdefault(key, {'wall': t['wall'], 'normal_xy': t['normal_xy'], 'xy': t['center_m'][:2],
                                             'near_v3_site': near_v3_site(t), 'stations': set(), 'z': set(),
                                             'max_abs_range_err_m': 0.})
                e['stations'].add(tuple(r['base']))
                e['z'].add(t['center_m'][2])
                e['max_abs_range_err_m'] = max(e['max_abs_range_err_m'], abs(d['range_m'] - d['true_range_m']))
                e['min_true_range_m'] = min(e.get('min_true_range_m', 99.), d['true_range_m'])
                per_station.setdefault(tuple(r['base']), set()).add(key)
        summary[group] = {
            'stations': len(stations), 'stations_with_any_candidate': len(per_station),
            'stations_with_a_v3_site_position': sum(any(per_pos[k]['near_v3_site'] for k in v) for v in per_station.values()),
            'stations_without_any_candidate': sorted([list(s) for s in stations - set(per_station)]),
            'positions': sorted(({**{k: v for k, v in e.items() if k not in ('stations', 'z')}, 'position': key,
                                  'stations_seen': len(e['stations']), 'z_seen': sorted(e['z']),
                                  'max_abs_range_err_m': round(e['max_abs_range_err_m'], 4)}
                                 for key, e in per_pos.items()), key=lambda e: -e['stations_seen'])}
    import cv2
    import mujoco
    import numpy as np
    raw = {'schema': 'ugrp.zone_env_v3.st6_candidates.v1', 'posthoc': True,
           'code': {'sha': sc.git('rev-parse', 'HEAD'),
                    'dirty': bool(sc.git('status', '--porcelain', '--', 'sim', 'harness', 'maps',
                                         'experiments/2026-09-26-zone-env-v3'))},
           'library_sha256': hashlib.sha256(a.library.read_bytes()).hexdigest(),
           'candidates': {'batches': batches, 'spacing_m': SPACING_M, 'heights_m': list(HEIGHTS_M),
                          'excluded_walls': list(EXCLUDED_WALLS)},
           'groups': GROUPS, 'look_pans': list(pans), 'summary': summary, 'views': rows,
           'env': {'mujoco': mujoco.__version__, 'opencv': cv2.__version__, 'numpy': np.__version__,
                   'threads': {k: os.environ.get(k) for k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                                              'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
           'load_average': {'start': [round(v, 2) for v in load_start], 'end': [round(v, 2) for v in os.getloadavg()]},
           'wall_s': round(time.time() - started, 1)}
    (a.output/'st6_candidates.json').write_text(json.dumps(raw, indent=1) + '\n')
    if a.record:
        rec = {k: raw[k] for k in ('schema', 'posthoc', 'code', 'library_sha256', 'candidates', 'groups', 'look_pans',
                                    'summary', 'env', 'load_average', 'wall_s')}
        rec['raw'] = str(a.output/'st6_candidates.json')
        rec['raw_sha256'] = hashlib.sha256((a.output/'st6_candidates.json').read_bytes()).hexdigest()
        a.record.write_text(json.dumps(rec, indent=1) + '\n')
    for group, s in summary.items():
        print(group, {k: v for k, v in s.items() if k != 'positions'})
        for e in s['positions'][:12]:
            print('   ', e['stations_seen'], e['position'], 'z', e['z_seen'], 'range', e['min_true_range_m'], 'err', e['max_abs_range_err_m'], e['near_v3_site'])


if __name__ == '__main__':
    main()
