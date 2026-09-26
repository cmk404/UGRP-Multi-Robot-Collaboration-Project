"""Evaluation-only TOP camera for the corridor map (#218). DIAGNOSTIC / EVALUATION TOOL.

Subcommands (raw outputs go to the PRIMARY checkout's outputs/, never a worktree):

  search    grid search of the moved ``cctv_top_north_east`` (prereg.json change_space,
            gate G1-G4, selection rule) with the env v3 ST5 helpers
  select    prereg rule and amendment A1 (G1b) applied to the saved search (no new measurement)
  coverage  unmodified env v3 ``st5()`` under a profile (make_world wrapped only to
            apply ``sim.zone_eval_top`` after scene.setup), all maps x both wall families
  stills    corridor v3 TOP mosaics under v1 and v2 with boxes and a robot placed in
            the bands v1 cannot see, plus the existing TOP colour detectors
  record    compact results.json for the experiment record from the raw outputs (hashes)

Simulator state is used only to place things and to measure what the TOP cameras
can see; nothing here is a robot input or a student result.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import io
import json
import math
import os
import platform
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

X = Path(__file__).resolve().parent
ENV_V3 = ROOT/'experiments'/'2026-09-26-zone-env-v3'
RAW = Path('/Users/changmin/projects/ugrp/outputs/zone-eval-topcam-20260926')
PREREG = json.loads((X/'prereg.json').read_text())
MOVED = 'cctv_top_north_east'
MAP_V3, MAP_V1 = 'zone_wide_corridor_tags_v3', 'zone_wide_corridor_tags_v1'
LEVELS = (.003, .032)
BOX_TOP_Z, BOX_XY = .032, (.034, .040)
LANE, BAY = 'corridor_corridor_1', 'passing_bay_bay_1'
MOSAIC = (('cctv_top_north', 'cctv_top_north_east'), ('cctv_top', 'cctv_top_east'))


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    import subprocess
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True).stdout.strip()


def static_checks():
    """env v3 static_checks.py imported byte-identical (its hash goes into every output)."""
    path = ENV_V3/'static_checks.py'
    spec = importlib.util.spec_from_file_location('env_v3_static_checks', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, sha_file(path)


def header(kind):
    import mujoco
    return {'schema': f'ugrp.zone_eval_topcam.{kind}.v1', 'code': {'sha': git('rev-parse', 'HEAD'), 'dirty': bool(git(
        'status', '--porcelain', '--', 'sim', 'harness', 'scripts', 'experiments/2026-09-26-zone-eval-topcam'))},
        'prereg_sha256': sha_file(X/'prereg.json'), 'env': {
            'python': platform.python_version(), 'mujoco': mujoco.__version__, 'numpy': np.__version__,
            'platform': platform.platform(), 'threads': {k: os.environ.get(k) for k in (
                'OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS', 'MKL_NUM_THREADS')}},
        'load_average_start': [round(v, 2) for v in os.getloadavg()],
        'note': 'diagnostic placement with simulator state; evaluation-only TOP; not a robot input'}


def fresh_dir(path):
    path = Path(path)
    if path.exists() and any(path.iterdir()):
        raise SystemExit(f'refusing to overwrite existing results in {path}')
    path.mkdir(parents=True, exist_ok=True)
    return path


def box_area_px(z, fovy=55., height_px=720):
    scale = height_px/(2*(z - BOX_TOP_Z)*math.tan(math.radians(fovy)/2))
    return BOX_XY[0]*BOX_XY[1]*scale*scale


# --------------------------------------------------------------------------- search

class Floor:
    """ST5 floor grid, regions and the three fixed TOPs of one map (same code path as st5)."""

    def __init__(self, sc, name):
        import mujoco
        from sim.zone_tag_rule_v3 import pickup_bays
        self.sc, self.name = sc, name
        self.scene, self.world = sc.make_world(name, render=False)
        sc.clear_scene(self.world, self.scene, keep_robot=None)
        static = self.scene.config['static_map']
        x0, x1, y0, y1 = static['bounds_m']
        xs = np.arange(x0 + .025 + sc.GRID_M/2, x1 - .025, sc.GRID_M)
        ys = np.arange(y0 + .025 + sc.GRID_M/2, y1 - .025, sc.GRID_M)
        gx, gy = np.meshgrid(xs, ys)
        inside = np.zeros_like(gx, bool)
        for o in (o for o in static['obstacles'] if o.get('kind') == 'wall'):
            (cx, cy), (hx, hy) = o['center_m'], o['half_extents_m']
            inside |= (np.abs(gx - cx) <= hx + .005) & (np.abs(gy - cy) <= hy + .005)
        floor = ~inside.ravel()
        self.regions = {k: m.ravel() & floor for k, m in sc.region_masks(static, gx, gy, pickup_bays).items()}
        self.regions = {k: m for k, m in self.regions.items() if m.any()}
        self.cam_id = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_CAMERA, MOVED)
        self.authored = [float(v) for v in self.world.model.cam_pos[self.cam_id]]
        self.pts, self.fixed = {}, {}
        for z in LEVELS:
            self.pts[z] = np.stack((gx.ravel(), gy.ravel(), np.full(gx.size, z)), axis=1)
            count = np.zeros(gx.size, np.int32)
            for c in static['top_cameras']:
                if c['name'] != MOVED:
                    count += self.visible(c['name'], self.pts[z])
            self.fixed[z] = count

    def visible(self, cname, pts):
        """Lines of st5: projection into the 640 x 480 image, then mj_multiRay (groups 0-4)."""
        import mujoco
        m, d, sc = self.world.model, self.world.data, self.sc
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cname)
        c = np.array(d.cam_xpos[cid]); rot = np.array(d.cam_xmat[cid]).reshape(3, 3)
        pc = (pts - c) @ rot
        depth = -pc[:, 2]
        f = 240./math.tan(math.radians(float(m.cam_fovy[cid]))/2)
        with np.errstate(divide='ignore', invalid='ignore'):
            u = 320. + f*pc[:, 0]/depth
            v = 240. - f*pc[:, 1]/depth
        in_img = (depth > 0) & (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
        vec = pts - c
        target = np.linalg.norm(vec, axis=1)
        dirs = vec/target[:, None]
        vis = np.zeros(len(pts), bool)
        idx = np.nonzero(in_img)[0]
        if len(idx):
            _, dist = sc.ray_hits(self.world, c, dirs[idx], sc.GEOMGROUP_TOP)
            vis[idx] = (dist < 0) | (dist >= target[idx] - 1e-3)
        return vis

    def fractions(self, position):
        """Region fractions (rounded like st5) with the moved TOP at ``position``."""
        import mujoco
        m = self.world.model
        m.cam_pos[self.cam_id] = position
        mujoco.mj_forward(m, self.world.data)
        out = {}
        for z in LEVELS:
            need = np.nonzero(self.fixed[z] == 0)[0]
            seen = self.fixed[z] > 0
            seen[need] = self.visible(MOVED, self.pts[z][need])
            out[f'z{z:.3f}'] = {k: round(float(seen[mask].mean()), 5) for k, mask in self.regions.items()}
        return out

    def close(self):
        self.world.close()


def gate(v3, v1, base3, base1, z):
    g1 = all(v3[lv][LANE] >= .97 and v3[lv][BAY] >= .97 for lv in v3)
    g2 = all(v3[lv][k] >= base3[lv][k] for lv in v3 for k in v3[lv] if k not in (LANE, BAY))
    g3 = v1 is not None and all(v1[lv][k] >= base1[lv][k] for lv in v1 for k in v1[lv])
    g4 = box_area_px(z) >= 60.
    return {'G1': g1, 'G2': g2, 'G3': g3, 'G4': g4}


def selection_key(row, authored):
    """prereg selection order: lowest z, smallest horizontal move, highest floor_all, smallest x, y."""
    x, y, z = row['position_m']
    return (z, round(math.hypot(x - authored[0], y - authored[1]), 6), -row['v3']['z0.003']['floor_all'], x, y)


def g1b(row):
    """Amendment A1: no ST5 grid point of the lane or the passing bay hidden (both floor levels)."""
    return all(row['v3'][lv][LANE] == 1. and row['v3'][lv][BAY] == 1. for lv in row['v3'])


def wall_strip_m(camera_y, camera_z, wall_face_y=.925, wall_h=.40, floor_z=.003):
    """Analytic floor strip hidden behind the corridor walls' north face (long wall, camera south of it)."""
    h, H = wall_h - floor_z, camera_z - floor_z
    return max(0., h*(wall_face_y - camera_y)/(H - h))


def run_select(out, log):
    """Apply prereg + amendment A1 to the saved search (no new measurement)."""
    path = RAW/'search'/'search.json'
    search = json.loads(path.read_text())
    auth = search['authored_position_m']
    passing = sorted((r for r in search['rows'] if r['pass']), key=lambda r: selection_key(r, auth))
    amended = [r for r in passing if g1b(r)]
    pick = lambda r: None if r is None else {**{k: r[k] for k in ('position_m', 'gate', 'v3', 'box_area_px_960x720')},
                                             'hidden_strip_north_of_corridor_walls_m': round(wall_strip_m(
                                                 r['position_m'][1], r['position_m'][2]), 4)}
    result = {**header('selection'), 'search': str(path), 'search_sha256': sha_file(path),
              'amendments_sha256': sha_file(X/'prereg_amendments.json'), 'search_code': search['code'],
              'passing_G1_G4': len(passing), 'passing_G1_G4_and_G1b': len(amended),
              'prereg_rule_selection': pick(passing[0] if passing else None),
              'amended_selection': pick(amended[0] if amended else None),
              'authored_position_m': auth,
              'authored_hidden_strip_m': round(wall_strip_m(auth[1], auth[2]), 4),
              'next_amended_candidates': [pick(r) for r in amended[1:6]]}
    (out/'selection.json').write_text(json.dumps(result, indent=1) + '\n')
    log(f"prereg rule {result['prereg_rule_selection']['position_m'] if passing else None}; "
        f"amended {result['amended_selection']['position_m'] if amended else None}")
    return result


def run_search(out, log):
    sc, sc_sha = static_checks()
    spec = PREREG['change_space']['grid']
    axis = {k: np.round(np.arange(a, b + 1e-9, s), 3) for k, (a, b, s) in spec.items()}
    v3, v1 = Floor(sc, MAP_V3), Floor(sc, MAP_V1)
    try:
        base3, base1 = v3.fractions(v3.authored), v1.fractions(v1.authored)
        log(f'authored {v3.authored}: v3 lane {base3["z0.003"][LANE]} bay {base3["z0.003"][BAY]}')
        rows = []
        for z in axis['z_m']:
            for x in axis['x_m']:
                for y in axis['y_m']:
                    pos = [float(x), float(y), float(z)]
                    f3 = v3.fractions(pos)
                    f1 = v1.fractions(pos) if all(
                        f3[lv][LANE] >= .97 and f3[lv][BAY] >= .97 for lv in f3) else None
                    g = gate(f3, f1, base3, base1, float(z))
                    rows.append({'position_m': pos, 'gate': g, 'pass': all(g.values()),
                                 'v3': {lv: {k: f3[lv][k] for k in (LANE, BAY, 'floor_all', 'within_0.30m_of_interior_walls')}
                                        for lv in f3},
                                 'v3_min_other_delta': min(f3[lv][k] - base3[lv][k] for lv in f3 for k in f3[lv]
                                                           if k not in (LANE, BAY)),
                                 'box_area_px_960x720': round(box_area_px(float(z)), 1)})
            log(f'z {z}: {sum(r["pass"] for r in rows)} passing so far of {len(rows)}')
        auth = v3.authored
        passing = sorted((r for r in rows if r['pass']), key=lambda r: selection_key(r, auth))
        selected = passing[0] if passing else None
        detail = None
        if selected:
            detail = {'v3': v3.fractions(selected['position_m']), 'v1': v1.fractions(selected['position_m'])}
    finally:
        v3.close(); v1.close()
    result = {**header('search'), 'static_checks_sha256': sc_sha, 'moved_camera': MOVED, 'authored_position_m': auth,
              'grid': {k: [float(v) for v in a] for k, a in axis.items()}, 'candidates': len(rows),
              'passing': len(passing), 'baseline_v1': {'v3': base3, 'v1': base1}, 'selected': selected,
              'selected_regions': detail, 'first_passing_10': passing[:10], 'rows': rows}
    (out/'search.json').write_text(json.dumps(result, indent=1) + '\n')
    return result


# --------------------------------------------------------------------------- coverage (unmodified st5)

def run_coverage(out, profile, log):
    import sim.zone_eval_top as zet
    sc, sc_sha = static_checks()
    original, applied = sc.make_world, []

    def make_world(name, seed=sc.SEED, render=True):
        scene, world = original(name, seed=seed, render=render)
        applied.append(zet.apply_to_world(world, scene.config['static_map'], profile))
        return scene, world
    sc.make_world = make_world
    try:
        st5 = sc.st5(out, log)
    finally:
        sc.make_world = original
    tables = sc.summarize({'ST5': st5})['ST5']
    result = {**header('coverage'), 'static_checks_sha256': sc_sha, 'profile': zet.profile_record(profile),
              'applied': applied, 'ST5': st5, 'tables': tables,
              'top_coverage_png_sha256': sha_file(out/'top_coverage.png')}
    result['load_average_end'] = [round(v, 2) for v in os.getloadavg()]
    (out/'coverage.json').write_text(json.dumps(result, indent=1, ensure_ascii=False) + '\n')
    return result


# --------------------------------------------------------------------------- stills

# (x, y) box spots per layout on the corridor v3 map. hidden_bands: floor v1 cannot see
# (env v3 ST5 unseen bands north of the corridor walls, east side of the bay, west of
# the bay west wall). open_floor: spots both the v1 and the v2 cctv_top_north_east see,
# so detector differences there come from the camera placement, not from occlusion.
STILL_LAYOUTS = {'hidden_bands': ((3.65, .965), (2.50, .965), (3.335, .50), (2.70, .60)),
                 'open_floor': ((4.60, .40), (4.60, -.20), (3.00, -.40), (4.00, .20))}
STILL_ROBOT = ((2.95, 1.18), 0.)
MATCH_M = .05


def still_world(sc, name):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_landmarks import TaggedZoneScene
    scene = TaggedZoneScene.from_tagged(name, sc.SEED, sc.GOAL, sc.EXTRA, contact_profile=sc.PROFILE)
    world = MultiMasterPiProductionV2(seed=sc.SEED, width=960, height=720, render=True,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    return scene, world


def place_still(sc, scene, world, spots):
    import mujoco
    rid = sorted(world.controllers)[0]
    sc.clear_scene(world, scene, keep_robot=rid)
    (x, y), yaw = STILL_ROBOT
    world.controllers[rid].set_base_pose_for_test((x, y, .0324), yaw)
    placed = []
    for (oid, obj), xy in zip(sorted(scene.config['setup_only']['objects'].items()), spots):
        sc.set_free(world, obj['body_name'], (xy[0], xy[1], .016), (1., 0., 0., 0.))
        placed.append({'object': oid, 'kind': obj['kind'], 'xy_m': list(xy)})
    mujoco.mj_forward(world.model, world.data)
    return rid, placed


def detect_placed(frames, cameras, placed):
    """Existing TOP colour detectors on every view; which placed box each camera found (<= 5 cm)."""
    from harness.zone_color_boxes import TOP_PROFILE_BASELINE, TOP_PROFILE_ZONE, detect_top
    kinds = sorted({p['kind'] for p in placed})
    out = {}
    for det_profile in (TOP_PROFILE_BASELINE, TOP_PROFILE_ZONE):
        rows = []
        for name, jpeg in frames.items():
            rows += detect_top(jpeg, cameras[name], kinds, profile=det_profile)
        found = {p['object']: sorted({r['camera'] for r in rows if r['kind'] == p['kind']
                                      and math.dist(r['floor_xy_m'], p['xy_m']) <= MATCH_M}) for p in placed}
        out[det_profile] = {'detections': rows, 'placed_found_by': found}
    return out


def run_stills(out, log):
    from PIL import Image
    import sim.zone_eval_top as zet
    sc, sc_sha = static_checks()
    rows, mosaics = [], {}
    for layout, spots in STILL_LAYOUTS.items():
        for profile in ('zone_eval_top_v1', 'zone_eval_top_v2'):
            scene, world = still_world(sc, MAP_V3)
            try:
                rid, placed = place_still(sc, scene, world, spots)
                static = scene.config['static_map']
                record = zet.apply_to_world(world, static, profile)
                cameras = {c['name']: c for c in zet.eval_top_cameras(static, profile)}
                frames = {}
                for name in cameras:
                    frames[name] = world.render_team_jpeg(camera=name, quality=95)
                    (out/f'{layout}_{profile}_{name}.jpg').write_bytes(frames[name])
                det = detect_placed(frames, cameras, placed)
            finally:
                world.close()
            tiles = [np.concatenate([np.asarray(Image.open(io.BytesIO(frames[n]))) for n in row], axis=1) for row in MOSAIC]
            mosaics[(layout, profile)] = np.concatenate(tiles, axis=0)
            rows.append({'layout': layout, 'profile': profile, 'record': record, 'robot': rid, 'robot_pose': STILL_ROBOT,
                         'boxes': placed, 'frames_sha256': {n: hashlib.sha256(j).hexdigest() for n, j in frames.items()},
                         'detectors': det})
            log(f'{layout} {profile}: ' + '; '.join(f"{k} {v['placed_found_by']}" for k, v in det.items()))
    for layout in STILL_LAYOUTS:
        both = np.concatenate([mosaics[(layout, 'zone_eval_top_v1')], mosaics[(layout, 'zone_eval_top_v2')]], axis=1)
        Image.fromarray(both).save(out/f'mosaic_{layout}_v1_left_v2_right.jpg', quality=90)
    result = {**header('stills'), 'static_checks_sha256': sc_sha, 'map': MAP_V3, 'rows': rows,
              'mosaics_sha256': {p.name: sha_file(p) for p in sorted(out.glob('mosaic_*.jpg'))}}
    result['load_average_end'] = [round(v, 2) for v in os.getloadavg()]
    (out/'stills.json').write_text(json.dumps(result, indent=1) + '\n')
    return result


def run_record(out, log):
    """Compact results.json for the experiment record, built only from the raw outputs (hashes kept)."""
    raw = {'search': RAW/'search'/'search.json', 'select': RAW/'select'/'selection.json',
           'coverage_v1': RAW/'coverage_zone_eval_top_v1'/'coverage.json',
           'coverage_v2': RAW/'coverage_zone_eval_top_v2'/'coverage.json',
           'stills_superseded': RAW/'stills'/'stills.json', 'stills': RAW/'stills2'/'stills.json'}
    data = {k: json.loads(p.read_text()) for k, p in raw.items()}
    recorded = json.loads((ENV_V3/'static_results.json').read_text())['tables']['ST5']
    v1, v2 = data['coverage_v1']['tables'], data['coverage_v2']['tables']
    changed = {k: {r: [v1[k][r], v2[k][r]] for r in v1[k] if v1[k][r] != v2[k][r]} for k in v1}
    levels = {}
    for fam in ('walls_0.10_v1', 'walls_0.40_v3'):
        for pid, key in (('zone_eval_top_v1', 'coverage_v1'), ('zone_eval_top_v2', 'coverage_v2')):
            res = data[key]['ST5']['results']['zone_wide_corridor'][fam]
            levels[f'{fam} {pid}'] = {lv: {r: res[lv][r]['top_visible_fraction'] for r in (
                LANE, BAY, 'floor_all', 'within_0.30m_of_interior_walls')} for lv in res}
    unseen = {k: {f: v for f, v in row.items() if f in ('unseen_points', 'unseen_bbox_m', 'blocking_geoms')}
              for k, row in data['coverage_v2']['ST5']['results']['zone_wide_corridor']['walls_0.40_v3']['z0.003'].items()
              if 'unseen_points' in row}
    stills = [{k: r[k] for k in ('layout', 'profile', 'boxes')} | {
        'found_by': {d: v['placed_found_by'] for d, v in r['detectors'].items()}} for r in data['stills']['rows']]
    result = {'schema': 'ugrp.zone_eval_topcam.results.v1',
              'raw': {k: {'path': str(p), 'sha256': sha_file(p), 'code': data[k]['code'],
                          'load_average_start': data[k].get('load_average_start'),
                          'load_average_end': data[k].get('load_average_end')} for k, p in raw.items()},
              'static_checks_sha256': data['coverage_v1']['static_checks_sha256'],
              'profiles': {pid: data[key]['profile'] for pid, key in (('zone_eval_top_v1', 'coverage_v1'),
                                                                       ('zone_eval_top_v2', 'coverage_v2'))},
              'search': {k: data['select'][k] for k in ('passing_G1_G4', 'passing_G1_G4_and_G1b', 'prereg_rule_selection',
                                                         'amended_selection', 'authored_hidden_strip_m')}
              | {'candidates': data['search']['candidates']},
              'v1_reproduces_env_v3_ST5_tables': v1 == recorded,
              'ST5_changed_regions_v1_to_v2_z0.003': changed, 'corridor_both_levels': levels,
              'v2_corridor_v3_unseen_z0.003': unseen, 'stills': stills,
              'stills_superseded_note': ('first stills run (source d9eb5895): one layout, detector kinds passed with a '
                                         'duplicate (red twice) so top_zone_v2 listed one red box twice; kept, superseded by stills2')}
    (X/'results.json').write_text(json.dumps(result, indent=1, ensure_ascii=False) + '\n')
    (out/'record.json').write_text(json.dumps({'results_sha256': sha_file(X/'results.json')}) + '\n')
    log(f"v1 reproduces env v3: {result['v1_reproduces_env_v3_ST5_tables']}")
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('command', choices=('search', 'select', 'coverage', 'stills', 'record'))
    p.add_argument('--profile', default=None, help='coverage: evaluation TOP profile id (required)')
    p.add_argument('--output', default=None)
    args = p.parse_args(argv)
    if args.command == 'coverage' and not args.profile:
        p.error('coverage needs --profile')
    name = args.command if args.command != 'coverage' else f'coverage_{args.profile}'
    out = fresh_dir(args.output or RAW/name)
    started, lines = time.time(), []

    def log(msg):
        lines.append(f'{time.time() - started:8.1f}s {msg}')
        print(lines[-1], flush=True)
    try:
        if args.command == 'search':
            run_search(out, log)
        elif args.command == 'select':
            run_select(out, log)
        elif args.command == 'coverage':
            run_coverage(out, args.profile, log)
        elif args.command == 'record':
            run_record(out, log)
        else:
            run_stills(out, log)
    except BaseException as exc:
        lines.append(f'FAILED {type(exc).__name__}: {exc}')
        raise
    finally:
        lines.append(f'wall_s {time.time() - started:.1f} load {os.getloadavg()}')
        (out/f'{name}.log').write_text('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
