#!/usr/bin/env python3
"""Render the zone cargo catalogue PNG and check TOP-RGB colour separability.

Places every catalogue kind on the ``zone_wide`` floor (plus the four painted
zone boxes in the pickup area), renders labelled oblique close-ups and the
robots' TOP views at their real resolution (960x720), and measures each kind's
median colour in the TOP image through MuJoCo segmentation (evaluation only).
Pairwise CIELAB distances (dE76) to the box paints, floor and zone paints are
written to ``colour_check.json``.

  .venv-sim/bin/python -m scripts.render_zone_cargo_catalogue --output outputs/zone-cargo/catalogue
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.zone_cargo import CATALOGUE, EXISTING_SOLO, bounding_box, catalogue_record  # noqa: E402

LAYOUT = {'can': (2.55, .95, 0.), 'tile': (2.95, .95, 0.), 'long_beam': (3.85, .95, 0.),
          'heavy_crate': (2.75, .05, 0.), 'tri_frame': (3.95, -.05, 0.)}
GOAL = {'A': {'red': 1, 'cyan': 1}, 'B': {'green': 1, 'yellow': 1}}
# Evaluation-only snapshot of the TOP zone profile in draft PR #163
# (harness/zone_color_boxes.py TOP_ZONE_HSV, branch claude/zone-rgb-color);
# the detector itself is not modified or imported here.
PR163_TOP_ZONE_HSV = {
    'cyan': (((84, 90, 40), (96, 255, 255)),),
    'red': (((0, 120, 50), (6, 255, 255)), ((172, 120, 50), (179, 255, 255))),
    'green': (((55, 100, 40), (72, 255, 255)),),
    'yellow': (((22, 140, 80), (33, 255, 255)),),
}


def hsv_overlap(rgb_pixels, table):
    """Fraction of pixels that fall in each box-colour HSV class."""
    import cv2
    if not len(rgb_pixels):
        return {}
    hsv = cv2.cvtColor(rgb_pixels.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV)
    out = {}
    for kind, ranges in table.items():
        mask = np.zeros(len(rgb_pixels), bool)
        for low, high in ranges:
            mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8)).ravel() > 0
        out[kind] = round(float(mask.mean()), 4)
    return out


def lab(rgb):
    import cv2
    px = np.uint8([[np.clip(rgb, 0, 255)]])
    L, a, b = cv2.cvtColor(px, cv2.COLOR_RGB2LAB)[0, 0].astype(float)
    return [L*100/255, a-128, b-128]


def main(argv=None):
    import cv2
    import mujoco
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from harness.zone_perception import HSV_RANGES
    from sim.zone_arena import top_views
    from sim.zone_cargo_scene import CargoZoneScene
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--output', required=True)
    a = p.parse_args(argv)
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    items = [{'item_id': k, 'kind': k, 'pose': list(v)} for k, v in LAYOUT.items()]
    scene = CargoZoneScene.from_cargo_config('zone_wide', 11, cargo=items, goal=GOAL)
    world = MultiMasterPiProductionV2(seed=11, width=320, height=240, render=False,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    m, d = world.model, world.data
    for _ in range(int(.5/m.opt.timestep)):
        mujoco.mj_step(m, d)
    # geoms per label
    groups = {}
    for g in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
        if name.startswith('cargo_') and '__' in name:
            groups.setdefault(name[6:].split('__')[0], set()).add(g)
        elif name.startswith('cargo_box_') and name.endswith('_geom'):
            oid = name[6:-5]
            kind = scene.config['setup_only']['objects'][oid]['kind']
            groups.setdefault('box_' + kind, set()).add(g)
        elif name == 'floor':
            groups['floor'] = {g}
        elif name.startswith('zone_zone_'):
            groups['paint_' + name[10:]] = {g}
        elif name == 'zone_pickup':
            groups['paint_pickup'] = {g}
        elif name[:4] in ('r1__', 'r2__', 'r3__'):
            groups.setdefault('robots', set()).add(g)
    # TOP views at the robots' resolution
    top = mujoco.Renderer(m, 720, 960)
    seg = mujoco.Renderer(m, 720, 960)
    seg.enable_segmentation_rendering()
    colours, pixels, top_imgs, robot_px, cargo_px = {}, {}, {}, [], {}
    for camera, label, *_ in top_views(scene.config['static_map']):
        top.update_scene(d, camera=camera)
        rgb = top.render().copy()
        seg.update_scene(d, camera=camera)
        ids = seg.render()[..., 0]
        top_imgs[label] = rgb
        cv2.imwrite(str(out/f'top-{label}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        robot_mask = np.isin(ids, list(groups.get('robots', ())))
        if robot_mask.any():
            robot_px.append(rgb[robot_mask])
        for key, geoms in groups.items():
            if key == 'robots':
                continue
            mask = np.isin(ids, list(geoms))
            if key.startswith('paint_') or key == 'floor':
                # paint/floor pixels not covered by anything else
                mask &= ~np.isin(ids, [g for k, gs in groups.items() if k not in (key,) for g in gs])
            n = int(mask.sum())
            if key in CATALOGUE and n:
                cargo_px.setdefault(key, []).append(rgb[mask])
            if n > pixels.get(key, (0,))[0]:
                pixels[key] = (n, label)
                px = rgb[mask].astype(float)
                # The TOP sees the lit upward faces; shaded sides would drag a
                # small object's median towards the floor. Use the brighter half.
                bright = px[px.sum(1) >= np.median(px.sum(1))]
                colours[key] = np.median(bright, axis=0).tolist()
    labs = {k: lab(v) for k, v in colours.items()}
    cargo_keys = sorted(CATALOGUE)
    ref_keys = sorted(k for k in colours if k not in cargo_keys)
    pairs = []
    for x, y in itertools.chain(itertools.combinations(cargo_keys, 2), itertools.product(cargo_keys, ref_keys)):
        if x in labs and y in labs:
            pairs.append({'a': x, 'b': y, 'dE76': round(float(np.linalg.norm(np.subtract(labs[x], labs[y]))), 1)})
    pairs.sort(key=lambda r: r['dE76'])
    # Robot parts (orange pads, yellow rollers, aluminium, black): count robot
    # pixels (three robots, all TOP views) within dE76 < 15 of each cargo colour.
    import cv2 as _cv2
    rpx = np.concatenate(robot_px) if robot_px else np.zeros((0, 3), np.uint8)
    rlab = _cv2.cvtColor(rpx.reshape(-1, 1, 3).astype(np.uint8), _cv2.COLOR_RGB2LAB).reshape(-1, 3).astype(float)
    rlab = np.column_stack([rlab[:, 0]*100/255, rlab[:, 1]-128, rlab[:, 2]-128])
    robot_clash = {k: int((np.linalg.norm(rlab - np.array(labs[k]), axis=1) < 15).sum())
                   for k in cargo_keys if k in labs}
    check = {'schema': 'ugrp.zone_cargo_colour_check.v1', 'resolution': [960, 720],
             'median_rgb': {k: [round(v, 1) for v in colours[k]] for k in sorted(colours)},
             'visible_pixels': {k: {'count': pixels[k][0], 'view': pixels[k][1]} for k in sorted(pixels)},
             'lab': {k: [round(v, 1) for v in labs[k]] for k in sorted(labs)},
             'closest_pairs': pairs[:12], 'min_dE76_cargo_vs_all': pairs[0] if pairs else None,
             'box_class_hsv_overlap': {
                 'zone_perception_v1 (main)': {k: hsv_overlap(np.concatenate(v), {
                     kind: [tuple(map(tuple, r)) for r in ranges] for kind, ranges in HSV_RANGES.items()})
                     for k, v in cargo_px.items()},
                 'top_zone_v2 (PR #163 snapshot)': {k: hsv_overlap(np.concatenate(v), PR163_TOP_ZONE_HSV)
                                                   for k, v in cargo_px.items()}},
             'robot_pixels_total': int(len(rpx)),
             'robot_pixels_within_dE15': robot_clash,
             'note': ('median colour of the brighter half of segmented pixels in the TOP render (evaluation only); shape and size '
                      'differ as well, and the robots receive only RGB')}
    (out/'colour_check.json').write_text(json.dumps(check, indent=2) + '\n')
    # oblique close-ups
    close = mujoco.Renderer(m, 300, 400)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    tiles = []
    entries = [('box', None)] + [(k, LAYOUT[k]) for k in ('can', 'tile', 'long_beam', 'heavy_crate', 'tri_frame')]
    for key, pose in entries:
        if key == 'box':
            oid = next(iter(scene.config['setup_only']['objects']))
            body = scene.config['setup_only']['objects'][oid]['body_name']
            target = d.body(body).xpos.copy()
            dist, text = .35, ['box (existing, 4 paints)', '34x40x32 mm  0.030 kg', 'robots: 1']
        else:
            spec = CATALOGUE[key]
            target = np.array([pose[0], pose[1], .02])
            size = max(bounding_box(spec)[:2])
            dist = max(.35, 1.6*size)
            dims = 'x'.join(str(round(v*1000)) for v in bounding_box(spec))
            text = [f'{spec.label} ({spec.colour})', f'{dims} mm  {spec.mass_kg:.3f} kg',
                    f'robots: {spec.required_carriers} ({spec.tier})']
        cam.lookat[:] = target
        cam.distance, cam.elevation, cam.azimuth = dist, -35., 215.
        close.update_scene(d, camera=cam)
        img = cv2.cvtColor(close.render(), cv2.COLOR_RGB2BGR)
        for k, line in enumerate(text):
            cv2.putText(img, line, (8, 22+22*k), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(img, line, (8, 22+22*k), cv2.FONT_HERSHEY_SIMPLEX, .55, (15, 15, 15), 1, cv2.LINE_AA)
        tiles.append(img)
    grid = np.vstack([np.hstack(tiles[0:3]), np.hstack(tiles[3:6])])
    ne = cv2.cvtColor(top_imgs['TOP_NE'], cv2.COLOR_RGB2BGR)
    ne = cv2.resize(ne, (800, 600), interpolation=cv2.INTER_AREA)
    cv2.putText(ne, 'TOP_NE as robots receive it (960x720, shown scaled)', (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                .6, (255, 255, 255), 2, cv2.LINE_AA)
    sheet = np.hstack([grid, ne])
    header = np.full((44, sheet.shape[1], 3), 245, np.uint8)
    cv2.putText(header, f'UGRP zone cargo catalogue v{catalogue_record()["version"]}  '
                f'(masses are SIM values relative to the SIM MasterPi; weld OFF)', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, .8, (20, 20, 20), 2, cv2.LINE_AA)
    sheet = np.vstack([header, sheet])
    cv2.imwrite(str(out/'catalogue.png'), sheet)
    record = {'catalogue': catalogue_record(), 'existing_solo': EXISTING_SOLO, 'layout': LAYOUT,
              'scene_xml_sha256': hashlib.sha256(world.scene_xml.encode()).hexdigest()}
    (out/'catalogue.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({'min_pair': check['min_dE76_cargo_vs_all'], 'closest': pairs[:6], 'robot_clash': robot_clash,
                      'pixels': {k: v['count'] for k, v in check['visible_pixels'].items()}}, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
