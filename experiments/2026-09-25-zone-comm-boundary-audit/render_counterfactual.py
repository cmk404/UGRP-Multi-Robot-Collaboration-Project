"""Counterfactual for RGB provenance: move boxes in simulator truth, re-render the
TOP cameras and check that the RGB estimate follows the image (not the layout).

Render-only (no controller, no LLM, no teacher job). The ZC2 s14 scene is built
exactly as scripts/run_zone_dispatch.ZoneRun does, stepped 0.5 SIM s as the run
does before its first capture, then selected box bodies are displaced by
writing their free-joint qpos and calling mj_forward (no further physics).
Prints one JSON document with per-box (true, RGB estimate, error) before and
after, and the label/observe outputs from the unchanged harness code.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from harness.zone_perception import detect_all, label_pickup, observe  # noqa: E402
from sim.zone_arena import episode, goal_counts  # noqa: E402

GOAL = {"A": {"red": 2}, "B": {"cyan": 2}, "C": {"green": 1, "yellow": 1}}
EXTRA = {"red": 1, "cyan": 1}
# Off-grid displacements (m): not multiples of the 0.6/0.8 m layout pitch.
MOVES = {'cargo_box_00': (.037, -.023), 'cargo_box_03': (-.051, .029)}


def match(detections, xy, kind):
    same = [d for d in detections if d['kind'] == kind]
    best = min(same, key=lambda d: math.dist(d['floor_xy_m'], xy)) if same else None
    return best and {'rgb_xy': best['floor_xy_m'], 'err_m': round(math.dist(best['floor_xy_m'], xy), 4)}


def main():
    import mujoco
    from scripts.run_zone_dispatch import ZoneRun
    p = argparse.ArgumentParser()
    p.add_argument('--seed', type=int, default=14)
    args = p.parse_args()
    config = episode('zone_wide', args.seed, goal=goal_counts(GOAL), extra_boxes=EXTRA)
    config['contact_solver_profile'] = 'local_contact_fine'
    config['extra_boxes'] = EXTRA
    out = Path(tempfile.mkdtemp(prefix='zone-render-cf-')) / 'run'
    zone = ZoneRun(config, out)
    zone.step(.5)
    static = config['static_map']
    objects = config['setup_only']['objects']

    def truth():
        return {o['body_name']: [round(float(v), 4) for v in zone.world.data.body(o['body_name']).xpos[:2]]
                for o in objects.values()}

    before_tops = zone.tops()
    before_det = detect_all(before_tops, static)
    labels = label_pickup(before_det, static)
    t0 = truth()
    model, data = zone.world.model, zone.world.data
    for body, (dx, dy) in MOVES.items():
        joint = next(o['joint_name'] for o in objects.values() if o['body_name'] == body)
        adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)]
        data.qpos[adr] += dx
        data.qpos[adr+1] += dy
    mujoco.mj_forward(model, data)
    after_tops = zone.tops()
    after_det = detect_all(after_tops, static)
    t1 = truth()
    kinds = {o['body_name']: o['kind'] for o in objects.values()}
    rows = []
    for body in sorted(t0):
        b, a = match(before_det, t0[body], kinds[body]), match(after_det, t1[body], kinds[body])
        moved = [round(t1[body][i]-t0[body][i], 4) for i in (0, 1)]
        rgb_moved = b and a and [round(a['rgb_xy'][i]-b['rgb_xy'][i], 4) for i in (0, 1)]
        rows.append({'body': body, 'kind': kinds[body], 'true_before': t0[body], 'true_after': t1[body],
                     'rgb_before': b, 'rgb_after': a, 'true_shift_m': moved, 'rgb_shift_m': rgb_moved})
    report = {'seed': args.seed, 'moves': MOVES, 'rows': rows,
              'labels': labels, 'observe_before': observe(before_tops, static, labels),
              'observe_after': observe(after_tops, static, labels),
              'layout_positions': {o['body_name']: o['position_m'][:2] for o in objects.values()}}
    zone.close()
    zone.world.close()
    print(json.dumps(report, indent=1, sort_keys=True))


if __name__ == '__main__':
    main()
