"""Stage 2 (door) test scenario generator: rng 20260928, seeds 811-816.

Beam west of door_1 roughly on the door axis: x U[0.85, 1.10], y 0.05 + U[-0.10, 0.10],
yaw U[-0.10, 0.10]; start offsets dx, dy U[-0.1, 0.1], dyaw U[-0.4, 0.4]. Kept only if
* both pre-stations are plannable from the spawns (runner keep-outs, v1/v2 envelope);
* every start chassis corner (+-0.15 m square) is >= 0.03 m from the west wall inner face
  (stage 1 finding: start offsets touched the wall);
* the #202 feasibility checker (harness.zone_scenario_feasibility, evaluation-only tool) says
  'feasible' for the beam to zone A.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from gen_stage1_seeds import ROLES, TaggedZoneScene, pa, plan_path, instances, world_grasps  # noqa: E402
from scripts.run_m2_pair import KEEPOUT_PAD_M, PARTNER_KEEPOUT_HALF_M  # noqa: E402
from harness.zone_scenario_feasibility import evaluate  # noqa: E402

WEST_FACE = -1.025


def start_clear(spawn, off):
    x, y, yaw = spawn[0] + off[0], spawn[1] + off[1], spawn[3] + off[2]
    c, s = math.cos(yaw), math.sin(yaw)
    xs = [x + c * dx - s * dy for dx in (-.15, .15) for dy in (-.15, .15)]
    return min(xs) - WEST_FACE >= .03


def plannable(static, spawns, beam):
    inst = instances([{'item_id': 'beam', 'kind': 'long_beam', 'pose': list(beam)}])[0]
    sheet = pa.coarse_order_sheet(beam)
    g = world_grasps(inst, pose=sheet['beam_xyyaw'])
    st = {r: g[role]['base_xyyaw'] for r, role in ROLES.items()}
    pre = {r: pa.prestation(st[r], .30) for r in ROLES}
    out = {}
    for r in ROLES:
        p = next(q for q in ROLES if q != r)
        ko = [pa.beam_keepout(sheet['beam_xyyaw'], .6, .04, KEEPOUT_PAD_M)] + [
            {'id': t, 'center_m': list(xy[:2]), 'half_extents_m': [PARTNER_KEEPOUT_HALF_M] * 2}
            for t, xy in (('s', st[p]), ('p', pre[p]))]
        res = plan_path(static, spawns[r][:2], pre[r][:2], pa.APPROACH_ENVELOPE, obstacles=ko, escape_start_m=.25)
        if res is None:
            return None
        out[r] = round(res['length_m'], 2)
    return out


def main():
    scene = TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 11, {'A': {'cyan': 1}}, None)
    static, spawns = scene.config['static_map'], scene.config['setup_only']['spawns']
    rng = np.random.default_rng(20260928)
    out, seed = {}, 811
    while seed <= 816:
        beam = (round(rng.uniform(.85, 1.10), 2), round(.05 + rng.uniform(-.10, .10), 2), round(rng.uniform(-.10, .10), 2))
        start = {r: (round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.4, .4), 2))
                 for r in ROLES}
        if not all(start_clear(spawns[r], start[r]) for r in ROLES):
            continue
        lengths = plannable(static, spawns, beam)
        if lengths is None:
            continue
        verdict = evaluate({'map_id': 'zone_wide_door_tags_v2', 'items': [
            {'item_id': 'beam', 'kind': 'long_beam', 'pose_m': list(beam), 'destination_zone': 'A'}]}).verdict
        if verdict != 'feasible':
            continue
        out[seed] = {'beam': beam, 'start': start, 'plan_length_m': lengths, 'feasibility_202': verdict}
        seed += 1
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
