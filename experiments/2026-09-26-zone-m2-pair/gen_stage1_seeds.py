"""Stage 1 test scenario generator (run once before the cohort; output pasted into SCENARIOS).

Fixed RNG (numpy default_rng(20260926)); draws beam pose x U[0.35, 0.85], y U[-1.9, -0.4],
yaw U[-0.6, 0.6] and per-robot start offsets dx, dy U[-0.1, 0.1], dyaw U[-0.4, 0.4]; keeps a draw only if
both pre-stations are plannable from the spawns (same keep-outs as the runner) and the carry end
keeps r2's body >= 0.30 m from the divider (x 2.17) and the beam y <= -0.2 m.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from harness import pair_owncam_approach as pa  # noqa: E402
from harness.map_goto import plan_path  # noqa: E402
from scripts.run_m2_pair import KEEPOUT_PAD_M, PARTNER_KEEPOUT_HALF_M, ROLES  # noqa: E402
from sim.zone_cargo import instances, world_grasps  # noqa: E402
from sim.zone_landmarks import TaggedZoneScene  # noqa: E402


def feasible(static, spawns, beam):
    inst = instances([{'item_id': 'beam', 'kind': 'long_beam', 'pose': list(beam)}])[0]
    sheet = pa.coarse_order_sheet(beam)
    g = world_grasps(inst, pose=sheet['beam_xyyaw'])
    st = {r: g[role]['base_xyyaw'] for r, role in ROLES.items()}
    pre = {r: pa.prestation(st[r], .30) for r in ROLES}
    lengths = {}
    for r in ROLES:
        p = next(q for q in ROLES if q != r)
        ko = [pa.beam_keepout(sheet['beam_xyyaw'], .6, .04, KEEPOUT_PAD_M)] + [
            {'id': t, 'center_m': list(xy[:2]), 'half_extents_m': [PARTNER_KEEPOUT_HALF_M] * 2}
            for t, xy in (('s', st[p]), ('p', pre[p]))]
        res = plan_path(static, spawns[r][:2], pre[r][:2], pa.APPROACH_ENVELOPE, obstacles=ko, escape_start_m=.25)
        if res is None:
            return None
        lengths[r] = round(res['length_m'], 2)
    u = (math.cos(beam[2]), math.sin(beam[2]))
    v = (-u[1], u[0])
    shift = (.6 * u[0] + .3 * v[0], .6 * u[1] + .3 * v[1])
    tg = world_grasps(inst)
    for r in ROLES:
        ex = tg[ROLES[r]]['base_xyyaw'][0] + shift[0]
        if ex + .2 > 2.17 - .30:
            return None
    if beam[1] + shift[1] > -.2:
        return None
    return lengths


def main():
    scene = TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 11, {'A': {'cyan': 1}}, None)
    static, spawns = scene.config['static_map'], scene.config['setup_only']['spawns']
    rng = np.random.default_rng(20260926)
    out, seed = {}, 711
    while seed <= 718:
        beam = (round(rng.uniform(.35, .85), 2), round(rng.uniform(-1.9, -.4), 2), round(rng.uniform(-.6, .6), 2))
        start = {r: (round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.4, .4), 2))
                 for r in ROLES}
        lengths = feasible(static, spawns, beam)
        if lengths is None:
            continue
        out[seed] = {'beam': beam, 'start': start, 'plan_length_m': lengths}
        seed += 1
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
