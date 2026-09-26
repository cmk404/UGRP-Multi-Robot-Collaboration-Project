"""Stage 2b (door v2) test scenario generator: same ranges and filters as gen_stage2_seeds.py,
rng 20260929, seeds 821-826 (fresh; 811-816 became development seeds for door v2)."""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from gen_stage2_seeds import ROLES, TaggedZoneScene, evaluate, plannable, start_clear  # noqa: E402


def main():
    scene = TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 11, {'A': {'cyan': 1}}, None)
    static, spawns = scene.config['static_map'], scene.config['setup_only']['spawns']
    rng = np.random.default_rng(20260929)
    out, seed = {}, 821
    while seed <= 826:
        beam = (round(rng.uniform(.85, 1.10), 2), round(.05 + rng.uniform(-.10, .10), 2), round(rng.uniform(-.10, .10), 2))
        start = {r: (round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.4, .4), 2))
                 for r in ROLES}
        # door v2 swaps the depot slots (order-sheet rule), so check the start clearance at both slots
        if not all(start_clear(spawns[slot], start[r]) for r in ROLES for slot in ROLES):
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
