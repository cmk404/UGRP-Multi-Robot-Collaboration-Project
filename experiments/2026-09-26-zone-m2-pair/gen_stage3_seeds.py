"""Stage 3 (failure propagation) scenario generator: same draw ranges and filter as stage 1
(``gen_stage1_seeds.feasible``), rng 20260927, seeds 721-724, plus the pre-registered
experimenter drop (robot, seconds after its carry start)."""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from gen_stage1_seeds import ROLES, TaggedZoneScene, feasible  # noqa: E402

DROPS = {721: 'r1:3.0', 722: 'r2:3.0', 723: 'r1:6.0', 724: 'r2:6.0'}


def main():
    scene = TaggedZoneScene.from_tagged('zone_wide_door_tags_v2', 11, {'A': {'cyan': 1}}, None)
    static, spawns = scene.config['static_map'], scene.config['setup_only']['spawns']
    rng = np.random.default_rng(20260927)
    out, seed = {}, 721
    while seed <= 724:
        beam = (round(rng.uniform(.35, .85), 2), round(rng.uniform(-1.9, -.4), 2), round(rng.uniform(-.6, .6), 2))
        start = {r: (round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.1, .1), 2), round(rng.uniform(-.4, .4), 2))
                 for r in ROLES}
        lengths = feasible(static, spawns, beam)
        if lengths is None:
            continue
        out[seed] = {'beam': beam, 'start': start, 'plan_length_m': lengths, 'drop': DROPS[seed]}
        seed += 1
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
