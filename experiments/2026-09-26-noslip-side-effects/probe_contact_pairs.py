#!/usr/bin/env python3
"""보조 진단: 감사 결과를 해석하기 위한 접촉 쌍·마찰 계수 목록 (평가 전용).

`wall_push`와 `drive_commands`에서 두 프로필의 차이가 거의 0인 이유를 확인한다.
noslip 후처리가 바꿀 수 있는 접선력은 마찰 원뿔 한계(mu x 법선력)로 묶여 있으므로,
실제로 벽·바닥을 짚는 geom의 mu가 얼마인지가 부작용 크기를 정한다.

물리 1회 실행(약 12 SIM s, 프로필 1개)만 하고 결과를 표준 출력으로 낸다.

  PYTHONPATH=. .venv-sim/bin/python experiments/2026-09-26-noslip-side-effects/probe_contact_pairs.py
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main():
    import numpy as np
    import mujoco
    from scripts.audit_contact_profiles import GOAL, SCENARIOS, VARIANT, BASE_Z
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.zone_cargo_scene import CargoZoneScene

    scene = CargoZoneScene.from_cargo_config(VARIANT, 11, cargo=[], goal=GOAL,
                                             contact_profile='local_contact_fine')
    world = MultiMasterPiProductionV2(seed=11, width=64, height=48, render=False,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    scene.setup(world)
    m, d = world.model, world.data
    x, y, a = SCENARIOS['wall_push']['start']['r1']
    world.robot('r1').set_base_pose_for_test((x, y, BASE_Z), a)
    mujoco.mj_forward(m, d)
    port = CameraRobotPort(world, 'r1', allow_reverse=True, allow_mecanum=True)
    name = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or ''
    pairs = defaultdict(lambda: {'samples': 0, 'max_normal_n': 0., 'mu': None})
    f6 = np.zeros(6)
    for phase, action, seconds in (('press', {'kind': 'mecanum', 'forward': .10, 'left': 0., 'turn': 0.,
                                              'duration_s': 1.}, 8.),
                                   ('slide', {'kind': 'mecanum', 'forward': .10, 'left': .08, 'turn': 0.,
                                              'duration_s': 1.}, 4.)):
        end = float(d.time) + seconds
        while float(d.time) < end:
            now = float(d.time)
            if now % 1. < float(m.opt.timestep):
                port.apply(action, now)
            port.tick(now)
            world._physics_step_for(world.controllers['r1'])
            for i in range(d.ncon):
                c = d.contact[i]
                key = (phase, name(c.geom1), name(c.geom2))
                mujoco.mj_contactForce(m, d, i, f6)
                row = pairs[key]
                row['samples'] += 1
                row['max_normal_n'] = max(row['max_normal_n'], abs(float(f6[0])))
                row['mu'] = round(float(c.friction[0]), 6)
    out = []
    for (phase, g1, g2), row in sorted(pairs.items(), key=lambda kv: -kv[1]['samples']):
        if 'wall' in g1 or 'wall' in g2 or 'floor' in (g1, g2):
            out.append({'phase': phase, 'geom1': g1, 'geom2': g2, 'contact_mu': row['mu'],
                        'samples': row['samples'], 'max_normal_n': round(row['max_normal_n'], 4)})
    print(json.dumps({'note': 'evaluation-only contact inventory, local_contact_fine, weld OFF',
                      'sim_time_s': round(float(d.time), 3), 'pairs': out}, indent=1))


if __name__ == '__main__':
    main()
