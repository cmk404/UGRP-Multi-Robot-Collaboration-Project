"""Finite native-scene integration checks. No task success or external model claim."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from scripts.sim_cli import write_json, source_info
from sim.research_dispatch_arena import FIXED_TOP
from sim.session import Simulation
from sim.session_config import validate_config
from sim.session_scenes import catalog


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result = {**source_info(), 'case': 'all-research-scenes', 'scope': 'scene construction/reset/RGB check only',
              'protocol_complete': False, 'model_calls': 0, 'commands': 0, 'cases': []}
    started = time.monotonic()
    try:
        for row in catalog():
            case = {'selection': row['id'], 'ok': False}
            result['cases'].append(case)
            config = validate_config({'version': 1, 'scene': {'layout': row['id'], 'seed': 11}})
            with Simulation(config) as sim:
                world = sim._world
                initial = world.data.qpos.copy()
                model, data = world.model, world.data
                sim.step(10)
                assert not world.data.eq_active.any()
                sim.reset()
                assert world.model is model and world.data is data
                assert np.allclose(initial, world.data.qpos, atol=1e-10), row['id']
                if row['family'] != 'legacy':
                    camera = world.model.camera('cctv_top')
                    assert np.allclose(camera.pos, FIXED_TOP['position_m'])
                    assert np.allclose(camera.quat, FIXED_TOP['quaternion_wxyz'])
                    assert float(camera.fovy[0]) == FIXED_TOP['fov_y_deg']
                    assert sim.scene.bounds == (sim.scene.config['static_map'] if sim.scene.config else sim.scene.map)['bounds_m']
                case.update(ok=True, inventory=sim.scene.inventory, ngeom=world.model.ngeom,
                            scene=sim.scene.record(), xml_sha256=hashlib.sha256(world.scene_xml.encode()).hexdigest())
            print('OK', row['id'], flush=True)
        # Exercise editable geometry on the real research scene, including reset.
        config = validate_config({'version': 1, 'scene': {'layout': 'dispatch/shared_crossing', 'seed': 11,
            'objects': [{'name': 'extra', 'shape': 'sphere', 'size_m': [.04], 'xyz_m': [1., -2., .2], 'dynamic': True}]}})
        with Simulation(config, render=True) as sim:
            initial = sim._world.data.qpos.copy()
            assert sim._world.model.body('user__extra').id >= 0
            sim.apply('r1', {'kind':'drive', 'forward':.08, 'turn':0, 'duration_s':.1})
            sim.step(round(.2 / sim.timestep))
            for rid in ('r1','r2','r3'):
                obs = sim.observe(rid)
                assert obs['image'] and obs['top_rgb']['image']
                assert not ({'setup_only','static_map','scene','contacts','success','robots_xyz_m'} & obs.keys())
            assert sim.observe('r1')['actuator_state']['motor_commands'] == [0.] * 4
            sim.reset()
            assert np.allclose(initial, sim._world.data.qpos, atol=1e-10)
            result['commands'] += 1
        result.update(protocol_complete=True, checked_scenes=len(result['cases']), reset_same_model=True,
                      modified_research_scene=True, rgb_boundary=True, weld_off=True)
        return 0
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        result['wall_s'] = time.monotonic()-started
        write_json(args.output/'result.json', result)


if __name__ == '__main__':
    raise SystemExit(main())
