"""Catalog completeness and research-scene boundary regressions (no physics run)."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.sim_cli import main
from sim.research_dispatch_arena import episode, digest
from sim.session_config import validate_config
from sim.session_scenes import ROOT, Scene, catalog, DEFAULT_SCENE


def resolve(selection, **kwargs):
    return Scene(validate_config({'version': 1, 'scene': {'layout': selection, 'seed': 11, **kwargs}})['scene'], ROOT)


def test_catalog_covers_every_existing_map_and_case_without_a_hidden_fallback():
    rows = catalog()
    assert len({r['id'] for r in rows}) == len(rows)
    assert Counter(r['family'] for r in rows) == dict(legacy=4, dispatch=5, navigation=9, pair_navigation=6, act=22, multi_object=12)
    for family in ('navigation', 'pair_navigation'):
        assert {p.stem for p in (ROOT/'maps'/family).glob('*.json') if p.name != 'catalog.json'} == {
            r['id'].split('/')[1] for r in rows if r['family'] == family}
    # Every case resolves through its own source, with recorded source bytes.
    for row in rows:
        scene = resolve(row['id'])
        assert scene.selection == row['id']
        if row['family'] != 'legacy':
            assert scene.sources and scene.bounds
        for path, entry in scene.sources.items():
            assert entry['sha256'] == hashlib.sha256(Path(path).read_bytes()).hexdigest()
    for selection in ('dispatch/typo', 'act/typo', 'multi_object/typo', ['invalid']):
        with pytest.raises(ValueError):
            resolve(selection)


def test_default_cli_preserves_dispatch_and_demo_requires_explicit_selection(tmp_path):
    path = tmp_path/'default.json'
    assert main(['init', str(path)]) == 0
    config = json.loads(path.read_text())
    assert config['scene']['layout'] == DEFAULT_SCENE and config['scene']['seed'] == 11
    directory = tmp_path/'research'
    main(['new', str(directory)])
    config = json.loads((directory/'config.json').read_text())
    assert config['scene']['layout'] == DEFAULT_SCENE
    assert config['controllers']['r1']['factory'].endswith(':create_idle_controller')
    assert Scene(config['scene'], directory).record()['geometry_modified'] is None
    namespace = {}
    exec((directory/'scene.py').read_text(), namespace)
    assert namespace['build_scene'](seed=11, params={}) == []


def test_existing_dispatch_map_obstruction_setup_and_seed_are_preserved():
    blocked = resolve('dispatch/north_blocked')
    expected = episode('north_blocked', 11)
    assert blocked.config == expected
    assert blocked.config['static_map'] == resolve('dispatch/shared_crossing').config['static_map']
    assert blocked.config['setup_only']['unexpected_obstacles']
    assert blocked.inventory == ['beam', 'box']
    assert resolve('navigation/open').inventory == []
    assert len(resolve('multi_object/mixed_eight').inventory) == 8


def test_custom_map_is_relative_to_config_and_camera_changes_fail_closed(tmp_path):
    path = tmp_path/'custom.json'
    data = json.loads((ROOT/'maps/navigation/open.json').read_text())
    path.write_text(json.dumps(data))
    config = validate_config({'version': 1, 'scene': {'layout': 'navigation/file', 'map_file': 'custom.json'}})
    scene = Scene(config['scene'], tmp_path)
    assert scene.map == data and str(path) in scene.sources
    data['top_camera']['fov_y_deg'] = 60
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='camera'):
        Scene(config['scene'], tmp_path)
    for patch in ({'layout': 'navigation/file'}, {'layout': 'standard', 'map_file': 'bad.json'},
                  {'layout': DEFAULT_SCENE, 'cargo_ids': ['wrong']},
                  {'layout': 'multi_object/single_box', 'contact_profile': 'local_contact_fine'}):
        with pytest.raises(ValueError):
            validate_config({'version': 1, 'scene': patch})


def test_research_catalog_inspection_needs_no_physics_or_model_dependencies():
    program = '''
import sys
from sim.session_scenes import catalog, Scene
from sim.session_config import validate_config
for row in catalog():
    Scene(validate_config({'version':1,'scene':{'layout':row['id']}})['scene'], '.')
assert not ({'mujoco', 'torch'} & sys.modules.keys())
'''
    subprocess.run([sys.executable, '-c', program], cwd=ROOT, check=True)


def test_workflow_entries_resolve_to_existing_source_and_documentation():
    rows = json.loads((ROOT/'configs/simulation_workflows.json').read_text())['workflows']
    assert len({r['id'] for r in rows}) == len(rows)
    for row in rows:
        assert (ROOT/row['entry']).is_file() and (ROOT/row['docs']).is_file()
