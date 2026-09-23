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


@pytest.mark.parametrize('selection', [row['id'] for row in catalog() if row['family'] == 'act'])
def test_act_scene_records_every_map_read_by_generation_and_split_validation(monkeypatch, selection):
    from sim import act_map_suite
    read_paths = set()
    original_read = act_map_suite.read_json

    def tracked_read(path):
        read_paths.add(str(Path(path).resolve()))
        return original_read(path)

    monkeypatch.setattr(act_map_suite, 'read_json', tracked_read)
    scene = resolve(selection)
    assert read_paths <= scene.sources.keys()
    for path in read_paths:
        content = Path(path).read_bytes()
        assert scene.sources[path] == {'bytes': content, 'sha256': hashlib.sha256(content).hexdigest()}


def test_generated_corner_records_its_template_without_legacy_regression(monkeypatch):
    from sim import act_map_suite
    row = next(row for row in json.loads(act_map_suite.SPEC.read_text())['cases'] if row['family'] == 'corner')
    monkeypatch.setattr(act_map_suite, 'load_suite', lambda: (
        {'include_legacy_regression': False}, [act_map_suite.generate_case(row)]))
    scene = resolve('act/'+row['id'])
    assert str(ROOT/'maps/pair_navigation/l-corner.json') in scene.sources
    assert str(ROOT/'maps/pair_navigation/catalog.json') not in scene.sources


def test_act_cli_inspect_exposes_transitive_source_hashes(tmp_path, capsys):
    from sim.act_map_suite import SPEC
    row = next(row for row in json.loads(SPEC.read_text())['cases'] if row['family'] == 'corner')
    config = tmp_path/'corner.json'
    assert main(['init', str(config), '--scene', 'act/'+row['id']]) == 0
    capsys.readouterr()
    assert main(['inspect', str(config)]) == 0
    report = json.loads(capsys.readouterr().out)
    source = ROOT/'maps/pair_navigation/l-corner.json'
    assert report['source_files'][str(source)] == hashlib.sha256(source.read_bytes()).hexdigest()


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


@pytest.mark.parametrize('host', ['Darwin', 'Linux'])
@pytest.mark.parametrize('action', ['run', 'trial'])
def test_existing_study_cli_accepts_local_hosts_and_preserves_runner_contract(tmp_path, monkeypatch, host, action):
    from scripts import run_rgb_communication_study as cli
    monkeypatch.setattr(cli.platform, 'system', lambda: host)
    manifest = tmp_path/'manifest.json'
    manifest.write_text('{"frozen_test_value": 1}')
    calls = []
    def runner(value, **kwargs):
        calls.append((value, kwargs))
        return {'ready': True}
    monkeypatch.setattr(cli, 'run_study', runner)
    monkeypatch.setattr(cli, 'run_trial', runner)
    args = [action, '--manifest', str(manifest), '--evidence-root', str(tmp_path), '--output', str(tmp_path/'result')]
    if action == 'trial': args += ['--run-id', 'test']
    assert cli.main(args) == 0
    assert calls[0][0] == {'frozen_test_value': 1}
    assert calls[0][1]['evidence_root'] == tmp_path
    assert calls[0][1]['output'] == tmp_path/'result'


def test_local_study_cli_does_not_bypass_readiness_failure(tmp_path, monkeypatch):
    from scripts import run_rgb_communication_study as cli
    monkeypatch.setattr(cli.platform, 'system', lambda: 'Darwin')
    manifest = tmp_path/'manifest.json';manifest.write_text('{}')
    def blocked(*args, **kwargs):
        raise cli.ContractError('source-bound evidence not ready')
    monkeypatch.setattr(cli, 'run_study', blocked)
    with pytest.raises(SystemExit) as error:
        cli.main(['run', '--manifest', str(manifest), '--evidence-root', str(tmp_path), '--output', str(tmp_path/'out')])
    assert error.value.code == 2 and not (tmp_path/'out').exists()
