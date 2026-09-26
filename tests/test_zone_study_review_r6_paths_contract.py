"""Regressions of the sixth review round of PR 194: logdir paths and contract v2.

Offline only: no simulator, no model call, no network. The SIM accounting
regressions of the same round are in ``tests/test_zone_study_review_r6.py``;
the records are in ``experiments/2026-09-26-zone-study-offline-smoke/review-r6/``.

* P2 (Codex ``codex-194-r6``) — the TensorBoard run-path check was lexical: a
  symlinked directory inside the logdir let a new run be written outside it.
* Integration (#229) — the contract rejected the tags_v2 map (three placement
  keys). Contract v2 declares them; v1 stays available for the frozen v1-v4
  records.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import pytest

from harness import zone_study_contract as c
from harness import zone_study_offline as off
from harness.zone_study_scenarios import bundle_for, load as load_scenario
from scripts import zone_study_report as report

ROOT = Path(__file__).resolve().parents[1]
SEED = 601
V4_CONFIG = ROOT / 'experiments' / '2026-09-26-zone-study-offline-smoke' / 'v4' / 'config.json'

# =========================================================================== #
# P2 — a symlink inside the logdir cannot carry a run outside it

def _payload(*runs):
    return {'runs': [{'run': run, 'step': 0, 'hparams': {'condition': 'peer_ko'},
                      'scalars': {'result/model_calls': 1.0}} for run in runs]}


def _files(directory):
    return sorted(str(p.relative_to(directory)) for p in directory.rglob('*'))


def test_r6_p2_codex_counterexample_a_symlinked_parent_does_not_escape(tmp_path):
    outside = tmp_path / 'outside'
    outside.mkdir()
    events = tmp_path / 'events'
    events.mkdir()
    (events / 'peer_ko').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match='symbolic link'):
        report.write_events(_payload('peer_ko/new-trial'), events, at=0.0)
    assert _files(outside) == []


def test_r6_p2_the_report_refuses_a_symlink_escape_before_writing_any_file(tmp_path):
    outside = tmp_path / 'outside'
    outside.mkdir()
    events = tmp_path / 'events'
    events.mkdir()
    (events / 'peer_ko').symlink_to(outside, target_is_directory=True)
    trials = tmp_path / 'trials'
    trials.mkdir()
    trial, result = off.run_trial('s1_normal_mixed', 'peer_ko', SEED, horizon_s=20.0)
    (trials / 'one.json').write_text(json.dumps(trial.trial_record(result), ensure_ascii=False))
    with pytest.raises(ValueError, match='symbolic link'):
        report.build([trials], tmp_path / 'report', resamples=20, now=0.0, tb_events=events)
    assert not (tmp_path / 'report').exists() and _files(outside) == []


@pytest.mark.parametrize('kind', ['dangling_leaf', 'inside_alias', 'leaf_to_outside'])
def test_r6_p2_every_symlinked_run_component_is_refused(tmp_path, kind):
    events = tmp_path / 'events'
    (events / 'cohort').mkdir(parents=True)
    (events / 'peer_ko').mkdir()
    target = {'dangling_leaf': tmp_path / 'missing', 'inside_alias': events / 'cohort',
              'leaf_to_outside': tmp_path}[kind]
    (events / 'peer_ko' / 'trial').symlink_to(target, target_is_directory=True)
    before = _files(events)
    with pytest.raises(ValueError, match='symbolic link'):
        report.write_events(_payload('peer_ko/trial'), events, at=0.0)
    assert _files(events) == before and not (tmp_path / 'missing').exists()


def test_r6_p2_a_symlinked_logdir_itself_is_written_at_its_real_path(tmp_path):
    real = tmp_path / 'real-logdir'
    real.mkdir()
    link = tmp_path / 'logdir-link'
    link.symlink_to(real, target_is_directory=True)
    paths = report._run_paths(_payload('peer_ko/a', 'cohort/peer_ko'), link)
    assert paths == [real.resolve() / 'peer_ko' / 'a', real.resolve() / 'cohort' / 'peer_ko']


def test_r6_p2_the_written_run_lands_inside_the_real_logdir(tmp_path):
    pytest.importorskip('tensorboard', reason='the offline CI job has no tensorboard; '
                                              'the _run_paths tests are the enforced check')
    real = tmp_path / 'real-logdir'
    real.mkdir()
    link = tmp_path / 'logdir-link'
    link.symlink_to(real, target_is_directory=True)
    report.write_events(_payload('peer_ko/a'), link, at=0.0)
    assert any(p.name.startswith('events.out.tfevents') for p in (real / 'peer_ko' / 'a').iterdir())


@pytest.mark.parametrize('runs, match', [
    (('peer_ko/Trial', 'peer_ko/trial'), 'duplicate run path'),        # one directory on macOS
    (('peer_ko/a', 'peer_ko/a'), 'duplicate run path'),
    (('peer_ko', 'peer_ko/a'), 'contain another run'),                 # would half-write
    (('peer_ko/a/b', 'peer_ko/a'), 'contain another run'),
])
def test_r6_p2_runs_that_share_or_nest_a_real_path_are_refused(tmp_path, runs, match):
    events = tmp_path / 'events'
    events.mkdir()
    with pytest.raises(ValueError, match=match):
        report.write_events(_payload(*runs), events, at=0.0)
    assert _files(events) == []


def test_r6_p2_boundary_a_logdir_that_is_a_file_is_refused_before_any_write(tmp_path):
    events = tmp_path / 'events'
    events.write_text('not a directory')
    with pytest.raises(NotADirectoryError):
        report._run_paths(_payload('peer_ko/a'), events)      # ``build`` calls this first
    assert events.read_text() == 'not a directory'


def test_r6_p2_a_symlink_that_appears_after_the_check_is_refused_at_write_time(tmp_path):
    events = tmp_path / 'events'
    (events / 'peer_ko').mkdir(parents=True)
    (path,) = report._run_paths(_payload('peer_ko/a'), events)
    path.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match='changed after the logdir check'):
        report._still_inside(path, events)


def test_r6_p2_write_events_rechecks_each_run_just_before_creating_it(tmp_path, monkeypatch):
    """A symlink planted between the check and the write (a race) is refused."""
    pytest.importorskip('tensorboard', reason='write_events imports tensorboard after the path check')
    events = tmp_path / 'events'
    (events / 'peer_ko').mkdir(parents=True)
    checked = report._run_paths

    def racing(scalars, directory):
        paths = checked(scalars, directory)
        paths[0].symlink_to(tmp_path / 'elsewhere', target_is_directory=True)
        return paths

    monkeypatch.setattr(report, '_run_paths', racing)
    with pytest.raises(ValueError, match='changed after the logdir check'):
        report.write_events(_payload('peer_ko/a'), events, at=0.0)
    assert not (tmp_path / 'elsewhere').exists()


def test_r6_p2_the_real_path_containment_still_holds_if_a_symlink_goes_unnoticed(tmp_path, monkeypatch):
    """Defence in depth: if the per-component symlink check is fooled (a link
    created between that check and ``resolve``), the resolved path is still
    compared with the real logdir."""
    outside = tmp_path / 'outside'
    outside.mkdir()
    events = tmp_path / 'events'
    events.mkdir()
    (events / 'peer_ko').symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(Path, 'is_symlink', lambda self: False)
    with pytest.raises(ValueError, match='outside the logdir'):
        report._run_paths(_payload('peer_ko/new-trial'), events)


def test_r6_p2_boundary_plain_runs_and_existing_runs_behave_as_in_r5(tmp_path):
    events = tmp_path / 'events'
    (events / 'peer_ko' / 'old').mkdir(parents=True)
    assert report._run_paths(_payload('peer_ko/new'), events) == [events.resolve() / 'peer_ko' / 'new']
    with pytest.raises(FileExistsError, match='peer_ko/old'):
        report._run_paths(_payload('peer_ko/new', 'peer_ko/old'), events)
    with pytest.raises(ValueError, match='run name'):
        report._run_paths(_payload('../outside'), events)


# =========================================================================== #
# Integration #229 — contract v2 accepts the tags_v2 map, v1 is kept

V2_REGISTRY = 'c9bb55567a82eef535e7b296ae69f35d5a08e8372479a56ef14c1ac76f08037c'


def _tags_v2_payload():
    scenario = copy.deepcopy(load_scenario('s1_normal_mixed'))
    scenario['map_id'] = 'zone_wide_door_tags_v2'
    trial = off.OfflineTrial(scenario, condition='peer_ko', seed=SEED, map_bundle=bundle_for(scenario))
    bundled = trial.build_inputs('r1', sim_time_s=0.0, request_id='req_r6')   # validates internally
    return copy.deepcopy(c.thaw_for_json(bundled.payload))


def _rehash(payload):
    static = payload['static_map']
    static['public_map_sha256'] = c.digest(static['public_map'])
    return payload


def test_r6_contract_v2_accepts_the_tags_v2_landmark_placement():
    payload = _tags_v2_payload()
    placement = payload['static_map']['public_map']['landmarks']['placement']
    assert {'near_door_spacing_m', 'near_door_radius_m', 'door_posts'} <= set(placement)
    assert c.CONTRACT_VERSION == 'ugrp.zone_study_contract.v2'
    assert c.payload_violations(payload, seed=SEED) == []
    # the frozen v1 contract still refuses it: v1 is kept, not widened
    problems = c.payload_violations(payload, seed=SEED, contract_version=c.CONTRACT_VERSION_V1)
    assert problems and all('placement' in p for p in problems)


def test_r6_contract_a_tags_v1_payload_is_valid_under_v1_and_v2():
    trial = off.OfflineTrial(load_scenario('s1_normal_mixed'), condition='peer_ko', seed=SEED)
    payload = c.thaw_for_json(trial.build_inputs('r1', sim_time_s=0.0, request_id='req_r6').payload)
    for version in c.CONTRACT_VERSIONS:
        assert c.payload_violations(payload, seed=SEED, contract_version=version) == []


@pytest.mark.parametrize('mutate', [
    lambda p: p['door_posts'].update(live_pose_m=[1., 2.]),
    lambda p: p['door_posts'].update(bracket_depth_m=0.01),          # a neutral name: closure only
    lambda p: p['door_posts'].update(width_m='wide'),
    lambda p: p['door_posts'].update(width_m=True),
    lambda p: p['door_posts'].update(tag_center_heights_m=None),
    lambda p: p['door_posts'].update(tag_center_heights_m='0.15'),
    lambda p: p['door_posts'].update(tag_center_heights_m=[0.15, {'x': 1}]),
    lambda p: p.update(door_posts=[0.05]),
    lambda p: p.update(door_posts=None),
    lambda p: p.update(near_door_radius_m='1 m'),
    lambda p: p.update(near_door_radius_m=None),
    lambda p: p.update(near_door_spacing_m=False),
    lambda p: p.update(survey_xy_m=[0.1, 0.2]),
], ids=['extra_live_key', 'extra_neutral_key', 'str_width', 'bool_width', 'none_heights', 'str_heights', 'object_height',
        'list_posts', 'none_posts', 'str_radius', 'none_radius', 'bool_spacing', 'renamed_live_key'])
def test_r6_contract_v2_keeps_the_placement_closed_and_typed(mutate):
    """Unlike a plain mutation, the projection hash is RECOMPUTED, so only the
    placement check itself can refuse it."""
    payload = _tags_v2_payload()
    mutate(payload['static_map']['public_map']['landmarks']['placement'])
    problems = c.payload_violations(_rehash(payload), seed=SEED)
    assert problems and all('landmarks.placement' in p for p in problems)


@pytest.mark.parametrize('value', [math.nan, math.inf])
def test_r6_contract_v2_boundary_non_finite_geometry_is_refused(value):
    """No re-hash here: a non-finite number cannot even be digested."""
    payload = _tags_v2_payload()
    payload['static_map']['public_map']['landmarks']['placement']['door_posts']['height_m'] = value
    problems = c.payload_violations(payload, seed=SEED)
    assert any('JSON serialisable' in p for p in problems)


@pytest.mark.parametrize('mutate', [
    lambda p: p.update(near_door_spacing_m=0),                        # 0 stays 0, never a default
    lambda p: p['door_posts'].update(tag_center_heights_m=[]),
    lambda p: p.update(door_posts={}),
    lambda p: p.pop('door_posts'),
], ids=['zero', 'empty_heights', 'empty_posts', 'no_posts'])
def test_r6_contract_v2_boundary_valid_edge_values_are_accepted(mutate):
    payload = _tags_v2_payload()
    mutate(payload['static_map']['public_map']['landmarks']['placement'])
    assert c.payload_violations(_rehash(payload), seed=SEED) == []


def test_r6_contract_v1_registry_hash_of_the_frozen_records_is_reproduced():
    recorded = json.loads(V4_CONFIG.read_text())['condition_registry_sha256']
    assert c.registry_sha256(c.CONTRACT_VERSION_V1) == recorded
    # v2 changes only the version string; the same hash the integration branch computes
    assert c.registry_sha256() == c.registry_sha256(c.CONTRACT_VERSION) == V2_REGISTRY != recorded
    assert c.condition_manifest('peer_ko', contract_version=c.CONTRACT_VERSION_V1)['contract_version'] \
        == c.CONTRACT_VERSION_V1


def test_r6_contract_boundary_manifest_declares_the_versioned_placement():
    v2 = c.boundary_manifest()['closed_sub_schemas']
    v1 = c.boundary_manifest(c.CONTRACT_VERSION_V1)['closed_sub_schemas']
    assert v2['static_map.public_map.landmarks.placement'] == list(c.PLACEMENT_KEYS)
    assert v2['static_map.public_map.landmarks.placement.door_posts'] == list(c.DOOR_POST_KEYS)
    assert v1['static_map.public_map.landmarks.placement'] == list(c.PLACEMENT_KEYS_V1)
    assert 'static_map.public_map.landmarks.placement.door_posts' not in v1
    assert set(c.PLACEMENT_KEYS) - set(c.PLACEMENT_KEYS_V1) == {'near_door_spacing_m', 'near_door_radius_m',
                                                                'door_posts'}


@pytest.mark.parametrize('version', ['ugrp.zone_study_contract.v3', '', None, 2, 'v1'])
def test_r6_contract_boundary_an_unknown_version_is_refused(version):
    with pytest.raises(ValueError, match='unknown contract version'):
        c.registry_sha256(version)
    with pytest.raises(ValueError, match='unknown contract version'):
        c.condition_manifest('peer_ko', contract_version=version)
    with pytest.raises(ValueError, match='unknown contract version'):
        c.boundary_manifest(version)
    with pytest.raises(ValueError, match='unknown contract version'):
        c.payload_violations({}, contract_version=version)
