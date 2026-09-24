import json

import pytest

from harness.dispatch_plan import build_dispatch_request, validate_dispatch_plan
from harness.dispatch_plan_guidance import (LEGACY_PASSAGES, PRIORS_PATH,
                                            PlanGuidance, load_priors, preview_plan)
from harness.three_robot_plan import digest
from sim.research_dispatch_arena import authored_map

JPEG = b'\xff\xd8\xff\xd9'


def task(obj, parts, route, after):
    return {'id': obj + '_job', 'object': obj, 'participants': parts, 'route': route, 'after': after}


BOX_FIRST = validate_dispatch_plan({'dock': 'dock_b', 'tasks': [
    task('box', ['r2'], 'south', []), task('beam', ['r1', 'r3'], 'north', ['box_job'])]})
INDEPENDENT = validate_dispatch_plan({'dock': 'dock_a', 'tasks': [
    task('beam', ['r1', 'r3'], 'north', []), task('box', ['r2'], 'south', [])]})
BEAM_FIRST = validate_dispatch_plan({'dock': 'dock_b', 'tasks': [
    task('beam', ['r1', 'r3'], 'north', []), task('box', ['r2'], 'south', ['beam_job'])]})


def agreement(plan=None):
    proposal = None if plan is None else {'proposal_id': 'run-plan-v1', 'version': 1,
                                          'plan_hash': digest(plan), 'plan': plan, 'proposer': 'r1'}
    return {'run_id': 'run', 'version': 1, 'proposer': 'r1', 'proposal': proposal}


def request(guidance=None, plan=None, rid='r2'):
    return build_dispatch_request(rid, task={'mission_id': 'm'}, request_id='req', own_rgb=JPEG,
                                  top_rgb=JPEG, agreement=agreement(plan), execution_pilot=True,
                                  guidance=guidance)


def system(req):
    return req['messages'][0]['content']


def context(req):
    return json.loads(req['messages'][1]['content'])


def test_legacy_guidance_is_byte_identical_to_original_request():
    for plan in (None, BOX_FIRST):
        base = request(plan=plan)
        legacy = request(PlanGuidance('legacy', authored_map('open')), plan=plan)
        assert legacy == base
    assert 'beam.after=[box_job]' in PlanGuidance('legacy', authored_map('open')).capability_scope


def test_objective_states_goal_neutral_consequences_and_open_map_passages():
    guidance = PlanGuidance('objective', authored_map('open'), auto_route_overlap=True)
    text = system(request(guidance, plan=BOX_FIRST))
    assert 'TEAM OBJECTIVE' in text and 'least total SIM time' in text
    assert 'after=[other job]' in text and 'run concurrently on this open map' in text
    assert LEGACY_PASSAGES not in text and 'no central island' in text
    assert 'beam.after=[box_job]' not in guidance.capability_scope
    assert 'proposal_schedule_preview' not in context(request(guidance, plan=BOX_FIRST))


def test_objective_keeps_island_description_on_cluttered_map():
    text = system(request(PlanGuidance('objective', authored_map('shared_crossing')), plan=None))
    assert LEGACY_PASSAGES in text
    assert 'this map runs them one at a time' in text


def test_preview_serial_box_first_shows_idle_pair_and_bay_clearing():
    priors, _ = load_priors()
    view = preview_plan(BOX_FIRST, authored_map('open'), priors, auto_route_overlap=True)
    assert view['execution_mode'] == 'one_job_at_a_time'
    assert view['execution_mode_reason'] == 'agreed_dependencies_require_serial'
    assert view['bay_clearing_by_box'] is True
    assert view['robots']['r1']['idle_before']['before_grasp'] > 60
    assert view['robots']['r2']['estimated_idle_sim_s'] == 0
    assert 180 < view['estimated_total_sim_s'] < 240


def test_preview_independent_routes_run_concurrently_beam_first():
    priors, _ = load_priors()
    view = preview_plan(INDEPENDENT, authored_map('open'), priors, auto_route_overlap=True)
    assert view['execution_mode'] == 'concurrent'
    assert view['robots']['r1']['estimated_idle_sim_s'] == 0
    serial = preview_plan(BOX_FIRST, authored_map('open'), priors, auto_route_overlap=True)
    beam_first = preview_plan(BEAM_FIRST, authored_map('open'), priors, auto_route_overlap=True)
    assert view['estimated_total_sim_s'] < beam_first['estimated_total_sim_s'] < serial['estimated_total_sim_s']


def test_preview_follows_run_gate_settings():
    priors, _ = load_priors()
    serial_runner = preview_plan(INDEPENDENT, authored_map('open'), priors, auto_route_overlap=False)
    assert serial_runner['execution_mode'] == 'one_job_at_a_time'
    cluttered = preview_plan(INDEPENDENT, authored_map('shared_crossing'), priors, auto_route_overlap=True)
    assert cluttered['execution_mode_reason'] == 'service_island_requires_serial'
    assert cluttered['estimated_total_sim_s'] is not None
    guidance = PlanGuidance('objective_preview', authored_map('open'), route_overlap=True)
    assert guidance.preview(BOX_FIRST)['execution_mode'] == 'not_executable_with_run_settings'


def test_preview_is_attached_only_to_frozen_proposals_and_logged():
    guidance = PlanGuidance('objective_preview', authored_map('open'), auto_route_overlap=True)
    assert 'proposal_schedule_preview' not in context(request(guidance, plan=None))
    ctx = context(request(guidance, plan=BOX_FIRST))
    assert ctx['proposal_schedule_preview']['plan_hash'] == digest(BOX_FIRST)
    assert 'ROUGH PRIOR' in ctx['proposal_schedule_preview']['scope']
    assert "host never changes or rejects a plan for time" in system(request(guidance, plan=BOX_FIRST))
    # After that proposal is rejected, the next proposer still sees the earlier estimate.
    later = context(request(guidance, plan=None, rid='r3'))
    assert later['earlier_proposal_previews'][0]['plan_hash'] == digest(BOX_FIRST)
    assert len(guidance.preview_log) == 2  # one entry per request that showed it
    record = guidance.record()
    assert record['mode'] == 'objective_preview' and record['priors_sha256']
    assert record['route_settings']['auto_route_overlap'] is True


def test_preview_reads_no_live_state():
    priors, _ = load_priors()
    view = preview_plan(INDEPENDENT, authored_map('open'), priors, auto_route_overlap=True)
    flat = json.dumps(view)
    for forbidden in ('qpos', 'xy_m', 'joint', 'contact', 'referee', 'physical_success'):
        assert forbidden not in flat
    assert set(view['duration_basis']) >= {'beam.approach', 'box.carry'}


def test_priors_record_sources_and_limits():
    priors, sha = load_priors()
    assert PRIORS_PATH.name == 'dispatch_stage_priors.json' and len(sha) == 64
    assert len(priors['sources']) == 2 and priors['limits']
    for source in priors['sources']:
        assert set(source['sha256']) == {'issued-commands.json', 'solo-decisions.json', 'result.json'}


def test_unknown_mode_rejected():
    with pytest.raises(ValueError):
        PlanGuidance('fast', authored_map('open'))


def test_cli_accepts_guidance_only_for_skills(tmp_path):
    from scripts.run_dispatch_e2e import main
    with pytest.raises(SystemExit):
        main(['--output', str(tmp_path / 'x'), '--executor', 'raw', '--plan-guidance', 'objective'])
    with pytest.raises(SystemExit):
        main(['--output', str(tmp_path / 'y'), '--plan-guidance', 'fastest'])
