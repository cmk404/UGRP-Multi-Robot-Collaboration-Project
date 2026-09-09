import json

from scripts.evaluate_gemini_cohort import run_cohort, infrastructure_problem


def test_seed_runner_passes_timeout_and_reasoning_to_child_without_changing_budgets(tmp_path):
    from scripts.run_gemini_seed_validation import command
    args = command(tmp_path, 43, 'medium', 60)
    for key, value in {'--request-timeout': '60', '--reasoning-effort': 'medium',
                       '--max-calls': '30', '--max-input-tokens': '120000',
                       '--seconds': '300', '--robots': '1'}.items():
        assert args[args.index(key) + 1] == value


def test_budget_ending_during_server_cooldown_remains_infrastructure_block(tmp_path):
    (tmp_path / 'result.json').write_text(json.dumps({
        'inference_unresolved': {'r1': 1},
        'outcomes': {'r1': {'reason': 'TIME_BUDGET', 'success': False}}}))
    assert infrastructure_problem(tmp_path, 0) == 'INFERENCE_UNAVAILABLE'


def test_successful_process_with_provider_failure_stops_remaining_trials(tmp_path):
    runs = [{'path': str(i)} for i in range(3)]
    attempted = []
    def execute(run):
        attempted.append(run['path'])
        path = tmp_path / run['path']
        path.mkdir()
        (path / 'result.json').write_text(json.dumps({'outcomes': {
            'r1': {'success': False, 'reason': 'LLM_ERROR:http'}}}))
        return 0
    state = run_cohort(tmp_path, runs, jobs=1, execute=execute)
    assert attempted == ['0']
    assert state['status'] == 'infrastructure_blocked'
    assert state['not_started'] == runs[1:]


def test_physical_task_failure_stays_in_denominator_and_does_not_abort(tmp_path):
    runs = [{'path': str(i)} for i in range(3)]
    def execute(run):
        path = tmp_path / run['path']
        path.mkdir()
        (path / 'result.json').write_text(json.dumps({'outcomes': {
            'r1': {'success': False, 'reason': 'VISUAL_ATTACHMENT_UNCONFIRMED'}}}))
        return 0
    state = run_cohort(tmp_path, runs, jobs=1, execute=execute)
    assert state['status'] == 'finished'
    assert len(state['attempted']) == 3
    assert not state['not_started']
