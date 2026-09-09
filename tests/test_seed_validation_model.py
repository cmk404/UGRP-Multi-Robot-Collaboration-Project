"""A proxy model choice must survive orchestration and the experiment record."""
import json
import sys
from unittest.mock import patch

from scripts import run_gemini_seed_validation as runner


def test_selected_model_reaches_child_and_manifest(tmp_path):
    output = tmp_path / 'trial'
    selected = 'gemini-3.8-flash-high'
    captured = []

    def cohort(path, runs, jobs, execute):
        for run in runs:
            assert execute(run) == 0
        return {'blocking_reason': None}

    def launch(command, **kwargs):
        captured.append(command)
        return type('Completed', (), {'returncode': 0})()

    with patch.object(sys, 'argv', ['seed-validation', '--execute', '--output', str(output),
                                   '--seeds', '45', '--model', selected]), \
         patch.object(runner, 'policy_hashes', return_value={}), \
         patch.object(runner, 'run_cohort', side_effect=cohort), \
         patch.object(runner.subprocess, 'run', side_effect=launch):
        assert runner.main() == 0
    manifest = json.loads((output / 'validation-manifest.json').read_text())
    assert manifest['model'] == selected
    assert captured[0][captured[0].index('--model') + 1] == selected
    default = runner.command(tmp_path / 'default', 45)
    assert default[default.index('--model') + 1] == 'gemini-3.8-flash'
