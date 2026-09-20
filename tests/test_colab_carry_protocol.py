"""Exercise protocol selection and completion without allocating a GPU."""
import json
from pathlib import Path
import sys

import pytest
from scripts import run_colab_carry_training as runner
from scripts.colab_carry_bundle import digest


@pytest.mark.parametrize('memory_saving', [False, True])
def test_four_model_extension_and_completed_resume(tmp_path, monkeypatch, memory_saving):
    protocol = {'dataset_sha256': 'data', 'training': {'steps': 8000, 'batch': 32, 'seeds': [1, 2]},
                'arms': [{'id': 'r512-h1', 'size': 512, 'history': 1},
                         {'id': 'r512-h4', 'size': 512, 'history': 4}]}
    if memory_saving:
        protocol['execution'] = {'checkpoint_encoder': True, 'cpu_evaluation_batch_size': 8}
    path = tmp_path/'protocol.json'; path.write_text(json.dumps(protocol))
    out = tmp_path/'results'
    monkeypatch.setattr(runner, 'require_cuda', lambda: None)
    monkeypatch.setattr(runner, 'source_identity', lambda: 'fixed-source')
    monkeypatch.setattr(runner, 'verify_dataset', lambda _: {'dataset_sha256': 'data'})
    calls = []

    def train(cmd, **kwargs):
        calls.append(cmd)
        args = {name: cmd[cmd.index(name)+1] for name in ('--out', '--size', '--history', '--seed')}
        directory = Path(args['--out']); (directory/'act').mkdir(parents=True)
        (directory/'act/model.safetensors').write_bytes(b'model')
        report = {'complete': True, 'completed_steps': 8000, 'steps': 8000, 'batch_size': 32,
                  'dataset_sha256': 'data', 'source_sha': 'fixed-source', 'seed': int(args['--seed']),
                  'adapter': {'size': int(args['--size']), 'history': int(args['--history'])},
                  'activation_checkpointing': '--checkpoint-encoder' in cmd,
                  'cpu_evaluation_batch_size': int(cmd[cmd.index('--cpu-evaluation-batch-size')+1]) if '--cpu-evaluation-batch-size' in cmd else 32,
                  'model_sha256': digest(directory/'act/model.safetensors')}
        (directory/'report.json').write_text(json.dumps(report))

    monkeypatch.setattr(runner.subprocess, 'run', train)
    argv = ['runner', '--dataset', 'dataset.json', '--out', str(out), '--protocol', str(path)]
    monkeypatch.setattr(sys, 'argv', argv)
    runner.main()
    result = json.loads((out/'cohort.json').read_text())
    assert result['complete'] and result['expected_models'] == 4 and len(result['models']) == 4
    assert result['protocol_sha256'] == digest(path) and len(calls) == 4
    monkeypatch.setattr(sys, 'argv', argv + ['--resume'])
    runner.main()
    assert json.loads((out/'cohort.json').read_text())['complete'] and len(calls) == 4
    report_path = out/'model-r512-h1-s1/report.json'
    report = json.loads(report_path.read_text()); report['completed_steps'] = 7999
    report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match='update count'):
        runner.main()
