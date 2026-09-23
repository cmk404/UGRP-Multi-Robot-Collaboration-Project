"""Validate the explicitly requested private GPU recovery job before submission.

Default CPU/offline jobs retain their separate stricter validator.
"""
import json
from pathlib import Path
from scripts.colab_simulation_cli import digest


def validate_gpu_job(output):
    output=Path(output)
    state=json.loads((output/'job.json').read_text())
    meta=json.loads((output/'kernel/kernel-metadata.json').read_text())
    if (meta.get('is_private') is not True or meta.get('enable_gpu') is not True
            or meta.get('enable_internet') is not True or meta.get('enable_tpu') is not False):
        raise ValueError('explicit private GPU/internet recovery configuration required')
    if meta['id']!=state['kernel'] or meta['dataset_sources']!=[state['dataset'],state['asset_dataset']]:
        raise ValueError('GPU job identity or inputs changed')
    if digest(output/'kernel/run.py')!=state['driver_sha256']:
        raise ValueError('GPU driver changed')
    if state['source_sha'] not in (output/'kernel/run.py').read_text():
        raise ValueError('source provenance missing from driver')
    if state.get('timeout_seconds')!=14400:
        raise ValueError('recovery must have a four-hour job limit')
    return state
