import json
import pytest
from scripts.cloud_collection import digest
from scripts.kaggle_gpu_recovery import validate_gpu_job


def test_private_gpu_job_rejects_changed_identity_and_public_inputs(tmp_path):
    kernel=tmp_path/'kernel';kernel.mkdir();(kernel/'run.py').write_text("source='abc'\n")
    state={'kernel':'owner/job','dataset':'owner/source','asset_dataset':'owner/assets','source_sha':'abc',
        'driver_sha256':digest(kernel/'run.py'),'timeout_seconds':14400}
    meta={'id':'owner/job','is_private':True,'enable_gpu':True,'enable_internet':True,'enable_tpu':False,
        'dataset_sources':['owner/source','owner/assets']}
    (tmp_path/'job.json').write_text(json.dumps(state))
    def save():(kernel/'kernel-metadata.json').write_text(json.dumps(meta))
    save();assert validate_gpu_job(tmp_path)==state
    meta['is_private']=False;save()
    with pytest.raises(ValueError):validate_gpu_job(tmp_path)
    meta['is_private']=True;meta['dataset_sources'].append('someone/extra');save()
    with pytest.raises(ValueError):validate_gpu_job(tmp_path)
    meta['dataset_sources'].pop();save();(kernel/'run.py').write_text('changed')
    with pytest.raises(ValueError):validate_gpu_job(tmp_path)
