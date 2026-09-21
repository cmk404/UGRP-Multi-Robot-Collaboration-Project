import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from scripts.setup_kaggle_egl import configure


def test_selects_existing_vendor_and_checks_actual_renderer(tmp_path,monkeypatch):
    libs=tmp_path/'libs';libs.mkdir();(libs/'libEGL_nvidia.so.0').write_bytes(b'library')
    calls=[]
    def run(args,**kwargs):
        calls.append((args,kwargs))
        return SimpleNamespace(returncode=0,stdout='Tesla T4, 550' if args[0]=='nvidia-smi' else json.dumps({'vendor':'NVIDIA Corporation','renderer':'T4'}),stderr='')
    monkeypatch.setattr('scripts.setup_kaggle_egl.subprocess.run',run)
    gpu,env=configure('/python',tmp_path/'out',search_roots=[libs])
    vendor=Path(env['__EGL_VENDOR_LIBRARY_FILENAMES'])
    assert json.loads(vendor.read_text())['ICD']['library_path']==str(libs/'libEGL_nvidia.so.0')
    assert calls[-1][1]['env']['__EGL_VENDOR_LIBRARY_FILENAMES']==str(vendor)
    assert gpu['renderer']=='T4'


def test_software_renderer_is_preserved_as_failure_evidence(tmp_path,monkeypatch):
    def run(args,**kwargs):
        return SimpleNamespace(returncode=0,stdout='T4' if args[0]=='nvidia-smi' else json.dumps({'vendor':'Mesa','renderer':'llvmpipe'}),stderr='')
    monkeypatch.setattr('scripts.setup_kaggle_egl.subprocess.run',run)
    with pytest.raises(RuntimeError):configure('/python',tmp_path/'out',search_roots=[tmp_path/'absent'])
    assert 'llvmpipe' in (tmp_path/'out/report.json').read_text()
