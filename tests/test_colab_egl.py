import json
from pathlib import Path
import pytest
from scripts import setup_colab_egl as egl


def test_registration_preserves_foreign_configuration(tmp_path,monkeypatch):
    library=tmp_path/'lib';library.mkdir();(library/'libEGL_nvidia.so.0').touch()
    vendor=tmp_path/'share/glvnd/egl_vendor.d/10_nvidia.json'
    vendor.parent.mkdir(parents=True);vendor.write_text('{"foreign":true}')
    monkeypatch.setattr(egl.subprocess,'run',lambda *a,**kw:pytest.fail('must not mutate loader'))
    with pytest.raises(ValueError,match='differs'):
        egl.register(library,tmp_path/'etc',tmp_path/'share')
    assert not (tmp_path/'etc').exists()
    assert json.loads(vendor.read_text())=={'foreign':True}


def test_probe_rejects_software_renderer(monkeypatch):
    from types import SimpleNamespace
    def fake(args,**kwargs):
        assert kwargs['env']['__EGL_VENDOR_LIBRARY_FILENAMES']=='/vendor.json'
        return SimpleNamespace(stdout=json.dumps({'vendor':'Mesa','renderer':'llvmpipe'}))
    monkeypatch.setattr(egl.subprocess,'run',fake)
    with pytest.raises(RuntimeError,match='actual OpenGL renderer'):
        egl.verify(Path('/python'),Path('/vendor.json'))
