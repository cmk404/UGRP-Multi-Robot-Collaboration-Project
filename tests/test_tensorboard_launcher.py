import json
import os

from scripts.run_tensorboard import resolve_logdir


def collection(root, name, exported=True, mtime_ns=1):
    path=root/name
    path.mkdir()
    payload={'exported':[{'name':'run'}] if exported else [],'failed':[]}
    marker=path/'collection.json'
    marker.write_text(json.dumps(payload))
    os.utime(marker,ns=(mtime_ns,mtime_ns))
    return path


def test_prefers_newest_export_collection_without_calling_it_experiment_time(tmp_path):
    old=collection(tmp_path,'review-old',mtime_ns=10)
    new=collection(tmp_path,'review-new',mtime_ns=20)
    assert resolve_logdir(tmp_path,True)==new.resolve()
    assert resolve_logdir(tmp_path,False)==tmp_path.resolve()
    assert old.resolve()!=new.resolve()


def test_latest_collection_ignores_empty_or_invalid_snapshots(tmp_path):
    valid=collection(tmp_path,'review-valid',mtime_ns=10)
    collection(tmp_path,'review-empty',exported=False,mtime_ns=30)
    broken=tmp_path/'review-broken';broken.mkdir();(broken/'collection.json').write_text('{')
    assert resolve_logdir(tmp_path,True)==valid.resolve()


def test_exact_collection_remains_exact(tmp_path):
    exact=collection(tmp_path,'review-exact',mtime_ns=10)
    assert resolve_logdir(exact,True)==exact.resolve()
