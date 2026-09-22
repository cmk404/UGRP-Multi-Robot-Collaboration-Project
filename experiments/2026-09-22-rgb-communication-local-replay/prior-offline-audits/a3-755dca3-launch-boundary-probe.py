"""Independent fake-child signal delivery at the post-Popen/pre-wait boundary."""
import hashlib
import inspect
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from unittest.mock import patch

SOURCE = Path('/Users/changmin/.codex/worktrees/a3-fresh-audit-755dca3/ugrp')
sys.path.insert(0, str(SOURCE))
from harness import rgb_communication_study as study

created = []
original_popen = subprocess.Popen

def capture_child(*args, **kwargs):
    child = original_popen(*args, **kwargs)
    created.append(child)
    return child

lines, first = inspect.getsourcelines(study.bounded_process)
boundary_line = first + next(i for i, line in enumerate(lines) if line.strip() == 'timed_out = False')
delivered = False

def trace(frame, event, arg):
    global delivered
    if not delivered and frame.f_code is study.bounded_process.__code__ and event == 'line' and frame.f_lineno == boundary_line:
        delivered = True
        os.kill(os.getpid(), signal.SIGTERM)
    return trace

with tempfile.TemporaryDirectory(prefix='a3-fake-child-') as temporary:
    result = None
    error = None
    try:
        with patch.object(study.subprocess, 'Popen', capture_child):
            sys.settrace(trace)
            result = study.bounded_process([sys.executable, '-c', 'import time; time.sleep(30)'],
                cwd=Path(temporary), log_path=Path(temporary)/'fake.log', timeout_s=2, cleanup_grace_s=.2)
    except BaseException as exc:
        error = type(exc).__name__
    finally:
        sys.settrace(None)
        alive_before_audit_cleanup = [child.pid for child in created if child.poll() is None]
        for child in created:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=2)
    report = {'source_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=SOURCE, text=True).strip(),
        'source_file_sha256': hashlib.sha256(Path(study.__file__).read_bytes()).hexdigest(),
        'scope': 'fake child only, no physics/render/model', 'boundary_line': boundary_line,
        'sigterm_delivered': delivered, 'returned_process_record': result, 'raised': error,
        'owned_child_alive_after_bounded_process_exit': bool(alive_before_audit_cleanup),
        'audit_cleanup_reaped_all_owned_fake_children': all(c.returncode is not None for c in created),
        'verdict': 'no_go' if alive_before_audit_cleanup else 'pass'}
    print(json.dumps(report, indent=2))
