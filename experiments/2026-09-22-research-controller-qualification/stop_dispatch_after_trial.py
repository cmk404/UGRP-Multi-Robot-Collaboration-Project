"""One finite, recorded handoff of the already-owned v3 B dispatcher.

The physical child must have exited and its row must be saved before signalling.
No source, protocol, result or physical child is changed by this watcher.
"""
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time

PID = 67845
ROOT = Path('/Users/changmin/.codex/worktrees/research-pixel-validation/ugrp')
REPORT = ROOT/'outputs/research-repair/acceptance-v3-b/report.json'
RECORD = ROOT/'outputs/research-repair/dispatch-handoff-v3.json'
FINISHED = 'ACT-RGB-refined--qualification-3--r0'
TRANSFERRED = 'ACT-guarded--qualification-3--r0'


def identity():
    text = subprocess.check_output(['ps', '-p', str(PID), '-o', 'lstart=,command='], text=True).strip()
    if 'scripts/run_matched_carry_cohort.py' not in text or 'acceptance-v3-b-protocol.json' not in text:
        raise RuntimeError('owned dispatcher identity mismatch')
    return text


expected = identity()
deadline = time.monotonic()+3600
last_stamp = None
print(json.dumps({'state': 'watching_saved_trial_boundary', 'pid': PID, 'after': FINISHED}), flush=True)
while time.monotonic() < deadline:
    stamp = REPORT.stat().st_mtime_ns
    if stamp != last_stamp:
        last_stamp = stamp
        report = json.loads(REPORT.read_text())
        ids = [row['trial_id'] for row in report['runs']]
        if TRANSFERRED in ids:
            raise RuntimeError('handoff boundary missed; preserve duplicate attempt for explicit review')
        if FINISHED in ids:
            if len(ids) != 4 or identity() != expected:
                raise RuntimeError('dispatcher changed before handoff')
            row = next(r for r in report['runs'] if r['trial_id'] == FINISHED)
            if row['outcome']['failure_kind'] == 'interrupted':
                raise RuntimeError('physical trial was interrupted; not a completed handoff')
            os.kill(PID, signal.SIGINT)
            with RECORD.open('x') as f:
                json.dump({'at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                           'pid': PID, 'identity': expected, 'signal': 'SIGINT',
                           'completed_trial_ids': ids, 'transferred_trial_id': TRANSFERRED,
                           'scope': 'Dispatcher stop after physical child exit and saved outcome; original report retained.'}, f, indent=2)
                f.write('\n')
            print(json.dumps({'state': 'dispatcher_handed_off', 'pid': PID}), flush=True)
            break
    time.sleep(.05)
else:
    raise TimeoutError('finite handoff deadline exceeded; no process signalled')
