"""Temporary mutations of the integration code: each must make the named tests fail; always restored.

Run from the worktree root: python experiments/2026-09-26-zone-study-integration/mutation_check.py
Writes mutations.json next to this file. Refuses to run on a dirty harness/scripts tree.
"""
import subprocess, sys, pathlib, json
ROOT = pathlib.Path(__file__).resolve().parents[2]
HERE = pathlib.Path(__file__).resolve().parent
if subprocess.run(['git', 'status', '--porcelain', '--', 'harness', 'scripts'], cwd=ROOT, capture_output=True, text=True).stdout.strip():
    sys.exit('dirty harness/scripts: commit first')
PY = sys.executable
M = [
 ('M1 peer belief leaks into snapshot', 'harness/zone_study_integration.py',
  "snap = {'frame': frame, 'belief': link.belief(),",
  "snap = {'frame': frame, 'belief': self.links['r2'].belief() if actor != 'r2' else link.belief(),",
  'test_peer_private_state_does_not_reach'),
 ('M2 shared board: peer job state in history', 'harness/zone_study_integration.py',
  "'history': [copy.deepcopy(e) for e in self._history[actor] if e['issued_at_sim_s'] <= t + 1e-9],",
  "'history': [copy.deepcopy(e) for a in self.actors for e in self._history[a] if e['issued_at_sim_s'] <= t + 1e-9],",
  'test_peer_private_state_does_not_reach'),
 ('M3 peer-job-end wake', 'harness/zone_study_integration.py',
  "            self.scheduler.trigger(rid, trigger, at=at_s)",
  "            [self.scheduler.trigger(a, trigger, at=at_s) for a in self.actors]",
  'test_peer_private_state_does_not_reach or test_an_executor_event_wakes_only'),
 ('M4 abort keeps host macros', 'scripts/run_zone_study_integration.py',
  "            slot.timeline, slot.capture_after = [], False",
  "            pass",
  'test_host_link_abort_drops'),
 ('M5 action released at call start, not charged time', 'harness/zone_study_integration.py',
  "        self.dispatch_log.append({'call_id': call_id, 'actor': actor, 'sim_s': sim_s,",
  "        self.dispatch_log.append({'call_id': call_id, 'actor': actor, 'sim_s': self.scheduler.calls[-1].started_sim_s,",
  'test_talk_and_think_cost'),
 ('M6 host arbitration: refuse a claim another robot holds', 'harness/zone_study_integration.py',
  "        ack = link.call(plan.api, *plan.args) if plan.api else None",
  "        taken = plan.api == 'deliver' and any((self.links[o].job() or {}).get('order_id') == plan.args[0] for o in self.actors if o != actor)\n        ack = link.call(plan.api, *plan.args) if plan.api and not taken else None\n        plan = Plan(None, rejected_reason='TAKEN') if taken else plan",
  'test_claim_reaches_own_executor'),
 ('M7 pair status only with a channel', 'harness/zone_study_integration.py',
  "                'inter_robot_channels': (['dialogue'] if self.spec.channel_open else []) + ['pair_status']}",
  "                'inter_robot_channels': (['dialogue', 'pair_status'] if self.spec.channel_open else [])}",
  'test_pair_status_channel_is_present'),
]
out = []
for name, path, old, new, tests in M:
    p = ROOT / path
    src = p.read_text()
    assert src.count(old) == 1, (name, src.count(old))
    try:
        p.write_text(src.replace(old, new))
        r = subprocess.run([PY, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', 'tests/test_zone_study_integration.py', '-k', tests],
                           capture_output=True, text=True, cwd=ROOT)
        tail = [l for l in r.stdout.splitlines() if 'passed' in l or 'failed' in l][-1:]
        out.append({'mutation': name, 'tests': tests, 'detected': r.returncode != 0, 'summary': tail})
    finally:
        p.write_text(src)
(HERE / 'mutations.json').write_text(json.dumps({'source_sha': subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True).stdout.strip(), 'mutations': out, 'all_detected': all(m['detected'] for m in out)}, indent=1, ensure_ascii=False) + '\n')
print(json.dumps(out, indent=1, ensure_ascii=False))
r = subprocess.run(['git', 'diff', '--stat', '--', 'harness/zone_study_integration.py', 'scripts/run_zone_study_integration.py'], capture_output=True, text=True, cwd=ROOT)
print('restored diff:', repr(r.stdout))
