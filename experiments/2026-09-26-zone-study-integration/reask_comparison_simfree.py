"""Sim-free call counts per condition, re-ask rule before/after (fake robots whose jobs never end, 445 SIM s).

Run from the worktree root; writes JSON to stdout (saved as reask_comparison_simfree.json). Illustration of the
scheduling rule only: no physics, fixture decisions.
"""
import sys, json
sys.path.insert(0, 'tests'); sys.path.insert(0, '.')
import test_zone_study_integration as T
from harness import zone_study_integration as zi
out = {}
for rule in ('single_pending_own_timer.v1', 'one_timer_per_action (#194 offline)'):
    if rule.startswith('one'):
        orig = zi.IntegratedTrial._arm_reask
        def old(self, actor, sim_s):
            if self.scheduler.metrics[actor]['calls'] >= self.policy.max_calls_per_actor:
                return
            busy = self.links[actor].job() is not None
            at = sim_s + (self.policy.busy_reask_s if busy else self.policy.idle_reask_s)
            if at <= self.horizon_s:
                self.scheduler.timer(actor, 'timer' if busy else 'idle', at=at)
        zi.IntegratedTrial._arm_reask = old
    out[rule] = {}
    for c in T.CONDITIONS:
        trial, result, _ = T.run(c, horizon=445.)
        out[rule][c] = {'calls': len(result.calls), 'http_attempts': result.cost['http_attempts'],
                        'end_reason': result.end_reason, 'messages': len(result.messages)}
print(json.dumps(out, indent=1))
