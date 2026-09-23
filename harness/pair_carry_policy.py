"""RGB reports and a bounded execution policy for an already grasped beam.

This is a local synchronization/controller experiment, not an LLM planner.
The fixed camera shows r1 below r3; no world pose or applied command is read.
"""
from __future__ import annotations

import math
from harness.camera_beam_features import extract_beams
from harness.pair_carry_sync import PairCarrySync

ROBOTS = ('r1', 'r3')


def payload_skew(top_jpeg: bytes):
    """Bottom minus top endpoint X, in pixels of the original camera image."""
    candidates = [b for b in extract_beams(top_jpeg)
                  if .3 <= b['center'][1] <= .7 and not b['touches_border']]
    if not candidates:
        return None
    beam = min(candidates, key=lambda b: (abs(b['center'][1] - .5), -b['length_px']))
    upper, lower = sorted(beam['endpoints'], key=lambda p: p[1])
    if lower[1] - upper[1] < .05:
        return None
    return (lower[0] - upper[0]) * beam['image_size'][0]


class PairCarryPolicy:
    """Fresh two-party permission, stop, image-based catch-up, and rejoin."""
    def __init__(self, task_id='pair-carry'):
        self.sync = PairCarrySync(task_id, report_ttl_s=.6)
        self.mode = 'CRUISE'
        self.until_s = 0.
        self.skew_anchor = None
        self.index = 0
        self.invalid_count = 0
        self.confirmations = 0
        self.recoveries = 0
        self.events = []

    def step(self, decisions, skew_px, frame_ids, now_s, delivered=ROBOTS, *, observed_at_s=None):
        if not math.isfinite(now_s):
            raise ValueError('finite monotonic clock required')
        # Legacy synchronous callers observe and decide at one frozen SIM
        # instant. Async callers must pass the RGB snapshot time explicitly.
        observed_at_s = now_s if observed_at_s is None else float(observed_at_s)
        if not math.isfinite(observed_at_s) or observed_at_s > now_s:
            raise ValueError('RGB observation must have a finite past SIM timestamp')
        duration = .20
        forwards = {r: 0. for r in ROBOTS}
        # Local predictors may run during a relay outage, but an undelivered
        # report must not influence the execution coordinator.
        decisions = {r: decisions[r] for r in delivered if r in decisions}
        missing = set(decisions) != set(ROBOTS)
        valid = (now_s-observed_at_s <= self.sync.report_ttl_s
                 and set(decisions) == set(ROBOTS)
                 and all(d.get('ok') is True and d.get('held_estimate') is True
                         for d in decisions.values())
                 and skew_px is not None and math.isfinite(skew_px))
        if self.skew_anchor is None and valid:
            self.skew_anchor = float(skew_px)
        error_px = None if not valid else float(skew_px) - self.skew_anchor
        self.index += 1
        if self.mode == 'ABORT':
            permission = self.sync.abort('terminal_abort', now_s)
        elif missing:
            self.confirmations = 0
            permission = self.sync.hold('fresh_pair_report_missing', now_s)
        elif not valid:
            self.confirmations = 0
            self.invalid_count += 1
            permission = self.sync.hold('visual_evidence_unavailable', now_s)
            if self.invalid_count >= 5:
                self.mode = 'ABORT'
                permission = self.sync.abort('visual_evidence_unavailable_after_reobserve', now_s)
        else:
            self.invalid_count = 0
            if abs(error_px) > 12.:
                self.mode = 'ABORT'
                permission = self.sync.abort('skew_outside_bounded_recovery', now_s)
            else:
                if self.mode in ('CRUISE', 'CONFIRM') and abs(error_px) > 3.:
                    self.mode, self.until_s = 'SETTLE', now_s + .4
                    self.confirmations = 0
                    self.recoveries += 1
                    self.sync.hold('visual_payload_skew', now_s)
                    self.events.append({'event': 'SKEW_HOLD', 'time_s': now_s,
                                        'skew_error_px': error_px})
                if self.recoveries > 8:
                    self.mode = 'ABORT'
                    permission = self.sync.abort('recovery_budget_exhausted', now_s)
                else:
                    if self.mode in ('SETTLE', 'REJOIN') and now_s + 1e-9 >= self.until_s:
                        self.mode = ('CRUISE' if self.mode == 'REJOIN' and abs(error_px) <= 3.
                                     else 'ALIGN')
                    if self.mode == 'ALIGN' and abs(error_px) <= 1.:
                        self.mode, self.until_s = 'REJOIN', now_s + .4
                        self.sync.hold('alignment_observed_reconfirm', now_s)
                    waiting = self.mode in ('SETTLE', 'REJOIN')
                    accepted = []
                    for rid in delivered:
                        accepted.append(self.sync.report(rid, plan_version=1, epoch=self.sync.epoch,
                            sequence=self.index, ready=not waiting, observed_at_s=observed_at_s,
                            received_at_s=now_s, frame_id=str(frame_ids[rid]),
                            reason='rgb_valid_and_held_estimate'))
                    if len(accepted) != len(ROBOTS) or not all(accepted):
                        self.confirmations = 0
                        permission = self.sync.hold('current_report_rejected', now_s)
                    else:
                        permission = self.sync.authorize(now_s)
                    if permission['phase'] == 'GO' and not waiting:
                        if self.mode == 'ALIGN':
                            # r1 is bottom, r3 top in this fixed-view fixture.
                            lagging = 'r3' if error_px > 0 else 'r1'
                            forwards[lagging] = .04
                        elif self.mode in ('CRUISE', 'CONFIRM'):
                            goal_ready = all(d['ready'] is True for d in decisions.values())
                            if goal_ready:
                                self.confirmations += 1
                                self.mode = 'DONE' if self.confirmations >= 3 else 'CONFIRM'
                                duration = .25
                            else:
                                self.confirmations = 0
                                self.mode = 'CRUISE'
                                forwards = {r: float(decisions[r]['forward']) for r in ROBOTS}
        return {'forwards': forwards, 'duration_s': duration, 'mode': self.mode,
                'permission': permission, 'skew_error_px': error_px,
                'done': self.mode == 'DONE', 'abort': self.mode == 'ABORT',
                'valid': valid, 'recovery_count': self.recoveries}
