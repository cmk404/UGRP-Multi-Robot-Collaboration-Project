"""Robot-facing results for zone protocol v2: what robots and the host are told.

Two ledgers exist in a v2 run and must not be confused:

- the executor's ``TeamJobLedger`` (``scripts.zone_team_teacher``) is
  teacher-motion state: which TeamJob holds which item, landing areas, the
  once-per-item decrement used to stop the teacher. It reads ground truth
  (contact forces, item height) and never reaches a robot directly;
- ``RobotResults`` (this module) is the robot-facing side: the status line in
  each robot's own job list, the finished/stopped reports on the peer board,
  re-ask timing, and the delivered list the host check and the fixtures use.

``RobotResults`` gets each ended claim's result from an outcome source. The only
source today is ``TeacherReceiptSource``: the teacher's receipt text (contact +
height decide "finished" vs "stopped"), i.e. audit leak L4, acceptable only for
the labelled teacher smoke and never for a student or RGB claim. An RGB outcome
source (PR #170: robot-verified outcome with an ``unconfirmed`` state and no
coupling to the teacher end time) plugs in here by implementing ``outcome``.

Message hook (Korean dialogue redesign): robot-facing wording of results lives
in the source's ``status`` text and in ``harness.zone_protocol_v2``; nothing
here formats model prompts.
"""
from __future__ import annotations

import copy

from harness.zone_team_jobs import RECEIPT_FINISHED

ROBOT_RESULTS_SCHEMA = 'ugrp.zone_robot_results.v1'


class TeacherReceiptSource:
    """Robot-facing outcome = the teacher's receipt (audit L4, teacher smoke only)."""
    name = 'teacher_receipt_L4'
    label = ('teacher ground-truth receipt: contact force and item height decide "issued sequence finished" vs '
             '"executor stopped before finishing" (audit L4 unresolved; teacher smoke only, never a student or '
             'RGB-verified outcome)')

    def outcome(self, ended, sim_time_s):
        """ended: the executor's ended claim (``.receipt``, ``.role_claim``). Returns the robot-facing outcome."""
        return {'status': ended.receipt, 'delivered': ended.receipt == RECEIPT_FINISHED,
                'confirmed': True, 'source': self.name}

    def record(self):
        return {'name': self.name, 'label': self.label}


class RobotResults:
    """Own job lists, peer-board reports and the robot-facing delivered list."""

    def __init__(self, robots, source):
        self.source = source
        self.own_jobs = {r: [] for r in robots}
        self.finished, self.stopped = [], []
        self.delivered_labels = {}     # item label -> {'zone', 'kind'}: counted once per label

    def issue(self, rid, claim, sim_time_s):
        self.own_jobs[rid].append({'item': claim.item, 'zone': claim.zone, 'role': claim.role, 'status': 'issued',
                                   'issued_at_sim_s': round(sim_time_s, 2)})

    def end(self, rid, ended, sim_time_s):
        """Record one ended claim through the outcome source; returns the robot-facing outcome."""
        out = self.source.outcome(ended, sim_time_s)
        rc = ended.role_claim
        self.own_jobs[rid][-1]['status'] = out['status']
        report = {'robot': rid, 'item': rc.item, 'zone': rc.zone, 'role': rc.role,
                  'executor_receipt': out['status'], 'sim_time_s': round(sim_time_s, 2)}
        (self.finished if out['delivered'] else self.stopped).append(report)
        if out['delivered'] and out['confirmed']:
            self.delivered_labels.setdefault(rc.item, {'zone': rc.zone, 'kind': rc.kind})
        return out

    def delivered(self):
        return [dict(v) for _, v in sorted(self.delivered_labels.items())]

    def board(self, active, *, finished_n=12, stopped_n=6):
        return {'active': {r: {'item': c.item, 'zone': c.zone, 'role': c.role} for r, c in active.items()},
                'finished_reports': copy.deepcopy(self.finished[-finished_n:]),
                'stopped_reports': copy.deepcopy(self.stopped[-stopped_n:])}

    def record(self):
        return {'schema': ROBOT_RESULTS_SCHEMA, 'outcome_source': self.source.record(),
                'delivered_labels': copy.deepcopy(self.delivered_labels),
                'finished_reports': copy.deepcopy(self.finished), 'stopped_reports': copy.deepcopy(self.stopped)}


__all__ = ['ROBOT_RESULTS_SCHEMA', 'TeacherReceiptSource', 'RobotResults']
