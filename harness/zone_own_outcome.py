"""Fixed-cadence, multi-observation decision for the own-camera zone judgments.

``harness/zone_own_perception.py`` answers one momentary question from one wrist
frame. This module is the decision layer: it is fed on a fixed SIM cadence and
only commits when the same answer repeats, so a single lucky or unlucky frame
never becomes a receipt.

The structure - fixed cadence, stability over ticks, a first-class
``unconfirmed`` decision, and recorded evidence - is the part of
``harness/zone_rgb_outcome.py`` (PR #170) that survives the 2026-09-26 input
change. That module's TOP reference/before/current frames and its use of the
other carriers' issued gripper commands cannot be used for a robot-facing
decision any more, so nothing is copied: this tracker takes own-RGB
observations only.

Policy
    * ``feed`` accepts one observation per tick; ticks closer together than
      ``CADENCE_S`` are refused (the caller must keep the cadence identical
      across communication conditions, so observation cost is comparable).
    * An answer is committed after ``STABLE_TICKS`` consecutive identical
      answers whose confidence reaches ``COMMIT_CONFIDENCE``.
    * ``unknown`` never commits and never decays into ``no``. After
      ``DEADLINE_S`` without a commit, the decision stays ``unconfirmed`` and
      the reason records that the robot could not tell.
    * A re-look plan (``pan_plan``) is offered, not executed: the caller drives
      the arm with its own issued commands.

Only a ``confirmed`` decision may become a receipt, a message claim or a stage
transition. ``unconfirmed`` must stay ``unknown`` in prompts.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from harness.zone_own_perception import ANSWERS, JUDGMENTS, PAN_PULSES, PROVENANCE

SCHEMA = 'ugrp.zone_own_outcome.v1'
CONFIRMED, UNCONFIRMED = 'confirmed', 'unconfirmed'
# PREREGISTERED (experiments/2026-09-26-zone-own-perception); identical for all
# communication conditions so that observation cost cannot differ between them.
CADENCE_S = 1.0
STABLE_TICKS = 2
COMMIT_CONFIDENCE = .65
DEADLINE_S = 40.0
MAX_OBSERVATIONS = 24
# Look-back re-observation: the three agreed pan pulses, straight ahead first,
# then the two off-axis ones, then give up.
PAN_PLAN = (1500, 1230, 1770)
assert sorted(PAN_PLAN) == sorted(PAN_PULSES), 'the re-look plan must use the agreed pan pulses'


class JudgmentTracker:
    """Commit one own-camera judgment from a fixed-cadence series of observations."""

    def __init__(self, judgment: str, *, question: Mapping[str, Any] | None = None,
                 cadence_s: float = CADENCE_S, stable_ticks: int = STABLE_TICKS,
                 commit_confidence: float = COMMIT_CONFIDENCE, deadline_s: float = DEADLINE_S,
                 max_observations: int = MAX_OBSERVATIONS, pan_plan: Sequence[int] = PAN_PLAN):
        if judgment not in JUDGMENTS:
            raise ValueError(f'judgment must be one of {JUDGMENTS}')
        if stable_ticks < 1 or cadence_s <= 0:
            raise ValueError('stable_ticks >= 1 and cadence_s > 0')
        self.judgment = judgment
        self.question = dict(question or {})
        self.cadence_s = float(cadence_s)
        self.stable_ticks = int(stable_ticks)
        self.commit_confidence = float(commit_confidence)
        self.deadline_s = float(deadline_s)
        self.max_observations = int(max_observations)
        self.pan_plan = tuple(int(v) for v in pan_plan)
        self.observations: list[dict[str, Any]] = []
        self.first_sim_s: float | None = None
        self.last_sim_s: float | None = None
        self._streak_answer: str | None = None
        self._streak = 0
        self.decision: dict[str, Any] | None = None

    # ------------------------------------------------------------------ input
    def feed(self, observation: Mapping[str, Any], sim_s: float) -> dict[str, Any]:
        """Add one momentary judgment; returns the current decision."""
        answer = observation.get('answer')
        if answer not in ANSWERS:
            raise ValueError(f'observation answer must be one of {ANSWERS}')
        if observation.get('judgment') != self.judgment:
            raise ValueError('observation is for another judgment')
        sim_s = float(sim_s)
        if self.last_sim_s is not None and sim_s < self.last_sim_s + self.cadence_s - 1e-9:
            raise ValueError('observations must keep the fixed cadence')
        if self.first_sim_s is None:
            self.first_sim_s = sim_s
        self.last_sim_s = sim_s
        confidence = float(observation.get('confidence') or 0.)
        row = {'sim_s': sim_s, 'answer': answer, 'confidence': confidence,
               'reason': observation.get('reason'), 'observed': observation.get('observed'),
               'pan_pwm': observation.get('pan_pwm')}
        self.observations.append(row)
        strong = answer != 'unknown' and confidence >= self.commit_confidence
        if strong and answer == self._streak_answer:
            self._streak += 1
        elif strong:
            self._streak_answer, self._streak = answer, 1
        else:
            # An unknown or weak frame breaks the streak but never flips it.
            self._streak_answer, self._streak = None, 0
        if self.decision is None and self._streak >= self.stable_ticks:
            self.decision = self._commit(answer, confidence, row)
        return self.status()

    def _commit(self, answer, confidence, row):
        supporting = [o for o in self.observations if o['answer'] == answer][-self.stable_ticks:]
        return {'status': CONFIRMED, 'answer': answer,
                'confidence': round(min(confidence, max(o['confidence'] for o in supporting)), 3),
                'reason': row['reason'], 'observed': row['observed'],
                'committed_at_sim_s': row['sim_s'], 'supporting_ticks': supporting}

    # ----------------------------------------------------------------- output
    def elapsed_s(self) -> float:
        if self.first_sim_s is None or self.last_sim_s is None:
            return 0.
        return self.last_sim_s - self.first_sim_s

    def exhausted(self) -> bool:
        return (self.decision is None
                and (self.elapsed_s() >= self.deadline_s or len(self.observations) >= self.max_observations))

    def next_pan_pwm(self) -> int | None:
        """Pan pulse to issue for the next re-look, or None when the plan is spent.

        A plan, not an action: the caller issues the command itself.
        """
        if self.decision is not None:
            return None
        index = len(self.observations)
        return self.pan_plan[index] if index < len(self.pan_plan) else None

    def status(self) -> dict[str, Any]:
        base = {'schema': SCHEMA, 'judgment': self.judgment, 'question': dict(self.question),
                'cadence_s': self.cadence_s, 'stable_ticks': self.stable_ticks,
                'commit_confidence': self.commit_confidence, 'deadline_s': self.deadline_s,
                'observations': len(self.observations), 'elapsed_s': round(self.elapsed_s(), 3),
                'unknown_ticks': sum(1 for o in self.observations if o['answer'] == 'unknown'),
                'answers': [o['answer'] for o in self.observations],
                'next_pan_pwm': self.next_pan_pwm(), 'provenance': PROVENANCE}
        if self.decision is not None:
            return {**base, **self.decision}
        reason = 'AWAITING_STABLE_ANSWER'
        if self.exhausted():
            reason = ('DEADLINE_WITHOUT_STABLE_ANSWER' if self.elapsed_s() >= self.deadline_s
                      else 'OBSERVATION_BUDGET_SPENT')
        return {**base, 'status': UNCONFIRMED, 'answer': 'unknown', 'confidence': 0.0,
                'reason': reason, 'observed': 'not_observed', 'exhausted': self.exhausted()}

    def record(self) -> dict[str, Any]:
        """Full audit row: the decision plus every observation that produced it."""
        return {**self.status(), 'observation_log': list(self.observations)}


def track(judgment: str, observations: Sequence[Mapping[str, Any]], *,
          cadence_s: float = CADENCE_S, start_sim_s: float = 0., **kwargs) -> dict[str, Any]:
    """Offline helper: feed a ready series of momentary judgments on the cadence."""
    tracker = JudgmentTracker(judgment, cadence_s=cadence_s, **kwargs)
    for index, observation in enumerate(observations):
        tracker.feed(observation, start_sim_s + index*cadence_s)
        if tracker.decision is not None:
            break
    return tracker.record()


__all__ = ['SCHEMA', 'CONFIRMED', 'UNCONFIRMED', 'CADENCE_S', 'STABLE_TICKS', 'COMMIT_CONFIDENCE',
           'DEADLINE_S', 'MAX_OBSERVATIONS', 'PAN_PLAN', 'JudgmentTracker', 'track']
