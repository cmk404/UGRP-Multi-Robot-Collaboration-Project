"""Fixed-cadence decision layer for the v3 own-camera judgments.

Same policy as v1/v2 (``harness/zone_own_outcome.py``), *inherited*, not copied:
cadence, stability, commit confidence, deadline and the first-class
``unconfirmed``/``unknown`` are v1's. Only the accepted judgment names grow.
Neither earlier file is modified.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from harness import zone_own_outcome as _v1
from harness import zone_own_outcome_v2 as _v2
from harness.zone_own_perception_v3 import JUDGMENTS as V3_JUDGMENTS, SCHEMA as V3_SCHEMA

SCHEMA = 'ugrp.zone_own_outcome.v3'
CONFIRMED, UNCONFIRMED = _v1.CONFIRMED, _v1.UNCONFIRMED
CADENCE_S = _v1.CADENCE_S
STABLE_TICKS = _v1.STABLE_TICKS
COMMIT_CONFIDENCE = _v1.COMMIT_CONFIDENCE
DEADLINE_S = _v1.DEADLINE_S
MAX_OBSERVATIONS = _v1.MAX_OBSERVATIONS
JUDGMENTS = tuple(V3_JUDGMENTS)
ALL_JUDGMENTS = tuple(_v2.ALL_JUDGMENTS) + JUDGMENTS


class JudgmentTrackerV3(_v1.JudgmentTracker):
    """v1's tracker, unchanged policy, extended to the v3 judgment names."""

    def __init__(self, judgment: str, **kwargs: Any):
        if judgment not in ALL_JUDGMENTS:
            raise ValueError(f'judgment must be one of {ALL_JUDGMENTS}')
        super().__init__(judgment if judgment in _v1.JUDGMENTS else _v1.JUDGMENTS[0], **kwargs)
        self.judgment = judgment

    def status(self) -> dict[str, Any]:
        row = super().status()
        row['schema'] = SCHEMA
        if self.judgment in JUDGMENTS:
            row['judgment_schema'] = V3_SCHEMA
        return row


def track(judgment: str, observations: Sequence[Mapping[str, Any]], *,
          cadence_s: float = CADENCE_S, start_sim_s: float = 0., **kwargs: Any) -> dict[str, Any]:
    """Offline helper: feed a ready series of momentary judgments on the cadence."""
    tracker = JudgmentTrackerV3(judgment, cadence_s=cadence_s, **kwargs)
    for index, observation in enumerate(observations):
        tracker.feed(observation, start_sim_s + index*cadence_s)
        if tracker.decision is not None:
            break
    return tracker.record()


__all__ = ['SCHEMA', 'CONFIRMED', 'UNCONFIRMED', 'CADENCE_S', 'STABLE_TICKS', 'COMMIT_CONFIDENCE', 'DEADLINE_S', 'MAX_OBSERVATIONS',
           'JUDGMENTS', 'ALL_JUDGMENTS', 'JudgmentTrackerV3', 'track']
