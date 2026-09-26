"""Fixed-cadence decision layer for the v2 own-camera judgments.

The policy is v1's (``harness/zone_own_outcome.py``, PR #193) and is *inherited*,
not copied: same cadence, same stability requirement, same commit confidence,
same deadline, same first-class ``unconfirmed``/``unknown``, same re-look plan.
That file is not modified, so the v1 judgments keep committing exactly as before.
Only the accepted judgment names change, because v1's tracker validates against
its own four names.

Only a ``confirmed`` decision may become a receipt, a message claim or a stage
transition; ``unconfirmed`` must stay ``unknown`` in prompts.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from harness import zone_own_outcome as _v1
from harness.zone_own_perception_v2 import JUDGMENTS as V2_JUDGMENTS, SCHEMA as V2_SCHEMA

SCHEMA = 'ugrp.zone_own_outcome.v2'
CONFIRMED, UNCONFIRMED = _v1.CONFIRMED, _v1.UNCONFIRMED
# PREREGISTERED: identical to v1 so observation cost cannot differ between the
# communication conditions or between the two judgment versions.
CADENCE_S = _v1.CADENCE_S
STABLE_TICKS = _v1.STABLE_TICKS
COMMIT_CONFIDENCE = _v1.COMMIT_CONFIDENCE
DEADLINE_S = _v1.DEADLINE_S
MAX_OBSERVATIONS = _v1.MAX_OBSERVATIONS
PAN_PLAN = _v1.PAN_PLAN
JUDGMENTS = tuple(V2_JUDGMENTS)
ALL_JUDGMENTS = tuple(_v1.JUDGMENTS) + JUDGMENTS


class JudgmentTrackerV2(_v1.JudgmentTracker):
    """v1's tracker, unchanged policy, extended to the v2 judgment names.

    ``super().__init__`` validates the name against v1's four judgments, so it is
    given a v1 name and the real judgment is set afterwards. Everything else -
    ``feed``, the streak rule, ``status``, ``record``, ``next_pan_pwm`` - is v1's
    code path.
    """

    def __init__(self, judgment: str, **kwargs: Any):
        if judgment not in ALL_JUDGMENTS:
            raise ValueError(f'judgment must be one of {ALL_JUDGMENTS}')
        super().__init__(judgment if judgment in _v1.JUDGMENTS else _v1.JUDGMENTS[0], **kwargs)
        self.judgment = judgment

    def status(self) -> dict[str, Any]:
        row = super().status()
        row['schema'] = SCHEMA
        row['judgment_schema'] = V2_SCHEMA if self.judgment in JUDGMENTS else row.get('judgment_schema')
        return row


def track(judgment: str, observations: Sequence[Mapping[str, Any]], *,
          cadence_s: float = CADENCE_S, start_sim_s: float = 0., **kwargs: Any) -> dict[str, Any]:
    """Offline helper: feed a ready series of momentary judgments on the cadence."""
    tracker = JudgmentTrackerV2(judgment, cadence_s=cadence_s, **kwargs)
    for index, observation in enumerate(observations):
        tracker.feed(observation, start_sim_s + index*cadence_s)
        if tracker.decision is not None:
            break
    return tracker.record()


__all__ = ['SCHEMA', 'CONFIRMED', 'UNCONFIRMED', 'CADENCE_S', 'STABLE_TICKS', 'COMMIT_CONFIDENCE',
           'DEADLINE_S', 'MAX_OBSERVATIONS', 'PAN_PLAN', 'JUDGMENTS', 'ALL_JUDGMENTS',
           'JudgmentTrackerV2', 'track']
