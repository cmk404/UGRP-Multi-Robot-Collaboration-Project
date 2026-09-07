"""Matched, condition-level accounting for warehouse protocol experiments."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import sqrt
from typing import Any, Mapping


@dataclass(frozen=True)
class EpisodeResult:
    condition: str
    seed: int
    scenario: str
    success: bool | None
    elapsed_s: float | None = None
    rounds: int | None = None
    actions: int | None = None
    input_tokens: int | None = None
    output_tokens: int | None = None
    tokens_total: int | None = None
    error: str | None = None
    provenance: str = "fixture"
    budget: Any = None
    sensor_profile: str | None = None
    controller_profile: str | None = None


class EvaluationLedger:
    """Collect complete matched episodes without imputing missing measurements."""

    def __init__(self, conditions, seeds, scenarios, *, budget=None,
                 sensor_profiles=None, controller_profiles=None):
        self.conditions = tuple(conditions)
        self.seeds = tuple(seeds)
        self.scenarios = tuple(scenarios)
        if not self.conditions or not self.seeds or not self.scenarios:
            raise ValueError("conditions, seeds, and scenarios cannot be empty")
        if (len(set(self.conditions)) != len(self.conditions) or len(set(self.seeds)) != len(self.seeds)
                or len(set(self.scenarios)) != len(self.scenarios)):
            raise ValueError("duplicate ledger configuration")
        self.budget = budget
        self.sensor_profiles = dict(sensor_profiles or {})
        self.controller_profiles = dict(controller_profiles or {})
        self._results: dict[tuple[str, int, str], EpisodeResult] = {}
        self._condition_provenance: dict[str, str] = {}

    @property
    def denominator(self) -> int:
        return len(self.conditions) * len(self.seeds) * len(self.scenarios)

    def append(self, result: EpisodeResult | Mapping[str, Any]) -> None:
        self.record(result)

    def record(self, result: EpisodeResult | Mapping[str, Any]) -> None:
        if isinstance(result, Mapping):
            result = EpisodeResult(**dict(result))
        if not isinstance(result, EpisodeResult):
            raise TypeError("result must be EpisodeResult or a mapping")
        key = (result.condition, result.seed, result.scenario)
        if result.condition not in self.conditions or result.seed not in self.seeds or result.scenario not in self.scenarios:
            raise ValueError("result is outside the matched ledger design")
        if key in self._results:
            raise ValueError("duplicate condition/seed/scenario result")
        expected_budget = self.budget
        if expected_budget is not None and result.budget != expected_budget:
            raise ValueError("budget mismatch")
        for expected, actual, label in (
            (self.sensor_profiles.get(result.condition), result.sensor_profile, "sensor profile"),
            (self.controller_profiles.get(result.condition), result.controller_profile, "controller profile"),
        ):
            if expected is not None and actual != expected:
                raise ValueError(f"{label} mismatch")
        if result.provenance not in ("fixture", "live"):
            raise ValueError("provenance must be fixture or live")
        if type(result.success) is not bool and result.success is not None:
            raise ValueError("success must be a boolean or None")
        prior_provenance = self._condition_provenance.get(result.condition)
        if prior_provenance is not None and prior_provenance != result.provenance:
            raise ValueError("cannot mix fixture and live evidence in one condition")
        self._condition_provenance[result.condition] = result.provenance
        self._results[key] = result

    def result(self, condition: str, seed: int, scenario: str) -> EpisodeResult | None:
        return self._results.get((condition, seed, scenario))

    def summary(self) -> dict[str, Any]:
        conditions = {}
        for condition in self.conditions:
            rows = [self._results.get((condition, seed, scenario))
                    for seed in self.seeds for scenario in self.scenarios]
            completed = [row for row in rows if row is not None]
            successes = sum(bool(row.success) for row in completed)
            times = [row.elapsed_s for row in completed if row.elapsed_s is not None]
            rounds = [row.rounds for row in completed if row.rounds is not None]
            actions = [row.actions for row in completed if row.actions is not None]
            tokens = [row.tokens_total if row.tokens_total is not None else row.input_tokens + row.output_tokens
                      for row in completed if row.tokens_total is not None or
                      (row.input_tokens is not None and row.output_tokens is not None)]
            conditions[condition] = {
                "denominator": len(rows), "recorded": len(completed),
                "missing": len(rows) - len(completed),
                "failures": len(rows) - successes,
                "recorded_failures": len(completed) - successes,
                "successes": successes, "success_rate": successes / len(rows),
                "elapsed_s_mean": self._mean(times), "rounds_mean": self._mean(rounds),
                "actions_mean": self._mean(actions),
                "total_tokens_mean": self._mean(tokens),
                "token_counts_missing": sum(row.tokens_total is None and
                                             (row.input_tokens is None or row.output_tokens is None)
                                             for row in completed),
                "provenance": {kind: sum(row.provenance == kind for row in completed)
                               for kind in ("fixture", "live")},
            }
        return {"denominator_per_condition": len(self.seeds) * len(self.scenarios),
                "complete": len(self._results) == self.denominator,
                "conditions": conditions}

    def paired(self, condition_a: str, condition_b: str) -> dict[str, Any]:
        if condition_a not in self.conditions or condition_b not in self.conditions:
            raise ValueError("unknown condition")
        pairs = []
        for seed in self.seeds:
            for scenario in self.scenarios:
                a = self.result(condition_a, seed, scenario)
                b = self.result(condition_b, seed, scenario)
                if a is None or b is None:
                    continue
                pairs.append({"seed": seed, "scenario": scenario,
                              "a_success": bool(a.success), "b_success": bool(b.success),
                              "success_delta": int(bool(b.success)) - int(bool(a.success)),
                              "time_delta_s": (b.elapsed_s - a.elapsed_s)
                              if a.elapsed_s is not None and b.elapsed_s is not None else None})
        deltas = [p["success_delta"] for p in pairs]
        win_fraction = (sum(d > 0 for d in deltas) / len(deltas)) if deltas else None
        return {"condition_a": condition_a, "condition_b": condition_b,
                "pairs": pairs, "paired_denominator": len(self.seeds) * len(self.scenarios),
                "paired_recorded": len(pairs), "success_delta_mean": self._mean(deltas),
                "b_win_fraction": win_fraction,
                "b_win_fraction_wilson_95": self._wilson(sum(d > 0 for d in deltas), len(deltas))}

    def records(self) -> tuple[EpisodeResult, ...]:
        return tuple(self._results.values())

    @staticmethod
    def _mean(values):
        return sum(values) / len(values) if values else None

    @staticmethod
    def _wilson(successes: int, total: int):
        if not total:
            return None
        z = 1.959963984540054
        p = successes / total
        den = 1 + z * z / total
        center = (p + z * z / (2 * total)) / den
        half = z * sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / den
        return (max(0.0, center - half), min(1.0, center + half))
