"""Contract tests for the cooperative payload-transport MVP.

The simulator implementation is intentionally discovered at runtime.  During
the split implementation phase the implementation may live in either
``harness`` or ``sim`` and may expose a factory or a class; the small adapter
below keeps these tests independent of that packaging choice.  The semantics
under test are deliberately *not* optional:

* the mission has distinct start/goal zones and a non-cubic, long/heavy object;
* one robot is refused for a payload that requires two carriers;
* two robots may transport it only with physical contact/grasp evidence; and
* the final pose is accepted only when zone, yaw, and settled/stable gates pass.

Set ``UGRP_COOPERATIVE_PAYLOAD_MODULE`` to the module under development when
it is not one of the conventional candidates below.  The tests are skipped
until that module exists so the rest of the legacy suite remains runnable
while the production implementation is being developed.
"""

from __future__ import annotations

import importlib
import inspect
import math
import os
import unittest
from collections.abc import Mapping
from typing import Any, Callable


def _load_backend() -> tuple[Any | None, str | None]:
    names = []
    selected = os.environ.get("UGRP_COOPERATIVE_PAYLOAD_MODULE")
    if selected:
        names.append(selected)
    names.extend(
        (
            "harness.cooperative_payload",
            "harness.cooperative_transport",
            "harness.team_transport",
            "sim.cooperative_payload",
            "sim.cooperative_transport",
            "sim.cooperative_beam",
        )
    )
    errors: list[str] = []
    for name in dict.fromkeys(names):
        try:
            return importlib.import_module(name), None
        except ModuleNotFoundError as exc:
            errors.append(f"{name}: {exc}")
    return None, "; ".join(errors)


BACKEND, BACKEND_IMPORT_ERROR = _load_backend()
BACKEND_AVAILABLE = BACKEND is not None


def _get(value: Any, *keys: str, default: Any = None) -> Any:
    """Read a field from either a mapping or a simple dataclass-like object."""
    if isinstance(value, Mapping):
        for key in keys:
            if key in value:
                return value[key]
    for key in keys:
        if hasattr(value, key):
            return getattr(value, key)
    return default


def _flatten_dict(value: Any) -> dict[str, Any]:
    """Flatten one result/spec level without making production shape prescriptive."""
    if isinstance(value, Mapping):
        nested = value.get("result")
        if isinstance(nested, Mapping):
            return {**value, **nested}
        return dict(value)
    return {
        name: getattr(value, name)
        for name in dir(value)
        if not name.startswith("_") and not callable(getattr(value, name, None))
    }


def _number(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _xyz(value: Any) -> tuple[float, float, float] | None:
    if isinstance(value, Mapping):
        value = _get(value, "position", "pos", "xyz", "center", "centre")
    if value is None or isinstance(value, (str, bytes)):
        return None
    try:
        values = list(value)
    except TypeError:
        return None
    if len(values) < 3:
        return None
    try:
        return tuple(float(v) for v in values[:3])  # type: ignore[return-value]
    except (TypeError, ValueError):
        return None


def _first(mapping: Any, *paths: tuple[str, ...], default: Any = None) -> Any:
    """Resolve the first available nested path from a mapping/object."""
    for path in paths:
        current = mapping
        for key in path:
            current = _get(current, key, default=None)
            if current is None:
                break
        if current is not None:
            return current
    return default


def _zone_center(spec: Any, which: str) -> tuple[float, float, float]:
    zone = _first(
        spec,
        (which,),
        (f"{which}_zone",),
        ("zones", which),
        ("task", which),
        default=None,
    )
    center = _first(zone, ("center",), ("centre",), ("position",), ("xyz",), default=zone)
    result = _xyz(center)
    if result is None:
        raise AssertionError(f"mission spec has no {which} zone center: {spec!r}")
    return result


def _zone_yaw(spec: Any, which: str) -> float:
    zone = _first(spec, (which,), (f"{which}_zone",), ("zones", which), default={})
    value = _first(zone, ("yaw",), ("heading",), ("orientation",), default=0.0)
    if isinstance(value, (list, tuple)):
        value = value[-1]
    return _number(value) or 0.0


def _spec_of(mission: Any) -> Any:
    spec = _get(mission, "spec", "mission_spec", "configuration", default=None)
    if callable(spec):
        spec = spec()
    if spec is None and isinstance(mission, Mapping):
        spec = mission
    if spec is None:
        raise AssertionError("mission must expose spec or mission_spec")
    return spec


def _invoke(fn: Callable[..., Any], values: Mapping[str, Any]) -> Any:
    """Call a future API with canonical names and conservative aliases."""
    aliases = {
        "payload_type": ("payload_type", "object_type", "shape", "kind"),
        "carriers": ("carriers", "carrier_ids", "robot_ids", "robots", "agents"),
        "state": ("state", "world_state", "final_state", "observation", "snapshot"),
        "spec": ("spec", "mission_spec", "config"),
    }
    kwargs: dict[str, Any] = {}
    try:
        parameters = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        parameters = {}
    has_kwargs = any(p.kind == p.VAR_KEYWORD for p in parameters.values())
    for canonical, value in values.items():
        choices = aliases.get(canonical, (canonical,))
        selected = next((name for name in choices if has_kwargs or name in parameters), None)
        if selected:
            kwargs[selected] = value
    if kwargs or not parameters:
        return fn(**kwargs)
    # A positional-only or opaque callable can still implement the canonical
    # one-argument factory contract.
    ordered = [values[name] for name in ("payload_type", "state", "carriers") if name in values]
    return fn(*ordered[: len(parameters)])


def _make_mission(payload_type: str = "beam") -> Any:
    if BACKEND is None:
        raise unittest.SkipTest(f"cooperative payload backend unavailable: {BACKEND_IMPORT_ERROR}")
    factories = (
        "build_mission",
        "create_mission",
        "build_cooperative_payload_mission",
        "cooperative_payload_mission",
        "CooperativePayloadMission",
        "CooperativeTransportMission",
    )
    for name in factories:
        factory = getattr(BACKEND, name, None)
        if factory is None:
            continue
        try:
            return _invoke(factory, {"payload_type": payload_type})
        except (TypeError, ValueError):
            try:
                return factory()
            except (TypeError, ValueError):
                continue
    raise AssertionError(
        "cooperative payload backend must expose build_mission/create_mission "
        "or CooperativePayloadMission"
    )


def _payload_catalog() -> Any:
    if BACKEND is None:
        raise unittest.SkipTest(f"cooperative payload backend unavailable: {BACKEND_IMPORT_ERROR}")
    for name in ("PAYLOAD_CATALOG", "PAYLOADS", "payload_catalog", "available_payloads"):
        value = getattr(BACKEND, name, None)
        if callable(value):
            value = value()
        if value is not None:
            return value
    # A mission factory may be the only catalog surface; probing known shapes
    # still verifies that the implementation supports multiple payload forms.
    return {kind: _spec_of(_make_mission(kind)) for kind in ("beam", "panel", "pipe", "tray")}


def _transport(mission: Any, *, carriers: list[str], state: Mapping[str, Any] | None = None) -> Any:
    values = {"carriers": carriers}
    if state is not None:
        values["state"] = state
    for name in (
        "attempt_transport",
        "transport",
        "run_transport",
        "execute_transport",
        "start_transport",
        "deliver",
        "run",
        "execute",
    ):
        fn = getattr(mission, name, None)
        if callable(fn):
            return _invoke(fn, values)
    if BACKEND is not None:
        for name in ("attempt_transport", "transport_payload", "run_transport", "start_transport", "deliver"):
            fn = getattr(BACKEND, name, None)
            if callable(fn):
                return _invoke(fn, {**values, "mission": mission})
    raise AssertionError("mission must expose a transport/attempt_transport execution method")


def _evaluate(mission: Any, state: Mapping[str, Any]) -> Any:
    for name in (
        "evaluate_final_state",
        "evaluate_delivery",
        "evaluate",
        "final_zone_and_stability",
        "check_success",
        "is_goal_achieved",
    ):
        fn = getattr(mission, name, None)
        if callable(fn):
            return _invoke(fn, {"state": state})
    if BACKEND is not None:
        for name in ("evaluate_final_state", "evaluate_delivery", "evaluate_payload", "check_success"):
            fn = getattr(BACKEND, name, None)
            if callable(fn):
                return _invoke(fn, {"state": state, "spec": _spec_of(mission)})
    raise AssertionError("mission/backend must expose final-state evaluation")


def _success(result: Any) -> bool:
    result = _flatten_dict(result)
    for key in ("success", "succeeded", "ok", "passed", "goal_achieved"):
        if key in result:
            return bool(result[key])
    status = str(result.get("status", result.get("outcome_status", ""))).upper()
    return status in {"SUCCESS", "SUCCEEDED", "ACHIEVED", "PASS", "PASSED", "COMPLETE", "COMPLETED"}


def _failure_text(result: Any) -> str:
    flat = _flatten_dict(result)
    keys = ("failure_code", "reason", "error", "status", "outcome_status", "failure_reason")
    return " ".join(str(flat.get(k, "")) for k in keys).upper()


def _result_state(result: Any, fallback: Mapping[str, Any]) -> Mapping[str, Any]:
    flat = _flatten_dict(result)
    for key in ("final_state", "state", "world_state", "observation"):
        candidate = flat.get(key)
        if isinstance(candidate, Mapping):
            return candidate
    return fallback


@unittest.skipUnless(BACKEND_AVAILABLE, "cooperative payload production module is not present yet")
class CooperativePayloadMissionTests(unittest.TestCase):
    def test_mission_spec_has_distinct_start_goal_and_two_carrier_geometry(self):
        spec = _spec_of(_make_mission("beam"))
        start = _zone_center(spec, "start")
        goal = _zone_center(spec, "goal")
        self.assertGreater(math.dist(start, goal), 0.05)

        geometry = _first(spec, ("payload",), ("object",), ("geometry",), default=spec)
        dimensions = _first(geometry, ("dimensions",), ("size",), ("extent",), default=None)
        self.assertIsNotNone(dimensions, f"payload geometry missing dimensions: {spec!r}")
        dims = [float(v) for v in dimensions]
        self.assertGreaterEqual(len(dims), 3)
        self.assertGreater(max(dims) / min(dims), 2.0, "MVP payload must be long/non-cubic")

        mass = _first(geometry, ("mass",), ("kg",), ("weight_kg",), default=None)
        self.assertIsNotNone(mass, f"payload geometry missing mass: {spec!r}")
        self.assertGreater(float(mass), 0.0)
        carriers = _first(
            spec,
            ("required_carriers",),
            ("min_carriers",),
            ("required_robots",),
            ("payload", "required_carriers"),
            default=0,
        )
        self.assertGreaterEqual(int(carriers), 2)

    def test_catalog_contains_multiple_long_or_heavy_shapes(self):
        catalog = _payload_catalog()
        entries = list(catalog.values()) if isinstance(catalog, Mapping) else list(catalog)
        self.assertGreaterEqual(len(entries), 3, "payload catalog must expose varied object shapes")
        long_or_heavy = 0
        for entry in entries:
            geometry = _first(entry, ("payload",), ("object",), ("geometry",), default=entry)
            dims = _first(geometry, ("dimensions",), ("size",), ("extent",), default=None)
            mass = _number(_first(geometry, ("mass",), ("kg",), ("weight_kg",), default=0)) or 0.0
            if dims is not None:
                values = [float(v) for v in dims]
                if min(values) > 0 and max(values) / min(values) >= 2.0:
                    long_or_heavy += 1
            if mass > 0.1:
                long_or_heavy += 1
        self.assertGreaterEqual(long_or_heavy, 3)

    def test_single_carrier_is_refused_without_mutating_object_pose(self):
        mission = _make_mission("beam")
        spec = _spec_of(mission)
        start = _zone_center(spec, "start")
        state = {"object_pose": {"position": start, "yaw": _zone_yaw(spec, "start")}, "contacts": []}
        result = _transport(mission, carriers=["r1"], state=state)
        self.assertFalse(_success(result), result)
        failure = _failure_text(result)
        self.assertRegex(failure, r"SINGLE|INSUFFICIENT|TWO|CARRIER|COLLABOR")
        final = _result_state(result, state)
        self.assertEqual(_xyz(_first(final, ("object_pose",), ("payload_pose",), ("object",), default=final)), start)

    def test_two_carriers_can_succeed_only_with_bilateral_contact_evidence(self):
        mission = _make_mission("beam")
        spec = _spec_of(mission)
        goal = _zone_center(spec, "goal")
        state = {
            "object_pose": {"position": goal, "yaw": _zone_yaw(spec, "goal")},
            "stable": True,
            "settled": True,
            "contacts": [
                {"robot_id": "r1", "payload_id": "beam", "grasp": True},
                {"robot_id": "r2", "payload_id": "beam", "grasp": True},
            ],
            "carriers": ["r1", "r2"],
        }
        result = _transport(mission, carriers=["r1", "r2"], state=state)
        self.assertTrue(_success(result), result)
        self.assertNotRegex(_failure_text(result), r"TELEPORT|NO.?CONTACT|CONTACT.?REQUIRED")

    def test_final_evaluator_requires_zone_yaw_and_stability(self):
        mission = _make_mission("beam")
        spec = _spec_of(mission)
        goal = _zone_center(spec, "goal")
        yaw = _zone_yaw(spec, "goal")
        good = {
            "object_pose": {"position": goal, "yaw": yaw},
            "stable": True,
            "settled": True,
            "contacts": [{"robot_id": "r1", "grasp": True}, {"robot_id": "r2", "grasp": True}],
        }
        self.assertTrue(_success(_evaluate(mission, good)))

        yaw_tolerance = _number(
            _first(spec, ("yaw_tolerance",), ("max_yaw_error_rad",), ("goal", "yaw_tolerance"), default=0.1)
        ) or 0.1
        bad_yaw = {**good, "object_pose": {"position": goal, "yaw": yaw + max(0.2, yaw_tolerance * 4)}}
        self.assertFalse(_success(_evaluate(mission, bad_yaw)))
        bad_stability = {**good, "stable": False, "settled": False}
        self.assertFalse(_success(_evaluate(mission, bad_stability)))
        outside = {**good, "object_pose": {"position": (goal[0] + 2.0, goal[1], goal[2]), "yaw": yaw}}
        self.assertFalse(_success(_evaluate(mission, outside)))

    def test_pose_jump_without_contact_is_rejected_as_teleport(self):
        mission = _make_mission("beam")
        spec = _spec_of(mission)
        start = _zone_center(spec, "start")
        goal = _zone_center(spec, "goal")
        teleported = {
            "object_pose": {"position": goal, "yaw": _zone_yaw(spec, "goal")},
            "previous_object_pose": {"position": start, "yaw": _zone_yaw(spec, "start")},
            "stable": True,
            "settled": True,
            "contacts": [],
            "grasped_by": [],
            "transport_trace": [],
        }
        result = _evaluate(mission, teleported)
        self.assertFalse(_success(result), result)
        self.assertRegex(_failure_text(result), r"CONTACT|GRASP|TELEPORT|CARRY|PHYSICAL")


class CooperativeBeamWorldTests(unittest.TestCase):
    def test_single_robot_public_action_cannot_move_beam(self):
        from sim.multi_masterpi_production import MultiMasterPiProductionV2

        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            before = tuple(world.beam_state()["position"])
            result = world.act(
                "r1", "team_beam_transport",
                mission_id="beam_transport_v1", role="carrier_left",
            )
            after = tuple(world.beam_state()["position"])
            self.assertFalse(result.ok)
            self.assertIn("COOPERATIVE_QUORUM_REQUIRED", result.reason)
            self.assertLess(math.dist(before, after), 0.002)
        finally:
            world.close()

    def test_three_role_batch_physically_delivers_beam(self):
        from sim.multi_masterpi_production import MultiMasterPiProductionV2

        world = MultiMasterPiProductionV2(seed=11, render=False)
        try:
            results = world.act_parallel([
                {
                    "robot_id": "r1", "action": "team_beam_transport",
                    "mission_id": "beam_transport_v1", "role": "carrier_left",
                },
                {
                    "robot_id": "r2", "action": "team_beam_transport",
                    "mission_id": "beam_transport_v1", "role": "scout",
                },
                {
                    "robot_id": "r3", "action": "team_beam_transport",
                    "mission_id": "beam_transport_v1", "role": "carrier_right",
                },
            ])
            self.assertTrue(all(result.ok for result in results.values()), results)
            beam = world.beam_state()
            self.assertEqual(beam["status"], "SUCCESS")
            self.assertTrue(beam["evaluation"]["success"])
            self.assertLess(beam["linear_speed_mps"], 0.01)
            self.assertFalse(any(beam["constraints_active"].values()))
            phases = [item["phase"] for item in beam["trace"]]
            self.assertIn("dual_grasp_verified", phases)
            self.assertIn("dual_lift_verified", phases)
            self.assertIn("cooperative_carry_complete", phases)
            self.assertEqual(phases[-1], "mission_complete")
            self.assertTrue(world.last_parallel_timing["joint_physics"])
            self.assertGreater(world.robot("r2").base_xyz()[0], 0.80)
        finally:
            world.close()


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
