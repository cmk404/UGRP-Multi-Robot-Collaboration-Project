"""Task executive: deterministic safety contracts plus an optional rule baseline."""

from __future__ import annotations

from dataclasses import dataclass

from .state import WorldState
from .goals import GoalSpec


@dataclass(frozen=True)
class ExecutiveDecision:
    allowed: bool
    failure_code: str | None = None
    required_state: str | None = None
    recovery: str | None = None
    reason: str | None = None


class TaskExecutive:
    """Checks skill preconditions without promoting UNKNOWN into certainty.

    Design intent: :meth:`check` is the runtime safety/validity boundary.
    :meth:`plan_calls` is retained only for an explicit deterministic
    baseline/debug mode; the default embodied VLM path must not call it to
    choose actions or recoveries on the model's behalf.

    `strict_pick_preconditions` is enabled for physical REAL execution. Generic
    harness/SIM dry-runs keep their historical permissive UNKNOWN semantics.
    """

    def __init__(self, *, strict_pick_preconditions: bool = False) -> None:
        self.strict_pick_preconditions = bool(strict_pick_preconditions)

    def plan(self, goal: GoalSpec, tool_names: list[str], state: WorldState | None = None) -> list[str]:
        """Compatibility view of :meth:`plan_calls` containing only tool names."""
        return [name for name, _args in self.plan_calls(goal, tool_names, state)]

    def plan_calls(
        self,
        goal: GoalSpec,
        tool_names: list[str],
        state: WorldState | None = None,
    ) -> list[tuple[str, dict[str, str]]]:
        """Compile a goal to generic calls without encoding colour in tool names.

        The legacy SIM/action-file shape is retained when it does not expose a
        generic ``place`` tool.  The REAL action file exposes that tool and is
        therefore always compiled with explicit runtime identities.
        """
        names = set(tool_names)
        if goal.kind == "SCAN":
            plan = ["scan_world"]
            return [(step, {}) for step in plan] if all(step in names for step in plan) else []

        color_by_subject = {"RED_BLOCK": "red", "BLUE_BLOCK": "blue", "YELLOW_BLOCK": "yellow"}
        color = color_by_subject.get(goal.subject)
        held_color = state.grasp.held_object_color if state is not None else None
        held = state is not None and state.grasp.state in {"PROBABLE_HELD", "HELD"}
        if color is None and held:
            color = held_color
        generic_real = "place" in names and all(step in names for step in ("search", "track", "approach", "pick"))

        destination_color = color_by_subject.get(goal.target)
        release_first = (
            goal.kind in {"GRASP", "LIFT"}
            and held
            and color is not None
            and held_color != color
            and "put_down" in names
        )
        if goal.kind == "PLACE" and held:
            # A follow-up placement must never restart acquisition while a
            # grasp may be present.  Identity disagreement is left unplanned
            # for the caller to report instead of risking a wrong release.
            if color is None or destination_color is None or (held_color is not None and color != held_color):
                return []
            if destination_color == color:
                return []
            if generic_real and "place" in names:
                plan = []
                if "search_destination" in names:
                    plan.append(("search_destination", {"target_color": color, "destination_color": destination_color}))
                plan.append(("place", {"target_color": color, "destination_color": destination_color}))
                return plan
            legacy = f"place_on_{destination_color}"
            return [(legacy, {})] if legacy in names else []

        if color is None:
            # REAL currently exposes only one generic manipulation pipeline: the
            # red-block search/track/approach/pick stack. Follow-up commands such
            # as "집어보라고" intentionally omit the colour after the target was
            # already established visually. In that narrow case, compile the
            # known red pipeline instead of dropping back to free-form LLM tool
            # selection. Never apply this shortcut when colour-specific
            # alternatives are present.
            generic_red_only = (
                goal.kind in {"GRASP", "LIFT"}
                and all(base in names for base in ("search", "track", "approach", "pick"))
                and not any(
                    f"{base}_{c}" in names
                    for base in ("search", "track", "approach", "pick")
                    for c in ("blue", "yellow")
                )
            )
            if generic_red_only:
                color = "red"
            else:
                return []

        if generic_real:
            def generic(base: str) -> tuple[str, dict[str, str]]:
                return base, {"target_color": color}

            def acquisition_bases() -> list[str]:
                # Positive current camera evidence can safely remove redundant
                # non-destructive acquisition stages.  Never skip ``approach``:
                # REAL pick consumes its short-lived FK/IK handoff, so even a
                # visually PREGRASP-looking frame is not sufficient by itself.
                bases = ["search", "track", "approach", "pick"]
                if state is None or release_first:
                    return bases
                target = state.target
                memory = state.spatial_memory.get(color) if isinstance(state.spatial_memory, dict) else None
                image_xy = memory.get("image_xy") if isinstance(memory, dict) else None
                memory_visible = isinstance(memory, dict) and memory.get("visible") is True
                memory_centered = (
                    isinstance(image_xy, (list, tuple))
                    and len(image_xy) >= 1
                    and isinstance(image_xy[0], (int, float))
                    and abs(float(image_xy[0]) - 0.5) <= 0.065
                )
                # set_goal_context() changes selected_color before planning, so
                # target.selected_color alone cannot prove that the cached target
                # fields belong to this requested colour. Require the current
                # per-colour camera memory too before skipping a stage.
                if memory_visible and target.visible is True:
                    bases.remove("search")
                    if target.centered is True and memory_centered:
                        bases.remove("track")
                return bases

            if goal.kind == "SEARCH":
                return [generic("search")]
            if goal.kind in {"GRASP", "LIFT"}:
                plan: list[tuple[str, dict[str, str]]] = []
                if release_first:
                    plan.append(("put_down", {}))
                plan.extend(generic(base) for base in acquisition_bases())
                if "carry" in names:
                    plan.append(("carry", {}))
                return plan
            if goal.kind == "PLACE":
                if destination_color is None or destination_color == color:
                    return []
                plan = [generic(base) for base in acquisition_bases()]
                if "carry" in names:
                    plan.append(("carry", {}))
                if "search_destination" in names:
                    plan.append(("search_destination", {"target_color": color, "destination_color": destination_color}))
                plan.append(("place", {"target_color": color, "destination_color": destination_color}))
                return plan
            return []

        def choose(base: str, c: str) -> str | None:
            # Keep the established red primitive names for REAL/backward
            # compatibility. Blue/yellow can never fall through to those aliases.
            if c == "red" and base in names:
                return base
            specific=f"{base}_{c}"
            if specific in names:
                return specific
            return None

        search=choose("search",color)
        track=choose("track",color)
        approach=choose("approach",color)
        pick=choose("pick",color)

        if goal.kind == "SEARCH":
            plan=[search] if search else []
        elif goal.kind in {"GRASP","LIFT"}:
            if not all((search,track,approach,pick)):
                return []
            plan=[search,track,approach,pick]
            if "carry" in names:
                plan.append("carry")
        elif goal.kind == "PLACE":
            target_color=color_by_subject.get(goal.target)
            if target_color is None or target_color == color:
                return []
            place=f"place_on_{target_color}"
            if place not in names or not all((search,track,approach,pick)):
                return []
            plan=[]
            mapped=f"map_{target_color}"
            if mapped in names:
                plan.append(mapped)
            plan.extend([search,track,approach,pick])
            if "carry" in names:
                plan.append("carry")
            plan.append(place)
        else:
            return []
        return [(p, {}) for p in plan if p] if plan and all(p in names for p in plan if p) else []

    def check(self, skill: str, state: WorldState, args: dict | None = None) -> ExecutiveDecision:
        args = args or {}
        if state.task.phase == "CARRY_LOST" and skill not in {"observe_scene", "stop_motion"}:
            return ExecutiveDecision(
                False,
                "CARRY_LOST_DURING_DELIVERY",
                "grasp.state=EMPTY; operator recovery required",
                None,
                "the carried object was reported lost during delivery; automatic destination, motion, and reacquisition actions are stopped",
            )
        target = state.target
        base_skill = next((base for base in ("search","track","approach","pick") if skill == base or skill.startswith(base + "_")), skill)
        color_suffix = next(("_" + c for c in ("red","blue","yellow") if skill.endswith("_" + c)), "")
        if skill == "search_destination":
            if state.grasp.state not in {"PROBABLE_HELD", "HELD"}:
                return ExecutiveDecision(False, "GRIPPER_EMPTY", "grasp.state=PROBABLE_HELD|HELD", None, "destination search is only valid while carrying a block")
            requested = args.get("target_color")
            held_color = state.grasp.held_object_color
            if requested is not None and held_color is not None and requested != held_color:
                return ExecutiveDecision(False, "HELD_OBJECT_IDENTITY_MISMATCH", "grasp.held_object_color=target_color", None, "destination search target_color must match the carried object")
            destination = args.get("destination_color")
            if destination not in {"red", "blue", "yellow"} or destination == requested:
                return ExecutiveDecision(False, "PLACEMENT_DESTINATION_INVALID", "destination_color!=target_color", None, "destination search requires a distinct supported destination color")
            return ExecutiveDecision(True)
        if (
            self.strict_pick_preconditions
            and base_skill in {"search", "track", "approach", "pick"}
            and state.grasp.state in {"PROBABLE_HELD", "HELD"}
        ):
            return ExecutiveDecision(
                False, "OBJECT_ALREADY_CARRIED", "grasp.state=EMPTY|UNKNOWN", None,
                "target acquisition/manipulation is blocked while the gripper may already carry an object",
            )
        if skill == "put_down":
            if state.grasp.state not in {"PROBABLE_HELD", "HELD"}:
                return ExecutiveDecision(
                    False, "GRIPPER_EMPTY", "grasp.state=PROBABLE_HELD|HELD", None,
                    "there is no carried object to put down",
                )
            return ExecutiveDecision(True)

        if base_skill == "search":
            return ExecutiveDecision(True)

        # track/approach are bounded visual controllers and may reacquire from
        # UNKNOWN state themselves. Pick is destructive, so it requires a
        # positively confirmed current target before its bounded near-field
        # alignment and grasp sequence.
        if base_skill in {"track", "approach"} and target.visible is False:
            return ExecutiveDecision(
                False, "TARGET_NOT_VISIBLE", "target.visible=true", f"search{color_suffix}",
                "target is not visible in the robot camera",
            )
        if base_skill == "pick" and (
            target.visible is False
            or (self.strict_pick_preconditions and target.visible is not True)
        ):
            return ExecutiveDecision(
                False, "TARGET_NOT_VISIBLE", "target.visible=true", f"search{color_suffix}",
                "target visibility is not positively confirmed in the robot camera",
            )

        if base_skill == "approach":
            return ExecutiveDecision(True)

        if base_skill == "track":
            return ExecutiveDecision(True)

        if base_skill == "pick":
            # PREGRASP_READY is now only the planner-side causal token that a
            # successful coarse approach left the target visible, centered and
            # inside the arm-reach corridor. Pick performs its own stopped
            # multi-frame measurement and IK; no persisted FK/IK plan is required.
            if self.strict_pick_preconditions and state.task.phase != "PREGRASP_READY":
                return ExecutiveDecision(
                    False, "PREGRASP_PLAN_MISSING", "task.phase=PREGRASP_READY",
                    f"approach{color_suffix}",
                    "fresh coarse pregrasp reach handoff is required before pick; run approach immediately before pick",
                )
            valid_ranges = {"PREGRASP"} if self.strict_pick_preconditions else {"UNKNOWN", "PREGRASP"}
            if target.range_class not in valid_ranges:
                return ExecutiveDecision(
                    False, "TARGET_TOO_FAR", "target.range_class=PREGRASP", f"approach{color_suffix}",
                    f"target range is not positively confirmed PREGRASP (current={target.range_class})",
                )
            centered_bad = target.centered is not True if self.strict_pick_preconditions else target.centered is False
            if centered_bad:
                return ExecutiveDecision(
                    False, "TARGET_NOT_CENTERED", "target.centered=true", f"track{color_suffix}",
                    "target centering is not positively confirmed",
                )
            return ExecutiveDecision(True)

        if skill == "carry":
            # No force sensor exists on the real current platform. Do not invent a grasp fact.
            # Carry is allowed after a pick attempt; its physical outcome remains UNKNOWN until sensed.
            last = state.last_action.name
            if last not in {None, "pick", "carry"} and state.grasp.state == "EMPTY":
                return ExecutiveDecision(
                    False, "GRIPPER_EMPTY", "grasp.state!=EMPTY", None,
                    "current transferable state says the gripper is empty",
                )
            return ExecutiveDecision(True)

        if skill == "place" or skill in {"place_on_red", "place_on_yellow", "place_on_blue"}:
            if state.grasp.state not in {"PROBABLE_HELD", "HELD"}:
                return ExecutiveDecision(
                    False, "GRASP_NOT_CONFIRMED", "grasp.state=PROBABLE_HELD|HELD", None,
                    "sensor evidence does not confirm that the requested block is being carried",
                )
            requested = args.get("target_color")
            held_color = state.grasp.held_object_color
            if requested is not None and requested != held_color:
                return ExecutiveDecision(
                    False, "HELD_OBJECT_IDENTITY_MISMATCH", "grasp.held_object_color=target_color", None,
                    "the requested source object does not match the object currently held",
                )
            if skill == "place":
                destination = args.get("destination_color")
                if destination not in {"red", "blue", "yellow"} or destination == requested:
                    return ExecutiveDecision(
                        False, "PLACEMENT_DESTINATION_INVALID", "destination_color!=target_color", None,
                        "placement requires a distinct supported destination color",
                    )
            return ExecutiveDecision(True)

        return ExecutiveDecision(True)

    def rejection_payload(self, skill: str, decision: ExecutiveDecision) -> dict:
        return {
            "ok": False,
            "skill": skill,
            "command_status": "REJECTED",
            "execution_status": "UNKNOWN",
            "outcome_status": "NOT_ACHIEVED",
            "failure_code": decision.failure_code,
            "required_state": decision.required_state,
            "recommended_recovery": decision.recovery,
            "reason": decision.reason or "skill precondition failed",
        }
