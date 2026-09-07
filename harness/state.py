"""Transferable robot world state built only from actor-visible sensor facts."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


def _ema(old: float | None, new: float | None, alpha: float = 0.45) -> float | None:
    if new is None:
        return old
    if old is None:
        return float(new)
    return (1.0 - alpha) * old + alpha * float(new)


@dataclass
class TargetState:
    selected_color: str | None = None  # red | blue | yellow; planner-selected identity
    visible: bool | None = None
    confidence: float = 0.0
    cx: float | None = None
    cy: float | None = None
    area_ratio: float | None = None
    filtered_cx: float | None = None
    filtered_cy: float | None = None
    filtered_area_ratio: float | None = None
    visible_streak: int = 0
    centered: bool | None = None
    centered_streak: int = 0
    range_class: str = "UNKNOWN"  # UNKNOWN | FAR | NEAR | PREGRASP


@dataclass
class GraspState:
    held_object_color: str | None = None
    state: str = "UNKNOWN"  # UNKNOWN | EMPTY | PROBABLE_HELD | HELD
    confidence: float = 0.0
    visual_support: bool | None = None
    held_streak: int = 0


@dataclass
class ActionState:
    name: str | None = None
    command_status: str = "UNKNOWN"  # ACCEPTED | REJECTED | UNKNOWN
    execution_status: str = "UNKNOWN"  # COMPLETED | FAILED | UNKNOWN
    outcome_status: str = "UNKNOWN"  # ACHIEVED | NOT_ACHIEVED | UNKNOWN
    failure_code: str | None = None
    required_state: str | None = None
    recommended_recovery: str | None = None


@dataclass
class TaskState:
    phase: str = "UNKNOWN"
    destination_color: str | None = None


@dataclass
class WorldState:
    target: TargetState = field(default_factory=TargetState)
    grasp: GraspState = field(default_factory=GraspState)
    last_action: ActionState = field(default_factory=ActionState)
    task: TaskState = field(default_factory=TaskState)
    spatial_memory: dict[str, dict[str, Any]] = field(default_factory=dict)
    semantic_map: dict[str, Any] = field(default_factory=dict)
    observation_count: int = 0

    def public(self) -> dict[str, Any]:
        """State safe to show the planner. Contains no simulator ground truth."""
        return asdict(self)


class StateEstimator:
    """Temporal estimator over transferable camera/tool observations."""

    def __init__(self, state: WorldState | None = None):
        self.state = state or WorldState()
        # Design intent: `target.centered` is the public alignment contract, not
        # merely `cx ~= 0.5`. The physical tracker may validly finish at a safe
        # pan/tilt servo limit while the target remains off the optical centre.
        # Keep that actor-visible TRACK postcondition latched across passive
        # camera refreshes; invalidate it only when later evidence/action can
        # actually break the alignment handoff.
        self._alignment_verified = False
        # A multi-frame public skill can prove target loss more strongly than a
        # single passive RGB sample. Keep that loss latched until a successful
        # search/track explicitly re-acquires the target.
        self._visibility_lost_verified = False
        # Successful approach latches a verified arm-reach fact. Pick re-measures
        # metric range before grasp, but passive RGB must not erase this stronger
        # stopped-controller result. Base motion or a failed approach invalidates it.
        self._arm_reach_verified = False

    def reset(self) -> WorldState:
        """Drop all temporal evidence at a simulation episode boundary."""
        self.state = WorldState()
        self._alignment_verified = False
        self._visibility_lost_verified = False
        self._arm_reach_verified = False
        return self.state

    def set_goal_context(self, *, target_color: str | None = None, destination_color: str | None = None) -> WorldState:
        """Record identities named by the user without asserting sensor truth."""
        if target_color in {"red", "blue", "yellow"}:
            self.state.target.selected_color = target_color
        if destination_color in {"red", "blue", "yellow"}:
            self.state.task.destination_color = destination_color
        return self.state

    def update_tool_result(self, payload: dict[str, Any]) -> WorldState:
        value = payload.get("result") if isinstance(payload.get("result"), dict) else payload
        if not isinstance(value, dict):
            return self.state

        target_color = _color_or_none(value.get("target_color"))
        destination_color = _color_or_none(value.get("destination_color"))
        if target_color is not None:
            self.state.target.selected_color = target_color
        if destination_color is not None:
            self.state.task.destination_color = destination_color

        # With no chassis odometry, any translation/rotation changes the robot
        # reference frame. Never let a later destructive skill consume target
        # centering/range or metric positions measured before that motion. The
        # loop will immediately attach a fresh post-action camera observation.
        if value.get("chassis_motion") is True:
            self._alignment_verified = False
            self._arm_reach_verified = False
            t = self.state.target
            t.visible = None
            t.confidence = 0.0
            t.cx = t.cy = t.area_ratio = None
            t.filtered_cx = t.filtered_cy = t.filtered_area_ratio = None
            t.visible_streak = 0
            t.centered = None
            t.centered_streak = 0
            t.range_class = "UNKNOWN"
            self.state.spatial_memory = {}
            self.state.semantic_map = {}
            # External base motion between public skills invalidates the
            # physical approach->pick handoff. Pick may itself move the base
            # after consuming that handoff; this state update occurs only after
            # the whole skill returns, so clearing the planner token here is safe.
            self.state.task.phase = "UNKNOWN"

        vision = value.get("target_vision")
        if not isinstance(vision, dict):
            vision = value.get("camera_red")
        if not isinstance(vision, dict):
            vision = value.get("vision")
        if isinstance(vision, dict):
            self.update_vision(vision)
        memory = value.get("spatial_memory")
        if isinstance(memory, dict):
            # Adapter memory contains only camera/FK/odometry-derived facts.
            self.state.spatial_memory = {str(k): dict(v) for k, v in memory.items() if isinstance(v, dict)}
        detections = value.get("detections")
        metric_estimates = value.get("metric_estimates")
        if isinstance(detections, dict):
            self.update_scene_memory(
                detections,
                metric_estimates=metric_estimates if isinstance(metric_estimates, dict) else None,
            )

        action = self.state.last_action
        action.name = str(value.get("skill") or value.get("action") or payload.get("tool") or "") or None
        action.command_status = str(value.get("command_status") or ("ACCEPTED" if payload.get("ok") is not False else "UNKNOWN"))
        action.execution_status = str(value.get("execution_status") or ("COMPLETED" if payload.get("ok") is not False else "FAILED"))
        action.outcome_status = str(value.get("outcome_status") or "UNKNOWN")
        action.failure_code = _none_or_str(value.get("failure_code"))
        action.required_state = _none_or_str(value.get("required_state"))
        action.recommended_recovery = _none_or_str(value.get("recommended_recovery"))
        action_base_for_visibility = next(
            (base for base in ("search", "track") if action.name == base or (action.name or "").startswith(base + "_")),
            action.name,
        )
        if (
            action_base_for_visibility == "track"
            and action.outcome_status == "NOT_ACHIEVED"
            and action.failure_code == "TARGET_NOT_VISIBLE"
        ):
            self._visibility_lost_verified = True
            self.state.target.visible = False
            self.state.target.confidence = 0.95
            self.state.target.centered = False
            self.state.target.centered_streak = 0
            self.state.target.range_class = "UNKNOWN"
        elif action_base_for_visibility in {"search", "track"} and action.outcome_status == "ACHIEVED":
            self._visibility_lost_verified = False
        self._update_phase(action.name, action.outcome_status)
        carry_evidence = value.get("carry_evidence")
        if action.name == "observe_scene" and isinstance(carry_evidence, dict):
            self.reconcile_probable_carry(carry_evidence)
        # Explicit skill postconditions override filter lag, but only when the
        # actor-visible adapter says the postcondition was deterministically achieved.
        action_base = next((base for base in ("search", "track", "approach", "pick", "place") if action.name == base or (action.name or "").startswith(base + "_")), action.name)
        if action_base == "approach":
            # A successful public approach means the target is inside the
            # coarse pregrasp/arm-reach corridor. Pick re-measures and solves IK
            # from the stopped camera view; no approach-created grasp plan exists.
            self._arm_reach_verified = action.outcome_status == "ACHIEVED"
        elif action_base in {"pick", "place", "put_down", "search_destination"}:
            self._arm_reach_verified = False
        # Any skill that can change gaze/base geometry invalidates the previous
        # TRACK handoff unless this very result positively re-certifies it. A
        # read-only observe_scene does not invalidate alignment.
        if action_base in {"search", "approach", "pick", "place", "put_down", "search_destination"} or value.get("chassis_motion") is True:
            self._alignment_verified = False
        if action_base == "track" and action.outcome_status != "ACHIEVED":
            self._alignment_verified = False
        if action.outcome_status == "ACHIEVED":
            if action_base == "approach":
                self.state.target.range_class = "PREGRASP"
                if value.get("center_verified") is True:
                    self._alignment_verified = True
                    self.state.target.centered = True
                    self.state.target.centered_streak = max(1, self.state.target.centered_streak)
            elif action_base == "track":
                self._alignment_verified = True
                self.state.target.centered = True
                self.state.target.centered_streak = max(1, self.state.target.centered_streak)
            elif action_base == "search":
                self.state.target.visible = True
        self._update_grasp(value, action)
        return self.state

    def reconcile_probable_carry(self, evidence: dict[str, Any] | None) -> bool:
        """Reconcile weak carry belief against read-only Pi causal evidence.

        ``HELD`` is never modified here.  For ``PROBABLE_HELD`` an unavailable
        transport also leaves state untouched (fail closed).  A known missing,
        stale, or invalid precision-pick handoff downgrades the weak belief to
        UNKNOWN so a new acquisition may be planned without claiming the
        gripper is definitely empty.
        """
        grasp = self.state.grasp
        if grasp.state != "PROBABLE_HELD" or not isinstance(evidence, dict):
            return False
        if evidence.get("known") is not True:
            return False
        if evidence.get("valid") is True:
            color = _color_or_none(evidence.get("target_color"))
            if color is not None:
                grasp.held_object_color = color
            grasp.confidence = max(grasp.confidence, 0.65)
            grasp.visual_support = True
            grasp.held_streak = max(1, grasp.held_streak)
            return False
        grasp.state = "UNKNOWN"
        grasp.confidence = 0.0
        grasp.visual_support = False
        grasp.held_streak = 0
        grasp.held_object_color = None
        if self.state.task.phase in {"LIFT_VERIFY", "RELEASE"}:
            self.state.task.phase = "UNKNOWN"
        return True

    def accept_operator_empty_gripper(self) -> bool:
        """Accept an explicit operator correction only for weak/unknown hold state.

        A future positively sensed HELD state is not silently overridden by text.
        Current REAL pickup-site disappearance produces PROBABLE_HELD, so an
        operator who can directly see the empty jaws can clear that stale belief.
        """
        grasp = self.state.grasp
        if grasp.state == "HELD":
            return False
        grasp.state = "EMPTY"
        grasp.confidence = 0.95
        grasp.visual_support = False
        grasp.held_streak = 0
        grasp.held_object_color = None
        if self.state.task.phase == "LIFT_VERIFY":
            self.state.task.phase = "UNKNOWN"
        return True

    def update_vision(self, vision: dict[str, Any]) -> WorldState:
        t = self.state.target
        visible = vision.get("visible")
        if not isinstance(visible, bool):
            return self.state
        self.state.observation_count += 1
        if visible and self._visibility_lost_verified:
            # Keep the stronger multi-frame loss fact until a public acquisition
            # skill positively re-certifies visibility. Preserve raw geometry in
            # spatial memory separately; do not turn one transient blob into an
            # executable track/pick precondition.
            t.visible = False
            t.confidence = max(t.confidence, 0.95)
            t.centered = False
            t.centered_streak = 0
            t.range_class = "UNKNOWN"
            return self.state
        t.visible = visible
        if not visible:
            self._alignment_verified = False
            t.visible_streak = 0
            t.cx = None
            t.cy = None
            t.area_ratio = None
            t.centered = False
            t.centered_streak = 0
            t.range_class = "UNKNOWN"
            t.confidence = max(0.0, t.confidence * 0.55)
            return self.state

        t.visible_streak += 1
        cx = _float_or_none(vision.get("cx"))
        cy = _float_or_none(vision.get("cy"))
        area = _float_or_none(vision.get("area_ratio"))
        t.cx, t.cy, t.area_ratio = cx, cy, area
        t.filtered_cx = _ema(t.filtered_cx, cx)
        t.filtered_cy = _ema(t.filtered_cy, cy)
        t.filtered_area_ratio = _ema(t.filtered_area_ratio, area)
        t.confidence = min(0.99, 0.62 + 0.08 * min(t.visible_streak, 4))

        if t.filtered_cx is not None:
            image_centered = abs(t.filtered_cx - 0.5) <= 0.065
            # A successful TRACK can be a servo-limit lock: the physical
            # controller has positively verified alignment even when raw cx is
            # not 0.5. Passive RGB refresh must not erase that stronger fact.
            centered = image_centered or self._alignment_verified
            t.centered = centered
            t.centered_streak = t.centered_streak + 1 if centered else 0
        if t.filtered_cy is not None:
            # A successful precision approach created the calibrated pick handoff.
            # Preserve its PREGRASP range classification across passive RGB refresh;
            # passive perception must not erase a stronger stopped-controller fact.
            if self._arm_reach_verified:
                t.range_class = "PREGRASP"
            elif t.filtered_cy >= 0.60:
                t.range_class = "PREGRASP"
            elif t.filtered_cy >= 0.43:
                t.range_class = "NEAR"
            else:
                t.range_class = "FAR"
        return self.state


    def update_scene_memory(
        self,
        detections: dict[str, Any],
        *,
        source: str = "robot_camera_rgb",
        metric_estimates: dict[str, dict[str, Any]] | None = None,
    ) -> WorldState:
        """Remember camera-visible landmarks without pretending REAL has metric XYZ.

        The memory is intentionally observation-relative until camera/FK/base
        calibration is verified on hardware. Metric SIM memory still arrives
        through update_tool_result() and is not replaced by this path.
        """
        selected_color = self.state.target.selected_color or "red"
        selected = detections.get(selected_color) if isinstance(detections, dict) else None
        if isinstance(selected, dict):
            self.update_vision(selected)
        else:
            self.state.observation_count += 1
        obs_id = self.state.observation_count
        for entry in self.state.spatial_memory.values():
            if entry.get("source") == source:
                entry["visible"] = False
                entry["age_observations"] = max(0, obs_id - int(entry.get("last_seen_observation", obs_id)))
        for color in ("red", "yellow", "blue"):
            det = detections.get(color) if isinstance(detections, dict) else None
            if not isinstance(det, dict) or det.get("visible") is not True:
                continue
            old = self.state.spatial_memory.get(color) or {}
            seen = int(old.get("seen_count", 0)) + 1
            cx = _float_or_none(det.get("cx"))
            cy = _float_or_none(det.get("cy"))
            area = _float_or_none(det.get("area_ratio"))
            # Approximate camera-relative image bearing for visualization only.
            # 60 deg is a placeholder FOV, explicitly marked uncalibrated.
            bearing = None if cx is None else (cx - 0.5) * 60.0
            entry = {
                "image_xy": None if cx is None or cy is None else [cx, cy],
                "area_ratio": area,
                "camera_bearing_hint_deg": bearing,
                "metric_position_available": False,
                "calibrated": False,
                "reference_frame": "camera_at_observation",
                "confidence": min(0.95, 0.62 + 0.07 * min(seen, 4)),
                "visible": True,
                "seen_count": seen,
                "last_seen_observation": obs_id,
                "age_observations": 0,
                "relation": str(old.get("relation") or "OBSERVED"),
                "source": source,
            }
            metric = metric_estimates.get(color) if isinstance(metric_estimates, dict) else None
            if isinstance(metric, dict) and metric.get("metric_position_available") is True:
                entry.update(metric)
                # Visibility aging for REAL camera memory remains keyed to the
                # observation source even when metric metadata is attached.
                entry["source"] = source
                entry["confidence"] = max(float(entry["confidence"]), 0.86)
            self.state.spatial_memory[color] = entry
        self._refresh_real_semantic_map(obs_id)
        return self.state

    def _refresh_real_semantic_map(self, observation_id: int) -> None:
        """Build a robot-base-relative semantic snapshot for REAL UI.

        This is not a persistent world map yet: without chassis odometry every
        metric point belongs to the robot base frame at its observation time.
        """
        zones: list[dict[str, Any]] = []
        for color in ("blue", "yellow"):
            entry = self.state.spatial_memory.get(color)
            if not isinstance(entry, dict):
                continue
            xy = entry.get("position_xy")
            if entry.get("metric_position_available") is not True or not isinstance(xy, list) or len(xy) != 2:
                continue
            zones.append({
                "name": f"{color.upper()} STACK TARGET",
                "center_xy": [float(xy[0]), float(xy[1])],
                "size_xy": [0.10, 0.10],
                "target_color": color,
                "kind": "stack_target",
                "confidence": float(entry.get("confidence") or 0.0),
                "reference_frame": "robot_base_at_observation",
            })
        self.state.semantic_map = {
            "reference_frame": "robot_base_at_observation",
            "robot_xy": [0.0, 0.0],
            "persistent_world_map": False,
            "odometry_available": False,
            "drop_zones": zones,
            "obstacles": [],
            "obstacle_provider": {
                "status": "unconfigured",
                "backend": None,
                "metric_depth_available": False,
            },
            "scan": {
                "observation_id": int(observation_id),
                "source": "robot_camera_rgb+commanded_arm_fk",
            },
        }

    def _update_phase(self, name: str | None, outcome: str) -> None:
        if not name:
            return
        base_name = next((base for base in ("search", "track", "approach", "pick", "place") if name == base or name.startswith(base + "_")), name)
        phases = {
            "search": "TARGET_ACQUISITION",
            "track": "ALIGN",
            "approach": "PREGRASP",
            "pick": "GRASP_VERIFY",
            "carry": "LIFT_VERIFY",
            "put_down": "RELEASE",
        }
        self.state.task.phase = phases.get(base_name, self.state.task.phase)
        if base_name == "approach" and outcome == "ACHIEVED":
            # Successful coarse approach creates a causal arm-reach handoff.
            # Passive RGB may refresh visibility but must not erase this phase
            # while the chassis/arm remain unchanged.
            self.state.task.phase = "PREGRASP_READY"

    def _update_grasp(self, value: dict[str, Any], action: ActionState) -> None:
        # Only use transferable evidence. Simulator contact/height truth is intentionally ignored.
        # REAL pick may provide positive grasp evidence in the future, but
        # pickup-site disappearance by itself is only PROBABLE_HELD. Motor
        # completion or a clear floor must never become HELD truth.
        verified_pick = value.get("grasp_verified")
        hold = value.get("visual_hold")
        action_is_pick = action.name == "pick" or (action.name or "").startswith("pick_")
        delivery_actions = {"search_destination", "place", "place_on_red", "place_on_blue", "place_on_yellow", "stack_on"}
        failure_reason = str(value.get("reason") or "").lower()
        known_carry_loss = (
            action.name in delivery_actions
            and action.outcome_status == "NOT_ACHIEVED"
            and (
                action.failure_code == "CARRY_LOST_DURING_DELIVERY"
                # Compatibility aliases historically surfaced this actor-facing
                # driver fact as SKILL_FAILED. The wording is evidence from the
                # action result, not privileged SIM contact/pose truth.
                or "lost during delivery" in failure_reason
                # Shared place.py only opens the gripper after the release pose
                # is reached.  If it then reports that release completed but the
                # stack could not be verified, the object is no longer a valid
                # carried object even when the driver cannot say where it landed.
                or (action.name != "search_destination" and "release completed" in failure_reason)
            )
        )
        if known_carry_loss:
            # A delivery driver explicitly reporting loss is stronger than the
            # previous probable-hold inference. Do not leave stale carry state
            # that could authorize another destination search or manual motion.
            self.state.grasp.state = "EMPTY"
            self.state.grasp.confidence = 0.95
            self.state.grasp.visual_support = False
            self.state.grasp.held_streak = 0
            self.state.grasp.held_object_color = None
            self.state.task.phase = "CARRY_LOST"
        elif action_is_pick and verified_pick is True and action.outcome_status == "ACHIEVED":
            self.state.grasp.state = "HELD"
            self.state.grasp.confidence = max(self.state.grasp.confidence, 0.95)
            self.state.grasp.held_streak = max(2, self.state.grasp.held_streak)
            self.state.grasp.visual_support = True
            self.state.grasp.held_object_color = _color_or_none(value.get("target_color")) or self.state.target.selected_color
        elif action_is_pick and verified_pick is False:
            self.state.grasp.state = "EMPTY"
            self.state.grasp.confidence = max(self.state.grasp.confidence, 0.90)
            self.state.grasp.held_streak = 0
            self.state.grasp.visual_support = False
            self.state.grasp.held_object_color = None
        elif isinstance(hold, dict):
            probable = hold.get("probable")
            confidence = _float_or_none(hold.get("confidence")) or 0.0
            self.state.grasp.visual_support = bool(probable) if isinstance(probable, bool) else None
            if probable is True:
                self.state.grasp.held_streak += 1
                self.state.grasp.confidence = max(self.state.grasp.confidence, confidence)
                self.state.grasp.state = "HELD" if self.state.grasp.held_streak >= 2 and confidence >= .85 else "PROBABLE_HELD"
                self.state.grasp.held_object_color = _color_or_none(value.get("target_color")) or self.state.target.selected_color
            elif probable is False:
                self.state.grasp.held_streak = 0
                self.state.grasp.confidence = confidence
                if action_is_pick or action.name == "carry":
                    self.state.grasp.state = "EMPTY"
                    self.state.grasp.held_object_color = None
        elif action.name == "carry" and action.outcome_status == "ACHIEVED":
            # The carry contract only reports ACHIEVED after its driver-level
            # grasp/lift postcondition succeeds. SIM and REAL may verify that
            # postcondition differently; the executive consumes the same abstraction.
            self.state.grasp.state = "HELD"
            self.state.grasp.confidence = max(self.state.grasp.confidence, 0.95)
            self.state.grasp.held_streak = max(2, self.state.grasp.held_streak)
            self.state.grasp.visual_support = None
            self.state.grasp.held_object_color = self.state.target.selected_color
        elif (
            action.name == "put_down"
            and action.outcome_status == "ACHIEVED"
            and value.get("release_verified") is True
        ):
            self.state.grasp.state = "EMPTY"
            self.state.grasp.confidence = 0.99
            self.state.grasp.held_streak = 0
            self.state.grasp.visual_support = False
            self.state.grasp.held_object_color = None
            self.state.task.phase = "RELEASED"
        elif (
            action.name in {"place", "place_on_red", "place_on_yellow", "place_on_blue", "stack_on", "stage_base"}
            and action.outcome_status == "ACHIEVED"
            and value.get("place_verified") is True
        ):
            # Placement is physical truth only when the adapter explicitly says
            # its postcondition verifier passed. A bare exit-code success must
            # not empty the gripper or mark the task PLACED.
            self.state.grasp.state = "EMPTY"
            self.state.grasp.confidence = 0.95
            self.state.grasp.held_streak = 0
            self.state.grasp.visual_support = None
            self.state.grasp.held_object_color = None
            self.state.task.phase = "PLACED"
        elif action_is_pick and action.execution_status == "COMPLETED":
            # Motor sequence completion is not physical grasp success.
            self.state.grasp.state = "UNKNOWN"
            self.state.grasp.confidence = 0.0


def _float_or_none(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _none_or_str(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _color_or_none(value: Any) -> str | None:
    color = str(value).strip().lower() if value is not None else ""
    return color if color in {"red", "blue", "yellow"} else None
