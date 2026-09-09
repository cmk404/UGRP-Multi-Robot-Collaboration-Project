"""Gemini policy over one robot's wrist and fixed navigation RGB cameras."""
from __future__ import annotations

import base64
import copy
import hashlib
import json
import math
import re
from collections.abc import Mapping
from typing import Any

import cv2
import numpy as np

from harness.coela_modules import PlanningError
from harness.transport_context import compact_own_memory, current_placement_guidance, memory_needs_nav_comparison
from harness.visual_progress import VisualProgressHistory
from harness.navigation_evidence import navigation_evidence
from harness.navigation_temporal import NavigationTemporalEvidence


_OBS_FIELDS = {"robot_id", "frame_id", "sim_time", "image", "sha256", "camera", "actuator_state"}
_STATES = {"idle", "approaching", "ready_to_pick", "picking", "carrying", "releasing",
           "released", "grip_uncertain", "failure"}
_FORBIDDEN = {"pose", "position", "positions", "coordinates", "coords", "x", "y", "z",
              "world_state", "map", "seed", "layout", "obstacle_positions", "peer_positions",
              "target_pose", "target_coords", "global_state"}

PROMPT = """Choose ONE action; executor does not plan/correct. Sources: current WRIST_CAM/NAV_CAM, own memory/state/feedback, supplied earlier own RGB only. Never use map, seed, off-screen coordinates, hidden objects/peers, simulator state, or inventions. A/B/C=blue/green/yellow; cargo=cyan, obstacle=orange, peer=magenta; turn +left/-right.

Separate proposal/execution/RGB: previous_model_action may be rejected; last_decision=disposition; last_macro=actual control/interruption; PREVIOUS_NAV predates it; nav_evidence=current, feedback=past. Pixels/absent or out-of-view detections prove no distance, identity, arrival, safety, passage, clearance.

navigation_temporal=own RGB+executed macros; time is not distance/passage. executed_since_forward counts turns/sign changes. potential_revisited_blocked_view=similar view without forward, not same place/object. Recurring clear/blocked views plus reversals may cycle. Evidence chooses no action/side.

forward_stop_check.vetoed=true or OWN_RGB_DRIVE_BLOCKED forbids fwd>0: zero-forward RGB turn/wait. Clear permits consideration, not safety. Own-RGB guard checks <=.25s, may interrupt, cannot guarantee safety.

drive: fwd 0..0.15, turn -0.2..0.2, duration >0..4s; wait >0..1s. fwd/turn magnitudes dimensionless; inspect new RGB. Broad aligned clear: fwd=.15 for 3-4s; clear curve: forward+turn 2-3s. Near hazards/fine placement use subsecond; large turn may use |turn|=.2 for 2-3s. Examples prove no safety/arrival.

Budget never weakens safety/grip/inside; terminal_input_reserve protects release+finish. Before input_tokens_remaining < terminal_input_reserve+2*next_request_estimate, navigate deliberately. Repeated TARGET_FOOTPRINT_OUTSIDE_VISIBLE_ZONE_MARGIN: change alignment/duration, not micro-drives; one-sided floor is not centered. current_placement_guidance comes from exact own images/PWM, not safe routing: entry_bearing_deg robot-frame 0=forward,+left,-right; estimated_entry_distance_cm separates travel/fine alignment. On a broad aligned clear corridor, make useful forward/curved progress, then inspect. During bypass, translate sufficiently before goalward return when current RGB/non-veto permits; respect active bypass. Steer into visible interior. No boundary fit: improve view; do not assume progress. WRIST need not be all zone; release footprint must be inside. Release only on fresh inside.

Every carrying reply needs Korean navigation_note {"observed":"current RGB <=100 chars","maneuver":"left|right|forward|wait|unknown","resume_when":"next visible condition <=100 chars"}; bypass adds "bypass_side":"left|right","bypass_until":"forward-camera condition". Side is obstacle-relative, not immediate turn; choose from RGB. Commitment persists; only bypass_side=none clears. Clear only on NEW image after executed goalward turn+visible destination approach+current non-veto. Goal/FOV/time/proposed turn insufficient; bypass_until cannot look behind. After forward reassess goal. If blocker returns, retain/restore bypass and, when RGB permits, advance materially farther before retest. Avoid turn alternation/endless travel away. You choose action/side.

approach/pick use wrist RGB, not destination travel; pick only ready_to_pick; no approach carrying. grip_uncertain: check_grip strict left/right/home inspection or wait for visible occlusion; no drive/release. carrying latest_placement=inside: release now without centering, using fresh before_release inside. After release stop; finish needs fresh WRIST/NAV+released-stage inside. outside allows approach->pick->drive->release; uncertain proves neither success nor loss. DESTINATION_SQUARE_UNDERCONSTRAINED=unseen boundaries, not more forward: inspect another edge/corner; reacquire by heading, never blind advance. Require fresh inside; respect recovery limits.

RECENT_BLOCKED_NAV=earlier own RGB veto. Clear after turn/no forward may hide it; compare before goal alignment. Current clear may support guarded translation; history does not veto now. 통신은 이번 단계에
없으며 invent nothing unseen.

Return one JSON object: top-level action+reason, plus navigation_note carrying, e.g. {"action":{"kind":"drive","fwd":0.15,"turn":0,"duration":4},"reason":"전방 통로가 열려 전진한다.","navigation_note":{"observed":"전방 통로가 열림","maneuver":"forward","resume_when":"다음 화면에서 재확인"}}. Never top-level kind/fwd/turn/duration. reason<=200 chars; notes concise Korean from current images; enums exact. action.kind=approach|pick|check_grip|release|finish|drive above|wait with duration. 허용된 navigation_note 외 추가 필드나 메시지를 넣지 마라.
"""

_COMMUNICATION_PROMPTS = {
    "status": """첫 idle 호출에서는 네가 맡은 화물과 목적지, 시작 의도를 outgoing_message로 직접 알려라.
그 뒤에는 보이는 장애물이나 동료와의 조우, 상태 변화가 동료 행동에 영향을 줄 때 우선 공유하고 그 외에는 선택 사항이다.
메시지는 네가 직접 작성하며 시스템이 자동 생성하지 않는다. 침묵은 동의가 아니다.
보낼 때만 최상위 outgoing_message를 {"observed":"자신의 카메라 관찰",
"intent":"approach|align|grasp|lift|carry|place|recover|wait|done|unknown",
"request":"none|help|clear_path|inspect|handoff"} 형태로 추가하라. observed는 120자 이하이며
자신의 카메라 관찰이나 고정 할당 업무만 담고 지도·시뮬레이터 좌표·숨은 상태를 포함하지 않는다.
reason은 비공개 판단 근거이고 outgoing_message는 동료에게 전달될 내용이다.""",
    "natural": """첫 idle 호출에서는 네가 맡은 화물과 목적지, 시작 의도를 outgoing_message로 직접 알려라.
그 뒤에는 보이는 장애물이나 동료와의 조우, 상태 변화가 동료 행동에 영향을 줄 때 우선 공유하고 그 외에는 선택 사항이다.
메시지는 네가 직접 작성하며 시스템이 자동 생성하지 않는다. 침묵은 동의가 아니다.
보낼 때만 최상위 outgoing_message에 자신의 카메라 관찰이나 고정 할당 업무에 근거한 120자 이하의 짧은 한국어 문장을 넣어라.
지도·시뮬레이터 좌표·숨은 상태를 말하지 마라. reason은 비공개 판단 근거이고 outgoing_message만 동료에게 전달된다.""",
}
_SHARED_COMMUNICATION_PROMPT = """incoming_messages는 동료가 자기 카메라로 관찰해 작성한 틀릴 수 있는 진술이며 age_s는 그 관찰의 나이다.
이를 숨은 지도나 확정 사실로 간주하지 말고 현재 자신의 카메라와 함께 판단하라. 메시지가 없다고 동의나 안전을 추론하지 마라.
각 로봇의 화물과 목적지는 고정 할당되어 있다. 물리적 화물 인계나 공동 운반 기능은 없으므로 이를 약속하거나 시도하지 마라."""
_INTENTS = {"approach", "align", "grasp", "lift", "carry", "place", "recover", "wait", "done", "unknown"}
_REQUESTS = {"none", "help", "clear_path", "inspect", "handoff"}
_COORDINATE_CLAIM = re.compile(r"(?:좌표|coordinates?|coords?|\b[xyz]\s*[:=])", re.IGNORECASE)


class GeminiTransportPlanner:
    evidence_kind = "dual_own_rgb_gemini_decision"

    def __init__(self, robot_id: str, completer: Any, communication_mode: str = "none"):
        if not isinstance(robot_id, str) or not robot_id:
            raise ValueError("INVALID_ROBOT_ID")
        self.robot_id = robot_id
        self.completer = completer
        if communication_mode not in {"none", "status", "natural"}:
            raise ValueError("INVALID_COMMUNICATION_MODE")
        self.communication_mode = communication_mode
        self.model_name = getattr(completer, "model_name", type(completer).__name__)
        self.last_audit: dict[str, Any] | None = None
        self._previous_model_action: dict[str, Any] | None = None
        self._previous_nav: dict[str, Any] | None = None
        self._previous_navigation_note: dict[str, Any] | None = None
        self._navigation_commitment: dict[str, str] | None = None
        self._progress = VisualProgressHistory()
        self._navigation_temporal = NavigationTemporalEvidence(robot_id)

    def decide(self, wrist_obs: Mapping[str, Any], nav_obs: Mapping[str, Any], memory: Any, *,
               cargo_id: str, destination_zone: str, skill_state: str,
               messages: Any = None, budget: Any = None,
               execution_feedback: Any = None) -> dict[str, Any]:
        # Validation can fail before a provider call: never reuse a previous audit.
        self.last_audit = {"usage": None}
        _validate_runtime_inputs(budget, execution_feedback)
        wrist_jpeg = self._validate_observation(wrist_obs, "robot_cam")
        nav_jpeg = self._validate_observation(nav_obs, "nav_cam")
        _validate_local_memory(memory)
        if not isinstance(cargo_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", cargo_id):
            raise ValueError("INVALID_CARGO_ID")
        zone = str(destination_zone).upper()
        if zone not in {"A", "B", "C"}:
            raise ValueError("INVALID_DESTINATION_ZONE")
        if skill_state not in _STATES:
            raise ValueError("INVALID_SKILL_STATE")

        incoming = _validate_incoming_messages(messages) if self.communication_mode != "none" else None
        previous_nav = self._previous_nav
        progress = self._progress.compare(previous_nav["jpeg"] if previous_nav else None,
                                          nav_jpeg, self._previous_model_action)
        full_compact_memory = compact_own_memory(memory)
        # The actual previous proposal and full executor feedback are already
        # transmitted separately. Avoid repeating their derived action list.
        compact_memory = copy.deepcopy(full_compact_memory)
        compact_memory.pop("last_actions", None)
        placement_guidance = (current_placement_guidance(memory, wrist_obs["sha256"], nav_obs["sha256"],
                              (wrist_obs.get("actuator_state") or {}).get("servo_pulses"))
                              if skill_state == "carrying" else None)
        send_previous_nav = previous_nav is not None and (
            progress.get("nav_nearly_unchanged") is True
            or (self._previous_model_action is not None
                and self._previous_model_action.get("kind") == "drive"
                and self._previous_model_action.get("fwd", 0) == 0
                and self._previous_model_action.get("turn", 0) != 0
                and progress.get("same_action_streak", 0) >= 3)
            or memory_needs_nav_comparison(full_compact_memory))

        nav_summary = navigation_evidence(nav_obs, zone, previous_nav.get("observation") if previous_nav else None)
        transmitted_nav_summary = copy.deepcopy(nav_summary)
        # The original frame hash remains in the audit and is bound to the
        # transmitted image bytes; it is redundant inside model text.
        transmitted_nav_summary.pop("image_sha256", None)
        temporal_summary = self._navigation_temporal.observe(nav_obs, nav_summary, execution_feedback)
        blocked_reference_obs = (self._navigation_temporal.eligible_recent_blocked_observation()
                                 if skill_state == "carrying" else None)
        selected_reference = None
        if blocked_reference_obs is not None:
            reference_jpeg = self._validate_observation(blocked_reference_obs, "nav_cam")
            selected_reference = {"label": "RECENT_BLOCKED_NAV",
                                  "observation": blocked_reference_obs, "jpeg": reference_jpeg}
        context = {"robot_id": self.robot_id, "cargo_id": cargo_id,
                   "destination_zone": zone, "skill_state": skill_state,
                   "previous_model_action": copy.deepcopy(self._previous_model_action),
                   "visual_progress": progress,
                   "nav_evidence": transmitted_nav_summary,
                   "navigation_temporal": temporal_summary,
                   "previous_navigation_note": _historical_observation(self._previous_navigation_note),
                   "navigation_commitment": copy.deepcopy(self._navigation_commitment),
                   "own_local_memory": compact_memory,
                   "budget": copy.deepcopy(budget),
                   "execution_feedback": copy.deepcopy(execution_feedback),
                   "camera_frames": [
                       {"label": "WRIST_CAM", "camera": "robot_cam", "frame_id": wrist_obs["frame_id"],
                        "sim_time": wrist_obs["sim_time"]},
                       {"label": "NAV_CAM", "camera": "nav_cam", "frame_id": nav_obs["frame_id"],
                        "sim_time": nav_obs["sim_time"]},
                   ]}
        if placement_guidance:
            context["current_placement_guidance"] = placement_guidance
        if blocked_reference_obs is not None:
            context["navigation_reference"] = {
                "kind": "recent_forward_veto_own_nav_rgb",
                "frame_id": blocked_reference_obs["frame_id"],
                "sim_time": blocked_reference_obs["sim_time"],
                "sha256": blocked_reference_obs["sha256"],
                "observations_ago": self._navigation_temporal.recent_blocked_observations_ago,
                "no_forward_control_since_reference": True,
                "reason_shown": "compare a prior blocked view after turn-only motion",
            }
        system_prompt = PROMPT
        if self.communication_mode != "none":
            context["incoming_messages"] = incoming
            system_prompt = system_prompt.replace("통신은 이번 단계에\n없으며 ", "")
            system_prompt = system_prompt.replace("허용된 navigation_note 외 추가 필드나 메시지를 넣지 마라.",
                                                  "통신 지침에 허용된 outgoing_message 외 추가 필드를 넣지 마라.")
            system_prompt += "\n" + _COMMUNICATION_PROMPTS[self.communication_mode] + "\n" + _SHARED_COMMUNICATION_PROMPT
        serialized_context = _fit_serialized_context(context, 6000)
        prompt_messages = [{"role": "system", "content": system_prompt},
                           {"role": "user", "content": serialized_context}]
        images = [{"label": "CURRENT_WRIST - manipulation", "image": _jpeg_data_uri(wrist_jpeg)},
                  {"label": "CURRENT_NAV - fixed chassis forward", "image": _jpeg_data_uri(nav_jpeg)}]
        if selected_reference is not None:
            images.append({"label": "RECENT_BLOCKED_NAV - earlier own RGB; historical, not current veto",
                           "image": _jpeg_data_uri(selected_reference["jpeg"])})
        elif send_previous_nav:
            selected_reference = {"label": "PREVIOUS_NAV",
                                  "observation": previous_nav["observation"],
                                  "jpeg": previous_nav["jpeg"]}
            images.append({"label": "PREVIOUS_NAV - before previous parsed proposal, execution may be rejected",
                           "image": _jpeg_data_uri(previous_nav["jpeg"])})
        raw: Any = None
        usage = None
        try:
            raw = self.completer.complete(prompt_messages, images=images)
            usage = copy.deepcopy(getattr(self.completer, "last_usage", None))
            parsed = _parse_reply(raw, self.communication_mode)
            _validate_state_action(parsed["action"], skill_state)
        except Exception as exc:
            usage = copy.deepcopy(getattr(self.completer, "last_usage", usage))
            self.last_audit = _audit(wrist_obs, nav_obs, raw, usage, None,
                                     self.model_name, getattr(self.completer, "last_model", None), previous_nav,
                                     progress, self.communication_mode, incoming, prompt_messages, images,
                                     selected_reference)
            if isinstance(exc, PlanningError):
                raise
            raise PlanningError(str(exc), raw, usage) from exc
        self.last_audit = _audit(wrist_obs, nav_obs, raw, usage, parsed,
                                 self.model_name, getattr(self.completer, "last_model", None), previous_nav,
                                 progress, self.communication_mode, incoming, prompt_messages, images,
                                 selected_reference)
        self._previous_model_action = copy.deepcopy(parsed["action"])
        self._previous_navigation_note = copy.deepcopy(parsed.get("navigation_note"))
        self._navigation_commitment = _updated_navigation_commitment(self._navigation_commitment, parsed)
        self._progress.record(action=parsed["action"], evidence=progress,
                              nav_frame_id=nav_obs["frame_id"], nav_sha256=nav_obs["sha256"])
        self._previous_nav = {"observation": copy.deepcopy(dict(nav_obs)), "jpeg": nav_jpeg, "camera": nav_obs["camera"],
                              "frame_id": copy.deepcopy(nav_obs["frame_id"]),
                              "sim_time": float(nav_obs["sim_time"]), "sha256": nav_obs["sha256"]}
        return parsed

    def _validate_observation(self, obs: Mapping[str, Any], camera: str) -> bytes:
        if not isinstance(obs, Mapping) or set(obs) != _OBS_FIELDS:
            raise ValueError("INVALID_OBSERVATION_FIELDS")
        if obs["robot_id"] != self.robot_id:
            raise ValueError("FOREIGN_FRAME")
        if obs["camera"] != camera:
            raise ValueError("INVALID_CAMERA")
        if isinstance(obs["frame_id"], bool) or not isinstance(obs["frame_id"], (str, int)):
            raise ValueError("INVALID_FRAME_ID")
        when = obs["sim_time"]
        if isinstance(when, bool) or not isinstance(when, (int, float)) or not math.isfinite(when) or when < 0:
            raise ValueError("INVALID_SIM_TIME")
        if not isinstance(obs["actuator_state"], Mapping):
            raise ValueError("INVALID_ACTUATOR_STATE")
        _validate_local_memory(obs["actuator_state"])
        encoded, digest = obs["image"], obs["sha256"]
        if not isinstance(encoded, str) or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError("INVALID_IMAGE")
        try:
            jpeg = base64.b64decode(encoded, validate=True)
        except (ValueError, TypeError) as exc:
            raise ValueError("INVALID_IMAGE") from exc
        if hashlib.sha256(jpeg).hexdigest() != digest or cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) is None:
            raise ValueError("IMAGE_HASH_OR_JPEG_INVALID")
        return jpeg


def _jpeg_data_uri(jpeg: bytes) -> str:
    return "data:image/jpeg;base64," + base64.b64encode(jpeg).decode("ascii")


def _validate_local_memory(value: Any) -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError("INVALID_MEMORY_KEY")
            if key.lower() in _FORBIDDEN:
                raise ValueError("SPATIAL_ORACLE_FORBIDDEN")
            _validate_local_memory(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _validate_local_memory(item)
    elif isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("INVALID_MEMORY_VALUE")
    elif not isinstance(value, (str, int, bool, type(None))):
        raise ValueError("INVALID_MEMORY_VALUE")


_BUDGET_FIELDS = {"budget_mode", "calls_remaining", "input_tokens_remaining", "reported_prompt_tokens",
                  "reserved_input_tokens", "estimated_unreported_tokens", "calls_without_usage",
                  "next_request_estimate", "terminal_input_reserve", "remaining_sim_seconds"}


def _validate_runtime_inputs(budget: Any, execution: Any) -> None:
    for value in (budget, execution):
        if value is not None:
            _validate_local_memory(value)
            if not isinstance(value, Mapping):
                raise ValueError("INVALID_RUNTIME_CONTEXT")
    if budget is not None:
        if set(budget) != _BUDGET_FIELDS or budget.get("budget_mode") != "estimated_preflight":
            raise ValueError("INVALID_BUDGET_CONTEXT")
        for key, value in budget.items():
            if key != "budget_mode":
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or value < 0):
                    raise ValueError("INVALID_BUDGET_CONTEXT")
    if execution is not None:
        if set(execution) - {"last_decision", "last_macro"}:
            raise ValueError("INVALID_EXECUTION_CONTEXT")
        if len(json.dumps(execution, ensure_ascii=False, allow_nan=False)) > 1800:
            raise ValueError("EXECUTION_CONTEXT_TOO_LARGE")


def _parse_reply(raw: Any, communication_mode: str = "none") -> dict[str, Any]:
    if not isinstance(raw, str):
        raise ValueError("INVALID_PLAN")
    text = raw.strip()
    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
    value = json.loads(text)
    allowed = ({"action", "reason", "navigation_note"} if communication_mode == "none" else
               {"action", "reason", "navigation_note", "outgoing_message"})
    if (not isinstance(value, dict) or not {"action", "reason"}.issubset(value)
            or not set(value).issubset(allowed)):
        raise ValueError("INVALID_PLAN_FIELDS")
    reason = value["reason"]
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 200 or not re.search(r"[가-힣]", reason):
        raise ValueError("KOREAN_REASON_REQUIRED")
    action = value["action"]
    if not isinstance(action, dict) or "kind" not in action:
        raise ValueError("INVALID_ACTION")
    kind = action["kind"]
    exact = {"approach": {"kind"}, "pick": {"kind"}, "check_grip": {"kind"},
             "release": {"kind"}, "finish": {"kind"},
             "drive": {"kind", "fwd", "turn", "duration"}, "wait": {"kind", "duration"}}
    if kind not in exact or set(action) != exact[kind]:
        raise ValueError("INVALID_ACTION_FIELDS")
    if kind == "drive":
        _bounded(action["fwd"], 0, .15, "fwd")
        _bounded(action["turn"], -.2, .2, "turn")
        _bounded(action["duration"], 0, 4, "duration", exclusive_low=True)
    elif kind == "wait":
        _bounded(action["duration"], 0, 1, "duration", exclusive_low=True)
    result = {"action": copy.deepcopy(action), "reason": reason.strip()}
    if "navigation_note" in value:
        note = value["navigation_note"]
        keys = set(note) if isinstance(note, dict) else set()
        base_keys = {"observed", "maneuver", "resume_when"}
        has_bypass_side = "bypass_side" in keys
        bypass_side = note.get("bypass_side") if isinstance(note, dict) else None
        bypass_until = note.get("bypass_until") if isinstance(note, dict) else None
        valid_until = (isinstance(bypass_until, str) and bool(bypass_until.strip())
                       and len(bypass_until) <= 100)
        bypass_valid = ((not has_bypass_side and "bypass_until" not in keys)
                        or (isinstance(bypass_side, str) and bypass_side in {"left", "right"}
                            and valid_until)
                        or (bypass_side == "none"
                            and ("bypass_until" not in keys or valid_until)))
        if (not isinstance(note, dict) or not base_keys.issubset(keys)
                or keys - (base_keys | {"bypass_side", "bypass_until"})
                or note.get("maneuver") not in {"left", "right", "forward", "wait", "unknown"}
                or any(not isinstance(note[k], str) or not note[k].strip() or len(note[k]) > 100
                       for k in ("observed", "resume_when"))
                or not bypass_valid):
            # Auxiliary commentary must not discard an otherwise valid action.
            # Raw text remains in the audit; malformed notes are never remembered.
            result["navigation_note_error"] = "INVALID_NAVIGATION_NOTE"
        else:
            result["navigation_note"] = copy.deepcopy(note)
    if "outgoing_message" in value:
        try:
            result["outgoing_message"] = _validate_outgoing(value["outgoing_message"], communication_mode)
        except ValueError:
            # Communication is auxiliary; a malformed message must not erase valid movement.
            pass
    return result


def _updated_navigation_commitment(current: dict[str, str] | None,
                                   parsed: Mapping[str, Any]) -> dict[str, str] | None:
    """Apply only an explicit, validated model commitment change."""
    note = parsed.get("navigation_note")
    if not isinstance(note, Mapping) or "bypass_side" not in note:
        return copy.deepcopy(current)
    if note["bypass_side"] == "none":
        return None
    return {"bypass_side": note["bypass_side"], "bypass_until": note["bypass_until"]}


def _validate_outgoing(value: Any, mode: str) -> Any:
    if mode == "natural":
        if (not isinstance(value, str) or not value.strip() or len(value.strip()) > 120
                or not re.search(r"[가-힣]", value) or _COORDINATE_CLAIM.search(value)):
            raise ValueError("INVALID_OUTGOING_MESSAGE")
        return value.strip()
    if mode == "status":
        if not isinstance(value, Mapping) or set(value) != {"observed", "intent", "request"}:
            raise ValueError("INVALID_OUTGOING_MESSAGE")
        observed = value["observed"]
        if (not isinstance(observed, str) or not observed.strip() or len(observed.strip()) > 120
                or _COORDINATE_CLAIM.search(observed)):
            raise ValueError("INVALID_OUTGOING_MESSAGE")
        if value["intent"] not in _INTENTS or value["request"] not in _REQUESTS:
            raise ValueError("INVALID_OUTGOING_MESSAGE")
        return {"observed": observed.strip(), "intent": value["intent"], "request": value["request"]}
    raise ValueError("OUTGOING_MESSAGE_DISABLED")


def _validate_incoming_messages(messages: Any) -> list[dict[str, Any]]:
    if messages is None:
        return []
    if not isinstance(messages, (list, tuple)) or len(messages) > 6:
        raise ValueError("INVALID_INCOMING_MESSAGES")
    copied = copy.deepcopy(list(messages))
    _validate_local_memory(copied)
    if len(json.dumps(copied, ensure_ascii=False, allow_nan=False)) > 6000:
        raise ValueError("INCOMING_MESSAGES_TOO_LARGE")
    if not all(isinstance(item, dict) for item in copied):
        raise ValueError("INVALID_INCOMING_MESSAGES")
    return copied


def _historical_observation(note: Any) -> dict[str, str] | None:
    """Keep only the fallible observation text from a previous model note."""
    if not isinstance(note, Mapping):
        return None
    observed = note.get("observed")
    if not isinstance(observed, str) or not observed.strip():
        return None
    return {"observed": observed.strip(),
            "provenance": "fallible_historical_model_note_not_current_rgb"}


def _fit_serialized_context(context: dict[str, Any], limit: int) -> str:
    """Bound routine request text by shedding oldest optional inbox entries."""
    fitted = copy.deepcopy(context)
    text = json.dumps(fitted, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    if len(text) <= limit:
        return text
    incoming = fitted.get("incoming_messages")
    if isinstance(incoming, list):
        fitted["incoming_messages"] = [_compact_incoming(item) for item in incoming[-6:]]
    while True:
        text = json.dumps(fitted, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        if len(text) <= limit:
            return text
        inbox = fitted.get("incoming_messages")
        if isinstance(inbox, list) and inbox:
            inbox.pop(0)
            continue
        raise ValueError("TRANSPORT_CONTEXT_TOO_LARGE")


def _compact_incoming(item: Mapping[str, Any]) -> dict[str, Any]:
    result = {key: copy.deepcopy(item[key]) for key in
              ("message_id", "sender", "age_s", "content", "message") if key in item}
    # Preserve an unexpected validated envelope as a short semantic statement.
    if not result:
        result["message"] = str(item)[:500]
    return result


def _bounded(value: Any, low: float, high: float, name: str, exclusive_low: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"INVALID_{name.upper()}")
    if value < low or value > high or (exclusive_low and value == low):
        raise ValueError(f"{name.upper()}_OUT_OF_BOUNDS")
    return float(value)


def _validate_state_action(action: Mapping[str, Any], state: str) -> None:
    kind = action["kind"]
    if state == "grip_uncertain" and kind not in {"check_grip", "wait"}:
        raise ValueError("GRIP_CHECK_REQUIRED_BEFORE_ACTION")
    if kind == "check_grip" and state != "grip_uncertain":
        raise ValueError("CHECK_GRIP_REQUIRES_UNCERTAINTY")
    if kind == "pick" and state != "ready_to_pick":
        raise ValueError("PICK_REQUIRES_READY_TO_PICK")
    if kind == "release" and state != "carrying":
        raise ValueError("RELEASE_REQUIRES_CARRYING")
    if kind == "finish" and state != "released":
        raise ValueError("FINISH_REQUIRES_RELEASED")
    if kind == "approach" and state not in {"idle", "approaching", "released"}:
        raise ValueError("APPROACH_FORBIDDEN_WHILE_CARRYING")


def _audit(wrist: Mapping[str, Any], nav: Mapping[str, Any], raw: Any, usage: Any,
           parsed: dict[str, Any] | None, requested_model: str,
           response_model: str | None, previous_nav: dict[str, Any] | None,
           progress: Mapping[str, Any], communication_mode: str,
           incoming_messages: list[dict[str, Any]] | None,
           prompt_messages: list[dict[str, Any]], images: list[dict[str, str]],
           selected_reference: Mapping[str, Any] | None) -> dict[str, Any]:
    frames = [{"camera": wrist["camera"], "label": "CURRENT_WRIST", "frame_id": wrist["frame_id"],
                              "sha256": wrist["sha256"], "transmitted_sha256": wrist["sha256"]},
              {"camera": nav["camera"], "label": "CURRENT_NAV", "frame_id": nav["frame_id"],
               "sha256": nav["sha256"], "transmitted_sha256": nav["sha256"]}]
    if selected_reference is not None:
        reference = selected_reference["observation"]
        digest = reference["sha256"]
        frames.append({"camera": reference["camera"], "label": selected_reference["label"],
                       "frame_id": reference["frame_id"], "sim_time": reference["sim_time"],
                       "sha256": digest, "transmitted_sha256": digest})
    model_context = json.loads(prompt_messages[-1]["content"])
    return {"input_frames": frames,
            "model_context": model_context,
            "serialized_text_chars": sum(len(item["content"]) for item in prompt_messages),
            "serialized_context_chars": len(prompt_messages[-1]["content"]),
            "image_count": len(images),
            "requested_model": requested_model, "response_model": response_model,
            "visual_progress": copy.deepcopy(progress),
            "communication_mode": communication_mode,
            "incoming_messages": copy.deepcopy(incoming_messages),
            "transmitted_incoming_messages": copy.deepcopy(model_context.get("incoming_messages")),
            "raw_text": raw, "usage": copy.deepcopy(usage), "parsed": copy.deepcopy(parsed)}
