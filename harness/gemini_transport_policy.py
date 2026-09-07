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
from harness.transport_context import compact_own_memory, memory_needs_nav_comparison
from harness.visual_progress import VisualProgressHistory


_OBS_FIELDS = {"robot_id", "frame_id", "sim_time", "image", "sha256", "camera", "actuator_state"}
_STATES = {"idle", "approaching", "ready_to_pick", "picking", "carrying", "releasing",
           "released", "grip_uncertain", "failure"}
_FORBIDDEN = {"pose", "position", "positions", "coordinates", "coords", "x", "y", "z",
              "world_state", "map", "seed", "layout", "obstacle_positions", "peer_positions",
              "target_pose", "target_coords", "global_state"}

PROMPT = """당신은 창고 운반 로봇 한 대의 실제 행동 결정자다. 매 호출마다 각각 별도로 제공되는
두 실제 RGB 카메라 화면과 자신의 제한된 로컬 메모리, 현재 스킬 상태만 보고 다음 행동 하나를
직접 선택하라. 첫 화면은 손목 카메라 WRIST_CAM이며 화물 접근·집기·놓기 판단용이다.
둘째 화면은 차체 고정 전방 카메라 NAV_CAM이며 주행 판단용이다. 화면 밖 좌표, 지도,
시드, 숨은 물체/동료 상태를 가정하지 마라. 어떤 규칙 알고리즘의 승인을 하는 역할이 아니다.
PREVIOUS_NAV가 함께 제공되면 직전 성공한 모델 행동 전의 전방 화면이다. CURRENT_NAV와
직접 비교해 실제 영상 변화와 진행 여부를 판단하라. 같은 전진/회전 명령 뒤 목표·장애물의
크기와 위치가 거의 변하지 않았다면 같은 명령을 맹목적으로 반복하지 말고, 보이는 근거에
따라 다른 제자리 회전이나 짧은 wait를 직접 선택하라. VISUAL_PROGRESS와
RECENT_VISUAL_HISTORY는 자신의 NAV RGB 픽셀 변화만 요약하며 경로나 행동을 추천하지 않는다.
정체 근거가 있으면 보이는 장면을 바탕으로 대안 행동을 네가 직접 선택하라.

목적지 A는 파랑, B는 초록, C는 노랑 바닥 구역이다. 청록은 운반 중인 작은 상자이며
목표 구역이 아니다. 주황 물체는 정적 장애물이고, 자홍색 물리 밴드는 다른 로봇이다.
NAV_CAM에서 이미지 왼쪽으로 회전은 turn 양수, 이미지 오른쪽으로 회전은 turn 음수다.
보이는 장애물과 동료를 피하거나 기다리면서 목표색 구역으로 향하라.
OWN_RGB_DRIVE_BLOCKED 피드백은 작은 양수 fwd도 거부하는 전방 위험이다. 이 경우 fwd=0인 제자리 회전으로 먼저 진행 방향을 충분히 비우고 새 화면을 확인하라.
목표색이 화면 중앙에서 충분히 보이고 진행 경로가 비어 있으면 duration=4.0과 fwd=0.15까지 사용할 수 있다. drive는 네가 고른 속도와 제한 시간이며 실행기는 0.25초 안전 구간마다 위험 시 즉시 중단한다. 이를 자율 경로로 간주하지 말고 새 화면마다 다시 판단하라. 한 호출의 이동만으로 도착했다고 가정하지 마라. 통신은 이번 단계에
없으며 보이지 않는 정보를 만들어내지 마라.

approach와 pick은 손목 RGB를 사용하는 기존 저수준 폐루프 스킬을 호출한다. 이 스킬들은
목적지까지 자율주행하지 않는다. pick은 ready_to_pick 상태에서만, release는 carrying
상태에서 목적지 도착을 화면으로 판단했을 때만, finish는 released 상태에서만 선택하라.
carrying 상태에서는 approach를 다시 선택하지 마라. JSON 하나만 반환하라:
{"action":{"kind":"drive","fwd":0.08,"turn":0.1,"duration":0.6},
 "reason":"현재 두 화면에 근거한 짧은 한국어 이유"}
grip_uncertain은 긴 운반 뒤 엄격한 시각 그립 검사가 불확실해진 상태다. 이때 상자가
보인다는 이유만으로 기준을 낮추거나 release/drive를 선택하지 말고, 기존 제어 팔의
좌/우/홈 소폭 프로브로 엄격한 검사를 다시 수행하도록 {"kind":"check_grip"}을 선택하라.
잠시 가림이 해소되길 기다릴 근거가 있을 때만 wait를 선택할 수 있다.
release 전에는 own_local_memory의 최신 카메라 배치 근거가 status=inside인지 확인하라.
release 후에는 멈추고 새 두 화면과 stage=released 배치 근거를 다시 검사하라. inside가
확인된 경우에만 finish를 선택한다. outside 또는 uncertain이면 finish를 선언하지 말고
approach를 명시적으로 선택해 다시 접근→pick→drive→release 순서로 복구할 수 있다.
previous_model_action은 직전 자신의 실제 모델 선택이며, 최신 own_local_memory 피드백과
함께 사용하되 성공 증거로 간주하지 마라. 배치 복구 횟수 제한 피드백을 존중하라.
action은 정확히 다음 중 하나다: {"kind":"approach"}, {"kind":"pick"}, {"kind":"check_grip"},
{"kind":"release"}, drive(fwd 0..0.15, turn -0.2..0.2, duration 0초 초과 4초 이하),
wait(duration 0초 초과 1초 이하), {"kind":"finish"}. 추가 필드나 메시지를 넣지 마라.
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
        self._progress = VisualProgressHistory()

    def decide(self, wrist_obs: Mapping[str, Any], nav_obs: Mapping[str, Any], memory: Any, *,
               cargo_id: str, destination_zone: str, skill_state: str,
               messages: Any = None) -> dict[str, Any]:
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
        compact_memory = compact_own_memory(memory)
        send_previous_nav = previous_nav is not None and (
            progress.get("nav_nearly_unchanged") is True
            or memory_needs_nav_comparison(compact_memory))

        context = {"robot_id": self.robot_id, "cargo_id": cargo_id,
                   "destination_zone": zone, "skill_state": skill_state,
                   "previous_model_action": copy.deepcopy(self._previous_model_action),
                   "visual_progress": progress,
                   "recent_visual_history": [_compact_progress_item(item)
                                               for item in self._progress.recent()[-3:]],
                   "own_local_memory": compact_memory,
                   "camera_frames": [
                       {"label": "WRIST_CAM", "camera": "robot_cam", "frame_id": wrist_obs["frame_id"],
                        "sim_time": wrist_obs["sim_time"], "sha256": wrist_obs["sha256"]},
                       {"label": "NAV_CAM", "camera": "nav_cam", "frame_id": nav_obs["frame_id"],
                        "sim_time": nav_obs["sim_time"], "sha256": nav_obs["sha256"]},
                   ]}
        system_prompt = PROMPT
        if self.communication_mode != "none":
            context["incoming_messages"] = incoming
            system_prompt = system_prompt.replace("통신은 이번 단계에\n없으며 ", "")
            system_prompt = system_prompt.replace("추가 필드나 메시지를 넣지 마라.",
                                                  "통신 지침에 허용된 outgoing_message 외 추가 필드를 넣지 마라.")
            system_prompt += "\n" + _COMMUNICATION_PROMPTS[self.communication_mode] + "\n" + _SHARED_COMMUNICATION_PROMPT
        serialized_context = _fit_serialized_context(context, 6000)
        prompt_messages = [{"role": "system", "content": system_prompt},
                           {"role": "user", "content": serialized_context}]
        images = [{"label": "CURRENT_WRIST - manipulation", "image": _jpeg_data_uri(wrist_jpeg)},
                  {"label": "CURRENT_NAV - fixed chassis forward", "image": _jpeg_data_uri(nav_jpeg)}]
        if send_previous_nav:
            images.append({"label": "PREVIOUS_NAV - before previous successful model action",
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
                                     send_previous_nav)
            if isinstance(exc, PlanningError):
                raise
            raise PlanningError(str(exc), raw, usage) from exc
        self.last_audit = _audit(wrist_obs, nav_obs, raw, usage, parsed,
                                 self.model_name, getattr(self.completer, "last_model", None), previous_nav,
                                 progress, self.communication_mode, incoming, prompt_messages, images,
                                 send_previous_nav)
        self._previous_model_action = copy.deepcopy(parsed["action"])
        self._progress.record(action=parsed["action"], evidence=progress,
                              nav_frame_id=nav_obs["frame_id"], nav_sha256=nav_obs["sha256"])
        self._previous_nav = {"jpeg": nav_jpeg, "camera": nav_obs["camera"],
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


def _parse_reply(raw: Any, communication_mode: str = "none") -> dict[str, Any]:
    if not isinstance(raw, str):
        raise ValueError("INVALID_PLAN")
    text = raw.strip()
    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
    value = json.loads(text)
    allowed = ({"action", "reason"} if communication_mode == "none" else
               {"action", "reason", "outgoing_message"})
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
    if "outgoing_message" in value:
        try:
            result["outgoing_message"] = _validate_outgoing(value["outgoing_message"], communication_mode)
        except ValueError:
            # Communication is auxiliary; a malformed message must not erase valid movement.
            pass
    return result


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


def _compact_progress_item(item: Mapping[str, Any]) -> dict[str, Any]:
    return {key: copy.deepcopy(item[key]) for key in
            ("action", "nav_mean_absolute_change", "nav_nearly_unchanged") if key in item}


def _fit_serialized_context(context: dict[str, Any], limit: int) -> str:
    """Bound routine request text by shedding oldest optional inbox entries."""
    fitted = copy.deepcopy(context)
    text = json.dumps(fitted, ensure_ascii=False, allow_nan=False)
    if len(text) <= limit:
        return text
    incoming = fitted.get("incoming_messages")
    if isinstance(incoming, list):
        fitted["incoming_messages"] = [_compact_incoming(item) for item in incoming[-6:]]
    while True:
        text = json.dumps(fitted, ensure_ascii=False, allow_nan=False)
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
           sent_previous_nav: bool) -> dict[str, Any]:
    frames = [{"camera": wrist["camera"], "label": "CURRENT_WRIST", "frame_id": wrist["frame_id"],
                              "sha256": wrist["sha256"], "transmitted_sha256": wrist["sha256"]},
              {"camera": nav["camera"], "label": "CURRENT_NAV", "frame_id": nav["frame_id"],
               "sha256": nav["sha256"], "transmitted_sha256": nav["sha256"]}]
    if previous_nav is not None and sent_previous_nav:
        frames.append({"camera": previous_nav["camera"], "label": "PREVIOUS_NAV",
                       "frame_id": previous_nav["frame_id"], "sim_time": previous_nav["sim_time"],
                       "sha256": previous_nav["sha256"],
                       "transmitted_sha256": previous_nav["sha256"]})
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
