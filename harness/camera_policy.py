"""Strict, stateless, RGB-only policy boundary for a physical MasterPi robot."""
from __future__ import annotations

import base64
import copy
import hashlib
import json
import math
import re

from harness.coela_modules import Communication, MODES, ROBOTS, PlanningError


OBSERVATION_FIELDS = {
    "robot_id", "frame_id", "sim_time", "image", "sha256", "camera",
    "actuator_state",
}
DECISION_FIELDS = {"action", "private_reason", "message"}
FORBIDDEN_SPATIAL_FIELDS = {
    "pose", "position", "positions", "coordinates", "coords", "x", "y", "z",
    "cargo", "obstacles", "obstacle_positions", "peer_positions", "world_state",
    "global_state", "map", "seed", "seeded_layout", "target_coords", "target_pose",
}

PROMPT = """당신은 실제 창고 로봇 한 대의 독립 정책이다. 현재 robot_cam RGB 사진과
자기 액추에이터 상태, 로컬 메모리의 주장, 수신 메시지만 사용하라. 이미지 밖의 정확한
월드/화물/장애물/동료 위치나 자기 전역 pose를 추측하거나 요구하지 마라. 메모리와 동료
보고는 검증된 진실이 아니라 주장이다. 다른 로봇은 현재 사진에서 직접 보거나 그 로봇의
보고를 받았을 때만 언급하라.

destination은 사람이 붙인 시각적 구역 이름이다. A는 파랑, B는 초록, C는 노랑 구역이다.
사진에서 색 표지나 경계를 찾아 이동하되 지도 좌표나 seeded layout을 가정하지 마라.
주요 임무 물체는 빨간 마커가 붙은 목재 판재(두 운반자 필요)와 청록색 작은 상자
두 개(각각 식별 마커 부착)다. 물체 ID는 사진의 ArUco 표식을 실제로 읽을 수 있을 때만 특정하고,
보이지 않는 ID를 지어내지 마라.
drive는 현재 카메라 시야에 근거한 짧은 전진/회전 명령이다. look은 물리 카메라 팬
servo 6 펄스이고 arm은 원시 서보 명령이다. servo_id 1은 그리퍼, 3은 손목, 4는 팔꿈치,
5는 어깨다. 카메라가 손목에 장착되어 있으므로 arm 동작도 다음 시야를 바꾼다. 정확한
pose를 꾸며내지 마라. 자동 집기, 자동 운반, 목표 좌표 이동 기능은
없으며 성공을 명령만으로 선언하지 마라. 필요하면 다음 사진을 보기 위한 짧은 동작을 하라.

JSON 하나만 반환하라:
{"action":{"kind":"drive","forward":0.1,"turn":0.0,"duration_s":0.5},
 "private_reason":"현재 사진에 근거한 짧은 이유","message":null}
action은 다음 중 정확히 하나다:
- drive: forward 0..0.15, turn -0.2..0.2, duration_s 0초 초과 1초 이하
- look: pan_pulse 정수 500..2500 (카메라 팬 servo 6)
- arm: servo_id는 1|3|4|5, pulse는 정수 500..2500
- wait: 추가 필드 없음
message는 생략할 수 없고 통신하지 않으면 null이다. none 모드에서는 반드시 null이다.
natural 모드 메시지는 {"recipients":["r2"],"content":"수신자가 이해할 한국어",
"kind":"intent","observed_ids":[]} 형식이다. 수신자를 명시하고 자기 자신을 제외하라.
kind는 observation|intent|completed|help_request|recovery 중 하나다. 사진에서 직접 식별한
화물만 observed_ids에 넣고 ID는 oak_plank_01|small_box_01|small_box_02만 허용된다.
structured 모드에서는 같은 recipients를 쓰고 content를 구조화한다:
{"recipients":["r2"],"content":{"kind":"intent","cargo_id":"small_box_01",
"participants":["r1"]},"observed_ids":[]}. content에는 kind, cargo_id, participants,
event_id, action, reason_code, observed_ids만 허용되며 기존 통신 계약의 열거값을 지켜라.
private_reason을 메시지에 복사하지 말고, 보거나 보고받지 않은 사실을 전하지 마라.
"""


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"INVALID_{name.upper()}")
    return float(value)


def _reject_spatial_oracle(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError("INVALID_CONTEXT_KEY")
            if key.lower() in FORBIDDEN_SPATIAL_FIELDS:
                raise ValueError("SPATIAL_ORACLE_FORBIDDEN")
            _reject_spatial_oracle(item)
    elif isinstance(value, list):
        for item in value:
            _reject_spatial_oracle(item)
    elif not isinstance(value, (str, int, float, bool, type(None))):
        raise ValueError("INVALID_CONTEXT_VALUE")
    elif isinstance(value, float) and not math.isfinite(value):
        raise ValueError("INVALID_CONTEXT_VALUE")


def _validate_observation(observation, robot_id):
    if not isinstance(observation, dict) or set(observation) != OBSERVATION_FIELDS:
        raise ValueError("INVALID_OBSERVATION_FIELDS")
    if observation["robot_id"] != robot_id or robot_id not in ROBOTS:
        raise ValueError("FOREIGN_FRAME")
    if observation["camera"] != "robot_cam":
        raise ValueError("INVALID_CAMERA")
    if isinstance(observation["frame_id"], bool) or not isinstance(observation["frame_id"], (str, int)):
        raise ValueError("INVALID_FRAME_ID")
    sim_time = _number(observation["sim_time"], "sim_time")
    if sim_time < 0:
        raise ValueError("INVALID_SIM_TIME")
    if not isinstance(observation["actuator_state"], dict):
        raise ValueError("INVALID_ACTUATOR_STATE")
    _reject_spatial_oracle(observation["actuator_state"])
    image = observation["image"]
    digest = observation["sha256"]
    if not isinstance(image, str) or not image or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("INVALID_IMAGE")
    try:
        jpeg = base64.b64decode(image, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError("INVALID_IMAGE") from exc
    if len(jpeg) < 4 or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
        raise ValueError("INVALID_JPEG")
    if hashlib.sha256(jpeg).hexdigest() != digest:
        raise ValueError("IMAGE_HASH_MISMATCH")
    return jpeg, sim_time


def _validate_action(action):
    if not isinstance(action, dict) or not isinstance(action.get("kind"), str):
        raise ValueError("INVALID_ACTION")
    kind = action["kind"]
    fields = {
        "drive": {"kind", "forward", "turn", "duration_s"},
        "look": {"kind", "pan_pulse"},
        "arm": {"kind", "servo_id", "pulse"},
        "wait": {"kind"},
    }
    if kind not in fields or set(action) != fields[kind]:
        raise ValueError("INVALID_ACTION_FIELDS")
    if kind == "drive":
        forward = _number(action["forward"], "forward")
        turn = _number(action["turn"], "turn")
        duration = _number(action["duration_s"], "duration_s")
        if not 0 <= forward <= .15 or not -.2 <= turn <= .2 or not 0 < duration <= 1:
            raise ValueError("ACTION_OUT_OF_BOUNDS")
    elif kind == "look":
        if isinstance(action["pan_pulse"], bool) or not isinstance(action["pan_pulse"], int) or not 500 <= action["pan_pulse"] <= 2500:
            raise ValueError("ACTION_OUT_OF_BOUNDS")
    elif kind == "arm":
        if isinstance(action["servo_id"], bool) or not isinstance(action["servo_id"], int) or action["servo_id"] not in {1, 3, 4, 5}:
            raise ValueError("ACTION_OUT_OF_BOUNDS")
        if isinstance(action["pulse"], bool) or not isinstance(action["pulse"], int) or not 500 <= action["pulse"] <= 2500:
            raise ValueError("ACTION_OUT_OF_BOUNDS")


def _validate_message(message, robot_id, mode):
    if message is None:
        return
    if mode == "none":
        raise ValueError("COMMUNICATION_DISABLED")
    if not isinstance(message, dict) or set(message) - {"recipients", "content", "kind", "observed_ids"}:
        raise ValueError("INVALID_MESSAGE")
    recipients = message.get("recipients")
    if (not isinstance(recipients, list) or not recipients or len(set(recipients)) != len(recipients)
            or robot_id in recipients or set(recipients) - set(ROBOTS)):
        raise ValueError("INVALID_RECIPIENTS")
    if mode == "natural":
        content = message.get("content")
        if not isinstance(content, str) or not content.strip() or not re.search(r"[가-힣]", content):
            raise ValueError("KOREAN_TEXT_REQUIRED")
    # Use the production bus as a pure contract validator.  This bus is local
    # to the call and therefore cannot deliver to the episode's real peers.
    Communication(mode).send(robot_id, copy.deepcopy(message), 0.0)


class CameraPlanner:
    """No internal memory: each decision is solely a function of explicit inputs."""

    evidence_kind = "rgb_only_multimodal_llm"

    def __init__(self, robot_id, completer):
        if robot_id not in ROBOTS:
            raise ValueError("INVALID_ROBOT_ID")
        self.robot_id = robot_id
        self.completer = completer
        self.model_name = getattr(completer, "model_name", type(completer).__name__)

    def decide(self, observation, memory, messages, mode="natural", destination="B"):
        if mode not in MODES:
            raise ValueError("UNKNOWN_COMMUNICATION_MODE")
        jpeg, sim_time = _validate_observation(observation, self.robot_id)
        _reject_spatial_oracle(memory)
        _reject_spatial_oracle(messages)
        if not isinstance(destination, str) or not destination.strip():
            raise ValueError("INVALID_DESTINATION")
        context = {
            "robot_id": self.robot_id,
            "frame_id": copy.deepcopy(observation["frame_id"]),
            "sim_time": sim_time,
            "camera": "robot_cam",
            "sha256": observation["sha256"],
            "actuator_state": copy.deepcopy(observation["actuator_state"]),
            "memory_claims": copy.deepcopy(memory),
            "received_messages": copy.deepcopy(messages),
            "communication_mode": mode,
            "destination_visual_name": destination,
        }
        prompt_messages = [
            {"role": "system", "content": PROMPT},
            {"role": "user", "content": json.dumps(context, ensure_ascii=False, allow_nan=False)},
        ]
        image_uri = "data:image/jpeg;base64," + observation["image"]
        raw = self.completer.complete(prompt_messages, image=image_uri)
        usage = copy.deepcopy(getattr(self.completer, "last_usage", None))
        try:
            if not isinstance(raw, str):
                raise ValueError("INVALID_PLAN")
            text = raw.strip()
            if text.startswith("```"):
                text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
            decision = json.loads(text)
            if not isinstance(decision, dict) or set(decision) != DECISION_FIELDS:
                raise ValueError("INVALID_PLAN_FIELDS")
            if not isinstance(decision["private_reason"], str) or not decision["private_reason"].strip():
                raise ValueError("PRIVATE_REASON_REQUIRED")
            _validate_action(decision["action"])
            _validate_message(decision["message"], self.robot_id, mode)
        except (ValueError, TypeError, IndexError, json.JSONDecodeError) as exc:
            raise PlanningError(str(exc), raw, usage) from exc
        return {
            "action": copy.deepcopy(decision["action"]),
            "private_reason": decision["private_reason"],
            "message": copy.deepcopy(decision["message"]),
            "raw": raw,
            "usage": usage,
            "input_frame": {
                "robot_id": self.robot_id,
                "frame_id": copy.deepcopy(observation["frame_id"]),
                "sim_time": sim_time,
                "camera": "robot_cam",
                "sha256": observation["sha256"],
                "jpeg_bytes": len(jpeg),
            },
        }
