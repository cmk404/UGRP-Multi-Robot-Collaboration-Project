"""Browser coworker: talk, look at the scene, then run allowlisted skills."""

from __future__ import annotations

import base64
import contextlib
import hmac
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import unquote, urlparse
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

from .catalog import default_registry, resolve_actions_path, tools_payload
from .chat import event_from_step, handle_turn, history_text, result_payload
from .loop import Completer
from .multi_agent import MultiRobotCompleter
from .pi_camera import (
    CameraError,
    ensure_camera_service,
    grab_snapshot,
    open_mjpeg_stream,
    pop_jpegs,
    start_camera_tunnel,
    stream_url_from_snapshot,
)
from .robot import RobotError, RobotPanel
from .vlm import VlmError
from .state import StateEstimator
from .action_queue import RobotActionQueue
from .executive import TaskExecutive
from .goals import infer_goal
from .mission_language import maybe_parse_transfer_command
from .perception import detect_red_target_bytes, detect_scene_bytes
from .real_geometry import metric_memory_entry
from .real_pose import PoseStabilityTracker, open_commanded_pose_stream, parse_commanded_pose, read_commanded_pose
from .real_carry import read_carry_evidence
from . import execution_trace as real_trace
from . import team_bus
from scripts import sim_actions
from sim.bridge_client import bridge_headers as sim_bridge_headers
from sim.team_batch import use_team_batch
from sim.team_layout import TEAM_STACK_SITE, TEAM_STACK_SITE_TOLERANCE_M

PROJECT_ROOT = Path(__file__).resolve().parents[1]
STATIC = Path(__file__).resolve().parent / "static" / "index.html"
THREE_JS = Path(__file__).resolve().parent / "static" / "three.min.js"
SIM3D_JS = Path(__file__).resolve().parent / "static" / "sim3d.js"
SIM_SCENE_XML = Path(__file__).resolve().parents[1] / "sim" / "masterpi_scene_v2.xml"
SIM_CALIBRATION_JSON = Path(__file__).resolve().parents[1] / "sim" / "masterpi_dynamics_calibration.json"
TEST_VIDEO_DIR = Path(__file__).resolve().parents[1] / "outputs" / "test_videos"
OFFLINE_PREVIEW_DIR = Path(__file__).resolve().parent / "static" / "offline"
GPU_RECOVERY_STATUS = Path(os.environ.get("UGRP_GPU_RECOVERY_STATUS", "/tmp/ugrp_gpu_recovery_status.json"))


def request_gpu_recovery() -> tuple[bool, str]:
    """Ask the Oracle systemd recovery service to start a SIM worker.

    Returns (started, detail). Hosts without systemctl (e.g. the Mac local
    stack) cannot self-recover this way; report that instead of raising
    FileNotFoundError into the HTTP handler.
    """
    if shutil.which("systemctl") is None:
        return False, "systemctl not available on this host; start the SIM worker manually"
    try:
        proc = subprocess.run(
            ["systemctl", "--user", "start", "--no-block", "ugrp-sim-gpu-recover.service"],
            cwd=PROJECT_ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=2.0, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"recovery request failed: {exc}"
    if proc.returncode == 0:
        return True, "recovery requested"
    return False, "recovery request refused"
REAL_RESUME_GRASP_PATH = Path(
    os.environ.get("UGRP_REAL_RESUME_GRASP_PATH", "/tmp/ugrp-real-resume-grasp.json")
)
# Main SIM R1 remains the public/static server. Other robot sessions are local
# children reached only through these explicit prefixes, preventing chat/cancel/
# camera state from crossing robot boundaries.
BACKEND_PORTS = {
    ("sim", "r1"): int(os.environ.get("UGRP_SIM_R1_PORT", "8082")),
    ("sim", "r2"): int(os.environ.get("UGRP_SIM_R2_PORT", "8084")),
    ("sim", "r3"): int(os.environ.get("UGRP_SIM_R3_PORT", "8085")),
    ("real", "r1"): int(os.environ.get("UGRP_REAL_R1_PORT", "8083")),
    ("real", "r2"): int(os.environ.get("UGRP_REAL_R2_PORT", "8086")),
    ("real", "r3"): int(os.environ.get("UGRP_REAL_R3_PORT", "8087")),
}
MAX_IMAGE_BYTES = 8 * 1024 * 1024
# A turn may carry one base64 data-URL image (~4/3 of MAX_IMAGE_BYTES) plus text.
MAX_REQUEST_BODY_BYTES = 12 * 1024 * 1024
IMAGE_TYPES = {
    "image/jpeg": ".jpg",
    "image/jpg": ".jpg",
    "image/png": ".png",
    "image/webp": ".webp",
}

_AGENT_ACTION_HINTS = (
    "찾아", "검색", "탐색", "추적", "따라가", "다가", "접근",
    "움직여", "움직여줘", "이동해", "이동해줘", "전진", "후진",
    "회전해", "돌아줘", "돌려", "정지해", "멈춰",
    "집어", "잡아", "들어줘", "놓아", "놓고", "올려", "얹어", "옮겨",
    "모아", "모으", "정리해", "정리하", "배치해", "배치하", "치워", "치우", "가져와", "가져오",
    "pick", "grasp", "place", "move", "track", "search", "approach",
    "turn left", "turn right", "move forward", "move backward",
)
_VISUAL_HINTS = (
    "카메라", "화면", "장면", "시야", "앞에", "보여", "보이", "보고",
    "블록", "블럭", "빨간", "빨강", "노란", "노랑", "파란", "파랑",
    "왼쪽", "오른쪽", "앞쪽", "뒤쪽",
    "camera", "screen", "scene", "visible", "see", "look", "block",
)


def restore_one_shot_real_grasp(estimator: StateEstimator, path: Path = REAL_RESUME_GRASP_PATH) -> bool:
    """Restore only conservative carry evidence across an intentional REAL reload.

    A service restart must not turn ``PROBABLE_HELD`` into ``UNKNOWN`` while the
    physical gripper may still contain a block.  The activation script snapshots
    only grasp identity/state immediately before a guarded restart.  This reader
    consumes that snapshot exactly once; it does not persist ordinary planner or
    vision state across sessions.
    """
    try:
        raw = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return False
    except OSError:
        return False
    try:
        obj = json.loads(raw)
    except (TypeError, ValueError):
        obj = {}
    finally:
        try:
            path.unlink(missing_ok=True)
        except OSError:
            pass

    state = str(obj.get("state") or "").upper()
    color = str(obj.get("held_object_color") or "").lower()
    if state not in {"PROBABLE_HELD", "HELD"} or color not in {"red", "blue", "yellow"}:
        return False

    grasp = estimator.state.grasp
    grasp.state = state
    grasp.held_object_color = color
    try:
        confidence = float(obj.get("confidence", 0.65 if state == "PROBABLE_HELD" else 0.95))
    except (TypeError, ValueError):
        confidence = 0.65 if state == "PROBABLE_HELD" else 0.95
    grasp.confidence = max(0.0, min(1.0, confidence))
    grasp.visual_support = obj.get("visual_support") if isinstance(obj.get("visual_support"), bool) else None
    try:
        grasp.held_streak = max(1, int(obj.get("held_streak", 1)))
    except (TypeError, ValueError):
        grasp.held_streak = 1
    return True


def _directional_go_command(t: str) -> bool:
    compact = " ".join(t.split())
    for direction in ("앞", "앞으로", "뒤", "뒤로", "왼쪽", "오른쪽"):
        for suffix in (" 가", " 가줘", " 가라", " 가자"):
            if direction + suffix in compact or direction + "으로" + suffix in compact:
                return True
    return False


def turn_intent(text: str) -> tuple[bool, bool]:
    """Return (agent_motion_intent, needs_current_scene)."""
    t = text.lower()
    goal = infer_goal(text)
    agent = (
        goal.kind != "GENERIC"
        or any(word in t for word in _AGENT_ACTION_HINTS)
        or _directional_go_command(t)
    )
    visual = agent or any(word in t for word in _VISUAL_HINTS)
    return agent, visual


def explicit_empty_gripper_statement(text: str) -> bool:
    """Recognize an unambiguous operator correction that the gripper is empty."""
    t = " ".join(text.strip().split())
    low = t.lower()
    if not t or "?" in t or any(q in low for q in ("맞아", "맞지", "없어?", "비었어?", "들고 있어?")):
        return False
    gripper_context = any(mark in low for mark in ("손에", "손은", "집게", "들고", "잡고"))
    empty_claim = any(mark in low for mark in (
        "없는데", "없어", "없음", "안 들고", "안들고", "안 잡고", "안잡고",
        "비어 있어", "비어있어", "비었어", "빈 상태", "아무것도 없어", "아무것도 안",
    ))
    return gripper_context and empty_claim


def spatial_context_statement(text: str) -> str | None:
    """Return a deterministic acknowledgement for a location correction.

    These statements provide context; they are neither actuator commands nor
    invitations for the VLM to invent extra scene details.
    """
    t = " ".join(text.strip().split())
    low = t.lower()
    if not t or "?" in t or any(q in low for q in ("뭐", "무엇", "누구", "어디", "어때", "맞아", "보여?", "있어?")):
        return None
    locations = ("왼쪽", "오른쪽", "앞쪽", "뒤쪽", "앞에", "뒤에", "바닥", "바로 앞", "옆")
    declarative = ("에 있어", "에 있음", "쪽이야", "쪽에", "보여", "있어")
    if not any(loc in low for loc in locations):
        return None
    if not any(mark in low for mark in declarative):
        return None
    if "오른쪽" in low:
        where = "오른쪽"
    elif "왼쪽" in low:
        where = "왼쪽"
    elif "바로 앞" in low or "앞에" in low or "앞쪽" in low:
        where = "앞쪽"
    elif "뒤쪽" in low or "뒤에" in low:
        where = "뒤쪽"
    elif "바닥" in low:
        where = "바닥 쪽"
    else:
        where = "그 위치"
    return f"알겠어. {where}에 있다는 위치 힌트로 기억할게."


class ChatState:
    def __init__(
        self,
        *,
        completer: Completer,
        password: str | None = None,
        execute: bool = False,
        registry=None,
        grab_camera=None,
        open_stream=None,
        robot: RobotPanel | None = None,
        robot_id: str = "r1",
        actions_path: str | Path | None = None,
        max_steps: int = 12,
        verify_final=None,
    ) -> None:
        self.completer = completer.for_robot(robot_id) if isinstance(completer, MultiRobotCompleter) else completer
        self.password = password
        self.execute = execute
        self.max_steps = max_steps
        self.verify_final = verify_final
        self.registry = registry or default_registry(actions_path=actions_path)
        self.state_estimator = StateEstimator()
        self.executive = TaskExecutive()
        self.action_queue = RobotActionQueue()
        self.grab_camera = grab_camera
        self.open_stream = open_stream
        self.robot = robot
        self.robot_id = team_bus.normalize_robot_id(robot_id)
        self.actions_path = resolve_actions_path(actions_path)
        self.history: list[dict[str, str]] = []
        self.last_image: str | None = None
        self.last_frame: bytes | None = None
        self.last_seen_jpeg: bytes | None = None
        self.image_dir = Path(tempfile.mkdtemp(prefix="ugrp-scene-"))
        self._image_count = 0
        self._frame_lock = threading.Lock()
        self._frame_condition = threading.Condition(self._frame_lock)
        self._frame_seq = 0
        self._frame_received_at = 0.0
        self._camera_pump_thread: threading.Thread | None = None
        self._camera_pump_stop = threading.Event()
        self._perception_thread: threading.Thread | None = None
        self._perception_stop = threading.Event()
        self._pose_thread: threading.Thread | None = None
        self._pose_stop = threading.Event()
        self._pose_proc = None
        self._low_latency_camera = False
        # Snapshot-backed SIM/cloud sources must be fetched on every /api/camera
        # request. They do not run the physical low-latency MJPEG pump, so the
        # generic cached-frame fast path would otherwise freeze the browser on
        # the one startup frame forever.
        self.prefer_fresh_camera_snapshot = False
        self._async_real_perception = False
        self._last_perception_at = 0.0
        self._last_pose_poll_at = 0.0
        self._real_pose = None
        self._real_pose_stable = False
        self._pose_stability = PoseStabilityTracker()
        self.sim_speed_control = os.environ.get("UGRP_SIM_SPEED_CONTROL") == "1"
        inferred_mode = "sim" if self.sim_speed_control or self.actions_path.name == "sim_actions.py" else "real"
        self.ui_mode = os.environ.get("UGRP_UI_MODE", inferred_mode).strip().lower() or inferred_mode
        if self.ui_mode == "real":
            restore_one_shot_real_grasp(self.state_estimator)
        # SIM executes the same short-lived precision approach->pick handoff as
        # REAL.  Both embodied modes therefore require the truthful destructive
        # pick preconditions; standalone/unit-test executives may still opt out.
        self.executive = TaskExecutive(strict_pick_preconditions=self.ui_mode in {"real", "sim"})
        try:
            stale_s = float(os.environ.get("UGRP_CAMERA_STALE_AFTER_S", "2.5"))
        except ValueError:
            stale_s = 2.5
        self.camera_stale_after_s = max(0.5, stale_s)
        self.sim_speed_file = Path(os.environ.get("UGRP_SIM_SPEED_FILE", "/tmp/ugrp_sim_speed"))
        self._sim_episode_id: int | None = None
        self._sim_health_lock = threading.Lock()
        self._sim_health_cache: dict[str, Any] | None = None
        self._sim_health_cached_at = 0.0
        self.cancel = threading.Event()
        # ThreadingHTTPServer can receive an operator turn and an automatic TEAM
        # wake at the same time. One robot owns one planner/executive state, so
        # overlapping turns would corrupt history/cancel/state handoffs.
        self._turn_lock = threading.Lock()
        self._team_dispatch_stop: threading.Event | None = None

    def turn_busy(self) -> bool:
        return self._turn_lock.locked()

    def remember_frame(self, jpeg: bytes, *, perceive: bool = True) -> None:
        """Publish a frame; planner perception can be explicitly separated from UI reads."""
        now = time.monotonic()
        with self._frame_condition:
            self.last_frame = jpeg
            self._frame_seq += 1
            self._frame_received_at = now
            self._frame_condition.notify_all()
        # Browser polling of snapshot-backed SIM cameras is presentation I/O,
        # not an ordered planner observation. Letting those concurrent requests
        # mutate WorldState races action-synchronous tool results/observations and
        # can overwrite a confirmed visible target with an arbitrary transient
        # motion frame. Planner-owned initial/post-action observations still
        # update the estimator through run_loop's image path.
        if not perceive:
            return
        # REAL production uses an asynchronous perception worker so camera I/O
        # cannot be stalled by colour detection/FK/pose telemetry.
        if self.ui_mode == "real" and self._async_real_perception:
            return
        self._process_perception(jpeg, now=now)

    def _process_perception(self, jpeg: bytes, *, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if now - self._last_perception_at < 0.15:
            return
        if self.ui_mode == "real":
            scene = detect_scene_bytes(jpeg)
            if scene is not None:
                pose = self._real_pose
                stable = self._real_pose_stable
                metric: dict[str, dict[str, Any]] = {}
                if pose is not None:
                    for color in ("red", "yellow", "blue"):
                        patch = metric_memory_entry(
                            pose.pose,
                            scene.get(color),
                            pose_age_s=pose.age_s,
                            pose_stable=stable,
                        )
                        if patch is not None:
                            metric[color] = patch
                self.state_estimator.update_scene_memory(scene, metric_estimates=metric)
        else:
            vision = detect_red_target_bytes(jpeg)
            if vision is not None:
                self.state_estimator.update_vision(vision)
        self._last_perception_at = now

    def cached_frame(self) -> bytes | None:
        with self._frame_lock:
            if self.last_frame is None:
                return None
            # REAL frames are sensor evidence, not timeless UI decoration.  A
            # disconnected camera must never keep an old physical frame alive
            # indefinitely (or leave the previous SIM frame visible in the UI).
            if self.ui_mode == "real":
                age = time.monotonic() - self._frame_received_at if self._frame_received_at > 0 else float("inf")
                if age > self.camera_stale_after_s:
                    return None
            return self.last_frame

    def cached_frame_info(self) -> tuple[bytes | None, int, float]:
        with self._frame_lock:
            return self.last_frame, self._frame_seq, self._frame_received_at

    def wait_for_frame(self, after_seq: int, timeout: float = 2.0) -> tuple[bytes | None, int, float]:
        deadline = time.monotonic() + max(0.0, timeout)
        with self._frame_condition:
            while self._frame_seq <= after_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._frame_condition.wait(remaining)
            return self.last_frame, self._frame_seq, self._frame_received_at

    def start_low_latency_camera(self, *, pose_stream: bool = True) -> None:
        """Start one upstream MJPEG reader and latest-frame fanout for REAL."""
        if self.open_stream is None:
            return
        self._low_latency_camera = True
        # Passive UI camera traffic must never mutate planner state. SIM already
        # uses this rule; REAL now follows the same ordered-observation contract.
        # Planner perception happens only inside run_loop on the exact initial or
        # post-action frame supplied to the LLM. Keep pose telemetry for tracing.
        self._async_real_perception = False
        self._camera_pump_stop.clear()
        if self._camera_pump_thread is None or not self._camera_pump_thread.is_alive():
            self._camera_pump_thread = threading.Thread(
                target=self._camera_pump_loop, name="ugrp-camera-pump", daemon=True
            )
            self._camera_pump_thread.start()
        if self.ui_mode == "real" and pose_stream:
            self._pose_stop.clear()
            if self._pose_thread is None or not self._pose_thread.is_alive():
                self._pose_thread = threading.Thread(
                    target=self._pose_loop, name="ugrp-pose-stream", daemon=True
                )
                self._pose_thread.start()

    def stop_low_latency_camera(self) -> None:
        self._camera_pump_stop.set()
        self._perception_stop.set()
        self._pose_stop.set()
        proc = self._pose_proc
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except Exception:
                pass
        with self._frame_condition:
            self._frame_condition.notify_all()

    def _camera_pump_loop(self) -> None:
        while not self._camera_pump_stop.is_set():
            stream = None
            try:
                stream = self.open_stream() if self.open_stream is not None else None
                if stream is None:
                    self._camera_pump_stop.wait(0.5)
                    continue
                buf = bytearray()
                while not self._camera_pump_stop.is_set():
                    chunk = stream.read(16384)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    frames = pop_jpegs(buf)
                    if frames:
                        # Deliberately drop every stale frame in this chunk.
                        self.remember_frame(frames[-1], perceive=False)
            except Exception:
                self._camera_pump_stop.wait(0.25)
            finally:
                if stream is not None:
                    try:
                        stream.close()
                    except Exception:
                        pass

    def _perception_loop(self) -> None:
        seq = 0
        while not self._perception_stop.is_set():
            frame, new_seq, _ = self.wait_for_frame(seq, timeout=0.5)
            if new_seq <= seq or frame is None:
                continue
            seq = new_seq
            try:
                self._process_perception(frame)
            except Exception:
                pass

    def _pose_loop(self) -> None:
        host = getattr(self.robot, "host", "ugrp1") if self.robot is not None else "ugrp1"
        while not self._pose_stop.is_set():
            proc = None
            try:
                proc = open_commanded_pose_stream(host=host)
                self._pose_proc = proc
                stdout = proc.stdout
                if stdout is None:
                    raise RuntimeError("pose stream has no stdout")
                while not self._pose_stop.is_set():
                    line = stdout.readline()
                    if not line:
                        break
                    pose = parse_commanded_pose(line)
                    if pose is None:
                        continue
                    self._real_pose = pose
                    self._real_pose_stable = self._pose_stability.observe(pose)
            except Exception:
                self._pose_stop.wait(0.5)
            finally:
                if proc is not None and proc.poll() is None:
                    try:
                        proc.terminate()
                    except Exception:
                        pass
                self._pose_proc = None

    def sim_speed(self) -> int:
        if not self.sim_speed_control:
            return 1
        try:
            value = int(float(self.sim_speed_file.read_text().strip()))
        except Exception:
            value = 1
        return value if value in (1, 2, 3) else 1

    def sim_bridge_health(self, *, force: bool = False) -> dict[str, Any]:
        """Read the authoritative SIM bridge health with a short cache.

        Browser status polling must not turn into one localhost request per
        tab/tick, but a local frame cache is not enough to prove that the
        shared-world worker is still alive. Keep this read-only and fail closed.
        """
        if self.ui_mode != "sim":
            return {}
        now = time.monotonic()
        with self._sim_health_lock:
            if not force and self._sim_health_cache is not None and now - self._sim_health_cached_at < 0.5:
                return dict(self._sim_health_cache)
        try:
            with urlopen("http://127.0.0.1:8091/health", timeout=0.5) as resp:
                value = json.loads(resp.read().decode())
            if not isinstance(value, dict):
                value = {"ok": False, "error": "invalid SIM health payload"}
        except Exception as exc:
            value = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        with self._sim_health_lock:
            self._sim_health_cache = dict(value)
            self._sim_health_cached_at = now
        return dict(value)

    def sim_camera_status(self) -> dict[str, Any]:
        """Return bridge-authoritative camera/worker readiness for this robot."""
        if self.ui_mode != "sim":
            _frame, seq, received = self.cached_frame_info()
            age = None if received <= 0 else max(0.0, time.monotonic() - received)
            return {
                "authoritative": False,
                "available": self.cached_frame() is not None,
                "frame_seq": seq,
                "frame_age_s": age,
            }
        health = self.sim_bridge_health()
        worker_ok = bool(health.get("remote_ws_connected") and health.get("remote_authoritative"))
        meta = health.get("robot_frame_meta") or {}
        robot_meta = meta.get(self.robot_id) if isinstance(meta, dict) else None
        generated = robot_meta.get("generated_wall_s") if isinstance(robot_meta, dict) else None
        try:
            age = max(0.0, time.time() - float(generated)) if generated is not None else None
        except (TypeError, ValueError):
            age = None
        fresh = age is not None and age <= self.camera_stale_after_s
        has_frame = isinstance(robot_meta, dict) and bool(robot_meta)
        return {
            "authoritative": worker_ok,
            # A quiet shared world does not make its last camera sample
            # semantically stale. The worker publishes a fresh controller frame
            # after every action; use age as observability, not as a talk-only
            # gate that disables an otherwise healthy SIM camera.
            "available": bool(worker_ok and has_frame),
            "frame_fresh": fresh,
            "worker_online": worker_ok,
            "frame_age_s": age,
            "remote_episode_id": health.get("remote_episode_id"),
            "error": health.get("error"),
        }

    def set_sim_speed(self, value: int) -> int:
        if not self.sim_speed_control:
            raise ValueError("simulation speed control is unavailable")
        value = int(value)
        if value not in (1, 2, 3):
            raise ValueError("speed must be 1, 2, or 3")
        self.sim_speed_file.write_text(str(value))
        try:
            req = Request(
                "http://127.0.0.1:8091/sim/speed",
                data=json.dumps({"speed": value}).encode(),
                headers=sim_bridge_headers(),
                method="POST",
            )
            with urlopen(req, timeout=1.5):
                pass
        except Exception:
            pass
        return value

    def sim_seed(self) -> int | None:
        if self.ui_mode != "sim":
            return None
        try:
            with urlopen("http://127.0.0.1:8091/health", timeout=0.5) as resp:
                obj = json.loads(resp.read().decode())
            seed = (obj.get("state") or {}).get("seed")
            return int(seed) if seed is not None else None
        except Exception:
            return None

    def sync_sim_episode(self) -> bool:
        """Reset planner/chat evidence when a replacement GPU owns a new world."""
        if self.ui_mode != "sim":
            return False
        obj = self.sim_bridge_health(force=True)
        try:
            episode_id = int(obj.get("remote_episode_id") or 0)
        except (TypeError, ValueError):
            return False
        if episode_id <= 0:
            return False
        if self._sim_episode_id is None:
            self._sim_episode_id = episode_id
            return False
        if episode_id == self._sim_episode_id:
            return False
        self._sim_episode_id = episode_id
        self.state_estimator.reset()
        self.action_queue.reset()
        self.history.clear()
        self.last_image = None
        self.last_seen_jpeg = None
        with self._frame_condition:
            self.last_frame = None
            self._frame_seq += 1
            self._frame_received_at = 0.0
            self._frame_condition.notify_all()
        self._last_perception_at = 0.0
        with self._sim_health_lock:
            self._sim_health_cache = None
            self._sim_health_cached_at = 0.0
        return True

    def reset_sim(self, seed: int) -> int:
        if self.ui_mode != "sim":
            raise ValueError("simulation reset is unavailable")
        if isinstance(seed, bool):
            raise ValueError("seed must be an integer")
        seed = int(seed)
        if seed < 0 or seed > 2**31 - 1:
            raise ValueError("seed must be between 0 and 2147483647")
        req = Request(
            "http://127.0.0.1:8091/command",
            data=json.dumps({"action": "reset", "seed": seed}).encode(),
            headers=sim_bridge_headers(),
            method="POST",
        )
        with urlopen(req, timeout=45) as resp:
            result = json.loads(resp.read().decode())
        if result.get("ok") is False:
            raise RuntimeError(result.get("reason") or "simulation reset failed")
        # A seeded reset starts a new episode. Do not carry planner-side temporal
        # evidence or chat action context across the episode boundary.
        self.cancel.set()
        self.state_estimator.reset()
        self.action_queue.reset()
        self.history.clear()
        self.last_image = None
        self.last_seen_jpeg = None
        with self._frame_condition:
            self.last_frame = None
            self._frame_seq += 1
            self._frame_received_at = 0.0
            self._frame_condition.notify_all()
        self._last_perception_at = 0.0
        with self._sim_health_lock:
            self._sim_health_cache = None
            self._sim_health_cached_at = 0.0
        # Seed reset is a new shared-world episode. Peer proposals/ACKs from the
        # previous episode are temporal evidence and must not survive it. REAL
        # coordination is a separate namespace and is intentionally untouched.
        team_bus.reset(namespace="sim")
        return seed



def authorized(handler: BaseHTTPRequestHandler, password: str | None) -> bool:
    if not password:
        return True
    header = handler.headers.get("Authorization", "")
    if not header.startswith("Basic "):
        return False
    try:
        decoded = base64.b64decode(header.split(" ", 1)[1]).decode("utf-8")
    except (ValueError, UnicodeDecodeError):
        return False
    if ":" not in decoded:
        return False
    _user, supplied = decoded.split(":", 1)
    return hmac.compare_digest(supplied.encode("utf-8"), password.encode("utf-8"))


def save_data_url(data_url: str, directory: Path, index: int) -> str:
    header, _, payload = data_url.partition(",")
    if not payload:
        raise ValueError("invalid image")
    mime = "image/jpeg"
    if header.startswith("data:") and ";base64" in header:
        mime = header[5:].split(";", 1)[0].strip() or mime
    suffix = IMAGE_TYPES.get(mime)
    if suffix is None:
        raise ValueError("unsupported image type")
    raw = base64.b64decode(payload, validate=False)
    if len(raw) > MAX_IMAGE_BYTES:
        raise ValueError("image too large")
    path = directory / f"scene-{index}{suffix}"
    path.write_bytes(raw)
    return str(path)


class ChatHandler(BaseHTTPRequestHandler):
    state: ChatState

    def log_message(self, format: str, *args: Any) -> None:
        # High-frequency read-only telemetry is expected browser traffic, not an
        # operational event. Avoid flooding journald and spending CPU formatting
        # several lines per second per open tab. Keep commands/turns/errors visible.
        path = urlparse(getattr(self, "path", "")).path
        if path in {"/api/sim/state", "/api/status", "/api/camera", "/api/camera/stream"} or (
            path.startswith("/api/observer/") and path.endswith("/stream")
        ):
            return
        sys_stderr = __import__("sys").stderr
        sys_stderr.write("%s - %s\n" % (self.address_string(), format % args))

    def _deny(self) -> None:
        self.send_response(401)
        self.send_header("WWW-Authenticate", 'Basic realm="UGRP"')
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write(b"password required")

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        try:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, OSError):
            # Browser camera/status polling is intentionally disposable. A tab
            # replacing an <img> or navigating away can close these sockets at
            # any point; that must not turn into a noisy server traceback.
            self.close_connection = True

    def _send_json(self, code: int, payload: dict[str, Any]) -> None:
        self._send(
            code,
            json.dumps(payload, ensure_ascii=False).encode(),
            "application/json; charset=utf-8",
        )

    def _begin_sse(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        # A physical/sim skill can block for several seconds without producing a
        # planner event. Keep the HTTP stream active so browsers/Tailscale do not
        # mistake that quiet interval for a dead request. All writes share one
        # lock because the heartbeat runs beside the synchronous robot loop.
        self._sse_write_lock = threading.Lock()
        self._sse_heartbeat_stop = threading.Event()
        self._sse_heartbeat_thread = threading.Thread(
            target=self._sse_heartbeat_loop,
            name=f"ugrp-sse-heartbeat-{self.state.robot_id}",
            daemon=True,
        )
        self._sse_heartbeat_thread.start()

    def _sse_write(self, data: bytes) -> bool:
        lock = getattr(self, "_sse_write_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._sse_write_lock = lock
        try:
            with lock:
                self.wfile.write(data)
                self.wfile.flush()
            return True
        except (BrokenPipeError, ConnectionResetError, OSError):
            # A browser tab/aborted fetch must not leave a long robot/LLM loop
            # running invisibly. REAL can also terminate its active child; SIM
            # observes the flag as soon as the current bridge action returns.
            self._request_execution_cancel()
            return False

    def _request_execution_cancel(self) -> bool:
        self.state.cancel.set()
        terminated = False
        cancel_current = getattr(self.state.registry, "cancel_current", None)
        if callable(cancel_current):
            try:
                terminated = bool(cancel_current())
            except Exception as exc:
                # The loop cancellation flag remains authoritative even if a
                # best-effort child termination hook itself has a problem.
                print(f"action cancellation hook failed: {exc}", file=sys.stderr, flush=True)
        return terminated

    def _sse_heartbeat_loop(self) -> None:
        stop = self._sse_heartbeat_stop
        interval = max(1.0, float(os.environ.get("UGRP_SSE_HEARTBEAT_S", "3.0")))
        while not stop.wait(interval):
            # SSE comments are deliberately invisible to readEvents(), but they
            # keep every proxy/socket hop active during a long skill call.
            if not self._sse_write(b": keepalive\n\n"):
                return

    def _stop_sse_heartbeat(self) -> None:
        stop = getattr(self, "_sse_heartbeat_stop", None)
        if stop is None:
            return
        stop.set()
        thread = getattr(self, "_sse_heartbeat_thread", None)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=0.25)
        self._sse_heartbeat_stop = None
        self._sse_heartbeat_thread = None

    def _sse(self, payload: dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False)
        self._sse_write(f"data: {data}\n\n".encode())

    def _proxy_backend(self, method: str, *, prefix: str, port: int, label: str) -> None:
        parsed = urlparse(self.path)
        upstream_path = parsed.path[len(prefix):] or "/"
        if parsed.query:
            upstream_path += "?" + parsed.query
        url = f"http://127.0.0.1:{int(port)}" + upstream_path
        data = None
        headers = {}
        if method == "POST":
            length = int(self.headers.get("Content-Length", "0"))
            if length < 0 or length > MAX_REQUEST_BODY_BYTES:
                self._send_json(413, {"ok": False, "error": f"request body must be 0..{MAX_REQUEST_BODY_BYTES} bytes"})
                return
            data = self.rfile.read(length) if length else b""
            if self.headers.get("Content-Type"):
                headers["Content-Type"] = self.headers.get("Content-Type")
        req = Request(url, data=data, headers=headers, method=method)
        try:
            resp = urlopen(req, timeout=180 if upstream_path.startswith("/api/turn") else 45)
        except HTTPError as exc:
            resp = exc
        except URLError as exc:
            self._send_json(502, {"error": f"{label} backend unavailable: {exc}"})
            return
        try:
            self.send_response(getattr(resp, "status", 200))
            ctype = resp.headers.get("Content-Type", "application/octet-stream")
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            if "text/event-stream" in ctype:
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                while True:
                    line = resp.readline()
                    if not line:
                        break
                    self.wfile.write(line); self.wfile.flush()
            elif "multipart/" in ctype:
                self.end_headers()
                while True:
                    reader = getattr(resp, "read1", None)
                    chunk = reader(65536) if reader is not None else resp.read(4096)
                    if not chunk:
                        break
                    self.wfile.write(chunk); self.wfile.flush()
            else:
                body = resp.read()
                self.send_header("Content-Length", str(len(body)))
                self.end_headers(); self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try: resp.close()
            except Exception: pass

    def _proxy_real(self, method: str) -> None:
        """Legacy /real/* alias remains REAL R1 for old bookmarks/tests."""
        self._proxy_backend(method, prefix="/real", port=BACKEND_PORTS[("real", "r1")], label="REAL R1")

    def _maybe_proxy_robot_backend(self, method: str, path: str) -> bool:
        routes = (
            ("/real/r1", ("real", "r1")),
            ("/real/r2", ("real", "r2")),
            ("/real/r3", ("real", "r3")),
            ("/sim/r1", ("sim", "r1")),
            ("/sim/r2", ("sim", "r2")),
            ("/sim/r3", ("sim", "r3")),
        )
        for prefix, key in routes:
            if path == prefix or path.startswith(prefix + "/"):
                self._proxy_backend(method, prefix=prefix, port=BACKEND_PORTS[key], label=f"{key[0].upper()} {key[1].upper()}")
                return True
        # Preserve the original /real/api/* path as R1 after the explicit robot
        # prefixes have had first chance to match.
        if path == "/real" or path.startswith("/real/"):
            self._proxy_real(method)
            return True
        return False

    def _team_payload(self, mode: str) -> dict[str, Any]:
        mode = "real" if mode == "real" else "sim"
        bus = team_bus.snapshot(namespace=mode)
        robots: dict[str, Any] = {}
        mission: dict[str, Any] | None = None
        if mode == "sim":
            try:
                with urlopen("http://127.0.0.1:8091/health", timeout=1.5) as resp:
                    health = json.loads(resp.read().decode())
                states = health.get("robots") or ((health.get("state") or {}).get("robots") if isinstance(health.get("state"), dict) else {}) or {}
                frame_meta = health.get("robot_frame_meta") or {}
                world_state = health.get("state") if isinstance(health.get("state"), dict) else {}
                raw_mission = None
                if isinstance(world_state, dict):
                    raw_mission = world_state.get("warehouse") or world_state.get("cooperative_payload")
                mission = dict(raw_mission) if isinstance(raw_mission, dict) else None
                worker_online = bool(health.get("remote_ws_connected") and health.get("remote_authoritative"))
                for rid in team_bus.ROBOT_IDS:
                    # A connected legacy/single-robot worker is not evidence
                    # that all three slots exist. Require namespaced publication
                    # from this robot before TEAM marks the slot online.
                    slot_present = rid in states or rid in frame_meta
                    robots[rid] = {
                        "backend_online": bool(worker_online and slot_present),
                        "worker_online": worker_online,
                        "state": states.get(rid) or {},
                    }
            except Exception as exc:
                for rid in team_bus.ROBOT_IDS:
                    robots[rid] = {"backend_online": False, "state": {}, "error": str(exc)}
        else:
            for rid in team_bus.ROBOT_IDS:
                port = BACKEND_PORTS[("real", rid)]
                try:
                    with urlopen(f"http://127.0.0.1:{port}/api/status", timeout=1.2) as resp:
                        status = json.loads(resp.read().decode())
                    robots[rid] = {
                        "backend_online": bool(status.get("robot_configured", True)),
                        "status": status,
                    }
                except Exception as exc:
                    robots[rid] = {"backend_online": False, "status": {}, "error": str(exc)}
        return {
            "ok": True,
            "mode": mode,
            "coordination": "decentralized_peer_bus",
            "goal": bus.get("goal"),
            "chat": bus.get("chat", []),
            "messages": bus.get("messages", []),
            "wakeups": bus.get("wakeups", [])[-80:],
            "events": bus.get("events", []),
            "mission": mission,
            "robots": robots,
            "updated_at": bus.get("updated_at"),
        }

    def _team_camera(self, mode: str, rid: str) -> None:
        try:
            if mode == "sim" and str(rid).strip().lower() == "overview":
                url = "http://127.0.0.1:8091/observer/team_overview/snapshot"
                with urlopen(url, timeout=4.0) as resp:
                    jpeg = resp.read()
                if not jpeg.startswith(b"\xff\xd8"):
                    raise ValueError("camera response is not JPEG")
                self._send(200, jpeg, "image/jpeg")
                return
            rid = team_bus.normalize_robot_id(rid)
            if mode == "sim":
                url = f"http://127.0.0.1:8091/robot/{rid}/snapshot"
            else:
                url = f"http://127.0.0.1:{BACKEND_PORTS[("real", rid)]}/api/camera"
            with urlopen(url, timeout=4.0) as resp:
                jpeg = resp.read()
            if not jpeg.startswith(b"\xff\xd8"):
                raise ValueError("camera response is not JPEG")
        except Exception as exc:
            try:
                self._send(503, str(exc).encode("utf-8", "replace"), "text/plain; charset=utf-8")
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return
        try:
            self._send(200, jpeg, "image/jpeg")
        except (BrokenPipeError, ConnectionResetError, OSError):
            # TEAM camera snapshots are presentation-only. Navigating away or
            # replacing an <img> may abort a request; that is not a server fault.
            return

    def do_GET(self) -> None:
        if not authorized(self, self.state.password):
            self._deny()
            return
        path = urlparse(self.path).path
        if self._maybe_proxy_robot_backend("GET", path):
            return
        if path == "/api/team":
            query = __import__("urllib.parse", fromlist=["parse_qs"]).parse_qs(urlparse(self.path).query)
            mode = (query.get("mode") or ["sim"])[0]
            self._send_json(200, self._team_payload(mode))
            return
        if path.startswith("/api/team/camera/"):
            parts = [x for x in path.split("/") if x]
            if len(parts) == 5 and parts[:3] == ["api", "team", "camera"]:
                self._team_camera(parts[3], parts[4]); return
            self._send_json(404, {"error": "team camera route not found"}); return
        if path in {"/", "/index.html"}:
            self._send(200, STATIC.read_bytes(), "text/html; charset=utf-8")
            return
        if path == "/three.min.js":
            self._send(200, THREE_JS.read_bytes(), "application/javascript; charset=utf-8")
            return
        if path == "/sim3d.js":
            self._send(200, SIM3D_JS.read_bytes(), "application/javascript; charset=utf-8")
            return
        if path == "/sim-scene.xml":
            self._send(200, SIM_SCENE_XML.read_bytes(), "application/xml; charset=utf-8")
            return
        if path.startswith("/offline/"):
            name = Path(path[len("/offline/"):]).name
            target = OFFLINE_PREVIEW_DIR / name
            if target.exists() and target.is_file() and target.suffix.lower() in {".jpg", ".jpeg"}:
                self._send(200, target.read_bytes(), "image/jpeg")
            else:
                self._send(404, b"not found", "text/plain; charset=utf-8")
            return
        if path.startswith("/artifacts/test-videos/"):
            name = Path(path[len("/artifacts/test-videos/"):]).name
            target = TEST_VIDEO_DIR / name
            if target.exists() and target.is_file() and target.suffix.lower() == ".mp4":
                self._send(200, target.read_bytes(), "video/mp4")
            else:
                self._send(404, b"not found", "text/plain; charset=utf-8")
            return
        if path == "/api/sim/state":
            if self.state.ui_mode != "sim":
                self._send_json(404, {"error": "simulation state unavailable"})
                return
            try:
                with urlopen("http://127.0.0.1:8091/health", timeout=1.5) as resp:
                    obj = json.loads(resp.read().decode())
                recovery = {}
                try:
                    recovery = json.loads(GPU_RECOVERY_STATUS.read_text()) if GPU_RECOVERY_STATUS.exists() else {}
                except Exception:
                    recovery = {}
                team_state = obj.get("state") or {}
                robot_states = obj.get("robots") or (team_state.get("robots") if isinstance(team_state, dict) else {}) or {}
                worker_state = robot_states.get(self.state.robot_id) or team_state
                calibration = {}
                try:
                    calibration = json.loads(SIM_CALIBRATION_JSON.read_text()) if SIM_CALIBRATION_JSON.exists() else {}
                except Exception:
                    calibration = {}
                self._send_json(200, {
                    "ok": True,
                    "worker_age_s": obj.get("worker_age_s"),
                    "remote_ws_connected": bool(obj.get("remote_ws_connected")),
                    "remote_ws_count": int(obj.get("remote_ws_count") or 0),
                    "remote_authoritative": bool(obj.get("remote_authoritative")),
                    "remote_provider": obj.get("remote_provider"),
                    "remote_machine": obj.get("remote_machine"),
                    "remote_instance_id": obj.get("remote_instance_id"),
                    "gpu_only": bool(obj.get("gpu_only")),
                    "gpu_recovery": recovery,
                    "scene_model": "masterpi_multi_v2_shared_world",
                    "physics_model": worker_state.get("model") or team_state.get("model") or "unknown",
                    "robot_id": self.state.robot_id,
                    "team_state": team_state,
                    "training_ready": calibration.get("validated") is True,
                    "calibration_fit_trials": int((calibration.get("results") or {}).get("fit_trials") or 0),
                    "calibration_held_out_trials": int((calibration.get("results") or {}).get("held_out_trials") or 0),
                    "state": worker_state,
                })
            except Exception as exc:
                self._send_json(503, {"ok": False, "error": str(exc), "state": {}})
            return
        if path == "/api/camera":
            self._camera()
            return
        if path == "/api/camera/stream":
            self._camera_stream()
            return
        if path == "/api/observer/stream":
            self._observer_stream()
            return
        if path == "/api/observer/snapshot":
            self._observer_snapshot()
            return
        if path.startswith("/api/observer/") and path.endswith("/stream"):
            name = path[len("/api/observer/"):-len("/stream")].strip("/")
            self._observer_stream(name)
            return
        if path.startswith("/api/observer/") and path.endswith("/snapshot"):
            name = path[len("/api/observer/"):-len("/snapshot")].strip("/")
            self._observer_snapshot(name)
            return
        if path == "/api/tools":
            self._send_json(
                200,
                tools_payload(self.state.registry, source=self.state.actions_path),
            )
            return
        if path == "/api/real-traces":
            if self.state.ui_mode != "real":
                self._send_json(404, {"error": "REAL traces are available only from the REAL backend"})
                return
            self._send_json(200, real_trace.trace_index(limit=12))
            return
        if path.startswith("/api/real-traces/"):
            if self.state.ui_mode != "real":
                self._send_json(404, {"error": "REAL traces are available only from the REAL backend"})
                return
            tail = path[len("/api/real-traces/"):]
            run_id, sep, asset_rel = tail.partition("/asset/")
            run_id = unquote(run_id)
            if sep:
                asset = real_trace.trace_asset(run_id, unquote(asset_rel))
                if asset is None:
                    self._send(404, b"trace asset not found", "text/plain; charset=utf-8")
                    return
                ctype = {
                    ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
                    ".png": "image/png", ".webp": "image/webp",
                }.get(asset.suffix.lower(), "application/octet-stream")
                self._send(200, asset.read_bytes(), ctype)
                return
            detail = real_trace.trace_detail(run_id)
            if detail is None:
                self._send_json(404, {"error": "trace run not found"})
                return
            self._send_json(200, detail)
            return
        if path == "/api/status":
            sim_camera = self.state.sim_camera_status()
            if self.state.ui_mode == "sim":
                talk_only = not bool(sim_camera.get("available"))
            else:
                talk_only = self.state.cached_frame() is None
            self._send_json(
                200,
                {
                    "execute_allowed": bool(self.state.execute),
                    "execute_default": os.environ.get("UGRP_UI_EXECUTE_DEFAULT") == "1",
                    "max_steps": self.state.max_steps,
                    "turn_busy": self.state.turn_busy(),
                    "action_queue": self.state.action_queue.snapshot(),
                    "talk_only": talk_only,
                    "camera_authoritative": bool(sim_camera.get("authoritative")),
                    "camera_available": bool(sim_camera.get("available")),
                    "camera_frame_fresh": sim_camera.get("frame_fresh"),
                    "camera_worker_online": sim_camera.get("worker_online"),
                    "camera_authoritative_age_s": sim_camera.get("frame_age_s"),
                    "sim_remote_episode_id": sim_camera.get("remote_episode_id"),
                    "camera_low_latency": bool(self.state._low_latency_camera),
                    "camera_frame_seq": self.state.cached_frame_info()[1],
                    "camera_frame_age_ms": (
                        None
                        if self.state.cached_frame_info()[2] <= 0
                        else round((time.monotonic() - self.state.cached_frame_info()[2]) * 1000.0, 1)
                    ),
                    "world_state": self.state.state_estimator.state.public(),
                    "architecture": "world-state-executive-v1",
                    "robot_id": self.state.robot_id,
                    "robot_configured": os.environ.get("UGRP_REAL_PLACEHOLDER") != "1",
                    "robot_endpoint": (
                        getattr(self.state.robot, "host", None)
                        if self.state.ui_mode == "real"
                        and self.state.robot is not None
                        and os.environ.get("UGRP_REAL_PLACEHOLDER") != "1"
                        else None
                    ),
                    "coordination": "decentralized_peer_bus",
                    "ui_mode": self.state.ui_mode,
                    "sim_speed_control": self.state.sim_speed_control,
                    "sim_speed": self.state.sim_speed(),
                    "sim_seed": self.state.sim_seed() if self.state.ui_mode == "sim" else None,
                },
            )
            return
        if path == "/api/robot":
            self._robot_get()
            return
        self._send(404, b"not found", "text/plain; charset=utf-8")

    def do_POST(self) -> None:
        if not authorized(self, self.state.password):
            self._deny()
            return
        path = urlparse(self.path).path
        if self._maybe_proxy_robot_backend("POST", path):
            return
        if path == "/api/team/chat":
            try:
                payload = self._read_json()
                mode = "real" if str(payload.get("mode") or self.state.ui_mode).strip().lower() == "real" else "sim"
                message = str(payload.get("message") or payload.get("text") or "")
                transfer = maybe_parse_transfer_command(message) if mode == "sim" else None
                intent = transfer.as_dict() if transfer is not None else None
                tower_workflow = mode == "sim" and _team_tower_intent(message)
                posted = team_bus.post_operator_chat(
                    message, namespace=mode, intent_override=intent,
                )
                # TEAM room commands are also the current shared goal when they
                # address the whole room. This keeps stale manual goal text from
                # poisoning the next autonomous turn, while @R2-specific chat
                # remains local and does not rewrite the other robots' goal.
                agent_intent, _needs_scene = turn_intent(message)
                goal = None
                if agent_intent and (
                    tower_workflow or set(posted.get("targets") or ()) == set(team_bus.ROBOT_IDS)
                ):
                    goal = team_bus.set_goal(message, source="operator_chat", namespace=mode)
                    team_bus.record_event(
                        self.state.robot_id, "team_goal", namespace=mode,
                        goal=goal.get("text"), source="operator_chat",
                    )
                self._send_json(200, {
                    "ok": True, "mode": mode, "goal": goal,
                    "intent": intent, **posted,
                })
            except Exception as exc:
                self._send_json(400, {"ok": False, "error": str(exc)})
            return
        if path == "/api/team/goal":
            try:
                payload = self._read_json()
                mode = "real" if str(payload.get("mode") or self.state.ui_mode).strip().lower() == "real" else "sim"
                goal = team_bus.set_goal(str(payload.get("goal") or payload.get("message") or ""), source="operator", namespace=mode)
                team_bus.record_event(self.state.robot_id, "team_goal", namespace=mode, goal=goal.get("text"))
                self._send_json(200, {"ok": True, "goal": goal})
            except Exception as exc:
                self._send_json(400, {"ok": False, "error": str(exc)})
            return
        if path == "/api/team/reset":
            payload = self._read_json()
            mode = "real" if str(payload.get("mode") or self.state.ui_mode).strip().lower() == "real" else "sim"
            self._send_json(200, {"ok": True, "team": team_bus.reset(namespace=mode)})
            return
        if path == "/api/turn":
            self._turn()
            return
        if path == "/api/warehouse/decision":
            if self.state.ui_mode != "sim":
                self._send_json(403, {"ok": False, "error": "warehouse research is SIM-only"})
                return
            try:
                from .warehouse_runtime import LLMPolicy
                payload = self._read_json()
                context = payload.get("context")
                if not isinstance(context, dict) or context.get("robot_id") != self.state.robot_id:
                    raise ValueError("robot-local decision context required")
                coela = payload.get("task_mode") == "coela"
                allowed = ({"robot_id", "team_ids", "manifest", "destination", "communication_mode", "observation", "observation_age_sim_s", "memory", "own_active_task", "own_pending_claim", "own_recovery_events"}
                           if coela else {"robot_id", "round", "observation", "manifest", "inbox", "own_history", "last_result", "available_ids", "idle_robots", "recovery_event"})
                if set(context) - allowed:
                    raise ValueError("unexpected actor context")
                if len(json.dumps(context)) > 48000:
                    raise ValueError("actor context too large")
                tokens = int(payload.get("response_tokens", 512))
                if not 128 <= tokens <= 2048:
                    raise ValueError("invalid output budget")
                if not self.state._turn_lock.acquire(blocking=False):
                    self._send_json(409, {"ok": False, "error": "robot_busy"})
                    return
                previous_tokens = getattr(self.state.completer, "max_tokens", None)
                try:
                    if coela:
                        from .coela_modules import Planner
                        if previous_tokens is not None: self.state.completer.max_tokens = tokens
                        policy = Planner(self.state.robot_id, self.state.completer)
                    elif payload.get("task_mode") == "mixed":
                        from .mixed_warehouse_runtime import MixedLLMPolicy
                        policy = MixedLLMPolicy(self.state.robot_id, self.state.completer)
                    else:
                        policy = LLMPolicy(self.state.robot_id, self.state.completer, response_tokens=tokens)
                    answer = policy.decide(context)
                finally:
                    if coela and previous_tokens is not None: self.state.completer.max_tokens = previous_tokens
                    self.state._turn_lock.release()
                self._send_json(200, {**answer, "model": policy.model_name})
            except Exception as exc:
                self._send_json(400, {"ok": False, "error": str(exc),
                    "raw": getattr(exc, "raw", None), "usage": getattr(exc, "usage", None)})
            return
        if path == "/api/sim/wake":
            if self.state.ui_mode != "sim":
                self._send_json(404, {"ok": False, "error": "SIM GPU wake is available only from the SIM backend"})
                return
            try:
                health = {}
                try:
                    with urlopen("http://127.0.0.1:8091/health", timeout=1.5) as resp:
                        health = json.loads(resp.read().decode())
                except Exception:
                    health = {}
                online = bool(health.get("remote_ws_connected") and health.get("remote_authoritative"))
                started, recovery_detail = (False, "")
                if not online:
                    started, recovery_detail = request_gpu_recovery()
                recovery = {}
                try:
                    recovery = json.loads(GPU_RECOVERY_STATUS.read_text()) if GPU_RECOVERY_STATUS.exists() else {}
                except Exception:
                    recovery = {}
                self._send_json(200, {
                    "ok": True,
                    "sim_online": online,
                    "recovery_started": started,
                    "recovery_detail": recovery_detail,
                    "gpu_recovery": recovery,
                })
            except Exception as exc:
                self._send_json(503, {"ok": False, "error": str(exc)})
            return
        if path == "/api/sim/activity":
            try:
                req = Request("http://127.0.0.1:8091/sim/user-activity", data=b"{}", headers={"Content-Type": "application/json"}, method="POST")
                with urlopen(req, timeout=1.5) as resp:
                    _ = resp.read()
                online = False
                try:
                    with urlopen("http://127.0.0.1:8091/health", timeout=1.5) as resp:
                        health = json.loads(resp.read().decode())
                    online = bool(health.get("remote_ws_connected") and health.get("remote_authoritative"))
                except Exception:
                    pass
                if not online:
                    # User-visible SIM activity may wake a configured GPU provider.
                    # Provider selection stays in gpu_worker_recover.py; the UI must
                    # never hard-code a paid cloud backend or start one while idle.
                    # No-op on hosts without systemctl (Mac local stack).
                    request_gpu_recovery()
                self._send_json(200, {"ok": True, "sim_online": online})
            except Exception as exc:
                self._send_json(503, {"ok": False, "error": str(exc)})
            return
        if path == "/api/sim/speed":
            try:
                payload = self._read_json()
                speed = self.state.set_sim_speed(int(payload.get("speed", 1)))
            except (ValueError, TypeError):
                self._send_json(400, {"error": "speed must be 1, 2, or 3"})
                return
            self._send_json(200, {"ok": True, "sim_speed": speed})
            return
        if path == "/api/sim/reset":
            try:
                payload = self._read_json()
                raw_seed = payload.get("seed")
                if raw_seed is None:
                    raise ValueError("seed is required")
                seed = self.state.reset_sim(raw_seed)
            except (ValueError, TypeError) as exc:
                self._send_json(400, {"error": str(exc)})
                return
            except Exception as exc:
                self._send_json(502, {"error": str(exc)})
                return
            self._send_json(200, {"ok": True, "sim_seed": seed})
            return
        if path == "/api/cancel":
            terminated = self._request_execution_cancel()
            self._send_json(200, {"ok": True, "action_terminated": terminated})
            return
        if path == "/api/robot/drive":
            self._robot_command("drive")
            return
        if path == "/api/robot/arm":
            self._robot_command("arm")
            return
        if path == "/api/robot/stop":
            self._robot_command("stop")
            return
        self._send(404, b"not found", "text/plain; charset=utf-8")

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length < 0 or length > MAX_REQUEST_BODY_BYTES:
            raise ValueError(f"request body must be 0..{MAX_REQUEST_BODY_BYTES} bytes")
        raw = self.rfile.read(length) if length else b"{}"
        payload = json.loads(raw.decode("utf-8") or "{}")
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def _turn(self) -> None:
        if not self.state._turn_lock.acquire(blocking=False):
            self._send_json(409, {"ok": False, "error": "robot_busy", "robot_id": self.state.robot_id})
            return
        try:
            self._turn_locked()
        finally:
            # _begin_sse() is optional and may be reached by several early-return
            # branches, so the request wrapper owns heartbeat cleanup.
            self._stop_sse_heartbeat()
            self.state._turn_lock.release()

    def _turn_locked(self) -> None:
        try:
            payload = self._read_json()
        except (json.JSONDecodeError, ValueError):
            self._send(400, json.dumps({"error": "invalid JSON"}).encode(), "application/json")
            return
        message = str(payload.get("message") or payload.get("text") or "")
        image = payload.get("image")
        chat_only = bool(payload.get("chat_only"))
        team_wake = bool(payload.get("team_wake"))
        agent_intent, visual_intent = (False, False) if chat_only else turn_intent(message)
        execute_requested = bool(payload.get("execute"))
        execute = bool(self.state.execute) and execute_requested and agent_intent
        stream = bool(payload.get("stream"))
        print(
            f"turn intent agent={agent_intent} visual={visual_intent} "
            f"execute_requested={execute_requested} execute_effective={execute} "
            f"message={message[:120]!r}",
            flush=True,
        )
        self.state.cancel.clear()
        # PROBABLE_HELD is weak camera-only evidence, not a positive gripper
        # sensor.  Before a new REAL physical request, reconcile it against the
        # Pi-side precision-pick handoff.  This SSH read never moves the robot.
        # Transport failure leaves the conservative belief untouched.
        if (
            self.state.ui_mode == "real"
            and execute
            and agent_intent
            and self.state.state_estimator.state.grasp.state == "PROBABLE_HELD"
        ):
            carry = read_carry_evidence(host=os.environ.get("UGRP_ROBOT_HOST", "ugrp1"))
            changed = self.state.state_estimator.reconcile_probable_carry(carry.public())
            if changed:
                print(
                    f"weak carry belief cleared before action: {carry.reason} "
                    f"age_s={carry.age_s!r}",
                    flush=True,
                )
        empty_correction = explicit_empty_gripper_statement(message) if not agent_intent else False
        if empty_correction:
            changed = self.state.state_estimator.accept_operator_empty_gripper()
            if changed:
                ack = "알겠어. 현재 집게는 빈 상태로 정정했어."
            else:
                ack = "현재는 더 강한 grasp 증거가 있어서 텍스트만으로 HELD 상태를 지우지는 않았어."
            if not team_wake:
                self.state.history.append({"role": "user", "content": message.strip()})
                self.state.history.append({"role": "assistant", "content": ack})
            body = {
                "text": ack, "final": ack, "stopped": "final",
                "log": "", "events": [{"type": "final", "text": ack}],
                "tools": [], "pending_plan": [],
                "world_state": self.state.state_estimator.state.public(),
            }
            if stream:
                self._begin_sse()
                self._sse({"type": "status", "text": "집게 상태를 정정했습니다"})
                self._sse({"type": "final", "text": ack})
                self._sse({"type": "done", **body})
            else:
                self._send_json(200, body)
            return
        context_ack = spatial_context_statement(message) if not agent_intent else None
        if context_ack is not None:
            # Do not ask the VLM to embellish a simple user-provided spatial
            # correction. Preserve it in conversation history for the next
            # actual command and return a deterministic acknowledgement.
            if not team_wake:
                self.state.history.append({"role": "user", "content": message.strip()})
                self.state.history.append({"role": "assistant", "content": context_ack})
            body = {
                "text": context_ack, "final": context_ack, "stopped": "final",
                "log": "", "events": [{"type": "final", "text": context_ack}],
                "tools": [], "pending_plan": [],
                "world_state": self.state.state_estimator.state.public(),
            }
            if stream:
                self._begin_sse()
                self._sse({"type": "status", "text": "위치 힌트를 반영했습니다"})
                self._sse({"type": "final", "text": context_ack})
                self._sse({"type": "done", **body})
            else:
                self._send_json(200, body)
            return
        episode_reset = self.state.sync_sim_episode() if self.state.ui_mode == "sim" else False
        if stream:
            self._begin_sse()
            self._sse({"type": "status", "text": "요청을 받았습니다"})
            if episode_reset:
                self._sse({"type": "status", "text": "GPU가 새 SIM episode로 복구되어 상태를 초기화했습니다"})
            if agent_intent:
                self._sse({
                    "type": "status",
                    "text": "로봇 실행을 준비하고 있습니다" if execute else "로봇 명령을 확인하고 있습니다 · 실제 실행 꺼짐",
                })
        try:
            image_path = self._resolve_image(image, include_camera=visual_intent)
        except (ValueError, OSError) as exc:
            if stream:
                self._sse({"type": "error", "text": f"image: {exc}"})
                self._sse({"type": "done", "error": f"image: {exc}", "text": "", "events": [], "tools": []})
                return
            self._send_json(200, {"error": f"image: {exc}", "text": ""})
            return

        if self.state.ui_mode == "real" and execute and image_path is None:
            text = "카메라 프레임이 아직 들어오지 않아 로봇을 움직이지 않았어."
            body = {
                "text": text, "final": text, "stopped": "camera_unavailable",
                "log": "", "events": [{"type": "final", "text": text}],
                "tools": [], "pending_plan": [],
                "world_state": self.state.state_estimator.state.public(),
            }
            if stream:
                self._sse({"type": "status", "text": "카메라 연결을 확인하고 있습니다"})
                self._sse({"type": "final", "text": text})
                self._sse({"type": "done", **body})
            else:
                self._send_json(200, body)
            return

        if self.state.ui_mode == "sim" and execute and agent_intent and image_path is None:
            # A cloud worker can be reconnecting while a user submits a command.
            # Never feed that transient offline state into the agent loop: doing
            # so used to emit repeated offline errors and could end with a false
            # statement that queued motions had run.
            text = "SIM GPU/카메라 프레임이 없어 이번 명령은 실행하지 않았어. GPU 복구 후 다시 시도해줘."
            body = {
                "text": text, "final": text, "stopped": "gpu_offline",
                "log": "", "events": [{"type": "final", "text": text}],
                "tools": [], "pending_plan": [],
                "world_state": self.state.state_estimator.state.public(),
            }
            if stream:
                self._sse({"type": "status", "text": "SIM GPU 연결을 확인하고 있습니다"})
                self._sse({"type": "final", "text": text})
                self._sse({"type": "done", **body})
            else:
                self._send_json(200, body)
            return

        def trace_pose() -> tuple[dict[int, int] | None, float | None, bool | None]:
            pose = self.state._real_pose
            if pose is None:
                return None, None, None
            return dict(pose.pose), pose.age_s, bool(self.state._real_pose_stable)

        trace_run_id: str | None = None
        trace_run_dir: Path | None = None
        if execute and agent_intent:
            try:
                if self.state.ui_mode == "real":
                    pose_now, pose_age_s, pose_stable = trace_pose()
                    trace_run_id, trace_run_dir = real_trace.create_run(
                        message=message,
                        execute=execute,
                        initial_world_state=self.state.state_estimator.state.public(),
                        initial_image=image_path,
                        initial_commanded_pose=pose_now,
                        initial_pose_age_s=pose_age_s,
                        initial_pose_stable=pose_stable,
                    )
                else:
                    # SIM planner turns keep the same per-turn record (raw model
                    # replies, tool results, frames) so LLM/team behaviour can be
                    # audited after the fact; sim_traces only cover the physics.
                    trace_run_id, trace_run_dir = real_trace.create_run(
                        message=message,
                        execute=execute,
                        initial_world_state=self.state.state_estimator.state.public(),
                        initial_image=image_path,
                        root=PROJECT_ROOT / "outputs" / "sim_turn_traces" / self.state.robot_id,
                        source=f"sim_{self.state.robot_id}",
                    )
            except Exception as exc:
                # Robot execution must remain available even if local trace I/O
                # is unhealthy. The journal still receives this explicit warning.
                print(f"turn trace initialization failed: {exc}", file=sys.stderr, flush=True)
                trace_run_id = None
                trace_run_dir = None

        observed_trace_seq = 0

        def observe() -> str | None:
            nonlocal observed_trace_seq
            # Snapshot-backed SIM has no continuously-owned camera pump. Fetch
            # the authoritative namespaced robot frame here so the planner's
            # post-action evidence cannot depend on unrelated browser polling.
            if self.state.prefer_fresh_camera_snapshot and self.state.grab_camera is not None:
                try:
                    jpeg = self.state.grab_camera()
                except CameraError:
                    return None
                self.state.last_seen_jpeg = jpeg
                stored = self._store_jpeg(jpeg)
                observed_trace_seq += 1
                pose_now, pose_age_s, pose_stable = trace_pose()
                real_trace.copy_frame(
                    stored, label=f"observe-{observed_trace_seq}", source="post_action_camera",
                    commanded_pose=pose_now, pose_age_s=pose_age_s, pose_stable=pose_stable,
                )
                return stored
            previous = self.state.last_seen_jpeg
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if self.state.cancel.is_set():
                    break
                cached = self.state.cached_frame()
                if cached and cached != previous:
                    self.state.last_seen_jpeg = cached
                    stored = self._store_jpeg(cached)
                    observed_trace_seq += 1
                    pose_now, pose_age_s, pose_stable = trace_pose()
                    real_trace.copy_frame(
                        stored, label=f"observe-{observed_trace_seq}", source="post_action_camera",
                        commanded_pose=pose_now, pose_age_s=pose_age_s, pose_stable=pose_stable,
                    )
                    return stored
                time.sleep(0.05)
            cached = self.state.cached_frame()
            if cached:
                self.state.last_seen_jpeg = cached
                stored = self._store_jpeg(cached)
                observed_trace_seq += 1
                pose_now, pose_age_s, pose_stable = trace_pose()
                real_trace.copy_frame(
                    stored, label=f"observe-{observed_trace_seq}", source="post_action_camera",
                    commanded_pose=pose_now, pose_age_s=pose_age_s, pose_stable=pose_stable,
                )
                return stored
            # Some camera sources are snapshot-only (and tests inject a
            # direct grabber) rather than a continuously refreshed MJPEG
            # cache. Fall back to one direct grab before declaring that no
            # post-action observation is available.
            grabber = self.state.grab_camera
            if grabber is not None:
                try:
                    jpeg = grabber()
                except CameraError:
                    return None
                self.state.last_seen_jpeg = jpeg
                stored = self._store_jpeg(jpeg)
                observed_trace_seq += 1
                pose_now, pose_age_s, pose_stable = trace_pose()
                real_trace.copy_frame(
                    stored, label=f"observe-{observed_trace_seq}", source="post_action_camera",
                    commanded_pose=pose_now, pose_age_s=pose_age_s, pose_stable=pose_stable,
                )
                return stored
            return None

        def on_step(step) -> None:
            real_trace.record_step(step)
            if stream:
                self._sse(event_from_step(step))

        def on_activity(kind: str, name: str | None) -> None:
            real_trace.record_activity(kind, name)
            if not stream:
                return
            if kind == "planning":
                self._sse({"type": "status", "text": "명령을 해석하고 있습니다"})
            elif kind == "tool_start" and name:
                self._sse({"type": "status", "text": f"{name} 실행 시작"})

        planner_trace_seq = 0

        def on_planner_input(planner_image: str | None, planner_state: dict[str, Any]) -> None:
            nonlocal planner_trace_seq
            planner_trace_seq += 1
            durable = None
            if planner_image:
                pose_now, pose_age_s, pose_stable = trace_pose()
                durable = real_trace.copy_frame(
                    planner_image,
                    label=f"planner-input-{planner_trace_seq}",
                    source="planner_input",
                    commanded_pose=pose_now, pose_age_s=pose_age_s, pose_stable=pose_stable,
                )
            else:
                pose_now, pose_age_s, pose_stable = trace_pose()
            real_trace.append_event(
                "planner_input",
                planner_call=planner_trace_seq,
                image_path=durable,
                world_state=planner_state,
                commanded_pose=pose_now,
                pose_age_s=pose_age_s,
                pose_stable=pose_stable,
            )

        trace_context = (
            real_trace.activate_run(trace_run_id, trace_run_dir)
            if trace_run_id is not None and trace_run_dir is not None
            else contextlib.nullcontext()
        )
        team_batch_context = (
            use_team_batch(
                payload.get("team_batch_id"),
                payload.get("team_batch_expected", 3),
            )
            if self.state.ui_mode == "sim" and execute and team_wake
            else contextlib.nullcontext()
        )
        try:
            with trace_context, team_batch_context:
                def planner_context_snapshot() -> dict[str, Any]:
                    # Peer evidence can arrive while this robot is already in a
                    # multi-step autonomous turn. Refresh it before every VLM
                    # decision rather than freezing it at HTTP turn entry.
                    peer_context = team_bus.context_for(
                        self.state.robot_id, after_seq=0, limit=8, namespace=self.state.ui_mode,
                        exclude_wakeup_id=(str(payload.get("team_wakeup_id") or "") or None) if team_wake else None,
                    )
                    # Local tool events are already represented more precisely
                    # in world_state/action_queue. Keep only peer/non-tool events
                    # here so TEAM context cannot duplicate the robot's own growing
                    # execution trace into every planner request.
                    team_events = [
                        event for event in (peer_context.get("events", []) or [])
                        if event.get("robot_id") != self.state.robot_id or event.get("kind") != "tool"
                    ]
                    goal_obj = peer_context.get("goal") or {}
                    try:
                        peers = team_bus.peer_summary(self.state.robot_id, namespace=self.state.ui_mode)
                    except Exception:
                        peers = {}
                    return {
                        "self_id": self.state.robot_id,
                        "team_goal": goal_obj.get("text") if isinstance(goal_obj, dict) else goal_obj,
                        "peer_messages": [
                            {k: m.get(k) for k in ("sender_id", "recipient", "message", "proposed_action", "required_partner") if m.get(k)}
                            for m in (peer_context.get("messages", []) or [])[-4:]
                        ],
                        "peers": peers,
                        "consensus_rounds": (peer_context.get("consensus_rounds") or [])[-2:],
                        "recent_team_events": team_events[-3:],
                        "coordination_rule": "Peers are equal; follow the TEAM rules in the system prompt.",
                    }

                _log, result = handle_turn(
                message,
                image=image_path,
                history=self.state.history[-12:],
                completer=self.state.completer,
                registry=self.state.registry,
                execute=execute,
                max_steps=self.state.max_steps,
                observe=observe if agent_intent and self.state.grab_camera is not None else None,
                on_step=on_step,
                on_activity=on_activity,
                on_planner_input=on_planner_input,
                should_cancel=self.state.cancel.is_set,
                talk_only=agent_intent and image_path is None,
                conversation_only=not agent_intent,
                auto_observe=agent_intent,
                verify_final=self.state.verify_final,
                state_estimator=self.state.state_estimator,
                executive=self.state.executive,
                planner_context_provider=planner_context_snapshot,
                action_queue=self.state.action_queue,
                planner_image_max_edge=224 if team_wake else None,
                auto_recommended_recovery=team_wake,
                executive_fallback_on_planner_error=team_wake,
                )
                trace_analysis = real_trace.finish_run(result)
        except VlmError as exc:
            if trace_run_id is not None and trace_run_dir is not None:
                with real_trace.activate_run(trace_run_id, trace_run_dir):
                    real_trace.finish_run(error=str(exc))
            if stream:
                self._sse({"type": "error", "text": str(exc)})
                self._sse({"type": "done", "error": str(exc), "text": "", "events": [], "tools": []})
                return
            self._send_json(200, {"error": str(exc), "text": ""})
            return
        except Exception as exc:
            if trace_run_id is not None and trace_run_dir is not None:
                with real_trace.activate_run(trace_run_id, trace_run_dir):
                    real_trace.finish_run(error=f"{type(exc).__name__}: {exc}")
            if stream:
                detail = f"{type(exc).__name__}: {exc}"
                self._sse({"type": "error", "text": detail})
                self._sse({"type": "done", "error": detail, "text": "", "events": [], "tools": []})
                return
            raise
        body = result_payload(result)
        if trace_run_id is not None and trace_run_dir is not None and self.state.ui_mode == "real":
            # The browser trace viewer only indexes the REAL trace root.
            body["real_run_id"] = trace_run_id
            body["real_trace_dir"] = str(trace_run_dir)
            if isinstance(trace_analysis, dict):
                body["real_trace_status"] = trace_analysis.get("status")
                body["real_trace_failure"] = trace_analysis.get("failure")
        if result.stopped != "empty" and not team_wake:
            self.state.history.append({"role": "user", "content": message.strip()})
            blob = history_text(result)
            if blob:
                self.state.history.append({"role": "assistant", "content": blob})
        if stream:
            self._sse({"type": "done", **body})
            return
        self._send_json(200, body)

    def _camera(self) -> None:
        grabber = self.state.grab_camera
        # An explicit snapshot URL (the authoritative MuJoCo bridge in SIM) is
        # the live source itself. Do not let the cached startup bitmap shadow it.
        # Physical REAL keeps the cache-first path because its single MJPEG pump
        # owns the camera connection and continuously refreshes that cache.
        if self.state.prefer_fresh_camera_snapshot and grabber is not None:
            try:
                jpeg = grabber()
            except CameraError as exc:
                self._send(503, str(exc).encode("utf-8"), "text/plain; charset=utf-8")
                return
            self._send(200, jpeg, "image/jpeg")
            return
        cached = self.state.cached_frame()
        if cached is not None:
            self._send(200, cached, "image/jpeg")
            return
        if grabber is None:
            self._send(
                503,
                "MasterPi 카메라가 연결되지 않았습니다.".encode("utf-8"),
                "text/plain; charset=utf-8",
            )
            return
        try:
            jpeg = grabber()
        except CameraError as exc:
            self._send(503, str(exc).encode("utf-8"), "text/plain; charset=utf-8")
            return
        self._send(200, jpeg, "image/jpeg")

    def _camera_stream(self) -> None:
        if self.state._low_latency_camera:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.end_headers()
            seq = -1
            try:
                while True:
                    frame, new_seq, received_at = self.state.wait_for_frame(seq, timeout=3.0)
                    if frame is None or new_seq <= seq:
                        continue
                    seq = new_seq
                    if self.state.ui_mode == "real":
                        age = time.monotonic() - received_at if received_at > 0 else float("inf")
                        if age > self.state.camera_stale_after_s:
                            # Do not emit the last cached image after the physical
                            # camera link has died. Wait for a genuinely new frame.
                            continue
                    part = (
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(frame)).encode()
                        + b"\r\n\r\n"
                        + frame
                        + b"\r\n"
                    )
                    self.wfile.write(part)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return
        opener = self.state.open_stream
        if opener is None:
            self._send(
                503,
                "MasterPi 카메라 스트림이 연결되지 않았습니다.".encode("utf-8"),
                "text/plain; charset=utf-8",
            )
            return
        try:
            stream = opener()
        except CameraError as exc:
            self._send(503, str(exc).encode("utf-8"), "text/plain; charset=utf-8")
            return
        self.send_response(200)
        self.send_header("Content-Type", stream.content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        buf = bytearray()
        try:
            while True:
                chunk = stream.read(8192)
                if not chunk:
                    break
                buf.extend(chunk)
                frames = pop_jpegs(buf)
                if frames:
                    self.state.remember_frame(frames[-1], perceive=False)
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            stream.close()

    def _observer_stream(self, name: str | None = None) -> None:
        base = os.environ.get("UGRP_SIM_BRIDGE_LOCAL", "").rstrip("/")
        if not base:
            self._send(404, b"observer stream unavailable", "text/plain")
            return
        bridge_name = name
        if self.state.ui_mode == "sim" and name and not name.startswith(("r1_", "r2_", "r3_")):
            bridge_name = f"{self.state.robot_id}_{name}"
        try:
            suffix = "/observer/stream" if not bridge_name else f"/observer/{bridge_name}/stream"
            stream = open_mjpeg_stream(url=base + suffix)
        except CameraError as exc:
            self._send(503, str(exc).encode("utf-8"), "text/plain; charset=utf-8")
            return
        self.send_response(200)
        self.send_header("Content-Type", stream.content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            while True:
                chunk = stream.read(8192)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            stream.close()

    def _observer_snapshot(self, name: str | None = None) -> None:
        """Proxy only the bridge's newest observer JPEG.

        The browser presentation deliberately uses snapshots instead of a
        long-lived MJPEG response. If a client/network falls behind, the next
        request skips old frames rather than replaying a TCP backlog.
        """
        base = os.environ.get("UGRP_SIM_BRIDGE_LOCAL", "").rstrip("/")
        if not base:
            self._send(404, b"observer snapshot unavailable", "text/plain")
            return
        bridge_name = name
        if self.state.ui_mode == "sim" and name and not name.startswith(("r1_", "r2_", "r3_")):
            bridge_name = f"{self.state.robot_id}_{name}"
        suffix = "/observer/snapshot" if not bridge_name else f"/observer/{bridge_name}/snapshot"
        try:
            with urlopen(base + suffix, timeout=1.0) as resp:
                jpeg = resp.read()
        except Exception as exc:
            self._send(503, str(exc).encode("utf-8"), "text/plain; charset=utf-8")
            return
        self._send(200, jpeg, "image/jpeg")

    def _robot_get(self) -> None:
        panel = self.state.robot
        if panel is None:
            self._send_json(503, {"error": "로봇 상태가 연결되지 않았습니다."})
            return
        self._send_json(200, panel.snapshot())

    def _robot_command(self, kind: str) -> None:
        panel = self.state.robot
        if panel is None:
            self._send_json(503, {"error": "로봇 상태가 연결되지 않았습니다."})
            return
        try:
            payload = {} if kind == "stop" else self._read_json()
        except (json.JSONDecodeError, ValueError):
            self._send_json(400, {"error": "invalid JSON"})
            return
        try:
            if kind == "drive":
                body = panel.drive(payload)
            elif kind == "arm":
                body = panel.arm(payload)
            else:
                body = panel.stop()
        except (RobotError, ValueError) as exc:
            self._send_json(502, {"error": str(exc)})
            return
        self._send_json(200, body)

    def _resolve_image(self, image: Any, *, include_camera: bool = True) -> str | None:
        if isinstance(image, str) and image.strip():
            path = self._store_data_url(image.strip())
            try:
                self.state.last_seen_jpeg = Path(path).read_bytes()
            except OSError:
                self.state.last_seen_jpeg = None
            return path
        if not include_camera:
            return None
        grabber = self.state.grab_camera
        if self.state.prefer_fresh_camera_snapshot and grabber is not None:
            try:
                jpeg = grabber()
            except CameraError:
                return self.state.last_image
            self.state.last_seen_jpeg = jpeg
            return self._store_jpeg(jpeg)
        cached = self.state.cached_frame()
        if cached:
            self.state.last_seen_jpeg = cached
            return self._store_jpeg(cached)
        if grabber is None:
            return self.state.last_image
        try:
            jpeg = grabber()
        except CameraError:
            cached = self.state.cached_frame()
            if cached:
                self.state.last_seen_jpeg = cached
                return self._store_jpeg(cached)
            return self.state.last_image
        self.state.last_seen_jpeg = jpeg
        return self._store_jpeg(jpeg)

    def _store_jpeg(self, jpeg: bytes) -> str:
        self.state._image_count += 1
        path = self.state.image_dir / f"scene-{self.state._image_count}.jpg"
        path.write_bytes(jpeg)
        self.state.last_image = str(path)
        return str(path)

    def _store_data_url(self, data_url: str) -> str:
        self.state._image_count += 1
        path = save_data_url(data_url, self.state.image_dir, self.state._image_count)
        self.state.last_image = path
        return path


def make_server(state: ChatState, *, host: str, port: int) -> ThreadingHTTPServer:
    handler = type("BoundChatHandler", (ChatHandler,), {"state": state})
    return ThreadingHTTPServer((host, port), handler)


def _temporary_team_retry_delay(error: str) -> float | None:
    text = str(error or "")
    low = text.lower()
    retryable = (
        "tpm" in low
        or "tokens per minute" in low
        or "shared cooldown" in low
        or "rate limit" in low
        or "too many requests" in low
        or ("gemini" in low and "http 503" in low)
        or "no capacity available" in low
        or "한도에 걸" in text
        or ("groq 모델 fallback까지 실패" in low and "groq 요청이 모두 실패" in low)
    )
    if not retryable:
        return None
    match = re.search(r"(?:try again in|retry in)\s+([0-9.]+)\s*(ms|s)", text, re.IGNORECASE)
    try:
        if match:
            delay = float(match.group(1)) / 1000.0 if match.group(2).lower() == "ms" else float(match.group(1))
        else:
            delay = 2.0 if ("http 503" in low or "no capacity available" in low) else 12.0
    except (TypeError, ValueError):
        delay = 2.0 if ("http 503" in low or "no capacity available" in low) else 12.0
    return max(0.5, min(delay + 0.35, 30.0))


_TEAM_ROLE_COLOR = {
    "red": "빨간", "blue": "파란", "yellow": "노란",
    "빨간": "빨간", "빨강": "빨간", "파란": "파란", "파랑": "파란", "노란": "노란", "노랑": "노란",
}

_TOWER_STEP_2 = "[TOWER_STEP_2]"
_TOWER_STEP_3 = "[TOWER_STEP_3]"


def _team_tower_intent(body: str) -> bool:
    text = str(body or "").strip()
    low = text.lower()
    has_block = any(k in low for k in ("블럭", "블록", "block"))
    has_stack = any(k in low for k in ("쌓", "3단", "삼단", "한 줄", "한줄", "1열", "tower", "stack"))
    has_team = any(k in low for k in ("세 로봇", "3 로봇", "3로봇", "세 개", "3개", "세 블", "모두", "협력"))
    has_specific_color = any(
        k in low
        for k in (
            "빨간", "빨강", "파란", "파랑", "노란", "노랑",
            "red", "blue", "yellow",
        )
    )
    # In the TEAM room, a generic request such as "블럭 쌓아봐" means the
    # shared three-block tower mission even when the operator omits "세 로봇".
    # Keep color-specific stacking requests local so "빨간 블럭 쌓아" does
    # not unexpectedly expand into the full three-robot workflow.
    return bool(has_block and has_stack and (has_team or not has_specific_color))


def _team_beam_intent(body: str) -> bool:
    low = str(body or "").strip().lower()
    has_payload = any(
        token in low for token in (
            "빔", "긴 물체", "긴 물건", "무거운 물체", "무거운 물건",
            "beam", "heavy payload", "long payload",
        )
    )
    has_transport = any(
        token in low for token in (
            "옮", "운반", "이동", "출발", "도착", "transport", "deliver", "carry",
        )
    )
    has_team = any(
        token in low for token in (
            "협업", "협력", "함께", "세 로봇", "3로봇", "team",
        )
    )
    return bool(has_payload and has_transport and has_team)


def _sim_team_tower_stage_complete(stage: int) -> bool:
    """Verify a TEAM tower handoff against authoritative shared MuJoCo state."""
    try:
        with urlopen("http://127.0.0.1:8091/health", timeout=2.0) as resp:
            health = json.loads(resp.read().decode("utf-8"))
        objects = (health.get("state") or health).get("shared_objects") or {}
        yellow = objects.get("yellow")
        blue = objects.get("blue")
        red = objects.get("red")
        if not all(isinstance(v, list) and len(v) >= 3 for v in (yellow, blue, red)):
            return False
        if stage == 1:
            return bool(
                math.hypot(float(yellow[0]) - TEAM_STACK_SITE[0], float(yellow[1]) - TEAM_STACK_SITE[1])
                <= TEAM_STACK_SITE_TOLERANCE_M
                and 0.008 <= float(yellow[2]) <= 0.022
            )
        yb_xy = math.hypot(float(blue[0]) - float(yellow[0]), float(blue[1]) - float(yellow[1]))
        yb_dz = float(blue[2]) - float(yellow[2])
        if yb_xy > 0.0135 or abs(yb_dz - 0.030) > 0.006:
            return False
        if stage == 2:
            return True
        br_xy = math.hypot(float(red[0]) - float(blue[0]), float(red[1]) - float(blue[1]))
        br_dz = float(red[2]) - float(blue[2])
        return bool(br_xy <= 0.0135 and abs(br_dz - 0.030) <= 0.006)
    except Exception:
        return False


def _sim_team_beam_complete() -> bool:
    """Read only the authoritative referee result; never expose it to planning."""
    try:
        with urlopen("http://127.0.0.1:8091/health", timeout=2.0) as resp:
            health = json.loads(resp.read().decode("utf-8"))
        payload = (health.get("state") or {}).get("cooperative_payload") or {}
        evaluation = payload.get("evaluation") or {}
        return bool(payload.get("status") == "SUCCESS" and evaluation.get("success"))
    except Exception:
        return False


def _sim_warehouse_complete() -> bool:
    try:
        with urlopen("http://127.0.0.1:8091/health", timeout=2.0) as resp:
            health = json.loads(resp.read().decode("utf-8"))
        warehouse = (health.get("state") or {}).get("warehouse") or {}
        return bool(
            warehouse.get("status") == "SUCCESS"
            and warehouse.get("success")
            and int(warehouse.get("moved_count") or 0)
            == int(warehouse.get("total_count") or -1)
            and not (warehouse.get("remaining_ids") or [])
        )
    except Exception:
        return False


def _team_local_verb(body: str) -> str:
    low = str(body or "").lower()
    # "찾아서 집어봐" is a pick mission: the terminal verb wins over the
    # search verb that precedes it, otherwise every robot stops after search.
    if any(k in low for k in ("집어", "잡아", "pick", "grasp")):
        return "집어봐"
    if any(k in low for k in ("찾", "탐색", "search", "find", "locate")):
        return "찾아봐"
    return ""


def _personalize_team_turn(
    body: str,
    robot_id: str,
    *,
    intent: Mapping[str, Any] | None = None,
) -> str:
    """Convert a room-wide command into one deterministic local mission.

    The original room text remains available as ``team_goal`` in planner context,
    so the local mission does not need to repeat every peer/color clause. This is
    especially important because deterministic goal parsing otherwise sees the
    first colour in the room sentence for every robot.
    """
    rid = team_bus.normalize_robot_id(robot_id).upper()
    text = str(body or "").strip()
    if isinstance(intent, Mapping) and intent.get("kind") == "WAREHOUSE_TRANSFER":
        source = str((intent.get("source") or {}).get("id") or "A")
        destination = str((intent.get("destination") or {}).get("id") or "B")
        selector_obj = intent.get("selector") or {}
        selector = str(
            selector_obj.get("object_type")
            or selector_obj.get("mode")
            or "all"
        )
        mission_id = str(intent.get("mission_id") or "")
        return (
            "TEAM warehouse consensus가 완료됐다. 세 로봇 모두 같은 shared-world "
            f"미션에 참여한다: {source}구역의 {selector} 화물을 {destination}구역으로 운반. "
            "각자 별도 짐을 고르지 말고 반드시 동일한 "
            f"team_zone_transfer(mission_id='{mission_id}', source_zone='{source}', "
            f"destination_zone='{destination}', selector='{selector}')를 정확히 한 번 호출해."
        )
    if _team_beam_intent(text):
        role = {
            "R1": "carrier_left",
            "R2": "scout",
            "R3": "carrier_right",
        }[rid]
        role_note = {
            "R1": "빔의 아래쪽 손잡이를 양쪽 집게 접촉으로 잡는 운반자",
            "R2": "도착 구역 B까지 경로를 먼저 확인하고 최종 배치를 검증하는 정찰자",
            "R3": "빔의 위쪽 손잡이를 양쪽 집게 접촉으로 잡는 운반자",
        }[rid]
        return (
            "TEAM 합의가 완료된 Cooperative Beam Transport MVP다. "
            f"mission_id=beam_transport_v1, 네 역할은 {role}({role_note})다. "
            "세 역할이 같은 batch에 모두 들어와야만 물체가 움직인다. "
            "다른 물리 스킬을 먼저 실행하지 말고 반드시 "
            f"team_beam_transport(mission_id='beam_transport_v1', role='{role}')를 정확히 한 번 호출해."
        )
    if _team_tower_intent(text):
        # The physical order (base first, then two stacks) is fixed by the task,
        # but tool choice and the handoff message are the planner's decision.
        if rid == "R3":
            return (
                "팀 목표: 세 로봇이 3단 탑을 쌓는다 (바닥 노란 → 그 위 파란 → 맨 위 빨간). "
                "네 단계는 1/3: 노란 블럭을 공용 스택 지점의 바닥층으로 옮겨. "
                "완료를 확인하면 send_peer_message로 R1에게 다음 단계(파란 블럭을 노란 블럭 위에 올리기)를 넘겨."
            )
        if rid == "R1":
            return (
                "팀 목표: 세 로봇이 3단 탑을 쌓는다. 네 단계는 2/3: 파란 블럭을 노란 블럭 위에 올려. "
                "완료를 확인하면 send_peer_message로 R2에게 다음 단계(빨간 블럭을 파란 블럭 위에 올리기)를 넘겨."
            )
        return "팀 목표: 세 로봇이 3단 탑을 쌓는다. 네 단계는 3/3(마지막): 빨간 블럭을 파란 블럭 위에 올려. 완료를 확인하면 팀에 알려."
    pattern = re.compile(
        rf"(?:@?{re.escape(rid)})(?:은|는)?\s*(?:담당(?:은|는|:)?\s*)?"
        r"(빨간|빨강|파란|파랑|노란|노랑|red|blue|yellow)",
        re.IGNORECASE,
    )
    match = pattern.search(text)
    if match:
        token = match.group(1).lower()
        color = _TEAM_ROLE_COLOR.get(token, _TEAM_ROLE_COLOR.get(match.group(1), match.group(1)))
        verb = _team_local_verb(text) or "찾아봐"
        return f"{color} 블럭 {verb}"

    low = text.lower()
    if any(k in low for k in ("모아", "모으", "한 곳", "한곳", "gather", "collect")) and any(
        k in low for k in ("블럭", "블록", "block")
    ):
        # With the current transferable skill catalog, a common coloured anchor
        # is the only calibrated notion of "one place". Yellow is the anchor;
        # R1/R2 deliver their objects to it while R3 locates/guards the anchor.
        if rid == "R1":
            return "빨간 블럭을 노란 블럭 위에 올려봐"
        if rid == "R2":
            return "파란 블럭을 노란 블럭 위에 올려봐"
        return "노란 블럭 찾아봐"
    return text


def _run_sim_tower_fallback(robot_id: str, stage: int) -> dict[str, Any]:
    """Execute the calibrated tower primitive when the planner provider fails.

    This remains inside the same public SIM action transport and still requires
    the shared-world postcondition check before any peer handoff.
    """
    from scripts import sim_actions

    rid = team_bus.normalize_robot_id(robot_id)
    if stage == 1:
        return sim_actions.run_for_robot("stage_base", rid, target_color="yellow")
    if stage == 2:
        return sim_actions.run_for_robot(
            "stack_on", rid, target_color="blue", destination_color="yellow"
        )
    if stage == 3:
        return sim_actions.run_for_robot(
            "stack_on", rid, target_color="red", destination_color="blue"
        )
    raise ValueError(f"invalid tower stage: {stage}")


def _wait_sim_team_tower_dependency(stage: int, timeout_s: float = 480.0) -> bool:
    """Wait only for the physical predecessor required by a tower placement.

    Consensus execution wakes all three robots together.  The upper two blocks
    cannot be placed until their support exists, so those workers wait on the
    authoritative shared-world postcondition instead of wandering/searching or
    serial handoff messages.
    """
    if stage <= 1:
        return True
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    predecessor = stage - 1
    while time.monotonic() < deadline:
        if _sim_team_tower_stage_complete(predecessor):
            return True
        time.sleep(0.05)
    return False


def _deliver_warehouse_research_wakeup(state: ChatState, wake: dict[str, Any], intent: dict) -> None:
    """Readiness is a transport barrier; roles are decided later by EACH LLM."""
    from .warehouse_runtime import BridgeEnvironment, EpisodeBudget, run_episode
    rid = team_bus.normalize_robot_id(wake.get("robot_id"))
    wid = str(wake.get("id") or "")
    round_id = str(wake.get("consensus_round_id") or "")
    if wake.get("reason") == "consensus_proposal":
        team_bus.submit_consensus_proposal(round_id, rid, {"ready": True, "kind": "research_transport_ready"}, namespace="sim")
        team_bus.complete_wakeup(wid, namespace="sim", ok=True)
        return
    if rid != "r1":
        # Endpoint /api/warehouse/decision runs on this robot's own backend.
        # This wake registers participation; it does not assert mission success.
        team_bus.record_event(rid, "research_participant_ready", namespace="sim", wakeup_id=wid)
        team_bus.complete_wakeup(wid, namespace="sim", ok=True)
        return
    class RemotePolicy:
        evidence_kind = "live_llm"
        def __init__(self, robot_id):
            self.robot_id = robot_id
            self.model_name = str(getattr(state.completer, "model_name", "configured_backend"))
        def decide(self, context):
            headers = {"Content-Type": "application/json"}
            if state.password:
                token = base64.b64encode(f"ugrp:{state.password}".encode()).decode("ascii")
                headers["Authorization"] = f"Basic {token}"
            req = Request(f"http://127.0.0.1:{BACKEND_PORTS[('sim', self.robot_id)]}/api/warehouse/decision",
                          data=json.dumps({"context": context, "response_tokens": 768 if "communication_mode" in context else 512,
                                           "task_mode": "coela" if "communication_mode" in context else ("mixed" if "available_ids" in context else "joint")}).encode(),
                          headers=headers, method="POST")
            try:
                with urlopen(req, timeout=55) as response:
                    return json.loads(response.read())
            except HTTPError as exc:
                if "communication_mode" not in context: raise
                from .coela_modules import PlanningError
                error = json.loads(exc.read())
                raise PlanningError(error.get("error", str(exc)), error.get("raw"), error.get("usage")) from exc
    try:
        from scripts import sim_actions
        health = sim_actions._bridge_health()
        world_state = health.get("state") or {}
        seed = world_state.get("world_seed", world_state.get("seed"))
        if seed is None:
            raise RuntimeError("current world seed unavailable; refusing an untracked run")
        mode = os.environ.get("UGRP_WAREHOUSE_MODE", "llm_peer_comm")
        if mode == "rule":
            from .warehouse_runtime import RulePolicy
            policies = {r: RulePolicy(r) for r in team_bus.ROBOT_IDS}
        else:
            policies = {r: RemotePolicy(r) for r in team_bus.ROBOT_IDS}
        selector_obj = intent.get("selector") or {}
        path = PROJECT_ROOT / "outputs" / "warehouse_research" / f"team-{round_id}-{time.time_ns()}.jsonl"
        if os.environ.get("UGRP_WAREHOUSE_LAYOUT", "mixed") == "mixed":
            if (intent.get("source") or {}).get("id", "A") != "A":
                raise ValueError("현재 혼합 과제는 A구역 출발입니다. 새 과제는 SIM 초기화 후 시작하세요.")
            from .mixed_warehouse_runtime import run_mixed_episode, MixedRulePolicy
            if mode == "rule":
                policies = {r: MixedRulePolicy(r) for r in team_bus.ROBOT_IDS}
            if os.environ.get("UGRP_WAREHOUSE_ARCHITECTURE") == "coela" and mode != "rule":
                from .coela_runtime import run_coela_episode
                communication_mode = {"llm_no_comm": "none", "llm_structured_comm": "structured", "llm_peer_comm": "natural"}[mode]
                result = run_coela_episode(BridgeEnvironment(), policies, mode=communication_mode,
                    destination=(intent.get("destination") or {}).get("id", "B"), journal_path=path,
                    event_callback=lambda event: team_bus.post_agent_chat(event["sender_id"],
                        f"→ {', '.join(event['recipients'])}: " + (event["content"] if isinstance(event["content"], str) else json.dumps(event["content"], ensure_ascii=False)),
                        namespace="sim", source="coela_message_sent"))
            else:
                result = run_mixed_episode(
                    BridgeEnvironment(), policies, condition=mode,
                    destination=(intent.get("destination") or {}).get("id", "B"), journal_path=path,
                    event_callback=lambda actor, message: team_bus.post_agent_chat(actor, message, namespace="sim", source="mixed_task_decision"),
                )
            result["delivered_ids"] = result.get("status", {}).get("completed_ids", [])
            result["rounds"] = sum(result.get("calls", {}).values())
        else:
            result = run_episode(
                BridgeEnvironment(), policies, condition=mode, seed=int(seed),
                source=(intent.get("source") or {}).get("id", "A"),
                destination=(intent.get("destination") or {}).get("id", "B"),
                selector=selector_obj.get("object_type") or selector_obj.get("mode") or "all",
                budget=EpisodeBudget(), journal_path=path,
                event_callback=lambda actor, message: team_bus.post_agent_chat(actor, message, namespace="sim", source="warehouse_peer_decision"),
            )
        team_bus.record_event(rid, "warehouse_research_result", namespace="sim", result=result)
        team_bus.post_agent_chat(rid,
            f"연구 경로: {result['reason']} · {len(result['delivered_ids'])}개 이송 · {result['rounds']}회 결정. 기록: {path.name}",
            namespace="sim", source="warehouse_referee_result")
        team_bus.complete_wakeup(wid, namespace="sim", ok=result["success"], error="" if result["success"] else result["reason"])
    except Exception as exc:
        team_bus.record_event(rid, "warehouse_research_failed", namespace="sim", error=str(exc))
        team_bus.complete_wakeup(wid, namespace="sim", ok=False, error=str(exc))


def _deliver_team_wakeup(state: ChatState, mode: str, wake: dict[str, Any]) -> None:
    """Deliver one TEAM room event to the addressed peer without planning centrally."""
    rid = team_bus.normalize_robot_id(wake.get("robot_id"))
    port = BACKEND_PORTS[(mode, rid)]
    sender = str(wake.get("sender_id") or "operator")
    body = str(wake.get("message") or "").strip()
    reason = str(wake.get("reason") or "team_chat")
    consensus_round_id = str(wake.get("consensus_round_id") or "").strip()
    consensus_proposal = reason == "consensus_proposal"
    consensus_goal = body
    mission_intent = wake.get("intent") if isinstance(wake.get("intent"), dict) else None
    if consensus_round_id:
        try:
            rounds = team_bus.snapshot(namespace=mode).get("consensus_rounds") or []
            round_state = next(
                (item for item in rounds if str(item.get("id") or "") == consensus_round_id),
                None,
            )
            if isinstance(round_state, dict):
                consensus_goal = str(round_state.get("message") or body).strip()
                if isinstance(round_state.get("intent"), dict):
                    mission_intent = dict(round_state["intent"])
        except Exception:
            consensus_goal = body
    tower_step = 0
    beam_workflow = False
    warehouse_workflow = bool(
        mode == "sim"
        and reason == "consensus_execute"
        and isinstance(mission_intent, dict)
        and mission_intent.get("kind") == "WAREHOUSE_TRANSFER"
    )
    if (mode == "sim" and isinstance(mission_intent, dict)
            and mission_intent.get("kind") == "WAREHOUSE_TRANSFER"
            and reason in {"consensus_proposal", "consensus_execute"}
            and os.environ.get("UGRP_WAREHOUSE_MODE", "llm_peer_comm") != "central_baseline"):
        _deliver_warehouse_research_wakeup(state, wake, mission_intent)
        return
    if mode == "sim":
        beam_workflow = bool(
            reason == "consensus_execute"
            and _team_beam_intent(consensus_goal or body)
        )
        if reason == "consensus_execute" and _team_tower_intent(consensus_goal):
            tower_step = {"r3": 1, "r1": 2, "r2": 3}.get(rid, 0)
        elif reason == "team_chat" and rid == "r3" and _team_tower_intent(body):
            tower_step = 1
        elif reason == "peer_message" and rid in {"r1", "r2"}:
            # The handoff may be the planner's own words or the router fallback;
            # the stage referee applies either way while the tower is the goal.
            prev = "r3" if rid == "r1" else "r1"
            tag = _TOWER_STEP_2 if rid == "r1" else _TOWER_STEP_3
            if tag in body or (sender == prev and _team_tower_intent(_team_goal_text(mode))):
                tower_step = 2 if rid == "r1" else 3
    if consensus_proposal:
        intent_note = (
            f" canonical_intent={json.dumps(mission_intent, ensure_ascii=False, separators=(',', ':'))}"
            if isinstance(mission_intent, dict) else ""
        )
        turn_message = (
            f"[TEAM CONSENSUS PROPOSAL · {rid.upper()}] {body}\n"
            "아직 행동하지 마세요. 세 로봇의 합의가 끝나기 전 단계입니다. "
            "현재 장면과 TEAM 정보를 바탕으로 본인이 맡을 구체적인 행동 계획만 짧게 제안하세요."
            f"{intent_note}"
        )
    elif reason == "peer_message":
        turn_message = f"[TEAM · {sender.upper()} → {rid.upper()}] {body}"
    else:
        turn_message = _personalize_team_turn(body, rid, intent=mission_intent)
    payload = {
        "message": turn_message,
        "execute": mode == "sim" and not consensus_proposal,
        "stream": False,
        # REAL TEAM chat is intentionally conversational until the operator
        # explicitly commands a physical robot through its own REAL tab.
        "chat_only": mode == "real",
        "team_wake": True,
        "team_wakeup_id": wake.get("id"),
        "team_chat_id": wake.get("source_chat_id"),
    }
    if reason == "consensus_execute" and consensus_round_id:
        # Independent planners finish at slightly different wall times. Tag
        # only their first actuator call so the bridge can form one true
        # act_parallel cohort instead of missing its ordinary 50 ms window.
        payload["team_batch_id"] = f"{mode}:{consensus_round_id}"
        payload["team_batch_expected"] = 3
    headers = {"Content-Type": "application/json"}
    if state.password:
        token = base64.b64encode(f"ugrp:{state.password}".encode()).decode("ascii")
        headers["Authorization"] = f"Basic {token}"
    req = Request(
        f"http://127.0.0.1:{port}/api/turn",
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers=headers, method="POST",
    )
    if int(wake.get("attempts") or 0) <= 1:
        team_bus.record_event(rid, "team_wake_start", namespace=mode, wakeup_id=wake.get("id"), reason=reason)

    # A committed tower is already a verified 3/3 TEAM decision.  Release all
    # three execution wakeups together, then use the calibrated atomic SIM
    # primitive for each role.  Upper levels wait only for the unavoidable
    # physical support dependency; they never spend planner turns on repeated
    # search/approach loops or wait for conversational handoff messages.
    if mode == "sim" and reason == "consensus_execute" and tower_step:
        if not _wait_sim_team_tower_dependency(tower_step):
            error = f"tower dependency stage {tower_step - 1} not complete"
            team_bus.complete_wakeup(
                str(wake.get("id")), namespace=mode, ok=False, error=error,
            )
            team_bus.record_event(
                rid, "tower_stage_failed", namespace=mode,
                wakeup_id=wake.get("id"), stage=tower_step, error=error,
            )
            return
        try:
            # Each robot now owns and executes its assigned stage. The shared
            # world/bridge receives the three wakeups independently, so R3, R1,
            # and R2 can progress at the same time instead of one robot owning
            # the whole task and making the others idle.
            direct = _run_sim_tower_fallback(rid, tower_step)
        except Exception as exc:
            direct = {"ok": False, "reason": f"{type(exc).__name__}: {exc}"}
        if not direct.get("ok") or not _sim_team_tower_stage_complete(tower_step):
            error = str(direct.get("reason") or f"tower stage {tower_step} postcondition not met")
            team_bus.complete_wakeup(str(wake.get("id")), namespace=mode, ok=False, error=error)
            team_bus.record_event(
                rid, "tower_stage_failed", namespace=mode,
                wakeup_id=wake.get("id"), stage=tower_step, error=error,
            )
            return
        chat = team_bus.post_agent_chat(
            rid,
            f"{rid.upper()} tower stage {tower_step} 완료",
            namespace=mode,
            reply_to=wake.get("source_chat_id"),
            source="team_consensus_execute",
        )
        team_bus.complete_wakeup(
            str(wake.get("id")), namespace=mode, ok=True, response_chat_id=chat.get("id")
        )
        team_bus.record_event(
            rid, "team_wake_done", namespace=mode,
            wakeup_id=wake.get("id"), stage=tower_step, action=direct.get("skill"),
        )
        if tower_step == 3:
            team_bus.record_event(rid, "tower_complete", namespace=mode, stage=3, action="team_tower")
        return
    # Decision 2026-09-02: the planner is always asked first, even for the tower.
    # The calibrated direct primitive is only a fallback when the planner
    # provider itself fails; it never pre-empts the LLM's own decision.
    handoff_seq_floor = _team_last_message_seq(mode)
    result = None
    try:
        with urlopen(req, timeout=180) as resp:
            result = json.loads(resp.read().decode("utf-8"))
    except HTTPError as exc:
        if exc.code == 409:
            attempts = max(1, int(wake.get("attempts") or 1))
            delay = min(6.0, 0.8 * (2 ** min(attempts - 1, 3)))
            team_bus.release_wakeup(
                str(wake.get("id")), namespace=mode, delay_s=delay, error="robot_busy"
            )
            return
        try:
            detail = exc.read().decode("utf-8", "replace")
        except Exception:
            detail = str(exc)
        team_bus.complete_wakeup(str(wake.get("id")), namespace=mode, ok=False, error=f"HTTP {exc.code}: {detail[:240]}")
        return
    except Exception as exc:
        team_bus.complete_wakeup(str(wake.get("id")), namespace=mode, ok=False, error=f"{type(exc).__name__}: {exc}")
        return

    error = str(result.get("error") or "").strip()
    if error:
        if mode == "sim" and tower_step:
            try:
                fallback = _run_sim_tower_fallback(rid, tower_step)
            except Exception as exc:
                fallback = {"ok": False, "reason": f"{type(exc).__name__}: {exc}"}
            if fallback.get("ok") and _sim_team_tower_stage_complete(tower_step):
                team_bus.record_event(
                    rid, "tower_planner_fallback", namespace=mode,
                    wakeup_id=wake.get("id"), stage=tower_step,
                    planner_error=error[:240], action=fallback.get("skill"),
                )
                error = ""
                result = {
                    "final": f"{rid.upper()} tower stage {tower_step} completed by verified SIM fallback",
                    "text": "",
                    "tools": [fallback],
                    "stopped": "tower_planner_fallback",
                }
            else:
                fallback_reason = str(fallback.get("reason") or "tower fallback postcondition not met")
                error = f"{error}; tower fallback failed: {fallback_reason}"
    if error:
        retry_delay = _temporary_team_retry_delay(error)
        if retry_delay is not None:
            retry_delay = min(30.0, retry_delay + {"r1": 0.0, "r2": 2.0, "r3": 4.0}.get(rid, 0.0))
        if retry_delay is not None and int(wake.get("attempts") or 0) < 6:
            released = team_bus.release_wakeup(
                str(wake.get("id")), namespace=mode, delay_s=retry_delay, error=error, force=True
            )
            if released.get("status") == "pending":
                team_bus.record_event(
                    rid, "team_wake_retry", namespace=mode, wakeup_id=wake.get("id"),
                    delay_s=round(retry_delay, 3), reason="temporary_tpm",
                )
                return
        team_bus.complete_wakeup(str(wake.get("id")), namespace=mode, ok=False, error=error)
        return
    if tower_step and not _sim_team_tower_stage_complete(tower_step):
        # A planner turn may terminate cleanly after repeated perception/tool
        # failures while still not satisfying the tower stage. Treat that the
        # same as a provider failure: run the calibrated SIM primitive and only
        # advance after the authoritative shared-world postcondition passes.
        try:
            fallback = _run_sim_tower_fallback(rid, tower_step)
        except Exception as exc:
            fallback = {"ok": False, "reason": f"{type(exc).__name__}: {exc}"}
        if fallback.get("ok") and _sim_team_tower_stage_complete(tower_step):
            team_bus.record_event(
                rid, "tower_planner_fallback", namespace=mode,
                wakeup_id=wake.get("id"), stage=tower_step,
                planner_error="planner finished without satisfying tower postcondition",
                action=fallback.get("skill"),
            )
            result = {
                "final": f"{rid.upper()} tower stage {tower_step} completed by verified SIM fallback",
                "text": "",
                "tools": [fallback],
                "stopped": "tower_planner_fallback",
            }
        else:
            error = f"tower stage {tower_step} postcondition not met"
            team_bus.complete_wakeup(str(wake.get("id")), namespace=mode, ok=False, error=error)
            team_bus.record_event(
                rid, "tower_stage_failed", namespace=mode,
                wakeup_id=wake.get("id"), stage=tower_step, error=error,
            )
            return

    if warehouse_workflow and not _sim_warehouse_complete():
        source = str((mission_intent.get("source") or {}).get("id") or "A")
        destination = str((mission_intent.get("destination") or {}).get("id") or "B")
        selector_obj = mission_intent.get("selector") or {}
        selector = str(selector_obj.get("object_type") or selector_obj.get("mode") or "all")
        mission_id = str(mission_intent.get("mission_id") or "")
        try:
            with use_team_batch(f"{mode}:{consensus_round_id}", 3):
                compiled = sim_actions.run_for_robot(
                    "team_zone_transfer",
                    rid,
                    mission_id=mission_id,
                    source_zone=source,
                    destination_zone=destination,
                    selector=selector,
                )
        except Exception as exc:
            compiled = {"ok": False, "reason": f"{type(exc).__name__}: {exc}"}
        team_bus.record_event(
            rid, "tool", namespace=mode,
            tool="team_zone_transfer", ok=bool(compiled.get("ok")),
            reason=str(compiled.get("reason") or "")[:240],
            mission_id=mission_id, source_zone=source,
            destination_zone=destination, selector=selector,
            source="consensus_execution_compiler",
        )
        authoritative_completion = bool(
            compiled.get("mission_verified")
            or compiled.get("outcome_status") == "ACHIEVED"
        )
        if not compiled.get("ok") or not (
            authoritative_completion or _sim_warehouse_complete()
        ):
            error = str(compiled.get("reason") or "warehouse referee not satisfied")
            team_bus.complete_wakeup(
                str(wake.get("id")), namespace=mode, ok=False, error=error,
            )
            team_bus.record_event(
                rid, "warehouse_mission_failed", namespace=mode,
                wakeup_id=wake.get("id"), mission_id=mission_id, error=error,
            )
            return
        result = {
            "final": f"{rid.upper()} warehouse mission 참여 완료: {source} → {destination}",
            "text": "", "tools": [compiled], "stopped": "warehouse_mission_complete",
        }
        team_bus.record_event(
            rid, "warehouse_mission_complete", namespace=mode,
            wakeup_id=wake.get("id"), mission_id=mission_id,
            source_zone=source, destination_zone=destination,
        )

    if beam_workflow and not _sim_team_beam_complete():
        # The three independent proposals already committed the mission and
        # roles. If a planner answers "ready" without emitting its agreed tool,
        # compile that committed role into the same public SIM action. This is
        # not a central motion shortcut: every role still submits one command,
        # the bridge requires the three-command quorum, and the worker refuses
        # to move until both physical handle contacts are verified.
        role = {
            "r1": "carrier_left",
            "r2": "scout",
            "r3": "carrier_right",
        }[rid]
        try:
            with use_team_batch(f"{mode}:{consensus_round_id}", 3):
                compiled = sim_actions.run_for_robot(
                    "team_beam_transport",
                    rid,
                    mission_id="beam_transport_v1",
                    role=role,
                )
        except Exception as exc:
            compiled = {"ok": False, "reason": f"{type(exc).__name__}: {exc}"}
        team_bus.record_event(
            rid,
            "tool",
            namespace=mode,
            tool="team_beam_transport",
            ok=bool(compiled.get("ok")),
            reason=str(compiled.get("reason") or "")[:240],
            mission_id="beam_transport_v1",
            role=role,
            source="consensus_execution_compiler",
        )
        if not compiled.get("ok") or not _sim_team_beam_complete():
            error = str(compiled.get("reason") or "beam destination referee not satisfied")
            team_bus.complete_wakeup(
                str(wake.get("id")), namespace=mode, ok=False, error=error,
            )
            team_bus.record_event(
                rid,
                "beam_mission_failed",
                namespace=mode,
                wakeup_id=wake.get("id"),
                mission_id="beam_transport_v1",
                role=role,
                error=error,
            )
            return
        result = {
            "final": f"{rid.upper()} {role} 역할로 빔 운반 미션 완료",
            "text": "",
            "tools": [compiled],
            "stopped": "beam_mission_complete",
        }
        team_bus.record_event(
            rid,
            "beam_mission_complete",
            namespace=mode,
            wakeup_id=wake.get("id"),
            mission_id="beam_transport_v1",
            role=role,
        )

    response = str(result.get("final") or result.get("text") or "").strip()
    if consensus_proposal:
        proposal: Any = response or {
            "tools": result.get("tools") or [],
            "stopped": result.get("stopped"),
        }
        try:
            commit = team_bus.submit_consensus_proposal(
                consensus_round_id, rid, proposal, namespace=mode,
            )
        except Exception as exc:
            team_bus.complete_wakeup(
                str(wake.get("id")), namespace=mode, ok=False,
                error=f"consensus proposal failed: {type(exc).__name__}: {exc}",
            )
            return
        team_bus.record_event(
            rid, "consensus_proposal", namespace=mode,
            wakeup_id=wake.get("id"), consensus_round_id=consensus_round_id,
            committed=bool(commit.get("committed")),
        )
        if commit.get("execution_wakeups"):
            team_bus.record_event(
                rid, "consensus_commit", namespace=mode,
                consensus_round_id=consensus_round_id,
                targets=[w.get("robot_id") for w in commit.get("execution_wakeups", [])],
            )
    response_chat_id = None
    if response:
        chat = team_bus.post_agent_chat(
            rid, response, namespace=mode, reply_to=wake.get("source_chat_id"), source="team_autowake_reply"
        )
        response_chat_id = chat.get("id")
    team_bus.complete_wakeup(
        str(wake.get("id")), namespace=mode, ok=True, response_chat_id=response_chat_id
    )
    team_bus.record_event(rid, "team_wake_done", namespace=mode, wakeup_id=wake.get("id"))
    if tower_step in (1, 2):
        next_robot = "r1" if tower_step == 1 else "r2"
        # The planner is expected to hand off itself (TEAM rules). Only when it
        # finished the verified stage without messaging the next robot does the
        # router add the handoff, so the chain cannot stall silently.
        if _team_message_sent_since(mode, rid, next_robot, handoff_seq_floor):
            team_bus.record_event(rid, "tower_handoff", namespace=mode, stage=tower_step, next_robot=next_robot, source="planner")
        else:
            if tower_step == 1:
                team_bus.send_message(
                    "r3", "r1",
                    f"{_TOWER_STEP_2} 협업 2/3: 노란 블럭이 바닥층에 놓였어. 이제 파란 블럭을 노란 블럭 위에 올려 줘.",
                    proposed_action="stack_on", required_partner="r2", namespace=mode,
                )
            else:
                team_bus.send_message(
                    "r1", "r2",
                    f"{_TOWER_STEP_3} 협업 3/3: 파란 블럭이 노란 블럭 위에 올라갔어. 이제 빨간 블럭을 파란 블럭 위에 올려 줘.",
                    proposed_action="stack_on", namespace=mode,
                )
            team_bus.record_event(rid, "tower_handoff", namespace=mode, stage=tower_step, next_robot=next_robot, source="router_fallback")
    elif tower_step == 3:
        team_bus.record_event("r2", "tower_complete", namespace=mode, stage=3)


def _team_goal_text(mode: str) -> str:
    try:
        goal = team_bus.snapshot(namespace=mode).get("goal") or {}
    except Exception:
        return ""
    return str(goal.get("text") if isinstance(goal, dict) else goal or "")


def _team_last_message_seq(mode: str) -> int:
    try:
        messages = team_bus.snapshot(namespace=mode).get("messages") or []
        return max((int(m.get("seq") or 0) for m in messages), default=0)
    except Exception:
        return 0


def _team_message_sent_since(mode: str, sender: str, recipient: str, seq_floor: int) -> bool:
    try:
        messages = team_bus.snapshot(namespace=mode).get("messages") or []
    except Exception:
        return False
    return any(
        int(m.get("seq") or 0) > int(seq_floor)
        and m.get("sender_id") == sender
        and m.get("recipient") in {recipient, "all"}
        for m in messages
    )


def start_team_wake_dispatcher(state: ChatState) -> threading.Thread | None:
    """Start the neutral message router on each robot service process.

    Every robot backend participates. Atomic wake claiming makes duplicate
    dispatch harmless while allowing R1/R2/R3 to execute their own wakeups
    independently and concurrently.
    """
    if os.environ.get("UGRP_TEAM_AUTOWAKE", "1") == "0":
        return None
    stop = threading.Event()
    state._team_dispatch_stop = stop

    def loop() -> None:
        stop.wait(0.35)
        mode = state.ui_mode
        while not stop.is_set():
            dispatched = False
            # Keep SIM and REAL routing ownership isolated. A long-lived REAL R1
            # process must never claim SIM wakeups (and vice versa), otherwise
            # stale code/config in the other service can decide retry/error state.
            for _ in range(3):
                wake = team_bus.claim_wakeup(namespace=mode)
                if wake is None:
                    break
                dispatched = True
                threading.Thread(
                    target=_deliver_team_wakeup, args=(state, mode, wake),
                    name=f"ugrp-team-{mode}-{wake.get('robot_id')}", daemon=True,
                ).start()
            stop.wait(0.08 if dispatched else 0.25)

    thread = threading.Thread(target=loop, name="ugrp-team-dispatcher", daemon=True)
    thread.start()
    return thread


def serve_chat(
    *,
    completer: Completer | None = None,
    replies: list[Any] | None = None,
    model: str | None = None,
    host: str = "127.0.0.1",
    port: int = 8080,
    password: str | None = None,
    execute: bool = False,
    robot: str = "ugrp1",
    robot_id: str = "r1",
    camera_url: str | None = None,
    camera: bool = True,
    actions_path: str | Path | None = None,
    max_steps: int = 12,
    backend: str = "auto",
) -> None:
    from .groq import live_completer
    from .loop import ReplayCompleter
    from .multi_agent import MultiRobotCompleter

    if host not in {"127.0.0.1", "localhost", "::1"} and not password:
        raise SystemExit("error: 외부에 열려면 --password 가 필요합니다.")
    name = "replay"
    if completer is None:
        if replies is not None:
            completer = ReplayCompleter(replies)
        else:
            # Three robots run as three independent planner instances by default.
            # Each instance keeps its own provider/conversation state and only
            # coordinates through team_bus.
            if (
                os.environ.get("UGRP_MULTI_LLM_AGENTS", "1") == "1"
                and os.environ.get("UGRP_MULTI_ROBOT_CHILD", "0") != "1"
            ):
                agents = {}
                for rid in ("r1", "r2", "r3"):
                    agents[rid], _ = live_completer(backend, model)
                completer = MultiRobotCompleter(agents, default=robot_id)
                name = "multi-llm"
            else:
                # Production SIM/REAL launches one child process per robot, so
                # one provider instance here means exactly three LLM agents in
                # the three child processes rather than nine mostly-unused ones.
                completer, name = live_completer(backend, model)
    tunnel = None
    tunnel_lock = threading.Lock()
    opener = None

    def ensure_camera_tunnel():
        nonlocal tunnel
        with tunnel_lock:
            if tunnel is not None and tunnel.proc.poll() is None:
                return tunnel
            if tunnel is not None:
                try:
                    tunnel.close()
                except Exception:
                    pass
                tunnel = None
            tunnel = start_camera_tunnel(host=robot, wait=2.0)
            return tunnel

    def invalidate_camera_tunnel(expected) -> None:
        nonlocal tunnel
        with tunnel_lock:
            if tunnel is not expected:
                return
            try:
                tunnel.close()
            except Exception:
                pass
            tunnel = None

    if camera:
        if camera_url:
            stream_url = stream_url_from_snapshot(camera_url)
            opener = lambda: open_mjpeg_stream(url=stream_url)
        else:
            ensure_camera_service(host=robot)
            tunnel = start_camera_tunnel(host=robot)

            def open_physical_stream():
                # A legacy workflow may have cleanly SIGTERM'd ustreamer.
                # Recover only if nothing already serves :8080; never stop or
                # replace an existing camera owner.
                ensure_camera_service(host=robot)
                current = ensure_camera_tunnel()
                if current is not None:
                    try:
                        return open_mjpeg_stream(url=current.stream_url)
                    except CameraError:
                        invalidate_camera_tunnel(current)
                # Direct SSH is slower to establish, but it is a robust fallback
                # and the pump keeps that one connection open once established.
                return open_mjpeg_stream(host=robot)

            opener = open_physical_stream
    final_verifier = None
    if name == "groq":
        from .verify import verify_final as _verify_final

        final_verifier = lambda goal, final_text, image: _verify_final(
            completer, goal, final_text, image
        )
    state = ChatState(
        completer=completer,
        password=password,
        execute=execute,
        open_stream=opener,
        robot=RobotPanel(host=robot),
        robot_id=robot_id,
        actions_path=actions_path,
        max_steps=max_steps,
        verify_final=final_verifier,
    )
    if camera:
        state.prefer_fresh_camera_snapshot = bool(camera_url)
        # Physical ustreamer prefers one MJPEG client, so its path reuses the
        # cached stream frame. Explicit HTTP camera URLs (simulation/cloud)
        # can safely fall back to /snapshot, which also makes headless API
        # turns work before any browser has opened /api/camera/stream.
        def grabber() -> bytes:
            # Snapshot-backed simulation/cloud cameras are cheap and must be
            # re-read after each action. Physical REAL may reuse only a FRESH
            # cached frame; stale camera evidence is never returned.
            if camera_url:
                jpeg = grab_snapshot(url=camera_url)
                # UI/API snapshot reads update only the presented frame cache.
                # Ordered planner perception is applied by run_loop to the exact
                # image used for its initial/post-action decision.
                state.remember_frame(jpeg, perceive=False)
                return jpeg
            cached = state.cached_frame()
            if cached is not None:
                return cached
            ensure_camera_service(host=robot)
            current = ensure_camera_tunnel()
            if current is not None:
                try:
                    jpeg = grab_snapshot(url=current.snapshot_url)
                except CameraError:
                    invalidate_camera_tunnel(current)
                    ensure_camera_service(host=robot)
                    jpeg = grab_snapshot(host=robot)
            else:
                jpeg = grab_snapshot(host=robot)
            state.remember_frame(jpeg, perceive=False)
            return jpeg

        state.grab_camera = grabber
        try:
            grabber()
        except CameraError:
            pass
        if not camera_url and opener is not None:
            state.start_low_latency_camera(pose_stream=True)
    httpd = make_server(state, host=host, port=port)
    team_dispatcher = start_team_wake_dispatcher(state)
    bound = httpd.server_address
    print(f"UGRP http://{bound[0]}:{bound[1]}/", flush=True)
    print("장면을 보고 말로 맞춘 다음, 허용된 스킬만 실행합니다.", flush=True)
    print("카메라가 없으면 대화만 합니다.", flush=True)
    print(f"실행 도구: {state.actions_path.name}", flush=True)
    if name == "groq":
        nkeys = len(getattr(completer, "keys", []))
        print(f"모델: groq {getattr(completer, 'model_name', '')} ({nkeys} keys)", flush=True)
    elif name == "replay":
        print("모델: replay", flush=True)
    else:
        print(f"모델: {name} {getattr(completer, 'model_name', '')}", flush=True)
    if execute:
        print("execute: on", flush=True)
    else:
        print("execute: dry-run", flush=True)
    if password:
        print("브라우저가 사용자/비밀번호를 물으면 비밀번호만 맞으면 됩니다.", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if state._team_dispatch_stop is not None:
            state._team_dispatch_stop.set()
        state.stop_low_latency_camera()
        if tunnel is not None:
            tunnel.close()
        httpd.server_close()
