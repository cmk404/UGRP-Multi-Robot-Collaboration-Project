from __future__ import annotations

import argparse
import base64
import json
import os
import secrets
import select
import socket
import threading
import time
import asyncio
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import websockets
from sim.worker_contract import REMOTE_WORKER_CONTRACT, worker_contract_compatible

ROOT = Path(__file__).resolve().parents[1]
TOKEN_FILE = ROOT / ".sim_bridge_token"
STREAM_INTERVAL = max(0.02, float(os.environ.get("UGRP_SIM_STREAM_INTERVAL", "0.05")))
SIM_TRACE_ROOT = ROOT / "outputs" / "sim_traces"
ROBOT_IDS = frozenset({"r1", "r2", "r3"})
# Worker publications carry base64 JPEG frames (and, for results, a bounded
# forensic trace); everything else is a small JSON control message.
MAX_WORKER_BODY_BYTES = 16 * 1024 * 1024
MAX_CONTROL_BODY_BYTES = 64 * 1024
# A deep queue only delays failure: each queued command already blocks one HTTP
# caller for up to the command timeout. Fail fast instead of buffering.
MAX_QUEUE_DEPTH = int(os.environ.get("UGRP_SIM_MAX_QUEUE_DEPTH", "16"))
MAX_RETAINED_RESULTS = 64
MAX_CONCURRENT_STREAMS = int(os.environ.get("UGRP_SIM_MAX_STREAMS", "24"))
# A single command should never wait for the team batching window.  Commands
# already present for two or more robot slots are still combined immediately;
# this small window is only a compatibility escape hatch for callers that
# explicitly request coalescing.
REMOTE_COALESCE_WINDOW_S = max(0.02, min(0.05, float(os.environ.get("UGRP_SIM_COALESCE_WINDOW_S", "0.05"))))
# Consensus planners may emit their first tools seconds apart. Only commands
# carrying an explicit shared team_batch_id pay this bounded rendezvous cost;
# ordinary single-robot work remains immediate.
REMOTE_TEAM_BATCH_WINDOW_S = max(
    0.25, min(8.0, float(os.environ.get("UGRP_SIM_TEAM_BATCH_WINDOW_S", "5.0")))
)
MAX_TRACE_QUEUE = max(1, int(os.environ.get("UGRP_SIM_TRACE_QUEUE_MAX", "8")))
# Keep every failure image sequence, but only one in N clean image sequences.
# Unsampled clean actions still persist JSON diagnostics, so regression counts
# and latency metrics remain complete without unbounded JPEG growth.
TRACE_CLEAN_IMAGE_SAMPLE_EVERY = max(
    1, int(os.environ.get("UGRP_SIM_TRACE_CLEAN_IMAGE_SAMPLE_EVERY", "5"))
)
# Endpoints that change simulator authority or robot state. Loopback binding is
# not an auth boundary, so these require the bridge token like worker channels.
CONTROL_ENDPOINTS = frozenset({"/command", "/remote/authority", "/sim/speed"})

# Keep the HTTP transport boundary aligned with the public REAL/SIM action
# contract.  In particular, carry-safe destination acquisition is a first-class
# action and must not be rejected before it reaches the shared REAL controller.
BRIDGE_ALLOWED_ACTIONS = frozenset({
    "warehouse_research",
    "move_forward", "move_backward",
    "turn_left", "turn_right", "stop_motion", "observe_scene",
    "search", "track", "approach", "pick", "put_down",
    "search_destination", "place", "place_on_yellow", "place_on_blue", "stage_base", "stack_on", "team_tower", "team_beam_transport", "team_zone_transfer",
    "map_red", "map_yellow", "map_blue",
    "search_red", "search_blue", "search_yellow",
    "track_red", "track_blue", "track_yellow",
    "approach_red", "approach_blue", "approach_yellow",
    "pick_red", "pick_blue", "pick_yellow",
    "place_on_red", "scan_world", "reset",
})
BRIDGE_COLOR_ACTIONS = frozenset({
    "search", "track", "approach", "pick", "put_down", "stage_base",
    "search_destination", "place", "stack_on",
    "place_on_blue", "place_on_yellow", "place_on_red",
})
BRIDGE_DESTINATION_ACTIONS = frozenset({
    "search_destination", "place", "stack_on",
    "place_on_blue", "place_on_yellow", "place_on_red",
})


def socket_peer_closed(conn: socket.socket) -> bool:
    """Return True after the HTTP stream client has sent FIN.

    MJPEG handlers may have no frame to write while a GPU worker is offline.
    Without an explicit socket check, a browser that switches away from SIM can
    leave the server thread sleeping forever in CLOSE_WAIT because no next
    write occurs to raise BrokenPipeError.
    """
    try:
        readable, _, _ = select.select([conn], [], [], 0)
        if not readable:
            return False
        flags = socket.MSG_PEEK | getattr(socket, "MSG_DONTWAIT", 0)
        try:
            return conn.recv(1, flags) == b""
        except BlockingIOError:
            return False
    except (OSError, ValueError):
        return True


def _safe_worker_label(value, default: str, max_len: int = 32) -> str:
    text = str(value or default).strip()
    cleaned = "".join(ch for ch in text if ch.isalnum() or ch in "_-.")[:max(1, int(max_len))]
    return cleaned or default


class BridgeState:
    def __init__(self, token: str):
        self.token = token
        self.gpu_only = os.environ.get("UGRP_SIM_GPU_ONLY", "0") == "1"
        self.frame: bytes | None = None  # legacy alias for R1
        self.robot_frames: dict[str, bytes] = {}
        self.robot_states: dict[str, dict] = {}
        self.robot_frame_meta: dict[str, dict] = {}
        self.observer_frame: bytes | None = None
        self.observer_frames: dict[str, bytes] = {}
        self.frame_meta: dict = {}
        self.observer_frame_meta: dict[str, dict] = {}
        self.state: dict = {}
        # Worker publications may arrive out of order because camera frames are
        # sent asynchronously. A monotonic sequence fences stale state snapshots.
        self.state_seq: int | None = None
        self.last_seen = 0.0
        self.remote_rtt_ms: float | None = None
        self.remote_clock_offset_ms: float | None = None
        try:
            _speed = int(float(Path(os.environ.get("UGRP_SIM_SPEED_FILE", "/tmp/ugrp_sim_speed")).read_text().strip()))
        except Exception:
            _speed = 1
        self.sim_speed = _speed if _speed in (1, 2, 3) else 1
        self.remote_ws_count = 0
        # Exactly one remote GPU worker is authoritative. Colab can leave a
        # short-lived ghost WebSocket after a runtime is stopped; a newly
        # authenticated worker replaces that socket so two simulators never
        # execute the same inflight command.
        self.remote_ws_generation = 0
        self.remote_ws = None
        self.remote_provider: str | None = None
        self.remote_machine: str | None = None
        self.remote_instance_id: str | None = None
        self.remote_worker_contract: str | None = None
        # Retain the last process identity across a WebSocket outage. A reconnect
        # from the same process preserves its MuJoCo world; a different process
        # is a new simulation episode and must never inherit queued/inflight work.
        self.remote_last_instance_id: str | None = None
        self.remote_episode_id = 0
        # Last world seed confirmed by a successful worker reset in this
        # episode.  Lets persisted traces carry the replay-correct world seed
        # even when the worker predates the world_seed trace field.
        self.last_reset_world_seed: int | None = None
        self.last_reset_episode: int = -1
        # A newly connected GPU worker is parked until the failover controller
        # explicitly hands authority to it. This prevents the CPU fallback and
        # a recovering GPU from consuming the same command queue at once.
        self.remote_authoritative = False
        # If the authoritative process merely lost its socket, the same MuJoCo
        # world is still running on the other side. Remember that so a reconnect
        # from that exact instance resumes immediately instead of parking behind
        # an inflight command it is itself executing (which used to deadlock).
        self.remote_authority_lost_instance_id: str | None = None
        # Browser-visible SIM activity is tracked separately from worker pings.
        # Remote workers publish continuously, so worker traffic must never keep
        # a paid cloud VM alive by itself.
        self.user_activity_ts = time.time()
        self.queue: deque[dict] = deque()
        # A claimed command remains here until /worker/result arrives. This
        # makes a broken long-poll response recoverable: the next worker poll
        # receives the same command instead of silently losing it.
        self.inflight: dict | None = None
        # Which executor claimed the inflight command ("remote" or "local").
        # The other executor must never be handed the same command.
        self.inflight_owner: str | None = None
        self.batch_partial_ids: set[str] = set()
        self.results: dict[str, dict] = {}
        # Command ids whose caller already timed out. A late worker result for
        # one of these must not be mistaken for the reply to a newer command.
        self.abandoned: deque[str] = deque(maxlen=256)
        self.last_sim_trace_id: str | None = None
        self.active_streams = 0
        self.stream_subscriptions: dict[str, int] = {}
        self.stream_control_seq = 0
        self.trace_cv = threading.Condition()
        self.trace_queue: deque[dict] = deque()
        self.trace_inflight = 0
        self.trace_queued = 0
        self.trace_completed = 0
        self.trace_dropped = 0
        self.trace_failed = 0
        self.trace_clean_seen = 0
        self.trace_frames_omitted = 0
        self.trace_writer = threading.Thread(target=self._trace_writer_loop, name="sim-trace-writer", daemon=True)
        self.trace_writer.start()
        self.cv = threading.Condition()

    def _reset_publication_fence_locked(self):
        """Start a clean publication epoch when simulator authority changes.

        state_seq is based on each worker's wall clock, so comparing values from
        two different machines (Oracle vs Colab) is unsafe across a handoff.
        Clearing cached frames/state also prevents the UI from showing the old
        world's state while the newly authoritative worker synchronizes.
        """
        self.state_seq = None
        self.state = {}
        self.frame = None
        self.robot_frames = {}
        self.robot_states = {}
        self.robot_frame_meta = {}
        self.observer_frame = None
        self.observer_frames = {}
        self.last_seen = 0.0

    def _state_is_fresh_locked(self, state_seq) -> bool:
        # Legacy workers without a sequence remain supported. Modern workers
        # always send one, which makes async result/frame ordering deterministic.
        if state_seq is None:
            return True
        try:
            seq = int(state_seq)
        except (TypeError, ValueError):
            return False
        if self.state_seq is not None and seq < self.state_seq:
            return False
        self.state_seq = seq
        return True

    def push_state(self, state: dict, state_seq=None):
        with self.cv:
            accepted = self._state_is_fresh_locked(state_seq)
            if accepted:
                self.state = state
                robots = state.get("robots") if isinstance(state, dict) else None
                if isinstance(robots, dict):
                    self.robot_states = {str(k): dict(v) for k, v in robots.items() if isinstance(v, dict)}
            self.last_seen = time.time()
            self.cv.notify_all()
            return accepted

    def push_frame(self, frame: bytes, state: dict, observer_frame: bytes | None = None, observer_frames: dict[str, bytes] | None = None, state_seq=None, frame_meta: dict | None = None, robot_id: str = "r1"):
        rid = str(robot_id or "r1").strip().lower()
        if rid not in {"r1", "r2", "r3"}:
            return False
        with self.cv:
            if not self._state_is_fresh_locked(state_seq):
                # Keep both image and state coherent: a stale camera frame should
                # not visually rewind the browser after an authoritative result.
                self.last_seen = time.time()
                self.cv.notify_all()
                return False
            self.robot_frames[rid] = frame
            self.robot_states[rid] = dict(state or {})
            if rid == "r1":
                self.frame = frame
            if frame_meta is not None:
                info = dict(frame_meta); info['received_wall_s'] = time.time()
                self.robot_frame_meta[rid] = info
                if rid == "r1":
                    self.frame_meta = dict(info)
            if observer_frame is not None:
                self.observer_frame = observer_frame
            if observer_frames:
                self.observer_frames.update(observer_frames)
                if self.observer_frame is None:
                    self.observer_frame = next(iter(observer_frames.values()))
            # In the 3x worker, frame state is per-robot and must not replace
            # the shared world state published by `type=state`. Keep the legacy
            # single-robot contract, however: older/local workers historically
            # published their only state together with the R1 camera frame.
            if not (isinstance(self.state, dict) and isinstance(self.state.get("robots"), dict)):
                self.state = dict(state or {})
            self.last_seen = time.time()
            self.cv.notify_all()
            return True


    def push_observer_frame(
        self, name: str, frame: bytes, meta: dict | None = None, *, robot_id: str | None = None
    ) -> None:
        """Update presentation-only observer imagery without fencing robot state.

        The three-robot worker reuses one MuJoCo camera name while moving that
        camera to follow a requested robot.  Store those images under a
        robot-qualified key so R1/R2/R3 web pages cannot overwrite each other's
        third-person view.  Keep the unqualified key as a latest-frame alias for
        legacy diagnostics and single-robot clients.
        """
        with self.cv:
            base_key = str(name or "observer")
            rid = str(robot_id or "").strip().lower()
            key = f"{rid}_{base_key}" if rid in {"r1", "r2", "r3"} else base_key
            info = dict(meta or {})
            info["received_wall_s"] = time.time()
            self.observer_frames[key] = frame
            self.observer_frame_meta[key] = dict(info)
            if key != base_key:
                self.observer_frames[base_key] = frame
                self.observer_frame_meta[base_key] = dict(info)
            if base_key == "cctv_front_left" or self.observer_frame is None:
                self.observer_frame = frame
            self.last_seen = time.time()
            self.cv.notify_all()

    def fresh_snapshot(self, key, frame_getter, meta_getter, *, timeout_s=2.0, max_age_s=0.75):
        """A bounded one-shot presentation subscription; never return old bytes."""
        def current():
            meta = meta_getter() or {}
            generated = meta.get("generated_wall_s")
            if not isinstance(generated, (int, float)):
                return None
            age = time.time() - generated
            return frame_getter() if 0.0 <= age <= max_age_s else None
        with self.cv:
            frame = current()
            if frame:
                return frame
        if not self.add_stream_subscription(key):
            return None
        deadline = time.monotonic() + timeout_s
        try:
            with self.cv:
                while True:
                    frame = current()
                    if frame:
                        return frame
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self.cv.wait(remaining)
        finally:
            self.remove_stream_subscription(key)

    def _abort_pending_for_episode_reset_locked(self) -> None:
        """Fail commands captured by an older MuJoCo process.

        A new worker process starts from a fresh world. Replaying an old inflight
        command into that world is unsafe because it may be a middle primitive
        such as pick/carry/place. Wake every waiting HTTP caller explicitly so
        the deterministic executive can restart the original goal from step 1.
        """
        pending: list[dict] = []
        if self.inflight is not None:
            pending.append(self.inflight)
            self.inflight = None
            self.inflight_owner = None
            self.batch_partial_ids.clear()
        while self.queue:
            pending.append(self.queue.popleft())
        for cmd in pending:
            # A coalesced remote batch has no HTTP caller for the synthetic
            # batch id; callers wait on the original per-robot member ids.
            # Fan the episode reset out to those ids or they remain blocked
            # until the 480-second command timeout after a worker replacement.
            members = cmd.get("commands")
            targets = (
                [member for member in members if isinstance(member, dict)]
                if isinstance(members, list)
                else [cmd]
            )
            for target in targets:
                cid = str(target.get("id") or "")
                if not cid:
                    continue
                self.results[cid] = {
                    "ok": False,
                    "action": target.get("action"),
                    "robot_id": target.get("robot_id"),
                    "reason": "SIM_EPISODE_RESET",
                    "failure_code": "SIM_EPISODE_RESET",
                }

    def register_remote(self, ws, provider: str | None = None, machine: str | None = None, instance_id: str | None = None, worker_contract: str | None = None):
        with self.cv:
            old_ws = self.remote_ws
            new_instance_id = _safe_worker_label(instance_id, "worker")
            previous_instance_id = self.remote_last_instance_id
            new_episode = previous_instance_id is None or new_instance_id != previous_instance_id
            if new_episode:
                self.remote_episode_id += 1
                if previous_instance_id is not None:
                    self._abort_pending_for_episode_reset_locked()
            self.remote_last_instance_id = new_instance_id
            self.remote_ws_generation += 1
            generation = self.remote_ws_generation
            self.remote_ws = ws
            self.remote_ws_count = 1
            self.remote_provider = _safe_worker_label(provider, "remote")
            self.remote_machine = _safe_worker_label(machine, "GPU")
            self.remote_instance_id = new_instance_id
            self.remote_worker_contract = _safe_worker_label(worker_contract, "legacy", 96)
            resumed = (
                not new_episode
                and old_ws is None
                and self.remote_authority_lost_instance_id == new_instance_id
            )
            if resumed:
                # Same process, same world, socket blip only: keep executing.
                self.remote_authoritative = True
            elif self.remote_authoritative:
                # Never make a newly recovered GPU authoritative implicitly.
                self.remote_authoritative = False
                self._reset_publication_fence_locked()
            self.remote_authority_lost_instance_id = None
            self.last_seen = time.time()
            self.cv.notify_all()
            return generation, old_ws

    def set_remote_authoritative(self, enabled: bool) -> bool:
        with self.cv:
            enabled = bool(enabled)
            if enabled and self.remote_ws_count != 1:
                return False
            if self.remote_authoritative != enabled:
                self.remote_authoritative = enabled
                self._reset_publication_fence_locked()
            self.cv.notify_all()
            return True

    def remote_is_authoritative(self, generation: int) -> bool:
        with self.cv:
            return (
                generation == self.remote_ws_generation
                and self.remote_ws_count == 1
                and self.remote_authoritative
            )

    def remote_is_current(self, generation: int) -> bool:
        with self.cv:
            return generation == self.remote_ws_generation and self.remote_ws_count == 1

    def unregister_remote(self, generation: int, ws=None):
        with self.cv:
            if generation != self.remote_ws_generation:
                return False
            if ws is not None and self.remote_ws is not ws:
                return False
            lost_instance_id = self.remote_instance_id
            self.remote_ws = None
            self.remote_ws_count = 0
            self.remote_provider = None
            self.remote_machine = None
            self.remote_instance_id = None
            self.remote_worker_contract = None
            if self.remote_authoritative:
                self.remote_authoritative = False
                self.remote_authority_lost_instance_id = lost_instance_id
                self._reset_publication_fence_locked()
            self.cv.notify_all()
            return True

    def mark_user_activity(self) -> None:
        with self.cv:
            self.user_activity_ts = time.time()
            self.cv.notify_all()

    def user_activity_age_s(self) -> float:
        with self.cv:
            return max(0.0, time.time() - float(self.user_activity_ts or 0.0))

    def set_active_streams(self, count: int) -> int:
        """Publish browser subscriber count for the remote worker.

        Worker publications are independent of browser subscribers.  Keeping
        this counter in the bridge lets the websocket sender send a single
        edge-triggered ``stream_control`` message instead of waking/rendering
        the MuJoCo worker forever after the first browser connection.
        """
        with self.cv:
            self.active_streams = max(0, min(MAX_CONCURRENT_STREAMS, int(count)))
            if self.active_streams:
                self.stream_subscriptions = {"legacy:all": self.active_streams}
            else:
                self.stream_subscriptions = {}
            self.stream_control_seq += 1
            self.cv.notify_all()
            return self.active_streams

    def add_stream_subscription(self, key: str) -> bool:
        """Register one concrete browser surface without enabling every view."""
        with self.cv:
            if self.active_streams >= MAX_CONCURRENT_STREAMS:
                return False
            normalized = str(key or "legacy:all")
            self.stream_subscriptions[normalized] = (
                int(self.stream_subscriptions.get(normalized, 0)) + 1
            )
            self.active_streams += 1
            self.stream_control_seq += 1
            self.cv.notify_all()
            return True

    def remove_stream_subscription(self, key: str) -> None:
        with self.cv:
            normalized = str(key or "legacy:all")
            count = int(self.stream_subscriptions.get(normalized, 0))
            if count <= 1:
                self.stream_subscriptions.pop(normalized, None)
            else:
                self.stream_subscriptions[normalized] = count - 1
            self.active_streams = max(0, self.active_streams - 1)
            self.stream_control_seq += 1
            self.cv.notify_all()

    def stream_status(self) -> tuple[bool, int, int]:
        with self.cv:
            return bool(self.active_streams), int(self.active_streams), int(self.stream_control_seq)

    def stream_control(self) -> tuple[dict, int]:
        """Return the minimal worker presentation set required by subscribers."""
        with self.cv:
            keys = {key for key, count in self.stream_subscriptions.items() if count > 0}
            legacy_all = "legacy:all" in keys
            robot_ids = {
                key.split(":", 1)[1]
                for key in keys
                if key.startswith("robot:") and key.split(":", 1)[1] in ROBOT_IDS
            }
            observer_ids = {
                key.split(":", 1)[1]
                for key in keys
                if key.startswith("observer:") and key.split(":", 1)[1] in ROBOT_IDS
            }
            robot_ids.update(observer_ids)
            payload = {
                "type": "stream_control",
                "enabled": bool(self.active_streams),
                "active_streams": int(self.active_streams),
                "robot_ids": sorted(ROBOT_IDS if legacy_all else robot_ids),
                "observer": bool(legacy_all or observer_ids),
                "team_overview": bool(legacy_all or "team_overview" in keys),
            }
            return payload, int(self.stream_control_seq)

    def enqueue(self, action: str, params: dict | None = None) -> dict:
        cmd = {"id": secrets.token_hex(8), "action": action, "created": time.time()}
        if params:
            cmd.update(params)
        with self.cv:
            if len(self.queue) >= MAX_QUEUE_DEPTH:
                raise QueueFull(f"simulation command queue is full ({MAX_QUEUE_DEPTH})")
            self.queue.append(cmd)
            self.cv.notify_all()
        return cmd

    def next_cmd(self, timeout=20.0, owner: str = "remote"):
        deadline = time.time() + timeout
        with self.cv:
            while self.inflight is None and not self.queue and time.time() < deadline:
                self.cv.wait(min(1.0, max(0.0, deadline-time.time())))
            if self.inflight is not None:
                # Redeliver only to the executor that claimed it. Handing the same
                # primitive to the other executor would run it twice.
                if self.inflight_owner not in (None, owner):
                    return None
                self.inflight_owner = owner
                return dict(self.inflight)
            if not self.queue:
                return None
            self.inflight = self.queue.popleft()
            self.inflight_owner = owner
            return dict(self.inflight)

    def next_remote_cmd(self, timeout=20.0, coalesce_s: float = REMOTE_COALESCE_WINDOW_S):
        """Claim remote work, batching pending commands without single-command tax.

        The old implementation waited 750 ms after every first command, even
        when no second robot command existed.  A single command is now claimed
        immediately.  If another robot command is already queued, the batch is
        formed immediately; ``coalesce_s`` is only used when an explicit caller
        asks for a short grace window while a partial team batch is arriving.
        """
        deadline = time.time() + timeout
        with self.cv:
            while self.inflight is None and not self.queue and time.time() < deadline:
                self.cv.wait(min(1.0, max(0.0, deadline - time.time())))
            if self.inflight is not None:
                if self.inflight_owner not in (None, "remote"):
                    return None
                self.inflight_owner = "remote"
                return dict(self.inflight)
            if not self.queue:
                return None

            first = self.queue[0]
            first_robot = str(first.get("robot_id") or "").strip().lower()
            if first_robot not in {"r1", "r2", "r3"} or first.get("action") in {"reset", "warehouse_research"}:
                self.inflight = self.queue.popleft()
                self.inflight_owner = "remote"
                return dict(self.inflight)

            explicit_batch_id = str(first.get("team_batch_id") or "").strip()
            if explicit_batch_id:
                expected = max(2, min(3, int(first.get("team_batch_expected") or 3)))
                # ``timeout`` is the worker's idle poll cadence (currently 1s),
                # not the consensus rendezvous budget. Once the first explicit
                # TEAM member exists, keep the batch open for its dedicated
                # window so independent LLM response skew cannot release a lone
                # carrier command.
                batch_deadline = time.time() + REMOTE_TEAM_BATCH_WINDOW_S
                selected: list[dict] = []
                while True:
                    selected = []
                    seen: set[str] = set()
                    for cmd in self.queue:
                        rid = str(cmd.get("robot_id") or "").strip().lower()
                        if (
                            str(cmd.get("team_batch_id") or "").strip() != explicit_batch_id
                            or rid not in {"r1", "r2", "r3"}
                            or rid in seen
                            or cmd.get("action") in {"reset", "warehouse_research"}
                        ):
                            continue
                        selected.append(cmd)
                        seen.add(rid)
                    if len(selected) >= expected or time.time() >= batch_deadline:
                        break
                    self.cv.wait(min(0.02, max(0.0, batch_deadline - time.time())))
                if len(selected) >= 2:
                    for cmd in selected:
                        self.queue.remove(cmd)
                    self.inflight = {
                        "id": f"parallel-{secrets.token_hex(8)}",
                        "action": "parallel",
                        "created": time.time(),
                        "team_batch_id": explicit_batch_id,
                        "commands": [dict(cmd) for cmd in selected],
                    }
                    self.batch_partial_ids.clear()
                    self.inflight_owner = "remote"
                    return dict(self.inflight)
                # A peer may legitimately decide that no actuator is needed.
                # Release the lone member after the bounded rendezvous instead
                # of mixing it with an unrelated robot command.
                self.inflight = self.queue.popleft()
                self.inflight_owner = "remote"
                return dict(self.inflight)

            # Do not delay a lone action.  An existing second robot command is
            # enough to batch; this preserves parallel dispatch without making
            # every ordinary action pay a fixed coalesce delay.
            robot_ids = {
                str(cmd.get("robot_id") or "").strip().lower()
                for cmd in self.queue
                if str(cmd.get("robot_id") or "").strip().lower() in {"r1", "r2", "r3"}
            }
            if len(robot_ids) < 2 and float(coalesce_s) > 0.0:
                # Explicitly requested grace is bounded to the 20–50 ms range
                # and is useful only for callers that enqueue a team burst.
                batch_deadline = min(deadline, time.time() + min(0.05, max(0.0, float(coalesce_s))))
                while time.time() < batch_deadline:
                    robot_ids = {
                        str(cmd.get("robot_id") or "").strip().lower()
                        for cmd in self.queue
                        if str(cmd.get("robot_id") or "").strip().lower() in {"r1", "r2", "r3"}
                    }
                    if len(robot_ids) >= 2:
                        break
                    self.cv.wait(min(0.01, max(0.0, batch_deadline - time.time())))
            if len(robot_ids) < 2:
                self.inflight = self.queue.popleft()
                self.inflight_owner = "remote"
                return dict(self.inflight)

            selected: list[dict] = []
            seen: set[str] = set()
            for cmd in list(self.queue):
                rid = str(cmd.get("robot_id") or "").strip().lower()
                if rid not in {"r1", "r2", "r3"} or rid in seen or cmd.get("action") in {"reset", "warehouse_research"}:
                    continue
                selected.append(cmd)
                seen.add(rid)
                if len(selected) == 3:
                    break

            if len(selected) < 2:
                self.inflight = self.queue.popleft()
                self.inflight_owner = "remote"
                return dict(self.inflight)

            for cmd in selected:
                self.queue.remove(cmd)
            self.inflight = {
                "id": f"parallel-{secrets.token_hex(8)}",
                "action": "parallel",
                "created": time.time(),
                "commands": [dict(cmd) for cmd in selected],
            }
            self.batch_partial_ids.clear()
            self.inflight_owner = "remote"
            return dict(self.inflight)

    def _drop_command_locked(self, cid: str, reason: str) -> None:
        if self.inflight is not None and self.inflight.get("id") == cid:
            self.inflight = None
            self.inflight_owner = None
        elif self.inflight is not None and isinstance(self.inflight.get("commands"), list):
            if any(str(cmd.get("id") or "") == cid for cmd in self.inflight.get("commands") or []):
                self.results.pop(cid, None)
                self.abandoned.append(cid)
                self.cv.notify_all()
                return
        for cmd in list(self.queue):
            if cmd.get("id") == cid:
                self.queue.remove(cmd)
        self.results.pop(cid, None)
        self.abandoned.append(cid)
        self.cv.notify_all()

    def _trace_job_locked(self, cid: str, trace: dict | None, diagnostic: dict | None) -> dict | None:
        """Create a bounded, immutable-enough trace job while holding ``cv``."""
        if not isinstance(trace, dict) and not isinstance(diagnostic, dict):
            return None
        trace_dict = trace or {}
        world_seed = trace_dict.get("world_seed")
        if (
            world_seed is None
            and self.last_reset_episode == int(self.remote_episode_id)
            and self.last_reset_world_seed is not None
        ):
            world_seed = self.last_reset_world_seed
        seed = _safe_worker_label(
            world_seed if world_seed is not None else trace_dict.get("seed", "na"), "na", 16)
        action = _safe_worker_label(trace_dict.get("action"), "action")
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        trace_id = f"{stamp}_ep{int(self.remote_episode_id):04d}_seed{seed}_{action}_{_safe_worker_label(cid, 'cmd', 20)}"
        return {
            "trace_id": trace_id,
            "world_seed": world_seed,
            "trace": trace,
            "diagnostic": diagnostic,
        }

    @staticmethod
    def _trace_is_failure(job: dict) -> bool:
        diagnostic = job.get("diagnostic") or {}
        status = str(diagnostic.get("status") or "").strip().upper()
        return status not in {"", "CLEAN", "OK", "SUCCESS"} or not bool(job.get("result_ok", True))

    def _enqueue_trace_job(self, job: dict) -> bool:
        """Queue forensic persistence without holding the action/result lock.

        Clean traces are lossy under pressure; failed traces get a short
        backpressure window and evict an older clean job if necessary.
        """
        failure = self._trace_is_failure(job)
        with self.trace_cv:
            if not failure:
                self.trace_clean_seen += 1
                if self.trace_clean_seen % TRACE_CLEAN_IMAGE_SAMPLE_EVERY:
                    trace = job.get("trace")
                    if isinstance(trace, dict) and trace.get("frames"):
                        compact = dict(trace)
                        omitted = len(list(compact.get("frames") or []))
                        compact["frames"] = []
                        compact["frames_omitted_by_sampling"] = omitted
                        job = {**job, "trace": compact}
                        self.trace_frames_omitted += omitted
            if len(self.trace_queue) >= MAX_TRACE_QUEUE:
                if failure:
                    clean_index = next(
                        (i for i, queued in enumerate(self.trace_queue) if not self._trace_is_failure(queued)),
                        None,
                    )
                    if clean_index is not None:
                        del self.trace_queue[clean_index]
                        self.trace_dropped += 1
                    else:
                        # Preserve bounded memory even during an all-failure
                        # storm; newest failure is the most actionable one.
                        self.trace_queue.popleft()
                        self.trace_dropped += 1
                else:
                    self.trace_dropped += 1
                    return False
            self.trace_queue.append(job)
            self.trace_queued += 1
            self.trace_cv.notify_all()
            return True

    def _trace_writer_loop(self):
        while True:
            with self.trace_cv:
                while not self.trace_queue:
                    self.trace_cv.wait()
                job = self.trace_queue.popleft()
                self.trace_inflight += 1
                self.trace_cv.notify_all()
            try:
                persisted = self._persist_sim_trace(job)
                with self.trace_cv:
                    self.trace_inflight -= 1
                    if persisted:
                        self.trace_completed += 1
                    else:
                        self.trace_failed += 1
                    self.trace_cv.notify_all()
            except Exception as exc:
                with self.trace_cv:
                    self.trace_inflight -= 1
                    self.trace_failed += 1
                    self.trace_cv.notify_all()
                print("sim trace persistence failed", repr(exc), flush=True)

    def _persist_sim_trace(self, job: dict) -> str | None:
        trace = job.get("trace") if isinstance(job, dict) else None
        diagnostic = job.get("diagnostic") if isinstance(job, dict) else None
        trace_id = str(job.get("trace_id") or "") if isinstance(job, dict) else ""
        world_seed = job.get("world_seed") if isinstance(job, dict) else None
        if not trace_id or (not isinstance(trace, dict) and not isinstance(diagnostic, dict)):
            return None
        try:
            SIM_TRACE_ROOT.mkdir(parents=True, exist_ok=True)
            trace_dict = trace or {}
            action = _safe_worker_label((trace or {}).get("action"), "action")
            run_dir = (SIM_TRACE_ROOT / trace_id).resolve()
            if run_dir.parent != SIM_TRACE_ROOT.resolve():
                raise ValueError("trace id escaped the trace root")
            run_dir.mkdir(parents=True, exist_ok=False)
            frames = list((trace or {}).get("frames") or [])
            clean_trace = dict(trace or {})
            if clean_trace.get("world_seed") is None and world_seed is not None:
                try:
                    clean_trace["world_seed"] = int(world_seed)
                except (TypeError, ValueError):
                    pass
            clean_trace["frames"] = []
            frame_manifest = []
            for idx, frame in enumerate(frames):
                item = dict(frame or {})
                robot_b64 = item.pop("robot_jpeg_b64", None)
                observer_b64 = item.pop("observer_jpeg_b64", None)
                entry = {"index": idx, **item}
                if robot_b64:
                    name = f"{idx:03d}_robot.jpg"
                    (run_dir / name).write_bytes(base64.b64decode(robot_b64))
                    entry["robot_image"] = name
                if observer_b64:
                    name = f"{idx:03d}_observer.jpg"
                    (run_dir / name).write_bytes(base64.b64decode(observer_b64))
                    entry["observer_image"] = name
                frame_manifest.append(entry)
            clean_trace["frames"] = frame_manifest
            (run_dir / "trace.json").write_text(json.dumps(clean_trace, ensure_ascii=False, indent=2), encoding="utf-8")
            (run_dir / "analysis.json").write_text(json.dumps(diagnostic or {}, ensure_ascii=False, indent=2), encoding="utf-8")
            issues = (diagnostic or {}).get("issues") or []
            lines = [f"# SIM self-observer: {trace_id}", "", f"- status: {(diagnostic or {}).get('status', 'UNKNOWN')}", f"- action: {(diagnostic or {}).get('action', action)}", f"- result reason: {(diagnostic or {}).get('result_reason', '')}", f"- captured frames: {len(frame_manifest)}", ""]
            if issues:
                lines.append("## Detected issues")
                for issue in issues:
                    lines.append(f"- [{issue.get('severity','?')}] {issue.get('code','?')}: {issue.get('summary','')}")
                    if issue.get('evidence'):
                        lines.append(f"  - evidence: {json.dumps(issue.get('evidence'), ensure_ascii=False)}")
            else:
                lines += ["## Detected issues", "- No deterministic anomaly detected for this action."]
            lines += ["", "## Metrics", "```json", json.dumps((diagnostic or {}).get("metrics") or {}, ensure_ascii=False, indent=2), "```", "", "This report is privileged SIM-debug evidence and is never planner input."]
            (run_dir / "analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
            return trace_id
        except Exception as exc:
            print("sim trace persistence failed", repr(exc), flush=True)
            return None

    def trace_status(self) -> dict:
        with self.trace_cv:
            return {
                "queued": len(self.trace_queue),
                "inflight": self.trace_inflight,
                "max_queue": MAX_TRACE_QUEUE,
                "completed": self.trace_completed,
                "dropped": self.trace_dropped,
                "failed": self.trace_failed,
                "clean_image_sample_every": TRACE_CLEAN_IMAGE_SAMPLE_EVERY,
                "frames_omitted": self.trace_frames_omitted,
            }

    def wait_for_trace_jobs(self, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self.trace_cv:
            while (self.trace_queue or self.trace_inflight) and time.monotonic() < deadline:
                self.trace_cv.wait(max(0.0, deadline - time.monotonic()))
            return not self.trace_queue and not self.trace_inflight

    def set_partial_result(self, batch_id: str, member_id: str, result: dict) -> bool:
        """Release one robot caller while the rest of its parallel batch runs."""
        with self.cv:
            if (
                self.inflight is None
                or self.inflight.get("id") != batch_id
                or not isinstance(self.inflight.get("commands"), list)
            ):
                return False
            member = next(
                (
                    cmd for cmd in self.inflight.get("commands") or []
                    if str(cmd.get("id") or "") == str(member_id)
                ),
                None,
            )
            if member is None or member_id in self.abandoned:
                return False
            item = dict(result or {})
            rid = str(member.get("robot_id") or "").strip().lower()
            item.setdefault("robot_id", rid)
            state = item.get("state")
            if rid in ROBOT_IDS and isinstance(state, dict):
                self.robot_states[rid] = dict(state)
                if isinstance(self.state.get("robots"), dict):
                    self.state["robots"][rid] = dict(state)
                self.last_seen = time.time()
            self.results[str(member_id)] = item
            self.batch_partial_ids.add(str(member_id))
            while len(self.results) > MAX_RETAINED_RESULTS:
                self.results.pop(next(iter(self.results)))
            self.cv.notify_all()
            return True

    def set_result(self, cid: str, result: dict):
        with self.cv:
            batch_members = []
            if (
                self.inflight is not None
                and self.inflight.get("id") == cid
                and isinstance(self.inflight.get("commands"), list)
            ):
                batch_members = [dict(cmd) for cmd in self.inflight.get("commands") or []]
            # Strip privileged simulator-debug evidence before the result reaches
            # the robot actor. Persist it as a separate forensic record instead.
            public_result = dict(result or {})
            trace = public_result.pop("_sim_trace", None)
            diagnostic = public_result.pop("_sim_diagnostic", None)
            trace_job = self._trace_job_locked(cid, trace, diagnostic)
            trace_id = trace_job.get("trace_id") if trace_job else None
            if trace_job:
                trace_job["result_ok"] = bool(public_result.get("ok", True))
            if trace_id:
                # Make the id visible immediately; the files may appear a
                # little later on the bounded writer thread.
                self.last_sim_trace_id = trace_id
                public_result["sim_trace_id"] = trace_id
                public_result["sim_observer_status"] = (diagnostic or {}).get("status", "UNKNOWN")
            # A command result is an authoritative action-boundary snapshot.
            result_state = public_result.get("state") if isinstance(public_result, dict) else None
            result_seq = public_result.get("state_seq") if isinstance(public_result, dict) else None
            if isinstance(result_state, dict) and self._state_is_fresh_locked(result_seq):
                rid = str(public_result.get("robot_id") or result_state.get("robot_id") or "")
                if rid in {"r1", "r2", "r3"}:
                    self.robot_states[rid] = dict(result_state)
                    if isinstance(self.state.get("robots"), dict):
                        self.state["robots"][rid] = dict(result_state)
                else:
                    # Backward-compatible single-worker behavior for old tests
                    # and local diagnostic tools that have no robot_id concept.
                    self.state = result_state
                self.last_seen = time.time()
                if (
                    public_result.get("action") == "reset"
                    and public_result.get("ok")
                    and result_state.get("seed") is not None
                ):
                    # A reset result carries the WORLD state: its seed is the
                    # replay key for every trace that follows in this episode.
                    try:
                        self.last_reset_world_seed = int(result_state["seed"])
                        self.last_reset_episode = int(self.remote_episode_id)
                    except (TypeError, ValueError):
                        pass
            if self.inflight is not None and self.inflight.get("id") == cid:
                self.inflight = None
                self.inflight_owner = None
            # Enqueue only after the public result/state has been prepared. The
            # disk-heavy writer never runs under ``self.cv`` and the worker can
            # receive its result immediately.
            if trace_job:
                self._enqueue_trace_job(trace_job)
            if batch_members:
                nested = public_result.get("results") if isinstance(public_result, dict) else None
                nested = nested if isinstance(nested, dict) else {}
                for member in batch_members:
                    member_id = str(member.get("id") or "")
                    rid = str(member.get("robot_id") or "").strip().lower()
                    if not member_id:
                        continue
                    if member_id in self.batch_partial_ids:
                        continue
                    item = dict(nested.get(rid) or {})
                    if not item:
                        item = {
                            "ok": False,
                            "action": member.get("action"),
                            "reason": "parallel worker omitted robot result",
                            "failure_code": "PARALLEL_RESULT_MISSING",
                        }
                    item.setdefault("robot_id", rid)
                    if public_result.get("parallel_timing"):
                        item.setdefault("parallel_timing", public_result.get("parallel_timing"))
                    if member_id not in self.abandoned:
                        self.results[member_id] = item
                self.batch_partial_ids.clear()
                self.cv.notify_all()
                return
            if cid in self.abandoned:
                # The caller already received a timeout for this command; the
                # world state above is still applied, the reply is discarded.
                self.cv.notify_all()
                return
            self.results[cid] = public_result
            while len(self.results) > MAX_RETAINED_RESULTS:
                self.results.pop(next(iter(self.results)))
            self.cv.notify_all()

    def wait_result(self, cid: str, timeout=30.0):
        deadline = time.time() + timeout
        with self.cv:
            while cid not in self.results and time.time() < deadline:
                self.cv.wait(min(1.0, max(0.0, deadline-time.time())))
            result = self.results.pop(cid, None)
            if result is None:
                # Timed out: release the executor slot so the next command is
                # not stuck behind a reply nobody is waiting for any more.
                self._drop_command_locked(cid, "timeout")
            return result


class QueueFull(RuntimeError):
    pass


class Handler(BaseHTTPRequestHandler):
    bridge: BridgeState
    def log_message(self, fmt, *args):
        return

    def _json(self, code, obj):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code); self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b))); self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(b)

    def _auth(self) -> bool:
        token = self.headers.get("X-UGRP-Sim-Token", "")
        return secrets.compare_digest(token, self.bridge.token)

    def do_GET(self):
        p = urlparse(self.path)
        if p.path == "/sim/speed":
            self._json(200, {"ok": True, "sim_speed": self.bridge.sim_speed})
            return
        if p.path == "/health":
            age = None if not self.bridge.last_seen else round(time.time()-self.bridge.last_seen, 2)
            self._json(200, {
                "ok": True, "worker_age_s": age, "state": self.bridge.state, "state_seq": self.bridge.state_seq,
                "remote_ws_connected": bool(self.bridge.remote_ws_count),
                "remote_ws_count": int(self.bridge.remote_ws_count),
                "remote_authoritative": bool(self.bridge.remote_authoritative),
                "remote_provider": self.bridge.remote_provider,
                "remote_machine": self.bridge.remote_machine,
                "remote_instance_id": self.bridge.remote_instance_id,
                "remote_worker_contract": self.bridge.remote_worker_contract,
                "remote_last_instance_id": self.bridge.remote_last_instance_id,
                "remote_episode_id": int(self.bridge.remote_episode_id),
                "remote_ws_generation": int(self.bridge.remote_ws_generation),
                "gpu_only": bool(self.bridge.gpu_only),
                "user_activity_age_s": round(self.bridge.user_activity_age_s(), 2),
                "active_streams": int(self.bridge.active_streams),
                "stream_subscriptions": dict(self.bridge.stream_subscriptions),
                "remote_rtt_ms": self.bridge.remote_rtt_ms,
                "queue_depth": len(self.bridge.queue),
                "trace": self.bridge.trace_status(),
                "robots": dict(self.bridge.robot_states),
                "robot_frame_meta": dict(self.bridge.robot_frame_meta),
                "frame_meta": dict(self.bridge.frame_meta),
                "observer_frame_meta": dict(self.bridge.observer_frame_meta),
                "last_sim_trace_id": self.bridge.last_sim_trace_id,
                "inflight": None if self.bridge.inflight is None else {
                    "id": self.bridge.inflight.get("id"),
                    "action": self.bridge.inflight.get("action"),
                },
            })
            return
        if p.path.startswith("/robot/") and p.path.endswith("/snapshot"):
            parts = [x for x in p.path.split("/") if x]
            rid = parts[1] if len(parts) == 3 else ""
            frame = self.bridge.fresh_snapshot(
                f"robot:{rid}", lambda: self.bridge.robot_frames.get(rid),
                lambda: self.bridge.robot_frame_meta.get(rid))
            if not frame:
                self.send_error(503, "no simulation frame")
                return
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame))); self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(frame); return
        if p.path.startswith("/robot/") and p.path.endswith("/state"):
            parts = [x for x in p.path.split("/") if x]
            rid = parts[1] if len(parts) == 3 else ""
            state = self.bridge.robot_states.get(rid)
            if state is None:
                self._json(404, {"ok": False, "error": "robot state unavailable"}); return
            state = dict(state)
            if state.get("world_seed") is None:
                ws = self.bridge.last_reset_world_seed
                if ws is not None and self.bridge.last_reset_episode == int(self.bridge.remote_episode_id):
                    try:
                        state["world_seed"] = int(ws)
                    except (TypeError, ValueError):
                        pass
            self._json(200, {"ok": True, "robot_id": rid, "state": state}); return
        if p.path == "/snapshot" or p.path.startswith("/observer/") and p.path.endswith("/snapshot"):
            if p.path == "/snapshot":
                frame = self.bridge.fresh_snapshot("robot:r1", lambda: self.bridge.frame,
                                                   lambda: self.bridge.robot_frame_meta.get("r1"))
            elif p.path == "/observer/snapshot":
                frame = self.bridge.observer_frame
            else:
                name = p.path[len("/observer/"):-len("/snapshot")].strip("/")
                observer_robot = next((rid for rid in ROBOT_IDS if name.startswith(rid + "_")), "r2")
                key = "team_overview" if name == "team_overview" else f"observer:{observer_robot}"
                frame = self.bridge.fresh_snapshot(key, lambda: self.bridge.observer_frames.get(name),
                                                   lambda: self.bridge.observer_frame_meta.get(name))
            if not frame:
                self.send_error(503, "no simulation frame")
                return
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame))); self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(frame); return
        if (
            p.path == "/stream"
            or (p.path.startswith("/robot/") and p.path.endswith("/stream"))
            or (p.path.startswith("/observer/") and p.path.endswith("/stream"))
        ):
            name = None
            robot_id = None
            if p.path.startswith("/robot/"):
                parts = [x for x in p.path.split("/") if x]
                robot_id = parts[1] if len(parts) == 3 else None
                if robot_id not in ROBOT_IDS:
                    self.send_error(404, "unknown robot"); return
            elif p.path not in {"/stream", "/observer/stream"}:
                name = p.path[len("/observer/"):-len("/stream")].strip("/")
            if robot_id:
                subscription_key = f"robot:{robot_id}"
            elif p.path == "/stream":
                subscription_key = "robot:r1"
            elif name == "team_overview":
                subscription_key = "team_overview"
            else:
                observer_robot = next(
                    (rid for rid in ROBOT_IDS if str(name or "").startswith(f"{rid}_")),
                    "r2",
                )
                subscription_key = f"observer:{observer_robot}"
            if not self.bridge.add_stream_subscription(subscription_key):
                self.send_error(503, "too many stream clients"); return
            self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store"); self.end_headers()
            try:
                while True:
                    if socket_peer_closed(self.connection):
                        break
                    if robot_id:
                        frame = self.bridge.robot_frames.get(robot_id)
                    elif p.path == "/stream":
                        frame = self.bridge.frame
                    elif name:
                        frame = self.bridge.observer_frames.get(name)
                    else:
                        frame = self.bridge.observer_frame
                    if frame:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                        self.wfile.flush()
                    time.sleep(STREAM_INTERVAL)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                self.bridge.remove_stream_subscription(subscription_key)
            return
        if p.path == "/sim/traces/latest":
            trace_id = self.bridge.last_sim_trace_id
            if not trace_id:
                self._json(404, {"ok": False, "error": "no sim trace yet"}); return
            run_dir = SIM_TRACE_ROOT / trace_id
            try:
                analysis = json.loads((run_dir / "analysis.json").read_text(encoding="utf-8"))
                trace = json.loads((run_dir / "trace.json").read_text(encoding="utf-8"))
                self._json(200, {"ok": True, "trace_id": trace_id, "analysis": analysis, "trace": trace})
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)} )
            return
        if p.path == "/sim/traces":
            items = []
            try:
                if SIM_TRACE_ROOT.exists():
                    for d in sorted((x for x in SIM_TRACE_ROOT.iterdir() if x.is_dir()), reverse=True)[:30]:
                        try:
                            a = json.loads((d / "analysis.json").read_text(encoding="utf-8"))
                        except Exception:
                            a = {}
                        items.append({"trace_id": d.name, "status": a.get("status"), "action": a.get("action"), "issues": a.get("issues", [])})
                self._json(200, {"ok": True, "traces": items})
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
            return
        if p.path == "/worker/next":
            if not self._auth(): self._json(401, {"error":"unauthorized"}); return
            if self.bridge.remote_authoritative:
                # The CPU worker can take a moment to stop after handoff. Keep
                # it alive but unable to claim new commands.
                self._json(200, {"command": None}); return
            timeout = min(60.0, max(0.0, float(parse_qs(p.query).get("timeout", [20])[0])))
            cmd = self.bridge.next_cmd(timeout, owner="local")
            self._json(200, {"command": cmd}); return
        self.send_error(404)

    def do_POST(self):
        p = urlparse(self.path)
        is_worker = p.path.startswith("/worker/")
        if (is_worker or p.path in CONTROL_ENDPOINTS) and not self._auth():
            self._json(401, {"error": "unauthorized", "reason": "missing or invalid X-UGRP-Sim-Token"}); return
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._json(400, {"error": "invalid Content-Length"}); return
        limit = MAX_WORKER_BODY_BYTES if is_worker else MAX_CONTROL_BODY_BYTES
        if n < 0 or n > limit:
            self._json(413, {"error": f"request body must be 0..{limit} bytes"}); return
        raw = self.rfile.read(n)
        if p.path == "/remote/authority":
            try:
                obj = json.loads(raw or b"{}")
                enabled = bool(obj.get("enabled"))
                ok = self.bridge.set_remote_authoritative(enabled)
                if not ok:
                    self._json(409, {"ok": False, "error": "no remote GPU worker connected"})
                else:
                    self._json(200, {"ok": True, "remote_authoritative": enabled})
            except Exception as e:
                self._json(400, {"error": str(e)})
            return
        if p.path == "/worker/frame":
            if self.bridge.remote_authoritative:
                self._json(200, {"ok": True, "ignored": "remote GPU authoritative"})
                return
            try:
                obj = json.loads(raw or b"{}")
                frame = base64.b64decode(obj["jpeg_b64"])
                if not frame.startswith(b"\xff\xd8"): raise ValueError("not jpeg")
                observer = None
                if obj.get("observer_jpeg_b64"):
                    observer = base64.b64decode(obj["observer_jpeg_b64"])
                    if not observer.startswith(b"\xff\xd8"): raise ValueError("observer not jpeg")
                observers = {}
                for name, encoded in (obj.get("observer_jpegs_b64") or {}).items():
                    data = base64.b64decode(encoded)
                    if not data.startswith(b"\xff\xd8"): raise ValueError(f"observer {name} not jpeg")
                    observers[str(name)] = data
                self.bridge.push_frame(frame, obj.get("state") or {}, observer, observers, obj.get("state_seq"), robot_id=str(obj.get("robot_id") or "r1"))
                self._json(200, {"ok": True})
            except Exception as e:
                self._json(400, {"error": str(e)})
            return
        if p.path == "/worker/result":
            if self.bridge.remote_authoritative:
                self._json(200, {"ok": True, "ignored": "remote GPU authoritative"})
                return
            try:
                obj = json.loads(raw or b"{}")
                self.bridge.set_result(str(obj["id"]), obj.get("result") or {})
                self._json(200, {"ok": True})
            except Exception as e:
                self._json(400, {"error": str(e)})
            return
        if p.path == "/sim/user-activity":
            self.bridge.mark_user_activity()
            self._json(200, {"ok": True})
            return
        if p.path == "/sim/speed":
            try:
                obj = json.loads(raw or b"{}")
                value = int(obj.get("speed", 1))
                if value not in (1, 2, 3):
                    raise ValueError("speed must be 1, 2, or 3")
                self.bridge.sim_speed = value
                self._json(200, {"ok": True, "sim_speed": value})
            except Exception as e:
                self._json(400, {"error": str(e)})
            return
        if p.path == "/command":
            try:
                obj = json.loads(raw or b"{}")
                action = str(obj.get("action") or "")
                self.bridge.mark_user_activity()
                if self.bridge.gpu_only and not self.bridge.remote_authoritative:
                    self._json(503, {"ok": False, "action": action, "reason": "GPU_OFFLINE"})
                    return
                # Agent-facing commands mirror scripts/robot_actions.py. Legacy
                # map/colour aliases remain bridge-only for diagnostics/tests,
                # but no independent task policy is exposed through sim_actions.
                allowed = set(BRIDGE_ALLOWED_ACTIONS)
                if os.environ.get("UGRP_ENABLE_LEGACY_GRASP_RL") == "1":
                    allowed.add("grasp_rl")
                if action not in allowed:
                    raise ValueError("unsupported action")
                params = {}
                robot_id = str(obj.get("robot_id") or "r1").strip().lower()
                if robot_id not in {"r1", "r2", "r3"}:
                    raise ValueError("robot_id must be r1, r2, or r3")
                params["robot_id"] = robot_id
                if action == "warehouse_research":
                    from sim.warehouse_research import validate_request
                    params["request"] = validate_request(obj.get("request"))
                if "seed" in obj:
                    if action != "reset":
                        raise ValueError("seed is only valid for reset")
                    raw_seed = obj.get("seed")
                    if isinstance(raw_seed, bool):
                        raise ValueError("seed must be an integer")
                    seed = int(raw_seed)
                    if seed < 0 or seed > 2**31 - 1:
                        raise ValueError("seed must be between 0 and 2147483647")
                    params["seed"] = seed
                if action in BRIDGE_COLOR_ACTIONS:
                    if "target_color" in obj:
                        target_color = str(obj.get("target_color") or "").strip().lower()
                        if target_color not in {"red", "blue", "yellow"}:
                            raise ValueError("target_color must be red, blue, or yellow")
                        params["target_color"] = target_color
                    if "destination_color" in obj:
                        if action not in BRIDGE_DESTINATION_ACTIONS:
                            raise ValueError("destination_color is only valid for destination/placement actions")
                        destination_color = str(obj.get("destination_color") or "").strip().lower()
                        if destination_color not in {"red", "blue", "yellow"}:
                            raise ValueError("destination_color must be red, blue, or yellow")
                        params["destination_color"] = destination_color
                if action in {
                    "move_forward", "move_backward",
                    "turn_left", "turn_right",
                }:
                    speed = int(obj.get("speed", 35))
                    duration = float(obj.get("duration", 0.30))
                    if not 31 <= speed <= 40:
                        raise ValueError("speed must be between 31 and 40")
                    if not 0.10 <= duration <= 0.80:
                        raise ValueError("duration must be between 0.10 and 0.80 seconds")
                    params.update({"speed": speed, "duration": duration})
                if action == "team_beam_transport":
                    mission_id = str(obj.get("mission_id") or "").strip()
                    role = str(obj.get("role") or "").strip().lower()
                    if mission_id != "beam_transport_v1":
                        raise ValueError("mission_id must be beam_transport_v1")
                    if role not in {"carrier_left", "scout", "carrier_right"}:
                        raise ValueError("invalid cooperative beam role")
                    params.update({"mission_id": mission_id, "role": role})
                if action == "team_zone_transfer":
                    mission_id = str(obj.get("mission_id") or "").strip()
                    source_zone = str(obj.get("source_zone") or "").strip().upper()
                    destination_zone = str(obj.get("destination_zone") or "").strip().upper()
                    selector = str(obj.get("selector") or "").strip().lower()
                    if not mission_id.startswith("warehouse_"):
                        raise ValueError("invalid warehouse mission_id")
                    if source_zone not in {"A", "B", "C"} or destination_zone not in {"A", "B", "C"}:
                        raise ValueError("warehouse zones must be A, B, or C")
                    if source_zone == destination_zone:
                        raise ValueError("warehouse source and destination must differ")
                    if selector not in {"all", "plank", "pipe", "crate"}:
                        raise ValueError("invalid warehouse selector")
                    params.update({
                        "mission_id": mission_id,
                        "source_zone": source_zone,
                        "destination_zone": destination_zone,
                        "selector": selector,
                    })
                known = {
                    "request",
                    "action", "robot_id", "seed", "target_color", "destination_color",
                    "speed", "duration", "mission_id", "role", "source_zone",
                    "destination_zone", "selector",
                }
                team_batch_id = str(obj.get("team_batch_id") or "").strip()
                if team_batch_id:
                    if len(team_batch_id) > 96 or not all(
                        ch.isalnum() or ch in "_.:-" for ch in team_batch_id
                    ):
                        raise ValueError("invalid team_batch_id")
                    expected = int(obj.get("team_batch_expected") or 3)
                    if expected not in {2, 3}:
                        raise ValueError("team_batch_expected must be 2 or 3")
                    params.update({
                        "team_batch_id": team_batch_id,
                        "team_batch_expected": expected,
                    })
                known.update({"team_batch_id", "team_batch_expected"})
                unknown = sorted(set(obj) - known)
                if unknown:
                    raise ValueError("unsupported command fields: " + ", ".join(unknown))
                try:
                    cmd = self.bridge.enqueue(action, params)
                except QueueFull as e:
                    self._json(429, {"ok": False, "action": action, "reason": "SIM_QUEUE_FULL", "detail": str(e)})
                    return
                command_timeout_s = float(os.environ.get("UGRP_SIM_COMMAND_TIMEOUT_S", "480"))
                command_timeout_s = max(35.0, min(command_timeout_s, 600.0))
                result = self.bridge.wait_result(cmd["id"], command_timeout_s)
                if result is None:
                    self._json(504, {"ok": False, "action": action, "reason": "simulation worker timeout",
                                     "failure_code": "SIM_WORKER_TIMEOUT"})
                else:
                    self._json(200, result)
            except Exception as e:
                self._json(400, {"error": str(e)})
            return
        self.send_error(404)


class WorkerOnlyHandler(Handler):
    """Local HTTP worker surface. Disabled entirely in production GPU-only mode."""
    def do_GET(self):
        if urlparse(self.path).path != "/worker/next":
            self.send_error(404)
            return
        if self.bridge.gpu_only:
            self._json(503, {"error": "GPU_ONLY", "reason": "Oracle CPU simulation worker is disabled"})
            return
        super().do_GET()

    def do_POST(self):
        if urlparse(self.path).path not in {"/worker/frame", "/worker/result"}:
            self.send_error(404)
            return
        if self.bridge.gpu_only:
            self._json(503, {"error": "GPU_ONLY", "reason": "Oracle CPU simulation worker is disabled"})
            return
        super().do_POST()



async def websocket_worker_handler(ws, bridge: BridgeState):
    """Persistent authenticated worker channel for remote GPU workers."""
    generation = None
    try:
        raw = await asyncio.wait_for(ws.recv(), timeout=10)
        auth = json.loads(raw)
        if auth.get("type") != "auth" or not secrets.compare_digest(str(auth.get("token") or ""), bridge.token):
            await ws.close(code=4401, reason="unauthorized")
            return
        if not worker_contract_compatible(auth.get("worker_contract")):
            await ws.close(code=4403, reason=f"incompatible worker contract; expected {REMOTE_WORKER_CONTRACT}")
            return
        generation, old_ws = bridge.register_remote(ws, auth.get("provider"), auth.get("machine"), auth.get("instance_id"), auth.get("worker_contract"))
        if old_ws is not None and old_ws is not ws:
            try:
                await old_ws.close(code=4000, reason="replaced by newer GPU worker")
            except Exception:
                pass
        await ws.send(json.dumps({"type": "auth_ok"}))

        async def command_sender():
            last_sent_id = None
            was_authoritative = False
            last_stream_control_seq = -1
            while bridge.remote_is_current(generation):
                if not bridge.remote_is_authoritative(generation):
                    was_authoritative = False
                    await ws.send(json.dumps({"type": "ping", "t": time.time(), "user_activity_age_s": round(bridge.user_activity_age_s(), 2)}, separators=(",", ":")))
                    await asyncio.sleep(0.25)
                    continue
                if not was_authoritative:
                    # The authority switch clears the previous worker's cached
                    # state, so explicitly request a fresh remote frame first.
                    await ws.send(json.dumps({"type": "sync"}, separators=(",", ":")))
                    was_authoritative = True
                    last_stream_control_seq = -1
                stream_control, stream_seq = bridge.stream_control()
                if stream_seq != last_stream_control_seq:
                    await ws.send(json.dumps(stream_control, separators=(",", ":")))
                    last_stream_control_seq = stream_seq
                cmd = await asyncio.to_thread(bridge.next_remote_cmd, 1.0)
                if not bridge.remote_is_current(generation):
                    return
                if cmd is not None:
                    cid = str(cmd.get("id"))
                    if cid != last_sent_id:
                        await ws.send(json.dumps({"type": "command", "command": cmd, "sim_speed": bridge.sim_speed}, separators=(",", ":")))
                        last_sent_id = cid
                    else:
                        await asyncio.sleep(0.05)
                else:
                    await ws.send(json.dumps({"type": "ping", "t": time.time(), "user_activity_age_s": round(bridge.user_activity_age_s(), 2)}, separators=(",", ":")))

        sender = asyncio.create_task(command_sender())
        try:
            async for raw in ws:
                if not bridge.remote_is_current(generation):
                    break
                obj = json.loads(raw)
                typ = obj.get("type")
                if typ == "result":
                    # A result can only belong to a command this socket generation
                    # was handed while authoritative. Accept it even if authority
                    # was revoked meanwhile, otherwise the inflight slot leaks.
                    if bridge.remote_is_current(generation):
                        bridge.set_result(str(obj["id"]), obj.get("result") or {})
                elif typ == "partial_result":
                    if bridge.remote_is_current(generation):
                        bridge.set_partial_result(
                            str(obj.get("id") or ""),
                            str(obj.get("member_id") or ""),
                            obj.get("result") or {},
                        )
                elif typ == "state":
                    if bridge.remote_is_authoritative(generation):
                        bridge.push_state(obj.get("state") or {}, obj.get("state_seq"))
                elif typ == "frame":
                    if not bridge.remote_is_authoritative(generation):
                        continue
                    frame = base64.b64decode(obj["jpeg_b64"])
                    if not frame.startswith(b"\xff\xd8"):
                        continue
                    observer = None
                    if obj.get("observer_jpeg_b64"):
                        observer = base64.b64decode(obj["observer_jpeg_b64"])
                    observers = {}
                    for name, encoded in (obj.get("observer_jpegs_b64") or {}).items():
                        data = base64.b64decode(encoded)
                        if data.startswith(b"\xff\xd8"):
                            observers[str(name)] = data
                    bridge.push_frame(
                        frame, obj.get("state") or {}, observer, observers, obj.get("state_seq"),
                        frame_meta={
                            "generated_wall_s": obj.get("generated_wall_s"),
                            "render_ms": obj.get("render_ms"),
                        },
                        robot_id=str(obj.get("robot_id") or "r1"),
                    )
                elif typ == "observer_frame":
                    if not bridge.remote_is_authoritative(generation):
                        continue
                    data = base64.b64decode(obj.get("jpeg_b64") or "")
                    if data.startswith(b"\xff\xd8"):
                        bridge.push_observer_frame(
                            str(obj.get("name") or "cctv_front_left"), data,
                            {"generated_wall_s": obj.get("generated_wall_s"), "render_ms": obj.get("render_ms")},
                            robot_id=str(obj.get("robot_id") or ""),
                        )
                elif typ in {"pong", "hello"}:
                    received = time.time()
                    bridge.last_seen = received
                    if typ == "pong":
                        try:
                            sent = float(obj.get("t"))
                            worker_wall = float(obj.get("worker_wall_s"))
                            if received >= sent:
                                bridge.remote_rtt_ms = round((received - sent) * 1000.0, 3)
                                midpoint = (sent + received) * 0.5
                                bridge.remote_clock_offset_ms = round((worker_wall - midpoint) * 1000.0, 3)
                        except (TypeError, ValueError):
                            pass
        finally:
            sender.cancel()
    except Exception as exc:
        print(f"worker websocket disconnected: {exc}", flush=True)
    finally:
        if generation is not None:
            bridge.unregister_remote(generation, ws)


def start_websocket_worker_server(bridge: BridgeState, host: str, port: int):
    async def runner():
        async with websockets.serve(
            lambda ws: websocket_worker_handler(ws, bridge),
            host,
            port,
            # Long MuJoCo/controller actions can spend tens of seconds inside a
            # synchronous physics/control call.  The bridge already has explicit
            # application ping/pong plus a bounded command timeout, so protocol
            # keepalive must not tear down a healthy in-flight episode.
            ping_interval=None,
            ping_timeout=None,
            close_timeout=5,
            max_size=16 * 1024 * 1024,
            compression=None,
        ):
            await asyncio.Future()
    asyncio.run(runner())


def load_token() -> str:
    if TOKEN_FILE.exists():
        return TOKEN_FILE.read_text().strip()
    token = secrets.token_urlsafe(32)
    TOKEN_FILE.write_text(token + "\n")
    TOKEN_FILE.chmod(0o600)
    return token


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--worker-port", type=int, default=8092)
    ap.add_argument("--worker-ws-port", type=int, default=8093)
    a = ap.parse_args()
    state = BridgeState(load_token())
    handler = type("BoundBridgeHandler", (Handler,), {"bridge": state})
    worker_handler = type("BoundWorkerHandler", (WorkerOnlyHandler,), {"bridge": state})
    srv = ThreadingHTTPServer((a.host, a.port), handler)
    worker_srv = ThreadingHTTPServer((a.host, a.worker_port), worker_handler)
    t = threading.Thread(target=worker_srv.serve_forever, daemon=True)
    t.start()
    ws_thread = threading.Thread(target=start_websocket_worker_server, args=(state, a.host, a.worker_ws_port), daemon=True)
    ws_thread.start()
    print(f"UGRP sim bridge local=http://{a.host}:{a.port} worker=http://{a.host}:{a.worker_port} ws=ws://{a.host}:{a.worker_ws_port}", flush=True)
    try:
        srv.serve_forever()
    finally:
        worker_srv.shutdown(); worker_srv.server_close(); srv.server_close()

if __name__ == "__main__":
    main()
