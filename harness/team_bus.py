"""Process-shared group chat and peer bus for the three-robot platform.

TEAM is a neutral room/router, not a central planner. It stores the shared room
transcript, peer messages, wake-delivery state, acknowledgements, and recent
action events. R1/R2/R3 keep separate planner state and decide locally. SIM and
REAL are isolated namespaces.
"""
from __future__ import annotations

import fcntl
import json
import os
import re
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterable, Mapping

ROBOT_IDS = ("r1", "r2", "r3")
BUS_PATH = Path(os.environ.get("UGRP_TEAM_BUS_PATH", "/tmp/ugrp-team-bus.json"))
LOCK_PATH = Path(os.environ.get("UGRP_TEAM_BUS_LOCK", "/tmp/ugrp-team-bus.lock"))
MAX_MESSAGES = 200
MAX_CHAT = 300
MAX_WAKEUPS = 400
MAX_EVENTS = 300
WAKE_CLAIM_STALE_S = 240.0
_MENTION_RE = re.compile(r"(?<!\w)@(r[123]|robot[123]|[123]|everyone|all|team)\b", re.IGNORECASE)


def _empty() -> dict[str, Any]:
    return {
        "version": 1,
        "goal": None,
        "chat": [],
        "messages": [],
        "wakeups": [],
        "consensus_rounds": [],
        "events": [],
        "updated_at": time.time(),
    }


def normalize_robot_id(value: object, *, allow_all: bool = False) -> str:
    text = str(value or "r1").strip().lower()
    aliases = {"1": "r1", "2": "r2", "3": "r3", "robot1": "r1", "robot2": "r2", "robot3": "r3"}
    text = aliases.get(text, text)
    if allow_all and text in {"all", "team", "everyone", "*"}:
        return "all"
    if text not in ROBOT_IDS:
        raise ValueError("robot id must be r1, r2, or r3")
    return text


def normalize_namespace(value: object | None = None) -> str:
    raw = value
    if raw is None:
        raw = os.environ.get("UGRP_TEAM_NAMESPACE") or os.environ.get("UGRP_UI_MODE") or ""
    text = str(raw or "").strip().lower()
    if not text:
        return ""
    if text not in {"sim", "real"}:
        raise ValueError("team namespace must be sim or real")
    return text


def _paths(namespace: object | None = None) -> tuple[Path, Path]:
    ns = normalize_namespace(namespace)
    if not ns:
        return BUS_PATH, LOCK_PATH
    upper = ns.upper()
    bus = Path(os.environ.get(f"UGRP_TEAM_BUS_{upper}_PATH", f"/tmp/ugrp-team-bus-{ns}.json"))
    lock = Path(os.environ.get(f"UGRP_TEAM_BUS_{upper}_LOCK", f"/tmp/ugrp-team-bus-{ns}.lock"))
    return bus, lock


@contextmanager
def _locked(namespace: object | None = None):
    bus_path, lock_path = _paths(namespace)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        try:
            yield bus_path
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def _read_unlocked(bus_path: Path) -> dict[str, Any]:
    try:
        obj = json.loads(bus_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return _empty()
    if not isinstance(obj, dict) or obj.get("version") != 1:
        return _empty()
    obj.setdefault("goal", None)
    for key in ("chat", "messages", "wakeups", "consensus_rounds", "events"):
        if not isinstance(obj.get(key), list):
            obj[key] = []
    return obj


def _write_unlocked(bus_path: Path, obj: dict[str, Any]) -> None:
    bus_path.parent.mkdir(parents=True, exist_ok=True)
    obj["updated_at"] = time.time()
    temp = bus_path.with_suffix(f".tmp.{os.getpid()}")
    temp.write_text(json.dumps(obj, ensure_ascii=False, indent=2), encoding="utf-8")
    temp.replace(bus_path)


def _next_id(items: Iterable[dict[str, Any]], prefix: str) -> str:
    largest = 0
    for item in items:
        raw = str(item.get("id") or "")
        if not raw.startswith(prefix):
            continue
        try:
            largest = max(largest, int(raw[len(prefix):]))
        except ValueError:
            pass
    return f"{prefix}{largest + 1:06d}"


def snapshot(*, namespace: object | None = None) -> dict[str, Any]:
    with _locked(namespace) as bus_path:
        return _read_unlocked(bus_path)


def reset(*, namespace: object | None = None) -> dict[str, Any]:
    with _locked(namespace) as bus_path:
        obj = _empty()
        _write_unlocked(bus_path, obj)
        return obj


def set_goal(text: str, *, source: str = "operator", namespace: object | None = None) -> dict[str, Any]:
    goal = " ".join(str(text or "").split()).strip()
    if not goal:
        raise ValueError("team goal may not be empty")
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        obj["goal"] = {"text": goal, "source": str(source), "created_at": time.time()}
        _write_unlocked(bus_path, obj)
        return dict(obj["goal"])


def mention_targets(text: str) -> list[str]:
    specific: list[str] = []
    everyone = False
    for match in _MENTION_RE.finditer(str(text or "")):
        token = match.group(1).lower()
        if token in {"everyone", "all", "team"}:
            everyone = True
            continue
        rid = normalize_robot_id(token)
        if rid not in specific:
            specific.append(rid)
    return list(ROBOT_IDS) if everyone else specific


def _append_chat(
    obj: dict[str, Any],
    *,
    sender_id: str,
    message: str,
    kind: str,
    recipient: str = "all",
    source: str = "",
    reply_to: str | None = None,
    peer_message_id: str | None = None,
    targets: Iterable[str] = (),
    intent: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    body = " ".join(str(message or "").split()).strip()
    if not body:
        raise ValueError("team chat message may not be empty")
    item = {
        "id": _next_id(obj["chat"], "c"),
        "sender_id": str(sender_id),
        "author": str(sender_id),
        "recipient": str(recipient or "all"),
        "targets": [str(x) for x in targets],
        "message": body,
        "kind": str(kind),
        "role": str(kind),
        "source": str(source or kind),
        "reply_to": str(reply_to or "") or None,
        "peer_message_id": str(peer_message_id or "") or None,
        "created_at": time.time(),
        "intent": dict(intent) if isinstance(intent, Mapping) else None,
    }
    obj["chat"].append(item)
    obj["chat"] = obj["chat"][-MAX_CHAT:]
    return item


def _append_wakeup(
    obj: dict[str, Any],
    *,
    robot_id: str,
    sender_id: str,
    message: str,
    reason: str,
    source_chat_id: str | None = None,
    source_message_id: str | None = None,
    delay_s: float = 0.0,
    available_at: float | None = None,
    consensus_round_id: str | None = None,
    execution_ready: bool = False,
    intent: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    rid = normalize_robot_id(robot_id)
    wake_id = _next_id(obj["wakeups"], "w")
    item = {
        "id": wake_id,
        "seq": int(wake_id[1:]),
        "robot_id": rid,
        "sender_id": str(sender_id or "operator"),
        "message": " ".join(str(message or "").split()).strip(),
        "reason": str(reason or "team_chat"),
        "source_chat_id": str(source_chat_id or "") or None,
        "source_message_id": str(source_message_id or "") or None,
        "status": "pending",
        "attempts": 0,
        "available_at": (
            float(available_at)
            if available_at is not None
            else time.time() + max(0.0, float(delay_s))
        ),
        "created_at": time.time(),
        "consensus_round_id": str(consensus_round_id or "") or None,
        "execution_ready": bool(execution_ready),
        "intent": dict(intent) if isinstance(intent, Mapping) else None,
    }
    obj["wakeups"].append(item)
    obj["wakeups"] = obj["wakeups"][-MAX_WAKEUPS:]
    return item


def post_operator_chat(
    message: str, *, namespace: object | None = None,
    targets_override: list[str] | tuple[str, ...] | None = None,
    intent_override: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    body = " ".join(str(message or "").split()).strip()
    if not body:
        raise ValueError("team chat message may not be empty")
    explicit = mention_targets(body)
    if targets_override is not None:
        targets = [normalize_robot_id(rid) for rid in targets_override]
        if not targets:
            raise ValueError("targets_override may not be empty")
        targets = list(dict.fromkeys(targets))
    else:
        # Like Grok group chat: ordinary room text is visible to everyone and lets
        # all peers independently decide whether/how to respond; @R2 narrows wakeup.
        targets = explicit or list(ROBOT_IDS)
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        chat = _append_chat(
            obj, sender_id="operator", message=body, kind="operator", recipient="all",
            source="operator_chat", targets=targets, intent=intent_override,
        )
        # A plain SIM TEAM-room message addressed to all three robots first
        # enters a proposal barrier. Directed messages and legacy/default
        # namespaces keep their immediate wakeup behavior.
        use_consensus = normalize_namespace(namespace) == "sim" and targets_override is None and targets == list(ROBOT_IDS)
        consensus_round = None
        if use_consensus:
            round_id = _next_id(obj["consensus_rounds"], "cr")
            consensus_round = {
                "id": round_id,
                "source_chat_id": chat["id"],
                "message": body,
                "targets": list(ROBOT_IDS),
                "status": "collecting",
                "proposals": {},
                "created_at": time.time(),
                "committed_at": None,
                "intent": dict(intent_override) if isinstance(intent_override, Mapping) else None,
            }
            obj["consensus_rounds"].append(consensus_round)
            reason = "consensus_proposal"
            stagger_s = 0.0
        else:
            reason = "team_chat"
            try:
                stagger_s = max(0.0, float(os.environ.get("UGRP_TEAM_WAKE_STAGGER_S", "0.0")))
            except ValueError:
                stagger_s = 0.0
        wakes = [
            _append_wakeup(
                obj,
                robot_id=rid,
                sender_id="operator",
                message=body,
                reason=reason,
                source_chat_id=chat["id"],
                delay_s=(index * stagger_s if len(targets) > 1 else 0.0),
                consensus_round_id=(consensus_round or {}).get("id"),
                intent=intent_override,
            )
            for index, rid in enumerate(targets)
        ]
        _write_unlocked(bus_path, obj)
        result = {"chat": dict(chat), "wakeups": [dict(w) for w in wakes], "targets": list(targets)}
        if consensus_round is not None:
            result["consensus_round"] = dict(consensus_round)
        return result


def submit_consensus_proposal(
    round_id: str,
    robot_id: str,
    proposal: Any,
    *,
    namespace: object | None = None,
) -> dict[str, Any]:
    """Record one robot proposal and atomically release execution at 3/3."""
    rid = normalize_robot_id(robot_id)
    cid = str(round_id or "").strip()
    if not cid:
        raise ValueError("round_id is required")
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        round_state = next((r for r in obj["consensus_rounds"] if r.get("id") == cid), None)
        if round_state is None:
            raise ValueError(f"unknown consensus round: {cid}")
        if rid not in round_state.get("targets", []):
            raise ValueError(f"{rid} is not part of consensus round {cid}")
        proposals = round_state.setdefault("proposals", {})
        proposals[rid] = proposal
        execution_wakes: list[dict[str, Any]] = []
        committed = round_state.get("status") == "committed"
        targets = list(round_state.get("targets") or [])
        if not committed and targets and all(target in proposals for target in targets):
            committed = True
            commit_at = time.time()
            round_state["status"] = "committed"
            round_state["committed_at"] = commit_at
            shared_plan = json.dumps(proposals, ensure_ascii=False, separators=(",", ":"))
            execution_message = (
                f"{round_state.get('message', '')} "
                f"[TEAM CONSENSUS COMMITTED {cid}] proposals={shared_plan}"
            ).strip()
            for target in targets:
                execution_wakes.append(_append_wakeup(
                    obj,
                    robot_id=target,
                    sender_id="team_consensus",
                    message=execution_message,
                    reason="consensus_execute",
                    source_chat_id=round_state.get("source_chat_id"),
                    available_at=commit_at,
                    consensus_round_id=cid,
                    execution_ready=True,
                    intent=round_state.get("intent"),
                ))
        _write_unlocked(bus_path, obj)
        return {
            "committed": bool(committed),
            "consensus_round": dict(round_state),
            "execution_wakeups": [dict(w) for w in execution_wakes],
        }


def post_agent_chat(
    robot_id: str,
    message: str,
    *,
    namespace: object | None = None,
    reply_to: str | None = None,
    source: str = "agent_reply",
) -> dict[str, Any]:
    rid = normalize_robot_id(robot_id)
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        chat = _append_chat(
            obj,
            sender_id=rid,
            message=message,
            kind="agent",
            recipient="all",
            source=source,
            reply_to=reply_to,
        )
        # A plain final reply is display-only. Explicit send_peer_message is the
        # handoff primitive that wakes another Bot, preventing reply storms.
        _write_unlocked(bus_path, obj)
        return dict(chat)


def send_message(
    sender: str,
    recipient: str,
    message: str,
    *,
    proposed_action: str = "",
    required_partner: str = "",
    namespace: object | None = None,
) -> dict[str, Any]:
    sender_id = normalize_robot_id(sender)
    recipient_id = normalize_robot_id(recipient, allow_all=True)
    body = " ".join(str(message or "").split()).strip()
    if not body:
        raise ValueError("peer message may not be empty")
    partner = ""
    if str(required_partner or "").strip():
        partner = normalize_robot_id(required_partner, allow_all=True)
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        seq = 1 + max((int(m.get("seq") or 0) for m in obj["messages"]), default=0)
        item = {
            "id": f"m{seq:06d}",
            "seq": seq,
            "sender_id": sender_id,
            "recipient": recipient_id,
            "message": body,
            "proposed_action": " ".join(str(proposed_action or "").split()).strip(),
            "required_partner": partner,
            "ack": {},
            "created_at": time.time(),
        }
        obj["messages"].append(item)
        obj["messages"] = obj["messages"][-MAX_MESSAGES:]
        targets = (
            [rid for rid in ROBOT_IDS if rid != sender_id]
            if recipient_id == "all"
            else ([] if recipient_id == sender_id else [recipient_id])
        )
        chat = _append_chat(
            obj,
            sender_id=sender_id,
            message=body,
            kind="peer",
            recipient=recipient_id,
            source="peer_message",
            peer_message_id=item["id"],
            targets=targets,
        )
        wakes = [
            _append_wakeup(
                obj,
                robot_id=rid,
                sender_id=sender_id,
                message=body,
                reason="peer_message",
                source_chat_id=chat["id"],
                source_message_id=item["id"],
            )
            for rid in targets
        ]
        item["chat_id"] = chat["id"]
        item["wakeup_ids"] = [w["id"] for w in wakes]
        _write_unlocked(bus_path, obj)
        return dict(item)


def acknowledge(robot_id: str, message_id: str, accepted: bool, note: str = "", *, namespace: object | None = None) -> dict[str, Any]:
    rid = normalize_robot_id(robot_id)
    mid = str(message_id or "").strip()
    if not mid:
        raise ValueError("message_id is required")
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        for item in obj["messages"]:
            if item.get("id") != mid:
                continue
            recipient = str(item.get("recipient") or "")
            sender_id = str(item.get("sender_id") or "")
            if sender_id == rid:
                raise ValueError("sender may not acknowledge its own peer message")
            if recipient not in {rid, "all"}:
                raise ValueError(f"peer message {mid} is not addressed to {rid}")
            item.setdefault("ack", {})[rid] = {
                "accepted": bool(accepted),
                "note": " ".join(str(note or "").split()).strip(),
                "created_at": time.time(),
            }
            _write_unlocked(bus_path, obj)
            return dict(item)
    raise ValueError(f"unknown peer message: {mid}")


def _absorb_visible_wakeups(
    obj: dict[str, Any], rid: str, chat_ids: set[str], message_ids: set[str],
    *, exclude_wakeup_id: str | None = None,
) -> bool:
    changed = False
    now = time.time()
    for wake in obj.get("wakeups", []):
        if wake.get("robot_id") != rid or wake.get("status") not in {"pending", "claimed"}:
            continue
        if exclude_wakeup_id and str(wake.get("id") or "") == str(exclude_wakeup_id):
            continue
        source_chat = str(wake.get("source_chat_id") or "")
        source_message = str(wake.get("source_message_id") or "")
        if source_chat not in chat_ids and source_message not in message_ids:
            continue
        wake["status"] = "absorbed"
        wake["completed_at"] = now
        wake["error"] = ""
        changed = True
    return changed


def context_for(
    robot_id: str, *, after_seq: int = 0, limit: int = 16, namespace: object | None = None,
    exclude_wakeup_id: str | None = None,
) -> dict[str, Any]:
    rid = normalize_robot_id(robot_id)
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        messages = [
            m for m in obj.get("messages", [])
            if int(m.get("seq") or 0) > int(after_seq)
            and (m.get("recipient") in {rid, "all"} or m.get("sender_id") == rid)
        ][-max(1, int(limit)):]
        chat = list(obj.get("chat", []))[-max(1, int(limit)):]
        changed = _absorb_visible_wakeups(
            obj,
            rid,
            {str(c.get("id") or "") for c in chat},
            {str(m.get("id") or "") for m in messages},
            exclude_wakeup_id=exclude_wakeup_id,
        )
        if changed:
            _write_unlocked(bus_path, obj)
        return {
            "goal": obj.get("goal"),
            "chat": chat,
            "messages": messages,
            "consensus_rounds": list(obj.get("consensus_rounds", []))[-4:],
            "last_seq": max((int(m.get("seq") or 0) for m in messages), default=int(after_seq)),
            "events": list(obj.get("events", []))[-20:],
        }


def peer_summary(robot_id: str, *, namespace: object | None = None, max_age_s: float = 900.0) -> dict[str, Any]:
    """Latest self-reported tool activity of every *other* robot.

    Built only from mirrored tool events (what each robot attempted and how it
    ended), never from simulator truth, so the same evidence exists in REAL.
    """
    rid = normalize_robot_id(robot_id)
    now = time.time()
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        events = list(obj.get("events", []))
    peers: dict[str, Any] = {}
    for event in reversed(events):
        pid = str(event.get("robot_id") or "")
        if pid == rid or pid in peers or event.get("kind") != "tool":
            continue
        age = now - float(event.get("created_at") or now)
        if age > max_age_s:
            continue
        peers[pid] = {
            "last_tool": event.get("tool"),
            "ok": bool(event.get("ok")),
            "target_color": event.get("target_color"),
            "age_s": round(age, 1),
        }
    return dict(sorted(peers.items()))


def claim_wakeup(*, namespace: object | None = None) -> dict[str, Any] | None:
    now = time.time()
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        # Recover a claim if the dispatching process disappeared mid-request.
        for wake in obj.get("wakeups", []):
            if wake.get("status") == "claimed" and now - float(wake.get("claimed_at") or now) > WAKE_CLAIM_STALE_S:
                wake["status"] = "pending"
                wake["available_at"] = now
        for wake in obj.get("wakeups", []):
            if wake.get("status") != "pending" or float(wake.get("available_at") or 0) > now:
                continue
            wake["status"] = "claimed"
            wake["claimed_at"] = now
            wake["attempts"] = int(wake.get("attempts") or 0) + 1
            wake["claimed_by_pid"] = os.getpid()
            _write_unlocked(bus_path, obj)
            return dict(wake)
    return None


def release_wakeup(
    wakeup_id: str,
    *,
    namespace: object | None = None,
    delay_s: float = 0.5,
    error: str = "",
    force: bool = False,
) -> dict[str, Any]:
    wid = str(wakeup_id or "").strip()
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        for wake in obj.get("wakeups", []):
            if wake.get("id") != wid:
                continue
            # If an already-running human/agent turn consumed this room message,
            # don't resurrect it after the dispatcher receives robot_busy.
            if wake.get("status") != "claimed" and not force:
                return dict(wake)
            wake["status"] = "pending"
            wake["available_at"] = time.time() + max(0.05, float(delay_s))
            wake["error"] = " ".join(str(error or "").split()).strip()[:300]
            _write_unlocked(bus_path, obj)
            return dict(wake)
    raise ValueError(f"unknown TEAM wakeup: {wid}")


def complete_wakeup(
    wakeup_id: str,
    *,
    namespace: object | None = None,
    ok: bool,
    error: str = "",
    response_chat_id: str | None = None,
) -> dict[str, Any]:
    wid = str(wakeup_id or "").strip()
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        for wake in obj.get("wakeups", []):
            if wake.get("id") != wid:
                continue
            wake["status"] = "done" if ok else "error"
            wake["completed_at"] = time.time()
            wake["error"] = " ".join(str(error or "").split()).strip()[:300]
            wake["response_chat_id"] = str(response_chat_id or "") or None
            _write_unlocked(bus_path, obj)
            return dict(wake)
    raise ValueError(f"unknown TEAM wakeup: {wid}")


def record_event(robot_id: str, kind: str, *, namespace: object | None = None, **fields: Any) -> None:
    try:
        rid = normalize_robot_id(robot_id)
    except ValueError:
        return
    with _locked(namespace) as bus_path:
        obj = _read_unlocked(bus_path)
        item = {"robot_id": rid, "kind": str(kind), "created_at": time.time(), **fields}
        obj["events"].append(item)
        obj["events"] = obj["events"][-MAX_EVENTS:]
        _write_unlocked(bus_path, obj)
