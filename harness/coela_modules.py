"""Independent observation, memory, communication, planning and execution modules.

CoELA-inspired interfaces, not a reproduction of its complete architecture.
Only explicit local evidence may cross the planner boundary.
"""
from __future__ import annotations

import copy
import json
import threading
import time
import uuid

ROBOTS = ("r1", "r2", "r3")
MODES = ("none", "structured", "natural")


class Perception:
    def __init__(self, environment, robot_id, episode_id):
        self.environment, self.robot_id, self.episode_id = environment, robot_id, episode_id

    def read(self):
        result = self.environment.request(self.robot_id, {"operation": "mixed_observe", "episode_id": self.episode_id})
        if not result.get("observation_id"):
            raise ValueError(result.get("reason", "OBSERVATION_UNAVAILABLE"))
        keys = ("robot_id", "observation_id", "revision", "sim_time", "cargo", "can_carry", "controller_contract", "capacity_contract")
        return copy.deepcopy({k: result[k] for k in keys if k in result})


class Memory:
    def __init__(self, robot_id, *, clock=time.monotonic):
        self.robot_id = robot_id
        self.observations = {}
        self.reports = []
        self.outcomes = []
        self.decisions = []
        self._seen_outcomes = set()
        self.clock = clock
        self._received_at = {}
        self.peer_observations = {}

    def observe(self, observation, sim_time):
        sim_time = observation.get("sim_time", sim_time)
        for cargo in observation["cargo"]:
            cid = cargo["cargo_id"]
            previous = self.observations.get(cid, {})
            self.observations[cid] = {**copy.deepcopy(cargo), "source_robot": self.robot_id,
                "last_seen_sim_time": cargo.get("observed_sim_time") if cargo.get("observed_sim_time") is not None else (sim_time if cargo.get("current_view") else previous.get("last_seen_sim_time")),
                "source_observation_id": observation["observation_id"]}

    def receive(self, message):
        # Peer assertions never overwrite directly measured object facts.
        self.reports.append(copy.deepcopy(message))
        self._received_at[message.get("message_id", str(len(self.reports)))] = self.clock()
        for cid in message.get("observed_ids", []):
            self.peer_observations[(message["sender_id"], cid)] = {
                "cargo_id": cid, "sender_id": message["sender_id"],
                "reported_sim_time": message.get("sent_sim_time"), "message_id": message.get("message_id"),
                "source": "explicit_peer_identity_report"}

    def _expired(self, message):
        if message.get("kind") != "intent": return False
        received = self._received_at.get(message.get("message_id"))
        return received is not None and self.clock() - received >= float(message.get("intent_ttl_s", 12.))

    def active_reports(self):
        return [copy.deepcopy(m) for m in self.reports[-12:] if not self._expired(m)]

    def execution(self, local):
        for item in local.get("history", []):
            key = (item["assignment_id"], item["event"])
            if key not in self._seen_outcomes:
                self.outcomes.append(copy.deepcopy(item)); self._seen_outcomes.add(key)

    def snapshot(self):
        return copy.deepcopy({"observations": list(self.observations.values()),
            "peer_reports": self.active_reports(), "peer_observations": list(self.peer_observations.values()),
            "expired_peer_intents": [{"sender_id": m["sender_id"], "content": m["content"],
                "status": "EXPIRED_UNCONFIRMED_INTENT"} for m in self.reports[-12:] if self._expired(m)],
            "own_execution_history": self.outcomes[-12:],
            "own_decisions": self.decisions[-6:]})


class Communication:
    """Transport with explicit recipients, optional silence and auditable delivery."""
    def __init__(self, mode, max_messages=24, max_bytes=24000):
        if mode not in MODES: raise ValueError("UNKNOWN_COMMUNICATION_MODE")
        self.mode, self.max_messages, self.max_bytes = mode, max_messages, max_bytes
        self.inboxes = {r: [] for r in ROBOTS}
        self.sent = []
        self.counts = {r: 0 for r in ROBOTS}
        self.byte_counts = {r: 0 for r in ROBOTS}
        self.lock = threading.RLock()

    def send(self, sender, message, sim_time):
        if message is None: return None
        if self.mode == "none": raise ValueError("COMMUNICATION_DISABLED")
        if sender not in ROBOTS or not isinstance(message, dict): raise ValueError("INVALID_MESSAGE")
        recipients = message.get("recipients")
        if not isinstance(recipients, list) or not recipients or set(recipients)-set(ROBOTS) or sender in recipients or len(set(recipients)) != len(recipients):
            raise ValueError("INVALID_RECIPIENTS")
        content = message.get("content")
        # Explicit observation assertions use the same envelope in B and C.
        # A cargo mention (including questions/negations) is never a receipt.
        observed_ids = message.get("observed_ids", content.get("observed_ids", []) if isinstance(content, dict) else [])
        if not isinstance(observed_ids, list) or set(observed_ids)-{"oak_plank_01", "small_box_01", "small_box_02"}:
            raise ValueError("INVALID_OBSERVATION_REPORT")
        kind = message.get("kind", content.get("kind", "intent") if isinstance(content, dict) else "intent")
        if kind not in {"observation", "intent", "completed", "help_request", "recovery"}: raise ValueError("INVALID_MESSAGE_KIND")
        if self.mode == "natural":
            if not isinstance(content, str) or not content.strip(): raise ValueError("TEXT_REQUIRED")
        else:
            if not isinstance(content, dict) or set(content)-{"kind", "cargo_id", "participants", "event_id", "action", "reason_code", "observed_ids"}:
                raise ValueError("INVALID_STRUCTURED_MESSAGE")
            if content.get("kind") not in {"observation", "intent", "completed", "help_request", "recovery"}:
                raise ValueError("INVALID_MESSAGE_KIND")
            if "cargo_id" in content and content["cargo_id"] not in {"oak_plank_01", "small_box_01", "small_box_02"}:
                raise ValueError("INVALID_CARGO_ID")
            if "participants" in content and (not isinstance(content["participants"], list) or set(content["participants"])-set(ROBOTS)):
                raise ValueError("INVALID_PARTICIPANTS")
            if "observed_ids" in content and (not isinstance(content["observed_ids"], list) or set(content["observed_ids"])-{"oak_plank_01", "small_box_01", "small_box_02"}):
                raise ValueError("INVALID_OBSERVATION_REPORT")
            if "action" in content and content["action"] not in {"resume", "replan", "request_help", "cancel", "wait", "claim"}:
                raise ValueError("INVALID_MESSAGE_ACTION")
            if "reason_code" in content and content["reason_code"] not in {"obstacle_changed", "overload", "cause_unknown", "task_selected", "task_completed", "need_partner"}:
                raise ValueError("INVALID_REASON_CODE")
            if "event_id" in content and (not isinstance(content["event_id"], str) or len(content["event_id"]) != 32 or any(c not in '0123456789abcdef' for c in content["event_id"])):
                raise ValueError("INVALID_EVENT_ID")
        size = len(json.dumps(message, ensure_ascii=False).encode())
        with self.lock:
            if self.counts[sender] >= self.max_messages or self.byte_counts[sender]+size > self.max_bytes:
                raise ValueError("COMMUNICATION_BUDGET_EXHAUSTED")
            event = {"message_id": uuid.uuid4().hex, "sender_id": sender, "recipients": recipients,
                     "content": copy.deepcopy(content), "kind": kind, "observed_ids": list(observed_ids),
                     "sent_sim_time": sim_time, "bytes": size, "mode": self.mode}
            if kind == "intent": event["intent_ttl_s"] = 12.
            self.counts[sender] += 1; self.byte_counts[sender] += size
            self.sent.append(event)
            for recipient in recipients: self.inboxes[recipient].append(copy.deepcopy(event))
            return copy.deepcopy(event)

    def receive(self, recipient, sim_time):
        with self.lock:
            events, self.inboxes[recipient] = self.inboxes[recipient], []
        return [{**e, "received_sim_time": sim_time} for e in events]


PROMPT = """You are one independent warehouse robot. Use ONLY your own local
observation, memory, received reports and actuator feedback. The manifest and
team IDs are static operator knowledge; other robots' availability and task
completion are UNKNOWN unless you observed them or received a report.
Choose your own work. One carrier for a small box; two consenting carriers for
the plank. Minimize TEAM completion time, not merely your nearest pickup:
two solo jobs can leave the third robot unable to work, whereas a consenting
pair can transport the plank while a third robot handles boxes. Decide using
your evidence and actual proposals; no robot has a fixed role.
For a NEW task first use action kind 'propose' with cargo_id and participants.
This is your local nonbinding intent, not an executed/reserved task. Use the next
planning turn to consider received intentions before kind 'claim' with the same
proposal; revise by proposing again if necessary. This stage also applies in
mode none, without sending anything. Avoid competing for cargo a peer explicitly
intends to carry when a compatible joint/solo task is available. Resolve competing
offers locally from received information, without inventing a peer acceptance.
Choose partners yourself. Never issue commands for a peer. Retain a
pending proposal unless new evidence warrants a change; partners must submit
matching sorted participants to begin. Own completed cargo should not be retried.
Reports are claims, not guaranteed facts. You can receive messages while busy;
while active only wait or recover, never claim another cargo. When a task pauses,
use local evidence and peer reports to decide whether to replan, resume, ask for
help, or cancel. Unknown stop cause is NOT evidence that it is safe to resume.
You may ask a partner for the cause or wait. For obstacle_changed, replan is the
available path recovery; overload must request_help or cancel. Joint recovery
requires the same action from both participants; unrelated robots keep working.
The execution layer uses common simulator geometry; do not claim camera-based
obstacle detection when evidence says controlled_scenario_event.
Return JSON only with action, private_reason and message:
{"action":{"kind":"propose","cargo_id":"ID","participants":["r1","r2"]},
"private_reason":"short reason","message":null}.
Other actions: {"kind":"propose","cargo_id":"ID","participants":["r1","r2"]},
{"kind":"wait"}, or {"kind":"recover","event_id":"exact ID",
"recovery_action":"replan|resume|request_help|cancel"}.
message is OPTIONAL. Choose useful recipients and avoid repeating stale messages.
For mode none it MUST be null. For natural use {"recipients":["r2"],"content":
"한국어 메시지; keep cargo IDs and event IDs literal so partners can identify them",
"kind":"intent","observed_ids":[]}.
For structured use {"recipients":["r2"],"content":{"kind":"intent",
"cargo_id":"ID","participants":["r1","r2"]}}. Allowed content fields are
kind (observation|intent|completed|help_request|recovery), cargo_id, participants,
event_id, action (resume|replan|request_help|cancel|wait|claim), observed_ids,
reason_code (obstacle_changed|overload|cause_unknown|task_selected|task_completed|need_partner).
Omit irrelevant fields; there is no free text in structured messages.
Never put private_reason in messages unless you explicitly choose natural text.
Use kind=intent for a proposal; never call it started or completed before your
own execution feedback confirms that status. In both message modes an optional
OUTER observed_ids array explicitly reports identities you observed; mere text
mention does not authorize a peer observation. Silence is preferable to repeating
unchanged information. Unknown peer availability must remain unknown.
An intent is a proposal, NEVER proof of reservation or completion. Unconfirmed
intents expire after 12 receiver-local seconds. Expired intentions mean UNKNOWN,
not busy and not completed; reconsider unhandled tasks instead of waiting on
an expired promise. Older private_reason text is not evidence of peer status.
When your own execution first confirms acceptance or completion, you get an
opportunity to notify peers. Explicitly state that confirmed status and tell
recipients of an earlier different proposal that you switched plans. Send to
the affected peers, not only the new partner. Only use kind=completed for a
cargo confirmed completed in your OWN execution history.
"""


class PlanningError(ValueError):
    def __init__(self, reason, raw, usage):
        super().__init__(reason)
        self.raw, self.usage = raw, usage


class Planner:
    evidence_kind = "live_llm"
    def __init__(self, robot_id, completer):
        self.robot_id, self.completer = robot_id, completer
        self.model_name = completer.model_name

    def decide(self, context):
        raw = self.completer.complete([{"role": "system", "content": PROMPT},
                                       {"role": "user", "content": json.dumps(context, ensure_ascii=False)}])
        usage = copy.deepcopy(getattr(self.completer, "last_usage", None))
        try:
            text = raw.strip()
            if text.startswith("```"): text = text.split("\n", 1)[1].rsplit("```", 1)[0]
            decision = json.loads(text)
            if not isinstance(decision, dict) or not isinstance(decision.get("action"), dict):
                raise ValueError("INVALID_PLAN")
        except (ValueError, IndexError, TypeError) as exc:
            raise PlanningError(str(exc), raw, usage) from exc
        return {"decision": decision, "raw": raw, "usage": usage}


class Execution:
    def __init__(self, environment, robot_id, episode_id, destination):
        self.environment, self.robot_id, self.episode_id, self.destination = environment, robot_id, episode_id, destination
        self.proposal = None

    def retire(self):
        return self.environment.request(self.robot_id, {"operation": "mixed_retire", "episode_id": self.episode_id})

    def local_status(self):
        value = self.environment.request(self.robot_id, {"operation": "mixed_local_status", "episode_id": self.episode_id})
        if not value.get("ok"): raise ValueError(value.get("reason", "LOCAL_STATUS_UNAVAILABLE"))
        return value

    def execute(self, action, observation, reports=()):
        kind = action.get("kind")
        if kind == "wait": return {"ok": True, "waiting": True}
        if kind in {"propose", "claim"}:
            participants = action.get("participants", [])
            if not isinstance(participants, list) or self.robot_id not in participants or set(participants)-set(ROBOTS):
                raise ValueError("INVALID_PARTICIPANTS")
            candidate = {"cargo_id": action["cargo_id"], "participants": sorted(participants)}
            if kind == "propose":
                self.proposal = candidate
                return {"ok": True, "proposed": copy.deepcopy(candidate), "not_started": True}
            pending = self.local_status().get("pending")
            if pending and pending["cargo_id"] == action["cargo_id"] and sorted(pending["participants"]) == sorted(participants):
                return {"ok": True, "pending": pending}
            if self.proposal != candidate: return {"ok": False, "reason": "PROPOSE_BEFORE_CLAIM"}
            proposal = {"robot_id": self.robot_id, "cargo_id": action["cargo_id"],
                "participants": participants, "destination": self.destination,
                "revision": observation["revision"], "observation_id": observation["observation_id"]}
            return self.environment.request(self.robot_id, {"operation": "mixed_local_submit", "episode_id": self.episode_id,
                "proposals": [proposal], "reports": list(reports)})
        if kind == "recover":
            return self.environment.request(self.robot_id, {"operation": "mixed_recover", "episode_id": self.episode_id,
                "event_id": action["event_id"], "action": action["recovery_action"]})
        raise ValueError("UNKNOWN_ACTION")


def actor_context(robot_id, manifest, destination, mode, observation, memory, local):
    # Explicit allowlist: no supervisor state dictionary is accepted here.
    return {"robot_id": robot_id, "team_ids": list(ROBOTS), "manifest": copy.deepcopy(manifest),
        "destination": destination, "communication_mode": mode, "observation": copy.deepcopy(observation),
        "observation_age_sim_s": max(0., local["sim_time"]-observation.get("sim_time", local["sim_time"])),
        "memory": memory.snapshot(), "own_active_task": copy.deepcopy(local["active"]),
        "own_pending_claim": copy.deepcopy(local["pending"]), "own_recovery_events": copy.deepcopy(local["recovery_events"])}
