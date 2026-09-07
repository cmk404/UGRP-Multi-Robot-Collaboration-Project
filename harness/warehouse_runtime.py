"""Independent robot decisions, message delivery and a non-planning barrier.

No role/cargo optimizer lives here. Every executed assignment must be authored
and agreed by all three policies. The rule policy is explicitly a comparator.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import dataclass, asdict
import copy
import hashlib
import json
from pathlib import Path
import time
from typing import Any

from harness.warehouse_protocol import ROBOT_IDS, ROLES

SYSTEM_PROMPT = """You are ONE robot in a warehouse experiment, not a team supervisor.
Choose your own next role, cargo, and a complete proposed roster using ONLY your
local RGB-D observation, your own capability, operator manifest/goal and inbox.
Camera bearing is positive to your RIGHT. Do not invent unseen objects or peer
locations. Only seen cargo or cargo named in a received peer report is eligible.
Locally remembered cargo IDs are valid evidence, but current_view=false means
you cannot claim to see them now and their old range/bearing is unavailable.
There are exactly 2 carriers (carrier_0, carrier_1) and 1 scout for each cargo.
The ONLY robot IDs are r1, r2, r3. There is NO r0. Every roster must contain
each of r1, r2, r3 exactly once, and your role must map to your own robot_id.
carrier_0 holds the negative-Y end, carrier_1 the positive-Y end. All three must
agree on the same cargo, destination and roster. A robot with can_carry=false
must be scout. Adapt to information from peers and previous failed attempts.
No external controller will repair your choice. With no inbox you have no peer
information; never claim to have received any. You may propose a simple initial
roster and revise it after messages. Keep message natural language and include
your capability, cargo ID, COMPLETE proposed roster and reason. Peers see ONLY
your natural-language message, not your JSON choices or sensor values. Report
the cargo IDs you actually see. If no cargo is supported, use cargo_id="".
Return ONLY JSON: {"robot_id":"YOUR_ID","role":"carrier_0|carrier_1|scout",
"cargo_id":"ID","destination":"ZONE",
"assignments":{"carrier_0":"r?","carrier_1":"r?","scout":"r?"},
"message":"brief natural-language evidence and proposed cooperation"}.
Revision/observation IDs are transport metadata stamped by your caller, not
decisions to increment. When peers disagree, prefer a compatible complete roster
already proposed by a peer and keep it stable while asking the others to agree;
do not keep swapping roles every round without new evidence. If two proposals
already have the same eligible roster/cargo, agree to that majority proposal.
If tied, adopt the eligible proposal from the lexicographically smallest sender
among yourself and received peers. Apply this locally; there is no central
arbiter selecting your plan. Reject a proposal that contradicts known capability.
"""


@dataclass(frozen=True)
class EpisodeBudget:
    max_rounds: int = 12
    max_actions: int = 6
    response_tokens: int = 512
    call_timeout_s: float = 60.0

    def __post_init__(self):
        if not (1 <= self.max_rounds <= 60 and 1 <= self.max_actions <= 30
                and 128 <= self.response_tokens <= 2048 and 1 <= self.call_timeout_s <= 180):
            raise ValueError("invalid experiment budget")


class Journal:
    def __init__(self, path: str | Path | None):
        self.path = Path(path) if path else None
        self.events = []
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists():
                raise ValueError("refusing to overwrite an experiment journal")

    def write(self, event: str, **fields):
        record = {"event": event, **fields}
        # Copy now; future actor mutations cannot alter recorded inputs.
        encoded = json.dumps(record, ensure_ascii=False, sort_keys=True, allow_nan=False)
        self.events.append(json.loads(encoded))
        if self.path:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(encoded + "\n")


class LocalEnvironment:
    """The same request/action contract as the live Bridge environment."""
    def __init__(self, world):
        self.world = world

    def request(self, robot_id, request):
        result = self.world.act(robot_id, "warehouse_research", request=request)
        return result.state.get("research", {"ok": False, "reason": result.reason})

    def observe_all(self, episode_id, sequence):
        result = self.request("r1", {"operation": "observe_batch", "episode_id": episode_id, "sequence": sequence})
        return result.get("observations", {})


class BridgeEnvironment:
    def observe_all(self, episode_id, sequence):
        result = self.request("r1", {"operation": "observe_batch", "episode_id": episode_id, "sequence": sequence})
        return result.get("observations", {})

    def request(self, robot_id, request):
        from scripts.sim_actions import _post_bridge_action
        result = _post_bridge_action("warehouse_research", {"request": request}, robot_id=robot_id)
        return result.get("research") or (result.get("state") or {}).get("research") or {
            "ok": False, "reason": result.get("reason", "RESEARCH_WORKER_UNAVAILABLE")}


class RulePolicy:
    """Robot-local, declared deterministic baseline; never an LLM fallback."""
    model_name = "robot_local_rule_v1"
    evidence_kind = "rule"

    def __init__(self, robot_id):
        self.robot_id = robot_id

    def decide(self, context):
        obs = context["observation"]
        visible = {c["cargo_id"] for c in obs["cargo"]}
        disabled = set()
        if not obs["capabilities"]["can_carry"]:
            disabled.add(self.robot_id)
        # This information exists only as explicit messages in the comm arm.
        for message in context["inbox"]:
            for c in message.get("observed_cargo", []):
                visible.add(c["cargo_id"])
            if not message.get("can_carry", True):
                disabled.add(message["sender_id"])
        scout = sorted(disabled)[0] if disabled else "r2"
        carriers = [rid for rid in ROBOT_IDS if rid != scout]
        assignments = dict(zip(ROLES, (*carriers, scout)))
        role = next(role for role, rid in assignments.items() if rid == self.robot_id)
        return {
            "robot_id": self.robot_id, "role": role,
            "cargo_id": sorted(visible)[0] if visible else "",
            "destination": obs["destination"], "revision": obs["revision"],
            "observation_id": obs["observation_id"], "assignments": assignments,
            "message": f"I can_carry={obs['capabilities']['can_carry']}; I propose {role} for the first observed pending cargo.",
        }


class LLMPolicy:
    evidence_kind = "live_llm"

    def __init__(self, robot_id, completer, *, response_tokens=512):
        self.robot_id, self.completer = robot_id, completer
        self.model_name = getattr(completer, "model_name", type(completer).__name__)
        if hasattr(completer, "max_tokens"):
            completer.max_tokens = response_tokens

    def decide(self, context):
        # No ChatState history, team snapshot, global world state or peer model
        # object is available to this call. An instance belongs to ONE robot.
        messages = [{"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": json.dumps(context, ensure_ascii=False)}]
        raw = self.completer.complete(messages)
        return {"raw": raw, "usage": getattr(self.completer, "last_usage", None), "model": self.model_name}


def parse_reply(value, expected_robot, observation=None):
    usage = None
    if isinstance(value, dict) and "raw" in value:
        raw, usage = value["raw"], value.get("usage")
    else:
        raw = json.dumps(value, ensure_ascii=False) if isinstance(value, dict) else str(value)
    if not isinstance(raw, str) or len(raw) > 12000:
        raise ValueError("invalid decision output")
    stripped = raw.strip()
    if stripped.startswith("```") and stripped.endswith("```"):
        stripped = stripped.split("\n", 1)[1].rsplit("```", 1)[0].strip()
    parsed = json.loads(stripped)
    if not isinstance(parsed, dict) or parsed.get("robot_id") != expected_robot:
        raise ValueError("decision identity mismatch")
    required = {"robot_id", "role", "cargo_id", "destination", "assignments", "message"}
    if not required <= parsed.keys() or not isinstance(parsed["message"], str):
        raise ValueError("decision schema incomplete")
    if parsed["role"] not in ROLES or not isinstance(parsed["assignments"], dict):
        raise ValueError("invalid role or roster")
    roster = parsed["assignments"]
    if (set(roster) != set(ROLES) or set(roster.values()) != set(ROBOT_IDS)
            or roster.get(parsed["role"]) != expected_robot):
        raise ValueError("roster must contain r1/r2/r3 exactly once and match your own role")
    parsed["raw_message"] = parsed["message"]
    parsed["parsed_decision"] = {k: parsed[k] for k in required}
    if observation is not None:
        # Trusted call envelope binds output to the exact input. This adds no
        # role/cargo decision; delayed outputs cannot obtain a newer envelope.
        parsed["revision"] = observation["revision"]
        parsed["observation_id"] = observation["observation_id"]
    return raw, parsed, usage


def run_episode(environment, policies, *, condition="llm_peer_comm", seed=11,
                source="A", destination="B", selector="all", scenario="normal",
                budget=None, journal_path=None, event_callback=None):
    """Run bounded rounds. All decisions in a round are made concurrently.

    scenario gripper_failure is private to r1; goal_revision changes the operator
    destination after the first delivered cargo. Both are specified before run.
    """
    if condition not in {"rule", "llm_no_comm", "llm_peer_comm"}:
        raise ValueError("invalid condition")
    if set(policies) != set(ROBOT_IDS) or len({id(v) for v in policies.values()}) != 3:
        raise ValueError("three independent policy instances required")
    if scenario not in {"normal", "gripper_failure", "goal_revision"}:
        raise ValueError("invalid scenario")
    kinds = {getattr(p, "evidence_kind", "fixture") for p in policies.values()}
    if condition == "rule" and kinds != {"rule"}:
        raise ValueError("rule condition requires explicit rule policies")
    budget = budget or EpisodeBudget()
    journal = Journal(journal_path)
    started = time.perf_counter()
    models = {rid: getattr(p, "model_name", type(p).__name__) for rid, p in policies.items()}
    evidence_kind = "live_llm" if kinds == {"live_llm"} else ("rule" if kinds == {"rule"} else "fixture")
    metadata = {"condition": condition, "seed": seed, "scenario": scenario,
                "source": source, "destination": destination, "selector": selector,
                "budget": asdict(budget), "models": models, "evidence_kind": evidence_kind,
                "sensor_profile": "parallel_rgbd_fiducial_memory_v4", "controller_profile": "independent_crew_forward_uncalibrated_v4"}
    journal.write("episode_start", **metadata)
    status = environment.request("r1", {"operation": "begin", "condition": condition,
        "mission_id": f"warehouse_research_{seed}", "source_zone": source,
        "destination_zone": destination, "selector": selector, "sensor_seed": seed,
        "disabled_carrier": "r1" if scenario == "gripper_failure" else None})
    rounds = actions = invalid = 0
    usage_records = []
    reason = "BUDGET_EXHAUSTED"
    inboxes = {rid: [] for rid in ROBOT_IDS}
    own_history = {rid: [] for rid in ROBOT_IDS}
    peer_comm = condition != "llm_no_comm"
    revised = False
    episode_id = status.get("episode_id")
    if not episode_id:
        reason = status.get("reason", "BEGIN_FAILED")
    else:
        if status.get("world_seed") != seed:
            raise ValueError("environment seed does not match experiment seed")
        manifest = status["manifest"]
        for round_index in range(budget.max_rounds):
            if status.get("complete") and (scenario != "goal_revision" or revised):
                reason = "SUCCESS"
                break
            if actions >= budget.max_actions:
                break
            if scenario == "goal_revision" and actions == 1 and not revised:
                revised = True
                status = environment.request("r1", {"operation": "revise", "episode_id": episode_id,
                                                       "destination": "C" if destination != "C" else "B"})
                inboxes = {rid: [] for rid in ROBOT_IDS}
                journal.write("goal_revision", destination=status.get("destination"), revision=status.get("revision"))
            if hasattr(environment, "observe_all"):
                observations = environment.observe_all(episode_id, round_index)
            else:
                observations = {rid: environment.request(rid, {"operation": "observe", "episode_id": episode_id,
                                                               "sequence": round_index}) for rid in ROBOT_IDS}
            if set(observations) != set(ROBOT_IDS) or any("observation_id" not in v for v in observations.values()):
                reason = "SENSOR_UNAVAILABLE"
                journal.write("sensor_failure", reason=reason, responses=observations)
                break
            contexts = {rid: {"robot_id": rid, "round": round_index,
                "observation": observations[rid], "manifest": [m for m in manifest if m["cargo_id"] in status.get("remaining_ids", [])],
                "inbox": (copy.deepcopy(inboxes[rid]) if condition == "rule" else
                          [{"sender_id": m["sender_id"], "message": m["message"]}
                           for m in inboxes[rid]]) if peer_comm else [],
                "own_history": copy.deepcopy(own_history[rid][-3:]),
                "last_result": reason if reason in {"AGREEMENT_REJECTED", "ACTION_REJECTED", "PHYSICAL_ACTION_FAILED"} else None}
                for rid in ROBOT_IDS}
            for rid in ROBOT_IDS:
                journal.write("actor_input", round=round_index, robot_id=rid, context=contexts[rid])
            pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix="warehouse-peer")
            call_start = time.perf_counter()
            futures = {rid: pool.submit(policies[rid].decide, copy.deepcopy(contexts[rid])) for rid in ROBOT_IDS}
            done, pending = wait(futures.values(), timeout=budget.call_timeout_s)
            pool.shutdown(wait=False, cancel_futures=True)
            rounds += 1
            if pending:
                # Never re-use these agents while an old completion is running.
                reason = "LLM_TIMEOUT"
                journal.write("round_timeout", round=round_index, completed=len(done))
                break
            proposals, replies = {}, {}
            for rid, future in futures.items():
                value = None
                try:
                    value = future.result()
                    usage_records.append(value.get("usage") if isinstance(value, dict) else None)
                    raw, parsed, usage = parse_reply(value, rid, observations[rid])
                    if (isinstance(usage, dict) and isinstance(usage.get("completion_tokens"), int)
                            and usage["completion_tokens"] > budget.response_tokens):
                        raise ValueError("PROVIDER_OUTPUT_BUDGET_EXCEEDED")
                    proposals[rid], replies[rid] = parsed, raw
                    own_history[rid].append({"decision": parsed["parsed_decision"]})
                    journal.write("decision", round=round_index, robot_id=rid, raw=raw,
                                  parsed=parsed, usage=usage, model=value.get("model", models[rid]) if isinstance(value, dict) else models[rid],
                                  batch_elapsed_s=time.perf_counter()-call_start)
                    if event_callback:
                        event_callback(rid, parsed["message"])
                except Exception as exc:
                    invalid += 1
                    if str(exc) == "PROVIDER_OUTPUT_BUDGET_EXCEEDED":
                        reason = "PROVIDER_OUTPUT_BUDGET_EXCEEDED"
                    journal.write("invalid_decision", round=round_index, robot_id=rid,
                                  raw=value, error=str(exc))
            if reason == "PROVIDER_OUTPUT_BUDGET_EXCEEDED":
                break
            # Delivery occurs after all completions: thread timing cannot make
            # one condition see another policy's current-round reply early.
            previous_inboxes = copy.deepcopy(inboxes)
            messages, reports = [], []
            for rid, proposal in proposals.items():
                visible = observations[rid].get("cargo", [])
                if peer_comm:
                    messages.append({"sender_id": rid, "message": proposal["message"],
                                     "decision": proposal["parsed_decision"],
                                     "observed_cargo": [c for c in visible if condition == "rule" or c["cargo_id"] in proposal["message"]],
                                     "can_carry": observations[rid]["capabilities"]["can_carry"]})
                    for c in visible:
                        reports.append({"sender_id": rid, "cargo_id": c["cargo_id"], "message": proposal["message"]})
            if peer_comm:
                for rid in ROBOT_IDS:
                    inboxes[rid] = [copy.deepcopy(m) for m in messages if m["sender_id"] != rid]
                    delivered = inboxes[rid] if condition == "rule" else [
                        {"sender_id": m["sender_id"], "message": m["message"]} for m in inboxes[rid]]
                    journal.write("message_delivery", round=round_index, robot_id=rid, inbox=delivered)
            # Each robot may only act on reports already present in its input,
            # never reports first delivered after the decision above.
            seen_reports = []
            if peer_comm:
                for rid in ROBOT_IDS:
                    # The previous round's evidence envelopes are transport
                    # receipts; LLM actors saw only their natural-language text.
                    for message in previous_inboxes[rid]:
                        sender = message["sender_id"]
                        current_visible = {c["cargo_id"] for c in observations[sender]["cargo"]}
                        for c in message.get("observed_cargo", []):
                            if c["cargo_id"] in current_visible:
                                report = {"sender_id": sender, "cargo_id": c["cargo_id"], "message": message["message"]}
                                if report not in seen_reports:
                                    seen_reports.append(report)
            if len(proposals) != 3:
                reason = "INVALID_DECISION"
                continue
            result = environment.request("r1", {"operation": "execute", "episode_id": episode_id,
                                                   "proposals": proposals, "reports": seen_reports})
            journal.write("action_result", round=round_index, result=result)
            reason = result.get("reason", "ACTION_FAILED")
            if result.get("executed_cargo_id") or reason == "PHYSICAL_ACTION_FAILED":
                actions += 1
                inboxes = {rid: [] for rid in ROBOT_IDS}
                own_history = {rid: [] for rid in ROBOT_IDS}
            status = {**status, **result}
        if status.get("complete") and (scenario != "goal_revision" or revised):
            reason = "SUCCESS"
    known_usage = [u for u in usage_records if isinstance(u, dict) and isinstance(u.get("total_tokens"), int)]
    usage_complete = len(known_usage) == len(usage_records) == rounds * 3 and bool(rounds)
    result = {**metadata, "episode_id": episode_id,
              "success": reason == "SUCCESS", "reason": reason,
              "rounds": rounds, "actions": actions, "invalid_decisions": invalid,
              "elapsed_s": round(time.perf_counter()-started, 4),
              "tokens_total": 0 if condition == "rule" else (sum(u["total_tokens"] for u in known_usage) if usage_complete else None),
              "known_tokens_total": sum(u["total_tokens"] for u in known_usage),
              "token_usage_complete": condition == "rule" or usage_complete,
              "recovery_success": (reason == "SUCCESS") if scenario != "normal" else None,
              "delivered_ids": status.get("delivered_ids", []),
              "journal": str(journal.path) if journal.path else None}
    journal.write("episode_result", **result)
    return result
