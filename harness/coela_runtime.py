"""Three independent local loops with a replaceable high-level message channel."""
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import threading
import time

from harness.coela_modules import ROBOTS, Communication, Execution, Memory, Perception, actor_context
from harness.warehouse_runtime import Journal

CONDITIONS = {"none": "llm_no_comm", "structured": "llm_structured_comm", "natural": "llm_peer_comm"}


def run_coela_episode(environment, planners, *, mode="natural", destination="B", timeout_s=150.,
                      max_calls_per_robot=24, decision_period_s=3., journal_path=None,
                      scenario_hook=None, event_callback=None):
    if mode not in CONDITIONS or set(planners) != set(ROBOTS) or len({id(p) for p in planners.values()}) != 3:
        raise ValueError("THREE_INDEPENDENT_PLANNERS_AND_VALID_MODE_REQUIRED")
    if timeout_s <= 0 or decision_period_s <= 0 or max_calls_per_robot < 1:
        raise ValueError("INVALID_BUDGET")
    journal, lock, stop = Journal(journal_path), threading.RLock(), threading.Event()

    def log(event, **fields):
        with lock:
            journal.write(event, **fields)

    begin = environment.request("r1", {"operation": "mixed_begin", "condition": CONDITIONS[mode],
        "information_mode": "strict_local", "destination_zone": destination})
    if not begin.get("episode_id"):
        raise ValueError(begin.get("reason", "BEGIN_FAILED"))
    eid, manifest = begin["episode_id"], copy.deepcopy(begin["manifest"])
    bus = Communication(mode)
    calls = {r: 0 for r in ROBOTS}
    errors, usage, receipts = [], [], []
    in_flight, retired = set(), set()
    execution = {r: Execution(environment, r, eid, destination) for r in ROBOTS}
    perception = {r: Perception(environment, r, eid) for r in ROBOTS}
    memories = {r: Memory(r) for r in ROBOTS}
    reserve = min(6, max_calls_per_robot // 4)
    normal_cap = max_calls_per_robot - reserve
    log("study_begin", mode=mode, information_mode="strict_local", manifest=manifest,
        models={r: planners[r].model_name for r in ROBOTS},
        evidence_kinds={r: getattr(planners[r], "evidence_kind", "unknown") for r in ROBOTS},
        timeout_s=timeout_s, max_calls_per_robot=max_calls_per_robot,
        normal_call_cap=normal_cap, recovery_call_reserve=reserve,
        decision_period_s=decision_period_s)
    preflight_start = time.monotonic()
    initial = {}
    try:
        for rid in ROBOTS:
            initial[rid] = perception[rid].read()
        # Once every camera has completed its physical initialization scan,
        # refresh each robot from its own passive view. This removes the scan
        # ordering age without sharing observations or adding servo motion.
        for rid in ROBOTS:
            initial[rid] = perception[rid].read()
    except Exception:
        environment.request("r1", {"operation": "mixed_stop", "episode_id": eid})
        raise
    preflight_s = time.monotonic() - preflight_start
    started = time.monotonic()
    deadline = started + timeout_s
    log("preflight_complete", elapsed_s=preflight_s)

    def own_key(local):
        return json.dumps([local.get("active"), local.get("pending"),
                           local.get("history", []), local.get("recovery_events", [])], sort_keys=True)

    def retire_once(rid):
        with lock:
            if rid in retired:
                return
            result = execution[rid].retire()
            retired.add(rid)
            journal.write("agent_retired", robot_id=rid, result=result)

    def agent_loop(rid):
        observation, memory = initial[rid], memories[rid]
        memory.observe(observation, execution[rid].local_status()["sim_time"])
        log("observation", robot_id=rid, observation=observation)
        last_event = None
        last_action = None
        last_recovery = None
        unchanged = 0
        retry_at = 0.
        proposal_followup_at = None
        seen_accepted = set()
        last_active_messages = None
        try:
            while not stop.is_set() and time.monotonic() < deadline:
                local = execution[rid].local_status()
                memory.execution(local)
                with lock:
                    received = bus.receive(rid, local["sim_time"])
                    for message in received:
                        memory.receive(message)
                        journal.write("message_received", robot_id=rid, **message)
                active_reports = (memory.active_reports() if hasattr(memory, "active_reports")
                                  else memory.reports)
                message_ids = tuple(m["message_id"] for m in active_reports[-12:])
                accepted = {item.get("assignment_id") for item in local.get("history", [])
                            if item.get("event") == "accepted"}
                accepted_notice = bool(accepted - seen_accepted)
                report_change = last_active_messages is not None and message_ids != last_active_messages
                event = json.dumps([own_key(local), message_ids], sort_keys=True)
                now = time.monotonic()
                proposal_due = proposal_followup_at is not None and now >= proposal_followup_at
                changed = event != last_event or proposal_due
                busy, recovering = bool(local.get("active")), bool(local.get("recovery_events"))
                if not (not busy or recovering or received or accepted_notice or report_change) or (not changed and now < retry_at):
                    stop.wait(.05)
                    continue
                if calls[rid] >= max_calls_per_robot:
                    return
                if not recovering and calls[rid] >= normal_cap:
                    if not busy:
                        return
                    stop.wait(.05)
                    continue
                if (calls[rid] and not busy and not local.get("pending")) or recovering:
                    try:
                        observation = perception[rid].read()
                        memory.observe(observation, local["sim_time"])
                        log("observation", robot_id=rid, observation=observation)
                    except ValueError as exc:
                        if "ROBOT_BUSY" in str(exc):
                            stop.wait(.05)
                            continue
                        raise
                context = actor_context(rid, manifest, destination, mode, observation, memory, local)
                input_own_key = own_key(local)
                # Capture the reports actually serialized into this planner
                # context; an intent can expire between the scheduler probe and
                # actor_context's receiver-clock snapshot.
                input_messages = tuple(m["message_id"] for m in
                    context.get("memory", {}).get("peer_reports", []))
                log("actor_input", robot_id=rid, context=context, sim_time=local["sim_time"],
                    wall_s=time.monotonic() - started)
                last_event = event
                last_active_messages = message_ids
                seen_accepted.update(accepted)
                proposal_followup_at = None
                calls[rid] += 1
                usage_recorded = False
                with lock:
                    in_flight.add(rid)
                try:
                    answer = planners[rid].decide(context)
                    decision = answer["decision"]
                    now = time.monotonic()
                    with lock:
                        in_flight.discard(rid)
                        if stop.is_set() or now >= deadline:
                            journal.write("late_reply", robot_id=rid, raw=answer.get("raw"),
                                usage=answer.get("usage"), wall_s=now-started,
                                discarded=True, discard_reason="DEADLINE")
                            continue
                        usage.append(answer.get("usage"))
                        usage_recorded = True
                        journal.write("actor_reply", robot_id=rid, raw=answer.get("raw"),
                            usage=answer.get("usage"), wall_s=now-started)
                        # Validation, communication, callback, and actuation share
                        # the supervisor's stop lock, leaving no post-cutoff gap.
                        latest = execution[rid].local_status()
                        new_message = bool(bus.inboxes[rid])
                        action = decision["action"]
                        current_messages = tuple(m["message_id"] for m in memory.active_reports()[-12:])
                        # A claim is only this robot's provisional consent; the
                        # referee still requires a separate matching partner.
                        # Inbox churn must not starve that consent. Recovery is
                        # safety-sensitive, so refresh it on changed evidence.
                        stale = ("OWN_STATE_CHANGED" if own_key(latest) != input_own_key else
                                 "NEW_MESSAGE" if new_message and action.get("kind") == "recover"
                                 else "PEER_REPORT_EXPIRED" if action.get("kind") == "recover"
                                     and current_messages != input_messages
                                 else "BUSY_ACTION" if latest.get("active") and
                                     action.get("kind") not in {"wait", "recover"}
                                 else None)
                        journal.write("actor_decision", robot_id=rid, raw=answer.get("raw"),
                            decision=decision, usage=answer.get("usage"), sim_time=latest.get("sim_time"),
                            wall_s=now-started, discarded=bool(stale), discard_reason=stale)
                        if stale:
                            last_event = None
                            continue
                        action_key = json.dumps(action, sort_keys=True)
                        if action.get("kind") == "recover":
                            recovery_key = (action.get("event_id"), action.get("recovery_action"),
                                            input_messages)
                            if recovery_key == last_recovery:
                                journal.write("decision_suppressed", robot_id=rid,
                                    reason="UNCHANGED_RECOVERY_REQUEST", decision=decision)
                                unchanged += 1
                                retry_at = now + min(30., decision_period_s * 2 ** min(unchanged, 3))
                                continue
                            last_recovery = recovery_key
                        unchanged = unchanged + 1 if action_key == last_action and not changed else 0
                        retry_at = now + min(30., decision_period_s * 2 ** min(unchanged, 3))
                        last_action = action_key
                        memory.decisions.append(copy.deepcopy(decision))
                        sent = bus.send(rid, decision.get("message"), latest["sim_time"])
                        if sent:
                            journal.write("message_sent", **sent)
                            reported = sent.get("observed_ids", [])
                            observed = {c["cargo_id"] for c in observation["cargo"]}
                            for cid in set(reported) & observed:
                                receipts.append({"sender_id": rid, "cargo_id": cid,
                                                 "recipients": sent["recipients"]})
                        own_receipts = copy.deepcopy([r for r in receipts if rid in r["recipients"]])
                        if sent and event_callback:
                            event_callback(sent)
                        result = execution[rid].execute(action, observation, own_receipts)
                        journal.write("execution_result", robot_id=rid, result=result)
                        memory.decisions[-1]["execution_result"] = copy.deepcopy(result)
                        if action.get("kind") == "propose":
                            proposal_followup_at = time.monotonic() + 1.
                            retry_at = min(retry_at, proposal_followup_at)
                except Exception as exc:
                    with lock:
                        in_flight.discard(rid)
                        if stop.is_set() or time.monotonic() >= deadline:
                            journal.write("late_reply", robot_id=rid, reason=str(exc),
                                raw=getattr(exc, "raw", None), usage=getattr(exc, "usage", None),
                                wall_s=time.monotonic()-started, discarded=True,
                                discard_reason="DEADLINE")
                            continue
                        errors.append({"robot_id": rid, "reason": str(exc)})
                        if not usage_recorded:
                            usage.append(getattr(exc, "usage", None))
                        memory.decisions.append({"error": str(exc)})
                        journal.write("agent_error", robot_id=rid, reason=str(exc),
                            raw=getattr(exc, "raw", None), usage=getattr(exc, "usage", None))
                    stop.wait(.2)
        finally:
            try:
                retire_once(rid)
            except Exception as exc:
                with lock:
                    errors.append({"robot_id": rid, "reason": f"RETIRE_FAILED:{exc}"})
                log("agent_error", robot_id=rid, reason=f"RETIRE_FAILED:{exc}")

    def guarded(rid):
        try:
            agent_loop(rid)
        except Exception as exc:
            with lock:
                errors.append({"robot_id": rid, "reason": str(exc)})
            log("agent_loop_error", robot_id=rid, reason=str(exc))

    status = begin
    pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix="coela-local")
    futures = [pool.submit(guarded, rid) for rid in ROBOTS]
    try:
        while time.monotonic() < deadline:
            status = environment.request("r1", {"operation": "mixed_status", "episode_id": eid})
            if scenario_hook:
                scenario_hook(environment, eid, status)
            if status.get("complete") or status.get("failed", {}).get("engine"):
                break
            if all(f.done() for f in futures) and not status.get("active"):
                break
            stop.wait(.05)
    finally:
        timed_out = time.monotonic() >= deadline
        with lock:
            stop.set()
            # Revoke every consent lease before closing the physics owner. Late
            # actor finally blocks observe the retired set and never touch it.
            for rid in ROBOTS:
                try:
                    retire_once(rid)
                except Exception as exc:
                    errors.append({"robot_id": rid, "reason": f"RETIRE_FAILED:{exc}"})
            status = environment.request("r1", {"operation": "mixed_stop", "episode_id": eid})
        pool.shutdown(wait=not timed_out, cancel_futures=True)
    engine_failures = status.get("failed", {}).get("engine", {})
    failure_reason = status.get("reason") or ("TIMEOUT" if timed_out else
        (next(iter(engine_failures.values())) if isinstance(engine_failures, dict) and engine_failures
         else str(engine_failures) if engine_failures else "INCOMPLETE"))
    with lock:
        result = copy.deepcopy({"success": bool(status.get("complete")),
            "reason": "SUCCESS" if status.get("complete") else failure_reason,
            "mode": mode, "calls": calls, "errors": errors, "elapsed_s": time.monotonic()-started,
            "preflight_s": preflight_s, "messages": len(bus.sent), "message_counts": bus.counts,
            "message_bytes": bus.byte_counts, "usage": usage, "status": status,
            "pending_model_replies": sorted(in_flight),
            "pending_usage": {rid: "unknown" for rid in sorted(in_flight)},
            "information_mode": "strict_local", "normal_call_cap": normal_cap,
            "recovery_call_reserve": reserve})
    log("episode_result", **result)
    return result
