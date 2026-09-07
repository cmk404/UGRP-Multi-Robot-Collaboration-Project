"""Independent agent loops for mixed solo and two-robot cargo transport."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import copy
import json
import threading
import time

from harness.warehouse_runtime import Journal, ROBOT_IDS


PROMPT = """You are one independent warehouse robot. Choose useful work while
other robots work; there is no permanent scout. Small boxes require 1 carrier,
the long plank requires 2. A solo task needs ONLY your own claim; a joint task
needs identical participant claims from BOTH carriers, never the third robot.
Use only your own camera/local identity memory, operator manifest and received
natural-language messages. Choose only cargo you observed or a peer reported.
Never select already completed/reserved cargo or a busy participant. When two
robots are moving a plank, the free robot should transport a small box. When
you finish, promptly choose another available task. Do not wait for the team.
For an initial compatible distribution, compare task requirements and peer
intentions: if enough idle robots remain, two can propose the joint task while
the remaining robot independently chooses a small box. You make this choice;
no central allocator repairs it. IDs are r1,r2,r3; NEVER r0.
Reply JSON only: {"robot_id":"your ID","cargo_id":"chosen ID or empty to wait",
"participants":["your ID", "partner only when required"],"destination":"B",
"message":"natural language observation, selected task, participants and reason"}.
Keep participants in consistent sorted order. Include the selected cargo ID and
partner IDs explicitly in your message so partners can consent. If you wait,
state what information you need; reconsider when messages/work availability change.
If recovery_event is present, your current task is paused. Inspect its evidence
and your fresh observation, then reply {"robot_id":"your ID","event_id":"exact ID",
"recovery_action":"resume|replan|request_help|cancel","message":"reason and peer request"}.
Use replan for changed obstacle paths, resume only when the pause cause is cleared.
For overload, request_help or cancel; never resume the overloaded lift.
Joint recovery requires both participants' consent; unrelated robots keep working.
Missing mass/width is unknown, not permission to claim that lifting is safe.
"""


class MixedLLMPolicy:
    evidence_kind = "live_llm"
    def __init__(self, robot_id, completer):
        self.robot_id, self.completer = robot_id, completer
        self.model_name = getattr(completer, "model_name", type(completer).__name__)
        if hasattr(completer, "max_tokens"):
            completer.max_tokens = 384
    def decide(self, context):
        raw = self.completer.complete([{"role":"system","content":PROMPT},
                                       {"role":"user","content":json.dumps(context,ensure_ascii=False)}])
        return {"raw":raw,"usage":getattr(self.completer,"last_usage",None),"model":self.model_name}


class MixedRulePolicy:
    evidence_kind = "rule"
    model_name = "mixed_local_rule_v1"
    def __init__(self, robot_id):
        self.robot_id = robot_id
    def decide(self, context):
        if context.get("recovery_event"):
            e=context["recovery_event"]
            return {"robot_id":self.robot_id,"event_id":e["event_id"],
                    "recovery_action":"replan" if e["reason"]=="obstacle_changed" else ("request_help" if e["reason"]=="overload" else "resume"),
                    "message":"Reassess the paused task from the current event and replan if its path changed."}
        manifest = context["manifest"]
        available = set(context["available_ids"])
        observed = {c["cargo_id"] for c in context["observation"]["cargo"]}
        solo = [m["cargo_id"] for m in manifest if m["required_carriers"]==1 and m["cargo_id"] in available & observed]
        joint = [m["cargo_id"] for m in manifest if m["required_carriers"]==2 and m["cargo_id"] in available & observed]
        idle = context["idle_robots"]
        if joint and self.robot_id in ("r1","r2") and {"r1","r2"} <= set(idle):
            cid, participants = joint[0], ["r1","r2"]
        elif solo:
            cid, participants = solo[0], [self.robot_id]
        else:
            cid, participants = "", [self.robot_id]
        return {"robot_id":self.robot_id,"cargo_id":cid,"participants":participants,
                "destination":context["observation"]["destination"],
                "message":f"I observed {sorted(observed)}; I propose {cid} with {participants}."}


def parse_mixed_reply(value, rid, observation):
    raw = value.get("raw") if isinstance(value,dict) and "raw" in value else json.dumps(value)
    if not isinstance(raw,str) or len(raw)>12000:
        raise ValueError("INVALID_REPLY")
    text=raw.strip()
    if text.startswith("```") and text.endswith("```"):
        text=text.split("\n",1)[1].rsplit("```",1)[0].strip()
    p=json.loads(text)
    if p.get("robot_id")==rid and "recovery_action" in p:
        if p["recovery_action"] not in {"resume","replan","request_help","cancel"} or not p.get("event_id") or not p.get("message"):
            raise ValueError("INVALID_RECOVERY_REPLY")
        return raw,p
    if p.get("robot_id") != rid or not isinstance(p.get("participants"),list):
        raise ValueError("INVALID_ORIGIN_OR_PARTICIPANTS")
    if set(p["participants"])-set(ROBOT_IDS) or rid not in p["participants"]:
        raise ValueError("INVALID_PARTICIPANTS")
    if not isinstance(p.get("message"),str) or not p["message"].strip():
        raise ValueError("MESSAGE_REQUIRED")
    p["revision"]=observation["revision"]
    p["observation_id"]=observation["observation_id"]
    return raw,p


def run_mixed_episode(environment, policies, *, condition="llm_peer_comm", destination="B",
                      max_calls_per_robot=12, timeout_s=240., journal_path=None, event_callback=None,
                      scenario_hook=None):
    if set(policies)!=set(ROBOT_IDS) or len({id(p) for p in policies.values()})!=3:
        raise ValueError("three independent policies required")
    journal=Journal(journal_path)
    lock=threading.RLock()
    stop=threading.Event()
    begin=environment.request("r1",{"operation":"mixed_begin","destination_zone":destination,"condition":condition})
    if not begin.get("episode_id"):
        return {"success":False,"reason":begin.get("reason","BEGIN_FAILED")}
    eid=begin["episode_id"]
    started=time.monotonic()
    inboxes={r:[] for r in ROBOT_IDS}
    peer_reports=[]
    calls={r:0 for r in ROBOT_IDS}
    errors=[]
    journal.write("mixed_begin",condition=condition,metadata=begin,
                  models={r:getattr(p,"model_name",type(p).__name__) for r,p in policies.items()})
    # Initial observation is completed before work begins so sensor motion
    # cannot interfere with another robot's grasp. Later idle observations are passive.
    initial={r:environment.request(r,{"operation":"mixed_observe","episode_id":eid}) for r in ROBOT_IDS}
    def agent_loop(rid):
        own=[]
        last_state_key=None
        own_obs=initial[rid]
        while not stop.is_set() and time.monotonic()-started<timeout_s:
            status=environment.request(rid,{"operation":"mixed_status","episode_id":eid})
            if status.get("complete") or status.get("failed",{}).get("engine"):
                return
            recovery=next((e for e in status.get("recovery_events",[]) if rid in e["participants"]),None)
            if rid not in status.get("idle_robots",[]) and recovery is None:
                time.sleep(.10);continue
            if not status.get("available_ids") and recovery is None:
                time.sleep(.10);continue
            with lock:
                inbox=copy.deepcopy(inboxes[rid]) if condition!="llm_no_comm" else []
                reports=copy.deepcopy(peer_reports)
            key=json.dumps([status.get("available_ids"),status.get("idle_robots"),inbox[-6:],status.get("history",[])[-3:],recovery],sort_keys=True)
            if key==last_state_key:
                time.sleep(.10);continue
            if calls[rid]>=max_calls_per_robot:
                return
            last_state_key=key
            if calls[rid]:
                refreshed=environment.request(rid,{"operation":"mixed_observe","episode_id":eid})
                if refreshed.get("observation_id"):
                    own_obs=refreshed
                else:
                    with lock:journal.write("observation_error",robot_id=rid,response=refreshed)
                    if refreshed.get("reason")=="ROBOT_BUSY" and recovery is None:
                        # A peer can complete consent between status and read.
                        # Keep this robot's loop alive for its later events.
                        last_state_key=None
                        time.sleep(.10)
                        continue
                    if recovery:
                        own_obs={**own_obs,"current_sensor_unavailable":True,"sensor_error":refreshed.get("reason")}
                    else:
                        with lock:errors.append({"robot_id":rid,"reason":"OBSERVATION_UNAVAILABLE"})
                        return
            if not own_obs.get("observation_id"):
                with lock:errors.append({"robot_id":rid,"reason":"OBSERVATION_UNAVAILABLE"})
                return
            context={"robot_id":rid,"observation":own_obs,"manifest":status["manifest"],
                     "available_ids":status["available_ids"],"idle_robots":status["idle_robots"],
                     "inbox":inbox[-6:],"own_history":own[-3:]}
            if recovery:context["recovery_event"]=recovery
            with lock:journal.write("actor_input",robot_id=rid,context=context,wall_s=time.monotonic()-started)
            try:
                calls[rid]+=1
                value=policies[rid].decide(context)
                raw,proposal=parse_mixed_reply(value,rid,own_obs)
                with lock:
                    journal.write("actor_decision",robot_id=rid,raw=raw,proposal=proposal,wall_s=time.monotonic()-started,
                                  usage=value.get("usage") if isinstance(value,dict) else None)
                    own.append(proposal)
                    if condition!="llm_no_comm":
                        for other in ROBOT_IDS:
                            if other!=rid:inboxes[other].append({"sender_id":rid,"message":proposal["message"]})
                        for c in own_obs["cargo"]:
                            if c["cargo_id"] in proposal["message"]:
                                peer_reports.append({"sender_id":rid,"cargo_id":c["cargo_id"],
                                                     "recipients":[r for r in ROBOT_IDS if r!=rid]})
                if event_callback:event_callback(rid,proposal["message"])
                if "recovery_action" in proposal:
                    result=environment.request(rid,{"operation":"mixed_recover","episode_id":eid,
                                                    "event_id":proposal["event_id"],"action":proposal["recovery_action"]})
                    with lock:journal.write("recovery_result",robot_id=rid,result=result)
                    continue
                if not proposal.get("cargo_id"):
                    continue
                result=environment.request(rid,{"operation":"mixed_submit","episode_id":eid,
                                                "proposals":[proposal],"reports":reports})
                with lock:journal.write("claim_result",robot_id=rid,result=result,wall_s=time.monotonic()-started)
            except Exception as exc:
                with lock:
                    errors.append({"robot_id":rid,"reason":str(exc)})
                    journal.write("agent_error",robot_id=rid,error=str(exc))
                time.sleep(.20)
    pool=ThreadPoolExecutor(max_workers=3,thread_name_prefix="mixed-independent-agent")
    def guarded_loop(rid):
        try:agent_loop(rid)
        except Exception as exc:
            with lock:
                errors.append({"robot_id":rid,"reason":str(exc)})
                journal.write("agent_loop_error",robot_id=rid,error=str(exc))
    futures=[pool.submit(guarded_loop,r) for r in ROBOT_IDS]
    status=begin
    try:
        while time.monotonic()-started<timeout_s:
            status=environment.request("r1",{"operation":"mixed_status","episode_id":eid})
            if scenario_hook:scenario_hook(environment,eid,status)
            if status.get("complete") or status.get("failed",{}).get("engine"):
                break
            if status.get("help_requests") and not status.get("active") and not status.get("available_ids"):
                break
            if all(f.done() for f in futures) and not status.get("active"):
                break
            time.sleep(.15)
    finally:
        stop.set()
        pool.shutdown(wait=True)
        environment.request("r1",{"operation":"mixed_stop","episode_id":eid})
    result={"success":bool(status.get("complete")),"reason":"SUCCESS" if status.get("complete") else ("ASSISTANCE_REQUIRED" if status.get("help_requests") else "INCOMPLETE"),
            "calls":calls,"errors":errors,"elapsed_s":time.monotonic()-started,
            "status":status,"condition":condition}
    with lock:journal.write("episode_result",**result)
    return result
