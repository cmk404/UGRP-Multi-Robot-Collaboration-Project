"""Versioned role agreement and bounded request recovery; no world access."""
from __future__ import annotations

import copy
import hashlib
import json
import time

from harness.gemini_proxy import GeminiProxyError


class RoleAgreement:
    """Serialize model proposals, then require both ACKs on the exact version.

    A rotating token decides who may propose, not which roles are correct.
    A rejection discards the proposal. Natural-language 'agree' is never an ACK.
    """
    def __init__(self, robots=("r1", "r3")):
        self.robots=tuple(robots)
        self.version=1
        self.pending=None
        self.committed=None
        self.events=[]
        self.history={r:[] for r in robots}

    def context(self):
        return {"version":self.version,"proposer":self.robots[(self.version-1)%len(self.robots)],
                "proposal":copy.deepcopy(self.pending)}

    def receive(self, replies, turn):
        if self.committed is not None:
            return copy.deepcopy(self.committed["roles"])
        if set(replies)!=set(self.robots):
            raise ValueError("complete role reply batch required")
        for r,v in replies.items():self.history[r].append(copy.deepcopy(v))
        if self.pending is None:
            proposer=self.context()["proposer"]
            candidate=replies[proposer]
            if not candidate["accept"]:
                self.version+=1
                return None
            roles=copy.deepcopy(candidate["roles"])
            digest=hashlib.sha256(json.dumps(roles,sort_keys=True,separators=(",",":")).encode()).hexdigest()
            self.pending={"proposal_id":f"roles-{self.version}","plan_hash":digest,
                          "roles":roles,"proposer":proposer,"created_turn":turn}
            self.events.append({"event":"PROPOSED", "turn":turn,**copy.deepcopy(self.pending)})
            return None
        p=self.pending
        for rid,v in replies.items():
            if (v.get("proposal_id"),v.get("plan_hash"))!=(p["proposal_id"],p["plan_hash"]):
                self.events.append({"event":"STALE_ACK_REJECTED","turn":turn,"robot_id":rid})
                return None
            if v["accept"] and v["roles"]!=p["roles"]:
                self.events.append({"event":"CONFLICTING_ACK_REJECTED","turn":turn,"robot_id":rid})
                return None
        if all(v["accept"] for v in replies.values()):
            self.committed=copy.deepcopy(p)
            self.events.append({"event":"COMMITTED","turn":turn,**copy.deepcopy(p)})
            return copy.deepcopy(p["roles"])
        self.events.append({"event":"REJECTED","turn":turn,**copy.deepcopy(p)})
        self.pending=None;self.version+=1
        return None


def request_with_recovery(completer, make_request, validate, record, *, max_attempts=3,
                          can_request=lambda:True):
    """Retry inference only. The caller holds actuators and freezes this capture.

    No actuator capability is accepted, and no malformed reply is repaired here.
    Each attempt is separately recorded, including failures and missing usage.
    """
    if not 1<=max_attempts<=3:raise ValueError("attempts must be 1..3")
    hint=None
    for attempt in range(max_attempts):
        if not can_request():return None,"request_budget"
        request=make_request(attempt,hint)
        started=time.monotonic()
        entry={"attempt":attempt,"request_id":request["request_id"]}
        try:
            raw=completer.complete(request["messages"],images=request["images"])
            entry["raw_response"]=raw
            entry["reply"]=validate(raw,request["request_id"])
        except (ValueError,TypeError,KeyError) as exc:
            entry.update(error=f"{type(exc).__name__}: {exc}",error_kind="reply_schema",retryable=True)
            hint={"kind":"reply_schema","detail":str(exc),"previous_response":entry.get("raw_response","")}
        except GeminiProxyError as exc:
            entry.update(error=str(exc),error_kind=exc.error_kind,retryable=exc.retryable)
            hint={"kind":exc.error_kind,"detail":"Previous inference attempt failed; return a fresh reply for this request."}
        finally:
            entry.update(usage=completer.last_usage,model=completer.last_model,
                         latency_ms=(time.monotonic()-started)*1000)
            record(entry)
        if "reply" in entry:return entry["reply"],None
        if not entry["retryable"]:return None,entry["error_kind"]
    return None,"request_retries_exhausted"
