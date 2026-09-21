#!/usr/bin/env python3
"""Non-actuating Jev pilot. Only archived own RGB estimates/commands leave host."""
from __future__ import annotations

import argparse
import getpass
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import time
import urllib.error
import urllib.request

ENDPOINT = "https://api.typesafe.ai/v1/systemone"
OPTIONS = {
    "continue": "Request continuation of the existing carry policy, subject to unchanged joint permission and RGB guards.",
    "reobserve": "Request fresh RGB evidence and completion/identity checking while maintaining the existing coordinated hold.",
    "request_recovery": "Ask the supervisor to assess a validated loaded alignment recovery; do not execute a recovery or bypass joint permission.",
    "hold_and_escalate": "Maintain coordinated hold and request supervisor help because visual attachment evidence is lost or the situation cannot be resolved locally.",
}
INSTRUCTIONS = (
    "Choose one execution-monitoring recommendation for this robot from the supplied local history. "
    "These are archived RGB-derived estimates, not contact sensors, true pose, force or success. "
    "Own completion estimates can be wrong; issued commands do not prove motion. "
    "An all-zero command history can reflect the joint hold protocol, not a jam. "
    "No partner private observations or completion estimates are available. "
    "No release or completion declaration is permitted. Consider continued execution, fresh observation, "
    "a recovery assessment or escalation. This is a shadow recommendation with no actuation."
)
# Preselected diagnostic slices; not independent episodes or an accuracy benchmark.
SLICES = [("20260918", "open-minus", 20, "r1"),
          ("20260918", "open-minus", 80, "r1"),
          ("20260919", "open-plus", 100, "r1"),
          ("20260919", "open-plus", 300, "r1"),
          ("20260919", "open-plus", 600, "r1"),
          ("20260919", "open-plus", 892, "r1")]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def make_case(root, seed, episode, index, slot):
    """Explicit whitelist: never serialize whole source rows or evaluation records."""
    from PIL import Image, ImageChops, ImageStat
    folder = root / seed / episode
    source = folder / "pair-decisions.json"
    rows = [r for r in json.loads(source.read_text()) if r.get("kind") == "act_carry"]
    selected = [r for r in rows if index - 4 <= r["index"] <= index]
    if len(selected) != 5 or selected[-1]["index"] != index:
        raise ValueError("Five consecutive archived observations required")
    history, evidence, previous = [], [], {}
    for row in selected:
        inp, decision = row["inputs"][slot], row["decisions"][slot]
        change = {}
        for camera in ("own", "top"):
            ref = inp["images"][camera]
            path = (folder / ref["path"]).resolve()
            if not path.is_relative_to(folder.resolve()):
                raise ValueError("Image path outside episode")
            if digest(path.read_bytes()) != ref["sha256"]:
                raise ValueError("Archived image hash mismatch")
            with Image.open(path) as raw:
                im = raw.convert("RGB")
            change[camera] = (sum(ImageStat.Stat(ImageChops.difference(im, previous[camera])).mean) / 3 / 255
                              if camera in previous else None)
            previous[camera] = im
            evidence.append({"index": row["index"], "camera": camera, **ref})
        hold = decision["own_attachment"]
        history.append({
            "relative_observation_step": row["index"] - index,
            "own_previous_issued_command": inp["context"][-3:],
            "own_rgb_policy_done_estimate": decision["done"],
            "own_rgb_policy_stop_score_uncalibrated": decision["stop_score"],
            "own_rgb_attachment_estimate": hold["held_estimate"],
            "own_rgb_attachment_area_ratio_to_anchor": hold["anchor_area_ratio"],
            "mean_rgb_pixel_change_0_to_1": change,
        })
    state = {"task": "Two robots jointly carry a beam to a fixed goal.",
             "observation": "Five historical own/top RGB-derived observations, spaced 0.2 simulation seconds apart. No images sent to this API.",
             "limitations": "Pixel change is global appearance change, not displacement or goal progress. Estimates are from the archived ACT and RGB guard, not a new perception model. No contact, measured joints, dynamic true pose, evaluation labels or partner private state.",
             "history": history}
    return {"id": f"{seed}-{episode}-{index}-{slot}", "state": state,
            "provenance": {"source": str(source), "source_sha256": digest(source.read_bytes()),
                           "slot": slot, "physical_robot_id": selected[-1]["inputs"][slot]["physical_robot_id"],
                           "images": evidence}}


def rule(state):
    h = state["history"]
    if not h[-1]["own_rgb_attachment_estimate"]:
        return "hold_and_escalate"
    if h[-1]["own_rgb_policy_done_estimate"] or all(not any(x["own_previous_issued_command"]) for x in h):
        return "reobserve"
    return "continue"


def request_body(state, model):
    return {"model": model, "state": state, "questions": {
        "action": {"type": "choice", "instructions": INSTRUCTIONS, "criteria": OPTIONS}}}


def validate_response(response):
    a = response["answers"]["action"]
    probabilities = a["probabilities"]
    if a["type"] != "choice" or a["choice"] not in OPTIONS or set(probabilities) != set(OPTIONS):
        raise ValueError("Invalid action schema")
    values = list(probabilities.values()) + [a["confidence"]]
    if not all(type(v) in (int, float) and math.isfinite(v) and 0 <= v <= 1 for v in values):
        raise ValueError("Invalid probabilities/confidence")
    if abs(sum(probabilities.values()) - 1) > 0.001:
        raise ValueError("Probabilities do not sum to one")
    if probabilities[a["choice"]] < max(probabilities.values()) - 1e-6:
        raise ValueError("Choice is not maximal")
    return a


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


def invoke(body, key):
    req = urllib.request.Request(ENDPOINT, json.dumps(body).encode(),
                                 {"Authorization": "Bearer " + key, "Content-Type": "application/json"})
    start = time.perf_counter()
    try:
        with urllib.request.build_opener(NoRedirect).open(req, timeout=30) as res:
            status, raw = res.status, res.read(2_000_000).decode()
    except urllib.error.HTTPError as exc:
        status, raw = exc.code, exc.read(2_000_000).decode(errors="replace")
    except (OSError, urllib.error.URLError) as exc:
        return {"status": "transport_error", "error_type": type(exc).__name__,
                "latency_ms": (time.perf_counter() - start) * 1000}
    # Never persist a reflected credential, including in provider error bodies.
    raw = raw.replace(key, "[REDACTED]")
    result = {"http_status": status, "latency_ms": (time.perf_counter() - start) * 1000,
              "raw_response": raw, "status": "http_error"}
    if status == 200:
        try:
            result["response"] = json.loads(raw)
            result["decision"] = validate_response(result["response"])
            result["status"] = "ok"
        except (ValueError, KeyError, TypeError):
            result["status"] = "invalid_response"
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source-root", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--execute", action="store_true")
    p.add_argument("--prompt-key", action="store_true")
    p.add_argument("--model", default="jev-latest")
    p.add_argument("--repeats", type=int, choices=(1, 2), default=1)
    args = p.parse_args()
    repo = Path(__file__).resolve().parents[1]
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=repo, text=True).strip()
    if args.execute and dirty:
        p.error("Commit experiment code before live execution")
    cases = [make_case(args.source_root, *spec) for spec in SLICES]
    key = (getpass.getpass("TypeSafe API key (hidden): ") if args.prompt_key else os.environ.get("TYPESAFE_API_KEY", "")) if args.execute else ""
    if args.execute and not key.strip():
        p.error("TYPESAFE_API_KEY or --prompt-key required")
    args.output.mkdir(parents=True, exist_ok=False)
    dump(args.output / "manifest.json", {"code_sha": sha, "python": platform.python_version(),
         "platform": platform.platform(), "execute": args.execute, "endpoint": ENDPOINT,
         "model_alias": args.model, "max_calls": len(cases) * args.repeats, "automatic_retries": 0,
         "actuation": False, "source_root": str(args.source_root),
         "scope": "Selected historical diagnostic states; no accuracy labels, no physical or real-time control validation. Latency is HTTP round trip including connection setup, excludes perception. Rule is a declared heuristic, not ground truth."})
    dump(args.output / "cases.json", cases)
    results = []
    for repeat in range(args.repeats):
        for case in cases:
            body = request_body(case["state"], args.model)
            prefix = f"{repeat}-{case['id']}"
            dump(args.output / (prefix + "-request.json"), body)
            result = invoke(body, key) if args.execute else {"status": "dry_run"}
            result.update(case_id=case["id"], repeat=repeat, rule=rule(case["state"]))
            dump(args.output / (prefix + "-result.json"), result)
            results.append(result)
            dump(args.output / "results.json", results)
            print(json.dumps({k: result[k] for k in ("case_id", "repeat", "status", "http_status", "latency_ms", "decision", "rule") if k in result}), flush=True)
            if args.execute and result["status"] != "ok":
                return 1  # preserve every failure, never silently retry a billable request
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
